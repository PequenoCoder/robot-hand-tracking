# hand_pose_fixes.py
# Utilities to stabilize hand-local frame + compute robust signed MCP flexion + smoothing + hysteresis.

from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple
import numpy as np


# ----------------------------
# Basic vector helpers
# ----------------------------

EPS = 1e-8

def _norm(v: np.ndarray) -> float:
    return float(np.linalg.norm(v))

def _unit(v: np.ndarray) -> np.ndarray:
    n = _norm(v)
    if n < EPS:
        return np.zeros(3, dtype=np.float32)
    return (v / n).astype(np.float32)

def _safe_cross(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    c = np.cross(a, b)
    if _norm(c) < EPS:
        return np.zeros(3, dtype=np.float32)
    return c.astype(np.float32)

def _project_onto_plane(v: np.ndarray, plane_normal_unit: np.ndarray) -> np.ndarray:
    # v_proj = v - (v·n)*n
    return (v - float(np.dot(v, plane_normal_unit)) * plane_normal_unit).astype(np.float32)

def signed_angle_between(u: np.ndarray, v: np.ndarray, axis_unit: np.ndarray) -> float:
    """
    Signed angle from u to v around axis_unit (right-hand rule).
    Returns degrees in [-180, 180].
    """
    u_u = _unit(u)
    v_u = _unit(v)
    if _norm(u_u) < EPS or _norm(v_u) < EPS or _norm(axis_unit) < EPS:
        return 0.0

    # atan2 of signed magnitude
    cross_uv = np.cross(u_u, v_u)
    sin_term = float(np.dot(cross_uv, axis_unit))
    cos_term = float(np.dot(u_u, v_u))
    angle = math.degrees(math.atan2(sin_term, cos_term))
    return angle

def unsigned_angle_between(u: np.ndarray, v: np.ndarray) -> float:
    """
    Unsigned angle between u and v.
    Returns degrees in [0, 180].
    """
    u_u = _unit(u)
    v_u = _unit(v)
    if _norm(u_u) < EPS or _norm(v_u) < EPS:
        return 0.0
    cos_term = float(np.dot(u_u, v_u))
    cos_term = max(-1.0, min(1.0, cos_term))
    return math.degrees(math.acos(cos_term))


# ----------------------------
# Robust hand frame
# ----------------------------

PALM_IDXS = [0, 5, 9, 13, 17]  # wrist + MCPs
INDEX_MCP = 5
MIDDLE_MCP = 9
RING_MCP = 13
PINKY_MCP = 17
INDEX_PIP = 6
MIDDLE_PIP = 10
RING_PIP = 14
PINKY_PIP = 18

@dataclass
class HandFrame:
    origin: np.ndarray      # (3,)
    x_axis: np.ndarray      # (3,) forward (wrist->palm)
    y_axis: np.ndarray      # (3,) across palm (pinky->index)
    z_axis: np.ndarray      # (3,) palm normal

    def rotation_world_to_hand(self) -> np.ndarray:
        # columns are axes in world; for world->hand, multiply by R^T
        R = np.stack([self.x_axis, self.y_axis, self.z_axis], axis=1)  # 3x3
        return R.T.astype(np.float32)

    def to_hand(self, p_world: np.ndarray) -> np.ndarray:
        R_T = self.rotation_world_to_hand()
        return (R_T @ (p_world - self.origin)).astype(np.float32)


def build_stable_hand_frame(world_landmarks: np.ndarray) -> Optional[HandFrame]:
    """
    Builds a more stable palm frame than using just wrist->middle_mcp and pinky->index alone.
    Uses:
      - origin: palm_center (average of wrist + MCPs)
      - y axis: pinky_mcp -> index_mcp
      - x axis: wrist -> avg(index_mcp, middle_mcp, ring_mcp)  (less jitter than wrist->middle alone)
      - z axis: cross(x, y) (palm normal)
    Re-orthonormalizes axes.
    """
    lm = np.asarray(world_landmarks, dtype=np.float32)
    if lm.shape[0] < 21 or lm.shape[1] < 3:
        return None

    wrist = lm[0, :3]
    index_mcp = lm[INDEX_MCP, :3]
    middle_mcp = lm[MIDDLE_MCP, :3]
    ring_mcp = lm[RING_MCP, :3]
    pinky_mcp = lm[PINKY_MCP, :3]

    palm_center = np.mean(lm[PALM_IDXS, :3], axis=0).astype(np.float32)

    # across palm
    y_axis = _unit(index_mcp - pinky_mcp)

    # forward direction (average of central MCPs reduces wobble during fist)
    mcp_avg = (index_mcp + middle_mcp + ring_mcp) / 3.0
    x_axis = _unit(mcp_avg - wrist)

    z_axis = _unit(_safe_cross(x_axis, y_axis))
    if _norm(z_axis) < EPS:
        return None

    # re-orthogonalize y to be perpendicular to x and z
    y_axis = _unit(_safe_cross(z_axis, x_axis))
    if _norm(x_axis) < EPS or _norm(y_axis) < EPS:
        return None

    return HandFrame(origin=palm_center, x_axis=x_axis, y_axis=y_axis, z_axis=z_axis)


# ----------------------------
# Robust MCP flexion (signed, plane-based)
# ----------------------------

def mcp_flexion_signed_deg(
    world_landmarks: np.ndarray,
    finger: str = "index"
) -> float:
    """
    Returns a stable MCP flexion angle for the chosen finger.
    - Uses stable hand frame.
    - Measures angle between:
        metacarpal vector (MCP -> palm_center)  [proxy for hand direction]
        proximal phalanx vector (MCP -> PIP)
      projected onto the flexion plane (plane spanned by x & z in hand frame; normal is y).
    - Returns FLEXION MAGNITUDE: 0° = fully extended, increases as you bend.
    """
    frame = build_stable_hand_frame(world_landmarks)
    if frame is None:
        return 0.0

    lm = np.asarray(world_landmarks, dtype=np.float32)
    palm_center = frame.origin  # in world coords

    finger_map = {
        "thumb": (1, 2),
        "index": (5, 6),
        "middle": (9, 10),
        "ring": (13, 14),
        "pinky": (17, 18),
    }
    if finger not in finger_map:
        raise ValueError(f"Unknown finger '{finger}'")

    mcp_i, pip_i = finger_map[finger]
    mcp = lm[mcp_i, :3]
    pip = lm[pip_i, :3]

    # vectors in world
    v_meta = (palm_center - mcp)     # toward palm center
    v_prox = (pip - mcp)             # finger direction

    # convert to hand frame
    v_meta_h = frame.rotation_world_to_hand() @ v_meta
    v_prox_h = frame.rotation_world_to_hand() @ v_prox

    # flexion plane: XZ plane in hand frame => normal is Y axis
    plane_normal = np.array([0.0, 1.0, 0.0], dtype=np.float32)

    u = _project_onto_plane(v_meta_h, plane_normal)
    v = _project_onto_plane(v_prox_h, plane_normal)

    # Compute unsigned angle between vectors in flexion plane
    # When the finger is extended, u and v are nearly opposite -> angle ~180
    # Define flexion as: 0 (extended) -> increasing as angle decreases
    angle = unsigned_angle_between(u, v)
    flexion = 180.0 - angle
    return float(flexion)


# ----------------------------
# Causal smoothing + hysteresis
# ----------------------------

@dataclass
class EMAFilter:
    alpha: float = 0.35  # 0..1; higher = less smoothing
    value: Optional[float] = None

    def update(self, x: float) -> float:
        if self.value is None or not math.isfinite(self.value):
            self.value = x
        else:
            self.value = self.alpha * x + (1.0 - self.alpha) * self.value
        return float(self.value)

    def reset(self) -> None:
        self.value = None


@dataclass
class HysteresisGate:
    """
    Stable boolean decision from a noisy scalar.
    - enter_threshold: must exceed to turn ON
    - exit_threshold: must drop below to turn OFF
    - debounce_frames: require condition to hold N frames before switching
    """
    enter_threshold: float
    exit_threshold: float
    debounce_frames: int = 3

    state: bool = False
    _count: int = 0

    def update(self, x: float) -> bool:
        if not self.state:
            # OFF -> ON
            if x >= self.enter_threshold:
                self._count += 1
                if self._count >= self.debounce_frames:
                    self.state = True
                    self._count = 0
            else:
                self._count = 0
        else:
            # ON -> OFF
            if x <= self.exit_threshold:
                self._count += 1
                if self._count >= self.debounce_frames:
                    self.state = False
                    self._count = 0
            else:
                self._count = 0
        return self.state

    def reset(self) -> None:
        self.state = False
        self._count = 0


# ----------------------------
# Convenience wrapper (per-finger)
# ----------------------------

class MCPFlexTracker:
    """
    Tracks robust MCP flexion for one finger:
      raw -> EMA -> flexed boolean (hysteresis)
    """
    def __init__(
        self,
        finger: str = "index",
        ema_alpha: float = 0.35,
        enter_deg: float = 35.0,
        exit_deg: float = 25.0,
        debounce_frames: int = 3,
    ):
        self.finger = finger
        self.ema = EMAFilter(alpha=ema_alpha)
        self.gate = HysteresisGate(
            enter_threshold=enter_deg,
            exit_threshold=exit_deg,
            debounce_frames=debounce_frames,
        )

    def update(self, world_landmarks: np.ndarray) -> Dict[str, float | bool]:
        raw = mcp_flexion_signed_deg(world_landmarks, finger=self.finger)
        smooth = self.ema.update(raw)
        flexed = self.gate.update(smooth)
        return {"raw_deg": raw, "smooth_deg": smooth, "flexed": flexed}

    def reset(self) -> None:
        self.ema.reset()
        self.gate.reset()
