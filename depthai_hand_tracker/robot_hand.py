#!/usr/bin/env python3

import numpy as np
import cv2
import serial
import serial.tools.list_ports
import time
import sys
from scipy import signal
from collections import deque
from HandTrackerRenderer import HandTrackerRenderer
from HandTracker import HandTracker  # Use Host mode for better compatibility
from hand_pose_fixes import mcp_flexion_signed_deg, build_stable_hand_frame

# ANSI color codes for terminal output
class Colors:
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    BOLD = '\033[1m'
    END = '\033[0m'

def print_colored(text, color=''):
    """Print colored text to terminal"""
    print(f"{color}{text}{Colors.END}")
    sys.stdout.flush()

# ===== Motor Control Configuration =====
SERIAL_PORT = "COM3"  # Arduino port
BAUD_RATE = 115200
UPDATE_RATE_HZ = 20  # Motor update rate (increased for responsiveness)
ENABLE_MOTORS = True  # Set to False to disable motor control
DEBUG_SERIAL = True  # Set to True to print serial communication debug info

# ===== NEW: Robust MCP Calculation =====
USE_ROBUST_MCP = True   # Use stable hand frame + robust MCP flexion (orientation-independent)
DEBUG_ANGLES = True     # Print raw MCP angle ranges to tune thresholds
MCP_ZERO_OFFSET_DEG = 15.0  # Subtract from robust MCP to make fully-extended = 0
MCP_DEADBAND_DEG = 8.0      # Ignore small flexion near 0 to prevent tiny bends
PIP_ZERO_OFFSET_DEG = 20.0  # Subtract from PIP angle to make fully-extended = 0
PIP_DEADBAND_DEG = 10.0     # Ignore small PIP flexion near 0
PIP_FLEX_MAX_DEG = 60.0     # Expected flexion range when making a fist

# Smoothing settings (fast + smooth)
FILTER_CUTOFF_HZ = 10.0  # Higher = faster response
FILTER_ORDER = 2         # Standard filter
FILTER_BUFFER_SIZE = 3   # Minimal buffer for low latency
MAX_MOTOR_CHANGE_PER_UPDATE = 80.0  # No rate limiting (Arduino handles it)

# Position hold settings
HOLD_LAST_POSITION = True  # Keep last position when hand is lost (prevents jerky reset)
RESET_TIMEOUT_SECONDS = 5.0  # Reset to 0° after this many seconds of no hand (0 = never)

# Motor angle limits (must match Arduino)
MOTOR_MIN = -80.0
MOTOR_MAX = 0.0

# Joint angle mapping (observed from testing)
STRAIGHT_ANGLE = 180.0  # Finger fully extended
BENT_ANGLE = 120.0      # Finger fully bent (increased for more motor movement/sensitivity)

# PIP joint sensitivity multiplier (makes PIP more responsive)
PIP_GAIN = 1.3  # Multiplies motor output for PIP joint (1.0 = normal, >1.0 = more movement)

# ===== JOINT TO MOTOR MAPPING =====
# Control one robot finger using multiple joints from one real finger
# Set which real finger to track:
CONTROL_FINGER = 'index'  # Options: 'thumb', 'index', 'middle', 'ring', 'pinky'

# Map that finger's joints to motors:
JOINT_TO_MOTOR = {
    'mcp_flexion': 1,    # Knuckle joint → Motor 1 (using robust calculation!)
    'pip_angle': 2,      # Middle joint → Motor 2
    'mcp_abduction': 0   # Spread (0 = disabled)
}
# Available joints: 'pip_angle', 'mcp_flexion', 'mcp_abduction'
# Set to 0 to disable a joint

# ===== FIXED ANGLE RANGES (No Calibration) =====

class LowPassFilter:
    """Butterworth low-pass filter for smoothing angle measurements"""
    def __init__(self, cutoff_freq=1.0, sample_rate=30.0, order=2, buffer_size=20):
        """
        cutoff_freq: cutoff frequency in Hz (lower = more smoothing)
                     - 1.0 Hz = very smooth (good for stable readings)
                     - 2.0 Hz = moderate smoothing (recommended)
                     - 5.0 Hz = light smoothing (faster response)
        sample_rate: sampling rate in Hz (typically 30 fps for camera)
        order: filter order (2 or 4 recommended)
        buffer_size: number of samples to keep in buffer
        """
        self.cutoff_freq = cutoff_freq
        self.sample_rate = sample_rate
        self.order = order
        self.buffer_size = buffer_size
        
        # Design Butterworth low-pass filter
        nyquist = 0.5 * sample_rate
        normal_cutoff = cutoff_freq / nyquist
        self.b, self.a = signal.butter(order, normal_cutoff, btype='low', analog=False)
        
        # Calculate minimum samples needed for filtfilt (padlen = 3 * max(len(b), len(a)))
        self.min_samples = 3 * max(len(self.b), len(self.a)) + 1
        
        # Storage for each angle signal
        self.buffers = {}
    
    def update(self, finger_name, angle_type, new_value):
        """Update and return filtered angle value"""
        key = f"{finger_name}_{angle_type}"
        
        if key not in self.buffers:
            # Initialize buffer with first value
            self.buffers[key] = deque([new_value], maxlen=self.buffer_size)
            return new_value
        
        # Add new value to buffer
        self.buffers[key].append(new_value)
        
        # Apply low-pass filter only if we have enough samples
        if len(self.buffers[key]) >= self.min_samples:
            try:
                filtered = signal.filtfilt(self.b, self.a, list(self.buffers[key]))
                return filtered[-1]  # Return most recent filtered value
            except ValueError:
                # Fallback to simple average if filter fails
                return np.mean(list(self.buffers[key]))
        else:
            # Not enough samples yet, use simple moving average
            return np.mean(list(self.buffers[key]))
    
    def reset(self):
        """Reset all buffers (call when hand is lost)"""
        self.buffers = {}

def calculate_angle_3d(p1, p2, p3):
    """Calculate angle at p2 formed by p1-p2-p3 in 3D space"""
    v1 = p1 - p2
    v2 = p3 - p2
    
    cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    angle = np.arccos(cos_angle)
    return np.degrees(angle)

def transform_to_hand_frame(landmarks):
    """
    Transform landmarks from world coordinates to hand-local coordinates.
    This makes angles independent of hand position/orientation in space.
    All angles are now relative to the palm/knuckle orientation.
    
    Returns: transformed landmarks in hand coordinate system
    """
    # Ensure all landmarks are 3D (add z=0 if missing)
    landmarks_3d = []
    for l in landmarks:
        if len(l) == 2:
            landmarks_3d.append(np.array([l[0], l[1], 0.0]))
        else:
            landmarks_3d.append(np.array(l[:3]))
    landmarks = np.array(landmarks_3d)
    
    # Define hand coordinate system using palm landmarks
    wrist = landmarks[0]          # Origin
    middle_mcp = landmarks[9]     # Middle finger knuckle
    index_mcp = landmarks[5]      # Index finger knuckle
    pinky_mcp = landmarks[17]     # Pinky knuckle
    
    # X-axis: from wrist to middle finger MCP (forward direction of hand)
    x_axis = middle_mcp - wrist
    x_norm = np.linalg.norm(x_axis)
    if x_norm < 1e-6:
        # Degenerate case - return identity transform
        return landmarks - wrist
    x_axis = x_axis / x_norm
    
    # Ensure x_axis is 3D
    if x_axis.shape[0] != 3:
        return landmarks - wrist
    
    # Y-axis: across palm from pinky to index (sideways)
    y_temp = index_mcp - pinky_mcp
    y_norm = np.linalg.norm(y_temp)
    if y_norm < 1e-6:
        # Fallback: use a perpendicular vector
        y_temp = np.array([0.0, 1.0, 0.0]) if abs(x_axis[1]) < 0.9 else np.array([1.0, 0.0, 0.0])
    else:
        y_temp = y_temp / y_norm
    
    # Ensure y_temp is 3D
    if y_temp.shape[0] != 3:
        return landmarks - wrist
    
    # Z-axis: perpendicular to palm (cross product)
    z_axis = np.cross(x_axis, y_temp)
    z_norm = np.linalg.norm(z_axis)
    if z_norm < 1e-6:
        # Vectors are parallel - return identity transform
        return landmarks - wrist
    z_axis = z_axis / z_norm
    
    # Ensure z_axis is valid 3D vector
    if z_axis.shape[0] != 3 or not np.isfinite(z_axis).all():
        return landmarks - wrist
    
    # Recalculate Y-axis to be orthogonal
    y_axis = np.cross(z_axis, x_axis)
    y_norm = np.linalg.norm(y_axis)
    if y_norm < 1e-6:
        # Shouldn't happen, but fallback just in case
        return landmarks - wrist
    y_axis = y_axis / y_norm
    
    # Ensure y_axis is valid 3D vector
    if y_axis.shape[0] != 3 or not np.isfinite(y_axis).all():
        return landmarks - wrist
    
    # Create rotation matrix (world to hand)
    rotation_matrix = np.array([x_axis, y_axis, z_axis]).T
    
    # Transform all landmarks to hand coordinate system
    transformed = []
    for landmark in landmarks:
        # Translate to wrist origin
        relative = landmark - wrist
        # Rotate to hand frame
        transformed_point = rotation_matrix.T @ relative
        transformed.append(transformed_point)
    
    return np.array(transformed)

def calculate_joint_angles(hand):
    """
    Calculate joint angles for robotic hand control
    Returns angles for each finger:
    - PIP angle (interphalangeal joint)
    - MCP flexion angle
    - MCP abduction angle
    
    All angles are calculated in hand-local coordinates (relative to palm/knuckles)
    so they remain stable regardless of hand position/orientation in space.
    """
    if not hasattr(hand, 'landmarks') or hand.landmarks is None:
        return None
    
    # Prefer true world landmarks (meters) when available
    world_landmarks = None
    if hasattr(hand, 'world_landmarks') and hand.world_landmarks is not None:
        world_landmarks = hand.world_landmarks
    else:
        world_landmarks = hand.landmarks
    
    # Landmark set for angle calculations
    # If robust MCP is enabled and world landmarks exist, use world coords for all angles
    # Otherwise, use the transformed image landmarks
    if USE_ROBUST_MCP and hasattr(hand, 'world_landmarks') and hand.world_landmarks is not None:
        landmarks = world_landmarks
    else:
        landmarks = transform_to_hand_frame(hand.landmarks)
    
    angles = {}
    
    # Finger definitions: [name, mcp_idx, pip_idx, dip_idx, tip_idx]
    fingers = {
        'thumb': [1, 2, 3, 4],      # Thumb has different structure
        'index': [5, 6, 7, 8],
        'middle': [9, 10, 11, 12],
        'ring': [13, 14, 15, 16],
        'pinky': [17, 18, 19, 20]
    }
    
    # Wrist landmark for reference (now in hand-local frame)
    wrist = landmarks[0]
    
    for finger_name, indices in fingers.items():
        mcp_idx, pip_idx, dip_idx, tip_idx = indices
        
        # Get landmark positions (now in hand-local coordinates)
        mcp = landmarks[mcp_idx]
        pip = landmarks[pip_idx]
        dip = landmarks[dip_idx]
        tip = landmarks[tip_idx]
        
        # 1. PIP ANGLE (Interphalangeal joint)
        # Angle at PIP joint formed by MCP-PIP-DIP
        pip_angle = calculate_angle_3d(mcp, pip, dip)
        
        # 2. MCP FLEXION ANGLE
        if USE_ROBUST_MCP:
            # Use robust signed MCP flexion (no dot product, plane-based, stable during fist)
            # Convert landmarks to proper numpy array format (21, 3)
            try:
                # Ensure landmarks are in correct format for robust function
                lm_array = np.array([[l[0], l[1], l[2] if len(l) > 2 else 0.0] for l in world_landmarks], dtype=np.float32)
                
                if DEBUG_ANGLES and finger_name == 'index':
                    src = "world_landmarks" if hasattr(hand, 'world_landmarks') and hand.world_landmarks is not None else "landmarks"
                    print(f"[DEBUG MCP] Source: {src}")
                    print(f"[DEBUG MCP] lm_array shape: {lm_array.shape}, dtype: {lm_array.dtype}")
                    print(f"[DEBUG MCP] First landmark: {lm_array[0]}")
                
                mcp_flexion = mcp_flexion_signed_deg(lm_array, finger=finger_name)
                
                if DEBUG_ANGLES and finger_name == 'index':
                    print(f"[DEBUG MCP] Returned mcp_flexion: {mcp_flexion}°")
            except Exception as e:
                if DEBUG_ANGLES and finger_name == 'index':
                    print(f"[DEBUG MCP ERROR] {e}")
                mcp_flexion = 0.0
        else:
            # Old method: 3-point angle (uses dot product)
            mcp_flexion = calculate_angle_3d(wrist, mcp, pip)
        
        # 3. MCP ABDUCTION ANGLE
        # Calculate abduction relative to middle finger (hand spread)
        if finger_name == 'middle':
            # Middle finger is the reference, no abduction
            mcp_abduction = 0.0
        else:
            # Use middle finger MCP as reference (in hand-local frame)
            middle_mcp = landmarks[9]
            
            # Calculate angle between finger and middle finger in hand frame
            finger_vector = pip - mcp
            middle_vector = landmarks[10] - middle_mcp
            
            # Calculate abduction angle
            cos_abd = np.dot(finger_vector, middle_vector) / (
                np.linalg.norm(finger_vector) * np.linalg.norm(middle_vector) + 1e-6
            )
            cos_abd = np.clip(cos_abd, -1.0, 1.0)
            mcp_abduction = np.degrees(np.arccos(cos_abd)) - 90  # Offset so 0 is neutral
        
        angles[finger_name] = {
            'pip_angle': pip_angle,           # Interphalangeal joint
            'mcp_flexion': mcp_flexion,       # MCP motor 1 (flex/extend)
            'mcp_abduction': mcp_abduction    # MCP motor 2 (spread/close)
        }
    
    return angles

def map_joint_to_motor(joint_angle, use_pip=True, calibration=None, use_robust=False):
    """
    Map joint angle to motor angle
    
    Args:
        joint_angle: Joint angle in degrees
        use_pip: If True, use PIP range; if False, use MCP range (ignored if calibration provided)
        calibration: Dict with 'open' and 'closed' calibrated angles, or None for default
        use_robust: If True, use robust MCP angle range (signed angles from new method)
    
    Returns:
        Motor angle: 0° (extended) to -80° (bent)
    """
    if calibration and 'open' in calibration and 'closed' in calibration:
        # Use calibrated range
        straight = calibration['open']
        bent = calibration['closed']
    else:
        # Use default ranges
        if use_robust:
            # Robust MCP: flexion magnitude (0 = extended, increases as you bend)
            # Apply offset + deadband to eliminate small residual bend when extended
            joint_angle = joint_angle - MCP_ZERO_OFFSET_DEG
            if abs(joint_angle) < MCP_DEADBAND_DEG:
                joint_angle = 0.0
            # Clamp after offset
            joint_angle = max(0.0, joint_angle)
            straight = 0.0     # Extended
            bent = 60.0        # Bent: typical fist flexion magnitude
        elif use_pip:
            straight = STRAIGHT_ANGLE
            bent = BENT_ANGLE
        else:
            # Old MCP method (high angle = extended, low angle = bent)
            # Based on actual observed values
            straight = 165.0  # Extended: hand fully open (~164-165°)
            bent = 145.0      # Bent: fist closed (~145-150°)
    
    # Clamp input angle
    joint_angle = np.clip(joint_angle, min(bent, straight), max(bent, straight))
    
    # Map to motor range
    if abs(straight - bent) < 0.1:  # Avoid division by zero
        normalized = 0.0
    else:
        normalized = (joint_angle - straight) / (bent - straight)  # 0 (straight) to 1 (bent)
    
    motor_angle = normalized * (MOTOR_MIN - MOTOR_MAX) + MOTOR_MAX
    
    # Clamp to safety limits
    return np.clip(motor_angle, MOTOR_MIN, MOTOR_MAX)

def map_flexion_to_motor(flexion_deg, flex_max_deg):
    """
    Map flexion magnitude (0..flex_max) to motor range (0..-80).
    """
    if flex_max_deg <= 0.0:
        return MOTOR_MAX
    flexion = np.clip(flexion_deg, 0.0, flex_max_deg)
    normalized = flexion / flex_max_deg  # 0 (extended) -> 1 (bent)
    motor_angle = normalized * (MOTOR_MIN - MOTOR_MAX) + MOTOR_MAX
    return np.clip(motor_angle, MOTOR_MIN, MOTOR_MAX)

def find_arduino_port():
    """Auto-detect Arduino serial port"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if any(x in port.description.lower() for x in ['arduino', 'teensy', 'ch340', 'usb', 'serial']):
            return port.device
    return None

def main():
    print_colored("=" * 60, Colors.CYAN + Colors.BOLD)
    print_colored("🤖 ROBOT HAND CONTROL - STARTING...", Colors.CYAN + Colors.BOLD)
    print_colored("=" * 60, Colors.CYAN + Colors.BOLD)
    
    # Step 1: Connect to Arduino FIRST (wait indefinitely)
    global SERIAL_PORT
    enable_motors = ENABLE_MOTORS
    ser = None
    
    print_colored("\n[1/2] 🔍 Searching for Arduino...", Colors.YELLOW + Colors.BOLD)
    
    if not enable_motors:
        print_colored("⚠ Motor control is disabled in config", Colors.RED)
        print_colored("   Set ENABLE_MOTORS = True to control the robot hand", Colors.RED)
        return
    
    if DEBUG_SERIAL:
        print_colored("[DEBUG] Serial debugging ENABLED", Colors.BLUE)
    
    # Wait indefinitely for Arduino
    try:
        connection_attempt = 0
        while ser is None:
            connection_attempt += 1
            
            # Find Arduino port if not specified
            if SERIAL_PORT is None:
                search_port = find_arduino_port()
                if search_port:
                    SERIAL_PORT = search_port
            
            if SERIAL_PORT is None:
                if connection_attempt == 1:
                    print_colored("⏳ Waiting for Arduino...", Colors.YELLOW)
                    print_colored("   Please connect Arduino via USB", Colors.YELLOW)
                    print_colored("   (Press Ctrl+C to cancel)", Colors.YELLOW)
                else:
                    print_colored(f"   Still searching... (attempt {connection_attempt})", Colors.YELLOW)
                time.sleep(2)
                continue
            
            # Try to connect
            try:
                if connection_attempt == 1:
                    print_colored(f"   📡 Found Arduino on {SERIAL_PORT}, connecting...", Colors.CYAN)
                ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
                time.sleep(2)
                print_colored(f"✓ Arduino connected on {SERIAL_PORT}", Colors.GREEN + Colors.BOLD)
                
                # Clear any startup messages from Arduino
                lines_cleared = 0
                while ser.in_waiting:
                    line = ser.readline()
                    lines_cleared += 1
                    if DEBUG_SERIAL:
                        print_colored(f"[SERIAL RX] Startup: {line.decode('utf-8', errors='ignore').strip()}", Colors.BLUE)
                if DEBUG_SERIAL and lines_cleared > 0:
                    print_colored(f"[DEBUG] Cleared {lines_cleared} startup message(s)", Colors.BLUE)
                    
            except Exception as e:
                print_colored(f"⚠ Connection attempt failed: {e}", Colors.RED)
                print_colored(f"   Retrying in 2 seconds...", Colors.YELLOW)
                ser = None
                SERIAL_PORT = None  # Reset to search again
                time.sleep(2)
    
    except KeyboardInterrupt:
        print_colored("\n\n✓ Cancelled by user", Colors.GREEN)
        return
    
    # Step 2: Initialize camera AFTER successful Arduino connection
    print_colored("\n[2/2] 📷 Initializing hand tracker...", Colors.YELLOW + Colors.BOLD)
    
    # Initialize hand tracker
    tracker = HandTracker(
        input_src=None,  # Use OAK camera
        use_lm=True,
        use_world_landmarks=True,  # Use real-world 3D coords (more orientation-independent)
        use_gesture=False,
        xyz=False,
        solo=True,  # Track one hand
        resolution='full',
        stats=True,
        trace=0
    )
    
    renderer = HandTrackerRenderer(tracker=tracker)
    
    # Create Butterworth low-pass filter for smoothing (with configurable settings)
    angle_filter = LowPassFilter(
        cutoff_freq=FILTER_CUTOFF_HZ, 
        sample_rate=30.0, 
        order=FILTER_ORDER,
        buffer_size=FILTER_BUFFER_SIZE
    )
    
    print_colored("✓ Hand tracker initialized", Colors.GREEN + Colors.BOLD)
    print_colored("\n" + "=" * 60, Colors.GREEN + Colors.BOLD)
    print_colored("🤖 ROBOT HAND CONTROL ACTIVE ✋", Colors.GREEN + Colors.BOLD)
    print_colored("=" * 60, Colors.GREEN + Colors.BOLD)
    print_colored("👋 Show your hand to the camera to control the robot", Colors.CYAN)
    print_colored("⌨️  Press 'q' or ESC to quit", Colors.YELLOW)
    print_colored("=" * 60 + "\n", Colors.GREEN + Colors.BOLD)
    
    # Debug counter
    serial_cmd_count = 0
    
    hand_detected_last_frame = False
    last_motor_update = 0
    motor_update_interval = 1.0 / UPDATE_RATE_HZ
    
    # Rate limiting - track previous motor angles
    previous_motor_angles = [0.0, 0.0, 0.0, 0.0, 0.0]
    
    # Position hold tracking
    last_hand_detected_time = time.time()
    held_motor_angles = [0.0, 0.0, 0.0, 0.0, 0.0]
    
    
    try:
        while True:
            frame, hands, bag = tracker.next_frame()
            if frame is None:
                break
            
            # Draw hands on frame
            frame = renderer.draw(frame, hands, bag)
            
            # Default motor angles (extended)
            motor_angles = [0.0, 0.0, 0.0, 0.0, 0.0]
            
            # Calculate and display joint angles
            if hands:
                hand_detected_last_frame = True
                last_hand_detected_time = time.time()  # Update last seen time
                for hand in hands:
                    angles = calculate_joint_angles(hand)
                    if angles:
                        # Apply filtering to all angles
                        filtered_angles = {}
                        
                        for finger_name, finger_angles in angles.items():
                            filtered_angles[finger_name] = {
                                'pip_angle': angle_filter.update(finger_name, 'pip', finger_angles['pip_angle']),
                                'mcp_flexion': angle_filter.update(finger_name, 'mcp_flex', finger_angles['mcp_flexion']),
                                'mcp_abduction': angle_filter.update(finger_name, 'mcp_abd', finger_angles['mcp_abduction'])
                            }
                        
                        # Map joints from CONTROL_FINGER to motors
                        if CONTROL_FINGER in filtered_angles:
                            finger_data = filtered_angles[CONTROL_FINGER]
                            for joint_name, motor_num in JOINT_TO_MOTOR.items():
                                if motor_num > 0 and motor_num <= 5 and joint_name in finger_data:
                                    joint_angle = finger_data[joint_name]
                                    
                                    # Use different mapping for MCP vs PIP (no calibration)
                                    if joint_name == 'mcp_flexion':
                                        motor_angles[motor_num - 1] = map_joint_to_motor(joint_angle, use_pip=False, calibration=None, use_robust=USE_ROBUST_MCP)
                                    elif joint_name == 'pip_angle':
                                        # PIP joint with increased sensitivity
                                        # Convert absolute angle to flexion magnitude
                                        pip_flex = 180.0 - joint_angle
                                        pip_flex = pip_flex - PIP_ZERO_OFFSET_DEG
                                        if abs(pip_flex) < PIP_DEADBAND_DEG:
                                            pip_flex = 0.0
                                        if pip_flex < 0.0:
                                            pip_flex = 0.0
                                        base_motor_angle = map_flexion_to_motor(pip_flex, PIP_FLEX_MAX_DEG)
                                        # Apply gain for more movement (clamp to motor limits)
                                        motor_angles[motor_num - 1] = np.clip(base_motor_angle * PIP_GAIN, MOTOR_MIN, MOTOR_MAX)
                                        if DEBUG_ANGLES:
                                            print(f"[DEBUG PIP] angle={joint_angle:.1f} flex={pip_flex:.1f} base={base_motor_angle:.1f} final={motor_angles[motor_num - 1]:.1f}")
                                    else:
                                        motor_angles[motor_num - 1] = map_joint_to_motor(joint_angle, use_pip=True, calibration=None)
                        
                        # Apply rate limiting for smooth movement
                        for i in range(5):
                            diff = motor_angles[i] - previous_motor_angles[i]
                            if abs(diff) > MAX_MOTOR_CHANGE_PER_UPDATE:
                                # Limit the change
                                motor_angles[i] = previous_motor_angles[i] + np.sign(diff) * MAX_MOTOR_CHANGE_PER_UPDATE
                        
                        # Store current motor angles for position hold
                        held_motor_angles = motor_angles.copy()
                        
                        # Send to motors
                        current_time = time.time()
                        if enable_motors and ser and (current_time - last_motor_update >= motor_update_interval):
                            last_motor_update = current_time
                            cmd = f"{motor_angles[0]:.1f},{motor_angles[1]:.1f},{motor_angles[2]:.1f},{motor_angles[3]:.1f},{motor_angles[4]:.1f}\n"
                            
                            try:
                                bytes_written = ser.write(cmd.encode())
                                serial_cmd_count += 1
                                if DEBUG_SERIAL:
                                    print(f"[SERIAL TX #{serial_cmd_count}] Sent {bytes_written} bytes: {cmd.strip()}")
                            except Exception as e:
                                print(f"[SERIAL ERROR] Command #{serial_cmd_count+1} failed: {e}")
                                enable_motors = False  # Disable motors on error
                            
                            # Update previous angles
                            previous_motor_angles = motor_angles.copy()
                        
                        y_offset = 30
                        
                        # Status line
                        status_text = f"Hand: {hand.label} | Motors: {'ON' if enable_motors else 'OFF'}"
                        cv2.putText(frame, status_text, 
                                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 
                                   0.5, (0, 255, 0) if enable_motors else (100, 100, 100), 2)
                        y_offset += 25
                        
                        # Display joint angles
                        for finger_name, finger_angles in filtered_angles.items():
                            text = f"{finger_name.upper()}: PIP={finger_angles['pip_angle']:.1f}° | " \
                                   f"MCP_flex={finger_angles['mcp_flexion']:.1f}° | " \
                                   f"MCP_abd={finger_angles['mcp_abduction']:.1f}°"
                            cv2.putText(frame, text, (10, y_offset), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                            y_offset += 20
                        
                        # Display motor angles
                        y_offset += 10
                        cv2.putText(frame, "Motor Angles:", (10, y_offset), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                        y_offset += 20
                        motor_text = " | ".join([f"M{i+1}:{angle:5.1f}°" 
                                                for i, angle in enumerate(motor_angles)])
                        cv2.putText(frame, motor_text, (10, y_offset), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                        
                        # Minimal console output (only if DEBUG_ANGLES enabled)
                        if DEBUG_ANGLES:
                            print(f"{hand.label} {CONTROL_FINGER}: ", end="")
                            for motor_num in range(1, 6):
                                if motor_num - 1 < len(motor_angles):
                                    print(f"M{motor_num}:{motor_angles[motor_num - 1]:5.1f}° ", end="")
                            print()
            else:
                # No hand detected
                if hand_detected_last_frame:
                    angle_filter.reset()
                    hand_detected_last_frame = False
                    if DEBUG_SERIAL:
                        print(f"[DEBUG] Hand lost, last position: {held_motor_angles}")
                
                # Hold last position or reset after timeout
                if HOLD_LAST_POSITION:
                    time_since_hand = time.time() - last_hand_detected_time
                    
                    # Check if we should reset after timeout
                    if RESET_TIMEOUT_SECONDS > 0 and time_since_hand > RESET_TIMEOUT_SECONDS:
                        # Timeout reached - reset to extended position
                        if enable_motors and ser:
                            try:
                                ser.write(b"0,0,0,0,0\n")
                                if DEBUG_SERIAL:
                                    print(f"[SERIAL TX] Reset: 0,0,0,0,0")
                            except Exception as e:
                                print(f"⚠ Serial error: {e}")
                            held_motor_angles = [0.0, 0.0, 0.0, 0.0, 0.0]
                            if DEBUG_SERIAL:
                                print(f"[DEBUG] Timeout ({RESET_TIMEOUT_SECONDS}s) - reset to extended")
                            last_hand_detected_time = time.time()  # Reset timer
                    else:
                        # Hold last known position
                        current_time = time.time()
                        if enable_motors and ser and (current_time - last_motor_update >= motor_update_interval):
                            last_motor_update = current_time
                            cmd = f"{held_motor_angles[0]:.1f},{held_motor_angles[1]:.1f},{held_motor_angles[2]:.1f},{held_motor_angles[3]:.1f},{held_motor_angles[4]:.1f}\n"
                            try:
                                bytes_written = ser.write(cmd.encode())
                                if DEBUG_SERIAL:
                                    print(f"[SERIAL TX] Hold ({bytes_written} bytes): {cmd.strip()}")
                            except Exception as e:
                                if DEBUG_SERIAL:
                                    print(f"[SERIAL ERROR] Failed to hold: {e}")
                else:
                    # Immediate reset when hand lost
                    if enable_motors and ser:
                        try:
                            ser.write(b"0,0,0,0,0\n")
                            if DEBUG_SERIAL:
                                print(f"[SERIAL TX] Immediate reset: 0,0,0,0,0")
                        except Exception as e:
                            print(f"[SERIAL ERROR] Failed to reset: {e}")
            
            cv2.imshow("Joint Angle Capture", frame)
            key = cv2.waitKey(1)
            
            # Handle key presses
            if key == 27 or key == ord('q'):
                break
    
    except KeyboardInterrupt:
        print_colored("\n\n🛑 Shutting down...", Colors.YELLOW + Colors.BOLD)
    finally:
        # Cleanup
        if enable_motors and ser:
            print_colored("   Resetting servos to safe position...", Colors.YELLOW)
            ser.write(b"0,0,0,0,0\n")  # Safe position
            time.sleep(0.1)
            ser.close()
        renderer.exit()
        tracker.exit()
        print_colored("\n✓ Robot Hand Control stopped\n", Colors.GREEN)

if __name__ == "__main__":
    main()
