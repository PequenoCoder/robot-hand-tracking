# Start Hand Tracking

## Basic Demo
```powershell
.\.venv\Scripts\Activate.ps1; python demo.py
```

## Joint Angle Capture (for robotic hand control)
```powershell
.\.venv\Scripts\Activate.ps1; python capture_joint_angles.py
```

### Features:
- **Butterworth Low-Pass Filter** - Professional-grade signal smoothing reduces jitter
- Real-time angle display on screen and console output

### Joint Angles Captured:
For each finger (thumb, index, middle, ring, pinky):
- **PIP angle** - Interphalangeal joint (1 angle)
- **MCP flexion** - Metacarpophalangeal flex/extend (motor 1)
- **MCP abduction** - Metacarpophalangeal spread/close (motor 2)

### Filter Settings:
To adjust smoothing in `capture_joint_angles.py`, change `cutoff_freq`:
- `cutoff_freq=2.0` (default) - Moderate smoothing, good balance
- Lower values (0.5-1.5 Hz) = More smooth, slower response
- Higher values (3.0-5.0 Hz) = Less smooth, faster response
