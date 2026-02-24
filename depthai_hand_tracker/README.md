# Hand Tracker for Robotic Hand Control

Real-time hand tracking using DepthAI OAK camera for robotic hand control.

## Quick Start

### 1. Install Dependencies

```bash
pip install -r requirements.txt
```

### 2. Test Camera

```bash
python test_oak.py
```

### 3. Run Robot Hand Control

```bash
# Control the robotic hand
python robot_hand.py
```

## Files

- `robot_hand.py` - Main program for robotic hand control
- `HandTracker.py` - Core hand tracking (host mode)
- `HandTrackerEdge.py` - Hand tracking (edge mode - faster)
- `HandTrackerRenderer.py` - Visualization renderer
- `mediapipe_utils.py` - MediaPipe utilities
- `hand_pose_fixes.py` - Hand pose corrections
- `FPS.py` - FPS counter
- `test_oak.py` - Camera testing utility
- `requirements.txt` - Python dependencies
- `MOTOR_CONTROL_GUIDE.md` - Motor control documentation

## Folders

- `models/` - Neural network blob files
- `motors1_5/` - Arduino motor control code

## Setup Guide

For complete Raspberry Pi setup instructions, see: `../README_RASPBERRY_PI.md`

## License

See `LICENSE.txt`
