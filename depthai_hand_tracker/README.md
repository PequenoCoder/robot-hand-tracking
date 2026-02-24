# DepthAI Hand Tracker

Real-time hand tracking using DepthAI OAK camera for robotic hand control.

## Quick Start

```bash
# Install dependencies
pip install -r requirements.txt

# Test camera
python test_oak.py

# Run robot hand control
python robot_hand.py
```

## 📖 Full Documentation

**See the main README for complete setup and usage instructions:**

→ **[../README.md](../README.md)**

Includes:
- Complete setup guides (Windows & Raspberry Pi)
- Hardware requirements & wiring
- Arduino/motor control setup
- Auto-start configuration
- Troubleshooting
- Performance tips
- Command reference

## Core Files

- `robot_hand.py` - **Main program** ⭐
- `HandTracker.py` - Hand tracking (host mode)
- `HandTrackerEdge.py` - Hand tracking (edge mode)
- `HandTrackerRenderer.py` - Visualization
- `test_oak.py` - Camera test utility
- `models/` - Neural network models
- `motors1_5/` - Arduino firmware

## License

See [LICENSE.txt](LICENSE.txt)
