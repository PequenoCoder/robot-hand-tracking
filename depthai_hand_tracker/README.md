# DepthAI Hand Tracker

Real-time hand tracking for robotic hand control.

## Quick Start

```bash
pip install -r requirements.txt
python test_oak.py
python robot_hand.py
```

## Documentation

See [../README.md](../README.md) for complete documentation.

## Files

- `robot_hand.py` - Main program
- `HandTracker.py` - Hand tracking (host mode)
- `HandTrackerEdge.py` - Hand tracking (edge mode)
- `HandTrackerRenderer.py` - Visualization
- `test_oak.py` - Camera test
- `models/` - Neural network blobs
- `motors1_5/` - Arduino firmware
