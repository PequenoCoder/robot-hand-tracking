# Quick Start Guide

## For Windows

```powershell
cd C:\Users\lando\Desktop\LabFiles\robot-hand-tracking\depthai_hand_tracker
.\.venv\Scripts\Activate.ps1
python robot_hand.py
```

## For Raspberry Pi

### First Time Setup
```bash
cd ~/robot-hand-tracking/depthai_hand_tracker
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### Run Manually
```bash
source .venv/bin/activate
python3 robot_hand.py
```

### Install Auto-Start
```bash
chmod +x install_service.sh
sudo bash install_service.sh
sudo systemctl start robot_hand
```

See `AUTOSTART_SETUP.md` for detailed auto-start instructions.

## Controls

- **ESC** or **q** - Quit program
- **Ctrl+C** - Cancel/Stop

## What It Does

1. 🔍 Searches for Arduino (waits indefinitely)
2. ✅ Connects to Arduino
3. 📷 Initializes camera
4. 🤖 Controls robot hand based on your hand movements

## Troubleshooting

### Arduino Not Found
- Check USB connection
- Try different USB port
- Verify Arduino sketch is uploaded

### Camera Error
```bash
python3 test_oak.py  # Test camera separately
```

### Import Errors
```bash
pip install --upgrade -r requirements.txt
```

For full documentation, see `README_RASPBERRY_PI.md`
