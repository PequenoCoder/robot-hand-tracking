# Raspberry Pi Auto-Start Setup Guide

This guide shows you how to make the robot hand control program start automatically when your Raspberry Pi boots up.

## Quick Setup (Recommended)

### 1. Clone or Update Repository on Raspberry Pi

```bash
cd ~
git clone https://github.com/PequenoCoder/robot-hand-tracking.git
# Or if already cloned:
cd ~/robot-hand-tracking
git pull origin main
```

### 2. Make Sure Everything Works First

```bash
cd ~/robot-hand-tracking/depthai_hand_tracker
source .venv/bin/activate
python3 robot_hand.py
```

Press Ctrl+C to stop when confirmed working.

### 3. Install Auto-Start Service

```bash
cd ~/robot-hand-tracking/depthai_hand_tracker
chmod +x install_service.sh
sudo bash install_service.sh
```

The installer will:
- ✅ Create a systemd service
- ✅ Set correct paths automatically
- ✅ Enable auto-start on boot
- ✅ Show you useful commands

### 4. Start the Service Now

```bash
sudo systemctl start robot_hand
```

### 5. Check Status

```bash
sudo systemctl status robot_hand
```

You should see: `Active: active (running)`

## Useful Commands

### Control the Service

```bash
# Start the robot hand control
sudo systemctl start robot_hand

# Stop the robot hand control
sudo systemctl stop robot_hand

# Restart the robot hand control
sudo systemctl restart robot_hand

# Check if it's running
sudo systemctl status robot_hand

# View live logs
sudo journalctl -u robot_hand -f

# View recent logs
sudo journalctl -u robot_hand -n 50

# Disable auto-start (won't start on boot)
sudo systemctl disable robot_hand

# Re-enable auto-start
sudo systemctl enable robot_hand
```

## Manual Setup (Alternative Method)

If you prefer to set it up manually:

### 1. Edit the Service File

```bash
cd ~/robot-hand-tracking/depthai_hand_tracker
nano robot_hand.service
```

Make sure the paths match your setup:
- `User=` should be your username (probably `pi`)
- `WorkingDirectory=` should point to the depthai_hand_tracker folder
- `ExecStart=` should point to your Python executable and script

### 2. Copy to Systemd

```bash
sudo cp robot_hand.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable robot_hand
```

### 3. Start the Service

```bash
sudo systemctl start robot_hand
```

## Troubleshooting

### Service Won't Start

```bash
# Check detailed error messages
sudo journalctl -u robot_hand -n 100

# Check if paths are correct
sudo systemctl cat robot_hand

# Test the command manually
cd ~/robot-hand-tracking/depthai_hand_tracker
.venv/bin/python3 robot_hand.py
```

### Arduino Not Found on Boot

The service will wait indefinitely for the Arduino to be connected. You can:

1. Connect Arduino before powering on the Pi
2. Or connect it anytime - the service will detect it automatically

Check logs to see the waiting status:
```bash
sudo journalctl -u robot_hand -f
```

### Camera Issues

If the camera doesn't initialize:

```bash
# Check if camera is detected
lsusb | grep -i luxonis

# Restart the service
sudo systemctl restart robot_hand
```

### Python Environment Issues

```bash
# Make sure virtual environment exists
ls ~/robot-hand-tracking/depthai_hand_tracker/.venv/

# If not, recreate it
cd ~/robot-hand-tracking/depthai_hand_tracker
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Viewing the Video Feed

By default, the service runs in the background. To see the video feed:

### Option 1: VNC Viewer

1. Enable VNC on Raspberry Pi:
   ```bash
   sudo raspi-config
   # Interface Options → VNC → Enable
   ```

2. Connect with VNC Viewer from your computer
3. You'll see the camera feed window

### Option 2: Connect Monitor Directly

- Connect HDMI monitor to Raspberry Pi
- The camera feed will appear on the display

### Option 3: Headless Mode (No Display)

Edit `robot_hand.py` and comment out display lines:
```python
# cv2.imshow("Joint Angle Capture", frame)
```

The robot will still work based on serial commands.

## Disabling Auto-Start

If you want to disable auto-start:

```bash
sudo systemctl stop robot_hand
sudo systemctl disable robot_hand
```

To completely remove:

```bash
sudo systemctl stop robot_hand
sudo systemctl disable robot_hand
sudo rm /etc/systemd/system/robot_hand.service
sudo systemctl daemon-reload
```

## Performance Tips for Auto-Start

### 1. Delay Start (Optional)

If you want to wait a bit after boot before starting:

Edit `/etc/systemd/system/robot_hand.service`:
```ini
[Service]
ExecStartPre=/bin/sleep 10
```

### 2. Priority Boost

Give the service higher priority:
```ini
[Service]
Nice=-10
```

### 3. Auto-Restart on Crash

Already configured! The service will restart automatically if it crashes:
```ini
Restart=on-failure
RestartSec=10
```

## Checking Boot Status

After rebooting your Pi:

```bash
# Wait a few seconds for boot, then:
sudo systemctl status robot_hand

# Or watch it start:
sudo journalctl -u robot_hand -f
```

You should see:
```
⏳ Waiting for Arduino...
✓ Arduino connected on /dev/ttyACM0
[2/2] Initializing hand tracker...
✓ Hand tracker initialized
ROBOT HAND CONTROL ACTIVE
```

## Summary

✅ **Auto-start installed** - Robot hand starts on boot
✅ **Waits for Arduino** - No manual intervention needed  
✅ **Auto-restart** - Recovers from crashes
✅ **Easy control** - Simple systemctl commands
✅ **Full logging** - journalctl for debugging

Your Raspberry Pi is now a plug-and-play robot hand controller! 🤖✋
