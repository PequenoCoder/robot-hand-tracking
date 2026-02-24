# Robot Hand Tracking System

Real-time hand tracking using DepthAI OAK camera to control a robotic hand via Arduino/Teensy.

---

## Table of Contents

- [Hardware Requirements](#hardware-requirements)
- [Power Configuration](#power-configuration)
- [Software Installation](#software-installation)
  - [Raspberry Pi](#raspberry-pi)
  - [Windows](#windows)
- [Arduino Setup](#arduino-setup)
- [Running the Program](#running-the-program)
- [Auto-Start Configuration](#auto-start-configuration)
- [Service Management](#service-management)
- [Configuration Options](#configuration-options)
- [Troubleshooting](#troubleshooting)
- [Command Reference](#command-reference)

---

## Hardware Requirements

| Component | Specification |
|-----------|---------------|
| Computer | Raspberry Pi 4 (4GB+ RAM) or Windows PC |
| Camera | OAK-D, OAK-D Lite, or OAK-1 (DepthAI) |
| Microcontroller | Arduino Uno/Nano or Teensy |
| Servos | 5x standard hobby servos |
| Pi Power | 5V 5A USB-C |
| Servo Power | 5-6V 10A (separate supply) |
| Storage | 32GB+ microSD (for Pi) |

**Arduino Pin Mapping:**
```
Motor 1 (PIP)         → Pin 3
Motor 2 (MCP Flex)    → Pin 5
Motor 3 (MCP Abduct)  → Pin 6
Motor 4 (Base Flex)   → Pin 9
Motor 5 (Base Rotate) → Pin 10
```

---

## Power Configuration

**CRITICAL: Servos require separate power supply.**

```
Pi Power (5V 5A):
  - Raspberry Pi
  - OAK Camera (USB)
  - Arduino (USB, signal only)

Servo Power (5-6V 10A):
  - All servo V+ pins
  - All servo GND pins
  - Common GND to Arduino GND
```

**Current Requirements:**
- Raspberry Pi 4: 3A
- OAK Camera: 1.5A
- Arduino: 0.5A
- Each Servo: 1-2A peak
- Total Servo: 5-10A

**Brownout/Reset Symptoms:** Arduino LED flashing randomly, system instability, servo jitter. Solution: Use separate servo power supply.

---

## Software Installation

### Raspberry Pi

```bash
# System packages
sudo apt update
sudo apt install -y python3 python3-pip python3-venv build-essential

# USB rules for OAK camera
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# Clone repository
cd ~
git clone https://github.com/PequenoCoder/robot-hand-tracking.git
cd robot-hand-tracking/depthai_hand_tracker

# Python environment
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

# Serial port permissions
sudo usermod -a -G dialout $USER
# Log out and back in

# Test camera
python3 test_oak.py
```

### Windows

```powershell
# Clone repository
git clone https://github.com/PequenoCoder/robot-hand-tracking.git
cd robot-hand-tracking\depthai_hand_tracker

# Create virtual environment
python -m venv .venv
.\.venv\Scripts\Activate.ps1

# Install dependencies
pip install -r requirements.txt

# Test camera
python test_oak.py
```

---

## Arduino Setup

1. Upload `motors1_5/motors1_5.ino` to your Arduino
2. Serial baud rate: 115200
3. Verify upload successful before proceeding

**Find Serial Port:**

Windows:
```
Device Manager → Ports (COM & LPT)
Note COM port number (e.g., COM3)
```

Raspberry Pi:
```bash
ls /dev/tty{USB,ACM}*
# Usually /dev/ttyACM0
```

**Configure in robot_hand.py (line 15):**
```python
SERIAL_PORT = "COM3"  # Windows
SERIAL_PORT = "/dev/ttyACM0"  # Raspberry Pi
SERIAL_PORT = None  # Auto-detect
```

---

## Running the Program

### Manual Execution

**Raspberry Pi:**
```bash
cd ~/robot-hand-tracking/depthai_hand_tracker
source .venv/bin/activate
python3 robot_hand.py
```

**Windows:**
```powershell
cd robot-hand-tracking\depthai_hand_tracker
.\.venv\Scripts\Activate.ps1
python robot_hand.py
```

**Startup Sequence:**
1. Searches for Arduino (waits indefinitely)
2. Connects to Arduino
3. Initializes camera
4. Starts hand tracking and motor control

**Exit:** Press 'q', ESC, or Ctrl+C

---

## Auto-Start Configuration

**Raspberry Pi only. Requires desktop environment for terminal display.**

### Installation

```bash
cd ~/robot-hand-tracking/depthai_hand_tracker
chmod +x install_service.sh
sudo bash install_service.sh
sudo systemctl start robot_hand
```

The service:
- Starts on boot
- Opens terminal window with status
- Waits for Arduino connection
- Auto-restarts on failure (10 second delay)

### Manual Service Setup

If install script fails:

```bash
# Copy service file
sudo cp robot_hand.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable robot_hand
sudo systemctl start robot_hand
```

---

## Service Management

```bash
# Control
sudo systemctl start robot_hand
sudo systemctl stop robot_hand
sudo systemctl restart robot_hand
sudo systemctl status robot_hand

# Configuration
sudo systemctl enable robot_hand   # Enable auto-start
sudo systemctl disable robot_hand  # Disable auto-start

# Logs
sudo journalctl -u robot_hand -f        # Live tail
sudo journalctl -u robot_hand -n 50     # Last 50 lines
sudo journalctl -u robot_hand --since today

# Uninstall
sudo systemctl stop robot_hand
sudo systemctl disable robot_hand
sudo rm /etc/systemd/system/robot_hand.service
sudo systemctl daemon-reload
```

---

## Configuration Options

**Edit robot_hand.py:**

```python
# Serial Configuration (line 15-19)
SERIAL_PORT = None              # Port or None for auto-detect
BAUD_RATE = 115200              # Must match Arduino
UPDATE_RATE_HZ = 20             # Motor command frequency
ENABLE_MOTORS = True            # False to test without motors
DEBUG_SERIAL = False            # Show serial TX/RX

# Angle Processing (line 22-28)
USE_ROBUST_MCP = True           # Better orientation handling
DEBUG_ANGLES = False            # Console output for debugging
MCP_ZERO_OFFSET_DEG = 15.0      # Calibrate zero position
MCP_DEADBAND_DEG = 8.0          # Ignore small movements
PIP_ZERO_OFFSET_DEG = 20.0
PIP_DEADBAND_DEG = 10.0
PIP_FLEX_MAX_DEG = 60.0

# Filtering (line 31-34)
FILTER_CUTOFF_HZ = 10.0         # Higher = faster response
FILTER_ORDER = 2                # Filter complexity
FILTER_BUFFER_SIZE = 3          # Samples to buffer
MAX_MOTOR_CHANGE_PER_UPDATE = 80.0  # Rate limiting

# Position Hold (line 37-38)
HOLD_LAST_POSITION = True       # Keep position when hand lost
RESET_TIMEOUT_SECONDS = 5.0     # Reset after N seconds (0 = never)

# Motor Limits (line 40-41)
MOTOR_MIN = -80.0               # MUST match Arduino
MOTOR_MAX = 0.0

# Joint Mapping (line 44-49)
STRAIGHT_ANGLE = 180.0          # Fully extended
BENT_ANGLE = 120.0              # Fully bent
PIP_GAIN = 1.3                  # PIP sensitivity multiplier
```

**Edit Hand Tracker Settings (line 463-472):**
```python
tracker = HandTracker(
    input_src=None,             # Use OAK camera
    use_lm=True,                # Use landmark model
    use_world_landmarks=True,   # Real-world 3D coords
    use_gesture=False,          # Disable gestures
    xyz=False,                  # Disable spatial location
    solo=True,                  # Track one hand only
    resolution='full',          # 'full' or 'ultra'
    stats=True,                 # Show statistics
    trace=0                     # Debug trace level
)
```

---

## Troubleshooting

### Arduino Not Found

**Check connection:**
```bash
# Raspberry Pi
ls /dev/tty{USB,ACM}*

# Windows: Device Manager → Ports
```

**Verify permissions (Linux):**
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

**Test serial:**
```bash
# Linux
sudo chmod 666 /dev/ttyACM0

# Try different USB port
```

---

### Camera Issues

**Camera not detected:**
```bash
# Check USB
lsusb | grep -i luxonis  # Should show "Movidius MyriadX"

# Reinstall rules
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules

# Reboot
sudo reboot
```

**Test camera:**
```bash
python3 test_oak.py
```

---

### Power Issues

**Symptoms:** Arduino randomly resets, LED flashing, brownouts, servo jitter

**Diagnosis:**
```bash
# Check voltage (Raspberry Pi)
vcgencmd measure_volts  # Should be ~5.1V
vcgencmd get_throttled  # 0x0 = good, else power problem
```

**Solutions:**
1. Use separate servo power supply (5-6V 10A minimum)
2. Common ground between servo supply and Arduino
3. Do NOT power servos from Arduino/Pi USB
4. Shorter USB cables
5. Add 1000µF capacitor across Arduino 5V/GND

---

### Low FPS

**Raspberry Pi optimization:**

Use Edge mode (faster):
```python
# Edit robot_hand.py line 437
from HandTrackerEdge import HandTracker
```

Lower internal FPS:
```python
# Add to tracker initialization
internal_fps=20
```

Reduce resolution:
```python
resolution='full'  # Instead of 'ultra'
```

**System optimization:**
```bash
# Close other applications
# Use Raspberry Pi OS Lite (no desktop)

# Optional: Overclock (at your own risk)
sudo nano /boot/config.txt
# Add:
over_voltage=6
arm_freq=2000
```

---

### Import Errors

```bash
# Activate virtual environment first
source .venv/bin/activate

# Reinstall dependencies
pip install --upgrade -r requirements.txt
```

**Specific packages:**
```bash
pip install depthai>=2.13,<3.0
pip install opencv-python
pip install scipy
pip install pyserial
```

---

### Service Fails to Start

```bash
# Check logs
sudo journalctl -u robot_hand -n 100

# Verify paths
sudo systemctl cat robot_hand

# Test manually
cd ~/robot-hand-tracking/depthai_hand_tracker
.venv/bin/python3 robot_hand.py

# Reinstall service
sudo bash install_service.sh
```

---

## Command Reference

### Running

```bash
# Raspberry Pi
source .venv/bin/activate
python3 robot_hand.py

# Windows
.\.venv\Scripts\Activate.ps1
python robot_hand.py
```

### Service Control (Raspberry Pi)

```bash
sudo systemctl start robot_hand
sudo systemctl stop robot_hand
sudo systemctl restart robot_hand
sudo systemctl status robot_hand
sudo systemctl enable robot_hand
sudo systemctl disable robot_hand
```

### Logs

```bash
sudo journalctl -u robot_hand -f
sudo journalctl -u robot_hand -n 50
sudo journalctl -u robot_hand --since today
```

### Testing

```bash
python3 test_oak.py                    # Test camera
ls /dev/tty{USB,ACM}*                  # Find serial ports
lsusb                                  # Check USB devices
vcgencmd measure_volts                 # Check Pi voltage
vcgencmd get_throttled                 # Check Pi throttling
pip list | grep -E "depthai|opencv"    # Check packages
```

---

## Project Structure

```
robot-hand-tracking/
├── README.md                    # This file
├── .gitignore
│
└── depthai_hand_tracker/
    ├── robot_hand.py            # Main program
    ├── HandTracker.py           # Hand tracking (host)
    ├── HandTrackerEdge.py       # Hand tracking (edge)
    ├── HandTrackerRenderer.py   # Visualization
    ├── mediapipe_utils.py       # Utilities
    ├── hand_pose_fixes.py       # Pose calculations
    ├── FPS.py                   # FPS counter
    ├── test_oak.py              # Camera test
    ├── requirements.txt         # Dependencies
    ├── LICENSE.txt
    ├── robot_hand.service       # Systemd service
    ├── install_service.sh       # Service installer
    ├── models/                  # Neural network blobs
    └── motors1_5/
        └── motors1_5.ino        # Arduino firmware
```

---

## Quick Reference

### Dependencies

- `depthai>=2.13,<3.0` - OAK camera interface
- `opencv-python>=4.5.1` - Video processing
- `scipy>=1.7.0` - Signal filtering
- `pyserial>=3.5` - Arduino communication

### Serial Protocol

Format: `M1,M2,M3,M4,M5\n`

Example: `-45.5,-60.2,-30.0,-50.1,-20.5\n`

- Values: -80.0 to 0.0 degrees
- Rate: 20 Hz (configurable)
- Baud: 115200

### Key Files

- `robot_hand.py` - Main executable
- `robot_hand.service` - Systemd unit
- `install_service.sh` - Auto-start installer
- `motors1_5.ino` - Arduino firmware
- `requirements.txt` - Python packages

---

## License

MIT License - see [LICENSE.txt](depthai_hand_tracker/LICENSE.txt)
