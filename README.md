# Robot Hand Tracking System

Real-time hand tracking using DepthAI OAK camera to control a robotic hand via Arduino/Teensy.

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE.txt)

---

## 📋 Table of Contents

### Getting Started
- [Overview](#overview)
- [Hardware Requirements](#hardware-requirements)
- [Quick Start](#quick-start)
  - [Windows](#windows-quick-start)
  - [Raspberry Pi](#raspberry-pi-quick-start)

### Setup Guides
- [Raspberry Pi Complete Setup](#raspberry-pi-complete-setup)
  - [Initial OS Setup](#1-install-raspberry-pi-os)
  - [Installing Dependencies](#2-install-dependencies)
  - [Project Installation](#3-install-project)
  - [Testing](#4-test-the-system)
- [Arduino/Motor Setup](#arduino-motor-control-setup)
  - [Hardware Wiring](#hardware-wiring)
  - [Power Requirements](#power-requirements)
  - [Uploading Firmware](#uploading-arduino-code)
  - [Serial Configuration](#serial-port-configuration)

### Auto-Start Configuration
- [Auto-Start on Boot](#auto-start-raspberry-pi-on-boot)
  - [Quick Installation](#quick-auto-start-installation)
  - [Service Management](#managing-the-service)
  - [Viewing Logs](#viewing-logs)
  - [Troubleshooting Auto-Start](#troubleshooting-auto-start)

### Usage & Controls
- [Running the Program](#running-the-program)
- [Keyboard Controls](#keyboard-controls)
- [Understanding the Output](#understanding-the-output)

### Troubleshooting
- [Common Issues](#troubleshooting)
  - [Arduino Connection Issues](#arduino-not-found)
  - [Camera Problems](#camera-issues)
  - [Performance Issues](#low-fps--performance-issues)
  - [Power Problems](#power-issues)
  - [Import/Dependency Errors](#import-errors)

### Advanced Topics
- [Performance Optimization](#performance-tips)
- [Headless Operation](#headless-mode-no-display)
- [VNC Setup](#vnc-setup-for-remote-viewing)
- [Custom Configuration](#configuration-options)

### Reference
- [Project Structure](#project-structure)
- [Command Reference](#command-reference)
- [Contributing](#contributing)
- [License](#license)

---

## Overview

This system uses:
- **OAK-D Camera** - Real-time hand tracking with MediaPipe
- **Raspberry Pi** or **Windows PC** - Processing unit
- **Arduino/Teensy** - Motor controller
- **5 Servo Motors** - Robotic hand actuation

**Features:**
- ✅ Real-time hand tracking (30 FPS)
- ✅ Finger joint angle calculation
- ✅ Smooth servo control with filtering
- ✅ Auto-connects to Arduino
- ✅ Auto-start on Raspberry Pi boot
- ✅ Visual feedback with colored terminal output
- ✅ Robust error handling

---

## Hardware Requirements

### Required Components

| Component | Specification | Notes |
|-----------|---------------|-------|
| **Computer** | Raspberry Pi 4 (4GB+ RAM) or Windows PC | Pi 3B+ works but slower |
| **Camera** | OAK-D, OAK-D Lite, or OAK-1 | USB 3.0 recommended |
| **Microcontroller** | Arduino Uno/Nano or Teensy | For motor control |
| **Servos** | 5x standard hobby servos | MG90S or similar |
| **Power Supply (Pi)** | 5V 5A USB-C | Official Raspberry Pi adapter |
| **Power Supply (Servos)** | 5-6V 5-10A | **SEPARATE from Pi power!** |
| **MicroSD Card** | 32GB+ Class 10 | For Raspberry Pi OS |
| **USB Cables** | USB-C to A, Micro USB | For camera & Arduino |

### Optional but Recommended
- Heatsink/fan for Raspberry Pi
- Powered USB 3.0 hub
- Monitor, keyboard, mouse (for initial setup)
- VNC for remote access

### ⚠️ IMPORTANT: Power Requirements

**DO NOT power servos from Raspberry Pi USB!**

```
✅ CORRECT SETUP:
[5V 5A Supply] → Raspberry Pi
                  ├→ OAK Camera (USB)
                  └→ Arduino (USB, control only)

[Separate 5-6V 10A Supply] → Servo Motors
                              └→ GND to Arduino GND (common ground!)

❌ WRONG (Will cause brownouts/crashes):
[5V Supply] → Pi → Camera + Arduino → Servos (TOO MUCH LOAD!)
```

**Why Separate Power?**
- Each servo: 1-2A peak
- 5 servos: 5-10A total
- Pi + Camera: 4-5A
- **Total would be 10-15A** - Way too much for Pi power supply!

---

## Quick Start

### Windows Quick Start

```powershell
# 1. Clone repository
git clone https://github.com/PequenoCoder/robot-hand-tracking.git
cd robot-hand-tracking\depthai_hand_tracker

# 2. Create virtual environment
python -m venv .venv
.\.venv\Scripts\Activate.ps1

# 3. Install dependencies
pip install -r requirements.txt

# 4. Connect Arduino and upload firmware (see Arduino Setup section)

# 5. Run the program
python robot_hand.py
```

**What happens:**
1. Searches for Arduino (waits until found)
2. Connects to Arduino
3. Initializes camera
4. Shows control windows
5. Mirrors your hand movements to robot hand

---

### Raspberry Pi Quick Start

```bash
# 1. Clone repository
cd ~
git clone https://github.com/PequenoCoder/robot-hand-tracking.git
cd robot-hand-tracking/depthai_hand_tracker

# 2. Create virtual environment
python3 -m venv .venv
source .venv/bin/activate

# 3. Install dependencies (takes 10-20 minutes)
pip install -r requirements.txt

# 4. Test camera
python3 test_oak.py

# 5. Connect Arduino (see Arduino Setup section)

# 6. Run the program
python3 robot_hand.py
```

---

## Raspberry Pi Complete Setup

### 1. Install Raspberry Pi OS

**Using Raspberry Pi Imager (Recommended):**

1. Download [Raspberry Pi Imager](https://www.raspberrypi.com/software/)
2. Insert microSD card (32GB+)
3. Open Raspberry Pi Imager
4. Choose OS: **Raspberry Pi OS (64-bit)**
5. Click Settings (⚙️):
   - Set hostname: `robothand`
   - Enable SSH
   - Set username/password
   - Configure WiFi
6. Write and wait for completion
7. Insert card into Pi and power on

**First Boot:**
```bash
# SSH into Pi
ssh pi@robothand.local

# Update system
sudo apt update && sudo apt upgrade -y
sudo reboot
```

---

### 2. Install Dependencies

```bash
# Update package lists
sudo apt update

# Install Python and build tools
sudo apt install -y python3 python3-pip python3-venv
sudo apt install -y build-essential cmake pkg-config

# Install OpenCV dependencies
sudo apt install -y libopencv-dev python3-opencv
sudo apt install -y libjpeg-dev libtiff-dev libpng-dev
sudo apt install -y libavcodec-dev libavformat-dev libswscale-dev

# Install USB rules for OAK camera
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**Optional: Increase swap (for Pi with 4GB RAM or less):**
```bash
sudo dphys-swapfile swapoff
sudo nano /etc/dphys-swapfile
# Change CONF_SWAPSIZE=100 to CONF_SWAPSIZE=2048
sudo dphys-swapfile setup
sudo dphys-swapfile swapon
```

---

### 3. Install Project

```bash
# Clone repository
cd ~
git clone https://github.com/PequenoCoder/robot-hand-tracking.git
cd robot-hand-tracking/depthai_hand_tracker

# Create virtual environment
python3 -m venv .venv
source .venv/bin/activate

# Install Python packages (10-20 minutes)
pip install -r requirements.txt
```

**Dependencies installed:**
- `opencv-python` - Video processing
- `depthai` - OAK camera interface
- `scipy` - Signal processing/filtering
- `pyserial` - Arduino communication

---

### 4. Test the System

```bash
# Test camera first
source .venv/bin/activate
python3 test_oak.py
```

Should show: Camera detected and working

```bash
# Test hand tracking (without Arduino)
python3 robot_hand.py
```

Will wait for Arduino - Press Ctrl+C to cancel

---

## Arduino Motor Control Setup

### Hardware Wiring

**Arduino Pin Connections:**
```
Motor 1 (PIP Joint)      → Pin 3
Motor 2 (MCP Flexion)    → Pin 5
Motor 3 (MCP Abduction)  → Pin 6
Motor 4 (Base Flex)      → Pin 9
Motor 5 (Base Rotation)  → Pin 10
```

**Servo Wiring:**
```
EACH SERVO:
  Brown/Black  → Servo Power Supply GND
  Red          → Servo Power Supply +5V
  Orange/White → Arduino signal pin

IMPORTANT:
  Servo GND → Arduino GND (common ground required!)
```

**Complete Power Setup:**
```
Pi Power (5V 5A):
  ├─ Raspberry Pi (USB-C)
  ├─ OAK Camera (USB 3.0)
  └─ Arduino (USB, control only)

Servo Power (5-6V 10A):
  ├─ All servo V+ pins
  ├─ All servo GND pins
  └─ GND connected to Arduino GND
```

---

### Power Requirements

**Current Draw:**
| Device | Current | Power |
|--------|---------|-------|
| Raspberry Pi 4 | 3A peak | 15W |
| OAK Camera | 1-1.5A | 5-7.5W |
| Arduino | 0.5A | 2.5W |
| **Servo (each)** | **1-2A peak** | **5-10W** |
| **5 Servos Total** | **5-10A** | **25-50W** |

**Why you need separate servo power:**
- Total system with servos: 10-15A
- Your Pi power supply: 5A max
- **Powering servos from Pi = brownouts, crashes, damage**

**Recommended Servo Power Supplies:**
1. **5V 10A Switching Supply** - Most reliable
2. **6V Battery Pack** - Portable option
3. **LiPo + 5V BEC** - For mobile robots

---

### Uploading Arduino Code

**Arduino IDE Method:**
1. Open Arduino IDE
2. File → Open → `motors1_5/motors1_5.ino`
3. Tools → Board → Select your Arduino
4. Tools → Port → Select COM port
5. Upload (➡️ button)
6. Wait for "Done uploading"

**Platform.io Method:**
```bash
pio run --target upload
```

**Verify Upload:**
- Arduino LED should be solid (not flashing rapidly)
- Serial Monitor (115200 baud) should show startup message

---

### Serial Port Configuration

**Find Arduino Port:**

**Windows:**
```powershell
# Device Manager → Ports (COM & LPT)
# Look for "Arduino" or "USB Serial Device"
# Note the COM port (e.g., COM3)
```

**Linux/Raspberry Pi:**
```bash
# List USB devices
ls /dev/tty{USB,ACM}*

# Usually /dev/ttyACM0 or /dev/ttyUSB0

# Give permission
sudo usermod -a -G dialout $USER
# Log out and back in
```

**Configure in robot_hand.py:**

Edit line 15:
```python
SERIAL_PORT = "COM3"  # Windows
# or
SERIAL_PORT = "/dev/ttyACM0"  # Raspberry Pi
```

Or set to `None` for auto-detection:
```python
SERIAL_PORT = None  # Auto-detect
```

---

## Auto-Start Raspberry Pi on Boot

### Quick Auto-Start Installation

```bash
cd ~/robot-hand-tracking/depthai_hand_tracker
chmod +x install_service.sh
sudo bash install_service.sh
```

**What the installer does:**
- ✅ Creates systemd service with correct paths
- ✅ Enables auto-start on boot
- ✅ Configures auto-restart on failure
- ✅ Sets up terminal window display
- ✅ Shows management commands

**Start service now:**
```bash
sudo systemctl start robot_hand
```

**Reboot to test:**
```bash
sudo reboot
```

After reboot, terminal window opens automatically with colored status output!

---

### Managing the Service

```bash
# Check status
sudo systemctl status robot_hand

# Start service
sudo systemctl start robot_hand

# Stop service
sudo systemctl stop robot_hand

# Restart service
sudo systemctl restart robot_hand

# Enable auto-start (on by default)
sudo systemctl enable robot_hand

# Disable auto-start
sudo systemctl disable robot_hand
```

---

### Viewing Logs

```bash
# Watch live logs (Ctrl+C to exit)
sudo journalctl -u robot_hand -f

# Show last 50 lines
sudo journalctl -u robot_hand -n 50

# Show today's logs
sudo journalctl -u robot_hand --since today

# Show logs with errors only
sudo journalctl -u robot_hand -p err
```

---

### Troubleshooting Auto-Start

**Service won't start:**
```bash
# Check detailed error
sudo journalctl -u robot_hand -n 100

# Verify paths
sudo systemctl cat robot_hand

# Test command manually
cd ~/robot-hand-tracking/depthai_hand_tracker
.venv/bin/python3 robot_hand.py
```

**Terminal window doesn't appear:**
```bash
# Check display environment
echo $DISPLAY  # Should show :0

# Reinstall service
sudo bash install_service.sh
sudo systemctl restart robot_hand
```

---

## Running the Program

### Manual Run

**Raspberry Pi:**
```bash
cd ~/robot-hand-tracking/depthai_hand_tracker
source .venv/bin/activate
python3 robot_hand.py
```

**Windows:**
```powershell
cd C:\...\robot-hand-tracking\depthai_hand_tracker
.\.venv\Scripts\Activate.ps1
python robot_hand.py
```

### What You'll See

**Terminal Output:**
```
============================================================
🤖 ROBOT HAND CONTROL - STARTING...
============================================================

[1/2] 🔍 Searching for Arduino...
⏳ Waiting for Arduino...
   Please connect Arduino via USB

   📡 Found Arduino on /dev/ttyACM0, connecting...
✓ Arduino connected on /dev/ttyACM0

[2/2] 📷 Initializing hand tracker...
✓ Hand tracker initialized

============================================================
🤖 ROBOT HAND CONTROL ACTIVE ✋
============================================================
👋 Show your hand to the camera to control the robot
⌨️  Press 'q' or ESC to quit
============================================================
```

**Color Coding:**
- 🟢 **Green** = Success/Active
- 🟡 **Yellow** = Waiting/Info
- 🔴 **Red** = Errors
- 🔵 **Blue** = Debug info

**Camera Window:**
- Shows live video feed
- Hand skeleton overlay
- Joint angle visualization
- FPS counter

---

## Keyboard Controls

| Key | Action |
|-----|--------|
| **ESC** or **q** | Quit program |
| **Ctrl+C** | Cancel/Stop (also during Arduino search) |
| **Space** | Pause video (if implemented) |

---

## Understanding the Output

### Terminal Console

Shows real-time status:
- Arduino connection status
- Camera initialization
- Hand detection status
- Motor commands (if DEBUG enabled)
- Error messages with troubleshooting tips

### Camera Window

Visual feedback:
- Live hand tracking
- Skeleton overlay on detected hand
- Joint angle annotations
- FPS performance indicator

---

## Troubleshooting

### Arduino Not Found

**Symptoms:**
- "⏳ Waiting for Arduino..." message loops
- Program never starts camera

**Solutions:**

1. **Check USB connection:**
   ```bash
   # Raspberry Pi
   ls /dev/tty{USB,ACM}*
   
   # Windows
   # Check Device Manager → Ports
   ```

2. **Verify Arduino sketch uploaded:**
   - Open Arduino IDE
   - Re-upload `motors1_5.ino`

3. **Check permissions (Linux):**
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and back in
   ```

4. **Try different USB port:**
   - Use blue USB 3.0 ports on Pi
   - Avoid USB hubs if possible

5. **Check cable:**
   - Use data cable (not charge-only)
   - Try different cable

---

### Camera Issues

**Camera not detected:**
```bash
# Check USB connection
lsusb | grep -i luxonis

# Should show: "Movidius MyriadX"

# Reinstall udev rules
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# Reboot
sudo reboot
```

**Camera test:**
```bash
python3 test_oak.py
```

---

### Low FPS / Performance Issues

**On Raspberry Pi:**

1. **Use Edge mode** (faster):
   ```python
   # Edit robot_hand.py, line 437
   from HandTrackerEdge import HandTracker  # Instead of HandTracker
   ```

2. **Lower internal FPS:**
   ```python
   # Edit robot_hand.py, add to tracker initialization:
   internal_fps=20  # Default is 30
   ```

3. **Overclock Pi (at your own risk):**
   ```bash
   sudo nano /boot/config.txt
   # Add:
   over_voltage=6
   arm_freq=2000
   gpu_freq=750
   
   sudo reboot
   ```

4. **Close other applications**

5. **Use Raspberry Pi OS Lite** (no desktop)

---

### Power Issues

**Symptoms:**
- Arduino randomly resets/flashing LED
- Pi crashes/reboots
- Servo jitter
- Under-voltage warning (⚡ icon)

**Check voltage:**
```bash
vcgencmd measure_volts
# Should be ~5.1V

vcgencmd get_throttled
# 0x0 = good, anything else = power problem
```

**Solutions:**

1. **Use separate servo power supply** (REQUIRED!)
2. **Better Pi power supply** (5V 5A minimum)
3. **Shorter USB cables**
4. **Remove USB devices you don't need**
5. **Add capacitor** (1000µF) across Arduino 5V/GND

---

### Import Errors

**"No module named 'depthai'":**
```bash
# Activate virtual environment first!
source .venv/bin/activate  # Linux
.\.venv\Scripts\Activate.ps1  # Windows

# Then reinstall
pip install --upgrade -r requirements.txt
```

**"No module named 'cv2'":**
```bash
pip install opencv-python
```

**General fix:**
```bash
pip install --upgrade pip setuptools wheel
pip install --upgrade -r requirements.txt
```

---

## Performance Tips

### Best Settings for Raspberry Pi 4

```bash
# Recommended launch command
python3 robot_hand.py
```

Configuration (in `robot_hand.py`):
```python
# Best balance of speed and accuracy:
tracker = HandTracker(
    input_src=None,
    use_lm=True,
    use_world_landmarks=True,
    solo=True,              # Track one hand only
    resolution='full',      # Or 'ultra' for better quality
    stats=False,            # Disable stats for slight speedup
)
```

---

### Headless Mode (No Display)

For running without monitor (SSH only):

**Option 1: Comment out display:**
```python
# Edit robot_hand.py, line ~683
# cv2.imshow("Joint Angle Capture", frame)
```

**Option 2: Use VNC** (see below)

---

### VNC Setup for Remote Viewing

Enable VNC on Raspberry Pi:
```bash
sudo raspi-config
# → Interface Options → VNC → Enable
```

From your computer:
1. Download [VNC Viewer](https://www.realvnc.com/download/viewer/)
2. Connect to `robothand.local:5900`
3. See camera feed remotely!

---

## Configuration Options

### In robot_hand.py

```python
# Line 15-19: Serial Configuration
SERIAL_PORT = None  # Auto-detect or "COM3" / "/dev/ttyACM0"
BAUD_RATE = 115200
UPDATE_RATE_HZ = 20  # Motor update frequency
ENABLE_MOTORS = True  # Set False to test without motors
DEBUG_SERIAL = False  # Show serial communication

# Line 22-28: Angle Processing
USE_ROBUST_MCP = True  # Better orientation handling
DEBUG_ANGLES = False   # Show raw angle values
MCP_ZERO_OFFSET_DEG = 15.0  # Calibrate zero position
MCP_DEADBAND_DEG = 8.0      # Ignore small movements

# Line 31-34: Filtering
FILTER_CUTOFF_HZ = 10.0  # Higher = faster, less smooth
FILTER_ORDER = 2
FILTER_BUFFER_SIZE = 3
MAX_MOTOR_CHANGE_PER_UPDATE = 80.0  # Rate limiting

# Line 37-38: Position Hold
HOLD_LAST_POSITION = True  # Keep position when hand lost
RESET_TIMEOUT_SECONDS = 5.0  # Reset after N seconds
```

---

## Project Structure

```
robot-hand-tracking/
├── README.md                          # This file
├── .gitignore                         # Git ignore rules
│
├── depthai_hand_tracker/              # Main project folder
│   ├── robot_hand.py                  # Main program ⭐
│   ├── HandTracker.py                 # Hand tracking (host mode)
│   ├── HandTrackerEdge.py             # Hand tracking (edge mode)
│   ├── HandTrackerRenderer.py         # Visualization
│   ├── mediapipe_utils.py             # MediaPipe utilities
│   ├── hand_pose_fixes.py             # Pose calculations
│   ├── FPS.py                         # FPS counter
│   ├── test_oak.py                    # Camera test utility
│   ├── requirements.txt               # Python dependencies
│   ├── README.md                      # Quick reference
│   ├── LICENSE.txt                    # License
│   │
│   ├── models/                        # Neural network models
│   │   ├── palm_detection_sh4.blob
│   │   ├── hand_landmark_lite_sh4.blob
│   │   ├── hand_landmark_full_sh4.blob
│   │   └── ...more models
│   │
│   ├── motors1_5/                     # Arduino firmware
│   │   └── motors1_5.ino              # Motor control sketch
│   │
│   ├── robot_hand.service             # Systemd service file
│   └── install_service.sh             # Auto-start installer
│
└── [Deleted: examples/, img/, custom_models/]  # Cleanup
```

---

## Command Reference

### Raspberry Pi Service Commands

```bash
# Service control
sudo systemctl start robot_hand        # Start now
sudo systemctl stop robot_hand         # Stop
sudo systemctl restart robot_hand      # Restart
sudo systemctl status robot_hand       # Check status
sudo systemctl enable robot_hand       # Enable auto-start
sudo systemctl disable robot_hand      # Disable auto-start

# Logs
sudo journalctl -u robot_hand -f       # Live logs
sudo journalctl -u robot_hand -n 50    # Last 50 lines
sudo journalctl -u robot_hand --since today  # Today's logs

# Installation
chmod +x install_service.sh
sudo bash install_service.sh

# Uninstall
sudo systemctl stop robot_hand
sudo systemctl disable robot_hand
sudo rm /etc/systemd/system/robot_hand.service
sudo systemctl daemon-reload
```

### Testing Commands

```bash
# Test camera
python3 test_oak.py

# Test hand tracking (no motors)
python3 robot_hand.py  # Press Ctrl+C when Arduino search starts

# Check Python packages
pip list | grep -E "depthai|opencv|scipy|serial"

# Check USB devices
lsusb  # Should show Movidius and Arduino

# Check serial ports
ls /dev/tty{USB,ACM}*  # Raspberry Pi
# Device Manager → Ports  # Windows

# Check system voltage (Pi)
vcgencmd measure_volts
vcgencmd get_throttled  # 0x0 = good
```

---

## Contributing

Contributions welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

---

## License

This project is licensed under the MIT License - see [LICENSE.txt](depthai_hand_tracker/LICENSE.txt) for details.

---

## Support

**Issues?** Check the [Troubleshooting](#troubleshooting) section above.

**Still stuck?**
- GitHub Issues: [Report a bug](https://github.com/PequenoCoder/robot-hand-tracking/issues)
- Check DepthAI docs: https://docs.luxonis.com/

---

**Made with ❤️ for robotics education and research**

🤖 Happy Building! ✋
