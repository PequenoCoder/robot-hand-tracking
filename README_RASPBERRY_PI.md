# Robot Hand Tracking - Raspberry Pi Setup Guide

Complete guide to run the hand tracking system on a Raspberry Pi for robotic hand control.

## Table of Contents
- [Hardware Requirements](#hardware-requirements)
- [Raspberry Pi Initial Setup](#raspberry-pi-initial-setup)
- [Installing Dependencies](#installing-dependencies)
- [Project Setup](#project-setup)
- [Running the Hand Tracker](#running-the-hand-tracker)
- [Arduino/Motor Control Setup](#arduinomotor-control-setup)
- [Troubleshooting](#troubleshooting)

---

## Hardware Requirements

### Required Hardware
- **Raspberry Pi 4** (4GB or 8GB RAM recommended)
  - Raspberry Pi 3B+ works but may have performance limitations
- **OAK-D Camera** (DepthAI compatible)
  - OAK-D, OAK-D Lite, or OAK-1
  - USB 3.0 connection recommended for best performance
- **MicroSD Card** (32GB minimum, Class 10 or better)
- **Power Supply** (Official Raspberry Pi 5V 3A USB-C adapter recommended)
- **Arduino** (for motor control) - Arduino Uno, Nano, or compatible
- **Servo Motors** (for robotic hand - see motor requirements below)
- **USB Cable** for Arduino connection

### Optional but Recommended
- Heatsinks or fan for Raspberry Pi (tracking is CPU-intensive)
- Powered USB 3.0 hub (if powering multiple devices)
- Monitor, keyboard, mouse for initial setup (or use SSH)

---

## Raspberry Pi Initial Setup

### 1. Install Raspberry Pi OS

**Option A: Using Raspberry Pi Imager (Recommended)**
1. Download [Raspberry Pi Imager](https://www.raspberrypi.com/software/)
2. Insert microSD card into your computer
3. Open Raspberry Pi Imager
4. Choose OS: **Raspberry Pi OS (64-bit)** recommended
5. Choose Storage: Select your microSD card
6. Click Settings (gear icon):
   - Set hostname (e.g., `robothand`)
   - Enable SSH
   - Set username and password
   - Configure WiFi (optional)
7. Click "Write" and wait for completion

**Option B: Manual Download**
1. Download [Raspberry Pi OS](https://www.raspberrypi.com/software/operating-systems/)
2. Flash to microSD using [balenaEtcher](https://www.balena.io/etcher/)

### 2. First Boot

1. Insert microSD card into Raspberry Pi
2. Connect OAK-D camera, monitor, keyboard, mouse
3. Power on Raspberry Pi
4. Complete initial setup wizard if using desktop version

### 3. Connect via SSH (Optional)

If you enabled SSH during imaging:
```bash
ssh pi@robothand.local
# or use the IP address
ssh pi@<raspberry-pi-ip>
```

### 4. Update System

```bash
sudo apt update
sudo apt upgrade -y
sudo reboot
```

---

## Installing Dependencies

### 1. Install System Packages

```bash
# Update package lists
sudo apt update

# Install Python 3 and pip
sudo apt install -y python3 python3-pip python3-venv

# Install OpenCV dependencies
sudo apt install -y libopencv-dev python3-opencv

# Install build tools
sudo apt install -y build-essential cmake pkg-config

# Install image/video libraries
sudo apt install -y libjpeg-dev libtiff-dev libpng-dev
sudo apt install -y libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
sudo apt install -y libxvidcore-dev libx264-dev

# Install GUI libraries (if using display)
sudo apt install -y libgtk-3-dev libcanberra-gtk3-module

# Install USB and udev rules for OAK camera
sudo apt install -y udev

# Install Arduino CLI tools (optional, for programming Arduino from Pi)
sudo apt install -y arduino
```

### 2. Set up USB Rules for OAK Camera

```bash
# Download and install DepthAI udev rules
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 3. Increase Swap Size (Recommended for Pi with 4GB RAM or less)

```bash
# Stop swap
sudo dphys-swapfile swapoff

# Edit swap configuration
sudo nano /etc/dphys-swapfile
# Change CONF_SWAPSIZE=100 to CONF_SWAPSIZE=2048

# Restart swap
sudo dphys-swapfile setup
sudo dphys-swapfile swapon
```

---

## Project Setup

### 1. Clone the Repository

```bash
# Navigate to home directory
cd ~

# Clone the repository
git clone https://github.com/PequenoCoder/robot-hand-tracking.git

# Enter project directory
cd robot-hand-tracking/depthai_hand_tracker
```

### 2. Create Python Virtual Environment

```bash
# Create virtual environment
python3 -m venv .venv

# Activate virtual environment
source .venv/bin/activate

# Upgrade pip
pip install --upgrade pip
```

### 3. Install Python Dependencies

```bash
# Install project requirements
pip install -r requirements.txt

# If you encounter issues, install packages individually:
pip install opencv-python>=4.5.1.48
pip install depthai>=2.13
pip install scipy>=1.7.0
pip install pyserial>=3.5
```

**Note:** Installing `depthai` may take 10-20 minutes on Raspberry Pi. Be patient!

### 4. Test OAK Camera Connection

```bash
# Activate virtual environment if not already active
source .venv/bin/activate

# Test camera
python3 test_oak.py
```

If you see camera feed, your OAK camera is working correctly!

---

## Running the Hand Tracker

### Basic Hand Tracking Demo

```bash
# Activate virtual environment
source .venv/bin/activate

# Run basic demo (Host mode)
python3 demo.py

# Run with Edge mode (recommended - faster)
python3 demo.py -e

# Run with gesture recognition
python3 demo.py -e -g
```

### Keyboard Controls During Demo

| Key | Function |
|-----|----------|
| **Esc** | Exit program |
| **Space** | Pause/Resume |
| **1** | Show/hide palm bounding box |
| **2** | Show/hide palm detection keypoints |
| **3** | Show/hide rotated bounding box |
| **4** | Show/hide landmarks |
| **5** | Show/hide handedness |
| **6** | Show/hide scores |
| **7** | Show/hide gestures |
| **f** | Show/hide FPS |

### For Robotic Hand Control

```bash
# Activate virtual environment
source .venv/bin/activate

# Run robot hand control
python3 robot_hand.py
```

**This will:**
- Connect to Arduino first (required)
- Initialize camera after successful Arduino connection
- Track your hand in real-time
- Calculate joint angles for each finger
- Send commands to control the robotic hand
- Display angles on screen

**Joint angles captured per finger:**
- **PIP angle** - Interphalangeal joint
- **MCP flexion** - Metacarpophalangeal flex/extend
- **MCP abduction** - Metacarpophalangeal spread/close

---

## Arduino/Motor Control Setup

### 1. Upload Arduino Code

1. Connect Arduino to your computer (not Pi initially)
2. Open Arduino IDE
3. Open `depthai_hand_tracker/motors1_5/motors1_5.ino`
4. Select your Arduino board: Tools → Board
5. Select correct port: Tools → Port
6. Click Upload

### 2. Connect Arduino to Raspberry Pi

```bash
# After uploading code, connect Arduino to Raspberry Pi via USB

# Check if Arduino is detected
ls /dev/ttyUSB* /dev/ttyACM*
# You should see something like /dev/ttyACM0 or /dev/ttyUSB0

# Give permission to access serial port
sudo usermod -a -G dialout $USER
# Log out and log back in for this to take effect
```

### 3. Configure Serial Port in Python

Edit `robot_hand.py` to match your Arduino port:

```python
# Find this line and update if needed
serial_port = '/dev/ttyACM0'  # or /dev/ttyUSB0
```

### 4. Test Full System

```bash
source .venv/bin/activate
python3 robot_hand.py
```

Your robotic hand should now mirror your real hand movements!

---

## Troubleshooting

### OAK Camera Not Detected

```bash
# Check USB connection
lsusb | grep -i luxonis

# Reinstall udev rules
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# Reboot
sudo reboot
```

### Low FPS / Performance Issues

**1. Use Edge Mode:**
```bash
python3 demo.py -e
```

**2. Lower Internal FPS:**
```bash
python3 demo.py -e --internal_fps 20
```

**3. Use Solo Mode (detect 1 hand only):**
```bash
python3 demo.py -e -s
```

**4. Reduce Resolution:**
```bash
python3 demo.py -e --resolution full  # instead of ultra
```

**5. Overclock Raspberry Pi (at your own risk):**
```bash
sudo nano /boot/config.txt
# Add these lines:
over_voltage=6
arm_freq=2000
gpu_freq=750

# Save and reboot
sudo reboot
```

### Import Errors

```bash
# If you get "No module named 'cv2'"
pip install opencv-python

# If you get "No module named 'depthai'"
pip install depthai

# If installation fails, try:
pip install --upgrade pip setuptools wheel
```

### Permission Denied for Serial Port

```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Or give temporary permission
sudo chmod 666 /dev/ttyACM0

# Log out and log back in
```

### Out of Memory Errors

```bash
# Increase swap size
sudo dphys-swapfile swapoff
sudo nano /etc/dphys-swapfile
# Set CONF_SWAPSIZE=2048
sudo dphys-swapfile setup
sudo dphys-swapfile swapon

# Close other applications
# Use Raspberry Pi OS Lite (no desktop) for better performance
```

### Camera Feed is Laggy

```bash
# Enable USB 3.0 mode (if your Pi supports it)
# Check current USB mode
lsusb -t

# Make sure OAK camera is connected to blue USB 3.0 port on Pi 4

# Reduce camera resolution
python3 demo.py -e --internal_frame_height 480
```

---

## Performance Tips

### Best Settings for Raspberry Pi 4

```bash
# Recommended command for best balance of speed and accuracy
python3 demo.py -e -s --internal_fps 25 --lm_model lite
```

### Headless Mode (No Display)

For running without monitor (SSH only), modify the code to save angles to file instead of displaying:

```bash
# Comment out cv2.imshow() lines in robot_hand.py
# Write angles to file or send directly to serial
```

### Auto-start on Boot

**Quick Setup:**

```bash
cd ~/robot-hand-tracking/depthai_hand_tracker
chmod +x install_service.sh
sudo bash install_service.sh
sudo systemctl start robot_hand
```

**For detailed instructions and troubleshooting, see: [`AUTOSTART_SETUP.md`](AUTOSTART_SETUP.md)**

The installer will:
- ✅ Create systemd service with correct paths
- ✅ Enable auto-start on boot
- ✅ Show useful management commands
- ✅ Set up automatic restart on failure

---

## Additional Resources

- **DepthAI Documentation:** https://docs.luxonis.com/
- **Project README:** See `README.md` for detailed feature documentation
- **Motor Control Guide:** See `MOTOR_CONTROL_GUIDE.md` for servo details
- **Mediapipe Hand Tracking:** https://google.github.io/mediapipe/solutions/hands

---

## Quick Reference Commands

```bash
# Activate virtual environment
source .venv/bin/activate

# Basic tracking
python3 demo.py -e

# Robotic hand control
python3 robot_hand.py

# Test camera
python3 test_oak.py

# Check Arduino port
ls /dev/tty{USB,ACM}*

# Monitor system resources
htop

# Check USB devices
lsusb
```

---

## Getting Help

If you encounter issues:
1. Check the [Troubleshooting](#troubleshooting) section
2. Ensure all cables are properly connected
3. Verify camera with `python3 test_oak.py`
4. Check system resources with `htop` (install with `sudo apt install htop`)
5. Review logs with `journalctl -u handtracker.service` if using systemd service

---

**Happy Tracking! 🤖✋**
