# Robot Hand Tracking

Real-time hand tracking with OAK-D camera controlling 5-servo robotic hand via serial connection.

## Contents

- [Hardware](#hardware)
- [Power Requirements](#power-requirements)
- [Installation](#installation)
- [Arduino Firmware](#arduino-firmware)
- [Configuration](#configuration)
- [Running](#running)
- [Systemd Service](#systemd-service)
- [Troubleshooting](#troubleshooting)
- [Technical Details](#technical-details)

---

## Hardware

**Required Components:**
- Computer: Raspberry Pi 4 (4GB+ RAM) or Windows PC
- Camera: OAK-D, OAK-D Lite, or OAK-1
- Microcontroller: Arduino Uno/Nano or Teensy
- Servos: 5x hobby servos
- Storage: 32GB+ microSD (Pi only)

**Pin Assignments:**
```
Motor 1 (PIP)         → Pin 3
Motor 2 (MCP Flex)    → Pin 5
Motor 3 (MCP Abduct)  → Pin 6
Motor 4 (Base Flex)   → Pin 9
Motor 5 (Base Rotate) → Pin 10
```

---

## Power Requirements

Servos draw 5-10A combined. Do not power from Pi or Arduino.

**Power Supply 1 (5V 5A):**
- Raspberry Pi
- OAK Camera (via USB)
- Arduino (via USB, logic only)

**Power Supply 2 (5-6V 10A):**
- All servo V+ and GND pins
- Common ground to Arduino GND

**Current Draw:**
- Pi: 3A
- OAK: 1.5A
- Arduino: 0.5A
- Each servo: 1-2A peak

Insufficient servo power causes brownouts, random resets, LED flashing, jitter.

---

## Installation

**Raspberry Pi:**
```bash
sudo apt update
sudo apt install -y python3 python3-pip python3-venv build-essential

echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

cd ~
git clone https://github.com/PequenoCoder/robot-hand-tracking.git
cd robot-hand-tracking/depthai_hand_tracker

python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

sudo usermod -a -G dialout $USER
# Log out and back in for permissions

python3 test_oak.py
```

**Windows:**
```powershell
git clone https://github.com/PequenoCoder/robot-hand-tracking.git
cd robot-hand-tracking\depthai_hand_tracker

python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt

python test_oak.py
```

**Dependencies:**
- `depthai>=2.13,<3.0` - OAK camera driver
- `opencv-python>=4.5.1` - Video processing
- `scipy>=1.7.0` - Signal filtering
- `pyserial>=3.5` - Serial communication

---

## Arduino Firmware

Upload `motors1_5/motors1_5.ino` using Arduino IDE at 115200 baud.

**Find Port:**
- Windows: Device Manager → Ports (COM3, COM4, etc.)
- Linux: `ls /dev/tty{USB,ACM}*` (typically `/dev/ttyACM0`)

**Configure in `robot_hand.py` (line 15):**
```python
SERIAL_PORT = "COM3"           # Windows
SERIAL_PORT = "/dev/ttyACM0"   # Linux
SERIAL_PORT = None             # Auto-detect
```

**Serial Protocol:**
- Format: `M1,M2,M3,M4,M5\n`
- Example: `-45.5,-60.2,-30.0,-50.1,-20.5\n`
- Range: -80.0 to 0.0 degrees
- Rate: 20 Hz
- Baud: 115200

---

## Configuration

Edit `robot_hand.py` for custom settings:

**Serial (lines 15-19):**
```python
SERIAL_PORT = None              # Port or auto-detect
BAUD_RATE = 115200              # Must match Arduino
UPDATE_RATE_HZ = 20             # Command frequency
ENABLE_MOTORS = True            # False for testing
DEBUG_SERIAL = False            # Show TX/RX
```

**Angle Processing (lines 22-28):**
```python
USE_ROBUST_MCP = True           # Better orientation handling
DEBUG_ANGLES = False            # Console output
MCP_ZERO_OFFSET_DEG = 15.0      # Calibrate zero
MCP_DEADBAND_DEG = 8.0          # Ignore small movements
PIP_ZERO_OFFSET_DEG = 20.0
PIP_DEADBAND_DEG = 10.0
PIP_FLEX_MAX_DEG = 60.0
```

**Filtering (lines 31-34):**
```python
FILTER_CUTOFF_HZ = 10.0         # Response speed
FILTER_ORDER = 2                # Complexity
FILTER_BUFFER_SIZE = 3          # Sample buffer
MAX_MOTOR_CHANGE_PER_UPDATE = 80.0  # Rate limit
```

**Position Hold (lines 37-38):**
```python
HOLD_LAST_POSITION = True       # Maintain when hand lost
RESET_TIMEOUT_SECONDS = 5.0     # Reset after N sec (0=never)
```

**Motor Limits (lines 40-41):**
```python
MOTOR_MIN = -80.0               # Must match Arduino
MOTOR_MAX = 0.0
```

**Hand Tracker (lines 463-472):**
```python
tracker = HandTracker(
    input_src=None,             # Use OAK camera
    use_lm=True,                # Landmark model
    use_world_landmarks=True,   # 3D coords
    use_gesture=False,          # Disable gestures
    xyz=False,                  # Disable spatial location
    solo=True,                  # Single hand only
    resolution='full',          # 'full' or 'ultra'
    stats=True,                 # Show FPS stats
    trace=0                     # Debug level
)
```

---

## Running

**Manual:**
```bash
# Raspberry Pi
cd ~/robot-hand-tracking/depthai_hand_tracker
source .venv/bin/activate
python3 robot_hand.py

# Windows
cd robot-hand-tracking\depthai_hand_tracker
.\.venv\Scripts\Activate.ps1
python robot_hand.py
```

Startup sequence: searches for Arduino → connects → initializes camera → starts tracking.

Exit: press 'q', ESC, or Ctrl+C.

---

## Systemd Service

Raspberry Pi auto-start on boot. Requires desktop environment.

**Install:**
```bash
cd ~/robot-hand-tracking/depthai_hand_tracker
chmod +x install_service.sh
sudo bash install_service.sh
sudo systemctl start robot_hand
```

Service behavior:
- Starts on boot
- Opens terminal window
- Waits for Arduino
- Auto-restarts on failure (10s delay)

**Manual installation if script fails:**
```bash
sudo cp robot_hand.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable robot_hand
sudo systemctl start robot_hand
```

**Control:**
```bash
sudo systemctl start|stop|restart|status robot_hand
sudo systemctl enable|disable robot_hand
```

**Logs:**
```bash
sudo journalctl -u robot_hand -f           # Live
sudo journalctl -u robot_hand -n 50        # Last 50
sudo journalctl -u robot_hand --since today
```

**Remove:**
```bash
sudo systemctl stop robot_hand
sudo systemctl disable robot_hand
sudo rm /etc/systemd/system/robot_hand.service
sudo systemctl daemon-reload
```

---


## Troubleshooting

**Arduino not found:**
```bash
# Check port exists
ls /dev/tty{USB,ACM}*               # Linux
# Device Manager → Ports             # Windows

# Fix permissions
sudo usermod -a -G dialout $USER    # Linux, then logout/login
sudo chmod 666 /dev/ttyACM0         # Temporary fix

# Set manual port in robot_hand.py
SERIAL_PORT = "/dev/ttyACM0"        # or "COM3"
```

**Camera not detected:**
```bash
lsusb | grep -i luxonis             # Should show Movidius MyriadX
python3 test_oak.py                 # Test connection

# Reinstall udev rules
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo reboot
```

**Power problems (brownouts, resets, LED flashing):**
```bash
vcgencmd measure_volts              # Should be ~5.1V
vcgencmd get_throttled              # 0x0 = good

# Solutions:
# - Separate 10A servo power supply
# - Common ground Arduino/servo supply
# - Never power servos from USB
# - 1000µF capacitor on Arduino 5V/GND
```

**Low FPS on Pi:**
```python
# Use Edge mode (line 437 in robot_hand.py)
from HandTrackerEdge import HandTracker

# Lower resolution
resolution='full'  # Instead of 'ultra'

# Add to tracker init
internal_fps=20
```

**Import errors:**
```bash
source .venv/bin/activate
pip install --upgrade -r requirements.txt
```

**Service won't start:**
```bash
sudo journalctl -u robot_hand -n 100
sudo systemctl cat robot_hand
.venv/bin/python3 robot_hand.py     # Test manually
sudo bash install_service.sh        # Reinstall
```

---

## Technical Details

**Project Structure:**
```
robot-hand-tracking/
├── README.md
├── .gitignore
└── depthai_hand_tracker/
    ├── robot_hand.py               # Main program
    ├── HandTracker.py              # Host-side tracking
    ├── HandTrackerEdge.py          # Edge-mode tracking
    ├── HandTrackerRenderer.py      # Visualization
    ├── mediapipe_utils.py          # Utilities
    ├── hand_pose_fixes.py          # Pose calculations
    ├── FPS.py                      # FPS counter
    ├── test_oak.py                 # Camera test
    ├── requirements.txt
    ├── LICENSE.txt
    ├── robot_hand.service          # Systemd unit
    ├── install_service.sh          # Service installer
    ├── models/                     # Neural network blobs
    └── motors1_5/
        └── motors1_5.ino           # Arduino firmware
```

**Hand Tracking Pipeline:**
1. OAK camera captures RGB frame
2. Palm detection model locates hand
3. Landmark model extracts 21 3D keypoints
4. `hand_pose_fixes.py` calculates joint angles
5. Butterworth filter smooths angles
6. Serial transmission to Arduino (20Hz)
7. Arduino maps angles to servo PWM

**Angle Calculation:**
- PIP (Proximal Interphalangeal): Distance between knuckle landmarks
- MCP (Metacarpophalangeal): Vector orientation with robust mode
- Deadband filtering removes jitter
- Zero offset calibration per joint
- Range: -80° to 0° (Arduino enforced)

**Performance:**
- Host mode: 15-25 FPS (Pi 4), 30-60 FPS (desktop)
- Edge mode: 20-30 FPS (Pi 4), faster inference
- Latency: ~50-100ms end-to-end

---

## License

MIT License - see [LICENSE.txt](depthai_hand_tracker/LICENSE.txt)
