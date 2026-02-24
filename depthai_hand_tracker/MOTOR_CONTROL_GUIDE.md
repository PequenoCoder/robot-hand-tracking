# Hand Tracker to Motor Control Guide

This guide explains how to connect the OAK camera hand tracker to your 5-motor robotic hand controller.

## Hardware Setup

1. **Connect the Arduino/Teensy** to your computer via USB
2. **Connect the OAK camera** to your computer via USB
3. Make sure your 5 motors are properly connected to the controller

## Software Setup

### 1. Install Python Dependencies

```bash
pip install -r requirements.txt
```

This will install:
- opencv-python (for video processing)
- depthai (for OAK camera)
- scipy (for hand tracking)
- pyserial (for Arduino communication)

### 2. Upload Arduino Firmware

1. Open `motors1_5/motors1_5.ino` in Arduino IDE
2. Select your board (Arduino/Teensy)
3. Upload the sketch

## Running the System

### Method 1: Automated (Recommended)

Run the bridge script:

```bash
python hand_to_motors.py
```

This will:
- Auto-detect the Arduino port
- Initialize the hand tracker
- Start streaming hand data to motors
- Show live visualization

### Method 2: Manual Control (Testing)

You can test motors manually via Serial Monitor (115200 baud):

```
# Single motor values (rotating through motors):
0
-5
-10

# Or all 5 motors at once (CSV format):
0,-5,-10,-15,-20
```

## How It Works

### Finger Mapping

The 5 motors correspond to the 5 fingers:

1. **Motor 1** → Thumb
2. **Motor 2** → Index finger
3. **Motor 3** → Middle finger
4. **Motor 4** → Ring finger
5. **Motor 5** → Pinky

### Angle Calculation

- **Straight finger** → Motor angle = 0°
- **Fully bent finger** → Motor angle = -20°
- Intermediate positions are calculated based on finger landmark positions

### Safety Limits

The Arduino firmware enforces strict safety limits:
- ✅ Valid range: **-20° to 0°**
- ❌ Motors **cannot** go positive (beyond 0°)
- ❌ Motors **cannot** go below -20°

All commands are automatically clamped to this safe range.

## Configuration

You can adjust settings in `hand_to_motors.py`:

```python
# Serial port (auto-detect if None)
SERIAL_PORT = None  # or "COM3", "/dev/ttyACM0", etc.

# Communication speed
BAUD_RATE = 115200

# Update rate (Hz)
UPDATE_RATE_HZ = 10  # Adjust based on motor response time

# Safety limits (must match Arduino)
ANGLE_MIN = -20.0
ANGLE_MAX = 0.0
```

## Troubleshooting

### Arduino Not Found

```
ERROR: Could not find Arduino!
```

**Solution:** Set `SERIAL_PORT` manually in the script:
```python
SERIAL_PORT = "COM3"  # Windows
# or
SERIAL_PORT = "/dev/ttyACM0"  # Linux/Mac
```

### OAK Camera Not Found

```
ERROR initializing tracker
```

**Solution:** 
1. Check USB connection
2. Test with: `python test_oak.py`
3. Try unplugging and reconnecting the camera

### Motors Not Moving

1. Check Serial Monitor output (115200 baud)
2. Verify motor power supply
3. Try manual commands: `0,-5,-10,-15,-20`

### Hand Not Detected

1. Ensure good lighting
2. Keep hand within camera view (~0.5-2m distance)
3. Try different hand positions
4. Check video window for visualization

## Controls

While running `hand_to_motors.py`:

- **ESC** or **Q** → Quit (motors return to safe position)
- **Video window** → Shows hand tracking visualization
- **Console** → Shows current motor angles

## Advanced Usage

### Change Tracking Mode

For tracking 2 hands (Duo mode):

```python
tracker = HandTracker(
    solo=False,  # Enable duo mode
    # ... other settings
)
```

### Adjust Sensitivity

Modify the `calculate_finger_bend_simple()` function to change how finger positions map to motor angles.

### Use Different Landmark Model

```python
lm_model="full",  # More accurate but slower
# or
lm_model="lite",  # Faster but less accurate (default)
```

## Safety Notes

⚠️ **Important:**
- Always have motors positioned safely before starting
- The system automatically resets motors to 0° on exit
- If motors behave unexpectedly, press ESC to stop
- Never bypass the safety limits in the Arduino code
- Test without load first

## Example Session

```bash
$ python hand_to_motors.py
============================================================
Hand Tracker to Motor Controller Bridge
============================================================

Searching for Arduino...
Using port: COM3
Connecting to Arduino...
✓ Connected to Arduino
Arduino: Commands:
Arduino: SAFETY LIMITS: All angles clamped to [-20° to 0°]

Initializing Hand Tracker...
✓ Hand Tracker initialized

Starting main loop...
Press 'q' or ESC to quit
------------------------------------------------------------
Motors: T:  0.0° I: -5.2° M: -8.1° R: -3.5° P: -2.1°
```

## Files

- `hand_to_motors.py` - Main bridge script
- `motors1_5/motors1_5.ino` - Arduino firmware
- `HandTracker.py` - Hand tracking implementation
- `test_oak.py` - Camera connection test

## Further Customization

The script uses a simple distance-based algorithm for finger bend detection. You can improve accuracy by:

1. Calibrating for your specific hand size
2. Using angle-based calculations (see `calculate_finger_bend()` function)
3. Adding smoothing/filtering for steadier motor control
4. Implementing gesture-based control instead of continuous tracking
