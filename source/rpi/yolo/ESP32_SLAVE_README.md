# ESP32 Slave Communication System

## Overview

`esp32_comm.py` is a **slave system** that runs on Raspberry Pi and receives commands from the ESP32 CoinExchanger.ino master controller. It handles:

1. **IR Sensor Detection** - Detects when a chit is inserted
2. **YOLO Chit Recognition** - Identifies chit denomination (5, 10, 20, 50 pesos)
3. **LCD Display** - Shows status messages on 20x4 I2C LCD
4. **Servo Control** - Dispenses chit after coin exchange

## Hardware Configuration

### GPIO Pins (BCM Numbering)
- **IR Sensor**: GPIO17
- **Servo Motor**: GPIO22
  - Initial Angle: 39°
  - Release Angle: 90°

### I2C LCD
- **Address**: 0x27
- **Size**: 20x4 characters

### Serial Connection
- **Port**: /dev/serial0
- **Baud Rate**: 115200
- **Connected to**: ESP32 TX/RX pins

## Command Protocol

### Commands Received from ESP32

| Command | Description | Response |
|---------|-------------|----------|
| `CHECK_IR` | Check if IR sensor detects chit | `IR_DETECTED` or `IR_CLEAR` |
| `DETECT_CHIT` | Run YOLO detection | `CHIT_5`, `CHIT_10`, `CHIT_20`, `CHIT_50`, or `CHIT_UNKNOWN` |
| `DISPLAY:<message>` | Display message on LCD (lines separated by \|) | `DISPLAY_OK` |
| `DISPENSE_CHIT` | Release chit via servo | `DISPENSE_COMPLETE` |
| `PING` | Check if slave is alive | `PONG` |
| `RESET` | Reset to initial state | `RESET_OK` |

### Responses Sent to ESP32

| Response | Description |
|----------|-------------|
| `SLAVE_READY` | System initialized and ready |
| `IR_DETECTED` | Chit detected by IR sensor |
| `IR_CLEAR` | No chit detected |
| `CHIT_5` | Detected 5 peso chit |
| `CHIT_10` | Detected 10 peso chit |
| `CHIT_20` | Detected 20 peso chit |
| `CHIT_50` | Detected 50 peso chit |
| `CHIT_UNKNOWN` | Could not identify chit |
| `DISPENSE_COMPLETE` | Chit dispensed successfully |
| `DISPLAY_OK` | LCD updated |
| `ERROR:<message>` | Error occurred |
| `SLAVE_SHUTDOWN` | System shutting down |

## Installation

### Prerequisites

```bash
# Update system
sudo apt-get update
sudo apt-get upgrade

# Install pigpio daemon (for servo control)
sudo apt-get install pigpio python3-pigpio
sudo systemctl enable pigpiod
sudo systemctl start pigpiod

# Install Python dependencies
pip3 install pyserial
pip3 install RPi.GPIO
pip3 install smbus2
pip3 install opencv-python
pip3 install ultralytics

# Enable serial port (disable console)
sudo raspi-config
# Navigate to: Interface Options -> Serial Port
# Disable login shell over serial: NO
# Enable serial port hardware: YES
# Reboot
```

### Model Setup

1. Place your trained YOLO model at:
   ```
   /home/pi/Desktop/PROJECTS/Chits-Exchanger/source/rpi/yolo/chit_model.pt
   ```

2. Or update `MODEL_PATH` in `esp32_comm.py` to your model location

## Usage

### Start the Slave System

```bash
cd /home/pi/Desktop/PROJECTS/Chits-Exchanger/source/rpi/yolo
python3 esp32_comm.py
```

### Expected Output

```
==================================================
ESP32 Chit Slave Controller Initializing...
==================================================
[LCD] Initializing...
[LCD] ✓ Initialized
[IR] Initializing...
[IR] ✓ Initialized
[SERVO] Initializing...
[SERVO] ✓ Initialized
[YOLO] Initializing...
[YOLO] Loading model: /home/pi/.../chit_model.pt
[YOLO] Model loaded successfully. Classes: {0: '5', 1: '10', 2: '20', 3: '50'}
[YOLO] ✓ Model loaded
[CAMERA] Opened camera index 0
[CAMERA] ✓ Opened
[SERIAL] Initializing...
[SERIAL] ✓ Connected on /dev/serial0 @ 115200 baud

==================================================
✓ ALL SYSTEMS INITIALIZED
==================================================

[TX → ESP32] SLAVE_READY
[SYSTEM] Slave controller running. Listening for ESP32 commands...
[SYSTEM] Press Ctrl+C to shutdown
```

### Run as Systemd Service (Auto-start on boot)

Create service file:

```bash
sudo nano /etc/systemd/system/chit-slave.service
```

Add content:

```ini
[Unit]
Description=Chit Exchanger Slave System
After=network.target pigpiod.service

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/Desktop/PROJECTS/Chits-Exchanger/source/rpi/yolo
ExecStart=/usr/bin/python3 /home/pi/Desktop/PROJECTS/Chits-Exchanger/source/rpi/yolo/esp32_comm.py
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

Enable and start:

```bash
sudo systemctl daemon-reload
sudo systemctl enable chit-slave.service
sudo systemctl start chit-slave.service

# Check status
sudo systemctl status chit-slave.service

# View logs
journalctl -u chit-slave.service -f
```

## Workflow Example

### Complete Transaction Flow

1. **ESP32 Sends**: `CHECK_IR`
   - **RPi Response**: `IR_DETECTED`

2. **ESP32 Sends**: `DETECT_CHIT`
   - **RPi**: Captures frames, runs YOLO
   - **RPi Response**: `CHIT_50` (or other denomination)

3. **ESP32 Sends**: `DISPLAY:Chit Detected!|Value: P50|Calculating...|`
   - **RPi**: Updates LCD
   - **RPi Response**: `DISPLAY_OK`

4. **ESP32**: Calculates coin combination, dispenses coins

5. **ESP32 Sends**: `DISPLAY:Dispensing|Complete!|Releasing chit...|`
   - **RPi Response**: `DISPLAY_OK`

6. **ESP32 Sends**: `DISPENSE_CHIT`
   - **RPi**: Servo rotates to release chit
   - **RPi Response**: `DISPENSE_COMPLETE`

7. **ESP32 Sends**: `RESET`
   - **RPi**: Resets to ready state
   - **RPi Response**: `RESET_OK`

## Configuration

Edit constants in `esp32_comm.py`:

```python
# GPIO Pins
IR_SENSOR_PIN = 17
SERVO_PIN = 22

# Servo Angles
SERVO_INITIAL_ANGLE = 39
SERVO_RELEASE_ANGLE = 90

# LCD
LCD_I2C_ADDR = 0x27

# Serial
SERIAL_PORT = '/dev/serial0'
SERIAL_BAUD = 115200

# YOLO
MODEL_PATH = '/path/to/your/chit_model.pt'
CAMERA_INDEX = 0
DETECTION_TIMEOUT = 5.0
CONFIDENCE_THRESHOLD = 0.6
```

## Troubleshooting

### Serial Communication Issues

```bash
# Check if serial port is available
ls -l /dev/serial0

# Test serial connection
sudo apt-get install minicom
minicom -b 115200 -o -D /dev/serial0
```

### Camera Issues

```bash
# List available cameras
v4l2-ctl --list-devices

# Test camera
python3 -c "import cv2; cap = cv2.VideoCapture(0); print('Camera OK' if cap.isOpened() else 'Camera Failed'); cap.release()"
```

### LCD Not Working

```bash
# Check I2C devices
sudo i2cdetect -y 1

# Should show 0x27 (or your LCD address)
```

### Pigpio Daemon Issues

```bash
# Check if pigpiod is running
sudo systemctl status pigpiod

# Restart if needed
sudo systemctl restart pigpiod
```

## Testing Individual Components

### Test IR Sensor
```bash
python3 /home/pi/Desktop/PROJECTS/Chits-Exchanger/source/rpi/test/ir_sensor_tester.py
```

### Test Servo
```bash
python3 /home/pi/Desktop/PROJECTS/Chits-Exchanger/source/rpi/test/servo_tester.py
```

### Test LCD
```bash
python3 /home/pi/Desktop/PROJECTS/Chits-Exchanger/source/rpi/test/lcd_tester.py
```

## ESP32 Master Integration

The ESP32 CoinExchanger.ino should send commands via Serial. Example Arduino code:

```cpp
void sendToRPi(String command) {
  Serial.println(command);
  Serial.flush();
}

String readFromRPi() {
  if (Serial.available()) {
    String response = Serial.readStringUntil('\n');
    response.trim();
    return response;
  }
  return "";
}

// Example usage
void detectChit() {
  // Check IR first
  sendToRPi("CHECK_IR");
  delay(100);
  String response = readFromRPi();
  
  if (response == "IR_DETECTED") {
    // Run YOLO detection
    sendToRPi("DETECT_CHIT");
    
    // Wait for detection result (may take 5 seconds)
    unsigned long startTime = millis();
    while (millis() - startTime < 10000) {
      response = readFromRPi();
      if (response.startsWith("CHIT_")) {
        // Parse chit value
        int chitValue = response.substring(5).toInt();
        handleChitDetected(chitValue);
        break;
      }
      delay(100);
    }
  }
}
```

## Architecture

```
┌─────────────────────────────────────────┐
│         ESP32 CoinExchanger.ino         │
│              (MASTER)                   │
│                                         │
│  - Receives coins                       │
│  - Manages coin hoppers                 │
│  - Calculates exchange                  │
│  - Controls transaction flow            │
└─────────────────┬───────────────────────┘
                  │ Serial (115200 baud)
                  │ TX/RX Connection
┌─────────────────▼───────────────────────┐
│         Raspberry Pi 4                  │
│         esp32_comm.py (SLAVE)           │
│                                         │
│  ┌──────────────────────────────────┐  │
│  │ IR Sensor (GPIO17)               │  │
│  │ - Detects chit presence          │  │
│  └──────────────────────────────────┘  │
│                                         │
│  ┌──────────────────────────────────┐  │
│  │ YOLO Detector (Camera)           │  │
│  │ - Identifies chit denomination   │  │
│  └──────────────────────────────────┘  │
│                                         │
│  ┌──────────────────────────────────┐  │
│  │ LCD Display (I2C 0x27)           │  │
│  │ - Shows transaction status       │  │
│  └──────────────────────────────────┘  │
│                                         │
│  ┌──────────────────────────────────┐  │
│  │ Servo Motor (GPIO22)             │  │
│  │ - Dispenses chit after exchange  │  │
│  └──────────────────────────────────┘  │
└─────────────────────────────────────────┘
```

## License

Part of Chits Exchanger System - November 2025
