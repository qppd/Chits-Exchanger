# Chit to Coin Exchanger System - Implementation Summary

## System Overview

This system exchanges detected chits (5, 10, 20, or 50 peso denominations) for coins using a fully automated workflow with Raspberry Pi, ESP32, and YOLO object detection.

## Complete Workflow

### 1. **Chit Detection** (Raspberry Pi)
- **IR Sensor (GPIO 17)** detects when a chit is inserted
- **ESP32-CAM** captures live video stream at http://<camera_ip>/stream
- **YOLO Detection** (`yolo_detect.py`) identifies chit denomination (5, 10, 20, 50)
- **Servo (GPIO 22)** releases the chit after detection
  - Initial position: 39°
  - Release position: 90°
  - Returns to initial after 2 seconds

### 2. **Serial Communication** (RPi ↔ ESP32)
Commands sent from RPi to ESP32:
```
IR_DETECTED              - IR sensor triggered
CHIT_DETECTED:<value>    - Chit denomination detected (e.g., "CHIT_DETECTED:50")
CHIT_RELEASED:<value>    - Chit physically released by servo
DETECTION_TIMEOUT        - No valid chit detected within timeout
```

### 3. **Denomination Selection** (ESP32 + LCD + Button)
- **LCD (I2C 0x27, 20x4)** displays:
  ```
  Chit Detected!
  Value: P50
  ```
- After 2 seconds, shows selection UI:
  ```
  Select Denom:
  > P5 coins
  Amount: P50
  Press to confirm
  ```
- **Button (GPIO 27)** cycles through denominations:
  - Press 1: Select P5 coins
  - Press 2: Select P10 coins  
  - Press 3: Select P20 coins
  - Press 4: Confirm selection

### 4. **Coin Calculation** (ESP32)
Uses algorithm from `ChitExchanger.ino`:
- Calculates optimal coin combination based on selected denomination
- Displays plan on LCD:
  ```
  Plan:
  5P:4 10P:1 20P:1
  Total: P50
  ```
- Auto-computes remainder if amount can't be fully dispensed

### 5. **Coin Dispensing** (ESP32 + 3 Coin Hoppers)

**Hardware Setup:**
- **Hopper 1:** 5 PHP coins (GPIO19 pulse, GPIO26 SSR)
- **Hopper 2:** 10 PHP coins (GPIO18 pulse, GPIO25 SSR)
- **Hopper 3:** 20 PHP coins (GPIO4 pulse, GPIO33 SSR)

**Dispensing Process:**
- SSR (Solid State Relay) powers each hopper
- ALLAN coin hopper dispenses coins
- Pulse detection counts each coin (1 pulse = 1 coin)
- LCD shows real-time progress:
  ```
  Dispensing...
  Dispensing 20 PHP
  Count: 3/5
  ```

### 6. **Completion** (ESP32)
- LCD displays:
  ```
  Complete!
  Dispensed: P50
  Thank you!
  ```
- If remainder exists:
  ```
  Complete!
  Dispensed: P48
  Remainder: P2
  (Keep chit)
  ```
- System resets to idle after 5 seconds

## File Modifications

### 1. `source/rpi/yolo/yolo_detect.py`
**Added:**
- GPIO setup for IR sensor (GPIO 17) and servo (GPIO 22)
- Serial communication with ESP32 at 115200 baud
- IR sensor detection with debouncing
- Servo control functions:
  - `angle_to_pulse()` - Convert angle to PWM
  - `set_servo_angle()` - Move servo to angle
  - `release_chit()` - Release mechanism
- Chit detection state machine:
  - Activates on IR sensor trigger
  - Detects chit value using YOLO (5, 10, 20, 50)
  - Sends `CHIT_DETECTED:<value>` to ESP32
  - Releases chit via servo
  - Sends `CHIT_RELEASED:<value>` confirmation
- Enhanced display showing IR status and detection state

**Requirements:**
```bash
pip install pyserial RPi.GPIO pigpio
sudo pigpiod  # Must run pigpio daemon
```

**Usage:**
```bash
python yolo_detect.py --model yolo11n_ncnn_model/model.ncnn.param \
                      --esp32_port /dev/ttyUSB0 \
                      --camera_ip 192.168.1.21
```

### 2. `source/esp32cam/IPCamera/IPCamera.ino`
**Status:** No changes needed
- Already provides HTTP streaming at `/stream`
- Flash control available at `/flash/on`, `/flash/off`, `/flash/toggle`
- RPi accesses stream directly

### 3. `source/esp32/CoinExchanger/CoinExchanger.ino`
**Complete Rewrite:**

**Added State Machine:**
- `STATE_IDLE` - Waiting for chit
- `STATE_CHIT_DETECTED` - Show detected value
- `STATE_DENOMINATION_SELECTION` - User selects coin denomination
- `STATE_CALCULATING` - Calculate coin combination
- `STATE_DISPENSING` - Dispense coins with pulse counting
- `STATE_COMPLETE` - Show completion message
- `STATE_ERROR` - Handle errors

**Added Components:**
- LCD display support (I2C 20x4)
- Button interrupt handling (GPIO 27)
- Serial command parsing from RPi
- Denomination selection UI
- Coin calculation algorithm (from ChitExchanger)
- Real-time dispensing progress
- Pulse counting per hopper

**Key Functions:**
- `handleChitDetected()` - Process chit detection
- `handleDenominationSelection()` - UI for coin selection
- `handleCalculation()` - Calculate coin combination
- `handleDispensing()` - Dispense with progress monitoring
- `handleComplete()` - Show results and reset
- `calculateCoinCombination()` - Optimal coin algorithm

**Test Commands:**
```
test_chit 50           - Simulate 50 peso chit detection
test_pulse 1/2/3       - Test hopper pulse detection
test_relay 1/2/3 on/off - Test SSR control
test_all               - Test all components
help                   - Show command list
```

## Hardware Connections

### Raspberry Pi
```
GPIO 17 → IR Sensor (Input)
GPIO 22 → Servo Signal (PWM)
GPIO 2  → I2C SDA (LCD)
GPIO 3  → I2C SCL (LCD)
USB     → ESP32 (Serial)
Network → ESP32-CAM (HTTP Stream)
```

### ESP32 (CoinExchanger)
```
GPIO 21    → I2C SDA (LCD)
GPIO 22    → I2C SCL (LCD)
GPIO 27    → Button Input (Pull-up)

Hopper 1 (5 PHP):
  GPIO 19  → Pulse Input
  GPIO 26  → SSR Control

Hopper 2 (10 PHP):
  GPIO 18  → Pulse Input
  GPIO 25  → SSR Control

Hopper 3 (20 PHP):
  GPIO 4   → Pulse Input
  GPIO 33  → SSR Control

TX/RX      → RPi (Serial 115200 baud)
```

### ESP32-CAM (IPCamera)
```
Standard ESP32-CAM pinout
Network/WiFi for streaming
```

## Testing Procedure

### 1. Test Individual Components

**Test RPi Servo:**
```bash
cd source/rpi/test
python3 servo_tester.py
```

**Test RPi IR Sensor:**
```bash
python3 ir_sensor_tester.py
```

**Test RPi LCD:**
```bash
python3 lcd_tester.py
```

**Test ESP32 Coin Hoppers:**
Upload `CoinExchanger.ino`, then:
```
test_pulse 1      # Test hopper 1
test_relay 1 on   # Power hopper 1
test_relay 1 off  # Stop hopper 1
test_all          # Test all 3 hoppers
```

### 2. Test ESP32-CAM Stream
- Upload `IPCamera.ino`
- Connect to WiFi or use AP mode
- Access `http://<camera_ip>/stream`
- Test flash: `http://<camera_ip>/flash/on`

### 3. Test Communication
```bash
# On RPi, start yolo detection
python yolo_detect.py --model yolo11n_ncnn_model/model.ncnn.param

# On ESP32 serial monitor, simulate chit detection
test_chit 50
```

### 4. Integration Test
1. Start YOLO detection on RPi
2. Insert chit into IR sensor area
3. Verify detection and servo release
4. Press button to select denomination
5. Verify coin dispensing
6. Check LCD displays throughout

## Serial Communication Protocol

### RPi → ESP32
```
CHIT_DETECTED:5        # 5 peso chit detected
CHIT_DETECTED:10       # 10 peso chit detected
CHIT_DETECTED:20       # 20 peso chit detected
CHIT_DETECTED:50       # 50 peso chit detected
IR_DETECTED            # IR sensor triggered
CHIT_RELEASED:50       # Chit released by servo
DETECTION_TIMEOUT      # Detection failed
SYSTEM_SHUTDOWN        # RPi shutting down
```

### ESP32 → RPi
```
DISPENSING_COMPLETE:50 # Dispensed 50 pesos worth of coins
```

## Troubleshooting

### Issue: YOLO not detecting chits
- Check camera stream is accessible
- Verify YOLO model path is correct
- Ensure labels match denominations (5, 10, 20, 50)
- Adjust confidence threshold

### Issue: Servo not moving
- Run `sudo pigpiod` before starting script
- Check GPIO 22 connection
- Test with `servo_tester.py`

### Issue: Coins not dispensing
- Check SSR connections and power
- Verify pulse detection with `test_pulse`
- Ensure hoppers are loaded with coins
- Check hopper power supply (typically 12V)

### Issue: LCD not displaying
- Verify I2C address (0x27) with `i2cdetect -y 1`
- Check SDA/SCL connections
- Test with `lcd_tester.py`

### Issue: Button not responding
- Check GPIO 27 connection
- Verify pull-up resistor or internal pull-up enabled
- Test with `button_tester.py`

### Issue: Serial communication failed
- Check USB connection RPi ↔ ESP32
- Verify baud rate (115200)
- Check port: `ls /dev/ttyUSB*`
- Test with: `screen /dev/ttyUSB0 115200`

## Configuration

### Change Camera IP
Edit `yolo_detect.py`:
```python
parser.add_argument('--camera_ip', default='YOUR_IP_HERE')
```

### Change Serial Port
```bash
python yolo_detect.py --esp32_port /dev/ttyUSB0
```

### Adjust Servo Angles
Edit `yolo_detect.py`:
```python
SERVO_INITIAL_ANGLE = 39  # Starting position
SERVO_RELEASE_ANGLE = 90  # Release position
```

### Change Coin Values
Edit `source/esp32/CoinExchanger/PIN_CONFIGURATION.h`:
```cpp
#define COIN_HOPPER_1_VALUE 5   // Change hopper 1 value
#define COIN_HOPPER_2_VALUE 10  // Change hopper 2 value
#define COIN_HOPPER_3_VALUE 20  // Change hopper 3 value
```

## System Architecture

```
┌─────────────────┐
│  User Inserts   │
│     Chit        │
└────────┬────────┘
         │
         ▼
┌─────────────────┐      ┌──────────────┐
│   IR Sensor     │──────▶│ Raspberry Pi │
│   (GPIO 17)     │      │   + YOLO     │
└─────────────────┘      └──────┬───────┘
                                │
         ┌──────────────────────┼──────────────────────┐
         │                      │                      │
         ▼                      ▼                      ▼
┌────────────────┐    ┌─────────────────┐   ┌─────────────────┐
│  ESP32-CAM     │    │     Servo       │   │     ESP32       │
│  Video Stream  │    │  (GPIO 22)      │   │  CoinExchanger  │
└────────────────┘    │  Release Chit   │   └────────┬────────┘
                      └─────────────────┘            │
                                                     │
                      ┌──────────────────────────────┤
                      │                              │
                      ▼                              ▼
            ┌──────────────────┐          ┌──────────────────┐
            │  LCD Display     │          │  Button Input    │
            │  Denomination    │          │  User Selection  │
            │  Selection UI    │          │  (GPIO 27)       │
            └──────────────────┘          └──────────────────┘
                      │
                      ▼
            ┌──────────────────┐
            │  3 Coin Hoppers  │
            │  5, 10, 20 PHP   │
            │  Pulse Counting  │
            └──────────────────┘
                      │
                      ▼
            ┌──────────────────┐
            │  Coins Dispensed │
            │  Transaction     │
            │  Complete        │
            └──────────────────┘
```

## Success Criteria

✅ IR sensor triggers YOLO detection
✅ YOLO correctly identifies chit denominations (5, 10, 20, 50)
✅ Servo releases chit after detection
✅ Serial commands sent from RPi to ESP32
✅ LCD displays denomination selection UI
✅ Button cycles through denominations and confirms
✅ Coin calculation follows ChitExchanger algorithm
✅ Coins dispensed from correct hoppers
✅ Pulse counting tracks dispensed coins accurately
✅ LCD shows real-time progress
✅ System handles remainder correctly
✅ System resets to idle after completion

## Next Steps

1. Flash `CoinExchanger.ino` to ESP32
2. Flash `IPCamera.ino` to ESP32-CAM
3. Install dependencies on RPi: `pip install -r requirements.txt`
4. Start pigpio daemon: `sudo pigpiod`
5. Configure WiFi on ESP32-CAM
6. Test individual components
7. Run integration test
8. Fine-tune YOLO confidence thresholds
9. Calibrate servo angles if needed
10. Load coins into hoppers and test full workflow

---

**Date:** October 24, 2025
**System:** Chits to Coin Exchanger
**Status:** Implementation Complete - Ready for Testing
