# ESP32 CoinExchanger - RPi Slave Integration Summary

## ✅ Integration Complete!

The ESP32 CoinExchanger.ino has been successfully integrated with the RPi slave system (esp32_comm.py).

## What Was Added

### 1. **New Command Constants**
```cpp
// Commands sent TO RPi slave
CMD_CHECK_IR          // Check if chit is present
CMD_DETECT_CHIT       // Run YOLO detection
CMD_DISPLAY           // Update LCD display
CMD_DISPENSE_CHIT     // Release chit via servo
CMD_PING              // Test communication
CMD_RESET             // Reset slave to ready state

// Responses received FROM RPi
RESP_SLAVE_READY      // Slave initialized
RESP_IR_DETECTED      // Chit present
RESP_IR_CLEAR         // No chit
RESP_CHIT_5/10/20/50  // Detected denomination
RESP_CHIT_UNKNOWN     // Could not identify
RESP_DISPENSE_COMPLETE // Chit released
RESP_DISPLAY_OK       // LCD updated
```

### 2. **RPi Communication Functions**

#### `waitForRPiSlave()`
- Waits up to 30 seconds for RPi to send `SLAVE_READY`
- Called during ESP32 setup()
- Shows status on LCD

#### `sendToRPi(String command)`
- Sends command to RPi via Serial
- Adds newline and flushes buffer

#### `waitForRPiResponse(unsigned long timeout)`
- Waits for response from RPi
- Returns response string or "TIMEOUT"

#### `displayOnRPiLCD(line1, line2, line3, line4)`
- Updates RPi LCD display (20x4)
- Lines separated by `|` character
- Waits for `DISPLAY_OK` confirmation

#### `detectChitWithRPi()`
- **Complete 3-step detection process:**
  1. Check IR sensor (`CHECK_IR`)
  2. Display "Identifying..." message
  3. Run YOLO detection (`DETECT_CHIT`)
- Returns: chit value (5/10/20/50), 0 (no chit), -1 (error)
- Timeout: 10 seconds for YOLO

#### `releaseChitWithRPi()`
- Commands RPi servo to release chit
- Sends `DISPENSE_CHIT` command
- Waits for `DISPENSE_COMPLETE` (3 sec timeout)

#### `resetRPiSlave()`
- Resets RPi to ready state
- Called after transaction complete

### 3. **Modified Workflow**

#### **Setup Phase**
```cpp
void setup() {
  // Initialize hardware...
  // Initialize LCD...
  // Initialize coin hoppers...
  
  waitForRPiSlave();  // NEW: Wait for RPi to connect
}
```

#### **STATE_IDLE - Automatic Chit Detection**
```cpp
case STATE_IDLE:
  // Check for chit every 2 seconds
  if (rpiSlaveReady && (millis() - lastIRCheckTime > IR_CHECK_INTERVAL)) {
    int chitValue = detectChitWithRPi();
    
    if (chitValue > 0) {
      detectedChitValue = chitValue;
      currentState = STATE_CALCULATING;  // Skip manual selection, auto-dispense
    }
  }
  break;
```

#### **STATE_CALCULATING - Display on Both LCDs**
```cpp
void handleCalculation() {
  // Calculate coin combination...
  
  // Display on ESP32 LCD (local)
  lcd.clear();
  lcd.print("Plan: 5P:X 10P:Y...");
  
  // Display on RPi LCD (remote) - NEW
  displayOnRPiLCD("Dispensing Coins", "5P:X 10P:Y", "20P:Z Tot:50", "Please wait...");
  
  currentState = STATE_DISPENSING;
}
```

#### **STATE_COMPLETE - Release Chit**
```cpp
void handleComplete() {
  // Display completion
  displayOnRPiLCD("Dispensing", "Complete!", "Releasing chit...", "");
  
  // Release chit via RPi servo - NEW
  releaseChitWithRPi();
  
  // Thank you message
  displayOnRPiLCD("Transaction", "Complete!", "Thank you!", "");
  
  // Reset RPi slave - NEW
  resetRPiSlave();
  
  currentState = STATE_IDLE;
}
```

## Complete Transaction Flow

```
┌─────────────────────────────────────────────────────┐
│ STEP 1: System Ready                               │
│ ESP32: Waiting for RPi...                          │
│ RPi → ESP32: "SLAVE_READY"                         │
│ Status: Both systems initialized                   │
└─────────────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────────┐
│ STEP 2: Periodic Chit Check (every 2 seconds)      │
│ ESP32 → RPi: "CHECK_IR"                            │
│ RPi → ESP32: "IR_CLEAR" or "IR_DETECTED"           │
└─────────────────────────────────────────────────────┘
                    ↓ (if IR_DETECTED)
┌─────────────────────────────────────────────────────┐
│ STEP 3: Update LCD                                 │
│ ESP32 → RPi: "DISPLAY:Chit Detected!|              │
│               Identifying...|Please wait...|"       │
│ RPi → ESP32: "DISPLAY_OK"                          │
└─────────────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────────┐
│ STEP 4: YOLO Detection (~5 seconds)                │
│ ESP32 → RPi: "DETECT_CHIT"                         │
│ RPi: Captures frames, runs YOLO                    │
│ RPi → ESP32: "CHIT_50" (or 5/10/20)                │
└─────────────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────────┐
│ STEP 5: Calculate Coin Combination (ESP32)         │
│ ESP32: Calculate optimal coins                     │
│   Example: 50 peso = 2×20P + 1×10P                 │
└─────────────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────────┐
│ STEP 6: Display Dispensing Plan                    │
│ ESP32 → RPi: "DISPLAY:Dispensing Coins|            │
│               5P:0 10P:1|20P:2 Tot:50|             │
│               Please wait...|"                      │
│ RPi → ESP32: "DISPLAY_OK"                          │
└─────────────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────────┐
│ STEP 7: Dispense Coins (ESP32 Controls)            │
│ ESP32: Activates coin hoppers                      │
│ ESP32: Counts coins via pulse detection            │
│ ESP32: Updates progress on both LCDs               │
└─────────────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────────┐
│ STEP 8: Display Completion                         │
│ ESP32 → RPi: "DISPLAY:Dispensing|Complete!|        │
│               Releasing chit...|"                   │
│ RPi → ESP32: "DISPLAY_OK"                          │
└─────────────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────────┐
│ STEP 9: Release Chit via Servo                     │
│ ESP32 → RPi: "DISPENSE_CHIT"                       │
│ RPi: Servo rotates 39° → 90° → 39°                │
│ RPi → ESP32: "DISPENSE_COMPLETE"                   │
└─────────────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────────┐
│ STEP 10: Thank You & Reset                         │
│ ESP32 → RPi: "DISPLAY:Transaction|Complete!|       │
│               Thank you!|"                          │
│ ESP32 → RPi: "RESET"                               │
│ RPi → ESP32: "RESET_OK"                            │
│ Status: Ready for next transaction                 │
└─────────────────────────────────────────────────────┘
```

## Hardware Connections

### ESP32 ↔ Raspberry Pi Serial
```
ESP32          →  Raspberry Pi
─────────────────────────────────
GND            →  GND (Pin 6, 9, 14...)
TX2 (GPIO17)   →  RXD (Pin 10, GPIO15)
RX2 (GPIO16)   →  TXD (Pin 8, GPIO14)
```

**Note**: Use Serial2 on ESP32 for RPi communication if Serial is used for debugging.

## Configuration

### Timing Constants (can be adjusted)
```cpp
const unsigned long IR_CHECK_INTERVAL = 2000;  // Check IR every 2 seconds

// Timeouts in waitForRPiResponse():
// CHECK_IR: 500ms
// DETECT_CHIT: 10000ms (10 seconds)
// DISPLAY: 500ms
// DISPENSE_CHIT: 3000ms (3 seconds)
// RESET: 500ms
```

### RPi Slave Check
```cpp
bool rpiSlaveReady = false;  // Set to true when SLAVE_READY received
```

## Testing

### 1. Test Serial Communication
```cpp
// In ESP32 Serial Monitor, type:
PING              // Should get PONG from RPi

// Or manually test detection:
CHECK_IR          // Should get IR_DETECTED or IR_CLEAR
DETECT_CHIT       // Should get CHIT_XX after ~5 seconds
```

### 2. Test Complete Workflow
1. Upload code to ESP32
2. Start RPi slave: `python3 esp32_comm.py`
3. Wait for "SLAVE_READY" message on ESP32 Serial Monitor
4. Insert chit into system
5. Watch automatic detection and dispensing

### 3. Test Commands (if test commands still work)
```
test_auto 50     // Simulate 50 peso detection and auto-dispense
```

## Troubleshooting

### ESP32 says "RPi slave not responding"
- Check serial wiring (TX→RX, RX→TX)
- Verify baud rate: 115200
- Ensure RPi script is running: `python3 esp32_comm.py`
- Check RPi serial port enabled: `ls -l /dev/serial0`

### YOLO detection timeouts
- Increase timeout in ESP32: change `10000` to `15000`
- Check RPi camera: `python3 -c "import cv2; cap=cv2.VideoCapture(0); print('OK' if cap.isOpened() else 'FAIL')"`
- Verify YOLO model loaded: Check RPi serial output

### Chit not releasing
- Check servo connections on RPi (GPIO22)
- Test servo independently: `python3 servo_tester.py`
- Verify pigpio daemon running: `sudo systemctl status pigpiod`

### LCD not updating
- Check I2C on RPi: `sudo i2cdetect -y 1`
- Should show address 0x27
- Test LCD: `python3 lcd_tester.py`

## Next Steps

1. **Upload code to ESP32**
   - Open CoinExchanger.ino in Arduino IDE
   - Select ESP32 board
   - Upload

2. **Start RPi slave**
   ```bash
   cd /home/pi/Desktop/PROJECTS/Chits-Exchanger/source/rpi/yolo
   python3 esp32_comm.py
   ```

3. **Test the system**
   - Insert chit
   - Watch automatic detection
   - Verify coin dispensing
   - Check chit release

4. **Optional: Set up auto-start**
   - Follow systemd service setup in `ESP32_SLAVE_README.md`

## Files Modified

- ✅ `source/esp32/CoinExchanger/CoinExchanger.ino` - ESP32 master code with RPi integration
- ✅ `source/rpi/yolo/esp32_comm.py` - RPi slave system (new)
- ✅ `source/rpi/yolo/ESP32_SLAVE_README.md` - Setup guide (new)
- ✅ `source/rpi/yolo/INTEGRATION_GUIDE.md` - Complete integration docs (new)
- ✅ `source/rpi/yolo/test_esp32_slave.py` - Testing utility (new)

---
**Status**: ✅ READY FOR TESTING  
**Date**: November 27, 2025  
**System**: Chits Exchanger - ESP32 Master + RPi Slave
