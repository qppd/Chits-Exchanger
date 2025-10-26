# Auto Dispense Feature - Changes Summary

## Date: October 26, 2025

## Overview
Modified the Chit Exchanger system to automatically dispense chits based on detected denomination (5, 10, 20, 50 pesos) without requiring manual button selection.

## Changes Made

### 1. `/source/rpi/yolo/yolo_detect.py`
**What changed:**
- Added automatic dispensing trigger after chit detection and release
- System now sends `AUTO_DISPENSE:<value>` command to ESP32 immediately after releasing the chit
- Removed need for user to select denomination - it's now fully automatic

**How it works:**
1. IR sensor detects chit insertion
2. YOLO model identifies denomination (5, 10, 20, or 50)
3. System releases the chit via servo
4. **NEW:** Automatically sends `AUTO_DISPENSE:<value>` to ESP32
5. ESP32 dispenses the equivalent chits

**Code added:**
```python
# Auto-dispense: Send command to ESP32 to dispense detected amount
print(f"AUTO-DISPENSING: Triggering dispense of ₱{detected_chit_value}")
send_to_esp32(f"AUTO_DISPENSE:{detected_chit_value}")

# Update LCD
lcd.display_lines(
    "AUTO DISPENSE!",
    f"Chit: P{detected_chit_value}",
    "Dispensing...",
    "Please wait"
)
time.sleep(3)
```

### 2. `/source/esp32/CoinExchanger/CoinExchanger.ino` ✅ ACTIVE
**What changed:**
- Added handler for `AUTO_DISPENSE:<value>` serial command from RPi
- ESP32 now accepts chit value from RPi and triggers automatic **COIN** dispensing using hoppers
- Automatically calculates optimal coin combination (5, 10, 20 peso coins)
- Dispenses coins using coin hoppers with pulse counting

**How it works:**
1. ESP32 receives `AUTO_DISPENSE:<value>` command via serial
2. Sets `detectedChitValue` to the received value
3. Calculates optimal coin combination using existing algorithm
4. Displays dispensing plan on LCD
5. Triggers `STATE_DISPENSING` to start coin hopper dispensing
6. Monitors pulse counting from each hopper to verify coins dispensed
7. Returns to idle state when complete

**Code added:**
```cpp
// Handle AUTO_DISPENSE:value - Automatic dispensing trigger
if (command.startsWith("AUTO_DISPENSE:")) {
  int colonIndex = command.indexOf(':');
  if (colonIndex > 0) {
    String valueStr = command.substring(colonIndex + 1);
    int chitValue = valueStr.toInt();
    
    if (chitValue == 5 || chitValue == 10 || chitValue == 20 || chitValue == 50) {
      Serial.println("🎯 AUTO_DISPENSE received: P" + String(chitValue));
      
      // Set the detected chit value
      detectedChitValue = chitValue;
      selectedDenomination = 5;  // Default to 5 peso coins
      
      // Calculate optimal coin combination
      currentPlan = calculateCoinCombination(detectedChitValue, selectedDenomination);
      
      // Display plan on LCD and Serial
      // [LCD display code...]
      
      // Start dispensing immediately
      currentState = STATE_DISPENSING;
      dispensedCoins = 0;
    }
  }
}
```

### 3. `/source/esp32/ChitExchanger/ChitExchanger.ino` (Alternative - Not Active)
**Note:** This version is for chit dispensing (servo-based), not currently in use.
The ChitExchanger code was also modified but CoinExchanger is the active firmware.

## Operation Flow

### Before (Manual Selection):
1. Insert chit → Detect → Release → **Wait for button** → ESP32 dispenses

### After (Automatic):
1. Insert chit → Detect → Release → **Auto command** → ESP32 dispenses

## Testing Instructions

### 1. Upload ESP32 Code
```bash
# Upload the modified ChitExchanger.ino to your ESP32
# Use Arduino IDE or PlatformIO
```

### 2. Run YOLO Detection System
```bash
cd /home/admin/Chits-Exchanger/source/rpi/yolo

# Make sure you have the correct model path
python3 yolo_detect.py \
  --model <path_to_your_model> \
  --esp32_port /dev/ttyUSB1 \
  --camera 0
```

### 3. Test the System
1. Insert a chit (5, 10, 20, or 50 peso)
2. Wait for IR detection and YOLO recognition
3. System will automatically:
   - Release the chit
   - Send AUTO_DISPENSE command to ESP32
   - ESP32 will dispense equivalent chits

## Serial Communication Protocol

### Commands from RPi to ESP32:
- `IR_DETECTED` - IR sensor detected something
- `CHIT_DETECTED:<value>` - YOLO detected specific denomination
- `CHIT_RELEASED:<value>` - Chit was released by servo
- **`AUTO_DISPENSE:<value>`** - **NEW: Trigger automatic dispensing**
- `DETECTION_TIMEOUT` - Detection failed/timed out
- `SYSTEM_SHUTDOWN` - System is shutting down

### Commands from ESP32 to RPi:
- `DISPENSING_COMPLETE` - Dispensing finished
- (Other status messages as needed)

## Hardware Requirements
- Raspberry Pi with camera
- ESP32 with ChitExchanger firmware
- IR sensor on GPIO 17
- Servo on GPIO 22
- Serial connection: RPi → ESP32 (typically /dev/ttyUSB0 or /dev/ttyUSB1)
- I2C LCD display (optional)

## Baud Rates
- **ChitExchanger ESP32:** 9600 baud (servo-based chit dispensing)
- **CoinExchanger ESP32:** 115200 baud ✅ **ACTIVE** (coin hopper dispensing)

Make sure you're using the correct baud rate for your ESP32 firmware!

## Current Setup
Based on your system:
- **ESP32 Port:** `/dev/ttyUSB0` (was `/dev/ttyUSB1` earlier)
- **Firmware:** CoinExchanger (115200 baud)
- **Hardware:** 3 Coin Hoppers (5, 10, 20 peso) with pulse counting
- **Communication:** Working! ESP32 responds to CHIT_DETECTED commands

## Troubleshooting

### ESP32 not responding to AUTO_DISPENSE
1. Check serial connection: `ls -l /dev/ttyUSB*`
2. Verify baud rate (9600 for ChitExchanger)
3. Check ESP32 serial monitor for received commands

### Chit not dispensing
1. Verify ESP32 received AUTO_DISPENSE command (check serial output)
2. Check if `triggerAutoDispensing()` is being called
3. Verify servo connections and PCA9685 I2C address (0x40)

### Detection issues
1. Check IR sensor connection (GPIO 17)
2. Verify YOLO model path
3. Check camera connection (`/dev/video0`)
4. Review confidence threshold (default: 0.5)

## Future Enhancements
- Add confirmation beep/sound when dispensing starts
- Add error handling for partial dispensing
- Add dispensing history log
- Add manual override option
- Add chit value validation

## Notes
- The system maintains backward compatibility with TEST and TESTALL commands
- The original coin and bill acceptor functionality remains unchanged
- Auto-dispense only triggers when valid chit values (5, 10, 20, 50) are detected
