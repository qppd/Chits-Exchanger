# ESP32-RPi Integration Guide for Chit Exchanger

## System Architecture

```
┌──────────────────────────────────────────────────────────┐
│                    ESP32 Master                          │
│              (CoinExchanger.ino)                         │
│                                                          │
│  Responsibilities:                                       │
│  • Receive coins from user                              │
│  • Control 3 coin hoppers (5, 10, 20 peso)             │
│  • Calculate optimal coin combinations                  │
│  • Manage transaction state machine                     │
│  • Command RPi slave for chit operations                │
└────────────────┬─────────────────────────────────────────┘
                 │
                 │ Serial (TX/RX)
                 │ 115200 baud
                 │
┌────────────────▼─────────────────────────────────────────┐
│                Raspberry Pi Slave                        │
│                 (esp32_comm.py)                          │
│                                                          │
│  Responsibilities:                                       │
│  • Detect chit insertion (IR sensor)                    │
│  • Identify chit denomination (YOLO)                    │
│  • Display transaction status (LCD)                     │
│  • Dispense chit after exchange (Servo)                 │
│  • Respond to ESP32 commands only                       │
└──────────────────────────────────────────────────────────┘
```

## Communication Protocol

### Command Format
- **Direction**: ESP32 → RPi
- **Format**: Plain text commands ending with `\n`
- **Response**: RPi → ESP32, plain text ending with `\n`

### Command Reference

| ESP32 Sends | RPi Responds | Description | Timing |
|-------------|--------------|-------------|--------|
| `CHECK_IR` | `IR_DETECTED` or `IR_CLEAR` | Check if chit present | <100ms |
| `DETECT_CHIT` | `CHIT_5/10/20/50` or `CHIT_UNKNOWN` | Run YOLO detection | ~5 seconds |
| `DISPLAY:msg` | `DISPLAY_OK` | Update LCD (lines split by \|) | <100ms |
| `DISPENSE_CHIT` | `DISPENSE_COMPLETE` | Release chit via servo | ~1.5 seconds |
| `PING` | `PONG` | Test communication | <50ms |
| `RESET` | `RESET_OK` | Reset to ready state | <100ms |

### Error Handling
- If RPi encounters error: `ERROR:<description>`
- ESP32 should implement timeout for all commands
- Recommended timeouts:
  - `CHECK_IR`: 500ms
  - `DETECT_CHIT`: 10 seconds
  - `DISPLAY`: 500ms
  - `DISPENSE_CHIT`: 3 seconds
  - `PING`: 200ms

## Transaction Workflow

### Complete Exchange Process

```
User inserts chit
       ↓
┌──────────────────────────────────────────────┐
│ STEP 1: Detect Chit Insertion               │
├──────────────────────────────────────────────┤
│ ESP32 → RPi: CHECK_IR                        │
│ RPi → ESP32: IR_DETECTED                     │
└──────────────────────────────────────────────┘
       ↓
┌──────────────────────────────────────────────┐
│ STEP 2: Identify Chit Denomination          │
├──────────────────────────────────────────────┤
│ ESP32 → RPi: DETECT_CHIT                     │
│ RPi: Captures frames, runs YOLO (~5 sec)    │
│ RPi → ESP32: CHIT_50                         │
└──────────────────────────────────────────────┘
       ↓
┌──────────────────────────────────────────────┐
│ STEP 3: Display Chit Value                  │
├──────────────────────────────────────────────┤
│ ESP32 → RPi: DISPLAY:Chit Detected!|        │
│              Value: P50|Please wait...|      │
│ RPi → ESP32: DISPLAY_OK                      │
└──────────────────────────────────────────────┘
       ↓
┌──────────────────────────────────────────────┐
│ STEP 4: Calculate Coin Combination (ESP32)  │
├──────────────────────────────────────────────┤
│ ESP32: Calculate optimal coins              │
│   50 peso = 2×20P + 1×10P                   │
└──────────────────────────────────────────────┘
       ↓
┌──────────────────────────────────────────────┐
│ STEP 5: Display Dispensing Plan             │
├──────────────────────────────────────────────┤
│ ESP32 → RPi: DISPLAY:Dispensing Coins|      │
│              5P:0 10P:1 20P:2||              │
│ RPi → ESP32: DISPLAY_OK                      │
└──────────────────────────────────────────────┘
       ↓
┌──────────────────────────────────────────────┐
│ STEP 6: Dispense Coins (ESP32 Controls)     │
├──────────────────────────────────────────────┤
│ ESP32: Activates coin hoppers               │
│ ESP32: Counts coins via pulse detection     │
│ ESP32: Updates LCD via RPi periodically     │
│ ESP32 → RPi: DISPLAY:Dispensing...|         │
│              Progress: 33%|20P: 1/2||        │
└──────────────────────────────────────────────┘
       ↓
┌──────────────────────────────────────────────┐
│ STEP 7: Display Completion                  │
├──────────────────────────────────────────────┤
│ ESP32 → RPi: DISPLAY:Dispensing Complete!|  │
│              Releasing chit...|              │
│ RPi → ESP32: DISPLAY_OK                      │
└──────────────────────────────────────────────┘
       ↓
┌──────────────────────────────────────────────┐
│ STEP 8: Release Chit                        │
├──────────────────────────────────────────────┤
│ ESP32 → RPi: DISPENSE_CHIT                  │
│ RPi: Servo rotates 39° → 90° → 39°         │
│ RPi → ESP32: DISPENSE_COMPLETE               │
└──────────────────────────────────────────────┘
       ↓
┌──────────────────────────────────────────────┐
│ STEP 9: Display Thank You                   │
├──────────────────────────────────────────────┤
│ ESP32 → RPi: DISPLAY:Transaction Complete!| │
│              Thank you!||                    │
└──────────────────────────────────────────────┘
       ↓
┌──────────────────────────────────────────────┐
│ STEP 10: Reset to Ready State               │
├──────────────────────────────────────────────┤
│ ESP32 → RPi: RESET                           │
│ RPi → ESP32: RESET_OK                        │
│ System ready for next transaction           │
└──────────────────────────────────────────────┘
```

## ESP32 Arduino Integration Code

### Basic Serial Communication Functions

```cpp
// Global serial buffer
String rpiResponse = "";
unsigned long lastCommandTime = 0;

void setupRPiCommunication() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println("ESP32 Ready");
  
  // Wait for RPi slave to initialize
  Serial.println("Waiting for RPi slave...");
  unsigned long startWait = millis();
  
  while (millis() - startWait < 30000) {  // 30 second timeout
    if (Serial.available()) {
      String msg = Serial.readStringUntil('\n');
      msg.trim();
      if (msg == "SLAVE_READY") {
        Serial.println("✓ RPi slave connected!");
        return;
      }
    }
    delay(100);
  }
  
  Serial.println("⚠ RPi slave not responding. Check connection.");
}

void sendToRPi(String command) {
  Serial.println(command);
  Serial.flush();
  lastCommandTime = millis();
  
  #ifdef DEBUG_SERIAL
  Serial.print("[TX→RPi] ");
  Serial.println(command);
  #endif
}

String waitForRPiResponse(unsigned long timeout) {
  unsigned long startTime = millis();
  String response = "";
  
  while (millis() - startTime < timeout) {
    if (Serial.available()) {
      response = Serial.readStringUntil('\n');
      response.trim();
      
      if (response.length() > 0) {
        #ifdef DEBUG_SERIAL
        Serial.print("[RX←RPi] ");
        Serial.println(response);
        #endif
        return response;
      }
    }
    delay(10);
  }
  
  return "TIMEOUT";
}

String sendCommandAndWait(String command, unsigned long timeout) {
  sendToRPi(command);
  return waitForRPiResponse(timeout);
}
```

### Example: Chit Detection Function

```cpp
int detectChit() {
  // Step 1: Check if chit is present via IR
  sendToRPi("CHECK_IR");
  String response = waitForRPiResponse(500);  // 500ms timeout
  
  if (response == "TIMEOUT") {
    Serial.println("⚠ RPi not responding to CHECK_IR");
    return -1;
  }
  
  if (response != "IR_DETECTED") {
    Serial.println("No chit detected by IR sensor");
    return 0;
  }
  
  Serial.println("✓ IR sensor detected chit");
  
  // Step 2: Display "detecting" message
  sendToRPi("DISPLAY:Chit Detected!|Identifying...|Please wait...|");
  delay(100);
  
  // Step 3: Run YOLO detection
  Serial.println("Running YOLO detection...");
  sendToRPi("DETECT_CHIT");
  response = waitForRPiResponse(10000);  // 10 second timeout for YOLO
  
  if (response == "TIMEOUT") {
    Serial.println("⚠ YOLO detection timeout");
    sendToRPi("DISPLAY:Detection Failed|Timeout|Please try again|");
    return -1;
  }
  
  // Parse chit value from response
  if (response == "CHIT_5") return 5;
  if (response == "CHIT_10") return 10;
  if (response == "CHIT_20") return 20;
  if (response == "CHIT_50") return 50;
  if (response == "CHIT_UNKNOWN") {
    Serial.println("⚠ Could not identify chit");
    sendToRPi("DISPLAY:Unknown Chit|Cannot identify|Please try again|");
    return -1;
  }
  
  // Unknown response
  Serial.print("⚠ Unexpected response: ");
  Serial.println(response);
  return -1;
}
```

### Example: Display Update Function

```cpp
void displayOnRPiLCD(String line1, String line2 = "", String line3 = "", String line4 = "") {
  String message = "DISPLAY:" + line1;
  if (line2.length() > 0) message += "|" + line2;
  if (line3.length() > 0) message += "|" + line3;
  if (line4.length() > 0) message += "|" + line4;
  
  sendToRPi(message);
  String response = waitForRPiResponse(500);
  
  if (response != "DISPLAY_OK") {
    Serial.println("⚠ LCD display update failed");
  }
}
```

### Example: Complete Transaction Function

```cpp
void handleChitExchange() {
  // Step 1: Detect chit
  displayOnRPiLCD("Ready", "Insert chit...", "", "");
  
  int chitValue = detectChit();
  if (chitValue <= 0) {
    return;  // Detection failed
  }
  
  Serial.print("Detected chit value: P");
  Serial.println(chitValue);
  
  // Step 2: Display detected value
  String line2 = "Value: P" + String(chitValue);
  displayOnRPiLCD("Chit Detected!", line2, "Calculating...", "");
  delay(1000);
  
  // Step 3: Calculate coin combination
  DispensePlan plan = calculateCoinCombination(chitValue, 5);
  
  // Step 4: Display dispensing plan
  String planLine = "5P:" + String(plan.coins_5) + " 10P:" + String(plan.coins_10);
  String planLine2 = "20P:" + String(plan.coins_20) + " Tot:" + String(plan.totalValue);
  displayOnRPiLCD("Dispensing Coins", planLine, planLine2, "Please wait...");
  
  // Step 5: Dispense coins
  bool success = dispenseCoins(plan);
  
  if (!success) {
    displayOnRPiLCD("Dispensing Error", "Please contact", "staff", "");
    delay(5000);
    return;
  }
  
  // Step 6: Display completion
  displayOnRPiLCD("Dispensing", "Complete!", "Releasing chit...", "");
  delay(1000);
  
  // Step 7: Release chit
  sendToRPi("DISPENSE_CHIT");
  String response = waitForRPiResponse(3000);
  
  if (response == "DISPENSE_COMPLETE") {
    Serial.println("✓ Chit dispensed successfully");
  } else {
    Serial.println("⚠ Chit dispense failed");
  }
  
  // Step 8: Thank you message
  displayOnRPiLCD("Transaction", "Complete!", "Thank you!", "");
  delay(3000);
  
  // Step 9: Reset
  sendToRPi("RESET");
  waitForRPiResponse(500);
  
  Serial.println("Transaction complete. Ready for next chit.");
}
```

### Integration in Main Loop

```cpp
void loop() {
  // Handle serial commands from RPi
  handleRPiMessages();
  
  // Update coin hoppers
  coinHopper1.update();
  coinHopper2.update();
  coinHopper3.update();
  
  // State machine
  switch (currentState) {
    case STATE_IDLE:
      // Periodically check for chit
      if (millis() - lastCheckTime > 1000) {
        lastCheckTime = millis();
        
        // Quick IR check (non-blocking)
        sendToRPi("CHECK_IR");
        // Don't wait for response here, handle in handleRPiMessages()
      }
      break;
      
    case STATE_CHIT_DETECTED:
      handleChitExchange();
      currentState = STATE_IDLE;
      break;
      
    // ... other states
  }
  
  delay(10);
}

void handleRPiMessages() {
  if (Serial.available()) {
    String message = Serial.readStringUntil('\n');
    message.trim();
    
    if (message == "IR_DETECTED") {
      currentState = STATE_CHIT_DETECTED;
    }
    else if (message == "IR_CLEAR") {
      // No action needed
    }
    else if (message.startsWith("ERROR:")) {
      Serial.print("RPi Error: ");
      Serial.println(message.substring(6));
    }
    // ... handle other messages
  }
}
```

## Hardware Wiring

### ESP32 ↔ Raspberry Pi Serial Connection

```
ESP32 Pin          →  Raspberry Pi Pin
─────────────────────────────────────────
GND                →  GND (Pin 6, 9, 14, 20, 25, 30, 34, 39)
TX2 (GPIO17)       →  RXD (GPIO15, Pin 10)
RX2 (GPIO16)       →  TXD (GPIO14, Pin 8)
```

**Important Notes:**
- ESP32 uses 3.3V logic (compatible with RPi)
- Enable serial port on RPi: `sudo raspi-config`
- Disable serial console, enable serial hardware
- Use `/dev/serial0` in Python code

### Raspberry Pi GPIO Connections

```
Component          GPIO (BCM)    Physical Pin
──────────────────────────────────────────────
IR Sensor          GPIO17        Pin 11
Servo Signal       GPIO22        Pin 15
LCD SDA            GPIO2         Pin 3
LCD SCL            GPIO3         Pin 5
Ground (Common)    GND           Pin 6, 9, 14...
5V Power (LCD)     5V            Pin 2, 4
```

## Configuration Files

### esp32_comm.py Configuration

Edit these constants at the top of `esp32_comm.py`:

```python
# GPIO Pins (BCM numbering)
IR_SENSOR_PIN = 17
SERVO_PIN = 22

# Servo angles
SERVO_INITIAL_ANGLE = 39
SERVO_RELEASE_ANGLE = 90

# LCD Configuration
LCD_I2C_ADDR = 0x27  # Use i2cdetect to verify
LCD_WIDTH = 20
LCD_ROWS = 4

# Serial Configuration
SERIAL_PORT = '/dev/serial0'
SERIAL_BAUD = 115200

# YOLO Configuration
MODEL_PATH = '/path/to/your/chit_model.pt'
CAMERA_INDEX = 0
DETECTION_TIMEOUT = 5.0
CONFIDENCE_THRESHOLD = 0.6
```

## Troubleshooting

### Serial Communication Issues

**Problem**: ESP32 and RPi not communicating

**Solutions**:
1. Check wiring (TX → RX, RX → TX)
2. Verify baud rate matches (115200)
3. Test serial port:
   ```bash
   # On RPi
   ls -l /dev/serial0
   sudo minicom -b 115200 -o -D /dev/serial0
   ```
4. Check if serial console is disabled:
   ```bash
   sudo raspi-config
   # Interface Options → Serial Port
   # Login shell: NO
   # Serial hardware: YES
   ```

### Timeout Issues

**Problem**: Commands timing out

**Solutions**:
1. Increase timeout values in ESP32 code
2. Check YOLO detection time (should be ~5 seconds)
3. Verify RPi is not overloaded (check `htop`)
4. Ensure camera is working properly

### YOLO Detection Failures

**Problem**: `CHIT_UNKNOWN` responses

**Solutions**:
1. Verify model path is correct
2. Check camera lighting conditions
3. Adjust `CONFIDENCE_THRESHOLD` (try 0.5 instead of 0.6)
4. Increase `DETECTION_TIMEOUT` (try 8 seconds)
5. Test camera independently:
   ```bash
   python3 -c "import cv2; cap = cv2.VideoCapture(0); ret, frame = cap.read(); print('OK' if ret else 'FAIL'); cap.release()"
   ```

## Testing Procedures

### 1. Test RPi Slave Independently

```bash
# Terminal 1: Start slave
cd /home/pi/Desktop/PROJECTS/Chits-Exchanger/source/rpi/yolo
python3 esp32_comm.py

# Terminal 2: Send test commands
python3 test_esp32_slave.py
```

### 2. Test Serial Connection

```bash
# On RPi, echo test
echo "PING" > /dev/serial0

# On ESP32 Serial Monitor, you should see "PING"
# Type "PONG" in ESP32 Serial Monitor
```

### 3. Test ESP32 Integration

Upload test sketch to ESP32:

```cpp
void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("CHECK_IR");
  delay(2000);
  
  Serial.println("DETECT_CHIT");
  delay(10000);
  
  Serial.println("DISPLAY:Test|Message|Working!|");
  delay(2000);
  
  Serial.println("DISPENSE_CHIT");
}

void loop() {
  if (Serial.available()) {
    String response = Serial.readStringUntil('\n');
    Serial.print("Got: ");
    Serial.println(response);
  }
}
```

## Performance Metrics

| Operation | Expected Time | Timeout Recommended |
|-----------|---------------|---------------------|
| CHECK_IR | < 50ms | 500ms |
| DETECT_CHIT | 3-7 seconds | 10 seconds |
| DISPLAY | < 50ms | 500ms |
| DISPENSE_CHIT | ~1.5 seconds | 3 seconds |
| RESET | < 50ms | 500ms |

## Security Considerations

1. **Serial Port Access**: Ensure only authorized users can access `/dev/serial0`
2. **Command Validation**: RPi validates all commands before execution
3. **Error Handling**: Both systems handle malformed commands gracefully
4. **Timeout Protection**: All operations have timeout limits
5. **State Machine**: ESP32 maintains transaction state to prevent race conditions

## Support

For issues or questions:
1. Check `ESP32_SLAVE_README.md` for detailed setup
2. Review this integration guide
3. Test individual components using test scripts
4. Check system logs: `journalctl -u chit-slave.service -f`

---
**Document Version**: 1.0  
**Last Updated**: November 27, 2025  
**System**: Chits Exchanger - ESP32-RPi Integration
