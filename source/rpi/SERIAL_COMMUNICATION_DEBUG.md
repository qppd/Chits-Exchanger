# ESP32-RPi Communication Issues and Solutions

**Date:** October 26, 2025  
**System:** Chit Exchanger - CoinExchanger.ino + yolo_detect.py

---

## 🔍 Problem Summary

The Raspberry Pi (`yolo_detect.py`) cannot communicate with ESP32 (`CoinExchanger.ino`) even though the `test_all` command works when connected directly via serial monitor.

---

## 📊 Analysis Results

### ✅ What's Working

1. **ESP32 Side (CoinExchanger.ino)**:
   - Serial initialization: `115200` baud ✓
   - Test commands work: `test_all`, `test_chit`, `help` ✓
   - Accepts RPi commands: `CHIT_DETECTED:<value>` ✓
   - Three relays for coin hoppers (GPIO 26, 25, 33) ✓

2. **Hardware**:
   - ESP32 connected to `/dev/ttyUSB0` (CP2102 UART Bridge) ✓
   - User in `dialout` group for serial access ✓
   - Webcam on `/dev/video0` ✓

### ❌ Communication Issues Found

#### Issue #1: Serial Port Detection
**Problem:** yolo_detect.py may not be using the correct serial port.

**Evidence:**
- ESP32 is on `/dev/ttyUSB0`
- yolo_detect.py defaults to `/dev/ttyUSB0` ✓ (This is correct)

#### Issue #2: Command Format Compatibility
**Problem:** yolo_detect.py sends commands with `\n` line ending, which should work with ESP32's `readStringUntil('\n')`.

**Evidence:**
- ESP32 expects: `CHIT_DETECTED:<value>\n`
- yolo_detect.py sends: `CHIT_DETECTED:50\n` ✓ (Format is correct)

#### Issue #3: Serial Buffer and Timing
**Problem:** Potential race condition or buffer issues.

**Symptoms:**
- ESP32 may not be ready when RPi starts sending
- 2-second delay in yolo_detect.py may not be enough
- No clearing of startup messages from ESP32

#### Issue #4: Error Handling
**Problem:** yolo_detect.py continues even if ESP32 connection fails.

**Code:**
```python
except Exception as e:
    print(f"WARNING: Could not connect to ESP32 on {args.esp32_port}: {e}")
    print("Continuing without ESP32 serial communication...")
    esp32_serial = None
```

This means the script runs but silently fails to communicate.

---

## 🔧 Solutions

### Solution 1: Test Serial Communication First

Use the diagnostic script to verify ESP32 communication:

```bash
cd /home/admin/Chits-Exchanger/source/rpi/test
python3 esp32_serial_tester.py
```

**What it tests:**
1. Serial port opening
2. Reading ESP32 startup messages
3. Sending test commands
4. Sending RPi protocol commands
5. Interactive testing

### Solution 2: Improve yolo_detect.py Serial Initialization

**Current code:**
```python
try:
    esp32_serial = serial.Serial(args.esp32_port, 115200, timeout=1)
    time.sleep(2)  # Wait for serial connection to establish
    print(f"Serial connection to ESP32 established on {args.esp32_port}")
except Exception as e:
    print(f"WARNING: Could not connect to ESP32...")
    esp32_serial = None
```

**Recommended improvements:**

1. **Clear startup messages:**
```python
# After opening serial port
time.sleep(2)
esp32_serial.reset_input_buffer()  # Clear ESP32 startup messages
esp32_serial.reset_output_buffer()
```

2. **Verify ESP32 is responding:**
```python
# Send a test command and wait for response
esp32_serial.write(b'help\n')
esp32_serial.flush()
time.sleep(1)

if esp32_serial.in_waiting > 0:
    response = esp32_serial.read(esp32_serial.in_waiting).decode()
    print(f"ESP32 responded: {response[:100]}...")  # Show first 100 chars
else:
    print("WARNING: ESP32 not responding to commands!")
```

3. **Better error messages:**
```python
except serial.SerialException as e:
    print(f"ERROR: Cannot open {args.esp32_port}: {e}")
    print("Troubleshooting:")
    print("  1. Check if ESP32 is connected: ls -la /dev/ttyUSB*")
    print("  2. Check permissions: groups (should include 'dialout')")
    print("  3. Check if port is in use: sudo lsof /dev/ttyUSB0")
    sys.exit(1)  # Don't continue without ESP32
```

### Solution 3: Add Serial Monitoring Thread

Add background thread to continuously monitor ESP32 messages:

```python
import threading

def esp32_monitor_thread():
    """Background thread to monitor ESP32 messages"""
    while esp32_running:
        try:
            if esp32_serial and esp32_serial.is_open and esp32_serial.in_waiting > 0:
                msg = esp32_serial.readline().decode().strip()
                if msg:
                    print(f"[ESP32] {msg}")
                    # Update LCD or handle specific messages
        except Exception as e:
            print(f"ESP32 monitor error: {e}")
        time.sleep(0.1)

# Start monitoring thread
esp32_running = True
monitor = threading.Thread(target=esp32_monitor_thread, daemon=True)
monitor.start()
```

### Solution 4: Verify Command Reception

After sending commands, verify ESP32 received them:

```python
def send_to_esp32(message):
    """Send message to ESP32 via serial with verification"""
    if esp32_serial and esp32_serial.is_open:
        try:
            esp32_serial.write((message + '\n').encode())
            esp32_serial.flush()
            print(f"→ ESP32: {message}")
            
            # Wait briefly for acknowledgment
            time.sleep(0.1)
            
            # Check for any immediate response
            if esp32_serial.in_waiting > 0:
                response = esp32_serial.readline().decode().strip()
                if response:
                    print(f"← ESP32: {response}")
                    
        except Exception as e:
            print(f"ERROR sending to ESP32: {e}")
            return False
    else:
        print("ERROR: ESP32 serial not available!")
        return False
    return True
```

---

## 🧪 Testing Procedure

### Step 1: Basic Serial Test

```bash
# Test serial connection directly
python3 /home/admin/Chits-Exchanger/source/rpi/test/esp32_serial_tester.py
```

**Expected results:**
- Serial port opens successfully
- Can read ESP32 startup messages
- `help` command shows available commands
- `test_chit 50` triggers ESP32 chit detection simulation

### Step 2: Test RPi Commands

In the serial tester interactive mode, try:
```
IR_DETECTED
CHIT_DETECTED:50
CHIT_RELEASED:50
```

**Expected ESP32 responses:**
- LCD updates
- State machine changes
- Confirmation messages in serial output

### Step 3: Test with yolo_detect.py

```bash
cd /home/admin/Chits-Exchanger/source/rpi/yolo
python3 yolo_detect.py --model <model_path> --esp32_port /dev/ttyUSB0
```

Monitor serial output to verify:
- Connection established
- IR sensor triggers work
- Detection messages sent
- ESP32 acknowledgments received

---

## 🐛 Common Issues and Fixes

### Issue: "Permission denied" on /dev/ttyUSB0

**Fix:**
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Log out and log back in, or run:
newgrp dialout

# Verify
groups  # Should show 'dialout'
```

### Issue: "Device or resource busy"

**Fix:**
```bash
# Check what's using the port
sudo lsof /dev/ttyUSB0

# Kill the process if needed
sudo killall -9 python3

# Or restart
sudo reboot
```

### Issue: No output from ESP32

**Possible causes:**
1. Wrong baud rate (must be 115200)
2. ESP32 not running CoinExchanger.ino
3. TX/RX wires disconnected
4. ESP32 not powered

**Fix:**
```bash
# Test with screen
screen /dev/ttyUSB0 115200

# Press Ctrl+A then K to exit

# Or test with minicom
minicom -D /dev/ttyUSB0 -b 115200
```

### Issue: Commands not recognized

**Check:**
1. Commands are case-sensitive
2. Line ending is `\n` (not `\r\n`)
3. No extra spaces before/after command
4. Command format matches ESP32 expectations

---

## 📝 Recommended Code Changes for yolo_detect.py

### Change 1: Better Serial Initialization (Lines 183-195)

**Before:**
```python
try:
    esp32_serial = serial.Serial(args.esp32_port, 115200, timeout=1)
    time.sleep(2)
    print(f"Serial connection to ESP32 established on {args.esp32_port}")
except Exception as e:
    print(f"WARNING: Could not connect to ESP32 on {args.esp32_port}: {e}")
    print("Continuing without ESP32 serial communication...")
    esp32_serial = None
```

**After:**
```python
try:
    print(f"Opening serial connection to ESP32 on {args.esp32_port}...")
    esp32_serial = serial.Serial(args.esp32_port, 115200, timeout=1)
    time.sleep(2.5)  # Longer delay for ESP32 startup
    
    # Clear any startup messages
    esp32_serial.reset_input_buffer()
    esp32_serial.reset_output_buffer()
    
    # Verify ESP32 is responding
    esp32_serial.write(b'help\n')
    esp32_serial.flush()
    time.sleep(0.5)
    
    response_received = False
    if esp32_serial.in_waiting > 0:
        response = esp32_serial.read_all().decode('utf-8', errors='ignore')
        if 'COIN EXCHANGER' in response or 'HELP' in response.upper():
            response_received = True
            print(f"✅ ESP32 connection verified on {args.esp32_port}")
        else:
            print(f"⚠️  ESP32 response: {response[:200]}")
    
    if not response_received:
        print("⚠️  ESP32 not responding to commands. Check if CoinExchanger.ino is running.")
    
except serial.SerialException as e:
    print(f"❌ ERROR: Cannot open {args.esp32_port}: {e}")
    print("\nTroubleshooting:")
    print("  1. Check connection: ls -la /dev/ttyUSB*")
    print("  2. Check permissions: groups (should include 'dialout')")
    print("  3. Check if port is in use: sudo lsof /dev/ttyUSB0")
    print("  4. Try unplugging and replugging ESP32")
    sys.exit(1)
except Exception as e:
    print(f"❌ ERROR: Unexpected error connecting to ESP32: {e}")
    sys.exit(1)
```

### Change 2: Enhanced send_to_esp32() (Lines 275-284)

**Before:**
```python
def send_to_esp32(message):
    """Send message to ESP32 via serial"""
    if esp32_serial and esp32_serial.is_open:
        try:
            esp32_serial.write((message + '\n').encode())
            esp32_serial.flush()
            print(f"Sent to ESP32: {message}")
        except Exception as e:
            print(f"Error sending to ESP32: {e}")
```

**After:**
```python
def send_to_esp32(message):
    """Send message to ESP32 via serial with verification"""
    if esp32_serial and esp32_serial.is_open:
        try:
            esp32_serial.write((message + '\n').encode())
            esp32_serial.flush()
            print(f"→ ESP32: {message}")
            
            # Wait briefly and check for acknowledgment
            time.sleep(0.15)
            if esp32_serial.in_waiting > 0:
                response = esp32_serial.readline().decode('utf-8', errors='ignore').strip()
                if response:
                    print(f"← ESP32: {response}")
            
            return True
        except Exception as e:
            print(f"❌ Error sending to ESP32: {e}")
            return False
    else:
        print(f"❌ ESP32 serial not available!")
        return False
```

### Change 3: Enhanced read_from_esp32() (Lines 286-306)

**Before:**
```python
def read_from_esp32():
    """Read messages from ESP32 if available"""
    if esp32_serial and esp32_serial.is_open:
        try:
            if esp32_serial.in_waiting > 0:
                message = esp32_serial.readline().decode().strip()
                # ... LCD update code ...
                return message
        except Exception as e:
            print(f"Error reading from ESP32: {e}")
    return None
```

**After:**
```python
def read_from_esp32():
    """Read messages from ESP32 if available"""
    if esp32_serial and esp32_serial.is_open:
        try:
            messages = []
            while esp32_serial.in_waiting > 0:
                message = esp32_serial.readline().decode('utf-8', errors='ignore').strip()
                if message:
                    messages.append(message)
                    print(f"← ESP32: {message}")
                    
                    # Update LCD with ESP32 messages
                    if message.startswith("DISPENSING_COMPLETE"):
                        lcd.display_lines(
                            "ESP32:",
                            "Dispensing",
                            "Complete!",
                            ""
                        )
                        time.sleep(2)
                        lcd.display_lines(
                            "Ready",
                            "Waiting for chit...",
                            "",
                            ""
                        )
            
            return messages if messages else None
        except Exception as e:
            print(f"❌ Error reading from ESP32: {e}")
    return None
```

---

## ✅ Verification Checklist

After making changes, verify:

- [ ] Serial port opens successfully
- [ ] ESP32 startup messages visible
- [ ] `help` command shows ESP32 commands
- [ ] `test_chit 50` triggers simulation on ESP32
- [ ] `IR_DETECTED` command received by ESP32
- [ ] `CHIT_DETECTED:50` triggers state machine
- [ ] ESP32 LCD shows detection status
- [ ] Bidirectional communication working
- [ ] Error messages are clear and helpful

---

## 📞 Quick Reference

**Serial Port:** `/dev/ttyUSB0` (CP2102 UART Bridge)  
**Baud Rate:** `115200`  
**Line Ending:** `\n`  
**ESP32 Commands:**
- `help` - Show commands
- `test_chit <value>` - Simulate chit detection (5, 10, 20, 50)
- `test_all` - Test all components
- `test_relay <id> on/off` - Test individual relay (1-3)
- `test_pulse <id>` - Test pulse detection (1-3)

**RPi → ESP32 Commands:**
- `IR_DETECTED` - IR sensor triggered
- `CHIT_DETECTED:<value>` - Chit detected (5, 10, 20, 50)
- `CHIT_RELEASED:<value>` - Chit released by servo
- `DETECTION_TIMEOUT` - Detection timed out
- `SYSTEM_SHUTDOWN` - RPi shutting down

**ESP32 → RPi Messages:**
- `DISPENSING_COMPLETE:<value>` - Coins dispensed
- Various status and debug messages

---

## 🚀 Next Steps

1. Run `esp32_serial_tester.py` to verify basic communication
2. Apply recommended code changes to `yolo_detect.py`
3. Test with actual hardware (IR sensor + chits)
4. Monitor both ESP32 and RPi serial output simultaneously
5. Document any additional issues found

---

**Created:** October 26, 2025  
**Last Updated:** October 26, 2025
