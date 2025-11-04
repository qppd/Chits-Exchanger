# Coin Exchanger Testing Guide

Complete testing guide for the integrated YOLO chit detection and coin dispensing system.

## System Overview

The system consists of two main components:
1. **RPi with YOLO Detection** (`yolo_detect.py`) - Detects chit denominations
2. **ESP32 Coin Exchanger** (`CoinExchanger.ino`) - Dispenses coins based on detected value

## Hardware Configuration

### Coin Hoppers (ESP32)
- **Hopper 1 (5 PHP coins)**
  - Pulse Detection: GPIO4
  - SSR Control: GPIO26
  
- **Hopper 2 (10 PHP coins)**
  - Pulse Detection: GPIO18
  - SSR Control: GPIO25
  
- **Hopper 3 (20 PHP coins)**
  - Pulse Detection: GPIO19
  - SSR Control: GPIO33

### Dispensing Logic

| Chit Value | Hopper 1 (5₱) | Hopper 2 (10₱) | Hopper 3 (20₱) | Total |
|------------|---------------|----------------|----------------|-------|
| 5 PHP      | 1 coin        | -              | -              | 5₱    |
| 10 PHP     | -             | 1 coin         | -              | 10₱   |
| 20 PHP     | -             | -              | 1 coin         | 20₱   |
| 50 PHP     | -             | 1 coin         | 2 coins        | 50₱   |

## Serial Communication Protocol

### RPi → ESP32 Commands

```
AUTO_DISPENSE:<value>   - Trigger automatic coin dispensing
    Example: AUTO_DISPENSE:50

IR_DETECTED             - IR sensor triggered (informational)

DETECTION_TIMEOUT       - No chit detected within timeout period
```

### ESP32 → RPi Responses

```
DISPENSING_COMPLETE:<value>   - Coins successfully dispensed
    Example: DISPENSING_COMPLETE:50
```

## Testing Procedures

### 1. Test Individual Hopper Components

Test each hopper's pulse detection and SSR relay independently.

#### Test Pulse Detection
```bash
# Connect to ESP32 Serial Monitor (115200 baud)

# Test Hopper 1 pulse reading
test_pulse 1

# Test Hopper 2 pulse reading
test_pulse 2

# Test Hopper 3 pulse reading
test_pulse 3
```

**Expected Result:** 
- Drop coins manually into the hopper
- System should detect and count each coin pulse
- Display: `✅ PULSE detected! Coin #X | Rate: X.XX coins/sec`

#### Test SSR Relay Control
```bash
# Test Hopper 1 relay
test_relay 1 on    # Turn ON relay (hopper motor runs)
test_relay 1 off   # Turn OFF relay (hopper motor stops)

# Test Hopper 2 relay
test_relay 2 on
test_relay 2 off

# Test Hopper 3 relay
test_relay 3 on
test_relay 3 off
```

**Expected Result:**
- ON: Hopper motor starts, LED indicator lights
- OFF: Hopper motor stops, LED indicator off

### 2. Test Individual Hopper Dispensing

Test complete dispensing cycle with pulse counting for each hopper.

```bash
# Dispense 3 coins from Hopper 1 (5 peso)
test_hopper 1 3

# Dispense 5 coins from Hopper 2 (10 peso)
test_hopper 2 5

# Dispense 2 coins from Hopper 3 (20 peso)
test_hopper 3 2
```

**Expected Result:**
```
=== HOPPER DISPENSING TEST ===
Hopper: 1
Coin Value: 5 PHP
Coins to Dispense: 3
Total Amount: 15 PHP
Pulse GPIO: 4
SSR GPIO: 26
==============================
🟢 Turning ON SSR relay...
🪙 Starting coin dispensing...
Progress: 1/3 coins (5/15 PHP) | Rate: 2.50 coins/sec
Progress: 2/3 coins (10/15 PHP) | Rate: 2.45 coins/sec
Progress: 3/3 coins (15/15 PHP) | Rate: 2.48 coins/sec
🔴 Turning OFF SSR relay...

📊 TEST RESULTS:
  Target Coins: 3
  Dispensed Coins: 3
  Target Amount: 15 PHP
  Dispensed Amount: 15 PHP
  ✅ TEST PASSED!
```

### 3. Test Auto-Dispense Logic

Simulate the full AUTO_DISPENSE flow without RPi.

```bash
# Test 5 peso dispense
test_auto 5

# Test 10 peso dispense
test_auto 10

# Test 20 peso dispense
test_auto 20

# Test 50 peso dispense (most complex - uses 2 hoppers)
test_auto 50
```

**Expected Result for `test_auto 50`:**
```
========================================
🧪 AUTO-DISPENSE TEST
Simulating detection of P50 chit
========================================

=== Dispensing Plan ===
5 PHP coins: 0 (0 PHP)
10 PHP coins: 1 (10 PHP)
20 PHP coins: 2 (40 PHP)
Total value: P50
======================

=== Starting Dispensing ===
Dispensing 2 x 20 PHP coins from Hopper 3
Enabling SSR for Hopper 3 (GPIO33)
[Coins dispensed...]
Disabling SSR for Hopper 3
Dispensed: 2/2 coins (40 PHP)

Dispensing 1 x 10 PHP coins from Hopper 2
Enabling SSR for Hopper 2 (GPIO25)
[Coins dispensed...]
Disabling SSR for Hopper 2
Dispensed: 1/1 coins (10 PHP)

=== Dispensing Complete ===

Transaction complete!
DISPENSING_COMPLETE:50
```

### 4. Test Serial Communication

Test the serial communication between RPi and ESP32.

#### From RPi Terminal

```bash
# Send AUTO_DISPENSE command via serial
echo "AUTO_DISPENSE:20" > /dev/ttyUSB0

# Monitor responses
cat /dev/ttyUSB0
```

#### From ESP32 Serial Monitor

Send commands directly:
```bash
AUTO_DISPENSE:50
```

**Expected Flow:**
1. ESP32 receives: `AUTO_DISPENSE:50`
2. ESP32 displays on LCD: Auto dispensing plan
3. ESP32 enables SSR for Hopper 3
4. ESP32 dispenses 2x 20 peso coins
5. ESP32 disables SSR for Hopper 3
6. ESP32 enables SSR for Hopper 2
7. ESP32 dispenses 1x 10 peso coin
8. ESP32 disables SSR for Hopper 2
9. ESP32 sends: `DISPENSING_COMPLETE:50`

### 5. Test Complete System Integration

Test the full system flow from YOLO detection to coin dispensing.

#### Setup
1. Connect ESP32 to RPi via USB (typically `/dev/ttyUSB0`)
2. Upload `CoinExchanger.ino` to ESP32
3. Start YOLO detection on RPi:

```bash
cd /home/admin/Chits-Exchanger/source/rpi/yolo/
python3 yolo_detect.py --model ../../ml/my_model.pt --esp32_port /dev/ttyUSB0 --camera 0
```

#### Test Procedure

1. **Insert Chit into Acceptor**
   - IR sensor detects chit presence
   - RPi displays: "IR DETECTED! Scanning chit..."
   - YOLO starts analyzing the chit

2. **YOLO Detection Phase**
   - Camera captures chit image
   - YOLO identifies denomination (5, 10, 20, or 50)
   - Confidence level calculated
   - Best detection displayed on screen

3. **Chit Release**
   - Servo releases chit after successful detection
   - RPi sends: `AUTO_DISPENSE:<value>`

4. **ESP32 Auto-Dispensing**
   - ESP32 receives command
   - Calculates coin combination
   - Displays plan on LCD
   - Enables SSR for required hoppers
   - Dispenses coins with pulse counting
   - Disables SSR after dispensing
   - Sends: `DISPENSING_COMPLETE:<value>`

5. **Completion**
   - RPi displays success message
   - System returns to idle state
   - Ready for next chit

#### Expected Console Output (RPi)

```
============================================================
🔍 IR SENSOR TRIGGERED - CHIT DETECTED
============================================================
   Starting YOLO detection...
   Timeout: 10 seconds
============================================================

💰 Detected: ₱50 chit | Conf: 95% | Time: 2s

============================================================
🎉 DETECTION COMPLETE
============================================================
   Detected Value: ₱50
   Confidence: 95%
   Detection Time: 2.34s
============================================================

🔓 Releasing chit via servo...
✅ Chit ₱50 released successfully

============================================================
🪙 AUTO-DISPENSING TRIGGERED
============================================================
   Sending to ESP32: AUTO_DISPENSE:50
   Expected dispensing:
     - 2 x 20 PHP coins (Hopper 3)
     - 1 x 10 PHP coin (Hopper 2)
============================================================

✅ Sent to ESP32: AUTO_DISPENSE:50

📨 ESP32: DISPENSING_COMPLETE:50

Waiting for next chit...
```

#### Expected Console Output (ESP32)

```
========================================
🎯 AUTO_DISPENSE received: P50
========================================

=== Auto Dispensing Plan ===
5 PHP coins: 0
10 PHP coins: 1
20 PHP coins: 2
Total value: P50
======================

🚀 Starting automatic coin dispensing...

=== Starting Dispensing ===

Dispensing 2 x 20 PHP coins from Hopper 3
Enabling SSR for Hopper 3 (GPIO33)
Dispensed: 2/2 coins (40 PHP)
Disabling SSR for Hopper 3

Dispensing 1 x 10 PHP coins from Hopper 2
Enabling SSR for Hopper 2 (GPIO25)
Dispensed: 1/1 coins (10 PHP)
Disabling SSR for Hopper 2

=== Dispensing Complete ===

Transaction complete!
DISPENSING_COMPLETE:50
```

## Troubleshooting

### Issue: Pulse Not Detected

**Symptoms:** `test_pulse` shows no pulses when coins are dropped

**Solutions:**
1. Check GPIO pin connections
2. Verify hopper pulse sensor wiring
3. Test with multimeter: pulse pin should go LOW when coin passes
4. Adjust debounce timing in code if needed

### Issue: SSR Not Activating

**Symptoms:** `test_relay on` doesn't turn on hopper motor

**Solutions:**
1. Check SSR power connections (3.3V, GND, GPIO)
2. Verify SSR LED indicator
3. Test SSR with multimeter
4. Check hopper motor power supply (12V/24V)

### Issue: Serial Communication Failure

**Symptoms:** ESP32 not receiving AUTO_DISPENSE commands

**Solutions:**
1. Check USB cable connection
2. Verify serial port: `ls /dev/ttyUSB*`
3. Check baud rate (must be 115200)
4. Grant permissions: `sudo usermod -a -G dialout $USER`
5. Close other serial monitors
6. Restart both ESP32 and RPi

### Issue: Incorrect Coin Count

**Symptoms:** Dispensed coins don't match requested amount

**Solutions:**
1. Clean coin hopper sensors
2. Verify coins are correct denomination
3. Check for coin jams
4. Adjust pulse detection sensitivity
5. Verify hopper motor speed (may be too fast/slow)

### Issue: System Busy Error

**Symptoms:** ESP32 shows "SYSTEM BUSY" message

**Solutions:**
1. Wait for current operation to complete
2. Check if hopper is jammed
3. Reset ESP32 if stuck
4. Check SSR state (should be OFF when idle)

## Test Commands Reference

### ESP32 Serial Commands

```bash
# Testing Commands
help                    # Show all available commands
test_pulse 1            # Test pulse detection on hopper 1
test_relay 1 on         # Turn on SSR for hopper 1
test_relay 1 off        # Turn off SSR for hopper 1
test_hopper 1 5         # Dispense 5 coins from hopper 1
test_auto 50            # Simulate auto-dispense for 50 peso
test_all                # Run comprehensive hardware test

# Simulation Commands
test_chit 50            # Simulate manual chit detection (with button UI)

# RPi Commands (sent via serial)
AUTO_DISPENSE:5         # Auto-dispense 5 peso
AUTO_DISPENSE:10        # Auto-dispense 10 peso
AUTO_DISPENSE:20        # Auto-dispense 20 peso
AUTO_DISPENSE:50        # Auto-dispense 50 peso
IR_DETECTED             # IR sensor triggered
DETECTION_TIMEOUT       # Detection timeout
```

## Performance Metrics

### Target Performance

- **Detection Time:** < 3 seconds
- **Dispensing Time per Coin:** ~0.4 seconds (2.5 coins/sec)
- **Total Transaction Time:**
  - 5 PHP: ~5 seconds
  - 10 PHP: ~5 seconds
  - 20 PHP: ~5 seconds
  - 50 PHP: ~7 seconds (3 coins)
  
### Accuracy Requirements

- **YOLO Detection Confidence:** > 50%
- **Pulse Detection Accuracy:** 100% (all coins counted)
- **SSR Reliability:** 100% (must turn on/off correctly)
- **Serial Communication:** 100% (no dropped messages)

## Maintenance

### Daily Checks
- [ ] Test all 3 hoppers with `test_all`
- [ ] Verify serial communication
- [ ] Check coin levels in hoppers
- [ ] Clean IR sensor and camera

### Weekly Checks
- [ ] Test complete system integration
- [ ] Verify pulse detection on all hoppers
- [ ] Check SSR relay operation
- [ ] Inspect wiring connections

### Monthly Checks
- [ ] Clean coin sensors
- [ ] Lubricate hopper mechanisms
- [ ] Test with all denominations (5, 10, 20, 50)
- [ ] Verify YOLO model accuracy

## Support

For issues or questions:
1. Check this testing guide
2. Review serial monitor output
3. Test individual components first
4. Check hardware connections
5. Verify power supply voltages

---

**Document Version:** 1.0  
**Last Updated:** October 26, 2025  
**System:** Chit Exchanger with YOLO Detection
