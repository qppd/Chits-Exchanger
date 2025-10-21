# Servo System Quick Reference Guide

## 🎯 Quick Start

### Power On Checklist
1. ✅ External 5V power supply connected to PCA9685
2. ✅ All grounds connected (ESP32, PCA9685, Power Supply)
3. ✅ I2C connections verified (SDA=21, SCL=22)
4. ✅ 8 servos connected to channels 0-7

### Test Commands
```
Serial Monitor @ 9600 baud:
TEST     → Test ₱10 pair (0.8s)
TESTALL  → Test all pairs
```

## 📊 Channel Mapping Quick Reference

| Denomination | Pair | Ch1 | Ch2 | Duration | Color Code |
|--------------|------|-----|-----|----------|------------|
| ₱50          | 1    | 0   | 1   | 800ms    | 🔴 Red     |
| ₱20          | 2    | 2   | 3   | 700ms    | 🟢 Green   |
| ₱10          | 3    | 4   | 5   | 600ms    | 🔵 Blue    |
| ₱5           | 4    | 6   | 7   | 500ms    | 🟡 Yellow  |

## 🔌 Wiring Quick Reference

### PCA9685 to ESP32
```
VCC  → 3.3V  (logic)
GND  → GND
SDA  → GPIO 21
SCL  → GPIO 22
V+   → 5V External PSU (+)
GND  → 5V External PSU (-)
```

### Servo Connections (Standard 3-pin)
```
Signal (Yellow/White) → PCA9685 Channel Pin
Power  (Red)          → PCA9685 V+ Rail
Ground (Black/Brown)  → PCA9685 GND Rail
```

## ⚡ Power Requirements

| Component | Voltage | Current | Notes |
|-----------|---------|---------|-------|
| ESP32 Logic | 3.3V | ~200mA | From USB/5V regulator |
| PCA9685 Logic | 3.3V | ~10mA | From ESP32 |
| 8 Servos | 5V | 4A+ | **Must use external PSU** |

**Total System**: 5V @ 4.5A minimum

## 🔧 Troubleshooting Quick Fixes

### Servos Don't Move
1. Check external 5V power supply
2. Verify I2C: Run I2C scanner (address should show 0x40)
3. Check serial output for initialization message

### Only One Servo Per Pair Works
1. Check servo wiring to correct channel
2. Verify servo power connection
3. Test individual servo with TEST command

### Weak Dispensing
1. Increase dispense duration in code
2. Check power supply voltage (should be 5V ± 0.2V)
3. Verify servo specifications (should handle 5V)

### Erratic Movement
1. Ensure PWM=0 when servos idle
2. Check for power supply noise/ripple
3. Add capacitor (1000µF) across PSU output

## 💻 Code Constants

### PWM Values
```cpp
SERVO_FORWARD  = 450  // Clockwise rotation
SERVO_BACKWARD = 300  // Counter-clockwise
SERVO_STOP     = 375  // Neutral (1.5ms pulse)
// Deactivated = 0     // No PWM signal
```

### Dispense Durations
```cpp
DISPENSE_DURATION_5  = 500  // ₱5 chits
DISPENSE_DURATION_10 = 600  // ₱10 chits
DISPENSE_DURATION_20 = 700  // ₱20 chits
DISPENSE_DURATION_50 = 800  // ₱50 chits
```

### I2C Addresses
```cpp
PCA9685_ADDR = 0x40  // PWM Driver
LCD_ADDR     = 0x27  // 20x4 LCD Display
```

## 🧪 Testing Procedures

### 1. Initial Setup Test
```
Upload code → Open Serial Monitor (9600)
Look for: "PCA9685 Initialized - 360° Servo Mode"
          "Servo channels 0-7 ready for card dispensing"
```

### 2. Single Pair Test
```
Type: TEST
Expected: Channels 4 & 5 rotate CW for 0.8s
```

### 3. Full System Test
```
Type: TESTALL
Expected: All 4 pairs test sequentially
  - Pair 1 (₱50): 800ms
  - Pair 2 (₱20): 700ms
  - Pair 3 (₱10): 600ms
  - Pair 4 (₱5):  500ms
```

### 4. Dispensing Test
```
Insert bill/coin → System calculates chits
Expected: Appropriate pairs dispense in sequence
Watch: Both servos in each pair should rotate together
```

## 📏 Mechanical Setup

### Servo Mounting
- Secure each servo with 2-4 screws
- Ensure servo horn is properly attached
- Allow 2-3mm clearance for chit movement
- Test manually before power-on

### Chit Storage
- Stack chits vertically in dispensers
- Keep stacks under 50 chits per denomination
- Ensure chits slide freely (no binding)
- Check for debris/obstructions

### Alignment
- Servo horns should be parallel to chit path
- Test with single chit before full stack
- Adjust timing if chits don't fully dispense
- Verify both servos in pair push evenly

## 🔍 Diagnostic Serial Output

### Normal Operation
```
=== DISPENSING PLAN ===
Total Amount: ₱100
₱50 chits: 2
Dispensing ₱50 chit using channels 0 & 1 for 800ms
Both servos on channels 0 & 1 stopped
```

### Test Mode
```
=== TESTING ALL SERVO PAIRS ===
--- Testing ₱50 Pair (Channels 0 & 1) ---
Dispensing ₱50 chit using channels 0 & 1 for 800ms
```

### Error Indicators
```
ERROR: Invalid dispensing amount
No I2C devices found!  (Check wiring)
Invalid chit value!    (Software issue)
```

## 📱 Support Resources

- Full Documentation: `SERVO_PAIR_CONFIGURATION.md`
- Code Reference: `source/esp32/ChitExchanger/`
- Hardware Guide: `README.md` (Hardware Setup section)
- Issues: [GitHub Issues](https://github.com/qppd/Chits-Exchanger/issues)

## ⚙️ Configuration Options

### Adjust Dispense Duration
Edit in `SERVO_DISPENSER.h`:
```cpp
const int DISPENSE_DURATION_5 = 500;   // Increase/decrease ms
const int DISPENSE_DURATION_10 = 600;
const int DISPENSE_DURATION_20 = 700;
const int DISPENSE_DURATION_50 = 800;
```

### Change Servo Speed
Edit in `SERVO_DISPENSER.h`:
```cpp
const int SERVO_FORWARD = 450;   // Lower = slower, Higher = faster
const int SERVO_BACKWARD = 300;  // (Range: 250-550)
```

### Test Duration Modification
Edit in `SERVO_DISPENSER.cpp` → `testAdditionalServos()`:
```cpp
delay(800);  // Change test duration (milliseconds)
```

## 🎓 Best Practices

✅ **DO**:
- Always use external power for servos
- Test individual pairs before full operation
- Keep servo wires short (<30cm from PCA9685)
- Use proper gauge wire (22-24 AWG for signal, 18-20 AWG for power)
- Document any modifications

❌ **DON'T**:
- Power servos from ESP32 (will damage board)
- Exceed 6V on servo power rail
- Run servos continuously without breaks
- Mix different servo types in same system
- Skip proper ground connections

---

**Last Updated**: October 21, 2025  
**Version**: 2.0.0  
**Compatibility**: ESP32 ChitExchanger Firmware v2.0+
