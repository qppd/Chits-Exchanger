# Servo Dispenser Pair Configuration

## Overview
The ChitExchanger now uses **8 servos (4 pairs)** for dispensing chits. Each denomination uses a pair of servos that work simultaneously for better dispensing performance.

## Channel Assignments

### PCA9685 PWM Driver Channels (0-7)

| Denomination | Channel 1 | Channel 2 | Purpose |
|-------------|-----------|-----------|---------|
| ₱50 Chits   | 0         | 1         | First pair (channels 0 & 1) |
| ₱20 Chits   | 2         | 3         | Second pair (channels 2 & 3) |
| ₱10 Chits   | 4         | 5         | Third pair (channels 4 & 5) |
| ₱5 Chits    | 6         | 7         | Fourth pair (channels 6 & 7) |

## New Constants (PIN_CONFIGURATION.h)

```cpp
// ₱50 chits - Channels 0 and 1
#define CHIT_50_CHANNEL_1 0
#define CHIT_50_CHANNEL_2 1

// ₱20 chits - Channels 2 and 3
#define CHIT_20_CHANNEL_1 2
#define CHIT_20_CHANNEL_2 3

// ₱10 chits - Channels 4 and 5
#define CHIT_10_CHANNEL_1 4
#define CHIT_10_CHANNEL_2 5

// ₱5 chits - Channels 6 and 7
#define CHIT_5_CHANNEL_1 6
#define CHIT_5_CHANNEL_2 7
```

## New Functions

### dispenseCardPair()
```cpp
void dispenseCardPair(int channel1, int channel2, int chitValue)
```
Dispenses a card using both servos in a pair simultaneously. Both servos start together, run for the specified duration, and stop together.

### testAllServoPairs()
```cpp
void testAllServoPairs()
```
Tests all 4 servo pairs sequentially:
1. ₱50 pair (channels 0 & 1)
2. ₱20 pair (channels 2 & 3)
3. ₱10 pair (channels 4 & 5)
4. ₱5 pair (channels 6 & 7)

Each pair is tested with its appropriate dispense duration, with 1-second delays between tests.

## Serial Commands

### TEST
Tests the ₱10 servo pair (channels 4 & 5) for 0.8 seconds.

**Usage:**
```
TEST
```

**Output:**
- Both servos on channels 4 and 5 run simultaneously CW for 0.8 seconds
- Serial output shows the test progress

### TESTALL
Tests all 4 servo pairs sequentially.

**Usage:**
```
TESTALL
```

**Output:**
- ₱50 pair runs for 800ms
- ₱20 pair runs for 700ms
- ₱10 pair runs for 600ms
- ₱5 pair runs for 500ms
- 1-second delay between each pair test
- Detailed serial output for each test

## How It Works

### Dispensing Process
1. When a denomination needs to be dispensed, `dispenseCardPair()` is called
2. Both servos in the pair start rotating forward (CW) simultaneously
3. They run for the predetermined duration based on the chit value
4. Both servos stop together
5. A 100ms delay occurs before the next operation

### Advantages of Servo Pairs
- **More Power**: Two servos provide more torque for dispensing
- **Better Reliability**: If one servo has issues, the other can still help dispense
- **Synchronized Operation**: Both servos work together for consistent dispensing
- **Redundancy**: Backup servo in case of failure

## Dispense Durations

| Chit Value | Duration | Constant |
|-----------|----------|----------|
| ₱5        | 500ms    | DISPENSE_DURATION_5 |
| ₱10       | 600ms    | DISPENSE_DURATION_10 |
| ₱20       | 700ms    | DISPENSE_DURATION_20 |
| ₱50       | 800ms    | DISPENSE_DURATION_50 |

## Servo Control Values

| Direction | PWM Value | Constant |
|-----------|-----------|----------|
| Stop      | 375       | SERVO_STOP |
| Forward (CW) | 450    | SERVO_FORWARD |
| Backward (CCW) | 300 | SERVO_BACKWARD |

## Hardware Setup

### PCA9685 Connections
- **I2C Address**: 0x40 (default)
- **SDA**: GPIO 21
- **SCL**: GPIO 22
- **PWM Frequency**: 50Hz (for servos)
- **Power**: External 5V supply for servos (recommended)

### Servo Wiring
Connect servos to PCA9685 PWM outputs:
- Channels 0-1: ₱50 dispenser pair
- Channels 2-3: ₱20 dispenser pair
- Channels 4-5: ₱10 dispenser pair
- Channels 6-7: ₱5 dispenser pair

## Testing Procedure

1. Upload the updated code to ESP32
2. Open Serial Monitor (9600 baud)
3. Wait for initialization messages
4. Type `TESTALL` to test all servo pairs
5. Observe each pair working:
   - Both servos in each pair should rotate together
   - Check for smooth rotation
   - Verify proper stopping
6. Use `TEST` to test individual ₱10 pair if needed

## Troubleshooting

### Servos Don't Move
- Check power supply to PCA9685
- Verify I2C connections (SDA/SCL)
- Check if PCA9685 is detected (I2C address 0x40)
- Ensure servo power is connected

### Only One Servo in Pair Works
- Check servo connections to PCA9685
- Verify servo power
- Check individual channel wiring
- Test individual channels using modified code

### Erratic Movement
- Check power supply voltage (should be stable 5V)
- Verify PWM values are correct
- Check for loose connections
- Ensure proper grounding

## Future Expansion
The PCA9685 has 16 channels total (0-15). Currently using 8 channels (0-7), leaving 8 channels (8-15) available for future expansion or additional features.

## Code Changes Summary

### Modified Files
1. **PIN_CONFIGURATION.h** - Added 8 servo channel definitions
2. **SERVO_DISPENSER.h** - Added `dispenseCardPair()` and `testAllServoPairs()` declarations
3. **SERVO_DISPENSER.cpp** - Implemented pair dispensing and testing functions
4. **ChitExchanger.ino** - Updated dispensing to use servo pairs, added TESTALL command

### Backward Compatibility
Legacy channel definitions are maintained for compatibility:
- `CHIT_5_CHANNEL` → `CHIT_5_CHANNEL_1`
- `CHIT_10_CHANNEL` → `CHIT_10_CHANNEL_1`
- `CHIT_20_CHANNEL` → `CHIT_20_CHANNEL_1`
- `CHIT_50_CHANNEL` → `CHIT_50_CHANNEL_1`
