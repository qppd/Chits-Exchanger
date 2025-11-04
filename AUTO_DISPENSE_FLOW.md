# Auto-Dispense System Flow

Quick reference guide for the integrated chit detection and auto-dispensing system.

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    YOLO CHIT DETECTION (RPi)                    │
│                      yolo_detect.py                             │
└───────────────────────────┬─────────────────────────────────────┘
                            │ Serial USB
                            │ AUTO_DISPENSE:<value>
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                 COIN DISPENSER (ESP32)                          │
│                  CoinExchanger.ino                              │
└─────────────────────────────────────────────────────────────────┘
                            │
        ┌───────────────────┼───────────────────┐
        ▼                   ▼                   ▼
   ┌─────────┐         ┌─────────┐         ┌─────────┐
   │Hopper 1 │         │Hopper 2 │         │Hopper 3 │
   │ 5 PHP   │         │ 10 PHP  │         │ 20 PHP  │
   │GPIO4/26 │         │GPIO18/25│         │GPIO19/33│
   └─────────┘         └─────────┘         └─────────┘
```

## Complete Transaction Flow

### Step 1: Chit Insertion
```
User inserts chit into acceptor
         ↓
IR Sensor detects presence (GPIO17)
         ↓
RPi: IR_DETECTED state activated
         ↓
Send to ESP32: "IR_DETECTED"
         ↓
LCD: "IR DETECTED! Scanning chit..."
```

### Step 2: YOLO Detection
```
Camera captures chit image
         ↓
YOLO model analyzes image
         ↓
Detects denomination (5, 10, 20, or 50)
         ↓
Calculates confidence level
         ↓
LCD: "DETECTING... Chit: PXX Conf: XX%"
```

### Step 3: Chit Release
```
Detection complete (confidence > 50%)
         ↓
Servo moves from 39° to 90° (GPIO22)
         ↓
Chit released
         ↓
Servo returns to 39°
         ↓
LCD: "DETECTED! Value: PXX Releasing chit..."
```

### Step 4: Auto-Dispense Trigger
```
RPi sends: "AUTO_DISPENSE:<value>"
         ↓
ESP32 receives command
         ↓
Validates chit value (5, 10, 20, 50)
         ↓
Checks system state (must be IDLE)
         ↓
LCD: "AUTO DISPENSE! Chit: PXX Calculating..."
```

### Step 5: Coin Calculation
```
Calculate optimal coin combination:
  5 PHP  → 1x 5₱ coin  (Hopper 1)
  10 PHP → 1x 10₱ coin (Hopper 2)
  20 PHP → 1x 20₱ coin (Hopper 3)
  50 PHP → 2x 20₱ + 1x 10₱ (Hoppers 3+2)
         ↓
Display plan on LCD
         ↓
State: DISPENSING
```

### Step 6: Coin Dispensing

#### For 5 PHP Chit:
```
Enable SSR for Hopper 1 (GPIO26 HIGH)
         ↓
Hopper 1 motor starts
         ↓
Dispense 1x 5₱ coin
         ↓
Pulse detected on GPIO4
         ↓
Count: 1/1 coins
         ↓
Disable SSR for Hopper 1 (GPIO26 LOW)
         ↓
Total dispensed: 5 PHP
```

#### For 10 PHP Chit:
```
Enable SSR for Hopper 2 (GPIO25 HIGH)
         ↓
Hopper 2 motor starts
         ↓
Dispense 1x 10₱ coin
         ↓
Pulse detected on GPIO18
         ↓
Count: 1/1 coins
         ↓
Disable SSR for Hopper 2 (GPIO25 LOW)
         ↓
Total dispensed: 10 PHP
```

#### For 20 PHP Chit:
```
Enable SSR for Hopper 3 (GPIO33 HIGH)
         ↓
Hopper 3 motor starts
         ↓
Dispense 1x 20₱ coin
         ↓
Pulse detected on GPIO19
         ↓
Count: 1/1 coins
         ↓
Disable SSR for Hopper 3 (GPIO33 LOW)
         ↓
Total dispensed: 20 PHP
```

#### For 50 PHP Chit:
```
Step 1: Dispense 20₱ coins
  Enable SSR for Hopper 3 (GPIO33 HIGH)
         ↓
  Dispense 2x 20₱ coins
         ↓
  Pulses detected on GPIO19 (count: 2)
         ↓
  Disable SSR for Hopper 3 (GPIO33 LOW)
         ↓
  Subtotal: 40 PHP

Step 2: Dispense 10₱ coin
  Enable SSR for Hopper 2 (GPIO25 HIGH)
         ↓
  Dispense 1x 10₱ coin
         ↓
  Pulse detected on GPIO18 (count: 1)
         ↓
  Disable SSR for Hopper 2 (GPIO25 LOW)
         ↓
  Subtotal: 10 PHP

Total dispensed: 50 PHP
```

### Step 7: Completion
```
All coins dispensed successfully
         ↓
State: COMPLETE
         ↓
Send to RPi: "DISPENSING_COMPLETE:XX"
         ↓
LCD: "Complete! Dispensed: PXX Thank you!"
         ↓
Wait 5 seconds
         ↓
State: IDLE
         ↓
LCD: "Ready! Waiting for chit..."
```

## Timing Diagram

```
Time    RPi                    ESP32                  Hoppers
----    ---                    -----                  -------
0.0s    IR detected
0.1s    IR_DETECTED → → → → → Received
0.5s    YOLO analyzing...
2.0s    Chit detected (50₱)
2.5s    Servo release
3.0s    AUTO_DISPENSE:50 → → → Received
3.5s                           Calculate plan
4.0s                           Enable Hopper 3 SSR → → Motor ON
4.2s                           Pulse 1 detected
4.6s                           Pulse 2 detected
4.8s                           Disable Hopper 3 SSR → Motor OFF
5.0s                           Enable Hopper 2 SSR → → Motor ON
5.4s                           Pulse 1 detected
5.6s                           Disable Hopper 2 SSR → Motor OFF
5.8s                           DISPENSING_COMPLETE:50
6.0s    ← ← ← ← ← ← ← ←       Completion received
6.5s    Display "Complete!"
11.5s   Ready for next
```

## Error Handling

### Detection Timeout
```
IR detected → YOLO scanning → 10 seconds elapsed
         ↓
No valid detection
         ↓
Send to ESP32: "DETECTION_TIMEOUT"
         ↓
LCD: "TIMEOUT! No valid chit detected"
         ↓
Return to IDLE
```

### System Busy
```
AUTO_DISPENSE received
         ↓
Check current state
         ↓
State = DISPENSING (busy)
         ↓
Serial: "⚠️ SYSTEM BUSY - Ignoring AUTO_DISPENSE"
         ↓
Continue current operation
         ↓
Command ignored
```

### Serial Communication Failure
```
RPi: send_to_esp32() → Error
         ↓
LCD: "ERROR! Communication failed"
         ↓
Retry or manual intervention required
```

### Coin Jam / Incomplete Dispensing
```
Dispensing in progress
         ↓
Timeout (30 seconds) reached
         ↓
Actual count < Target count
         ↓
Serial: "WARNING: Dispensing incomplete!"
         ↓
Disable SSR
         ↓
State: ERROR
         ↓
LCD: "ERROR! Please contact staff"
```

## GPIO Pin Reference

### ESP32 Connections

| Component | Function | GPIO Pin | Direction | Notes |
|-----------|----------|----------|-----------|-------|
| Hopper 1 | Pulse | 4 | Input | 5₱ coin detection |
| Hopper 1 | SSR | 26 | Output | 5₱ motor control |
| Hopper 2 | Pulse | 18 | Input | 10₱ coin detection |
| Hopper 2 | SSR | 25 | Output | 10₱ motor control |
| Hopper 3 | Pulse | 19 | Input | 20₱ coin detection |
| Hopper 3 | SSR | 33 | Output | 20₱ motor control |
| LCD | SDA | 21 | I2C | Display data |
| LCD | SCL | 22 | I2C | Display clock |
| Button | Input | 27 | Input | Manual controls |

### RPi Connections

| Component | Function | GPIO Pin | Direction | Notes |
|-----------|----------|----------|-----------|-------|
| IR Sensor | Detection | 17 | Input | Chit presence |
| Servo | Release | 22 | PWM Output | Chit release mechanism |
| Camera | USB | - | USB | YOLO detection |
| ESP32 | Serial | - | /dev/ttyUSB0 | Commands/responses |

## Serial Protocol

### Message Format
```
Command format: COMMAND:VALUE\n
Response format: RESPONSE:VALUE\n
Baud rate: 115200
Encoding: UTF-8
Line ending: \n (newline)
```

### Valid Commands (RPi → ESP32)

| Command | Format | Example | Description |
|---------|--------|---------|-------------|
| Auto Dispense | `AUTO_DISPENSE:value` | `AUTO_DISPENSE:50` | Trigger auto dispensing |
| IR Detected | `IR_DETECTED` | `IR_DETECTED` | IR sensor activated |
| Timeout | `DETECTION_TIMEOUT` | `DETECTION_TIMEOUT` | Detection timeout |

### Valid Responses (ESP32 → RPi)

| Response | Format | Example | Description |
|----------|--------|---------|-------------|
| Complete | `DISPENSING_COMPLETE:value` | `DISPENSING_COMPLETE:50` | Dispensing finished |
| Error | `ERROR:message` | `ERROR:Hopper jam` | Error occurred |

## Hopper Mapping Quick Reference

```
┌──────────┬─────────┬─────────┬─────────┬────────┐
│ Chit     │ Hopper 1│ Hopper 2│ Hopper 3│ Total  │
│ Value    │ (5₱)    │ (10₱)   │ (20₱)   │ Output │
├──────────┼─────────┼─────────┼─────────┼────────┤
│ 5 PHP    │ 1 coin  │ -       │ -       │ 5₱     │
│ 10 PHP   │ -       │ 1 coin  │ -       │ 10₱    │
│ 20 PHP   │ -       │ -       │ 1 coin  │ 20₱    │
│ 50 PHP   │ -       │ 1 coin  │ 2 coins │ 50₱    │
└──────────┴─────────┴─────────┴─────────┴────────┘
```

## Quick Test Commands

```bash
# Test individual hopper
test_hopper 1 3          # Dispense 3x 5₱ coins

# Test auto-dispense
test_auto 50             # Simulate 50 peso chit

# Test pulse detection
test_pulse 1             # Monitor Hopper 1 pulses

# Test SSR control
test_relay 1 on          # Turn ON Hopper 1
test_relay 1 off         # Turn OFF Hopper 1

# Show help
help                     # Display all commands
```

---

**Version:** 1.0  
**Date:** October 26, 2025  
**System:** Chit Exchanger Auto-Dispense
