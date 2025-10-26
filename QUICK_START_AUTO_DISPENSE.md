# 🚀 Quick Start - Auto Dispense Coin System

## ✅ Current Status
- **ESP32 Communication:** WORKING! ✅
- **Firmware:** CoinExchanger.ino (115200 baud)
- **Port:** /dev/ttyUSB0
- **Hardware:** 3 Coin Hoppers (5₱, 10₱, 20₱)

## 📝 What You Need To Do

### Step 1: Upload Modified ESP32 Code
```bash
# Open Arduino IDE
# File: /home/admin/Chits-Exchanger/source/esp32/CoinExchanger/CoinExchanger.ino
# Upload to your ESP32 board
```

### Step 2: Test the System
```bash
cd /home/admin/Chits-Exchanger/source/rpi/yolo

# Run YOLO detection with auto-dispense
python3 yolo_detect.py \
  --model chit_model_ncnn_model \
  --camera 0 \
  --resolution 640x480 \
  --esp32_port /dev/ttyUSB0
```

### Step 3: Insert a Chit and Watch!
1. Insert a chit (5, 10, 20, or 50 peso)
2. System detects via IR sensor
3. YOLO identifies the denomination
4. Servo releases the chit
5. **AUTO_DISPENSE command sent to ESP32**
6. **ESP32 calculates optimal coin combination**
7. **Coin hoppers dispense coins!** 🎉

## 🔧 How It Works

### Communication Flow:
```
RPi Camera → YOLO Detection → Detected: P50
    ↓
IR Sensor Triggered
    ↓
Servo Releases Chit
    ↓
RPi sends: AUTO_DISPENSE:50
    ↓
ESP32 receives command
    ↓
ESP32 calculates: 2x20₱ + 1x10₱ coins
    ↓
Hopper 3 dispenses 2x 20₱ coins
Hopper 2 dispenses 1x 10₱ coins
    ↓
COMPLETE! ✅
```

### Serial Commands:
- `IR_DETECTED` - IR sensor triggered
- `CHIT_DETECTED:50` - YOLO detected 50 peso chit
- `CHIT_RELEASED:50` - Servo released the chit
- **`AUTO_DISPENSE:50`** - **Trigger automatic coin dispensing** 🎯

## 🎮 Coin Hopper Setup

### Hopper Configuration:
| Hopper | GPIO (Pulse) | GPIO (SSR) | Coin Value | Function |
|--------|-------------|-----------|------------|----------|
| 1      | 19          | 26        | 5₱         | Dispense 5 peso coins |
| 2      | 18          | 25        | 10₱        | Dispense 10 peso coins |
| 3      | 4           | 33        | 20₱        | Dispense 20 peso coins |

### How Dispensing Works:
1. **ESP32 receives AUTO_DISPENSE:50**
2. **Calculates optimal combination:**
   - 50₱ = 2x 20₱ + 1x 10₱
3. **Activates Hopper 3 (20₱):**
   - Turns on SSR (GPIO 33)
   - Counts pulses on GPIO 4
   - Stops after 2 coins detected
4. **Activates Hopper 2 (10₱):**
   - Turns on SSR (GPIO 25)
   - Counts pulses on GPIO 18
   - Stops after 1 coin detected
5. **Done!** Updates LCD and returns to idle

## 📊 Example Dispensing Plans

### For 50₱ Chit:
```
Option 1: 2x 20₱ + 1x 10₱
Option 2: 5x 10₱
Option 3: 10x 5₱
```

### For 20₱ Chit:
```
Option 1: 1x 20₱
Option 2: 2x 10₱
Option 3: 4x 5₱
```

### For 10₱ Chit:
```
Option 1: 1x 10₱
Option 2: 2x 5₱
```

### For 5₱ Chit:
```
Option: 1x 5₱
```

## 🧪 Testing Commands

### Test Individual Hoppers:
```bash
# Open serial monitor (115200 baud) and send:
test_pulse 1        # Test Hopper 1 (5₱) pulse detection
test_pulse 2        # Test Hopper 2 (10₱) pulse detection  
test_pulse 3        # Test Hopper 3 (20₱) pulse detection

test_relay 1 on     # Turn on Hopper 1 relay
test_relay 1 off    # Turn off Hopper 1 relay

test_all           # Test all components

help               # Show all commands
```

### Test Auto Dispense:
```bash
# Send via serial monitor:
AUTO_DISPENSE:50   # Dispense coins for 50 peso chit
AUTO_DISPENSE:20   # Dispense coins for 20 peso chit
AUTO_DISPENSE:10   # Dispense coins for 10 peso chit
AUTO_DISPENSE:5    # Dispense coins for 5 peso chit
```

## 📺 What You'll See on LCD

### During Detection:
```
AUTO DISPENSE!
Chit: P50
Calculating...
```

### Dispensing Plan:
```
Dispensing Plan:
5P:0 10P:1
20P:2
Total: P50
```

### During Dispensing:
```
Dispensing...
Dispensing 20 PHP
Count: 2/2
```

### Complete:
```
Dispensing Complete!
Total: P50
Status: OK
```

## ⚠️ Troubleshooting

### ESP32 Not Responding:
```bash
# Check connection
ls -l /dev/ttyUSB*

# Check if port is in use
sudo lsof /dev/ttyUSB0

# Try reconnecting
sudo systemctl restart pigpiod
```

### Coins Not Dispensing:
1. Check hopper power supply
2. Verify SSR connections
3. Test pulse detection: `test_pulse 1/2/3`
4. Check if hoppers have coins!

### Pulse Count Wrong:
1. Hopper might be jammed
2. Pulse sensor might be dirty
3. Adjust pulse detection timing
4. Check coin sensor alignment

## 📖 Serial Monitor Output Example

```
========================================
🎯 AUTO_DISPENSE received: P50
========================================
=== Auto Dispensing Plan ===
5 PHP coins: 0
10 PHP coins: 1
20 PHP coins: 2
Total value: P50
🚀 Starting automatic coin dispensing...

=== Starting Dispensing ===
Dispensing 2 x 20 PHP coins...
✅ Pulse detected! Coin #1
✅ Pulse detected! Coin #2
✅ Successfully dispensed 2 x 20 PHP coins

Dispensing 1 x 10 PHP coins...
✅ Pulse detected! Coin #1
✅ Successfully dispensed 1 x 10 PHP coins

=== Dispensing Complete ===
Total dispensed: P50
Status: SUCCESS
```

## 🎯 Success Indicators

✅ ESP32 prints: "🎯 AUTO_DISPENSE received: P50"
✅ LCD shows: "AUTO DISPENSE! Chit: P50"
✅ Coin hoppers activate (you hear relays clicking)
✅ Pulses are counted (ESP32 serial shows coin counts)
✅ LCD shows: "Dispensing Complete!"
✅ System returns to idle, ready for next chit

## 🔄 Next Transaction

After successful dispensing:
1. System automatically resets
2. LCD shows: "Ready" / "Waiting for chit..."
3. Insert next chit to repeat process
4. Fully automatic - no button needed! 🎉

---

**Ready to test? Upload the ESP32 code and run the YOLO detector!** 🚀
