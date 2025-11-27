# 🚀 Quick Start Guide - Chits Exchanger System

## ✅ System Status: READY FOR DEPLOYMENT

Both ESP32 master and RPi slave are fully integrated and ready to test!

---

## 📋 Pre-Deployment Checklist

### Hardware Setup
- [ ] ESP32 connected to 3 coin hoppers (GPIO 4, 18, 19)
- [ ] ESP32 LCD connected (I2C 0x27)
- [ ] RPi connected to ESP32 via serial (TX→RX, RX→TX, GND→GND)
- [ ] RPi IR sensor connected (GPIO17)
- [ ] RPi servo motor connected (GPIO22)
- [ ] RPi LCD connected (I2C 0x27)
- [ ] RPi camera connected and working
- [ ] All power supplies connected

### Software Setup
- [ ] Arduino IDE installed on PC
- [ ] ESP32 board support installed in Arduino IDE
- [ ] Python 3 installed on RPi
- [ ] Required Python packages installed (see below)
- [ ] YOLO model file available on RPi
- [ ] pigpio daemon enabled and running on RPi

---

## 🔧 RPi Setup (One-Time)

### 1. Install Dependencies
```bash
# System update
sudo apt-get update
sudo apt-get upgrade

# Install pigpio for servo control
sudo apt-get install pigpio python3-pigpio
sudo systemctl enable pigpiod
sudo systemctl start pigpiod

# Install Python packages
pip3 install pyserial
pip3 install RPi.GPIO
pip3 install smbus2
pip3 install opencv-python
pip3 install ultralytics
```

### 2. Enable Serial Port
```bash
sudo raspi-config
# Navigate to: Interface Options → Serial Port
# Disable login shell over serial: NO
# Enable serial port hardware: YES
# Reboot: YES
```

### 3. Verify Hardware
```bash
# Check serial port
ls -l /dev/serial0

# Check I2C LCD
sudo i2cdetect -y 1
# Should show 0x27

# Check camera
python3 -c "import cv2; cap=cv2.VideoCapture(0); print('Camera OK' if cap.isOpened() else 'Camera FAIL'); cap.release()"
```

### 4. Configure Model Path
Edit `esp32_comm.py` line 67:
```python
MODEL_PATH = '/home/pi/Desktop/PROJECTS/Chits-Exchanger/source/rpi/yolo/chit_model.pt'
```
Update to your actual YOLO model location.

---

## 🎯 ESP32 Setup (One-Time)

### 1. Open Arduino IDE
```
File → Open → CoinExchanger.ino
```

### 2. Select ESP32 Board
```
Tools → Board → ESP32 Dev Module (or your specific ESP32 board)
Tools → Port → COM# (select your ESP32 port)
```

### 3. Verify Compilation
```
Sketch → Verify/Compile
```
Should compile without errors.

### 4. Upload to ESP32
```
Sketch → Upload
```

---

## 🚦 Starting the System

### Step 1: Start RPi Slave First
```bash
cd /home/pi/Desktop/PROJECTS/Chits-Exchanger/source/rpi/yolo
python3 esp32_comm.py
```

**Expected Output:**
```
==================================================
ESP32 Chit Slave Controller Initializing...
==================================================
[LCD] ✓ Initialized
[IR] ✓ Initialized
[SERVO] ✓ Initialized
[YOLO] ✓ Model loaded
[CAMERA] ✓ Opened
[SERIAL] ✓ Connected on /dev/serial0 @ 115200 baud

==================================================
✓ ALL SYSTEMS INITIALIZED
==================================================

[TX → ESP32] SLAVE_READY
[SYSTEM] Slave controller running. Listening for ESP32 commands...
```

### Step 2: Power On/Reset ESP32

**Expected Output on ESP32 Serial Monitor:**
```
=== ESP32 Coin Exchanger System ===
Integrated with RPi Chit Detection

Initializing 3 ALLAN Coin Hoppers...
✅ All 3 coin hoppers initialized!

⏳ Waiting for RPi slave to initialize...
✅ RPi slave is ready!

=== SYSTEM READY ===
```

### Step 3: System is Ready!

Both LCDs should show:
- **ESP32 LCD**: "Ready! Waiting for chit..."
- **RPi LCD**: "Slave System Ready / Waiting for ESP32 commands..."

---

## 💡 Testing the System

### Test 1: Complete Transaction
1. **Insert chit** into the system
2. **RPi detects** via IR sensor (automatic every 2 seconds)
3. **RPi identifies** chit using YOLO (~5 seconds)
4. **ESP32 calculates** coin combination
5. **ESP32 dispenses** coins via hoppers
6. **RPi releases** chit via servo
7. **System resets** for next transaction

### Test 2: Manual Serial Commands

**On ESP32 Serial Monitor, test:**
```
test_auto 50     // Simulate 50 peso chit detection
```

**Watch both systems:**
- ESP32: Calculates and dispenses coins
- RPi: Shows status on LCD, releases chit

---

## 📊 System Operation Flow

```
╔══════════════════════════════════════════════════════╗
║                  TRANSACTION FLOW                    ║
╠══════════════════════════════════════════════════════╣
║                                                      ║
║  1. User inserts chit                               ║
║  2. ESP32 → RPi: "CHECK_IR"                         ║
║  3. RPi → ESP32: "IR_DETECTED"                      ║
║  4. ESP32 → RPi: "DETECT_CHIT"                      ║
║  5. RPi runs YOLO (~5 sec)                          ║
║  6. RPi → ESP32: "CHIT_50"                          ║
║  7. ESP32 calculates: 2×20P + 1×10P                 ║
║  8. ESP32 updates RPi LCD                           ║
║  9. ESP32 dispenses coins                           ║
║ 10. ESP32 → RPi: "DISPENSE_CHIT"                    ║
║ 11. RPi servo releases chit                         ║
║ 12. ESP32 → RPi: "RESET"                            ║
║ 13. System ready for next transaction               ║
║                                                      ║
╚══════════════════════════════════════════════════════╝
```

---

## 🔍 Monitoring & Debugging

### View RPi Logs
```bash
# Real-time logs
python3 esp32_comm.py

# Example output during transaction:
[RX ← ESP32] CHECK_IR
[TX → ESP32] IR_DETECTED
[RX ← ESP32] DETECT_CHIT
[YOLO] Starting chit detection...
[YOLO] Detected '50' with confidence 0.87
[TX → ESP32] CHIT_50
[RX ← ESP32] DISPLAY:Dispensing Coins|5P:0 10P:1|20P:2 Tot:50|Please wait...
[TX → ESP32] DISPLAY_OK
[RX ← ESP32] DISPENSE_CHIT
[SERVO] Moving to 90° (release)
[SERVO] Returning to 39° (initial)
[TX → ESP32] DISPENSE_COMPLETE
```

### View ESP32 Logs
Open Arduino IDE Serial Monitor (115200 baud):
```
=== Starting Chit Detection ===
[1/3] Checking IR sensor...
✓ IR sensor detected chit
[2/3] Updating LCD...
[3/3] Running YOLO detection (~5 seconds)...
✓ Detected chit value: P50
=== Detection Complete ===

Calculating coin combination...
=== Dispensing Plan ===
5 PHP coins: 0
10 PHP coins: 1
20 PHP coins: 2
Total value: P50
======================

🚀 Starting automatic coin dispensing...
```

---

## ⚠️ Troubleshooting

### Problem: "RPi slave not responding"

**Check:**
1. Is `esp32_comm.py` running on RPi?
2. Serial wiring correct? (TX→RX, RX→TX, common GND)
3. Baud rate: 115200 on both sides
4. Serial port enabled: `ls -l /dev/serial0`

**Fix:**
```bash
# Restart RPi serial
sudo systemctl restart serial-getty@serial0.service

# Or reboot RPi
sudo reboot
```

### Problem: "YOLO detection timeout"

**Check:**
1. Camera working: `python3 -c "import cv2; cap=cv2.VideoCapture(0); print(cap.isOpened())"`
2. Model file exists: `ls -l /path/to/chit_model.pt`
3. Good lighting on chit

**Fix:**
- Increase timeout in ESP32 (line ~334): change `10000` to `15000`
- Improve lighting
- Use higher confidence model

### Problem: "Servo not moving"

**Check:**
1. pigpio daemon: `sudo systemctl status pigpiod`
2. Servo wiring: Signal on GPIO22, GND, 5V power
3. Test servo: `python3 /path/to/servo_tester.py`

**Fix:**
```bash
sudo systemctl restart pigpiod
```

### Problem: "LCD not updating"

**Check:**
1. I2C address: `sudo i2cdetect -y 1` (should show 0x27)
2. I2C enabled: `sudo raspi-config` → Interface Options → I2C
3. Test LCD: `python3 /path/to/lcd_tester.py`

---

## 🔄 Auto-Start on Boot (Optional)

### Create systemd service for RPi slave

```bash
sudo nano /etc/systemd/system/chit-slave.service
```

**Content:**
```ini
[Unit]
Description=Chit Exchanger Slave System
After=network.target pigpiod.service

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/Desktop/PROJECTS/Chits-Exchanger/source/rpi/yolo
ExecStart=/usr/bin/python3 esp32_comm.py
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

**Enable:**
```bash
sudo systemctl daemon-reload
sudo systemctl enable chit-slave.service
sudo systemctl start chit-slave.service

# Check status
sudo systemctl status chit-slave.service

# View logs
journalctl -u chit-slave.service -f
```

---

## 📁 Important Files

### ESP32 (Master)
- `source/esp32/CoinExchanger/CoinExchanger.ino` - Main controller

### Raspberry Pi (Slave)
- `source/rpi/yolo/esp32_comm.py` - Main slave system
- `source/rpi/yolo/chit_model.pt` - YOLO model (you provide this)

### Documentation
- `ESP32_RPI_INTEGRATION_COMPLETE.md` - Integration summary
- `source/rpi/yolo/ESP32_SLAVE_README.md` - Detailed setup guide
- `source/rpi/yolo/INTEGRATION_GUIDE.md` - Complete technical guide

### Testing
- `source/rpi/yolo/test_esp32_slave.py` - Test utility
- `source/rpi/test/lcd_tester.py` - Test LCD
- `source/rpi/test/ir_sensor_tester.py` - Test IR sensor
- `source/rpi/test/servo_tester.py` - Test servo

---

## 🎉 Success Indicators

✅ **System Working Correctly When:**
1. ESP32 says "RPi slave is ready!"
2. Both LCDs show "Ready" message
3. Inserting chit triggers detection automatically
4. YOLO correctly identifies chit denomination
5. Coins dispense according to chit value
6. Chit is released after dispensing
7. System resets for next transaction

---

## 📞 Support

If you encounter issues not covered here:
1. Check `INTEGRATION_GUIDE.md` for detailed troubleshooting
2. Review `ESP32_SLAVE_README.md` for RPi setup
3. Test individual components using test scripts
4. Check system logs for error messages

---

**System Version**: 1.0  
**Last Updated**: November 27, 2025  
**Status**: ✅ PRODUCTION READY
