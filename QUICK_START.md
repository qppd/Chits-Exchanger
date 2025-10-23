# Quick Start Guide - Chits to Coin Exchanger

## Prerequisites

### Hardware Required
- Raspberry Pi 4 with Raspbian OS
- ESP32 Development Board
- ESP32-CAM module
- 3x ALLAN Coin Hoppers (5, 10, 20 PHP)
- 3x Solid State Relays (SSR)
- IR Sensor module
- Servo motor (SG90 or similar)
- I2C LCD 20x4 (address 0x27)
- Push button
- Power supplies (5V for ESP32, 12V for hoppers)
- Jumper wires and breadboard

### Software Required
- Arduino IDE (for ESP32)
- Python 3.7+ (on Raspberry Pi)
- VS Code (optional)

## Step 1: Install Dependencies on Raspberry Pi

```bash
# Update system
sudo apt-get update
sudo apt-get upgrade

# Install Python packages
cd source/rpi/yolo
pip3 install -r requirements.txt
pip3 install pyserial RPi.GPIO pigpio

# Install and start pigpio daemon (required for servo control)
sudo apt-get install pigpio python-pigpio python3-pigpio
sudo pigpiod

# Make pigpio start on boot (optional)
sudo systemctl enable pigpiod
```

## Step 2: Flash ESP32 Boards

### Flash CoinExchanger to ESP32
1. Open `source/esp32/CoinExchanger/CoinExchanger.ino` in Arduino IDE
2. Select board: "ESP32 Dev Module"
3. Select port: Your ESP32's COM port
4. Click Upload
5. Open Serial Monitor at 115200 baud
6. You should see: "=== ESP32 Coin Exchanger System ==="

### Flash IPCamera to ESP32-CAM
1. Open `source/esp32cam/IPCamera/IPCamera.ino` in Arduino IDE
2. Install WiFiManager library: Tools → Manage Libraries → Search "WiFiManager"
3. Select board: "AI Thinker ESP32-CAM"
4. Connect ESP32-CAM to programmer
5. Click Upload
6. Open Serial Monitor to see WiFi setup instructions
7. Connect to WiFi or use AP mode
8. Note the IP address displayed

## Step 3: Hardware Connections

### Raspberry Pi Connections
```
GPIO 2  (SDA) ──→ LCD SDA
GPIO 3  (SCL) ──→ LCD SCL
GPIO 17       ──→ IR Sensor OUT
GPIO 22       ──→ Servo Signal
5V            ──→ Power rails
GND           ──→ Ground rails
USB           ──→ ESP32 (for serial)
```

### ESP32 Connections
```
GPIO 21    ──→ LCD SDA
GPIO 22    ──→ LCD SCL
GPIO 27    ──→ Button (with pull-up)

Hopper 1 (5 PHP):
  GPIO 19  ──→ Hopper Pulse Output
  GPIO 26  ──→ SSR Input (controls hopper power)

Hopper 2 (10 PHP):
  GPIO 18  ──→ Hopper Pulse Output
  GPIO 25  ──→ SSR Input

Hopper 3 (20 PHP):
  GPIO 4   ──→ Hopper Pulse Output
  GPIO 33  ──→ SSR Input

TX/RX      ──→ RPi USB
```

### SSR Wiring (for each hopper)
```
SSR Input (+) ──→ ESP32 GPIO (26, 25, or 33)
SSR Input (-) ──→ ESP32 GND
SSR Load (+)  ──→ 12V Power Supply (+)
SSR Load (-)  ──→ Hopper Power (+)
Hopper (-)    ──→ 12V Power Supply (-)
```

## Step 4: Test Components

### Test 1: ESP32 Coin Hoppers
```
1. Open Arduino Serial Monitor (115200 baud)
2. Type: help
3. Type: test_relay 1 on
   - Hopper 1 should power on
4. Type: test_pulse 1
   - Drop coins into hopper 1
   - Should see pulse detection messages
5. Type: test_relay 1 off
6. Repeat for hoppers 2 and 3
```

### Test 2: ESP32-CAM Stream
```
1. Open browser
2. Go to: http://<camera_ip>/stream
3. You should see live video
4. Test flash: http://<camera_ip>/flash/on
```

### Test 3: Raspberry Pi Components
```bash
# Test IR Sensor
cd source/rpi/test
python3 ir_sensor_tester.py
# Wave hand in front of sensor - should detect

# Test Servo
python3 servo_tester.py
# Servo should move between 40° and 90°

# Test LCD
python3 lcd_tester.py
# LCD should display test messages
```

### Test 4: Serial Communication
```
Terminal 1 (RPi):
  screen /dev/ttyUSB0 115200
  
Terminal 2 (ESP32 Serial Monitor):
  Type: test_chit 50
  
You should see messages flowing between devices
Press Ctrl+A then K to exit screen
```

## Step 5: Run Complete System

### Start the System

```bash
# On Raspberry Pi
cd source/rpi/yolo

# Make sure pigpio is running
sudo pigpiod

# Start YOLO detection (replace with your camera IP and serial port)
python3 yolo_detect.py \
  --model yolo11n_ncnn_model \
  --camera_ip 192.168.1.21 \
  --esp32_port /dev/ttyUSB0
```

### Test the Workflow

1. **Insert a chit** into the IR sensor area
   - You should see "IR SENSOR DETECTED CHIT" in terminal
   
2. **Wait for detection** (up to 10 seconds)
   - YOLO will detect the denomination
   - Terminal shows: "Detected: ₱50 chit"
   - Servo releases the chit
   
3. **ESP32 LCD displays:**
   ```
   Chit Detected!
   Value: P50
   ```

4. **Press button** to select denomination
   - First press: Select 5 PHP coins
   - Second press: Select 10 PHP coins
   - Third press: Select 20 PHP coins
   - Fourth press: Confirm selection
   
5. **LCD shows calculation:**
   ```
   Plan:
   5P:2 10P:1 20P:2
   Total: P50
   ```

6. **Coins dispense automatically**
   - LCD shows progress for each hopper
   - Pulse counting tracks each coin
   
7. **LCD shows completion:**
   ```
   Complete!
   Dispensed: P50
   Thank you!
   ```

8. **System resets** to idle after 5 seconds

## Troubleshooting

### Camera stream not accessible
```bash
# Check ESP32-CAM IP
# Connect to ESP32-CAM_ConfigAP WiFi and configure

# Ping the camera
ping 192.168.1.21

# Try different URL
http://<ip>/
```

### pigpio errors
```bash
# Restart pigpio daemon
sudo killall pigpiod
sudo pigpiod

# Check if running
ps aux | grep pigpiod
```

### Serial port not found
```bash
# List USB devices
ls /dev/ttyUSB*
ls /dev/ttyACM*

# Add user to dialout group
sudo usermod -a -G dialout $USER
# Then logout and login again
```

### YOLO not detecting
```bash
# Check model path
ls yolo11n_ncnn_model/

# Lower confidence threshold
python3 yolo_detect.py --thresh 0.3 ...

# Check camera stream first
firefox http://192.168.1.21/stream
```

### Hopper not dispensing
```
1. Check SSR LED - should light up when active
2. Check 12V power supply to hopper
3. Test SSR with: test_relay 1 on
4. Manually test hopper with external power
5. Check pulse wire connection
```

### LCD not displaying
```bash
# Check I2C devices
i2cdetect -y 1

# Should show 0x27
# If not, check wiring or LCD address
```

## Common Commands

### ESP32 Test Commands
```
test_chit 50           - Simulate 50 peso chit
test_pulse 1           - Test hopper 1 pulse detection
test_relay 1 on        - Turn on hopper 1
test_relay 1 off       - Turn off hopper 1
test_all               - Test all components
help                   - Show help menu
```

### Run YOLO with Different Settings
```bash
# Different camera IP
python3 yolo_detect.py --model yolo11n_ncnn_model --camera_ip 192.168.1.100

# Different serial port
python3 yolo_detect.py --model yolo11n_ncnn_model --esp32_port /dev/ttyACM0

# Lower confidence threshold
python3 yolo_detect.py --model yolo11n_ncnn_model --thresh 0.3

# Record video
python3 yolo_detect.py --model yolo11n_ncnn_model --resolution 640x480 --record
```

## Safety Notes

⚠️ **Important:**
- Always test SSRs before connecting coin hoppers
- Use proper 12V power supply for hoppers (check hopper specs)
- Don't exceed hopper motor ratings
- Keep coins loaded in hoppers to avoid dry runs
- Monitor hopper temperatures during extended use
- Use proper fuses and circuit protection
- Ground all devices properly

## Maintenance

### Daily
- Check coin levels in hoppers
- Clean IR sensor lens
- Verify LCD brightness

### Weekly
- Test all components with test commands
- Clean coin path in hoppers
- Check all connections

### Monthly
- Deep clean hoppers
- Inspect SSR contacts
- Update YOLO model if needed
- Backup configuration

## Support

For issues or questions:
1. Check `SYSTEM_IMPLEMENTATION.md` for detailed documentation
2. Review error messages in serial monitor and terminal
3. Test individual components first
4. Check hardware connections
5. Verify power supplies

---

**System Status:** Implementation Complete
**Last Updated:** October 24, 2025
