# Quick Start Guide - I2C LCD Display for Chit Detection

## 1. Install Dependencies

```bash
cd /home/admin/Chits-Exchanger/source/rpi/yolo

# Install smbus2 library
pip3 install smbus2

# Optional: Install I2C tools
sudo apt-get update
sudo apt-get install -y i2c-tools python3-smbus
```

## 2. Enable I2C on Raspberry Pi

```bash
sudo raspi-config
```

Navigate to:
- **Interface Options** → **I2C** → **Enable**
- Reboot: `sudo reboot`

## 3. Connect LCD Hardware

```
LCD Pin  →  Raspberry Pi
VCC      →  5V (Pin 2 or 4)
GND      →  Ground (Pin 6)
SDA      →  GPIO2 (Pin 3)
SCL      →  GPIO3 (Pin 5)
```

## 4. Verify I2C Connection

```bash
# Scan for I2C devices (should show 0x27)
sudo i2cdetect -y 1
```

Expected output - look for **27**:
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- 27 -- -- -- -- -- -- -- -- 
```

## 5. Test LCD Display

### Test LCD independently:
```bash
cd /home/admin/Chits-Exchanger/source/rpi/test
python3 lcd_tester.py
```

### Test all LCD states:
```bash
cd /home/admin/Chits-Exchanger/source/rpi/yolo
python3 test_lcd_states.py
```

### Test with YOLO system:
```bash
cd /home/admin/Chits-Exchanger/source/rpi/yolo
python yolo_detect.py --model my_model_ncnn_model --camera 0 --resolution 640x480
```

## 6. Expected Behavior

When you run `yolo_detect.py`, the LCD will show:

1. **Startup:**
   ```
   Chit Detection
   System Ready
   Waiting for chit...
   ```

2. **When IR detects chit:**
   ```
   IR DETECTED!
   Scanning chit...
   Please wait...
   ```

3. **During detection:**
   ```
   DETECTING...
   Chit: P50
   Conf: 85%
   Time: 3s
   ```

4. **Success:**
   ```
   DETECTED!
   Value: P50
   Conf: 95%
   Releasing chit...
   ```

5. **After release:**
   ```
   SUCCESS!
   Chit: P50
   Released
   Sent to ESP32
   ```

6. **Back to waiting:**
   ```
   Ready
   Waiting for chit...
   ```

## 7. Troubleshooting

### "WARNING: smbus2 not available"
```bash
pip3 install smbus2
```

### "WARNING: Could not initialize LCD"
1. Check I2C is enabled: `lsmod | grep i2c`
2. Scan for device: `sudo i2cdetect -y 1`
3. Check wiring connections
4. Verify 5V power (not 3.3V)

### LCD shows at wrong address (e.g., 0x3F instead of 0x27)
Edit `yolo_detect.py`:
```python
I2C_ADDR = 0x3F  # Change to your address
```

### LCD shows garbage characters
1. Power cycle the LCD
2. Check voltage is 5V
3. Verify wiring is correct

### Permission denied errors
```bash
sudo usermod -a -G i2c $USER
# Then logout and login again
```

## 8. Running the System

### Normal operation:
```bash
cd /home/admin/Chits-Exchanger/source/rpi/yolo
python yolo_detect.py --model my_model_ncnn_model --camera 0 --resolution 640x480
```

### With ESP32 on different port:
```bash
python yolo_detect.py --model my_model_ncnn_model --camera 0 --resolution 640x480 --esp32_port /dev/ttyUSB1
```

### Without GUI (headless):
The system automatically detects if DISPLAY is available. If running headless (SSH), it will skip the OpenCV window but LCD will still work.

## 9. Notes

- ✅ System works **with or without** LCD connected
- ✅ LCD updates are **non-blocking**
- ✅ Error messages go to both **console and LCD**
- ✅ **Automatic fallback** if LCD fails
- ✅ Clean shutdown clears the LCD

## 10. Files Added

```
source/rpi/yolo/
├── yolo_detect.py          (updated with LCD support)
├── install_lcd.sh          (installation script)
├── test_lcd_states.py      (test all display states)
├── LCD_README.md           (detailed documentation)
├── LCD_INTEGRATION_SUMMARY.md
└── QUICKSTART_LCD.md       (this file)
```

## Support

For issues:
1. Check hardware connections
2. Run `test_lcd_states.py` to isolate LCD issues
3. Run `lcd_tester.py` from test folder
4. Verify I2C with `sudo i2cdetect -y 1`

✅ **Ready to use!** The LCD will automatically initialize when you run the detection system.
