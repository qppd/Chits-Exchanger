# I2C LCD Display Integration for YOLO Chit Detection

## Overview
The YOLO detection script now includes support for a 20x4 I2C LCD display to show real-time status information.

## Hardware Requirements
- **20x4 I2C LCD Module** (with PCF8574 I2C backpack)
- **I2C Address**: 0x27 (default, can be changed in code)
- **Raspberry Pi 4** with I2C enabled

## Wiring Connections

```
LCD Module  ->  Raspberry Pi
VCC         ->  5V (Pin 2 or Pin 4)
GND         ->  Ground (Pin 6, 9, 14, 20, 25, 30, 34, or 39)
SDA         ->  GPIO2 (Pin 3)
SCL         ->  GPIO3 (Pin 5)
```

## Installation

### 1. Enable I2C on Raspberry Pi
```bash
sudo raspi-config
# Navigate to: Interface Options -> I2C -> Enable
# Reboot after enabling
```

### 2. Install Required Dependencies
```bash
# Install I2C tools
sudo apt-get update
sudo apt-get install -y i2c-tools python3-smbus

# Install Python smbus2 library
pip3 install smbus2
```

Or use the installation script:
```bash
cd /home/admin/Chits-Exchanger/source/rpi/yolo
chmod +x install_lcd.sh
./install_lcd.sh
```

### 3. Verify I2C Connection
```bash
# Scan for I2C devices (should show 0x27)
sudo i2cdetect -y 1
```

Expected output:
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- 27 -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- --
```

## LCD Display Information

### Screen Layout (20x4)

**Waiting State:**
```
Ready
Waiting for chit...
(empty)
(empty)
```

**IR Detection:**
```
IR DETECTED!
Scanning chit...
Please wait...
(empty)
```

**Active Detection:**
```
DETECTING...
Chit: P50
Conf: 85%
Time: 3s
```

**Detection Success:**
```
DETECTED!
Value: P50
Conf: 95%
Releasing chit...
```

**After Release:**
```
SUCCESS!
Chit: P50
Released
Sent to ESP32
```

**ESP32 Response:**
```
ESP32:
Dispensing
Complete!
(empty)
```

**Detection Timeout:**
```
TIMEOUT!
No valid chit
detected
Try again...
```

**System Shutdown:**
```
Shutting down...
(empty)
(empty)
(empty)
```

## Usage

The LCD display is automatically initialized when running the YOLO detection script:

```bash
cd /home/admin/Chits-Exchanger/source/rpi/yolo
python yolo_detect.py --model my_model_ncnn_model --camera 0 --resolution 640x480
```

The script will:
- ✅ Automatically detect and initialize the LCD if connected
- ✅ Display status messages throughout the detection workflow
- ✅ Show detection results in real-time
- ✅ Update when ESP32 sends messages
- ✅ Continue working even if LCD is not connected (graceful degradation)

## Troubleshooting

### LCD not working?

1. **Check I2C is enabled:**
   ```bash
   lsmod | grep i2c
   # Should show: i2c_bcm2835
   ```

2. **Verify I2C address:**
   ```bash
   sudo i2cdetect -y 1
   ```
   If you see a different address (e.g., 0x3F), update the `I2C_ADDR` variable in `yolo_detect.py`

3. **Check wiring:**
   - Ensure VCC is connected to 5V (not 3.3V)
   - Verify SDA/SCL connections are correct
   - Check for loose connections

4. **Test LCD separately:**
   ```bash
   cd /home/admin/Chits-Exchanger/source/rpi/test
   python3 lcd_tester.py
   ```

5. **Check permissions:**
   ```bash
   # Add user to i2c group
   sudo usermod -a -G i2c $USER
   # Logout and login again
   ```

### Common Issues

**"WARNING: smbus2 not available"**
- Solution: `pip3 install smbus2`

**"WARNING: Could not initialize LCD"**
- Check I2C is enabled in raspi-config
- Verify LCD is connected to correct pins
- Check I2C address with `sudo i2cdetect -y 1`

**LCD shows garbage characters:**
- Power cycle the LCD
- Check voltage (should be 5V, not 3.3V)
- Verify I2C address matches

**LCD works but shows wrong characters:**
- May need to adjust timing constants (E_PULSE, E_DELAY)
- Some LCD modules have different memory mappings

## Features

- ✅ **Auto-detection**: Gracefully handles missing LCD
- ✅ **Real-time updates**: Shows current detection status
- ✅ **ESP32 integration**: Displays messages from ESP32
- ✅ **Error handling**: Continues operation even if LCD fails
- ✅ **Clean shutdown**: Clears display on exit

## Configuration

Edit these constants in `yolo_detect.py` if needed:

```python
# LCD Configuration
I2C_ADDR = 0x27      # Change if your LCD uses different address
LCD_WIDTH = 20       # Characters per line
LCD_ROWS = 4         # Number of rows

# Common alternative addresses: 0x3F, 0x27, 0x20, 0x26
```

## Testing

Test the LCD independently:
```bash
# Test with dedicated tester
cd /home/admin/Chits-Exchanger/source/rpi/test
python3 lcd_tester.py

# Test with full system
cd /home/admin/Chits-Exchanger/source/rpi/yolo
python yolo_detect.py --model my_model_ncnn_model --camera 0 --resolution 640x480
```

## Benefits

1. **User Feedback**: Clear visual feedback without monitor
2. **Status Monitoring**: See system state at a glance
3. **Debugging**: Easier to diagnose issues in production
4. **Professional**: More polished user experience
5. **Standalone Operation**: No need for external display

## Technical Details

- **Library**: smbus2 (pure Python I2C)
- **Protocol**: PCF8574 I2C expander
- **Update Rate**: On-demand (event-driven)
- **Character Set**: Standard ASCII
- **Max Line Length**: 20 characters
- **Lines**: 4 rows

## Future Enhancements

Potential improvements:
- [ ] Custom characters for icons
- [ ] Progress bars for detection confidence
- [ ] Multiple language support
- [ ] Backlight control based on activity
- [ ] Sound notifications via buzzer
- [ ] Multi-line scrolling messages
