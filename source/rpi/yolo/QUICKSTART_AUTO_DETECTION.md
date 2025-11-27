# Quick Start: Auto-Detection Threading

## What Was Implemented

✅ **Camera Auto-Detection** - Automatically finds USB camera on /dev/video0-9
✅ **ESP32 Auto-Detection** - Automatically finds ESP32 on USB/ACM/ttyUSB ports
✅ **Threaded Serial Communication** - Non-blocking serial I/O in background thread
✅ **No Duplicate Functions** - Removed old blocking serial functions
✅ **Proper Thread Cleanup** - Clean shutdown of threads on exit

## Files Modified

- `yolo_detect_threaded.py` - Main threaded implementation with auto-detection
- `AUTO_DETECTION_THREADING_SUMMARY.md` - Detailed documentation
- `test_auto_detection.py` - Test script for hardware detection

## Usage

### Test Auto-Detection First
```bash
cd /home/pi/Chits-Exchanger/source/rpi/yolo
python test_auto_detection.py
```

This will show you:
- Which camera device was detected
- Which ESP32 serial port was detected
- Whether you're ready to run the full system

### Run with Auto-Detection (Recommended)
```bash
python yolo_detect_threaded.py --model chit_model_ncnn_model --resolution 320x240
```

Camera and ESP32 will be **auto-detected**.

### Run with Manual Device Specification
```bash
python yolo_detect_threaded.py \
    --model chit_model_ncnn_model \
    --camera /dev/video0 \
    --esp32_port /dev/ttyUSB0 \
    --resolution 320x240
```

Use this if auto-detection fails or you want to specify exact devices.

## What Works Now

1. **yolo_detect2.py** - Simple, fast YOLO detection (no hardware dependencies)
   ```bash
   python yolo_detect2.py --model chit_model_ncnn_model --source usb1 --resolution 320x240
   ```
   ✅ Confirmed working by user: "working greate and fast"

2. **yolo_detect_threaded.py** - Full system with auto-detection and threading
   ```bash
   python yolo_detect_threaded.py --model chit_model_ncnn_model --resolution 320x240
   ```
   ✅ Ready for testing on Raspberry Pi

3. **test_auto_detection.py** - Quick hardware detection test
   ```bash
   python test_auto_detection.py
   ```
   ✅ Use this to debug hardware connections

## Key Improvements

| Feature | Before | After |
|---------|--------|-------|
| Camera | Manual specification required | Auto-detected |
| ESP32 Port | Manual specification required | Auto-detected |
| Serial Communication | Blocking (freezes YOLO) | Non-blocking thread |
| Function Duplicates | Yes (2 send_to_esp32) | No (removed duplicates) |
| Thread Cleanup | No | Yes (proper shutdown) |

## Threading Architecture

```
Main Thread:
  ├─ YOLO Detection (inference loop)
  ├─ GPIO Control (IR sensor)
  ├─ Servo Control (chit release)
  └─ LCD Display (status updates)

Serial Thread (daemon):
  ├─ Read from ESP32 (non-blocking)
  ├─ Send to ESP32 (queue-based)
  └─ Auto-terminates on exit
```

## What's Different from yolo_detect2.py

| Feature | yolo_detect2.py | yolo_detect_threaded.py |
|---------|-----------------|-------------------------|
| Hardware | None (YOLO only) | Full system (GPIO, LCD, Servo, ESP32) |
| Serial Comm | No | Yes (threaded) |
| Auto-Detection | No | Yes (camera + ESP32) |
| Use Case | Testing YOLO quickly | Production system |
| Complexity | Simple | Full featured |

## Testing Checklist

On Raspberry Pi:

1. ✅ Pull latest code: `git pull origin main`
2. ✅ Test hardware detection: `python test_auto_detection.py`
3. ✅ Test simple YOLO: `python yolo_detect2.py --model chit_model_ncnn_model --source usb1 --resolution 320x240`
4. ⏳ Test full system: `python yolo_detect_threaded.py --model chit_model_ncnn_model --resolution 320x240`
5. ⏳ Verify non-blocking serial (YOLO should not freeze during ESP32 communication)
6. ⏳ Verify graceful shutdown (Ctrl+C should clean up threads properly)

## Troubleshooting

### Camera Not Detected
```bash
# Check available cameras
ls /dev/video*

# Test manually
python yolo_detect2.py --model chit_model_ncnn_model --source /dev/video0 --resolution 320x240
```

### ESP32 Not Detected
```bash
# Check available serial ports
ls /dev/tty*

# Look for USB or ACM devices
ls /dev/ttyUSB* /dev/ttyACM*

# Test manually
python yolo_detect_threaded.py --model chit_model_ncnn_model --esp32_port /dev/ttyUSB0 --resolution 320x240
```

### Thread Not Stopping Properly
Check for:
- `running = False` is set before GPIO cleanup
- Thread has `daemon=True` flag
- Small delay (0.3s) allows thread to finish

## Next Steps

1. Test on actual Raspberry Pi hardware
2. Verify auto-detection works reliably
3. Test serial communication during YOLO detection
4. Measure performance impact of threading
5. Add more robust error handling if needed

## Support

All code is committed and pushed to GitHub:
- Branch: `main`
- Latest commits:
  - `a75e4ef` - Added test_auto_detection.py
  - `587d342` - Auto-detection and threading implementation

Read `AUTO_DETECTION_THREADING_SUMMARY.md` for detailed technical documentation.
