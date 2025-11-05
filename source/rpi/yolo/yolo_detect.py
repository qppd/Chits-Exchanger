import os
import sys
import argparse
import glob
import time
import serial
import RPi.GPIO as GPIO
import pigpio

import cv2
import numpy as np
from ultralytics import YOLO

# I2C LCD Support
try:
    import smbus2 as smbus
    LCD_AVAILABLE = True
except ImportError:
    print("WARNING: smbus2 not available. LCD display will be disabled.")
    print("Install with: pip install smbus2")
    LCD_AVAILABLE = False

# ===== LCD Configuration =====
I2C_ADDR = 0x27  # I2C device address
LCD_WIDTH = 20   # Maximum characters per line
LCD_ROWS = 4     # Number of display lines

# LCD Commands
LCD_CHR = 1  # Mode - Sending data
LCD_CMD = 0  # Mode - Sending command

LCD_LINE_1 = 0x80  # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0  # LCD RAM address for the 2nd line
LCD_LINE_3 = 0x94  # LCD RAM address for the 3rd line
LCD_LINE_4 = 0xD4  # LCD RAM address for the 4th line

# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005

class LCD:
    """I2C LCD Display Class for 20x4 character display"""
    def __init__(self, addr=I2C_ADDR, bus=1):
        """Initialize I2C bus and LCD"""
        if not LCD_AVAILABLE:
            self.enabled = False
            return
        
        try:
            self.bus = smbus.SMBus(bus)
            self.addr = addr
            self.enabled = True
            
            # Initialize display
            self.lcd_byte(0x33, LCD_CMD)  # 110011 Initialize
            self.lcd_byte(0x32, LCD_CMD)  # 110010 Initialize
            self.lcd_byte(0x06, LCD_CMD)  # 000110 Cursor move direction
            self.lcd_byte(0x0C, LCD_CMD)  # 001100 Display On, Cursor Off, Blink Off
            self.lcd_byte(0x28, LCD_CMD)  # 101000 Data length, number of lines, font size
            self.lcd_byte(0x01, LCD_CMD)  # 000001 Clear display
            time.sleep(E_DELAY)
            print(f"LCD initialized at address 0x{addr:02X}")
        except Exception as e:
            print(f"WARNING: Could not initialize LCD: {e}")
            self.enabled = False

    def lcd_byte(self, bits, mode):
        """Send byte to data pins"""
        if not self.enabled:
            return
        
        try:
            bits_high = mode | (bits & 0xF0) | 0x08
            bits_low = mode | ((bits << 4) & 0xF0) | 0x08

            # High bits
            self.bus.write_byte(self.addr, bits_high)
            self.lcd_toggle_enable(bits_high)

            # Low bits
            self.bus.write_byte(self.addr, bits_low)
            self.lcd_toggle_enable(bits_low)
        except:
            pass

    def lcd_toggle_enable(self, bits):
        """Toggle enable"""
        time.sleep(E_DELAY)
        self.bus.write_byte(self.addr, (bits | 0x04))
        time.sleep(E_PULSE)
        self.bus.write_byte(self.addr, (bits & ~0x04))
        time.sleep(E_DELAY)

    def lcd_string(self, message, line):
        """Send string to display"""
        if not self.enabled:
            return
        
        message = message.ljust(LCD_WIDTH, " ")
        self.lcd_byte(line, LCD_CMD)

        for i in range(min(LCD_WIDTH, len(message))):
            self.lcd_byte(ord(message[i]), LCD_CHR)

    def clear(self):
        """Clear LCD display"""
        if not self.enabled:
            return
        
        self.lcd_byte(0x01, LCD_CMD)
        time.sleep(E_DELAY)
    
    def display_lines(self, line1="", line2="", line3="", line4=""):
        """Display multiple lines at once"""
        if not self.enabled:
            return
        
        if line1:
            self.lcd_string(line1, LCD_LINE_1)
        if line2:
            self.lcd_string(line2, LCD_LINE_2)
        if line3:
            self.lcd_string(line3, LCD_LINE_3)
        if line4:
            self.lcd_string(line4, LCD_LINE_4)

# Determine if GUI is available (e.g., from terminal or desktop session)
use_gui = "DISPLAY" in os.environ

# Define and parse user input arguments
parser = argparse.ArgumentParser()
parser.add_argument('--model', help='Path to YOLO model file (example: "runs/detect/train/weights/best.pt")',
                    required=True)
parser.add_argument('--thresh', help='Minimum confidence threshold for displaying detected objects (example: "0.4")',
                    default=0.5)
parser.add_argument('--resolution', help='Resolution in WxH to display inference results at (example: "640x480"), \
                    otherwise, match source resolution',
                    default=None)
parser.add_argument('--record', help='Record results from video or webcam and save it as "demo1.avi". Must specify --resolution argument to record.',
                    action='store_true')
parser.add_argument('--esp32_port', help='Serial port for ESP32 communication (example: "/dev/ttyUSB0")',
                    default='/dev/ttyUSB0')
parser.add_argument('--camera', help='USB camera device ID (example: "0" for /dev/video0)',
                    default='0')

args = parser.parse_args()

# USB camera device
img_source = int(args.camera)

# GPIO Pin Configurations for RPi
IR_SENSOR_PIN = 17  # IR sensor input
SERVO_PIN = 22      # Servo control pin

# Servo angles
SERVO_INITIAL_ANGLE = 39
SERVO_RELEASE_ANGLE = 90

# Valid chit denominations
VALID_CHITS = [5, 10, 20, 50]

# Parse user inputs
model_path = args.model
min_thresh = args.thresh
user_res = args.resolution
record = args.record

# Check if model file exists and is valid
if (not os.path.exists(model_path)):
    print('ERROR: Model path is invalid or model was not found. Make sure the model filename was entered correctly.')
    sys.exit(0)

# Load the model into memory and get labemap
model = YOLO(model_path, task='detect')
labels = model.names

# Set source type to USB webcam
source_type = 'usb'
print(f'Using USB webcam at /dev/video{args.camera}')

# Parse user-specified display resolution
resize = False
if user_res:
    resize = True
    resW, resH = int(user_res.split('x')[0]), int(user_res.split('x')[1])

# Initialize GPIO for IR sensor and Servo
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(IR_SENSOR_PIN, GPIO.IN)

# Initialize pigpio for servo control
pi = pigpio.pi()
if not pi.connected:
    print("ERROR: Could not connect to pigpio daemon. Run 'sudo pigpiod' first.")
    sys.exit(1)

# Initialize serial communication with ESP32
try:
    print(f"Initializing serial connection to ESP32 on {args.esp32_port}...")
    
    # Open serial port with proper settings
    # IMPORTANT: Disable DTR/RTS to prevent ESP32 auto-reset
    esp32_serial = serial.Serial()
    esp32_serial.port = args.esp32_port
    esp32_serial.baudrate = 115200
    esp32_serial.timeout = 0.01  # Very short timeout for non-blocking reads
    esp32_serial.write_timeout = 1.0  # 1 second write timeout
    esp32_serial.bytesize = serial.EIGHTBITS
    esp32_serial.parity = serial.PARITY_NONE
    esp32_serial.stopbits = serial.STOPBITS_ONE
    
    # Disable DTR and RTS BEFORE opening to prevent ESP32 reset
    esp32_serial.dtr = False
    esp32_serial.rts = False
    
    # Now open the port
    esp32_serial.open()
    
    # Ensure DTR and RTS stay disabled (some systems re-enable on open)
    esp32_serial.dtr = False
    esp32_serial.rts = False
    
    # Wait for connection to stabilize (reduced from 2s since no reset occurs)
    time.sleep(0.5)
    
    # Clear any buffered data from initialization
    esp32_serial.reset_input_buffer()
    esp32_serial.reset_output_buffer()
    
    print(f"✅ Serial connection to ESP32 established successfully")
    print(f"   Port: {args.esp32_port}")
    print(f"   Baud: 115200")
    print(f"   Mode: Non-blocking (DTR/RTS disabled)")
    
except serial.SerialException as e:
    print(f"❌ Serial connection failed: {e}")
    print(f"   Please check:")
    print(f"   1. ESP32 is connected to {args.esp32_port}")
    print(f"   2. User has permissions (run: sudo usermod -a -G dialout $USER)")
    print(f"   3. No other program is using the port")
    print("⚠️  Continuing without ESP32 serial communication...")
    esp32_serial = None
except Exception as e:
    print(f"❌ Unexpected error initializing serial: {e}")
    print("⚠️  Continuing without ESP32 serial communication...")
    esp32_serial = None

# Initialize I2C LCD Display
lcd = LCD()
if lcd.enabled:
    lcd.clear()
    lcd.display_lines(
        "Chit Detection",
        "System Ready",
        "Waiting for chit...",
        ""
    )
    time.sleep(2)
else:
    print("LCD display not available - continuing without LCD")

# Check if recording is valid and set up recording
if record:
    if source_type not in ['video','usb']:
        print('Recording only works for video and camera sources. Please try again.')
        sys.exit(0)
    if not user_res:
        print('Please specify resolution to record video at.')
        sys.exit(0)
    
    # Set up recording
    record_name = 'demo1.avi'
    record_fps = 30
    recorder = cv2.VideoWriter(record_name, cv2.VideoWriter_fourcc(*'MJPG'), record_fps, (resW,resH))

# Helper functions for servo control
def angle_to_pulse(angle):
    """Convert angle (0-180) to pulse width (500-2500us)"""
    return int(500 + (angle / 180.0) * 2000)

def set_servo_angle(angle):
    """Set servo to specified angle"""
    if angle < 0:
        angle = 0
    elif angle > 180:
        angle = 180
    pulse_width = angle_to_pulse(angle)
    pi.set_servo_pulsewidth(SERVO_PIN, pulse_width)

def release_chit():
    """Release chit by moving servo to release position and back"""
    print(f"Releasing chit: Moving servo from {SERVO_INITIAL_ANGLE}° to {SERVO_RELEASE_ANGLE}°")
    set_servo_angle(SERVO_RELEASE_ANGLE)
    time.sleep(2)  # Hold for 2 seconds
    print(f"Returning servo to initial position {SERVO_INITIAL_ANGLE}°")
    set_servo_angle(SERVO_INITIAL_ANGLE)

def send_to_esp32(message):
    """Send message to ESP32 via serial with proper formatting and error handling"""
    if esp32_serial and esp32_serial.is_open:
        try:
            # Clear any pending output first
            esp32_serial.reset_output_buffer()
            
            # Send message with newline terminator
            full_message = message + '\n'
            esp32_serial.write(full_message.encode('utf-8'))
            esp32_serial.flush()  # Ensure data is sent immediately
            
            print(f"✅ Sent to ESP32: {message}")
            return True
        except serial.SerialException as e:
            print(f"❌ Serial error sending to ESP32: {e}")
            return False
        except Exception as e:
            print(f"❌ Error sending to ESP32: {e}")
            return False
    else:
        print(f"⚠️  Cannot send '{message}' - Serial not connected")
        return False

def read_from_esp32():
    """Read messages from ESP32 if available (non-blocking) with proper error handling"""
    if esp32_serial and esp32_serial.is_open:
        try:
            # Check if data is available without blocking
            if esp32_serial.in_waiting > 0:
                # Read line with timeout protection
                message = esp32_serial.readline().decode('utf-8', errors='ignore').strip()
                
                # Only process non-empty messages
                if message:
                    # Filter out debug messages (keep only important ones)
                    if message.startswith("DISPENSING_COMPLETE"):
                        print(f"📨 ESP32: {message}")
                        
                        # Update LCD with completion message
                        lcd.display_lines(
                            "ESP32:",
                            "Dispensing",
                            "Complete!",
                            ""
                        )
                        time.sleep(2)
                        lcd.display_lines(
                            "Ready",
                            "Waiting for chit...",
                            "",
                            ""
                        )
                    elif message.startswith("✅") or message.startswith("❌") or message.startswith("⚠️"):
                        # Important status messages
                        print(f"📨 ESP32: {message}")
                    elif "AUTO_DISPENSE" in message or "SYSTEM BUSY" in message:
                        # Important operational messages
                        print(f"📨 ESP32: {message}")
                    
                    return message
        except serial.SerialException as e:
            print(f"❌ Serial exception reading from ESP32: {e}")
        except UnicodeDecodeError as e:
            print(f"⚠️  Decode error reading from ESP32: {e}")
        except Exception as e:
            print(f"❌ Unexpected error reading from ESP32: {e}")
    return None

def is_ir_detected():
    """Check if IR sensor detects a chit"""
    return not GPIO.input(IR_SENSOR_PIN)  # Active LOW

# Initialize servo to home position
set_servo_angle(SERVO_INITIAL_ANGLE)
print(f"Servo initialized to {SERVO_INITIAL_ANGLE}°")

# Initialize USB webcam connection
cap = cv2.VideoCapture(img_source)

# Set camera resolution if specified by user
if user_res:
    ret = cap.set(3, resW)
    ret = cap.set(4, resH)

# Check if connection is successful
if not cap.isOpened():
    print(f"Failed to open USB camera /dev/video{args.camera}")
    sys.exit(1)

print("Successfully connected to USB webcam")

# Set bounding box colors (using the Tableu 10 color scheme)
bbox_colors = [(164,120,87), (68,148,228), (93,97,209), (178,182,133), (88,159,106), 
              (96,202,231), (159,124,168), (169,162,241), (98,118,150), (172,176,184)]

# Detection state variables
last_ir_state = False
detection_enabled = False  # Flag to enable/disable detection
NUM_CAPTURE_IMAGES = 3  # Number of images to capture and analyze
CAPTURE_DELAY = 0.1  # Delay between captures (seconds)
DISPLAY_TIME = 3  # Time to display each captured image (seconds)


# Helper function to capture and analyze images
def capture_and_detect():
    """Capture 3 images quickly and run inference to detect chit value"""
    print(f"\n{'='*60}")
    print(f"📸 CAPTURING {NUM_CAPTURE_IMAGES} IMAGES FOR DETECTION")
    print(f"{'='*60}")
    
    best_chit_value = None
    best_confidence = 0.0
    all_detections = []
    captured_frames = []
    
    t_detect_start = time.perf_counter()
    
    for img_num in range(NUM_CAPTURE_IMAGES):
        # Capture frame
        ret, frame = cap.read()
        if not ret or frame is None:
            print(f"❌ Failed to capture image {img_num + 1}")
            continue
        
        # Resize if needed
        if resize:
            frame = cv2.resize(frame, (resW, resH))
        
        # Store original frame for display
        display_frame = frame.copy()
        
        # Run inference
        print(f"   Image {img_num + 1}/{NUM_CAPTURE_IMAGES}: Running inference...")
        results = model(frame, verbose=False)
        detections = results[0].boxes
        
        # Process detections and draw on frame
        for i in range(len(detections)):
            conf = detections[i].conf.item()
            
            if conf > 0.5:
                classidx = int(detections[i].cls.item())
                classname = labels[classidx]
                
                # Get bounding box coordinates
                xyxy_tensor = detections[i].xyxy.cpu()
                xyxy = xyxy_tensor.numpy().squeeze()
                xmin, ymin, xmax, ymax = xyxy.astype(int)
                
                # Draw bounding box
                color = bbox_colors[classidx % 10]
                cv2.rectangle(display_frame, (xmin, ymin), (xmax, ymax), color, 2)
                
                # Draw label
                label = f'{classname}: {int(conf*100)}%'
                labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                label_ymin = max(ymin, labelSize[1] + 10)
                cv2.rectangle(display_frame, (xmin, label_ymin-labelSize[1]-10), 
                            (xmin+labelSize[0], label_ymin+baseLine-10), color, cv2.FILLED)
                cv2.putText(display_frame, label, (xmin, label_ymin-7), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
                
                try:
                    chit_value = int(classname)
                    if chit_value in VALID_CHITS:
                        all_detections.append((chit_value, conf))
                        print(f"      ✓ Detected: ₱{chit_value} | Conf: {conf:.2%}")
                        
                        if conf > best_confidence:
                            best_chit_value = chit_value
                            best_confidence = conf
                except ValueError:
                    pass
        
        # Add image number and timestamp to frame
        cv2.putText(display_frame, f'Image {img_num + 1}/{NUM_CAPTURE_IMAGES}', 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Store frame for display
        captured_frames.append(display_frame)
        
        # Small delay between captures
        if img_num < NUM_CAPTURE_IMAGES - 1:
            time.sleep(CAPTURE_DELAY)
    
    t_detect_stop = time.perf_counter()
    detection_time = t_detect_stop - t_detect_start
    
    print(f"\n   Total detections: {len(all_detections)}")
    print(f"   Detection time: {detection_time:.3f}s")
    print(f"{'='*60}\n")
    
    # Display all captured frames with detections
    print(f"📺 Displaying {len(captured_frames)} captured images...")
    for idx, frame in enumerate(captured_frames):
        # Add detection result text
        result_text = f"Best: P{best_chit_value} ({int(best_confidence*100)}%)" if best_chit_value else "No chit detected"
        cv2.putText(frame, result_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        # Show the frame
        cv2.imshow(f'Chit Detection - Image {idx + 1}', frame)
        cv2.waitKey(DISPLAY_TIME * 1000)  # Display for DISPLAY_TIME seconds
        cv2.destroyWindow(f'Chit Detection - Image {idx + 1}')
    
    return best_chit_value, best_confidence, detection_time

# Begin YOLO detection using USB webcam
print("Connected to USB webcam. Fast capture mode enabled.")
print(f"IR Sensor on GPIO {IR_SENSOR_PIN}")
print(f"Servo on GPIO {SERVO_PIN}")
print(f"Capture mode: {NUM_CAPTURE_IMAGES} images per detection")
print(f"Display time: {DISPLAY_TIME} seconds per image")
print("\n⏳ Loading complete! System ready for operation.")
print("Detection will start automatically when IR sensor is triggered.")
print("Waiting for IR sensor to detect chit...")

# Enable detection flag after loading
detection_enabled = True

# Begin monitoring loop
while True:
    
    # Check for messages from ESP32
    esp32_msg = read_from_esp32()
    if esp32_msg:
        print(f"ESP32: {esp32_msg}")
    
    # Check IR sensor state
    ir_detected = is_ir_detected()
    
    # Start detection when IR sensor is triggered (rising edge) AND detection is enabled
    if ir_detected and not last_ir_state and detection_enabled:
        print(f"\n{'='*60}")
        print(f"🔍 IR SENSOR TRIGGERED - CHIT DETECTED")
        print(f"{'='*60}")
        
        send_to_esp32("IR_DETECTED")
        
        # Update LCD
        lcd.display_lines(
            "IR DETECTED!",
            "Capturing images...",
            "Please wait...",
            ""
        )
        
        # Capture and detect
        detected_chit_value, detection_confidence, detection_time = capture_and_detect()
        
        if detected_chit_value:
            print(f"\n{'='*60}")
            print(f"🎉 DETECTION COMPLETE")
            print(f"{'='*60}")
            print(f"   Detected Value: ₱{detected_chit_value}")
            print(f"   Confidence: {detection_confidence:.2%}")
            print(f"   Detection Time: {detection_time:.3f}s")
            print(f"{'='*60}")
            
            # Update LCD
            lcd.display_lines(
                "DETECTED!",
                f"Value: P{detected_chit_value}",
                f"Conf: {int(detection_confidence*100)}%",
                "Releasing chit..."
            )
            
            # Send detection result to ESP32
            send_to_esp32(f"CHIT_DETECTED:{detected_chit_value}")
            
            # Release the chit
            print(f"🔓 Releasing chit via servo...")
            release_chit()
            print(f"✅ Chit ₱{detected_chit_value} released successfully")
            
            # Auto-dispense: Send command to ESP32 to dispense detected amount
            print(f"\n{'='*60}")
            print(f"🪙 AUTO-DISPENSING TRIGGERED")
            print(f"{'='*60}")
            print(f"   Sending to ESP32: AUTO_DISPENSE:{detected_chit_value}")
            print(f"   Expected dispensing:")
            
            # Show expected coin breakdown
            if detected_chit_value == 5:
                print(f"     - 1 x 5 PHP coin (Hopper 1)")
            elif detected_chit_value == 10:
                print(f"     - 1 x 10 PHP coin (Hopper 2)")
            elif detected_chit_value == 20:
                print(f"     - 1 x 20 PHP coin (Hopper 3)")
            elif detected_chit_value == 50:
                print(f"     - 2 x 20 PHP coins (Hopper 3)")
                print(f"     - 1 x 10 PHP coin (Hopper 2)")
            
            print(f"{'='*60}\n")
            
            # Send the command
            if send_to_esp32(f"AUTO_DISPENSE:{detected_chit_value}"):
                # Update LCD
                lcd.display_lines(
                    "AUTO DISPENSE!",
                    f"Chit: P{detected_chit_value}",
                    "Dispensing...",
                    "Please wait"
                )
                time.sleep(3)
            else:
                print(f"⚠️  Failed to send AUTO_DISPENSE command")
                lcd.display_lines(
                    "ERROR!",
                    "Communication",
                    "failed",
                    ""
                )
                time.sleep(2)
                
        else:
            print(f"\n{'='*60}")
            print(f"❌ NO VALID CHIT DETECTED")
            print(f"{'='*60}")
            print(f"   No valid denomination found in captured images")
            print(f"{'='*60}\n")
            
            send_to_esp32("DETECTION_TIMEOUT")
            
            # Update LCD
            lcd.display_lines(
                "NO CHIT FOUND!",
                "No valid chit",
                "detected",
                "Try again..."
            )
            time.sleep(2)
        
        print("Waiting for next chit...\n")
        
        # Reset LCD to waiting state
        lcd.display_lines(
            "Ready",
            "Waiting for chit...",
            "",
            ""
        )
    
    last_ir_state = ir_detected
    
    # Small delay to prevent CPU spinning
    time.sleep(0.05)

# Clean up
print(f'\nShutting down...')

# Update LCD
lcd.display_lines(
    "Shutting down...",
    "",
    "",
    ""
)
time.sleep(1)

# Clean up GPIO and servo
set_servo_angle(SERVO_INITIAL_ANGLE)
pi.set_servo_pulsewidth(SERVO_PIN, 0)  # Stop servo
pi.stop()
GPIO.cleanup()

# Close serial connection
if esp32_serial and esp32_serial.is_open:
    send_to_esp32("SYSTEM_SHUTDOWN")
    esp32_serial.close()

# Clear LCD before exit
lcd.clear()

cap.release()
if record: recorder.release()

print("System shutdown complete.")
