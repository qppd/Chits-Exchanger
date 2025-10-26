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
    esp32_serial = serial.Serial(args.esp32_port, 115200, timeout=1)
    time.sleep(2)  # Wait for serial connection to establish
    print(f"Serial connection to ESP32 established on {args.esp32_port}")
except Exception as e:
    print(f"WARNING: Could not connect to ESP32 on {args.esp32_port}: {e}")
    print("Continuing without ESP32 serial communication...")
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
    """Send message to ESP32 via serial"""
    if esp32_serial and esp32_serial.is_open:
        try:
            esp32_serial.write((message + '\n').encode())
            esp32_serial.flush()
            print(f"Sent to ESP32: {message}")
        except Exception as e:
            print(f"Error sending to ESP32: {e}")

def read_from_esp32():
    """Read messages from ESP32 if available"""
    if esp32_serial and esp32_serial.is_open:
        try:
            if esp32_serial.in_waiting > 0:
                message = esp32_serial.readline().decode().strip()
                
                # Update LCD with ESP32 messages
                if message.startswith("DISPENSING_COMPLETE"):
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
                
                return message
        except Exception as e:
            print(f"Error reading from ESP32: {e}")
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

# Initialize control and status variables
avg_frame_rate = 0
frame_rate_buffer = []
fps_avg_len = 200
img_count = 0

# Detection state variables
detection_active = False
last_ir_state = False
detected_chit_value = None
detection_confidence = 0.0
detection_start_time = 0
DETECTION_TIMEOUT = 10  # seconds

# Begin YOLO detection using USB webcam
print("Connected to USB webcam. Running detection...")
print(f"IR Sensor on GPIO {IR_SENSOR_PIN}")
print(f"Servo on GPIO {SERVO_PIN}")
print("Waiting for IR sensor to detect chit...")

# Begin inference loop
while True:
    
    # Check for messages from ESP32
    esp32_msg = read_from_esp32()
    if esp32_msg:
        print(f"ESP32: {esp32_msg}")
    
    # Check IR sensor state
    ir_detected = is_ir_detected()
    
    # Start detection when IR sensor is triggered
    if ir_detected and not last_ir_state:
        print("\n=== IR SENSOR DETECTED CHIT ===")
        detection_active = True
        detected_chit_value = None
        detection_confidence = 0.0
        detection_start_time = time.time()
        send_to_esp32("IR_DETECTED")
        
        # Update LCD
        lcd.display_lines(
            "IR DETECTED!",
            "Scanning chit...",
            "Please wait...",
            ""
        )
    
    # Stop detection if IR sensor no longer detects or timeout
    if detection_active and (not ir_detected or (time.time() - detection_start_time > DETECTION_TIMEOUT)):
        if detected_chit_value:
            print(f"\n=== DETECTION COMPLETE ===")
            print(f"Detected: ₱{detected_chit_value} chit (confidence: {detection_confidence:.2%})")
            
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
            release_chit()
            
            # Notify ESP32 that chit was released
            send_to_esp32(f"CHIT_RELEASED:{detected_chit_value}")
            
            print(f"Chit ₱{detected_chit_value} released successfully")
            
            # Auto-dispense: Send command to ESP32 to dispense detected amount
            print(f"AUTO-DISPENSING: Triggering dispense of ₱{detected_chit_value}")
            send_to_esp32(f"AUTO_DISPENSE:{detected_chit_value}")
            
            # Update LCD
            lcd.display_lines(
                "AUTO DISPENSE!",
                f"Chit: P{detected_chit_value}",
                "Dispensing...",
                "Please wait"
            )
            time.sleep(3)
        elif time.time() - detection_start_time > DETECTION_TIMEOUT:
            print("Detection timeout - no valid chit detected")
            send_to_esp32("DETECTION_TIMEOUT")
            
            # Update LCD
            lcd.display_lines(
                "TIMEOUT!",
                "No valid chit",
                "detected",
                "Try again..."
            )
            time.sleep(2)
        
        detection_active = False
        detected_chit_value = None
        detection_confidence = 0.0
        print("Waiting for next chit...\n")
        
        # Reset LCD to waiting state
        lcd.display_lines(
            "Ready",
            "Waiting for chit...",
            "",
            ""
        )
    
    last_ir_state = ir_detected

    t_start = time.perf_counter()

    # Read frame from USB webcam
    ret, frame = cap.read()
    if not ret or frame is None:
        print('Unable to read frame from USB webcam. Exiting...')
        break

    # Resize frame to desired display resolution
    if resize == True:
        frame = cv2.resize(frame,(resW,resH))

    # Run inference on frame
    results = model(frame, verbose=False)

    # Extract results
    detections = results[0].boxes

    # Initialize variable for basic object counting example
    object_count = 0

    # Go through each detection and get bbox coords, confidence, and class
    for i in range(len(detections)):

        # Get bounding box coordinates
        # Ultralytics returns results in Tensor format, which have to be converted to a regular Python array
        xyxy_tensor = detections[i].xyxy.cpu() # Detections in Tensor format in CPU memory
        xyxy = xyxy_tensor.numpy().squeeze() # Convert tensors to Numpy array
        xmin, ymin, xmax, ymax = xyxy.astype(int) # Extract individual coordinates and convert to int

        # Get bounding box class ID and name
        classidx = int(detections[i].cls.item())
        classname = labels[classidx]

        # Get bounding box confidence
        conf = detections[i].conf.item()

        # Draw box if confidence threshold is high enough
        if conf > 0.5:

            color = bbox_colors[classidx % 10]
            cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), color, 2)

            label = f'{classname}: {int(conf*100)}%'
            labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1) # Get font size
            label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
            cv2.rectangle(frame, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), color, cv2.FILLED) # Draw white box to put label text in
            cv2.putText(frame, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1) # Draw label text

            # Basic example: count the number of objects in the image
            object_count = object_count + 1
            
            # If detection is active, check for valid chit denominations
            if detection_active:
                try:
                    # Try to extract denomination from class name (e.g., "5", "10", "20", "50")
                    chit_value = int(classname)
                    if chit_value in VALID_CHITS:
                        # Update detected chit if confidence is higher
                        if conf > detection_confidence:
                            detected_chit_value = chit_value
                            detection_confidence = conf
                            print(f"Detected: ₱{chit_value} chit with {conf:.2%} confidence")
                            
                            # Update LCD with current best detection
                            elapsed = int(time.time() - detection_start_time)
                            lcd.display_lines(
                                "DETECTING...",
                                f"Chit: P{chit_value}",
                                f"Conf: {int(conf*100)}%",
                                f"Time: {elapsed}s"
                            )
                except ValueError:
                    # Class name is not a number, skip
                    pass

    # Calculate and draw framerate (if using video, USB, or Picamera source)
    if source_type == 'video' or source_type == 'usb' or source_type == 'picamera':
        cv2.putText(frame, f'FPS: {avg_frame_rate:0.2f}', (10,20), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2) # Draw framerate
    
    # Display detection results and status
    cv2.putText(frame, f'Objects: {object_count}', (10,40), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2)
    
    # Display IR sensor status
    ir_status = "DETECTED" if ir_detected else "NO CHIT"
    ir_color = (0,255,0) if ir_detected else (0,0,255)
    cv2.putText(frame, f'IR: {ir_status}', (10,60), cv2.FONT_HERSHEY_SIMPLEX, .7, ir_color, 2)
    
    # Display detection status
    if detection_active:
        status_text = f'DETECTING... ({int(time.time() - detection_start_time)}s)'
        if detected_chit_value:
            status_text = f'DETECTED: P{detected_chit_value} ({detection_confidence:.0%})'
        cv2.putText(frame, status_text, (10,80), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,0), 2)
    
    if use_gui:
        cv2.imshow('Chit Detection System', frame) # Display image
    if record: recorder.write(frame)

    # Wait for key press (5ms timeout)
    if use_gui:
        key = cv2.waitKey(5) & 0xFF
    else:
        key = -1
    
    if key == ord('q') or key == ord('Q'): # Press 'q' to quit
        break
    elif key == ord('s') or key == ord('S'): # Press 's' to pause inference
        if use_gui: cv2.waitKey()
    elif key == ord('p') or key == ord('P'): # Press 'p' to save a picture of results on this frame
        cv2.imwrite('capture.png',frame)
    
    # Calculate FPS for this frame
    t_stop = time.perf_counter()
    frame_rate_calc = float(1/(t_stop - t_start))

    # Append FPS result to frame_rate_buffer (for finding average FPS over multiple frames)
    if len(frame_rate_buffer) >= fps_avg_len:
        temp = frame_rate_buffer.pop(0)
        frame_rate_buffer.append(frame_rate_calc)
    else:
        frame_rate_buffer.append(frame_rate_calc)

    # Calculate average FPS for past frames
    avg_frame_rate = np.mean(frame_rate_buffer)


# Clean up
print(f'\nShutting down...')
print(f'Average pipeline FPS: {avg_frame_rate:.2f}')

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
if use_gui:
    cv2.destroyAllWindows()

print("System shutdown complete.")
