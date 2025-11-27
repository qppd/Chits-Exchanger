import os
import sys
import argparse
import glob
import time
import serial
import serial.tools.list_ports
import RPi.GPIO as GPIO
import pigpio
import threading
from queue import Queue, Empty

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

# Auto-detection functions
def find_camera():
    """Auto-detect USB camera"""
    print("Auto-detecting USB camera...")
    for i in range(10):  # Check /dev/video0 to /dev/video9
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, frame = cap.read()
            cap.release()
            if ret and frame is not None:
                print(f"✓ Found camera at /dev/video{i}")
                return i
    print("✗ No camera found")
    return None

def find_esp32_port():
    """Auto-detect ESP32 serial port"""
    print("Auto-detecting ESP32 serial port...")
    ports = serial.tools.list_ports.comports()
    for port in ports:
        # Look for common ESP32 identifiers
        if 'USB' in port.device or 'ACM' in port.device or 'ttyUSB' in port.device:
            try:
                # Try to open port
                ser = serial.Serial(port.device, 115200, timeout=1)
                ser.close()
                print(f"✓ Found serial port: {port.device}")
                return port.device
            except:
                pass
    print("✗ No ESP32 serial port found")
    return None

# Define and parse user input arguments
parser = argparse.ArgumentParser()
parser.add_argument('--model', help='Path to YOLO model file',
                    required=True)
parser.add_argument('--thresh', help='Minimum confidence threshold',
                    default=0.7)
parser.add_argument('--resolution', help='Resolution in WxH (example: "320x240")',
                    default=None)
parser.add_argument('--esp32_port', help='Serial port for ESP32 (auto-detect if not specified)',
                    default=None)
parser.add_argument('--camera', help='USB camera device ID (auto-detect if not specified)',
                    default=None)

args = parser.parse_args()

# Auto-detect camera if not specified
if args.camera is None:
    camera_id = find_camera()
    if camera_id is None:
        print("ERROR: No camera found. Exiting.")
        sys.exit(1)
    img_source = camera_id
else:
    img_source = int(args.camera)

# Auto-detect ESP32 port if not specified
if args.esp32_port is None:
    esp32_port = find_esp32_port()
    if esp32_port is None:
        print("WARNING: No ESP32 found. Continuing without serial communication.")
else:
    esp32_port = args.esp32_port

# GPIO Pin Configurations for RPi
IR_SENSOR_PIN = 17  # IR sensor input
SERVO_PIN = 22      # Servo control pin

# Servo angles
SERVO_INITIAL_ANGLE = 39
SERVO_RELEASE_ANGLE = 90

# Valid chit denominations
VALID_CHITS = [5, 10, 20, 50]

# Thread-safe queues for communication
serial_tx_queue = Queue()  # Queue for outgoing serial messages
serial_rx_queue = Queue()  # Queue for incoming serial messages

# Thread control flags
running = True
inference_enabled = True  # Simple flag to control detection (like your working code)

# Parse user inputs
model_path = args.model
min_thresh = args.thresh
user_res = args.resolution

# Check if model file exists and is valid
if (not os.path.exists(model_path)):
    print('ERROR: Model path is invalid or model was not found. Make sure the model filename was entered correctly.')
    sys.exit(0)

print(f"\n{'='*60}")
print(f"CHIT DETECTION SYSTEM - STARTUP")
print(f"{'='*60}")
print(f"Model: {model_path}")
print(f"Camera: /dev/video{img_source}")
print(f"ESP32 Port: {esp32_port if esp32_port else 'Not connected'}")
print(f"{'='*60}\n")

# Parse user-specified display resolution
resize = False
if user_res:
    resize = True
    resW, resH = int(user_res.split('x')[0]), int(user_res.split('x')[1])

# Initialize GPIO for IR sensor and Servo
print("Initializing GPIO...")
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(IR_SENSOR_PIN, GPIO.IN)
print("âœ… GPIO initialized")

# Initialize pigpio for servo control
print("Initializing servo control...")
pi = pigpio.pi()
if not pi.connected:
    print("ERROR: Could not connect to pigpio daemon. Run 'sudo pigpiod' first.")
    sys.exit(1)

# Initialize serial communication with ESP32
esp32_serial = None
if esp32_port:
    try:
        print(f"Initializing serial connection to ESP32 on {esp32_port}...")
    
        # Open serial port with proper settings
        # IMPORTANT: Disable DTR/RTS to prevent ESP32 auto-reset
        esp32_serial = serial.Serial()
        esp32_serial.port = esp32_port
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
        
        print(f"Serial connection to ESP32 established successfully")
        print(f"   Port: {esp32_port}")
        print(f"   Baud: 115200")
        print(f"   Mode: Non-blocking (DTR/RTS disabled)")
    
    except serial.SerialException as e:
        print(f"Serial connection failed: {e}")
        print(f"   Please check:")
        print(f"   1. ESP32 is connected to {esp32_port}")
        print(f"   2. User has permissions (run: sudo usermod -a -G dialout $USER)")
        print(f"   3. No other program is using the port")
        print("WARNING: Continuing without ESP32 serial communication...")
        esp32_serial = None
    except Exception as e:
        print(f"Unexpected error initializing serial: {e}")
        print("WARNING: Continuing without ESP32 serial communication...")
        esp32_serial = None
else:
    print("Skipping ESP32 serial initialization (not detected)")

# Initialize I2C LCD Display
print("\nInitializing LCD display...")
lcd = LCD()
if lcd.enabled:
    lcd.clear()
    lcd.display_lines(
        "Chit Detection",
        "System Initializing",
        "Loading model...",
        "Please wait..."
    )
    print("LCD ready - showing loading screen")
else:
    print("LCD display not available - continuing without LCD")

# Load the model into memory and get labelmap
print(f"\n⏳ Loading YOLO model: {model_path}")
print("   This may take 10-30 seconds depending on model size...")

# Show progress on LCD with animation
if lcd.enabled:
    import threading
    stop_animation = threading.Event()
    
    def animate_loading():
        """Animate loading screen on LCD"""
        spinner = ['|', '/', '-', '\\']
        idx = 0
        while not stop_animation.is_set():
            lcd.display_lines(
                "Loading Model",
                model_path.split('/')[-1],
                f"Please wait... {spinner[idx % 4]}",
                f"Time: {int(time.time() - model_load_start)}s"
            )
            idx += 1
            time.sleep(0.5)
    
    model_load_start = time.time()
    anim_thread = threading.Thread(target=animate_loading, daemon=True)
    anim_thread.start()
else:
    model_load_start = time.time()

print("   ⏳ Initializing model engine...")
model = YOLO(model_path, task='detect')
print("   ⏳ Loading weights and preparing inference...")
labels = model.names

# Stop animation
if lcd.enabled:
    stop_animation.set()
    time.sleep(0.6)  # Let animation thread finish

model_load_time = time.time() - model_load_start
print(f"✅ Model loaded successfully in {model_load_time:.2f} seconds")
print(f"   Detected classes: {list(labels.values())}")
if lcd.enabled:
    lcd.display_lines(
        "Model Loaded!",
        f"Time: {model_load_time:.1f}s",
        "Initializing...",
        ""
    )
    time.sleep(1)

# Define serial communication thread function
def serial_communication_thread():
    """Background thread for handling serial communication with ESP32"""
    global running, inference_enabled
    
    while running:
        # Handle outgoing messages (non-blocking)
        try:
            message = serial_tx_queue.get(timeout=0.01)
            if esp32_serial and esp32_serial.is_open:
                try:
                    esp32_serial.reset_output_buffer()
                    full_message = message + '\\n'
                    esp32_serial.write(full_message.encode('utf-8'))
                    esp32_serial.flush()
                    print(f"Sent to ESP32: {message}")
                except Exception as e:
                    print(f"Error sending: {e}")
        except Empty:
            pass
        
        # Handle incoming messages (non-blocking)
        if esp32_serial and esp32_serial.is_open:
            try:
                if esp32_serial.in_waiting > 0:
                    message = esp32_serial.readline().decode('utf-8', errors='ignore').strip()
                    if message:
                        serial_rx_queue.put(message)
                        print(f"ESP32: {message}")
                        
                        # Handle special messages
                        if "DISPENSING_COMPLETE" in message and lcd.enabled:
                            lcd.display_lines("ESP32:", "Dispensing", "Complete!", "")
            except Exception as e:
                if running:
                    print(f"Error reading: {e}")
        
        time.sleep(0.01)

# Start serial communication thread
if esp32_serial:
    serial_thread = threading.Thread(target=serial_communication_thread, daemon=True, name="SerialComm")
    serial_thread.start()
    print("Serial communication thread started")
    time.sleep(0.1)

# Keyboard input thread for sending commands to ESP32
def keyboard_input_thread():
    """Background thread for keyboard input to send commands to ESP32"""
    import select
    import sys
    
    print("\n" + "="*60)
    print("KEYBOARD COMMANDS:")
    print("  Type commands and press ENTER to send to ESP32")
    print("  Examples: test_hopper 1 3, test_chit 50, help")
    print("  Press Ctrl+C to exit")
    print("="*60 + "\n")
    
    while running:
        try:
            # Check if input is available (non-blocking on Linux)
            if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0]:
                command = sys.stdin.readline().strip()
                if command:
                    print(f">>> Sending to ESP32: {command}")
                    send_to_esp32(command)
        except Exception as e:
            if running:
                print(f"Keyboard input error: {e}")
            break

# Start keyboard input thread (only if ESP32 is connected)
if esp32_serial:
    kbd_thread = threading.Thread(target=keyboard_input_thread, daemon=True, name="KeyboardInput")
    kbd_thread.start()

# Helper functions for serial communication
def send_to_esp32(message):
    """Queue message to be sent to ESP32 via serial (non-blocking)"""
    serial_tx_queue.put(message)
    return True

def read_from_esp32():
    """Get message from ESP32 queue (non-blocking, returns None if empty)"""
    try:
        return serial_rx_queue.get_nowait()
    except Empty:
        return None

# Set source type to USB webcam
source_type = 'usb'

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

def is_ir_detected():
    """Check if IR sensor detects a chit"""
    return not GPIO.input(IR_SENSOR_PIN)  # Active LOW

# Initialize servo to home position
set_servo_angle(SERVO_INITIAL_ANGLE)
print(f"Servo initialized to {SERVO_INITIAL_ANGLE}Â°")

# Initialize USB webcam connection
cap = cv2.VideoCapture(img_source)

# Set camera resolution if specified by user
if user_res:
    ret = cap.set(3, resW)
    ret = cap.set(4, resH)

# Check if connection is successful
if not cap.isOpened():
    print(f"Failed to open USB camera /dev/video{img_source}")
    sys.exit(1)

print(f"Successfully connected to USB camera at /dev/video{img_source}")

# Set bounding box colors (using the Tableu 10 color scheme)
bbox_colors = [(164,120,87), (68,148,228), (93,97,209), (178,182,133), (88,159,106), 
              (96,202,231), (159,124,168), (169,162,241), (98,118,150), (172,176,184)]

# Detection state variables
last_ir_state = False
detection_enabled = False  # Flag to enable/disable detection

# Real-time detection tracking
CONFIRMATION_FRAMES = 3  # Reduced from 5 to 3 for faster detection
CONFIDENCE_THRESHOLD = 0.6  # Increased from 0.5 for more reliable detections
FRAME_SKIP = 2  # Process every 3rd frame (skip 2 frames)
detection_buffer = []  # Buffer to store recent detections

# FPS tracking
frame_count = 0
fps_start_time = time.time()
fps = 0.0
frame_skip_counter = 0


# Helper function for real-time detection with frame processing
def process_frame_detection(frame):
    """Process a single frame and return detection results"""
    # Run inference
    results = model(frame, verbose=False)
    detections = results[0].boxes
    
    detected_chits = []
    display_frame = frame.copy()
    
    # Process detections and draw on frame
    for i in range(len(detections)):
        conf = detections[i].conf.item()
        
        if conf > CONFIDENCE_THRESHOLD:
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
                    detected_chits.append((chit_value, conf))
            except ValueError:
                pass
    
    return display_frame, detected_chits


def get_confirmed_detection():
    """Analyze detection buffer and return confirmed chit value if consistent"""
    if len(detection_buffer) < CONFIRMATION_FRAMES:
        return None, 0.0
    
    # Count occurrences of each detected value in recent frames
    value_counts = {}
    value_confidences = {}
    
    for chit_value, conf in detection_buffer[-CONFIRMATION_FRAMES:]:
        if chit_value not in value_counts:
            value_counts[chit_value] = 0
            value_confidences[chit_value] = []
        value_counts[chit_value] += 1
        value_confidences[chit_value].append(conf)
    
    # Find most common value
    if value_counts:
        most_common_value = max(value_counts.items(), key=lambda x: x[1])
        chit_value, count = most_common_value
        
        # Require majority of frames to agree
        if count >= CONFIRMATION_FRAMES * 0.6:  # 60% threshold
            avg_confidence = sum(value_confidences[chit_value]) / len(value_confidences[chit_value])
            return chit_value, avg_confidence
    
    return None, 0.0

# Begin YOLO detection using USB webcam
print(f"\n{'='*60}")
print("✅ SYSTEM INITIALIZATION COMPLETE")
print(f"{'='*60}")
print(f"Camera: {img_source}")
print(f"IR Sensor: GPIO {IR_SENSOR_PIN}")
print(f"Servo: GPIO {SERVO_PIN}")
print(f"Detection mode: Real-time continuous detection")
print(f"Confirmation frames: {CONFIRMATION_FRAMES}")
print(f"Confidence threshold: {CONFIDENCE_THRESHOLD}")
print(f"Frame skip rate: {FRAME_SKIP} (process every {FRAME_SKIP+1} frames)")
print(f"{'='*60}")
print("\nðŸŽ‰ System ready for operation!")
print("Real-time detection running. Insert chit when ready.")
print("Waiting for chit insertion...\n")

# Update LCD to ready state
if lcd.enabled:
    lcd.display_lines(
        "System Ready!",
        "Real-time mode",
        "Insert chit",
        ""
    )
    time.sleep(2)
    lcd.display_lines(
        "Ready - Scanning",
        "Waiting for chit...",
        "",
        ""
    )

# Enable detection flag after loading
detection_enabled = True

# State machine for detection
detection_state = "WAITING"  # WAITING, DETECTING, CONFIRMED, DISPENSING
confirmed_chit_value = None
confirmed_confidence = 0.0

# Begin monitoring loop with real-time detection
print("Starting real-time detection loop...")
while True:
    
    # Check for messages from ESP32
    esp32_msg = read_from_esp32()
    if esp32_msg:
        print(f"ESP32: {esp32_msg}")
    
    # Capture and process frame continuously
    ret, frame = cap.read()
    if not ret or frame is None:
        print("âŒ Failed to capture frame")
        time.sleep(0.1)
        continue
    
    # Resize if needed
    if resize:
        frame = cv2.resize(frame, (resW, resH))
    
    # Calculate FPS
    frame_count += 1
    if frame_count % 30 == 0:
        fps_end_time = time.time()
        fps = 30 / (fps_end_time - fps_start_time)
        fps_start_time = fps_end_time
    
    # Check IR sensor state
    ir_detected = is_ir_detected()
    
    # State machine logic
    if detection_state == "WAITING":
        # Process frame for real-time detection (with frame skipping)
        frame_skip_counter += 1
        if frame_skip_counter >= FRAME_SKIP:
            frame_skip_counter = 0
            display_frame, detected_chits = process_frame_detection(frame)
        else:
            # Use previous frame data or skip processing
            display_frame = frame.copy()
            detected_chits = []
        
        # Add FPS and status to display
        cv2.putText(display_frame, f'FPS: {fps:.1f}', (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(display_frame, 'Status: Waiting', (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        
        # Show real-time view
        if use_gui:
            cv2.imshow('Chit Detection - Real-time', display_frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("\nQuitting...")
                running = False
                break
        
        # Check if IR sensor triggered
        if ir_detected and not last_ir_state and detection_enabled:
            print(f"\n{'='*60}")
            print(f"ðŸ” IR SENSOR TRIGGERED - STARTING DETECTION")
            print(f"{'='*60}")
            
            send_to_esp32("IR_DETECTED")
            
            # Update LCD
            lcd.display_lines(
                "IR DETECTED!",
                "Detecting chit...",
                "Hold steady...",
                ""
            )
            
            detection_state = "DETECTING"
            detection_buffer.clear()
            t_detect_start = time.perf_counter()
    
    elif detection_state == "DETECTING":
        # Process frame for detection (with frame skipping for better performance)
        frame_skip_counter += 1
        if frame_skip_counter >= FRAME_SKIP:
            frame_skip_counter = 0
            display_frame, detected_chits = process_frame_detection(frame)
        else:
            # Use previous frame data or skip processing
            display_frame = frame.copy()
            detected_chits = []
        
        # Add FPS and status to display
        cv2.putText(display_frame, f'FPS: {fps:.1f}', (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(display_frame, 'Status: Detecting...', (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(display_frame, f'Buffer: {len(detection_buffer)}/{CONFIRMATION_FRAMES}', 
                   (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # Show real-time view
        if use_gui:
            cv2.imshow('Chit Detection - Real-time', display_frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("\nQuitting...")
                running = False
                break
        
        # Add best detection from this frame to buffer
        if detected_chits:
            best_detection = max(detected_chits, key=lambda x: x[1])
            detection_buffer.append(best_detection)
            print(f"   Frame detection: â‚±{best_detection[0]} | Conf: {best_detection[1]:.2%}")
        else:
            # Add None to buffer if no detection
            detection_buffer.append((None, 0.0))
        
        # Keep buffer size limited
        if len(detection_buffer) > CONFIRMATION_FRAMES * 2:
            detection_buffer.pop(0)
        
        # Check for confirmed detection
        confirmed_value, avg_conf = get_confirmed_detection()
        
        if confirmed_value:
            print(f"\n{'='*60}")
            print(f"âœ… DETECTION CONFIRMED!")
            print(f"{'='*60}")
            print(f"   Detected Value: â‚±{confirmed_value}")
            print(f"   Average Confidence: {avg_conf:.2%}")
            print(f"   Frames analyzed: {len(detection_buffer)}")
            print(f"{'='*60}")
            
            confirmed_chit_value = confirmed_value
            confirmed_confidence = avg_conf
            detection_state = "CONFIRMED"
        
        # Timeout if no IR detection after some time
        elif not ir_detected and (time.perf_counter() - t_detect_start) > 3.0:
            print(f"\n{'='*60}")
            print(f"âŒ DETECTION TIMEOUT - No consistent detection")
            print(f"{'='*60}")
            
            send_to_esp32("DETECTION_TIMEOUT")
            
            # Update LCD
            lcd.display_lines(
                "TIMEOUT!",
                "No valid chit",
                "detected",
                "Try again..."
            )
            time.sleep(2)
            
            lcd.display_lines(
                "Ready - Scanning",
                "Waiting for chit...",
                "",
                ""
            )
            
            detection_state = "WAITING"
            detection_buffer.clear()
    
    elif detection_state == "CONFIRMED":
        # Detection confirmed, now dispense
        t_detect_stop = time.perf_counter()
        detection_time = t_detect_stop - t_detect_start
        
        print(f"\n{'='*60}")
        print(f"ðŸŽ‰ DETECTION COMPLETE")
        print(f"{'='*60}")
        print(f"   Detected Value: â‚±{confirmed_chit_value}")
        print(f"   Confidence: {confirmed_confidence:.2%}")
        print(f"   Detection Time: {detection_time:.3f}s")
        print(f"{'='*60}")
        
        # Update LCD
        lcd.display_lines(
            "DETECTED!",
            f"Value: P{confirmed_chit_value}",
            f"Conf: {int(confirmed_confidence*100)}%",
            "Releasing chit..."
        )
        
        # Send detection result to ESP32
        send_to_esp32(f"CHIT_DETECTED:{confirmed_chit_value}")
        
        # Release the chit
        print(f"ðŸ”“ Releasing chit via servo...")
        release_chit()
        print(f"âœ… Chit â‚±{confirmed_chit_value} released successfully")
        
        # Auto-dispense: Send command to ESP32 to dispense detected amount
        print(f"\n{'='*60}")
        print(f"ðŸª™ AUTO-DISPENSING TRIGGERED")
        print(f"{'='*60}")
        print(f"   Sending to ESP32: AUTO_DISPENSE:{confirmed_chit_value}")
        print(f"   Expected dispensing:")
        
        # Show expected coin breakdown
        if confirmed_chit_value == 5:
            print(f"     - 1 x 5 PHP coin (Hopper 1)")
        elif confirmed_chit_value == 10:
            print(f"     - 1 x 10 PHP coin (Hopper 2)")
        elif confirmed_chit_value == 20:
            print(f"     - 1 x 20 PHP coin (Hopper 3)")
        elif confirmed_chit_value == 50:
            print(f"     - 2 x 20 PHP coins (Hopper 3)")
            print(f"     - 1 x 10 PHP coin (Hopper 2)")
        
        print(f"{'='*60}\n")
        
        # Send the command
        if send_to_esp32(f"AUTO_DISPENSE:{confirmed_chit_value}"):
            # Update LCD
            lcd.display_lines(
                "AUTO DISPENSE!",
                f"Chit: P{confirmed_chit_value}",
                "Dispensing...",
                "Please wait"
            )
            time.sleep(3)
        else:
            print(f"âš ï¸  Failed to send AUTO_DISPENSE command")
            lcd.display_lines(
                "ERROR!",
                "Communication",
                "failed",
                ""
            )
            time.sleep(2)
        
        print("Waiting for next chit...\n")
        
        # Reset LCD to waiting state
        lcd.display_lines(
            "Ready - Scanning",
            "Waiting for chit...",
            "",
            ""
        )
        
        # Reset state machine
        detection_state = "WAITING"
        detection_buffer.clear()
        confirmed_chit_value = None
        confirmed_confidence = 0.0
    
    last_ir_state = ir_detected
    
    # Small delay to prevent CPU spinning (very small for real-time)
    time.sleep(0.01)

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

# Stop serial communication thread
print("Stopping serial communication thread...")
running = False  # Signal thread to stop
time.sleep(0.3)  # Give thread time to finish

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

print("System shutdown complete.")

