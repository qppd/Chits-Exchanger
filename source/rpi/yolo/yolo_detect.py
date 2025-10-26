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
parser.add_argument('--camera_ip', help='ESP32-CAM IP address',
                    default='192.168.1.21')

args = parser.parse_args()

# ESP32-CAM HTTP stream URL
img_source = f'http://{args.camera_ip}/stream'

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

# Set source type to RTSP stream
source_type = 'rtsp'
print(f'Connecting to ESP32-CAM RTSP stream at: {img_source}')

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
                return esp32_serial.readline().decode().strip()
        except Exception as e:
            print(f"Error reading from ESP32: {e}")
    return None

def is_ir_detected():
    """Check if IR sensor detects a chit"""
    return not GPIO.input(IR_SENSOR_PIN)  # Active LOW

# Initialize servo to home position
set_servo_angle(SERVO_INITIAL_ANGLE)
print(f"Servo initialized to {SERVO_INITIAL_ANGLE}°")

# Initialize RTSP stream connection
cap = cv2.VideoCapture(img_source)

# Set camera resolution if specified by user
if user_res:
    ret = cap.set(3, resW)
    ret = cap.set(4, resH)

# Check if connection is successful
if not cap.isOpened():
    print(f"Failed to connect to ESP32-CAM stream at {img_source}")
    sys.exit(1)

print("Successfully connected to ESP32-CAM stream")

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

# Begin YOLO detection using ESP32-CAM RTSP stream
print("Connected to ESP32-CAM. Running detection...")
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
    
    # Stop detection if IR sensor no longer detects or timeout
    if detection_active and (not ir_detected or (time.time() - detection_start_time > DETECTION_TIMEOUT)):
        if detected_chit_value:
            print(f"\n=== DETECTION COMPLETE ===")
            print(f"Detected: ₱{detected_chit_value} chit (confidence: {detection_confidence:.2%})")
            
            # Send detection result to ESP32
            send_to_esp32(f"CHIT_DETECTED:{detected_chit_value}")
            
            # Release the chit
            release_chit()
            
            # Notify ESP32 that chit was released
            send_to_esp32(f"CHIT_RELEASED:{detected_chit_value}")
            
            print(f"Chit ₱{detected_chit_value} released successfully")
        elif time.time() - detection_start_time > DETECTION_TIMEOUT:
            print("Detection timeout - no valid chit detected")
            send_to_esp32("DETECTION_TIMEOUT")
        
        detection_active = False
        detected_chit_value = None
        detection_confidence = 0.0
        print("Waiting for next chit...\n")
    
    last_ir_state = ir_detected

    t_start = time.perf_counter()

    # Read frame from RTSP stream
    ret, frame = cap.read()
    if not ret or frame is None:
        print('Unable to read frame from ESP32-CAM stream. Attempting to reconnect...')
        cap.release()
        cap = cv2.VideoCapture(img_source)
        if not cap.isOpened():
            print('Reconnection failed. Exiting program.')
            break
        continue

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

# Clean up GPIO and servo
set_servo_angle(SERVO_INITIAL_ANGLE)
pi.set_servo_pulsewidth(SERVO_PIN, 0)  # Stop servo
pi.stop()
GPIO.cleanup()

# Close serial connection
if esp32_serial and esp32_serial.is_open:
    send_to_esp32("SYSTEM_SHUTDOWN")
    esp32_serial.close()

cap.release()
if record: recorder.release()
if use_gui:
    cv2.destroyAllWindows()

print("System shutdown complete.")
