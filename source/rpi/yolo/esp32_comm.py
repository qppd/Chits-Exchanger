#!/usr/bin/env python3
"""
ESP32 Communication Module - Slave System for Chit Detection and Dispensing
This module acts as a slave that receives commands from ESP32 CoinExchanger.ino

Commands received from ESP32:
- CHECK_IR - Check if IR sensor detects chit
- DETECT_CHIT - Run YOLO detection to identify chit denomination
- DISPLAY:<message> - Display message on LCD
- DISPENSE_CHIT - Control servo to release chit

Responses sent to ESP32:
- IR_DETECTED - IR sensor detected chit
- IR_CLEAR - No chit detected by IR
- CHIT_5 / CHIT_10 / CHIT_20 / CHIT_50 - Detected chit denomination
- CHIT_UNKNOWN - Could not identify chit
- DISPENSE_COMPLETE - Servo finished dispensing
- DISPLAY_OK - LCD updated successfully
- ERROR:<message> - Error occurred

Hardware:
- IR Sensor: GPIO17
- Servo Motor: GPIO22 (angles: 39° initial, 90° release)
- LCD: I2C 0x27 (20x4)
- Serial: /dev/serial0 (connected to ESP32)

Author: Chits Exchanger System
Date: November 2025
"""

import os
import sys
import time
import signal
import serial
import serial.tools.list_ports
import threading
from pathlib import Path
import importlib.util

# GPIO and Hardware Libraries
try:
    import RPi.GPIO as GPIO
    import pigpio
    import smbus2 as smbus
except ImportError as e:
    print(f"Error importing hardware libraries: {e}")
    print("Make sure RPi.GPIO, pigpio, and smbus2 are installed")
    sys.exit(1)

# YOLO imports
try:
    import cv2
    import numpy as np
    from ultralytics import YOLO
except ImportError as e:
    print(f"Error importing YOLO libraries: {e}")
    print("Make sure opencv-python and ultralytics are installed")
    sys.exit(1)

# ==================== AUTO-DETECTION FUNCTIONS ====================

def find_camera():
    """Auto-detect USB camera"""
    print("[AUTO-DETECT] Searching for USB camera...")
    for i in range(10):  # Check /dev/video0 to /dev/video9
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, frame = cap.read()
            cap.release()
            if ret and frame is not None:
                print(f"[AUTO-DETECT] ✓ Found camera at /dev/video{i}")
                return i
    print("[AUTO-DETECT] ✗ No camera found")
    return None

def find_esp32_port():
    """Auto-detect ESP32 serial port"""
    print("[AUTO-DETECT] Searching for ESP32 serial port...")
    ports = serial.tools.list_ports.comports()
    for port in ports:
        # Look for common ESP32/serial identifiers
        if 'USB' in port.device or 'ACM' in port.device or 'ttyUSB' in port.device or 'serial' in port.device:
            try:
                # Try to open port
                ser = serial.Serial(port.device, 115200, timeout=1)
                ser.close()
                print(f"[AUTO-DETECT] ✓ Found serial port: {port.device}")
                return port.device
            except:
                pass
    print("[AUTO-DETECT] ✗ No ESP32 serial port found")
    return None

# ==================== CONFIGURATION ====================

# GPIO Pins (BCM numbering)
IR_SENSOR_PIN = 17
SERVO_PIN = 22

# Servo angles
SERVO_INITIAL_ANGLE = 39
SERVO_RELEASE_ANGLE = 90

# LCD Configuration (optional - system works without it)
LCD_I2C_ADDR = 0x27
LCD_WIDTH = 20
LCD_ROWS = 4
LCD_ENABLED = True  # Set to False to disable LCD

# Serial Configuration
SERIAL_PORT = '/dev/ttyUSB0'  # Default, will auto-detect if not found
SERIAL_BAUD = 115200

# YOLO Configuration
MODEL_PATH = '/home/pi/Desktop/PROJECTS/Chits-Exchanger/source/rpi/yolo/chit_model.pt'
CAMERA_INDEX = 0  # Default USB camera index, will auto-detect if not found
DETECTION_TIMEOUT = 5.0  # seconds
CONFIDENCE_THRESHOLD = 0.7

# ==================== LCD CLASS ====================

# LCD Commands
LCD_CHR = 1
LCD_CMD = 0
LCD_LINE_1 = 0x80
LCD_LINE_2 = 0xC0
LCD_LINE_3 = 0x94
LCD_LINE_4 = 0xD4
E_PULSE = 0.0005
E_DELAY = 0.0005

class LCD:
    """I2C LCD 20x4 Display Controller"""
    
    def __init__(self, addr=LCD_I2C_ADDR):
        self.bus = None
        self.addr = addr
        self.enabled = False
        
        try:
            self.bus = smbus.SMBus(1)
            self._initialize()
            self.enabled = True
            print("[LCD] ✓ Initialized")
        except Exception as e:
            print(f"[LCD] ⚠ Not available: {e}")
            print("[LCD] System will continue without LCD display")
            self.enabled = False
    
    def _initialize(self):
        """Initialize LCD display"""
        self.lcd_byte(0x33, LCD_CMD)
        self.lcd_byte(0x32, LCD_CMD)
        self.lcd_byte(0x06, LCD_CMD)
        self.lcd_byte(0x0C, LCD_CMD)
        self.lcd_byte(0x28, LCD_CMD)
        self.lcd_byte(0x01, LCD_CMD)
        time.sleep(E_DELAY)
    
    def lcd_byte(self, bits, mode):
        """Send byte to LCD"""
        if not self.enabled or not self.bus:
            return
            
        try:
            bits_high = mode | (bits & 0xF0) | 0x08
            bits_low = mode | ((bits << 4) & 0xF0) | 0x08
            
            self.bus.write_byte(self.addr, bits_high)
            self._toggle_enable(bits_high)
            
            self.bus.write_byte(self.addr, bits_low)
            self._toggle_enable(bits_low)
        except:
            self.enabled = False
    
    def _toggle_enable(self, bits):
        """Toggle enable bit"""
        if not self.enabled or not self.bus:
            return
            
        time.sleep(E_DELAY)
        self.bus.write_byte(self.addr, (bits | 0x04))
        time.sleep(E_PULSE)
        self.bus.write_byte(self.addr, (bits & ~0x04))
        time.sleep(E_DELAY)
    
    def display_line(self, message, line):
        """Display message on specific line (1-4)"""
        if not self.enabled:
            return
            
        line_addresses = {1: LCD_LINE_1, 2: LCD_LINE_2, 3: LCD_LINE_3, 4: LCD_LINE_4}
        if line not in line_addresses:
            return
        
        message = message.ljust(LCD_WIDTH, " ")[:LCD_WIDTH]
        self.lcd_byte(line_addresses[line], LCD_CMD)
        
        for char in message:
            self.lcd_byte(ord(char), LCD_CHR)
    
    def clear(self):
        """Clear LCD display"""
        if not self.enabled:
            return
            
        self.lcd_byte(0x01, LCD_CMD)
        time.sleep(E_DELAY)
    
    def display_message(self, line1="", line2="", line3="", line4=""):
        """Display multi-line message"""
        if not self.enabled:
            return
            
        if line1: self.display_line(line1, 1)
        if line2: self.display_line(line2, 2)
        if line3: self.display_line(line3, 3)
        if line4: self.display_line(line4, 4)

# ==================== HARDWARE CLASSES ====================

class IRSensor:
    """IR Obstacle Detection Sensor"""
    
    def __init__(self, pin=IR_SENSOR_PIN):
        self.pin = pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.IN)
    
    def is_detected(self):
        """Check if chit is detected (LOW = detected)"""
        return not GPIO.input(self.pin)
    
    def cleanup(self):
        """Cleanup GPIO"""
        GPIO.cleanup()

class ServoController:
    """Servo Motor Controller using pigpio"""
    
    def __init__(self, pin=SERVO_PIN):
        self.pin = pin
        self.pi = pigpio.pi()
        
        if not self.pi.connected:
            raise RuntimeError("Could not connect to pigpio daemon!")
        
        self.set_angle(SERVO_INITIAL_ANGLE)
    
    def angle_to_pulse(self, angle):
        """Convert angle (0-180) to pulse width (500-2500us)"""
        return int(500 + (angle / 180.0) * 2000)
    
    def set_angle(self, angle):
        """Set servo to specific angle"""
        pulse_width = self.angle_to_pulse(angle)
        self.pi.set_servo_pulsewidth(self.pin, pulse_width)
    
    def release_chit(self):
        """Release chit by rotating servo"""
        print(f"[SERVO] Moving to {SERVO_RELEASE_ANGLE}° (release)")
        self.set_angle(SERVO_RELEASE_ANGLE)
        time.sleep(1.0)  # Wait for chit to fall
        
        print(f"[SERVO] Returning to {SERVO_INITIAL_ANGLE}° (initial)")
        self.set_angle(SERVO_INITIAL_ANGLE)
        time.sleep(0.5)
    
    def cleanup(self):
        """Stop servo and cleanup"""
        self.pi.set_servo_pulsewidth(self.pin, 0)
        self.pi.stop()

class ChitDetector:
    """NCNN-based Chit Detection System (using chit_model_ncnn_model/model_ncnn.py)"""
    def __init__(self, model_path=None, camera_index=CAMERA_INDEX):
        self.camera_index = camera_index
        self.cap = None
        self.labels = {}
        self.model = None
        self.ncnn_module = None
        self.chit_mapping = {
            '5': 5,
            '10': 10,
            '20': 20,
            '50': 50
        }

    def load_model(self):
        """Load NCNN model from chit_model_ncnn_model/model_ncnn.py"""
        ncnn_path = os.path.join(os.path.dirname(__file__), 'chit_model_ncnn_model', 'model_ncnn.py')
        if not os.path.exists(ncnn_path):
            print(f"[ERROR] NCNN model file not found: {ncnn_path}")
            return False
        try:
            print(f"[NCNN] Loading model from: {ncnn_path}")
            spec = importlib.util.spec_from_file_location("model_ncnn", ncnn_path)
            ncnn_module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(ncnn_module)
            self.ncnn_module = ncnn_module
            if hasattr(ncnn_module, 'load_model'):
                self.model = ncnn_module.load_model()
            if hasattr(ncnn_module, 'LABELS'):
                self.labels = ncnn_module.LABELS
            print(f"[NCNN] Model loaded successfully. Labels: {self.labels}")
            return True
        except Exception as e:
            print(f"[ERROR] Failed to load NCNN model: {e}")
            return False

    def open_camera(self):
        """Open camera for detection (320x240)"""
        try:
            self.cap = cv2.VideoCapture(self.camera_index)
            if not self.cap.isOpened():
                print("[ERROR] Cannot open camera")
                return False
            # Set camera properties for 320x240
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
            print(f"[CAMERA] Opened USB camera index {self.camera_index} at 320x240")
            return True
        except Exception as e:
            print(f"[ERROR] Failed to open camera: {e}")
            return False

    def detect_chit(self, timeout=DETECTION_TIMEOUT):
        """
        Detect chit denomination using NCNN model
        Returns: (success, chit_value, confidence)
        """
        if not self.cap or not self.ncnn_module or not hasattr(self.ncnn_module, 'detect_chit'):
            return False, 0, 0.0
        print("[NCNN] Starting chit detection...")
        start_time = time.time()
        best_detection = None
        best_confidence = 0.0
        frames_analyzed = 0
        while (time.time() - start_time) < timeout:
            ret, frame = self.cap.read()
            if not ret or frame is None:
                print("[WARN] Failed to read frame")
                continue
            frames_analyzed += 1
            # Run NCNN inference (assume detect_chit returns (label, confidence))
            try:
                result = self.ncnn_module.detect_chit(self.model, frame)
                if result is not None:
                    class_name, confidence = result
                    class_name = str(class_name).lower()
                    print(f"[NCNN] Frame {frames_analyzed}: Detected '{class_name}' with confidence {confidence:.2f}")
                    if confidence > CONFIDENCE_THRESHOLD and confidence > best_confidence:
                        chit_value = self._map_class_to_value(class_name)
                        if chit_value > 0:
                            best_detection = chit_value
                            best_confidence = confidence
                            print(f"[NCNN] Best detection so far: {chit_value} peso ({confidence:.2%})")
            except Exception as e:
                print(f"[NCNN] Detection error: {e}")
            time.sleep(0.1)
        print(f"[NCNN] Detection complete. Analyzed {frames_analyzed} frames")
        if best_detection:
            print(f"[NCNN] Final result: {best_detection} peso with {best_confidence:.2%} confidence")
            return True, best_detection, best_confidence
        else:
            print("[NCNN] No valid chit detected")
            return False, 0, 0.0
    
    def _map_class_to_value(self, class_name):
        """Map YOLO class name to chit value"""
        class_name = class_name.lower().replace(' ', '').replace('_', '')
        return self.chit_mapping.get(class_name, 0)
    
    def release_camera(self):
        """Release camera resource"""
        if self.cap:
            self.cap.release()
            self.cap = None
            print("[CAMERA] Released")

# ==================== MAIN SLAVE CONTROLLER ====================

class ChitSlaveController:
    """Main slave controller that receives commands from ESP32"""
    
    def __init__(self):
        self.running = False
        self.serial = None
        
        # Initialize hardware
        self.lcd = None
        self.ir_sensor = None
        self.servo = None
        self.detector = None
        
        # State tracking
        self.last_ir_state = False
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        print("\n[SYSTEM] Shutdown signal received")
        self.shutdown()
        sys.exit(0)
    
    def initialize(self):
        """Initialize all hardware components"""
        print("\n" + "="*50)
        print("ESP32 Chit Slave Controller Initializing...")
        print("="*50)
        
        try:
            # Initialize LCD
            print("[LCD] Initializing...")
            if LCD_ENABLED:
                self.lcd = LCD()
                if self.lcd.enabled:
                    self.lcd.clear()
                    self.lcd.display_message("Chit Slave System", "Initializing...", "", "")
            else:
                print("[LCD] Disabled in configuration")
                self.lcd = None
            print("[LCD] ✓ Initialized")
            
            # Initialize IR Sensor
            print("[IR] Initializing...")
            self.ir_sensor = IRSensor()
            print("[IR] ✓ Initialized")
            
            # Initialize Servo
            print("[SERVO] Initializing...")
            self.servo = ServoController()
            print("[SERVO] ✓ Initialized")
            
            # Initialize YOLO Detector
            print("[YOLO] Initializing...")
            
            # Auto-detect camera if default doesn't work
            camera_index = CAMERA_INDEX
            test_cap = cv2.VideoCapture(camera_index)
            if not test_cap.isOpened():
                print(f"[CAMERA] Default camera {camera_index} not available, auto-detecting...")
                detected_camera = find_camera()
                if detected_camera is not None:
                    camera_index = detected_camera
                else:
                    raise RuntimeError("No camera detected")
            test_cap.release()
            
            self.detector = ChitDetector(camera_index=camera_index)
            if not self.detector.load_model():
                raise RuntimeError("Failed to load YOLO model")
            print("[YOLO] ✓ Model loaded")
            
            # Open camera
            if not self.detector.open_camera():
                raise RuntimeError("Failed to open camera")
            print("[CAMERA] ✓ Opened")
            
            # Initialize Serial Communication
            print("[SERIAL] Initializing...")
            
            # Try default port first
            serial_port = SERIAL_PORT
            try:
                self.serial = serial.Serial(
                    port=serial_port,
                    baudrate=SERIAL_BAUD,
                    timeout=1.0,
                    write_timeout=1.0
                )
            except (serial.SerialException, FileNotFoundError):
                # Auto-detect if default port fails
                print(f"[SERIAL] Default port {serial_port} not available, auto-detecting...")
                detected_port = find_esp32_port()
                if detected_port is None:
                    raise RuntimeError("No ESP32 serial port detected")
                serial_port = detected_port
                self.serial = serial.Serial(
                    port=serial_port,
                    baudrate=SERIAL_BAUD,
                    timeout=1.0,
                    write_timeout=1.0
                )
            
            time.sleep(2)  # Wait for serial to stabilize
            print(f"[SERIAL] ✓ Connected on {serial_port} @ {SERIAL_BAUD} baud")
            
            # Display ready status
            self.lcd.clear()
            self.lcd.display_message("Slave System Ready", "Waiting for ESP32", "commands...", "")
            
            print("\n" + "="*50)
            print("✓ ALL SYSTEMS INITIALIZED")
            print("="*50 + "\n")
            
            # Send ready signal to ESP32
            self.send_to_esp32("SLAVE_READY")
            
            return True
            
        except Exception as e:
            print(f"\n[ERROR] Initialization failed: {e}")
            if self.lcd and hasattr(self.lcd, 'display_message'):
                self.lcd.display_message("INIT ERROR", str(e)[:20], "", "")
            return False
    
    def send_to_esp32(self, message):
        """Send message to ESP32 via serial"""
        if self.serial and self.serial.is_open:
            try:
                self.serial.write(f"{message}\n".encode())
                self.serial.flush()
                print(f"[TX → ESP32] {message}")
            except Exception as e:
                print(f"[ERROR] Failed to send to ESP32: {e}")
    
    def handle_command(self, command):
        """Handle commands received from ESP32"""
        command = command.strip()
        if not command:
            return
        
        print(f"[RX ← ESP32] {command}")
        
        # Parse command
        if command == "CHECK_IR":
            self.cmd_check_ir()
        
        elif command == "DETECT_CHIT":
            self.cmd_detect_chit()
        
        elif command.startswith("DISPLAY:"):
            message = command[8:]  # Remove "DISPLAY:" prefix
            self.cmd_display(message)
        
        elif command == "DISPENSE_CHIT":
            self.cmd_dispense()
        
        elif command == "PING":
            self.send_to_esp32("PONG")
        
        elif command == "RESET":
            self.cmd_reset()
        
        else:
            print(f"[WARN] Unknown command: {command}")
            self.send_to_esp32(f"ERROR:Unknown command '{command}'")
    
    def cmd_check_ir(self):
        """Check IR sensor status"""
        try:
            is_detected = self.ir_sensor.is_detected()
            
            if is_detected:
                self.send_to_esp32("IR_DETECTED")
                self.lcd.display_line("IR: Chit detected", 4)
            else:
                self.send_to_esp32("IR_CLEAR")
                self.lcd.display_line("IR: Clear", 4)
                
        except Exception as e:
            print(f"[ERROR] IR check failed: {e}")
            self.send_to_esp32(f"ERROR:IR check failed")
    
    def cmd_detect_chit(self):
        """Run YOLO detection to identify chit"""
        try:
            self.lcd.clear()
            self.lcd.display_message("Detecting chit...", "Using YOLO", "Please wait...", "")
            
            success, chit_value, confidence = self.detector.detect_chit()
            
            if success and chit_value > 0:
                # Send result to ESP32
                self.send_to_esp32(f"CHIT_{chit_value}")
                
                # Update LCD
                self.lcd.clear()
                self.lcd.display_message(
                    "Chit Detected!",
                    f"Value: P{chit_value}",
                    f"Confidence: {confidence:.0%}",
                    ""
                )
            else:
                self.send_to_esp32("CHIT_UNKNOWN")
                self.lcd.clear()
                self.lcd.display_message("Detection Failed", "No valid chit", "detected", "")
                
        except Exception as e:
            print(f"[ERROR] Detection failed: {e}")
            self.send_to_esp32(f"ERROR:Detection failed")
            self.lcd.display_message("ERROR", "Detection failed", "", "")
    
    def cmd_display(self, message):
        """Display message on LCD"""
        try:
            # Parse multi-line message (lines separated by '|')
            lines = message.split('|')
            
            self.lcd.clear()
            for i, line in enumerate(lines[:4], start=1):
                self.lcd.display_line(line, i)
            
            self.send_to_esp32("DISPLAY_OK")
            
        except Exception as e:
            print(f"[ERROR] Display failed: {e}")
            self.send_to_esp32(f"ERROR:Display failed")
    
    def cmd_dispense(self):
        """Control servo to dispense chit"""
        try:
            self.lcd.display_line("Dispensing chit...", 4)
            
            self.servo.release_chit()
            
            self.send_to_esp32("DISPENSE_COMPLETE")
            self.lcd.display_line("Dispense complete", 4)
            
        except Exception as e:
            print(f"[ERROR] Dispense failed: {e}")
            self.send_to_esp32(f"ERROR:Dispense failed")
            self.lcd.display_line("Dispense ERROR", 4)
    
    def cmd_reset(self):
        """Reset system to initial state"""
        try:
            self.lcd.clear()
            self.lcd.display_message("Slave System Ready", "Waiting for ESP32", "commands...", "")
            self.send_to_esp32("RESET_OK")
        except Exception as e:
            print(f"[ERROR] Reset failed: {e}")
            self.send_to_esp32(f"ERROR:Reset failed")
    
    def run(self, debug_stdin=False):
        """Main run loop - listen for commands from ESP32 and show camera feed. If debug_stdin is True, also read commands from stdin."""
        if not self.initialize():
            print("[FATAL] Initialization failed. Exiting.")
            return

        self.running = True
        print("\n[SYSTEM] Slave controller running. Listening for ESP32 commands...")
        print("[SYSTEM] Press Ctrl+C to shutdown\n")

        # For imshow, get the camera object from detector
        cap = self.detector.cap if self.detector else None
        window_name = "ChitExchanger Camera"
        if cap is not None:
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window_name, 640, 480)

        import select
        while self.running:
            try:
                # Show camera feed if available
                if cap is not None:
                    ret, frame = cap.read()
                    if ret and frame is not None:
                        cv2.imshow(window_name, frame)
                        # Allow window to refresh and check for ESC to quit
                        if cv2.waitKey(1) & 0xFF == 27:
                            print("[SYSTEM] ESC pressed, shutting down...")
                            self.running = False
                            break
                # Read command from ESP32 serial
                if self.serial and self.serial.in_waiting > 0:
                    command = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if command:
                        self.handle_command(command)
                # Read command from stdin (debug mode)
                if debug_stdin:
                    # Use select to check for input without blocking
                    if select.select([sys.stdin], [], [], 0.01)[0]:
                        stdin_cmd = sys.stdin.readline().strip()
                        if stdin_cmd:
                            print(f"[DEBUG] Received from stdin: {stdin_cmd}")
                            self.handle_command(stdin_cmd)
                # Small delay to prevent CPU overload
                time.sleep(0.01)
            except serial.SerialException as e:
                print(f"[ERROR] Serial error: {e}")
                time.sleep(1)
            except Exception as e:
                print(f"[ERROR] Unexpected error: {e}")
                time.sleep(0.5)
        # Cleanup imshow window on exit
        if cap is not None:
            cv2.destroyWindow(window_name)
    
    def shutdown(self):
        """Shutdown all hardware gracefully"""
        print("\n[SYSTEM] Shutting down...")
        self.running = False
        
        try:
            if self.lcd and hasattr(self.lcd, 'enabled') and self.lcd.enabled:
                self.lcd.clear()
                self.lcd.display_message("System Shutdown", "", "", "")
        except:
            pass
        
        try:
            if self.servo:
                self.servo.cleanup()
        except:
            pass
        
        try:
            if self.detector:
                self.detector.release_camera()
        except:
            pass
        
        try:
            if self.ir_sensor:
                self.ir_sensor.cleanup()
        except:
            pass
        
        try:
            if self.serial and self.serial.is_open:
                self.send_to_esp32("SLAVE_SHUTDOWN")
                self.serial.close()
        except:
            pass
        
        print("[SYSTEM] ✓ Shutdown complete")

# ==================== MAIN ENTRY POINT ====================

def main():
    """Main entry point"""
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--stdin', action='store_true', help='Read commands from standard input for debugging')
    args = parser.parse_args()
    controller = ChitSlaveController()
    try:
        controller.run(debug_stdin=args.stdin)
    except KeyboardInterrupt:
        print("\n[SYSTEM] Keyboard interrupt received")
    except Exception as e:
        print(f"\n[FATAL] Unhandled exception: {e}")
        import traceback
        traceback.print_exc()
    finally:
        controller.shutdown()

if __name__ == "__main__":
    main()
