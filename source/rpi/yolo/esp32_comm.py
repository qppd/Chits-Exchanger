"""
ESP32 Communication & Hardware Management Module
- Monitors IR sensor (GPIO 17)
- Manages ESP32 serial communication (115200 baud)
- Controls servo for chit release (GPIO 22)
- Drives LCD display (I2C)
- Listens for YOLO detection results and triggers dispensing
- Sends IR trigger signal to YOLO when chit detected
"""

import os
import sys
import argparse
import time
import serial
import RPi.GPIO as GPIO
import pigpio
from multiprocessing import Queue

# I2C LCD Support
try:
    import smbus2 as smbus
    LCD_AVAILABLE = True
except ImportError:
    print("WARNING: smbus2 not available. LCD display will be disabled.")
    LCD_AVAILABLE = False

# ===== LCD Configuration =====
I2C_ADDR = 0x27
LCD_WIDTH = 20
LCD_ROWS = 4

LCD_CHR = 1
LCD_CMD = 0

LCD_LINE_1 = 0x80
LCD_LINE_2 = 0xC0
LCD_LINE_3 = 0x94
LCD_LINE_4 = 0xD4

E_PULSE = 0.0005
E_DELAY = 0.0005


class LCD:
    """I2C LCD Display Class for 20x4 character display"""
    def __init__(self, addr=I2C_ADDR, bus=1):
        if not LCD_AVAILABLE:
            self.enabled = False
            return
        
        try:
            self.bus = smbus.SMBus(bus)
            self.addr = addr
            self.enabled = True
            
            self.lcd_byte(0x33, LCD_CMD)
            self.lcd_byte(0x32, LCD_CMD)
            self.lcd_byte(0x06, LCD_CMD)
            self.lcd_byte(0x0C, LCD_CMD)
            self.lcd_byte(0x28, LCD_CMD)
            self.lcd_byte(0x01, LCD_CMD)
            time.sleep(E_DELAY)
            print(f"LCD initialized at address 0x{addr:02X}")
        except Exception as e:
            print(f"WARNING: Could not initialize LCD: {e}")
            self.enabled = False

    def lcd_byte(self, bits, mode):
        if not self.enabled:
            return
        try:
            bits_high = mode | (bits & 0xF0) | 0x08
            bits_low = mode | ((bits << 4) & 0xF0) | 0x08
            self.bus.write_byte(self.addr, bits_high)
            self.lcd_toggle_enable(bits_high)
            self.bus.write_byte(self.addr, bits_low)
            self.lcd_toggle_enable(bits_low)
        except:
            pass

    def lcd_toggle_enable(self, bits):
        time.sleep(E_DELAY)
        self.bus.write_byte(self.addr, (bits | 0x04))
        time.sleep(E_PULSE)
        self.bus.write_byte(self.addr, (bits & ~0x04))
        time.sleep(E_DELAY)

    def lcd_string(self, message, line):
        if not self.enabled:
            return
        message = message.ljust(LCD_WIDTH, " ")
        self.lcd_byte(line, LCD_CMD)
        for i in range(min(LCD_WIDTH, len(message))):
            self.lcd_byte(ord(message[i]), LCD_CHR)

    def clear(self):
        if not self.enabled:
            return
        self.lcd_byte(0x01, LCD_CMD)
        time.sleep(E_DELAY)
    
    def display_lines(self, line1="", line2="", line3="", line4=""):
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


# GPIO Pin Configurations for RPi
IR_SENSOR_PIN = 17
SERVO_PIN = 22

# Servo angles
SERVO_INITIAL_ANGLE = 39
SERVO_RELEASE_ANGLE = 90

# Valid chit denominations
VALID_CHITS = [5, 10, 20, 50]


def angle_to_pulse(angle):
    """Convert angle (0-180) to pulse width (500-2500us)"""
    return int(500 + (angle / 180.0) * 2000)


def set_servo_angle(pi, angle):
    """Set servo to specified angle"""
    if angle < 0:
        angle = 0
    elif angle > 180:
        angle = 180
    pulse_width = angle_to_pulse(angle)
    pi.set_servo_pulsewidth(SERVO_PIN, pulse_width)


def release_chit(pi):
    """Release chit by moving servo"""
    print(f"Releasing chit: Moving servo from {SERVO_INITIAL_ANGLE}° to {SERVO_RELEASE_ANGLE}°")
    set_servo_angle(pi, SERVO_RELEASE_ANGLE)
    time.sleep(2)
    print(f"Returning servo to initial position {SERVO_INITIAL_ANGLE}°")
    set_servo_angle(pi, SERVO_INITIAL_ANGLE)


def is_ir_detected():
    """Check if IR sensor detects a chit (Active LOW)"""
    return not GPIO.input(IR_SENSOR_PIN)


def send_to_esp32(esp32_serial, message):
    """Send message to ESP32 via serial"""
    if esp32_serial and esp32_serial.is_open:
        try:
            esp32_serial.reset_output_buffer()
            full_message = message + '\n'
            esp32_serial.write(full_message.encode('utf-8'))
            esp32_serial.flush()
            print(f"✅ Sent to ESP32: {message}")
            return True
        except serial.SerialException as e:
            print(f"❌ Serial error sending to ESP32: {e}")
            return False
        except Exception as e:
            print(f"❌ Error sending to ESP32: {e}")
            return False
    return False


def read_from_esp32(esp32_serial):
    """Read messages from ESP32 (non-blocking)"""
    if esp32_serial and esp32_serial.is_open:
        try:
            if esp32_serial.in_waiting > 0:
                message = esp32_serial.readline().decode('utf-8', errors='ignore').strip()
                if message and ("AUTO_DISPENSE" in message or "DISPENSING_COMPLETE" in message or "✅" in message or "❌" in message):
                    print(f"📨 ESP32: {message}")
                    return message
        except:
            pass
    return None


def run_esp32_comm_loop(detection_queue, ir_trigger_queue, esp32_port):
    """
    Main communication loop
    - Monitors IR sensor
    - Sends IR trigger to YOLO
    - Listens for detection results
    - Controls servo and ESP32 dispensing
    - Manages LCD display
    """
    print(f"\n{'='*60}")
    print("🔌 ESP32 COMMUNICATION MODULE - STARTUP")
    print(f"{'='*60}")
    
    # Initialize GPIO
    print("Initializing GPIO...")
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(IR_SENSOR_PIN, GPIO.IN)
    print("✅ GPIO initialized")
    
    # Initialize pigpio for servo
    print("Initializing servo control...")
    pi = pigpio.pi()
    if not pi.connected:
        print("ERROR: Could not connect to pigpio daemon. Run 'sudo pigpiod' first.")
        sys.exit(1)
    set_servo_angle(pi, SERVO_INITIAL_ANGLE)
    print(f"✅ Servo initialized to {SERVO_INITIAL_ANGLE}°")
    
    # Initialize serial to ESP32
    esp32_serial = None
    try:
        print(f"Initializing serial connection to ESP32 on {esp32_port}...")
        esp32_serial = serial.Serial()
        esp32_serial.port = esp32_port
        esp32_serial.baudrate = 115200
        esp32_serial.timeout = 0.01
        esp32_serial.write_timeout = 1.0
        esp32_serial.bytesize = serial.EIGHTBITS
        esp32_serial.parity = serial.PARITY_NONE
        esp32_serial.stopbits = serial.STOPBITS_ONE
        esp32_serial.dtr = False
        esp32_serial.rts = False
        esp32_serial.open()
        esp32_serial.dtr = False
        esp32_serial.rts = False
        time.sleep(0.5)
        esp32_serial.reset_input_buffer()
        esp32_serial.reset_output_buffer()
        print(f"✅ Serial connection to ESP32 established")
    except serial.SerialException as e:
        print(f"❌ Serial connection failed: {e}")
        esp32_serial = None
    
    # Initialize LCD
    print("Initializing LCD display...")
    lcd = LCD()
    if lcd.enabled:
        lcd.clear()
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
    
    print(f"{'='*60}\n")
    print("🎉 ESP32 Communication module ready!")
    print("Monitoring IR sensor and YOLO detections...\n")
    
    last_ir_state = False
    
    try:
        while True:
            # Check IR sensor state
            ir_detected = is_ir_detected()
            
            # IR rising edge: trigger YOLO detection
            if ir_detected and not last_ir_state:
                print(f"\n{'='*60}")
                print(f"🔍 IR SENSOR TRIGGERED - CHIT DETECTED")
                print(f"{'='*60}")
                
                # Signal YOLO to start detecting
                ir_trigger_queue.put("IR_DETECTED")
                
                lcd.display_lines(
                    "IR DETECTED!",
                    "YOLO detecting...",
                    "Please wait...",
                    ""
                )
            
            last_ir_state = ir_detected
            
            # Check for detection results from YOLO
            try:
                detection_result = detection_queue.get_nowait()
                msg_type, chit_value, confidence = detection_result
                
                if msg_type == "CHIT_DETECTED":
                    print(f"\n{'='*60}")
                    print(f"🎉 CHIT DETECTED: ₱{chit_value}")
                    print(f"   Confidence: {confidence:.2%}")
                    print(f"{'='*60}")
                    
                    lcd.display_lines(
                        "DETECTED!",
                        f"Value: P{chit_value}",
                        f"Conf: {int(confidence*100)}%",
                        "Releasing chit..."
                    )
                    
                    # Release chit locally
                    release_chit(pi)
                    print(f"✅ Chit ₱{chit_value} released")
                    
                    # Send to ESP32 for coin dispensing
                    print(f"\n{'='*60}")
                    print(f"🪙 AUTO-DISPENSING TRIGGERED")
                    print(f"{'='*60}")
                    print(f"   Sending to ESP32: AUTO_DISPENSE:{chit_value}")
                    
                    send_to_esp32(esp32_serial, f"AUTO_DISPENSE:{chit_value}")
                    
                    lcd.display_lines(
                        "AUTO DISPENSE!",
                        f"Chit: P{chit_value}",
                        "Dispensing...",
                        "Please wait"
                    )
                    time.sleep(3)
                    
                    lcd.display_lines(
                        "Ready - Scanning",
                        "Waiting for chit...",
                        "",
                        ""
                    )
                
                elif msg_type == "DETECTION_TIMEOUT":
                    print(f"\n❌ Detection timeout - no valid chit")
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
            
            except:
                pass  # Queue empty
            
            # Check for ESP32 messages
            esp32_msg = read_from_esp32(esp32_serial)
            
            time.sleep(0.05)
    
    except KeyboardInterrupt:
        print("\n\nShutdown signal received")
    
    finally:
        print("Cleaning up...")
        set_servo_angle(pi, SERVO_INITIAL_ANGLE)
        pi.set_servo_pulsewidth(SERVO_PIN, 0)
        pi.stop()
        GPIO.cleanup()
        
        if esp32_serial and esp32_serial.is_open:
            send_to_esp32(esp32_serial, "SYSTEM_SHUTDOWN")
            esp32_serial.close()
        
        lcd.clear()
        print("ESP32 Communication module closed.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--esp32_port', help='Serial port for ESP32 (example: "/dev/ttyUSB0")',
                        default='/dev/ttyUSB0')
    args = parser.parse_args()
    
    print("\n⚠️  NOTE: This module should be run via start_detection_system.py")
    print("Starting ESP32 communication in standalone mode...\n")
    
    # Create dummy queues for standalone testing
    detection_queue = Queue()
    ir_trigger_queue = Queue()
    
    try:
        run_esp32_comm_loop(detection_queue, ir_trigger_queue, args.esp32_port)
    except Exception as e:
        print(f"Error: {e}")
