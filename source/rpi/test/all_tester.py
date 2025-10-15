#!/usr/bin/env python3

import RPi.GPIO as GPIO
import smbus2
import time
import signal
import sys
from threading import Thread, Event

# Pin Configurations
BUTTON_PIN = 17
SERVO_PIN = 18

# LCD Configuration
LCD_ADDR = 0x27
LCD_WIDTH = 20
LCD_LINES = 4

class LCD:
    def __init__(self, addr=LCD_ADDR, bus=1):
        self.addr = addr
        self.bus = smbus2.SMBus(bus)
        
        # LCD Commands
        self.LCD_CLEARDISPLAY = 0x01
        self.LCD_RETURNHOME = 0x02
        self.LCD_ENTRYMODESET = 0x04
        self.LCD_DISPLAYCONTROL = 0x08
        self.LCD_FUNCTIONSET = 0x20
        self.LCD_SETCGRAMADDR = 0x40
        self.LCD_SETDDRAMADDR = 0x80
        
        # Add retry mechanism for initialization
        retries = 3
        while retries > 0:
            try:
                self.init_display()
                break
            except Exception as e:
                retries -= 1
                if retries == 0:
                    raise
                time.sleep(0.5)
    
    def init_display(self):
        try:
            # Initialize display in 4-bit mode
            self.write_cmd(0x33)
            self.write_cmd(0x32)
            self.write_cmd(0x28)  # 2 lines, 5x8 font
            self.write_cmd(0x0C)  # Display on, no cursor
            self.write_cmd(0x06)  # Entry mode set
            self.write_cmd(0x01)  # Clear display
            time.sleep(0.2)
        except Exception as e:
            print(f"LCD Init Error: {e}")
            raise
    
    def write_cmd(self, cmd):
        self.bus.write_byte_data(self.addr, 0x00, cmd)
        time.sleep(0.001)
    
    def write_char(self, char):
        self.bus.write_byte_data(self.addr, 0x40, ord(char))
        time.sleep(0.001)
    
    def clear(self):
        self.write_cmd(self.LCD_CLEARDISPLAY)
        time.sleep(0.2)
    
    def write_line(self, text, line):
        if line == 1:
            self.write_cmd(0x80)
        elif line == 2:
            self.write_cmd(0xC0)
        elif line == 3:
            self.write_cmd(0x94)
        elif line == 4:
            self.write_cmd(0xD4)
        
        for char in text[:LCD_WIDTH]:
            self.write_char(char)

class ServoControl:
    def __init__(self, pin):
        self.pin = pin
        GPIO.setup(pin, GPIO.OUT)
        self.pwm = GPIO.PWM(pin, 50)  # 50Hz frequency
        self.pwm.start(0)
    
    def set_angle(self, angle):
        # More gentle servo movement
        duty = angle / 18 + 2
        # Start with a lower duty cycle and gradually increase
        for d in range(0, int(duty * 10), 1):
            self.pwm.ChangeDutyCycle(d/10)
            time.sleep(0.01)
        time.sleep(0.2)  # Hold at position
        self.pwm.ChangeDutyCycle(0)  # Stop pulse to prevent jitter
    
    def cleanup(self):
        self.pwm.stop()

class ButtonHandler:
    def __init__(self, pin, callback):
        self.pin = pin
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(pin, GPIO.BOTH, callback=callback, bouncetime=200)

class AllTester:
    def __init__(self):
        # Initialize GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Setup stop event
        self.stop_event = Event()
        
        try:
            # Initialize components
            self.lcd = LCD()
            self.servo = ServoControl(SERVO_PIN)
            self.button = ButtonHandler(BUTTON_PIN, self.button_callback)
            
            # State variables
            self.servo_angle = 0
            self.test_count = 0
            
        except Exception as e:
            print(f"Initialization Error: {e}")
            self.cleanup()
            sys.exit(1)
    
    def button_callback(self, channel):
        if not GPIO.input(channel):  # Button pressed
            self.test_count += 1
            
            # Update LCD first
            try:
                self.lcd.clear()
                time.sleep(0.1)  # Small delay after clear
                self.lcd.write_line(f"Button Press #{self.test_count}", 1)
                time.sleep(0.1)  # Small delay between writes
                self.lcd.write_line(f"Moving servo...", 2)
                time.sleep(0.1)  # Small delay before servo move
                
                # Move servo on button press
                self.servo_angle = (self.servo_angle + 45) % 181
                self.servo.set_angle(self.servo_angle)
                
                # Update final servo position
                time.sleep(0.1)  # Small delay after servo move
                self.lcd.write_line(f"Servo Angle: {self.servo_angle}", 2)
            except Exception as e:
                print(f"LCD Update Error: {e}")
                # Try to reinitialize LCD if there's an error
                try:
                    self.lcd.init_display()
                except:
                    pass
    
    def run_test_sequence(self):
        try:
            # Initial display
            self.lcd.clear()
            self.lcd.write_line("All Tester Ready", 1)
            self.lcd.write_line("Press button to test", 2)
            self.lcd.write_line("Ctrl+C to exit", 3)
            
            # Keep running until stopped
            while not self.stop_event.is_set():
                time.sleep(0.1)
                
        except Exception as e:
            print(f"Test Sequence Error: {e}")
            self.cleanup()
    
    def cleanup(self):
        print("\nCleaning up...")
        try:
            self.lcd.clear()
            self.lcd.write_line("Shutting down...", 1)
            time.sleep(1)
            self.servo.cleanup()
            GPIO.cleanup()
        except:
            pass

def signal_handler(signum, frame):
    tester.stop_event.set()
    tester.cleanup()
    sys.exit(0)

if __name__ == "__main__":
    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    print("Starting All Tester...")
    print("Press button to test servo movement")
    print("Press Ctrl+C to exit")
    
    try:
        tester = AllTester()
        tester.run_test_sequence()
    except Exception as e:
        print(f"Main Error: {e}")
        if 'tester' in locals():
            tester.cleanup()
        sys.exit(1)