#!/usr/bin/env python3

import RPi.GPIO as GPIO
import pigpio
import smbus2
import time
import signal
import sys

# Pin Configurations
BUTTON_PIN = 27
IR_PIN = 17
SERVO_PIN = 22

# LCD Configuration
LCD_ADDR = 0x27
LCD_WIDTH = 20
LCD_LINES = 4

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
    def __init__(self, addr=LCD_ADDR, bus=1):
        self.addr = addr
        self.bus = smbus2.SMBus(bus)
        
        # Initialize display
        self.lcd_byte(0x33, LCD_CMD)  # 110011 Initialize
        self.lcd_byte(0x32, LCD_CMD)  # 110010 Initialize
        self.lcd_byte(0x06, LCD_CMD)  # 000110 Cursor move direction
        self.lcd_byte(0x0C, LCD_CMD)  # 001100 Display On, Cursor Off, Blink Off
        self.lcd_byte(0x28, LCD_CMD)  # 101000 Data length, number of lines, font size
        self.lcd_byte(0x01, LCD_CMD)  # 000001 Clear display
        time.sleep(E_DELAY)
    
    def lcd_byte(self, bits, mode):
        """Send byte to data pins"""
        bits_high = mode | (bits & 0xF0) | 0x08
        bits_low = mode | ((bits << 4) & 0xF0) | 0x08

        # High bits
        self.bus.write_byte(self.addr, bits_high)
        self.lcd_toggle_enable(bits_high)

        # Low bits
        self.bus.write_byte(self.addr, bits_low)
        self.lcd_toggle_enable(bits_low)

    def lcd_toggle_enable(self, bits):
        """Toggle enable"""
        time.sleep(E_DELAY)
        self.bus.write_byte(self.addr, (bits | 0x04))
        time.sleep(E_PULSE)
        self.bus.write_byte(self.addr, (bits & ~0x04))
        time.sleep(E_DELAY)
    
    def clear(self):
        """Clear LCD display"""
        self.lcd_byte(0x01, LCD_CMD)
        time.sleep(E_DELAY)
    
    def write_line(self, text, line):
        """Send string to display"""
        line_addresses = [LCD_LINE_1, LCD_LINE_2, LCD_LINE_3, LCD_LINE_4]
        message = text.ljust(LCD_WIDTH, " ")
        self.lcd_byte(line_addresses[line - 1], LCD_CMD)
        
        for i in range(LCD_WIDTH):
            self.lcd_byte(ord(message[i]), LCD_CHR)

class ServoControl:
    def __init__(self, pin):
        self.pin = pin
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("Could not connect to pigpio daemon!")
        self.current_angle = None
    
    def angle_to_pulse(self, angle):
        # Map angle (0-180) to pulse width (500-2500us)
        return int(500 + (angle / 180.0) * 2000)
    
    def set_angle(self, angle):
        # Ensure angle is at least 40 degrees
        if angle < 40:
            angle = 40
        
        # Only update if angle changed
        if self.current_angle != angle:
            pulse_width = self.angle_to_pulse(angle)
            self.pi.set_servo_pulsewidth(self.pin, pulse_width)
            self.current_angle = angle
            time.sleep(0.3)  # Give servo time to move
    
    def cleanup(self):
        self.pi.set_servo_pulsewidth(self.pin, 0)
        self.pi.stop()

class ButtonHandler:
    def __init__(self, pin, callback):
        self.pin = pin
        self.callback = callback
        try:
            # Clean up any existing events
            try:
                GPIO.remove_event_detect(pin)
            except:
                pass
            
            # Set up the pin
            print(f"Setting up button on pin {pin}")
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            time.sleep(0.1)  # Give some time for setup
            
            # Try to add the event detection
            print("Adding event detection...")
            GPIO.add_event_detect(pin, GPIO.FALLING, callback=self._handle_button, bouncetime=200)
            print("Event detection added successfully")
            
        except Exception as e:
            print(f"GPIO Error Details: {type(e).__name__}: {str(e)}")
            raise RuntimeError(f"Failed to initialize button on pin {pin}")
    
    def _handle_button(self, channel):
        # Only trigger on button press (FALLING edge)
        if not GPIO.input(channel):
            self.callback(channel)

class AllTester:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Initialize components
        self.lcd = LCD()
        self.servo = ServoControl(SERVO_PIN)
        
        # Setup inputs
        GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(IR_PIN, GPIO.IN)
        
        self.running = True
        self.servo_angle = 40
        # Move servo to home position (40 degrees)
        self.servo.set_angle(40)
        self.last_button_state = None
        self.last_ir_state = None
        self.last_servo_angle = None
        self.display_initialized = False
    
    def update_display(self):
        button_state = "PRESSED" if not GPIO.input(BUTTON_PIN) else "RELEASED"
        ir_state = "DETECTED" if not GPIO.input(IR_PIN) else "NONE"
        
        # Only update if something changed or first time
        if (not self.display_initialized or 
            button_state != self.last_button_state or 
            ir_state != self.last_ir_state or 
            self.servo_angle != self.last_servo_angle):
            
            if not self.display_initialized:
                self.lcd.clear()
                self.display_initialized = True
            
            self.lcd.write_line(f"Button: {button_state}", 1)
            self.lcd.write_line(f"IR: {ir_state}", 2)
            self.lcd.write_line(f"Servo: {self.servo_angle}", 3)
            self.lcd.write_line("Ctrl+C to exit", 4)
            
            self.last_button_state = button_state
            self.last_ir_state = ir_state
            self.last_servo_angle = self.servo_angle
    
    def run(self):
        print("Testing all components...")
        print("- Button on GPIO 27")
        print("- IR Sensor on GPIO 17")
        print("- Servo on GPIO 22")
        print("Press Ctrl+C to exit")
        
        try:
            while self.running:
                if not GPIO.input(BUTTON_PIN):  # Button pressed
                    # Move to 90 degrees when pressed
                    self.servo_angle = 90
                    self.servo.set_angle(self.servo_angle)
                    time.sleep(0.2)  # Debounce
                else:
                    # Return to 40 degrees (home position) when not pressed
                    if self.servo_angle != 40:
                        self.servo_angle = 40
                        self.servo.set_angle(self.servo_angle)
                
                self.update_display()
                time.sleep(0.5)  # Reduced update frequency to prevent blinking
        
        except KeyboardInterrupt:
            self.cleanup()
    
    def cleanup(self):
        self.running = False
        self.lcd.clear()
        self.lcd.write_line("Shutting down...", 1)
        self.servo.cleanup()
        GPIO.cleanup()

if __name__ == "__main__":
    tester = AllTester()
    tester.run()