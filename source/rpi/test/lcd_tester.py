#!/usr/bin/env python3
"""
LCD Tester for 20x4 I2C LCD Display
Address: 0x27
Connected to Raspberry Pi 4 I2C pins (SDA and SCL)
"""

import smbus2 as smbus
import time

# LCD Module Configuration
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
    def __init__(self):
        """Initialize I2C bus and LCD"""
        self.bus = smbus.SMBus(1)  # Rev 2 Pi uses 1
        
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
        self.bus.write_byte(I2C_ADDR, bits_high)
        self.lcd_toggle_enable(bits_high)

        # Low bits
        self.bus.write_byte(I2C_ADDR, bits_low)
        self.lcd_toggle_enable(bits_low)

    def lcd_toggle_enable(self, bits):
        """Toggle enable"""
        time.sleep(E_DELAY)
        self.bus.write_byte(I2C_ADDR, (bits | 0x04))
        time.sleep(E_PULSE)
        self.bus.write_byte(I2C_ADDR, (bits & ~0x04))
        time.sleep(E_DELAY)

    def lcd_string(self, message, line):
        """Send string to display"""
        message = message.ljust(LCD_WIDTH, " ")
        self.lcd_byte(line, LCD_CMD)

        for i in range(LCD_WIDTH):
            self.lcd_byte(ord(message[i]), LCD_CHR)

    def clear(self):
        """Clear LCD display"""
        self.lcd_byte(0x01, LCD_CMD)
        time.sleep(E_DELAY)

def main():
    """Main program for testing LCD"""
    try:
        lcd = LCD()
        print("LCD initialized successfully")
        
        while True:
            # Test all lines
            print("\nDisplaying test message on LCD...")
            lcd.lcd_string("Line 1: LCD Test", LCD_LINE_1)
            lcd.lcd_string("Line 2: 20x4 Display", LCD_LINE_2)
            lcd.lcd_string("Line 3: I2C Address", LCD_LINE_3)
            lcd.lcd_string("Line 4: 0x27", LCD_LINE_4)
            time.sleep(3)

            # Clear display
            print("Clearing display...")
            lcd.clear()
            time.sleep(1)

            # Scrolling text demo
            message = "Welcome to Chits Exchanger!"
            for i in range(len(message) - LCD_WIDTH + 1):
                lcd.lcd_string(message[i:i + LCD_WIDTH], LCD_LINE_1)
                time.sleep(0.3)
            
            # Show different test patterns
            print("Showing test patterns...")
            lcd.lcd_string("*" * LCD_WIDTH, LCD_LINE_1)
            lcd.lcd_string("-" * LCD_WIDTH, LCD_LINE_2)
            lcd.lcd_string("=" * LCD_WIDTH, LCD_LINE_3)
            lcd.lcd_string("#" * LCD_WIDTH, LCD_LINE_4)
            time.sleep(3)

    except KeyboardInterrupt:
        print("\nCleaning up...")
        lcd.clear()
        print("Program terminated by user")
    except Exception as e:
        print(f"Error: {e}")
        print("\nTroubleshooting tips:")
        print("1. Check if I2C is enabled (sudo raspi-config)")
        print("2. Verify the I2C address (sudo i2cdetect -y 1)")
        print("3. Check wire connections:")
        print("   - SDA -> GPIO2 (Pin 3)")
        print("   - SCL -> GPIO3 (Pin 5)")
        print("   - VCC -> 5V")
        print("   - GND -> Ground")

if __name__ == "__main__":
    main()