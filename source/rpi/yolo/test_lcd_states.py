#!/usr/bin/env python3
"""
Quick test script for LCD display on Chit Detection System
Tests all the display states that will be shown during operation
"""

import time
import sys

try:
    import smbus2 as smbus
except ImportError:
    print("ERROR: smbus2 not installed")
    print("Install with: pip3 install smbus2")
    sys.exit(1)

# LCD Configuration
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
    def __init__(self, addr=I2C_ADDR, bus=1):
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
            print(f"✅ LCD initialized at address 0x{addr:02X}")
        except Exception as e:
            print(f"❌ Could not initialize LCD: {e}")
            self.enabled = False

    def lcd_byte(self, bits, mode):
        if not self.enabled:
            return
        
        bits_high = mode | (bits & 0xF0) | 0x08
        bits_low = mode | ((bits << 4) & 0xF0) | 0x08

        self.bus.write_byte(self.addr, bits_high)
        self.lcd_toggle_enable(bits_high)

        self.bus.write_byte(self.addr, bits_low)
        self.lcd_toggle_enable(bits_low)

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
        
        self.lcd_string(line1, LCD_LINE_1)
        self.lcd_string(line2, LCD_LINE_2)
        self.lcd_string(line3, LCD_LINE_3)
        self.lcd_string(line4, LCD_LINE_4)

def test_lcd_states():
    """Test all LCD display states used in the chit detection system"""
    
    print("\n=== LCD Display States Test ===\n")
    
    lcd = LCD()
    if not lcd.enabled:
        print("❌ LCD not available. Exiting...")
        return
    
    states = [
        ("INITIALIZATION", [
            "Chit Detection",
            "System Ready",
            "Waiting for chit...",
            ""
        ], 3),
        
        ("IR DETECTED", [
            "IR DETECTED!",
            "Scanning chit...",
            "Please wait...",
            ""
        ], 2),
        
        ("DETECTING", [
            "DETECTING...",
            "Chit: P50",
            "Conf: 85%",
            "Time: 3s"
        ], 2),
        
        ("DETECTION SUCCESS", [
            "DETECTED!",
            "Value: P50",
            "Conf: 95%",
            "Releasing chit..."
        ], 2),
        
        ("CHIT RELEASED", [
            "SUCCESS!",
            "Chit: P50",
            "Released",
            "Sent to ESP32"
        ], 2),
        
        ("ESP32 RESPONSE", [
            "ESP32:",
            "Dispensing",
            "Complete!",
            ""
        ], 2),
        
        ("READY AGAIN", [
            "Ready",
            "Waiting for chit...",
            "",
            ""
        ], 2),
        
        ("TIMEOUT", [
            "TIMEOUT!",
            "No valid chit",
            "detected",
            "Try again..."
        ], 2),
        
        ("SHUTDOWN", [
            "Shutting down...",
            "",
            "",
            ""
        ], 2),
    ]
    
    for state_name, lines, delay in states:
        print(f"Displaying: {state_name}")
        lcd.display_lines(*lines)
        time.sleep(delay)
    
    print("\nClearing display...")
    lcd.clear()
    
    print("✅ LCD test complete!\n")

if __name__ == "__main__":
    try:
        test_lcd_states()
    except KeyboardInterrupt:
        print("\n\n⚠️  Test interrupted by user")
        print("Clearing LCD...")
        try:
            lcd = LCD()
            lcd.clear()
        except:
            pass
    except Exception as e:
        print(f"\n❌ Error: {e}")
        print("\nTroubleshooting:")
        print("1. Enable I2C: sudo raspi-config")
        print("2. Check wiring:")
        print("   VCC -> 5V")
        print("   GND -> Ground")
        print("   SDA -> GPIO2 (Pin 3)")
        print("   SCL -> GPIO3 (Pin 5)")
        print("3. Scan I2C: sudo i2cdetect -y 1")
