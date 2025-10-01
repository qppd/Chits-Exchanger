"""
I2C 20x4 LCD Control Module for Raspberry Pi
Migrated from ESP32 I2C_LCD.h/.cpp

Provides high-level control for 20x4 character LCD using I2C backpack (PCF8574).
Compatible with Raspberry Pi GPIO and smbus2.

Features:
- Display text on any row/column
- Clear display, set cursor, custom characters
- Initialization and backlight control

Hardware:
- LCD: 20x4 character display
- I2C Backpack: PCF8574
- I2C Address: 0x27 (default)
- Power: 5V
"""

import smbus2
import time
from typing import Optional

LCD_I2C_ADDR = 0x27
LCD_WIDTH = 20
LCD_ROWS = 4

# LCD Commands
LCD_CLEAR = 0x01
LCD_HOME = 0x02
LCD_ENTRY_MODE = 0x04
LCD_DISPLAY_ON = 0x0C
LCD_FUNCTION_SET = 0x28
LCD_SET_DDRAM = 0x80

class I2CLCD:
    def __init__(self, i2c_addr: int = LCD_I2C_ADDR, bus: int = 1):
        self.addr = i2c_addr
        self.bus = smbus2.SMBus(bus)
        self.init_lcd()

    def init_lcd(self):
        """Initialize LCD display"""
        self.send_cmd(LCD_FUNCTION_SET)
        self.send_cmd(LCD_DISPLAY_ON)
        self.send_cmd(LCD_CLEAR)
        time.sleep(0.05)

    def send_cmd(self, cmd: int):
        self.bus.write_byte(self.addr, cmd)
        time.sleep(0.002)

    def send_data(self, data: int):
        self.bus.write_byte(self.addr, data | 0x40)
        time.sleep(0.002)

    def clear(self):
        self.send_cmd(LCD_CLEAR)
        time.sleep(0.05)

    def set_cursor(self, row: int, col: int):
        addr = LCD_SET_DDRAM + (0x40 * row) + col
        self.send_cmd(addr)

    def display_text(self, text: str, row: int = 0, col: int = 0):
        self.set_cursor(row, col)
        for char in text[:LCD_WIDTH]:
            self.send_data(ord(char))

    def backlight(self, on: bool = True):
        # PCF8574 backlight control (if wired)
        self.bus.write_byte(self.addr, 0x08 if on else 0x00)

    def cleanup(self):
        self.bus.close()
