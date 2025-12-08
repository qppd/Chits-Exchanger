import time
from smbus2 import SMBus

# LCD constants
LCD_ADDR = 0x27
LCD_WIDTH = 16     # Maximum characters per line
LCD_CHR = 1      # Mode - Sending data
LCD_CMD = 0      # Mode - Sending command
LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line
LCD_BACKLIGHT = 0x08  # On
ENABLE = 0b00000100   # Enable bit

# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005


def lcd_init(bus):
    lcd_byte(bus, 0x33, LCD_CMD)  # 110011 Initialise
    lcd_byte(bus, 0x32, LCD_CMD)  # 110010 Initialise
    lcd_byte(bus, 0x06, LCD_CMD)  # 000110 Cursor move direction
    lcd_byte(bus, 0x0C, LCD_CMD)  # 001100 Display On,Cursor Off, Blink Off
    lcd_byte(bus, 0x28, LCD_CMD)  # 101000 Data length, number of lines, font size
    lcd_byte(bus, 0x01, LCD_CMD)  # 000001 Clear display
    time.sleep(E_DELAY)


def lcd_byte(bus, bits, mode):
    bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
    bits_low = mode | ((bits << 4) & 0xF0) | LCD_BACKLIGHT
    bus.write_byte(LCD_ADDR, bits_high)
    lcd_toggle_enable(bus, bits_high)
    bus.write_byte(LCD_ADDR, bits_low)
    lcd_toggle_enable(bus, bits_low)


def lcd_toggle_enable(bus, bits):
    time.sleep(E_DELAY)
    bus.write_byte(LCD_ADDR, (bits | ENABLE))
    time.sleep(E_PULSE)
    bus.write_byte(LCD_ADDR, (bits & ~ENABLE))
    time.sleep(E_DELAY)


def lcd_string(bus, message, line):
    message = message.ljust(LCD_WIDTH, " ")
    lcd_byte(bus, line, LCD_CMD)
    for i in range(LCD_WIDTH):
        lcd_byte(bus, ord(message[i]), LCD_CHR)


def main():
    try:
        bus = SMBus(1)
        lcd_init(bus)
        lcd_string(bus, "LCD Test Line 1", LCD_LINE_1)
        lcd_string(bus, "Addr 0x27 OK!", LCD_LINE_2)
        time.sleep(5)
        lcd_string(bus, "Test Complete", LCD_LINE_1)
        lcd_string(bus, "Goodbye!", LCD_LINE_2)
        time.sleep(2)
        lcd_byte(bus, 0x01, LCD_CMD)  # Clear display
    except Exception as e:
        print(f"LCD test failed: {e}")

if __name__ == '__main__':
    main()
