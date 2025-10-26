#!/bin/bash
# Install I2C LCD dependencies for Raspberry Pi

echo "=== Installing I2C LCD Dependencies ==="
echo ""

# Install smbus2 for I2C communication
echo "Installing smbus2..."
pip3 install smbus2

# Check if I2C is enabled
echo ""
echo "Checking I2C configuration..."
if lsmod | grep -q i2c_bcm; then
    echo "✅ I2C kernel module is loaded"
else
    echo "⚠️  I2C kernel module not loaded"
    echo "Enable I2C using: sudo raspi-config"
    echo "Go to: Interface Options -> I2C -> Enable"
fi

# Check for I2C devices
echo ""
echo "Scanning for I2C devices..."
if command -v i2cdetect &> /dev/null; then
    echo "I2C devices detected:"
    sudo i2cdetect -y 1
    echo ""
    echo "Look for address 0x27 (typical for 20x4 LCD)"
else
    echo "i2c-tools not installed"
    echo "Install with: sudo apt-get install i2c-tools"
fi

echo ""
echo "=== LCD Wiring Guide ==="
echo "Connect the I2C LCD to your Raspberry Pi:"
echo "  VCC -> 5V (Pin 2 or 4)"
echo "  GND -> Ground (Pin 6, 9, 14, 20, 25, 30, 34, or 39)"
echo "  SDA -> GPIO2 (Pin 3)"
echo "  SCL -> GPIO3 (Pin 5)"
echo ""
echo "Installation complete!"
