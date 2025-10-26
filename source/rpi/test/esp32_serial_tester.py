#!/usr/bin/env python3
"""
esp32_serial_tester.py
Test serial communication between Raspberry Pi and ESP32 CoinExchanger

This script helps debug communication issues by:
1. Testing serial port connection
2. Reading ESP32 output
3. Sending test commands
4. Verifying bidirectional communication

Usage:
    python3 esp32_serial_tester.py
    python3 esp32_serial_tester.py --port /dev/ttyUSB0
"""

import serial
import time
import sys
import signal
import argparse

# Parse command line arguments
parser = argparse.ArgumentParser(description='ESP32 Serial Communication Tester')
parser.add_argument('--port', default='/dev/ttyUSB0', help='Serial port (default: /dev/ttyUSB0)')
parser.add_argument('--baud', type=int, default=115200, help='Baud rate (default: 115200)')
args = parser.parse_args()

# Serial configuration
SERIAL_PORT = args.port
BAUD_RATE = args.baud
ser = None

def cleanup(signum=None, frame=None):
    """Clean up and exit"""
    print("\n\n🛑 Cleaning up...")
    if ser and ser.is_open:
        print("Closing serial port...")
        ser.close()
    print("✅ Cleanup complete")
    sys.exit(0)

def test_serial_connection():
    """Test if serial port can be opened"""
    global ser
    
    print(f"\n{'='*60}")
    print(f"🔌 ESP32 SERIAL COMMUNICATION TESTER")
    print(f"{'='*60}")
    print(f"Port: {SERIAL_PORT}")
    print(f"Baud Rate: {BAUD_RATE}")
    print(f"{'='*60}\n")
    
    try:
        print(f"📡 Attempting to open {SERIAL_PORT}...")
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Wait for connection to establish
        print(f"✅ Serial port opened successfully!")
        print(f"   Port: {ser.port}")
        print(f"   Baud rate: {ser.baudrate}")
        print(f"   Is open: {ser.is_open}")
        return True
    except serial.SerialException as e:
        print(f"❌ Failed to open serial port: {e}")
        print(f"\n💡 Troubleshooting tips:")
        print(f"   1. Check if device exists: ls -la {SERIAL_PORT}")
        print(f"   2. Check permissions: ls -la {SERIAL_PORT}")
        print(f"   3. Check if user in dialout group: groups")
        print(f"   4. Add user to dialout: sudo usermod -a -G dialout $USER")
        print(f"   5. Check if port is in use: sudo lsof {SERIAL_PORT}")
        return False
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        return False

def read_esp32_output(duration=5):
    """Read and display ESP32 output for specified duration"""
    print(f"\n{'='*60}")
    print(f"📖 Reading ESP32 output for {duration} seconds...")
    print(f"{'='*60}\n")
    
    if not ser or not ser.is_open:
        print("❌ Serial port not open!")
        return
    
    start_time = time.time()
    line_count = 0
    
    try:
        while time.time() - start_time < duration:
            if ser.in_waiting > 0:
                try:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        line_count += 1
                        print(f"[{line_count:3d}] {line}")
                except Exception as e:
                    print(f"❌ Error reading line: {e}")
            time.sleep(0.01)
        
        print(f"\n✅ Read {line_count} lines in {duration} seconds")
        
        if line_count == 0:
            print("\n⚠️  No output received from ESP32!")
            print("   Possible issues:")
            print("   1. ESP32 not powered on")
            print("   2. ESP32 not running CoinExchanger.ino")
            print("   3. Wrong baud rate (ESP32 uses 115200)")
            print("   4. TX/RX wires swapped or disconnected")
            
    except KeyboardInterrupt:
        print("\n⏹️  Reading interrupted")

def send_test_commands():
    """Send test commands to ESP32 and read responses"""
    print(f"\n{'='*60}")
    print(f"📤 Sending test commands to ESP32")
    print(f"{'='*60}\n")
    
    if not ser or not ser.is_open:
        print("❌ Serial port not open!")
        return
    
    # Test commands based on CoinExchanger.ino
    test_commands = [
        ("help", "Request help/command list"),
        ("test_chit 50", "Simulate 50 peso chit detection"),
    ]
    
    for cmd, description in test_commands:
        print(f"\n🔹 Testing: {description}")
        print(f"   Command: {cmd}")
        
        try:
            # Clear input buffer
            ser.reset_input_buffer()
            
            # Send command
            ser.write((cmd + '\n').encode())
            ser.flush()
            print(f"   ✅ Sent: {cmd}")
            
            # Wait for response
            time.sleep(1)
            
            # Read response
            response_count = 0
            print(f"   📥 Response:")
            start_time = time.time()
            while time.time() - start_time < 2:  # Wait up to 2 seconds
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        response_count += 1
                        print(f"      {line}")
                time.sleep(0.01)
            
            if response_count == 0:
                print(f"      ⚠️  No response received")
            else:
                print(f"   ✅ Received {response_count} lines")
                
        except Exception as e:
            print(f"   ❌ Error: {e}")
        
        time.sleep(1)

def send_rpi_commands():
    """Send RPi-specific commands that ESP32 expects"""
    print(f"\n{'='*60}")
    print(f"📤 Sending RPi Protocol Commands")
    print(f"{'='*60}\n")
    
    if not ser or not ser.is_open:
        print("❌ Serial port not open!")
        return
    
    # Commands that RPi should send to ESP32
    rpi_commands = [
        ("IR_DETECTED", "IR sensor detected chit"),
        ("CHIT_DETECTED:50", "Detected 50 peso chit"),
        ("CHIT_RELEASED:50", "Released 50 peso chit"),
        ("DETECTION_TIMEOUT", "Detection timed out"),
    ]
    
    for cmd, description in rpi_commands:
        print(f"\n🔹 {description}")
        print(f"   Command: {cmd}")
        
        try:
            # Clear buffers
            ser.reset_input_buffer()
            
            # Send command
            ser.write((cmd + '\n').encode())
            ser.flush()
            print(f"   ✅ Sent to ESP32")
            
            # Wait and read response
            time.sleep(0.5)
            
            print(f"   📥 ESP32 Response:")
            response_count = 0
            start_time = time.time()
            while time.time() - start_time < 2:
                if ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        response_count += 1
                        print(f"      {line}")
                time.sleep(0.01)
            
            if response_count == 0:
                print(f"      (No immediate response)")
                
        except Exception as e:
            print(f"   ❌ Error: {e}")
        
        time.sleep(1)

def interactive_mode():
    """Interactive command mode"""
    print(f"\n{'='*60}")
    print(f"⌨️  INTERACTIVE MODE")
    print(f"{'='*60}")
    print("Type commands to send to ESP32 (or 'exit' to quit)")
    print("Press Ctrl+C to return to menu\n")
    
    if not ser or not ser.is_open:
        print("❌ Serial port not open!")
        return
    
    try:
        while True:
            cmd = input("esp32> ").strip()
            
            if cmd.lower() in ['exit', 'quit']:
                break
            
            if not cmd:
                continue
            
            try:
                ser.write((cmd + '\n').encode())
                ser.flush()
                print(f"✅ Sent: {cmd}")
                
                # Read responses for 1 second
                time.sleep(0.5)
                while ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        print(f"   {line}")
                    
            except Exception as e:
                print(f"❌ Error: {e}")
                
    except KeyboardInterrupt:
        print("\n⏹️  Exiting interactive mode")

def main_menu():
    """Display main menu and handle user selection"""
    while True:
        print(f"\n{'='*60}")
        print("📋 MAIN MENU")
        print(f"{'='*60}")
        print("1. Read ESP32 output (5 seconds)")
        print("2. Send test commands (help, test_chit)")
        print("3. Send RPi protocol commands")
        print("4. Interactive command mode")
        print("5. Reconnect serial port")
        print("6. Exit")
        print(f"{'='*60}")
        
        choice = input("\nSelect option (1-6): ").strip()
        
        if choice == '1':
            read_esp32_output(5)
        elif choice == '2':
            send_test_commands()
        elif choice == '3':
            send_rpi_commands()
        elif choice == '4':
            interactive_mode()
        elif choice == '5':
            if ser and ser.is_open:
                ser.close()
            if test_serial_connection():
                print("✅ Reconnected successfully")
            else:
                print("❌ Reconnection failed")
                break
        elif choice == '6':
            break
        else:
            print("❌ Invalid choice")

def main():
    """Main function"""
    # Register cleanup handler
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)
    
    # Test serial connection
    if not test_serial_connection():
        print("\n❌ Cannot proceed without serial connection")
        sys.exit(1)
    
    # Read initial output
    print("\n📖 Reading initial ESP32 output...")
    read_esp32_output(3)
    
    # Show main menu
    main_menu()
    
    # Cleanup
    cleanup()

if __name__ == "__main__":
    main()
