#!/usr/bin/env python3
"""
Serial Auto-Dispense Test Script
Tests the AUTO_DISPENSE command flow between RPi and ESP32

Usage:
    python serial_auto_dispense_test.py /dev/ttyUSB1

Author: Chits-Exchanger System
Date: October 26, 2025
"""

import serial
import time
import sys

def test_serial_connection(port, baud=115200, timeout=0.01):
    """Test basic serial connection"""
    print(f"Testing serial connection to {port}...")
    try:
        ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(2)  # Wait for connection
        
        # Clear buffers
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        print(f"✅ Serial connection established")
        print(f"   Baud: {baud}")
        print(f"   Timeout: {timeout}s")
        return ser
    except Exception as e:
        print(f"❌ Failed to connect: {e}")
        return None

def send_command(ser, command):
    """Send command to ESP32"""
    try:
        ser.write((command + '\n').encode())
        ser.flush()
        print(f"📤 Sent: {command}")
        return True
    except Exception as e:
        print(f"❌ Send failed: {e}")
        return False

def read_responses(ser, duration=5):
    """Read responses from ESP32 for specified duration"""
    print(f"📥 Reading responses for {duration} seconds...")
    start_time = time.time()
    responses = []
    
    while time.time() - start_time < duration:
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"   ESP32: {line}")
                    responses.append(line)
            except Exception as e:
                print(f"   Error reading: {e}")
        time.sleep(0.01)
    
    return responses

def test_auto_dispense(ser, chit_value):
    """Test AUTO_DISPENSE command"""
    print(f"\n{'='*60}")
    print(f"Testing AUTO_DISPENSE:{chit_value}")
    print(f"{'='*60}\n")
    
    # Send AUTO_DISPENSE command
    send_command(ser, f"AUTO_DISPENSE:{chit_value}")
    
    # Read responses for 30 seconds (enough time for dispensing)
    responses = read_responses(ser, duration=30)
    
    # Check for expected responses
    print(f"\n{'='*60}")
    print("Response Analysis:")
    print(f"{'='*60}")
    
    found_received = False
    found_plan = False
    found_dispensing = False
    found_complete = False
    
    for response in responses:
        if "AUTO_DISPENSE received" in response:
            print("✅ AUTO_DISPENSE command received by ESP32")
            found_received = True
        elif "Auto Dispensing Plan" in response:
            print("✅ Dispensing plan calculated")
            found_plan = True
        elif "Starting automatic coin dispensing" in response:
            print("✅ Dispensing started")
            found_dispensing = True
        elif "DISPENSING_COMPLETE" in response:
            print("✅ Dispensing completed")
            found_complete = True
        elif "SYSTEM BUSY" in response:
            print("⚠️  System was busy - command rejected")
            return False
        elif "Invalid chit value" in response:
            print("❌ Invalid chit value")
            return False
    
    success = found_received and found_plan and found_dispensing and found_complete
    
    if success:
        print("\n🎉 AUTO_DISPENSE test PASSED!")
    else:
        print("\n❌ AUTO_DISPENSE test FAILED")
        if not found_received:
            print("   - Command not received")
        if not found_plan:
            print("   - Plan not calculated")
        if not found_dispensing:
            print("   - Dispensing not started")
        if not found_complete:
            print("   - Dispensing not completed")
    
    return success

def test_all_denominations(ser):
    """Test all valid chit denominations"""
    denominations = [5, 10, 20, 50]
    results = {}
    
    print("\n" + "="*60)
    print("Testing All Denominations")
    print("="*60)
    
    for denom in denominations:
        print(f"\n\nWaiting 5 seconds before next test...")
        time.sleep(5)
        
        success = test_auto_dispense(ser, denom)
        results[denom] = success
    
    # Summary
    print("\n\n" + "="*60)
    print("Test Summary")
    print("="*60)
    
    for denom, success in results.items():
        status = "✅ PASS" if success else "❌ FAIL"
        print(f"  P{denom:2d}: {status}")
    
    all_passed = all(results.values())
    if all_passed:
        print("\n🎉 All tests PASSED!")
    else:
        print("\n❌ Some tests FAILED")
    
    return all_passed

def main():
    if len(sys.argv) < 2:
        print("Usage: python serial_auto_dispense_test.py <serial_port>")
        print("Example: python serial_auto_dispense_test.py /dev/ttyUSB1")
        sys.exit(1)
    
    port = sys.argv[1]
    
    print("="*60)
    print("Serial Auto-Dispense Test")
    print("="*60)
    print(f"Port: {port}")
    print(f"Date: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    print("="*60 + "\n")
    
    # Test connection
    ser = test_serial_connection(port)
    if not ser:
        sys.exit(1)
    
    try:
        # Interactive menu
        while True:
            print("\n" + "="*60)
            print("Test Options:")
            print("="*60)
            print("1. Test single denomination (manual entry)")
            print("2. Test P5")
            print("3. Test P10")
            print("4. Test P20")
            print("5. Test P50")
            print("6. Test all denominations (5, 10, 20, 50)")
            print("7. Send custom command")
            print("8. Monitor serial output")
            print("9. Exit")
            print("="*60)
            
            choice = input("\nSelect option (1-9): ").strip()
            
            if choice == '1':
                value = input("Enter chit value (5/10/20/50): ").strip()
                try:
                    value = int(value)
                    if value in [5, 10, 20, 50]:
                        test_auto_dispense(ser, value)
                    else:
                        print("❌ Invalid value. Use 5, 10, 20, or 50")
                except ValueError:
                    print("❌ Invalid input")
            
            elif choice == '2':
                test_auto_dispense(ser, 5)
            
            elif choice == '3':
                test_auto_dispense(ser, 10)
            
            elif choice == '4':
                test_auto_dispense(ser, 20)
            
            elif choice == '5':
                test_auto_dispense(ser, 50)
            
            elif choice == '6':
                test_all_denominations(ser)
            
            elif choice == '7':
                cmd = input("Enter command: ").strip()
                send_command(ser, cmd)
                read_responses(ser, duration=3)
            
            elif choice == '8':
                print("\nMonitoring serial output (Ctrl+C to stop)...")
                try:
                    while True:
                        if ser.in_waiting > 0:
                            line = ser.readline().decode('utf-8', errors='ignore').strip()
                            if line:
                                print(f"ESP32: {line}")
                        time.sleep(0.01)
                except KeyboardInterrupt:
                    print("\n\nMonitoring stopped")
            
            elif choice == '9':
                print("\nExiting...")
                break
            
            else:
                print("❌ Invalid option")
    
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    
    finally:
        if ser and ser.is_open:
            ser.close()
            print("Serial connection closed")

if __name__ == "__main__":
    main()
