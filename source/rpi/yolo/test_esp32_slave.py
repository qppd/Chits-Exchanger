#!/usr/bin/env python3
"""
Test script for ESP32 Slave Communication System
Simulates ESP32 commands via terminal input

Usage: python3 test_esp32_slave.py
"""

import sys
import time

def print_menu():
    """Print command menu"""
    print("\n" + "="*60)
    print("ESP32 SLAVE SYSTEM TEST MENU")
    print("="*60)
    print("Available Commands:")
    print("  1. CHECK_IR        - Check IR sensor status")
    print("  2. DETECT_CHIT     - Run YOLO chit detection")
    print("  3. DISPLAY         - Send custom LCD message")
    print("  4. DISPENSE_CHIT   - Trigger servo to release chit")
    print("  5. PING            - Test communication")
    print("  6. RESET           - Reset slave system")
    print("  7. FULL_TEST       - Run complete workflow")
    print("  0. EXIT            - Quit test program")
    print("="*60)

def send_command(command):
    """Simulate sending command to slave"""
    print(f"\n>>> Sending to Slave: {command}")
    print(f">>> (In real system: Serial.println(\"{command}\"))")
    print(">>> Waiting for response...")

def test_ir():
    """Test IR sensor check"""
    send_command("CHECK_IR")
    print(">>> Expected responses: 'IR_DETECTED' or 'IR_CLEAR'")

def test_detect():
    """Test YOLO detection"""
    send_command("DETECT_CHIT")
    print(">>> Expected responses:")
    print("    - CHIT_5  (if 5 peso detected)")
    print("    - CHIT_10 (if 10 peso detected)")
    print("    - CHIT_20 (if 20 peso detected)")
    print("    - CHIT_50 (if 50 peso detected)")
    print("    - CHIT_UNKNOWN (if no valid chit)")
    print(">>> Note: This takes ~5 seconds for detection")

def test_display():
    """Test LCD display"""
    print("\nEnter LCD message (use | to separate lines):")
    print("Example: Line 1|Line 2|Line 3|Line 4")
    message = input("Message: ")
    send_command(f"DISPLAY:{message}")
    print(">>> Expected response: 'DISPLAY_OK'")

def test_dispense():
    """Test servo dispense"""
    send_command("DISPENSE_CHIT")
    print(">>> Expected response: 'DISPENSE_COMPLETE'")
    print(">>> Servo should move from 39° to 90° and back")

def test_ping():
    """Test communication"""
    send_command("PING")
    print(">>> Expected response: 'PONG'")

def test_reset():
    """Test reset"""
    send_command("RESET")
    print(">>> Expected response: 'RESET_OK'")

def test_full_workflow():
    """Test complete transaction workflow"""
    print("\n" + "="*60)
    print("FULL WORKFLOW TEST")
    print("="*60)
    print("\nThis simulates a complete chit exchange transaction:\n")
    
    # Step 1: Check IR
    print("\n[STEP 1] Checking if chit is inserted...")
    send_command("CHECK_IR")
    input(">>> Press Enter to continue...")
    
    # Step 2: Detect chit
    print("\n[STEP 2] Running YOLO detection to identify chit...")
    send_command("DETECT_CHIT")
    print(">>> (Wait ~5 seconds for detection)")
    input(">>> Press Enter to continue...")
    
    # Step 3: Display chit detected
    print("\n[STEP 3] Displaying chit detection result...")
    send_command("DISPLAY:Chit Detected!|Value: P50|Processing...|")
    input(">>> Press Enter to continue...")
    
    # Step 4: Display calculation
    print("\n[STEP 4] ESP32 calculates coin combination...")
    print(">>> (ESP32 side: calculating optimal coins)")
    time.sleep(1)
    
    # Step 5: Display dispensing
    print("\n[STEP 5] Displaying coin dispensing status...")
    send_command("DISPLAY:Dispensing Coins|5P: 0  10P: 1|20P: 2  Total: 50|Please wait...")
    input(">>> Press Enter to continue...")
    
    # Step 6: Display dispensing progress
    print("\n[STEP 6] Updating dispensing progress...")
    send_command("DISPLAY:Dispensing...|Progress: 50%|20P: 1/2 coins|")
    input(">>> Press Enter to continue...")
    
    # Step 7: Display complete
    print("\n[STEP 7] Coin dispensing complete...")
    send_command("DISPLAY:Dispensing|Complete!|Releasing chit...|")
    input(">>> Press Enter to continue...")
    
    # Step 8: Dispense chit
    print("\n[STEP 8] Releasing chit via servo...")
    send_command("DISPENSE_CHIT")
    input(">>> Press Enter to continue...")
    
    # Step 9: Display thank you
    print("\n[STEP 9] Displaying thank you message...")
    send_command("DISPLAY:Transaction|Complete!|Thank you!|")
    input(">>> Press Enter to continue...")
    
    # Step 10: Reset
    print("\n[STEP 10] Resetting to ready state...")
    send_command("RESET")
    
    print("\n" + "="*60)
    print("FULL WORKFLOW TEST COMPLETE!")
    print("="*60)

def main():
    """Main test program"""
    print("\n" + "="*60)
    print("ESP32 SLAVE COMMUNICATION SYSTEM - TEST UTILITY")
    print("="*60)
    print("\nThis program simulates ESP32 commands to test the slave system.")
    print("Run this alongside the actual esp32_comm.py to verify responses.")
    print("\nIMPORTANT: Start esp32_comm.py first in another terminal!")
    print("\nFor actual ESP32 integration, use Serial.println() to send commands")
    print("and Serial.readStringUntil('\\n') to receive responses.")
    
    input("\nPress Enter when esp32_comm.py is running...")
    
    while True:
        print_menu()
        choice = input("\nEnter choice (0-7): ").strip()
        
        if choice == "0":
            print("\nExiting test program. Goodbye!")
            break
        elif choice == "1":
            test_ir()
        elif choice == "2":
            test_detect()
        elif choice == "3":
            test_display()
        elif choice == "4":
            test_dispense()
        elif choice == "5":
            test_ping()
        elif choice == "6":
            test_reset()
        elif choice == "7":
            test_full_workflow()
        else:
            print("\n❌ Invalid choice. Please enter 0-7.")
        
        input("\nPress Enter to continue...")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTest program interrupted. Exiting...")
        sys.exit(0)
