#!/usr/bin/env python3
"""
relay_tester.py
Manual testing script for 3 Solid State Relays (SSR) controlling coin hoppers
For CoinExchanger.ino system

Hardware:
- Relay 1 (Hopper 1 - 5 PHP): GPIO26
- Relay 2 (Hopper 2 - 10 PHP): GPIO25  
- Relay 3 (Hopper 3 - 20 PHP): GPIO33

Usage:
    python3 relay_tester.py
    
Commands:
    1 on   - Turn relay 1 ON
    1 off  - Turn relay 1 OFF
    2 on   - Turn relay 2 ON
    2 off  - Turn relay 2 OFF
    3 on   - Turn relay 3 ON
    3 off  - Turn relay 3 OFF
    all on - Turn all relays ON
    all off- Turn all relays OFF
    status - Show status of all relays
    help   - Show help message
    exit   - Exit program
"""

import RPi.GPIO as GPIO
import time
import signal
import sys

# GPIO pins for the 3 SSR relays (BCM numbering)
RELAY_1_PIN = 26  # Hopper 1 - 5 PHP coins
RELAY_2_PIN = 25  # Hopper 2 - 10 PHP coins
RELAY_3_PIN = 33  # Hopper 3 - 20 PHP coins

# Relay configuration
RELAYS = {
    1: {"pin": RELAY_1_PIN, "name": "Hopper 1 (5 PHP)", "state": False},
    2: {"pin": RELAY_2_PIN, "name": "Hopper 2 (10 PHP)", "state": False},
    3: {"pin": RELAY_3_PIN, "name": "Hopper 3 (20 PHP)", "state": False}
}

def setup():
    """Initialize GPIO pins for relay control"""
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Set up all relay pins as outputs and initialize to OFF (LOW)
    for relay_id, relay in RELAYS.items():
        GPIO.setup(relay["pin"], GPIO.OUT)
        GPIO.output(relay["pin"], GPIO.LOW)
        relay["state"] = False
    
    print("✅ GPIO initialized - All relays OFF")

def cleanup(signum=None, frame=None):
    """Clean up GPIO and exit safely"""
    print("\n🔴 Turning OFF all relays...")
    turn_all_off()
    GPIO.cleanup()
    print("✅ Cleanup complete")
    sys.exit(0)

def turn_relay_on(relay_id):
    """Turn specific relay ON"""
    if relay_id not in RELAYS:
        print(f"❌ Invalid relay ID: {relay_id}")
        return
    
    relay = RELAYS[relay_id]
    GPIO.output(relay["pin"], GPIO.HIGH)
    relay["state"] = True
    print(f"🟢 Relay {relay_id} ({relay['name']}) turned ON - GPIO{relay['pin']}")

def turn_relay_off(relay_id):
    """Turn specific relay OFF"""
    if relay_id not in RELAYS:
        print(f"❌ Invalid relay ID: {relay_id}")
        return
    
    relay = RELAYS[relay_id]
    GPIO.output(relay["pin"], GPIO.LOW)
    relay["state"] = False
    print(f"🔴 Relay {relay_id} ({relay['name']}) turned OFF - GPIO{relay['pin']}")

def turn_all_on():
    """Turn all relays ON"""
    print("🟢 Turning ON all relays...")
    for relay_id in RELAYS:
        turn_relay_on(relay_id)
    print("✅ All relays ON")

def turn_all_off():
    """Turn all relays OFF"""
    print("🔴 Turning OFF all relays...")
    for relay_id in RELAYS:
        turn_relay_off(relay_id)
    print("✅ All relays OFF")

def show_status():
    """Display current status of all relays"""
    print("\n" + "="*50)
    print("📊 RELAY STATUS")
    print("="*50)
    for relay_id, relay in RELAYS.items():
        state_icon = "🟢 ON " if relay["state"] else "🔴 OFF"
        print(f"Relay {relay_id}: {state_icon} | GPIO{relay['pin']:2d} | {relay['name']}")
    print("="*50 + "\n")

def show_help():
    """Display help message"""
    print("\n" + "="*50)
    print("🛠️  RELAY TESTER - HELP")
    print("="*50)
    print("Commands:")
    print("  1 on      - Turn relay 1 ON (Hopper 1 - 5 PHP)")
    print("  1 off     - Turn relay 1 OFF")
    print("  2 on      - Turn relay 2 ON (Hopper 2 - 10 PHP)")
    print("  2 off     - Turn relay 2 OFF")
    print("  3 on      - Turn relay 3 ON (Hopper 3 - 20 PHP)")
    print("  3 off     - Turn relay 3 OFF")
    print("  all on    - Turn all relays ON")
    print("  all off   - Turn all relays OFF")
    print("  status    - Show current status of all relays")
    print("  help      - Show this help message")
    print("  exit      - Exit program (turns off all relays)")
    print("="*50)
    print("\n💡 Hardware Connections:")
    print(f"   Relay 1: GPIO{RELAY_1_PIN} -> Hopper 1 SSR (5 PHP)")
    print(f"   Relay 2: GPIO{RELAY_2_PIN} -> Hopper 2 SSR (10 PHP)")
    print(f"   Relay 3: GPIO{RELAY_3_PIN} -> Hopper 3 SSR (20 PHP)")
    print("="*50 + "\n")

def process_command(command):
    """Process user commands"""
    command = command.strip().lower()
    
    if not command:
        return True
    
    parts = command.split()
    
    if command == "exit" or command == "quit":
        return False
    
    elif command == "help":
        show_help()
    
    elif command == "status":
        show_status()
    
    elif command == "all on":
        turn_all_on()
    
    elif command == "all off":
        turn_all_off()
    
    elif len(parts) == 2:
        # Format: <relay_id> <on/off>
        try:
            relay_id = int(parts[0])
            action = parts[1]
            
            if relay_id not in [1, 2, 3]:
                print("❌ Invalid relay ID. Use 1, 2, or 3")
                return True
            
            if action == "on":
                turn_relay_on(relay_id)
            elif action == "off":
                turn_relay_off(relay_id)
            else:
                print("❌ Invalid action. Use 'on' or 'off'")
        
        except ValueError:
            print("❌ Invalid command format. Type 'help' for usage")
    
    else:
        print("❌ Invalid command. Type 'help' for usage")
    
    return True

def main():
    """Main program loop"""
    try:
        # Setup GPIO
        setup()
        
        # Register cleanup handlers for graceful exit
        signal.signal(signal.SIGINT, cleanup)
        signal.signal(signal.SIGTERM, cleanup)
        
        # Display banner
        print("\n" + "="*50)
        print("🔌 RELAY TESTER FOR COINEXCHANGER")
        print("="*50)
        print("Testing 3 SSR relays for coin hopper control")
        print("Type 'help' for commands, 'exit' to quit\n")
        
        show_status()
        
        # Main command loop
        while True:
            try:
                command = input("relay> ")
                if not process_command(command):
                    break
            
            except EOFError:
                break
            
            except KeyboardInterrupt:
                cleanup()
        
        # Clean exit
        cleanup()
    
    except Exception as e:
        print(f"\n❌ Error: {e}")
        cleanup()

if __name__ == "__main__":
    main()
