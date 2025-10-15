#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time
import signal
import sys

# GPIO pin for the tactile button
BUTTON_PIN = 17

def setup():
    # Set GPIO mode to BCM
    GPIO.setmode(GPIO.BCM)
    # Set up the button pin as input with pull-up resistor
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def cleanup(signum=None, frame=None):
    print("\nCleaning up...")
    GPIO.cleanup()
    sys.exit(0)

def button_callback(channel):
    # Button is active LOW with pull-up resistor
    if GPIO.input(channel):
        print("Button RELEASED")
    else:
        print("Button PRESSED")

def main():
    try:
        # Setup GPIO
        setup()
        
        # Register cleanup handler for graceful exit
        signal.signal(signal.SIGINT, cleanup)
        signal.signal(signal.SIGTERM, cleanup)

        # Add event detection for both rising and falling edges
        GPIO.add_event_detect(BUTTON_PIN, GPIO.BOTH, callback=button_callback, bouncetime=200)

        print("Button Tester Started")
        print("Press the button (connected to GPIO 17) to test")
        print("Press Ctrl+C to exit")

        # Keep the script running
        while True:
            time.sleep(0.1)

    except Exception as e:
        print(f"Error: {e}")
        cleanup()

if __name__ == "__main__":
    main()