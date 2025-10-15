#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time
import signal
import sys

# GPIO pin for IR sensor
IR_SENSOR_PIN = 27

def setup():
    # Set GPIO mode to BCM
    GPIO.setmode(GPIO.BCM)
    # Set up the IR sensor pin as input
    GPIO.setup(IR_SENSOR_PIN, GPIO.IN)

def cleanup(signum=None, frame=None):
    print("\nCleaning up...")
    GPIO.cleanup()
    sys.exit(0)

def detect_obstacle():
    # Return True if obstacle detected (sensor reads LOW)
    # Return False if no obstacle (sensor reads HIGH)
    return not GPIO.input(IR_SENSOR_PIN)

def main():
    try:
        # Setup GPIO
        setup()
        
        # Register cleanup handler for graceful exit
        signal.signal(signal.SIGINT, cleanup)
        signal.signal(signal.SIGTERM, cleanup)

        print("IR Sensor Tester Started")
        print("IR Sensor connected to GPIO 27")
        print("Press Ctrl+C to exit")
        
        # Variables for state tracking
        last_state = None
        detection_count = 0
        
        while True:
            current_state = detect_obstacle()
            
            # Only print when state changes to avoid flooding the console
            if current_state != last_state:
                detection_count += 1
                if current_state:
                    print(f"Detection #{detection_count}: Obstacle detected!")
                else:
                    print(f"Detection #{detection_count}: Path clear")
                last_state = current_state
            
            # Small delay to prevent CPU overload
            time.sleep(0.1)

    except Exception as e:
        print(f"Error: {e}")
        cleanup()

if __name__ == "__main__":
    main()