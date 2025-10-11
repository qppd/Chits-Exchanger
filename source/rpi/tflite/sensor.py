# sensor.py - IR sensor detection module
import RPi.GPIO as GPIO
import time

class IRSensor:
    def __init__(self, pin=17):
        """Initialize IR sensor on specified GPIO pin"""
        self.pin = pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
    def is_object_detected(self):
        """Check if IR sensor detects an object"""
        # Returns True if object detected (LOW signal for most IR sensors)
        return not GPIO.input(self.pin)
    
    def wait_for_object(self, timeout=None):
        """Wait for object detection with optional timeout"""
        start_time = time.time()
        print("Waiting for object detection...")
        
        while True:
            if self.is_object_detected():
                print("Object detected!")
                return True
            
            if timeout and (time.time() - start_time > timeout):
                print("Timeout reached, no object detected")
                return False
                
            time.sleep(0.1)  # Small delay to prevent excessive CPU usage
    
    def cleanup(self):
        """Clean up GPIO resources"""
        GPIO.cleanup()