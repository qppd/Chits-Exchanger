# servo.py - Servo motor control module
import RPi.GPIO as GPIO
import time

class ServoMotor:
    def __init__(self, pin=18):
        """Initialize servo motor on specified GPIO pin"""
        self.pin = pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, 50)  # 50Hz frequency
        self.pwm.start(0)
        
    def set_angle(self, angle):
        """Set servo to specific angle (0-180 degrees)"""
        if angle < 0:
            angle = 0
        elif angle > 180:
            angle = 180
            
        # Convert angle to duty cycle (2-12% for 0-180 degrees)
        duty_cycle = 2 + (angle / 180) * 10
        self.pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(0.5)  # Allow time for servo to move
        self.pwm.ChangeDutyCycle(0)  # Stop sending signal
        
    def move_to_capture_position(self):
        """Move servo to image capture position"""
        print("Moving servo to capture position...")
        self.set_angle(90)  # Adjust angle as needed for your setup
        
    def move_to_idle_position(self):
        """Move servo to idle position"""
        print("Moving servo to idle position...")
        self.set_angle(0)  # Adjust angle as needed for your setup
        
    def cleanup(self):
        """Clean up PWM and GPIO resources"""
        self.pwm.stop()
        GPIO.cleanup()