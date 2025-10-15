# servo_tester.py
# Simple script to test servo motor functionality

import time
import RPi.GPIO as GPIO

SERVO_PIN = 18  # Change as needed

GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)

pwm = GPIO.PWM(SERVO_PIN, 50)  # 50Hz for servo
pwm.start(0)

try:
    while True:
        for angle in range(0, 181, 10):
            duty = 2 + (angle / 18)
            pwm.ChangeDutyCycle(duty)
            print(f"Angle: {angle} -> Duty Cycle: {duty}")
            time.sleep(0.5)
        for angle in range(180, -1, -10):
            duty = 2 + (angle / 18)
            pwm.ChangeDutyCycle(duty)
            print(f"Angle: {angle} -> Duty Cycle: {duty}")
            time.sleep(0.5)
except KeyboardInterrupt:
    pass
finally:
    pwm.stop()
    GPIO.cleanup()
    print("Servo test finished.")
