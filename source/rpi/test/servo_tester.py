# servo_tester.py
# Simple script to test servo motor functionality




import time
import pigpio

SERVO_PIN = 22  # BCM numbering

pi = pigpio.pi()
if not pi.connected:
    print("Could not connect to pigpio daemon!")
    exit(1)

# For 180° servo:
# 0°   = 500us
# 90°  = 1500us
# 180° = 2500us

def angle_to_pulse(angle):
    # Map angle (0-180) to pulse width (500-2500us)
    return int(500 + (angle / 180.0) * 2000)

try:
    while True:
        print("Moving to 40°")
        pi.set_servo_pulsewidth(SERVO_PIN, angle_to_pulse(40))
        time.sleep(2)
        print("Moving to 90°")
        pi.set_servo_pulsewidth(SERVO_PIN, angle_to_pulse(90))
        time.sleep(2)
except KeyboardInterrupt:
    pass
finally:
    pi.set_servo_pulsewidth(SERVO_PIN, 0)  # Stop servo
    pi.stop()
    
    print("Servo test finished.")
