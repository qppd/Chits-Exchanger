# servo_tester.py
# Simple script to test servo motor functionality




import time
import pigpio

PIN = 22  # BCM numbering

pi = pigpio.pi()
if not pi.connected:
    print("Could not connect to pigpio daemon!")
    exit(1)

try:
    while True:
        print("GPIO22 HIGH")
        pi.write(PIN, 1)
        time.sleep(5)
        print("GPIO22 LOW")
        pi.write(PIN, 0)
        time.sleep(5)
except KeyboardInterrupt:
    pass
finally:
    pi.write(PIN, 0)
    pi.stop()
    
    print("Servo test finished.")
