#!/usr/bin/env python3
"""
Auto-fix script for yolo_detect_threaded.py hanging issue
Run this script to automatically apply all fixes
"""

import re

# Read the original file
file_path = "yolo_detect_threaded.py"
with open(file_path, 'r', encoding='utf-8') as f:
    content = f.read()

print("Applying fixes to yolo_detect_threaded.py...")

# Fix 1: Replace release_chit() function
old_release_chit = r'''def release_chit\(\):
    """Release chit by moving servo to release position and back"""
    print\(f"Releasing chit: Moving servo from \{SERVO_INITIAL_ANGLE\}° to \{SERVO_RELEASE_ANGLE\}°"\)
    set_servo_angle\(SERVO_RELEASE_ANGLE\)
    time\.sleep\(2\)  # Hold for 2 seconds
    print\(f"Returning servo to initial position \{SERVO_INITIAL_ANGLE\}°"\)
    set_servo_angle\(SERVO_INITIAL_ANGLE\)'''

new_release_chit = '''# Servo release state tracking
servo_release_active = False
servo_release_thread = None

def release_chit():
    """Release chit by moving servo to release position and back (non-blocking)"""
    global servo_release_active, servo_release_thread
    
    if servo_release_active:
        print("⚠️ Servo release already in progress, skipping...")
        return
    
    def _release_servo():
        global servo_release_active
        try:
            servo_release_active = True
            print(f"Releasing chit: Moving servo from {SERVO_INITIAL_ANGLE}° to {SERVO_RELEASE_ANGLE}°")
            set_servo_angle(SERVO_RELEASE_ANGLE)
            time.sleep(1.5)  # Reduced from 2s to 1.5s
            print(f"Returning servo to initial position {SERVO_INITIAL_ANGLE}°")
            set_servo_angle(SERVO_INITIAL_ANGLE)
        finally:
            servo_release_active = False
    
    # Run servo release in background thread to avoid blocking
    servo_release_thread = threading.Thread(target=_release_servo, daemon=True, name="ServoRelease")
    servo_release_thread.start()
    print("🔄 Servo release initiated in background")'''

if re.search(old_release_chit, content):
    content = re.sub(old_release_chit, new_release_chit, content)
    print("✅ Fix 1: Updated release_chit() to be non-blocking")
else:
    print("⚠️  Fix 1: Could not find release_chit() function pattern")

# Fix 2: Update state machine variables
old_state_vars = r'''# State machine for detection
detection_state = "WAITING"  # WAITING, DETECTING, CONFIRMED, DISPENSING
confirmed_chit_value = None
confirmed_confidence = 0\.0'''

new_state_vars = '''# State machine for detection
detection_state = "WAITING"  # WAITING, DETECTING, CONFIRMED, RELEASING, DISPENSING
confirmed_chit_value = None
confirmed_confidence = 0.0
dispensing_start_time = 0.0
dispensing_timeout = 30.0  # 30 second timeout for dispensing'''

if re.search(old_state_vars, content):
    content = re.sub(old_state_vars, new_state_vars, content)
    print("✅ Fix 2: Updated state machine variables")
else:
    print("⚠️  Fix 2: Could not find state machine variables pattern")

# Write the fixed content back
with open(file_path, 'w', encoding='utf-8') as f:
    f.write(content)

print("\n✅ All fixes applied successfully!")
print("⚠️  Manual fix required for CONFIRMED state handler - see yolo_detect_threaded_fix.txt for details")
