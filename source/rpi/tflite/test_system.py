#!/usr/bin/env python3
# test_system.py - Test individual components and system integration

import time
import sys
from sensor import IRSensor
from servo import ServoMotor
from classifier import ImageClassifier
from hopper import AllanCoinHopper

def test_ir_sensor():
    """Test IR sensor functionality"""
    print("=== Testing IR Sensor ===")
    try:
        sensor = IRSensor(pin=17)
        print("IR sensor initialized successfully")
        
        print("Testing object detection (timeout in 5 seconds)...")
        if sensor.wait_for_object(timeout=5):
            print("✓ Object detection working")
        else:
            print("✗ No object detected within timeout")
            
        sensor.cleanup()
        print("IR sensor test completed\n")
        return True
    except Exception as e:
        print(f"✗ IR sensor test failed: {e}\n")
        return False

def test_servo():
    """Test servo motor functionality"""
    print("=== Testing Servo Motor ===")
    try:
        servo = ServoMotor(pin=18)
        print("Servo motor initialized successfully")
        
        print("Testing servo movements...")
        servo.move_to_capture_position()
        time.sleep(2)
        servo.move_to_idle_position()
        time.sleep(2)
        
        servo.cleanup()
        print("✓ Servo motor test completed\n")
        return True
    except Exception as e:
        print(f"✗ Servo motor test failed: {e}\n")
        return False

def test_classifier():
    """Test image classifier functionality"""
    print("=== Testing Image Classifier ===")
    try:
        classifier = ImageClassifier()
        print("Image classifier initialized successfully")
        
        print("Testing single image capture and classification...")
        image = classifier.capture_image()
        if image is not None:
            label, confidence = classifier.classify_image(image)
            print(f"✓ Classification result: {label} (confidence: {confidence:.3f})")
        else:
            print("✗ Failed to capture image")
            
        print("Image classifier test completed\n")
        return True
    except Exception as e:
        print(f"✗ Image classifier test failed: {e}\n")
        return False

def test_hopper():
    """Test ALLAN coin hopper functionality"""
    print("=== Testing ALLAN Coin Hopper ===")
    try:
        hopper = AllanCoinHopper(pin_5_peso=22, pin_10_peso=23, pin_20_peso=24)
        print("ALLAN coin hopper initialized successfully")
        
        print("Testing all hoppers (0.5 second duration each)...")
        hopper.test_all_hoppers(duration=0.5)
        
        hopper.cleanup()
        print("✓ ALLAN coin hopper test completed\n")
        return True
    except Exception as e:
        print(f"✗ ALLAN coin hopper test failed: {e}\n")
        return False

def test_full_system():
    """Test complete system integration"""
    print("=== Testing Full System Integration ===")
    try:
        from main import ChitExchangerSystem
        
        print("Initializing full system...")
        system = ChitExchangerSystem()
        
        print("System initialized successfully!")
        print("Running one complete workflow cycle...")
        
        # Simulate object detection and processing
        system.process_detected_object()
        
        system.cleanup()
        print("✓ Full system integration test completed\n")
        return True
    except Exception as e:
        print(f"✗ Full system integration test failed: {e}\n")
        return False

def main():
    """Run all tests"""
    print("Chit Exchanger System - Component Tests")
    print("=" * 50)
    
    tests = [
        ("IR Sensor", test_ir_sensor),
        ("Servo Motor", test_servo),
        ("Image Classifier", test_classifier),
        ("ALLAN Coin Hopper", test_hopper),
        ("Full System", test_full_system)
    ]
    
    results = {}
    
    for test_name, test_func in tests:
        try:
            results[test_name] = test_func()
        except KeyboardInterrupt:
            print("\nTest interrupted by user")
            break
        except Exception as e:
            print(f"✗ {test_name} test crashed: {e}\n")
            results[test_name] = False
    
    # Summary
    print("=" * 50)
    print("Test Results Summary:")
    for test_name, result in results.items():
        status = "✓ PASS" if result else "✗ FAIL"
        print(f"{test_name}: {status}")
    
    passed = sum(results.values())
    total = len(results)
    print(f"\nOverall: {passed}/{total} tests passed")

if __name__ == '__main__':
    main()