#!/usr/bin/env python3
"""
Quick test script for auto-detection features
Tests camera and ESP32 auto-detection without running full YOLO system
"""

import cv2
import serial
import serial.tools.list_ports

def test_camera_detection():
    """Test camera auto-detection"""
    print("\n=== Testing Camera Auto-Detection ===")
    for i in range(10):
        device = f'/dev/video{i}'
        print(f"Checking {device}...", end=" ")
        try:
            cap = cv2.VideoCapture(device)
            if cap.isOpened():
                ret, frame = cap.read()
                cap.release()
                if ret:
                    print(f"✅ FOUND! Camera working at {device}")
                    return device
                else:
                    print("❌ Device exists but can't read frames")
            else:
                print("❌ Can't open")
        except Exception as e:
            print(f"❌ Error: {e}")
    
    print("⚠️  No camera found on /dev/video0-9")
    return None

def test_esp32_detection():
    """Test ESP32 auto-detection"""
    print("\n=== Testing ESP32 Auto-Detection ===")
    ports = list(serial.tools.list_ports.comports())
    
    if not ports:
        print("⚠️  No serial ports found")
        return None
    
    print(f"Found {len(ports)} serial port(s):")
    for port in ports:
        print(f"  - {port.device}")
        print(f"    Description: {port.description}")
        print(f"    Manufacturer: {port.manufacturer}")
        
        if 'USB' in port.device or 'ACM' in port.device or 'ttyUSB' in port.device:
            print(f"    ✅ Matches ESP32 pattern - SELECTED")
            return port.device
        else:
            print(f"    ❌ Doesn't match ESP32 pattern")
    
    print("⚠️  No ESP32-compatible port found")
    return None

if __name__ == "__main__":
    print("=" * 50)
    print("Auto-Detection Test Script")
    print("=" * 50)
    
    # Test camera detection
    camera = test_camera_detection()
    
    # Test ESP32 detection
    esp32_port = test_esp32_detection()
    
    # Summary
    print("\n" + "=" * 50)
    print("DETECTION SUMMARY")
    print("=" * 50)
    print(f"Camera: {camera if camera else 'NOT FOUND'}")
    print(f"ESP32:  {esp32_port if esp32_port else 'NOT FOUND'}")
    print("=" * 50)
    
    if camera and esp32_port:
        print("\n✅ Both devices detected! Ready to run:")
        print(f"   python yolo_detect_threaded.py --model chit_model_ncnn_model --resolution 320x240")
    elif camera:
        print("\n⚠️  Camera found but no ESP32. You can still test YOLO:")
        print(f"   python yolo_detect2.py --model chit_model_ncnn_model --source {camera} --resolution 320x240")
    else:
        print("\n❌ Missing required devices. Check connections.")
