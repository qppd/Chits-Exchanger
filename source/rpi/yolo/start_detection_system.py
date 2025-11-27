#!/usr/bin/env python3
"""
Start Detection System Launcher
- Starts yolo_detect_optimized.py and esp32_comm.py in separate processes
- Manages inter-process communication via multiprocessing.Queue
- Handles graceful shutdown of both modules
"""

import sys
import time
import argparse
from multiprocessing import Process, Queue
import signal


def print_header():
    print(f"\n{'='*70}")
    print(f"🚀 CHITS EXCHANGER - DETECTION SYSTEM LAUNCHER")
    print(f"{'='*70}\n")


def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print("\n\n⚠️  Shutdown signal received. Cleaning up...\n")
    sys.exit(0)


def start_yolo_detection(detection_queue, ir_trigger_queue, yolo_config):
    """Start YOLO detection process"""
    from yolo_detect_optimized import run_detection_loop
    try:
        run_detection_loop(detection_queue, ir_trigger_queue, yolo_config)
    except Exception as e:
        print(f"❌ YOLO process error: {e}")


def start_esp32_communication(detection_queue, ir_trigger_queue, esp32_args):
    """Start ESP32 communication process"""
    from esp32_comm import run_esp32_comm_loop
    try:
        run_esp32_comm_loop(detection_queue, ir_trigger_queue, esp32_args['esp32_port'])
    except Exception as e:
        print(f"❌ ESP32 process error: {e}")


def main():
    parser = argparse.ArgumentParser(description="Start Chits Exchanger detection system")
    
    # YOLO detection arguments
    parser.add_argument('--model', help='Path to YOLO model file (required)',
                        required=True)
    parser.add_argument('--thresh', help='Confidence threshold (default: 0.5)',
                        default=0.5, type=float)
    parser.add_argument('--resolution', help='Display resolution in WxH (example: "640x480")',
                        default=None)
    parser.add_argument('--inference-size', help='YOLO inference size (default: 320)',
                        default=320, type=int)
    parser.add_argument('--display', help='Show real-time detection window',
                        action='store_true')
    parser.add_argument('--camera', help='USB camera device ID (default: 0)',
                        default='0')
    parser.add_argument('--confirmation-frames', help='Frames to confirm detection (default: 3)',
                        default=3, type=int)
    
    # ESP32 communication arguments
    parser.add_argument('--esp32_port', help='Serial port for ESP32 (default: /dev/ttyUSB0)',
                        default='/dev/ttyUSB0')
    
    args = parser.parse_args()
    
    print_header()
    
    print(f"YOLO Configuration:")
    print(f"  Model: {args.model}")
    print(f"  Confidence Threshold: {args.thresh}")
    print(f"  Inference Size: {args.inference_size}x{args.inference_size}")
    print(f"  Camera Device: /dev/video{args.camera}")
    print(f"  Display Enabled: {args.display}")
    print(f"  Confirmation Frames: {args.confirmation_frames}")
    
    print(f"\nESP32 Configuration:")
    print(f"  Serial Port: {args.esp32_port}")
    
    print(f"\n{'='*70}\n")
    
    # Create IPC queues
    detection_queue = Queue()  # YOLO -> ESP32
    ir_trigger_queue = Queue()  # ESP32 -> YOLO
    
    # Prepare arguments for each process
    yolo_args = {
        'model': args.model,
        'thresh': args.thresh,
        'resolution': args.resolution,
        'inference_size': args.inference_size,
        'display': args.display,
        'camera': args.camera,
        'confirmation_frames': args.confirmation_frames,
    }
    
    esp32_args = {
        'esp32_port': args.esp32_port,
    }
    
    # Set up signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    
    # Start both processes
    print("Starting processes...\n")
    
    yolo_process = Process(
        target=start_yolo_detection,
        args=(detection_queue, ir_trigger_queue, yolo_args),
        name="YOLO-Detection"
    )
    
    esp32_process = Process(
        target=start_esp32_communication,
        args=(detection_queue, ir_trigger_queue, esp32_args),
        name="ESP32-Communication"
    )
    
    yolo_process.start()
    time.sleep(1)  # Give YOLO time to initialize
    esp32_process.start()
    
    print(f"✅ YOLO Detection process started (PID: {yolo_process.pid})")
    print(f"✅ ESP32 Communication process started (PID: {esp32_process.pid})\n")
    print(f"{'='*70}\n")
    print("🎉 Detection system running. Press Ctrl+C to stop.\n")
    
    try:
        # Wait for processes to complete
        yolo_process.join()
        esp32_process.join()
    except KeyboardInterrupt:
        print("\n\nTerminating processes...")
        yolo_process.terminate()
        esp32_process.terminate()
        yolo_process.join(timeout=5)
        esp32_process.join(timeout=5)
        print("✅ All processes terminated")


if __name__ == '__main__':
    main()
