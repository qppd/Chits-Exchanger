"""
Optimized YOLO Detection Module
- Runs continuous real-time detection after IR trigger
- Communicates detection results via multiprocessing.Queue
- Listens for IR trigger signal from esp32_comm process
- No GPIO/serial/LCD code (keeps it fast and focused)
"""

import os
import sys
import time
import cv2
import numpy as np
from ultralytics import YOLO

# Determine if GUI is available
use_gui = "DISPLAY" in os.environ

# Valid chit denominations
VALID_CHITS = [5, 10, 20, 50]

# Set bounding box colors
bbox_colors = [(164,120,87), (68,148,228), (93,97,209), (178,182,133), (88,159,106), 
              (96,202,231), (159,124,168), (169,162,241), (98,118,150), (172,176,184)]


def process_frame_detection(frame, model, labels, min_thresh, inference_size):
    """Process frame with YOLO inference"""
    results = model(frame, verbose=False, imgsz=inference_size)
    detections = results[0].boxes
    
    detected_chits = []
    display_frame = frame.copy()
    
    for i in range(len(detections)):
        conf = detections[i].conf.item()
        
        if conf > min_thresh:
            classidx = int(detections[i].cls.item())
            classname = labels[classidx]
            
            xyxy_tensor = detections[i].xyxy.cpu()
            xyxy = xyxy_tensor.numpy().squeeze()
            xmin, ymin, xmax, ymax = xyxy.astype(int)
            
            color = bbox_colors[classidx % 10]
            cv2.rectangle(display_frame, (xmin, ymin), (xmax, ymax), color, 2)
            
            label = f'{classname}: {int(conf*100)}%'
            labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
            label_ymin = max(ymin, labelSize[1] + 10)
            cv2.rectangle(display_frame, (xmin, label_ymin-labelSize[1]-10), 
                        (xmin+labelSize[0], label_ymin+baseLine-10), color, cv2.FILLED)
            cv2.putText(display_frame, label, (xmin, label_ymin-7), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
            
            try:
                chit_value = int(classname)
                if chit_value in VALID_CHITS:
                    detected_chits.append((chit_value, conf))
            except ValueError:
                pass
    
    return display_frame, detected_chits


def get_confirmed_detection(buffer, confirmation_frames):
    """Analyze detection buffer and return confirmed chit value"""
    if len(buffer) < confirmation_frames:
        return None, 0.0
    
    value_counts = {}
    value_confidences = {}
    
    for chit_value, conf in buffer[-confirmation_frames:]:
        if chit_value not in value_counts:
            value_counts[chit_value] = 0
            value_confidences[chit_value] = []
        value_counts[chit_value] += 1
        value_confidences[chit_value].append(conf)
    
    if value_counts:
        most_common_value = max(value_counts.items(), key=lambda x: x[1])
        chit_value, count = most_common_value
        
        if count >= confirmation_frames * 0.6:
            avg_confidence = sum(value_confidences[chit_value]) / len(value_confidences[chit_value])
            return chit_value, avg_confidence
    
    return None, 0.0


def run_detection_loop(detection_queue, ir_trigger_queue, config):
    """
    Main detection loop
    - Waits for IR trigger signal from ir_trigger_queue
    - Runs real-time detection until confirmation
    - Sends confirmed detection to detection_queue
    """
    # Extract config
    model_path = config['model']
    min_thresh = config['thresh']
    user_res = config['resolution']
    inference_size = config['inference_size']
    display_enabled = config['display']
    img_source = int(config['camera'])
    confirmation_frames = config['confirmation_frames']
    
    # Check if model exists
    if not os.path.exists(model_path):
        print(f'ERROR: Model path is invalid: {model_path}')
        return
    
    print(f"\n{'='*60}")
    print(f"🚀 YOLO DETECTION MODULE - STARTUP")
    print(f"{'='*60}")
    print(f"Model: {model_path}")
    print(f"Camera: /dev/video{img_source}")
    print(f"Inference Size: {inference_size}x{inference_size}")
    print(f"Display: {display_enabled}")
    print(f"Confirmation Frames: {confirmation_frames}")
    print(f"{'='*60}\n")
    
    # Parse resolution
    resize = False
    resW, resH = None, None
    if user_res:
        resize = True
        resW, resH = int(user_res.split('x')[0]), int(user_res.split('x')[1])
    
    # Load YOLO model
    print(f"⏳ Loading YOLO model: {model_path}")
    model_load_start = time.time()
    model = YOLO(model_path, task='detect')
    labels = model.names
    model_load_time = time.time() - model_load_start
    print(f"✅ Model loaded in {model_load_time:.2f}s")
    print(f"   Classes: {list(labels.values())}\n")
    
    # Initialize USB webcam
    cap = cv2.VideoCapture(img_source)
    if not cap.isOpened():
        print(f"❌ Failed to open USB camera /dev/video{img_source}")
        return
    
    # Set camera resolution if specified
    if user_res:
        cap.set(3, resW)
        cap.set(4, resH)
    
    print(f"✅ USB camera initialized")
    print("\n✅ YOLO Detection loop ready")
    print("Waiting for IR trigger signal...\n")
    
    frame_count = 0
    fps = 0.0
    fps_start_time = time.time()
    
    detection_state = "WAITING"
    detection_buffer = []
    t_detect_start = None
    
    while True:
        # Non-blocking check for IR trigger
        try:
            ir_signal = ir_trigger_queue.get_nowait()
            if ir_signal == "IR_DETECTED":
                print(f"\n{'='*60}")
                print("🔍 IR TRIGGER RECEIVED - Starting detection")
                print(f"{'='*60}")
                detection_state = "DETECTING"
                detection_buffer.clear()
                t_detect_start = time.time()
        except:
            pass  # Queue empty
        
        # Capture frame
        ret, frame = cap.read()
        if not ret or frame is None:
            time.sleep(0.01)
            continue
        
        if resize:
            frame = cv2.resize(frame, (resW, resH))
        
        # Calculate FPS
        frame_count += 1
        if frame_count % 30 == 0:
            fps_end_time = time.time()
            fps = 30 / (fps_end_time - fps_start_time)
            fps_start_time = fps_end_time
        
        # Process frame
        display_frame, detected_chits = process_frame_detection(frame, model, labels, min_thresh, inference_size)
        
        if detection_state == "WAITING":
            # Just show waiting status if display is enabled
            if display_enabled and use_gui:
                cv2.putText(display_frame, f'FPS: {fps:.1f}', (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(display_frame, 'Status: Waiting for IR', (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                cv2.imshow('YOLO Detection', display_frame)
                cv2.waitKey(1)
        
        elif detection_state == "DETECTING":
            # Actively detecting
            if detected_chits:
                best_detection = max(detected_chits, key=lambda x: x[1])
                detection_buffer.append(best_detection)
                print(f"   Frame: ₱{best_detection[0]} | Conf: {best_detection[1]:.2%}")
            else:
                detection_buffer.append((None, 0.0))
            
            if len(detection_buffer) > confirmation_frames * 2:
                detection_buffer.pop(0)
            
            # Check for confirmed detection
            confirmed_value, avg_conf = get_confirmed_detection(detection_buffer, confirmation_frames)
            
            if confirmed_value:
                print(f"\n{'='*60}")
                print(f"✅ DETECTION CONFIRMED: ₱{confirmed_value}")
                print(f"   Confidence: {avg_conf:.2%}")
                print(f"{'='*60}\n")
                
                # Send to esp32_comm
                detection_queue.put(("CHIT_DETECTED", confirmed_value, avg_conf))
                
                # Reset state
                detection_state = "WAITING"
                detection_buffer.clear()
            
            elif (time.time() - t_detect_start) > 3.0:
                # Timeout
                print(f"\n❌ Detection timeout (no consistent detection)")
                detection_queue.put(("DETECTION_TIMEOUT", None, 0.0))
                detection_state = "WAITING"
                detection_buffer.clear()
            
            # Show detecting status
            if display_enabled and use_gui:
                cv2.putText(display_frame, f'FPS: {fps:.1f}', (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(display_frame, 'Status: Detecting...', (10, 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.putText(display_frame, f'Buffer: {len(detection_buffer)}/{confirmation_frames}', 
                           (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.imshow('YOLO Detection', display_frame)
                cv2.waitKey(1)
        
        # Very small delay for responsiveness
        time.sleep(0.01)
    
    # Cleanup on exit
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    import argparse
    from multiprocessing import Queue
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', required=True)
    parser.add_argument('--thresh', default=0.5, type=float)
    parser.add_argument('--resolution', default=None)
    parser.add_argument('--inference-size', default=320, type=int)
    parser.add_argument('--display', action='store_true')
    parser.add_argument('--camera', default='0')
    parser.add_argument('--confirmation-frames', default=3, type=int)
    args = parser.parse_args()
    
    config = {
        'model': args.model,
        'thresh': args.thresh,
        'resolution': args.resolution,
        'inference_size': args.inference_size,
        'display': args.display,
        'camera': args.camera,
        'confirmation_frames': args.confirmation_frames,
    }
    
    detection_queue = Queue()
    ir_trigger_queue = Queue()
    
    print("\n⚠️  NOTE: This module should be run via start_detection_system.py")
    print("Starting detection loop in standalone mode...\n")
    
    try:
        run_detection_loop(detection_queue, ir_trigger_queue, config)
    except KeyboardInterrupt:
        print("\n\nShutdown signal received")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        cv2.destroyAllWindows()
        print("Detection module closed.")
