#!/bin/bash
echo "Activating virtual environment..."
source /home/admin/Chits-Exchanger/source/rpi/yolo/venv/bin/activate

echo "Running YOLO detection with auto-detection..."
python3 /home/admin/Chits-Exchanger/source/rpi/yolo/yolo_detect.py \
  --model=/home/admin/Chits-Exchanger/source/rpi/yolo/chit_model_ncnn_model \
  --resolution=640x480

# Note: Camera and ESP32 port are now auto-detected
# No need to specify --camera or --esp32_port parameters