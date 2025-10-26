#!/bin/bash
echo "Activating virtual environment..."
source /home/admin/Chits-Exchanger/source/rpi/yolo/venv/bin/activate

echo "Running YOLO detection..."
python3 /home/admin/Chits-Exchanger/source/rpi/yolo/yolo_detect.py \
  --model=/home/admin/Chits-Exchanger/source/rpi/yolo/yolo11n_ncnn_model \
  --source=usb0 \
  --resolution=640x480