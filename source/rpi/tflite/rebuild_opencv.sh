#!/bin/bash

# Remove existing OpenCV packages
sudo apt-get remove -y python3-opencv opencv-python-headless

# Activate virtual environment
source ./venv_tflite/bin/activate

# Install OpenCV with GUI support
pip install opencv-python==4.8.0.74

# Test import
python3 -c "import cv2; print('OpenCV version:', cv2.__version__)"