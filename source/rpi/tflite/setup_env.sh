#!/bin/bash
# setup_env.sh - Setup script for Raspberry Pi TensorFlow Lite environment

echo "Setting up Python virtual environment for TensorFlow Lite on Raspberry Pi..."

# Navigate to the tflite directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$SCRIPT_DIR"

# Create virtual environment
echo "Creating virtual environment..."
python3 -m venv venv_tflite

# Activate virtual environment
echo "Activating virtual environment..."
source venv_tflite/bin/activate

# Upgrade pip
echo "Upgrading pip..."
pip install --upgrade pip

# Install TensorFlow Lite runtime (optimized for Raspberry Pi)
echo "Installing TensorFlow Lite runtime..."
pip install tflite-runtime

# Install other required packages
echo "Installing additional packages..."
pip install opencv-python
pip install numpy
pip install pillow
pip install RPi.GPIO
pip install gpiozero
pip install matplotlib

# Install packages from requirements.txt if it exists
if [ -f "requirements.txt" ]; then
    echo "Installing packages from requirements.txt..."
    pip install -r requirements.txt
fi

echo ""
echo "Environment setup complete!"
echo "Virtual environment created: venv_tflite"
echo ""
echo "To activate the environment manually, run:"
echo "source venv_tflite/bin/activate"
echo ""
echo "To deactivate, run:"
echo "deactivate"
echo ""
echo "To run the system:"
echo "./activate.sh"