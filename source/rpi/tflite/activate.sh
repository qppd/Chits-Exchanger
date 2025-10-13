#!/bin/bash
# activate.sh - Manual activation script for TensorFlow Lite environment

echo "Activating TensorFlow Lite environment..."

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$SCRIPT_DIR"

# Check if virtual environment exists
if [ ! -d "venv_tflite" ]; then
    echo "Virtual environment not found!"
    echo "Please run setup_env.sh first:"
    echo "./setup_env.sh"
    exit 1
fi

# Activate the environment
source venv_tflite/bin/activate

echo "Environment activated successfully!"
echo "Virtual environment: $(which python)"
echo "Python version: $(python --version)"
echo ""
echo "Available commands:"
echo "  python main.py          - Run the main Chit Exchanger System"
echo "  python test_system.py   - Run component tests"
echo "  python classifier.py    - Test classifier only"
echo "  deactivate             - Exit virtual environment"
echo ""

# Start a new bash session with the activated environment
exec bash