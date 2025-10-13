#!/usr/bin/env python3
# activate_env.py - Python activation helper for TensorFlow Lite environment

import os
import sys
import subprocess

def activate_environment():
    """Activate the virtual environment and run the main script"""
    
    # Get script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Path to virtual environment
    venv_path = os.path.join(script_dir, 'venv_tflite')
    activate_script = os.path.join(venv_path, 'bin', 'activate')
    
    if not os.path.exists(activate_script):
        print("Virtual environment not found!")
        print("Please run setup_env.sh first to create the environment:")
        print("./setup_env.sh")
        return False
    
    # Activate environment and run main script
    cmd = f"cd {script_dir} && source {activate_script} && python main.py"
    
    try:
        print("Activating environment and running Chit Exchanger System...")
        subprocess.run(cmd, shell=True, executable='/bin/bash')
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print(f"Error running script: {e}")
    
    return True

def run_tests():
    """Activate environment and run test script"""
    
    # Get script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Path to virtual environment
    venv_path = os.path.join(script_dir, 'venv_tflite')
    activate_script = os.path.join(venv_path, 'bin', 'activate')
    
    if not os.path.exists(activate_script):
        print("Virtual environment not found!")
        print("Please run setup_env.sh first.")
        return False
    
    # Activate environment and run test script
    cmd = f"cd {script_dir} && source {activate_script} && python test_system.py"
    
    try:
        print("Running system tests...")
        subprocess.run(cmd, shell=True, executable='/bin/bash')
    except KeyboardInterrupt:
        print("\nTests interrupted by user")
    except Exception as e:
        print(f"Error running tests: {e}")
    
    return True

if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'test':
        run_tests()
    else:
        activate_environment()