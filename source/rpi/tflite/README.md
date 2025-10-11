# Raspberry Pi 4B Chit Exchanger System

## Overview
This system uses a Raspberry Pi 4B to exchange peso bills for coins using:
- IR sensor for object detection
- Servo motor for positioning
- Camera and TensorFlow Lite for bill classification
- ALLAN coin hoppers for dispensing coins

## Hardware Setup

### GPIO Pin Assignments
- **IR Sensor**: GPIO17 (input)
- **Servo Motor**: GPIO18 (PWM output)
- **ALLAN Coin Hoppers**:
  - 5 Peso: GPIO22
  - 10 Peso: GPIO23
  - 20 Peso: GPIO24

### Required Hardware
- Raspberry Pi 4B
- IR sensor module
- Servo motor (SG90 or similar)
- USB camera or Pi camera
- 3x ALLAN coin hoppers
- 3x relay modules for hoppers
- Appropriate power supplies

## Software Setup

### 1. Install Dependencies
```bash
pip install -r requirements.txt
```

### 2. Enable Camera (if using Pi camera)
```bash
sudo raspi-config
# Navigate to Interface Options > Camera > Enable
```

### 3. Test System Components
```bash
python test_system.py
```

### 4. Run Main System
```bash
python main.py
```

## Workflow

1. **Object Detection**: IR sensor monitors for peso bills
2. **Positioning**: Servo moves to capture position when object detected
3. **Classification**: Camera captures image and TFLite model classifies bill (5, 10, 20, 50 peso)
4. **Coin Dispensing**: Appropriate ALLAN hopper dispenses coins for 5, 10, 20 peso bills
5. **Reset**: Servo returns to idle position, ready for next bill

## Files Description

- `main.py`: Main system orchestration
- `sensor.py`: IR sensor control
- `servo.py`: Servo motor control
- `classifier.py`: TensorFlow Lite image classification
- `hopper.py`: ALLAN coin hopper control
- `test_system.py`: Component testing script
- `requirements.txt`: Python dependencies

## Model Information

The TensorFlow Lite model is from Teachable Machine with labels: 5, 10, 20, 50 (peso denominations).
Model path: `../../ml/converted_tflite_quantized/model.tflite`
Labels path: `../../ml/converted_tflite_quantized/labels.txt`

## Usage Notes

- Ensure proper GPIO permissions (run as sudo if needed)
- Camera index may need adjustment based on your setup
- Servo angles can be adjusted in `servo.py` for your mechanical setup
- Hopper duration can be adjusted based on coin dispensing requirements
- System includes graceful shutdown handling (Ctrl+C)

## Troubleshooting

1. **GPIO Permission Errors**: Run with sudo or add user to gpio group
2. **Camera Not Found**: Check camera connection and enable in raspi-config
3. **Model Loading Errors**: Verify model and labels files exist
4. **Servo Not Moving**: Check power supply and wiring
5. **Hopper Not Dispensing**: Verify relay connections and power