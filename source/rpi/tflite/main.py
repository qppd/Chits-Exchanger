# main.py for rpi/tflite
# Entry point for TensorFlow Lite model inference on Raspberry Pi

import tensorflow as tf
import numpy as np
import os

MODEL_PATH = os.path.join(os.path.dirname(__file__), '../../ml/converted_tflite_quantized/model.tflite')
LABELS_PATH = os.path.join(os.path.dirname(__file__), '../../ml/converted_tflite_quantized/labels.txt')

# Load TFLite model and allocate tensors.
interpreter = tf.lite.Interpreter(model_path=MODEL_PATH)
interpreter.allocate_tensors()

# Get input and output tensors.
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Load labels
with open(LABELS_PATH, 'r') as f:
    labels = [line.strip() for line in f.readlines()]

def preprocess_input(image_path):
    # Dummy preprocessing: replace with actual image preprocessing
    img = np.zeros(input_details[0]['shape'], dtype=np.float32)
    return img

def run_inference(image_path):
    input_data = preprocess_input(image_path)
    interpreter.set_tensor(input_details[0]['index'], input_data)
    interpreter.invoke()
    output_data = interpreter.get_tensor(output_details[0]['index'])
    predicted_label = labels[np.argmax(output_data)]
    print(f'Predicted: {predicted_label}')

if __name__ == '__main__':
    # Example usage
    test_image = 'test.jpg'  # Replace with actual image path
    run_inference(test_image)
