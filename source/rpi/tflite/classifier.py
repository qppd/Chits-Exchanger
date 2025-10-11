# classifier.py - Image classification module using TensorFlow Lite
import tensorflow as tf
import numpy as np
import cv2
import os
from collections import Counter

class ImageClassifier:
    def __init__(self, model_path=None, labels_path=None):
        """Initialize TensorFlow Lite classifier"""
        if model_path is None:
            model_path = os.path.join(os.path.dirname(__file__), '../../ml/converted_tflite_quantized/model.tflite')
        if labels_path is None:
            labels_path = os.path.join(os.path.dirname(__file__), '../../ml/converted_tflite_quantized/labels.txt')
            
        # Load TFLite model and allocate tensors
        self.interpreter = tf.lite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()
        
        # Get input and output tensors
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        
        # Load labels
        with open(labels_path, 'r') as f:
            self.labels = [line.strip() for line in f.readlines()]
            
        # Get input shape for preprocessing
        self.input_shape = self.input_details[0]['shape']
        print(f"Model input shape: {self.input_shape}")
        print(f"Loaded labels: {self.labels}")
        
    def preprocess_image(self, image):
        """Preprocess image for Teachable Machine model"""
        # Teachable Machine typically expects 224x224 RGB images
        height, width = self.input_shape[1], self.input_shape[2]
        
        # Resize image
        image_resized = cv2.resize(image, (width, height))
        
        # Convert BGR to RGB (OpenCV uses BGR by default)
        image_rgb = cv2.cvtColor(image_resized, cv2.COLOR_BGR2RGB)
        
        # Normalize to [0, 1] range
        image_normalized = image_rgb.astype(np.float32) / 255.0
        
        # Add batch dimension
        image_batch = np.expand_dims(image_normalized, axis=0)
        
        return image_batch
        
    def capture_image(self, camera_index=0):
        """Capture image from camera"""
        cap = cv2.VideoCapture(camera_index)
        
        if not cap.isOpened():
            print("Error: Could not open camera")
            return None
            
        # Allow camera to warm up
        for _ in range(5):
            ret, frame = cap.read()
            
        ret, frame = cap.read()
        cap.release()
        
        if ret:
            print("Image captured successfully")
            return frame
        else:
            print("Error: Failed to capture image")
            return None
            
    def classify_image(self, image):
        """Classify a single image"""
        if image is None:
            return None, 0.0
            
        # Preprocess image
        input_data = self.preprocess_image(image)
        
        # Run inference
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        self.interpreter.invoke()
        output_data = self.interpreter.get_tensor(self.output_details[0]['index'])
        
        # Get prediction
        predicted_index = np.argmax(output_data)
        confidence = float(output_data[0][predicted_index])
        predicted_label = self.labels[predicted_index]
        
        return predicted_label, confidence
        
    def classify_multiple_times(self, num_classifications=5, camera_index=0):
        """Capture and classify multiple times, return majority vote"""
        print(f"Running {num_classifications} classifications...")
        
        predictions = []
        confidences = []
        
        for i in range(num_classifications):
            print(f"Classification {i+1}/{num_classifications}")
            
            # Capture image
            image = self.capture_image(camera_index)
            if image is None:
                continue
                
            # Classify
            label, confidence = self.classify_image(image)
            if label is not None:
                predictions.append(label)
                confidences.append(confidence)
                print(f"  Result: {label} (confidence: {confidence:.3f})")
                
        if not predictions:
            print("No successful classifications")
            return None, 0.0
            
        # Get majority vote
        vote_counter = Counter(predictions)
        final_prediction = vote_counter.most_common(1)[0][0]
        
        # Calculate average confidence for the final prediction
        final_confidences = [conf for pred, conf in zip(predictions, confidences) if pred == final_prediction]
        avg_confidence = sum(final_confidences) / len(final_confidences)
        
        print(f"Final prediction: {final_prediction} (avg confidence: {avg_confidence:.3f})")
        print(f"Vote breakdown: {dict(vote_counter)}")
        
        return final_prediction, avg_confidence