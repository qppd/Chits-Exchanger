# classifier.py - Image classification module using TensorFlow Lite for Teachable Machine
import numpy as np
import cv2
from PIL import Image
import time
import os
from collections import Counter

# Try to import tflite_runtime first (for Raspberry Pi), fall back to tensorflow
try:
    import tflite_runtime.interpreter as tflite
    print("Using tflite_runtime")
except ImportError:
    import tensorflow as tf
    tflite = tf.lite
    print("Using tensorflow.lite")

class ImageClassifier:
    def __init__(self, model_path=None, labels_path=None, camera_url="http://192.168.1.15/stream", enable_gui=False):
        """Initialize TensorFlow Lite classifier for Teachable Machine model"""
        
        # Default paths for Teachable Machine exported model
        if model_path is None:
            model_path = os.path.join(os.path.dirname(__file__), '../../../ml/converted_tflite_quantized/model.tflite')
        if labels_path is None:
            labels_path = os.path.join(os.path.dirname(__file__), '../../../ml/converted_tflite_quantized/labels.txt')
            
        self.model_path = os.path.abspath(model_path)
        self.labels_path = os.path.abspath(labels_path)
        self.camera_url = camera_url
        
        print(f"Loading model from: {self.model_path}")
        print(f"Loading labels from: {self.labels_path}")
        
        try:
            # Load TensorFlow Lite model
            self.interpreter = tflite.Interpreter(model_path=self.model_path)
            self.interpreter.allocate_tensors()
            
            # Get input and output details
            self.input_details = self.interpreter.get_input_details()
            self.output_details = self.interpreter.get_output_details()
            
            # Get input shape (Teachable Machine typically uses 224x224)
            self.input_shape = self.input_details[0]['shape']
            self.input_height = self.input_shape[1]
            self.input_width = self.input_shape[2]
            
            print(f"Model input shape: {self.input_shape}")
            print(f"Input size: {self.input_width}x{self.input_height}")
            
            # Check if model is quantized
            self.is_quantized = self.input_details[0]['dtype'] == np.uint8
            print(f"Model is quantized: {self.is_quantized}")
            
            # Load labels (Teachable Machine format)
            self.labels = self.load_teachable_machine_labels()
            print(f"Loaded {len(self.labels)} labels: {self.labels}")
            
            # Set GUI mode based on parameter and display availability
            self.gui_mode = enable_gui and self._check_display_available()
            print(f"GUI mode enabled: {self.gui_mode}")
            
            # Initialize IP camera
            self.camera = cv2.VideoCapture(self.camera_url)
            if not self.camera.isOpened():
                raise Exception(f"Could not open IP camera at {self.camera_url}")
            
            # Warm up camera
            print("Warming up camera...")
            for _ in range(5):
                ret, frame = self.camera.read()
                time.sleep(0.1)
            
            print("Camera initialized successfully")
            
        except Exception as e:
            print(f"Error initializing classifier: {e}")
            raise
    
    def _check_display_available(self):
        """Check if display is available (GUI mode vs headless)"""
        try:
            import os
            display = os.environ.get('DISPLAY')
            if display:
                return True
            # Also check for VNC or other display methods
            return False
        except:
            return False
    
    def load_teachable_machine_labels(self):
        """Load labels from Teachable Machine format"""
        labels = []
        try:
            with open(self.labels_path, 'r') as f:
                for line in f.readlines():
                    # Teachable Machine format: "0 Class_Name" or just "Class_Name"
                    label = line.strip()
                    if label:
                        # Remove index if present (e.g., "0 5_peso" -> "5_peso")
                        if ' ' in label and label[0].isdigit():
                            label = label.split(' ', 1)[1]
                        labels.append(label)
            return labels
        except Exception as e:
            print(f"Error loading labels: {e}")
            # Fallback labels for peso bills
            return ["5", "10", "20", "50"]
    
    def preprocess_teachable_machine_image(self, image):
        """Preprocess image for Teachable Machine model"""
        try:
            # Resize to model input size (typically 224x224 for Teachable Machine)
            image_resized = cv2.resize(image, (self.input_width, self.input_height))
            
            # Convert BGR to RGB
            image_rgb = cv2.cvtColor(image_resized, cv2.COLOR_BGR2RGB)
            
            # Convert to PIL Image for consistent preprocessing
            pil_image = Image.fromarray(image_rgb)
            
            # Convert to numpy array
            input_data = np.array(pil_image, dtype=np.float32)
            
            if self.is_quantized:
                # For quantized models, keep values in 0-255 range
                input_data = input_data.astype(np.uint8)
            else:
                # For float models, normalize to 0-1 range (Teachable Machine standard)
                input_data = input_data / 255.0
            
            # Add batch dimension
            input_data = np.expand_dims(input_data, axis=0)
            
            return input_data
            
        except Exception as e:
            print(f"Error preprocessing image: {e}")
            return None
    
    def capture_image(self, show_preview=True, save_debug=True):
        """Capture image from camera with optional GUI preview"""
        try:
            print("Attempting to read from camera...")
            ret, frame = self.camera.read()
            if not ret:
                print("Failed to capture image: camera.read() returned False")
                return None
            print("Successfully read frame from camera")
            
            # Show preview if GUI mode is available and requested
            if self.gui_mode and show_preview:
                preview_frame = frame.copy()
                cv2.putText(preview_frame, "Capturing for classification...", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.imshow("Chit Exchanger - Camera Preview", preview_frame)
                cv2.waitKey(1000)  # Show for 1 second
                cv2.destroyAllWindows()
            
            # Save debug image if requested
            if save_debug:
                timestamp = int(time.time())
                debug_path = f"debug_capture_{timestamp}.jpg"
                cv2.imwrite(debug_path, frame)
                print(f"Debug image saved: {debug_path}")
            
            return frame
            
        except Exception as e:
            print(f"Error capturing image: {e}")
            return None
    
    def classify_image(self, image):
        """Classify a single image using Teachable Machine model"""
        try:
            # Preprocess image
            input_data = self.preprocess_teachable_machine_image(image)
            if input_data is None:
                return None, 0.0
            
            # Set input tensor
            self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
            
            # Run inference
            start_time = time.time()
            self.interpreter.invoke()
            inference_time = time.time() - start_time
            
            # Get output
            output_data = self.interpreter.get_tensor(self.output_details[0]['index'])
            
            # Get prediction
            prediction_index = np.argmax(output_data[0])
            confidence = float(output_data[0][prediction_index])
            
            # Normalize confidence (it's coming from a quantized model)
            if self.is_quantized:
                confidence = confidence / 255.0
            
            # Convert to percentage
            confidence_percent = confidence * 100
            
            print("Processing prediction...")
            # Get label
            if prediction_index < len(self.labels):
                predicted_label = self.labels[prediction_index]
            else:
                predicted_label = f"Unknown_{prediction_index}"
            
            print(f"Classification complete: {predicted_label} ({confidence_percent:.2f}%) - Time: {inference_time:.3f}s")
            
            return predicted_label, confidence_percent
            
        except Exception as e:
            print(f"Error during classification: {e}")
            return None, 0.0
    
    def control_flash(self, state):
        """Control the IP camera's flash LED"""
        try:
            import urllib.request
            base_url = self.camera_url.rsplit('/', 1)[0]  # Remove 'stream' from the end
            url = f"{base_url}/flash/{'on' if state else 'off'}"
            print(f"Sending flash {'ON' if state else 'OFF'} request to: {url}")
            
            # Create request with timeout
            request = urllib.request.Request(url)
            response = urllib.request.urlopen(request, timeout=2)  # 2 second timeout
            
            if response.status == 200:
                print(f"Flash {'ON' if state else 'OFF'} command successful")
            else:
                print(f"Flash control failed with status: {response.status}")
                
        except urllib.error.URLError as e:
            print(f"Network error controlling flash: {e}")
            # Continue with classification even if flash control fails
        except Exception as e:
            print(f"Error controlling flash: {e}")
            # Continue with classification even if flash control fails
    
    def classify_multiple_times(self, num_classifications=5, delay_between=0.2, show_preview=None):
        """Run classification multiple times and return majority vote"""
        if show_preview is None:
            show_preview = self.gui_mode
            
        print("\nAttempting to turn on flash...")
        # Try to turn on flash but continue even if it fails
        self.control_flash(True)
        
        print(f"Starting classification ({num_classifications} times)...")
        print("Initializing prediction arrays...")
        
        predictions = []
        confidences = []
        print("Arrays initialized successfully")
        
        for i in range(num_classifications):
            print(f"Classification {i+1}/{num_classifications}")
            
            print(f"\nAttempting capture for classification {i+1}/{num_classifications}")
            # Capture fresh image for each classification
            image = self.capture_image(show_preview=show_preview, save_debug=(i == 0))  # Only save first debug image
            if image is None:
                print(f"Failed to capture image for classification {i+1}")
                continue
            print(f"Successfully captured image for classification {i+1}")
            
            # Classify image
            prediction, confidence = self.classify_image(image)
            
            if prediction is not None:
                predictions.append(prediction)
                confidences.append(confidence)
                print(f"  Result: {prediction} ({confidence:.2f}%)")
            
            # Small delay between captures
            if i < num_classifications - 1:
                time.sleep(delay_between)
        
        if not predictions:
            print("No successful classifications")
            return None, 0.0
        
        # Use majority vote
        vote_counter = Counter(predictions)
        final_prediction = vote_counter.most_common(1)[0][0]
        
        # Calculate average confidence for the winning prediction
        winning_confidences = [conf for pred, conf in zip(predictions, confidences) if pred == final_prediction]
        average_confidence = sum(winning_confidences) / len(winning_confidences)
        
        vote_count = vote_counter[final_prediction]
        print(f"Final result: {final_prediction} (voted {vote_count}/{len(predictions)} times, avg confidence: {average_confidence:.2f}%)")
        
        # Show result preview if in GUI mode
        if self.gui_mode and show_preview:
            self.show_classification_result(final_prediction, average_confidence, vote_counter)
        
        return final_prediction, average_confidence
    
    def show_classification_result(self, prediction, confidence, vote_breakdown):
        """Display classification result in GUI mode"""
        try:
            import matplotlib.pyplot as plt
            
            # Create a simple result display
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))
            
            # Result text
            ax1.text(0.5, 0.7, f"Classification Result:", ha='center', va='center', fontsize=16, weight='bold')
            ax1.text(0.5, 0.5, f"{prediction} Peso", ha='center', va='center', fontsize=24, color='green')
            ax1.text(0.5, 0.3, f"Confidence: {confidence:.1f}%", ha='center', va='center', fontsize=14)
            ax1.set_xlim(0, 1)
            ax1.set_ylim(0, 1)
            ax1.axis('off')
            
            # Vote breakdown
            labels = list(vote_breakdown.keys())
            votes = list(vote_breakdown.values())
            ax2.bar(labels, votes)
            ax2.set_title("Vote Breakdown")
            ax2.set_ylabel("Number of Votes")
            ax2.set_xlabel("Prediction")
            
            plt.tight_layout()
            plt.show(block=False)
            plt.pause(3)  # Show for 3 seconds
            plt.close()
            
        except Exception as e:
            print(f"Could not display GUI result: {e}")
    
    def show_live_preview(self, duration=10):
        """Show live camera preview for debugging/setup (GUI mode only)"""
        if not self.gui_mode:
            print("Live preview not available in headless mode")
            return
            
        print(f"Showing live preview for {duration} seconds... Press 'q' to quit early")
        
        start_time = time.time()
        while time.time() - start_time < duration:
            ret, frame = self.camera.read()
            if ret:
                # Add overlay text
                cv2.putText(frame, "Live Preview - Press 'q' to quit", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, f"Time: {int(duration - (time.time() - start_time))}s", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                cv2.imshow("Chit Exchanger - Live Preview", frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                print("Failed to read frame")
                break
        
        cv2.destroyAllWindows()
        print("Live preview ended")
    
    def cleanup(self):
        """Clean up camera resources"""
        try:
            # Turn off flash
            self.control_flash(False)
            
            if hasattr(self, 'camera') and self.camera.isOpened():
                self.camera.release()
                print("Camera released")
        except Exception as e:
            print(f"Error during camera cleanup: {e}")

# Test function for standalone testing
def test_classifier():
    """Test the classifier with Teachable Machine model"""
    try:
        # Enable GUI for preview
        classifier = ImageClassifier(enable_gui=True)
        
        print("\nShowing real-time preview. Press 'q' when ready to classify...")
        while True:
            ret, frame = classifier.camera.read()
            if ret:
                cv2.imshow("Camera Preview - Press 'q' when ready", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    cv2.destroyAllWindows()
                    break
            else:
                print("Failed to get frame from camera")
                break
        
        print("Testing classifier with 3 classifications...")
        prediction, confidence = classifier.classify_multiple_times(num_classifications=3)
        
        if prediction:
            print(f"Test result: {prediction} with {confidence:.2f}% confidence")
        else:
            print("Test failed - no prediction")
            
    except Exception as e:
        print(f"Test error: {e}")
    finally:
        if 'classifier' in locals():
            classifier.cleanup()

def main():
    """Main function for testing different modes"""
    import sys
    
    if len(sys.argv) > 1:
        if sys.argv[1] == 'preview':
            # Show live preview mode
            classifier = ImageClassifier()
            classifier.show_live_preview(duration=30)
            classifier.cleanup()
        elif sys.argv[1] == 'single':
            # Single classification test
            classifier = ImageClassifier()
            image = classifier.capture_image()
            if image:
                result, conf = classifier.classify_image(image)
                print(f"Single classification: {result} ({conf:.2f}%)")
            classifier.cleanup()
        else:
            print("Usage: python classifier.py [preview|single]")
    else:
        test_classifier()

if __name__ == '__main__':
    main()