# main.py for rpi/tflite
# Entry point for Raspberry Pi 4B Chit Exchanger System
# Workflow: IR sensor -> Servo movement -> Image classification -> Coin dispensing

import time
import sys
import signal
from sensor import IRSensor
from servo import ServoMotor
from classifier import ImageClassifier
from hopper import AllanCoinHopper

class ChitExchangerSystem:
    def __init__(self):
        """Initialize all system components"""
        print("Initializing Chit Exchanger System...")
        
        try:
            self.ir_sensor = IRSensor(pin=17)
            self.servo = ServoMotor(pin=18)
            self.classifier = ImageClassifier()
            self.hopper = AllanCoinHopper(pin_5_peso=22, pin_10_peso=23, pin_20_peso=24)
            
            print("All components initialized successfully!")
            
        except Exception as e:
            print(f"Error initializing system: {e}")
            self.cleanup()
            sys.exit(1)
            
    def process_detected_object(self):
        """Process workflow when object is detected"""
        try:
            # Step 1: Move servo to capture position
            self.servo.move_to_capture_position()
            time.sleep(1)  # Allow time for positioning
            
            # Step 2: Run image classification 5 times
            prediction, confidence = self.classifier.classify_multiple_times(num_classifications=5)
            
            if prediction is None:
                print("Classification failed, returning servo to idle position")
                self.servo.move_to_idle_position()
                return
                
            # Step 3: Process classification result
            try:
                peso_value = int(prediction)
                print(f"Detected: {peso_value} peso bill")
                
                # Step 4: Dispense appropriate coins if valid denomination
                if peso_value in [5, 10, 20]:
                    self.hopper.dispense_coins(peso_value, duration=2.0)
                    print(f"Successfully dispensed {peso_value} peso coins")
                elif peso_value == 50:
                    print("50 peso detected - no coin dispensing configured")
                else:
                    print(f"Unknown denomination: {peso_value}")
                    
            except ValueError:
                print(f"Invalid prediction format: {prediction}")
                
            # Step 5: Return servo to idle position
            self.servo.move_to_idle_position()
            
        except Exception as e:
            print(f"Error processing object: {e}")
            self.servo.move_to_idle_position()  # Ensure servo returns to safe position
            
    def run(self):
        """Main system loop"""
        print("Chit Exchanger System started!")
        print("Waiting for objects...")
        
        try:
            while True:
                # Wait for IR sensor to detect object
                if self.ir_sensor.wait_for_object(timeout=1):  # 1 second timeout for responsiveness
                    print("Processing detected object...")
                    self.process_detected_object()
                    
                    # Small delay before next detection
                    time.sleep(2)
                    print("Ready for next object...")
                    
        except KeyboardInterrupt:
            print("\nShutdown requested...")
        except Exception as e:
            print(f"System error: {e}")
        finally:
            self.cleanup()
            
    def cleanup(self):
        """Clean up all system resources"""
        print("Cleaning up system resources...")
        
        try:
            if hasattr(self, 'ir_sensor'):
                self.ir_sensor.cleanup()
            if hasattr(self, 'servo'):
                self.servo.cleanup()
            if hasattr(self, 'hopper'):
                self.hopper.cleanup()
        except Exception as e:
            print(f"Error during cleanup: {e}")
            
        print("Cleanup complete")

def signal_handler(sig, frame):
    """Handle system signals for graceful shutdown"""
    print("\nSignal received, shutting down...")
    sys.exit(0)

if __name__ == '__main__':
    # Setup signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Create and run the system
    system = ChitExchangerSystem()
    system.run()
