# hopper.py - ALLAN coin hopper control module
import RPi.GPIO as GPIO
import time

class AllanCoinHopper:
    def __init__(self, pin_5_peso=22, pin_10_peso=23, pin_20_peso=24):
        """Initialize ALLAN coin hopper relays"""
        self.hoppers = {
            5: pin_5_peso,
            10: pin_10_peso,
            20: pin_20_peso
        }
        
        GPIO.setmode(GPIO.BCM)
        
        # Setup all hopper pins as outputs
        for peso_value, pin in self.hoppers.items():
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)  # Initially off
            
    def dispense_coins(self, peso_value, duration=1.0):
        """Dispense coins of specified peso value"""
        if peso_value not in self.hoppers:
            print(f"Error: No hopper configured for {peso_value} peso")
            return False
            
        pin = self.hoppers[peso_value]
        print(f"Dispensing {peso_value} peso coins...")
        
        # Activate relay (turn on hopper)
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(duration)  # Run for specified duration
        GPIO.output(pin, GPIO.LOW)  # Turn off hopper
        
        print(f"Dispensed {peso_value} peso coins")
        return True
        
    def test_all_hoppers(self, duration=0.5):
        """Test all hoppers for functionality"""
        print("Testing all hoppers...")
        for peso_value in [5, 10, 20]:
            print(f"Testing {peso_value} peso hopper...")
            self.dispense_coins(peso_value, duration)
            time.sleep(1)  # Delay between tests
            
    def cleanup(self):
        """Clean up GPIO resources"""
        # Ensure all hoppers are off
        for pin in self.hoppers.values():
            GPIO.output(pin, GPIO.LOW)
        GPIO.cleanup()