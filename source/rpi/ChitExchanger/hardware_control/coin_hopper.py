#!/usr/bin/env python3
"""
Standard Coin Hopper Control System
Migrated from ESP32 COIN_HOPPER.h/.cpp

This module provides control for a standard motorized coin hopper
that was originally implemented on the ESP32 platform. The functionality
has been migrated to the Raspberry Pi for centralized coin dispensing control.

Hardware Requirements:
- Motor Control: GPIO 22 (via relay/driver)
- Sensor Input: GPIO 23 (coin detection sensor)
- Power: 12V for motor, 5V for sensor

Original ESP32 Implementation Logic:
- Motor-driven coin dispensing mechanism
- Optical sensor for accurate coin counting
- Configurable dispensing quantities
- Jam detection and prevention
"""

import RPi.GPIO as GPIO
import time
import logging
from typing import Dict, Optional
from dataclasses import dataclass
from enum import Enum

# GPIO Pin Configuration (migrated from ESP32)
HOPPER_MOTOR_PIN = 22  # Motor control (originally GPIO 10 on ESP32)
HOPPER_SENSOR_PIN = 23  # Coin detection sensor (originally GPIO 11 on ESP32)

class HopperStatus(Enum):
    """Hopper status enumeration"""
    READY = "ready"
    DISPENSING = "dispensing"
    JAMMED = "jammed"
    EMPTY = "empty"
    ERROR = "error"

@dataclass
class HopperConfig:
    """Configuration for standard coin hopper"""
    motor_pin: int = HOPPER_MOTOR_PIN
    sensor_pin: int = HOPPER_SENSOR_PIN
    motor_pulse_duration: float = 0.1  # Duration to run motor (originally 100ms delay in ESP32)
    sensor_debounce_delay: float = 0.2  # Sensor debounce delay (originally 200ms in ESP32)
    motor_rest_delay: float = 0.1      # Rest between motor pulses (originally 100ms in ESP32)
    max_dispense_attempts: int = 10     # Maximum attempts per coin
    jam_detection_timeout: float = 5.0  # Timeout to detect jam
    
class StandardCoinHopper:
    """
    Standard Coin Hopper Controller
    
    This class replicates the exact functionality from the ESP32 COIN_HOPPER.cpp
    implementation, adapted for Raspberry Pi GPIO control.
    """
    
    def __init__(self, config: Optional[HopperConfig] = None):
        """
        Initialize the standard coin hopper
        
        Args:
            config: Optional configuration object
        """
        self.config = config or HopperConfig()
        self.status = HopperStatus.READY
        self.total_dispensed = 0
        self.error_count = 0
        self.last_dispense_time = None
        
        # Setup logging
        self.logger = logging.getLogger(__name__)
        
        # Initialize GPIO
        self._init_gpio()
        
        self.logger.info("Standard coin hopper initialized (migrated from ESP32)")
    
    def _init_gpio(self):
        """
        Initialize GPIO pins
        Replicates the initCoinHopper() function from ESP32
        """
        try:
            # Set GPIO mode
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # Setup motor pin as output (originally pinMode(HOPPER_MOTOR_PIN, OUTPUT))
            GPIO.setup(self.config.motor_pin, GPIO.OUT)
            GPIO.output(self.config.motor_pin, GPIO.LOW)  # Initially OFF
            
            # Setup sensor pin as input with pullup (originally pinMode(HOPPER_SENSOR_PIN, INPUT_PULLUP))
            GPIO.setup(self.config.sensor_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
            self.logger.info(f"GPIO initialized - Motor: {self.config.motor_pin}, Sensor: {self.config.sensor_pin}")
            
        except Exception as e:
            self.logger.error(f"Failed to initialize GPIO: {e}")
            self.status = HopperStatus.ERROR
            raise
    
    def dispense_coins(self, count: int) -> bool:
        """
        Dispense specified number of coins
        
        This method replicates the dispenseCoins(int count) function from ESP32
        with the exact same logic and timing.
        
        Args:
            count: Number of coins to dispense
            
        Returns:
            bool: True if successful, False if failed
            
        Original ESP32 Logic:
        ```cpp
        void dispenseCoins(int count) {
            int dispensed = 0;
            while (dispensed < count) {
                digitalWrite(HOPPER_MOTOR_PIN, HIGH); // Start motor
                delay(100); // Allow time for coin to move

                if (digitalRead(HOPPER_SENSOR_PIN) == LOW) { // Coin detected
                    dispensed++;
                    delay(200); // Debounce sensor
                }

                digitalWrite(HOPPER_MOTOR_PIN, LOW); // Stop motor briefly
                delay(100);
            }
        }
        ```
        """
        if count <= 0:
            self.logger.warning(f"Invalid coin count: {count}")
            return False
            
        if self.status != HopperStatus.READY:
            self.logger.warning(f"Hopper not ready. Status: {self.status}")
            return False
        
        self.logger.info(f"Starting to dispense {count} coins")
        self.status = HopperStatus.DISPENSING
        
        dispensed = 0
        start_time = time.time()
        
        try:
            while dispensed < count:
                # Check for timeout (jam detection)
                if time.time() - start_time > self.config.jam_detection_timeout:
                    self.logger.error("Dispense timeout - possible jam detected")
                    self.status = HopperStatus.JAMMED
                    return False
                
                # Start motor (digitalWrite(HOPPER_MOTOR_PIN, HIGH))
                GPIO.output(self.config.motor_pin, GPIO.HIGH)
                
                # Allow time for coin to move (delay(100) in ESP32)
                time.sleep(self.config.motor_pulse_duration)
                
                # Check if coin detected (digitalRead(HOPPER_SENSOR_PIN) == LOW)
                if GPIO.input(self.config.sensor_pin) == GPIO.LOW:
                    dispensed += 1
                    self.total_dispensed += 1
                    self.logger.debug(f"Coin {dispensed} dispensed")
                    
                    # Debounce sensor (delay(200) in ESP32)
                    time.sleep(self.config.sensor_debounce_delay)
                
                # Stop motor briefly (digitalWrite(HOPPER_MOTOR_PIN, LOW))
                GPIO.output(self.config.motor_pin, GPIO.LOW)
                
                # Rest delay (delay(100) in ESP32)
                time.sleep(self.config.motor_rest_delay)
            
            self.status = HopperStatus.READY
            self.last_dispense_time = time.time()
            self.logger.info(f"Successfully dispensed {count} coins")
            return True
            
        except Exception as e:
            self.logger.error(f"Error during coin dispensing: {e}")
            self.status = HopperStatus.ERROR
            self.error_count += 1
            # Ensure motor is stopped
            GPIO.output(self.config.motor_pin, GPIO.LOW)
            return False
    
    def get_status(self) -> Dict:
        """
        Get comprehensive hopper status
        
        Returns:
            Dict: Status information
        """
        return {
            'status': self.status.value,
            'total_dispensed': self.total_dispensed,
            'error_count': self.error_count,
            'last_dispense_time': self.last_dispense_time,
            'sensor_state': GPIO.input(self.config.sensor_pin) if self.status != HopperStatus.ERROR else None,
            'motor_pin': self.config.motor_pin,
            'sensor_pin': self.config.sensor_pin
        }
    
    def test_motor(self) -> bool:
        """
        Test motor functionality
        
        Returns:
            bool: True if motor test passed
        """
        if self.status != HopperStatus.READY:
            return False
            
        try:
            self.logger.info("Testing motor...")
            
            # Run motor for short duration
            GPIO.output(self.config.motor_pin, GPIO.HIGH)
            time.sleep(0.5)
            GPIO.output(self.config.motor_pin, GPIO.LOW)
            
            self.logger.info("Motor test completed")
            return True
            
        except Exception as e:
            self.logger.error(f"Motor test failed: {e}")
            return False
    
    def test_sensor(self) -> Dict:
        """
        Test sensor functionality
        
        Returns:
            Dict: Sensor test results
        """
        try:
            sensor_readings = []
            
            # Take multiple readings
            for i in range(10):
                reading = GPIO.input(self.config.sensor_pin)
                sensor_readings.append(reading)
                time.sleep(0.1)
            
            # Analyze readings
            high_count = sum(sensor_readings)
            low_count = len(sensor_readings) - high_count
            
            return {
                'readings': sensor_readings,
                'high_count': high_count,
                'low_count': low_count,
                'current_state': GPIO.input(self.config.sensor_pin),
                'stable': high_count == 10 or low_count == 10
            }
            
        except Exception as e:
            self.logger.error(f"Sensor test failed: {e}")
            return {'error': str(e)}
    
    def reset(self) -> bool:
        """
        Reset hopper to ready state
        
        Returns:
            bool: True if reset successful
        """
        try:
            # Ensure motor is stopped
            GPIO.output(self.config.motor_pin, GPIO.LOW)
            
            # Reset status
            self.status = HopperStatus.READY
            self.error_count = 0
            
            self.logger.info("Hopper reset successful")
            return True
            
        except Exception as e:
            self.logger.error(f"Hopper reset failed: {e}")
            return False
    
    def emergency_stop(self):
        """
        Emergency stop - immediately halt all operations
        """
        try:
            GPIO.output(self.config.motor_pin, GPIO.LOW)
            self.status = HopperStatus.ERROR
            self.logger.warning("Emergency stop activated")
        except Exception as e:
            self.logger.error(f"Emergency stop failed: {e}")
    
    def cleanup(self):
        """
        Cleanup GPIO resources
        """
        try:
            GPIO.output(self.config.motor_pin, GPIO.LOW)
            GPIO.cleanup([self.config.motor_pin, self.config.sensor_pin])
            self.logger.info("GPIO cleanup completed")
        except Exception as e:
            self.logger.error(f"GPIO cleanup failed: {e}")
    
    def __del__(self):
        """Destructor to ensure cleanup"""
        self.cleanup()

# Utility functions for compatibility with ESP32 code style
def init_coin_hopper() -> StandardCoinHopper:
    """
    Initialize coin hopper (ESP32 style function)
    
    Returns:
        StandardCoinHopper: Initialized hopper instance
    """
    return StandardCoinHopper()

def dispense_coins(hopper: StandardCoinHopper, count: int) -> bool:
    """
    Dispense coins (ESP32 style function)
    
    Args:
        hopper: Hopper instance
        count: Number of coins to dispense
        
    Returns:
        bool: Success status
    """
    return hopper.dispense_coins(count)

# Example usage and testing
if __name__ == "__main__":
    # Setup logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    try:
        # Initialize hopper
        hopper = StandardCoinHopper()
        
        # Test motor
        print("Testing motor...")
        hopper.test_motor()
        
        # Test sensor
        print("Testing sensor...")
        sensor_test = hopper.test_sensor()
        print(f"Sensor test result: {sensor_test}")
        
        # Get status
        status = hopper.get_status()
        print(f"Hopper status: {status}")
        
        # Dispense test (uncomment for actual testing)
        # print("Dispensing 3 test coins...")
        # success = hopper.dispense_coins(3)
        # print(f"Dispense result: {success}")
        
    except KeyboardInterrupt:
        print("Test interrupted by user")
    except Exception as e:
        print(f"Test failed: {e}")
    finally:
        # Cleanup
        if 'hopper' in locals():
            hopper.cleanup()
        print("Test completed")