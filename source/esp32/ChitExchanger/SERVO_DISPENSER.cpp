#include "SERVO_DISPENSER.h"

// Initialize the PCA9685 object for PWM control
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// I2C device scanner for debugging
void scanI2CDevices() {
  Serial.println("Scanning I2C bus for devices...");
  int deviceCount = 0;
  
  for (int address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    int error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      
      // Identify common devices
      if (address == 0x27 || address == 0x3F) {
        Serial.println(" (LCD Display)");
      } else if (address == 0x40) {
        Serial.println(" (PCA9685 Servo Driver)");
      } else {
        Serial.println(" (Unknown device)");
      }
      deviceCount++;
    }
  }
  
  if (deviceCount == 0) {
    Serial.println("No I2C devices found!");
  } else {
    Serial.print("Found ");
    Serial.print(deviceCount);
    Serial.println(" I2C device(s)");
  }
}

// Function to initialize the PCA9685 for servo control
void initSERVO() {
  // I2C is already initialized in main setup()
  
  // Scan for I2C devices
  scanI2CDevices();
  
  pwm.begin();
  pwm.setPWMFreq(50);  // Set PWM frequency to 50Hz for servos

  delay(10);
  
  // Deactivate all servo outputs (set PWM to 0)
  for (int i = 0; i < 4; i++) {
    pwm.setPWM(i, 0, 0);
  }
  
  Serial.println("PCA9685 Initialized - 360° Servo Mode");
  Serial.println("Servo channels 0-3 ready for card dispensing");
}

// ========================================
// NEW: 360-DEGREE SERVO TIMING FUNCTIONS
// ========================================

// Set servo speed/direction for 360-degree servo
void setServoSpeed(int channel, int speed) {
  pwm.setPWM(channel, 0, speed);
}

// Stop specific servo
void stopServo(int channel) {
  // Fully deactivate servo output
  pwm.setPWM(channel, 0, 0);
}

// Operate servo for a specific duration
void operateServoTimed(int channel, int direction, int duration) {
  Serial.print("Operating servo channel ");
  Serial.print(channel);
  Serial.print(" for ");
  Serial.print(duration);
  Serial.println("ms");
  
  // Start servo rotation
  setServoSpeed(channel, direction);
  
  // Wait for specified duration
  delay(duration);
  
  // Stop servo
  stopServo(channel);
  
  Serial.print("Servo channel ");
  Serial.print(channel);
  Serial.println(" stopped");
}

// Dispense card based on chit value
void dispenseCard(int channel, int chitValue) {
  int duration;
  
  // Get duration based on chit value
  switch (chitValue) {
    case 5:
      duration = DISPENSE_DURATION_5;
      break;
    case 10:
      duration = DISPENSE_DURATION_10;
      break;
    case 20:
      duration = DISPENSE_DURATION_20;
      break;
    case 50:
      duration = DISPENSE_DURATION_50;
      break;
    default:
      Serial.println("Invalid chit value!");
      return;
  }
  
  Serial.print("Dispensing ₱");
  Serial.print(chitValue);
  Serial.print(" chit on channel ");
  Serial.print(channel);
  Serial.print(" for ");
  Serial.print(duration);
  Serial.println("ms");
  
  // Dispense card with forward rotation
  operateServoTimed(channel, SERVO_FORWARD, duration);
  
  // Small delay between operations
  delay(100);
}

// ========================================
// LEGACY: ANGLE-BASED FUNCTIONS (COMPATIBILITY)
// ========================================

// Function to set servo to a specified angle (for standard servos)
void setServoAngle(int channel, int angle) {
  int pulseLen = map(angle, 0, 180, 150, 600);  // Using original SERVO_MIN/MAX values
  pwm.setPWM(channel, 0, pulseLen);
}

void repeatOperateSERVO(int channel, int startAngle, int endAngle, int speed, int repeatCount) {
  for (int i = 0; i < repeatCount; i++) {
    operateSERVO(channel, startAngle, endAngle, speed);
  }
}

// Function to operate the servo, sweeping between two angles with speed control
void operateSERVO(int channel, int startAngle, int endAngle, int speed) {
  int step = (startAngle < endAngle) ? 1 : -1;  // Determine direction of sweep

  // Sweep back from endAngle to startAngle
  for (int angle = endAngle; angle != startAngle; angle -= step) {
    setServoAngle(channel, angle);
    delay(speed);  // Control sweep speed with delay between steps
  }

  setServoAngle(channel, startAngle);  // Ensure it returns to startAngle

  // Sweep from startAngle to endAngle
  for (int angle = startAngle; angle != endAngle; angle += step) {
    setServoAngle(channel, angle);
    delay(speed);  // Control sweep speed with delay between steps
  }

  setServoAngle(channel, endAngle);  // Ensure it reaches the exact endAngle
}
