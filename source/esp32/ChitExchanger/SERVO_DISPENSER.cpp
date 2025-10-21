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
  for (int i = 0; i < 8; i++) {  // Updated to support channels 0-7 (8 servos)
    pwm.setPWM(i, 0, 0);
  }
  
  Serial.println("PCA9685 Initialized - 360° Servo Mode");
  Serial.println("Servo channels 0-7 ready for card dispensing");
  Serial.println("Configuration:");
  Serial.println("  ₱50 chits: Channels 0 & 1");
  Serial.println("  ₱20 chits: Channels 2 & 3");
  Serial.println("  ₱10 chits: Channels 4 & 5");
  Serial.println("  ₱5 chits:  Channels 6 & 7");
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

// Dispense card using both servos in a pair (runs simultaneously)
void dispenseCardPair(int channel1, int channel2, int chitValue) {
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
  Serial.print(" chit using channels ");
  Serial.print(channel1);
  Serial.print(" & ");
  Serial.print(channel2);
  Serial.print(" for ");
  Serial.print(duration);
  Serial.println("ms");
  
  // Start both servos simultaneously
  setServoSpeed(channel1, SERVO_FORWARD);
  setServoSpeed(channel2, SERVO_FORWARD);
  
  // Wait for specified duration
  delay(duration);
  
  // Stop both servos
  stopServo(channel1);
  stopServo(channel2);
  
  Serial.print("Both servos on channels ");
  Serial.print(channel1);
  Serial.print(" & ");
  Serial.print(channel2);
  Serial.println(" stopped");
  
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

// ========================================
// TEST FUNCTION FOR ADDITIONAL SERVOS
// ========================================

// Test function for servo channels 4 and 5 (₱10 pair)
void testAdditionalServos() {
  Serial.println("=== TESTING ₱10 SERVO PAIR ===");
  Serial.println("Starting test sequence for channels 4 and 5...");
  
  // Start both servos simultaneously - CW (Clockwise) for 0.8 seconds
  Serial.println("Testing Servo Channels 4 and 5 (both CW for 0.8 seconds)...");
  setServoSpeed(CHIT_10_CHANNEL_1, SERVO_FORWARD);  // Start Channel 4 CW
  setServoSpeed(CHIT_10_CHANNEL_2, SERVO_FORWARD);  // Start Channel 5 CW
  
  delay(800);  // Both run together for 0.8 seconds
  
  // Stop both servos
  stopServo(CHIT_10_CHANNEL_1);
  stopServo(CHIT_10_CHANNEL_2);
  Serial.println("Both servos stopped");
  
  Serial.println("=== TEST SEQUENCE COMPLETE ===");
}

// Test all 4 pairs of servos
void testAllServoPairs() {
  Serial.println("=== TESTING ALL SERVO PAIRS ===");
  Serial.println("Testing all 8 servos (4 pairs)...");
  
  // Test ₱50 pair (Channels 0 & 1)
  Serial.println("\n--- Testing ₱50 Pair (Channels 0 & 1) ---");
  dispenseCardPair(CHIT_50_CHANNEL_1, CHIT_50_CHANNEL_2, CHIT_VALUE_50);
  delay(1000);
  
  // Test ₱20 pair (Channels 2 & 3)
  Serial.println("\n--- Testing ₱20 Pair (Channels 2 & 3) ---");
  dispenseCardPair(CHIT_20_CHANNEL_1, CHIT_20_CHANNEL_2, CHIT_VALUE_20);
  delay(1000);
  
  // Test ₱10 pair (Channels 4 & 5)
  Serial.println("\n--- Testing ₱10 Pair (Channels 4 & 5) ---");
  dispenseCardPair(CHIT_10_CHANNEL_1, CHIT_10_CHANNEL_2, CHIT_VALUE_10);
  delay(1000);
  
  // Test ₱5 pair (Channels 6 & 7)
  Serial.println("\n--- Testing ₱5 Pair (Channels 6 & 7) ---");
  dispenseCardPair(CHIT_5_CHANNEL_1, CHIT_5_CHANNEL_2, CHIT_VALUE_5);
  delay(1000);
  
  Serial.println("\n=== ALL SERVO PAIRS TEST COMPLETE ===");
}
