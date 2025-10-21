#ifndef SERVO_DISPENSER_H
#define SERVO_DISPENSER_H

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "PIN_CONFIGURATION.h"

// Initialize the PCA9685 object for PWM control
extern Adafruit_PWMServoDriver pwm;

// 360-degree servo control constants
const int SERVO_STOP = 375;        // PWM value for stopping 360-degree servo (1.5ms pulse)
const int SERVO_FORWARD = 450;     // PWM value for forward rotation
const int SERVO_BACKWARD = 300;    // PWM value for backward rotation

// Timing variables for card dispensing (in milliseconds)
const int DISPENSE_DURATION_5 = 500;   // Duration to dispense ₱5 chit (ms)
const int DISPENSE_DURATION_10 = 600;  // Duration to dispense ₱10 chit (ms)
const int DISPENSE_DURATION_20 = 700;  // Duration to dispense ₱20 chit (ms)
const int DISPENSE_DURATION_50 = 800;  // Duration to dispense ₱50 chit (ms)

// Function declarations
void scanI2CDevices(); // I2C device scanner for debugging
void initSERVO();
void setServoSpeed(int channel, int speed); // speed: SERVO_STOP, SERVO_FORWARD, SERVO_BACKWARD
void operateServoTimed(int channel, int direction, int duration); // New timing-based function
void dispenseCard(int channel, int chitValue); // Dispense specific chit value (single servo)
void dispenseCardPair(int channel1, int channel2, int chitValue); // Dispense using servo pair
void stopServo(int channel); // Stop specific servo

// Test functions
void testAdditionalServos(); // Test function for servo channels 4 and 5
void testAllServoPairs(); // Test all 4 pairs of servos

// Legacy angle-based functions (kept for compatibility)
void setServoAngle(int channel, int angle);
void operateSERVO(int channel, int startAngle, int endAngle, int speed);
void repeatOperateSERVO(int channel, int startAngle, int endAngle, int speed, int repeatCount);

#endif // SERVO_DISPENSER_H
