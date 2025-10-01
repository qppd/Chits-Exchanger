#ifndef SERVO_DISPENSER_H
#define SERVO_DISPENSER_H

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "PIN_CONFIGURATION.h"

// Initialize the PCA9685 object for PWM control
extern Adafruit_PWMServoDriver pwm;

const int SERVO_MIN = 150;  // Minimum pulse length for 0 degrees
const int SERVO_MAX = 600;  // Maximum pulse length for 180 degrees

// Function declarations
void initSERVO();
void setServoAngle(int channel, int angle);
void operateSERVO(int channel, int startAngle, int endAngle, int speed);
void repeatOperateSERVO(int channel, int startAngle, int endAngle, int speed, int repeatCount);

#undef SERVO_SDA
#undef SERVO_SCL

// Use pin definitions from PIN_CONFIGURATION.h

#endif // SERVO_DISPENSER_H
