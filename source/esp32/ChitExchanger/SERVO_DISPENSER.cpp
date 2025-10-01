#include "SERVO_DISPENSER.h"

// Initialize the PCA9685 object for PWM control
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int SERVO_MIN = 150;  // Minimum pulse length for 0 degrees
const int SERVO_MAX = 600;  // Maximum pulse length for 180 degrees

// Function to initialize the PCA9685 for servo control
void initSERVO() {
  Wire.begin(21, 22);  // Initialize I2C with SDA on GPIO 21 and SCL on GPIO 22
  pwm.begin();
  pwm.setPWMFreq(50);  // Set PWM frequency to 50Hz for servos

  delay(10);
  Serial.println("PCA9685 Initialized");
}

// Function to set servo to a specified angle
void setServoAngle(int channel, int angle) {
  int pulseLen = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(channel, 0, pulseLen);  // Set PWM pulse width to move servo to angle
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
