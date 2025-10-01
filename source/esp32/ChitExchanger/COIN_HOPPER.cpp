#include "COIN_HOPPER.h"

void initCoinHopper() {
  pinMode(HOPPER_MOTOR_PIN, OUTPUT);
  pinMode(HOPPER_SENSOR_PIN, INPUT_PULLUP);
  digitalWrite(HOPPER_MOTOR_PIN, LOW);
}

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
