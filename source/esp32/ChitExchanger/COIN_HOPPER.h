#ifndef COIN_HOPPER_H
#define COIN_HOPPER_H

#include <Arduino.h>

#define HOPPER_MOTOR_PIN 10
#define HOPPER_SENSOR_PIN 11

void initCoinHopper();
void dispenseCoins(int count);

#endif // COIN_HOPPER_H
