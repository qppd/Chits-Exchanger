#ifndef PIEZO_BUZZER_H
#define PIEZO_BUZZER_H

#include <Arduino.h>

// Define the pin for the piezo buzzer
#define BUZZER_PIN 12

// Function declarations
void initBuzzer();
void playTone(int frequency, int duration);
void stopTone();

#endif
