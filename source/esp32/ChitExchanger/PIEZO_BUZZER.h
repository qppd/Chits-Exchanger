#ifndef PIEZO_BUZZER_H
#define PIEZO_BUZZER_H


#include <Arduino.h>
#include "PIN_CONFIGURATION.h"

// Function declarations
void initBuzzer();
void playTone(int frequency, int duration);
void stopTone();

#endif
