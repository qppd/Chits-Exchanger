#ifndef COIN_SLOT_H
#define COIN_SLOT_H

#include <Arduino.h>
#include "PIN_CONFIGURATION.h"

extern volatile bool coinInserted;
extern volatile unsigned long coinLastDebounceTime;
extern const unsigned long coinDebounceDelay;

void IRAM_ATTR ITRCOIN();
void initALLANCOIN();

#endif // COIN_SLOT_H
