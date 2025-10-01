#ifndef COIN_SLOT_H
#define COIN_SLOT_H

#include <Arduino.h>
#include "PIN_CONFIGURATION.h"

#undef coinPin
#undef billPin
#undef SERVO_SDA
#undef SERVO_SCL
#undef LCD_BUTTON_PIN
#undef COIN_BUTTON_PIN

extern volatile bool coinInserted;
extern volatile unsigned long coinLastDebounceTime;
extern const unsigned long coinDebounceDelay;

void IRAM_ATTR ITRCOIN();
void initALLANCOIN();

#endif // COIN_SLOT_H
