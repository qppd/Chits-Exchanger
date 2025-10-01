#include "COIN_SLOT.h"

volatile bool coinInserted = false;
volatile unsigned long coinLastDebounceTime = 0;
const unsigned long coinDebounceDelay = 50; // debounce delay in ms

void IRAM_ATTR ITRCOIN() {
  if (digitalRead(coinPin) == LOW) {
    unsigned long coinCurrentTime = millis();
    if ((coinCurrentTime - coinLastDebounceTime) > coinDebounceDelay) {
      if (digitalRead(coinPin) == LOW) {
        coinInserted = true;
        coinLastDebounceTime = coinCurrentTime;
      }
    }
  }
}

void initALLANCOIN() {
  pinMode(coinPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(coinPin), ITRCOIN, FALLING);
}
