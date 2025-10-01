#include "BILL_ACCEPTOR.h"

volatile bool billAccepted = false;
volatile unsigned long billLastDebounceTime = 0;
const unsigned long billDebounceDelay = 100; // debounce delay in ms

void IRAM_ATTR ITRBILL() {
  if (digitalRead(billPin) == LOW) {
    unsigned long billCurrentTime = millis();
    if ((billCurrentTime - billLastDebounceTime) > billDebounceDelay) {
      if (digitalRead(billPin) == LOW) {
        billAccepted = true;
        billLastDebounceTime = billCurrentTime;
      }
    }
  }
}

void initBILLACCEPTOR() {
  pinMode(billPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(billPin), ITRBILL, FALLING);
}
