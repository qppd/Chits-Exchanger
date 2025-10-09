#include "BILL_ACCEPTOR.h"


volatile unsigned int pulseCount = 0;
unsigned int billCredit = 0;
unsigned long lastPulseTime = 0;
const unsigned long pulseDebounce = 50; // ms debounce

void IRAM_ATTR billPulseISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastPulseTime > pulseDebounce) {
    pulseCount++;
    lastPulseTime = currentTime;
  }
}

void initBILLACCEPTOR() {
  pinMode(billPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(billPin), billPulseISR, FALLING);
}
