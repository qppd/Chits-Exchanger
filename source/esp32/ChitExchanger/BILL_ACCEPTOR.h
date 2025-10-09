#ifndef BILL_ACCEPTOR_H
#define BILL_ACCEPTOR_H

#include <Arduino.h>
#include "PIN_CONFIGURATION.h"


extern volatile unsigned int pulseCount;
extern unsigned int billCredit;
extern unsigned long lastPulseTime;
extern const unsigned long pulseDebounce;

void IRAM_ATTR billPulseISR();
void initBILLACCEPTOR();

#endif // BILL_ACCEPTOR_H
