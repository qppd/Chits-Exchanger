#include "BILL_ACCEPTOR.h"

volatile unsigned int pulseCount = 0;
unsigned int billCredit = 0;
unsigned long lastPulseTime = 0;
const unsigned long pulseDebounce = 50; // ms debounce
volatile int detectedBillValue = 0; // Current detected bill value

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
  Serial.println("Bill Acceptor initialized");
  Serial.println("Accepts: P20 and P50 bills");
}

// Determine bill value based on pulse count
// This function should be called after pulse counting is complete
int getBillValue() {
  // Bill acceptor typically sends different pulse counts for different denominations
  // Adjust these values based on your specific bill acceptor model
  // Common configurations:
  // - 1 pulse for P20 bill
  // - 2 pulses for P50 bill (or different pattern based on your hardware)
  
  int billValue = 0;
  
  if (pulseCount == 1) {
    billValue = BILL_VALUE_20; // P20 bill
  } else if (pulseCount == 2) {
    billValue = BILL_VALUE_50; // P50 bill (adjust based on your hardware)
  } else if (pulseCount > 0) {
    // If pulse count doesn't match expected patterns, calculate based on P10 per pulse
    // This is a fallback method - adjust based on your bill acceptor specs
    billValue = pulseCount * 10; // Fallback: P10 per pulse
    
    // Validate that the calculated value matches supported denominations
    if (billValue != BILL_VALUE_20 && billValue != BILL_VALUE_50) {
      Serial.print("Warning: Unexpected bill value calculated: P");
      Serial.println(billValue);
      // You might want to reject the bill or handle this case differently
    }
  }
  
  return billValue;
}
