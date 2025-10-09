#include "COIN_SLOT.h"

volatile bool coinInserted = false;
volatile unsigned long coinLastDebounceTime = 0;
const unsigned long coinDebounceDelay = 50; // debounce delay in ms
volatile unsigned int coinPulseCount = 0;
unsigned int coinCredit = 0;
unsigned long lastCoinPulseTime = 0;

void IRAM_ATTR ITRCOIN() {
  unsigned long coinCurrentTime = millis();
  if ((coinCurrentTime - coinLastDebounceTime) > coinDebounceDelay) {
    coinPulseCount++;
    coinInserted = true;
    coinLastDebounceTime = coinCurrentTime;
    lastCoinPulseTime = coinCurrentTime;
  }
}

void initALLANCOIN() {
  pinMode(coinPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(coinPin), ITRCOIN, FALLING);
  Serial.println("Coin Slot initialized");
  Serial.println("Accepts: P1, P5, P10, P20 coins");
}

// Determine coin value based on pulse count
// This function should be called after pulse counting is complete
int getCoinValue() {
  // Coin acceptor typically sends different pulse counts for different denominations
  // Adjust these values based on your specific coin acceptor model
  // Common configurations:
  // - 1 pulse for P1 coin
  // - 5 pulses for P5 coin
  // - 10 pulses for P10 coin
  // - 20 pulses for P20 coin
  // OR different patterns based on your hardware
  
  int coinValue = 0;
  
  if (coinPulseCount == 1) {
    coinValue = COIN_VALUE_1; // P1 coin
  } else if (coinPulseCount == 5) {
    coinValue = COIN_VALUE_5; // P5 coin
  } else if (coinPulseCount == 10) {
    coinValue = COIN_VALUE_10; // P10 coin
  } else if (coinPulseCount == 20) {
    coinValue = COIN_VALUE_20; // P20 coin
  } else if (coinPulseCount > 0) {
    // Alternative method: Some coin acceptors send pulse count equal to value
    if (coinPulseCount == 1 || coinPulseCount == 5 || 
        coinPulseCount == 10 || coinPulseCount == 20) {
      coinValue = coinPulseCount; // Direct value mapping
    } else {
      Serial.print("Warning: Unexpected coin pulse count: ");
      Serial.println(coinPulseCount);
      // You might want to reject the coin or handle this case differently
    }
  }
  
  return coinValue;
}

// Reset coin detection variables
void resetCoinDetection() {
  coinPulseCount = 0;
  coinInserted = false;
}
