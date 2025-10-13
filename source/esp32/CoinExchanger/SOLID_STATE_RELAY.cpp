/*
 * SOLID_STATE_RELAY.cpp
 * ESP32 Solid State Relay Control Implementation
 * 
 * Implementation of SSR control functionality for coin hopper power management.
 * Provides safe and reliable control of solid state relays.
 * 
 * Author: ESP32 CoinExchanger System
 * Date: October 2025
 */

#include "SOLID_STATE_RELAY.h"

// Default constructor
SOLID_STATE_RELAY::SOLID_STATE_RELAY() {
  ssrPin = -1;
  ssrState = false;
  initialized = false;
  relayName = "SSR";
}

// Constructor with pin
SOLID_STATE_RELAY::SOLID_STATE_RELAY(int pin) {
  ssrPin = pin;
  ssrState = false;
  initialized = false;
  relayName = "SSR_GPIO" + String(pin);
  begin(pin);
}

// Constructor with pin and name
SOLID_STATE_RELAY::SOLID_STATE_RELAY(int pin, String name) {
  ssrPin = pin;
  ssrState = false;
  initialized = false;
  relayName = name;
  begin(pin);
}

// Destructor - ensure SSR is turned off
SOLID_STATE_RELAY::~SOLID_STATE_RELAY() {
  if (initialized) {
    emergencyOff();
  }
}

// Initialize SSR with pin
bool SOLID_STATE_RELAY::begin(int pin) {
  return begin(pin, "SSR_GPIO" + String(pin));
}

// Initialize SSR with pin and name
bool SOLID_STATE_RELAY::begin(int pin, String name) {
  if (pin < 0 || pin > 39) {
    Serial.print("❌ Invalid SSR pin: ");
    Serial.println(pin);
    return false;
  }
  
  ssrPin = pin;
  relayName = name;
  
  // Configure GPIO pin as output
  pinMode(ssrPin, OUTPUT);
  
  // Initialize to OFF state for safety
  digitalWrite(ssrPin, LOW);
  ssrState = false;
  initialized = true;
  
  Serial.print("✅ SSR initialized: ");
  Serial.print(relayName);
  Serial.print(" on GPIO");
  Serial.println(ssrPin);
  
  return true;
}

// Turn SSR ON (power device)
void SOLID_STATE_RELAY::turnOn() {
  if (!initialized) {
    Serial.println("❌ SSR not initialized");
    return;
  }
  
  digitalWrite(ssrPin, HIGH);
  ssrState = true;
  
  Serial.print("🟢 ");
  Serial.print(relayName);
  Serial.println(" turned ON");
}

// Turn SSR OFF (cut power to device)
void SOLID_STATE_RELAY::turnOff() {
  if (!initialized) {
    Serial.println("❌ SSR not initialized");
    return;
  }
  
  digitalWrite(ssrPin, LOW);
  ssrState = false;
  
  Serial.print("🔴 ");
  Serial.print(relayName);
  Serial.println(" turned OFF");
}

// Set specific state
void SOLID_STATE_RELAY::setState(bool state) {
  if (state) {
    turnOn();
  } else {
    turnOff();
  }
}

// Toggle current state
void SOLID_STATE_RELAY::toggle() {
  setState(!ssrState);
}

// Get current state
bool SOLID_STATE_RELAY::getState() const {
  return ssrState;
}

// Get GPIO pin number
int SOLID_STATE_RELAY::getPin() const {
  return ssrPin;
}

// Get relay name
String SOLID_STATE_RELAY::getName() const {
  return relayName;
}

// Check if initialized
bool SOLID_STATE_RELAY::isInitialized() const {
  return initialized;
}

// Set relay name
void SOLID_STATE_RELAY::setName(String name) {
  relayName = name;
}

// Get human-readable status
String SOLID_STATE_RELAY::getStatusString() const {
  if (!initialized) {
    return relayName + " (NOT INITIALIZED)";
  }
  
  String status = relayName + " (GPIO" + String(ssrPin) + "): ";
  status += ssrState ? "ON" : "OFF";
  return status;
}

// Emergency shutdown
void SOLID_STATE_RELAY::emergencyOff() {
  if (initialized && ssrPin >= 0) {
    digitalWrite(ssrPin, LOW);
    ssrState = false;
    
    Serial.print("🚨 EMERGENCY OFF: ");
    Serial.println(relayName);
  }
}

// Basic self-test
bool SOLID_STATE_RELAY::selfTest() {
  if (!initialized) {
    Serial.print("❌ Self-test failed: ");
    Serial.print(relayName);
    Serial.println(" not initialized");
    return false;
  }
  
  Serial.print("🔧 Self-testing ");
  Serial.print(relayName);
  Serial.println("...");
  
  // Test OFF state
  turnOff();
  delay(100);
  bool offTest = (digitalRead(ssrPin) == LOW) && (ssrState == false);
  
  // Test ON state  
  turnOn();
  delay(100);
  bool onTest = (digitalRead(ssrPin) == HIGH) && (ssrState == true);
  
  // Return to OFF state for safety
  turnOff();
  
  bool testResult = offTest && onTest;
  
  Serial.print("🔧 Self-test ");
  Serial.print(relayName);
  Serial.print(": ");
  Serial.println(testResult ? "PASSED" : "FAILED");
  
  return testResult;
}

// Static method: Turn off all relays in array
void SOLID_STATE_RELAY::turnOffAll(SOLID_STATE_RELAY* relays[], int count) {
  Serial.print("🔴 Turning OFF all ");
  Serial.print(count);
  Serial.println(" SSRs...");
  
  for (int i = 0; i < count; i++) {
    if (relays[i] != nullptr && relays[i]->isInitialized()) {
      relays[i]->turnOff();
    }
  }
  
  Serial.println("✅ All SSRs turned OFF");
}

// Static method: Turn on all relays in array
void SOLID_STATE_RELAY::turnOnAll(SOLID_STATE_RELAY* relays[], int count) {
  Serial.print("🟢 Turning ON all ");
  Serial.print(count);
  Serial.println(" SSRs...");
  
  for (int i = 0; i < count; i++) {
    if (relays[i] != nullptr && relays[i]->isInitialized()) {
      relays[i]->turnOn();
    }
  }
  
  Serial.println("✅ All SSRs turned ON");
}

// Static method: Count active (ON) relays
int SOLID_STATE_RELAY::countActiveRelays(SOLID_STATE_RELAY* relays[], int count) {
  int activeCount = 0;
  
  for (int i = 0; i < count; i++) {
    if (relays[i] != nullptr && relays[i]->isInitialized() && relays[i]->getState()) {
      activeCount++;
    }
  }
  
  return activeCount;
}