/*
 * COIN_HOPPER.cpp
 * Implementation file for ALLAN Coin Hopper control
 * 
 * This file contains the implementation of interrupt-based pulse counting
 * and coin dispensing functionality for the ALLAN coin hopper.
 */

#include "PIN_CONFIGURATION.h"
#include "COIN_HOPPER.h"

// Static member initialization
COIN_HOPPER* COIN_HOPPER::instances[3] = {nullptr, nullptr, nullptr};
COIN_HOPPER* g_coinHopperInstances[3] = {nullptr, nullptr, nullptr};

// Default Constructor
COIN_HOPPER::COIN_HOPPER() {
    pulsePin = COIN_HOPPER_1_PULSE_PIN;
    hopperId = 0; // Default to hopper 1
    pulseCount = 0;
    lastPulseTime = 0;
    lastDebounceTime = 0;
    pulseRateIndex = 0;
    lastRateCalculation = 0;
    isInitialized = false;
    isDispensing = false;
    dispensingStartTime = 0;
    targetDispenseCount = 0;
    totalCoinsDetected = 0;
    currentPulseRate = 0.0;
    
    // Initialize pulse rate buffer
    for (int i = 0; i < 10; i++) {
        pulseRateBuffer[i] = 0;
    }
}

// Constructor with hopper ID
COIN_HOPPER::COIN_HOPPER(int hopperIdNumber) {
    hopperId = hopperIdNumber;
    
    // Set default pin based on hopper ID
    switch(hopperId) {
        case 0: pulsePin = COIN_HOPPER_1_PULSE_PIN; break;
        case 1: pulsePin = COIN_HOPPER_2_PULSE_PIN; break;
        case 2: pulsePin = COIN_HOPPER_3_PULSE_PIN; break;
        default: pulsePin = COIN_HOPPER_1_PULSE_PIN; hopperId = 0; break;
    }
    
    pulseCount = 0;
    lastPulseTime = 0;
    lastDebounceTime = 0;
    pulseRateIndex = 0;
    lastRateCalculation = 0;
    isInitialized = false;
    isDispensing = false;
    dispensingStartTime = 0;
    targetDispenseCount = 0;
    totalCoinsDetected = 0;
    currentPulseRate = 0.0;
    
    // Initialize pulse rate buffer
    for (int i = 0; i < 10; i++) {
        pulseRateBuffer[i] = 0;
    }
}

// Initialization methods
bool COIN_HOPPER::begin() {
    return begin(pulsePin, hopperId);
}

bool COIN_HOPPER::begin(int pulsePinNumber) {
    return begin(pulsePinNumber, hopperId);
}

bool COIN_HOPPER::begin(int pulsePinNumber, int hopperIdNumber) {
    pulsePin = pulsePinNumber;
    hopperId = hopperIdNumber;
    
    // Validate hopper ID
    if (hopperId < 0 || hopperId >= 3) {
        Serial.print("ERROR: Invalid hopper ID ");
        Serial.println(hopperId);
        return false;
    }
    
    // Store instance in static array for interrupt handling
    instances[hopperId] = this;
    g_coinHopperInstances[hopperId] = this;
    
    // Configure pulse pin as input with pull-up
    pinMode(pulsePin, INPUT_PULLUP);
    
    // Attach appropriate interrupt based on hopper ID
    switch(hopperId) {
        case 0:
            attachInterrupt(digitalPinToInterrupt(pulsePin), pulseISR_Hopper1, FALLING);
            break;
        case 1:
            attachInterrupt(digitalPinToInterrupt(pulsePin), pulseISR_Hopper2, FALLING);
            break;
        case 2:
            attachInterrupt(digitalPinToInterrupt(pulsePin), pulseISR_Hopper3, FALLING);
            break;
        default:
            Serial.println("ERROR: No ISR available for this hopper ID");
            return false;
    }
    
    // Initialize timing
    lastRateCalculation = millis();
    
    isInitialized = true;
    
    Serial.print("COIN_HOPPER ");
    Serial.print(hopperId + 1);
    Serial.print(" initialized on GPIO");
    Serial.println(pulsePin);
    
    return true;
}

// Static interrupt service routines for each hopper
void IRAM_ATTR COIN_HOPPER::pulseISR_Hopper1() {
    if (instances[0] != nullptr) {
        instances[0]->handlePulseInterrupt();
    }
}

void IRAM_ATTR COIN_HOPPER::pulseISR_Hopper2() {
    if (instances[1] != nullptr) {
        instances[1]->handlePulseInterrupt();
    }
}

void IRAM_ATTR COIN_HOPPER::pulseISR_Hopper3() {
    if (instances[2] != nullptr) {
        instances[2]->handlePulseInterrupt();
    }
}

// Interrupt handler (called from ISR)
void IRAM_ATTR COIN_HOPPER::handlePulseInterrupt() {
    unsigned long currentTime = millis();
    
    // Debounce check - reduced for more responsive counting
    if (currentTime - lastDebounceTime > (DEBOUNCE_TIME_MS / 2)) {
        pulseCount++;
        lastPulseTime = currentTime;
        lastDebounceTime = currentTime;
        totalCoinsDetected++;
    }
}

// Main update function
void COIN_HOPPER::update() {
    if (!isInitialized) return;
    
    // Calculate pulse rate periodically
    if (millis() - lastRateCalculation >= 1000) {
        calculatePulseRate();
        lastRateCalculation = millis();
    }
    
    // Handle dispensing timeout
    if (isDispensing && (millis() - dispensingStartTime > DISPENSE_TIMEOUT_MS)) {
        Serial.println("Dispensing timeout - stopping operation");
        stopDispensing();
    }
}

// Pulse counting methods
unsigned long COIN_HOPPER::getTotalCoins() const {
    return totalCoinsDetected;
}

unsigned long COIN_HOPPER::getPulseCount() const {
    return pulseCount;
}

void COIN_HOPPER::resetCounter() {
    noInterrupts();
    pulseCount = 0;
    totalCoinsDetected = 0;
    lastPulseTime = 0;
    interrupts();
    
    // Reset pulse rate buffer
    for (int i = 0; i < 10; i++) {
        pulseRateBuffer[i] = 0;
    }
    pulseRateIndex = 0;
    currentPulseRate = 0.0;
    
    Serial.println("Coin counter reset");
}

// Pulse rate calculation
void COIN_HOPPER::calculatePulseRate() {
    static unsigned long lastPulseCountForRate = 0;
    unsigned long currentPulseCount = pulseCount;
    
    // Calculate pulses in the last second
    unsigned long pulsesThisSecond = currentPulseCount - lastPulseCountForRate;
    lastPulseCountForRate = currentPulseCount;
    
    // Update circular buffer
    pulseRateBuffer[pulseRateIndex] = pulsesThisSecond;
    pulseRateIndex = (pulseRateIndex + 1) % 10;
    
    // Calculate average pulse rate over last 10 seconds
    unsigned long totalPulses = 0;
    for (int i = 0; i < 10; i++) {
        totalPulses += pulseRateBuffer[i];
    }
    
    currentPulseRate = totalPulses / 10.0;
}

float COIN_HOPPER::getPulseRate() const {
    return currentPulseRate;
}

unsigned long COIN_HOPPER::getLastPulseTime() const {
    if (lastPulseTime == 0) return 0;
    return millis() - lastPulseTime;
}

// Coin dispensing methods
bool COIN_HOPPER::dispenseCoins(int numberOfCoins) {
    if (!isInitialized || isDispensing) {
        Serial.println("Cannot dispense: hopper not ready or already dispensing");
        return false;
    }
    
    if (numberOfCoins <= 0) {
        Serial.println("Invalid number of coins to dispense");
        return false;
    }
    
    Serial.print("Starting dispensing sequence for ");
    Serial.print(numberOfCoins);
    Serial.println(" coins");
    
    // Reset pulse counter for this dispensing operation
    unsigned long initialCount = pulseCount;
    
    isDispensing = true;
    dispensingStartTime = millis();
    targetDispenseCount = numberOfCoins;
    
    // Note: In a real implementation, you would control a motor or solenoid here
    // to actually dispense coins. This is a simulation that waits for pulses.
    
    // Wait for the required number of pulses or timeout
    while (isDispensing && (pulseCount - initialCount < numberOfCoins)) {
        delay(10);
        update();
        
        // Check for timeout
        if (millis() - dispensingStartTime > DISPENSE_TIMEOUT_MS) {
            Serial.println("Dispensing timeout reached");
            break;
        }
    }
    
    stopDispensing();
    
    unsigned long actualDispensed = pulseCount - initialCount;
    Serial.print("Dispensed ");
    Serial.print(actualDispensed);
    Serial.print(" of ");
    Serial.print(numberOfCoins);
    Serial.println(" requested coins");
    
    return (actualDispensed == numberOfCoins);
}

bool COIN_HOPPER::isCurrentlyDispensing() const {
    return isDispensing;
}

void COIN_HOPPER::stopDispensing() {
    isDispensing = false;
    dispensingStartTime = 0;
    targetDispenseCount = 0;
    
    // Note: In a real implementation, you would stop the motor/solenoid here
}

// Status methods
bool COIN_HOPPER::isReady() const {
    return isInitialized && !isDispensing;
}

void COIN_HOPPER::printStatus() const {
    Serial.println("=== COIN HOPPER STATUS ===");
    Serial.print("Initialized: ");
    Serial.println(isInitialized ? "Yes" : "No");
    Serial.print("Pulse Pin: GPIO");
    Serial.println(pulsePin);
    Serial.print("Total Coins: ");
    Serial.println(totalCoinsDetected);
    Serial.print("Pulse Count: ");
    Serial.println(pulseCount);
    Serial.print("Pulse Rate: ");
    Serial.print(currentPulseRate);
    Serial.println(" pulses/sec");
    Serial.print("Last Pulse: ");
    Serial.print(getLastPulseTime());
    Serial.println(" ms ago");
    Serial.print("Currently Dispensing: ");
    Serial.println(isDispensing ? "Yes" : "No");
    Serial.println("========================");
}

// Configuration methods
void COIN_HOPPER::setDebounceTime(unsigned long debounceMs) {
    // This would require modifying the constant, which isn't possible at runtime
    // In a more advanced implementation, this could be made configurable
    Serial.print("Debounce time is fixed at ");
    Serial.print(DEBOUNCE_TIME_MS);
    Serial.println(" ms");
}

void COIN_HOPPER::setPulsePin(int pin) {
    if (isInitialized) {
        Serial.println("Cannot change pulse pin after initialization");
        return;
    }
    pulsePin = pin;
}

int COIN_HOPPER::getHopperId() const {
    return hopperId;
}

int COIN_HOPPER::getPulsePin() const {
    return pulsePin;
}

// Diagnostics
void COIN_HOPPER::runDiagnostics() {
    Serial.println("=== COIN HOPPER DIAGNOSTICS ===");
    
    // Test pin configuration
    Serial.print("Testing GPIO");
    Serial.print(pulsePin);
    Serial.println(" configuration...");
    
    // Read current pin state
    int pinState = digitalRead(pulsePin);
    Serial.print("Current pin state: ");
    Serial.println(pinState ? "HIGH" : "LOW");
    
    // Test interrupt functionality
    Serial.println("Testing interrupt functionality for 5 seconds...");
    Serial.println("Generate pulses on the pin to test...");
    
    unsigned long testStart = millis();
    unsigned long initialCount = pulseCount;
    
    while (millis() - testStart < 5000) {
        delay(100);
        if (pulseCount > initialCount) {
            Serial.print("Pulse detected! Count: ");
            Serial.println(pulseCount - initialCount);
            initialCount = pulseCount;
        }
    }
    
    Serial.println("Diagnostics complete");
    Serial.println("==============================");
}

bool COIN_HOPPER::testPulseDetection(int testDurationMs) {
    Serial.print("Testing pulse detection for ");
    Serial.print(testDurationMs);
    Serial.println(" ms...");
    
    unsigned long startTime = millis();
    unsigned long startCount = pulseCount;
    
    while (millis() - startTime < testDurationMs) {
        delay(10);
    }
    
    unsigned long endCount = pulseCount;
    unsigned long detectedPulses = endCount - startCount;
    
    Serial.print("Detected ");
    Serial.print(detectedPulses);
    Serial.println(" pulses during test");
    
    return detectedPulses > 0;
}