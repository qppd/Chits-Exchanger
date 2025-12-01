/*
 * COIN_HOPPER.cpp
 * Implementation file for ALLAN Coin Hopper control
 * 
 * This file contains the implementation of interrupt-based pulse counting
 * and coin dispensing functionality for the ALLAN coin hopper.
 */

#include "PIN_CONFIGURATION.h"
#include "COIN_HOPPER.h"
#include "SOLID_STATE_RELAY.h"

// Static member initialization
COIN_HOPPER* COIN_HOPPER::instances[3] = {nullptr, nullptr, nullptr};
COIN_HOPPER* g_coinHopperInstances[3] = {nullptr, nullptr, nullptr};

// Default Constructor
COIN_HOPPER::COIN_HOPPER() {
    pulsePin = COIN_HOPPER_1_PULSE_PIN;
    hopperId = 0; // Default to hopper 1
    coinValue = COIN_HOPPER_1_VALUE;
    debounceTime = 40; // Set to 40ms for faster coin detection
    ssr = nullptr;  // Initialize SSR pointer to null
    pulseCount = 0;
    lastPulseTime = 0;
    lastDebounceTime = 0;
    pulseRateIndex = 0;
    lastRateCalculation = 0;
    isInitialized = false;
    isDispensing = false;
    dispensingStartTime = 0;
    targetDispenseCount = 0;
    targetDispenseAmount = 0;
    dispensedAmount = 0;
    totalCoinsDetected = 0;
    currentPulseRate = 0.0;
    waitingForPulseISR = false;
    pulseDetectedISR = false;
    
    // Initialize pulse rate buffer
    for (int i = 0; i < 10; i++) {
        pulseRateBuffer[i] = 0;
    }
}

// Constructor with hopper ID
COIN_HOPPER::COIN_HOPPER(int hopperIdNumber) {
    hopperId = hopperIdNumber;
    
    // Set default pins and coin values based on hopper ID
    switch(hopperId) {
        case 0: 
            pulsePin = COIN_HOPPER_1_PULSE_PIN; 
            coinValue = COIN_HOPPER_1_VALUE;
            debounceTime = 40; // 5 peso: 40ms
            break;
        case 1: 
            pulsePin = COIN_HOPPER_2_PULSE_PIN; 
            coinValue = COIN_HOPPER_2_VALUE;
            debounceTime = 40; // 10 peso: 40ms
            break;
        case 2: 
            pulsePin = COIN_HOPPER_3_PULSE_PIN; 
            coinValue = COIN_HOPPER_3_VALUE;
            debounceTime = 40; // 20 peso: 40ms
            break;
        default: 
            pulsePin = COIN_HOPPER_1_PULSE_PIN; 
            coinValue = COIN_HOPPER_1_VALUE;
            debounceTime = 40;
            hopperId = 0; 
            break;
    }
    
    ssr = nullptr;  // Initialize SSR pointer to null
    
    pulseCount = 0;
    lastPulseTime = 0;
    lastDebounceTime = 0;
    pulseRateIndex = 0;
    lastRateCalculation = 0;
    isInitialized = false;
    isDispensing = false;
    dispensingStartTime = 0;
    targetDispenseCount = 0;
    targetDispenseAmount = 0;
    dispensedAmount = 0;
    totalCoinsDetected = 0;
    currentPulseRate = 0.0;
    waitingForPulseISR = false;
    pulseDetectedISR = false;
    
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
    // Set default SSR pin based on hopper ID
    int defaultSSRPin;
    switch(hopperIdNumber) {
        case 0: defaultSSRPin = COIN_HOPPER_1_SSR_PIN; break;
        case 1: defaultSSRPin = COIN_HOPPER_2_SSR_PIN; break;
        case 2: defaultSSRPin = COIN_HOPPER_3_SSR_PIN; break;
        default: defaultSSRPin = COIN_HOPPER_1_SSR_PIN; break;
    }
    
    return begin(pulsePinNumber, hopperIdNumber, defaultSSRPin);
}

bool COIN_HOPPER::begin(int pulsePinNumber, int hopperIdNumber, int ssrPinNumber) {
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
    
    // Initialize SSR controller
    String ssrName = "Hopper" + String(hopperId + 1) + "_SSR";
    if (!initializeSSR(ssrPinNumber, ssrName)) {
        Serial.print("ERROR: Failed to initialize SSR for hopper ");
        Serial.println(hopperId);
        return false;
    }
    
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
    Serial.print(" initialized - Pulse: GPIO");
    Serial.print(pulsePin);
    Serial.print(", SSR: GPIO");
    Serial.print(ssr->getPin());
    Serial.print(", Debounce: ");
    Serial.print(debounceTime);
    Serial.println("ms");
    
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
    
    // Use per-hopper debounce timing (hopper 3/20peso needs longer debounce)
    if (currentTime - lastPulseTime > debounceTime) {
        pulseCount++;
        lastPulseTime = currentTime;
        totalCoinsDetected++;
        
        // If we are actively waiting for a pulse, shut off SSR immediately
        if (isDispensing && waitingForPulseISR) {
            pulseDetectedISR = true;   // mark pulse arrival
            waitingForPulseISR = false; // prevent duplicate handling
            if (ssr != nullptr) {
                // Direct hardware cut (avoid Serial inside ISR)
                int pin = ssr->getPin();
                if (pin >= 0) {
                    digitalWrite(pin, LOW);
                }
            }
        }

        // Also ensure target auto-shutoff
        if (isDispensing && targetDispenseCount > 0) {
            unsigned long currentCount = pulseCount - initialPulseCount;
            if (currentCount >= targetDispenseCount) {
                if (ssr != nullptr) {
                    int pin = ssr->getPin();
                    if (pin >= 0) {
                        digitalWrite(pin, LOW);
                    }
                }
            }
        }
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
    
    // Wait a bit to ensure any previous coin has fully exited the sensor
    delay(200);
    
    // NOW capture the actual starting count (after any residual pulses)
    initialCount = pulseCount;
    
    // Turn ON motor ONCE at the start
    Serial.println("🟢 Motor ON - Starting continuous dispensing");
    enableSSR();
    
    // Wait for all coins to be dispensed (keep motor running)
    unsigned long targetCount = initialCount + numberOfCoins;
    unsigned long timeout = millis() + DISPENSE_TIMEOUT_MS;
    
    Serial.print("Waiting for ");
    Serial.print(numberOfCoins);
    Serial.println(" pulses...");
    
    while (pulseCount < targetCount && millis() < timeout) {
        delay(10);
        update();
        
        // Show progress when a pulse is detected
        unsigned long currentDispensed = pulseCount - initialCount;
        static unsigned long lastReported = 0;
        
        if (currentDispensed > lastReported) {
            Serial.print("✓ Coin ");
            Serial.print(currentDispensed);
            Serial.print(" of ");
            Serial.print(numberOfCoins);
            Serial.println(" detected");
            lastReported = currentDispensed;
        }
    }
    
    unsigned long actualDispensed = pulseCount - initialCount;
    
    // Add small delay before turning off motor to ensure last coin drops
    Serial.println("⏱️  Waiting 200ms for last coin to drop...");
    delay(200);
    
    // Turn OFF motor after all coins dispensed
    Serial.println("🔴 Motor OFF");
    disableSSR();
    
    stopDispensing();
    
    Serial.println("\n========== DISPENSING SUMMARY ==========");
    Serial.print("Requested: ");
    Serial.print(numberOfCoins);
    Serial.println(" coins");
    Serial.print("Dispensed: ");
    Serial.print(actualDispensed);
    Serial.println(" coins (based on pulses)");
    
    if (actualDispensed != numberOfCoins) {
        Serial.println("⚠ MISMATCH DETECTED!");
        Serial.print("  Expected pulses: ");
        Serial.println(numberOfCoins);
        Serial.print("  Actual pulses: ");
        Serial.println(actualDispensed);
        Serial.print("  Difference: ");
        Serial.println((int)actualDispensed - numberOfCoins);
        
        // Check if timeout occurred
        if (millis() >= timeout) {
            Serial.println("  Reason: TIMEOUT");
        }
    } else {
        Serial.println("✓ Perfect match!");
    }
    Serial.println("========================================\n");
    
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

// SSR Control methods (delegated to SOLID_STATE_RELAY)
void COIN_HOPPER::enableSSR() {
    if (!isInitialized || ssr == nullptr) {
        Serial.println("⚠ WARNING: Cannot enable SSR - not initialized!");
        return;
    }
    
    Serial.print("  [SSR] Turning ON SSR on GPIO ");
    Serial.println(ssr->getPin());
    ssr->turnOn();
}

void COIN_HOPPER::disableSSR() {
    if (!isInitialized || ssr == nullptr) {
        Serial.println("⚠ WARNING: Cannot disable SSR - not initialized!");
        return;
    }
    
    Serial.print("  [SSR] Turning OFF SSR on GPIO ");
    Serial.println(ssr->getPin());
    ssr->turnOff();
}

void COIN_HOPPER::setSSRState(bool state) {
    if (!isInitialized || ssr == nullptr) return;
    
    ssr->setState(state);
}

bool COIN_HOPPER::getSSRState() const {
    if (ssr == nullptr) return false;
    return ssr->getState();
}

int COIN_HOPPER::getSSRPin() const {
    if (ssr == nullptr) return -1;
    return ssr->getPin();
}

SOLID_STATE_RELAY* COIN_HOPPER::getSSRController() const {
    return ssr;
}

bool COIN_HOPPER::initializeSSR(int ssrPin, String ssrName) {
    // Clean up existing SSR if any
    if (ssr != nullptr) {
        delete ssr;
        ssr = nullptr;
    }
    
    // Create new SSR instance
    ssr = new SOLID_STATE_RELAY();
    if (ssr == nullptr) {
        Serial.println("ERROR: Failed to create SSR instance");
        return false;
    }
    
    // Initialize the SSR
    String name = ssrName.isEmpty() ? ("Hopper" + String(hopperId + 1) + "_SSR") : ssrName;
    if (!ssr->begin(ssrPin, name)) {
        delete ssr;
        ssr = nullptr;
        return false;
    }
    
    return true;
}



// Amount-based dispensing methods
bool COIN_HOPPER::dispenseAmount(int amountInPesos) {
    if (!isInitialized || isDispensing) {
        Serial.println("Cannot dispense: hopper not ready or already dispensing");
        return false;
    }
    
    if (amountInPesos <= 0 || amountInPesos % coinValue != 0) {
        Serial.print("Invalid amount: ");
        Serial.print(amountInPesos);
        Serial.print(" PHP. Must be multiple of ");
        Serial.println(coinValue);
        return false;
    }
    
    int coinsNeeded = calculateCoinsNeeded(amountInPesos);
    if (coinsNeeded <= 0) {
        Serial.println("Cannot dispense: amount not achievable with this coin value");
        return false;
    }
    
    Serial.print("\n=== DISPENSING ");
    Serial.print(amountInPesos);
    Serial.print(" PHP (");
    Serial.print(coinsNeeded);
    Serial.println(" coins) ===");
    
    // Ensure relay is OFF before starting
    disableSSR();
    unsigned long settleStart = millis();
    while (millis() - settleStart < 5) {
        // brief settle without blocking long
        yield();
    }
    
    // Capture starting pulse count
    unsigned long startPulseCount = pulseCount;
    unsigned long dispensingStart = millis();
    int coinsDispensed = 0;
    
    // Dispensing state machine
    enum State { IDLE, RELAY_ON, WAIT_PULSE, RELAY_OFF, INTER_WAIT, DONE };
    State state = IDLE;
    unsigned long lastPulseCount = startPulseCount;
    unsigned long stateStartTime = 0;
    
    while (state != DONE && (millis() - dispensingStart < DISPENSE_TIMEOUT_MS)) {
        // Always check if target reached
        unsigned long currentCoins = pulseCount - startPulseCount;
        if (currentCoins >= coinsNeeded) {
            Serial.print("✓ TARGET REACHED: ");
            Serial.print(currentCoins);
            Serial.println(" coins dispensed!");
            disableSSR();
            state = DONE;
            break;
        }
        
        switch (state) {
            case IDLE:
                // Check if we need more coins
                if (coinsDispensed < coinsNeeded) {
                    Serial.print("\n[Coin ");
                    Serial.print(coinsDispensed + 1);
                    Serial.print("/");
                    Serial.print(coinsNeeded);
                    Serial.println("] Starting...");
                    state = RELAY_ON;
                    stateStartTime = millis();
                } else {
                    state = DONE;
                }
                break;
                
            case RELAY_ON:
                // Turn ON relay
                Serial.println("  → SSR ON");
                enableSSR();
                lastPulseCount = pulseCount;
                state = WAIT_PULSE;
                stateStartTime = millis();
                waitingForPulseISR = true;
                pulseDetectedISR = false;
                break;
                
            case WAIT_PULSE:
                // Wait for pulse (non-blocking)
                if (pulseDetectedISR || pulseCount > lastPulseCount) {
                    // Pulse detected!
                    coinsDispensed++;
                    Serial.print("  ✓ PULSE detected! Total: ");
                    Serial.println(pulseCount - startPulseCount);
                    state = RELAY_OFF;
                } else if (millis() - stateStartTime > 5000) {
                    // Timeout
                    Serial.println("  ⚠ Timeout - no pulse");
                    state = RELAY_OFF;
                }
                break;
                
            case RELAY_OFF:
                // Turn OFF relay immediately and forcefully
                Serial.println("  → SSR OFF");
                disableSSR();
                // Start short non-blocking settle
                stateStartTime = millis();
                state = INTER_WAIT;
                break;

            case INTER_WAIT:
                // Non-blocking settle and inter-coin spacing
                // 40ms settle + 120ms spacing ~= 160ms total
                if (millis() - stateStartTime >= 160) {
                    // Check if target reached
                    if ((pulseCount - startPulseCount) >= coinsNeeded) {
                        state = DONE;
                    } else {
                        state = IDLE;
                    }
                }
                break;
                
            case DONE:
                // Do nothing, will exit loop
                break;
        }
        
        // Minimal yield to prevent watchdog
        yield();
    }
    
    // Final cleanup
    disableSSR();
    
    unsigned long finalCoins = pulseCount - startPulseCount;
    int actualAmount = finalCoins * coinValue;
    
    Serial.print("\n=== RESULT ===");
    Serial.print("\n  Requested: ");
    Serial.print(coinsNeeded);
    Serial.print(" coins (");
    Serial.print(amountInPesos);
    Serial.println(" PHP)");
    Serial.print("  Dispensed: ");
    Serial.print(finalCoins);
    Serial.print(" coins (");
    Serial.print(actualAmount);
    Serial.println(" PHP)");
    Serial.print("  Time: ");
    Serial.print((millis() - dispensingStart) / 1000.0);
    Serial.println(" sec");
    Serial.println("==============\n");
    
    return (finalCoins == coinsNeeded);
}

int COIN_HOPPER::calculateCoinsNeeded(int amountInPesos) const {
    if (amountInPesos % coinValue == 0) {
        return amountInPesos / coinValue;
    }
    return 0; // Cannot achieve this amount with this coin value
}

int COIN_HOPPER::getDispensedAmount() const {
    return dispensedAmount;
}

int COIN_HOPPER::getTargetAmount() const {
    return targetDispenseAmount;
}

int COIN_HOPPER::getCoinValue() const {
    return coinValue;
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