/*
 * COIN_HOPPER.h
 * Header file for ALLAN Coin Hopper control
 * 
 * This file contains the class definition for controlling an ALLAN coin hopper
 * using interrupt-based pulse counting on ESP32.
 * 
 * Features:
 * - Interrupt-based pulse detection
 * - Coin counting and dispensing
 * - Pulse rate calculation
 * - Hardware control functions
 */

#ifndef COIN_HOPPER_H
#define COIN_HOPPER_H

#include <Arduino.h>
#include "SOLID_STATE_RELAY.h"

// Coin hopper constants
#define DEBOUNCE_TIME_MS 100         // Debounce time to prevent double-counting coins
#define MAX_PULSE_RATE 20            // Maximum expected pulses per second
#define DISPENSE_TIMEOUT_MS 30000    // Timeout for coin dispensing operation
#define PULSE_RATE_WINDOW_MS 5000    // Window for calculating pulse rate

class COIN_HOPPER {
private:
    // Hardware pins and components
    int pulsePin;
    int hopperId;     // Unique identifier for this hopper instance
    int coinValue;    // Value of coins in this hopper (PHP)
    SOLID_STATE_RELAY* ssr;  // SSR controller instance
    
    // Pulse counting variables
    volatile unsigned long pulseCount;
    volatile unsigned long lastPulseTime;
    volatile unsigned long lastDebounceTime;
    
    // Pulse rate calculation
    unsigned long pulseRateBuffer[10];
    int pulseRateIndex;
    unsigned long lastRateCalculation;
    
    // State variables
    bool isInitialized;
    bool isDispensing;
    unsigned long dispensingStartTime;
    int targetDispenseCount;
    int targetDispenseAmount;  // Target amount in pesos
    int dispensedAmount;       // Amount dispensed so far in pesos
    
    // Statistics
    unsigned long totalCoinsDetected;
    float currentPulseRate;
    
    // Static arrays for multiple instance support
    static COIN_HOPPER* instances[3];  // Fixed size instead of NUM_COIN_HOPPERS
    static void IRAM_ATTR pulseISR_Hopper1();
    static void IRAM_ATTR pulseISR_Hopper2();
    static void IRAM_ATTR pulseISR_Hopper3();
    
    // Private helper methods
    void handlePulseInterrupt();
    void calculatePulseRate();
    void updatePulseRateBuffer();
    
public:
    // Constructor
    COIN_HOPPER();
    COIN_HOPPER(int hopperIdNumber);
    
    // Initialization
    bool begin();
    bool begin(int pulsePinNumber);
    bool begin(int pulsePinNumber, int hopperIdNumber);
    bool begin(int pulsePinNumber, int hopperIdNumber, int ssrPinNumber);
    
    // Core functionality
    void update();
    
    // Pulse counting methods
    unsigned long getTotalCoins() const;
    unsigned long getPulseCount() const;
    void resetCounter();
    
    // Pulse rate methods
    float getPulseRate() const;
    unsigned long getLastPulseTime() const;
    
    // Coin dispensing methods
    bool dispenseCoins(int numberOfCoins);
    bool dispenseAmount(int amountInPesos);
    int calculateCoinsNeeded(int amountInPesos) const;
    bool isCurrentlyDispensing() const;
    void stopDispensing();
    int getDispensedAmount() const;
    int getTargetAmount() const;
    int getCoinValue() const;
    
    // Status methods
    bool isReady() const;
    void printStatus() const;
    
    // SSR Control methods (delegated to SOLID_STATE_RELAY)
    void enableSSR();
    void disableSSR();
    void setSSRState(bool state);
    bool getSSRState() const;
    int getSSRPin() const;
    SOLID_STATE_RELAY* getSSRController() const;
    
    // Configuration methods
    void setDebounceTime(unsigned long debounceMs);
    void setPulsePin(int pin);
    bool initializeSSR(int ssrPin, String ssrName = "");
    int getHopperId() const;
    int getPulsePin() const;
    
    // Diagnostics
    void runDiagnostics();
    bool testPulseDetection(int testDurationMs = 5000);
};

// Global instance pointers for interrupt handling (multiple hoppers)
extern COIN_HOPPER* g_coinHopperInstances[3];  // Fixed size instead of NUM_COIN_HOPPERS

#endif // COIN_HOPPER_H
