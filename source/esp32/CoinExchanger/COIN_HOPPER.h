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

// Coin hopper constants
#define DEBOUNCE_TIME_MS 25          // Reduced debounce time for real-time counting
#define MAX_PULSE_RATE 20            // Maximum expected pulses per second
#define DISPENSE_TIMEOUT_MS 30000    // Timeout for coin dispensing operation
#define PULSE_RATE_WINDOW_MS 5000    // Window for calculating pulse rate

class COIN_HOPPER {
private:
    // Hardware pins
    int pulsePin;
    int hopperId;  // Unique identifier for this hopper instance
    
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
    bool isCurrentlyDispensing() const;
    void stopDispensing();
    
    // Status methods
    bool isReady() const;
    void printStatus() const;
    
    // Configuration methods
    void setDebounceTime(unsigned long debounceMs);
    void setPulsePin(int pin);
    int getHopperId() const;
    int getPulsePin() const;
    
    // Diagnostics
    void runDiagnostics();
    bool testPulseDetection(int testDurationMs = 5000);
};

// Global instance pointers for interrupt handling (multiple hoppers)
extern COIN_HOPPER* g_coinHopperInstances[3];  // Fixed size instead of NUM_COIN_HOPPERS

#endif // COIN_HOPPER_H