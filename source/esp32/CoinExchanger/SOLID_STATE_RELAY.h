/*
 * SOLID_STATE_RELAY.h
 * ESP32 Solid State Relay Control Class
 * 
 * This class provides control for Solid State Relays (SSR) used to power
 * coin hoppers on/off. Follows SOLID principles with single responsibility.
 * 
 * Features:
 * - GPIO pin control for SSR
 * - State management (ON/OFF)
 * - Status reporting
 * - Safe initialization and cleanup
 * 
 * Author: ESP32 CoinExchanger System
 * Date: October 2025
 */

#ifndef SOLID_STATE_RELAY_H
#define SOLID_STATE_RELAY_H

#include <Arduino.h>

class SOLID_STATE_RELAY {
private:
  int ssrPin;           // GPIO pin connected to SSR control
  bool ssrState;        // Current state of SSR (true = ON, false = OFF)
  bool initialized;     // Track if SSR has been properly initialized
  String relayName;     // Optional name for identification
  
public:
  // Constructor
  SOLID_STATE_RELAY();
  SOLID_STATE_RELAY(int pin);
  SOLID_STATE_RELAY(int pin, String name);
  
  // Destructor
  ~SOLID_STATE_RELAY();
  
  // Initialization
  bool begin(int pin);
  bool begin(int pin, String name);
  
  // Core SSR Control Methods
  void turnOn();        // Enable SSR (power ON)
  void turnOff();       // Disable SSR (power OFF)
  void setState(bool state);  // Set specific state (true = ON, false = OFF)
  void toggle();        // Toggle current state
  
  // Status and Information Methods
  bool getState() const;           // Get current SSR state
  int getPin() const;              // Get GPIO pin number
  String getName() const;          // Get relay name
  bool isInitialized() const;      // Check if properly initialized
  
  // Utility Methods
  void setName(String name);       // Set relay name for identification
  String getStatusString() const;  // Get human-readable status
  
  // Safety Methods
  void emergencyOff();            // Emergency shutdown (immediate OFF)
  bool selfTest();                // Basic self-test functionality
  
  // Static Methods for Multiple SSR Management
  static void turnOffAll(SOLID_STATE_RELAY* relays[], int count);
  static void turnOnAll(SOLID_STATE_RELAY* relays[], int count);
  static int countActiveRelays(SOLID_STATE_RELAY* relays[], int count);
};

#endif // SOLID_STATE_RELAY_H