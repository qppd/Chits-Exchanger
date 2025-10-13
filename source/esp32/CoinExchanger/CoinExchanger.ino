/*
 * CoinExchanger.ino
 * ESP32 Coin Exchanger with ALLAN Coin Hopper
 * 
 * This program reads pulses from an ALLAN coin hopper using interrupts
 * and provides coin dispensing functionality.
 * 
 * Hardware Connections:
 * - ALLAN Coin Hopper:
 *   Pin 1: 3.3V (Power)
 *   Pin 2: GPIO19 (Pulse Signal)
 *   Pin 3: GND (Ground)
 *   Pin 4: Unused
 * 
 * Author: ESP32 CoinExchanger System
 * Date: October 2025
 */

#include "PIN_CONFIGURATION.h"
#include "COIN_HOPPER.h"

// Global variables
COIN_HOPPER coinHopper;
unsigned long lastSerialOutput = 0;
const unsigned long SERIAL_OUTPUT_INTERVAL = 100; // Output every 100ms for real-time
unsigned long lastCoinCount = 0;
bool realTimeMode = true;

// RPi Communication variables
bool countingActive = false;
bool rpiMode = false;
unsigned long sessionStartCount = 0;
unsigned long sessionStartTime = 0;
String inputBuffer = "";

// Communication protocol constants
const String CMD_START_COUNT = "START_COUNT";
const String CMD_STOP_COUNT = "STOP_COUNT";
const String CMD_GET_COUNT = "GET_COUNT";
const String CMD_GET_STATUS = "GET_STATUS";
const String CMD_RESET_COUNT = "RESET_COUNT";
const String CMD_SET_RPI_MODE = "SET_RPI_MODE";
const String CMD_PING = "PING";

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // Wait for serial to initialize
  }
  
  Serial.println("=== ESP32 Coin Exchanger System ===");
  Serial.println("Initializing ALLAN Coin Hopper...");
  
  // Initialize coin hopper
  coinHopper.begin();
  
  Serial.println("System ready!");
  Serial.println("Coin hopper is monitoring for pulses...");
  Serial.println("Commands:");
  Serial.println("  Manual: 'count', 'reset', 'dispense X', 'realtime on/off'");
  Serial.println("  RPi: 'START_COUNT', 'STOP_COUNT', 'GET_COUNT', 'GET_STATUS'");
  Serial.println("       'RESET_COUNT', 'SET_RPI_MODE ON/OFF', 'PING'");
  Serial.println("================================");
  Serial.println("Real-time counting: ENABLED");
  Serial.println("RPi Mode: DISABLED (use SET_RPI_MODE ON to enable)");
  Serial.println("Drop coins to see live counting...");
}

void loop() {
  // Handle serial commands
  handleSerialCommands();
  
  // Real-time coin counting display
  unsigned long currentCoinCount = coinHopper.getTotalCoins();
  
  // Check if there's a new coin detected
  if (currentCoinCount != lastCoinCount) {
    // Send coin event to RPi if in RPi mode and counting is active
    if (rpiMode && countingActive) {
      sendCoinEvent(currentCoinCount);
    }
    
    // Show manual mode display
    if (realTimeMode && !rpiMode) {
      Serial.print("💰 COIN #");
      Serial.print(currentCoinCount);
      Serial.print(" detected! [Rate: ");
      Serial.print(coinHopper.getPulseRate(), 1);
      Serial.print(" coins/sec] [Time: ");
      Serial.print(millis());
      Serial.println("ms]");
    }
    lastCoinCount = currentCoinCount;
  }
  
  // Periodic status summary (less frequent) - only in manual mode
  if (realTimeMode && !rpiMode && millis() - lastSerialOutput >= SERIAL_OUTPUT_INTERVAL) {
    if (currentCoinCount > 0) {
      Serial.print("📊 Total: ");
      Serial.print(currentCoinCount);
      Serial.print(" coins | Rate: ");
      Serial.print(coinHopper.getPulseRate(), 2);
      Serial.print(" coins/sec | Last: ");
      Serial.print(coinHopper.getLastPulseTime());
      Serial.println("ms ago");
      
      if (countingActive) {
        Serial.print("🎯 Session: ");
        Serial.print(currentCoinCount - sessionStartCount);
        Serial.println(" coins");
      }
    }
    lastSerialOutput = millis();
  }
  
  // Update coin hopper
  coinHopper.update();
  
  // Minimal delay for real-time responsiveness
  delay(5);
}

void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    // Handle RPi commands (case-sensitive)
    if (handleRPiCommand(command)) {
      return; // Command was processed by RPi handler
    }
    
    // Convert to lowercase for manual commands
    command.toLowerCase();
    
    if (command == "count") {
      Serial.print("Current coin count: ");
      Serial.println(coinHopper.getTotalCoins());
    }
    else if (command == "reset") {
      coinHopper.resetCounter();
      Serial.println("Coin counter reset to 0");
    }
    else if (command.startsWith("dispense")) {
      int spaceIndex = command.indexOf(' ');
      if (spaceIndex > 0) {
        int coinsToDispense = command.substring(spaceIndex + 1).toInt();
        if (coinsToDispense > 0 && coinsToDispense <= 100) {
          Serial.print("Dispensing ");
          Serial.print(coinsToDispense);
          Serial.println(" coins...");
          
          bool success = coinHopper.dispenseCoins(coinsToDispense);
          if (success) {
            Serial.println("Coins dispensed successfully!");
          } else {
            Serial.println("Failed to dispense coins. Check hopper status.");
          }
        } else {
          Serial.println("Invalid number of coins. Range: 1-100");
        }
      } else {
        Serial.println("Usage: dispense <number>");
      }
    }
    else if (command == "status") {
      Serial.println("=== Coin Hopper Status ===");
      Serial.print("Total coins detected: ");
      Serial.println(coinHopper.getTotalCoins());
      Serial.print("Pulse rate: ");
      Serial.print(coinHopper.getPulseRate());
      Serial.println(" pulses/sec");
      Serial.print("Last pulse time: ");
      Serial.print(coinHopper.getLastPulseTime());
      Serial.println(" ms ago");
      Serial.println("=========================");
    }
    else if (command.startsWith("realtime")) {
      int spaceIndex = command.indexOf(' ');
      if (spaceIndex > 0) {
        String mode = command.substring(spaceIndex + 1);
        if (mode == "on") {
          realTimeMode = true;
          Serial.println("✅ Real-time counting: ENABLED");
          Serial.println("You will see each coin as it's detected!");
        } else if (mode == "off") {
          realTimeMode = false;
          Serial.println("❌ Real-time counting: DISABLED");
          Serial.println("Use 'count' command to check total");
        } else {
          Serial.println("Usage: realtime on/off");
        }
      } else {
        Serial.print("Real-time mode is currently: ");
        Serial.println(realTimeMode ? "ENABLED" : "DISABLED");
      }
    }
    else if (command == "help") {
      Serial.println("=== MANUAL COMMANDS ===");
      Serial.println("  count - Display current coin count");
      Serial.println("  reset - Reset coin counter");
      Serial.println("  dispense X - Dispense X coins (1-100)");
      Serial.println("  realtime on/off - Toggle real-time counting display");
      Serial.println("  status - Show detailed hopper status");
      Serial.println("  help - Show this help message");
      Serial.println("");
      Serial.println("=== RPI COMMANDS (case-sensitive) ===");
      Serial.println("  START_COUNT - Begin counting session");
      Serial.println("  STOP_COUNT - End counting session");
      Serial.println("  GET_COUNT - Get current count data (JSON)");
      Serial.println("  GET_STATUS - Get system status (JSON)");
      Serial.println("  RESET_COUNT - Reset coin counter");
      Serial.println("  SET_RPI_MODE ON/OFF - Enable/disable RPi mode");
      Serial.println("  PING - Test communication");
    }
    else {
      Serial.println("Unknown command. Type 'help' for available commands.");
    }
  }
}

// RPi Communication Functions
bool handleRPiCommand(String command) {
  command.trim();
  
  // Check if it's an RPi command (uppercase format)
  if (command == CMD_START_COUNT) {
    startCounting();
    sendResponse("START_COUNT", "OK", "Counting started");
    return true;
  }
  else if (command == CMD_STOP_COUNT) {
    stopCounting();
    sendResponse("STOP_COUNT", "OK", "Counting stopped");
    return true;
  }
  else if (command == CMD_GET_COUNT) {
    sendCountData();
    return true;
  }
  else if (command == CMD_GET_STATUS) {
    sendStatusData();
    return true;
  }
  else if (command == CMD_RESET_COUNT) {
    coinHopper.resetCounter();
    sessionStartCount = 0;
    sessionStartTime = millis();
    sendResponse("RESET_COUNT", "OK", "Counter reset");
    return true;
  }
  else if (command.startsWith(CMD_SET_RPI_MODE)) {
    String mode = command.substring(CMD_SET_RPI_MODE.length());
    mode.trim();
    if (mode == "ON") {
      rpiMode = true;
      realTimeMode = false; // Disable console output in RPi mode
      sendResponse("SET_RPI_MODE", "OK", "RPi mode enabled");
    } else if (mode == "OFF") {
      rpiMode = false;
      realTimeMode = true; // Re-enable console output
      sendResponse("SET_RPI_MODE", "OK", "RPi mode disabled");
    } else {
      sendResponse("SET_RPI_MODE", "ERROR", "Invalid parameter. Use ON or OFF");
    }
    return true;
  }
  else if (command == CMD_PING) {
    sendResponse("PING", "OK", "ESP32 CoinExchanger ready");
    return true;
  }
  
  return false; // Not an RPi command
}

void startCounting() {
  if (!countingActive) {
    countingActive = true;
    sessionStartCount = coinHopper.getTotalCoins();
    sessionStartTime = millis();
    
    if (!rpiMode) {
      Serial.println("🟢 Counting session STARTED");
    }
  }
}

void stopCounting() {
  if (countingActive) {
    countingActive = false;
    
    if (!rpiMode) {
      unsigned long sessionCoins = coinHopper.getTotalCoins() - sessionStartCount;
      Serial.print("🔴 Counting session STOPPED. Session coins: ");
      Serial.println(sessionCoins);
    }
  }
}

void sendCountData() {
  unsigned long totalCoins = coinHopper.getTotalCoins();
  unsigned long sessionCoins = countingActive ? (totalCoins - sessionStartCount) : 0;
  float pulseRate = coinHopper.getPulseRate();
  unsigned long lastPulse = coinHopper.getLastPulseTime();
  
  Serial.print("{\"command\":\"GET_COUNT\",\"status\":\"OK\",\"data\":{");
  Serial.print("\"total_coins\":");
  Serial.print(totalCoins);
  Serial.print(",\"session_coins\":");
  Serial.print(sessionCoins);
  Serial.print(",\"pulse_rate\":");
  Serial.print(pulseRate, 2);
  Serial.print(",\"last_pulse_ms\":");
  Serial.print(lastPulse);
  Serial.print(",\"counting_active\":");
  Serial.print(countingActive ? "true" : "false");
  Serial.print(",\"timestamp\":");
  Serial.print(millis());
  Serial.println("}}");
}

void sendStatusData() {
  unsigned long uptime = millis();
  bool hopperReady = coinHopper.isReady();
  
  Serial.print("{\"command\":\"GET_STATUS\",\"status\":\"OK\",\"data\":{");
  Serial.print("\"hopper_ready\":");
  Serial.print(hopperReady ? "true" : "false");
  Serial.print(",\"counting_active\":");
  Serial.print(countingActive ? "true" : "false");
  Serial.print(",\"rpi_mode\":");
  Serial.print(rpiMode ? "true" : "false");
  Serial.print(",\"uptime_ms\":");
  Serial.print(uptime);
  Serial.print(",\"session_duration_ms\":");
  Serial.print(countingActive ? (uptime - sessionStartTime) : 0);
  Serial.print(",\"pulse_pin\":");
  Serial.print(COIN_HOPPER_PULSE_PIN);
  Serial.println("}}");
}

void sendResponse(String command, String status, String message) {
  Serial.print("{\"command\":\"");
  Serial.print(command);
  Serial.print("\",\"status\":\"");
  Serial.print(status);
  Serial.print("\",\"message\":\"");
  Serial.print(message);
  Serial.print("\",\"timestamp\":");
  Serial.print(millis());
  Serial.println("}");
}

void sendCoinEvent(unsigned long coinNumber) {
  if (rpiMode && countingActive) {
    Serial.print("{\"event\":\"COIN_DETECTED\",\"data\":{");
    Serial.print("\"coin_number\":");
    Serial.print(coinNumber);
    Serial.print(",\"session_coin\":");
    Serial.print(coinNumber - sessionStartCount);
    Serial.print(",\"timestamp\":");
    Serial.print(millis());
    Serial.print(",\"pulse_rate\":");
    Serial.print(coinHopper.getPulseRate(), 2);
    Serial.println("}}");
  }
}