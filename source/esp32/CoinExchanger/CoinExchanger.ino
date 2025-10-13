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

// Global variables - 3 Coin Hoppers
COIN_HOPPER coinHopper1(0);  // Hopper 1 - GPIO19
COIN_HOPPER coinHopper2(1);  // Hopper 2 - GPIO18
COIN_HOPPER coinHopper3(2);  // Hopper 3 - GPIO4

unsigned long lastSerialOutput = 0;
const unsigned long SERIAL_OUTPUT_INTERVAL = 100; // Output every 100ms for real-time
unsigned long lastCoinCount[NUM_COIN_HOPPERS] = {0, 0, 0};
bool realTimeMode = true;

// RPi Communication variables
bool countingActive = false;
bool rpiMode = false;
unsigned long sessionStartCount[NUM_COIN_HOPPERS] = {0, 0, 0};
unsigned long sessionStartTime = 0;
String inputBuffer = "";

// Simple timing and pulse counting mode variables
bool simpleTimingMode = false;
int simpleCounts[3] = {0, 0, 0};
unsigned long simpleLastInterruptTimes[3] = {0, 0, 0};
bool simpleCountingActive = false;

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
  Serial.println("Initializing 3 ALLAN Coin Hoppers...");
  
  // Initialize all 3 coin hoppers
  bool allInitialized = true;
  allInitialized &= coinHopper1.begin(COIN_HOPPER_1_PULSE_PIN, 0);
  allInitialized &= coinHopper2.begin(COIN_HOPPER_2_PULSE_PIN, 1);
  allInitialized &= coinHopper3.begin(COIN_HOPPER_3_PULSE_PIN, 2);
  
  if (allInitialized) {
    Serial.println("✅ All 3 coin hoppers initialized successfully!");
  } else {
    Serial.println("❌ Error initializing one or more coin hoppers!");
  }
  
  Serial.println("System ready!");
  Serial.println("All coin hoppers are monitoring for pulses...");
  Serial.println("Commands:");
  Serial.println("  Manual: 'count', 'reset', 'dispense X', 'realtime on/off'");
  Serial.println("  RPi: 'START_COUNT', 'STOP_COUNT', 'GET_COUNT', 'GET_STATUS'");
  Serial.println("       'RESET_COUNT', 'SET_RPI_MODE ON/OFF', 'PING'");
  Serial.println("================================");
  Serial.println("Real-time counting: ENABLED");
  Serial.println("RPi Mode: DISABLED (use SET_RPI_MODE ON to enable)");
  Serial.println("Drop coins to see live counting...");
  
  // Initialize simple timing mode interrupts (alternative to COIN_HOPPER class)
  // These will be used when simpleTimingMode is enabled
  // Using the same pins but different interrupt handlers for direct timing control
}

// Simple timing mode interrupt handlers (140ms debounce timing)
void IRAM_ATTR simpleHopper1ISR() {
  if (!simpleTimingMode || !simpleCountingActive) return;
  
  unsigned long currentTime = millis();
  if (currentTime - simpleLastInterruptTimes[0] > 140) {  // 140ms debounce
    simpleCounts[0]++;
    simpleLastInterruptTimes[0] = currentTime;
  }
}

void IRAM_ATTR simpleHopper2ISR() {
  if (!simpleTimingMode || !simpleCountingActive) return;
  
  unsigned long currentTime = millis();
  if (currentTime - simpleLastInterruptTimes[1] > 140) {  // 140ms debounce
    simpleCounts[1]++;
    simpleLastInterruptTimes[1] = currentTime;
  }
}

void IRAM_ATTR simpleHopper3ISR() {
  if (!simpleTimingMode || !simpleCountingActive) return;
  
  unsigned long currentTime = millis();
  if (currentTime - simpleLastInterruptTimes[2] > 140) {  // 140ms debounce
    simpleCounts[2]++;
    simpleLastInterruptTimes[2] = currentTime;
  }
}

void loop() {
  // Handle serial commands
  handleSerialCommands();
  
  // Handle simple timing mode output
  if (simpleTimingMode) {
    handleSimpleTimingMode();
  }
  
  // Check all 3 coin hoppers for new coins (only if not in simple timing mode)
  if (!simpleTimingMode) {
    COIN_HOPPER* hoppers[] = {&coinHopper1, &coinHopper2, &coinHopper3};
  
  for (int i = 0; i < NUM_COIN_HOPPERS; i++) {
    unsigned long currentCoinCount = hoppers[i]->getTotalCoins();
    
    // Check if there's a new coin detected on this hopper
    if (currentCoinCount != lastCoinCount[i]) {
      // Send coin event to RPi if in RPi mode and counting is active
      if (rpiMode && countingActive) {
        sendCoinEvent(i + 1, currentCoinCount);
      }
      
      // Show manual mode display
      if (realTimeMode && !rpiMode) {
        Serial.print("💰 HOPPER ");
        Serial.print(i + 1);
        Serial.print(" - COIN #");
        Serial.print(currentCoinCount);
        Serial.print(" detected! [Rate: ");
        Serial.print(hoppers[i]->getPulseRate(), 1);
        Serial.print(" coins/sec] [GPIO");
        Serial.print(hoppers[i]->getPulsePin());
        Serial.print("] [Time: ");
        Serial.print(millis());
        Serial.println("ms]");
      }
      lastCoinCount[i] = currentCoinCount;
    }
  }
  
  // Periodic status summary (less frequent) - only in manual mode
  if (realTimeMode && !rpiMode && millis() - lastSerialOutput >= SERIAL_OUTPUT_INTERVAL) {
    unsigned long totalAllHoppers = coinHopper1.getTotalCoins() + coinHopper2.getTotalCoins() + coinHopper3.getTotalCoins();
    
    if (totalAllHoppers > 0) {
      Serial.print("📊 Total ALL: ");
      Serial.print(totalAllHoppers);
      Serial.print(" | H1:");
      Serial.print(coinHopper1.getTotalCoins());
      Serial.print(" H2:");
      Serial.print(coinHopper2.getTotalCoins());
      Serial.print(" H3:");
      Serial.print(coinHopper3.getTotalCoins());
      
      if (countingActive) {
        unsigned long sessionTotal = 0;
        for (int i = 0; i < NUM_COIN_HOPPERS; i++) {
          sessionTotal += hoppers[i]->getTotalCoins() - sessionStartCount[i];
        }
        Serial.print(" | 🎯 Session: ");
        Serial.print(sessionTotal);
      }
      Serial.println();
    }
    lastSerialOutput = millis();
  }
  }
  
  // Update all coin hoppers (only if not in simple timing mode)
  if (!simpleTimingMode) {
    coinHopper1.update();
    coinHopper2.update();
    coinHopper3.update();
  }
  
  // Minimal delay for real-time responsiveness
  delay(5);
}

// Simple timing mode handler function
void handleSimpleTimingMode() {
  static unsigned long lastSimpleOutput = 0;
  static int lastSimpleCounts[3] = {0, 0, 0};
  
  // Check for new coins and display immediately
  for (int i = 0; i < 3; i++) {
    if (simpleCounts[i] != lastSimpleCounts[i]) {
      Serial.print("💰 SIMPLE H");
      Serial.print(i + 1);
      Serial.print(" - COIN #");
      Serial.print(simpleCounts[i]);
      Serial.print(" detected! [140ms debounce] [GPIO");
      Serial.print(i == 0 ? COIN_HOPPER_1_PULSE_PIN : (i == 1 ? COIN_HOPPER_2_PULSE_PIN : COIN_HOPPER_3_PULSE_PIN));
      Serial.print("] [Time: ");
      Serial.print(millis());
      Serial.println("ms]");
      lastSimpleCounts[i] = simpleCounts[i];
    }
  }
  
  // Periodic summary every 1 second
  if (millis() - lastSimpleOutput >= 1000) {
    int totalSimple = simpleCounts[0] + simpleCounts[1] + simpleCounts[2];
    if (totalSimple > 0 || simpleCountingActive) {
      Serial.print("📊 SIMPLE MODE | H1:");
      Serial.print(simpleCounts[0]);
      Serial.print(" H2:");
      Serial.print(simpleCounts[1]);
      Serial.print(" H3:");
      Serial.print(simpleCounts[2]);
      Serial.print(" | Total:");
      Serial.print(totalSimple);
      Serial.print(" | Status:");
      Serial.println(simpleCountingActive ? "COUNTING" : "STOPPED");
    }
    lastSimpleOutput = millis();
  }
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
      Serial.println("=== Coin Counts ===");
      Serial.print("Hopper 1 (GPIO19): ");
      Serial.println(coinHopper1.getTotalCoins());
      Serial.print("Hopper 2 (GPIO18): ");
      Serial.println(coinHopper2.getTotalCoins());
      Serial.print("Hopper 3 (GPIO4):  ");
      Serial.println(coinHopper3.getTotalCoins());
      Serial.print("Total: ");
      Serial.println(coinHopper1.getTotalCoins() + coinHopper2.getTotalCoins() + coinHopper3.getTotalCoins());
    }
    else if (command == "reset") {
      coinHopper1.resetCounter();
      coinHopper2.resetCounter();
      coinHopper3.resetCounter();
      for (int i = 0; i < NUM_COIN_HOPPERS; i++) {
        lastCoinCount[i] = 0;
        sessionStartCount[i] = 0;
      }
      Serial.println("All coin counters reset to 0");
    }
    else if (command.startsWith("dispense")) {
      int spaceIndex = command.indexOf(' ');
      if (spaceIndex > 0) {
        int coinsToDispense = command.substring(spaceIndex + 1).toInt();
        if (coinsToDispense > 0 && coinsToDispense <= 100) {
          Serial.print("Dispensing ");
          Serial.print(coinsToDispense);
          Serial.println(" coins from Hopper 1...");
          
          bool success = coinHopper1.dispenseCoins(coinsToDispense);
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
      Serial.println("Hopper 1 (GPIO19):");
      Serial.print("  Total coins: ");
      Serial.println(coinHopper1.getTotalCoins());
      Serial.print("  Pulse rate: ");
      Serial.print(coinHopper1.getPulseRate());
      Serial.println(" pulses/sec");
      Serial.print("  Last pulse: ");
      Serial.print(coinHopper1.getLastPulseTime());
      Serial.println(" ms ago");
      
      Serial.println("Hopper 2 (GPIO18):");
      Serial.print("  Total coins: ");
      Serial.println(coinHopper2.getTotalCoins());
      Serial.print("  Pulse rate: ");
      Serial.print(coinHopper2.getPulseRate());
      Serial.println(" pulses/sec");
      Serial.print("  Last pulse: ");
      Serial.print(coinHopper2.getLastPulseTime());
      Serial.println(" ms ago");
      
      Serial.println("Hopper 3 (GPIO4):");
      Serial.print("  Total coins: ");
      Serial.println(coinHopper3.getTotalCoins());
      Serial.print("  Pulse rate: ");
      Serial.print(coinHopper3.getPulseRate());
      Serial.println(" pulses/sec");
      Serial.print("  Last pulse: ");
      Serial.print(coinHopper3.getLastPulseTime());
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
    else if (command == "simple") {
      if (!simpleTimingMode) {
        // Switch to simple timing mode
        simpleTimingMode = true;
        
        // Detach existing interrupts and attach simple ones
        detachInterrupt(digitalPinToInterrupt(COIN_HOPPER_1_PULSE_PIN));
        detachInterrupt(digitalPinToInterrupt(COIN_HOPPER_2_PULSE_PIN));
        detachInterrupt(digitalPinToInterrupt(COIN_HOPPER_3_PULSE_PIN));
        
        attachInterrupt(digitalPinToInterrupt(COIN_HOPPER_1_PULSE_PIN), simpleHopper1ISR, FALLING);
        attachInterrupt(digitalPinToInterrupt(COIN_HOPPER_2_PULSE_PIN), simpleHopper2ISR, FALLING);
        attachInterrupt(digitalPinToInterrupt(COIN_HOPPER_3_PULSE_PIN), simpleHopper3ISR, FALLING);
        
        // Reset simple counters
        for (int i = 0; i < 3; i++) {
          simpleCounts[i] = 0;
          simpleLastInterruptTimes[i] = 0;
        }
        
        Serial.println("✅ SIMPLE TIMING MODE ENABLED");
        Serial.println("140ms debounce timing active");
        Serial.println("Commands: 'simplestart', 'simplestop', 'simplecount', 'simplereset', 'normal'");
      } else {
        Serial.println("Already in simple timing mode");
      }
    }
    else if (command == "normal") {
      if (simpleTimingMode) {
        // Switch back to normal mode
        simpleTimingMode = false;
        simpleCountingActive = false;
        
        // Detach simple interrupts
        detachInterrupt(digitalPinToInterrupt(COIN_HOPPER_1_PULSE_PIN));
        detachInterrupt(digitalPinToInterrupt(COIN_HOPPER_2_PULSE_PIN));
        detachInterrupt(digitalPinToInterrupt(COIN_HOPPER_3_PULSE_PIN));
        
        // Reinitialize coin hoppers (this will reattach their interrupts)
        coinHopper1.begin(COIN_HOPPER_1_PULSE_PIN, 0);
        coinHopper2.begin(COIN_HOPPER_2_PULSE_PIN, 1);
        coinHopper3.begin(COIN_HOPPER_3_PULSE_PIN, 2);
        
        Serial.println("✅ NORMAL MODE RESTORED");
        Serial.println("Advanced COIN_HOPPER class active");
      } else {
        Serial.println("Already in normal mode");
      }
    }
    else if (command == "simplestart") {
      if (simpleTimingMode) {
        simpleCountingActive = true;
        // Reset counters when starting
        for (int i = 0; i < 3; i++) {
          simpleCounts[i] = 0;
          simpleLastInterruptTimes[i] = 0;
        }
        Serial.println("🎯 Simple counting STARTED - Counters reset");
      } else {
        Serial.println("Not in simple timing mode. Use 'simple' command first.");
      }
    }
    else if (command == "simplestop") {
      if (simpleTimingMode) {
        simpleCountingActive = false;
        Serial.println("🛑 Simple counting STOPPED");
        Serial.print("Final counts -> H1:");
        Serial.print(simpleCounts[0]);
        Serial.print(" H2:");
        Serial.print(simpleCounts[1]);
        Serial.print(" H3:");
        Serial.print(simpleCounts[2]);
        Serial.print(" Total:");
        Serial.println(simpleCounts[0] + simpleCounts[1] + simpleCounts[2]);
      } else {
        Serial.println("Not in simple timing mode. Use 'simple' command first.");
      }
    }
    else if (command == "simplecount") {
      if (simpleTimingMode) {
        Serial.println("=== Simple Mode Counts ===");
        Serial.print("Hopper 1 (GPIO19): ");
        Serial.println(simpleCounts[0]);
        Serial.print("Hopper 2 (GPIO18): ");
        Serial.println(simpleCounts[1]);
        Serial.print("Hopper 3 (GPIO4):  ");
        Serial.println(simpleCounts[2]);
        Serial.print("Total: ");
        Serial.println(simpleCounts[0] + simpleCounts[1] + simpleCounts[2]);
        Serial.print("Status: ");
        Serial.println(simpleCountingActive ? "COUNTING" : "STOPPED");
        Serial.println("Debounce: 140ms per hopper");
      } else {
        Serial.println("Not in simple timing mode. Use 'simple' command first.");
      }
    }
    else if (command == "simplereset") {
      if (simpleTimingMode) {
        for (int i = 0; i < 3; i++) {
          simpleCounts[i] = 0;
          simpleLastInterruptTimes[i] = 0;
        }
        Serial.println("Simple mode counters reset to 0");
      } else {
        Serial.println("Not in simple timing mode. Use 'simple' command first.");
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
      Serial.println("=== SIMPLE TIMING MODE (140ms debounce) ===");
      Serial.println("  simple - Switch to simple timing mode");
      Serial.println("  normal - Switch back to normal mode");
      Serial.println("  simplestart - Start simple counting (resets counters)");
      Serial.println("  simplestop - Stop simple counting");
      Serial.println("  simplecount - Display simple mode counts");
      Serial.println("  simplereset - Reset simple mode counters");
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
    coinHopper1.resetCounter();
    coinHopper2.resetCounter();
    coinHopper3.resetCounter();
    for (int i = 0; i < NUM_COIN_HOPPERS; i++) {
      sessionStartCount[i] = 0;
      lastCoinCount[i] = 0;
    }
    sessionStartTime = millis();
    sendResponse("RESET_COUNT", "OK", "All counters reset");
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
    sessionStartCount[0] = coinHopper1.getTotalCoins();
    sessionStartCount[1] = coinHopper2.getTotalCoins();
    sessionStartCount[2] = coinHopper3.getTotalCoins();
    sessionStartTime = millis();
    
    if (!rpiMode) {
      Serial.println("🟢 Counting session STARTED for all 3 hoppers");
    }
  }
}

void stopCounting() {
  if (countingActive) {
    countingActive = false;
    
    if (!rpiMode) {
      unsigned long sessionCoins1 = coinHopper1.getTotalCoins() - sessionStartCount[0];
      unsigned long sessionCoins2 = coinHopper2.getTotalCoins() - sessionStartCount[1];
      unsigned long sessionCoins3 = coinHopper3.getTotalCoins() - sessionStartCount[2];
      unsigned long totalSessionCoins = sessionCoins1 + sessionCoins2 + sessionCoins3;
      
      Serial.print("🔴 Counting session STOPPED. Session coins - H1:");
      Serial.print(sessionCoins1);
      Serial.print(" H2:");
      Serial.print(sessionCoins2);
      Serial.print(" H3:");
      Serial.print(sessionCoins3);
      Serial.print(" Total:");
      Serial.println(totalSessionCoins);
    }
  }
}

void sendCountData() {
  COIN_HOPPER* hoppers[] = {&coinHopper1, &coinHopper2, &coinHopper3};
  
  unsigned long totalCoins1 = coinHopper1.getTotalCoins();
  unsigned long totalCoins2 = coinHopper2.getTotalCoins();
  unsigned long totalCoins3 = coinHopper3.getTotalCoins();
  unsigned long totalAllCoins = totalCoins1 + totalCoins2 + totalCoins3;
  
  unsigned long sessionCoins1 = countingActive ? (totalCoins1 - sessionStartCount[0]) : 0;
  unsigned long sessionCoins2 = countingActive ? (totalCoins2 - sessionStartCount[1]) : 0;
  unsigned long sessionCoins3 = countingActive ? (totalCoins3 - sessionStartCount[2]) : 0;
  unsigned long sessionTotal = sessionCoins1 + sessionCoins2 + sessionCoins3;
  
  Serial.print("{\"command\":\"GET_COUNT\",\"status\":\"OK\",\"data\":{");
  Serial.print("\"total_coins_all\":");
  Serial.print(totalAllCoins);
  Serial.print(",\"session_coins_all\":");
  Serial.print(sessionTotal);
  Serial.print(",\"hoppers\":[");
  
  for (int i = 0; i < NUM_COIN_HOPPERS; i++) {
    if (i > 0) Serial.print(",");
    Serial.print("{\"id\":");
    Serial.print(i + 1);
    Serial.print(",\"gpio\":");
    Serial.print(hoppers[i]->getPulsePin());
    Serial.print(",\"total_coins\":");
    Serial.print(hoppers[i]->getTotalCoins());
    Serial.print(",\"session_coins\":");
    Serial.print(countingActive ? (hoppers[i]->getTotalCoins() - sessionStartCount[i]) : 0);
    Serial.print(",\"pulse_rate\":");
    Serial.print(hoppers[i]->getPulseRate(), 2);
    Serial.print(",\"last_pulse_ms\":");
    Serial.print(hoppers[i]->getLastPulseTime());
    Serial.print("}");
  }
  
  Serial.print("],\"counting_active\":");
  Serial.print(countingActive ? "true" : "false");
  Serial.print(",\"timestamp\":");
  Serial.print(millis());
  Serial.println("}}");
}

void sendStatusData() {
  unsigned long uptime = millis();
  COIN_HOPPER* hoppers[] = {&coinHopper1, &coinHopper2, &coinHopper3};
  
  Serial.print("{\"command\":\"GET_STATUS\",\"status\":\"OK\",\"data\":{");
  Serial.print("\"num_hoppers\":");
  Serial.print(NUM_COIN_HOPPERS);
  Serial.print(",\"counting_active\":");
  Serial.print(countingActive ? "true" : "false");
  Serial.print(",\"rpi_mode\":");
  Serial.print(rpiMode ? "true" : "false");
  Serial.print(",\"uptime_ms\":");
  Serial.print(uptime);
  Serial.print(",\"session_duration_ms\":");
  Serial.print(countingActive ? (uptime - sessionStartTime) : 0);
  Serial.print(",\"hoppers\":[");
  
  for (int i = 0; i < NUM_COIN_HOPPERS; i++) {
    if (i > 0) Serial.print(",");
    Serial.print("{\"id\":");
    Serial.print(i + 1);
    Serial.print(",\"gpio\":");
    Serial.print(hoppers[i]->getPulsePin());
    Serial.print(",\"ready\":");
    Serial.print(hoppers[i]->isReady() ? "true" : "false");
    Serial.print(",\"dispensing\":");
    Serial.print(hoppers[i]->isCurrentlyDispensing() ? "true" : "false");
    Serial.print("}");
  }
  
  Serial.println("]}}");
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

void sendCoinEvent(int hopperId, unsigned long coinNumber) {
  if (rpiMode && countingActive) {
    COIN_HOPPER* hoppers[] = {&coinHopper1, &coinHopper2, &coinHopper3};
    int hopperIndex = hopperId - 1; // Convert to 0-based index
    
    Serial.print("{\"event\":\"COIN_DETECTED\",\"data\":{");
    Serial.print("\"hopper_id\":");
    Serial.print(hopperId);
    Serial.print(",\"gpio\":");
    Serial.print(hoppers[hopperIndex]->getPulsePin());
    Serial.print(",\"coin_number\":");
    Serial.print(coinNumber);
    Serial.print(",\"session_coin\":");
    Serial.print(coinNumber - sessionStartCount[hopperIndex]);
    Serial.print(",\"timestamp\":");
    Serial.print(millis());
    Serial.print(",\"pulse_rate\":");
    Serial.print(hoppers[hopperIndex]->getPulseRate(), 2);
    Serial.println("}}");
  }
}