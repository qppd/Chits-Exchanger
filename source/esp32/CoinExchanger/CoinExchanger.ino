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
#include "SOLID_STATE_RELAY.h"
#include "COIN_HOPPER.h"

// Global variables - 3 Coin Hoppers
COIN_HOPPER coinHopper1(0);  // Hopper 1 - GPIO19
COIN_HOPPER coinHopper2(1);  // Hopper 2 - GPIO18
COIN_HOPPER coinHopper3(2);  // Hopper 3 - GPIO4

// System variables (simplified)
unsigned long lastCoinCount[NUM_COIN_HOPPERS] = {0, 0, 0};

// Essential RPi communication commands
const String CMD_DISPENSE_COINS = "DISPENSE_COINS";
const String CMD_GET_COUNT = "GET_COUNT";
const String CMD_GET_STATUS = "GET_STATUS";
const String CMD_RESET_COUNT = "RESET_COUNT";

// Testing commands for Serial monitor
const String CMD_TEST_PULSE = "test_pulse";       // test_pulse 1/2/3 - Test pulse detection from coin hopper
const String CMD_TEST_RELAY = "test_relay";       // test_relay 1/2/3 on/off - Test SSR relay control
const String CMD_TEST_ALL = "test_all";           // Test all hoppers and relays
const String CMD_HELP = "help";                   // Show help menu

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // Wait for serial to initialize
  }
  
  Serial.println("=== ESP32 Coin Exchanger System ===");
  Serial.println("Initializing 3 ALLAN Coin Hoppers...");
  
  // Initialize all 3 coin hoppers with SSR control
  bool allInitialized = true;
  allInitialized &= coinHopper1.begin(COIN_HOPPER_1_PULSE_PIN, 0, COIN_HOPPER_1_SSR_PIN);
  allInitialized &= coinHopper2.begin(COIN_HOPPER_2_PULSE_PIN, 1, COIN_HOPPER_2_SSR_PIN);
  allInitialized &= coinHopper3.begin(COIN_HOPPER_3_PULSE_PIN, 2, COIN_HOPPER_3_SSR_PIN);
  
  if (allInitialized) {
    Serial.println("✅ All 3 coin hoppers initialized successfully!");
  } else {
    Serial.println("❌ Error initializing one or more coin hoppers!");
  }
  
  Serial.println("System ready!");
  Serial.println("=== SIMPLIFIED COIN EXCHANGER ===");
  Serial.println("💡 Type 'help' for testing commands");
  Serial.println();
  Serial.println("🧪 Quick Tests:");
  Serial.println("  test_pulse 1/2/3  - Test pulse detection");
  Serial.println("  test_relay 1/2/3 on/off - Test SSR relays");
  Serial.println("  test_all          - Test everything");
  Serial.println();
  Serial.println("📡 RPi Commands:");
  Serial.println("  DISPENSE_COINS, GET_COUNT, GET_STATUS, RESET_COUNT");
  Serial.println();
  Serial.println("Coin values: 5PHP(H1), 10PHP(H2), 20PHP(H3)");
  Serial.println("==================================");
  // These will be used when simpleTimingMode is enabled
  // Using the same pins but different interrupt handlers for direct timing control
}



void loop() {
  // Handle serial commands from RPi
  handleSerialCommands();
  
  // Check all 3 coin hoppers for new coins
  COIN_HOPPER* hoppers[] = {&coinHopper1, &coinHopper2, &coinHopper3};
  
  for (int i = 0; i < NUM_COIN_HOPPERS; i++) {
    unsigned long currentCoinCount = hoppers[i]->getTotalCoins();
    
    // Check if there's a new coin detected on this hopper
    if (currentCoinCount != lastCoinCount[i]) {
      // Send coin event to RPi
      sendCoinEvent(i + 1, currentCoinCount);
      lastCoinCount[i] = currentCoinCount;
    }
  }
  
  // Update all coin hoppers
  coinHopper1.update();
  coinHopper2.update();
  coinHopper3.update();
  
  // Minimal delay for real-time responsiveness
  delay(5);
}



void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    // Check for test commands first (case-insensitive)
    String lowerCommand = command;
    lowerCommand.toLowerCase();
    
    if (handleTestCommand(lowerCommand)) {
      return; // Test command was processed
    }
    
    // Handle RPi commands (essential commands only)
    handleRPiCommand(command);
  }
}

// Test Command Handler
bool handleTestCommand(String command) {
  if (command.startsWith("test_pulse")) {
    int spaceIndex = command.indexOf(' ');
    if (spaceIndex > 0) {
      int hopperId = command.substring(spaceIndex + 1).toInt();
      testPulseDetection(hopperId);
    } else {
      Serial.println("Usage: test_pulse 1/2/3");
      Serial.println("Example: test_pulse 1 (tests hopper 1 pulse detection)");
    }
    return true;
  }
  else if (command.startsWith("test_relay")) {
    // Parse "test_relay 1 on" or "test_relay 2 off"
    int firstSpace = command.indexOf(' ');
    int secondSpace = command.indexOf(' ', firstSpace + 1);
    
    if (firstSpace > 0 && secondSpace > 0) {
      int hopperId = command.substring(firstSpace + 1, secondSpace).toInt();
      String state = command.substring(secondSpace + 1);
      testRelayControl(hopperId, state);
    } else {
      Serial.println("Usage: test_relay 1/2/3 on/off");
      Serial.println("Example: test_relay 1 on (turns on hopper 1 relay)");
    }
    return true;
  }
  else if (command == "test_all") {
    testAllComponents();
    return true;
  }
  else if (command == "help") {
    showTestHelp();
    return true;
  }
  
  return false; // Not a test command
}

// RPi Communication Functions
void handleRPiCommand(String command) {
  command.trim();
  
  // Only essential RPi commands
  if (command == CMD_DISPENSE_COINS) {
    String coinSpec = command.substring(CMD_DISPENSE_COINS.length());
    coinSpec.trim();
    
    bool success = dispenseSpecificCoins(coinSpec);
    if (success) {
      sendResponse("DISPENSE_COINS", "OK", "Coins dispensed successfully");
    } else {
      sendResponse("DISPENSE_COINS", "ERROR", "Failed to dispense specified coins");
    }
  }
  else if (command == CMD_GET_COUNT) {
    sendCountData();
  }
  else if (command == CMD_GET_STATUS) {
    sendStatusData();
  }
  else if (command == CMD_RESET_COUNT) {
    coinHopper1.resetCounter();
    coinHopper2.resetCounter();
    coinHopper3.resetCounter();
    for (int i = 0; i < NUM_COIN_HOPPERS; i++) {
      lastCoinCount[i] = 0;
    }
    sendResponse("RESET_COUNT", "OK", "All counters reset");
  }
}



void sendCountData() {
  COIN_HOPPER* hoppers[] = {&coinHopper1, &coinHopper2, &coinHopper3};
  
  unsigned long totalCoins1 = coinHopper1.getTotalCoins();
  unsigned long totalCoins2 = coinHopper2.getTotalCoins();
  unsigned long totalCoins3 = coinHopper3.getTotalCoins();
  unsigned long totalAllCoins = totalCoins1 + totalCoins2 + totalCoins3;
  
  Serial.print("{\"command\":\"GET_COUNT\",\"status\":\"OK\",\"data\":{");
  Serial.print("\"total_coins_all\":");
  Serial.print(totalAllCoins);
  Serial.print(",\"hoppers\":[");
  
  for (int i = 0; i < NUM_COIN_HOPPERS; i++) {
    if (i > 0) Serial.print(",");
    Serial.print("{\"id\":");
    Serial.print(i + 1);
    Serial.print(",\"gpio\":");
    Serial.print(hoppers[i]->getPulsePin());
    Serial.print(",\"total_coins\":");
    Serial.print(hoppers[i]->getTotalCoins());
    Serial.print(",\"pulse_rate\":");
    Serial.print(hoppers[i]->getPulseRate(), 2);
    Serial.print(",\"last_pulse_ms\":");
    Serial.print(hoppers[i]->getLastPulseTime());
    Serial.print(",\"coin_value\":");
    Serial.print(hoppers[i]->getCoinValue());
    Serial.print("}");
  }
  
  Serial.print("],\"timestamp\":");
  Serial.print(millis());
  Serial.println("}}");
}

void sendStatusData() {
  unsigned long uptime = millis();
  COIN_HOPPER* hoppers[] = {&coinHopper1, &coinHopper2, &coinHopper3};
  
  Serial.print("{\"command\":\"GET_STATUS\",\"status\":\"OK\",\"data\":{");
  Serial.print("\"num_hoppers\":");
  Serial.print(NUM_COIN_HOPPERS);
  Serial.print(",\"uptime_ms\":");
  Serial.print(uptime);
  Serial.print(",\"hoppers\":[");
  
  for (int i = 0; i < NUM_COIN_HOPPERS; i++) {
    if (i > 0) Serial.print(",");
    Serial.print("{\"id\":");
    Serial.print(i + 1);
    Serial.print(",\"pulse_gpio\":");
    Serial.print(hoppers[i]->getPulsePin());
    Serial.print(",\"ssr_gpio\":");
    Serial.print(hoppers[i]->getSSRPin());
    Serial.print(",\"ssr_state\":");
    Serial.print(hoppers[i]->getSSRState() ? "true" : "false");
    Serial.print(",\"ready\":");
    Serial.print(hoppers[i]->isReady() ? "true" : "false");
    Serial.print(",\"dispensing\":");
    Serial.print(hoppers[i]->isCurrentlyDispensing() ? "true" : "false");
    Serial.print(",\"coin_value\":");
    Serial.print(hoppers[i]->getCoinValue());
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
  COIN_HOPPER* hoppers[] = {&coinHopper1, &coinHopper2, &coinHopper3};
  int hopperIndex = hopperId - 1; // Convert to 0-based index
  
  Serial.print("{\"event\":\"COIN_DETECTED\",\"data\":{");
  Serial.print("\"hopper_id\":");
  Serial.print(hopperId);
  Serial.print(",\"gpio\":");
  Serial.print(hoppers[hopperIndex]->getPulsePin());
  Serial.print(",\"coin_number\":");
  Serial.print(coinNumber);
  Serial.print(",\"timestamp\":");
  Serial.print(millis());
  Serial.print(",\"pulse_rate\":");
  Serial.print(hoppers[hopperIndex]->getPulseRate(), 2);
  Serial.println("}}");
}





// Dispense specific coins based on denomination:quantity pairs
bool dispenseSpecificCoins(String coinSpec) {
  Serial.print("🪙 Dispensing specific coins: ");
  Serial.println(coinSpec);
  
  // Parse format: "5:3,10:2,20:1" (denomination:quantity pairs)
  // This means: 3 pieces of 5PHP, 2 pieces of 10PHP, 1 piece of 20PHP
  
  // Arrays to store parsed data
  int denominations[3] = {0, 0, 0}; // For 5, 10, 20 PHP
  int quantities[3] = {0, 0, 0};    // Quantities for each denomination
  
  // Parse the coin specification
  if (!parseCoinSpecification(coinSpec, denominations, quantities)) {
    Serial.println("❌ Invalid coin specification format");
    return false;
  }
  
  // Validate denominations match our hoppers
  if (!validateDenominations(denominations)) {
    Serial.println("❌ Invalid denominations. Available: 5, 10, 20 PHP");
    return false;
  }
  
  // Calculate total amount and display dispensing plan
  int totalAmount = 0;
  Serial.println("💰 Dispensing plan:");
  for (int i = 0; i < 3; i++) {
    if (quantities[i] > 0) {
      int denomination = (i == 0) ? 5 : (i == 1) ? 10 : 20;
      int amount = quantities[i] * denomination;
      totalAmount += amount;
      
      Serial.print("  ");
      Serial.print(quantities[i]);
      Serial.print(" x ");
      Serial.print(denomination);
      Serial.print(" PHP = ");
      Serial.print(amount);
      Serial.println(" PHP");
    }
  }
  Serial.print("  Total: ");
  Serial.print(totalAmount);
  Serial.println(" PHP");
  
  // Dispense from specific hoppers
  bool allSuccess = true;
  int totalDispensed = 0;
  
  // Map to hoppers: 0=5PHP(hopper1), 1=10PHP(hopper2), 2=20PHP(hopper3)
  COIN_HOPPER* hoppers[] = {&coinHopper1, &coinHopper2, &coinHopper3};
  
  for (int i = 0; i < 3; i++) {
    if (quantities[i] > 0) {
      int denomination = (i == 0) ? 5 : (i == 1) ? 10 : 20;
      Serial.print("🔄 Dispensing ");
      Serial.print(quantities[i]);
      Serial.print(" coins of ");
      Serial.print(denomination);
      Serial.print(" PHP from Hopper ");
      Serial.println(i + 1);
      
      bool success = hoppers[i]->dispenseCoins(quantities[i]);
      
      if (success) {
        int amountFromThisHopper = quantities[i] * denomination;
        totalDispensed += amountFromThisHopper;
        Serial.print("✅ Successfully dispensed ");
        Serial.print(quantities[i]);
        Serial.print(" x ");
        Serial.print(denomination);
        Serial.print(" PHP = ");
        Serial.print(amountFromThisHopper);
        Serial.println(" PHP");
      } else {
        Serial.print("❌ Failed to dispense ");
        Serial.print(quantities[i]);
        Serial.print(" x ");
        Serial.print(denomination);
        Serial.println(" PHP");
        allSuccess = false;
      }
      
      // Small delay between hoppers
      delay(500);
    }
  }
  
  Serial.print("🎯 Final result: ");
  Serial.print(totalDispensed);
  Serial.print(" PHP dispensed of ");
  Serial.print(totalAmount);
  Serial.println(" PHP requested");
  
  if (allSuccess && totalDispensed == totalAmount) {
    Serial.println("✅ Specific coin dispensing completed successfully!");
    return true;
  } else {
    Serial.println("❌ Specific coin dispensing failed or incomplete!");
    return false;
  }
}

// Parse coin specification string (format: "5:3,10:2,20:1")
bool parseCoinSpecification(String spec, int* denominations, int* quantities) {
  // Initialize arrays
  for (int i = 0; i < 3; i++) {
    denominations[i] = 0;
    quantities[i] = 0;
  }
  
  // Split by comma
  int startIndex = 0;
  int pairCount = 0;
  
  while (startIndex < spec.length() && pairCount < 3) {
    int commaIndex = spec.indexOf(',', startIndex);
    if (commaIndex == -1) {
      commaIndex = spec.length(); // Last pair
    }
    
    String pair = spec.substring(startIndex, commaIndex);
    pair.trim();
    
    // Parse "denomination:quantity"
    int colonIndex = pair.indexOf(':');
    if (colonIndex == -1) {
      return false; // Invalid format
    }
    
    String denomStr = pair.substring(0, colonIndex);
    String quantStr = pair.substring(colonIndex + 1);
    denomStr.trim();
    quantStr.trim();
    
    int denomination = denomStr.toInt();
    int quantity = quantStr.toInt();
    
    if (quantity <= 0) {
      return false; // Invalid quantity
    }
    
    // Map denomination to array index
    int index = -1;
    if (denomination == 5) index = 0;
    else if (denomination == 10) index = 1;
    else if (denomination == 20) index = 2;
    
    if (index == -1) {
      return false; // Invalid denomination
    }
    
    denominations[index] = denomination;
    quantities[index] = quantity;
    pairCount++;
    
    startIndex = commaIndex + 1;
  }
  
  return true;
}

// Validate that denominations match our available hoppers
bool validateDenominations(int* denominations) {
  for (int i = 0; i < 3; i++) {
    if (denominations[i] != 0) {
      int expected = (i == 0) ? 5 : (i == 1) ? 10 : 20;
      if (denominations[i] != expected) {
        return false;
      }
    }
  }
  return true;
}

// ========== TESTING FUNCTIONS ==========

void testPulseDetection(int hopperId) {
  if (hopperId < 1 || hopperId > 3) {
    Serial.println("❌ Invalid hopper ID. Use 1, 2, or 3");
    return;
  }
  
  COIN_HOPPER* hoppers[] = {&coinHopper1, &coinHopper2, &coinHopper3};
  COIN_HOPPER* hopper = hoppers[hopperId - 1];
  
  Serial.print("🔍 Testing pulse detection for Hopper ");
  Serial.print(hopperId);
  Serial.print(" (GPIO");
  Serial.print(hopper->getPulsePin());
  Serial.print(", ");
  Serial.print(hopper->getCoinValue());
  Serial.println(" PHP coins)");
  
  unsigned long startCount = hopper->getTotalCoins();
  unsigned long testStartTime = millis();
  
  Serial.println("Drop coins into the hopper... (Test duration: 10 seconds)");
  Serial.println("Watching for pulses...");
  
  while (millis() - testStartTime < 10000) { // 10 second test
    hopper->update();
    unsigned long currentCount = hopper->getTotalCoins();
    
    if (currentCount != startCount) {
      Serial.print("✅ PULSE detected! Coin #");
      Serial.print(currentCount);
      Serial.print(" | Rate: ");
      Serial.print(hopper->getPulseRate(), 2);
      Serial.print(" coins/sec | Last pulse: ");
      Serial.print(hopper->getLastPulseTime());
      Serial.println("ms ago");
      startCount = currentCount;
    }
    delay(50);
  }
  
  unsigned long finalCount = hopper->getTotalCoins();
  unsigned long coinsDetected = finalCount - startCount;
  
  Serial.println("📊 Test Results:");
  Serial.print("  Total coins detected: ");
  Serial.println(coinsDetected);
  Serial.print("  Final count: ");
  Serial.println(finalCount);
  Serial.print("  Average rate: ");
  Serial.print(hopper->getPulseRate(), 2);
  Serial.println(" coins/sec");
}

void testRelayControl(int hopperId, String state) {
  if (hopperId < 1 || hopperId > 3) {
    Serial.println("❌ Invalid hopper ID. Use 1, 2, or 3");
    return;
  }
  
  if (state != "on" && state != "off") {
    Serial.println("❌ Invalid state. Use 'on' or 'off'");
    return;
  }
  
  COIN_HOPPER* hoppers[] = {&coinHopper1, &coinHopper2, &coinHopper3};
  COIN_HOPPER* hopper = hoppers[hopperId - 1];
  
  Serial.print("🔌 Testing SSR relay for Hopper ");
  Serial.print(hopperId);
  Serial.print(" (GPIO");
  Serial.print(hopper->getSSRPin());
  Serial.print(") - Turning ");
  Serial.println(state);
  
  if (state == "on") {
    hopper->enableSSR();
    Serial.println("✅ SSR turned ON - Hopper powered");
  } else {
    hopper->disableSSR();
    Serial.println("✅ SSR turned OFF - Hopper unpowered");
  }
  
  Serial.print("Current SSR state: ");
  Serial.println(hopper->getSSRState() ? "ON" : "OFF");
  
  // Show voltage reading if available
  Serial.println("💡 Check if hopper LED/motor is running to verify SSR operation");
}

void testAllComponents() {
  Serial.println("🧪 COMPREHENSIVE HARDWARE TEST");
  Serial.println("==============================");
  
  // Test all relays first (turn them ON)
  Serial.println("\n1. Testing all SSR relays (turning ON):");
  for (int i = 1; i <= 3; i++) {
    testRelayControl(i, "on");
    delay(1000);
  }
  
  delay(2000);
  
  // Test pulse detection for all hoppers
  Serial.println("\n2. Testing pulse detection (drop coins in any hopper):");
  Serial.println("Monitoring all 3 hoppers for 15 seconds...");
  
  COIN_HOPPER* hoppers[] = {&coinHopper1, &coinHopper2, &coinHopper3};
  unsigned long startCounts[3];
  
  for (int i = 0; i < 3; i++) {
    startCounts[i] = hoppers[i]->getTotalCoins();
  }
  
  unsigned long testStart = millis();
  while (millis() - testStart < 15000) { // 15 second test
    for (int i = 0; i < 3; i++) {
      hoppers[i]->update();
      unsigned long currentCount = hoppers[i]->getTotalCoins();
      
      if (currentCount != startCounts[i]) {
        Serial.print("✅ Hopper ");
        Serial.print(i + 1);
        Serial.print(" pulse detected! Count: ");
        Serial.print(currentCount);
        Serial.print(" | GPIO");
        Serial.println(hoppers[i]->getPulsePin());
        startCounts[i] = currentCount;
      }
    }
    delay(50);
  }
  
  // Turn off all relays
  Serial.println("\n3. Turning OFF all SSR relays:");
  for (int i = 1; i <= 3; i++) {
    testRelayControl(i, "off");
    delay(500);
  }
  
  Serial.println("\n📊 Final Test Results:");
  for (int i = 0; i < 3; i++) {
    Serial.print("  Hopper ");
    Serial.print(i + 1);
    Serial.print(" - GPIO");
    Serial.print(hoppers[i]->getPulsePin());
    Serial.print(" - Count: ");
    Serial.print(hoppers[i]->getTotalCoins());
    Serial.print(" - SSR: GPIO");
    Serial.print(hoppers[i]->getSSRPin());
    Serial.print(" (");
    Serial.print(hoppers[i]->getSSRState() ? "ON" : "OFF");
    Serial.println(")");
  }
  Serial.println("✅ Comprehensive test completed!");
}

void showTestHelp() {
  Serial.println("=== COIN HOPPER TESTING COMMANDS ===");
  Serial.println();
  Serial.println("🔍 PULSE DETECTION TESTS:");
  Serial.println("  test_pulse 1    - Test hopper 1 pulse detection (GPIO19, 5 PHP)");
  Serial.println("  test_pulse 2    - Test hopper 2 pulse detection (GPIO18, 10 PHP)");
  Serial.println("  test_pulse 3    - Test hopper 3 pulse detection (GPIO4, 20 PHP)");
  Serial.println();
  Serial.println("🔌 SSR RELAY TESTS:");
  Serial.println("  test_relay 1 on  - Turn ON hopper 1 SSR (GPIO26)");
  Serial.println("  test_relay 1 off - Turn OFF hopper 1 SSR (GPIO26)");
  Serial.println("  test_relay 2 on  - Turn ON hopper 2 SSR (GPIO25)");
  Serial.println("  test_relay 2 off - Turn OFF hopper 2 SSR (GPIO25)");
  Serial.println("  test_relay 3 on  - Turn ON hopper 3 SSR (GPIO33)");
  Serial.println("  test_relay 3 off - Turn OFF hopper 3 SSR (GPIO33)");
  Serial.println();
  Serial.println("🧪 COMPREHENSIVE TESTS:");
  Serial.println("  test_all        - Test all hoppers and relays (15 sec)");
  Serial.println("  help            - Show this help menu");
  Serial.println();
  Serial.println("📋 RPi COMMANDS (for production):");
  Serial.println("  DISPENSE_COINS 5:X,10:Y,20:Z - Dispense specific coins");
  Serial.println("  GET_COUNT       - Get current coin counts");
  Serial.println("  GET_STATUS      - Get system status");
  Serial.println("  RESET_COUNT     - Reset all counters");
  Serial.println();
  Serial.println("💡 Hardware Setup:");
  Serial.println("  - Connect ALLAN coin hoppers to GPIO19, GPIO18, GPIO4");
  Serial.println("  - Connect SSR relays to GPIO26, GPIO25, GPIO33");
  Serial.println("  - Hopper values: 5PHP(H1), 10PHP(H2), 20PHP(H3)");
  Serial.println();
  Serial.println("🔧 Architecture:");
  Serial.println("  - SOLID_STATE_RELAY class handles SSR control");
  Serial.println("  - COIN_HOPPER class handles pulse detection & dispensing");
  Serial.println("  - Modular design following SOLID principles");
  Serial.println("=====================================");
}