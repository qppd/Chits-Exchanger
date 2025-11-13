/*
 * CoinExchanger.ino
 * ESP32 Coin Exchanger with ALLAN Coin Hopper
 * 
 * Integrated system with RPi for chit detection and coin dispensing
 * 
 * Workflow:
 * 1. RPi detects chit using IR sensor and YOLO (5, 10, 20, 50 peso)
 * 2. RPi sends CHIT_DETECTED command to ESP32
 * 3. ESP32 displays denomination selection UI on LCD
 * 4. User selects desired denominations via button
 * 5. ESP32 calculates optimal coin combination
 * 6. ESP32 dispenses coins using 3 hoppers (5, 10, 20 peso)
 * 7. ESP32 counts coins via pulse detection
 * 8. ESP32 displays progress on LCD
 * 
 * Hardware Connections:
 * - ALLAN Coin Hoppers:
 *   Hopper 1 (5 PHP): GPIO19 (pulse), GPIO26 (SSR)
 *   Hopper 2 (10 PHP): GPIO18 (pulse), GPIO25 (SSR)
 *   Hopper 3 (20 PHP): GPIO4 (pulse), GPIO33 (SSR)
 * - I2C LCD: SDA=GPIO21, SCL=GPIO22
 * - Button: GPIO (to be configured)
 * - Serial: Connected to RPi for communication
 * 
 * Author: ESP32 CoinExchanger System
 * Date: October 2025
 */

#include "PIN_CONFIGURATION.h"
#include "SOLID_STATE_RELAY.h"
#include "COIN_HOPPER.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// LCD Configuration
#define LCD_ADDR 0x27
#define LCD_COLS 20
#define LCD_ROWS 4
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

// Button Pin
#define BUTTON_PIN 27  // BCM pin for button input

// Global variables - 3 Coin Hoppers
COIN_HOPPER coinHopper1(0);  // Hopper 1 - GPIO19 - 5 PHP
COIN_HOPPER coinHopper2(1);  // Hopper 2 - GPIO18 - 10 PHP
COIN_HOPPER coinHopper3(2);  // Hopper 3 - GPIO4 - 20 PHP

// System variables
unsigned long lastCoinCount[NUM_COIN_HOPPERS] = {0, 0, 0};

// System states
enum SystemState {
  STATE_IDLE,
  STATE_CHIT_DETECTED,
  STATE_DENOMINATION_SELECTION,
  STATE_CALCULATING,
  STATE_DISPENSING,
  STATE_COMPLETE,
  STATE_ERROR
};

SystemState currentState = STATE_IDLE;

// Chit detection variables
int detectedChitValue = 0;  // Value of detected chit (5, 10, 20, 50)

// Denomination selection variables
int selectedDenomination = 5;  // Current selection (5, 10, or 20)
const int availableDenominations[] = {5, 10, 20};
const int numDenominations = 3;
int denominationIndex = 0;

// Dispensing variables
struct DispensePlan {
  int coins_5;
  int coins_10;
  int coins_20;
  int totalCoins;
  int totalValue;
  int remainder;
};

DispensePlan currentPlan = {0, 0, 0, 0, 0, 0};
int dispensedCoins = 0;

// RPi communication commands
const String CMD_IR_DETECTED = "IR_DETECTED";
const String CMD_CHIT_DETECTED = "CHIT_DETECTED";
const String CMD_CHIT_RELEASED = "CHIT_RELEASED";
const String CMD_DETECTION_TIMEOUT = "DETECTION_TIMEOUT";
const String CMD_SYSTEM_SHUTDOWN = "SYSTEM_SHUTDOWN";

// Testing commands for Serial monitor
const String CMD_TEST_PULSE = "test_pulse";
const String CMD_TEST_RELAY = "test_relay";
const String CMD_TEST_ALL = "test_all";
const String CMD_HELP = "help";
const String CMD_TEST_CHIT = "test_chit";  // Simulate chit detection
const String CMD_TEST_HOPPER = "test_hopper";  // Test individual hopper
const String CMD_TEST_AUTO = "test_auto";  // Test auto-dispense

// Button handling
volatile bool buttonPressed = false;
volatile unsigned long lastButtonTime = 0;
const unsigned long DEBOUNCE_DELAY = 200;  // 200ms debounce

// Button ISR
void IRAM_ATTR buttonISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastButtonTime > DEBOUNCE_DELAY) {
    buttonPressed = true;
    lastButtonTime = currentTime;
  }
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  
  Serial.println("\n=== ESP32 Coin Exchanger System ===");
  Serial.println("Integrated with RPi Chit Detection");
  
  // Initialize I2C for LCD
  // Wire.begin(21, 22);  // SDA, SCL
  
  // // Initialize LCD
  // lcd.init();
  // lcd.backlight();
  // lcd.clear();
  // lcd.setCursor(0, 0);
  // lcd.print("Coin Exchanger");
  // lcd.setCursor(0, 1);
  // lcd.print("Initializing...");
  
  // Initialize button with interrupt
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);
  
  // Initialize all 3 coin hoppers with SSR control
  Serial.println("Initializing 3 ALLAN Coin Hoppers...");
  bool allInitialized = true;
  allInitialized &= coinHopper1.begin(COIN_HOPPER_1_PULSE_PIN, 0, COIN_HOPPER_1_SSR_PIN);
  allInitialized &= coinHopper2.begin(COIN_HOPPER_2_PULSE_PIN, 1, COIN_HOPPER_2_SSR_PIN);
  allInitialized &= coinHopper3.begin(COIN_HOPPER_3_PULSE_PIN, 2, COIN_HOPPER_3_SSR_PIN);
  
  if (allInitialized) {
    Serial.println("✅ All 3 coin hoppers initialized!");
    lcd.setCursor(0, 2);
    lcd.print("Hoppers: OK");
  } else {
    Serial.println("❌ Error initializing hoppers!");
    lcd.setCursor(0, 2);
    lcd.print("Hoppers: ERROR");
  }
  
  delay(2000);
  
  // Display ready message
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Ready!");
  lcd.setCursor(0, 1);
  lcd.print("Waiting for chit...");
  
  Serial.println("\n=== SYSTEM READY ===");
  Serial.println("Hopper values:");
  Serial.println("  Hopper 1: 5 PHP (GPIO19)");
  Serial.println("  Hopper 2: 10 PHP (GPIO18)");
  Serial.println("  Hopper 3: 20 PHP (GPIO4)");
  Serial.println("\nCommands from RPi:");
  Serial.println("  CHIT_DETECTED:<value>");
  Serial.println("\nTest commands:");
  Serial.println("  test_chit 50      - Simulate 50 peso chit");
  Serial.println("  test_auto 20      - Test auto-dispense for 20 peso");
  Serial.println("  test_hopper 1 3   - Dispense 3 coins from hopper 1");
  Serial.println("  test_pulse 1      - Test pulse reading on hopper 1");
  Serial.println("  test_relay 1 on   - Turn on relay for hopper 1");
  Serial.println("  help              - Show all commands");
  Serial.println("====================\n");
}



void loop() {
  // Handle serial commands from RPi
  handleSerialCommands();
  
  // Update all coin hoppers
  coinHopper1.update();
  coinHopper2.update();
  coinHopper3.update();
  
  // Handle state machine
  switch (currentState) {
    case STATE_IDLE:
      // Waiting for chit detection from RPi
      break;
      
    case STATE_CHIT_DETECTED:
      // Chit detected, show value and prompt for denomination selection
      handleChitDetected();
      break;
      
    case STATE_DENOMINATION_SELECTION:
      // User selecting denomination
      handleDenominationSelection();
      break;
      
    case STATE_CALCULATING:
      // Calculate optimal coin combination
      handleCalculation();
      break;
      
    case STATE_DISPENSING:
      // Dispense coins
      handleDispensing();
      break;
      
    case STATE_COMPLETE:
      // Dispensing complete
      handleComplete();
      break;
      
    case STATE_ERROR:
      // Error state
      handleError();
      break;
  }
  
  // Minimal delay
  delay(10);
}



// ========== STATE HANDLERS ==========

void handleChitDetected() {
  // Display chit value and start denomination selection
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Chit Detected!");
  lcd.setCursor(0, 1);
  lcd.print("Value: P");
  lcd.print(detectedChitValue);
  
  Serial.print("Chit detected: P");
  Serial.println(detectedChitValue);
  
  delay(2000);
  
  // Move to denomination selection
  currentState = STATE_DENOMINATION_SELECTION;
  denominationIndex = 0;
  selectedDenomination = availableDenominations[0];
}

void handleDenominationSelection() {
  // Display denomination selection UI
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Select Denom:");
  lcd.setCursor(0, 1);
  lcd.print("> P");
  lcd.print(selectedDenomination);
  lcd.print(" coins");
  
  lcd.setCursor(0, 2);
  lcd.print("Amount: P");
  lcd.print(detectedChitValue);
  
  lcd.setCursor(0, 3);
  lcd.print("Press to confirm");
  
  // Handle button press to cycle through denominations or confirm
  if (buttonPressed) {
    buttonPressed = false;
    
    // Cycle to next denomination
    denominationIndex++;
    if (denominationIndex >= numDenominations) {
      // User confirmed selection, move to calculation
      Serial.print("User selected denomination: P");
      Serial.println(selectedDenomination);
      currentState = STATE_CALCULATING;
    } else {
      selectedDenomination = availableDenominations[denominationIndex];
      Serial.print("Denomination changed to: P");
      Serial.println(selectedDenomination);
    }
  }
}

void handleCalculation() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Calculating...");
  
  Serial.println("Calculating coin combination...");
  
  // Calculate optimal coin combination based on ChitExchanger algorithm
  currentPlan = calculateCoinCombination(detectedChitValue, selectedDenomination);
  
  // Display plan
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Plan:");
  lcd.setCursor(0, 1);
  lcd.print("5P:");
  lcd.print(currentPlan.coins_5);
  lcd.print(" 10P:");
  lcd.print(currentPlan.coins_10);
  lcd.print(" 20P:");
  lcd.print(currentPlan.coins_20);
  
  lcd.setCursor(0, 2);
  lcd.print("Total: P");
  lcd.print(currentPlan.totalValue);
  
  if (currentPlan.remainder > 0) {
    lcd.setCursor(0, 3);
    lcd.print("Remainder: P");
    lcd.print(currentPlan.remainder);
  }
  
  Serial.println("=== Dispensing Plan ===");
  Serial.print("5 PHP coins: ");
  Serial.println(currentPlan.coins_5);
  Serial.print("10 PHP coins: ");
  Serial.println(currentPlan.coins_10);
  Serial.print("20 PHP coins: ");
  Serial.println(currentPlan.coins_20);
  Serial.print("Total value: P");
  Serial.println(currentPlan.totalValue);
  Serial.print("Remainder: P");
  Serial.println(currentPlan.remainder);
  
  delay(3000);
  
  // Move to dispensing
  currentState = STATE_DISPENSING;
  dispensedCoins = 0;
}

void handleDispensing() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Dispensing...");
  
  Serial.println("\n=== Starting Dispensing ===");
  
  COIN_HOPPER* hoppers[] = {&coinHopper1, &coinHopper2, &coinHopper3};
  int coinsToDispense[] = {currentPlan.coins_5, currentPlan.coins_10, currentPlan.coins_20};
  String denomNames[] = {"5 PHP", "10 PHP", "20 PHP"};
  int denomValues[] = {5, 10, 20};
  
  // Dispense from each hopper in order: 5, 10, 20
  for (int i = 0; i < 3; i++) {
    int coinsNeeded = coinsToDispense[i];
    
    if (coinsNeeded > 0) {
      lcd.setCursor(0, 1);
      lcd.print("Dispensing ");
      lcd.print(denomNames[i]);
      lcd.setCursor(0, 2);
      lcd.print("Count: 0/");
      lcd.print(coinsNeeded);
      
      Serial.print("Dispensing ");
      Serial.print(coinsNeeded);
      Serial.print(" x ");
      Serial.print(denomNames[i]);
      Serial.print(" coins from Hopper ");
      Serial.println(i + 1);
      
      // Reset counter for this hopper
      unsigned long startCount = hoppers[i]->getTotalCoins();
      
      // Turn ON SSR relay for this hopper
      Serial.print("Enabling SSR for Hopper ");
      Serial.print(i + 1);
      Serial.print(" (GPIO");
      Serial.print(hoppers[i]->getSSRPin());
      Serial.println(")");
      hoppers[i]->enableSSR();
      delay(100);  // Small delay for relay to activate
      
      // Start dispensing by amount (coins * value)
      int amountToDispense = coinsNeeded * denomValues[i];
      hoppers[i]->dispenseAmount(amountToDispense);
      
      // Monitor dispensing progress
      unsigned long targetCount = startCount + coinsNeeded;
      unsigned long timeout = millis() + 30000;  // 30 second timeout
      
      while (hoppers[i]->getTotalCoins() < targetCount && millis() < timeout) {
        hoppers[i]->update();
        
        unsigned long currentCount = hoppers[i]->getTotalCoins();
        int dispensed = currentCount - startCount;
        
        // Update LCD
        lcd.setCursor(7, 2);
        lcd.print(dispensed);
        lcd.print(" ");  // Clear extra chars
        
        delay(50);
      }
      
      // Turn OFF SSR relay for this hopper
      Serial.print("Disabling SSR for Hopper ");
      Serial.println(i + 1);
      hoppers[i]->disableSSR();
      
      unsigned long finalCount = hoppers[i]->getTotalCoins();
      int actuallyDispensed = finalCount - startCount;
      
      Serial.print("Dispensed: ");
      Serial.print(actuallyDispensed);
      Serial.print("/");
      Serial.print(coinsNeeded);
      Serial.print(" coins (");
      Serial.print(actuallyDispensed * denomValues[i]);
      Serial.print(" PHP)");
      Serial.println();
      
      if (actuallyDispensed < coinsNeeded) {
        Serial.println("WARNING: Dispensing incomplete!");
      }
      
      delay(500);
    }
  }
  
  Serial.println("=== Dispensing Complete ===\n");
  
  // Move to complete state
  currentState = STATE_COMPLETE;
}

void handleComplete() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Complete!");
  lcd.setCursor(0, 1);
  lcd.print("Dispensed: P");
  lcd.print(currentPlan.totalValue);
  
  if (currentPlan.remainder > 0) {
    lcd.setCursor(0, 2);
    lcd.print("Remainder: P");
    lcd.print(currentPlan.remainder);
    lcd.setCursor(0, 3);
    lcd.print("(Keep chit)");
  } else {
    lcd.setCursor(0, 2);
    lcd.print("Thank you!");
  }
  
  Serial.println("Transaction complete!");
  
  // Send completion message to RPi
  Serial.print("DISPENSING_COMPLETE:");
  Serial.println(currentPlan.totalValue);
  
  delay(5000);
  
  // Reset and return to idle
  detectedChitValue = 0;
  selectedDenomination = 5;
  denominationIndex = 0;
  currentPlan = {0, 0, 0, 0, 0, 0};
  dispensedCoins = 0;
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Ready!");
  lcd.setCursor(0, 1);
  lcd.print("Waiting for chit...");
  
  currentState = STATE_IDLE;
}

void handleError() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ERROR!");
  lcd.setCursor(0, 1);
  lcd.print("Please contact");
  lcd.setCursor(0, 2);
  lcd.print("staff");
  
  Serial.println("ERROR STATE - Waiting for reset");
  
  delay(5000);
  
  // Return to idle after displaying error
  currentState = STATE_IDLE;
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Ready!");
  lcd.setCursor(0, 1);
  lcd.print("Waiting for chit...");
}

// Calculate coin combination based on auto-dispense logic
// Map chit values to hoppers:
// - 5 PHP chit -> Hopper 1 (5 peso coins): 1 coin
// - 10 PHP chit -> Hopper 2 (10 peso coins): 1 coin  
// - 20 PHP chit -> Hopper 3 (20 peso coins): 1 coin
// - 50 PHP chit -> Hopper 3 (20 peso coins): 2 coins + 10 peso: 1 coin (remainder calculated)
DispensePlan calculateCoinCombination(int chitValue, int preferredDenom) {
  DispensePlan plan = {0, 0, 0, 0, 0, 0};
  
  int remaining = chitValue;
  
  // Optimal dispensing strategy based on chit value
  if (chitValue == 5) {
    // 5 PHP: Use Hopper 1 (5 peso coins)
    plan.coins_5 = 1;
    remaining = 0;
  } 
  else if (chitValue == 10) {
    // 10 PHP: Use Hopper 2 (10 peso coins)
    plan.coins_10 = 1;
    remaining = 0;
  } 
  else if (chitValue == 20) {
    // 20 PHP: Use Hopper 3 (20 peso coins)
    plan.coins_20 = 1;
    remaining = 0;
  } 
  else if (chitValue == 50) {
    // 50 PHP: Use Hopper 3 (20 peso) x2 + Hopper 2 (10 peso) x1
    plan.coins_20 = 2;  // 40 PHP
    plan.coins_10 = 1;  // 10 PHP
    remaining = 0;
  }
  else {
    // For other amounts, use greedy algorithm (largest denomination first)
    // Start with 20 peso coins
    plan.coins_20 = remaining / 20;
    remaining %= 20;
    
    // Then 10 peso coins
    plan.coins_10 = remaining / 10;
    remaining %= 10;
    
    // Finally 5 peso coins
    plan.coins_5 = remaining / 5;
    remaining %= 5;
  }
  
  plan.totalCoins = plan.coins_5 + plan.coins_10 + plan.coins_20;
  plan.totalValue = (plan.coins_5 * 5) + (plan.coins_10 * 10) + (plan.coins_20 * 20);
  plan.remainder = remaining;
  
  return plan;
}

// ========== SERIAL COMMUNICATION ==========

void handleSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    // Check for test commands first
    String lowerCommand = command;
    lowerCommand.toLowerCase();
    
    if (handleTestCommand(lowerCommand)) {
      return;
    }
    
    // Handle RPi commands
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
    }
    return true;
  }
  else if (command.startsWith("test_relay")) {
    int firstSpace = command.indexOf(' ');
    int secondSpace = command.indexOf(' ', firstSpace + 1);
    
    if (firstSpace > 0 && secondSpace > 0) {
      int hopperId = command.substring(firstSpace + 1, secondSpace).toInt();
      String state = command.substring(secondSpace + 1);
      testRelayControl(hopperId, state);
    } else {
      Serial.println("Usage: test_relay 1/2/3 on/off");
    }
    return true;
  }
  else if (command.startsWith("test_hopper")) {
    // test_hopper 1 5 - dispense 5 coins from hopper 1
    int firstSpace = command.indexOf(' ');
    int secondSpace = command.indexOf(' ', firstSpace + 1);
    
    if (firstSpace > 0 && secondSpace > 0) {
      int hopperId = command.substring(firstSpace + 1, secondSpace).toInt();
      int numCoins = command.substring(secondSpace + 1).toInt();
      testHopperDispense(hopperId, numCoins);
    } else {
      Serial.println("Usage: test_hopper 1/2/3 <num_coins>");
      Serial.println("Example: test_hopper 1 5");
    }
    return true;
  }
  else if (command.startsWith("test_auto")) {
    // test_auto 50 - test auto-dispense for 50 peso
    int spaceIndex = command.indexOf(' ');
    if (spaceIndex > 0) {
      int chitValue = command.substring(spaceIndex + 1).toInt();
      testAutoDispense(chitValue);
    } else {
      Serial.println("Usage: test_auto <value>");
      Serial.println("Example: test_auto 50");
    }
    return true;
  }
  else if (command == "test_all") {
    testAllComponents();
    return true;
  }
  else if (command.startsWith("test_chit")) {
    int spaceIndex = command.indexOf(' ');
    if (spaceIndex > 0) {
      int chitValue = command.substring(spaceIndex + 1).toInt();
      testChitDetection(chitValue);
    } else {
      Serial.println("Usage: test_chit <value>");
      Serial.println("Example: test_chit 50");
    }
    return true;
  }
  else if (command == "help") {
    showTestHelp();
    return true;
  }
  
  return false;
}

// RPi Communication Functions
void handleRPiCommand(String command) {
  command.trim();
  
  // Handle AUTO_DISPENSE:value - Automatic dispensing trigger
  if (command.startsWith("AUTO_DISPENSE:")) {
    int colonIndex = command.indexOf(':');
    if (colonIndex > 0) {
      String valueStr = command.substring(colonIndex + 1);
      int chitValue = valueStr.toInt();
      
      if (chitValue == 5 || chitValue == 10 || chitValue == 20 || chitValue == 50) {
        // Check if system is busy
        if (currentState != STATE_IDLE && currentState != STATE_COMPLETE && currentState != STATE_ERROR) {
          Serial.println("========================================");
          Serial.println("⚠️  SYSTEM BUSY - Ignoring AUTO_DISPENSE");
          Serial.print("Current state: ");
          Serial.println(currentState);
          Serial.println("========================================");
          return;
        }
        
        Serial.println("========================================");
        Serial.print("🎯 AUTO_DISPENSE received: P");
        Serial.println(chitValue);
        Serial.println("========================================");
        
        // Set the detected chit value
        detectedChitValue = chitValue;
        
        // Set default denomination preference (5 peso coins)
        selectedDenomination = 5;
        
        // Update LCD
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("AUTO DISPENSE!");
        lcd.setCursor(0, 1);
        lcd.print("Chit: P");
        lcd.print(chitValue);
        lcd.setCursor(0, 2);
        lcd.print("Calculating...");
        
        delay(1000);
        
        // Calculate optimal coin combination
        currentPlan = calculateCoinCombination(detectedChitValue, selectedDenomination);
        
        // Display plan
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Dispensing Plan:");
        lcd.setCursor(0, 1);
        lcd.print("5P:");
        lcd.print(currentPlan.coins_5);
        lcd.print(" 10P:");
        lcd.print(currentPlan.coins_10);
        lcd.setCursor(0, 2);
        lcd.print("20P:");
        lcd.print(currentPlan.coins_20);
        lcd.setCursor(0, 3);
        lcd.print("Total: P");
        lcd.print(currentPlan.totalValue);
        
        Serial.println("=== Auto Dispensing Plan ===");
        Serial.print("5 PHP coins: ");
        Serial.println(currentPlan.coins_5);
        Serial.print("10 PHP coins: ");
        Serial.println(currentPlan.coins_10);
        Serial.print("20 PHP coins: ");
        Serial.println(currentPlan.coins_20);
        Serial.print("Total value: P");
        Serial.println(currentPlan.totalValue);
        
        if (currentPlan.remainder > 0) {
          Serial.print("⚠️  Remainder: P");
          Serial.println(currentPlan.remainder);
        }
        
        delay(2000);
        
        // Start dispensing immediately
        currentState = STATE_DISPENSING;
        dispensedCoins = 0;
        
        Serial.println("🚀 Starting automatic coin dispensing...");
      } else {
        Serial.print("❌ Invalid chit value: ");
        Serial.println(chitValue);
      }
    }
  }
  // Handle CHIT_DETECTED:value
  else if (command.startsWith(CMD_CHIT_DETECTED + ":")) {
    int colonIndex = command.indexOf(':');
    if (colonIndex > 0) {
      String valueStr = command.substring(colonIndex + 1);
      int chitValue = valueStr.toInt();
      
      if (chitValue == 5 || chitValue == 10 || chitValue == 20 || chitValue == 50) {
        Serial.print("Received chit detection: P");
        Serial.println(chitValue);
        
        detectedChitValue = chitValue;
        currentState = STATE_CHIT_DETECTED;
      } else {
        Serial.print("Invalid chit value: ");
        Serial.println(chitValue);
      }
    }
  }
  else if (command == CMD_IR_DETECTED) {
    Serial.println("IR sensor activated");
  }
  else if (command.startsWith(CMD_CHIT_RELEASED)) {
    Serial.println("Chit released by RPi servo");
  }
  else if (command == CMD_DETECTION_TIMEOUT) {
    Serial.println("RPi detection timeout");
  }
  else if (command == CMD_SYSTEM_SHUTDOWN) {
    Serial.println("RPi system shutting down");
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

// Test chit detection (simulate RPi command)
void testChitDetection(int chitValue) {
  Serial.print("\n=== SIMULATING CHIT DETECTION: P");
  Serial.print(chitValue);
  Serial.println(" ===");
  
  if (chitValue != 5 && chitValue != 10 && chitValue != 20 && chitValue != 50) {
    Serial.println("Invalid chit value! Use 5, 10, 20, or 50");
    return;
  }
  
  detectedChitValue = chitValue;
  currentState = STATE_CHIT_DETECTED;
  
  Serial.println("Test started - follow LCD prompts");
  Serial.println("Press button to cycle denominations and confirm");
}

// Test individual hopper dispensing with pulse reading
void testHopperDispense(int hopperId, int numCoins) {
  if (hopperId < 1 || hopperId > 3) {
    Serial.println("❌ Invalid hopper ID. Use 1, 2, or 3");
    return;
  }
  
  if (numCoins <= 0 || numCoins > 20) {
    Serial.println("❌ Invalid number of coins. Use 1-20");
    return;
  }
  
  COIN_HOPPER* hoppers[] = {&coinHopper1, &coinHopper2, &coinHopper3};
  COIN_HOPPER* hopper = hoppers[hopperId - 1];
  int coinValues[] = {5, 10, 20};
  int coinValue = coinValues[hopperId - 1];
  
  Serial.println("\n=== HOPPER DISPENSING TEST ===");
  Serial.print("Hopper: ");
  Serial.println(hopperId);
  Serial.print("Coin Value: ");
  Serial.print(coinValue);
  Serial.println(" PHP");
  Serial.print("Coins to Dispense: ");
  Serial.println(numCoins);
  Serial.print("Total Amount: ");
  Serial.print(numCoins * coinValue);
  Serial.println(" PHP");
  Serial.print("Pulse GPIO: ");
  Serial.println(hopper->getPulsePin());
  Serial.print("SSR GPIO: ");
  Serial.println(hopper->getSSRPin());
  Serial.println("==============================");
  
  unsigned long startCount = hopper->getTotalCoins();
  
  // Let COIN_HOPPER manage SSR on/off internally
  Serial.println("🪙 Starting coin dispensing...");
  Serial.println("Watching for pulses (30 second timeout)...");
  
  // Dispense by amount (this function handles everything internally)
  unsigned long testStart = millis();
  int amountToDispense = numCoins * coinValue;
  hopper->dispenseAmount(amountToDispense);
  
  // Final results
  unsigned long finalCount = hopper->getTotalCoins();
  int actualDispensed = finalCount - startCount;
  int actualAmount = actualDispensed * coinValue;
  unsigned long totalTime = millis() - testStart;
  
  Serial.println("\n📊 TEST RESULTS:");
  Serial.print("  Target Coins: ");
  Serial.println(numCoins);
  Serial.print("  Dispensed Coins: ");
  Serial.println(actualDispensed);
  Serial.print("  Target Amount: ");
  Serial.print(numCoins * coinValue);
  Serial.println(" PHP");
  Serial.print("  Dispensed Amount: ");
  Serial.print(actualAmount);
  Serial.println(" PHP");
  Serial.print("  Total Count: ");
  Serial.println(finalCount);
  Serial.print("  Pulse Rate: ");
  Serial.print(hopper->getPulseRate(), 2);
  Serial.println(" coins/sec");
  Serial.print("  Time: ");
  Serial.print(totalTime / 1000.0, 2);
  Serial.println(" seconds");
  
  if (actualDispensed == numCoins) {
    Serial.println("✅ TEST PASSED!");
  } else {
    Serial.println("❌ TEST FAILED - Coin count mismatch!");
  }
  Serial.println("==============================\n");
}

// Test auto-dispense logic (simulates AUTO_DISPENSE command)
void testAutoDispense(int chitValue) {
  if (chitValue != 5 && chitValue != 10 && chitValue != 20 && chitValue != 50) {
    Serial.println("❌ Invalid chit value. Use 5, 10, 20, or 50");
    return;
  }
  
  // Check if system is busy
  if (currentState != STATE_IDLE && currentState != STATE_COMPLETE && currentState != STATE_ERROR) {
    Serial.println("========================================");
    Serial.println("⚠️  SYSTEM BUSY - Cannot test auto-dispense");
    Serial.print("Current state: ");
    Serial.println(currentState);
    Serial.println("========================================");
    return;
  }
  
  Serial.println("\n========================================");
  Serial.println("🧪 AUTO-DISPENSE TEST");
  Serial.print("Simulating detection of P");
  Serial.print(chitValue);
  Serial.println(" chit");
  Serial.println("========================================");
  
  // Set the detected chit value
  detectedChitValue = chitValue;
  selectedDenomination = 5;  // Default to 5 peso coins
  
  // Update LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("TEST AUTO DISPENSE");
  lcd.setCursor(0, 1);
  lcd.print("Chit: P");
  lcd.print(chitValue);
  lcd.setCursor(0, 2);
  lcd.print("Calculating...");
  
  delay(1000);
  
  // Calculate optimal coin combination
  currentPlan = calculateCoinCombination(detectedChitValue, selectedDenomination);
  
  // Display plan
  Serial.println("=== Dispensing Plan ===");
  Serial.print("5 PHP coins: ");
  Serial.print(currentPlan.coins_5);
  Serial.print(" (");
  Serial.print(currentPlan.coins_5 * 5);
  Serial.println(" PHP)");
  Serial.print("10 PHP coins: ");
  Serial.print(currentPlan.coins_10);
  Serial.print(" (");
  Serial.print(currentPlan.coins_10 * 10);
  Serial.println(" PHP)");
  Serial.print("20 PHP coins: ");
  Serial.print(currentPlan.coins_20);
  Serial.print(" (");
  Serial.print(currentPlan.coins_20 * 20);
  Serial.println(" PHP)");
  Serial.print("Total value: P");
  Serial.println(currentPlan.totalValue);
  
  if (currentPlan.remainder > 0) {
    Serial.print("⚠️  Remainder: P");
    Serial.println(currentPlan.remainder);
  }
  Serial.println("======================");
  
  // Display on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Dispensing Plan:");
  lcd.setCursor(0, 1);
  lcd.print("5P:");
  lcd.print(currentPlan.coins_5);
  lcd.print(" 10P:");
  lcd.print(currentPlan.coins_10);
  lcd.setCursor(0, 2);
  lcd.print("20P:");
  lcd.print(currentPlan.coins_20);
  lcd.setCursor(0, 3);
  lcd.print("Total: P");
  lcd.print(currentPlan.totalValue);
  
  delay(3000);
  
  // Start dispensing
  currentState = STATE_DISPENSING;
  dispensedCoins = 0;
  
  Serial.println("🚀 Starting automatic coin dispensing test...");
  Serial.println("Press Ctrl+C to abort if needed\n");
}

void showTestHelp() {
  Serial.println("\n=== COIN EXCHANGER HELP ===");
  Serial.println();
  Serial.println("🪙 SYSTEM WORKFLOW:");
  Serial.println("  1. RPi detects chit with IR + YOLO");
  Serial.println("  2. RPi sends AUTO_DISPENSE:<value> to ESP32");
  Serial.println("  3. ESP32 calculates optimal coin combination");
  Serial.println("  4. ESP32 enables SSR and dispenses coins");
  Serial.println("  5. Coins counted via pulse detection");
  Serial.println("  6. SSR disabled after dispensing");
  Serial.println();
  Serial.println("🧪 TEST COMMANDS:");
  Serial.println("  test_auto 50        - Test auto-dispense for 50 peso");
  Serial.println("  test_hopper 1 5     - Dispense 5 coins from hopper 1");
  Serial.println("  test_pulse 1        - Test pulse detection on hopper 1");
  Serial.println("  test_relay 1 on/off - Test SSR relay for hopper 1");
  Serial.println("  test_all            - Test all components");
  Serial.println("  test_chit 50        - Simulate manual chit detection");
  Serial.println("  help                - Show this help");
  Serial.println();
  Serial.println("📌 RPi COMMANDS:");
  Serial.println("  AUTO_DISPENSE:<value> - Auto dispense for detected chit");
  Serial.println("    Example: AUTO_DISPENSE:50");
  Serial.println("  CHIT_DETECTED:<value> - Manual chit processing");
  Serial.println("  IR_DETECTED           - IR sensor activated");
  Serial.println();
  Serial.println("🎯 HOPPER MAPPING:");
  Serial.println("  5 PHP chit  -> Hopper 1 (5 peso): 1 coin");
  Serial.println("  10 PHP chit -> Hopper 2 (10 peso): 1 coin");
  Serial.println("  20 PHP chit -> Hopper 3 (20 peso): 1 coin");
  Serial.println("  50 PHP chit -> Hopper 3 (20 peso): 2 coins");
  Serial.println("                 Hopper 2 (10 peso): 1 coin");
  Serial.println();
  Serial.println("💡 HARDWARE:");
  Serial.println("  Button: GPIO27 (cycles denominations)");
  Serial.println("  LCD: I2C 0x27 (20x4)");
  Serial.println("  Hopper 1: GPIO4 pulse + GPIO26 SSR (5 PHP)");
  Serial.println("  Hopper 2: GPIO18 pulse + GPIO25 SSR (10 PHP)");
  Serial.println("  Hopper 3: GPIO19 pulse + GPIO33 SSR (20 PHP)");
  Serial.println();
  Serial.println("📖 EXAMPLES:");
  Serial.println("  Test single hopper:");
  Serial.println("    > test_hopper 1 3");
  Serial.println("  ");
  Serial.println("  Test auto-dispense:");
  Serial.println("    > test_auto 50");
  Serial.println("  ");
  Serial.println("  Test pulse reading:");
  Serial.println("    > test_pulse 2");
  Serial.println("    (Drop coins manually, watch for pulses)");
  Serial.println("===========================\n");
}