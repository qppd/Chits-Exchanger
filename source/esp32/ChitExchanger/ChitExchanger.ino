// ChitExchanger.ino

#include "COIN_SLOT.h"
#include "SERVO_DISPENSER.h"
#include "BILL_ACCEPTOR.h"
#include "I2C_LCD.h"
#include "PIEZO_BUZZER.h"

// Dispensing State Machine
enum DispensingState {
  IDLE,
  CALCULATING,
  DISPENSING,
  COMPLETE
};

// Global variables
const int ledPin = 13;      // Pin for LED output
int tb74Credit = 0;        // Variable to track TB74 credit
DispensingState currentState = IDLE;
unsigned long lastStateChange = 0;
bool dispensingInProgress = false;

// Dispensing calculation structure
struct ChitDispense {
  int chits_50;
  int chits_20;
  int chits_10;
  int chits_5;
  int totalValue;
  bool isValid;
};

// Function declarations
ChitDispense calculateOptimalChits(int creditAmount);
void dispenseChits(ChitDispense dispensePlan);
void dispenseChitType(int channel, int count, int chitValue);
void playDispensingFeedback(int chitValue, int count);
void updateDispensingDisplay(int chitValue, int count, bool isComplete);
void triggerAutoDispensing(); // New function for triggering dispensing

void setup() {
  pinMode(ledPin, OUTPUT);      // Initialize the LED pin as an output
  Serial.begin(9600);           // Start serial communication
  
  initALLANCOIN(); // Initialize coin slot interrupt
  initSERVO(); // Initialize servo dispenser (PCA9685)
  initBILLACCEPTOR(); // Initialize bill acceptor interrupt
  initLCD(); // Initialize the LCD
  initBuzzer(); // Initialize the buzzer
  
  displayMessage("Welcome!", 0); // Display a welcome message on the first row
  displayMessage("Insert bills", 1); // Shortened

  // Play a tone to indicate setup is finished
  playTone(1200, 300); // 1.2kHz tone for 300ms
  
  Serial.println("ChitExchanger initialized successfully!");
  Serial.println("Ready for automatic chit dispensing:");
  Serial.println("- Chit values: P5, P10, P20, P50");
  Serial.println("- Bills accepted: P20, P50");
  Serial.println("- Coins accepted: P1, P5, P10, P20");
  Serial.println("- Using existing SERVO_DISPENSER for chit dispensing");
}

// Calculate optimal chit combination for given credit amount
ChitDispense calculateOptimalChits(int creditAmount) {
  ChitDispense result = {0, 0, 0, 0, 0, false};
  
  // Check if amount is valid (must be multiple of 5)
  if (creditAmount < 5 || creditAmount % 5 != 0) {
    return result;
  }
  
  int remaining = creditAmount;
  
  // Greedy algorithm: Start with highest value chits
  result.chits_50 = remaining / CHIT_VALUE_50;
  remaining %= CHIT_VALUE_50;
  
  result.chits_20 = remaining / CHIT_VALUE_20;
  remaining %= CHIT_VALUE_20;
  
  result.chits_10 = remaining / CHIT_VALUE_10;
  remaining %= CHIT_VALUE_10;
  
  result.chits_5 = remaining / CHIT_VALUE_5;
  remaining %= CHIT_VALUE_5;
  
  result.totalValue = creditAmount;
  result.isValid = (remaining == 0);
  
  return result;
}

// Dispense individual chit type
void dispenseChitType(int channel, int count, int chitValue) {
  if (count <= 0) return;
  
  for (int i = 0; i < count; i++) {
    updateDispensingDisplay(chitValue, i + 1, false);
    
    // Use timing-based dispensing for 360-degree servos
    dispenseCard(channel, chitValue);
    
    playDispensingFeedback(chitValue, i + 1);
    delay(300); // Short delay between dispensing individual chits
  }
}

// Main dispensing function
void dispenseChits(ChitDispense dispensePlan) {
  if (!dispensePlan.isValid) {
    displayMessage("Invalid Amount!", 1);
    playTone(500, 1000); // Error tone
    Serial.println("ERROR: Invalid dispensing amount");
    return;
  }
  
  currentState = DISPENSING;
  dispensingInProgress = true;
  
  // Log dispensing plan
  Serial.println("=== DISPENSING PLAN ===");
  Serial.printf("Total Amount: ₱%d\n", dispensePlan.totalValue);
  Serial.printf("₱50 chits: %d\n", dispensePlan.chits_50);
  Serial.printf("₱20 chits: %d\n", dispensePlan.chits_20);
  Serial.printf("₱10 chits: %d\n", dispensePlan.chits_10);
  Serial.printf("₱5 chits: %d\n", dispensePlan.chits_5);
  Serial.println("======================");
  
  // Play start dispensing tone
  playTone(1500, 200);
  delay(300);
  
  // Dispense ₱50 chits
  if (dispensePlan.chits_50 > 0) {
    Serial.printf("Dispensing %d x ₱50 chits...\n", dispensePlan.chits_50);
    dispenseChitType(CHIT_50_CHANNEL, dispensePlan.chits_50, CHIT_VALUE_50);
  }
  
  // Dispense ₱20 chits
  if (dispensePlan.chits_20 > 0) {
    Serial.printf("Dispensing %d x ₱20 chits...\n", dispensePlan.chits_20);
    dispenseChitType(CHIT_20_CHANNEL, dispensePlan.chits_20, CHIT_VALUE_20);
  }
  
  // Dispense ₱10 chits
  if (dispensePlan.chits_10 > 0) {
    Serial.printf("Dispensing %d x ₱10 chits...\n", dispensePlan.chits_10);
    dispenseChitType(CHIT_10_CHANNEL, dispensePlan.chits_10, CHIT_VALUE_10);
  }
  
  // Dispense ₱5 chits
  if (dispensePlan.chits_5 > 0) {
    Serial.printf("Dispensing %d x ₱5 chits...\n", dispensePlan.chits_5);
    dispenseChitType(CHIT_5_CHANNEL, dispensePlan.chits_5, CHIT_VALUE_5);
  }
  
  // Dispensing complete
  currentState = COMPLETE;
  updateDispensingDisplay(0, 0, true);
  Serial.println("=== DISPENSING COMPLETE ===");
  
  // Play completion tone sequence
  for (int i = 0; i < 3; i++) {
    playTone(1800, 150);
    delay(100);
  }
  
  // Reset credit and state after delay
  delay(3000);
  billCredit = 0;
  currentState = IDLE;
  dispensingInProgress = false;
  displayMessage("Insert bills", 1); // Shortened
}

// Display dispensing progress
void updateDispensingDisplay(int chitValue, int count, bool isComplete) {
  if (isComplete) {
    displayMessage("Complete!", 0);
    displayMessage("Enjoy your chits!", 1);
    return;
  }
  
  char msg1[21], msg2[21];
  snprintf(msg1, sizeof(msg1), "Dispensing...");
  snprintf(msg2, sizeof(msg2), "P%d #%d", chitValue, count); // Shortened format
  
  displayMessage(msg1, 0);
  displayMessage(msg2, 1);
}

// Play feedback tones for dispensing
void playDispensingFeedback(int chitValue, int count) {
  // Different tones for different chit values
  int frequency = 1000;
  switch (chitValue) {
    case 50: frequency = 1600; break;
    case 20: frequency = 1400; break;
    case 10: frequency = 1200; break;
    case 5: frequency = 1000; break;
  }
  
  playTone(frequency, 100);
}

// Trigger automatic dispensing when minimum credit is reached
void triggerAutoDispensing() {
  if (billCredit >= 5) {
    // Calculate optimal chit combination
    ChitDispense dispensePlan = calculateOptimalChits(billCredit);
    
    if (dispensePlan.isValid) {
      // Display dispensing plan briefly
      char msg1[21], msg2[21];
      snprintf(msg1, sizeof(msg1), "Total: P%d", billCredit);
      int totalChits = dispensePlan.chits_50 + dispensePlan.chits_20 + 
                      dispensePlan.chits_10 + dispensePlan.chits_5;
      snprintf(msg2, sizeof(msg2), "%d chits ready", totalChits);
      
      displayMessage(msg1, 0);
      displayMessage(msg2, 1);
      delay(2000);
      
      // Start dispensing
      dispenseChits(dispensePlan);
    } else {
      displayMessage("Invalid Amount!", 0);
      displayMessage("Must be multiple of 5", 1);
      playTone(500, 1000); // Error tone
      delay(3000);
      billCredit = 0; // Reset invalid credit
      displayMessage("Insert bills", 1);
    }
  }
}

void loop() {
  // Skip processing if dispensing is in progress
  if (dispensingInProgress) {
    delay(100);
    return;
  }
  
  // Handle coin insertion
  if (coinInserted && (millis() - lastCoinPulseTime > 300)) {
    int coinValue = getCoinValue();
    if (coinValue > 0) {
      coinCredit += coinValue;
      billCredit += coinValue; // Add coin value to total credit
      playTone(1000 + (coinValue * 50), 200); // Different tone based on coin value
      Serial.print("Coin inserted: P");
      Serial.print(coinValue);
      Serial.print(" | Total credit: P");
      Serial.println(billCredit);
      
      // Update display - show accumulating credit
      displayMessage("COIN ADDED", 0);
      char coinMsg[21];
      if (billCredit < 1000) {
        snprintf(coinMsg, sizeof(coinMsg), "P%d | Total: P%d", coinValue, billCredit);
      } else {
        snprintf(coinMsg, sizeof(coinMsg), "P%d | Total: P%dk", coinValue, billCredit/1000);
      }
      displayMessage(coinMsg, 1);
      delay(1500);
      
      // Check if we have enough credit to start dispensing
      if (billCredit >= 5) {
        // Trigger automatic dispensing for accumulated coins
        triggerAutoDispensing();
      } else {
        // Show waiting message for small amounts
        displayMessage("CREDIT", 0);
        char waitMsg[21];
        snprintf(waitMsg, sizeof(waitMsg), "P%d (Min: P5)", billCredit);
        displayMessage(waitMsg, 1);
      }
    }
    resetCoinDetection();
  }

  // Bill acceptor pulse counting and credit update
  // Show 'Counting...' while pulses are being received
  if (pulseCount > 0 && (millis() - lastPulseTime <= 250)) {
    currentState = CALCULATING;
    displayMessage("Counting bills...", 0);
    char creditMsg[21];
    int tempCredit = billCredit + getBillValue();
    snprintf(creditMsg, sizeof(creditMsg), "CREDIT: P%d", tempCredit);
    displayMessage(creditMsg, 1);
  }
  // When counting is done, show final credit and start dispensing
  else if (pulseCount > 0 && (millis() - lastPulseTime > 250)) {
    int billValue = getBillValue();
    if (billValue > 0) {
      billCredit += billValue;
      Serial.print("Bill accepted: P");
      Serial.print(billValue);
      Serial.print(" | Final Credit: P");
      Serial.println(billCredit);
    }
    pulseCount = 0;
    
    // Start automatic dispensing process
    if (billCredit >= 5) {
      triggerAutoDispensing();
    } else {
      displayMessage("Minimum P5", 0); // Shortened
      displayMessage("Insert more", 1);
      delay(2000);
      displayMessage("Insert bills", 1); // Shortened
    }
  }
  // Idle state - show current credit if any
  else if (currentState == IDLE && billCredit > 0) {
    displayMessage("CREDIT", 0);
    char creditMsg[21];
    snprintf(creditMsg, sizeof(creditMsg), "P%d", billCredit); // Shortened
    displayMessage(creditMsg, 1);
  }

  delay(50); // Small delay for loop optimization
}