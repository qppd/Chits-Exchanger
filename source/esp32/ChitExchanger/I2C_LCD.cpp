#include "I2C_LCD.h"
#include <Arduino.h>

// Initialize the LCD object for a 20x4 display at I2C address 0x27
LiquidCrystal_I2C lcd(0x27, 20, 4);

void initLCD() {
  // I2C is already initialized in main setup()
  Serial.println("Initializing LCD at I2C address 0x27...");
  
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("LCD Initialized");
  delay(1000);
  lcd.clear();
  
  Serial.println("LCD initialization complete");
}

void displayMessage(const char* message, int row) {
  if (row >= 0 && row < 4) { // Ensure row is within bounds
    lcd.setCursor(0, row);
    lcd.print("                    "); // Clear the row
    lcd.setCursor(0, row);
    lcd.print(message);
  }
}
