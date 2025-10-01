#ifndef I2C_LCD_H
#define I2C_LCD_H

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Initialize the LCD object for a 20x4 display at I2C address 0x27
extern LiquidCrystal_I2C lcd;

void initLCD();
void displayMessage(const char* message, int row);

#endif // I2C_LCD_H
