#ifndef PIN_CONFIGURATION_H
#define PIN_CONFIGURATION_H

// Pin configuration for all components

// Coin Slot
#define coinPin 4

// Bill Acceptor
#define billPin 26

// Servo Dispenser
#define SERVO_SDA 21
#define SERVO_SCL 22

// Chit Dispenser Servo Channels
#define CHIT_5_CHANNEL 0    // Servo channel for ₱5 chits
#define CHIT_10_CHANNEL 1   // Servo channel for ₱10 chits
#define CHIT_20_CHANNEL 2   // Servo channel for ₱20 chits
#define CHIT_50_CHANNEL 3   // Servo channel for ₱50 chits

// Chit Values
#define CHIT_VALUE_5 5
#define CHIT_VALUE_10 10
#define CHIT_VALUE_20 20
#define CHIT_VALUE_50 50

// Piezo Buzzer
#define BUZZER_PIN 27

#endif // PIN_CONFIGURATION_H
