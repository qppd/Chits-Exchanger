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

// Chit Dispenser Servo Channels (8 servos - 4 pairs, 2 servos per denomination)
// ₱50 chits - Channels 0 and 1
#define CHIT_50_CHANNEL_1 0    // First servo for ₱50 chits
#define CHIT_50_CHANNEL_2 1    // Second servo for ₱50 chits

// ₱20 chits - Channels 2 and 3
#define CHIT_20_CHANNEL_1 2    // First servo for ₱20 chits
#define CHIT_20_CHANNEL_2 3    // Second servo for ₱20 chits

// ₱10 chits - Channels 4 and 5
#define CHIT_10_CHANNEL_1 4    // First servo for ₱10 chits
#define CHIT_10_CHANNEL_2 5    // Second servo for ₱10 chits

// ₱5 chits - Channels 6 and 7
#define CHIT_5_CHANNEL_1 6     // First servo for ₱5 chits
#define CHIT_5_CHANNEL_2 7     // Second servo for ₱5 chits

// Legacy channel definitions (for compatibility - map to first servo of each pair)
#define CHIT_5_CHANNEL CHIT_5_CHANNEL_1
#define CHIT_10_CHANNEL CHIT_10_CHANNEL_1
#define CHIT_20_CHANNEL CHIT_20_CHANNEL_1
#define CHIT_50_CHANNEL CHIT_50_CHANNEL_1

// Chit Values
#define CHIT_VALUE_5 5
#define CHIT_VALUE_10 10
#define CHIT_VALUE_20 20
#define CHIT_VALUE_50 50

// Piezo Buzzer
#define BUZZER_PIN 27

#endif // PIN_CONFIGURATION_H
