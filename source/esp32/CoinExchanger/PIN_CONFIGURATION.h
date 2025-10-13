/*
 * PIN_CONFIGURATION.h
 * Pin configuration definitions for ESP32 CoinExchanger
 * 
 * This file contains all GPIO pin assignments for the coin exchanger system
 * including the ALLAN coin hopper connections.
 * 
 * Hardware Setup:
 * ALLAN Coin Hopper:
 * - Pin 1: 3.3V (Power supply)
 * - Pin 2: GPIO19 (Pulse signal input)
 * - Pin 3: GND (Ground)
 * - Pin 4: Unused
 */

#ifndef PIN_CONFIGURATION_H
#define PIN_CONFIGURATION_H

// ===== COIN HOPPER PIN DEFINITIONS =====

// ALLAN Coin Hopper - Pulse Detection Pin
#define COIN_HOPPER_PULSE_PIN    19      // GPIO19 - Connected to hopper Pin 2

// Additional GPIO pins that might be used for future expansion
#define COIN_HOPPER_ENABLE_PIN   -1      // Not used in current configuration
#define COIN_HOPPER_MOTOR_PIN    -1      // Not used in current configuration

// ===== SYSTEM STATUS LED PINS =====
#define STATUS_LED_PIN           2       // Built-in LED for status indication
#define ERROR_LED_PIN            4       // External LED for error indication

// ===== SERIAL COMMUNICATION PINS =====
// Default UART0 pins (already defined by Arduino framework)
// TX: GPIO1, RX: GPIO3

// ===== I2C PINS (for potential LCD display) =====
#define I2C_SDA_PIN             21       // I2C Serial Data
#define I2C_SCL_PIN             22       // I2C Serial Clock

// ===== SPI PINS (for potential SD card or display) =====
#define SPI_MOSI_PIN            23       // SPI Master Out Slave In
#define SPI_MISO_PIN            19       // SPI Master In Slave Out (conflicts with hopper - choose one)
#define SPI_SCK_PIN             18       // SPI Serial Clock
#define SPI_CS_PIN              5        // SPI Chip Select

// ===== ANALOG INPUT PINS =====
#define VOLTAGE_MONITOR_PIN     36       // ADC1_CH0 - Monitor supply voltage
#define CURRENT_MONITOR_PIN     39       // ADC1_CH3 - Monitor current consumption

// ===== DIGITAL INPUT PINS =====
#define MANUAL_DISPENSE_BUTTON  0        // Boot button (GPIO0) for manual dispensing
#define EMERGENCY_STOP_PIN      35       // Emergency stop input
#define DOOR_SENSOR_PIN         34       // Coin compartment door sensor

// ===== DIGITAL OUTPUT PINS =====
#define BUZZER_PIN              25       // Piezo buzzer for audio feedback
#define RELAY_OUTPUT_PIN        26       // General purpose relay output
#define COIN_GATE_SERVO_PIN     27       // Servo for coin gate control

// ===== PWM OUTPUT PINS =====
#define LED_STRIP_PIN           32       // WS2812B LED strip control
#define FAN_CONTROL_PIN         33       // PWM fan speed control

// ===== PIN VALIDATION MACROS =====
#define IS_VALID_GPIO(pin)      ((pin >= 0) && (pin <= 39) && (pin != 6) && (pin != 7) && (pin != 8) && (pin != 9) && (pin != 10) && (pin != 11))
#define IS_OUTPUT_PIN(pin)      (IS_VALID_GPIO(pin) && (pin != 34) && (pin != 35) && (pin != 36) && (pin != 39))
#define IS_INPUT_PIN(pin)       IS_VALID_GPIO(pin)

// ===== PIN USAGE NOTES =====
/*
 * ESP32 Pin Usage Guidelines:
 * 
 * Input Only Pins: 34, 35, 36, 39 (ADC1_CH6, ADC1_CH7, ADC1_CH0, ADC1_CH3)
 * - These pins can only be used as inputs
 * 
 * Boot Strapping Pins: 0, 2, 5, 12, 15
 * - GPIO0: Boot button (pulled up, boot fails if pulled low)
 * - GPIO2: Must be low during boot, connected to built-in LED
 * - GPIO5: Must be high during boot
 * - GPIO12: Boot voltage selector (low=3.3V, high=1.8V)
 * - GPIO15: Must be high during boot
 * 
 * SPI Flash Pins (DO NOT USE): 6, 7, 8, 9, 10, 11
 * - These are connected to internal SPI flash
 * 
 * UART Pins:
 * - GPIO1 (TX0), GPIO3 (RX0): Primary UART (USB serial)
 * - GPIO16 (RX2), GPIO17 (TX2): Secondary UART
 * 
 * I2C Pins (default):
 * - GPIO21 (SDA), GPIO22 (SCL)
 * 
 * SPI Pins (VSPI - default):
 * - GPIO23 (MOSI), GPIO19 (MISO), GPIO18 (SCK), GPIO5 (CS)
 * 
 * ADC2 Pins (avoid when using WiFi): 0, 2, 4, 12, 13, 14, 15, 25, 26, 27
 * - ADC2 is used by WiFi driver and cannot be used simultaneously
 */

// ===== CONFIGURATION VALIDATION =====
#if !IS_VALID_GPIO(COIN_HOPPER_PULSE_PIN)
    #error "COIN_HOPPER_PULSE_PIN must be a valid GPIO pin"
#endif

#if !IS_INPUT_PIN(COIN_HOPPER_PULSE_PIN)
    #error "COIN_HOPPER_PULSE_PIN must support input mode"
#endif

// Check for pin conflicts
#if (COIN_HOPPER_PULSE_PIN == STATUS_LED_PIN)
    #warning "Pin conflict: COIN_HOPPER_PULSE_PIN and STATUS_LED_PIN use the same GPIO"
#endif

#endif // PIN_CONFIGURATION_H