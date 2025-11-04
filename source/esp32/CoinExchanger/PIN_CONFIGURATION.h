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

// ALLAN Coin Hoppers - Pulse Detection Pins (3 hoppers) // 4  for 5 peso, 19 for 10 peso, and 18 for 20 peso.  
#define COIN_HOPPER_1_PULSE_PIN  4      // GPIO19 - Hopper 1 pulse signal
#define COIN_HOPPER_2_PULSE_PIN  19      // GPIO18 - Hopper 2 pulse signal  
#define COIN_HOPPER_3_PULSE_PIN  18      // GPIO4  - Hopper 3 pulse signal

// ALLAN Coin Hoppers - Solid State Relay Control Pins (3 SSRs)
#define COIN_HOPPER_1_SSR_PIN    26      // GPIO26 - Hopper 1 power control (SSR)
#define COIN_HOPPER_2_SSR_PIN    33      // GPIO25 - Hopper 2 power control (SSR)
#define COIN_HOPPER_3_SSR_PIN    25      // GPIO33 - Hopper 3 power control (SSR)

// Legacy definition for backward compatibility
#define COIN_HOPPER_PULSE_PIN    COIN_HOPPER_1_PULSE_PIN

// Total number of coin hoppers
#define NUM_COIN_HOPPERS         3

// Coin Values in Philippine Pesos
#define COIN_HOPPER_1_VALUE      5       // 5 peso coins
#define COIN_HOPPER_2_VALUE      10      // 10 peso coins
#define COIN_HOPPER_3_VALUE      20      // 20 peso coins

// SSR Control Settings
#define SSR_ACTIVE_HIGH          true    // SSR is activated with HIGH signal
#define SSR_STARTUP_DELAY_MS     500     // Delay after turning on SSR before reading pulses
#define SSR_SHUTDOWN_DELAY_MS    100     // Delay before turning off SSR after counting stops

// Dispensing Settings
#define MAX_DISPENSE_AMOUNT      1000    // Maximum amount that can be dispensed (PHP)
#define DISPENSE_TIMEOUT_MS      30000   // Timeout for dispensing operations
#define COIN_DISPENSE_DELAY_MS   200     // Delay between coin dispenses

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
#define BUZZER_PIN              32       // Piezo buzzer for audio feedback (moved from 25)
#define RELAY_OUTPUT_PIN        27       // General purpose relay output (moved from 26)
#define COIN_GATE_SERVO_PIN     14       // Servo for coin gate control (moved from 27)
// NOTE: GPIO25, GPIO26, GPIO33 are now used for coin hopper SSR control

// ===== PWM OUTPUT PINS =====
#define LED_STRIP_PIN           13       // WS2812B LED strip control (moved from 32)
#define FAN_CONTROL_PIN         12       // PWM fan speed control (moved from 33)
// NOTE: GPIO32 moved to BUZZER_PIN, GPIO33 now used for coin hopper SSR control

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

// Check SSR pin conflicts
#if (COIN_HOPPER_1_SSR_PIN == COIN_HOPPER_2_SSR_PIN) || (COIN_HOPPER_1_SSR_PIN == COIN_HOPPER_3_SSR_PIN) || (COIN_HOPPER_2_SSR_PIN == COIN_HOPPER_3_SSR_PIN)
    #error "SSR pin conflict: Each hopper must have a unique SSR control pin"
#endif

#if !IS_OUTPUT_PIN(COIN_HOPPER_1_SSR_PIN) || !IS_OUTPUT_PIN(COIN_HOPPER_2_SSR_PIN) || !IS_OUTPUT_PIN(COIN_HOPPER_3_SSR_PIN)
    #error "SSR pins must support output mode"
#endif

#endif // PIN_CONFIGURATION_H
