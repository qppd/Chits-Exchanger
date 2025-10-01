# IoT Chits Exchanger

## Overview
The IoT Chits Exchanger is a smart system designed to automate the process of exchanging coins and bills for chits. This system is built using an ESP32 microcontroller and integrates various peripherals to provide a seamless user experience. The system is ideal for applications such as vending machines, ticket dispensers, and other automated kiosks.

## Features
- **Coin Slot**: Detects coin insertion using interrupt-based pulse detection with debounce logic.
- **Bill Acceptor**: Handles bill insertion with debounce logic to ensure accurate detection.
- **Servo Dispenser**: Dispenses chits using a servo motor controlled by the PCA9685 driver.
- **I2C LCD Display**: A 20x4 LCD screen displays messages and instructions to the user.
- **Tactile Buttons**: Provides user input options with debounce logic for reliability.
- **Coin Hopper**: Dispenses coins based on user input.
- **Integration with Arduino Libraries**: Utilizes libraries like Adafruit PWM Servo Driver and LiquidCrystal_I2C for efficient hardware control.

## Hardware Components
- **ESP32 Microcontroller**: The brain of the system, responsible for controlling all peripherals.
- **Coin Slot**: Detects and processes coin inputs.
- **Bill Acceptor**: Detects and processes bill inputs.
- **Servo Motor**: Dispenses chits.
- **PCA9685 Driver**: Controls the servo motor.
- **20x4 I2C LCD**: Displays messages and instructions.
- **Tactile Buttons**: Allows user interaction.
- **Coin Hopper**: Dispenses coins based on user input.
- **Power Supply**: Provides power to all components.
- **Wires and Connectors**: For connecting all components securely.
- **Resistors and Capacitors**: For signal conditioning and stability.
- **Piezo Buzzer**: Provides audio feedback for user actions and system events.

## Software Components
- **Interrupt Service Routines (ISRs)**: Handle real-time events like coin and bill insertion.
- **Debounce Logic**: Ensures reliable detection of user inputs.
- **LCD Control**: Manages the display of messages on the LCD screen.
- **Servo Control**: Handles the movement of the servo motor for chit dispensing.
- **Coin Hopper Control**: Manages the dispensing of coins.
- **Pin Configuration**: Centralized file for managing pin assignments.

## Progress
### Completed
- Coin slot functionality with debounce logic.
- Bill acceptor integration using TB74 pulse logic.
- Servo dispenser control using PCA9685.
- 20x4 I2C LCD configuration for message display.
- Tactile button handling with debounce logic.
- Coin hopper functionality for dispensing coins.

### In Progress
- Integration testing of all components.
- Code optimization for performance and reliability.

### Not Yet Done
- Database implementation for transaction logging.
- Detailed user manual creation.

## Setup Instructions
### Prerequisites
- Arduino IDE installed on your computer.
- Required libraries installed in the Arduino IDE:
  - Adafruit PWM Servo Driver
  - LiquidCrystal_I2C

### Steps
1. **Clone the Repository**:
   ```bash
   git clone https://github.com/qppd/Chits-Exchanger.git
   ```
2. **Open the Project**:
   Open the project folder in the Arduino IDE.
3. **Install Libraries**:
   Install the required libraries from the Arduino Library Manager.
4. **Connect Hardware**:
   - Follow the pin configuration in `PIN_CONFIGURATION.h` to connect the hardware components.
   - Ensure all connections are secure.
5. **Upload Code**:
   - Select the correct board (ESP32) and port in the Arduino IDE.
   - Upload the code to the ESP32.
6. **Test Components**:
   - Test each component individually to ensure proper functionality.
   - Use the serial monitor for debugging.
7. **Run the System**:
   - Once all components are tested, run the full system.

## Pin Configuration
The pin configuration is defined in `PIN_CONFIGURATION.h`. Below is a summary:
- **Coin Slot**: Pin 27
- **Bill Acceptor**: Pin 26
- **Servo Dispenser**: Pins 21 (SDA) and 22 (SCL)
- **LCD Button**: Pin 8
- **Coin Button**: Pin 9
- **Coin Hopper Motor**: Pin 10
- **Coin Hopper Sensor**: Pin 11

## Contributing
Contributions are welcome! To contribute:
1. Fork this repository.
2. Create a new branch for your feature or bug fix.
3. Submit a pull request with a detailed description of your changes.

## License
This project is licensed under the MIT License. See the `LICENSE` file for details.

## Contact
For questions or support, please contact the repository owner

## Future Enhancements
- **Database Integration**: Add a database to log transactions.
