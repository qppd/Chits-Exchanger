#include "TACTILE_BUTTON.h"

int inputState[numOfInputs];
int lastInputState[numOfInputs] = { HIGH, HIGH };
bool inputFlags[numOfInputs] = { HIGH, HIGH };
long lastDebounceTime[numOfInputs] = { 0, 0 };
long debounceDelay = 10;
const int inputPins[numOfInputs] = { LCD_BUTTON_PIN, COIN_BUTTON_PIN };

void setInputFlags() {
  for (int i = 0; i < numOfInputs; i++) {
    int reading = digitalRead(inputPins[i]);
    if (reading != lastInputState[i]) {
      lastDebounceTime[i] = millis();
    }
    if (millis() - lastDebounceTime[i] > debounceDelay) {
      if (reading != inputState[i]) {
        inputState[i] = reading;
        if (inputState[i] == LOW) {
          inputFlags[i] = LOW;
        }
      }
    }
    lastInputState[i] = reading;
  }
}

void resolveInputFlags() {
  for (int i = 0; i < numOfInputs; i++) {
    if (inputFlags[i] == LOW) {
      inputAction(i);
      delay(10);
      inputFlags[i] = HIGH;
    }
  }
}

void inputAction(int buttonIndex) {
  switch (buttonIndex) {
    case 0:
      Serial.println("LCD Button Pressed");
      break;
    case 1:
      Serial.println("Coin Button Pressed");
      break;
    default:
      break;
  }
}
