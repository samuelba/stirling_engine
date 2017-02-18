/*
 * stirling_engine.cpp
 *
 * Created: 18.02.2017
 *  Author: Sam (www.bachmann.io)
 */

#define F_CPU 16000000L
#define ARDUINO 101

#include <avr/interrupt.h>
#include <avr/io.h>

#include "stdint.h"
#include <math.h>

#include "Arduino.h"

// Prototypes
uint8_t displayValue(int Value);

// Definitions
// Button pin
#define buttonPin       13
// External interrupt pin
#define interruptPin    2
// Display pins high
#define NumberHIGH      4
uint8_t pinHIGH[NumberHIGH] = {12, 11, 10, 9};
// Display pins low
#define NumberLOW       7
uint8_t pinLOW[NumberLOW] = {17, 3, 5, 16, 8, 4, 6};
// Numbers
uint8_t Numbers[10] = {
    0b11111100, // 0
    0b01100000, // 1
    0b11011010, // 2
    0b11110010, // 3
    0b01100110, // 4
    0b10110110, // 5
    0b10111110, // 6
    0b11100000, // 7
    0b11111110, // 8
    0b11110110  // 9
};
// Special characters
uint8_t Minus = 0b00000010;
uint8_t Point = 0b00000001;

// Speed
float rotationSpeed = 0;

// Button state
bool buttonState = false;
bool lastButtonState = false;

// Serial print state
bool serialState = false;

// Measurement state
bool measurementState = false;

// Interrupt counter
int stateCounter = 0;

// Time
long double startTime = 0;
long double recentTime = 0;
long double stopTime = 0;

// Interrupt
ISR(INT0_vect) {
  if (measurementState)
    stateCounter++;
}

int main(void) {
  // Initialize
  init();

  // Serial output
  Serial.begin(9600);

  // Set pins for 7 segment display
  for (int i = 0; i < NumberHIGH; i++)
    pinMode(pinHIGH[i], OUTPUT);
  for (int i = 0; i < NumberHIGH; i++)
    digitalWrite(pinHIGH[i], LOW);
  for (int i = 0; i < NumberLOW; i++)
    pinMode(pinLOW[i], OUTPUT);
  for (int i = 0; i < NumberLOW; i++)
    digitalWrite(pinLOW[i], HIGH);
  pinMode(buttonPin, INPUT);

  // Enable interrupt
  sei();

  // Initialize interrupt pin
  pinMode(interruptPin, INPUT);
  digitalWrite(interruptPin, LOW);

  // Initialize interrupt register
  EICRA = (1 << ISC01) | (1 << ISC00); // rising edge, pin 2
  EIMSK = (1 << INT0);                 // interrupt 1

  // Start main loop
  while (1) {
    // Display value on 4 x 7 segment display
    displayValue((int)rotationSpeed);

    // Get button state
    buttonState = digitalRead(buttonPin);
    // Start/stop measurement
    if (buttonState != lastButtonState) {
      if (buttonState) {
        Serial.println("on");
        if (measurementState) {
          // Stop measurement
          measurementState = false;
          Serial.println("Measurement stop");
          stopTime = millis();
        } else {
          // Start measurement
          stateCounter = 0;
          rotationSpeed = 0;
          measurementState = true;
          Serial.println("Measurement start");
          startTime = millis();
        }
      } else {
        Serial.println("off");
      }
    }
    lastButtonState = buttonState;

    // After x rotations calculate the average rotation speed
    if (measurementState && stateCounter >= 50) {
      int lastStateCounter = stateCounter;
      stateCounter = 0; // Reset counter

      stopTime = millis();
      recentTime = stopTime - startTime;
      startTime = millis();
      // Get time in seconds
      float time = (float)recentTime / (float)1000;
      if (serialState) {
        Serial.print("Time (seconds): ");
        Serial.println(time);
      }
      // Get number of rotations
      float numberRotation = lastStateCounter * 60.0;
      if (serialState) {
        Serial.print("Number of rotations: ");
        Serial.println(numberRotation);
      }
      // Calculate rotation speed
      rotationSpeed = numberRotation / (float)time;
      if (serialState) {
        Serial.print("Rotational speed: ");
        Serial.print(rotationSpeed);
        Serial.println(" 1/s");
      }
    }
  }
  return 0;
}


/* Functions */

// Display the value on the 4 x 7 segment display
uint8_t displayValue(int Value) {
  // Reset led
  for (int i = 0; i < NumberHIGH; i++)
    digitalWrite(pinHIGH[i], LOW);
  for (int i = 0; i < NumberLOW; i++)
    digitalWrite(pinLOW[i], HIGH);

  // Set led
  for (int a = 0; a < NumberHIGH; a++) {
    // Split value
    int aValue = (Value % (int)pow(10, (a + 1))) / pow(10, a);
    // Don't display zeros
    if (a > 0 && aValue == 0 && Value < (int)pow(10, (a + 1)))
      break;

    // Display
    // Led on
    digitalWrite(pinHIGH[a], HIGH);
    for (int i = 0; i < 7; i++) {
      digitalWrite(pinLOW[i], !(Numbers[aValue] << i & 0x80));
    }
    delay(1); // Show led time
    // Led off
    digitalWrite(pinHIGH[a], LOW);
    for (int i = 0; i < 7; i++)
      digitalWrite(pinLOW[i], HIGH);

    Value -= aValue;
  }
  return 0;
}
