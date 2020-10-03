#include "Encoder.h"
#include <Arduino.h>

Encoder::Encoder(int argPinA, int argPinB) {
  pinA = argPinA;
  pinB = argPinB;
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
}

// Forward (positive stepCount)
//             _______
// Output A __|       |______
//                 _______
// Output B ______|       |__

// Backward (negative stepCount)
//                 _______
// Output A ______|       |__
//             _______
// Output B __|       |______

void Encoder::encoderInterruptA() {
  boolean outputA = digitalRead(pinA);
  boolean outputB = digitalRead(pinB);
  if (outputA == outputB) {
    value++;
  } else {
    value--;
  }
}

void Encoder::encoderInterruptB() {
  boolean outputA = digitalRead(pinA);
  boolean outputB = digitalRead(pinB);
  if (outputA == outputB) {
    value--;
  } else {
    value++;
  }
}

// Gets the value on the encoder.  It's an integer, positive or negative.
signed int Encoder::getValue() {
  noInterrupts();
  int value2 = value;
  interrupts();
  return value2;
}

// Gets the in/de-crement of value on the encoder since last read.  It's an integer, positive or negative.
signed int Encoder::getDeltaValue() {
  noInterrupts();
  int delta = value - lastReadValue;
  lastReadValue = value;
  interrupts();
  return delta;
}

void Encoder::resetValue() {
  noInterrupts();
  value = 0;
  lastReadValue = 0;
  interrupts();
}
