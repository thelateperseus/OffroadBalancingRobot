#include "Encoder.h"

const int ENCODER_A_PIN = 2;
const int ENCODER_B_PIN = 3;

const int IN1_PIN = 5;
const int IN2_PIN = 7;
const int PWM_PIN = 6;

const int MOTOR_SPEEDS[] = { 12, 16, 24, 32, 48, 64, 96, 128, 160, 192, 224, 255 };
const int MOTOR_SPEEDS_LENGTH = 12;
const int LOOP_MICROS = 1000000;

long loopEndTime = 0;
long measurementTime = 0;
int speedIndex = 0;
int speedLoopCount = 0;

Encoder encoder(ENCODER_A_PIN, ENCODER_B_PIN);

void encoderInterruptA() {
  encoder.encoderInterruptA();
}

void encoderInterruptB() {
  encoder.encoderInterruptB();
}

void setup() {
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);

  Serial.begin(2000000);
  while (!Serial);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderInterruptA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), encoderInterruptB, CHANGE);

  measurementTime = micros();
  loopEndTime = micros() + LOOP_MICROS;
}

void loop() {
  if (speedIndex >= MOTOR_SPEEDS_LENGTH) {
    analogWrite(PWM_PIN, 0);
    while (true);
  }

  int speed = MOTOR_SPEEDS[speedIndex];
  speedLoopCount++;
  if (speedLoopCount >= 10) {
    speedIndex++;
    speedLoopCount = 0;
  }

  //control speed 
  analogWrite(PWM_PIN, speed);
  //control direction 
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);

  int value = encoder.getDeltaValue();

  //long measurementInterval = now - measurementTime;
  //measurementTime = now;

  Serial.print(speed);
  Serial.print(",");
  Serial.println(value);

  while(loopEndTime > micros());
  loopEndTime += LOOP_MICROS;
}
