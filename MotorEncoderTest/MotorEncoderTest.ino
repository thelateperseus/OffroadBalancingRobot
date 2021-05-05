#include "Encoder.h"

const int ENCODER_A_L = 9;
const int ENCODER_B_L = 10;
const int ENCODER_A_R = 11;
const int ENCODER_B_R = 13;

const int INA_L = A0;
const int INB_L = A1;
const int PWM_L = 5;
const int INA_R = A2;
const int INB_R = A3;
const int PWM_R = 6;

const int MOTOR_SPEEDS[] = { 16, 20, 24, 32, 48, 64, 96, 128, 160, 192, 224, 255 };
const int MOTOR_SPEEDS_LENGTH = 12;
const int LOOP_MICROS = 1000000;

long loopEndTime = 0;
long measurementTime = 0;
int speedIndex = 0;
int speedLoopCount = 0;

Encoder encoderLeft(ENCODER_A_L, ENCODER_B_L);
Encoder encoderRight(ENCODER_A_R, ENCODER_B_R);

void encoderLeftInterruptA() {
  encoderLeft.encoderInterruptA();
}

void encoderLeftInterruptB() {
  encoderLeft.encoderInterruptB();
}

void encoderRightInterruptA() {
  encoderRight.encoderInterruptA();
}

void encoderRightInterruptB() {
  encoderRight.encoderInterruptB();
}

void setup() {
  pinMode(INA_L, OUTPUT);
  pinMode(INB_L, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(INA_R, OUTPUT);
  pinMode(INB_R, OUTPUT);
  pinMode(PWM_R, OUTPUT);

  Serial.begin(2000000);
  while (!Serial);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_L), encoderLeftInterruptA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_L), encoderLeftInterruptB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_R), encoderRightInterruptA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_R), encoderRightInterruptB, CHANGE);

  measurementTime = micros();
  loopEndTime = micros() + LOOP_MICROS;
}

void loop() {
  if (speedIndex >= MOTOR_SPEEDS_LENGTH) {
    analogWrite(PWM_L, 0);
    analogWrite(PWM_R, 0);
    while (true);
  }

  int speed = 40; //MOTOR_SPEEDS[speedIndex];
  /*speedLoopCount++;
  if (speedLoopCount >= 10) {
    speedIndex++;
    speedLoopCount = 0;
  }*/

  //control speed 
  analogWrite(PWM_L, speed);
  //control direction 
  digitalWrite(INA_L, LOW);
  digitalWrite(INB_L, HIGH);

  //control speed
  analogWrite(PWM_R, speed);
  //control direction
  digitalWrite(INA_R, HIGH);
  digitalWrite(INB_R, LOW);

  int valueLeft = encoderLeft.getDeltaValue();
  int valueRight = -encoderRight.getDeltaValue();

  //long measurementInterval = now - measurementTime;
  //measurementTime = now;

  Serial.print(speed);
  Serial.print(",");
  Serial.print(valueLeft);
  Serial.print(",");
  Serial.print(valueRight);
  Serial.println();

  while(loopEndTime > micros());
  loopEndTime += LOOP_MICROS;
}
