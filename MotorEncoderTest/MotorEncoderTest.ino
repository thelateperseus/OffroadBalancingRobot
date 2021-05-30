#include <SAMD21turboPWM.h>
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

//const int MOTOR_SPEEDS[] = { 16, 20, 24, 32, 48, 64, 96, 128, 160, 192, 224, 255 };
//const int MOTOR_SPEEDS_LENGTH = 12;
//const int MOTOR_SPEEDS[] = { 8, 12, 16, 20, 24, 32 };
const int MOTOR_SPEEDS[] = { 7, 8, 9, 10, 16, 20, 24, 32 };
const int MOTOR_SPEEDS_LENGTH = 8;
const int LOOP_MICROS = 1000000;

long loopEndTime = 0;
long measurementTime = 0;
int speedIndex = 0;
int speedLoopCount = 0;

TurboPWM pwm;
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

  // Turbo = true
  pwm.setClockDivider(1, false);
  // For the Arduino Nano 33 IoT, you need to initialise timer 1 for pins 4 and 7, timer 0 for pins 5, 6, 8, and 12, and timer 2 for pins 11 and 13;
  // timer 1, prescaler 1, resolution 4800, fast PWM
  pwm.timer(0, 1, 1200, false);
  Serial.println(pwm.frequency(0));
  //analogWriteResolution(10);

  measurementTime = micros();
  loopEndTime = micros() + LOOP_MICROS;
}

void loop() {
  if (speedIndex >= MOTOR_SPEEDS_LENGTH) {
    pwm.analogWrite(PWM_L, 0);
    pwm.analogWrite(PWM_R, 0);
    while (true);
  }

  // Deadband left: 11-12, right: 9-10
  int speed = 12;
  //int speed = MOTOR_SPEEDS[speedIndex];
  speedLoopCount++;
  if (speedLoopCount >= 10) {
    speedIndex++;
    speedLoopCount = 0;
  }

  int speedLeft = speed;
  int speedRight = speed;

  //control speed 
  pwm.analogWrite(PWM_L, abs(speedLeft));
  //control direction 
  digitalWrite(INA_L, speedLeft > 0 ? LOW : HIGH);
  digitalWrite(INB_L, speedLeft > 0 ? HIGH : LOW);

  //control speed
  pwm.analogWrite(PWM_R, abs(speedRight));
  //control direction
  digitalWrite(INA_R, speedRight > 0 ? HIGH : LOW);
  digitalWrite(INB_R, speedRight > 0 ? LOW : HIGH);

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
