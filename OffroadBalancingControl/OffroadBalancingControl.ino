#include "Encoder.h"
#include "IMUFilter.h"
#include "PID_v1.h"
#include <SAMD21turboPWM.h>
#include "SerialCommand.h"
#include "SpeedFilter.h"

/*---------
    More constants than physics
  ---------*/

const int LOOP_MICROS = 4000;
long loopEndTime = 0;

const int ENCODER_A_L = 9;
const int ENCODER_B_L = 10;
const int ENCODER_A_R = 11;
const int ENCODER_B_R = A1;

const int INA_L = 7;
const int INB_L = 8;
const int PWM_L = 5;
const int INA_R = A2;
const int INB_R = A3;
const int PWM_R = 6;

const int RC_CHANNEL1_PIN = 2;
const int RC_CHANNEL2_PIN = 3;

boolean started = false;

double motorDeadBand = 56;

long encoderLeftValue;
long encoderRightValue;

double kps = 0.25; // 0.3
double kis = 0.18; // 0.15
double kds = 0;
double speedSetPoint = 0;
double filteredSpeed = 0;
double speedPidOutput = 0;
PID speedPid(&filteredSpeed, &speedPidOutput, &speedSetPoint, kps, kis, kds, DIRECT);

double kpp = 40;
double kip = 90;
double kdp = 2; // TODO 3
double pitchSetPoint = 0;
double pitchReading = 0;
double pitchPidOutput = 0;
PID pitchPid(&pitchReading, &pitchPidOutput, &pitchSetPoint, kpp, kip, kdp, REVERSE);

double kpd = 0.2;
double kid = 0;
double kdd = 0;
double directionSetPoint = 0;
double directionReading = 0;
double directionPidOutput = 0;
PID directionPid(&directionReading, &directionPidOutput, &directionSetPoint, kpd, kid, kdd, DIRECT);

TurboPWM pwm;
Encoder encoderLeft(ENCODER_A_L, ENCODER_B_L);
Encoder encoderRight(ENCODER_A_R, ENCODER_B_R);
SpeedFilter speedFilter;

SerialCommand cmd;

IMUData imuData;
IMUFilter imuFilter;

volatile unsigned long rcChannel1PulseStart = 0;
volatile long rcChannel1PulseDuration = 0;
volatile unsigned long rcChannel2PulseStart = 0;
volatile long rcChannel2PulseDuration = 0;

/*---------
    Setup
  ---------*/

void setKpp() {
  char* arg = cmd.next();
  if (arg != NULL) {
    kpp = atof(arg);
    pitchPid.SetTunings(kpp, kip, kdp);
  }
}

void setKip() {
  char* arg = cmd.next();
  if (arg != NULL) {
    kip = atof(arg);
    pitchPid.SetTunings(kpp, kip, kdp);
  }
}

void setKdp() {
  char* arg = cmd.next();
  if (arg != NULL) {
    kdp = atof(arg);
    pitchPid.SetTunings(kpp, kip, kdp);
  }
}

void setKps() {
  char* arg = cmd.next();
  if (arg != NULL) {
    kps = atof(arg);
    speedPid.SetTunings(kps, kis, kds);
  }
}

void setKis() {
  char* arg = cmd.next();
  if (arg != NULL) {
    kis = atof(arg);
    speedPid.SetTunings(kps, kis, kds);
  }
}

void setKds() {
  char* arg = cmd.next();
  if (arg != NULL) {
    kds = atof(arg);
    speedPid.SetTunings(kps, kis, kds);
  }
}

void setDeadband() {
  char* arg = cmd.next();
  if (arg != NULL) {
    motorDeadBand = atof(arg);
  }
}

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

void rcChannel1Interrupt() {
  boolean rcChannel1 = digitalRead(RC_CHANNEL1_PIN);
  unsigned long now = micros();
  if (rcChannel1) {
    rcChannel1PulseStart = now;
  } else {
    rcChannel1PulseDuration = now - rcChannel1PulseStart;
  }
}

void rcChannel2Interrupt() {
  boolean rcChannel2 = digitalRead(RC_CHANNEL2_PIN);
  unsigned long now = micros();
  if (rcChannel2) {
    rcChannel2PulseStart = now;
  } else {
    rcChannel2PulseDuration = now - rcChannel2PulseStart;
  }
}

void setup() {
  //Serial.begin(115200);
  //while (!Serial); // Nano 33 will lose initial output without this
  Serial1.begin(115200);

  cmd.addCommand("kpp", setKpp);
  cmd.addCommand("kip", setKip);
  cmd.addCommand("kdp", setKdp);
  cmd.addCommand("kps", setKps);
  cmd.addCommand("kis", setKis);
  cmd.addCommand("kds", setKds);
  cmd.addCommand("db", setDeadband);

  //Serial.println("Initializing IMU...");
  imuFilter.initialise();

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(INA_L, OUTPUT);
  pinMode(INB_L, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(INA_R, OUTPUT);
  pinMode(INB_R, OUTPUT);
  pinMode(PWM_R, OUTPUT);

  pinMode(ENCODER_A_L, INPUT);
  pinMode(ENCODER_B_L, INPUT);
  pinMode(ENCODER_A_R, INPUT);
  pinMode(ENCODER_B_R, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_L), encoderLeftInterruptA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_L), encoderLeftInterruptB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_R), encoderRightInterruptA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_R), encoderRightInterruptB, CHANGE);

  pinMode(RC_CHANNEL1_PIN, INPUT);
  pinMode(RC_CHANNEL2_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RC_CHANNEL1_PIN), rcChannel1Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_CHANNEL2_PIN), rcChannel2Interrupt, CHANGE);

  // Turbo = false
  pwm.setClockDivider(1, false);
  // For the Arduino Nano 33 IoT, you need to initialise timer 1 for pins 4 and 7, timer 0 for pins 5, 6, 8, and 12, and timer 2 for pins 11 and 13;
  // timer 1, prescaler 1, resolution 1200, no fast PWM
  pwm.timer(0, 1, 1200, false);
  pwm.analogWrite(PWM_L, 0);
  pwm.analogWrite(PWM_R, 0);

  speedPid.SetSampleTime(LOOP_MICROS / 1000);
  speedPid.SetOutputLimits(-15, 15);
  pitchPid.SetSampleTime(LOOP_MICROS / 1000);
  pitchPid.SetOutputLimits(-1000, 1000);
  directionPid.SetSampleTime(LOOP_MICROS / 1000);
  directionPid.SetOutputLimits(-30, 30);

  reset();

  digitalWrite(LED_BUILTIN, HIGH);
  loopEndTime = micros() + LOOP_MICROS;
}

/*---------
    Loop
  ---------*/

void reset() {
  speedPid.SetMode(MANUAL);
  speedSetPoint = 0;
  filteredSpeed = 0;
  speedPidOutput = 0;
  speedFilter.reset();

  pitchPid.SetMode(MANUAL);
  pitchSetPoint = 0;
  pitchReading = 0;
  pitchPidOutput = 0;

  directionPid.SetMode(MANUAL);
  directionSetPoint = 0;
  directionReading = 0;
  directionPidOutput = 0;

  started = false;
}

void loop() {
  cmd.readSerial(&Serial1);

  imuFilter.getFilteredAngle(imuData);

  // Start balancing when angle is close to zero
  if (!started && abs(imuData.pitchAccelerometer) < 1.0) {
    reset();
    imuFilter.resetToAccelerometerAngle();
    started = true;
    encoderLeftValue = 0;
    encoderRightValue = 0;
    encoderLeft.resetValue();
    encoderRight.resetValue();
    speedPid.SetMode(AUTOMATIC);
    pitchPid.SetMode(AUTOMATIC);
    directionPid.SetMode(AUTOMATIC);
  }

  if (started) {
    // Calculate speed from the encoder readings
    long newEncoderLeftValue = encoderLeft.getValue();
    long newEncoderRightValue = -encoderRight.getValue();
    long speedLeft = newEncoderLeftValue - encoderLeftValue;
    long speedRight = newEncoderRightValue - encoderRightValue;
    double overallSpeed = (speedLeft + speedRight) / 2.0;
    filteredSpeed = speedFilter.step(overallSpeed);
    encoderLeftValue = newEncoderLeftValue;
    encoderRightValue = newEncoderRightValue;

    // Use RC controller to set speed
    if (rcChannel1PulseDuration > 100 && 
        (rcChannel1PulseDuration < 1460 || rcChannel1PulseDuration > 1560)) {
      long rcSpeed = (rcChannel1PulseDuration - 1510) / 80;
      speedSetPoint = constrain(rcSpeed, -6, 6);
    } else {
      speedSetPoint = 0;
    }

    //  Set the desired angle based on the error in the speed
    speedPid.Compute();
    pitchSetPoint = speedPidOutput;

    pitchReading = imuData.pitchReading;
    pitchPid.Compute();
    double pwmLeft = pitchPidOutput;
    double pwmRight = pitchPidOutput;

    // Calculate direction from the encoder readings and apply direction control to keep the robot straight
    directionReading = encoderLeftValue - encoderRightValue;

    // Use RC controller to turn
    if (rcChannel2PulseDuration > 100 &&
        (rcChannel2PulseDuration < 1460 || rcChannel2PulseDuration > 1560)) {
      directionPid.SetMode(MANUAL);
      directionSetPoint = directionReading;
      directionPidOutput = 0;
      long steering = (rcChannel2PulseDuration - 1510) / 5;
      steering = constrain(steering, -100, 100);
      pwmLeft -= steering;
      pwmRight += steering;
    } else {
      directionPid.SetMode(AUTOMATIC);
      directionPid.Compute();
      pwmLeft += directionPidOutput;
      pwmRight -= directionPidOutput;
    }

    boolean directionLeft = pwmLeft > 0;
    boolean directionRight = pwmRight > 0;
    pwmLeft = map(abs(pwmLeft), 0.0, 1000.0, motorDeadBand + 1, 1000.0);
    pwmRight = map(abs(pwmRight), 0.0, 1000.0, motorDeadBand, 1000.0);

    Serial1.print("sp:");
    Serial1.print(pitchSetPoint);
    Serial1.print(", pv:");
    Serial1.print(pitchReading);
    /*Serial1.print(", o:");
      Serial1.print(pitchPidOutput);
      Serial1.print(", s:");
      Serial1.print(pwmLeft);*/
    /*Serial1.print(", d:");
      Serial1.print(directionReading);*/
    Serial1.print(", ssp:");
    Serial1.print(speedSetPoint);
    Serial1.print(", s:");
    Serial1.print(overallSpeed);
    Serial1.print(", fs:");
    Serial1.print(filteredSpeed);
    Serial1.println();

    if (pitchReading > 45 || pitchReading < -45) {
      pwmLeft = 0;
      pwmRight = 0;
      reset();
    }

    pwmLeft = constrain(pwmLeft, -1000, 1000);
    pwmRight = constrain(pwmRight, -1000, 1000);

    pwm.analogWrite(PWM_L, pwmLeft);
    digitalWrite(INA_L, directionLeft ? LOW : HIGH);
    digitalWrite(INB_L, directionLeft ? HIGH : LOW);
    pwm.analogWrite(PWM_R, pwmRight);
    digitalWrite(INA_R, directionRight > 0 ? HIGH : LOW);
    digitalWrite(INB_R, directionRight > 0 ? LOW : HIGH);

  } else {
    pwm.analogWrite(PWM_L, 0);
    digitalWrite(INA_L, LOW);
    digitalWrite(INB_L, LOW);
    pwm.analogWrite(PWM_R, 0);
    digitalWrite(INA_R, LOW);
    digitalWrite(INB_R, LOW);
  }

  while (loopEndTime > micros());
  loopEndTime = micros() + LOOP_MICROS;
}
