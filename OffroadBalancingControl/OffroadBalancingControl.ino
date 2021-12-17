#include "Encoder.h"
#include "IMUFilter.h"
#include "PID_v1.h"
#include "RCChannel.h"
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

const int RC_CHANNEL1_PIN = 3;
const int RC_CHANNEL2_PIN = 2;

boolean started = false;

double motorDeadBand = 56;

long encoderLeftValue;
long encoderRightValue;

double kps = 0.3; // 0.3
double kis = 0.15; // 0.15
double kds = 0;
double speedSetPoint = 0;
double filteredSpeed = 0;
double speedPidOutput = 0;
PID speedPid(&filteredSpeed, &speedPidOutput, &speedSetPoint, kps, kis, kds, DIRECT);

double kpp = 40;
double kip = 90;
double kdp = 2;
double pitchSetPoint = 0;
double pitchReading = 0;
double pitchPidOutput = 0;
PID pitchPid(&pitchReading, &pitchPidOutput, &pitchSetPoint, kpp, kip, kdp, REVERSE);

double kpd = 0.4;
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

SerialCommand bluetoothCmd;
SerialCommand serialCmd;

IMUData imuData;
IMUFilter imuFilter;

RCChannel rcChannel1 = RCChannel(RC_CHANNEL1_PIN);
RCChannel rcChannel2 = RCChannel(RC_CHANNEL2_PIN);

volatile double serialSpeed;
volatile double serialSteering;
unsigned long serialCommandExpiryTime;

/*---------
    Setup
  ---------*/

void setKpp() {
  char* arg = bluetoothCmd.next();
  if (arg != NULL) {
    kpp = atof(arg);
    pitchPid.SetTunings(kpp, kip, kdp);
  }
}

void setKip() {
  char* arg = bluetoothCmd.next();
  if (arg != NULL) {
    kip = atof(arg);
    pitchPid.SetTunings(kpp, kip, kdp);
  }
}

void setKdp() {
  char* arg = bluetoothCmd.next();
  if (arg != NULL) {
    kdp = atof(arg);
    pitchPid.SetTunings(kpp, kip, kdp);
  }
}

void setKps() {
  char* arg = bluetoothCmd.next();
  if (arg != NULL) {
    kps = atof(arg);
    speedPid.SetTunings(kps, kis, kds);
  }
}

void setKis() {
  char* arg = bluetoothCmd.next();
  if (arg != NULL) {
    kis = atof(arg);
    speedPid.SetTunings(kps, kis, kds);
  }
}

void setKds() {
  char* arg = bluetoothCmd.next();
  if (arg != NULL) {
    kds = atof(arg);
    speedPid.SetTunings(kps, kis, kds);
  }
}

void setDeadband() {
  char* arg = bluetoothCmd.next();
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
  rcChannel1.handleInterrupt();
}

void rcChannel2Interrupt() {
  rcChannel2.handleInterrupt();
}

void receiveSerialCommand(const char* arg1) {
  if (arg1 != NULL) {
    char* arg2 = serialCmd.next();
    if (arg2 != NULL) {
      serialSpeed = constrain(atof(arg1), -5, 5);
      serialSteering = constrain(atof(arg2), -40, 40);
      serialCommandExpiryTime = millis() + 300;
    }
  }
}

void setup() {
  Serial.begin(115200);
  //while (!Serial); // Nano 33 will lose initial output without this
  Serial1.begin(115200);

  bluetoothCmd.addCommand("kpp", setKpp);
  bluetoothCmd.addCommand("kip", setKip);
  bluetoothCmd.addCommand("kdp", setKdp);
  bluetoothCmd.addCommand("kps", setKps);
  bluetoothCmd.addCommand("kis", setKis);
  bluetoothCmd.addCommand("kds", setKds);
  bluetoothCmd.addCommand("db", setDeadband);

  serialCmd.setDefaultHandler(receiveSerialCommand);

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
  serialCmd.readSerial(&Serial);
  bluetoothCmd.readSerial(&Serial1);

  if (serialCommandExpiryTime > 0 && millis() > serialCommandExpiryTime) {
    serialSpeed = 0;
    serialSteering = 0;
    serialCommandExpiryTime = 0;
  }

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
    long rcChannel1PulseDuration = rcChannel1.getPulseDuration();
    if (rcChannel1PulseDuration > 100 &&
        (rcChannel1PulseDuration < 1450 || rcChannel1PulseDuration > 1550)) {
      long rcSpeed = (rcChannel1PulseDuration - 1500) / 32;
      speedSetPoint = constrain(rcSpeed, -15, 15);
    } else if (serialSpeed != 0) {
      speedSetPoint = constrain(serialSpeed, -5, 5);
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
    /*Serial1.print("rc1:");
      Serial1.print(rcChannel1PulseDuration);
      Serial1.print(", rc2:");
      Serial1.print(rcChannel2PulseDuration);
      Serial1.print(", sp:");
      Serial1.print(directionSetPoint);
      Serial1.print(", d:");
      Serial1.print(directionReading);
      Serial1.print(", ");*/

    // Use RC controller or Serial command from Pi to turn
    long steering = 0;
    long rcChannel2PulseDuration = rcChannel2.getPulseDuration();
    if (rcChannel2PulseDuration > 100 &&
        (rcChannel2PulseDuration < 1450 || rcChannel2PulseDuration > 1550)) {
      steering = (rcChannel2PulseDuration - 1500) / 4;
      steering = constrain(steering, -120, 120);
    } else if (serialSteering != 0) {
      steering = constrain(serialSteering, -60, 60);
    }

    if (steering != 0) {
      directionPid.SetMode(MANUAL);
      directionSetPoint = directionReading;
      directionPidOutput = 0;
      pwmLeft -= steering;
      pwmRight += steering;
      Serial1.print("st:");
      Serial1.print(steering);
      // Serial1.print(", ");
    } else {
      directionPid.SetMode(AUTOMATIC);
      directionPid.Compute();
      pwmLeft += directionPidOutput;
      pwmRight -= directionPidOutput;
      Serial1.print("st:");
      Serial1.print(directionPidOutput);
      // Serial1.print(", ");
    }

    boolean directionLeft = pwmLeft > 0;
    boolean directionRight = pwmRight > 0;
    pwmLeft = map(abs(pwmLeft), 0.0, 1000.0, motorDeadBand + 1, 1000.0);
    pwmRight = map(abs(pwmRight), 0.0, 1000.0, motorDeadBand, 1000.0);

    //    Serial1.print("sp:");
    //    Serial1.print(pitchSetPoint);
    //    Serial1.print(", pv:");
    //    Serial1.print(pitchReading);
    Serial1.print(", ssp:");
    Serial1.print(speedSetPoint);
    //    Serial1.print(", s:");
    //    Serial1.print(overallSpeed);
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
