#include "Encoder.h"
#include "LSM6DS3.h"
#include "PID_v1.h"
#include <SAMD21turboPWM.h>
#include "SerialCommand.h"

/*---------
    Encoder Speed Filter
  ---------*/

// Low pass butterworth filter order=2 alpha1=0.016
class  FilterBuLp2
{
  public:
    FilterBuLp2()
    {
      this -> reset();
    }
  private:
    double v[3];
  public:
    double step(double x) //class II
    {
      v[0] = v[1];
      v[1] = v[2];
      v[2] = (2.357208772852337209e-3 * x)
             + (-0.86747213379166820957 * v[0])
             + (1.85804329870025886073 * v[1]);
      return
        (v[0] + v[2])
        + 2 * v[1];
    }

    void reset()
    {
      v[0] = 0.0;
      v[1] = 0.0;
      v[2] = 0.0;
    }
};

/*---------
    More constants than physics
  ---------*/

const int LOOP_MICROS = 4000;
long loopEndTime = 0;
long lastImuMesaurementMicros = 0;

const float RADIANS_TO_DEGREES = 57.296;

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

struct IMUData {
  float gyroXCalibration = 0;
  float gyroYCalibration = 0;
  float gyroZCalibration = 0;
  float accelerometerXCalibration = 0;
  float accelerometerYCalibration = 0;
  float accelerometerZCalibration = 0;
  float pitchAccelerometer = 0;
  float pitchReading = 0;
};

IMUData imuData;

boolean started = false;

const double PITCH_OFFSET = 0.4;
double motorDeadBand = 56;

long encoderLeftValue;
long encoderRightValue;

double kps = 0.2;
double kis = 0.1;
double kds = 0;
double speedSetPoint = 0;
double filteredSpeed = 0;
double speedPidOutput = 0;
PID speedPid(&filteredSpeed, &speedPidOutput, &speedSetPoint, kps, kis, kds, DIRECT);

double kpp = 30; // 50
double kip = 120; // 150
double kdp = 0.5;
double pitchSetPoint = PITCH_OFFSET;
double pitchReading = 0;
double pitchPidOutput = 0;
PID pitchPid(&pitchReading, &pitchPidOutput, &pitchSetPoint, kpp, kip, kdp, REVERSE);

double kpd = 0.1;
double kid = 0;
double kdd = 0;
double directionSetPoint = 0;
double directionReading = 0;
double directionPidOutput = 0;
PID directionPid(&directionReading, &directionPidOutput, &directionSetPoint, kpd, kid, kdd, DIRECT);

TurboPWM pwm;
Encoder encoderLeft(ENCODER_A_L, ENCODER_B_L);
Encoder encoderRight(ENCODER_A_R, ENCODER_B_R);
FilterBuLp2 speedFilter;

SerialCommand cmd;

volatile unsigned long rcChannel1PulseStart = 0;
volatile long rcChannel1PulseDuration = 0;
volatile unsigned long rcChannel2PulseStart = 0;
volatile long rcChannel2PulseDuration = 0;

/*---------
    Setup
  ---------*/

void calibrateIMU() {
  //Serial.println("Measuring internal IMU calibration values...");
  float gxSum = 0;
  float gySum = 0;
  float gzSum = 0;
  float axSum = 0;
  float aySum = 0;
  float azSum = 0;
  for (int i = 0; i < 500; i++) {
    if (i % 15 == 0) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));        //Change the state of the LED every 15 loops to make the LED blink fast
    }

    float gx, gy, gz;
    IMU.readGyroscope(gx, gy, gz);
    gxSum += gx;
    gySum += gy;
    gzSum += gz;

    float ax, ay, az;
    IMU.readAcceleration(ax, ay, az);
    axSum += ax;
    aySum += ay;
    azSum += az;

    while (!IMU.accelerationAvailable() && !IMU.gyroscopeAvailable());
  }
  lastImuMesaurementMicros = micros();
  // Get the average from the 500 readings
  imuData.gyroXCalibration = gxSum / 500;
  imuData.gyroYCalibration = gySum / 500;
  imuData.gyroZCalibration = gzSum / 500;
  imuData.accelerometerXCalibration = axSum / 500;
  imuData.accelerometerYCalibration = aySum / 500;
  imuData.accelerometerZCalibration = azSum / 500;

  // Gyro calibration complete, gy: 275, ax: -138, ay: -286, az: 18227
  /*Serial.print("Internal IMU calibration complete, gx: ");
    Serial.print(imuData.gyroXCalibration);
    Serial.print(", gy: ");
    Serial.print(imuData.gyroYCalibration);
    Serial.print(", gz: ");
    Serial.print(imuData.gyroZCalibration);
    Serial.print(", ax: ");
    Serial.print(imuData.accelerometerXCalibration);
    Serial.print(", ay: ");
    Serial.print(imuData.accelerometerYCalibration);
    Serial.print(", az: ");
    Serial.print(imuData.accelerometerZCalibration);
    Serial.println();*/

  // 12:39:01.706 -> Internal IMU calibration complete, gx: 0.91, gy: -3.77, gz: -4.53, ax: 0.03, ay: 0.01, az: 0.99
}

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

  if (!IMU.begin()) {
    //Serial.println("Failed to initialize internal IMU!");
    while (1) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(500);
    }
  }

  calibrateIMU();

  speedPid.SetSampleTime(LOOP_MICROS / 1000);
  speedPid.SetOutputLimits(-15, 15);
  pitchPid.SetSampleTime(LOOP_MICROS / 1000);
  pitchPid.SetOutputLimits(-1000, 1000);
  directionPid.SetSampleTime(LOOP_MICROS / 1000);
  directionPid.SetOutputLimits(-15, 15);

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
  pitchSetPoint = PITCH_OFFSET;
  pitchReading = 0;
  pitchPidOutput = 0;

  directionPid.SetMode(MANUAL);
  directionSetPoint = 0;
  directionReading = 0;
  directionPidOutput = 0;

  started = false;
}

void computePitch() {
  // Read x, y, z accelerometer values
  float ax, ay, az;
  IMU.readAcceleration(ax, ay, az);

  // Calculate the current pitch angle in degrees according to the accelerometer
  float accelAngleX = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * RADIANS_TO_DEGREES;
  //float accelAngleY = atan(-1 * ax / sqrt(pow(ay, 2) + pow(az, 2))) * RADIANS_TO_DEGREES;

  imuData.pitchAccelerometer = accelAngleX;

  // Read gyro values
  float gx, gy, gz;
  IMU.readGyroscope(gx, gy, gz);

  // Apply calibration value - not actually necessary it turns out
  //gx -= imuData.gyroXCalibration;
  //gy -= imuData.gyroYCalibration;

  long currentMicros = micros();
  float dt = (currentMicros - lastImuMesaurementMicros) / 1000000.0;
  lastImuMesaurementMicros = currentMicros;

  // Calculate the angle in degrees traveled during this loop angle
  // (Gyroscope Angle) = (Last Measured Filtered Angle) + ω×Δt
  float pitchGyro = imuData.pitchReading + gx * dt;

  // Complementary filter to combine the gyro and accelerometer angle
  // Filtered Angle = α × (Gyroscope Angle) + (1 − α) × (Accelerometer Angle)
  imuData.pitchReading = 0.996 * pitchGyro + 0.004 * imuData.pitchAccelerometer;
}

void loop() {
  cmd.readSerial(&Serial1);

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    computePitch();
  }

  // Start balancing when angle is close to zero
  if (!started
      && imuData.pitchAccelerometer > -1.0 + PITCH_OFFSET
      && imuData.pitchAccelerometer < 1.0 + PITCH_OFFSET) {
    reset();
    imuData.pitchReading = imuData.pitchAccelerometer;
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
    if (rcChannel1PulseDuration > 100 && rcChannel1PulseDuration < 1250) {
      speedSetPoint = -3;
    } else if (rcChannel1PulseDuration > 1750) {
      speedSetPoint = 3;
    } else {
      speedSetPoint = 0;
    }

    //  Set the desired angle based on the error in the speed
    speedPid.Compute();
    pitchSetPoint = speedPidOutput + PITCH_OFFSET; // imperfect construction of robot

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
      long steering = (rcChannel2PulseDuration - 1510) / 10;
      steering = constrain(steering, -50, 50);
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
