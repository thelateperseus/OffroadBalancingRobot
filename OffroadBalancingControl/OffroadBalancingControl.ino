#include "SerialCommand.h"
#include "LSM6DS3.h"
#include "PID_v1.h"
#include <SAMD21turboPWM.h>

const int LOOP_MICROS = 4000;
long loopEndTime = 0;
long lastInternalMesaurementMicros = 0;

const float RADIANS_TO_DEGREES = 57.296;

const int INA_L = 7;
const int INB_L = 8;
const int PWM_L = 5;
const int INA_R = A2;
const int INB_R = A3;
const int PWM_R = 6;

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

const double PITCH_OFFSET = -0.2;
double motorDeadBand = 56;

double kp = 50;
double ki = 150;
double kd = 0.5;
double pitchSetPoint = PITCH_OFFSET;
double pitchReading = 0;
double pitchPidOutput = 0;
PID pitchPid(&pitchReading, &pitchPidOutput, &pitchSetPoint, kp, ki, kd, REVERSE);

TurboPWM pwm;

SerialCommand cmd;

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
  lastInternalMesaurementMicros = micros();
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

void setKp() {
  char* arg = cmd.next();
  if (arg != NULL) {
    kp = atof(arg);
    pitchPid.SetTunings(kp, ki, kd);
  }
}

void setKi() {
  char* arg = cmd.next();
  if (arg != NULL) {
    ki = atof(arg);
    pitchPid.SetTunings(kp, ki, kd);
  }
}

void setKd() {
  char* arg = cmd.next();
  if (arg != NULL) {
    kd = atof(arg);
    pitchPid.SetTunings(kp, ki, kd);
  }
}

void setDeadband() {
  char* arg = cmd.next();
  if (arg != NULL) {
    motorDeadBand = atof(arg);
  }
}

void setup() {
  //Serial.begin(115200);
  //while (!Serial); // Nano 33 will lose initial output without this
  Serial1.begin(115200);

  cmd.addCommand("kp", setKp);
  cmd.addCommand("ki", setKi);
  cmd.addCommand("kd", setKd);
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

  pitchPid.SetSampleTime(4);
  pitchPid.SetOutputLimits(-1000, 1000);

  reset();

  digitalWrite(LED_BUILTIN, HIGH);
  loopEndTime = micros() + LOOP_MICROS;
}

/*---------
    Loop
  ---------*/

void reset() {
  pitchReading = 0;
  pitchSetPoint = PITCH_OFFSET;
  pitchPidOutput = 0;
  started = false;
  pitchPid.SetMode(MANUAL);
}

void computePitch() {
  // Read x, y, z accelerometer values
  float ax, ay, az;
  IMU.readAcceleration(ax, ay, az);

  // Calculate the current pitch angle in degrees according to the accelerometer
  float accelAngleX = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * RADIANS_TO_DEGREES;
  //float accelAngleY = atan(-1 * ax / sqrt(pow(ay, 2) + pow(az, 2))) * RADIANS_TO_DEGREES;

  imuData.pitchAccelerometer = accelAngleX;

  /*Serial1.print("ipa:");
    Serial1.print(pitchAccelerometer);
    Serial1.print(", ");*/

  // Read gyro values
  float gx, gy, gz;
  IMU.readGyroscope(gx, gy, gz);

  // Apply calibration value
  gx -= imuData.gyroXCalibration;
  //gy -= imuData.gyroYCalibration;

  long currentMicros = micros();
  float dt = (currentMicros - lastInternalMesaurementMicros) / 1000000.0;
  lastInternalMesaurementMicros = currentMicros;

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
    /*Serial1.print("p:");
      Serial1.print(imuData.pitchReading);
      Serial1.println();*/
  }

  // Start balancing when angle is close to zero
  if (!started
      && imuData.pitchAccelerometer > -1.0 + PITCH_OFFSET
      && imuData.pitchAccelerometer < 1.0 + PITCH_OFFSET) {
    imuData.pitchReading = imuData.pitchAccelerometer;
    started = true;
    pitchPid.SetMode(AUTOMATIC);
  }

  if (started) {
    pitchReading = imuData.pitchReading;
    pitchPid.Compute();
    double speedLeft = pitchPidOutput;
    double speedRight = pitchPidOutput;
    boolean directionLeft = speedLeft > 0;
    boolean directionRight = speedRight > 0;
    speedLeft = map(abs(speedLeft), 0.0, 1000.0, motorDeadBand, 1000.0);
    speedRight = map(abs(speedRight), 0.0, 1000.0, motorDeadBand, 1000.0);

    Serial1.print("sp:");
    Serial1.print(pitchSetPoint);
    Serial1.print(", pv:");
    Serial1.print(pitchReading);
    Serial1.print(", o:");
    Serial1.print(pitchPidOutput);
    Serial1.print(", s:");
    Serial1.print(speedLeft);
    Serial1.println();

    // Automatic balance point correction
    if (pitchPidOutput < -10) {
      pitchSetPoint += 0.002;
    }
    if (pitchPidOutput > 10) {
      pitchSetPoint -= 0.002;
    }

    if (pitchReading > 45 || pitchReading < -45) {
      speedLeft = 0;
      speedRight = 0;
      reset();
    }

    speedLeft = constrain(speedLeft, -1000, 1000);
    speedRight = constrain(speedRight, -1000, 1000);

    pwm.analogWrite(PWM_L, speedLeft);
    digitalWrite(INA_L, directionLeft ? LOW : HIGH);
    digitalWrite(INB_L, directionLeft ? HIGH : LOW);
    pwm.analogWrite(PWM_R, speedRight);
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
