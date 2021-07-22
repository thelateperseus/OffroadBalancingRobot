#include "IMUFilter.h"

IMUFilter::IMUFilter() {
  
}

// Offset value to compensate for robot centre of mass
const double PITCH_OFFSET = -4.5;

const float RADIANS_TO_DEGREES = 57.296;

void IMUFilter::initialise() {

  if (!IMU.begin()) {
    //Serial.println("Failed to initialize internal IMU!");
    while (1) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(500);
    }
  }

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

    while (!IMU.dataAvailable());
  }
  lastImuMesaurementMicros = micros();
  // Get the average from the 500 readings
  gyroXCalibration = gxSum / 500;
  gyroYCalibration = gySum / 500;
  gyroZCalibration = gzSum / 500;
  accelerometerXCalibration = axSum / 500;
  accelerometerYCalibration = aySum / 500;
  accelerometerZCalibration = azSum / 500;

  /*Serial.print("Internal IMU calibration complete, gx: ");
    Serial.print(gyroXCalibration);
    Serial.print(", gy: ");
    Serial.print(gyroYCalibration);
    Serial.print(", gz: ");
    Serial.print(gyroZCalibration);
    Serial.print(", ax: ");
    Serial.print(accelerometerXCalibration);
    Serial.print(", ay: ");
    Serial.print(accelerometerYCalibration);
    Serial.print(", az: ");
    Serial.print(accelerometerZCalibration);
    Serial.println();*/

  // 12:39:01.706 -> Internal IMU calibration complete, gx: 0.91, gy: -3.77, gz: -4.53, ax: 0.03, ay: 0.01, az: 0.99

  lastPitchValue = computeAccelerometerPitch(accelerometerXCalibration, accelerometerYCalibration, accelerometerZCalibration);
}

// Calculate the current pitch angle in degrees according to the accelerometer
float IMUFilter::computeAccelerometerPitch(float ax, float ay, float az) {
  //float accelAngleY = atan(-1 * ax / sqrt(pow(ay, 2) + pow(az, 2))) * RADIANS_TO_DEGREES;
  return atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * RADIANS_TO_DEGREES;
}

void IMUFilter::getPitchAngle(IMUData& imuData) {
  if (!IMU.dataAvailable()) {
    return;
  }

  // Read x, y, z accelerometer values
  float ax, ay, az;
  IMU.readAcceleration(ax, ay, az);

  ax = accelerometerFilterX.step(ax);
  ay = accelerometerFilterY.step(ay);
  az = accelerometerFilterZ.step(az);

  float pitchAccelerometer = computeAccelerometerPitch(ax, ay, az);

  // Read gyro values
  float gx, gy, gz;
  IMU.readGyroscope(gx, gy, gz);

  // Apply calibration value
  gx -= gyroXCalibration;
  //gy -= gyroYCalibration;

  long currentMicros = micros();
  float dt = (currentMicros - lastImuMesaurementMicros) / 1000000.0;
  lastImuMesaurementMicros = currentMicros;

  // Calculate the angle in degrees traveled during this loop angle
  // (Gyroscope Angle) = (Last Measured Filtered Angle) + ω×Δt
  float pitchGyro = lastPitchValue + gx * dt;

  // Complementary filter to combine the gyro and accelerometer angle
  // Filtered Angle = α × (Gyroscope Angle) + (1 − α) × (Accelerometer Angle)
  lastPitchValue = 0.996 * pitchGyro + 0.004 * pitchAccelerometer;

  imuData.pitchAccelerometer = pitchAccelerometer - PITCH_OFFSET;
  imuData.pitchReading = lastPitchValue - PITCH_OFFSET;
}
