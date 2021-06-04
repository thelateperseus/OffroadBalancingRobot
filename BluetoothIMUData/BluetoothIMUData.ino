#include "LSM6DS3.h"

//const float PITCH_CALIBRATION = 0.34;
const int LOOP_MICROS = 4000;
long loopEndTime = 0;
long lastInternalMesaurementMicros = 0;

const float RADIANS_TO_DEGREES = 57.296;

struct IMUData {
  float gyroXCalibration = 0;
  float gyroYCalibration = 0;
  float gyroZCalibration = 0;
  float accelerometerXCalibration = 0;
  float accelerometerYCalibration = 0;
  float accelerometerZCalibration = 0;
  float pitchReading = 0;
};

IMUData internalData;
IMUData externalData;

void setup() {
  //Serial.begin(115200);
  //while (!Serial); // Nano 33 will lose initial output without this
  Serial1.begin(115200);

  //Serial.println("Initializing IMU...");

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

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

    while (!IMU.accelerationAvailable() && !IMU.gyroscopeAvailable());
  }
  lastInternalMesaurementMicros = micros();
  // Get the average from the 500 readings
  internalData.gyroXCalibration = gxSum / 500;
  internalData.gyroYCalibration = gySum / 500;
  internalData.gyroZCalibration = gzSum / 500;
  internalData.accelerometerXCalibration = axSum / 500;
  internalData.accelerometerYCalibration = aySum / 500;
  internalData.accelerometerZCalibration = azSum / 500;

  // Gyro calibration complete, gy: 275, ax: -138, ay: -286, az: 18227
  /*Serial.print("Internal IMU calibration complete, gx: ");
  Serial.print(internalData.gyroXCalibration);
  Serial.print(", gy: ");
  Serial.print(internalData.gyroYCalibration);
  Serial.print(", gz: ");
  Serial.print(internalData.gyroZCalibration);
  Serial.print(", ax: ");
  Serial.print(internalData.accelerometerXCalibration);
  Serial.print(", ay: ");
  Serial.print(internalData.accelerometerYCalibration);
  Serial.print(", az: ");
  Serial.print(internalData.accelerometerZCalibration);
  Serial.println();*/

  // 12:39:01.706 -> Internal IMU calibration complete, gx: 0.91, gy: -3.77, gz: -4.53, ax: 0.03, ay: 0.01, az: 0.99

  digitalWrite(LED_BUILTIN, HIGH);
  loopEndTime = micros() + LOOP_MICROS;
}

void loop() {
  // Internal IMU angle calculation
  // ------------------------------

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    // Read x, y, z accelerometer values
    float ax, ay, az;
    IMU.readAcceleration(ax, ay, az);

    // Calculate the current pitch angle in degrees according to the accelerometer
    float accelAngleX = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * RADIANS_TO_DEGREES;
    //float accelAngleY = atan(-1 * ax / sqrt(pow(ay, 2) + pow(az, 2))) * RADIANS_TO_DEGREES;

    float pitchAccelerometer = accelAngleX;

    Serial1.print("ipa:");
    Serial1.print(pitchAccelerometer);
    Serial1.print(", ");

    // Read gyro values
    float gx, gy, gz;
    IMU.readGyroscope(gx, gy, gz);

    // Apply calibration value
    gx -= internalData.gyroXCalibration;
    //gy -= internalData.gyroYCalibration;

    long currentMicros = micros();
    float dt = (currentMicros - lastInternalMesaurementMicros) / 1000000.0;
    lastInternalMesaurementMicros = currentMicros;

    // Calculate the angle in degrees traveled during this loop angle
    // (Gyroscope Angle) = (Last Measured Filtered Angle) + ω×Δt
    float pitchGyro = internalData.pitchReading + gx * dt;

    // Complementary filter to combine the gyro and accelerometer angle
    // Filtered Angle = α × (Gyroscope Angle) + (1 − α) × (Accelerometer Angle)
    internalData.pitchReading = 0.96 * pitchGyro + 0.04 * pitchAccelerometer;

    Serial1.print("ipc:");
    Serial1.print(internalData.pitchReading);
    Serial1.println();
  }

  // The external IMU angle calculations are tuned for a loop time of 4 milliseconds
  while(loopEndTime > micros());
  loopEndTime = micros() + LOOP_MICROS;
}
