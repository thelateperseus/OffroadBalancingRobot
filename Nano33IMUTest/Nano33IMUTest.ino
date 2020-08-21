#include "MPU6050.h"

const float PITCH_CALIBRATION = 0.34;
const int LOOP_MICROS = 4000;

float gyroYCalibration = 0;
float accelerometerXCalibration = 0;
float accelerometerYCalibration = 0;
float accelerometerZCalibration = 0;
long loopEndTime = 0;
float pitchReading = 0;

MPU6050 mpu6050;

void setup() {
  Serial.begin(2000000);
  while (!Serial); // Nano 33 will lose initial output without this

  Serial.println("Initializing MPU-6050...");

  mpu6050.begin();

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println("Measuring gyro calibration values...");
  loopEndTime = micros() + LOOP_MICROS;
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
    mpu6050.readGyroscope(gx, gy, gz);
    gxSum += gx;
    gySum += gy;
    gzSum += gz;

    float ax, ay, az;
    mpu6050.readAccelerometer(ax, ay, az);
    axSum += ax;
    aySum += ay;
    azSum += az;

    delayMicroseconds(loopEndTime - micros()); // Simulate the main program loop time
    loopEndTime = micros() + LOOP_MICROS;
  }
  // Get the average from the 500 readings
  gyroYCalibration = gySum / 500;
  accelerometerXCalibration = axSum / 500;
  accelerometerYCalibration = aySum / 500;
  accelerometerZCalibration = azSum / 500;

  // Gyro calibration complete, gy: 275, ax: -138, ay: -286, az: 18227
  Serial.print("Gyro calibration complete, gy: ");
  Serial.print(gyroYCalibration);
  Serial.print(", ax: ");
  Serial.print(accelerometerXCalibration);
  Serial.print(", ay: ");
  Serial.print(accelerometerYCalibration);
  Serial.print(", az: ");
  Serial.print(accelerometerZCalibration);
  Serial.println();

  digitalWrite(LED_BUILTIN, HIGH);
  loopEndTime = micros() + LOOP_MICROS;
}

void loop() {
  // Read x, y, z accelerometer values
  float ax, ay, az;
  mpu6050.readAccelerometer(ax, ay, az);

  // Calculate the current pitch angle in degrees according to the accelerometer
  float pitchAccelerometer = atan(-1 * ax / sqrt(pow(ay, 2) + pow(az, 2))) * 57.296;
  pitchAccelerometer -= PITCH_CALIBRATION;

  Serial.print(", pa: ");
  Serial.print(pitchAccelerometer);

  // Read gyro values
  float gx, gy, gz;
  mpu6050.readGyroscope(gx, gy, gz);

  // Apply calibration value
  gy -= gyroYCalibration;

  float dt = LOOP_MICROS / 1000000.0;

  // Calculate the angle in degrees traveled during this loop angle
  // (Gyroscope Angle) = (Last Measured Filtered Angle) + ω×Δt
  float pitchGyro = pitchReading + gy * dt;

  // Complementary filter to combine the gyro and accelerometer angle
  // Filtered Angle = α × (Gyroscope Angle) + (1 − α) × (Accelerometer Angle)
  pitchReading = 0.96 * pitchGyro + 0.04 * pitchAccelerometer;

  Serial.print(", pc: ");
  Serial.print(pitchReading);

  Serial.println();

  // The angle calculations are tuned for a loop time of 4 milliseconds
  delayMicroseconds(loopEndTime - micros()); // Simulate the main program loop time
  loopEndTime = micros() + LOOP_MICROS;
}
