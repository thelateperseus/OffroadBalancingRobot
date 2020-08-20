#include <Wire.h>

const int GYRO_ADDRESS = 0x68;
const float PITCH_CALIBRATION = 0.34;
const int LOOP_MICROS = 4000;

int16_t gyroYCalibration = 0;
int16_t accelerometerXCalibration = 0;
int16_t accelerometerYCalibration = 0;
int16_t accelerometerZCalibration = 0;
long loopEndTime = 0;
float pitchReading = 0;

void setup() {
  Serial.begin(2000000);
  while (!Serial); // Nano 33 will lose initial output without this

  Serial.println("Initializing MPU-6050...");

  Wire.begin();
  Wire.setClock(1000000);

  // Wake up the MPU-6050
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0x00);
  Wire.endTransmission();
  // Set the full scale of the gyro to +/- 250 degrees per second
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.write(0x1B); // GYRO_CONFIG register
  Wire.write(0x00);                                                         //Set the register bits as 00000000 (250dps full scale)
  Wire.endTransmission();                                                   //End the transmission with the gyro
  // Set the full scale of the accelerometer to +/- 2g.
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.write(0x1C); // ACCEL_CONFIG register
  Wire.write(0x00);
  Wire.endTransmission();
  // Enable Digital Low Pass Filter to improve the raw data.
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.write(0x1A); // CONFIG register
  Wire.write(0x03); // ~43Hz
  Wire.endTransmission();

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println("Measuring gyro calibration values...");
  loopEndTime = micros() + LOOP_MICROS;
  long gyroYSum = 0;
  long accelerometerXSum = 0;
  long accelerometerYSum = 0;
  long accelerometerZSum = 0;
  for (int i = 0; i < 500; i++) {
    if (i % 15 == 0) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));        //Change the state of the LED every 15 loops to make the LED blink fast
    }
    Wire.beginTransmission(GYRO_ADDRESS);
    Wire.write(0x45); // GYRO_YOUT[15:8]
    Wire.endTransmission();
    Wire.requestFrom(GYRO_ADDRESS, 2); // GYRO_YOUT[15:8], GYRO_YOUT[7:0]
    int16_t gyroY = Wire.read() << 8 | Wire.read();
    gyroYSum += gyroY;

    // Read x, y, z accelerometer values
    Wire.beginTransmission(GYRO_ADDRESS);
    Wire.write(0x3B); // ACCEL_XOUT[15:8]
    Wire.endTransmission();
    Wire.requestFrom(GYRO_ADDRESS, 6); // ACCEL_XOUT[15:8], ACCEL_XOUT[7:0], ACCEL_YOUT[15:8], ACCEL_YOUT[7:0], ACCEL_ZOUT[15:8], ACCEL_ZOUT[7:0]
    int16_t accelerometerX = Wire.read() << 8 | Wire.read();
    accelerometerXSum += accelerometerX;
    int16_t accelerometerY = Wire.read() << 8 | Wire.read();
    accelerometerYSum += accelerometerY;
    int16_t accelerometerZ = Wire.read() << 8 | Wire.read();
    accelerometerZSum += accelerometerZ;

    delayMicroseconds(loopEndTime - micros()); // Simulate the main program loop time
    loopEndTime = micros() + LOOP_MICROS;
  }
  // Get the average from the 500 readings
  gyroYCalibration = gyroYSum / 500;
  accelerometerXCalibration = accelerometerXSum / 500;
  accelerometerYCalibration = accelerometerYSum / 500;
  accelerometerZCalibration = accelerometerZSum / 500;

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
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.write(0x3B); // ACCEL_XOUT[15:8]
  Wire.endTransmission();
  Wire.requestFrom(GYRO_ADDRESS, 6); // ACCEL_XOUT[15:8], ACCEL_XOUT[7:0], ACCEL_YOUT[15:8], ACCEL_YOUT[7:0], ACCEL_ZOUT[15:8], ACCEL_ZOUT[7:0]
  int16_t accelerometerX = Wire.read() << 8 | Wire.read();
  int16_t accelerometerY = Wire.read() << 8 | Wire.read();
  int16_t accelerometerZ = Wire.read() << 8 | Wire.read();

  // Calculate the current pitch angle in degrees according to the accelerometer
  //double pitchAccelerometer = asin((double)accelerometerX / 8200.0) * 57.296;
  float pitchAccelerometer = atan(-1 * accelerometerX / sqrt(pow(accelerometerY, 2) + pow(accelerometerZ, 2))) * 57.296;
  pitchAccelerometer -= PITCH_CALIBRATION;

  Serial.print(", pa: ");
  Serial.print(pitchAccelerometer);

  // Read X and Y gyro values
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.write(0x45); // GYRO_YOUT[15:8]
  Wire.endTransmission();
  Wire.requestFrom(GYRO_ADDRESS, 2); // GYRO_YOUT[15:8], GYRO_YOUT[7:0]
  int16_t gyroYRaw = Wire.read() << 8 | Wire.read();

  // Convert to degrees per second
  float gyroY = (gyroYRaw - gyroYCalibration) / 131.0;

  float dt = LOOP_MICROS / 1000000.0;

  // Calculate the angle in degrees traveled during this loop angle
  // (Gyroscope Angle) = (Last Measured Filtered Angle) + ω×Δt
  float pitchGyro = pitchReading + gyroY * dt;

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
