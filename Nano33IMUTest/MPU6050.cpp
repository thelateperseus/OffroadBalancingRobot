#include "MPU6050.h"

MPU6050::MPU6050() {
}

int MPU6050::begin() {
  Wire.begin();
  Wire.setClock(1000000);

  // Wake up the MPU-6050
  writeRegister(0x6B, 0x00); // PWR_MGMT_1 register
  // Set the full scale of the gyro to +/- 250 degrees per second
  writeRegister(0x1B, 0x00); // GYRO_CONFIG register
  // Set the full scale of the accelerometer to +/- 2g.
  writeRegister(0x1C, 0x00); // ACCEL_CONFIG register
  // Enable Digital Low Pass Filter ~43Hz to improve the raw data.
  writeRegister(0x1A, 0x03); // CONFIG register
}

int MPU6050::writeRegister(uint8_t address, uint8_t value) {
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission();
  return 1;
}

// Results are in g (earth gravity)
int MPU6050::readAccelerometer(float& ax, float& ay, float& az) {
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.write(0x3B); // ACCEL_XOUT[15:8]
  Wire.endTransmission();
  Wire.requestFrom(GYRO_ADDRESS, 6); // ACCEL_XOUT[15:8], ACCEL_XOUT[7:0], ACCEL_YOUT[15:8], ACCEL_YOUT[7:0], ACCEL_ZOUT[15:8], ACCEL_ZOUT[7:0]
  int16_t axRaw = Wire.read() << 8 | Wire.read();
  ax = axRaw / 16384.0;
  int16_t ayRaw = Wire.read() << 8 | Wire.read();
  ay = ayRaw / 16384.0;
  int16_t azRaw = Wire.read() << 8 | Wire.read();
  az = azRaw / 16384.0;
  return 1;
}

// Results are in degrees per second
int MPU6050::readGyroscope(float& gx, float& gy, float& gz) {
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.write(0x43); // GYRO_XOUT[15:8]
  Wire.endTransmission();
  Wire.requestFrom(GYRO_ADDRESS, 6); // GYRO_XOUT[15:8], GYRO_XOUT[7:0], GYRO_YOUT[15:8], GYRO_YOUT[7:0], GYRO_ZOUT[15:8], GYRO_ZOUT[7:0]
  int16_t gxRaw = Wire.read() << 8 | Wire.read();
  gx = gxRaw / 131.0;
  int16_t gyRaw = Wire.read() << 8 | Wire.read();
  gy = gyRaw / 131.0;
  int16_t gzRaw = Wire.read() << 8 | Wire.read();
  gz = gzRaw / 131.0;
  return 1;
}
