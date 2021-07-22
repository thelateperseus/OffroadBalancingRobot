/*
  This file is part of the Arduino_LSM6DS3 library.
  Copyright (c) 2019 Arduino SA. All rights reserved.
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "LSM6DS3.h"

#define LSM6DS3_ADDRESS            0x6A

#define LSM6DS3_WHO_AM_I_REG       0X0F
#define LSM6DS3_CTRL1_XL           0X10
#define LSM6DS3_CTRL2_G            0X11

#define LSM6DS3_STATUS_REG         0X1E

#define LSM6DS3_CTRL6_C            0X15
#define LSM6DS3_CTRL7_G            0X16
#define LSM6DS3_CTRL8_XL           0X17

#define LSM6DS3_OUTX_L_G           0X22
#define LSM6DS3_OUTX_H_G           0X23
#define LSM6DS3_OUTY_L_G           0X24
#define LSM6DS3_OUTY_H_G           0X25
#define LSM6DS3_OUTZ_L_G           0X26
#define LSM6DS3_OUTZ_H_G           0X27

#define LSM6DS3_OUTX_L_XL          0X28
#define LSM6DS3_OUTX_H_XL          0X29
#define LSM6DS3_OUTY_L_XL          0X2A
#define LSM6DS3_OUTY_H_XL          0X2B
#define LSM6DS3_OUTZ_L_XL          0X2C
#define LSM6DS3_OUTZ_H_XL          0X2D


LSM6DS3Class::LSM6DS3Class(TwoWire& wire, uint8_t slaveAddress) :
  _wire(&wire),
  _slaveAddress(slaveAddress)
{
}

LSM6DS3Class::~LSM6DS3Class()
{
}

int LSM6DS3Class::begin()
{
  _wire->begin();
  _wire->setClock(400000);

  if (readRegister(LSM6DS3_WHO_AM_I_REG) != 0x69) {
    end();
    return 0;
  }

  //set the gyroscope control register to work at 208 Hz, 2000 dps and in bypass mode
  writeRegister(LSM6DS3_CTRL2_G, 0x5C);

  // Set the Accelerometer control register to work at 208 Hz (ODR_XL), with range +-4g, and anti-aliasing filter 100 Hz
  writeRegister(LSM6DS3_CTRL1_XL, 0x5A);

  // set gyroscope power mode to high performance and bandwidth to 16 MHz
  writeRegister(LSM6DS3_CTRL7_G, 0x00);

  // Set the accelerometer low pass filter to ODR_XL/50, i.e. 208/50 = 4.16Hz
  // LPF2_XL_EN | HPCF_XL1 | HPCF_XL0 | 0 | HP_SLOPE_XL_EN | 0 | LOW_PASS_ON_6D
  //
  // HPCF_XL[1:0]   LPF2 digital filter cutoff frequency [Hz]
  // --------------------------------------------------------
  // *   00         ODR_XL/50
  //     01         ODR_XL/100
  //     10         ODR_XL/9
  //     11         ODR_XL/400
  //writeRegister(LSM6DS3_CTRL8_XL, 0x80);

  return 1;
}

void LSM6DS3Class::end()
{
  writeRegister(LSM6DS3_CTRL2_G, 0x00);
  writeRegister(LSM6DS3_CTRL1_XL, 0x00);
  _wire->end();
}

boolean LSM6DS3Class::dataAvailable() {
  int statusRegister = readRegister(LSM6DS3_STATUS_REG);
  return (statusRegister & 0x01) && (statusRegister & 0x02);
}

int LSM6DS3Class::readAcceleration(float& x, float& y, float& z)
{
  int16_t data[3];

  if (!readRegisters(LSM6DS3_OUTX_L_XL, (uint8_t*)data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  x = data[0] * 4.0 / 32768.0;
  y = data[1] * 4.0 / 32768.0;
  z = data[2] * 4.0 / 32768.0;

  return 1;
}

int LSM6DS3Class::readGyroscope(float& x, float& y, float& z) {
  int16_t data[3];

  if (!readRegisters(LSM6DS3_OUTX_L_G, (uint8_t*)data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  x = data[0] * 2000.0 / 32768.0;
  y = data[1] * 2000.0 / 32768.0;
  z = data[2] * 2000.0 / 32768.0;

  return 1;
}

int LSM6DS3Class::readRegister(uint8_t address)
{
  uint8_t value;
  
  if (readRegisters(address, &value, sizeof(value)) != 1) {
    return -1;
  }
  
  return value;
}

int LSM6DS3Class::readRegisters(uint8_t address, uint8_t* data, size_t length)
{
  _wire->beginTransmission(_slaveAddress);
  _wire->write(address);

  if (_wire->endTransmission(false) != 0) {
    return -1;
  }

  if (_wire->requestFrom(_slaveAddress, length) != length) {
    return 0;
  }

  for (size_t i = 0; i < length; i++) {
    *data++ = _wire->read();
  }
  return 1;
}

int LSM6DS3Class::writeRegister(uint8_t address, uint8_t value)
{
  _wire->beginTransmission(_slaveAddress);
  _wire->write(address);
  _wire->write(value);
  if (_wire->endTransmission() != 0) {
    return 0;
  }
  return 1;
}

LSM6DS3Class IMU(Wire, LSM6DS3_ADDRESS);
