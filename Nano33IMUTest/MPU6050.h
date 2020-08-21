#include <Wire.h>

const int GYRO_ADDRESS = 0x68;

class MPU6050 {
  public:
    MPU6050();

    int begin();

    int readAccelerometer(float& ax, float& ay, float& az); // Results are in G (earth gravity).

    int readGyroscope(float& gx, float& gy, float& gz); // Results are in degrees/second.


  private:
    int writeRegister(uint8_t address, uint8_t value);
};
