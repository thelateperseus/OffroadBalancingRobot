#include "LSM6DS3.h"
#include <Arduino.h>

//Low pass butterworth filter order=1 alpha1=0.02 
class  AccelerometerFilter
{
  public:
    AccelerometerFilter()
    {
      v[0]=0.0;
      v[1]=0.0;
      v[2]=0.0;
    }
  private:
    float v[3];
  public:
    float step(float x) //class II 
    {
      v[0] = v[1];
      v[1] = v[2];
      v[2] = (3.621681514928615665e-3 * x)
         + (-0.83718165125602272969 * v[0])
         + (1.82269492519630826877 * v[1]);
      return 
         (v[0] + v[2])
        +2 * v[1];
    }
};

struct IMUData {
  float pitchAccelerometer = 0;
  float pitchReading = 0;
};

class IMUFilter {
  public:
    IMUFilter();
    void initialise();
    void getPitchAngle(IMUData& imuData);

  private:
    float computeAccelerometerPitch(float ax, float ay, float az);

    float gyroXCalibration = 0;
    float gyroYCalibration = 0;
    float gyroZCalibration = 0;
    float accelerometerXCalibration = 0;
    float accelerometerYCalibration = 0;
    float accelerometerZCalibration = 0;
    float lastPitchValue = 0;
    long lastImuMesaurementMicros = 0;
    AccelerometerFilter accelerometerFilterX;
    AccelerometerFilter accelerometerFilterY;
    AccelerometerFilter accelerometerFilterZ;
};
