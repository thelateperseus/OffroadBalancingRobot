#include "IMUFilter.h"

const int LOOP_MICROS = 4000;
long loopEndTime = 0;

IMUFilter imuFilter;
IMUData imuData;

void setup() {
  Serial.begin(115200);
  while (!Serial); // Nano 33 will lose initial output without this
  Serial1.begin(115200);

  //Serial.println("Initializing IMU...");
  imuFilter.initialise();

  digitalWrite(LED_BUILTIN, HIGH);
  loopEndTime = micros() + LOOP_MICROS;
}

void loop() {
  // Internal IMU angle calculation
  // ------------------------------

  imuFilter.getPitchAngle(imuData);

  /*Serial.print("a:");
  Serial.print(imuData.pitchAccelerometer);
  Serial.print(", ");

  Serial.print("c:");
  Serial.print(imuData.pitchReading);
  Serial.println();*/

  // The external IMU angle calculations are tuned for a loop time of 4 milliseconds
  while(loopEndTime > micros());
  loopEndTime = micros() + LOOP_MICROS;
}
