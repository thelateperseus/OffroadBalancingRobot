#include <SAMD21turboPWM.h>
#include "Encoder.h"

const int ENCODER_A_L = 9;
const int ENCODER_B_L = 10;
const int ENCODER_A_R = 11;
const int ENCODER_B_R = A1;

const int INA_L = 7;
const int INB_L = 8;
const int PWM_L = 5;
const int INA_R = A2;
const int INB_R = A3;
const int PWM_R = 6;

const int LOOP_MICROS = 4000;

long loopEndTime = 0;
long loopCount = 0;

long pwmValue = 0;

TurboPWM pwm;
Encoder encoderLeft(ENCODER_A_L, ENCODER_B_L);
Encoder encoderRight(ENCODER_A_R, ENCODER_B_R);

long encoderLeftValue = 0;
long encoderRightValue = 0;

//Low pass butterworth filter order=4 alpha1=0.016 
class  FilterBuLp4
{
  public:
    FilterBuLp4()
    {
      for(int i=0; i <= 4; i++)
        v[i]=0.0;
    }
  private:
    double v[5];
  public:
    double step(double x) //class II 
    {
      v[0] = v[1];
      v[1] = v[2];
      v[2] = v[3];
      v[3] = v[4];
      v[4] = (5.616562286377135838e-6 * x)
         + (-0.76887274386665194204 * v[0])
         + (3.27743279390187858269 * v[1])
         + (-5.24600330601763076288 * v[2])
         + (3.73735339098582208806 * v[3]);
      return 
         (v[0] + v[4])
        +4 * (v[1] + v[3])
        +6 * v[2];
    }
};

//Low pass butterworth filter order=2 alpha1=0.016 
class  FilterBuLp2
{
  public:
    FilterBuLp2()
    {
      v[0]=0.0;
      v[1]=0.0;
      v[2]=0.0;
    }
  private:
    double v[3];
  public:
    double step(double x) //class II 
    {
      v[0] = v[1];
      v[1] = v[2];
      v[2] = (2.357208772852337209e-3 * x)
         + (-0.86747213379166820957 * v[0])
         + (1.85804329870025886073 * v[1]);
      return 
         (v[0] + v[2])
        +2 * v[1];
    }
};

//Low pass butterworth filter order=1 alpha1=0.004
class  FilterBuLp1
{
  public:
    FilterBuLp1()
    {
      v[0]=v[1]=0.0;
    }
  private:
    double v[2];
  public:
    double step(double x) //class II 
    {
      v[0] = v[1];
      v[1] = (1.241106190967544882e-2 * x)
         + (0.97517787618064910582 * v[0]);
      return 
         (v[0] + v[1]);
    }
};

FilterBuLp2 filterLeft;
FilterBuLp2 filterRight;

void encoderLeftInterruptA() {
  encoderLeft.encoderInterruptA();
}

void encoderLeftInterruptB() {
  encoderLeft.encoderInterruptB();
}

void encoderRightInterruptA() {
  encoderRight.encoderInterruptA();
}

void encoderRightInterruptB() {
  encoderRight.encoderInterruptB();
}

void setup() {
  pinMode(INA_L, OUTPUT);
  pinMode(INB_L, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(INA_R, OUTPUT);
  pinMode(INB_R, OUTPUT);
  pinMode(PWM_R, OUTPUT);

  Serial.begin(2000000);
  while (!Serial);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A_L), encoderLeftInterruptA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_L), encoderLeftInterruptB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_R), encoderRightInterruptA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_R), encoderRightInterruptB, CHANGE);

  // Turbo = true
  pwm.setClockDivider(1, false);
  // For the Arduino Nano 33 IoT, you need to initialise timer 1 for pins 4 and 7, timer 0 for pins 5, 6, 8, and 12, and timer 2 for pins 11 and 13;
  // timer 1, prescaler 1, resolution 4800, fast PWM
  pwm.timer(0, 1, 1200, false);

  loopEndTime = micros() + LOOP_MICROS;
}

void loop() {
  long newEncoderLeftValue = encoderLeft.getValue();
  long newEncoderRightValue = encoderRight.getValue();
  long encoderDeltaLeft = newEncoderLeftValue - encoderLeftValue;
  long encoderDeltaRight = newEncoderRightValue - encoderRightValue;
  encoderLeftValue = newEncoderLeftValue;
  encoderRightValue = newEncoderRightValue;

  double filteredSpeedLeft = filterRight.step(encoderDeltaLeft);
  double filteredSpeedRight = filterRight.step(-encoderDeltaRight);

  pwm.analogWrite(PWM_L, pwmValue);
  digitalWrite(INA_L, pwmValue > 0 ? LOW : HIGH);
  digitalWrite(INB_L, pwmValue > 0 ? HIGH : LOW);

  pwm.analogWrite(PWM_R, pwmValue);
  digitalWrite(INA_R, pwmValue > 0 ? HIGH : LOW);
  digitalWrite(INB_R, pwmValue > 0 ? LOW : HIGH);

  Serial.print("el:");
  Serial.print(encoderDeltaLeft);
  /*Serial.print(", er:");
  Serial.print(encoderDeltaRight);*/
  Serial.print(", fl:");
  Serial.print(filteredSpeedLeft);
  /*Serial.print(", fr:");
  Serial.print(filteredSpeedRight);*/
  Serial.println();

  loopCount++;
  if (loopCount / 2000 % 2 == 0) {
    pwmValue = 0;
  } else {
    pwmValue = 30;
  }

  while (loopEndTime > micros());
  loopEndTime += LOOP_MICROS;
}
