#include <SAMD21turboPWM.h>
#include "PID_v1.h"
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
int motorDeadBand = 8;

TurboPWM pwm;
Encoder encoderLeft(ENCODER_A_L, ENCODER_B_L);
Encoder encoderRight(ENCODER_A_R, ENCODER_B_R);

long encoderLeftValue = 0;
long encoderRightValue = 0;

double speedSetPoint = 2;
double filteredSpeedLeft = 0;
double filteredSpeedRight = 0;
double pwmLeft = 0;
double pwmRight = 0;
PID speedPidLeft(&filteredSpeedLeft, &pwmLeft, &speedSetPoint, 20, 15, 2, DIRECT);
PID speedPidRight(&filteredSpeedRight, &pwmRight, &speedSetPoint, 20, 15, 2, DIRECT);

// Low pass butterworth filter order=2 alpha1=0.004
class  FilterBuLp2
{
  public:
    FilterBuLp2()
    {
      this -> reset();
    }
  private:
    double v[3];
  public:
    double step(double x) //class II
    {
      v[0] = v[1];
      v[1] = v[2];
      v[2] = (1.551484234757205538e-4 * x)
         + (-0.96508117389913528061 * v[0])
         + (1.96446058020523239840 * v[1]);
      return 
         (v[0] + v[2])
        +2 * v[1];
    }

    void reset()
    {
      v[0] = 0.0;
      v[1] = 0.0;
      v[2] = 0.0;
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
  //Serial.println(pwm.frequency(0));

  speedPidLeft.SetSampleTime(LOOP_MICROS / 1000);
  speedPidLeft.SetOutputLimits(-1000, 1000);
  speedPidLeft.SetMode(AUTOMATIC);

  speedPidRight.SetSampleTime(LOOP_MICROS / 1000);
  speedPidRight.SetOutputLimits(-1000, 1000);
  speedPidRight.SetMode(AUTOMATIC);

  loopEndTime = micros() + LOOP_MICROS;
}

void loop() {
  long newEncoderLeftValue = encoderLeft.getValue();
  long newEncoderRightValue = encoderRight.getValue();
  long encoderDeltaLeft = newEncoderLeftValue - encoderLeftValue;
  long encoderDeltaRight = newEncoderRightValue - encoderRightValue;
  encoderLeftValue = newEncoderLeftValue;
  encoderRightValue = newEncoderRightValue;

  filteredSpeedLeft = filterRight.step(encoderDeltaLeft);
  filteredSpeedRight = filterRight.step(-encoderDeltaRight);

  speedPidLeft.Compute();
  speedPidRight.Compute();

  long scaledPwmLeft = map(abs(pwmLeft), 0, 1000, motorDeadBand, 1000);
  long scaledPwmRight = map(abs(pwmRight), 0, 1000, motorDeadBand, 1000);

  pwm.analogWrite(PWM_L, scaledPwmLeft);
  digitalWrite(INA_L, pwmLeft > 0 ? LOW : HIGH);
  digitalWrite(INB_L, pwmLeft > 0 ? HIGH : LOW);

  pwm.analogWrite(PWM_R, scaledPwmRight);
  digitalWrite(INA_R, pwmRight > 0 ? HIGH : LOW);
  digitalWrite(INB_R, pwmRight > 0 ? LOW : HIGH);

  Serial.print("el:");
  Serial.print(encoderDeltaLeft);
  /*Serial.print(", er:");
  Serial.print(encoderDeltaRight);*/
  Serial.print(", fl:");
  Serial.print(filteredSpeedLeft);
  /*Serial.print(", fr:");
  Serial.print(filteredSpeedRight);*/
  /*Serial.print(", pl:");
  Serial.print(pwmLeft);
  Serial.print(", pr:");
  Serial.print(pwmRight);*/
  Serial.println();

  loopCount++;
  if (loopCount / 2000 % 2 == 0) {
    speedSetPoint = 2;
  } else {
    speedSetPoint = 6;
  }

  while (loopEndTime > micros());
  loopEndTime += LOOP_MICROS;
}
