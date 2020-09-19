const int ENCODER_A = 2;
const int ENCODER_B = 3;

const int IN1 = 5;
const int IN2 = 7;
const int PWM = 6;

const int MOTOR_SPEEDS[] = { 0, 8, 12, 16, 24, 32, 48, 64, 96, 128, 160, 192, 224, 255 };
const int MOTOR_SPEEDS_LENGTH = 14;
const int LOOP_MICROS = 1000000;

volatile int pulseCount = 0;
long loopEndTime = 0;
long measurementTime = 0;
int speedIndex = 0;
int speedLoopCount = 0;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT);

  Serial.begin(2000000);
  while (!Serial);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderPulse, RISING);

  measurementTime = micros();
  loopEndTime = micros() + LOOP_MICROS;
}

void encoderPulse() {
  pulseCount++;
}

void loop() {
  if (speedIndex >= MOTOR_SPEEDS_LENGTH) {
    analogWrite(PWM, 0);
    while (true);
  }

  int speed = MOTOR_SPEEDS[speedIndex];
  speedLoopCount++;
  if (speedLoopCount >= 15) {
    speedIndex++;
    speedLoopCount = 0;
  }

  //control speed 
  analogWrite(PWM, speed);
  //control direction 
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  noInterrupts();
  long now = micros();
  int pulseCountValue = pulseCount;
  pulseCount = 0;
  interrupts();

  long measurementInterval = now - measurementTime;
  measurementTime = now;

  Serial.print(speed);
  Serial.print(",");
  Serial.print(measurementInterval);
  Serial.print(",");
  Serial.println(pulseCountValue);

  while(loopEndTime > micros());
  loopEndTime += LOOP_MICROS;
}
