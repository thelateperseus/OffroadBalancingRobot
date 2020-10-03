const int ENCODER_A_PIN = 2;
const int ENCODER_B_PIN = 3;

const int IN1_PIN = 5;
const int IN2_PIN = 7;
const int PWM_PIN = 6;

const int MOTOR_SPEEDS[] = { 0, 8, 12, 16, 24, 32, 48, 64, 96, 128, 160, 192, 224, 255 };
const int MOTOR_SPEEDS_LENGTH = 14;
const int LOOP_MICROS = 1000000;

volatile int pulseCount = 0;
volatile int stepCount = 0;
long loopEndTime = 0;
long measurementTime = 0;
int speedIndex = 0;
int speedLoopCount = 0;

void setup() {
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);

  Serial.begin(2000000);
  while (!Serial);

  pinMode(ENCODER_A_PIN, INPUT);
  pinMode(ENCODER_B_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderPulseA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), encoderPulseB, CHANGE);

  measurementTime = micros();
  loopEndTime = micros() + LOOP_MICROS;
}

// Forward (positive stepCount)
//             _______
// Output A __|       |______
//                 _______
// Output B ______|       |__

// Backward (negative stepCount)
//                 _______
// Output A ______|       |__
//             _______
// Output B __|       |______

void encoderPulseA() {
  boolean outputA = digitalRead(ENCODER_A_PIN);
  boolean outputB = digitalRead(ENCODER_B_PIN);
  pulseCount++;
  if (outputA == outputB) {
    stepCount++;
  } else {
    stepCount--;
  }
}

void encoderPulseB() {
  boolean outputA = digitalRead(ENCODER_A_PIN);
  boolean outputB = digitalRead(ENCODER_B_PIN);
  pulseCount++;
  if (outputA == outputB) {
    stepCount--;
  } else {
    stepCount++;
  }
}

void loop() {
  if (speedIndex >= MOTOR_SPEEDS_LENGTH) {
    analogWrite(PWM_PIN, 0);
    while (true);
  }

  int speed = MOTOR_SPEEDS[speedIndex];
  speedLoopCount++;
  if (speedLoopCount >= 10) {
    speedIndex++;
    speedLoopCount = 0;
  }

  //control speed 
  analogWrite(PWM_PIN, speed);
  //control direction 
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);

  noInterrupts();
  //long now = micros();
  int pulseCountValue = pulseCount;
  pulseCount = 0;
  int stepCountValue = stepCount;
  stepCount = 0;
  interrupts();

  //long measurementInterval = now - measurementTime;
  //measurementTime = now;

  Serial.print(speed);
  Serial.print(",");
  Serial.print(stepCountValue);
  Serial.print(",");
  Serial.println(pulseCountValue);

  while(loopEndTime > micros());
  loopEndTime += LOOP_MICROS;
}
