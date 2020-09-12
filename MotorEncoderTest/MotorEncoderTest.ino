const int ENCODER_A = 2;
const int ENCODER_B = 3;

const int IN1 = 5;
const int IN2 = 7;
const int PWM = 6;

const int MOTOR_SPEED = 12;
const int LOOP_MICROS = 1000000;

volatile int pulseCount = 0;
long loopEndTime = 0;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT);

  Serial.begin(2000000);
  while (!Serial);
  Serial.print("PWM Value: ");
  Serial.println(MOTOR_SPEED);

  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderPulse, RISING);

  loopEndTime = micros() + LOOP_MICROS;
}

void encoderPulse() {
  pulseCount++;
}

void loop() {
  //control speed 
  analogWrite(PWM, MOTOR_SPEED);
  //control direction 
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  noInterrupts();
  int pulseCountValue = pulseCount;
  pulseCount = 0;
  interrupts();

  Serial.println(pulseCountValue);

  long delayTime = loopEndTime - micros();
  if (delayTime > 0) {
    delayMicroseconds(delayTime);
  }
  loopEndTime = micros() + LOOP_MICROS;
}
