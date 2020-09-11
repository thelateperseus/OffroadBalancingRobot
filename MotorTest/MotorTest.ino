const int IN1A = 5;
const int IN2A = 7;
const int PWMA = 6;
const int IN1B = 9;
const int IN2B = 11;
const int PWMB = 10;

void setup() {

  pinMode(IN1A, OUTPUT);
  pinMode(IN2A, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(IN1B, OUTPUT);
  pinMode(IN2B, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {

  //control speed 
  analogWrite(PWMA, 256);
  //control direction 
  digitalWrite(IN1A, HIGH);
  digitalWrite(IN2A, LOW);
/*
  //control speed 
  analogWrite(PWMB, 40);
  //control direction 
  digitalWrite(IN1B, HIGH);
  digitalWrite(IN2B, LOW);*/
}
