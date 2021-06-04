const int INA_L = 7;
const int INB_L = 8;
const int PWM_L = 5;
const int INA_R = A2;
const int INB_R = A3;
const int PWM_R = 6;

void setup() {

  pinMode(INA_L, OUTPUT);
  pinMode(INB_L, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(INA_R, OUTPUT);
  pinMode(INB_R, OUTPUT);
  pinMode(PWM_R, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {

  //control speed
  analogWrite(PWM_L, 40);
  //control direction
  digitalWrite(INA_L, LOW);
  digitalWrite(INB_L, HIGH);

  //control speed
  analogWrite(PWM_R, 40);
  //control direction
  digitalWrite(INA_R, HIGH);
  digitalWrite(INB_R, LOW);
}
