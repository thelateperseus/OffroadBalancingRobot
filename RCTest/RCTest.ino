const int CHANNEL1_PIN = 2;
const int CHANNEL2_PIN = 3;

volatile unsigned long channel1PulseStart = 0;
volatile unsigned long channel1PulseDuration = 0;
volatile unsigned long channel2PulseStart = 0;
volatile unsigned long channel2PulseDuration = 0;

void channel1Interrupt() {
  boolean channel1 = digitalRead(CHANNEL1_PIN);
  unsigned long now = micros();
  if (channel1) {
    channel1PulseStart = now;
  } else {
    channel1PulseDuration = now - channel1PulseStart;
  }
}

void channel2Interrupt() {
  boolean channel2 = digitalRead(CHANNEL2_PIN);
  unsigned long now = micros();
  if (channel2) {
    channel2PulseStart = now;
  } else {
    channel2PulseDuration = now - channel2PulseStart;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(CHANNEL1_PIN, INPUT);
  pinMode(CHANNEL2_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(CHANNEL1_PIN), channel1Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CHANNEL2_PIN), channel2Interrupt, CHANGE);
}

void loop() {
  delay(20);
  Serial.print("x:");
  Serial.print(channel1PulseDuration);
  Serial.print(", y:");
  Serial.println(channel2PulseDuration);
}