#include "RCChannel.h"

const int CHANNEL1_PIN = 3;
const int CHANNEL2_PIN = 2;

RCChannel channel1 = RCChannel(CHANNEL1_PIN);
RCChannel channel2 = RCChannel(CHANNEL2_PIN);

void channel1Interrupt() {
  channel1.handleInterrupt();
}

void channel2Interrupt() {
  channel2.handleInterrupt();
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  attachInterrupt(digitalPinToInterrupt(CHANNEL1_PIN), channel1Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CHANNEL2_PIN), channel2Interrupt, CHANGE);
}

void loop() {
  long channel1PulseDuration = channel1.getPulseDuration();
  long channel2PulseDuration = channel2.getPulseDuration();
  delay(20);
  Serial.print("x:");
  Serial.print(channel1PulseDuration);
  Serial.print(", y:");
  Serial.print(channel2PulseDuration);
  Serial.print(", t:");
  Serial.print((channel1PulseDuration - 1500) / 10);

  long steering = (channel2PulseDuration - 1500) / 10;
  steering = constrain(steering, -50, 50);

  Serial.print(", d:");
  Serial.print(steering);
  Serial.println();
}
