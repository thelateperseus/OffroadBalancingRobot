#include "RCChannel.h"
#include <Arduino.h>

RCChannel::RCChannel(int argPin) {
  pin = argPin;
  pinMode(pin, INPUT);
}

void RCChannel::handleInterrupt() {
  boolean pinValue = digitalRead(pin);
  unsigned long now = micros();
  if (pinValue) {
    pulseStart = now;
  } else {
    long duration = now - pulseStart;
    // Ignore up to 2 consecutive spikes
    if (abs(duration - pulseDuration) < 400 || skipCount >= 2) {
      pulseDuration = constrain(duration, 0, 2000);
      skipCount = 0;
    } else {
      skipCount += 1;
    }
  }
}

long RCChannel::getPulseDuration() {
  return pulseDuration;
}

void RCChannel::reset() {
  pulseDuration = 1500;
}
