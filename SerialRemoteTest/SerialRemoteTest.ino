#include "SerialCommand.h"

SerialCommand cmd;

volatile double speed;
volatile double steering;
unsigned long commandExpiryTime;

void receiveSerialCommand(const char* arg1) {
  if (arg1 != NULL) {
    char* arg2 = cmd.next();
    if (arg2 != NULL) {
      speed = constrain(atof(arg1), -5, 5);
      steering = constrain(atof(arg2), -40, 40);
      commandExpiryTime = millis() + 300;
    }
  }
}

void setup() {
  Serial1.begin(115200);
  Serial.begin(115200);
  while (!Serial);
  cmd.setDefaultHandler(receiveSerialCommand);
  Serial1.println("Started");
}

void loop() {
  cmd.readSerial(&Serial);

  if (commandExpiryTime > 0 && millis() > commandExpiryTime) {
    speed = 0;
    steering = 0;
    commandExpiryTime = 0;
  }

  Serial1.print("speed:");
  Serial1.print(speed);
  Serial1.print(", ");
  Serial1.print("steering:");
  Serial1.print(steering);
  Serial1.print(", ");
  Serial1.print("commandExpiryTime:");
  Serial1.print(commandExpiryTime);
  Serial1.println();
  delay(100);
}
