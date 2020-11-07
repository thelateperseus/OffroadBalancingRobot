#include "SerialCommand.h"

SerialCommand cmd;

void setup()
{
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Go!");

  Serial1.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  cmd.addCommand("kp", setKp);
  cmd.addCommand("ki", setKi);
  cmd.addCommand("kd", setKd);
}

void loop() {
  cmd.readSerial(&Serial1);
}

void setKp() {
  char* arg = cmd.next();
  if (arg != NULL) {
    double kp = atof(arg);
    Serial.print("Kp: ");
    Serial.println(kp, 12);
  }
  else {
    Serial.println("Kp No arguments");
  }
}

void setKi() {
  char* arg = cmd.next();
  if (arg != NULL) {
    double ki = atof(arg);
    Serial.print("Ki: ");
    Serial.println(ki, 12);
  }
  else {
    Serial.println("Ki No arguments");
  }
}

void setKd() {
  char* arg = cmd.next();
  if (arg != NULL) {
    double kd = atof(arg);
    Serial.print("Kd: ");
    Serial.println(kd, 12);
  }
  else {
    Serial.println("Kd No arguments");
  }
}
