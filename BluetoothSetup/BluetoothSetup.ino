// Basic Bluetooth sketch HC-06_01
// Connect the Hc-06 module and communicate using the serial monitor
//
// The HC-06 defaults to AT mode when first powered on.
// The default baud rate is 9600
// The Hc-06 requires all AT commands to be in uppercase. NL+CR MUST be added to the command string
//
// AT+NAME:BarbieSegway
// AT+PSWD="0852"
// AT+UART:115200,0,0
// AT+UART=57600,0,0 for remote programming (apparently)
// HC-06 address is 98D3:11:F86879
// HC-05 address is 0019:10:094345

// Connect the HC-06 TX to the Arduino Nano 33 IoT RX on pin 0.
// Connect the HC-06 RX to the Arduino Nano 33 IoT TX on pin 1.
//


void setup()
{
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Enter AT commands:");

  // HC-06 default serial speed is 9600
  Serial1.begin(115200);
}

void loop()
{

  // Keep reading from HC-06 and send to Arduino Serial Monitor
  if (Serial1.available()) {
    Serial.write(Serial1.read());
    //digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  // Keep reading from Arduino Serial Monitor and send to HC-06
  if (Serial.available()) {
    Serial1.write(Serial.read());
  }

}
