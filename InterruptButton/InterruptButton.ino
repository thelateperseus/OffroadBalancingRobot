// constants won't change. They're used here to set pin numbers:
const int buttonPin = 2;     // the number of the pushbutton pin
const int ledPin =  13;      // the number of the LED pin

// variables will change:
volatile int ledState = HIGH;         // variable for reading the pushbutton status
volatile int changeCount = 0;

void setup() {
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);

  Serial.begin(115200);
  while (!Serial);

  digitalWrite(ledPin, ledState);
  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonPressed, FALLING);
}

void buttonPressed() {
  changeCount++;
  ledState = !ledState;
  digitalWrite(ledPin, ledState);
}

void loop() {
  noInterrupts();
  int changeCountValue = changeCount;
  changeCount = 0;
  interrupts();

  Serial.println(changeCountValue);
  delay(1000);
}
