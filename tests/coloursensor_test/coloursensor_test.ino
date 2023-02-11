
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
// constants won't change. They're used here to set pin numbers:
const int buttonPin = 3;  // the number of the pushbutton pin
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// variables will change:
bool buttonState = 0;  // variable for reading the pushbutton status
bool startProgram = false;

void setup() {
  // initialize the LED pin as an output:
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);

  AFMS.begin();
  Serial.begin(9600);

}

void loop() {
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  Serial.print(buttonState);
  Serial.print('\n');
  delay(100);
}