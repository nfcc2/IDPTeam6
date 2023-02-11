
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
// constants won't change. They're used here to set pin numbers:
const int buttonPin = A3;  // the number of the pushbutton pin
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// variables will change:
bool buttonState = 1;  // variable for reading the pushbutton status
bool startProgram = false;

void setup() {
  // initialize the LED pin as an output:
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT_PULLUP);

  AFMS.begin();
  Serial.begin(9600);

          // robot starts moving when we push the start buttonn
         while (startProgram==false) {
           Serial.println("Hi");
            if (digitalRead(buttonPin) == false) { // pullup mode: output = LOW when button is pressed
                startProgram=true;
                break;
            }
        } 
}

void loop() {
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  Serial.print(buttonState);
  Serial.print('\n');
  delay(100);
}