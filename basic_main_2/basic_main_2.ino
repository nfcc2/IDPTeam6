// use line following sensors to go one loop clockwise around the arena. Pick up block if detected using push button.

// include required libraries
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "NewPing.h"
#include <Arduino.h>

// create objects and specify pins
// motors
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1); // pin M1
Adafruit_DCMotor *leftMotor = AFMS.getMotor(2); //pin M2

// push button input pins
const int blockButtonPin = 2;
const int startButtonPin = 3;

// optoswitch pins for line following
#define farLeftOSwitch 4
#define leftOSwitch 5
#define rightOSwitch 6
#define farRightOSwitch 7

// optoswitch pin for colour identification
#define colourOSwitch 8

// LED output pins
const int amberLEDPin = 9;

// variable initialisation
int robotState = 0;
int motorSpeed1 = 150; // speed ranges from 0 to 255
int count = 0; // counts number of left intersections
bool holdingBlock = false;
bool redBlock = false;
bool start = false;
String sensorReading = "0101"; 
// for blinking amber LED when moving
bool LEDState = false; 
bool startLED = false;

void setup() {
  AFMS.begin();
  Serial.begin(9600);
  pinMode(blockButtonPin, INPUT);
  pinMode(startButtonPin, INPUT);
  pinMode(amberLEDPin, OUTPUT);
  
  // set up timer interrupt for flashing amber LED
  cli();  //stop interrupts for till we make the settings
  TCB0.CTRLB = TCB_CNTMODE_INT_gc; // Use timer compare mode
  TCB0.CCMP = 125000; // Value to compare with. 25000 is 1/10th of the tick rate, so 10 Hz
  TCB0.INTCTRL = TCB_CAPT_bm; // Enable the interrupt
  TCB0.CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_ENABLE_bm; // Use Timer A as clock, enable timer
  sei(); //Enable back the interrupts
}

void loop() {
  switch (robotState) {
    case 0: // leave start area and turn left
        Serial.println("State 0");
        stop();

        // robot starts moving when we push the start buttonn
/*         while (start==false) {
            if (digitalRead(startButtonPin) == true) {
                start=true;
                break;
            }
        } */

        forward();
        delay(1000);
        if (OSwitchReadings() == "0000") {
            rotateLeft(90);
        }
        robotState = 1;
        break;

    case 1: // line following
        follow_line();

        // detect left intersection
        if (OSwitchReadings() == "0111") {
            count++;

            if (holdingBlock) {
                if (redBlock && count%4 == 2) {
                    robotState = 3;
                    break;
                } else if (!redBlock && count%4==0) {
                    robotState = 3;
                    break;
                }
            }
        }

        // detect block with push button
        bool buttonState = digitalRead(blockButtonPin);
        if (buttonState==1) {
            robotState = 2;
            break;
        }

        break;

    case 2: // pick up block and identify colour
        stop();
        holdingBlock = true;

        // activate gripper

        delay(1000); // wait till gripper has finnished moving
        redBlock = detectColour();

        if (redBlock) {
            Serial.print("Block colour is red");
        } else {
            Serial.print("Block colour is blue");
        }
        delay(500);

        robotState = 1;
        break;

    case 3: // robot puts down block
        // enter box
        stop();
        rotateLeft(90);
        forward();
        delay(1000);
        stop();

        // raise gripper and release block
        holdingBlock = false;
        delay(2000); // wait till gripper has finnished movinng

        // reverse out
        backward();
        delay(1000);
        rotateRight(90);
        
        robotState = 1;
        break;
  }
}

// functions go here

// navigation functions
void follow_line() {
  String newSensorReading = OSwitchReadings();
  // if new readings are the same as the old readings, don't send repeated commands to the motors
  if (newSensorReading == sensorReading) {
    return;
  }
  sensorReading = newSensorReading;

  if (sensorReading == "1001") {
    forward();
  } else if (sensorReading == "1011") {
    turnLeft();
  } else if (sensorReading == "1101") {
    turnRight();
  } else if (sensorReading == "1111") {
    forward();
}
}

// sensor functions

// Returns line following sensor readings in binary (left to right)
String OSwitchReadings() {
    bool V1 = digitalRead(farLeftOSwitch); // 1=black, 0=white
    bool V2 = digitalRead(leftOSwitch);
    bool V3 = digitalRead(rightOSwitch);
    bool V4 = digitalRead(farRightOSwitch);

    // convert 4 booleans to a string
    String newSensorReading = String(V1) + String(V2) + String(V3) + String(V4);
    return newSensorReading;
} 

// Returns boolean to indicate colour (Red=1, blue=0). Averages 5 sensor readings
bool detectColour() {
    int count = 0;
    for (int i = 0; i < 5; i++) {
        if (digitalRead(colourOSwitch) == 1) {
            count++;
        }
        delay(100);
    }
    return (count >= 3);
}

// functions for movement
void forward() {
  leftMotor->setSpeed(motorSpeed1);
  rightMotor->setSpeed(motorSpeed1);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
  LEDState = true;
}

void backward() {
  leftMotor->setSpeed(motorSpeed1);
  rightMotor->setSpeed(motorSpeed1);
  leftMotor->run(BACKWARD);
  rightMotor->run(BACKWARD);
  startLED = true;
}

void stop() {
    leftMotor->run(RELEASE);
    rightMotor->run(RELEASE);
    startLED = false;
}

void turnLeft() {
    leftMotor->setSpeed(motorSpeed1);
    rightMotor->setSpeed(motorSpeed1);
    leftMotor->run(RELEASE);
    rightMotor->run(FORWARD);
}

void turnRight() {
    leftMotor->setSpeed(motorSpeed1);
    rightMotor->setSpeed(motorSpeed1);
    leftMotor->run(FORWARD);
    rightMotor->run(RELEASE);
}

void turnBack() {
   leftMotor->setSpeed(motorSpeed1);
   rightMotor->setSpeed(motorSpeed1);
   leftMotor->run(FORWARD);
   rightMotor->run(BACKWARD);
   delay(200);
}

void rotateLeft(int degrees) {
    leftMotor->setSpeed(motorSpeed1);
    rightMotor->setSpeed(motorSpeed1);
    startLED = true;
    leftMotor->run(BACKWARD);
    rightMotor->run(FORWARD);
    delay((1000/90)*degrees); // 1000 is time taken to turn 90 degrees (assume 1 second for now)
    leftMotor->run(RELEASE);
    rightMotor->run(RELEASE);
}

void rotateRight(int degrees) {
    leftMotor->setSpeed(motorSpeed1);
    rightMotor->setSpeed(motorSpeed1);
    startLED = true;
    leftMotor->run(FORWARD);
    rightMotor->run(BACKWARD);
    delay((1000/90)*degrees); // 1000 is time taken to turn 90 degrees (assume 1 second for now)
    leftMotor->run(RELEASE);
    rightMotor->run(RELEASE);
}

// LED functions

// Flashing amber LED. Triggered every 500ms (2 Hz)
ISR(TCB0_INT_vect)
{
// Do something
if (startLED) { // startLED = true when motors are running
  LEDState = !LEDState; // toggle LED bbetween on and off
  } else {
    LEDState = 0; // turn off LED
  }
digitalWrite(amberLEDPin, LEDState);
// Clear interrupt flag
TCB0.INTFLAGS = TCB_CAPT_bm;
}