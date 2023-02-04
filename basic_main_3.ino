// use line following sensors to go one loop clockwise around the arena. Pick up block if detected using push button.
//assumed farRightOswitch isn't working. set to 0.

//tasks
// test if millis() works
//if so add timing redundancies after testinng the algprithm of this program - by inputting line sensor ips?

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
const int startButtonPin = 2;
const int blockButtonPin = 3;

// optoswitch pins for line following
#define farLeftOSwitch 4
#define leftOSwitch 5
#define rightOSwitch 6
#define farRightOSwitch 7

// LED output pins
const int amberLEDPin = 8;
const int redLEDPin = 10;
const int greenLEDPin = 11;

// ultrasonnic pins
#define USonic1Trigger 12
#define USonic1Echo 13
#define MAX_DISTANCE 100
NewPing sonar1(USonic1Trigger, USonic1Echo, MAX_DISTANCE);

// optoswitch pin for colour identification
#define colourOSwitch A2

// variable initialisation
int robotState = 0;
const int motorSpeed1 = 150; // speed ranges from 0 to 255
const int rotateTime = 1600; // time taken to rotate 90 degrees
int count = 0; // counts number of left intersections
bool holdingBlock = false;
bool redBlock = false;
bool startProgram = false;
String sensorReading = "0101"; 
// for flashing amber LED when moving
bool LEDState = false; 
bool startLED = false;
// timer to exit loops
unsigned long currentTime;
unsigned long startTime;

void setup() {
  AFMS.begin();
  Serial.begin(9600);
  pinMode(blockButtonPin, INPUT_PULLUP);
  pinMode(startButtonPin, INPUT_PULLUP);
  pinMode(amberLEDPin, OUTPUT);
  
  // set up timer interrupt for flashing amber LED
  cli();
  TCB0.CTRLB = TCB_CNTMODE_INT_gc;
  TCB0.CCMP = 125000; // Value to compare with. 25000 is 10 Hz
  TCB0.INTCTRL = TCB_CAPT_bm; 
  TCB0.CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_ENABLE_bm; 
  sei(); 
}

void loop() {
  switch (robotState) {
    case 0: // leave start area and turn left
        Serial.println("State 0");
        stop();
        forward();
        delay(1000); // allow enough time for robot to leave start box and cross first intersection

        // robot starts moving when we push the start buttonn
/*         while (startProgram==false) {
            if (digitalRead(startButtonPin) == false) {
                startProgram=true;
                break;
            }
        } */
        
        // follow line until we reach the intersection
        startTime = millis();
        currentTime = millis();
        while (sensorReading != "1110") { 
            follow_line();
            currentTime = millis();
/*             if ((currentTime - startTime) >= 2000) { // 2000 is a bit more than time taken to go from first intersectionn to second itnersectionn
                break;
            } */
        }
        
        rotateLeft(90);
        robotState = 1;
        startTime = millis();
        break;

    case 1: // line following until tunnel
        follow_line();
        currentTime = millis();

        // robot reached tunnel
/*         if (sensorReading == "0000" && (currentTime - startTime) > 6000) { // robot in tunnel. 6000 is time taken to reach tunnel after left turn
            robotState = 2;
            break;
        } */

        if (sensorReading == "0000") {
            robotState = 2;
            forward();
            break;
        }

        robotState = 1;
        break;

    case 2: // through tunnel
        sensorReading = OSwitchReadings();

        if (sensorReading == "0100" || sensorReading == "0100" || sensorReadinng == "0110") { // reached end of tunnel
            robotState = 3;
            break;
        }

        robotState = 2;
        break; 

    case 3: // line following after tunnel

            // detect left intersection
        if (OSwitchReadings() == "1110") {
            if (sensorReading == "1110") {
                forward();
            } else {
                count++;
                sensorReading = "1110";
                if (holdingBlock) {
                    if (redBlock && count==1) {
                        robotState = 5;
                        break;
                    } else if (!redBlock && count==3) {
                        robotState = 5;
                        break;
                    }
                }
        }
        }

        // detect block with push button
        bool buttonState = digitalRead(blockButtonPin);
        if (buttonState==1) {
            robotState = 4;
            break;
        }

    case 4: // pick up block and identify colour
        stop();
        holdingBlock = true;

        // activate gripper

        delay(1000); // wait till gripper has finnished moving
        redBlock = detectColour();

        if (redBlock) {
            Serial.print("Block colour is red");
            digitalWrite(redLEDPin, HIGH);
            delay(5000);
            digitalWrite(redLEDPin, LOW);
        } else {
            Serial.print("Block colour is blue");
            digitalWrite(greenLEDPin, HIGH);
            delay(5000);
            digitalWrite(greenLEDPin, LOW);
        }

        delay(500);
        count = 0;
        robotState = 1;
        break;

    case 5: // robot puts down block
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

  if (sensorReading == "0110") { // 1 is white, 0 is black
    forward();
  } else if (sensorReading == "0100") {
    turnLeft();
  } else if (sensorReading == "0010") {
    turnRight();
  } 
}

// sensor functions

// Returns line following sensor readings in binary (left to right)
String OSwitchReadings() {
    bool V1 = digitalRead(farLeftOSwitch); // 1=black, 0=white
    bool V2 = digitalRead(leftOSwitch);
    bool V3 = digitalRead(rightOSwitch);
    // V4 isn't working. set to 0.
    //bool V4 = digitalRead(farRightOSwitch);
    bool V4 = 0;

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

// returns distance measured by front ultrasonic sensor in cm
int frontDistance() {
  const int iterations = 5;
  return (sonar1.ping_median(iterations) / 2) * 0.0343;
}

// robot rotates and stops when distace from the block is the smallest
// trigger using intersection? or do a smaller angle swweep from further away? e.g. use gyroscope. sweep once straight after tunnel
int findBlock() { 
    int minDistance = 100;
    float d = 100;
    const int sweepAngle = 180;
    const int halfSweepTime = (rotateTime/90)*(sweepAngle/2);
    unsigned long startSweepTime = millis();
    unsigned long currentSweepTime = millis() + 1;
    int state = 0;
    int sweepCount = 0;

    for(;;) {
    switch (state) {
        case 0:
        rotate(true);
        startSweepTime = millis();
        state = 1;
        continue;

        case 1:
        if (currentSweepTime - startSweepTime >= halfSweepTime) { // finished half sweep
            sweepCount++;
            if (sweepCount == 3) {
            state = 3;
            continue;
            }  else {
            state = 2;
            continue;
            }
        }

        if (d != 0) {
            d = frontDistance();
        }
        
        if (d < minDistance) {
            minDistance = d;
        } 
        currentSweepTime = millis(); 
        state = 1;
        continue;

        case 2:
        rotate(false);
        startSweepTime = millis();
        state = 1;
        continue;

        case 3:
        rotate(true);
        startSweepTime = millis();
        state = 4;
        continue;

        case 4:
        if (currentSweepTime - startSweepTime >= halfSweepTime*2) { // finished full sweep
            stop(); // can't find block
        }

        d = frontDistance();
        if (int(d) == int(minDistance)) { // round to integers
            stop(); // found block
            break;
        }
        currentSweepTime = millis(); 
        state = 4;
        continue;
    }
    break;
    }
}

// start sweep to locate block
void rotate(bool left) {
  leftMotor->setSpeed(motorSpeed1);
  rightMotor->setSpeed(motorSpeed1);
  startLED = true;
  if (left) {
    leftMotor->run(BACKWARD);
    rightMotor->run(FORWARD);
  } else {
    leftMotor->run(FORWARD);
    rightMotor->run(BACKWARD);
  }
}

// functions for movement
void forward() {
  leftMotor->setSpeed(motorSpeed1);
  rightMotor->setSpeed(motorSpeed1);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
  startLED = true;
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

void rotateLeft(int degrees) {
    leftMotor->setSpeed(motorSpeed1);
    rightMotor->setSpeed(motorSpeed1);
    startLED = true;
    leftMotor->run(BACKWARD);
    rightMotor->run(FORWARD);
    delay((rotateTime/90)*degrees);
    leftMotor->run(RELEASE);
    rightMotor->run(RELEASE);
}

void rotateRight(int degrees) {
    leftMotor->setSpeed(motorSpeed1);
    rightMotor->setSpeed(motorSpeed1);
    startLED = true;
    leftMotor->run(FORWARD);
    rightMotor->run(BACKWARD);
    delay((rotateTime/90)*degrees);
    leftMotor->run(RELEASE);
    rightMotor->run(RELEASE);
}

// LED functions

// Flashing amber LED. Triggered every 500ms (2 Hz)
ISR(TCB0_INT_vect) {
if (startLED) { // startLED = true when motors are running
  LEDState = !LEDState; // toggle LED bbetween on and off
  } else {
    LEDState = 0; // turn off LED
  }
digitalWrite(amberLEDPin, LEDState);
// Clear interrupt flag
TCB0.INTFLAGS = TCB_CAPT_bm;
}