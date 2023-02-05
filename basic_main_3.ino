// use line following sensors to go one loop clockwise around the arena. Pick up block if detected using push button.
//assumed farRightOswitch isn't working. set to 0.

//tasks
// test recover()
// add timing redundancies after testinng the algprithm of this program
// tune motor functions and experimet with motor speed for ramp annd turns
// make annother sweep algprithm (look for consecutive small distances (round to nearest cm). )

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
bool buttonState = false;
bool holdingBlock = false;
bool redBlock = false;
bool startProgram = false;
String sensorReading = "0101"; 
String previousSensorReading = "0101";
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
        Serial.println("State 0: leave start area and turn left");
        stop();
        forward();
        delay(1000); // allow enough time for robot to leave start box and cross first intersection

        // robot starts moving when we push the start buttonn
/*         while (startProgram==false) {
            if (digitalRead(startButtonPin) == false) { // pullup mode: output = LOW when button is pressed
                startProgram=true;
                break;
            }
        } */
        
        // follow line until we reach the intersection
        //startTime = millis(); // commented out timinng redundancies
        //currentTime = millis();
        while (sensorReading != "1110") { 
            followLine();
            //currentTime = millis();
/*             if ((currentTime - startTime) >= 2000) { // 2000 is a bit more than time taken to go from first intersectionn to second itnersectionn
                break;
            } */
        }
        
        rotateLeft(90);
        robotState = 1;
        startTime = millis();
        break;

    case 1: // line following until tunnel
        Serial.println("State 1: line following unntil tunnel");
        followLine();
        //currentTime = millis();

        // robot reached tunnel
        // commented out timing redunndanncies
/*         if (sensorReading == "0000" && (currentTime - startTime) > 6000) { // robot in tunnel. 6000 is time taken to reach tunnel after left turn
            robotState = 2;
            forward();
            break;
        } else {
            recover();
        } */

        if (sensorReading == "0000") {
            robotState = 2;
            count = 0; // reset intersection count after tunnel
            forward();
            break;
        }

        robotState = 1;
        break;

    case 2: // through tunnel
        Serial.println("State 2: inn tunnel");
        sensorReading = OSwitchReadings();

        if (sensorReading == "0100" || sensorReading == "0100" || sensorReading == "0110") { // reached end of tunnel
            robotState = 3;
            break;
        }

        robotState = 2;
        break; 

    case 3: // line following after tunnel
        Serial.println("State 3: line folowing after tunnel");
        followLine();

            // detect left intersection
        if (sensorReading == "1110") { // if sennsorReading = 1110
            if (previousSensorReading == "1110") { // do nothinng if this isn't first instance of detecting the junnctionn
                forward();
            } else { // first detection
                count++;
                if (holdingBlock) {
                    if (redBlock && count==2) { // ennter red box
                        robotState = 5;
                        break;
                    } else if (!redBlock && count==4) { // enter green box
                        robotState = 5;
                        break;
                    }
                }
            }
        }

        previousSensorReading = sensorReading;

        // detect block with push button
        buttonState = !digitalRead(blockButtonPin); // pullup mode - 1=open, 0=closed
        if (buttonState==1) {
            robotState = 4;
            break;
        }

        robotState = 3;
        break;

    case 4: // pick up block and identify colour
    Serial.println("State 4: pick up block and identify colour");
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
        robotState = 3;
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
        rotateLeft(180);
        delay(1000);

        // if time close to 5 mins, return to start box
        
        robotState = 0;
        break;
  }
}

// functions go here

// navigation functions

// updates line sensor readings and sends commands to motors based on readings
void followLine() {
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
  } else if (sensorReading == "0000" && robotState != 1) { // robot is off-course (unless it is in the tunnnel). Enter recovery mode.
    recover();
}
}

// Robot reverses until it detects the line.
void recover() {
    backward();
    while (sensorReading == "0000") {
        String newSensorReading = OSwitchReadings();
        // if new readings are the same as the old readings, don't send repeated commands to the motors
        if (newSensorReading == sensorReading) {
            continue;
        }
        sensorReading = newSensorReading;

        if (sensorReading == "0100") { // 1 is white, 0 is black
            turnLeft();
        } else if (sensorReading == "0010") {
            turnRight();
        }
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