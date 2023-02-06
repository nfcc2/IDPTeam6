// line folloiwng program with recovery mode.
// 2 workinng sennsors (LHS and RHS ones set to 0);
// if reading = 0000, robot enters recovery mode. it reverses unntil it finds a line again

//---------------
// TASKS
// ***turnLeft and turnRight are goinng in opposite directions???


// 1. tune robot speed - can it make it up the ramp?, is it going too slow/fast? (it may oscillate if too fast)
// 2. leftTurnn/rightTurn algorithms. e.g. Should we set the other motor to RELEASE or BACKWWARDS?
// 3. Rotate time. Time the number of milliseconds taken to turn 90 degrees. Set rotateTime = this value.
// 4. Test recovery algorithm. Does it activate when it should? Can the robot find the line and do the right thing?
// --------------
// after these tasks are done - see basic_main_3
// 1. can the robot effectively count the number of intersections? (currenntly, count = 0 after tunnel. counnt = 2 at red, count = 4 at green
// 2. we need to test the main algorithm (basic_main_3) - does the robot go innto the red box with a red block?

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

// 3 white
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

// variables for testing
const int motorSpeed1 = 255; // speed ranges from 0 to 255
const int rotateTime = 1600; // time taken to rotate 90 degrees

// other variables
int robotState = 0;
int count = 0; // counts number of left intersections
bool buttonState = false;
bool holdingBlock = false;
bool redBlock = false;
bool startProgram = false;
String sensorReading = "0101"; 
String previousSensorReading = "0101";
String newSensorReading = "0101";
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
        //previousSensorReading = sensorReading;
        followLine();

            // detect left intersection
        if (sensorReading == "1110") { // if sennsorReading = 1110
            if (previousSensorReading == "1110") { // do nothinng if this isn't first instance of detecting the junnctionn
                forward();
            } else { // first detection
                count++;
                Serial.print("This is intersection nnumber ");
                Serial.print(count);
                if (holdingBlock) {
                    if (redBlock && count==2) { // ennter red box
                        robotState = 5;
                        
                    } else if (!redBlock && count==4) { // enter green box
                        robotState = 5;
                        
                    }
                }
            }
        }

        previousSensorReading = sensorReading;
}

// functions go here
// navigation functions

// updates line sensor readings and sends commands to motors based on readings
void followLine() {
  Serial.println("Line following");
  newSensorReading = OSwitchReadings();
  Serial.println(newSensorReading);
  // if new readings are the same as the old readings, don't send repeated commands to the motors
  if (newSensorReading == sensorReading) {
    return;
  }
  sensorReading = newSensorReading;

  if (sensorReading == "0110") { // 1 is white, 0 is black
    forward();
  } else if (sensorReading == "0100") {
    turnRight(); // swap??? Robot is turning left when we are telling it to turnn right
  } else if (sensorReading == "0010") {
    turnLeft();
  } else if (sensorReading == "0000" && robotState != 1) { // robot is off-course (unless it is in the tunnnel). Enter recovery mode.
    //recover();
    stop();
    //forward();
}
}

// Robot reverses until it detects the line.
void recover() {
    int recoveryState = 0;
    motorSpeed1 = 200; // maybe reducing motor speed will help detecting changes in line sensor readings?
    backward();

    for(;;) {
    Serial.println("Recovery");
    switch (recoveryState) {
      case 0:
        newSensorReading = OSwitchReadings();
        // if new readings are the same as the old readings, don't send repeated commands to the motors
        if (newSensorReading == sensorReading) {
            recoveryState = 0;
            continue;
        }

        // robot finds the line - approaching from RHS
        if (newSensorReading == "0100") {
            recoveryState = 1;
            backLeft(); // left wheel goes backwards
            continue;

        // robot approaches line from LHS
        } else if (newSensorReading == "0010") {
          recoveryState = 2;
          backRight();
          continue;
        } 
        sensorReading = newSensorReading; 

      recoveryState = 0;
      continue;

      case 1: // robot approached line from RHS
        newSensorReading = OSwitchReadings();
        // if new readings are the same as the old readings, don't send repeated commands to the motors
        if (newSensorReading == sensorReading) {
            recoveryState = 1;
            continue;
        }
        sensorReading = newSensorReading; 

        // the other sensor reaches line - start going forwards inn opposite directionn
        if (sensorReading == "0110" || sensorReading == "0010") {
            //stop(); - so motors don't get confused by reverse commands?

            // test this
            turnLeft();
            motorSpeed1 = 255;
            break; // return to line following
        } 

      recoveryState = 1;
      continue;
    
    case 2:
        newSensorReading = OSwitchReadings();
        // if new readings are the same as the old readings, don't send repeated commands to the motors
        if (newSensorReading == sensorReading) {
            recoveryState = 1;
            continue;
        }
        sensorReading = newSensorReading; 

        if (sensorReading == "0110" || sensorReading == "0010") {
          //stop();
            turnRight();
            motorSpeed1 = 255;
            break;
        } 

      recoveryState = 2;
      continue;

    }
    break;
}
}

// sensor functions

// Returns line following sensor readings in binary (left to right)
String OSwitchReadings() {
    //bool V1 = digitalRead(farLeftOSwitch); // 1=black, 0=white
    bool V2 = digitalRead(leftOSwitch);
    bool V3 = digitalRead(rightOSwitch);
    // V4 isn't working. set to 0.
    //bool V4 = digitalRead(farRightOSwitch);
    bool V1 = 0;
    bool V4 = 0;

    // convert 4 booleans to a string
    String reading = String(V1) + String(V2) + String(V3) + String(V4);
    return reading;
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

void backLeft() {
    leftMotor->setSpeed(motorSpeed1);
    rightMotor->setSpeed(motorSpeed1*0.5);
    leftMotor->run(BACKWARD);
    rightMotor->run(BACKWARD);
}

void backRight() {
    leftMotor->setSpeed(motorSpeed1*0.5);
    rightMotor->setSpeed(motorSpeed1);
    leftMotor->run(BACKWARD);
    rightMotor->run(BACKWARD);
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

