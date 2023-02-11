// line folloiwng program with recovery mode.
// 2 workinng sennsors (LHS and RHS ones set to 0);
// if reading = 0000, robot enters recovery mode. it reverses unntil it finds a line again

//---------------
// TASKS
// ***turnLeft and turnRight are goinng in opposite directions???

// 1. tune robot speed - can it make it up the ramp?, is it going too slow/fast? (it may oscillate if too fast)
// 2. leftTurnn/rightTurn algorithms. e.g. Should we set the other motor to RELEASE or BACKWWARDS?
// 3. Rotate time. Time the number of milliseconds taken to turn 90 degrees. Set rotateTime = this value.
// 4. Please test checkRightTurnn and recovery2. Supposedly, if the robot gets too close to the front wall, 
// it will turn right then use recovery2 to find the lines again.
// 5. measure and enter distances annd times
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
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2); // pin M2
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1); // pin M1

// push button input pin
const int startButtonPin = 2;

// microswitch input pin
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
#define USonic2Trigger A0
#define USonic2Echo A1
#define MAX_DISTANCE 100
NewPing sonar1(USonic1Trigger, USonic1Echo, MAX_DISTANCE);
NewPing sonar2(USonic2Trigger, USonic2Echo, MAX_DISTANCE);

// pin for colour sensor
#define colourSensor A2

// IR sensor (80cm) pin
#define IRSensor A3

// variable initialisation

// variables we can change
// motor and movement variables
int motorSpeed1 = 255; // speed ranges from 0 to 255
const int rotateTime = 1600; // time taken to rotate 90 degrees

// box times
const int leaveBoxTime = 1000; // time taken to leave box and cross first intersectionn
const int followLineTime1 = 1000; // time taken to go from first to second intersectionn after leaving box
const int followLineTime2 = 3000; // time taken to go from red/green box to white box

// wall distances
const int frontWallDistance1 = 5; // distance (in cm) from front of robot to wall when it should turn (Before tunnel and before ramp)
const int frontWallDistance2 = 10; // distance (in cm) from front of robot to wall when it should turn (after tunnel and after ramp)
const int frontWallDistance3 = 50; // dist from start box to front wall
const int leftWallDistance = 4; // distance (in cm) from left of robot to wall in tunnel
const int wallDistanceTolerance = 1; // tolerannce for usonnic sensor readings

// block distannces and times
const int blockDistance = 2; // if fronntDist smaller than this, we go forwrad a bit and pick up the block
const int IRThreshold = 400; // if analog IR reading bigger than this, block detected.
const int blockTime = 1000; // time to move forward to get the block if it is in proximity

// variables that don't need changing
int robotState = 0; // controls state machine
int leftCount = 0; // counts number of left intersections
int tCount = 0; // counts number of T inntersections
bool buttonState = false;
bool holdingBlock = false;
bool redBlock = false; // green = 0, red = 1
bool startProgram = false;
String sensorReading = "0101"; 
String previousSensorReading = "0101";

// for flashing amber LED when moving
bool LEDState = false; 
bool startLED = false;

// timer to exit loops
unsigned long currentTime;
unsigned long startTime;

// for ultrasonnic distannce readings
float leftDist = leftWallDistance;
float frontDist = frontWallDistance1;

// for block pick up annd return
bool RHSBlock = false;

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

  //robotState = 1; // choose robot state to 
}

void loop() {
        //previousSensorReading = sensorReading;
        followLine();

        previousSensorReading = sensorReading;
}

// functions go here
// navigation functions

// updates line sensor readings and sends commands to motors based on readings
void followLine() {
  //Serial.println("Line following");
  String newSensorReading = OSwitchReadings();
  Serial.println(newSensorReading);
  // if new readings are the same as the old readings, don't send repeated commands to the motors
  if (newSensorReading == sensorReading) {
    return;
  }
  sensorReading = newSensorReading;

  if (sensorReading == "0110") { // 1 is white, 0 is black
    forward();
  } else if (sensorReading == "0100") {
    turnLeft(); // swap??? Robot is turning left when we are telling it to turnn right
  } else if (sensorReading == "0010") {
    turnRight();
  } else if (sensorReading == "0000" && robotState != 1) { // robot is off-course (unless it is in the tunnnel). Enter recovery mode.
    //recover();
    //stop();
    //forward();
}
}

// checks if robot should've turned right at this point. If yes, forces right turn and activates forward recovery to find the line.
// checks if robot should've turned right at this point. If yes, forces right turn and activates forward recovery to find the line.

// Returns line following sensor readings in binary (left to right)
String OSwitchReadings() {
    bool V1 = digitalRead(farLeftOSwitch); // 1=black, 0=white
    bool V2 = digitalRead(leftOSwitch);
    bool V3 = digitalRead(rightOSwitch);
    // V4 isn't working. set to 0.
    bool V4 = digitalRead(farRightOSwitch);
    //bool V1 = 0;
    //bool V4 = 0;

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

