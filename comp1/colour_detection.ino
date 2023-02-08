// use line following sensors to go one loop clockwise around the arena. Pick up block if detected using push button.
//assumed farRightOswitch isn't working. set to 0.
// added ultrasonic sensor redundancies in tunnel

// tasks
// if so add timing redundancies after testinng the algprithm of this program
// tune motor functions and experimet with motor speed for ramp annd turns

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
// const int blockButtonPin = 3;

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
#define colourSensor 3

// IR sensor (80cm) pin
#define IRSensor A2

// variable initialisation

// variables we can change
// motor and movement variables
int motorSpeed1 = 255; // speed ranges from 0 to 255
const int rotateTime = 1600; // time taken to rotate 90 degrees

// box times
const int leaveBoxTime = 1000; // time taken to leave box and cross first intersectionn
const int enterBoxTime = 3000; // time taken to enter box from inntersection
const int followLineTime1 = 1000; // time taken to go from first to second intersectionn after leaving box
const int followLineTime2 = 3000; // time taken to go from red/green box to white box

// wall distances
const int frontWallDistance1 = 8; // distance (in cm) from front of robot to wall when it should turn (Before tunnel)
const int frontWallDistance2 = 20; // right turnn after tunnnel
const int frontWallDistance3 = 10; // right turnn before ramp
const int frontWallDistance4 = 40; // right turn after ramp
const int leftWallDistance = 5; // distance (in cm) from left of robot to wall in tunnel
const int wallDistanceTolerance = 1; // tolerannce for usonnic sensor readings

// block distannces and times
const int blockDistance = 3; // if fronntDist smaller than this, we go forwrad a bit and pick up the block
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

  // set pinmode for buttons and LEDs. Other pins are configured to INPUT by default.
  pinMode(startButtonPin, INPUT_PULLUP);
  pinMode(amberLEDPin, OUTPUT);
  pinMode(redLEDPin, OUTPUT);
  pinMode(greenLEDPin, OUTPUT);
  
  // set up timer interrupt for flashing amber LED
  cli();
  TCB0.CTRLB = TCB_CNTMODE_INT_gc;
  TCB0.CCMP = 125000; // Value to compare with. 25000 is 10 Hz
  TCB0.INTCTRL = TCB_CAPT_bm; 
  TCB0.CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_ENABLE_bm; 
  sei(); 

    // robot starts moving when we push the start buttonn
    while (startProgram==false) {
    if (digitalRead(startButtonPin) == false) { // pullup mode: output = LOW when button is pressed
        startProgram=true;
        break;
    }
    } 

}

void loop() {
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

        if (RHSBlock) { // need to reverse annd return to linne following
            rotateLeft(180);
            forward();
            delay(blockTime);
            rotateRight(90);
        }
}

// functions go here
// updates line sensor readings and sends commands to motors based on readings
void followLine() {
  Serial.println("Line following");
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
    turnLeft();
  } else if (sensorReading == "0010") {
    turnRight();
  } //else if (sensorReading == "0000" && robotState != 1) { // robot is off-course (unless it is in the tunnnel). Enter recovery mode.
    //recover();
    //stop();
    //forward();
}

// Robot goes forward until it detects the line.
void recover2() {
    int recovery2State = 0;
    String newReading = "0101";
    motorSpeed1 = 200; // maybe reducing motor speed will help detecting changes in line sensor readings?
    forward();

    for(;;) {
    Serial.println("Recovery");
    switch (recovery2State) {
      case 0:
        newReading = OSwitchReadings();
        // if new readings are the same as the old readings, don't send repeated commands to the motors
        if (newReading == sensorReading) {
            recovery2State = 0;
            continue;
        }

        // robot finds the line - approaching from RHS
        if (newReading == "0100") {
            recovery2State = 1;
            turnRight();
            continue;

        // robot approaches line from LHS
        } else if (newReading == "0010") {
          recovery2State = 2;
          turnLeft();
          continue;
        } 
 
        // if robot is getting to close to wall on LHS, it should turn right
        leftDist = leftDistance();
        if (leftDist < leftWallDistance && leftDist != 0) { // filter out anonomalous zero readings
            rotateRight(10);
        }

        sensorReading = newReading; 

        recovery2State = 0;
        continue;

      case 1: // robot approached line from RHS
        newReading = OSwitchReadings();
        // if new readings are the same as the old readings, don't send repeated commands to the motors
        if (newReading == sensorReading) {
            recovery2State = 1;
            continue;
        }
        sensorReading = newReading; 

        if (sensorReading == "0000" || sensorReading == "0010") { // robot should have recovered
            motorSpeed1 = 255;
            return;
        } 

      recovery2State = 1;
      continue;
    
    case 2:
        newReading = OSwitchReadings();
        // if new readings are the same as the old readings, don't send repeated commands to the motors
        if (newReading == sensorReading) {
            recovery2State = 1;
            continue;
        }
        sensorReading = newReading; 

        if (sensorReading == "0000" || sensorReading == "0100") {
          //stop();
            motorSpeed1 = 255;
            return;
        } 

      recovery2State = 2;
      continue;

    }
    break;
}
}

// checks if robot should've turned right at this point. If yes, forces right turn and activates forward recovery to find the line.
void checkRightTurn() {
    frontDist = frontDistance();
    int frontWallDistance;

    switch (robotState) {
        case 1:
        frontWallDistance = frontWallDistance1;
        case 2:
        frontWallDistance = frontWallDistance2;
        case 3:
        if (holdingBlock) {
            // frontWallDistance = frontWallDistance3;
            return; // block would mess up front usonic sennsor reading
        } else {
            return; // so robot can get close to block to pick it up
        }
        default:
        //frontWallDistance = frontWallDistance1;
        return;

    }

    if (frontDist < frontWallDistance && frontDist != 0) { // filter out anonomalous zero readings
        rotateRight(90);

        // robot has left ramp (unable to detect line). go to line followign once recovery is done.
        if (robotState == 4) { // ramp to line following
            leftCount = 0;
            robotState = 5;
        } else if (robotState == 2) { // tunnnel to line folllowing
            robotState = 3;
        }

        recover2();
    }
}

void checkLeftDistance() {
    leftDist = leftDistance();

    if (leftDist < (leftWallDistance - wallDistanceTolerance) && leftDist != 0) { // too close to left wall
        turnRight();
    } else if (leftDist > (leftWallDistance + wallDistanceTolerance) && leftDist != 0) {
        turnLeft();
    } else {
        forward();
    }
}

// sensor functions

// Returns line following sensor readings in binary (left to right)
String OSwitchReadings() {
    bool V1 = digitalRead(farLeftOSwitch); // 1=black, 0=white
    bool V2 = digitalRead(leftOSwitch);
    bool V3 = digitalRead(rightOSwitch);
    // V4 isn't working. set to 0.
    bool V4 = digitalRead(farRightOSwitch);
    //bool V4 = 0;

    // convert 4 booleans to a string
    String newSensorReading = String(V1) + String(V2) + String(V3) + String(V4);
    return newSensorReading;
} 

// Returns boolean to indicate colour (Red=1, blue=0). Averages 5 sensor readings
bool detectColour() {
    int j = 0;
    for (int i = 0; i < 5; i++) {
        if (digitalRead(colourSensor) == 1) {
            j++;
        }
        delay(100);
    }
    return (j >= 3);
}

// ultrasonic sensor functions
// returns distance measured by front ultrasonic sensor in cm
float frontDistance() {
  return (sonar1.ping_median(5) / 2) * 0.0343;
}

// distannce measured by left usonic sensor
float leftDistance() {
  return (sonar2.ping_median(5) / 2) * 0.0343;
}

// returns true if block detected on RHS by IR sensor
bool detectRHSBlock() {
    if (analogRead(IRSensor) > IRThreshold) {
        return true;
    } else {
        return false;
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
    startLED = true;
}

void turnRight() {
    leftMotor->setSpeed(motorSpeed1);
    rightMotor->setSpeed(motorSpeed1);
    leftMotor->run(FORWARD);
    rightMotor->run(RELEASE);
    startLED = true;
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