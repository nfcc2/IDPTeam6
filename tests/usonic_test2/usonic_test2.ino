// use line following sensors to go one loop clockwise around the arena. Pick up block if detected using push button.
//assumed farRightOswitch isn't working. set to 0.
// added ultrasonic sensor redundancies in tunnel

// tasks
// test if millis() works
// if so add timing redundancies after testinng the algprithm of this program
// tune motor functions and experimet with motor speed for ramp annd turns
// make annother sweep algprithm (look for consecutive small distances (round to nearest cm). )
// flowcharts

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
Adafruit_DCMotor *leftMotor = AFMS.getMotor(2); // pin M2

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
#define USonic2Trigger A0
#define USonic2Echo A1
#define MAX_DISTANCE 100
NewPing sonar1(USonic1Trigger, USonic1Echo, MAX_DISTANCE);
NewPing sonar2(USonic2Trigger, USonic2Echo, MAX_DISTANCE);

// optoswitch pin for colour identification
#define colourOSwitch A2

// variable initialisation

// variables we can change
// motor and movement variables
const int motorSpeed1 = 150; // speed ranges from 0 to 255
const int rotateTime = 5000; // time taken to rotate 90 degrees

// times
const int leaveBoxTime = 1000; // time taken to leave box and cross first intersectionn
const int followLineTime1 = 1000; // time taken to go from first to second intersectionn after leaving box
const int followLineTime2 = 3000; // time taken to go from red/green box to white box

// distances
const int frontWallDistance = 10; // distance (in cm) from front of robot to wall when it should turn
const int leftWallDistance = 5; // distance (in cm) from left of robot to wall in tunnel
const int wallDistanceTolerance = 1; // tolerannce for usonnic sensor readings

// variables that don't need changing
int robotState = 0; // controls state machine
int count = 0; // counts number of left intersections
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
float frontDist = frontWallDistance;
float d = leftWallDistance;

void setup() {
  AFMS.begin();
  Serial.begin(9600);

  // set pinmode for buttons and LEDs. Configured to INPUT by default.
  pinMode(blockButtonPin, INPUT_PULLUP);
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

  findBlock2();
}

void loop() {
  Serial.println(frontDistance());
}

// functions go here
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

// Robot goes forward until it detects the line.
void recover2() {
    forward();
    while (sensorReading == "0000") {
        String newSensorReading = OSwitchReadings();
        // if new readings are the same as the old readings, don't send repeated commands to the motors
        if (newSensorReading == sensorReading) {
            continue;
        }
        sensorReading = newSensorReading;

        if (sensorReading == "0010") { // 1 is white, 0 is black
            turnLeft();
        } else if (sensorReading == "0100") {
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

// ultrasonic sensor and sweep functions

// returns distance measured by front ultrasonic sensor in cm
float frontDistance() {
  return (sonar1.ping_median(5) / 2) * 0.0343;
}

// distannce measured by left usonic sensor
float leftDistance() {
  return (sonar2.ping_median(5) / 2) * 0.0343;
}

// robot rotates and stops when distace from the block is the smallest - look for 3 consecutive small readings?
// trigger using intersection? or do a smaller angle swweep from further away? e.g. use gyroscope. sweep once straight after tunnel
// returns booleann foundBlock. true if founnd, false if not
// would robot mistake wall as block?
bool findBlock() { 
    int minDistance = 13; // give min dist a threshold e.g. 10cm
    float d = 100;
    const int sweepAngle = 120;
    const int halfSweepTime = (rotateTime/90)*(sweepAngle/2);
    unsigned long startSweepTime = millis();
    unsigned long currentSweepTime = millis() + 1;
    int state = 0;
    int sweepCount = 0;

    for(;;) { // set up loop. using continue brings us to start of loop. break breaks out of loop
    switch (state) {
        case 0: // start sweep to the left
        rotate(true);
        startSweepTime = millis();
        currentSweepTime = millis() + 1;
        state = 1;
        continue;

        case 1:
        if (currentSweepTime - startSweepTime >= halfSweepTime) { // finished half sweep
        Serial.println("Finnsihed sweep");
            stop();
            sweepCount++;
            if (sweepCount == 3) {
            state = 3;
            continue;
            } else {
            state = 2;
            continue;
            }
        }

        // record minimum distance
        d = frontDistance();
        Serial.println(d);
        if (d != 0) { // filter out '0' annomalous readings
            if (d < minDistance) {
                minDistance = d;
            } 
        }
        
        currentSweepTime = millis(); 
        state = 1;
        continue;

        case 2: // start sweep to the right
        rotate(false);
        startSweepTime = millis();
        currentSweepTime = millis() + 1;
        
        state = 1;
        continue;

        case 3: // start final sweep to the left to locate block
        rotate(true);
        startSweepTime = millis();
        currentSweepTime = millis() + 1;
        state = 4;
        continue;

        case 4: // finnal full sweep to locate block
        if (currentSweepTime - startSweepTime >= halfSweepTime*2) { // finished full sweep
            stop(); // can't find block
            Serial.println("Can't find block");
            return false;
        }

        d = frontDistance();
        if (minDistance - 1 < d && d < minDistance + 1) {
            stop(); // found block
            Serial.println("Found block");
            return true;
        }
        currentSweepTime = millis(); 
        state = 4;
        continue;
    }
    break;
    }
}

// looks for 2 consecutive small readings instead of min distance to detect block
bool findBlock2() {
    float prevDist = 100; // stores previous distance readinng
    int minDistance = 14;
    const int sweepAngle = 120;
    const int halfSweepTime = (rotateTime/90)*(sweepAngle/2);
    unsigned long startSweepTime = millis();
    unsigned long currentSweepTime = millis() + 1;
    int state = 0;
    int sweepCount = 0;

    for(;;) { // set up loop. using continue brings us to start of loop. break breaks out of loop
    switch (state) {
        case 0: // start sweep to the left
        rotate(true);
        startSweepTime = millis();
        currentSweepTime = millis() + 1;
        state = 1;
        continue;

        case 1: // sweepinng and recording distannces
        if (currentSweepTime - startSweepTime >= halfSweepTime) { // finished half sweep
            stop();
            sweepCount++;
            if (sweepCount == 3) {
                state = 0;
            } else if (sweepCount == 4) {
              Serial.println("Block not found");
                stop();
                return false;
            } else {
                state = 2;
            }
            continue;
            }

        d = frontDistance();
        if (d != 0) { // filter out '0' annomalous readings

            // checks if previous annd current distance readings are similar and small enough < 14cm (ie block detected)
            if ((d < minDistance) && (abs(prevDist - d) < 1)) {
                // block found
                Serial.println("Block found");
                stop();
                return true;
            }

            prevDist = d;
        }
        
        currentSweepTime = millis(); 
        state = 1;
        continue;

        case 2: // start sweep to the right
        rotate(false);
        startSweepTime = millis();
        currentSweepTime = millis() + 1;
        state = 1;
        continue;
    }
    break;
    }
}

// start sweep to locate block
void rotate(bool left) {
  Serial.println("Starting sweep");
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