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
const int blockButtonPin = A3;
// ADD colour detection when button is pressed. this confirms we have the block.

// optoswitch pins for line following
#define farLeftOSwitch 4
#define leftOSwitch 5
#define rightOSwitch 7
#define farRightOSwitch 6 // nnot working

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
const int rotateTime = 1300; // time taken to rotate 90 degrees
const int rotateTimeDelay = 650; // time delay robot nneeds to go forward for before a 90 degree turn

// box times
const int leaveBoxTime = 1000; // time taken to leave box and cross first intersectionn
const int enterBoxTime = 3000; // time taken to enter box from inntersection
const int followLineTime1 = 11000; // time taken to go from start box intersection to tunnel
const int followLineTime2 = 30000; // time taken to go from outside tunnel to ramp
const int followLineTime3 = 3000; // time taken to go from red/green box to white box

//const int followLineTime2 = 4000; // time taken to go from outside tunnel to ramp FOR TESTING

// wall distances
const int frontWallDistance1 = 6; // distance (in cm) from front of robot to wall when it should turn (Before tunnel)
const int frontWallDistance2 = 20; // right turnn after tunnnel
const int frontWallDistance3 = 9; // right turnn before ramp
const int frontWallDistance4 = 40; // right turn after ramp
const int leftWallDistance1 = 5; // distance (in cm) from left of robot to wall in tunnel
const int leftWallDistance2 = 6; // distance from left to wall on ramp
const int leftWallDistance3 = 17; // distance from left to wall in area 3
const int wallDistanceTolerance = 1; // tolerannce for usonnic sensor readings
const int wallDistanceTolerance2 = 2; // tolerannce for usonnic sensor readings

// block distannces and times
const int blockDistance = 3; // if fronntDist smaller than this, we go forwrad a bit and pick up the block
const int IRThreshold = 400; // if analog IR reading bigger than this, block detected.
const int blockTime = 1000; // time to move forward to get the block if it is in proximity

// tunnel/ramp times
const int tunnelTime = 3500; // time taken to cross tunnnel
const int rampTime = 7000; // time taken to make it up the ramp

// variables that don't need changing
int robotState = 0; // controls state machine
int leftCount = 0; // counts number of left intersections
int tCount = 0; // counts number of T inntersections
bool buttonState = false;
bool holdingBlock = false;
bool redBlock = false; // green = 0, red = 1
bool startProgram = false;
bool foundLine = true;
String sensorReading = "0101"; 
String previousSensorReading = "0101";

// for flashing amber LED when moving
bool LEDState = false; 
bool startLED = false;

// timer to exit loops
unsigned long currentTime;
unsigned long startTime;

// for ultrasonnic distannce readings
float leftDist = leftWallDistance1;
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
    robotState = 3;
    forward();

}

void loop() {
  checkLeftDistance();
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
  } else if (sensorReading == "0000" || sensorReading == "1100" || sensorReading == "1000") { // robot is off-course (unless it is in the tunnnel). Enter recovery mode.
    // deactivate sweep at locations where the sensor reading can be 0000
    switch (robotState) {
      case 1:
        if ((currentTime-startTime) < followLineTime1) { // nnot reached tunnel yet
            sweepForLine(true);
        } else {
            return;
        }

        break;

      case 3:
        if ((currentTime-startTime) < followLineTime2) { // nnot reached ramp yet
            sweepForLine(true);
        } else {
            rotateRight(25);
            forward();
        }

        break;

      case 8:
      return;
      break;

      default:
      sweepForLine(true);
      break;
    }
}
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
        if (leftDist < leftWallDistance1 && leftDist != 0) { // filter out anonomalous zero readings
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
    //Serial.println(frontDist);

    switch (robotState) {
        case 1:
        frontWallDistance = frontWallDistance1;
        break;
        case 2:
        frontWallDistance = frontWallDistance2;
        break;
        case 3:
        frontWallDistance = frontWallDistance3;
        break;
/*         if (holdingBlock) {
            // frontWallDistance = frontWallDistance3;
            return; // block would mess up front usonic sennsor reading
        } else {
            return; // so robot can get close to block to pick it up
        } */
        case 4:
        frontWallDistance = frontWallDistance4;
        break;
        case 5:
        frontWallDistance = frontWallDistance4;
        break;
        default:
        return;
    }
    
    // Serial.println(frontWallDistance);

    if (frontDist < frontWallDistance && frontDist != 0) { // filter out anonomalous zero readings
        rotateRight(90);

        // robot has left ramp (unable to detect line). go to line followign once recovery is done.
        if (robotState == 4) { // ramp to line following
            leftCount = 0;
            robotState = 5;
        } else if (robotState == 2) { // tunnnel to line folllowing
            robotState = 3;
        }

        sweepForLine(false); // recovery included in sweepForLine.
    }
}

// FINISH WRITING THIS CODE
// robot goes forward annd sweeps for line. repeat unntil it finds the line.

void checkLeftDistance() {
    leftDist = leftDistance();
    // Serial.println(leftDist);
    int leftD = leftWallDistance1;
    int tolerance;

    switch (robotState) {
    case 1:
    leftD = frontWallDistance4; // we nneed huge tolerannces if we do this. after each turn the distance changes. maybe do this as a last resort after recovery2.
    tolerance = wallDistanceTolerance2;
    break;
    case 2:
    leftD = leftWallDistance1;
    tolerance = wallDistanceTolerance;
    break;
    case 3:
    leftD = frontWallDistance2;
    tolerance = wallDistanceTolerance2;
    break;
    case 4:
    if (sensorReading == "0000" || sensorReading == "1000") {
      leftD = leftWallDistance2;
      tolerance = wallDistanceTolerance;
    } else {
      return;
    }
    break;
    default:
    //frontWallDistance = frontWallDistance1;
    return;
    }

    if (leftDist < (leftD - tolerance) && leftDist != 0) { // too close to left wall
        Serial.println("Too close to left wall. adjusting");
        turnRight();
    } else if (leftDist > (leftD + tolerance) && leftDist != 0) {
      Serial.println("Too far to left wall. adjusting");
        turnLeft();
    } else {
      Serial.println("left wall distance ok");
        forward();
    }
}


// robot does a sweep after a rotation and stops when it finds the lines.
// set left=true to start sweeping right first
// returns bool. true = linne found, false = line not found.
bool sweepForLine(bool left) {
  stop();
  
  // initialise variables
  int sweepTime = rotateTime/90*35;
  int sweepState = 0;
  String reading; 
  unsigned long startSweepTime = millis();
  unsigned long currentSweepTime = millis();
  bool sweeping = true;

  while (sweeping == true) {
    switch (sweepState) {
      case 0: // start right sweep 
        rotate(!left); // if robot turned left, we rotate right first
        // Serial.println(currentSweepTime-startSweepTime);

        // use timer to stop rotation
        if ((currentSweepTime - startSweepTime) >= sweepTime) {
          // Serial.println("Finished 1st sweep");
          stop();
          sweepState = 1;
          startSweepTime = millis();
          currentSweepTime = millis();
          break;
        }
        
        // detect line -> go back to line folloing
        reading = OSwitchReadings();
        if (reading == "0100" || reading == "0010" || reading == "0110") {
          stop();
          sweeping = false;
          return true;
          // break;
        }

        currentSweepTime = millis();
        sweepState = 0;
        break;

      case 1: // left sweep (if left is true)
        // Serial.println("Start 2nd sweep");
        rotate(left);

        // use timer to stop rotation
        if ((currentSweepTime - startSweepTime) >= sweepTime*2) {
            stop();
            startSweepTime = millis();
            currentSweepTime = millis();
            sweepState = 2; // finished sweep and we can't finnd the line.
            break;
        }

        // detect line -> go back to line folloing
        reading = OSwitchReadings();
        if (reading == "0100" || reading == "0010" || reading == "0110") {
          // Serial.println("finished 2nd sweep. founnd line");
          stop();
          sweeping = false;
          return true;
          // break;
        }

        currentSweepTime = millis();
        sweepState = 1;
        break;

      case 2: // cann't find line
      // Serial.println("can't find line");

      // return to original orienntation
      rotate(!left);
      
        if ((currentSweepTime - startSweepTime) >= sweepTime) {
            stop();

            // go right then go forward unntil it finds the linnne 
            // after turning left, we rotate right
            // after turning right, we rotate left

            if (left) {
              rotateRight(25);
            } else {
              rotateLeft(25);
            }
            
            forward();
            delay(1500);
            sweepState = 0;
            startSweepTime = millis();
            currentSweepTime = millis();
            break;
            //sweeping = false;
            // forward();
            //return false;
            // recover?
            //break;
        }

        currentSweepTime = millis();
        sweepState = 2;
        break;
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