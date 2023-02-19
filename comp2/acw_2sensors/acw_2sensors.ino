// use line following sensors to go anticlockwise around the arena. Push block if detected using sensors.
// Uses 2 line sensors positioned outside the line.

// include required libraries
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "NewPing.h"

// create objects and specify pins
// motors
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1); // pin M1
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2); // pin M2

// push button input pin
const int startButtonPin = 2;
// pin for colour sensor
const int colourSensor = 3;

// line sensor pins for line following
const int farLeftLineSensor = 7; // 7 not working
const int leftLineSensor = 5;
const int rightLineSensor = 8;
const int farRightLineSensor = 6; // floats between 0 and 1

// LED output pins
const int amberLEDPin = 9;
const int redLEDPin = 10;
const int greenLEDPin = 11;

// ultrasonic pins
const int USonic1Trigger = 12;
const int USonic1Echo = 13;
#define USonic2Trigger A0
#define USonic2Echo A1
const int MAX_DISTANCE = 200;
NewPing sonar1(USonic1Trigger, USonic1Echo, MAX_DISTANCE);
NewPing sonar2(USonic2Trigger, USonic2Echo, MAX_DISTANCE);

// IR sensor (80cm) pin
#define IRSensor A2

// variable initialisation

// variables we can change
// motor and movement variables
int motorSpeed1 = 255; // speed ranges from 0 to 255
int motorSpeed2 = 200; // try slower speed for more gentle turns during line following
const int rotateTime = 1320; // time taken to rotate 90 degrees
const int rotateTimeDelay = 400; // time delay robot nneeds to go forward for before a 90 degree turn
const int junctionTime = 1000; // time taken for robot to get from initial junction detection to after the junction

// IR Sensor analogue reading threshold
const int IRThreshold = 350; // if the analog IR reading is bigger than this value, block detected/in tunnel.

// other variables used by the program
// integers
int robotState = 0; // controls state machine

// to count number of junctions
int rightCount = 0;
int leftCount = 0;
int tCount = 0;

// boolean variables
bool startProgram = false; // default false; true when start button is pressed
bool holdingBlock = false; // default false; true when pushing block
bool redBlock = false; // blue = 0, red = 1
bool foundLine = false; // true when line is detected after the tunnel
bool LHSBlock = false; // for block pick up if block is on LHS of robot
bool detectedBlock = false; // disables block searching functions
bool pastBlockJunction = false; // false before block junction, true after block junction in area 3

// for flashing amber LED when moving
bool LEDState = false; 
bool startLED = false;

// initialise strings to hold line sensor readings
String sensorReading = "0101"; 
String previousSensorReading = "0101";

// timer to disable functions
unsigned long currentTime;
unsigned long startTime; // records time when we move between robot states
unsigned long startTime2; // records time when we finish the block collecting routine
unsigned long programStartTime; // records time when we push the start button
unsigned long junctionStartTime; // records time when we stop at a left junction to detect the block
unsigned long blockStartTime; // start timer for colour detection

// for ultrasonic distance readings
float rightDist = 5;
float frontDist = 25;

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

  // program starts when we push the start buttonn
  while (startProgram == false) {
    if (digitalRead(startButtonPin) == false) { // pullup mode: output = LOW when button is pressed
        programStartTime = millis();
        startProgram = true;
        break;
    }
}

  forward();
  startTime = millis();
  robotState = 0;
}

void loop() {
  switch (robotState) {
    case 0: // leave start area and turn right
        //Serial.println("State 0: leave start area and turn right");
        
        currentTime = millis();
        followLine();

        if (sensorReading == "0110" && previousSensorReading != "0110") { // only increase intersection count for first instance
            tCount++;
        }

        // reached second T intersection
        if (tCount == 2 || currentTime - startTime >= 3000) {
            forward();
            delay(rotateTimeDelay+150);
            rotateRight(90);
            sweepForLine(false);

            startTime = millis();
            robotState = 1;
            break;
        }

        previousSensorReading = sensorReading;

        robotState = 0;
        break;

    case 1: // line following until ramp
        //Serial.println("State 1: line following until ramp");

        currentTime = millis();
        followLine();

        // ignore right junctions
        if (rightJunction()) {
          forward();
          delay(junctionTime+150);
        }

        // if close to ramp (small right wall distance), move to state 2
         rightDist = rightDistance();
        if (rightDist < (9 + 2) && rightDist != 0 && (currentTime - startTime) >= 13000) {
          //Serial.println("Detected ramp");

          // go to state 2
          forward();
          startTime = millis();
          foundLine = false;
          robotState = 2;
          break;
        } 
      
        // This function checks if robot is getting too close to the wall in front. it should've turned left at this point.
        if (checkFrontDistance(8)) {
          forceLeftTurn(); // rotates 90 degrees left

          // go to state 2
          forward();
          startTime = millis();
          foundLine = false;
          robotState = 2;
          break;
        }

        // checks if the robot is too far or close from the wall on RHS
        if (sensorReading == "0000" && previousSensorReading == "0000") {
          maintainRightDistance(26, 20); // between 6 and 41 cm
        }

        previousSensorReading = sensorReading;

        robotState = 1;
        break;

    case 2: // on ramp
        Serial.println("State 2: on ramp");

        currentTime = millis();
        
        // robot goes along wall
        maintainRightDistance(8, 3);

        // timing so upslope anomalous readings don't mess up orientation
        if (currentTime - startTime >= 7000) {
            // check if the robot is too close to the front wall. Force 90 degree left turn if it is.
            if (checkFrontDistance(19)) {
              forceLeftTurn(); // rotate and do one sweep to find line
              rotateLeft(8);

              // move to state 3
              detectedBlock = false;
              pastBlockJunction = false;
              startTime = millis();
              startTime2 = millis();
              leftCount = 0;

              robotState = 3;
              forward();
              break;
            }
        }

        robotState = 2;
        break; 

    case 3: // line following after ramp - robot looks for block
        //Serial.println("State 3: line following after ramp");
        motorSpeed1 = 255;

        currentTime = millis();
        followLine();

        if (sensorReading == "0110") {
          pastBlockJunction = true;
          motorSpeed1 = 190; // lower motor speed so there is a higher chance the robot makes the left turn
        }
        
        // detected block in front with ultrasonic sensor and we haven't detected a block yet
        if (checkFrontDistance(5) && !detectedBlock && leftCount < 2) {
          //Serial.println("Front block");
    
          detectedBlock = true;
          LHSBlock = false;
          robotState = 6;
          break;
        }

        // detected block on LHS of robot with IR sensor
        if (detectLHSBlock() && !detectedBlock && currentTime - startTime > 4000) {
          rightDist = rightDistance();

          if (rightDist <7 && rightDist != 0) {
            // in tunnel
            robotState = 4;
            startTime = millis();
            forward();
            break;
          }

          detectedBlock = true;
      
          LHSBlock = true;
          robotState = 6;
          break;
        }
        if (detectedBlock) {
          pastBlockJunction = true;
          motorSpeed1 = 190;
        }

        // use left IR sensor and line sensor readings to determine if robot is in tunnel
        if (detectLHSBlock() && pastBlockJunction && (currentTime-startTime2 > 12000)) {
          // robot in tunnel
          robotState = 4;
          motorSpeed1 = 255;
          startTime = millis();
          forward();
          break;
        }
          
        if (sensorReading == "0000" && previousSensorReading == "0000" && currentTime-startTime < 30000 && currentTime-startTime>3000) {
          maintainRightDistance(17, 2); // right wall distance should be between 6 and 17cm
        }

        previousSensorReading = sensorReading;

        // force left turn if block not in front of usonic sensor
        frontDist = frontDistance();
        if (frontDist < 7 && frontDist != 0 && (currentTime - startTime2 > 12000) && pastBlockJunction) {
          forceLeftTurn();
          startTime = millis();
          robotState = 4;
          motorSpeed1 = 255;
          forward();
          break;
        }

        // if robot is pushing the block and missed the previous left turn, use timer to force left turn before tunnel
        if ((currentTime - startTime2) > 18000 && holdingBlock && pastBlockJunction) {
          //Serial.println("Recovering");
          backward();
          delay(220);
          forceLeftTurn();
          startTime = millis();
          motorSpeed1 = 255;
          robotState = 4;
          forward();
          break;
        }

        robotState = 3;
        break;

    case 4: // in tunnel
        //Serial.println("State 4: in tunnel");

        currentTime = millis();
        
        // robot goes along wall
        maintainRightDistance(5, 2);

        // check if robot has exited tunnel - either it detects the line or the IR sensor doesn't detect anything in proximity
        sensorReading = lineSensorReadings();

            if ((currentTime - startTime) > 5000) {
              if (!detectLHSBlock() || sensorReading != "0000") {

                  sweepForLine(true);

                  if (!foundLine) {
                    forward();
                    delay(2000);
                    forceLeftTurn();
                    sweepForLine(true);
                    forward();
                    startTime = millis();
                    rightCount = 0;
                    robotState = 5;
                    break;
                  } else {
                    startTime = millis();
                    forward();
                    rightCount = 0;
                    robotState = 5;
                    break;
                  }
              }
            }

        robotState = 4;
        break;    

    case 5: // line following after tunnel
        //Serial.println("State 5: line following after tunnel");

        motorSpeed1 = 190;

        currentTime = millis();
        followLine();
        
        // detect right junctions
        if (rightJunction()) {
          rightCount++;

          // if holding block, enter corresponding delivery box
          if (holdingBlock) {
            if (redBlock && rightCount == 3) { // enter red box
                robotState = 7;
                break;
            } else if (!redBlock && rightCount == 1) { // enter green box
                robotState = 7;
                break;
              }
          } else { // robot was not pushing the block (e.g. if it failed to detect the block). Go to state 1 and loop again.

            // return to start box
            if (millis() - programStartTime > 60000*3.5) {
              if (rightCount == 2) {
                forward();
                delay(rotateTimeDelay);
                rotateRight(90);
                sweepForLine(false);
                motorSpeed1 = 255;
                robotState = 11;
                break;
              }
            }

            // go to state 1
            motorSpeed1 = 255;
            startTime = millis();
            robotState = 1;
            break;
          }

          // go past junction
          forward();
          delay(junctionTime);
        }

          // if robot got past any right junctions, we use distance 1 (left turn before ramp) which is shorter than distance 4 (left turn after tunnel).
          if (rightCount > 0) { // after right junctions
            if (checkFrontDistance(7)) {
              forceLeftTurn();
              motorSpeed1 = 255;
              startTime = millis();
              robotState = 2;
              break;
            }
          } else { // before right junctions (ie turn after tunnel)
            if (checkFrontDistance(34)) { // may need to chenge this distannce
              forceLeftTurn();
            
              sweepForLine(true);
              forward();
            }
          }

        robotState = 5;
        break;

    case 6: // pick up block and identify colour
        //Serial.println("State 6: pick up block and identify colour");
        stop();

        // position robot to block
        if (LHSBlock) {
          // reverse robot then collect block at 45 degrees
          backward();
          delay(750);
          rotateLeft(45);
          forward();
          delay(600);
          stop();
        }

        // position block in front of colour sensor
        backward();
        delay(150);
        rotateRight(9);
        forward();
        delay(500);
        stop();
    
        holdingBlock = true;

        delay(1000);
        blockStartTime = millis();
        redBlock = false;
        delay(3000);

        // detect colour
        while (millis() - blockStartTime <= 4000) {
          // Serial.println(digitalRead(colourSensor));
          if (detectColour() == 1) {
            redBlock = true;
          } 
        }
            
        // activate red/green LED to indicate block colour
        blockLED();

        // re-orientate and return to line following
        if (LHSBlock) {
            rotateRight(70);
            forward();
            delay(1100);
            rotateLeft(35);
            forward();
            delay(300);
            sweepForLine(true);
        } else {
          rotateLeft(25);
          forward();
          delay(200);
          rotateRight(15);
        }

        startTime2 = millis();
        robotState = 3;
        break;

    case 7: // robot enters delivery box
    //Serial.println("State 7: Enter delivery box");
        motorSpeed1 = 255;
        stop();
        forward();
        delay(rotateTimeDelay*0.7);
        rotateRight(90);

        forward();
        delay(1500);
        holdingBlock = false;

        backward();
        delay(1800);

        startTime = millis();
        robotState = 8;
        break;

    case 8: // decide whether to go back to start box or continue looping
    //Serial.println("State 8");

        currentTime = millis();

        if (millis() - programStartTime > 60000*3) {
          //Serial.println("Returning to start box");
          robotState = 9;
          break;          
        }

        // continue
        rotateLeft(90);
        sweepForLine(true);
        startTime = millis();
        robotState = 1;
        break;

    case 9: // return to start box
    motorSpeed1 = 255;
        //Serial.println("State 9: Return to start box");

        if (redBlock) {
          rotateRight(90);
        } else {
          rotateLeft(90);
        }

        forward();
        delay(2500);
        robotState = 10;
        break;

    case 10: // line following to return to start box
        //Serial.println("State 10: follow line to return to start box");
        
        currentTime = millis();
        followLine();

        if (redBlock) {
          if (leftJunction()) {
            forward();
            delay(rotateTimeDelay);
            rotateLeft(90);
            sweepForLine(true);

            robotState = 11;
            break;
          }
        } else {
          if (rightJunction()) {
            forward();
            delay(rotateTimeDelay);
            rotateRight(90);
            sweepForLine(true);

            robotState = 11;
            break;
          }
        }

        robotState = 10;
        break;

    case 11: // Follow line from junction to start box and stop.
        Serial.println("State 11: Follow line to start box");

        followLine();

        // Arrived at start box junction
        if (sensorReading == "0110") {
            forward();
            delay(1000);
            stop();

            robotState = 12;
            break;
        }

        robotState = 11;
        break;

    case 12: // Finished. Robot stops.
        Serial.println("Finished");
        stop();
        robotState = 12;
        break;
  }
}

// functions go here
// updates line sensor readings and sends commands to motors based on readings
void followLine() {
  String newSensorReading = lineSensorReadings();
  Serial.println(newSensorReading);
  // if new readings are the same as the old readings, don't send repeated commands to the motors
  if (newSensorReading == sensorReading) {
    return;
  }
  sensorReading = newSensorReading;

  if (sensorReading == "0000") { // 1 is white, 0 is black
    forward();
  } else if (sensorReading == "0100") {
    turnLeft();
  } else if (sensorReading == "0010") {
    turnRight();
  } else if (sensorReading == "0110") {
    forward();
  }
}

// checks front dist to determine if robot should've turned right at this point. If yes, forces right turn and activates forward recovery to find the line.
// argument frontD = distance from front wall when robot should turn left. 
// returns boolean. True = too close -> turn left. False = don't turn left.
bool checkFrontDistance(int frontD) {
    float frontDist = frontDistance();
    //Serial.println(frontDist);

    if (frontDist <= frontD && frontDist != 0) {
      return true;
    } else {
      return false;
    }
}

// usually called if checkFrontDistance = true (i.e. robot is too close to front wall and should've turned at this point)
// robot rotates left, then sweeps for the line. Disabled sweep since it causes instability when using 2 line sensors.
void forceLeftTurn() {
    Serial.println("Force left turn");
    rotateLeft(90);
    // sweepForLine(true);
}

// uses right usonic sensor to maintain a sensible distance from wall on RHS
void maintainRightDistance(int rightD, int tolerance) {
    rightDist = rightDistance();
    // Serial.println(rightDist);
    
    if (rightDist < (rightD - tolerance) && rightDist != 0) { // too close to right wall
        // Serial.println("Too close to right wall. adjusting left");
        turnLeft();

    } else if (rightDist > (rightD + tolerance) && rightDist != 0) {
        // Serial.println("Too far from right wall. adjusting right");
        turnRight();

    } else {
        // Serial.println("right wall distance ok");
        forward();
    }
}

// determines whether 0010 is right adjustment or junction
bool rightJunction() {
    // ignore non right readings
    if (sensorReading != "0010" && sensorReading != "0110") {
      return false;
    }

    if (sensorReading == "0010") {
      // rotate right until sensor reading changes
      while (sensorReading == "0010") {
        motorSpeed2 = 100; // slow down so we don't miss the junction
        rotate(false);
        sensorReading = lineSensorReadings();
      }

      motorSpeed2 = 200;

      if (sensorReading == "0110") { // at junction         
        rotateLeft(5); // re-centre robot
        return true;
      } else {
        return false;
      }

      } else if (sensorReading == "0110") { // at junction
        return true;
      }
}

// determines whether 0100/0110 is left adjustment or junction
bool leftJunction() {
    // ignore non left readings
    if (sensorReading != "0100" && sensorReading != "0110") {
      return false;
    }

    // rotate left until line sensor readings change
    if (sensorReading == "0100") {
      while (sensorReading == "0100") {
        motorSpeed2 = 100;
        rotate(true);
        sensorReading = lineSensorReadings();
      }

      motorSpeed2 = 200;

      // at junction
      if (sensorReading == "0110") { 
        // re-orientate robot       
        rotateRight(5);
        return true;
      } else {
        return false;
      }

    // if line sensors read 0110, robot must be at a junction (can be T or left)
    } else if (sensorReading == "0110") {
      return true;
    }
}

// robot does a sweep after a rotation and stops when it finds the lines.
// set left=true to start sweeping right first
// returns bool. true = linne found, false = line not found.
void sweepForLine(bool left) {
  stop();
  
  // initialise variables
  const int sweepAngle = 35;
  const int sweepTime = (rotateTime / 90) * sweepAngle; 
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
        reading = lineSensorReadings();
        if (reading == "0100" || reading == "0010" || reading == "0110") {
          stop();
          foundLine = true;
          sweeping = false;
          return;
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
        reading = lineSensorReadings();
        if (reading == "0100" || reading == "0010" || reading == "0110") {
          // Serial.println("finished 2nd sweep. founnd line");
          stop();
          sweeping = false;
          foundLine = true;
          return;
        }

        currentSweepTime = millis();
        sweepState = 1;
        break;

      case 2: // can't find line
      // Serial.println("can't find line");

      // return to original orientation
      rotate(!left);
      
        if ((currentSweepTime - startSweepTime) >= sweepTime) {
            stop();

            foundLine = false;
            sweeping = false;
            return;
        }

        currentSweepTime = millis();
        sweepState = 2;
        break;
    }
  }
}

// sensor functions

// Returns line following sensor readings in binary (left to right)
String lineSensorReadings() {
    // V1 and V4 are not working. Set to 0.
    // 0 = black, 1 = white
    bool V1 = 0;
    bool V2 = digitalRead(leftLineSensor); 
    bool V3 = digitalRead(rightLineSensor);
    bool V4 = 0;

    // convert 4 booleans to a string
    String newSensorReading = String(V1) + String(V2) + String(V3) + String(V4);
    return newSensorReading;
} 

// Returns boolean to indicate colour (Red=1, blue=0). Averages 5 sensor readings
bool detectColour() {
    int j = 0;
    for (int i = 0; i < 5; i++) {
        if (digitalRead(colourSensor) == 0) { // sensor reading = 0 if red
            j++; // counts red
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

// returns distance measured by right ultrasonic sensor
float rightDistance() {
  return (sonar2.ping_median(5) / 2) * 0.0343;
}

// returns true if a IR sensor reading is over a threshold (block detected or in tunnel)
bool detectLHSBlock() {
    // Serial.println(analogRead(IRSensor));
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
    rightMotor->setSpeed(250);
    leftMotor->run(RELEASE);
    rightMotor->run(FORWARD);
    startLED = true;
}

void turnLeft2() {
      leftMotor->setSpeed(100);
    rightMotor->setSpeed(255);
    leftMotor->run(BACKWARD);
    rightMotor->run(FORWARD);
    startLED = true;
}

void turnRight() {
    leftMotor->setSpeed(250);
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

// for sweeps. Rotate left if left=true, else rotate right
void rotate(bool left) {
  leftMotor->setSpeed(motorSpeed2);
  rightMotor->setSpeed(motorSpeed2);
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
// Timer interrupt for flashing amber LED. Triggered every 500ms (2 Hz)
ISR(TCB0_INT_vect) {
if (startLED) { // startLED = true when motors are running
  LEDState = !LEDState; // toggle LED bbetween on and off
  } else {
    LEDState = 0; // turn off LED
  }
digitalWrite(amberLEDPin, LEDState);
// clear interrupt flag
TCB0.INTFLAGS = TCB_CAPT_bm;
}

// activates red/green LED for 5 seconds after block colour detection
void blockLED() {
  int LEDPin;
  if (redBlock) {
    LEDPin = redLEDPin;
  } else {
    LEDPin = greenLEDPin;
  }

  digitalWrite(LEDPin, HIGH);
  delay(5000);
  digitalWrite(LEDPin, LOW);
}