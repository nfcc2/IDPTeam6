// use line following sensors to go one loop ANTICLOCKWISE around the arena. Pick up block if detected using sensors.
// farLeftLineSensor isn't working. set to 0.

// TESTS
// state 0 annd whether "0111" in followline affects junnctionn detection
// maintainRightDistance - usinng rotate instead of turn
// tunnel detection with detectLHSBlock();

// include required libraries
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "NewPing.h"
#include <Arduino.h>

// create objects and specify pins
// motors
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1); // pin M1
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2); // pin M2

// servo

// push button input pin
const int startButtonPin = 2;

// pin for colour sensor
const int colourSensor = 3;

// microswitch input pin
#define blockButtonPin A3
// ADD colour detection when button is pressed. this confirms we have the block.

// line sensor pins for line following
const int farLeftLineSensor = 6; // 6 nnot working
const int leftLineSensor = 4;
const int rightLineSensor = 5;
const int farRightLineSensor = 7; 

// LED output pins
const int amberLEDPin = 8;
const int redLEDPin = 10;
const int greenLEDPin = 11;

// ultrasonnic pins
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
const int rotateTime = 1300; // time taken to rotate 90 degrees
const int rotateTimeDelay = 650; // time delay robot nneeds to go forward for before a 90 degree turn

// box times
const int enterBoxTime = 3000; // time taken to enter box from junction after 90 degree right turn
const int leaveBoxTime = 1500; // time taken to leave start box and get past box junction
const int startBoxTime = 5000; // time taken to follow line from red/green box junction to startBox junction
const int startBoxDistance = 110; // distance from start box junction to wall

// line following times
const int followLineTime1 = 13000; // time taken to go from start box junction to ramp
const int followLineTime2 = 20000; // time raken to go from finishing block routine to left turn before tunnnel
const int followLineTime3 = 30000; // time taken to go from end of ramp to 

// wall distances
const int frontWallDistance1 = 8; // distance (in cm) from front of robot to wall when it should turn (Before ramp)
const int frontWallDistance2 = 20; // left turnn after ramp
const int frontWallDistance3 = 9; // left turnn before tunnel
const int frontWallDistance4 = 40; // left turn after tunnel

const int rightWallDistance1 = 36; // distance (in cm) from right to wall in area 1
const int rightWallDistance2 = 6; // distance (in cm) from right to wall on ramp
const int rightWallDistance3 = 17; // distance from right to wall inn area 3
const int rightWallDistance4 = 5; // distance from right to wall in tunnel

const int tolerance1 = 1; // tolerannce for usonic sensor readings
const int tolerance2 = 2; 
const int tolerance3 = 5; 

// block distannces and times
const int IRThreshold = 400; // if analog IR reading bigger than this, block detected/in tunnel.
const int blockTime = 500; // time to move forward to get the block if it is in proximity
const int blockTime2 = 750; // time to move backward to collect block at 45 degrees
const int blockTime3 = 600; // time to move forward after 45 degree rotation to sweep for block
const int blockDistance1 = 5;// front distance threshold to detect block
const int blockDistance2 = 150; // dist from front to wall at first block junnctio
const int blockDistance3 = 70; // dist from front to wall at seconnd block junnction

// tunnel/ramp times
const int tunnelTime = 3500; // time taken to cross tunnnel
const int rampTime = 7000; // time taken to make it up the ramp

// variables that don't need changing
// integers
int robotState = 0; // controls state machine

// to count  number of junctions
int rightCount = 0;
int leftCount = 0;
int tCount = 0;

// count nnumber of sweeps on ramp
int sweepCount = 0;

// boolean variables
bool startProgram = false;
bool holdingBlock = false;
bool redBlock = false; // blue = 0, red = 1
bool foundLine = false;
bool LHSBlock = false; // for block pick up if block is on LHS of robot
bool detectedBlock = false; // disables block searching functions
bool pastBlockJunction = false; // false before block junnnction, true after block junnction in area 3

// for flashing amber LED when moving
bool LEDState = false; 
bool startLED = false;

// initialise strings to hold line sensor readings
String sensorReading = "0101"; 
String previousSensorReading = "0101";

// timer to disable functions
unsigned long currentTime;
unsigned long startTime;
unsigned long startTime2;

// for ultrasonnic distannce readings
float rightDist = rightWallDistance1;
float frontDist = frontWallDistance1;

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
        startProgram = true;
        break;
    }
  } 

  startTime = millis();

  // robot moves forward to get past start box junction
  forward();
  delay(leaveBoxTime);
}

void loop() {
  switch (robotState) {
    case 0: // leave start area and turn right
        Serial.println("State 0: leave start area and turn right");
        
        currentTime = millis();
        followLine();

        // detect T junction to turn right
        if (sensorReading == "0111") {
          forward();
          delay(rotateTimeDelay);
          rotateRight(90);
          sweepForLine(false);

          startTime = millis();
          robotState = 1;
          break;
        }

        // OLD CODE: robot goes forward and turns right at second junction. New code tries to implement line following from outside box to 2nd junnction.
/*         if (sensorReading == "0111" && previousSensorReading != "0111") { // onnly increase junction count for first instance
            tCount++;
        }

        // reached second T junction. turn right.
        if (tCount == 2) {
            forward();
            delay(rotateTimeDelay);
            rotateRight(90);
            sweepForLine(false);

            startTime = millis();
            robotState = 1;
            break;
        } 

        previousSensorReading = sensorReading; */

        robotState = 0;
        break;

    case 1: // line following until ramp
        Serial.println("State 1: line following until ramp");

        currentTime = millis();
        followLine();

        // if close to ramp (small right wall distance), go forward on ramp
        rightDist = rightDistance();
        // Serial.println(rightDist);
        if (rightDist < (rightWallDistance2 + tolerance2) && rightDist != 0 && (currentTime - startTime) >= followLineTime1) {
          Serial.println("Detected ramp");
          forward();
          startTime = millis();
          sweepCount = 0;
          robotState = 2;
          break;
        }
        
        // robot is not able to follow the line after sweep. call ultrasonic functions
        if (!foundLine) {
    
          // This function checks if robot is getting too close to the wall in front. it should've turned left at this point.
          if (checkFrontDistance(frontWallDistance1)) {
            forceLeftTurn();
          }

          // checks if the robot is too far or close from the wall on RHS
          maintainRightDistance(rightWallDistance2, tolerance2); // robot usually loses line after left turn so use distance 2
        }

        robotState = 1;
        break;

    case 2: // on ramp
        Serial.println("State 2: on ramp");

        currentTime = millis();

        // robot goes along wall
        maintainRightDistance(rightWallDistance2, tolerance1);

        // check if robot can detect the lines againn 
        // timinng so upslope anomalous readings don't mess up orientation
        sensorReading = lineSensorReadings();
        if ((sensorReading == "0100" || sensorReading == "0010" || sensorReading == "0110") && (currentTime-startTime) >= rampTime) {
            foundLine = true;
        }

        // robot can't find the line on the ramp. do a sweep on the ramp
        if ((currentTime - startTime) >= (rampTime * 1.2) && sweepCount == 0) {
            sweepForLine(false);
            sweepCount++;
        }

        // robot has come off the ramp. do second sweep to find line.
        if ((currentTime - startTime) >= (rampTime * 3) && sweepCount == 1) {
            sweepForLine(false);
            sweepCount++;
        }

        // line found from line sensors/sweeps. go to line following.
        if (foundLine) {
            detectedBlock = false;
            pastBlockJunction = false;
            startTime = millis();
            startTime2 = millis();
            leftCount = 0;
            robotState = 3;
            break;
        }

        // check if the robot is too close to the front wall. Force 90 degree left turn if it is.
        if (checkFrontDistance(frontWallDistance2)) {
          forceLeftTurn(); // rotate, sweep to find line, then set up for next state
          detectedBlock = false;
          pastBlockJunction = false;
          startTime = millis();
          startTime2 = millis();
          leftCount = 0;
          robotState = 3;
          break;
        }

        robotState = 2;
        break; 

    case 3: // line following after ramp - robot looks for block
        Serial.println("State 3: line following after ramp");

        // currentTime = millis();
        followLine();
        
        // robot got past the block junction - we use this instead of leftCount because front usonnic distance is not as reliable.
        if (sensorReading == "0111") {
          pastBlockJunction = true;
        }
        
        // detected block in front with ultrasonic sensor and we haven't detected a block yet
        if (checkFrontDistance(blockDistance1) && !detectedBlock) {
          detectedBlock = true;
          LHSBlock = false;
          robotState = 6;
          break;
        }

        // detected block on LHS of robot with IR sensor
        if (detectLHSBlock() && !detectedBlock) {
          detectedBlock = true;
          LHSBlock = true;
          robotState = 6;
          break;
        }

        // use front sensor and stop at the 2 left junnctions to detect block
        if (checkFrontDistance(blockDistance2) && leftCount == 0 && !detectedBlock) {
          leftCount++;
          stop();

          if (detectLHSBlock()) {
            LHSBlock = true;
            robotState = 6;
            break;
          }
        }

        if (checkFrontDistance(blockDistance3) && leftCount < 2 && !detectedBlock) {
          leftCount++;
          stop();

          if (detectLHSBlock()) {
            LHSBlock = true;
            robotState = 6;
            break;
          }
        }

        // robot can't find line
        if (!foundLine) {

          // use left IR sensor and line sensor readings to determine if robot is in tunnel
          if (sensorReading == "0000" && detectLHSBlock()) {
            // robot in tunnel
            robotState = 4;
            startTime = millis();
            forward();
            break;
          }
          
          maintainRightDistance(rightWallDistance3, tolerance2); // robot goes along wall

          // check if the robot should turn left based on distance from front to wall. This is disabled if the robot is pushing the block.
          // if robot is past the block junction, this left turn would bring it to the tunnnel.
          // then rely on maintainRightDistance to navigate. Move to state 4.
          if (pastBlockJunction) { // turn before tunnel
            if (checkFrontDistance(frontWallDistance3)) {
              forceLeftTurn();
              startTime = millis();
              robotState = 4;
              break;
            }

            // if robot is pushing the block and off-course, use timer to force left turn 
            if ((currentTime - startTime2) > followLineTime2 && holdingBlock) {
              forceLeftTurn();
              startTime = millis();
              robotState = 4;
              break;
            }

          } else { // turnn after ramp
            if (checkFrontDistance(frontWallDistance2)) {
              forceLeftTurn();
            }
          }

        robotState = 3;
        break;

    case 4: // in tunnel
        Serial.println("State 4: in tunnel");

        currentTime = millis();
        
        // robot goes along wall
        maintainRightDistance(rightWallDistance4, tolerance1);

        // check if robot has exited tunnel - either it detects the line or the IR sensor doesn't detect anything in proximity
        sensorReading = lineSensorReadings();
        if ((sensorReading == "0100" || sensorReading == "0010" || sensorReading == "0110" || !detectLHSBlock()) && (currentTime - startTime) > tunnelTime) {
            stop();
            sweepForLine(true); // updates foundLine
            startTime = millis();
            rightCount = 0;
            robotState = 5;
            break;
        }

        // if pushing the block, we use a timer to force a left turn if the robot is off course.
        if (holdingBlock) {
          if (currentTime - startTime > tunnelTime * 1.5) {
              forceLeftTurn();
              startTime = millis();
              rightCount = 0;
              robotState = 5;
              break;
          }
        } else { // not pushing block - use front sensor
          if (checkFrontDistance(frontWallDistance4)) {
              forceLeftTurn();
              startTime = millis();
              rightCount = 0;
              robotState = 5;
              break;
          }
        }

        robotState = 4;
        break;

    case 5: // line following after tunnel
        Serial.println("State 5: line following after tunnel");

        currentTime = millis();
        followLine();
        
        // detect right junctions
        if (sensorReading == "0111") { 
            if (previousSensorReading == "0111") { // do nothing if this isn't first instance of detecting the junnctionn
                forward();
            } else { // first detection
                rightCount++;
                if (holdingBlock) {
                    if (redBlock && rightCount == 3) { // ennter red box
                        robotState = 7;
                        break;
                    } else if (!redBlock && rightCount == 1) { // enter green box
                        robotState = 7;
                        break;
                    }
                } else if (rightCount == 3) { // robot wasn't pushing the block. reset to state 1.
                    startTime = millis();
                    robotState = 1;
                    break;
                }
            }
        }

        if (!foundLine) {
          // keep sensible distance from right wall
          maintainRightDistance(rightWallDistance1, tolerance3);

          // only check for left turns if robot isn't pushing the block
          if (!holdingBlock) {
            // if robot got past any right junctions, we use distance 1 (left turn before ramp) which is shorter than distance 4 (left turn after tunnel).
            if (rightCount > 0) { // after right junction
              if (checkFrontDistance(frontWallDistance1)) {
                forceLeftTurn();
                robotState = 1;
                break;
              }
            } else { // before right junction
              if (checkFrontDistance(frontWallDistance4)) {
                forceLeftTurn();
              }
            }
          }
        }

        previousSensorReading = sensorReading;

        robotState = 5;
        break;

    case 6: // pick up block and identify colour
        Serial.println("State 6: pick up block and identify colour");
        
        stop();

        // position robot to block
        if (LHSBlock) {
          // reverse robot then collect block at 45 degrees

          backward();
          delay(blockTime2);
          rotateLeft(45);
          forward();
          delay(blockTime3);
          stop();

          // do an ultrasonic sweep to find block for more accuracy
          sweepForBlock();
          }

          // position block in front of colour sensor
          rotateRight(10);

          forward();
          delay(blockTime);
          stop();
      
          holdingBlock = true;

          delay(500);
          redBlock = detectColour();

          // activate LEDs to indicate block colour
          if (redBlock) {
              // Serial.print("Block colour is red");
              digitalWrite(redLEDPin, HIGH);
              delay(5000);
              digitalWrite(redLEDPin, LOW);
          } else {
              // Serial.print("Block colour is blue");
              digitalWrite(greenLEDPin, HIGH);
              delay(5000);
              digitalWrite(greenLEDPin, LOW);
          }

        if (LHSBlock) { // rotate and return to line following
            rotateRight(70);
            forward();
            delay(blockTime3 + blockTime);
            rotateLeft(35);
            sweepForLine(false);
        } else {
          rotateLeft(10); // straighten robot
        }

        startTime2 = millis();
        robotState = 3;
        break;

    case 7: // robot enters delivery box part 1 - get past junction to start line following
    Serial.println("State 7: Enter delivery box 1");

        // turn right and find the line
        stop();
        forward();
        delay(rotateTimeDelay);
        rotateRight(90);
        sweepForLine(false);

        startTime = millis();
        robotState = 8;
        break;

    case 8: // enter delivery box part 2 - follow line to box then release block
    Serial.println("state 8: follow line to box annd release block");

        currentTime = millis();
        followLine();

        // reached delivery box junction
        if (sensorReading == "0000" && (currentTime - startTime > enterBoxTime)) { 
            // leave the block in the box
            holdingBlock = false;

            forward();
            delay(blockTime);
            stop();
            backward();
            delay(blockTime*1.5);

            // reverse orientation
            rotateLeft(180);
            sweepForLine(true);

            // if less than 1 minute left, return to start box
            if (millis() > 60000*4) {
                robotState = 9;
                break;
            }
            
            tCount = 1; // reset junction count to 1 - assume robot can't detect red/green lines
            robotState = 0;
            break;
        }

        robotState = 8;
        break;

    case 9: // return to start box
        Serial.println("State 9: Return to start box");

        // follow line until we reach the junction
        while (sensorReading != "1110") { 
            followLine();
        }

        stop();
        forward();
        delay(rotateTimeDelay);
        stop();

        if (redBlock) { // start box on LHS
            rotateLeft(90);
            sweepForLine(true);

        } else { // start box on RHS
            rotateRight(90);
            sweepForLine(false);
        }

        startTime = millis();
        robotState = 10;
        break;

    case 10: // line following to return to start box 1: detect start box junction to turn left/right
        Serial.println("State 10: line follow to return to start box");
        
        currentTime = millis();
        followLine();

        // detected right junction (going from green box to start box)
        if (sensorReading == "0111" && !redBlock) {
            forward();
            delay(rotateTimeDelay);
            rotateRight(90);
            sweepForLine(false);

            robotState = 11;
            break;
        }

        // detected left junction (going from red box to start box)
        // use distance or timing to force a left turn
        if ((checkFrontDistance(startBoxDistance) || (currentTime - startTime) > startBoxTime) && redBlock) {
          stop();
          forward();
          delay(rotateTimeDelay);
          rotateLeft(90);
          sweepForLine(true);

          robotState = 11;
          break;
        }

        robotState = 10;
        break;

    case 11: // Follow line to start box and stop.
        Serial.println("State 11: Follow line to start box");

        followLine();

        // arrived at start box junction
        if (sensorReading == "0111") {
            forward();
            delay(leaveBoxTime);
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
}

// functions go here
// updates line sensor readings and sends commands to motors based on readings
// if no lines detected, do a sweep.
void followLine() {
  String newSensorReading = lineSensorReadings();
  Serial.println(newSensorReading);

  // if new readings are the same as the old readings, don't send repeated commands to the motors
  if (newSensorReading == sensorReading) {
    return;
  }
  sensorReading = newSensorReading;

  if (sensorReading == "0110") { // 1 is white, 0 is black
    foundLine = true;
    forward();
  } else if (sensorReading == "0100") {
    foundLine = true;
    turnLeft();
  } else if (sensorReading == "0010") {
    foundLine = true;
    turnRight();
  } else if (sensorReading == "0011") {
    turnRight();
  } else if (sensorReading == "0111") { // this may affect junction detection. test.
    forward();
  } else if (sensorReading == "0001") {
    turnRight();
  } else if (sensorReading == "0000") { // robot is off-course (unless it is in the tunnnel). Do a sweep.
    foundLine = false;

    // deactivate sweep at tunnel/ramp where the sensor reading can be 0000
    switch (robotState) {
      case 1:
        if ((currentTime-startTime) < followLineTime1) { // not reached ramp yet
            sweepForLine(false);
        } else {
            return;
        }
        break;

      case 3:
        if ((currentTime-startTime) < followLineTime3) { // not reached tunnel yet
            sweepForLine(false);
        } else {
            return;
        }
        break;

      case 8:
        if ((currentTime-startTime) < enterBoxTime) { // not reached delivery box yet
            sweepForLine(false);
        } else {
          return;
        }
        break;

      default:
      sweepForLine(false);
      break;
    }
}
}

// checks front dist to determinen if robot should've turned right at this point. If yes, forces right turn and activates forward recovery to find the line.
// argument frontD = distance from front wall when robot should turn left. 
// returns boolean. True = too close -> turn left. False = don't turn left.
bool checkFrontDistance(int frontD) {
    frontDist = frontDistance();
    //Serial.println(frontDist);

    // block would mess up front sensor readings.
    if (holdingBlock) {
      return false;
    }

    if (frontDist < frontD && frontDist != 0) {
      return true;

    } else {
      return false;
    }
}

// usually called if checkFrontDistance = true (i.e. robot is too close to front wall and should've turned at this point)
// robot rotates left, then sweeps for the line. If the first sweep fails, robot enters recovery mode.
void forceLeftTurn() {
    rotateLeft(90);
    sweepForLine(true);
}

// uses right usonic sensor to maintain a sensible distance from wall on RHS
void maintainRightDistance(int rightD, int tolerance) {
    rightDist = rightDistance();
    // Serial.println(rightDist);
    
    if (rightDist < (rightD - tolerance) && rightDist != 0) { // too close to right wall
        Serial.println("Too close to right wall. adjusting left");
        // turnLeft();
        rotateLeft(10);
        forward();

    } else if (rightDist > (rightD + tolerance) && rightDist != 0) {
        Serial.println("Too far from right wall. adjusting right");
        // turnRight();
        rotateRight(10);
        forward();

    } else {
        Serial.println("right wall distance ok");
        forward();
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

      case 2: // cann't find line
      // Serial.println("can't find line");

      // return to original orienntation
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

// returns boolean foundBlock. true if found, false if not
// robot rotates and looks for 2 consecutive small readings to detect block
bool sweepForBlock() {
    // initialise variables
    float prevDist = 100; // stores previous distance readinng
    float d = 100;
    int minDistance = 14;
    const int sweepAngle = 30;
    unsigned long startSweepTime = millis();
    unsigned long currentSweepTime = millis() + 1;
    int sweepState = 0;
    bool sweeping = true;

    motorSpeed1 = 180; // slow down for sweeps
    const int rotateTime2 = 3000; // time taken to rotate 90 degrees at this new speed
    const int sweepTime = (rotateTime2/90) * sweepAngle;
    
  while (sweeping == true) {
    switch (sweepState) {
      case 0: // start right sweep 
        rotate(false);

        // use timer to stop rotation
        if ((currentSweepTime - startSweepTime) >= sweepTime) {
          // Serial.println("Finished 1st sweep");
          stop();
          sweepState = 1;
          startSweepTime = millis();
          currentSweepTime = millis();
          break;
        }

        // measure front distance
        d = frontDistance();
        if (d != 0) { // filter out '0' anomalous readings
            // checks if previous annd current distance readings are similar and small enough < 14cm (ie block detected)
            if ((d < minDistance) && (abs(prevDist - d) < 1)) {
                // block found
                Serial.println("Block found");
                stop();
                motorSpeed1 = 255;
                sweeping = false;
                return true;
            }
        }

        currentSweepTime = millis();
        sweepState = 0;
        break;

      case 1: // left sweep
        // Serial.println("Start 2nd sweep");
        rotate(true);

        // use timer to stop rotation
        if ((currentSweepTime - startSweepTime) >= sweepTime*2) {
            stop();
            startSweepTime = millis();
            currentSweepTime = millis();
            sweepState = 2; // finished sweep and we can't find the block.
            break;
        }

        // measure front distance
        d = frontDistance();
        if (d != 0) { // filter out '0' anomalous readings
            // checks if previous annd current distance readings are similar and small enough < 14cm (ie block detected)
            if ((d < minDistance) && (abs(prevDist - d) < 1)) {
                // block found
                Serial.println("Block found");
                stop();
                motorSpeed1 = 255;
                sweeping = false;
                return true;
            }
        }

        currentSweepTime = millis();
        sweepState = 1;
        break;

      case 2: // can't find block
      // Serial.println("can't find block");

      // return to original orienntation
      rotate(false);
      
        if ((currentSweepTime - startSweepTime) >= sweepTime) {
            stop();
            sweeping = false;
            return false;
        }

        currentSweepTime = millis();
        sweepState = 2;
        break;
    }
  }
}

// called if line not found after sweep and maintainRightDistance. robot rotates then goes forward and does a sweep. continnue unntil line find.
// alternnate direction between left and right after time delay.
// nnot usinng this function at the moment. robot will use rightWallDistance to navigate if line not found after sweep.
void recover() {

  // initialise timing variables
  unsigned long startRecoverTime = millis();
  unsigned long currentRecoverTime = millis();
  const int timeInterval = 2500; // do a sweep every 2.5s

  while (!foundLine) {
    
    if (holdingBlock) { // block would mess up fronnt usonic readings
      rotateRight(10);
      forward();
    } else {
      // if too close to front wall, reverse
      frontDist = frontDistance();
      if (frontDist > frontWallDistance1 && frontDist != 0) {
          forward();
      } else {
          backward(); 
      }
    }

    // sweep every 2.5s
    if (currentRecoverTime - startRecoverTime > timeInterval) {
        sweepForLine(false); // updates foundLine
        startRecoverTime = millis();
    }

    followLine(); // updates foundLine
    currentRecoverTime = millis();
  }
}

// sensor functions

// Returns line following sensor readings in binary (left to right)
String lineSensorReadings() {
    // V1 isn't working. set to 0.
    // 0 = black, 1 = white
    bool V1 = 0;
    bool V2 = digitalRead(leftLineSensor); 
    bool V3 = digitalRead(rightLineSensor);
    bool V4 = digitalRead(farRightLineSensor);

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
float rightDistance() {
  return (sonar2.ping_median(5) / 2) * 0.0343;
}

// returns true if block detected on RHS by IR sensor
bool detectLHSBlock() {
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