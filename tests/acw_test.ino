// use line following sensors to go one loop ANTICLOCKWISE around the arena. Pick up block if detected using push button.
//assumed farLeftOSwitch isn't working. set to 0.

// TEST TUNNNNEL DETECTION

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
#define farLeftOSwitch 6 // 6 nnot working
#define leftOSwitch 4
#define rightOSwitch 5
#define farRightOSwitch 7 

// LED output pins
const int amberLEDPin = 8;
const int redLEDPin = 10;
const int greenLEDPin = 11;

// ultrasonnic pins
#define USonic1Trigger 12
#define USonic1Echo 13
#define USonic2Trigger A0 // USONIC 2 MOUNTED ON RHS
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
const int followLineTime1 = 13000; // time taken to go from start box intersection to tunnel
const int followLineTime2 = 30000; // time taken to go from outside tunnel to ramp
const int followLineTime3 = 3000; // time taken to go from red/green box to white box
const int followLineTime4 = 4000;

//const int followLineTime2 = 4000; // time taken to go from outside tunnel to ramp FOR TESTING

// wall distances
const int frontWallDistance1 = 8; // distance (in cm) from front of robot to wall when it should turn (Before tunnel)
const int frontWallDistance2 = 20; // right turnn after tunnnel
const int frontWallDistance3 = 9; // right turnn before ramp
const int frontWallDistance4 = 40; // right turn after ramp
const int rightWallDistance1 = 5; // distance (in cm) from left of robot to wall in tunnel
const int rightWallDistance2 = 6; // distance from left to wall on ramp
const int rightWallDistance3 = 17; // distance from left to wall in area 3
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
int rightCount = 0; // counts number of left intersections
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
float rightDist = rightWallDistance1;
float frontDist = frontWallDistance1;

// for block pick up annd return
bool LHSBLock = false;

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
    //robotState = 3; // for testing
    //startTime = millis();
    //robotState = 2;

}

void loop() {
  switch (robotState) {
    case 0: // leave start area and turn left
        Serial.println("State 0: leave start area and turn left");
        
        forward();

        // count T intersections
        sensorReading = OSwitchReadings();
        if (sensorReading == "0111" && previousSensorReading != "0111") { // onnly increase intersection count for first instance
            tCount++;
        }

        // reached second T intersection. turn left
        if (tCount == 2) {
            forward();
            delay(rotateTimeDelay);
            rotateRight(90);
            sweepForLine(false);

            startTime = millis();
            robotState = 1;
            break;
        }

        previousSensorReading = sensorReading;

        robotState = 0;
        break;

    case 1: // line following until tunnel
        Serial.println("State 1: line following unntil tunnel");

        followLine();
        currentTime = millis();

        if (sensorReading == "0000") {
              Serial.println("Detected ramp");
                forward();
                startTime = millis();
                currentTime = millis();
                robotState = 2;
                break;

        }
        

        // if close to ramp (small left wall distance), if line sensors = 0000, go forward on ramp
/*         rightDist = rightDistance();
        Serial.println(rightDist);
        if (rightDist < (rightWallDistance1 + wallDistanceTolerance2) && rightDist != 0 && (currentTime - startTime) >= followLineTime2) { // maybe time from block to ramp instead of tunnel to ramp
            //if (sensorReading == "0000") {
              Serial.println("Detected ramp");
                forward();
                startTime = millis();
                currentTime = millis();
                robotState = 2;
                break;
        }

        checkrightDistance(); // robbot goes along wall */

/*         // robot reached tunnel - maybe use IR sennsor here too???
         if (sensorReading == "0000" && (currentTime - startTime) > followLineTime1) { // robot in tunnel. 6000 is time taken to reach tunnel after left turn
            robotState = 2;
            startTime = millis();
            forward();
            break;
        } */

        //checkrightDistance();

        // This function checks if robot is getting too close to the wall. it should've turned right at this point.
        // this can happen because 'recovery mode 1' isn't active for robotState=1
        checkLeftTurn();
        robotState = 1;
        break;

    case 2: // on ramp

            Serial.println("State 4: on ramp");
        
        // robot goes along wall
        //checkrightDistance();

        // add sweep after some time?

        // check if robot can detect the lines againn 
        // timinng so upslope anomalous readings don't mess up orientation
        sensorReading = OSwitchReadings();
        if ((sensorReading == "0100" || sensorReading == "0010" || sensorReading == "0110") && (currentTime-startTime) >= rampTime) {
            rightCount = 0; // reset left juctionn counnt
            robotState = 5; // go to line folloing
            break;
        }

        // robot can't find the line on the ramp. do a sweep  on the ramp
        if ((currentTime - startTime) >= (rampTime * 1.2)) {
            if (sweepForLine(true)) { // linne found
                // go to line folloing
                rightCount = 0;
                robotState = 5;
                break;
            }
        }

        // robot has come off the ramp. find line.
        if ((currentTime - startTime) >= (rampTime * 3)) {
            if (sweepForLine(true)) { // linne found
                // go to line folloing
                rightCount = 0;
                robotState = 5;
                break;
            }
        }

        // if robot is close to fronnt wall, it has left the ramp. return to linne following. include this checkLeftTurn 
/*         frontDist = frontDistance();
        if (frontDist < (frontWallDistance4 + wallDistanceTolerance)) {
            rightCount = 0;
            robotState = 5;
            break;
        } */

        //checkLeftTurn();
        currentTime = millis();

        robotState = 4;
        break;
/*         Serial.println("State 2: inn tunnel");
        sensorReading = OSwitchReadings();
        Serial.println(sensorReading);
        currentTime = millis();

        if ((sensorReading == "0100" || sensorReading == "0010" || sensorReading == "0110" || !detectLHSBlock()) && (currentTime - startTime) > tunnelTime) { // reached end of tunnel
            stop();
            sweepForLine(true);
            startTime = millis();
            robotState = 3;
            break;
        }

        // keeps the robot a sensible distance away from the left wall in the tunnel
        // in the hopes that this will help the robot find the line when it exits the tunnel
        checkrightDistance();

        // if the robot completely misses the line after exiting the tunnel, this function forces a right turn when the robot is gettinng close to the front wall
        checkLeftTurn();
        
        robotState = 2;
        break; 
 */
    case 3: // line following after tunnel - robot looks for block
        Serial.println("State 3: line folowing after tunnel");

        followLine();
        currentTime = millis();

        // only check left distance ad blocks once robot has executed right turnn
        if (currentTime - startTime >= followLineTime4) {
                // detect block in fronnt with ultrasonic sensor
            frontDist = frontDistance();
            if (frontDist < blockDistance && frontDist != 0) { // robot found block inn fronnt
                stop();
                forward();
                delay(blockTime);
                robotState = 6;
                break;
            }

        // detect block on RHS of robot with IR sensor
        if (detectLHSBlock()) {
          LHSBlock = true;
          stop();
          rotateRight(90);
          forward();
          delay(blockTime);
          robotState = 6;
          break;
      }

              // if close to ramp (small left wall distance), if line sensors = 0000, go forward on ramp
        rightDist = rightDistance();
        Serial.println(rightDist);
        if (rightDist < (rightWallDistance1 + wallDistanceTolerance2) && rightDist != 0 && (currentTime - startTime) >= followLineTime2) { // maybe time from block to ramp instead of tunnel to ramp
            //if (sensorReading == "0000") {
              Serial.println("Detected ramp");
                forward();
                startTime = millis();
                currentTime = millis();
                robotState = 4;
                break;
            //}
        }
        checkrightDistance(); // robbot goes along wall
        }
        
        checkLeftTurn();

        robotState = 3;
        break;

    case 4: // on ramp
        Serial.println("State 4: on ramp");
        
        // robot goes along wall
        checkrightDistance();

        // add sweep after some time?

        // check if robot can detect the lines againn 
        // timinng so upslope anomalous readings don't mess up orientation
        sensorReading = OSwitchReadings();
        if ((sensorReading == "0100" || sensorReading == "0010" || sensorReading == "0110") && (currentTime-startTime) >= rampTime) {
            rightCount = 0; // reset left juctionn counnt
            robotState = 5; // go to line folloing
            break;
        }

        // robot can't find the line on the ramp. do a sweep  on the ramp
        if ((currentTime - startTime) >= (rampTime * 1.2)) {
            if (sweepForLine(true)) { // linne found
                // go to line folloing
                rightCount = 0;
                robotState = 5;
                break;
            }
        }

        // robot has come off the ramp. find line.
        if ((currentTime - startTime) >= (rampTime * 3)) {
            if (sweepForLine(true)) { // linne found
                // go to line folloing
                rightCount = 0;
                robotState = 5;
                break;
            }
        }

        // if robot is close to fronnt wall, it has left the ramp. return to linne following. include this checkLeftTurn 
/*         frontDist = frontDistance();
        if (frontDist < (frontWallDistance4 + wallDistanceTolerance)) {
            rightCount = 0;
            robotState = 5;
            break;
        } */

        //checkLeftTurn();
        currentTime = millis();

        robotState = 4;
        break;

    case 5: // line following after ramp
        Serial.println("State 5: line following after ramp");

        followLine();
        
        // detect left intersections
        if (sensorReading == "1110") { 
            if (previousSensorReading == "1110") { // do nothinng if this isn't first instance of detecting the junnctionn
                forward();
            } else { // first detection
                rightCount++;
                if (holdingBlock) {
                    if (redBlock && rightCount==1) { // ennter red box
                        robotState = 7;
                        break;
                    } else if (!redBlock && rightCount==3) { // enter green box
                        robotState = 7;
                        break;
                    }
                }
            }
        }

        //checkLeftTurn();

        previousSensorReading = sensorReading;

        robotState = 5;
        break;

    case 6: // pick up block and identify colour
    Serial.println("State 6: pick up block and identify colour");

        stop();
        holdingBlock = true;

        // activate gripper

        delay(1000); // wait till gripper has finnished moving

        // if microswitch on, block is grabbed successfully. detect colour
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

        if (LHSBlock) { // need to reverse annd return to linne following
            rotateLeft(180);
            forward();
            delay(blockTime); // might nneed to be lonnger
            rotateRight(90);
            sweepForLine(false);
        }

        robotState = 3;
        break;

    case 7: // robot enters box part 1 - get past intersection to start linne following
    Serial.println("State 7: Enter box 1");

        // turn left and find the line
        stop();
        forward();
        delay(rotateTimeDelay);
        rotateLeft(90);
        sweepForLine(true);

        robotState = 8;
        break;

    case 8: // ennter box part 2 - follow line to box then release block when we reach the box junction
    Serial.println("state 8: follow line to box annd release block");

        followLine();

        if (sensorReading == "0000") { // reached box
            // raise gripper and release block
            holdingBlock = false;
            delay(2000); // wait till gripper has finnished movinng
            // reverse orientation
            rotateLeft(180);
            sweepForLine(true);

            // if less than 1 minute left, return to start box
            if (millis() > 60000*4) {
                robotState = 9;
                break;
            }
            
            tCount = 1; // reset junction count to 1 - assume robot can't detect red/greenn lines
            robotState = 0;
            break;
        }

        robotState = 8;
        break;

    case 9: // return to start box
        Serial.println("State 9: Return to start box");

        // follow line until we reach the intersection
        while (sensorReading != "1110") { 
            followLine();
        }

        stop();

        if (redBlock) { // start box on LHS
            rotateLeft(90);
            sweepForLine(true);

        } else { // start box on RHS
            rotateRight(90);
            sweepForLine(false);
        }

        robotState = 10;
        break;

    case 10: // line following to returnn to start box
        Serial.println("State 10: line follow to return to start box");

        followLine();

        // detected left intersection
        if (sensorReading == "1110" && redBlock) {
            forward();
            delay(rotateTimeDelay);
            rotateLeft(90);
            sweepForLine(true);

            // follow line to box
            while (sensorReading != "1110") {
                followLine();
            }

            // arrived at intersection outside box
            forward();
            delay(enterBoxTime);
            robotState = 11;
            break;
        }

        // detected right intersection
        frontDist = frontDistance(); // cann use frot sensor distance readinng
        if (frontDist < frontWallDistance3) { 
            rotateRight(90);

            sweepForLine(false);

            // follow line to box
            while (sensorReading != "1110") {
                followLine();
            }

            // arrived at intersection outside box
            forward();
            delay(enterBoxTime);
            robotState = 11;
            break;
        }

/*         if (sensorReading == "0111" && !redBlock) {
            rotateRight(90);

            // enter box
            forward();
            delay(followLineTime2);
            robotState = 11;
            break;
        } */

        robotState = 10;
        break;

    case 11: // finished. robot stops.
        Serial.println("Finished");
        stop();
        robotState = 11;
        break;
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
  } else if (sensorReading == "1100") {
    turnLeft();
  } else if (sensorReading == "1000") {
    turnLeft();
  } else if (sensorReading == "0000") { // robot is off-course (unless it is in the tunnnel). Enter recovery mode.
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

// UPDATE THIS FUNNCTIONN AFTER TESTING
// checks if robot should've turned right at this point. If yes, forces right turn and activates forward recovery to find the line.
// checks if robot should've turned right at this point. If yes, forces right turn and activates forward recovery to find the line.
void checkLeftTurn() {
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
            rightCount = 0;
            robotState = 5;
        } else if (robotState == 2) { // tunnnel to line folllowing
            robotState = 3;
        }

        sweepForLine(false); // recovery included in sweepForLine.
    }
}

void checkrightDistance() {
    rightDist = rightDistance();
    // Serial.println(rightDist);
    int rightD = rightWallDistance1;
    int tolerance;

    switch (robotState) {
    case 1:
    rightD = frontWallDistance4; // we nneed huge tolerannces if we do this. after each turn the distance changes. maybe do this as a last resort after recovery2.
    tolerance = wallDistanceTolerance2;
    break;

    case 2:
    rightD = rightWallDistance1;
    tolerance = wallDistanceTolerance;
    Serial.println(rightD);
    break;

    case 3:
    rightD = frontWallDistance2;
    tolerance = wallDistanceTolerance2;
    break;

    case 4:
    if (sensorReading == "0000" || sensorReading == "0001") {
      rightD = rightWallDistance2;
      tolerance = wallDistanceTolerance;
    } else {
      return;
    }
    break;

    default:
    //frontWallDistance = frontWallDistance1;
    return;
    }
    Serial.println("Hello");

    if (rightDist < (rightD - tolerance) && rightDist != 0) { // too close to left wall
        Serial.println("Too close to right wall. adjusting");
        turnRight();
    } else if (rightDist > (rightD + tolerance) && rightDist != 0) {
      Serial.println("Too far to right wall. adjusting");
        turnLeft();
    } else {
      Serial.println("right wall distance ok");
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
    bool V2 = digitalRead(leftOSwitch); // 1=black, 0=white
    bool V3 = digitalRead(rightOSwitch);
    bool V4 = digitalRead(farRightOSwitch);
    // V1 isn't working. set to 0.
    //bool V4 = digitalRead(farRightOSwitch);
    bool V1 = 0;

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