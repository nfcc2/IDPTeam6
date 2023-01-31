// use line following sensors to go one loop clockwise around the arena. Pick up block if detected using push button.

// include required libraries
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "NewPing.h"

// create objects and specify pins
// motors
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1); // pin M1
Adafruit_DCMotor *leftMotor = AFMS.getMotor(2); //pin M2

// optoswitch pins for line following
#define farLeftOSwitch 6
#define leftOSwitch 7
#define rightOSwitch 8

// optoswitch pin for colour identification
#define colourOSwitch 9

// push button input pin
const int buttonPin = 5;

/* // ultrasonic sensor pins sensor 1 front, sensor 2 left
#define USonic1Trigger 10
#define USonic1Echo 11
#define USonic2Trigger 12
#define USonic2Echo 13
#define MAX_DISTANCE 400
NewPing sonar1(USonic1Trigger, USonic1Echo, MAX_DISTANCE);
NewPing sonar2(USonic2Trigger, USonic2Echo, MAX_DISTANCE); */

// variables
int motorSpeed1 = 150; // speed ranges from 0 to 255
int count = 0; // counts number of left intersections
bool holdingBlock = false;
bool redBlock = false;

// initialisation
int robotState = 0;
char sensorReading[] = "010"; 

void setup() {
  // put your setup code here, to run once:
  AFMS.begin();
  Serial.begin(9600);
  pinMode(buttonPin, input);
}

void loop() {
  // put your main code here, to run repeatedly:

  switch (robotState) {
    case 0: // leave start area and turn left
        Serial.println('State 0');
        forward();
        delay(1000);
        if (OSwitchReadings == "000") {
            rotateLeft(90);
        }
        robotState = 1;
        break;

    case 1: // line following
        follow_line();

        // detect left intersection
        if (OSwitchReadings == "001") {
            count++;

            if (holdingBlock) {
                if (redBlock && count%4 == 2) {
                    robotState = 3;
                } else if (!redBlock && count%4==0) {
                    robotState = 3;
                }
            }
        }

        // detect block with push button
        buttonState = digitalRead(buttonPin);
        if (buttonState==1) {
            robotState = 2;
        }

        break;

    case 2: // pick up block and identify colour
        stop();
        holdingBlock = true;

        // activate gripper

        delay(1000); // wait till gripper has finnished moving
        redBlock = detectColour();

        if (redBlock) {
            Serial.print("Block colour is red");
        } else {
            Serial.print("Block colour is blue");
        }
        delay(500);
        
        //count = 0; // can idenntify area 1 using ultrasonic sensor distance from wall
        robotState = 1;
        break;

    case 3: // robot puts down block
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
  newSensorReading = OSwitchReadings();
  // if new readings are the same as the old readings, don't send repeated commands to the motors
  if (newSensorReading == sensorReading) {
    return
  }
  sensorReading = newSensorReading;

  switch newSensorReading { // 1=black, 0=white
    case "111":
        forward();
    case "101":
        turnLeft();
    case "110":
        turnRight();
  }
}

// sensor functions

// Returns line following sensor readings in binary (left to right)
void OSwitchReadings() {
    bool V1 = digitalRead(farLeftOSwitch); // 1=black, 0=white
    bool V2 = digitalRead(leftOSwitch);
    bool V3 = digitalRead(rightOSwitch);
    bool V4 = digitalRead(colourOSwitch);

    // convert 4 booleans to a string
    char newSensorReading[] = to_string(V1) + to_string(V2) + to_string(V3);
    return newSensorReading;
} 

// Returns boolean to indicate colour (Red=1, blue=0). Averages 5 sensor readings
bool detectColour() {
    int count = 0
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
  delay(200);
}

void backward() {
    leftMotor->setSpeed(motorSpeed1);
  rightMotor->setSpeed(motorSpeed1);
  leftMotor->run(BACKWARD);
  rightMotor->run(BACKWARD);
  delay(200);
}

void stop() {
    leftMotor->run(RELEASE);
    rightMotor->run(RELEASE);
    delay(200);
}

void turnLeft() {
    leftMotor->setSpeed(motorSpeed1);
    rightMotor->setSpeed(motorSpeed1);
    leftMotor->run(RELEASE);
    rightMotor->run(FORWARD);
    delay(200);
}

void turnRight() {
    leftMotor->setSpeed(motorSpeed1);
    rightMotor->setSpeed(motorSpeed1);
    leftMotor->run(FORWARD);
    rightMotor->run(RELEASE);
    delay(200);
}

void turnBack() {
   leftMotor->setSpeed(motorSpeed1);
   rightMotor->setSpeed(motorSpeed1);
   leftMotor->run(FORWARD);
   rightMotor->run(BACKWARD);
   delay(200);
}

void rotateLeft(int degrees) {
    leftMotor->setSpeed(motorSpeed1);
    rightMotor->setSpeed(motorSpeed1);
    leftMotor->run(BACKWARD);
    rightMotor->run(FORWARD);
    delay((1000/90)*degrees) // 1000 is time taken to turn 90 degrees (assume 1 second for now)
    leftMotor->run(RELEASE);
    rightMotor->run(RELEASE);
}

