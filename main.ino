// include required libraries
#include <string>
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

// ultrasonic sensor pins sensor 1 front, sensor 2 left
#define USonic1Trigger 10
#define USonic1Echo 11
#define USonic2Trigger 12
#define USonic2Echo 13
#define MAX_DISTANCE 400
NewPing sonar1(USonic1Trigger, USonic1Echo, MAX_DISTANCE);
NewPing sonar2(USonic2Trigger, USonic2Echo, MAX_DISTANCE);

// variables
int motorSpeed1 = 150; // speed ranges from 0 to 255
int motorSpeed2 = -150;

// initialisation
int robotState = 0;
std::string sensorReading = "101"; 

void setup() {
  // put your setup code here, to run once:
  AFMS.begin();
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  switch (robotState) {
    case 0: // leave start area and turn left
        Serial.println('State 0');
        forward();
        delay(1000);
        turnLeft();
        delay(1000);
        stop();
        robotState = 1;
        break;

    case 1: // line following
        Serial.println('State 1');
        robotState = 3;
        break;

    case 2: // look for block

    default:


  }
}

// functions go here

// navigation functions
void follow_line() {
  newSensorReading = OSwitchReadings();
  // if new readings are the same as the old readings, don't send repeated commands to the motors
  if newSensorReading == sensorReading {
    return
  }

  switch newSensorReading {
    case "000":
        forward();
    case "010":
        turnLeft();
    case "001":
        turnRight();
    case "111":
        stop();
    case "110":
        // detected left junction
  }
}

// Check if robot is too close or too far away from the left wall
void leftWallCheck() {           
    if leftSensor() < 5 { // going towards left wall
        turnBack(); // until we detect the lines again?
    } else if leftSensor() > 30 {
        // go towards wall
    }
}

// sensor functions

// Returns line following sensor readings in binary (left to right)
void OSwitchReadings() {
    bool V1 = digitalRead(farLeftOSwitch); // 1=white, 0=black
    bool V2 = digitalRead(leftOSwitch);
    bool V3 = digitalRead(rightOSwitch);
    bool V4 = digitalRead(colourOSwitch);

    // convert 4 booleans to a string
    std::string newSensorReading = to_string(V1) + to_string(V2) + to_string(V3);
    return newSensorReading;
} 

// These return the distance (in cm) measured by the front and left USonic sensors. if robot gets too close to a wall, it turns back
void frontSensor() {
    distance = (sonar1.ping_median(5) / 2) * 0.0343; // averages 5 sensor readings
    if distance != 0 { // remove anomalous zero readings
        return distance;
    }
}

void leftSensor() {
    distance = (sonar1.ping_median(5) / 2) * 0.0343;
    if distance != 0 { 
        return distance;
    }
}

// functions for movement
void forward() {
  leftMotor.setSpeed(motorSpeed1);
  rightMotor.setSpeed(motorSpeed1);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
  delay(200);
}

void stop() {
    leftMotor->run(RELEASE);
    rightMotor->run(RELEASE);
    delay(200);
}

void turnLeft() {
    leftMotor.setSpeed(motorSpeed1);
    rightMotor.setSpeed(motorSpeed1);
    leftMotor->run(RELEASE);
    rightMotor->run(FORWARD);
    delay(200);
}

void turnRight() {
    leftMotor.setSpeed(motorSpeed1);
    rightMotor.setSpeed(motorSpeed1);
    leftMotor->run(FORWARD);
    rightMotor->run(RELEASE);
    delay(200);
}

void turnBack() {
   leftMotor.setSpeed(motorSpeed1);
   rightMotor.setSpeed(motorSpeed2);
   leftMotor->run(FORWARD);
   rightMotor->run(FORWARD);
   delay(200);
}

