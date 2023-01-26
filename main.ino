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

// ultrasonic sensors
#define USonic1Trigger 10
#define USonic1Echo 11
#define USonic2Trigger 12
#define USonic2Echo 13
#define MAX_DISTANCE 400
NewPing sonar1(USonic1Trigger, USonic1Echo, MAX_DISTANCE);
NewPing sonar2(USonic2Trigger, USonic2Echo, MAX_DISTANCE);

// variables
int robotState = 0; // initial state
int motorSpeed1 = 150; // speed ranges from 0 to 255
int motorSpeed2 = -150;
std::string sensorReading = "101"; // string holding 4 optoswitch sensor readings
std::string distanceReading = "0"; //Inital distance reading provided there's no obstacle at the start

void setup() {
  // put your setup code here, to run once:
  AFMS.begin();
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  switch (robotState) {
    case 0: // leave start area
        Serial.println('State 1');
        robotState = 2;
        break;

    case 1: // line following
        robotState = 3;
        break;

    case 2: // look for block

    default:


  }
}

// functions go here

void follow_line() {
  // get optoswitch sensor readings from digital pins; 1=white, 0=black
  bool V1 = digitalRead(farLeftOSwitch);
  bool V2 = digitalRead(leftOSwitch);
  bool V3 = digitalRead(rightOSwitch);
  bool V4 = digitalRead(colourOSwitch);
  bool V5 = digitalRead(USonic);

  // convert 4 booleans to a string
  std::string newSensorReading = to_string(V1) + to_string(V2) + to_string(V3);
  std::string newDistanceReading = to_string(V5);

  
  // if new readings are the same as the old readings, don't send repeated commands to the motors
  if newSensorReading == sensorReading {
    return
  }
  if distanceReading == newDistanceReading {
    return
  }

  if newsensorReading == "000" {
    forward();
  } else if (newsensorReading == "010") {
    turnLeft();
  } else if (newsensorReadinng == "001") {
    turnRight();
  } else if (newsensorReading == "111") {
    stop();
  } else if newsensorReading == "110" {
    // decide whether to enter delivery box
    Serial.println('Delivery')
  }

  if newDistanceReading == "1" {
    turnBack();
  } else if (newsensorReading == "0100") {
    return
}
}

// sensor functions

// This function returns the distance measured by the front USonic sensor. if robot gets too close to a wall, it turns back
void frontSensor() {
    distance = (sonar.ping_median(5) / 2) * 0.0343; // averages 5 sensor readings
    return distance;
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

void turnBack()
   leftMotor.setSpeed(motorSpeed1);
   rightMotor.setSpeed(moterSpeed2);
   leftMotor->run(FORWARD);
   rightMotor->run(FORWARD);
   delay(200)
   
}

