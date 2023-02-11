// assumes 4 working line sensors.
// robot stops when no lines detected or at any kind of intersectionn.
// prints line sensor readinngs too

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
#define farLeftOSwitch 4
#define leftOSwitch 5 // left on green board. white wire
#define rightOSwitch 6 // 
#define farRightOSwitch 7

// optoswitch pin for colour identification
#define colourOSwitch 8

// push button input pin
const int buttonPin = 3;

int motorSpeed1 = 160; // speed ranges from 0 to 255
int count = 0; // counts number of left intersections
bool holdingBlock = false;
bool redBlock = false;

// initialisation
int robotState = 0;
String sensorReading = "0101";

void setup() {
  // put your setup code here, to run once:
  AFMS.begin();
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  // Serial.println(millis());
  // rotateLeft(90);
  // Serial.println(millis());
  //forward(); 

  follow_line();

}

// functions go here

// navigation functions
void follow_line() {
  String newSensorReading = OSwitchReadings();
  Serial.println(newSensorReading);
  // if new readings are the same as the old readings, don't send repeated commands to the motors
  if (newSensorReading == sensorReading) {
    return;
  }
  sensorReading = newSensorReading;

  if (sensorReading == "0000") {
    stop();
  } else if (sensorReading == "0100") {
    turnLeft();
  } else if (sensorReading == "0010") {
    turnRight();
  } else if (sensorReading == "0110") {
    forward();
  } else if (sensorReading == "1111") {
    stop();
  } else if (sensorReading == "1110") {
    stop();
  } else if (sensorReading == "0111") {
    stop();
  }
}

// sensor functions

// Returns line following sensor readings in binary (left to right)
String OSwitchReadings() {
    bool V1 = digitalRead(farLeftOSwitch); // 1=black, 0=white
    bool V2 = digitalRead(leftOSwitch);
    bool V3 = digitalRead(rightOSwitch);
    bool V4 = digitalRead(farRightOSwitch);

    //bool V1 = 0;
    //bool V4 = 0;

    // convert 4 booleans to a string
    String newSensorReading = String(V1) + String(V2) + String(V3) + String(V4);
    return newSensorReading;
} 

// functions for movement
void forward() {
  leftMotor->setSpeed(motorSpeed1);
  rightMotor->setSpeed(motorSpeed1);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
}

void backward() {
    leftMotor->setSpeed(motorSpeed1);
  rightMotor->setSpeed(motorSpeed1);
  leftMotor->run(BACKWARD);
  rightMotor->run(BACKWARD);
}

void stop() {
    leftMotor->run(RELEASE);
    rightMotor->run(RELEASE);
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
    delay((1000/90)*degrees); // 1000 is time taken to turn 90 degrees (assume 1 second for now)
    leftMotor->run(RELEASE);
    rightMotor->run(RELEASE);
}

