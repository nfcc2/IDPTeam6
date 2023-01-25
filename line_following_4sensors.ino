// basic line following program

// include required libraries
#include <string>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// create objects and specify pins
// motors
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1); // pin M1
Adafruit_DCMotor *leftMotor = AFMS.getMotor(2); //pin M2

// optoswitch pins for line following
#define farLeftOSwitch 6
#define leftOSwitch 7
#define rightOSwitch 8
#define farRightOSwitch 9

// variables
int motorSpeed = 150; // speed ranges from 0 to 255
std::string sensorReading = "1001"; // string holding 4 optoswitch sensor readings

void setup() {
  // put your setup code here, to run once:
  AFMS.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  follow_line()
}

// functions go here

void follow_line() {
  // get optoswitch sensor readings from digital pins; 1=white, 0=black
  bool V1 = digitalRead(farLeftOSwitch);
  bool V2 = digitalRead(leftOSwitch);
  bool V3 = digitalRead(rightOSwitch);
  bool V4 = digitalRead(farRightOSwitch);

  // convert 4 booleans to a string
  std::string newSensorReading = to_string(V1) + to_string(V2) + to_string(V3) + to_string(V4)
  
  // if new readings are the same as the old readings, don't send repeated commands to the motors
  if newSensorReading == sensorReading {
    return
  }

  if sensorReading == "0110" {
    forward();
  } else if (sensorReading == "0100") {
    turnLeft();
  } else if (sensorReadinng == "0010") {
    turnRight();
  } else if (sensorReading == "1111") {
    stop();
  } else if sensorReading == "0000" {
    // tunnel
    forward();
    Serial.print('Tunnel')
  } else if sensorReading = "0111" {
    // look for block and pick it up
    Serial.print('Block')
  } else if sensorReading = "1110" {
    // decide whether to enter delivery box
    Serial.print('Delivery')
  }
}

// functions for movement
void forward() {
  leftMotor.setSpeed(motorSpeed);
  rightMotor.setSpeed(motorSpeed);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
}

void stop() {
    leftMotor->run(RELEASE);
    rightMotor->run(RELEASE);
}

void turnLeft() {
    leftMotor.setSpeed(motorSpeed);
    rightMotor.setSpeed(motorSpeed);
    leftMotor->run(RELEASE);
    rightMotor->run(FORWARD);
}

void turnRight() {
    leftMotor.setSpeed(motorSpeed);
    rightMotor.setSpeed(motorSpeed);
    leftMotor->run(FORWARD);
    rightMotor->run(RELEASE);
}

