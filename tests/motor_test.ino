// include required libraries
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// create objects
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1);
Adafruit_DCMotor *leftMotor = AFMS.getMotor(2);

// variables
int motorSpeed = 150; // speed ranges from 0 to 255

void setup() {
  // put your setup code here, to run once:
  AFMS.begin();

  rightMotor->setSpeed(motorSpeed);
}

void loop() {
  // put your main code here, to run repeatedly:
  rightMotor->run(FORWARD);
  delay(10000);
  rightMotor->run(RELEASE);
  delay(10000);
}

// functions go here


// functions for movement
void forward() {
  leftMotor.setSpeed(motorSpeed);
}

void stop() {}

void turnLeft() {}

void turnRight() {}
