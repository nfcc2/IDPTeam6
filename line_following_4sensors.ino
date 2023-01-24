// include required libraries
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
  // get optoswitch sensor readings from digital pins
  bool farLeftV = digitalRead(farLeftOSwitch);
  bool leftV = digitalRead(leftOSwitch);
  bool rightV = digitalRead(rightOSwitch);
  bool farRightV = digitalRead(farRightOSwitch);

  if (leftV == 0 && rightV == 0) { // both black
    forward();
  } else if (leftV == 1 && rightV == 0) {
    turnLeft();
  } else if (leftV == 0 && rightV == 1) {
    turnRight();
  } else if (leftV == 1 && rightV == 1) {
    stop();
  }
  delay(50); // frequency of sensor readings?
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

