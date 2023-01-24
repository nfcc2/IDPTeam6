// include required libraries
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>

// create objects and specify pins
// motors
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1); // pin M1
Adafruit_DCMotor *leftMotor = AFMS.getMotor(2); //pin M2

//servos
servo servo1;

// optoswitch pins for line following
#define farLeftOSwitch 6
#define leftOSwitch 7
#define rightOSwitch 8
#define farRightOSwitch 10

// variables
int motorSpeed = 150; // speed ranges from 0 to 255
int servoPos = 0; // servo angle

void setup() {
  // put your setup code here, to run once:
  AFMS.begin();

  servo1.attach(9)
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
    Serial.println("forward");
  } else if (leftV == 1 && rightV == 0) {
    if farLeftV == 1 {
      // robot at square intersection
      square()
    } else {
      turnLeft();
    }
  } else if (leftV == 0 && rightV == 1) {
    turnRight();
  } else if (leftV == 1 && rightV == 1 && farRightV == 1) {
    stop();
    // pick up box?
  }
  delay(50); // frequency of sensor readings?
}

// functions go here

// decides what to do at a square intersection
void square() {
// can use distace sensors on the side to decide if it's a truck (delivery) or wall (start area)
// or use variable to count which intersection we are at
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
    leftMotor->run(BACKWARD);
    rightMotor->run(FORWARD);
}

void turnRight() {
    leftMotor.setSpeed(motorSpeed);
    rightMotor.setSpeed(motorSpeed);
    leftMotor->run(FORWARD);
    rightMotor->run(BACKWARD);
}

