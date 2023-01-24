// include required libraries
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// create objects and specify pins
// motors and servos
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1); // pin M1
Adafruit_DCMotor *leftMotor = AFMS.getMotor(2); //pin M2

// optoswitch pins for line following
#define leftOSwitch 6
#define centreOSwitch 7
#define rightOSwitch 8


// variables
int motorSpeed = 150; // speed ranges from 0 to 255

void setup() {
  // put your setup code here, to run once:
  AFMS.begin();
}

void loop() {
  // put your main code here, to run repeatedly:

}

// functions go here

void follow_line() {
  // get optoswitch sensor readings from digital pins
  bool leftV = digitalRead(leftOSwitch);
  bool centerV = digitalRead(centreOSwitch);
  bool rightV = digitalRead(rightOSwitch);

  Serial.println(rightV);

  if (leftV == 1 && centerV == 0 && rightV == 1) {
    forward();
    Serial.println("forward");
  } else if (leftV == 0 && centerV == 0 && rightV == 0) {
    stop();
    // revserse?
  } else if (leftV == 1 && centerV == 1 && rightV == 1) {
    stop();
  } else if (leftV == 0 && centerV == 0 && rightV == 1) {
    turnLeft();
  } else if (leftV == 1 && centerV == 0 && rightV == 0) {
    turnRight();
  } else if (leftV == 0 && centerV == 1 && rightV == 1) {
    turnLeft();
  } else if (leftV == 1 && centerV == 1 && rightV == 0) {
    turnRight();
  }

  delay(50); // choose frequency of sensor readings
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

