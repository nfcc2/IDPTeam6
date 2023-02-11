// use line following sensors to go one loop clockwise around the arena. Pick up block if detected using push button.

// include required libraries
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "NewPing.h"
#include <Arduino.h>

// create objects and specify pins
// motors
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1); // pin M1
Adafruit_DCMotor *leftMotor = AFMS.getMotor(2); //pin M2

// push button input pins
const int blockButtonPin = 2;
const int startButtonPin = 3;

// optoswitch pins for line following
#define farLeftOSwitch 4
#define leftOSwitch 5
#define rightOSwitch 6
#define farRightOSwitch 7

// optoswitch pin for colour identification
#define colourOSwitch 8

// LED output pins
const int amberLEDPin = 9;

// variable initialisation
int robotState = 0;
int motorSpeed1 = 150; // speed ranges from 0 to 255
int count = 0; // counts number of left intersections
bool holdingBlock = false;
bool redBlock = false;
bool start = false;
String sensorReading = "010"; 
// for blinking amber LED when moving
bool LEDState = false; 
bool startLED = false;
unsigned long previousMillis = 0;
const int interval = 500; // 2 Hz

void setup() {
  AFMS.begin();
  Serial.begin(9600);
  pinMode(blockButtonPin, INPUT);
  pinMode(startButtonPin, INPUT);
  pinMode(amberLEDPin, OUTPUT);
  
  // set up timer interrupt for flashing amber LED
  cli();  //stop interrupts for till we make the settings
  /*1. First we reset the control register to amke sure we start with everything disabled.*/
TCB0.CTRLB = TCB_CNTMODE_INT_gc; // Use timer compare mode
TCB0.CCMP = 125000; // Value to compare with. This is 1/10th of the tick rate, so 10 Hz
TCB0.INTCTRL = TCB_CAPT_bm; // Enable the interrupt
TCB0.CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_ENABLE_bm; // Use Timer A as clock, enable timer
  sei();                     //Enable back the interrupts
  startLED = true;
  //Serial.println("start");
}

void loop(){

}

ISR(TCB0_INT_vect)
{
// Do something
  if (startLED) { // startLED = true when motors are running
  //TCNT1  = 0;
  LEDState = !LEDState; // toggle LED bbetween on and off
  } else {
    LEDState = 0; // turn off LED
}
digitalWrite(amberLEDPin, LEDState);
// Clear interrupt flag
TCB0.INTFLAGS = TCB_CAPT_bm;
}