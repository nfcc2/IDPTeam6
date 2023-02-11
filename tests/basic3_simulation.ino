/* Example code with timer intyerrutp that will create an interruption each 
 *  500ms using timer1 and prescalar of 256.
Calculations (for 500ms): 
  System clock 16 Mhz and Prescalar 256;
  Timer 1 speed = 16Mhz/256 = 62.5 Khz    
  Pulse time = 1/62.5 Khz =  16us  
  Count up to = 500ms / 16us = 31250 (so this is the value the OCR register should have)*/  
bool LED_STATE = true;
bool LED_START = false;
const int startButtonPin=7;
const int blockButtonPin=8;
const int redLEDPin = 9;
const int greenLEDPin = 10;
String sensorReading = "1010";
String previousSensorReading = "1010";
bool redBlock = false;
bool holdingBlock = false;
bool buttonState = false;

int robotState = 0;
bool startProgram = false;
unsigned long currentTime;
unsigned long startTime;
int count = 0;

const int frontWallDistance = 10; // distance (in cm) from front of robot to wall when it should turn
const int leftWallDistance = 5; // distance (in cm) from left of robot to wall in tunnel
const int wallDistanceTolerance = 2; // tolerannce for usonnic sensor readings
float d = leftWallDistance;

void setup() {
  Serial.begin(9600);
  
  pinMode(blockButtonPin, INPUT_PULLUP);
  Serial.println("hi");
                  //Enable back the interrupts
}

void loop() {
  switch (robotState) {
    case 0: // leave start area and turn left
        Serial.println("State 0: leave start area and turn left");
        stop();
        forward();
        delay(1000); // allow enough time for robot to leave start box and cross first intersection

        // robot starts moving when we push the start buttonn
/*         while (startProgram==false) {
            if (digitalRead(startButtonPin) == false) { // pullup mode: output = LOW when button is pressed
                startProgram=true;
                break;
            }
        } */
        
        // follow line until we reach the intersection
        //startTime = millis(); // commented out timinng redundancies
        //currentTime = millis();
        while (sensorReading != "1110") { 
            followLine();
            //currentTime = millis();
/*             if ((currentTime - startTime) >= 2000) { // 2000 is a bit more than time taken to go from first intersectionn to second itnersectionn
                break;
            } */
        }
        
        rotateLeft(90);
        robotState = 1;
        startTime = millis();
        break;

    case 1: // line following until tunnel
        Serial.println("State 1: line following unntil tunnel");
        followLine();
        // currentTime = millis();

        // robot reached tunnel
        // commented out timing redunndanncies
/*         if (sensorReading == "0000" && (currentTime - startTime) > 6000) { // robot in tunnel. 6000 is time taken to reach tunnel after left turn
            robotState = 2;
            forward();
            break;
        } else {
            recover();
        } */

        // robot is getting too close to the wall. it should've turned right at this point.
        // this can happen because 'recovery mode 1' isn't active for robotState=1
        if (frontDistance() < frontWallDistance) {
            rotateRight(90);
            recover2();
        }

        // robot reaches tunnel when it can't detect any linnes
        if (sensorReading == "0000") {
            robotState = 2;
            count = 0; // reset intersection count after tunnel
            forward();
            break;
        }

        robotState = 1;
        break;

    case 2: // through tunnel
        Serial.println("State 2: inn tunnel");
        sensorReading = OSwitchReadings();

        if (sensorReading == "0100" || sensorReading == "0100" || sensorReading == "0110") { // reached end of tunnel
            robotState = 3;
            break;
        }

        // keeps the robot a sensible distance away from the left wall in the tunnel
        // in the hopes that this will help the robot find the line when it exits the tunnel
        d = leftDistance();
        if (d < (leftWallDistance - wallDistanceTolerance)) { // too close to left wall
            turnRight();
        } else if (d > (leftWallDistance + wallDistanceTolerance)) {
            turnLeft();
        } else {
            forward();
        }

        robotState = 2;
        break; 

    case 3: // line following after tunnel
        Serial.println("State 3: line folowing after tunnel");
        followLine();

            // detect left intersection
        if (sensorReading == "1110") { // if sennsorReading = 1110
            if (previousSensorReading == "1110") { // do nothinng if this isn't first instance of detecting the junnctionn
                forward();
            } else { // first detection
                count++;
                if (holdingBlock) {
                    if (redBlock && count==2) { // ennter red box
                        robotState = 5;
                        break;
                    } else if (!redBlock && count==4) { // enter green box
                        robotState = 5;
                        break;
                    }
                }
            }
        }

        previousSensorReading = sensorReading;

        // detect block with push button
        buttonState = !digitalRead(blockButtonPin); // pullup mode - 1=open, 0=closed
        if (buttonState==1) {
            robotState = 4;
            break;
        }

        robotState = 3;
        break;

    case 4: // pick up block and identify colour
    Serial.println("State 4: pick up block and identify colour");
        stop();
        holdingBlock = true;

        // activate gripper

        delay(1000); // wait till gripper has finnished moving
        redBlock = detectColour();

        if (redBlock) {
            Serial.print("Block colour is red");
            digitalWrite(redLEDPin, HIGH);
            delay(5000);
            digitalWrite(redLEDPin, LOW);
        } else {
            Serial.print("Block colour is blue");
            digitalWrite(greenLEDPin, HIGH);
            delay(5000);
            digitalWrite(greenLEDPin, LOW);
        }

        delay(500);
        robotState = 3;
        break;

    case 5: // robot puts down block
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
        rotateLeft(180);
        delay(1000);

        // if time close to 5 mins, return to start box
        
        robotState = 0;
        break;
  }
}


// obtains line sensor readings and sends commands to motors based on readings
void followLine() {
  String newSensorReading = OSwitchReadings();
  // if new readings are the same as the old readings, don't send repeated commands to the motors
  if (newSensorReading == sensorReading) {
    return;
  }
  sensorReading = newSensorReading;

  if (sensorReading == "0110") { // 1 is white, 0 is black
    forward();
  } else if (sensorReading == "0100") {
    turnLeft();
  } else if (sensorReading == "0010") {
    turnRight();
  } else if (sensorReading == "0000" && robotState != 1) { // robot is off-course (unless it is in the tunnnel). Enter recovery mode.
    recover();
}
}

// Robot reverses until it detects the line.
void recover() {
    backward();
    while (sensorReading == "0000") {
        String newSensorReading = OSwitchReadings();
        // if new readings are the same as the old readings, don't send repeated commands to the motors
        if (newSensorReading == sensorReading) {
            continue;
        }
        sensorReading = newSensorReading;

        if (sensorReading == "0100") { // 1 is white, 0 is black
            turnLeft();
        } else if (sensorReading == "0010") {
            turnRight();
        }
    }
}

void recover2() {
    forward();
    while (sensorReading == "0000") {
        String newSensorReading = OSwitchReadings();
        // if new readings are the same as the old readings, don't send repeated commands to the motors
        if (newSensorReading == sensorReading) {
            continue;
        }
        sensorReading = newSensorReading;

        if (sensorReading == "0010") { // 1 is white, 0 is black
            turnLeft();
        } else if (sensorReading == "0100") {
            turnRight();
        }
    }
}


bool findBlock() { 
    int minDistance = 100;
    float d = 100;
    const int sweepAngle = 120;
    const int halfSweepTime = (rotateTime/90)*(sweepAngle/2);
    unsigned long startSweepTime = millis();
    unsigned long currentSweepTime = millis() + 1;
    int state = 0;
    int sweepCount = 0;

    for(;;) { // set up loop. using continue brings us to start of loop. break breaks out of loop
    switch (state) {
        case 0: // start sweep to the left
      Serial.println("Start 1st left sweep");
        rotate(true);
        startSweepTime = millis();
        state = 1;
        continue;

        case 1:
        if (currentSweepTime - startSweepTime >= halfSweepTime) { // finished half sweep
          Serial.println("finihsed sweep");  
          sweepCount++;
            if (sweepCount == 3) {
            state = 3;
            continue;
            } else {
            state = 2;
            continue;
            }
        }

        // record minimum distance
        d = frontDistance();
        if (d != 0) { // filter out '0' annomalous readings
            if (d < minDistance) {
                minDistance = d;
            } 
        }
        
        currentSweepTime = millis(); 
        state = 1;
        continue;

        case 2: // start sweep to the right
        rotate(false);
        startSweepTime = millis();
        state = 1;
        continue;

        case 3: // start final sweep to the left to locate block
        rotate(true);
        startSweepTime = millis();
        state = 4;
        continue;

        case 4:
        if (currentSweepTime - startSweepTime >= halfSweepTime*2) { // finished full sweep
            stop(); // can't find block
            return false;
        }

        d = frontDistance();
        if (minDistance - 1 < d && d < minDistance + 1) {
            stop(); // found block
            return true;
            break;
        }
        currentSweepTime = millis(); 
        state = 4;
        continue;
    }
    break;
    }
}


// looks for 3 consecutive small readings instead of min distance to detect block
bool findBlock2() {
    float dists[3] = {100, 100, 100}; // stores last 3 distance readings
    float d = 100;
    const int sweepAngle = 120;
    const int halfSweepTime = (rotateTime/90)*(sweepAngle/2);
    unsigned long startSweepTime = millis();
    unsigned long currentSweepTime = millis() + 1;
    int state = 0;
    int sweepCount = 0;
    int i = 0;

    for(;;) { // set up loop. using continue brings us to start of loop. break breaks out of loop
    switch (state) {
        case 0: // start sweep to the left
      Serial.println("Start 1st left sweep");
        rotate(true);
        startSweepTime = millis();
      currentSweepTime = millis() + 1;
        state = 1;
        continue;

        case 1: // sweepinng and recording distannces
      Serial.println("sweepinng");
        if (currentSweepTime - startSweepTime >= halfSweepTime) { // finished half sweep
            stop();
            sweepCount++;
          Serial.print("sweep counnt: ");
          Serial.println(sweepCount);
            if (sweepCount == 3) {
            state = 0;
            continue;
            } else if (sweepCount == 4) {
              Serial.println("block not found");
                return false;
            } else {
                state = 2;
            }
            continue;
            }
        

        d = frontDistance();
        if (d != 0) { // filter out '0' annomalous readings
            dists[i % 3] = d;

            // checks if previous annd current distance readings are similar and small enough < 10cm (ie block detected)
            if ((d < 10) && (abs(dists[(i+1)%3] - d) < 1) && (abs(dists[(i+2)%3] - d) < 1)) {
                // block found
                stop();
              Serial.println("block found");
                return true;
            }

            i++;
        }
        
        currentSweepTime = millis(); 
        state = 1;
        continue;

        case 2: // start sweep to the right
      Serial.println("Start 1st right sweep");
        rotate(false);
        startSweepTime = millis();
      currentSweepTime = millis() + 1;
        state = 1;
        continue;
    }
    break;
    }
}

String OSwitchReadings() {
  Serial.println("Ennter line sennsor reading: ");
  while (Serial.available() == 0) {
  }
  String newSensorReading = Serial.readString();
    return newSensorReading;
} 

bool detectColour() {
    return true;
}

void forward() {
  Serial.println("Forward");
}

void backward() {
  Serial.println("Backward");
}

void stop() {
    Serial.println("Stop");
}

void turnLeft() {
    Serial.println("Left");
}

void turnRight() {
    Serial.println("Right");
}

void rotateLeft(int degrees) {
    Serial.print("Rotate ");
    Serial.print(degrees);
    Serial.println("left");
}

void rotateRight(int degrees) {
    Serial.print("Rotate ");
    Serial.print(degrees);
    Serial.println("right");
}

float frontDistance() {
    Serial.println("Ennter fronnt usonic sensor reading: ");
  while (Serial.available() == 0) {
  }
  float d = Serial.parseFloat();
    return d;
}

// distannce measured by left usonic sensor
float leftDistance() {
      Serial.println("Ennter left usonic sensor reading: ");
        while (Serial.available() == 0) {
  }
  float d = Serial.parseFloat();
    return d;

}


void rotate(bool left) {
  if (left) {
    Serial.println("Left rotate");
  } else {
    Serial.println("Right rotate");
  }
}