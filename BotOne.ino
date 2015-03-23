#include <Adafruit_LSM303_U.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include "pitches.h"

#define STAND_STILL_MICROS 1500

const float Pi = 3.14159;

#ifndef BOT_H_
#define BOT_H_

const int piezoSpeakerPin = 4;
const int whiskerLeftPin = 5;
const int whiskerRightPin = 7;
const int whiskerLeftIndicatorPin = 8;
const int whiskerRightIndicatorPin = 2;
const int servoSensorPin = 10;
const int servoLeftPin = 11;
const int servoRightPin = 12;
const uint8_t irSensorPin = A0;

const int rampIncrement = 5;

const int servoPulseMillis = 20;
const int standStillMicros = 1500;
const int fullSpeedClockwiseMicros = 1400;
const int fullSpeedCounterClockwiseMicros = 1600;
const int maxSpeedChg = 100;
const int servoSensorDelay = 0;
const int minIRObstacleDistance = 350; // millimeters
const int irSampleSize = 10;
const float headingPrecision = 5.0;

const int debug = 3;   // debug level. 1 is basic/info; 3 is details from within loops

//----------------------------- BEGIN IR Sensor GP2Y0A02YK Readout functions ---------------------------
// Convert IR Sensor GP2Y0A02YK Voltage read to millimeters
int convertIRvoltsToMM(float v) { 
    return -0.00003983993846*v*v*v+ 0.0456899769 *v*v - 17.48535575 * v + 2571.052715; 
}
// Return average distance in millimeters based on defined number of samples
int irAvgDistance(int numSamples) {
  float irSum = 0.0;
  for (int i = 0; i < numSamples; i++) irSum += analogRead(irSensorPin);
  return convertIRvoltsToMM(irSum / float(numSamples));
}
//----------------------------- END IR Sensor GP2Y0A02YK Readout functions -----------------------------


class Bot {
  public:
    int servoSensorPosition, servoSensorPositionOld;
    int servoLeftcurrentSpeed, servoRightCurrentSpeed;
    byte whiskerLeftState, whiskerRightState;
    bool obstacleLeft, obstacleRight, irObstacle, alreadyRampedFwd;
    int irObstacleDistance;
    Servo servoLeft, servoRight, servoSensor;
    Bot();
    /* ------------------------- Ramping version of movement primitives ------------------------- */
    void rampFwd(int time);
    void rampBack(int time);
    void rampRight(int deg);
    void rampLeft(int deg);
    /* ------------------------- Non-ramp version of movement primitives ------------------------- */
    void fullSpeedFwd(int time);
    void fullSpeedBack(int time);
    void turnRight(int deg);
    void turnLeft(int deg);
    void standStill();
    void standStill(int time);
    void endMovement();
    void updateMotion();
    void updateSensors();
    bool obstaclePresent();
    void updateIndicators();
    void moveServoSensor(int positionDegrees);
};
//-------------------------------------- BOT CONSTRUCTOR -----------------------------------------//
Bot::Bot() {
  servoSensor.attach(servoSensorPin);
  servoLeft.attach(servoLeftPin);
  servoRight.attach(servoRightPin);
  servoLeftcurrentSpeed = standStillMicros;
  servoRightCurrentSpeed = standStillMicros;
  obstacleLeft = false;
  obstacleRight = false;
  irObstacle = false;
  alreadyRampedFwd = false;
  irObstacleDistance = 65534;
  servoSensorPosition = 90;
  servoSensorPositionOld = 90;
  pinMode(irSensorPin, INPUT);
}
//--------------------------------------RAMP FWD/BACK -----------------------------------------//
void Bot::rampFwd(int time) {
  if (debug > 0) {
    Serial.print("Initiate RAMP FWD: ");
    Serial.println(time);
  }
  unsigned long currentTime = millis();
  if (!alreadyRampedFwd) {
    for (int speedChg = 0; speedChg <= maxSpeedChg; speedChg += rampIncrement) {
      updateSensors();
      if (!obstaclePresent()) {
        
        servoLeft.writeMicroseconds(standStillMicros + speedChg);
        if (debug > 2) {
          Serial.print("servoLeft: ");
          Serial.print(standStillMicros + speedChg);
        }
        servoRight.writeMicroseconds(standStillMicros - speedChg);
        if (debug > 2) {
          Serial.print("; servoRight: ");
          Serial.println(standStillMicros - speedChg);
        }
        delay(servoPulseMillis);
      } //end ramping, maintain top speed
      alreadyRampedFwd = true;
    }
  }
  while ((millis() - currentTime < time - servoPulseMillis * (maxSpeedChg / rampIncrement)) && !obstaclePresent()) { // run for specified time less ramp time
    updateSensors();
    delay(servoPulseMillis);
  }
  //standStill();
}
void Bot::rampBack(int time) {
  if (debug > 0) {
    Serial.print("Initiate RAMP BACK: ");
    Serial.println(time);
  }
  unsigned long currentTime = millis();
  for (int speedChg = 0; speedChg <= 100; speedChg += rampIncrement) {
    servoLeft.writeMicroseconds(standStillMicros - speedChg);
    servoRight.writeMicroseconds(standStillMicros + speedChg);
    delay(servoPulseMillis);
  }
  while ((millis() - currentTime < time - servoPulseMillis * (100 / rampIncrement))) { // run for specified time less ramp time
    updateSensors();
    delay(servoPulseMillis);
  }
}
//----------------------------------------RAMP TURN METHODS-----------------------------------------//
void Bot::rampRight(int deg) {
  for (int speedChg = 0; speedChg <= 100; speedChg += rampIncrement) {
    servoLeft.writeMicroseconds(standStillMicros - speedChg);
    servoRight.writeMicroseconds(standStillMicros - speedChg);
    delay(servoPulseMillis);
  }
  int turnTime = map(deg, 0, 180, 0, 1500 - 100 / rampIncrement);
  delay(turnTime);
}
void Bot::rampLeft(int deg) {
  for (int speedChg = 0; speedChg <= 100; speedChg += rampIncrement) {
    servoLeft.writeMicroseconds(standStillMicros + speedChg);
    servoRight.writeMicroseconds(standStillMicros + speedChg);
    delay(servoPulseMillis);
  }
  int turnTime = map(deg, 0, 180, 0, 1600 - 100 / rampIncrement);
  delay(turnTime);
}
//--------------------------------------NON-RAMP FWD/BACK/STILL -----------------------------------------//
void Bot::standStill() {
  servoLeft.writeMicroseconds(standStillMicros);
  servoRight.writeMicroseconds(standStillMicros);
}
void Bot::standStill(int time) {
  servoLeft.writeMicroseconds(standStillMicros);
  servoRight.writeMicroseconds(standStillMicros);
  delay(time);
}
void Bot::endMovement() {
  standStill(servoPulseMillis);
  servoLeft.detach();
  servoRight.detach();
}
void Bot::fullSpeedFwd(int time) {
  servoLeft.writeMicroseconds(fullSpeedCounterClockwiseMicros);
  servoRight.writeMicroseconds(fullSpeedClockwiseMicros);
  delay(time);
}
void Bot::fullSpeedBack(int time) {
  servoLeft.writeMicroseconds(fullSpeedClockwiseMicros);;
  servoRight.writeMicroseconds(fullSpeedCounterClockwiseMicros);
  delay(time);
}
//----------------------------------------NON-RAMP TURN METHODS-----------------------------------------//
void Bot::turnRight(int deg) {
  int originalHeading = getHeading();
  int targetHeading = (originalHeading + deg) % 360;
  if (debug > 0) {
    Serial.print("Initiate TURN RIGHT: ");
    Serial.print(deg);
    Serial.print(";   Old heading: ");
    Serial.print(originalHeading);
    Serial.print(";   Target heading: ");
    Serial.println(targetHeading);
  }
  if (originalHeading < targetHeading) { // not crossing 359->0 boundary
    while (getHeading() < targetHeading) {
      if (debug > 2) Serial.println(getHeading());
      servoLeft.writeMicroseconds(fullSpeedCounterClockwiseMicros);
      servoRight.writeMicroseconds(fullSpeedCounterClockwiseMicros);
      delay(servoPulseMillis);
    }
  } else if (originalHeading > targetHeading) {
    while (getHeading() > targetHeading) { //only works until it hits 0
      if (debug > 2) Serial.println(getHeading());
      servoLeft.writeMicroseconds(fullSpeedCounterClockwiseMicros);
      servoRight.writeMicroseconds(fullSpeedCounterClockwiseMicros);
      delay(servoPulseMillis);
    }
    turnRight(targetHeading); //remainder of the turn after hitting 0
  }
  standStill();
}
void Bot::turnLeft(int deg) {
  if (debug > 0) {
    Serial.print("Initiate TURN LEFT: ");
    Serial.println(deg);
  }
  servoLeft.writeMicroseconds(fullSpeedClockwiseMicros);
  servoRight.writeMicroseconds(fullSpeedClockwiseMicros);
  int turnTime = map(deg, 0, 180, 0, standStillMicros);
  delay(turnTime);
}
//---------------------------------------------MOTION UPDATE-----------------------------------------//
void Bot::updateMotion() {
  while (obstacleLeft || irObstacle) {
    rampBack(1000);
    turnRight(90);
    updateSensors();
  }
  while (obstacleRight) {
    rampBack(1500);
    turnLeft(90);
    updateSensors();
  }
  while (!obstaclePresent()) {
    rampFwd(3000);
  }
}


// NON-DELAY METHODS!
//---------------------------------------------SENSOR METHODS-----------------------------------------//
void Bot::updateSensors() {
  // Update IR Sensor info, averaged over specified number of samples
  irObstacleDistance = irAvgDistance(irSampleSize);
  if (irObstacleDistance <= minIRObstacleDistance) {
    irObstacle = true;
    if (debug > 0) {
      Serial.print("IR Obstacle Distance: ");
      Serial.println(irObstacleDistance);
    } 
  } else {
    irObstacle = false;
      if (debug > 3) {
        Serial.print("IR Obstacle Distance: ");
        Serial.println(irObstacleDistance);
      } 
  }
  
  // Update Whisker sensor states
  whiskerLeftState = digitalRead(whiskerLeftPin);
  whiskerRightState = digitalRead(whiskerRightPin);
  if (whiskerLeftState == 0) {
    digitalWrite(whiskerLeftIndicatorPin, HIGH);
    obstacleLeft = true;
    if (debug > 0) Serial.println("Obstacle on Left encountered!");
  } else {
    digitalWrite(whiskerLeftIndicatorPin, LOW);
    obstacleLeft = false;
  }
  if (whiskerRightState == 0) {
    digitalWrite(whiskerRightIndicatorPin, HIGH);
    obstacleRight = true;
    if (debug > 0) Serial.println("Obstacle on Right encountered!");
  } else {
    digitalWrite(whiskerRightIndicatorPin, LOW);
    obstacleRight = false;
  }
}
bool Bot::obstaclePresent() { return obstacleLeft || obstacleRight || irObstacle; }
void Bot::updateIndicators() {
}
void Bot::moveServoSensor(int degreePosition) {
  if (degreePosition >= servoSensorPositionOld) {
    for(int i = servoSensorPositionOld; i < degreePosition; i++)  
    {                                   
      servoSensor.write(i); 
      if (debug > 2) {     
        Serial.print("Wrote to servoSensor: ");
        Serial.println(i);
      }
      delay(servoSensorDelay);                       
    } 
  } else if (degreePosition < servoSensorPositionOld) {
    for(int i = servoSensorPositionOld; i > degreePosition; i--)     
    {                                
      servoSensor.write(i);               
      if (debug > 2) {
        Serial.print("Wrote to servoSensor: ");
        Serial.println(i);
      }
      delay(servoSensorDelay);                    
    } 
  }
  servoSensorPositionOld = degreePosition;
  if (debug > 0) {
    Serial.print("Wrote to servoSensor: ");
    Serial.println(degreePosition);
  }
}

#endif /* BOT_H_ */



void startupChime() {
  int pin = piezoSpeakerPin;
  int duration = 300;
  int chord[] {NOTE_G5, NOTE_C5, NOTE_E5};
  for (unsigned int note = 0; note < (sizeof(chord) / sizeof(*chord)); note++) {
    tone(pin, chord[note], duration); //pin, frequency, duration
    delay(duration);
    noTone(pin);
  }
}

Bot *me;
/* Assign a unique ID to magnetometer */
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

void displaySensorDetails(void) {
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}
int getHeading() {
  sensors_event_t event; 
  mag.getEvent(&event);
  float heading = (atan2(event.magnetic.y,event.magnetic.x) * 180) / Pi; // Calculate the angle of the vector y,x
  if (heading < 0) heading = 360 + heading;  // Normalize to 0-360
  if (debug > 1) {
    Serial.print("Compass Heading: ");
    Serial.println(int(heading)); 
  }
  return int(heading);
}

void setup() {
  // put your setup code here, to run once:
  startupChime(); // Play at startup to detect brownouts
  me = new Bot();
  pinMode(whiskerLeftPin, INPUT);
  pinMode(whiskerRightPin, INPUT);
  pinMode(whiskerLeftIndicatorPin, OUTPUT);
  pinMode(whiskerRightIndicatorPin, OUTPUT);
  Serial.begin(9600);
  
  Serial.println("Magnetometer Test"); Serial.println("");
  mag.enableAutoRange(true); /* Enable auto-gain */
  if(!mag.begin()) /* Initialise the sensor */
  {
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  displaySensorDetails();

}


int loopCounter = 1;
int maxLoop = 1;
int pos;

void loop() {
  // Punctuate the loop execution with Serial input, please enter integers 0-180 only
  //while (Serial.available() == 0);
  //pos = Serial.parseInt();
  
  if (debug > 0) {
    Serial.print("loop: ");
    Serial.println(loopCounter++);
  }
  
  // Code for testing the IR-sensor servo
  //me->moveServoSensor(pos);
  
  //me->updateSensors();
  //me->updateMotion();
  //Serial.println(irAvgDistance(irSampleSize));
  
  me->turnRight(90);
  delay(5000);
  
}


