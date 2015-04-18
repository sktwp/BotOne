// Motion/Sensor test platform - Arduino Uno R3

#include <Adafruit_LSM303_U.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <XBee.h>
#include "pitches.h"

#define STAND_STILL_MICROS 1500
#define SERIAL_BAUD 57600
#define HAS_WHISKERS
#define MAGNETOMETER_ID 8780
#define ARDUINO_REMOTE
#define REMOTE_XBEE_16ADDR 0x0001
#define OBSTACLE_AVOIDANCE_TURN 45 //degrees

//======================================== GLOBALS ====================================================
const float Pi = 3.14159;
const int debug = 2;   // debug level. 1 is basic/info; 3 is details from within loops

#ifndef BOT_H_
#define BOT_H_

const bool hasWhiskers = false; // whether whiskers are on or not;

const uint8_t piezoSpeakerPin = 4;

// Only needed if Bot has whiskers
const uint8_t whiskerLeftPin = 5;
const uint8_t whiskerRightPin = 7;
const uint8_t whiskerLeftIndicatorPin = 8;
const uint8_t whiskerRightIndicatorPin = 2;

const uint8_t servoSensorPin = 10;
const uint8_t servoLeftPin = 11;
const uint8_t servoRightPin = 12;
const uint8_t irSensorPin = A0;
const uint8_t ssRX = 3; //Soft Serial RX Pin
const uint8_t ssTX = 2; //SoftSerial TX Pin

const int rampIncrement = 5;

const int servoPulseMillis = 20;
const int standStillMicros = 1500;
const int fullSpeedClockwiseMicros = 1400;
const int fullSpeedCounterClockwiseMicros = 1600;
const int slowSpeedClockwiseMicros = 1470;
const int slowSpeedCounterClockwiseMicros = 1530;
const int maxSpeedChg = 100;
const int servoSensorDelay = 0;
const int minIRObstacleDistance = 350; // millimeters
const int irSampleSize = 10;
const float headingPrecision = 5.0;
const int tolerance = 10; // ABS(speed) below this value do not induce motion, also deg for turns
//======================================== END GLOBALS =================================================


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
    Adafruit_LSM303_Mag_Unified mag;
    int servoSensorPosition, servoSensorPositionOld;
    int servoLeftcurrentSpeed, servoRightCurrentSpeed;
    byte whiskerLeftState, whiskerRightState;
    bool obstacleLeft, obstacleRight, irObstacle, alreadyRampedFwd;
    int irObstacleDistance;
    Servo servoLeft, servoRight, servoSensor;
    
    Bot();
    void setLinearSpeed(int linearSpeed);
    void setTurnSpeed(int turnSpeed, int linearSpeed);
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
    void displayLSM303Details();
    int getHeading();
    void startupChime();
};
//-------------------------------------- BOT CONSTRUCTOR -----------------------------------------//
Bot::Bot() {
  mag = Adafruit_LSM303_Mag_Unified(MAGNETOMETER_ID);
  if (debug > 1) Serial.println("Magnetometer Test");
  mag.enableAutoRange(true); /* Enable auto-gain */
  if (!mag.begin()) Serial.println("No LSM303 detected"); /* Initialise the sensor */

  
  //servoSensor.attach(servoSensorPin);
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
  if(hasWhiskers) {
    pinMode(whiskerLeftPin, INPUT);
    pinMode(whiskerRightPin, INPUT);
    pinMode(whiskerLeftIndicatorPin, OUTPUT);
    pinMode(whiskerRightIndicatorPin, OUTPUT);
  }
}
//-------------------------------------- LINEAR AND TURN SPEED -------------------------------//
void Bot::setLinearSpeed(int linearSpeed) { //needs to be in -100 to 100 range
  if (abs(linearSpeed) < tolerance) {
    standStill();
  } else if (linearSpeed <= 100 || linearSpeed >= -100) {
    servoLeft.writeMicroseconds(standStillMicros + linearSpeed);
    servoRight.writeMicroseconds(standStillMicros - linearSpeed);
  } 
}
// Because both turns and linear motion use the same servos, needed to create a smooth turn function
// slowing one of the wheels, so turns take linear motion into account
void Bot::setTurnSpeed(int turnSpeed, int linearSpeed) {
  if (abs(linearSpeed) < tolerance && (turnSpeed <= 100 || turnSpeed >= -100)) {
    // Turning from stationary position - take control of servos
    servoLeft.writeMicroseconds(standStillMicros + turnSpeed);
    servoRight.writeMicroseconds(standStillMicros + turnSpeed);
  } else if (turnSpeed <= 100 || turnSpeed >= -100) {
    // Turning while moving
    servoLeft.writeMicroseconds(standStillMicros + linearSpeed + turnSpeed);
    servoRight.writeMicroseconds(standStillMicros - linearSpeed + turnSpeed);
  } 
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

//---------------------------------------- MAG HEADING-BASED TURN METHODS------------------//
void Bot::turnRight(int deg) {
  int originalHeading = getHeading();
  int targetHeading = (originalHeading + deg) % 360;
  if (debug > 0) {
    Serial.print("Initiate TURN RIGHT: "); Serial.print(deg);
    Serial.print(";   Old heading: "); Serial.print(originalHeading);
    Serial.print(";   Target heading: "); Serial.println(targetHeading);
  }
  while (abs(getHeading() - targetHeading) > tolerance) {
    if (debug > 1) { Serial.print("headingDiff: "); Serial.println(abs(getHeading() - targetHeading)); }
    servoLeft.writeMicroseconds(slowSpeedCounterClockwiseMicros);
    servoRight.writeMicroseconds(slowSpeedCounterClockwiseMicros);
    delay(servoPulseMillis);
  }
  standStill();
}
void Bot::turnLeft(int deg) {
  int originalHeading = getHeading();
  int targetHeading = (originalHeading + deg) % 360;
  if (debug > 0) {
    Serial.print("Initiate TURN LEFT: "); Serial.print(deg);
    Serial.print(";   Old heading: "); Serial.print(originalHeading);
    Serial.print(";   Target heading: "); Serial.println(targetHeading);
  }
  while (abs(getHeading() - targetHeading) > tolerance || abs(getHeading() - targetHeading) < 360 - tolerance) {
    if (debug > 1) { Serial.print("headingDiff: "); Serial.println(abs(getHeading() - targetHeading)); }
    servoLeft.writeMicroseconds(slowSpeedClockwiseMicros);
    servoRight.writeMicroseconds(slowSpeedClockwiseMicros);
    delay(servoPulseMillis);
  }
  standStill();
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
  // @TODO - replace polling with interrupts - digital pin 2,3 for Uno
  if (hasWhiskers) {
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
void Bot::displayLSM303Details() {
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

int Bot::getHeading() {
  sensors_event_t event; 
  mag.getEvent(&event);
  float heading = (atan2(event.magnetic.y,event.magnetic.x) * 180) / Pi; // Calculate the angle of the vector y,x
  if (heading < 0) heading = 360 + heading;  // Normalize to 0-360
  if (debug > 4) {
    Serial.print("Compass Heading: ");
    Serial.println(int(heading)); 
  }
  return int(heading);
}
void Bot::startupChime() {
  int pin = piezoSpeakerPin;
  int duration = 300;
  int chord[] {NOTE_G5, NOTE_C5, NOTE_E5};
  for (unsigned int note = 0; note < (sizeof(chord) / sizeof(*chord)); note++) {
    tone(pin, chord[note], duration); //pin, frequency, duration
    delay(duration);
    noTone(pin);
  }
}


#endif // BOT_H_ 

//======================================== INITIALIZATION ============================================

Bot *bot;
SoftwareSerial xbeeSerial(ssRX, ssTX);
XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
Rx16Response rx16 = Rx16Response();
Rx16IoSampleResponse ioSample = Rx16IoSampleResponse();
uint8_t xbeeRxOption = 0;
uint8_t xbeeRxData[2];
String msg = "";
unsigned long lastTransmission = millis();

//=========================================== XBEE HELPERS ======================================

void updateJoystickValues(int *xV, int *yV) {
  xbee.readPacket();
  if (xbee.getResponse().isAvailable()) {
    if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE) {
      TxStatusResponse txStatus = TxStatusResponse();
      xbee.getResponse().getZBTxStatusResponse(txStatus);
      if (txStatus.getStatus() == SUCCESS && debug > 2) Serial.println("SUCCESS");
      else if (txStatus.getStatus() != SUCCESS && debug > 0) Serial.println("TX ERROR");
    } else if (xbee.getResponse().getApiId() == RX_16_RESPONSE) {
      xbee.getResponse().getRx16Response(rx16);
      xbeeRxOption = rx16.getOption();
      xbeeRxData[0] = rx16.getData(0);
      xbeeRxData[1] = rx16.getData(1);
      (*xV) = xbeeRxData[0];
      (*yV) = xbeeRxData[1];
      if (debug > 1) {
        msg = "xBee RFrx Data: "+ String(xbeeRxData[0]) + ", "+ String(xbeeRxData[1]); Serial.println(msg);
      }  
    }
   #ifndef ARDUINO_REMOTE 
    else if (xbee.getResponse().getApiId() == RX_16_IO_RESPONSE) {
      xbee.getResponse().getRx16IoSampleResponse(ioSample);
      if (debug > 2) {
        Serial.print("Received I/O Sample from: "); Serial.println(ioSample.getRemoteAddress16(), HEX);  
        Serial.print("Sample size is "); Serial.println(ioSample.getSampleSize(), DEC);
        if (ioSample.containsAnalog())  Serial.println("Sample contains analog data");
        if (ioSample.containsDigital()) Serial.println("Sample contains digital data");
        for (int k = 0; k < ioSample.getSampleSize(); k++) {
          Serial.print("Sample "); Serial.print(k + 1, DEC); Serial.println(":");    
          for (int i = 0; i <= 5; i++) {
            if (ioSample.isAnalogEnabled(i)) { msg = "Analog (AI") + String(i, DEC) + ") is " + String(ioSample.getAnalog(i, k)); Serial.println(msg); }
          }
          for (int i = 0; i <= 8; i++) {
            if (ioSample.isDigitalEnabled(i)) { msg = "Digtal (DI"+ String(i, DEC) + ") is "+ String(ioSample.isDigitalOn(i, k)); Serial.println(msg);}
          }
        }
      }
      if(ioSample.isAnalogEnabled(0)) {
        (*xV) = ioSample.getAnalog(0, 0); //analog value index, sample index
        (*yV) = ioSample.getAnalog(1, 0);
        if (debug > 2) { msg = "x = " + String(*xV) + "; y = " + String(*yV); Serial.println(msg); }
      }
    } else if (debug > 1) {
      Serial.print("Expected I/O Sample, got "); Serial.println(xbee.getResponse().getApiId(), HEX);
    }
    #endif
  } else if (xbee.getResponse().isError() && debug > 1) {
    Serial.print("Pkt read err: "); Serial.println(xbee.getResponse().getErrorCode());
  }
}

void sendToRemote(String msg, long transmissionInterval) {
  if (millis() - lastTransmission > transmissionInterval) {
    uint8_t payload[msg.length()];
    msg.getBytes(payload, msg.length());
    Tx16Request tx = Tx16Request(REMOTE_XBEE_16ADDR, payload, sizeof(payload));
    xbee.send(tx);
    lastTransmission = millis();
  }
}

//======================================== SETUP() ============================================
void setup() {
  bot = new Bot();
  bot->startupChime(); // Play at startup to detect brownouts
  Serial.begin(SERIAL_BAUD); // Terminal
  xbeeSerial.begin(SERIAL_BAUD); // xBee
  xbee.setSerial(xbeeSerial);
  if (debug > 1) bot->displayLSM303Details();

}

//======================================== LOOP() ============================================
int loopCounter = 1;
int maxLoop = 1;
int pos;
int xV, yV; // x and y values read from remote control joystick via xBee

void loop() {
  updateJoystickValues(&xV, &yV);
  int linearSpeed = map(yV, 0, 255, -100, 100);
  int turnSpeed   = map(xV, 0, 255, -100, 100);
  if (debug > 0) {
    msg = "turn:" + String(turnSpeed) + ";linear:"+ String(linearSpeed) + " "; Serial.println(msg);
    sendToRemote(msg, 100);
  }
  bot->setLinearSpeed(linearSpeed);
  bot->setTurnSpeed(turnSpeed, linearSpeed);
  /*
  bot->updateSensors();
  if (bot->irObstacleDistance < 300) {
    tone(piezoSpeakerPin, map(bot->irObstacleDistance, 0, 300, 3000, 50), 300); //pin, Hz, millis
    // @TODO - very rough test - replace later
    msg = "obstacle:" + String(bot->irObstacleDistance) + "mm; turn " + OBSTACLE_AVOIDANCE_TURN + "deg";
    sendToRemote(msg, 100);
    bot->turnRight(OBSTACLE_AVOIDANCE_TURN);
  }
  */

}





