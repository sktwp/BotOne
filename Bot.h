#ifndef BOT_H_
#define BOT_H_

#include <Adafruit_LSM303_U.h>
#include <Servo.h>
#include "pitches.h"

#define STAND_STILL_MICROS 1500
#define MAGNETOMETER_ID 8780
#define ARDUINO_REMOTE
#define REMOTE_XBEE_16ADDR 0x0001
#define OBSTACLE_AVOIDANCE_TURN 45 //degrees

//======================================== GLOBALS ====================================================
const float Pi = 3.14159;
const int debug = 3;   // debug level. 1 is basic/info; 3 is details from within loops



//======================================== PIN ASSIGNMENTS - DIGITAL ==================================
const uint8_t ssRX = 3; //Soft Serial RX Pin
const uint8_t ssTX = 2; //SoftSerial TX Pin
const uint8_t piezoSpeakerPin = 4;
const uint8_t sonarPWPin = 6;
const uint8_t servoSensorPin = 10;
const uint8_t servoLeftPin = 11;
const uint8_t servoRightPin = 12;

//======================================== PIN ASSIGNMENTS - ANALOG ===================================
const uint8_t irSensorPin = A0;
const uint8_t sonarANPin = A1;
const uint8_t sonarRXPin = A2;

//======================================== KEY OPERATING PARAMETERS ===================================
const int rampIncrement = 5;

const int servoPulseMillis = 20;
const int standStillMicros = 1500;
const int fullSpeedClockwiseMicros = 1400;
const int fullSpeedCounterClockwiseMicros = 1600;
const int slowSpeedClockwiseMicros = 1470;
const int slowSpeedCounterClockwiseMicros = 1530;
const int maxSpeedChg = 100;
const int servoSensorDelay = 0;
const int minIRObstacleDistance = 250; // millimeters
const int minSonarObstacleDistance = 250; // millimeters
const int irSampleSize = 10;
const int sonarSampleSize = 10;
const float headingPrecision = 5.0;
const int tolerance = 10; // ABS(speed) below this value does not induce motion, also deg for turns
//======================================== END GLOBALS =================================================

class Bot {
  public:
    Adafruit_LSM303_Mag_Unified mag;
    int servoSensorPosition, servoSensorPositionOld;
    int servoLeftcurrentSpeed, servoRightCurrentSpeed;
    bool sonarObstacle, irObstacle, alreadyRampedFwd;
    int sonarObstacleDistance, irObstacleDistance;
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

#endif //BOT_H_
