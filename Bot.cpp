#include "Bot.h"



//----------------------------- IR Sensor GP2Y0A02YK Readout functions ---------------------------
// Convert IR Sensor GP2Y0A02YK Voltage read to millimeters
int convertIRvoltsToMM(float v) { 
    return -0.00003983993846*v*v*v+ 0.0456899769*v*v - 17.48535575*v + 2571.052715; 
}
// Return average distance in millimeters based on defined number of samples
int irAvgDistance(int numSamples) {
  float irSum = 0.0;
  for (int i = 0; i < numSamples; i++) { irSum += analogRead(irSensorPin); delay(1); }
  return convertIRvoltsToMM(irSum / float(numSamples));
}
//----------------------------- Sonar  Maxbotix LV-EZ3 Readout functions -----------------------------
int convertSonarVoltsToMM(float v) {
  return 0.0179*v*v*v - 1.1553*v*v + 39.334*v - 101.28;
  //return int(v);
}
int sonarAvgDistance(int numSamples) {
  float sonarSum = 0.0;
  for (int i = 0; i < numSamples; i++) { sonarSum += analogRead(sonarANPin); delay(1); }
  return convertSonarVoltsToMM(sonarSum / float(numSamples));
}

//-------------------------------------- CONSTRUCTOR --------------------------------------//
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
  sonarObstacle = false;
  irObstacle = false;
  alreadyRampedFwd = false;
  sonarObstacleDistance = 65534;
  irObstacleDistance = 65534;
  servoSensorPosition = 90;
  servoSensorPositionOld = 90;
  pinMode(irSensorPin, INPUT);
  pinMode(sonarANPin, INPUT);
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
  if (debug > 0) { Serial.print("Initiate RAMP FWD: "); Serial.println(time); }
  unsigned long currentTime = millis();
  if (!alreadyRampedFwd) {
    for (int speedChg = 0; speedChg <= maxSpeedChg; speedChg += rampIncrement) {
      updateSensors();
      if (!obstaclePresent()) {
        servoLeft.writeMicroseconds(standStillMicros + speedChg);
        if (debug > 2) { Serial.print("servoLeft: "); Serial.print(standStillMicros + speedChg); }
        servoRight.writeMicroseconds(standStillMicros - speedChg);
        if (debug > 2) { Serial.print("; servoRight: "); Serial.println(standStillMicros - speedChg); }
        delay(servoPulseMillis);
      } //end ramping, maintain top speed
      alreadyRampedFwd = true;
    }
  }
  // run for specified time less ramp time
  while ((millis() - currentTime < time - servoPulseMillis * (maxSpeedChg / rampIncrement)) && !obstaclePresent()) { 
    updateSensors();
    delay(servoPulseMillis);
  }
}
void Bot::rampBack(int time) {
  if (debug > 0) { Serial.print("Initiate RAMP BACK: "); Serial.println(time); }
  unsigned long currentTime = millis();
  for (int speedChg = 0; speedChg <= 100; speedChg += rampIncrement) {
    servoLeft.writeMicroseconds(standStillMicros - speedChg);
    servoRight.writeMicroseconds(standStillMicros + speedChg);
    delay(servoPulseMillis);
  }
 // run for specified time less ramp time
  while ((millis() - currentTime < time - servoPulseMillis * (100 / rampIncrement))) {
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
  while (abs(getHeading() - targetHeading) > tolerance || abs(getHeading() - targetHeading) < 360 - tolerance) {
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
  while (obstaclePresent()) {
    rampBack(1000);
    turnRight(OBSTACLE_AVOIDANCE_TURN);
    updateSensors();
  }
}


// NON-DELAY METHODS!
//---------------------------------------------SENSOR METHODS-----------------------------------------//
void Bot::updateSensors() {
  // Update IR Sensor info, averaged over specified number of samples
  irObstacleDistance = irAvgDistance(irSampleSize);
  if (irObstacleDistance <= minIRObstacleDistance) {
    irObstacle = true;
    if (debug > 0) { Serial.print("IR Obstacle Distance: "); Serial.println(irObstacleDistance); } 
  } else {
    irObstacle = false;
    if (debug > 3) { Serial.print("IR Obstacle Distance: "); Serial.println(irObstacleDistance); } 
  }
  sonarObstacleDistance = sonarAvgDistance(sonarSampleSize);
  if (sonarObstacleDistance <= minSonarObstacleDistance) {
    sonarObstacle = true;
    if (debug > 0) { Serial.print("SONAR Obstacle Distance: "); Serial.println(sonarObstacleDistance); } 
  } else {
    sonarObstacle = false;
    if (debug > 3) { Serial.print("SONAR Obstacle Distance: "); Serial.println(sonarObstacleDistance); } 
  }
}
bool Bot::obstaclePresent() { return sonarObstacle || irObstacle; }
void Bot::updateIndicators() {
}
void Bot::moveServoSensor(int degreePosition) {
  if (degreePosition >= servoSensorPositionOld) {
    for(int i = servoSensorPositionOld; i < degreePosition; i++) {                                   
      servoSensor.write(i); 
      if (debug > 2) { Serial.print("Wrote to servoSensor: "); Serial.println(i); }
      delay(servoSensorDelay);                       
    } 
  } else if (degreePosition < servoSensorPositionOld) {
    for(int i = servoSensorPositionOld; i > degreePosition; i--) {                                
      servoSensor.write(i);               
      if (debug > 2) { Serial.print("Wrote to servoSensor: "); Serial.println(i); }
      delay(servoSensorDelay);                    
    } 
  }
  servoSensorPositionOld = degreePosition;
  if (debug > 0) { Serial.print("Wrote to servoSensor: "); Serial.println(degreePosition); }
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
}

int Bot::getHeading() {
  sensors_event_t event; 
  mag.getEvent(&event);
  float heading = (atan2(event.magnetic.y,event.magnetic.x) * 180) / Pi; // Calculate the angle of the vector y,x
  if (heading < 0) heading = 360 + heading;  // Normalize to 0-360
  if (debug > 4) { Serial.print("Compass Heading: "); Serial.println(int(heading)); }
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


