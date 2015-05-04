// Motion/Sensor test platform - Arduino Uno R3 - BRANCH 2 - comm debug

#include <Adafruit_LSM303_U.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <XBee.h>
#include "pitches.h"
#include "Bot.h"

#define SERIAL_BAUD 57600

// See Bot.h for pin assignments and key parameters

//======================================== INITIALIZATION ============================================

Bot *bot;
SoftwareSerial xbeeSerial(ssRX, ssTX);
XBee xbee = XBee();
uint8_t xbeeRxOption = 0;
uint8_t xbeeRxData[2] = {128, 128};
String msg = "";
unsigned long lastTransmission = millis();

//=========================================== XBEE HELPERS ======================================

void updateJoystickValues(int *xV, int *yV) {
  //(*xV) = 128;
  //(*yV) = 128;
  XBeeResponse response = XBeeResponse();
  xbee.readPacket();
  xbee.getResponse(response);
  if (response.isAvailable()) {
    if (response.getApiId() == RX_16_RESPONSE) {
      Rx16Response rx16 = Rx16Response();
      response.getRx16Response(rx16);
      xbeeRxOption = rx16.getOption();
      xbeeRxData[0] = rx16.getData(0);
      xbeeRxData[1] = rx16.getData(1);
      (*xV) = xbeeRxData[0];
      (*yV) = xbeeRxData[1];
      if (debug > 1) {
        msg = "xBee RFrx Data: "+ String(xbeeRxData[0]) + ", "+ String(xbeeRxData[1]); Serial.println(msg);
      }  
    } else if (response.getApiId() == TX_STATUS_RESPONSE) {
      TxStatusResponse txStatus = TxStatusResponse();
      response.getZBTxStatusResponse(txStatus);
      if (txStatus.getStatus() == SUCCESS && debug > 2) Serial.println("SUCCESS");
      else if (txStatus.getStatus() != SUCCESS && debug > 0) Serial.println("TX ERROR");
    }
   #ifndef ARDUINO_REMOTE 
    else if (response.getApiId() == RX_16_IO_RESPONSE) {
      Rx16IoSampleResponse ioSample = Rx16IoSampleResponse();
      response.getRx16IoSampleResponse(ioSample);
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
    if (debug > 2) { Serial.print("Sent MSG to Remote: "); Serial.println(msg); }
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
  //xbee.begin(xbeeSerial);
  delay(2000);
  if (debug > 1) bot->displayLSM303Details();
}

//======================================== LOOP() ============================================
int loopCounter = 1;
int maxLoop = 1;
int pos;
int xV, yV; // x and y values read from remote control joystick via xBee

void loop() {
  updateJoystickValues(&xV, &yV);
  int linearSpeed = map(yV, 0, 254, -100, 100);
  int turnSpeed   = map(xV, 1, 253, -100, 100);
  if (debug > 0) {
    msg = "turn:" + String(xV) + ";lin:"+ String(yV) + " "; //Serial.println(msg);
    sendToRemote(msg, 1000);
  }
  bot->setLinearSpeed(linearSpeed);
  bot->setTurnSpeed(turnSpeed, linearSpeed);
  
  //bot->updateSensors();
  /*
  if (bot->irObstacleDistance < 300) {
    tone(piezoSpeakerPin, map(bot->irObstacleDistance, 0, 300, 3000, 50), 300); //pin, Hz, millis
    // @TODO - very rough test - replace later
    msg = "obstacle:" + String(bot->irObstacleDistance) + "mm; turn " + OBSTACLE_AVOIDANCE_TURN + "*"; 
    Serial.println(msg);
    sendToRemote(msg, 100);
    bot->turnRight(OBSTACLE_AVOIDANCE_TURN);
  }
  */

}





