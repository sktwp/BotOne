/* 
 Remote based on Arduino Uno R3 + Sparkfun xBee shield + Sparkfun Joystick shield
 This is a more compact remote, with addition of LCD display with i2c/SPI LCD backpack
 using MCP23008 I2C expander (Adafuit)
 
 LCD circuit:
 * CLK to Analog #5
 * DAT to Analog #4
*/
  
#include <SoftwareSerial.h>
#include <XBee.h>
#include <Wire.h>
#include <LiquidTWI.h>

#define XBEE_SHIELD
#define SERIAL_BAUD 9600
#define BOT_XBEE_16ADDR 0x0002
#define LCD_BUFFER 40
#define MAX_DATA_PAYLOAD 200

//======================================== GLOBALS ============================================
// XBee's DOUT (TX) is connected to pin 2 (Arduino's Software RX)
// XBee's DIN  (RX) is connected to pin 3 (Arduino's Software TX)
const uint8_t ssRX = 2; //Soft Serial RX Pin
const uint8_t ssTX = 3; //SoftSerial TX Pin

//Joystick direction:
const byte PIN_ANALOG_X = A0;
const byte PIN_ANALOG_Y = A1;

//Arduino pins associated with each input from Sparkfun Joystick shield
const byte PIN_BUTTON_UP = 4;
const byte PIN_BUTTON_DOWN = 5;
const byte PIN_BUTTON_LEFT = 6;

#ifndef XBEE_SHIELD
const byte PIN_BUTTON_RIGHT = 3;
const byte PIN_BUTTON_SELECT = 2; // Select button is triggered when joystick is pressed
#endif

const int debug = 3;
//======================================== END GLOBALS ========================================


void setupJoystickShield() {
  // Enable Arduino's internal "pull-up" resistors for each pushbutton input, 
  // so the shield doesn't have to have resistors on it.
  // When a pull-up resistor is used on a pin the meaning of the values read 
  // are reversed compared to their usual meanings:
  //    * HIGH = the button is not pressed
  //    * LOW = the button is pressed  
  
  pinMode(PIN_BUTTON_LEFT, INPUT);  
  digitalWrite(PIN_BUTTON_LEFT, HIGH);
  
  pinMode(PIN_BUTTON_UP, INPUT);  
  digitalWrite(PIN_BUTTON_UP, HIGH);
  
  pinMode(PIN_BUTTON_DOWN, INPUT);  
  digitalWrite(PIN_BUTTON_DOWN, HIGH);

  #ifndef XBEE_SHIELD
/* These two buttons are wired to pins 2 & 3 on Sparkfun Joystick shield, 
   same as RX/TX pins on xBee shield, and therefore cannot use them
   @TODO: change xBee shield wiring to redirect TX & RX to other pins */    
  pinMode(PIN_BUTTON_RIGHT, INPUT);  
  digitalWrite(PIN_BUTTON_RIGHT, HIGH);
  
  pinMode(PIN_BUTTON_SELECT, INPUT);  
  digitalWrite(PIN_BUTTON_SELECT, HIGH);  
  #endif 
}

String readInputState() {
  //Button states
  String msg = "l:" + String(digitalRead(PIN_BUTTON_LEFT)) + " u:" + String(digitalRead(PIN_BUTTON_UP)) + " d:" + String(digitalRead(PIN_BUTTON_DOWN));
  #ifndef XBEE_SHIELD
  msg += " r:" + String(digitalRead(PIN_BUTTON_RIGHT)) + " s:" + String(digitalRead(PIN_BUTTON_SELECT));
  #endif
  //Joystick position:
  msg += " x:"+ String(analogRead(PIN_ANALOG_X)) +" y:" + String(analogRead(PIN_ANALOG_Y)); 
  return msg;
}


//======================================== INITIALIZATION =====================================
SoftwareSerial xbeeSerial(ssRX, ssTX); // RX, TX
XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
Rx16Response rx16 = Rx16Response();
TxStatusResponse txStatus = TxStatusResponse();
LiquidTWI lcd(0);
String msg = "";
String msg1 = "";
unsigned long lastLCDRefresh = millis();
unsigned long lastTransmission = millis();

//======================================== XBEE HELPERS =======================================
void receiveFromBot(uint8_t* data, long* dataLength) {
  xbee.readPacket();
  xbee.getResponse(response);
  if (response.isAvailable()) { 
    if (response.getApiId() == RX_16_RESPONSE || true) {
      response.getRx16Response(rx16);
      if (debug > 2) { Serial.print("Rx Data Length: "); Serial.println(rx16.getDataLength()); }
      long limitedLength = min(rx16.getDataLength(), MAX_DATA_PAYLOAD);
      uint8_t option = 0;
      option = rx16.getOption();
      (*dataLength) = limitedLength;
      for (int i = 0; i < limitedLength; i++) data[i] = rx16.getData(i);
    } else if (response.getApiId() == TX_STATUS_RESPONSE) {
      response.getZBTxStatusResponse(txStatus);
      if (txStatus.getStatus() == SUCCESS && debug > 2) Serial.println("SUCCESS");
      else if (txStatus.getStatus() != SUCCESS && debug > 0) Serial.println("TX ERROR");
    }  
  } else if (response.isError()) {
      if (debug > 0) { 
      String msg = "Pkt read err: " + String(response.getErrorCode()); Serial.println(msg); 
      msg = "Got API ID: " + String(response.getApiId(), HEX); Serial.println(msg);
    }
  }
}
void sendToBot(uint8_t* payload, long transmissionInterval) {
  if (millis() - lastTransmission > transmissionInterval) {
    Tx16Request tx = Tx16Request(BOT_XBEE_16ADDR, payload, sizeof(payload));
    xbee.send(tx);
    Serial.println("Sent to Bot");
    lastTransmission = millis();
    /*
    xbee.getResponse(response);
    if (response.isAvailable() && response.getApiId() == TX_STATUS_RESPONSE) {
      response.getZBTxStatusResponse(txStatus);
      if (txStatus.getStatus() == SUCCESS && debug > 2) Serial.println("SUCCESS");
      else if (txStatus.getStatus() != SUCCESS && debug > 0) Serial.println("TX ERROR");
    }
    */
  }
}
//========================================  LCD HELPERS =======================================
String bufferSpaces(String message) {
  int len = LCD_BUFFER - message.length();
  if (len < 0) return message.substring(0, LCD_BUFFER - 1);
  String msg = message;
  for (int i = 0; i < len; i++) msg += " ";
  return msg;
}
void refreshLCD(String line0, String line1, bool clearLCD, long refreshInterval) {
  if (millis() - lastLCDRefresh > refreshInterval) { 
    if (clearLCD) lcd.clear();
    // set the cursor to column 0, line 0 (note: line 1 is the second row, since counting begins with 0):
    lcd.setCursor(0,0);
    lcd.print(bufferSpaces(line0));
    lcd.setCursor(0,1);
    lcd.print(bufferSpaces(line1));
    lastLCDRefresh = millis();
  }
}
//======================================== SETUP() ============================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  xbeeSerial.begin(SERIAL_BAUD);
  xbee.setSerial(xbeeSerial);
  xbee.begin(xbeeSerial);
  setupJoystickShield();
  lcd.begin(20, 4);
  lcd.setBacklight(128);  // LOW || HIGH || uint8_t
  lcd.autoscroll();
  if (debug > 0) refreshLCD("setup() complete, 2s delay", "", true, 0);
  delay(2000);
}

//======================================== LOOP() ============================================
void loop() {
  if (debug > 4) Serial.println(readInputState());
  int xV = map(analogRead(PIN_ANALOG_X), 0, 1024, 0, 255);
  int yV = map(analogRead(PIN_ANALOG_Y), 0, 1024, 0, 255); 
  msg = "X:" + String(xV) + "; Y:" + String(yV);
  
  uint8_t payload[2] = {xV, yV};
  sendToBot(payload, 200);

  uint8_t data[MAX_DATA_PAYLOAD]; 
  long dataLength = 0;
  receiveFromBot(data, &dataLength);
  if (dataLength > 0) {
    msg1 = "";
    for (int i = 0; i < dataLength - 1; i++) {
      char c = data[i];
      msg1 += c;
    } 
    Serial.println(msg);
    Serial.println(msg1);
    refreshLCD(msg, msg1, true, 200);
  }
  
}

