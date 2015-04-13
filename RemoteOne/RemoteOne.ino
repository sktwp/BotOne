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
#define SERIAL_BAUD 57600
#define BOT_XBEE_16ADDR 0x0002

//======================================== GLOBALS ============================================
// XBee's DOUT (TX) is connected to pin 2 (Arduino's Software RX)
// XBee's DIN  (RX) is connected to pin 3 (Arduino's Software TX)
const uint8_t ssRX = 2; //Soft Serial RX Pin
const uint8_t ssTX = 3; //SoftSerial TX Pin

//Joystick direction:
const byte PIN_ANALOG_X = 0;
const byte PIN_ANALOG_Y = 1;

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
LiquidTWI lcd(0);
String msg = "";

//======================================== XBEE HELPERS =======================================
void sendToBot(uint8_t* payload) {
  Tx16Request tx = Tx16Request(BOT_XBEE_16ADDR, payload, sizeof(payload));
  TxStatusResponse txStatus = TxStatusResponse();
  xbee.send(tx);
  if (xbee.readPacket(200)) {
    if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE) {
      xbee.getResponse().getZBTxStatusResponse(txStatus);
      if (txStatus.getStatus() == SUCCESS) Serial.println("SUCCESS");
      else Serial.println("TX ERROR");
    }
  } else if (xbee.getResponse().isError()) {
    Serial.print("Pkt read err: "); Serial.println(xbee.getResponse().getErrorCode());
  }
}
//======================================== SETUP() ============================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  xbeeSerial.begin(SERIAL_BAUD);
  xbee.setSerial(xbeeSerial);
  setupJoystickShield();
  lcd.begin(20, 4);
  lcd.setBacklight(LOW);  // LOW || HIGH || uint8_t
  lcd.autoscroll();
  lcd.clear();
  // set the cursor to column 0, line 1 (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 1);
  if (debug > 0) lcd.print("setup() complete");
  delay(500);
}

//======================================== LOOP() ============================================
void loop() {
  if (debug > 1) Serial.println(readInputState());
  int xV = map(analogRead(PIN_ANALOG_X), 0, 1024, 0, 255);
  int yV = map(analogRead(PIN_ANALOG_Y), 0, 1024, 0, 255);
  uint8_t payload[2] = {xV, yV};
  sendToBot(payload);
  if (xbeeSerial.available()) lcd.print(xbeeSerial.read());
  delay (200);
}

