//Written by Nick, YO - 2016
//For Watchdog-Breakout-Board-V1.0.0 (should also work on the Daughter-Board-
//V2.0.0)

//PIN STUFF
//////////////////////////////////////////////////////////////////////////////
//This is how the 'Arduino Leonardo' pins map to the IDC connector of the
//that connects the 'Watchdog' on the daughter board to the deck board.
//            PIN 1 =    -     5V0
//            PIN 2 =    -     5V0
const uint8_t PIN_3 =    9;  //RGB Red Channel
const uint8_t PIN_4 =    2;  //HW I2C SCL
const uint8_t PIN_5 =   10;  //RGB Green Channel
const uint8_t PIN_6 =    3;  //HW I2C SDA
const uint8_t PIN_7 =   11;  //RGB Blue Channel
const uint8_t PIN_8 =    1;  //UART1 TX
const uint8_t PIN_9 =    4;  //Aux Enable (12V)
const uint8_t PIN_10 =   0;  //UART1 RX
const uint8_t PIN_11 =   7;  //Motors Enable
const uint8_t PIN_12 =  A5;  //Currently unused
const uint8_t PIN_13 =   8;  //Aux 5 Volts Enable
const uint8_t PIN_14 =  A4;  //Currently unused
const uint8_t PIN_15 =  A0;  //Jack voltage (4:1 voltage divider)
const uint8_t PIN_16 =  A3;  //Currently unused
const uint8_t PIN_17 =  A1;  //Battery voltage (4:1 voltage divider)
const uint8_t PIN_18 =  A2;  //Currently unused
//            PIN 19 =   -     GND
//            PIN 20 =   -     GND

//Timer
//////////////////////////////////////////////////////////////////////////////
#include <Timer.h>

Timer oneSecondTimer(1000); 

//STATUS LED STUFF
//////////////////////////////////////////////////////////////////////////////
Timer statusLEDTimer(250);
const uint8_t STATUS_LED_PIN = 13;

uint8_t statusLEDState = LOW;

//RGB STUFF
//////////////////////////////////////////////////////////////////////////////
const uint8_t LED_R_PIN = PIN_3;
const uint8_t LED_G_PIN = PIN_5;
const uint8_t LED_B_PIN = PIN_7;

uint8_t redVal   = 0;
uint8_t greenVal = 0;
uint8_t blueVal  = 0;

//ENABLE STUFF
//////////////////////////////////////////////////////////////////////////////
const uint8_t AUX_EN_PIN       = PIN_9;
const uint8_t MOTORS_EN_PIN    = PIN_11;
const uint8_t FIVE_VOLT_EN_PIN = PIN_13;

uint8_t auxEnState      = LOW;
uint8_t motorsEnState   = LOW;
uint8_t fiveVoltEnState = LOW;

//VOLTAGE MEASUREMENT STUFF
//////////////////////////////////////////////////////////////////////////////
const uint8_t JACK_V_PIN = PIN_15;
const uint8_t BATT_V_PIN = PIN_17;

const float JACK_TRIM = 1.1089; //Through experimentation, hopefully the
const float BATT_TRIM = 1.0540; //op-amped version of the deckboard solves
                                //this
float jackVolts = -0.0;
float battVolts = -0.0;




//////////////////////////////////////////////////////////////////////////////
//SETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPSETUPS
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void setup(){
  setupStatusLED();
  setupRGB();
  setupEnablePins();
  setupVoltageMeasurement();
}

//////////////////////////////////////////////////////////////////////////////
//LOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOPLOOP
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
void loop(){
  blinkStatusLED();
  if(oneSecondTimer.trigger())flipFiveVoltEn();
}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//STATUS LED STUFF
//////////////////////////////////////////////////////////////////////////////
void setupStatusLED(){
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);
}

void blinkStatusLED(){
  if(statusLEDTimer.trigger())flipStatusLED();
}

void setStatusLED(int state){
    statusLEDState = state;
    digitalWrite(STATUS_LED_PIN, statusLEDState);
}

void setStatusLEDHigh(){
  setStatusLED(HIGH);
}
void setStatusLEDLow(){
  setStatusLED(LOW);
}
void flipStatusLED(){
  setStatusLED(!statusLEDState);
}

//RGB STUFF
//////////////////////////////////////////////////////////////////////////////
void setupRGB(){
  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);
}

void setRGB(uint8_t rVal, uint8_t gVal, uint8_t bVal){
  setRed(rVal); setGreen(gVal); setBlue(bVal);
}

void setRed(uint8_t val){
  redVal = val;
  analogWrite(LED_R_PIN, redVal);
}
void setGreen(uint8_t val){
  greenVal = val;
  analogWrite(LED_G_PIN, greenVal);
}
void setBlue(uint8_t val){
  blueVal = val;
  analogWrite(LED_B_PIN, blueVal);
}

void setRedOn(){
  redVal = HIGH;
  digitalWrite(LED_R_PIN, redVal);
}
void setRedOff(){
  redVal = LOW;
  digitalWrite(LED_R_PIN, redVal);
}
void flipRed(){
  redVal = !redVal;
  digitalWrite(LED_R_PIN, redVal);
}

void setGreenOn(){
  greenVal = HIGH;
  digitalWrite(LED_G_PIN, greenVal);
}
void setGreenOff(){
  greenVal = LOW;
  digitalWrite(LED_G_PIN, greenVal);
}
void flipGreen(){
  greenVal = !greenVal;
  digitalWrite(LED_G_PIN, greenVal);
}

void setBlueOn(){
  blueVal = HIGH;
  digitalWrite(LED_B_PIN, blueVal);
}
void setBlueOff(){
  redVal = LOW;
  digitalWrite(LED_B_PIN, blueVal);
}
void flipBlue(){
  blueVal = !blueVal;
  digitalWrite(LED_B_PIN, blueVal);
}
//ENABLE STUFF
//////////////////////////////////////////////////////////////////////////////
void setupEnablePins(){
  pinMode(AUX_EN_PIN, OUTPUT);
  pinMode(MOTORS_EN_PIN, OUTPUT);
  pinMode(FIVE_VOLT_EN_PIN, OUTPUT);

  digitalWrite(AUX_EN_PIN, LOW);
  digitalWrite(MOTORS_EN_PIN, LOW);
  digitalWrite(FIVE_VOLT_EN_PIN, LOW);
}

void setAuxEn(uint8_t state){
  auxEnState = state;
  digitalWrite(AUX_EN_PIN, auxEnState);
}

void flipAuxEn(){
    setAuxEn(!auxEnState);
}

void setAuxEnHigh(){
    setAuxEn(HIGH);
}

void setAuxEnLow(){
    setAuxEn(LOW);
}

void setMotorsEn(uint8_t state){
  motorsEnState = state;
  digitalWrite(MOTORS_EN_PIN, motorsEnState);
}

void flipMotorsEn(){
    setMotorsEn(!motorsEnState);
}

void setMotorsEnHigh(){
    setMotorsEn(HIGH);
}

void setMotorsEnLow(){
    setMotorsEn(LOW);
}

void setFiveVoltEn(uint8_t state){
  fiveVoltEnState = state;
  digitalWrite(FIVE_VOLT_EN_PIN, fiveVoltEnState);
}

void flipFiveVoltEn(){
    setFiveVoltEn(!fiveVoltEnState);
}

void setFiveVoltEnHigh(){
    setFiveVoltEn(HIGH);
}

void setFiveVoltEnLow(){
    setFiveVoltEn(LOW);
}

//VOLTAGE MEASUREMENT STUFF
//////////////////////////////////////////////////////////////////////////////
void setupVoltageMeasurement(){
  pinMode(JACK_V_PIN, INPUT);
  pinMode(BATT_V_PIN, INPUT);
}

void readVolts(){
  readJackVolts();
  readBattVolts();
}

void readJackVolts(){
  jackVolts = analogRead(JACK_V_PIN) * 0.01953125 * JACK_TRIM; 
}

void readBattVolts(){
  battVolts = analogRead(BATT_V_PIN) * 0.01953125 * BATT_TRIM;
}

void printVolts(){
  printJackVolts();
  printBattVolts();
}

void printJackVolts(){
  Serial.println(jackVolts);
}

void printBattVolts(){
  Serial.println(battVolts);
}

void printHRJackVolts(){
  Serial.print("jack = ");
  Serial.print(jackVolts);
  Serial.println(" v");
}

void printHRBattVolts(){
  Serial.print("batt = ");
  Serial.print(battVolts);
  Serial.println(" v");
}


