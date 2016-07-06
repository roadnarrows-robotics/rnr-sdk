////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Firmware:  Arduino Compatible Firmware, v3.
//
// File:      laeWatchDogV3.ino
//
/*! \file
 *
 * $LastChangedDate$
 * $Rev$
 *
 * \brief Laelaps Arduino compatible watchdog firmware.
 *
 * * Board: Arduino Leonardo
 * * CPU:   ATmega32u4 5V 16MHz
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Nick Anderson (nick@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2016  RoadNarrows
 * (http://www.RoadNarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

//
// Firmware defines
//
#define LAE_ARDUINO       1 ///< arduino target 
#define LAE_WD_FW_VERSION 3 ///< firmware version

#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <Wire.h>

//
// Bridge between standard POSIX C/C++ Linux and Arduino constructs.
//
typedef byte byte_t;

// Interface between firmware and software.
#include "laeWatchDog.h"

using namespace laelaps;

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Pin Mapping and Conversions
//
// Arduino Leonardo physical pin to digital/analog number map.
//  PIN 1  =   -    5V0
//  PIN 2  =   -    5V0
//  PIN_3  =   9    RGB Red Channel
//  PIN_4  =   2    HW I2C SCL
//  PIN_5  =  10    RGB Green Channel
//  PIN_6  =   3    HW I2C SDA
//  PIN_7  =  11    RGB Blue Channel
//  PIN_8  =   1    UART1 TX
//  PIN_9  =   4    Aux Enable (12V)
//  PIN_10 =   0    UART1 RX
//  PIN_11 =   7    Motors Enable
//  PIN_12 =  A5    Unused
//  PIN_13 =   8    Aux 5 Volts Enable
//  PIN_14 =  A4    Unused
//  PIN_15 =  A0    Jack voltage (4:1 voltage divider)
//  PIN_16 =  A3    Unused
//  PIN_17 =  A1    Battery voltage (4:1 voltage divider)
//  PIN_18 =  A2    Unused
//  PIN 19 =   -    GND
//  PIN 20 =   -    GND
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

//
// Digital and Analog I/O
//
const int MAX_D_PINS = 14;  ///< maximum number of digital pins 0 - 13

//
// LEDs
//
const int RED           = 0;    ///< red channel index
const int GREEN         = 1;    ///< green channel index
const int BLUE          = 2;    ///< blue channel index
const int NUM_CHANS     = 3;    ///< number of channels
const int MAX_LED_TRANS = 3;    ///< maximum LED transitions per pattern
const int PIN_PWM_RGB[NUM_CHANS] = {9, 10, 11}; ///< RGB LED PWM pins

// Enable/Disable Hardware Pins
const int PIN_D_EN_MOTOR_CTLRS    = 7;  ///< motor controllers power enable
const int PIN_D_EN_AUX_PORT_5V    = 8;  ///< aux. port 5 volt power out enable
const int PIN_D_EN_AUX_PORT_BATT  = 4;  ///< aux. port battery power out enable

// User Analog Input Pins
const int PIN_A_USER_MIN  = A0;   ///< minimum user available pin number
const int PIN_A_USER_MAX  = A3;   ///< maximum user available pin number

//
// Battery Charging Input Pins and values
// Through experimentation, the op-amp version of the deckboard input voltage
// biases and trims are part-to-part stable.
//
const int   PIN_A_JACK_V  = A0;       ///< jack voltage into battery charging
const int   PIN_A_BATT_V  = A1;       ///< battery output voltage
const float ADC_V_PER_BIT = 0.01955;  ///< v/bit = 0.0V - 20.0V in 1023 values
const float JACK_V_MIN    = 13.9;     ///< required minimum voltage to charge
const float JACK_V_BIAS   = 0.7;      ///< jack bias
const float JACK_V_TRIM   = 1.1089;   ///< jack trim
const float BATT_V_BIAS   = 0.7;      ///< battery bias
//const float BATT_V_TRIM   = 1.0540;   ///< battery trim
const float BATT_V_TRIM   = 0.95385;  ///< battery trim

#if 0 // DEPRECATED
// Digital Pin Shadow State
int DPinDir[MAX_D_PINS];          ///< direction 
int DPinVal[MAX_D_PINS];          ///< value
#endif // DEPRECATED


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// State
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Operational states in order of highest(0) to lowest priority.
 */
enum OpState
{
  OpStateNoService,   ///< no service
  OpStateAlarmed,     ///< robot is alarmed
  OpStateNominal      ///< robot operating nominally
};

//
// Current State
//
int           CurOpState;         ///< robot operational state
unsigned long CurTimeout;         ///< watchdog timeout (msec)
float         CurJackV;           ///< current sensed jack input voltage
float         CurBattV;           ///< current sensed battery output voltage
boolean       CurBattIsCharging;  ///< battery is [not] currently being charged
unsigned int  CurBattSoC;         ///< battery state of charge 0% - 100%
unsigned int  CurAlarms;          ///< current alarms
byte          CurLed[NUM_CHANS];  ///< current RGB LED values
int           CurLedPatIdx;       ///< active LED pattern index
boolean       CurUserOverride;    ///< user has [no] RGB LED pattern override
int           CurSeqNum;          ///< rsp sequence number (for some commands)

//
// Pending state modifiers.
//
boolean       PleasePetTheDog;    ///< do [not] pet the dog

//
// Timers
//
unsigned long Tcur;         ///< current up time (msec)
unsigned long Twd;          ///< last time of watchdog pet
unsigned long Tled;         ///< last time of LED pattern activation/transition


/*!
 * \brief LED pattern indices in order of highest(0) to lowest priority.
 */
const int LedPatIdxNoService  = 0;    ///< no service pattern
const int LedPatIdxAlarmCrit  = 1;    ///< critical alarm pattern
const int LedPatIdxAlarm      = 2;    ///< alarm pattern
const int LedPatIdxBattCrit   = 3;    ///< battery critical pattern
const int LedPatIdxUser       = 4;    ///< user pattern
const int LedPatIdxBatt       = 5;    ///< battery charge pattern
const int LedPatIdxNumOf      = 6;    ///< number of patterns

/*!
 * \brief LED pattern structure.
 */
struct LedPattern_T
{
  byte          curPos;                       ///< current transition position
  byte          numPos;                       ///< number of transition pos
  unsigned long msec[MAX_LED_TRANS];          ///< pattern durations (millisecs)
  byte          rgb[MAX_LED_TRANS][NUM_CHANS]; ///< RGB patterns
};

/*!
 * \brief Supported LED patterns.
 */
LedPattern_T LedPat[LedPatIdxNumOf] =
{
  // no service pattern - fast flashing green [LedPatIdxNoService]
  { 0, 2, {100, 100, 0}, {{0, 255, 0}, {0, 0, 0}, {0, 0, 0}} },

  // critical alarm pattern - solid red [LedPatIdxAlarmCrit]
  { 0, 1, {0, 0, 0}, { {255, 0, 0}, {0, 0, 0}, {0, 0, 0}} },

  // alarm pattern - slow flashing red [LedPatIdxAlarm]
  { 0, 2, {1000, 1000, 0}, {{255, 0, 0}, {0, 0, 0}, {0, 0, 0}} },

  // battery critical pattern - slow flashing amber [LedPatIdxBattCrit]
  { 0, 2, {1000, 1000, 0}, { {0xff, 0x73, 0}, {0, 0, 0}, {0, 0, 0}} },
 
  // user pattern - user defined solid [LedPatIdxUser]
  { 0, 1, {0, 0, 0}, { {0, 0, 0}, {0, 0, 0}, {0, 0, 0}} },

  // battery charge pattern - white to dark amber (calculated) [LedPatIdxBatt]
  { 0, 1, {0, 0, 0}, { {0, 0, 0}, {0, 0, 0}, {0, 0, 0}} }
};

//
// I2C slave data
//
byte        I2CRspBuf[LaeWdMaxRspLen];  ///< response buffer
int         I2CRspLen;                  ///< response length

//
// Serial CLI
//
char      SerLine[LaeWdSerMaxCmdLen+1];
int       SerLinePos;
char      SerArgv[LaeWdSerMaxCmdArgc][LaeWdSerMaxCmdArgLen];
int       SerArgc;
const int SerCmdIdx = 0;

//
// Forward declarations
//
void updateInServiceState(boolean bForceUpdate=false);


//------------------------------------------------------------------------------
// Arduino Hook Functions
//------------------------------------------------------------------------------

/*!
 * \brief Arduino Sketch setup() hook.
 *
 * Called once on power-up.
 */
void setup()
{
  byte  pin;
  byte  dir;
  byte  val;

  //
  // State
  //
  CurOpState        = OpStateNoService;
  CurTimeout        = LaeWdTimeoutDft;
  CurJackV          = 0.0;
  CurBattV          = 0.0;
  CurBattIsCharging = false;
  CurBattSoC        = LaeWdArgBattSoCMax;
  CurAlarms         = LaeWdArgAlarmNone;
  CurLed[RED]       = 0;
  CurLed[GREEN]     = 0;
  CurLed[BLUE]      = 0;
  CurLedPatIdx      = LedPatIdxNoService;
  CurUserOverride   = false;
  CurSeqNum         = 0;

  // State modifiers
  PleasePetTheDog   = false;

  //
  // Configure digital pins defaults.
  //
  // Arduino digital pin default to input. Inputs reduce power consumption over
  // outputs. However, without connected circuitry, input are very noisy.
  //
  for(pin = 0; pin <= MAX_D_PINS; ++pin)
  {
    switch( pin )
    {
      case PIN_D_EN_MOTOR_CTLRS:
        dir = OUTPUT;
        val = LOW;
        break;
      case PIN_D_EN_AUX_PORT_BATT:
      case PIN_D_EN_AUX_PORT_5V:
        dir = OUTPUT;
        val = HIGH;
        break;
      default:
        dir = INPUT;
        val = LOW;
        break;
    }

    if( dir == OUTPUT )
    {
      pinMode(pin, dir);
      digitalWrite(pin, val);
    }

#if 0 // DEPRECATED
    DPinDir[pin] = dir;
    DPinVal[pin] = val;
#endif // DEPRECATED
  }

#if 0 // DEPRECATED
  // configure analog input pins
  for(pin = PIN_A_USER_MIN; pin <= PIN_A_USER_MIN; ++pin)
  {
    digitalWrite(pin, LOW);   // no pull-ups
  }
#endif // DEPRECATED

  //
  // I2C slave setup. Receive and send from/to I2C master by asynchronous
  // callbacks.
  //
  I2CRspLen = 0;
  Wire.begin(LaeI2CAddrArduino);
  Wire.onReceive(i2cReceiveCmd);
  Wire.onRequest(i2cSendRsp);
  
  //
  // Serial command-line ASCII interface (baud does not matter over USB) 
  // 
  SerLinePos  = 0;
  SerArgc     = 0;
  Serial.begin(115200);

  //
  // Timers and timeout periods
  //
  Tcur  = millis();
  Twd   = Tcur;
  Tled  = Tcur;
}

/*!
 * \brief Arduino Sketch loop() hook.
 *
 * Called each time through Sketch main loop.
 */
void loop()
{
  // current up time
  Tcur = millis();

  // watchdog timed out
  if( (CurOpState != OpStateNoService) && (dt(Tcur, Twd) > CurTimeout) )
  {
    enterNoServiceState();
  }

  //
  // Hooks for any special state processing.
  //
  switch( CurOpState )
  {
    case OpStateNoService:   // no service
    case OpStateAlarmed:     // robot is alarmed
    case OpStateNominal:     // robot operating nominally
    default:
      break;
  }

  // read voltages
  readVoltages();

  // update current LED pattern transition
  updateLedPattern();

  //
  // Process any serial command input.
  //
  if( serRcvCmd() )
  {
    if( serParseCmd() )
    {
      serExecCmd();
    }
  }

  //
  // Update in-service state data.
  //
  if( PleasePetTheDog )
  {
    updateInServiceState();
    Twd = millis();
    PleasePetTheDog = false;
  }

  delay(2);
}


//------------------------------------------------------------------------------
// Core Functions and Utilities
//------------------------------------------------------------------------------

/*!
 * \brief Elapse delta time.
 *
 * \param t1  Most recent time (e.g. current time).
 * \param t0  Some earlier, previous time.
 *
 * \return Milliseconds.
 */
unsigned long dt(unsigned long t1, unsigned long t0)
{
  // time t1 is later than t0
  if( t1 >= t0 )
  {
    return t1 - t0;
  }
  // must haved wrapped - happens about every 50 days of up time.
  else
  {
    return 4294967295 - t0 + t1;
  }
}

#if 0 // DEPRECATED
/*!
 * \brief Map user pin number to analog input pin.
 * 
 * \param pin   User pin number.
 *
 * \return Hardware analog input pin number.
 */
byte mapToAn(byte pin)
{
  return A0 + (pin - LaeWdArgAInPinNumMin);
}
#endif // DEPRECATED

/*
 * \brief Enter no-service operational state. 
 *
 * Some state information is reset here.
 */
void enterNoServiceState()
{
  CurOpState      = OpStateNoService;
  CurBattSoC      = LaeWdArgBattSoCMax;
  CurAlarms       = LaeWdArgAlarmNone;
  CurLedPatIdx    = LedPatIdxNoService;
  CurUserOverride = false;

  PleasePetTheDog = false;

  digitalWrite(PIN_D_EN_MOTOR_CTLRS, LOW);

#if 0 // DEPRECATED
  DPinVal[PIN_D_EN_MOTOR_CTLRS] = LOW;
#endif // DEPRECATED

  updateBatteryRgb(CurBattSoC);
  activateLedPattern(CurLedPatIdx);
}

/*!
 * \brief Update change operational state data. 
 *
 * The new state is determined from the current operational state, battery
 * state of charge, alarms, and user overrides.
 */
void updateInServiceState(boolean bForceUpdate)
{
  int newOpState    = CurOpState;
  int newLedPatIdx  = CurLedPatIdx;

  // robot is critically alarmed
  if( CurAlarms & LaeWdArgAlarmCrit )
  {
    newOpState   = OpStateAlarmed;
    newLedPatIdx = LedPatIdxAlarmCrit;
  }

  // battery is critically alarmed
  else if( CurAlarms & LaeWdArgAlarmBattCrit )
  {
    newOpState   = OpStateAlarmed;
    newLedPatIdx = LedPatIdxBattCrit;
  }

  // robot is alarmed
  else if( CurAlarms & LaeWdArgAlarmTypeMask )
  {
    newOpState   = OpStateAlarmed;
    newLedPatIdx = LedPatIdxAlarm;
  }

  // robot is nominal, but user has overridden the RGB LED
  else if( CurUserOverride )
  {
    newOpState   = OpStateNominal;
    newLedPatIdx = LedPatIdxUser;
  }

  // robot is nominal
  else
  {
    newOpState   = OpStateNominal;
    newLedPatIdx = LedPatIdxBatt;
    updateBatteryRgb(CurBattSoC);
  }

  // new LED pattern
  if( (newOpState != CurOpState) ||
      (newLedPatIdx != CurLedPatIdx) ||
      bForceUpdate )
  {
    CurOpState      = newOpState;
    activateLedPattern(newLedPatIdx);
  }
}

/*!
 * \brief Update battery charge state.
 */
void updateBatteryRgb(unsigned int batt_soc)
{
  float h, s, v;

  if( batt_soc >= 95 )
  {
    LedPat[LedPatIdxBatt].rgb[0][RED]   = LaeWdArgRgbLedMax;
    LedPat[LedPatIdxBatt].rgb[0][GREEN] = LaeWdArgRgbLedMax;
    LedPat[LedPatIdxBatt].rgb[0][BLUE]  = LaeWdArgRgbLedMax;
  }
  else if( batt_soc <= 5 )
  {
    LedPat[LedPatIdxBatt].rgb[0][RED]   = 0x7b;
    LedPat[LedPatIdxBatt].rgb[0][GREEN] = 0x4a;
    LedPat[LedPatIdxBatt].rgb[0][BLUE]  = 0x03;
  }
  else
  {
    h = 35.0;
    s = (float)(LaeWdArgBattSoCMax - batt_soc);
    v = (float)(LaeWdArgBattSoCMax + batt_soc) / 2.0;

    // white to amber
    HSVtoRGB(h, s, v, LedPat[LedPatIdxBatt].rgb[0]);
  }
}

/*!
 * \brief Active new LED pattern.
 *
 * \param newLedPatIdx  New LED pattern index.
 */
void activateLedPattern(int newLedPatIdx)
{
  CurLedPatIdx = newLedPatIdx;
  LedPat[CurLedPatIdx].curPos = 0;
  Tled = millis();

  lightLed();
}

/*!
 * \brief Update LED pattern state.
 */
void updateLedPattern()
{
  byte          numPos;   // number of different colors in pattern
  byte          curPos;   // current light switch position

  numPos = LedPat[CurLedPatIdx].numPos;
  curPos = LedPat[CurLedPatIdx].curPos;

  // only one fixed illumination transition
  if( numPos <= 1 )
  {
    return;
  }

  // the current illumination has not timed out
  else if( dt(Tcur, Tled) < LedPat[CurLedPatIdx].msec[curPos] )
  {
    return;
  }

  // new illumination transition
  LedPat[CurLedPatIdx].curPos = (curPos + 1) % numPos;
  Tled = millis();

  lightLed();
}

/*!
 * \brief Light the RGB LED.
 */
void lightLed()
{
  byte    curPos;   // current light switch position
  int     chan;     // RBG channel

  curPos = LedPat[CurLedPatIdx].curPos;

  for(chan = 0; chan < NUM_CHANS; ++chan)
  {
    CurLed[chan] = LedPat[CurLedPatIdx].rgb[curPos][chan];
    analogWrite(PIN_PWM_RGB[chan], CurLed[chan]);
  }
}

/*!
 * \brief Convert color in HSV space to equivalent in RGB space.
 *
 * \param h           Hue [0.0, 360.0).
 * \param s           Saturation [0.0, 100.0].
 * \param v           Value [0.0, 100.0].
 * \param [out] rgb   Array of three bytes holding the RGB values [0, 255].
 */
void HSVtoRGB(float h, float s, float v, byte rgb[])
{
  int   sector;
  float f, p, q, t;
  float r, g, b;

  // normalize
  s /= 100.0;
  v /= 100.0;

  //
  // cap
  if( h >= 360.0 )
  {
    h = 359.9;
  }
  if( s > 1.0 )
  {
    s = 1.0;
  }
  if( v > 1.0 )
  {
    v = 1.0;
  }

  // gray
  if( s <= 0.0 )
  {
    rgb[RED]   = (byte)(v * 255.0);
    rgb[GREEN] = (byte)(v * 255.0);
    rgb[BLUE]  = (byte)(v * 255.0);
    return;
  }

  h /= 60.0;                // hue sector
  sector = (int)h;          // sector
  f = h - (float)sector;    // fraction in sector
  p = v * (1.0 - s); 
  q = v * (1.0 - s * f);
  t = v * (1.0 - s * (1.0 - f));

  //
  // RGB' [0.0, 1.0]
  //
  switch( sector )
  {
    case 0:
      r = v; g = t; b = p;
      break;
    case 1:
      r = q; g = v; b = p;
      break;
    case 2:
      r = p; g = v; b = t;
      break;
    case 3:
      r = p; g = q; b = v;
      break;
    case 4:
      r = t; g = p; b = v;
      break;
    case 5:
    default:
      r = v; g = p; b = q;
      break;
  }

  // conversion
  rgb[RED]   = (byte)(r * 255.0);
  rgb[GREEN] = (byte)(g * 255.0);
  rgb[BLUE]  = (byte)(b * 255.0);
}

/*!
 * \brief Read all monitored voltages.
 */
void readVoltages()
{
  int   sensorval;

  // read jack voltage input to battery charging circuitry
  sensorval = analogRead(PIN_A_JACK_V);
  CurJackV  = ((float)sensorval * ADC_V_PER_BIT - JACK_V_BIAS) * JACK_V_TRIM;

  if( CurJackV >= JACK_V_MIN )
  {
    CurBattIsCharging = true;
  }
  else
  {
    CurBattIsCharging = false;
  }

  // read battery output voltage
  sensorval = analogRead(PIN_A_BATT_V);
  CurBattV  = ((float)sensorval * ADC_V_PER_BIT - BATT_V_BIAS) * BATT_V_TRIM;
}


//------------------------------------------------------------------------------
// Slave Interface to Host
//------------------------------------------------------------------------------

/*!
 * \brief Arduino Sketch I2C receive callback.
 *
 * Receive and process command. The commands and responses are over the I2C bus.
 *
 * This function is called asynchronously by the Arduino I2C slave framework.
 */
void i2cReceiveCmd(int n)
{
  byte    cmdId;
  boolean bOk;

  if( Wire.available() > 0 )
  {
    cmdId     = Wire.read();
    I2CRspLen = 0;

    switch( cmdId )
    {
      case LaeWdCmdIdPetDog:
        bOk = i2cExecPetDog();
        break;
      case LaeWdCmdIdGetVersion:
        bOk = i2cExecGetVersion();
        break;
      case LaeWdCmdIdSetBattCharge:
        bOk = i2cExecSetBattCharge();
        break;
      case LaeWdCmdIdSetAlarms:
        bOk = i2cExecSetAlarms();
        break;
      case LaeWdCmdIdSetRgbLed:
        bOk = i2cExecSetRgbLed();
        break;
      case LaeWdCmdIdResetRgbLed:
        bOk = i2cExecResetRgbLed();
        break;

#if 0 // DEPRECATED
      case LaeWdCmdIdConfigDPin:
        bOk = i2cExecConfigDPin();
        break;
      case LaeWdCmdIdReadDPin:
        bOk = i2cExecReadDPin();
        break;
      case LaeWdCmdIdWriteDPin:
        bOk = i2cExecWriteDPin();
        break;
      case LaeWdCmdIdReadAPin:
        bOk = i2cExecReadAPin();
        break;
      case LaeWdCmdIdWriteAPin:
        bOk = i2cExecWriteAPin();
        break;
#endif // DEPRECATED

      case LaeWdCmdIdEnableMotorCtlrs:
        bOk = i2cExecEnableMotorCtlrs();
        break;
      case LaeWdCmdIdEnableAuxPort:
        bOk = i2cExecEnableAuxPort();
        break;
      case LaeWdCmdIdReadEnables:
        bOk = i2cExecReadEnables();
        break;
      case LaeWdCmdIdReadVolts:
        bOk = i2cExecReadVolts();
        break;
#if 0 // DEPRECATED
      case LaeWdCmdIdTest:
        bOk = i2cExecTest();
        break;
#endif // DEPRECATED
      case LaeWdCmdIdConfigFw:
        bOk = i2cExecConfigFw();
        break;
      default:
        bOk = false;
        break;
    }
  }

  if( !bOk )
  {
    i2cFlushRead();
  }

  // mollify the dog if received a good command
  PleasePetTheDog = bOk;
}

/*!
 * \brief Send any response loaded in response buffer.
 */
void i2cSendRsp()
{
  if( I2CRspLen > 0 )
  {
    Wire.write(I2CRspBuf, I2CRspLen);
    I2CRspLen = 0;
  }
}

/*!
 * \brief Error response for bad commands the expect data back.
 *
 * \param n   Length of expected response.
 */
void i2cErrorRsp(int n)
{
  byte  b;

  // flush input 
  i2cFlushRead();

  // fill response buffer with recognizable error pattern ABC...
  for(b = 'A'; I2CRspLen < n; ++I2CRspLen, ++b)
  {
    I2CRspBuf[I2CRspLen] = b;
  }
}

/*!
 * \brief Flush the I2C input buffer.
 */
void i2cFlushRead()
{
  byte  d;

  while( Wire.available() > 0 )
  {
    d = Wire.read();
  }
}

/*!
 * \brief Execute I2C command to pet the watchdog.
 *
 * \return Returns true on success, false on failure.
 */
boolean i2cExecPetDog()
{
  I2CRspBuf[I2CRspLen++] = CurBattIsCharging?
                              LaeWdArgDPinValHigh: LaeWdArgDPinValLow;
  return true;
}

/*!
 * \brief Execute I2C command to get the firmware's version number.
 *
 * \return Returns true on success, false on failure.
 */
boolean i2cExecGetVersion()
{
  I2CRspBuf[I2CRspLen++] = (byte)LAE_WD_FW_VERSION;
  return true;
}

/*!
 * \brief Execute I2C command to set the battery charge state.
 *
 * \return Returns true on success, false on failure.
 */
boolean i2cExecSetBattCharge()
{
  byte          len = LaeWdCmdLenSetBattCharge - 1; 
  unsigned int  batt_soc;

  if( Wire.available() == len )
  {
    batt_soc = (unsigned int)Wire.read();
    if( batt_soc > LaeWdArgBattSoCMax )
    {
      batt_soc = LaeWdArgBattSoCMax;
    }
    else if( batt_soc < LaeWdArgBattSoCMin )
    {
      batt_soc = LaeWdArgBattSoCMin;
    }

    // new charge state
    if( batt_soc != CurBattSoC )
    {
      CurBattSoC = batt_soc;
      updateBatteryRgb(batt_soc);
      updateInServiceState(true);
    }

    return true;
  }

  // error
  return false;
}

/*!
 * \brief Execute I2C command to set/clear alarm bits.
 *
 * \return Returns true on success, false on failure.
 */
boolean i2cExecSetAlarms()
{
  byte          len = LaeWdCmdLenSetAlarms - 1; 
  unsigned int  val_hi, val_lo;

  if( Wire.available() == len )
  {
    val_hi = Wire.read();
    val_lo = Wire.read();

    CurAlarms = (val_hi << 8) | val_lo;

    return true;
  }

  // error
  return false;
}

/*!
 * \brief Execute I2C command for user set the RGB LED with an override value.
 *
 * \return Returns true on success, false on failure.
 */
boolean i2cExecSetRgbLed()
{
  byte len = LaeWdCmdLenSetRgbLed - 1; 

  if( Wire.available() == len )
  {
    LedPat[LedPatIdxUser].rgb[0][RED]   = Wire.read();
    LedPat[LedPatIdxUser].rgb[0][GREEN] = Wire.read();
    LedPat[LedPatIdxUser].rgb[0][BLUE]  = Wire.read();

    CurUserOverride = true;
    updateInServiceState(true);

    return true;
  }

  // error
  return false;
}

/*!
 * \brief Execute I2C command to clear any user's RGB LED override value.
 *
 * \return Returns true on success, false on failure.
 */
boolean i2cExecResetRgbLed()
{
  byte len = LaeWdCmdLenResetRgbLed - 1; 

  CurUserOverride = false;
  updateInServiceState(true);

  return true;
}

#if 0 // DEPRECATED
/*!
 * \brief Execute I2C command to configure a digital pin.
 *
 * \return Returns true on success, false on failure.
 */
boolean i2cExecConfigDPin()
{
  byte len = LaeWdCmdLenConfigDPin - 1;
  byte pin;
  byte dir;

  if( Wire.available() == len )
  {
    pin = Wire.read();
    dir = Wire.read();

    if( (pin >= LaeWdArgDPinNumWMin) && (pin <= LaeWdArgDPinNumWMax) )
    {
      dir = dir? OUTPUT: INPUT;
      pinMode(pin, dir);
      DPinDir[pin] = dir;
      return true;
    }
  }

  // error
  return false;
}

/*!
 * \brief Execute I2C command to read a value of a digital pin.
 *
 * \return Returns true on success, false on failure.
 */
boolean i2cExecReadDPin()
{
  byte len = LaeWdCmdLenReadDPin - 1;
  byte pin;
  byte val;

  if( Wire.available() == len )
  {
    pin = Wire.read();

    if( (pin >= LaeWdArgDPinNumMin) && (pin <= LaeWdArgDPinNumMax) )
    {
      val = digitalRead(pin) == LOW? LaeWdArgDPinValLow: LaeWdArgDPinValHigh;
      I2CRspBuf[I2CRspLen++] = pin;
      I2CRspBuf[I2CRspLen++] = val;
      DPinVal[pin] = val;
      return true;
    }
  }

  // error
  i2cErrorRsp(LaeWdRspLenReadDPin);

  return false;
}

/*!
 * \brief Execute I2C command to write a value to a digital pin.
 *
 * \return Returns true on success, false on failure.
 */
boolean i2cExecWriteDPin()
{
  byte  len = LaeWdCmdLenWriteDPin - 1;
  byte  pin;
  byte  val;

  if( Wire.available() == len )
  {
    pin = Wire.read();
    val = Wire.read();

    if( (pin >= LaeWdArgDPinNumWMin) && (pin <= LaeWdArgDPinNumWMax) )
    {
      val = val? HIGH: LOW;
      if( DPinDir[pin] == OUTPUT )
      {
        digitalWrite(pin, val);
        DPinVal[pin] = val;
        return true;
      }
    }
  }

  // error
  return false;
}

/*!
 * \brief Execute I2C command to read the value from an analog pin.
 *
 * \return Returns true on success, false on failure.
 */
boolean i2cExecReadAPin()
{
  byte  len = LaeWdCmdLenReadAPin - 1;
  byte  pin;
  byte  apin;
  int   val;
  byte  val_hi, val_lo;

  if( Wire.available() == len )
  {
    pin = Wire.read();

    if( (pin >= LaeWdArgAInPinNumMin) && (pin <= LaeWdArgAInPinNumMax) )
    {
      apin = mapToAn(pin);

      if( (apin >= PIN_A_USER_MIN) && (apin <= PIN_A_USER_MAX) )
      {
        val = analogRead(apin);

        val_hi = (byte)((val>>8) & 0x03);
        val_lo = (byte)(val & 0xff);

        I2CRspBuf[I2CRspLen++] = pin;
        I2CRspBuf[I2CRspLen++] = val_hi;
        I2CRspBuf[I2CRspLen++] = val_lo;

        return true;
      }
    }
  }

  // error
  i2cErrorRsp(LaeWdRspLenReadAPin);

  return false;
}

/*!
 * \brief Execute I2C command to write a value to an analog pin.
 *
 * \return Returns true on success, false on failure.
 */
boolean i2cExecWriteAPin()
{
  byte    len = LaeWdCmdLenWriteAPin - 1;
  byte    pin;
  byte    val;

  if( Wire.available() == len )
  {
    pin = Wire.read();
    val = Wire.read();

    if( (pin >= LaeWdArgAOutPinNumMin) && (pin <= LaeWdArgAOutPinNumMax) )
    {
      if( val > LaeWdArgAOutPinValMax )
      {
        val = LaeWdArgAOutPinValMax;
      }

      if( DPinDir[pin] == OUTPUT )
      {
        analogWrite(pin, val);
        DPinVal[pin] = val;
        return true;
      }
    }
  }

  // error
  return false;
}
#endif // DEPRECATED

/*!
 * \brief Execute I2C command to enable/disable power to motor controllers.
 *
 * \return Returns true on success, false on failure.
 */
boolean i2cExecEnableMotorCtlrs()
{
  byte    len = LaeWdCmdLenEnableMotorCtlrs - 1;
  byte    wval;
  byte    rval;

  if( Wire.available() == len )
  {
    wval = Wire.read();
    wval = wval? HIGH: LOW;

    digitalWrite(PIN_D_EN_MOTOR_CTLRS, wval);

    delayMicroseconds(5);

    rval = digitalRead(PIN_D_EN_MOTOR_CTLRS);

    if( rval == wval )
    {
#if 0 // DEPRECATED
      DPinVal[PIN_D_EN_MOTOR_CTLRS] = wval;
#endif // DEPRECATED
      I2CRspBuf[I2CRspLen++] = LaeWdArgPass;
    }
    else
    {
      I2CRspBuf[I2CRspLen++] = LaeWdArgFail;
    }

    return true;
  }

  // error
  i2cErrorRsp(LaeWdRspLenEnableMotorCtlrs);

  return false;
}

/*!
 * \brief Execute I2C command to enable/disable power to aux port.
 *
 * \return Returns true on success, false on failure.
 */
boolean i2cExecEnableAuxPort()
{
  byte    len = LaeWdCmdLenEnableAuxPort - 1;
  byte    pin;
  byte    val;

  if( Wire.available() == len )
  {
    pin = Wire.read();
    val = Wire.read();

    // remap
    if( pin == LaeWdArgAuxPort5V )
    {
      pin = PIN_D_EN_AUX_PORT_5V;
    }
    else if( pin == LaeWdArgAuxPortBatt )
    {
      pin = PIN_D_EN_AUX_PORT_BATT;
    }
    else
    {
      return false;
    }

    val = val? HIGH: LOW;

    digitalWrite(pin, val);
#if 0 // DEPRECATED
    DPinVal[pin] = val;
#endif // DEPRECATED

    return true;
  }

  // error
  return false;
}

/*!
 * \brief Execute I2C command to read GPIO enable lines.
 *
 * \return Returns true on success, false on failure.
 */
boolean i2cExecReadEnables()
{
  byte  val;

  val = digitalRead(PIN_D_EN_MOTOR_CTLRS);
  I2CRspBuf[I2CRspLen++] = val == LOW? LaeWdArgDPinValLow: LaeWdArgDPinValHigh;

  val = digitalRead(PIN_D_EN_AUX_PORT_5V);
  I2CRspBuf[I2CRspLen++] = val == LOW? LaeWdArgDPinValLow: LaeWdArgDPinValHigh;

  val = digitalRead(PIN_D_EN_AUX_PORT_BATT);
  I2CRspBuf[I2CRspLen++] = val == LOW? LaeWdArgDPinValLow: LaeWdArgDPinValHigh;

  return true;
}

/*!
 * \brief Execute I2C command to read sensed voltages.
 *
 * \return Returns true on success, false on failure.
 */
boolean i2cExecReadVolts()
{
  byte    len = LaeWdCmdLenReadVolts - 1;
  byte    val;

  val = (byte)(CurJackV * LaeWdArgVMult);
  I2CRspBuf[I2CRspLen++] = val;

  val = (byte)(CurBattV * LaeWdArgVMult);
  I2CRspBuf[I2CRspLen++] = val;

  return true;
}

#if 0 // DEPRECATED
/*!
 * \brief Execute I2C command to test interface.
 *
 * \return Returns true on success, false on failure.
 */
boolean i2cExecTest()
{
  I2CRspBuf[I2CRspLen++] = (byte)CurSeqNum;
  I2CRspBuf[I2CRspLen++] = (byte)CurOpState;
  I2CRspBuf[I2CRspLen++] = (byte)(CurAlarms>>8);
  I2CRspBuf[I2CRspLen++] = (byte)(CurAlarms&0xff);
  I2CRspBuf[I2CRspLen++] = (byte)CurLedPatIdx;

  ++CurSeqNum;

  return true;
}
#endif // DEPRECATED

/*!
 * \brief Execute I2C command to configure firmware operation
 *
 * \return Returns true on success, false on failure.
 */
boolean i2cExecConfigFw()
{
  byte          len = LaeWdCmdLenConfigFw - 1; 
  unsigned int  val_hi, val_lo;

  if( Wire.available() == len )
  {
    val_hi = Wire.read();
    val_lo = Wire.read();

    CurTimeout = (val_hi << 8) | val_lo;
    
    if( CurTimeout < LaeWdTimeoutMin )
    {
      CurTimeout = LaeWdTimeoutMin;
    }
    else if( CurTimeout > LaeWdTimeoutMin )
    {
      CurTimeout = LaeWdTimeoutMax;
    }

    return true;
  }

  // error
  return false;
}


//------------------------------------------------------------------------------
// Serial Functions
//------------------------------------------------------------------------------

/*!
 * \brief Format and print data to ascii serial interface. 
 *
 * \param fmt   Print format string. A subset if printf(3). Note that float
 *              format conversion specifiers are not supported.
 *
 * ...          Variable arguments.
 */
void p(const char *fmt, ...)
{
  va_list args;
  char    buf[LaeWdSerMaxRspLen];

  va_start(args, fmt);
  vsnprintf(buf, LaeWdSerMaxRspLen, fmt, args);
  va_end(args);

  Serial.print(buf);
}

/*!
 * \brief Format print float into buffer.
 *
 * The arduino print facilities do not normally contain float/double print
 * formatting  support.
 *
 * Format: %<width>.0<precision>f
 *
 * \param [out] buf   Output formatted buffer.
 * \param val         FPN to be formatted.
 * \param width       Output width.
 * \param precision   Output precision.
 */
void sfloat(char *buf, float val, int width=6, int precision=1)
{
  int32_t   val_int, val_frac;
  int32_t   mult;
  int       i;

  for(i = 0, mult = 1; i < precision; ++i, mult *= 10);

  val_int = (int32_t)val;
  val = val - (float)val_int;
  if( val < 0.0 )
  {
    val = -val;
  }
  val_frac = (int32_t)(val * mult);

  // don't work with width fileds
  //sprintf(buf, "%*d.%0*d", width, val_int, precision, val_frac);
 
  sprintf(buf, "%d.%d", val_int, val_frac);
}

/*!
 * \brief Format and print standard response to ascii serial interface. 
 *
 * \param fmt   Print format string. A subset if printf(3). Note that float
 *              format conversion specifiers are not supported.
 *
 * ...          Variable arguments.
 */
void serRsp(const char *fmt, ... )
{
  String  strFmt;
  va_list args;
  char    buf[LaeWdSerMaxRspLen];

  sprintf(buf, "%s ", SerArgv[SerCmdIdx]);
  strFmt  = buf;
  strFmt += fmt;
  strFmt += LaeWdSerEoR;

  va_start(args, fmt);
  vsnprintf(buf, LaeWdSerMaxRspLen, strFmt.c_str(), args);
  va_end(args);

  Serial.print(buf);
}

/*!
 * \brief Format and print error response to ascii serial interface. 
 *
 * \param fmt   Print format string. A subset if printf(3). Note that float
 *              format conversion specifiers are not supported.
 *
 * ...          Variable arguments.
 */
void serErrorRsp(const char *fmt, ... )
{
  String  strFmt;
  va_list args;
  char    buf[LaeWdSerMaxRspLen];

  sprintf(buf, "%s %s ", LaeWdSerArgErrRsp, SerArgv[SerCmdIdx]);
  strFmt  = buf;
  strFmt += fmt;
  strFmt += LaeWdSerEoR;

  va_start(args, fmt);
  vsnprintf(buf, LaeWdSerMaxRspLen, strFmt.c_str(), args);
  va_end(args);

  Serial.print(buf);
}

/*!
 * \brief Test is character c is whitespace.
 *
 * \return Returns true or false.
 */
inline boolean whitespace(char c)
{
  return((c == ' ') || (c == '\t'));
}

/*!
 * \brief Test is character c is end-of-line.
 *
 * \return Returns true or false.
 */
inline boolean eol(char c)
{
  return((c == '\n') || (c == '\r'));
}

/*!
 * \brief Receive serial characters and build command.
 *
 * \return Returns true when a complete command has been received. Otherwise
 * false is returned.
 */
boolean serRcvCmd()
{
  char    c;

  while( Serial.available() > 0 )
  {
    c = Serial.read();

    // command too long - truncate (could still be a valid command).
    if( SerLinePos >= LaeWdSerMaxCmdLen )
    {
      SerLine[LaeWdSerMaxCmdLen] = 0;
      SerLinePos = 0;
      return true;
    }

    // end-of-line
    else if( eol(c) )
    {
      SerLine[SerLinePos] = 0;
      SerLinePos = 0;
      return true;
    }

    // command character
    else
    {
      SerLine[SerLinePos++] = c;
    }
  }

  return false;
}

/*!
 * \brief Parse received command into a series of arguments.
 *
 * \return Returns true if cmdid [arg arg...] found. False otherwise.
 */
boolean serParseCmd()
{
  int i, j;
  int len;

  SerArgc = 0;
  len     = strlen(SerLine);
  i       = 0;

  do
  {
    // skip leading white space
    while( (i < len) && whitespace(SerLine[i]) )
    {
      ++i;
    }

    // no more arguments
    if( i >= len )
    {
      break;
    }

    j = 0;

    // copy argument
    while((i < len) && (j < LaeWdSerMaxCmdArgLen-1) && !whitespace(SerLine[i]))
    {
      SerArgv[SerArgc][j++] = SerLine[i++];
    }

    SerArgv[SerArgc++][j] = 0;

  } while( SerArgc < LaeWdSerMaxCmdArgc );

  return SerArgc > 0;
}

/*!
 * \brief Check received argument count vs. expected.
 *
 * \param nExpected   Number of expected arguments.
 *
 * \return Returns true if matched. Else prints error response and returns
 * false.
 */
boolean serChkArgCnt(int nExpected)
{
  if( SerArgc != nExpected )
  {
    serErrorRsp("requires %d args, got %d", nExpected-1, SerArgc-1);
    return false;
  }

  return true;
}

/*!
 * \brief Parse command get/set/reset operator.
 *
 * \param sArg  Operator argument.
 *
 * \return Returns 'g' (get), 's' (set), or 'r' (reset) on success.
 * Prints error response and returns '?' on failure.
 */
int serParseOp(char *sArg, boolean bIncludeReset=false)
{
  String  str(sArg);

  // cmd_id op ...
  if( SerArgc < 2 )
  {
    serErrorRsp("requires >= 1 args, got %d", SerArgc-1);
    return LaeWdSerOpBad;
  }
  else if( str == LaeWdSerArgGet )
  {
    return LaeWdSerOpGet;
  }
  else if( str == LaeWdSerArgSet )
  {
    return LaeWdSerOpSet;
  }
  else if( bIncludeReset && (str == LaeWdSerArgReset) )
  {
    return LaeWdSerOpReset;
  }
  else
  {
    serErrorRsp("unknown operator %s", sArg);
    return LaeWdSerOpBad;
  }
}

/*!
 * \brief Parse and convert decimal number string argument.
 *
 * \param sName       Argument name.
 * \param sVal        Argument string value to convert.
 * \param nMin        Minimum value.
 * \param nMax        Maximum value.
 * \param [out] nVal  Converted value.
 *
 * \return Returns true on success. On error, prints error response and returns
 * false.
 */
boolean serParseNumber(const char *sName, const char *sVal,
                       long  nMin,  long nMax,
                       long &nVal)
{
  char *sEnd;

  if( (sVal == NULL) || (*sVal == 0) )
  {
    serErrorRsp("no %s argument", sName);
    return false;
  }

  nVal = strtol(sVal, &sEnd, 0);
  
  if( *sEnd != 0 )
  {
    serErrorRsp("%s %s is not a number", sName, sVal);
    return false;
  }
  else if( (nVal < nMin) || (nVal > nMax) )
  {
    serErrorRsp("%s %s is out-of-range", sName, sVal);
    return false;
  }

  return true;
}

/*!
 * \brief Execute recieved command.
 */
void serExecCmd()
{
  char    cmdId;
  boolean bMode;

  if( strlen(SerArgv[SerCmdIdx]) > 1 )
  {
    serErrorRsp("bad command");
    return;
  }

  switch( SerArgv[SerCmdIdx][0] )
  {
    case LaeWdSerCmdIdHelp:
      serExecHelp();
      break;
    case LaeWdSerCmdIdGetVersion:
      serExecGetVersion();
      break;
    case LaeWdSerCmdIdPetTheDog:
      serExecPetTheDog();
      break;
    case LaeWdSerCmdIdOpBattSoC:
      serExecOpBattSoC();
      break;
    case LaeWdSerCmdIdOpAlarms:
      serExecOpAlarms();
      break;
    case LaeWdSerCmdIdOpLed:
      serExecOpLed();
      break;
    case LaeWdSerCmdIdOpEnMotorCtlrs:
      serExecOpEnMotorCtlrs();
      break;
    case LaeWdSerCmdIdOpEnAuxPorts:
      serExecOpEnAuxPorts();
      break;
    case LaeWdSerCmdIdReadVolts:
      serExecReadVolts();
      break;
    case LaeWdSerCmdIdOpConfig:
      serExecOpConfig();
      break;
    default:
      serErrorRsp("unknown command");
      return;
  }

  PleasePetTheDog = true;
}

/*!
 * \brief Execute serial command to print help.
 */
void serExecHelp()
{
  const char *cmds[][2] = 
  {
    {"a {g|s} [alarm_bits]",        "get/set robot alarms"},
    {"b {g|s} [batt_soc]",          "get/set battery state of charge"},
    {"c {g|s} [timeout]",           "get/set firmare configuration"},
    {"l {g|s|r} [red green blue]",  "get/set/reset user led override"},
    {"m {g|s} [motor_ctlrs]",       "get/set motor controllers enable line"},
    {"p",                           "pet the watchdog"},
    {"r",                           "read battery and jack voltages"},
    {"v",                           "get firmware version"},
    {"x {g|s} [aux_batt aux_5v]",   "get/set auxilliary port enable lines"},
    {NULL, NULL}
  };

  int   i;

  p("%c", LaeWdSerEoR);

  for(i = 0; cmds[i][0] != NULL; ++i)
  {
    p("%-28s- %s%c", cmds[i][0], cmds[i][1], LaeWdSerEoR);
  }
}

/*!
 * \brief Execute serial command to get the firmware version.
 */
void serExecGetVersion()
{
  if( serChkArgCnt(LaeWdSerCmdArgcGetVersion) )
  {
    serRsp("Laelaps WatchDog v%d", LAE_WD_FW_VERSION);
  }
}

/*!
 * \brief Execute serial command to pet the watchdog.
 */
void serExecPetTheDog()
{
  int   isCharging;

  if( serChkArgCnt(LaeWdSerCmdArgcPetTheDog) )
  {
    isCharging = CurBattIsCharging? LaeWdArgDPinValHigh: LaeWdArgDPinValLow;
    serRsp("%d", isCharging);
  }
}

/*!
 * \brief Execute serial command to get/set the battery state of charge.
 */
void serExecOpBattSoC()
{
  boolean       ok;
  int           op;
  long          val;
  unsigned int  batt_soc;

  op = serParseOp(SerArgv[1]);

  switch( op )
  {
    // Get
    case LaeWdSerOpGet:
      ok = serChkArgCnt(LaeWdSerCmdArgcGetBattSoC);
      break;

    // Set
    case LaeWdSerOpSet:
      if( (ok = serChkArgCnt(LaeWdSerCmdArgcSetBattSoC)) )
      {
        ok = serParseNumber("batt_soc", SerArgv[2],
                      (long)LaeWdArgBattSoCMin, (long)LaeWdArgBattSoCMax,
                      val);
      }

      if( ok )
      {
        batt_soc = (unsigned int)val;

        // new charge state
        if( batt_soc != CurBattSoC )
        {
          CurBattSoC = batt_soc;
          updateBatteryRgb(batt_soc);
          updateInServiceState(true);
        }
      }
      break;

    // Bad
    case LaeWdSerOpBad:
    default:
      ok = false;
      break;
  }

  // successful command
  if( ok )
  {
    serRsp("%d", CurBattSoC);
  }
}

/*!
 * \brief Execute serial command to get/set robot alarms.
 */
void serExecOpAlarms()
{
  boolean       ok;
  int           op;
  long          val;
  unsigned int  alarms;

  op = serParseOp(SerArgv[1]);

  switch( op )
  {
    // Get
    case LaeWdSerOpGet:
      ok = serChkArgCnt(LaeWdSerCmdArgcGetAlarms);
      break;

    // Set
    case LaeWdSerOpSet:
      if( (ok = serChkArgCnt(LaeWdSerCmdArgcSetAlarms)) )
      {
        ok = serParseNumber("alarm_bits", SerArgv[2],
                        (long)LaeWdArgAlarmNone, (long)LaeWdArgAlarmMask,
                        val);
      }

      if( ok )
      {
        CurAlarms = (unsigned int)val;
      }
      break;

    // Bad
    case LaeWdSerOpBad:
    default:
      ok = false;
      break;
  }

  // successful command
  if( ok )
  {
    serRsp("0x%04x", CurAlarms);
  }
}

/*!
 * \brief Execute serial command to get/set/reset RGB LED.
 */
void serExecOpLed()
{
  boolean   ok;
  int       op;
  long      val;
  byte      rgb[NUM_CHANS];
  int       chan;

  op = serParseOp(SerArgv[1], true);

  switch( op )
  {
    // Get
    case LaeWdSerOpGet:
      ok = serChkArgCnt(LaeWdSerCmdArgcGetLed);
      break;

    // Set
    case LaeWdSerOpSet:
      ok = serChkArgCnt(LaeWdSerCmdArgcSetLed);

      for(chan = 0; ok && (chan < NUM_CHANS); ++chan)
      {
          ok = serParseNumber("color", SerArgv[2+chan],
                        (long)LaeWdArgRgbLedMin, (long)LaeWdArgRgbLedMax,
                        val);
          rgb[chan] = (byte)val;
      }

      if( ok )
      {
        LedPat[LedPatIdxUser].rgb[0][RED]   = rgb[RED];
        LedPat[LedPatIdxUser].rgb[0][GREEN] = rgb[GREEN];
        LedPat[LedPatIdxUser].rgb[0][BLUE]  = rgb[BLUE];

        CurUserOverride = true;
        updateInServiceState(true);
      }
      break;

    // Reset
    case LaeWdSerOpReset:
      if( (ok = serChkArgCnt(LaeWdSerCmdArgcResetLed)) )
      {
        CurUserOverride = false;
        updateInServiceState(true);
      }
      break;

    // Bad
    case LaeWdSerOpBad:
    default:
      ok = false;
      return;
  }

  // successful command
  if( ok )
  {
    serRsp("0x%02x 0x%02x 0x%02x", CurLed[RED], CurLed[GREEN], CurLed[BLUE]);
  }
}

/*!
 * \brief Execute serial command to get/set motor controllers power-in enable
 * line.
 */
void serExecOpEnMotorCtlrs()
{
  boolean       ok;
  int           op;
  long          val;
  byte          enable;

  op = serParseOp(SerArgv[1]);

  switch( op )
  {
    // Get
    case LaeWdSerOpGet:
      ok = serChkArgCnt(LaeWdSerCmdArgcGetEnMotorCtlrs);
      break;

    // Set
    case LaeWdSerOpSet:
      if( (ok = serChkArgCnt(LaeWdSerCmdArgcSetEnMotorCtlrs)) )
      {
        ok = serParseNumber("motor_ctlrs", SerArgv[2], 0, 1, val);
      }

      if( ok )
      {
        enable = val? HIGH: LOW;
        digitalWrite(PIN_D_EN_MOTOR_CTLRS, enable);
        delayMicroseconds(5);
      }
      break;

    // Bad
    case LaeWdSerOpBad:
    default:
      ok = false;
      break;
  }

  // successful command
  if( ok )
  {
    enable = digitalRead(PIN_D_EN_MOTOR_CTLRS);
    serRsp("%d", enable);
  }
}

/*!
 * \brief Execute serial command to get/set motor controllers power-in enable
 * line.
 */
void serExecOpEnAuxPorts()
{
  boolean       ok;
  int           op;
  long          val;
  byte          en_batt, en_5v;

  op = serParseOp(SerArgv[1]);

  switch( op )
  {
    // Get
    case LaeWdSerOpGet:
      ok = serChkArgCnt(LaeWdSerCmdArgcGetEnAuxPorts);
      break;

    // Set
    case LaeWdSerOpSet:
      ok = serChkArgCnt(LaeWdSerCmdArgcSetEnAuxPorts);
      if( ok )
      {
        val = 0;
        ok = serParseNumber("aux_batt", SerArgv[2], 0, 1, val);
        en_batt = val? HIGH: LOW;
      }
      if( ok )
      {
        val = 0;
        ok = serParseNumber("aux_5v", SerArgv[3], 0, 1, val);
        en_5v = val? HIGH: LOW;
      }

      if( ok )
      {
        digitalWrite(PIN_D_EN_AUX_PORT_BATT, en_batt);
        digitalWrite(PIN_D_EN_AUX_PORT_5V, en_5v);
        delayMicroseconds(5);
      }
      break;

    // Bad
    case LaeWdSerOpBad:
    default:
      ok = false;
      break;
  }

  // successful command
  if( ok )
  {
    en_batt = digitalRead(PIN_D_EN_AUX_PORT_BATT);
    en_5v   = digitalRead(PIN_D_EN_AUX_PORT_5V);
    serRsp("%d %d", en_batt, en_5v);
  }
}

/*!
 * \brief Execute serial command to read voltages.
 */
void serExecReadVolts()
{
  char  buf_batt_v[16], buf_jack_v[16];
  int bv, jv;

  if( serChkArgCnt(LaeWdSerCmdArgcReadVolts) )
  {
    readVoltages();
    //sfloat(buf_batt_v, CurBattV, 4, 1);
    //sfloat(buf_jack_v, CurJackV, 4, 1);
    //serRsp("%s %s", buf_jack_v, buf_batt_v);
    bv = (int)(CurBattV * 10.0);
    jv = (int)(CurJackV * 10.0);
    serRsp("%d %d", jv, bv);
  }
}

/*!
 * \brief Execute serial command to get/set firmware operation configuration.
 */
void serExecOpConfig()
{
  boolean       ok;
  int           op;
  long          val;

  op = serParseOp(SerArgv[1]);

  switch( op )
  {
    // Get
    case LaeWdSerOpGet:
      ok = serChkArgCnt(LaeWdSerCmdArgcGetConfig);
      break;

    // Set
    case LaeWdSerOpSet:
      ok = serChkArgCnt(LaeWdSerCmdArgcSetConfig);
      if( ok )
      {
        ok = serParseNumber("timeout", SerArgv[2],
            (long)LaeWdTimeoutMin, (long)LaeWdTimeoutMax, val);
      }

      if( ok )
      {
        CurTimeout = (unsigned long)val;
      }
      break;

    // Bad
    case LaeWdSerOpBad:
    default:
      ok = false;
      break;
  }

  // successful command
  if( ok )
  {
    serRsp("%u", CurTimeout);
  }
}
