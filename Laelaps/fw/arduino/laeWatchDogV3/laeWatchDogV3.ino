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
 * \author Nick Andersen (nick@roadnarrows.com)
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

//
// Bridge between standard POSIX C/C++ Linux and Arduino constructs.
//
typedef byte byte_t;

// Interface between firmware and software.
#include "laeWatchDog.h"

using namespace laelaps;

#include <Wire.h>

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

// Battery Charging Input Pins
const int   PIN_A_JACK_V  = A0;         ///< jack voltage into battery charging
const int   PIN_A_BATT_V  = A1;         ///< battery output voltage
const float ADC_V_PER_VAL = 0.01953125; ///< 0.0V - 20.0V at 1023 values
const float JACK_V_MIN    = 13.9;       ///< required minimum voltage to charge
const float JACK_V_TRIM   = 1.1089;     ///< thru experimentation, hopefully the
const float BATT_V_TRIM   = 1.0540;     ///< op-amped version of the deckboard
                                        ///< provides part-to-part consistency

// Digital Pin Shadow State
int DPinDir[MAX_D_PINS];          ///< direction 
int DPinVal[MAX_D_PINS];          ///< value


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
int     CurOpState;         ///< robot operational state
float   CurJackV;           ///< current sensed jack input voltage
float   CurBattV;           ///< current sensed battery output voltage
boolean CurBattIsCharging;  ///< battery is [not] currently being charged
int     CurBattSoC;         ///< battery state of charge 0% - 100%
int     CurAlarms;          ///< current alarms
int     CurLedPatIdx;       ///< active LED pattern index
boolean CurUserOverride;    ///< user has [no] RGB LED pattern override
boolean ForceLedUpdate;     ///< do [not] force RGB LED update
int     CurSeqNum;          ///< response sequence number (for some commands)

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
// Response data
//
byte        RspBuf[LaeWdMaxRspLen];         ///< response buffer
int         RspLen;                         ///< response length
const byte  RspErrorBuf[LaeWdMaxRspLen] =   ///< response error pattern
{
  0xde, 0xad, 0x01, 0x02, 0x03, 0x04, 0xfa, 0xce
};


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
  CurJackV          = 0.0;
  CurBattV          = 0.0;
  CurBattIsCharging = false;
  CurBattSoC        = LaeWdArgBattChargeMax;
  CurAlarms         = LaeWdArgAlarmNone;
  CurLedPatIdx      = LedPatIdxNoService;
  CurUserOverride   = false;
  ForceLedUpdate    = false;
  CurSeqNum         = 0;

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

    DPinDir[pin] = dir;
    DPinVal[pin] = val;
  }

  // configure analog input pins
  for(pin = PIN_A_USER_MIN; pin <= PIN_A_USER_MIN; ++pin)
  {
    digitalWrite(pin, LOW);   // no pull-ups
  }

  //
  // Timers and timeout periods
  //
  Tcur  = millis();
  Twd   = Tcur;
  Tled  = Tcur;

  //
  // Response
  //
  RspLen = 0;

  //
  // I2C slave
  //
  Wire.begin(LaeI2CAddrArduino);
  Wire.onReceive(receiveCmd);
  Wire.onRequest(sendRsp);
}

/*!
 * \brief Arduino Sketch loop() hook.
 *
 * Called each time through Sketch main loop.
 */
void loop()
{
  int   sensorval;

  // current up time
  Tcur = millis();

  // watchdog timed out
  if( (CurOpState != OpStateNoService) && (dt(Tcur, Twd) >= LaeWdTimeout) )
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

  // update current LED pattern transition
  updateLedPattern();

  // read jack voltage input to battery charging circuitry
  sensorval = analogRead(PIN_A_JACK_V);
  CurJackV  = (float)sensorval * ADC_V_PER_VAL * JACK_V_TRIM;
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
  CurBattV  = (float)sensorval * ADC_V_PER_VAL * BATT_V_TRIM;

  delay(10);
}

/*!
 * \brief Arduino Sketch I2C receive callback.
 *
 * Receive and process command. The commands and responses are over the I2C bus.
 *
 * This function is called asynchronously by the Arduino I2C slave framework.
 */
void receiveCmd(int n)
{
  byte    cmdId;
  boolean bOk;

  if( Wire.available() > 0 )
  {
    cmdId   = Wire.read();
    RspLen  = 0;

    switch( cmdId )
    {
      case LaeWdCmdIdPetDog:
        bOk = execPetDog();
        break;
      case LaeWdCmdIdGetVersion:
        bOk = execGetVersion();
        break;
      case LaeWdCmdIdSetBattCharge:
        bOk = execSetBattCharge();
        break;
      case LaeWdCmdIdSetAlarms:
        bOk = execSetAlarms();
        break;
      case LaeWdCmdIdSetRgbLed:
        bOk = execSetRgbLed();
        break;
      case LaeWdCmdIdResetRgbLed:
        bOk = execResetRgbLed();
        break;

#if 0 // DEPRECATED
      case LaeWdCmdIdConfigDPin:
        bOk = execConfigDPin();
        break;
      case LaeWdCmdIdReadDPin:
        bOk = execReadDPin();
        break;
      case LaeWdCmdIdWriteDPin:
        bOk = execWriteDPin();
        break;
      case LaeWdCmdIdReadAPin:
        bOk = execReadAPin();
        break;
      case LaeWdCmdIdWriteAPin:
        bOk = execWriteAPin();
        break;
#endif // DEPRECATED

      case LaeWdCmdIdEnableMotorCtlrs:
        bOk = execEnableMotorCtlrs();
        break;
      case LaeWdCmdIdEnableAuxPort:
        bOk = execEnableAuxPort();
        break;
      case LaeWdCmdIdReadEnables:
        bOk = execReadEnables();
        break;
      case LaeWdCmdIdReadVolts:
        bOk = execReadVolts();
        break;
      case LaeWdCmdIdTest:
        execTest();
        break;
      default:
        flushRead();
        bOk = false;
    }
  }

  // the dog has been mollified
  if( bOk )
  {
    setInServiceState();
    Twd = millis();
  }
}

/*!
 * \brief Send any response loaded in response buffer.
 */
void sendRsp()
{
  if( RspLen > 0 )
  {
    Wire.write(RspBuf, RspLen);
    RspLen = 0;
  }
}

/*!
 * \brief Error response for bad commands the expect data back.
 *
 * \param n   Length of expected response.
 */
void errorRsp(int n)
{
  int   i;

  // flush input 
  flushRead();

  // fill response buffer with recognizable error pattern
  for(; RspLen < n; ++RspLen)
  {
    RspBuf[RspLen] = RspErrorBuf[RspLen];
  }
}

/*!
 * \brief Flush the I2C input buffer.
 */
void flushRead()
{
  byte  d;

  while( Wire.available() > 0 )
  {
    d = Wire.read();
  }
}


//------------------------------------------------------------------------------
// Command Functions
//------------------------------------------------------------------------------

/*!
 * \brief Execute I2C command to pet the watchdog.
 *
 * \return Returns true on success, false on failure.
 */
boolean execPetDog()
{
  RspBuf[RspLen++] = CurBattIsCharging? LaeWdArgDPinValHigh: LaeWdArgDPinValLow;
  return true;
}

/*!
 * \brief Execute I2C command to get the firmware's version number.
 *
 * \return Returns true on success, false on failure.
 */
boolean execGetVersion()
{
  RspBuf[RspLen++] = (byte)LAE_WD_FW_VERSION;
  return true;
}

/*!
 * \brief Execute I2C command to set the battery charge state.
 *
 * \return Returns true on success, false on failure.
 */
boolean execSetBattCharge()
{
  byte  len = LaeWdCmdLenSetBattCharge - 1; 
  byte  batt;

  if( Wire.available() == len )
  {
    batt = Wire.read();
    if( batt > LaeWdArgBattChargeMax )
    {
      batt = LaeWdArgBattChargeMax;
    }

    // new charge state
    if( batt != CurBattSoC )
    {
      updateBatteryRgb(batt);
      CurBattSoC = batt;
      ForceLedUpdate = true;
    }

    return true;
  }

  // error
  flushRead();
  return false;
}

/*!
 * \brief Execute I2C command to set/clear alarm bits.
 *
 * \return Returns true on success, false on failure.
 */
boolean execSetAlarms()
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
  flushRead();
  return false;
}

/*!
 * \brief Execute I2C command for user set the RGB LED with an override value.
 *
 * \return Returns true on success, false on failure.
 */
boolean execSetRgbLed()
{
  byte len = LaeWdCmdLenSetRgbLed - 1; 

  if( Wire.available() == len )
  {
    LedPat[LedPatIdxUser].rgb[0][RED]   = Wire.read();
    LedPat[LedPatIdxUser].rgb[0][GREEN] = Wire.read();
    LedPat[LedPatIdxUser].rgb[0][BLUE]  = Wire.read();

    CurUserOverride = true;
    ForceLedUpdate  = true;

    return true;
  }

  // error
  flushRead();
  return false;
}

/*!
 * \brief Execute I2C command to clear any user's RGB LED override value.
 *
 * \return Returns true on success, false on failure.
 */
boolean execResetRgbLed()
{
  byte len = LaeWdCmdLenResetRgbLed - 1; 

  CurUserOverride = false;
  ForceLedUpdate  = true;

  return true;
}

#if 0 // DEPRECATED
/*!
 * \brief Execute I2C command to configure a digital pin.
 *
 * \return Returns true on success, false on failure.
 */
boolean execConfigDPin()
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
  flushRead();
  return false;
}

/*!
 * \brief Execute I2C command to read a value of a digital pin.
 *
 * \return Returns true on success, false on failure.
 */
boolean execReadDPin()
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
      RspBuf[RspLen++] = pin;
      RspBuf[RspLen++] = val;
      DPinVal[pin] = val;
      return true;
    }
  }

  // error
  errorRsp(LaeWdRspLenReadDPin);

  return false;
}

/*!
 * \brief Execute I2C command to write a value to a digital pin.
 *
 * \return Returns true on success, false on failure.
 */
boolean execWriteDPin()
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
  flushRead();
  return false;
}

/*!
 * \brief Execute I2C command to read the value from an analog pin.
 *
 * \return Returns true on success, false on failure.
 */
boolean execReadAPin()
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

        RspBuf[RspLen++] = pin;
        RspBuf[RspLen++] = val_hi;
        RspBuf[RspLen++] = val_lo;

        return true;
      }
    }
  }

  // error
  errorRsp(LaeWdRspLenReadAPin);

  return false;
}

/*!
 * \brief Execute I2C command to write a value to an analog pin.
 *
 * \return Returns true on success, false on failure.
 */
boolean execWriteAPin()
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
  flushRead();
  return false;
}
#endif // DEPRECATED

/*!
 * \brief Execute I2C command to enable/disable power to motor controllers.
 *
 * \return Returns true on success, false on failure.
 */
boolean execEnableMotorCtlrs()
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
      DPinVal[PIN_D_EN_MOTOR_CTLRS] = wval;
      RspBuf[RspLen++] = LaeWdArgPass;
    }
    else
    {
      RspBuf[RspLen++] = LaeWdArgFail;
    }

    return true;
  }

  // error
  errorRsp(LaeWdRspLenEnableMotorCtlrs);

  return false;
}

/*!
 * \brief Execute I2C command to enable/disable power to aux port.
 *
 * \return Returns true on success, false on failure.
 */
boolean execEnableAuxPort()
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
      flushRead();
      return false;
    }

    val = val? HIGH: LOW;

    digitalWrite(pin, val);
    DPinVal[pin] = val;

    return true;
  }

  // error
  flushRead();
  return false;
}

/*!
 * \brief Execute I2C command to read GPIO enable lines.
 *
 * \return Returns true on success, false on failure.
 */
boolean execReadEnables()
{
  byte  val;

  val = digitalRead(PIN_D_EN_MOTOR_CTLRS);
  RspBuf[RspLen++] = val == LOW? LaeWdArgDPinValLow: LaeWdArgDPinValHigh;

  val = digitalRead(PIN_D_EN_AUX_PORT_5V);
  RspBuf[RspLen++] = val == LOW? LaeWdArgDPinValLow: LaeWdArgDPinValHigh;

  val = digitalRead(PIN_D_EN_AUX_PORT_BATT);
  RspBuf[RspLen++] = val == LOW? LaeWdArgDPinValLow: LaeWdArgDPinValHigh;

  return true;
}

/*!
 * \brief Execute I2C command to read sensed voltages.
 *
 * \return Returns true on success, false on failure.
 */
boolean execReadVolts()
{
  byte    len = LaeWdCmdLenReadVolts - 1;
  byte    val;

  val = (byte)(CurJackV * LaeWdArgVMult);
  RspBuf[RspLen++] = val;

  val = (byte)(CurBattV * LaeWdArgVMult);
  RspBuf[RspLen++] = val;

  return true;
}

/*!
 * \brief Execute I2C command to test interface.
 *
 * \return Returns true on success, false on failure.
 */
boolean execTest()
{
  RspBuf[RspLen++] = (byte)CurSeqNum;
  RspBuf[RspLen++] = (byte)CurOpState;
  RspBuf[RspLen++] = (byte)(CurAlarms>>8);
  RspBuf[RspLen++] = (byte)(CurAlarms&0xff);
  RspBuf[RspLen++] = (byte)CurLedPatIdx;

  ++CurSeqNum;

  return true;
}


//------------------------------------------------------------------------------
// Utilities
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

/*
 * \brief Enter no-service operational state. 
 *
 * Some state information is reset here.
 */
void enterNoServiceState()
{
  CurOpState      = OpStateNoService;
  CurBattSoC      = LaeWdArgBattChargeMax;
  CurAlarms       = LaeWdArgAlarmNone;
  CurLedPatIdx    = LedPatIdxNoService;
  CurUserOverride = false;
  ForceLedUpdate  = false;

  digitalWrite(PIN_D_EN_MOTOR_CTLRS, LOW);
  DPinVal[PIN_D_EN_MOTOR_CTLRS] = LOW;

  updateBatteryRgb(CurBattSoC);
  activateLedPattern(CurLedPatIdx);
}

/*!
 * \brief Change operational state. 
 *
 * The new state is determined from the current operational state, alarms, and
 * user overrides.
 */
void setInServiceState()
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
      ForceLedUpdate )
  {
    CurOpState      = newOpState;
    ForceLedUpdate  = false;
    activateLedPattern(newLedPatIdx);
  }
}

/*!
 * \brief Update battery charge state.
 */
void updateBatteryRgb(byte batt)
{
  float h, s, v;

  if( batt >= 95 )
  {
    LedPat[LedPatIdxBatt].rgb[0][RED]   = LaeWdArgRgbLedMax;
    LedPat[LedPatIdxBatt].rgb[0][GREEN] = LaeWdArgRgbLedMax;
    LedPat[LedPatIdxBatt].rgb[0][BLUE]  = LaeWdArgRgbLedMax;
  }
  else if( batt <= 5 )
  {
    LedPat[LedPatIdxBatt].rgb[0][RED]   = 0x7b;
    LedPat[LedPatIdxBatt].rgb[0][GREEN] = 0x4a;
    LedPat[LedPatIdxBatt].rgb[0][BLUE]  = 0x03;
  }
  else
  {
    h = 35.0;
    s = (float)(LaeWdArgBattChargeMax - batt);
    v = (float)(LaeWdArgBattChargeMax + batt) / 2.0;

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
  int     i;

  curPos = LedPat[CurLedPatIdx].curPos;

  for(i = 0; i < NUM_CHANS; ++i)
  {
    analogWrite(PIN_PWM_RGB[i], LedPat[CurLedPatIdx].rgb[curPos][i]);
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
