////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Firmware:  Time-of-Flight Multiplex Arduino Compatible Firmware, v2.
//
// File:      laeToFMuxV2.ino
//
/*! \file
 *
 * \brief Laelaps time-of-flight multiplexor Arduino compatible firmware.
 *
 * The firmware is an I2C slave device to the host and an I2C master to the
 * ToF sensors. Each sensor is attached to a software bit-banged I2C port.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2016-2017  RoadNarrows
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
#define LAE_ARDUINO             1 ///< arduino target 
#define LAE_TOF_MUX_FW_VERSION  2 ///< firmware version

#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <Wire.h>
#include <SoftwareWire.h>

//#include <avr/wdt.h>    // dangerous

//
// Bridge between standard POSIX C/C++ Linux and Arduino constructs.
//
typedef byte byte_t;

#include "laeToFMux.h"  // interface between firmware and software.
#include "VL6180X.h"    // interface to sensor class

using namespace laelaps;

//
// SoftwareWire constructors.
// Parameters:
//   (1) pin for the software sda
//   (2) pin for the software scl
//   (3) use internal pullup resistors. Default true. Set to false to disable
//       them.
//   (4) allow the Slave to stretch the clock pulse. Default true. Set to false
//       for faster code.
//
SoftwareWire softWire0(A4, A5, true, false);
SoftwareWire softWire1( 4,  5, true, false);
SoftwareWire softWire2( 6,  7, true, false);
SoftwareWire softWire3( 8,  9, true, false);
SoftwareWire softWire4(10, 11, true, false);
SoftwareWire softWire5(12, 13, true, false);
SoftwareWire softWire6(A0, A1, true, false);
SoftwareWire softWire7(A2, A3, true, false);

//
// The sensor I2C interface.
//
SoftwareWire *SoftWire[LaeToFMuxNumOfChan] =
{
  &softWire0, &softWire1, &softWire2, &softWire3,
  &softWire4, &softWire5, &softWire6, &softWire7
};

//
// The Sensors
//
VL6180x    *ToFSensor[LaeToFMuxNumOfChan];  ///< the sensor objects
int         ToFNumConn;                     ///< number of sensors connected

const int   DistAlsRatio = 16;              ///< dist to amb sense approx ratio
int         AlsSensorId;                    ///< current ALS sensor id
int         AlsCounter;                     ///< ALS sense counter
int         AlsFreq;                        ///< ALS sense frequency 

//
// I2C slave data
//
byte        I2CRspBuf[LaeToFMuxI2CMaxRspLen];  ///< response buffer
int         I2CRspLen;                      ///< response length

//
// Serial CLI
//
char      SerLine[LaeToFMuxSerMaxCmdLen+1];
int       SerLinePos;
char      SerArgv[LaeToFMuxSerMaxCmdArgc][LaeToFMuxSerMaxCmdArgLen];
int       SerArgc;
const int SerCmdIdx = 0;
boolean   SerContinuousMode;
byte      SerContinuousOutput;

//
// One or more sensors need reseting. Do this by letting this subprocessor
// watchdog timeout.
//
//boolean   ForceWdtTimeout;

//
// Define level of serial help output. There is limited global variable space.
// Choices are: HELP_MIN HELP_FULL HELP_SYN
//
#define HELP_SYN

//------------------------------------------------------------------------------
// Arduino Hook Functions
//------------------------------------------------------------------------------

/*!
 * \brief Arduino setup() hook.
 */
void setup()
{
  int   i;

  //
  // Do this quickly.
  //
  // Note:
  // Watchdog timer is dangerous. It can brick the processor while downloading
  // new firmware. Also, the reset on timeout does not pull the VL6180 sensors
  // to power reset, which is where the problem lies. Hung sensors. And the 
  // sensors provide no reset register.
  //
  //wdt_disable();

  //
  // Begin software wires.
  //
  for(i = 0; i < LaeToFMuxNumOfChan; ++i)
  {
    SoftWire[i]->begin();
  }

  //
  // Create sensors.
  //
  for(i = 0; i < LaeToFMuxNumOfChan; ++i)
  {
    ToFSensor[i] = new VL6180x(i, *SoftWire[i]);
  }

  //
  // Probe and initialize sensors and class object data.
  //
  probe();

#ifndef LAE_USE_ALS
  //
  // Disable firmware from taking ALS measurements.
  //
  // Note:
  // The serial command-line interface can reenable.
  //
  AlsSensorId = -1;
#endif

  //
  // I2C slave setup. Receive and send from/to I2C master by asynchronous
  // callbacks.
  //
  I2CRspLen = 0;
  Wire.begin(LaeI2CAddrToFMux);
  Wire.onReceive(i2cReceiveCmd);
  Wire.onRequest(i2cSendRsp);

  //
  // Serial command-line ASCII interface (baud does not matter over USB) 
  // 
  SerLinePos        = 0;
  SerArgc           = 0;
  SerContinuousMode = false;
  Serial.begin(115200);

  //
  // Enable watchdog timer.
  //
  // Note:
  //  The maximum watchdog timeout is 8 seconds.
  //
  //wdt_enable(WDTO_8S);
  //ForceWdtTimeout = false;
}

/*!
 * \brief Arduino loop() hook.
 */
void loop()
{
  //
  // Take asynchronous measurements.
  //
  measure();

  //
  // Produce any serial continuous mode output.
  //
  if( SerContinuousMode )
  {
    serContinuousOutput();
  }

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
  // Pet the watchdog be reseting its timeout counter.
  //
  //if( !ForceWdtTimeout )
  //{
  //  wdt_reset();  
  //}
}


//------------------------------------------------------------------------------
// Master Interface to Sensors
//------------------------------------------------------------------------------

/*!
 * \brief Probe for all connected I2C sensors.
 *  
 * Each discovered sensor is (re)initialized. Each unconnected or non-repsonsive
 * sensor is blacklisted.
 */
void probe()
{
  int   i;

  ToFNumConn  = 0;
  AlsSensorId = -1;
  AlsFreq     = DistAlsRatio;

  for(i = 0; i < LaeToFMuxNumOfChan; ++i)
  {
    if( ToFSensor[i]->waitForBootup(5) && ToFSensor[i]->ping(3) )
    {
      ToFSensor[i]->whitelist();
      ToFSensor[i]->readIdent();
      ToFSensor[i]->initSensor();
      ToFSensor[i]->writeSensorDefaults();
      ToFSensor[i]->readTunes();

      ++ToFNumConn;
      AlsSensorId = i;
    }
    else
    {
      ToFSensor[i]->blacklist();
    }
  }

#ifndef LAE_USE_ALS
  AlsSensorId = -1;
#endif // !LAE_USE_ALS

  if( ToFNumConn > 0 )
  {
    AlsFreq = DistAlsRatio / ToFNumConn;
  }

  AlsCounter = AlsFreq;
}

/*!
 * \brief Take distance and ambient light measurements from all sensors.
 */
void measure()
{
  int     i;

  //
  // Loop through all sensors and take the appropriate measurements.
  //
  for(i = 0; i < LaeToFMuxNumOfChan; ++i)
  {
    //
    // No sensor attached.
    //
    if( ToFSensor[i]->isBlacklisted() )
    {
      continue;
    }

    //
    // This sensor is scheduled or is actively taking an ambient light
    // sensor measurement.
    //
    if( AlsSensorId == i )
    {
      //
      // However, sensor is still taking a range measurement.
      //
      if( AlsCounter > 0 )
      {
        // Pushing distance measurement through its states.
        // Returns true when finished.
        if( ToFSensor[i]->asyncMeasureRange() )
        {
          --AlsCounter; // when done decrement ALS counter.
          //if( i == 0 ) p("%d %d\n", i, AlsCounter);
        }
      }

      //
      // Take an ALS measurement.
      //
      else
      {
        // Push ambient light measurement through its states.
        // Returns true when finished.
        if( ToFSensor[i]->asyncMeasureAmbientLight() )
        {
          //if( i == 0 ) p("%d m ..... done\n", i);
          // when done move to next sensor
          AlsSensorId = nextSensor(AlsSensorId);
          AlsCounter  = AlsFreq;
        }
        else
        {
          //if( i == 0 ) p("%d m .....\n", i);
        }
      }
    }

    //
    // This sensor is not marked for taking an ambient light sensor measurement.
    // So take a distance measurement.
    //
    else
    {
      // Push. Returns true when finished.
      ToFSensor[i]->asyncMeasureRange();
    }

    //
    // Any hung sensor will force the processor watchdog timer to enventually
    // timeout and reset the processor.
    //
    // Dangerouse, not used.
    //
    //if( ToFSensor[i].isHung() )
    //{
    //  ForceWdtTimeout = true;
    //}
  }
}

/*!
 * \brief Find the next sensor in sequence.
 *
 * \param sensor  Starting sensor id.
 *
 * \param Next sesnor id.
 */
int nextSensor(int sensor)
{
  int   i;

  // no sensors
  if( ToFNumConn == 0 )
  {
    return -1;
  }

  sensor = (sensor + 1) % LaeToFMuxNumOfChan;

  for(i = 0; i < LaeToFMuxNumOfChan; ++i)
  {
    if( ToFSensor[sensor]->isBlacklisted() )
    {
      sensor = (sensor + 1) % LaeToFMuxNumOfChan;
    }
    else
    {
      break;
    }
  }

  return sensor;
}

#ifdef INCLUDE_EXTRAS
/*!
 * \brief Synchronously take distance measurements from all connected sensors.
 */
void measureRanges()
{
  int   i;
  byte  dist;

  for(i = 0; i < LaeToFMuxNumOfChan; ++i)
  {
    if( !ToFSensor[i]->isBlacklisted() )
    {
      dist = ToFSensor[i]->measureRange();
      delay(5);
    }
  }
}

/*!
 * \brief Synchronously take an ambient light measurement from the given
 * sensor.
 *
 * \param sensor  Sensor id.
 */
void measureAmbient(int sensor)
{
  float flux;

  if( !ToFSensor[sensor]->isBlacklisted() )
  {
    flux = ToFSensor[sensor]->measureAmbientLight();
    delay(5);
  }
}
#endif // INCLUDE_EXTRAS


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
      case LaeToFMuxI2CCmdIdGetVersion:
        bOk = i2cExecGetVersion();
        break;
      case LaeToFMuxI2CCmdIdGetIdent:
        bOk = i2cExecGetIdent();
        break;
      case LaeToFMuxI2CCmdIdGetRanges:
        bOk = i2cExecGetRanges();
        break;
      case LaeToFMuxI2CCmdIdGetLux:
        bOk = i2cExecGetAmbients();
        break;
      case LaeToFMuxI2CCmdIdTuneToFSensor:
        bOk = i2cExecTuneRange();
        break;
      case LaeToFMuxI2CCmdIdTuneAls:
        bOk = i2cExecTuneAls();
        break;
      case LaeToFMuxI2CCmdIdGetTunes:
        bOk = i2cExecGetTunes();
        break;
      default:
        i2cFlushRead();
        bOk = false;
    }
  }
}

/*!
 * \brief Send any response pre-loaded in response buffer.
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
 * \brief I2C error response for bad commands the expect data back.
 *
 * \param n   Length of expected non-error response.
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
  byte  b;

  while( Wire.available() > 0 )
  {
    b = Wire.read();
  }
}

/*!
 * \brief Execute I2C command to get the firmware's version number.
 *
 * \return Returns true on success, false on failure.
 */
boolean i2cExecGetVersion()
{
  I2CRspBuf[I2CRspLen++] = (byte)LAE_TOF_MUX_FW_VERSION;
  return true;
}

/*!
 * \brief Execute I2C command to get a sensor's identity.
 *
 * \return Returns true on success, false on failure.
 */
boolean i2cExecGetIdent()
{
  byte                  len = LaeToFMuxI2CCmdLenGetIdent - 1; 
  byte                  sensor;
  VL6180xIdentification ident;

  if( Wire.available() == len )
  {
    sensor = Wire.read();

    if( sensor < LaeToFMuxNumOfChan ) 
    {
      ToFSensor[sensor]->getIdent(&ident);

      I2CRspBuf[I2CRspLen++] = (byte)ident.idModel;
      I2CRspBuf[I2CRspLen++] = (byte)ident.idModelRevMajor;
      I2CRspBuf[I2CRspLen++] = (byte)ident.idModelRevMinor;
      I2CRspBuf[I2CRspLen++] = (byte)ident.idModuleRevMajor;
      I2CRspBuf[I2CRspLen++] = (byte)ident.idModuleRevMinor;
      I2CRspBuf[I2CRspLen++] = (byte)(ident.idDate >> 8);
      I2CRspBuf[I2CRspLen++] = (byte)(ident.idDate & 0xff);
      I2CRspBuf[I2CRspLen++] = (byte)(ident.idTime >> 8);
      I2CRspBuf[I2CRspLen++] = (byte)(ident.idTime & 0xff);

      return true;
    }
  }

  // error
  i2cErrorRsp(LaeToFMuxI2CRspLenGetIdent);

  return false;
}

/*!
 * \brief Execute I2C command to get measured distances.
 *
 * \return Returns true on success, false on failure.
 */
boolean i2cExecGetRanges()
{
  int   i;
  byte  dist;

  for(i = 0; i < LaeToFMuxNumOfChan; ++i)
  {
    if( ToFSensor[i]->isBlacklisted() )
    {
      dist = LaeToFMuxRangeNoDev;
    }
    else
    {
      dist = ToFSensor[i]->getRange();
      if( dist > LaeToFMuxRangeMax )
      {
        dist = LaeToFMuxRangeNoObj;
      }
    }

    I2CRspBuf[I2CRspLen++] = dist;
  }

  return true;
}

/*!
 * \brief Execute I2C command to get ambient light measurements.
 *
 * \return Returns true on success, false on failure.
 */
boolean i2cExecGetAmbients()
{
  int       i;
  float     lux;
  uint32_t  val;

  for(i = 0; i < LaeToFMuxNumOfChan; ++i)
  {
    if( ToFSensor[i]->isBlacklisted() )
    {
      lux = LaeToFMuxLuxNoLight;
    }
    else
    {
      lux = ToFSensor[i]->getAmbientLight();
    }

    val = (uint32_t)(lux * LaeToFMuxI2CArgLuxMult);

    I2CRspBuf[I2CRspLen++] = (byte)((val >> 24) & 0xff);
    I2CRspBuf[I2CRspLen++] = (byte)((val >> 16) & 0xff);
    I2CRspBuf[I2CRspLen++] = (byte)((val >>  8) & 0xff);
    I2CRspBuf[I2CRspLen++] = (byte)(val & 0xff);
  }

  return true;
}

/*!
 * \brief Execute I2C command to tune range sensor parameters.
 *
 * \return Returns true on success, false on failure.
 */
boolean i2cExecTuneRange()
{
  byte      len = LaeToFMuxI2CCmdLenTuneToFSensor - 1; 
  byte      sensor;
  uint16_t  val_hi, val_lo;
  byte      offset;
  uint16_t  crosstalk;

  if( Wire.available() == len )
  {
    sensor    = Wire.read();
    offset    = Wire.read();
    val_hi    = (uint16_t)Wire.read();
    val_lo    = (uint16_t)Wire.read();
    crosstalk = val_hi << 8 | val_lo;

    if( sensor < LaeToFMuxNumOfChan ) 
    {
      ToFSensor[sensor]->markRangeForTuning(offset, crosstalk);
      return true;
    }
  }

  // error
  i2cErrorRsp(LaeToFMuxI2CRspLenTuneToFSensor);

  return false;
}

/*!
 * \brief Execute I2C command to tune ambient light sensor parameters.
 *
 * \return Returns true on success, false on failure.
 */
boolean i2cExecTuneAls()
{
  byte      len = LaeToFMuxI2CCmdLenTuneAls - 1; 
  byte      sensor;
  uint16_t  val_hi, val_lo;
  byte      gain;
  uint16_t  intPeriod;

  if( Wire.available() == len )
  {
    sensor    = Wire.read();
    gain      = Wire.read();
    val_hi    = (uint16_t)Wire.read();
    val_lo    = (uint16_t)Wire.read();
    intPeriod = val_hi << 8 | val_lo;

    if( sensor < LaeToFMuxNumOfChan ) 
    {
      ToFSensor[sensor]->markAlsForTuning(gain, intPeriod);
      return true;
    }
  }

  // error
  i2cErrorRsp(LaeToFMuxI2CRspLenTuneAls);

  return false;
}

/*!
 * \brief Execute I2C command to get sensor tune parameters.
 *
 * \return Returns true on success, false on failure.
 */
boolean i2cExecGetTunes()
{
  byte      len = LaeToFMuxI2CCmdLenGetTunes - 1; 
  byte      sensor;
  byte      offset;
  uint16_t  crosstalk;
  byte      gain;
  uint16_t  intPeriod;
  byte      val_hi, val_lo;

  if( Wire.available() == len )
  {
    sensor = Wire.read();

    if( sensor < LaeToFMuxNumOfChan ) 
    {
      ToFSensor[sensor]->getTunes(offset, crosstalk, gain, intPeriod);

      // tof
      I2CRspBuf[I2CRspLen++] = offset;
      val_hi = (byte)(crosstalk >> 8);
      val_lo = (byte)(crosstalk & 0xff);
      I2CRspBuf[I2CRspLen++] = val_hi;
      I2CRspBuf[I2CRspLen++] = val_lo;

      // als
      I2CRspBuf[I2CRspLen++] = gain;
      val_hi = (byte)(intPeriod >> 8);
      val_lo = (byte)(intPeriod & 0xff);
      I2CRspBuf[I2CRspLen++] = val_hi;
      I2CRspBuf[I2CRspLen++] = val_lo;

      return true;
    }
  }

  // error
  i2cErrorRsp(LaeToFMuxI2CRspLenGetTunes);

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
  char    buf[LaeToFMuxSerMaxRspLen];

  va_start(args, fmt);
  vsnprintf(buf, LaeToFMuxSerMaxRspLen, fmt, args);
  va_end(args);

  Serial.print(buf);
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
  char    buf[LaeToFMuxSerMaxRspLen];

  sprintf(buf, "%s ", SerArgv[SerCmdIdx]);
  strFmt  = buf;
  strFmt += fmt;
  strFmt += LaeToFMuxSerEoR;

  va_start(args, fmt);
  vsnprintf(buf, LaeToFMuxSerMaxRspLen, strFmt.c_str(), args);
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
  char    buf[LaeToFMuxSerMaxRspLen];

  sprintf(buf, "%s %s ", LaeToFMuxSerArgErrRsp, SerArgv[SerCmdIdx]);
  strFmt  = buf;
  strFmt += fmt;
  strFmt += LaeToFMuxSerEoR;

  va_start(args, fmt);
  vsnprintf(buf, LaeToFMuxSerMaxRspLen, strFmt.c_str(), args);
  va_end(args);

  Serial.print(buf);
}

/*!
 * \brief Print all current measured distances
 */
void serPrintDist()
{
  int   i;
  byte  dist;

  for(i = 0; i < LaeToFMuxNumOfChan; ++i)
  {
    if( ToFSensor[i]->isBlacklisted() )
    {
      dist = LaeToFMuxRangeNoDev;
    }
    else
    {
      dist = ToFSensor[i]->getRange();
      if( dist > LaeToFMuxRangeMax )
      {
        dist = LaeToFMuxRangeNoObj;
      }
    }

    switch( dist )
    {
      case LaeToFMuxRangeNoObj:
        p(" %6s", LaeToFMuxSerArgNoObj);
        break;
      case LaeToFMuxRangeErr:
        p(" %6s", LaeToFMuxSerArgSensorErr);
        break;
      case LaeToFMuxRangeNoDev:
        p(" %6s", LaeToFMuxSerArgNoSensor);
        break;
      default:
        p(" %6d", dist);
        break;
    }
  }
}

/*!
 * \brief Print all current ambient light measurements
 */
void serPrintLux()
{
  int       i;
  float     lux;
  uint32_t  lux_int;
  uint32_t  lux_frac;

  for(i = 0; i < LaeToFMuxNumOfChan; ++i)
  {
    if( ToFSensor[i]->isBlacklisted() )
    {
      p(" %7s", LaeToFMuxSerArgNoSensor);
    }
    else
    {
      lux = ToFSensor[i]->getAmbientLight();

      lux_int = (uint32_t)lux;
      lux = lux - (float)lux_int;
      lux_frac = (uint32_t)(lux * 100.0);

      p(" %7d.%02d", lux_int, lux_frac);
    }
  }
}

/*!
 * \brief Test is character c is whitespace.
 *
 * \return Returns true or false.
 */
inline boolean whitespace(char c)
{
  return ((c == ' ') || (c == '\t'));
}

/*!
 * \brief Test is character c is end-of-line.
 *
 * \return Returns true or false.
 */
inline boolean eol(char c)
{
  return ((c == '\n') || (c == '\r'));
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
    if( SerLinePos >= LaeToFMuxSerMaxCmdLen )
    {
      SerLine[LaeToFMuxSerMaxCmdLen] = 0;
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

  SerArgc   = 0;
  len       = strlen(SerLine);
  i         = 0;

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
    while( (i < len) &&
            (j < LaeToFMuxSerMaxCmdArgLen-1) &&
            !whitespace(SerLine[i]) )
    {
      SerArgv[SerArgc][j++] = SerLine[i++];
    }

    SerArgv[SerArgc++][j] = 0;

  } while( SerArgc < LaeToFMuxSerMaxCmdArgc );

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
 * \brief Parse command get/set operator.
 *
 * \param sArg  Operator argument.
 *
 * \return Returns 'g' (get) or 's' (set) on success.
 * Prints error response and returns '?' on failure.
 */
int serParseOp(char *sArg)
{
  String  str(sArg);

  // cmd_id op ...
  if( SerArgc < 2 )
  {
    serErrorRsp("requires >= 1 args, got %d", SerArgc-1);
    return LaeToFMuxSerOpBad;
  }
  if( str == LaeToFMuxSerArgGet )
  {
    return LaeToFMuxSerOpGet;
  }
  else if( str == LaeToFMuxSerArgSet )
  {
    return LaeToFMuxSerOpSet;
  }
  else
  {
    serErrorRsp("unknown operator %s", sArg);
    return LaeToFMuxSerOpBad;
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
 * \brief Parse sensor argument.
 *
 * \param sVal          Sensor string value.
 * \param [out] sensor  Converted sensor id value.
 *
 * \return Returns true on success. On error, prints error response and returns
 * false.
 */
boolean serParseSensorId(char *sVal, int &sensor)
{
  long  n;

  if( !serParseNumber("sensor", sVal, LaeToFMuxMinChan, LaeToFMuxMaxChan, n) )
  {
    return false;
  }
  else if( ToFSensor[n]->isBlacklisted() )
  {
    serErrorRsp("sensor %s not connected", sVal);
    return false;
  }
  else
  {
    sensor = (int)n;
    return true;
  }
}

/*!
 * \brief Parse and convert tune parameter.
 *
 * \param sName       Argument name.
 * \param sVal        Argument string value to convert. Value can be:\r
 *                      '-'  ==> leave as current value\n
 *                      'r'  ==> reset to factory default\n
 *                      n    ==> numeric decimal string
 * \param nCur        Current value.
 * \param nDft        Factory default value.
 * \param nMin        Minimum value.
 * \param nMax        Maximum value.
 * \param [out] nVal  Converted value.
 *
 * \return Returns true on success. On error, prints error response and returns
 * false.
 */
boolean serParseTuneParam(const char *sName, const char *sVal,
                          long nCur,  long nDft,
                          long nMin,  long nMax,
                          long &nVal)
{
  String  str(sVal);

  if( str == LaeToFMuxSerArgStet )
  {
    nVal = nCur;
  }
  else if( str == LaeToFMuxSerArgReset )
  {
    nVal = nDft;
  }
  else if( !serParseNumber(sName, sVal, nMin, nMax, nVal) )
  {
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

  // any command turns off continuous output mode
  bMode = false;

  switch( SerArgv[SerCmdIdx][0] )
  {
    case LaeToFMuxSerCmdIdHelp:
      serExecHelp();
      break;
    case LaeToFMuxSerCmdIdGetVersion:
      serExecGetVersion();
      break;
    case LaeToFMuxSerCmdIdConfig:
      serExecConfig();
      break;
    case LaeToFMuxSerCmdIdGetIdent:
      serExecGetIdent();
      break;
    case LaeToFMuxSerCmdIdGetDist:
      serExecGetDist();
      break;
    case LaeToFMuxSerCmdIdGetLux:
      serExecGetLux();
      break;
    case LaeToFMuxSerCmdIdTunes:
      serExecTunes();
      break;
    case LaeToFMuxSerCmdIdProbe:
      serExecProbe();
      break;
    case LaeToFMuxSerCmdIdList:
      serExecList();
      break;
    case LaeToFMuxSerCmdIdCont:
      serExecCont();
      bMode = SerContinuousMode;
      break;
    case LaeToFMuxSerCmdIdReadReg:
      serExecReadReg();
      break;
    case LaeToFMuxSerCmdIdWriteReg:
      serExecWriteReg();
      break;
    case LaeToFMuxSerCmdIdDebug:
      serExecDebug();
      break;
    default:
      serErrorRsp("unknown command");
      break;
  }

  SerContinuousMode = bMode;
}

/*!
 * \brief Execute help serial command.
 */
void serExecHelp()
{
#ifdef HELP_MIN
  int i;

  char cmdids[] =
  {
    LaeToFMuxSerCmdIdGetLux,
    LaeToFMuxSerCmdIdConfig,
    LaeToFMuxSerCmdIdGetDist,
    LaeToFMuxSerCmdIdGetIdent,
    LaeToFMuxSerCmdIdList,
    LaeToFMuxSerCmdIdCont,
    LaeToFMuxSerCmdIdProbe,
    LaeToFMuxSerCmdIdReadReg,
    LaeToFMuxSerCmdIdTunes,
    LaeToFMuxSerCmdIdGetVersion,
    LaeToFMuxSerCmdIdWriteReg,
    LaeToFMuxSerCmdIdDebug,
    0
  };

  p("%c ", LaeToFMuxSerCmdIdHelp);
  for(i=0; cmdids[i]!=0; ++i)
  {
    p("%c ", cmdids[i]);
  }

  Serial.print(LaeToFMuxSerEoR);
#endif // HELP_MIN

#ifdef HELP_FULL
  const char *cmds[][2] = 
  {
    {"a",                           "get als measurements"},
    {"c {g|s} [als_state]",         "get/set firmare configuration"},
    {"d",                           "get tof measurements"},
    {"i sensor",                    "get sensor information"},
    {"l",                           "list connected sensors"},
    {"o {a|d}",                     "set continuous als/tof output"},
    {"p",                           "probe for sensors"},
    {"r sensor addr {1|2}",         "read sensor register"},
    {"t {g|s} sensor [offset xtalk gain period]",
                                    "get/set tune params"},
    {"v",                           "get firmware version"},
    {"w sensor addr {1|2} val",     "write sensor register"},
    {"x sensor {0|1}",              "debug sensor"},
    {NULL, NULL}
  };

  int   i;

  p("%c", LaeToFMuxSerEoR);

  for(i = 0; cmds[i][0] != NULL; ++i)
  {
    p("%-42s- %s%c", cmds[i][0], cmds[i][1], LaeToFMuxSerEoR);
  }
#endif // HELP_FULL

#ifdef HELP_SYN
  const char *cmds[] = 
  {
    "a",
    "c {g|s} [als_state]",
    "d",
    "i sensor",
    "l",
    "o {a|d}",
    "p",
    "r sensor addr {1|2}",
    "t {g|s} sensor [offset xtalk gain period]",
    "v",
    "w sensor addr {1|2} val",
    "x sensor {0|1}",
    NULL
  };

  int   i;

  p("%c", LaeToFMuxSerEoR);

  for(i = 0; cmds[i][0] != NULL; ++i)
  {
    p("%s%c", cmds[i], LaeToFMuxSerEoR);
  }
#endif // HELP_SYN
}

/*!
 * \brief Execute get version serial command.
 */
void serExecGetVersion()
{
  if( serChkArgCnt(LaeToFMuxSerCmdArgcGetVersion) )
  {
    serRsp("Laelaps ToFMux v%d - 1", LAE_TOF_MUX_FW_VERSION);
  }
}

/*!
 * \brief Execute get/set configuration command.
 */
void serExecConfig()
{
  boolean     ok;
  int         op;
  long        state;

  op = serParseOp(SerArgv[1]);

  switch( op )
  {
    // Get
    case LaeToFMuxSerOpGet:
      ok = serChkArgCnt(LaeToFMuxSerCmdArgcGetConfig);
      break;

    // Set
    case LaeToFMuxSerOpSet:
      if( (ok = serChkArgCnt(LaeToFMuxSerCmdArgcSetConfig)) )
      {
        ok = serParseNumber("als_state", SerArgv[2], 0, 1, state);
      }

      if( ok )
      {
#ifdef LAE_USE_ALS
        if( state == 0 )
        {
          AlsSensorId = -1;
        }
        else
        {
          AlsSensorId = AlsSensorId < 0? nextSensor(0): AlsSensorId;
        }
#endif // LAE_USE_ALS
      }
      break;

    // Bad
    case LaeToFMuxSerOpBad:
    default:
      ok = false;
      break;
  }

  // successful command
  if( ok )
  {
    if( AlsSensorId < 0 )
    {
      serRsp("%s", LaeToFMuxSerArgOff);
    }
    else
    {
      serRsp("%s", LaeToFMuxSerArgOn);
    }
  }
}

/*!
 * \brief Execute get sensor identify serial command.
 */
void serExecGetIdent()
{
  int                   sensor;
  VL6180xIdentification ident;
 
  if( !serChkArgCnt(LaeToFMuxSerCmdArgcGetIdent) )
  {
    return;
  }
  else if( !serParseSensorId(SerArgv[1], sensor) ) 
  {
    return;
  }
  else
  {
    ToFSensor[sensor]->getIdent(&ident);

    serRsp("0x%02x %d.%d %d.%d %d %d",
      ident.idModel,
      ident.idModelRevMajor, ident.idModelRevMinor,
      ident.idModuleRevMajor, ident.idModuleRevMinor,
      ident.idDate, ident.idTime);
  }
}

/*!
 * \brief Execute get distance measurements serial command.
 */
void serExecGetDist()
{
  if( !serChkArgCnt(LaeToFMuxSerCmdArgcGetDist) )
  {
    return;
  }

  Serial.print(SerArgv[SerCmdIdx]);

  serPrintDist();

  Serial.print(LaeToFMuxSerEoR);
}

/*!
 * \brief Execute get ambient light measurements serial command.
 */
void serExecGetLux()
{
  if( !serChkArgCnt(LaeToFMuxSerCmdArgcGetLux) )
  {
    return;
  }

  Serial.print(SerArgv[SerCmdIdx]);

  serPrintLux();

  Serial.print(LaeToFMuxSerEoR);
}

/*!
 * \brief Execute get/set sensor tune parameters serial command.
 */
void serExecTunes()
{
  int       op;
  int       sensor;
  byte      rangeOffset, rangeOffsetDft;
  uint16_t  rangeCrossTalk, rangeCrossTalkDft;
  byte      alsGain, alsGainDft;
  uint16_t  alsIntPeriod, alsIntPeriodDft;
  long      val;

  op = serParseOp(SerArgv[1]);

  switch( op )
  {
    // Get
    case LaeToFMuxSerOpGet:
      if( !serChkArgCnt(LaeToFMuxSerCmdArgcGetTunes) )
      {
        return;
      }
      break;

    // Set
    case LaeToFMuxSerOpSet:
      if( !serChkArgCnt(LaeToFMuxSerCmdArgcSetTunes) )
      {
        return;
      }
      break;

    // Bad
    case LaeToFMuxSerOpBad:
    default:
      return;
  }

  if( !serParseSensorId(SerArgv[2], sensor) )
  {
    return;
  }

  // get factory default tune values
  ToFSensor[sensor]->getDefaultTunes(rangeOffsetDft, rangeCrossTalkDft,
                                     alsGainDft,     alsIntPeriodDft);

  // get current tune values
  ToFSensor[sensor]->getTunes(rangeOffset, rangeCrossTalk,
                              alsGain,     alsIntPeriod);


  if( op == LaeToFMuxSerOpSet )
  {
    // tof range sensor offset
    if( !serParseTuneParam("tof_offset", SerArgv[3],
                        rangeOffset, rangeOffsetDft,
                        VL6180X_RANGE_OFFSET_MIN, VL6180X_RANGE_OFFSET_MIN,
                        val) )
    {
      return;
    }

    rangeOffset = (byte)val;

    // tof range sensor cross talk
    if( !serParseTuneParam("tof_cross_talk", SerArgv[4],
                          rangeCrossTalk, rangeCrossTalkDft,
                          VL6180X_RANGE_XTALK_MIN, VL6180X_RANGE_XTALK_MIN,
                          val) )
    {
      return;
    }

    rangeCrossTalk = (uint16_t)val;

    // ambient light sensor gain
    if( !serParseTuneParam("als_gain", SerArgv[5],
                          alsGain, alsGainDft,
                          VL6180X_AMBIENT_GAIN_MIN, VL6180X_AMBIENT_GAIN_MAX,
                          val) )
    {
      return;
    }

    alsGain = (byte)val;
        
    if( !serParseTuneParam("als_int_period", SerArgv[6],
                          alsIntPeriod, alsIntPeriodDft,
                          VL6180X_AMBIENT_INT_T_MIN, VL6180X_AMBIENT_INT_T_MAX,
                          val) )
    {
      return;
    }

    alsIntPeriod = (uint16_t)val;

    // mark changes for tuning
    ToFSensor[sensor]->markRangeForTuning(rangeOffset, rangeCrossTalk);
    ToFSensor[sensor]->markAlsForTuning(alsGain, alsIntPeriod);
  }


  // successful command
  // Note:  The tune parameters are updated asynchronously, so the current
  //        values may not match the target set values yet.
  serRsp("%u %u %u %u", rangeOffset, rangeCrossTalk, alsGain, alsIntPeriod);
}

/*!
 * \brief Execute probe and initialize sensors serial command.
 */
void serExecProbe()
{
  if( !serChkArgCnt(LaeToFMuxSerCmdArgcProbe) )
  {
    return;
  }

  probe();

  serRsp("%d", ToFNumConn);
}

/*!
 * \brief Execute list sensor connection status serial command.
 */
void serExecList()
{
  int   i;

  if( !serChkArgCnt(LaeToFMuxSerCmdArgcList) )
  {
    return;
  }

  Serial.print(SerArgv[SerCmdIdx]);

  for(i = 0; i < LaeToFMuxNumOfChan; ++i)
  {
    if( ToFSensor[i]->isBlacklisted() )
    {
      p(" %s", LaeToFMuxSerArgNoSensor);
    }
    else
    {
      p(" %d", i);
    }
  }

  Serial.print(LaeToFMuxSerEoR);
}

/*!
 * \brief Execute continous measurement output serial command.
 */
void serExecCont()
{
  String  str;

  if( !serChkArgCnt(LaeToFMuxSerCmdArgcCont) )
  {
    return; 
  }

  str = SerArgv[1];

  if( str == "a" )
  {
    SerContinuousOutput = 'a';
  }
  else if( str == "d" )
  {
    SerContinuousOutput = 'd';
  }
  else
  {
    serErrorRsp("%s bad output mode", str.c_str());
    return;
  }

  SerContinuousMode = true;

  serRsp("%s", str.c_str());
}

/*!
 * \brief Generate continuous measurement output.
 */
void serContinuousOutput()
{
  if( SerContinuousOutput == 'a' )
  {
    //serRsp("%d %d %d", AlsSensorId, AlsCounter, AlsFreq);
    serPrintLux();
  }
  else if( SerContinuousOutput == 'd' )
  {
    serPrintDist();
  }

  Serial.print(LaeToFMuxSerEoR);
}

/*!
 * \brief Execute read sensor register command.
 */
void serExecReadReg()
{
  int       sensor;
  long      regAddr;
  long      regSize;
  uint16_t  regVal;
 
  if( !serChkArgCnt(LaeToFMuxSerCmdArgcReadReg) )
  {
    return;
  }
  else if( !serParseSensorId(SerArgv[1], sensor) ) 
  {
    return;
  }
  else if( !serParseNumber("addr", SerArgv[2],
        VL6180X_IDENTIFICATION_MODEL_ID, VL6180X_INTERLEAVED_MODE_ENABLE,
        regAddr) )
  {
    return;
  }
  else if( !serParseNumber("size", SerArgv[3], 1, 2, regSize) )
  {
    return;
  }

  switch( regSize )
  {
    case 2:
      regVal = ToFSensor[sensor]->readReg16((uint16_t)regAddr);
      serRsp("0x%04x", regVal);
      break;
    case 1:
    default:
      regVal = (uint16_t)ToFSensor[sensor]->readReg8((uint16_t)regAddr);
      serRsp("0x%02x", regVal);
      break;
  }
}

/*!
 * \brief Execute write sensor register command.
 */
void serExecWriteReg()
{
  int       sensor;
  long      regAddr;
  long      regSize;
  long      regWVal;
  uint16_t  regRVal;
 
  if( !serChkArgCnt(LaeToFMuxSerCmdArgcWriteReg) )
  {
    return;
  }
  else if( !serParseSensorId(SerArgv[1], sensor) ) 
  {
    return;
  }
  else if( !serParseNumber("addr", SerArgv[2],
        VL6180X_IDENTIFICATION_MODEL_ID, VL6180X_INTERLEAVED_MODE_ENABLE,
        regAddr) )
  {
    return;
  }
  else if( !serParseNumber("size", SerArgv[3], 1, 2, regSize) )
  {
    return;
  }
  else if( !serParseNumber("val", SerArgv[4], 0, 0xffff, regWVal) )
  {
    return;
  }

  switch( regSize )
  {
    case 2:
      ToFSensor[sensor]->writeReg16((uint16_t)regAddr, (uint16_t)regWVal);
      regRVal = ToFSensor[sensor]->readReg16((uint16_t)regAddr);
      serRsp("0x%04x", regRVal);
      break;
    case 1:
    default:
      ToFSensor[sensor]->writeReg8((uint16_t)regAddr, (byte)regWVal);
      regRVal = (uint16_t)ToFSensor[sensor]->readReg8((uint16_t)regAddr);
      serRsp("0x%02x", regRVal);
      break;
  }
}

/*!
 * \brief Execute debug sensor.
 */
void serExecDebug()
{
  int     sensor;
  long    enable;

  if( !serChkArgCnt(LaeToFMuxSerCmdArgcDebug) )
  {
    return;
  }
  else if( !serParseSensorId(SerArgv[1], sensor) ) 
  {
    return;
  }
  else if( !serParseNumber("enable", SerArgv[2], 0, 1, enable) )
  {
    return;
  }

  ToFSensor[sensor]->debug(enable==1);

  serRsp("%d", enable);
}
