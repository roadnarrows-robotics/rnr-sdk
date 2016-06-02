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
 * $LastChangedDate$
 * $Rev$
 *
 * \brief Laelaps time-of-flight multiplexor Arduino compatible firmware.
 *
 * The firmware is an I2C slave device to the host and an I2C master to the
 * ToF sensors. Each sensor is attached to a software bit-banged I2C port.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
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
#define LAE_ARDUINO             1 ///< arduino target 
#define LAE_TOF_MUX_FW_VERSION  2 ///< firmware version

//
// Bridge between standard POSIX C/C++ Linux and Arduino constructs.
//
typedef byte byte_t;

// Interface between firmware and software.
#include "laeToFMux.h"

using namespace laelaps;

#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <Wire.h>
#include <SoftwareWire.h>

#include "VL6180X.h"

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
char      SerArgs[LaeToFMuxSerMaxCmdArgs][LaeToFMuxSerMaxCmdArgLen];
int       SerArgCnt;
const int SerCmdIdx = 0;
boolean   SerContinuousMode;
byte      SerContinuousOutput;


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
    ToFSensor[i] = new VL6180x(*SoftWire[i]);
  }

  //
  // Probe and initialize sensors and class object data.
  //
  probe();

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
  SerArgCnt         = 0;
  SerContinuousMode = false;
  Serial.begin(115200);
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
    if( ToFSensor[i]->ping(3) )
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
    // measurement.
    //
    if( AlsSensorId == i )
    {
      //
      // Scheduled.
      //
      if( AlsCounter > 0 )
      {
        // continue taking distance measurements
        if( ToFSensor[i]->asyncMeasureRange() )
        {
          --AlsCounter; // when done decrement ALS counter.
        }
      }

      //
      // Active.
      //
      else
      {
        // push ambient light measurement until done
        if( ToFSensor[i]->asyncMeasureAmbientLight() )
        {
          // when done move to next sensor
          AlsSensorId = nextSensor(AlsSensorId);
          AlsCounter  = AlsFreq;
        }
      }
    }

    //
    // This sensor is not scheduled or is actively taking an ambient light
    // measurement. So take a distance measurement.
    //
    else
    {
      ToFSensor[i]->asyncMeasureRange();
    }
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

  sprintf(buf, "%s ", SerArgs[SerCmdIdx]);
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

  sprintf(buf, "%s %s ", LaeToFMuxSerArgErrRsp, SerArgs[SerCmdIdx]);
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
        p(" %6s", LaeToFMuxSerArgNotPresent);
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
      p(" %7s", LaeToFMuxSerArgNotPresent);
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

  SerArgCnt = 0;
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
      SerArgs[SerArgCnt][j++] = SerLine[i++];
    }

    SerArgs[SerArgCnt++][j] = 0;

  } while( SerArgCnt < LaeToFMuxSerMaxCmdArgs );

  return SerArgCnt > 0;
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
  int   argCnt;

  argCnt = SerArgCnt - 1;

  if( argCnt != nExpected )
  {
    serErrorRsp("requires %d args, got %d", nExpected, argCnt);
    return false;
  }

  return true;
}

/*!
 * \brief Parse command get/set operator.
 *
 * \param sArg  Operator argument.
 *
 * \return Returns 'g' (get) or 's' (set) on success. Prints error response
 * and returns '?' on failure.
 */
int serParseOp(char *sArg)
{
  String  str(sArg);

  if( str == LaeToFMuxSerArgGet )
  {
    return 'g';
  }
  else if( str == LaeToFMuxSerArgSet )
  {
    return 's';
  }
  else
  {
    serErrorRsp("unknown operator %s", sArg);
    return '?';
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
  String      str = "";
  const char *s;
  char        c;

  if( (sVal == NULL) || (*sVal == 0) )
  {
    serErrorRsp("no %s argument", sName);
    return false;
  }

  for(s = sVal; *s != 0; ++s)
  {
    c = *s;
    if( isDigit(c) )
    {
      str += c;
    }
    else
    {
      serErrorRsp("%s value %s is not a number", sName, sVal);
      return false;
    }
  }
    
  nVal = str.toInt();

  if( (nVal < nMin) || (nVal > nMax) )
  {
    serErrorRsp("%s value %s out-of-range", sName, sVal);
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

  if( strlen(SerArgs[SerCmdIdx]) > 1 )
  {
    serErrorRsp("bad command");
    return;
  }

  // any command turns off continuous output mode
  bMode = false;

  switch( SerArgs[SerCmdIdx][0] )
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
    case LaeToFMuxSerCmdIdDebug:
      break;  // TODO
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
  serRsp("%c %c %c %c %c %c %c %c %c",
      LaeToFMuxSerCmdIdGetLux,
      LaeToFMuxSerCmdIdConfig,
      LaeToFMuxSerCmdIdGetDist,
      LaeToFMuxSerCmdIdGetIdent,
      LaeToFMuxSerCmdIdList,
      LaeToFMuxSerCmdIdCont,
      LaeToFMuxSerCmdIdProbe,
      LaeToFMuxSerCmdIdTunes,
      LaeToFMuxSerCmdIdGetVersion);

#if 0 // RDK not sufficient memory
  serRsp("%s 10", SerArgs[SerCmdIdx]);
  p("%c                       - get ambient light measurements from all sensors%s",
      LaeToFMuxSerCmdIdGetLux, LaeToFMuxSerEoR);
  p("%c <op> [cfg]            - get/set firmware operation%s",
      LaeToFMuxSerCmdIdConfig, LaeToFMuxSerEoR);
  p("%c                       - get distance measurements from all sensors%s",
      LaeToFMuxSerCmdIdGetDist, LaeToFMuxSerEoR);
  p("%c                       - print this help%s",
      LaeToFMuxSerCmdIdHelp, LaeToFMuxSerEoR);
  p("%c <sensor>              - get ToF sensor identify%s",
      LaeToFMuxSerCmdIdGetIdent, LaeToFMuxSerEoR);
  p("%c                       - list sensor connected state%s",
      LaeToFMuxSerCmdIdList, LaeToFMuxSerEoR);
  p("%c <output>              - enable continuous output mode%s",
      LaeToFMuxSerCmdIdCont, LaeToFMuxSerEoR);
  p(%s"%c                       - probe for connected sensors%s",
      LaeToFMuxSerCmdIdProbe, LaeToFMuxSerEoR);
  p("%c <op> <sensor> [tunes] - get/set ToF sensor tune parameters%s",
      LaeToFMuxSerCmdIdTunes, LaeToFMuxSerEoR);
  p("%c                       - get firmware version%s",
      LaeToFMuxSerCmdIdGetVersion, LaeToFMuxSerEoR);
#endif // RDK
}

/*!
 * \brief Execute get version serial command.
 */
void serExecGetVersion()
{
  if( serChkArgCnt(LaeToFMuxSerCmdArgsGetVersion) )
  {
    serRsp("%d", LAE_TOF_MUX_FW_VERSION);
  }
}

/*!
 * \brief Execute get/set configuration command.
 */
void serExecConfig()
{
  int   op;
  long  state;

  if( (SerArgCnt - 1) == 0 )
  {
    serErrorRsp("no operator");
  }
  else if( (op = serParseOp(SerArgs[1])) == '?' )
  {
    return;
  }

  if( op == 'g' )
  {
    if( !serChkArgCnt(LaeToFMuxSerCmdArgsGetConfig) )
    {
      return;
    }
  }
  else if( op == 's' )
  {
    if( !serChkArgCnt(LaeToFMuxSerCmdArgsSetConfig) )
    {
      return;
    }
    
    if( !serParseNumber("als_state", SerArgs[2], 0, 1, state) )
    {
      return;
    }

    if( state == 0 )
    {
      AlsSensorId = -1;
    }
    else
    {
      AlsSensorId = AlsSensorId < 0? nextSensor(0): AlsSensorId;
    }
  }

  if( AlsSensorId < 0 )
  {
    serRsp("%s", LaeToFMuxSerArgOff);
  }
  else
  {
    serRsp("%s", LaeToFMuxSerArgOn);
  }
}

/*!
 * \brief Execute get sensor identify serial command.
 */
void serExecGetIdent()
{
  int                   sensor;
  VL6180xIdentification ident;
 
  if( !serChkArgCnt(LaeToFMuxSerCmdArgsGetIdent) )
  {
    return;
  }
  else if( !serParseSensorId(SerArgs[1], sensor) ) 
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
  if( !serChkArgCnt(LaeToFMuxSerCmdArgsGetDist) )
  {
    return;
  }

  Serial.print(SerArgs[SerCmdIdx]);

  serPrintDist();

  Serial.print(LaeToFMuxSerEoR);
}

/*!
 * \brief Execute get ambient light measurements serial command.
 */
void serExecGetLux()
{
  if( !serChkArgCnt(LaeToFMuxSerCmdArgsGetLux) )
  {
    return;
  }

  Serial.print(SerArgs[SerCmdIdx]);

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

  if( (SerArgCnt - 1) == 0 )
  {
    serErrorRsp("no operator");
    return;
  }
  else if( (op = serParseOp(SerArgs[1])) == '?' )
  {
    return;
  }
  else if( !serParseSensorId(SerArgs[2], sensor) )
  {
    return;
  }

  // get current tune values
  ToFSensor[sensor]->getTunes(rangeOffset, rangeCrossTalk,
                              alsGain,     alsIntPeriod);

  //
  // get
  //
  if( op == 'g' )
  {
    if( serChkArgCnt(LaeToFMuxSerCmdArgsGetTunes) )
    {
      serRsp("%u %u %u %u", rangeOffset, rangeCrossTalk, alsGain, alsIntPeriod);
    }
    return;
  }

  //
  // set
  //

  if( !serChkArgCnt(LaeToFMuxSerCmdArgsSetTunes) )
  {
    return;
  }

  // get factory default tune values
  ToFSensor[sensor]->getDefaultTunes(rangeOffsetDft, rangeCrossTalkDft,
                                     alsGainDft,     alsIntPeriodDft);

  //
  // parse tune arguments
  //

  if( serParseTuneParam("tof_offset", SerArgs[3],
                          rangeOffset, rangeOffsetDft,
                          VL6180X_RANGE_OFFSET_MIN, VL6180X_RANGE_OFFSET_MIN,
                          val) )
  {
    rangeOffset = (byte)val;
  }
  else
  {
    return;
  }

  if( serParseTuneParam("tof_cross_talk", SerArgs[4],
                          rangeCrossTalk, rangeCrossTalkDft,
                          VL6180X_RANGE_XTALK_MIN, VL6180X_RANGE_XTALK_MIN,
                          val) )
  {
    rangeCrossTalk = (uint16_t)val;
  }
  else
  {
    return;
  }

  if( serParseTuneParam("als_gain", SerArgs[5],
                          alsGain, alsGainDft,
                          VL6180X_AMBIENT_GAIN_MIN, VL6180X_AMBIENT_GAIN_MAX,
                          val) )
  {
    alsGain = (byte)val;
  }
  else
  {
    return;
  }

  if( serParseTuneParam("als_int_period", SerArgs[6],
                          alsIntPeriod, alsIntPeriodDft,
                          VL6180X_AMBIENT_INT_T_MIN, VL6180X_AMBIENT_INT_T_MAX,
                          val) )
  {
    alsIntPeriod = (uint16_t)val;
  }
  else
  {
    return;
  }

  ToFSensor[sensor]->markRangeForTuning(rangeOffset, rangeCrossTalk);
  ToFSensor[sensor]->markAlsForTuning(alsGain, alsIntPeriod);
  
  //
  // Get new current tune values. Note that tune parameters are updated 
  // asynchronously, so the current value may not match the target set values
  // yet.
  //
  ToFSensor[sensor]->getTunes(rangeOffset, rangeCrossTalk,
                              alsGain,     alsIntPeriod);

  serRsp("%u %u %u %u", rangeOffset, rangeCrossTalk, alsGain, alsIntPeriod);
}

/*!
 * \brief Execute probe and initialize sensors serial command.
 */
void serExecProbe()
{
  if( !serChkArgCnt(LaeToFMuxSerCmdArgsProbe) )
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

  if( !serChkArgCnt(LaeToFMuxSerCmdArgsList) )
  {
    return;
  }

  Serial.print(SerArgs[SerCmdIdx]);

  for(i = 0; i < LaeToFMuxNumOfChan; ++i)
  {
    if( ToFSensor[i]->isBlacklisted() )
    {
      p(" %s", LaeToFMuxSerArgNotPresent);
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

  if( !serChkArgCnt(LaeToFMuxSerCmdArgsCont) )
  {
    return; 
  }

  str = SerArgs[1];

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
    serRsp("%d %d %d", AlsSensorId, AlsCounter, AlsFreq);
    //serPrintLux();
  }
  else if( SerContinuousOutput == 'd' )
  {
    serPrintDist();
  }

  Serial.print(LaeToFMuxSerEoR);
}
