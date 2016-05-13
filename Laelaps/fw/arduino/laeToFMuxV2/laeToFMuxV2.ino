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
// Using pin 2 (software sda) and 3 (software scl) in this example.
//
SoftwareWire softWire0( 2,  3, true, false);
SoftwareWire softWire1( 4,  5, true, false);
SoftwareWire softWire2( 6,  7, true, false);
SoftwareWire softWire3( 8,  9, true, false);
SoftwareWire softWire4(10, 11, true, false);
SoftwareWire softWire5(12, 13, true, false);
SoftwareWire softWire6(A0, A1, true, false);
SoftwareWire softWire7(A2, A3, true, false);

SoftwareWire *SoftWire[LaeToFMuxNumOfChan] =
{
  &softWire0, &softWire1, &softWire2, &softWire3,
  &softWire4, &softWire5, &softWire6, &softWire7
};

//
// The Sensors
//
#define ALS_FREQ 2

//
// Sensors
//
VL6180x  *ToFSensor[LaeToFMuxNumOfChan];
int       AlsSensorId;
int       AlsFreqCnt;

//
// I2C Response data
//
byte        RspBuf[LaeToFMuxMaxRspLen];         ///< response buffer
int         RspLen;                             ///< response length
const byte  RspErrorBuf[LaeToFMuxMaxRspLen] =   ///< response error pattern
{
  0xde, 0xad, 0xfa, 0xce, 0xba, 0xad, 0xf0, 0x0d, 0,
};

//
// Serial CLI
//
char      SerLine[LaeToFMuxSerMaxCmdLen+1];
int       SerLinePos;
char      SerArgs[LaeToFMuxSerMaxCmdArgs][LaeToFMuxSerMaxCmdArgLen];
int       SerArgCnt;
const int SerCmdIdx = 0;
boolean   SerContinuousMode;

//------------------------------------------------------------------------------
// Arduino Hook Functions
//------------------------------------------------------------------------------

void setup()
{
  int   i;

  //
  // Begin software wires
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
  // Probe and initialize sensors.
  //
  probe();

  AlsFreqCnt  = ALS_FREQ;

  //
  // Response
  //
  RspLen = 0;

  //
  // I2C slave
  //
  Wire.begin(LaeI2CAddrToFMux);
  Wire.onReceive(receiveCmd);
  Wire.onRequest(sendRsp);

  //
  // Serial command-line interface (baud does not matter over USB) 
  // 
  SerLinePos        = 0;
  SerArgCnt         = 0;
  SerContinuousMode = false;
  Serial.begin(115200);
}

void loop()
{
  // take distance measurements from all connected sensors
  measureRanges();

  // take one ambient light measurement
  if( --AlsFreqCnt <= 0 )
  {
    measureAmbient();
    nextAmbientSensor();
  }

  if( SerContinuousMode )
  {
    serContinuousOutput();
  }

  // process any serial input
  if( serRcvCmd() )
  {
    if( serParseCmd() )
    {
      serExecCmd();
    }
  }
}

void probe()
{
  int   i;

  for(i = 0; i < LaeToFMuxNumOfChan; ++i)
  {
    if( ToFSensor[i]->ping(3) )
    {
      ToFSensor[i]->readIdent();
      ToFSensor[i]->initSensor();
      ToFSensor[i]->writeSensorDefaults();
      ToFSensor[i]->readTunes();
      AlsSensorId = i;
    }
    else
    {
      ToFSensor[i]->blacklist();
    }
  }
}


//------------------------------------------------------------------------------
// Master Interface to Sensors
//------------------------------------------------------------------------------

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

void measureAmbient()
{
  float flux;

  if( !ToFSensor[AlsSensorId]->isBlacklisted() )
  {
    flux = ToFSensor[AlsSensorId]->measureAmbientLight();
    delay(5);
  }
}

void nextAmbientSensor()
{
  int   i;

  AlsFreqCnt  = ALS_FREQ;
  AlsSensorId = (AlsSensorId + 1) % LaeToFMuxNumOfChan;

  for(i = 0; i < LaeToFMuxNumOfChan; ++i)
  {
    if( !ToFSensor[AlsSensorId]->isBlacklisted() )
    {
      break;
    }
    AlsSensorId = (AlsSensorId + 1) % LaeToFMuxNumOfChan;
  }
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
      case LaeToFMuxCmdIdGetVersion:
        bOk = execGetVersion();
        break;
      case LaeToFMuxCmdIdGetIdent:
        bOk = execGetIdent();
        break;
      case LaeToFMuxCmdIdGetRanges:
        bOk = execGetRanges();
        break;
      case LaeToFMuxCmdIdGetLux:
        bOk = execGetAmbients();
        break;
      case LaeToFMuxCmdIdTuneRangeSensor:
        bOk = execTuneRange();
        break;
      case LaeToFMuxCmdIdTuneAls:
        bOk = execTuneAls();
        break;
      case LaeToFMuxCmdIdGetTunes:
        bOk = execGetTunes();
        break;
      default:
        flushRead();
        bOk = false;
    }
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
 * \brief Execute I2C command to get the firmware's version number.
 *
 * \return Returns true on success, false on failure.
 */
boolean execGetVersion()
{
  RspBuf[RspLen++] = (byte)LAE_TOF_MUX_FW_VERSION;
  return true;
}

boolean execGetIdent()
{
  byte                  len = LaeToFMuxCmdLenGetIdent - 1; 
  byte                  sensor;
  VL6180xIdentification ident;

  if( Wire.available() == len )
  {
    sensor = Wire.read();

    if( sensor < LaeToFMuxNumOfChan ) 
    {
      ToFSensor[sensor]->getIdent(&ident);

      RspBuf[RspLen++] = (byte)ident.idModel;
      RspBuf[RspLen++] = (byte)ident.idModelRevMajor;
      RspBuf[RspLen++] = (byte)ident.idModelRevMinor;
      RspBuf[RspLen++] = (byte)ident.idModuleRevMajor;
      RspBuf[RspLen++] = (byte)ident.idModuleRevMinor;
      RspBuf[RspLen++] = (byte)(ident.idDate >> 8);
      RspBuf[RspLen++] = (byte)(ident.idDate & 0xff);
      RspBuf[RspLen++] = (byte)(ident.idTime >> 8);
      RspBuf[RspLen++] = (byte)(ident.idTime & 0xff);

      return true;
    }
  }

  // error
  errorRsp(LaeToFMuxRspLenGetIdent);

  return false;
}

boolean execGetRanges()
{
  int   i;
  byte  dist;

  for(i = 0; i < LaeToFMuxNumOfChan; ++i)
  {
    if( ToFSensor[i]->isBlacklisted() )
    {
      dist = LaeToFMuxArgRangeNoDev;
    }
    else
    {
      dist = ToFSensor[i]->getRange();
      if( dist > LaeToFMuxArgRangeMax )
      {
        dist = LaeToFMuxArgRangeNoObj;
      }
    }

    RspBuf[RspLen++] = dist;
  }

  return true;
}

boolean execGetAmbients()
{
  int       i;
  float     lux;
  uint32_t  val;

  for(i = 0; i < LaeToFMuxNumOfChan; ++i)
  {
    if( ToFSensor[i]->isBlacklisted() )
    {
      lux = LaeToFMuxArgLuxNoLight;
    }
    else
    {
      lux = ToFSensor[i]->getAmbientLight();
    }

    val = (uint32_t)(lux * LaeToFMuxArgLuxMult);

    RspBuf[RspLen++] = (byte)((val >> 24) & 0xff);
    RspBuf[RspLen++] = (byte)((val >> 16) & 0xff);
    RspBuf[RspLen++] = (byte)((val >>  8) & 0xff);
    RspBuf[RspLen++] = (byte)(val & 0xff);
  }

  return true;
}

boolean execTuneRange()
{
  byte  len = LaeToFMuxCmdLenTuneRangeSensor - 1; 
  byte  sensor;
  byte  offset;
  byte  crosstalk;

  if( Wire.available() == len )
  {
    sensor    = Wire.read();
    offset    = Wire.read();
    crosstalk = Wire.read();

    if( sensor < LaeToFMuxNumOfChan ) 
    {
      ToFSensor[sensor]->markRangeForTuning(offset, crosstalk);
      return true;
    }
  }

  // error
  errorRsp(LaeToFMuxRspLenGetIdent);

  return false;
}

boolean execTuneAls()
{
  byte      len = LaeToFMuxCmdLenTuneAls - 1; 
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
  errorRsp(LaeToFMuxRspLenGetIdent);

  return false;
}

boolean execGetTunes()
{
  byte      len = LaeToFMuxCmdLenGetTunes - 1; 
  byte      sensor;
  byte      offset;
  byte      crosstalk;
  byte      gain;
  uint16_t  intPeriod;
  uint16_t  val_hi, val_lo;

  if( Wire.available() == len )
  {
    sensor = Wire.read();

    if( sensor < LaeToFMuxNumOfChan ) 
    {
      ToFSensor[sensor]->getTunes(offset, crosstalk, gain, intPeriod);

      val_hi    = (byte)(intPeriod >> 8);
      val_lo    = (byte)(intPeriod & 0xff);

      RspBuf[RspLen++] = offset;
      RspBuf[RspLen++] = crosstalk;
      RspBuf[RspLen++] = gain;
      RspBuf[RspLen++] = val_hi;
      RspBuf[RspLen++] = val_lo;

      return true;
    }
  }

  // error
  errorRsp(LaeToFMuxRspLenGetTunes);

  return false;
}


//------------------------------------------------------------------------------
// Serial Functions
//------------------------------------------------------------------------------

void p(char *fmt, ... )
{
  va_list args;
  char    buf[LaeToFMuxSerMaxRspLen];

  va_start(args, fmt);
  vsnprintf(buf, LaeToFMuxSerMaxRspLen, fmt, args);
  va_end(args);

  Serial.print(buf);
}

void rsp(char *fmt, ... )
{
  va_list args;
  char    buf[LaeToFMuxSerMaxRspLen];

  va_start(args, fmt);
  vsnprintf(buf, LaeToFMuxSerMaxRspLen, fmt, args);
  va_end(args);

  Serial.print(buf);
  Serial.print(LaeToFMuxSerEoR);
}

inline boolean whitespace(char c)
{
  return ((c == ' ') || (c == '\t'));
}

inline boolean eol(char c)
{
  return ((c == '\n') || (c == '\r'));
}

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

int serParseSensorNumber(char *s)
{
  int   n;
  char *endptr;

  if( (s == NULL) || (*s == 0) )
  {
    rsp("%s %s no sensor argument", LaeToFMuxSerArgErrRsp, SerArgs[SerCmdIdx]);
    return -1;
  }

  n = (int)strtol(s, &endptr, 0);

  if( *endptr != 0 )
  {
    rsp("%s %s %s not a number",
        LaeToFMuxSerArgErrRsp, SerArgs[SerCmdIdx], s);
    return -1;
  }
  else if( (n < LaeToFMuxMinChan) || (n > LaeToFMuxMaxChan) )
  {
    rsp("%s %s %s sensor number out-of-range",
        LaeToFMuxSerArgErrRsp, SerArgs[SerCmdIdx], s);
    return -1;
  }
  else if( ToFSensor[n]->isBlacklisted() )
  {
    rsp("%s %s %s no sensor connected",
        LaeToFMuxSerArgErrRsp, SerArgs[SerCmdIdx], s);
    return -1;
  }
  else
  {
    return n;
  }
}

boolean serChkArgCnt(int nExpected)
{
  if( (SerArgCnt-1) != nExpected )
  {
    rsp("%s %s require %d args, got %d",
        LaeToFMuxSerArgErrRsp, SerArgs[SerCmdIdx], nExpected, SerArgCnt-1);
    return false;
  }

  return true;
}

void serExecCmd()
{
  char    cmdId;
  boolean bMode;

  if( strlen(SerArgs[SerCmdIdx]) > 1 )
  {
    rsp("%s %s bad command", LaeToFMuxSerArgErrRsp, SerArgs[SerCmdIdx]);
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
    case LaeToFMuxSerCmdIdGetIdent:
      serExecGetIdent();
      break;
    case LaeToFMuxSerCmdIdGetDist:
      serExecGetDist();
      break;
    case LaeToFMuxSerCmdIdGetLux:
      serExecGetLux();
      break;
    case LaeToFMuxSerCmdIdGetTunes:
      serExecGetTunes();
      break;
    case LaeToFMuxSerCmdIdProbe:
      serExecProbe();
      break;
    case LaeToFMuxSerCmdIdList:
      serExecList();
      break;
    case LaeToFMuxSerCmdIdCont:
      serExecCont();
      bMode = SerContinuousMode; // this command toggles continuous mode
      break;
    default:
      rsp("%s %s unknown command", LaeToFMuxSerArgErrRsp, SerArgs[SerCmdIdx]);
      break;
  }

  SerContinuousMode = bMode;
}

void serExecHelp()
{
  rsp("%c          - get ambient light measurements from all sensors",
                           LaeToFMuxSerCmdIdGetLux);
  rsp("%c          - turn on/off continuous output mode",
                           LaeToFMuxSerCmdIdCont);
  rsp("%c          - get distance measurements from all sensors",
                           LaeToFMuxSerCmdIdGetDist);
  rsp("%c <sensor> - get ToF sensor identify", LaeToFMuxSerCmdIdGetIdent);
  rsp("%c          - list sensor connected state",
                           LaeToFMuxSerCmdIdProbe);
  rsp("%c          - probe for connected sensors",
                           LaeToFMuxSerCmdIdProbe);
  rsp("%c <sensor> - get ToF sensor tune parameters",
                            LaeToFMuxSerCmdIdGetTunes);
  rsp("%c          - get firmware version", LaeToFMuxSerCmdIdGetVersion);

  rsp("");
  rsp("%c          - print this help", LaeToFMuxSerCmdIdHelp);
}

void serExecGetVersion()
{
  if( serChkArgCnt(LaeToFMuxSerCmdArgsGetVersion) )
  {
    rsp("%s %d", SerArgs[SerCmdIdx], LAE_TOF_MUX_FW_VERSION);
  }
}

void serExecGetIdent()
{
  int                   sensor;
  VL6180xIdentification ident;
 
  if( !serChkArgCnt(LaeToFMuxSerCmdArgsGetIdent) )
  {
    return;
  }
  else if( (sensor = serParseSensorNumber(SerArgs[1])) < 0 )
  {
    return;
  }
  else
  {
    ToFSensor[sensor]->getIdent(&ident);

    rsp("%s 0x%02x %d.%d %d.%d, %d, %d",
      SerArgs[SerCmdIdx],
      ident.idModel,
      ident.idModelRevMajor, ident.idModelRevMinor,
      ident.idModuleRevMajor, ident.idModuleRevMinor,
      ident.idDate,
      ident.idTime);
  }
}

void serExecGetDist()
{
  int   i;
  byte  dist;

  if( !serChkArgCnt(LaeToFMuxSerCmdArgsGetDist) )
  {
    return;
  }

  Serial.print(SerArgs[SerCmdIdx]);

  for(i = 0; i < LaeToFMuxNumOfChan; ++i)
  {
    if( ToFSensor[i]->isBlacklisted() )
    {
      dist = LaeToFMuxArgRangeNoDev;
    }
    else
    {
      dist = ToFSensor[i]->getRange();
    }

    switch( dist )
    {
      case LaeToFMuxArgRangeNoObj:
        p(" %6s", LaeToFMuxSerArgNoObj);
        break;
      case LaeToFMuxArgRangeErr:
        p(" %6s", LaeToFMuxSerArgSensorErr);
        break;
      case LaeToFMuxArgRangeNoDev:
        p(" %6s", LaeToFMuxSerArgNotPresent);
        break;
      default:
        p(" %6d", dist);
        break;
    }
  }

  Serial.print(LaeToFMuxSerEoR);
}

void serExecGetLux()
{
  int       i;
  float     lux;
  uint32_t  lux_int;
  uint32_t  lux_frac;

  if( !serChkArgCnt(LaeToFMuxSerCmdArgsGetLux) )
  {
    return;
  }

  Serial.print(SerArgs[SerCmdIdx]);

  for(i = 0; i < LaeToFMuxNumOfChan; ++i)
  {
    if( ToFSensor[i]->isBlacklisted() )
    {
      lux = LaeToFMuxArgLuxNoLight;
    }
    else
    {
      lux = ToFSensor[i]->getAmbientLight();
    }

    lux_int = (uint32_t)lux;
    lux = lux - (float)lux_int;
    lux_frac = (uint32_t)(lux * 100.0);

    p(" %7d.%02d", lux_int, lux_frac);
  }

  Serial.print(LaeToFMuxSerEoR);
}

void serExecGetTunes()
{
  int       sensor;
  byte      offset;
  byte      crosstalk;
  byte      gain;
  uint16_t  intPeriod;

  if( !serChkArgCnt(LaeToFMuxSerCmdArgsGetTunes) )
  {
    return;
  }
  else if( (sensor = serParseSensorNumber(SerArgs[1])) < 0 )
  {
    return;
  }
  else
  {
    ToFSensor[sensor]->getTunes(offset, crosstalk, gain, intPeriod);

    rsp("%s %u %u %u %u",
        SerArgs[SerCmdIdx], offset, crosstalk, gain, intPeriod);
  }
}

void serExecProbe()
{
  int   i;
  int   nSensors;

  if( !serChkArgCnt(LaeToFMuxSerCmdArgsProbe) )
  {
    return;
  }

  probe();

  // count
  for(i = 0, nSensors = 0; i < LaeToFMuxNumOfChan; ++i)
  {
    if( !ToFSensor[i]->isBlacklisted() )
    {
      ++nSensors;
    }
  }

  rsp("%s %d", SerArgs[SerCmdIdx], nSensors);
}

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
      p(" %s", LaeToFMuxSerArgPresent);
    }
  }

  Serial.print(LaeToFMuxSerEoR);
}

void serExecCont()
{
  if( !serChkArgCnt(LaeToFMuxSerCmdArgsCont) )
  {
    return; 
  }

  if( !SerContinuousMode )
  {
    SerContinuousMode = true;
    rsp("%s %s", SerArgs[SerCmdIdx], LaeToFMuxSerArgOn);
  }
  else
  {
    SerContinuousMode = false;
    rsp("%s %s", SerArgs[SerCmdIdx], LaeToFMuxSerArgOff);
  }
}

void serContinuousOutput()
{
  int i;
  int dist;

  for(i = 0; i < LaeToFMuxNumOfChan; ++i)
  {
    if( ToFSensor[i]->isBlacklisted() )
    {
      dist = LaeToFMuxArgRangeNoDev;
    }
    else
    {
      dist = ToFSensor[i]->getRange();
    }

    switch( dist )
    {
      case LaeToFMuxArgRangeNoObj:
        p(" %6s", LaeToFMuxSerArgNotPresent);
        break;
      case LaeToFMuxArgRangeErr:
        p(" %6s", LaeToFMuxSerArgSensorErr);
        break;
      case LaeToFMuxArgRangeNoDev:
        p(" %6s", LaeToFMuxSerArgNotPresent);
        break;
      default:
        p(" %6d", dist);
        break;
    }
  }

  Serial.print(LaeToFMuxSerEoR);
}
