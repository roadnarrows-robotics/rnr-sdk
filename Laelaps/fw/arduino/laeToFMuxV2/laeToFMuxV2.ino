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
const int   DistAlsRatio = 16;              ///< dist/amb meas approx ratio
int         AlsSensorId;                    ///< current ALS sensor id
int         AlsCounter;                     ///< ALS sense counter
int         AlsFreq;                        ///< ALS sense frequency 

//
// I2C slave data
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
byte      SerContinuousOutput;


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
  // Probe and initialize sensors and data.
  //
  probe();

  //
  // I2C slave setup. Receive and send from/to master handlee by asynchronous
  // callbacks.
  //
  RspLen = 0;
  Wire.begin(LaeI2CAddrToFMux);
  Wire.onReceive(receiveCmd);
  Wire.onRequest(sendRsp);

  //
  // Serial command-line ASCII interface (baud does not matter over USB) 
  // 
  SerLinePos        = 0;
  SerArgCnt         = 0;
  SerContinuousMode = false;
  Serial.begin(115200);
}

void loop()
{
  //
  // Take asynchronous measurements.
  //
  //RDK measure();

  //
  // Produce any serial continuous mode output.
  //
  if( SerContinuousMode )
  {
    //RDK serContinuousOutput();
  }

  //
  // Process any serial input.
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
    // This sensor is scheduled or actively taking an ambient light measurement.
    //
    if( AlsSensorId == i )
    {
      //
      // Scheduled.
      //
      if( AlsCounter > 0 )
      {
        // done with a distance measurement
        if( ToFSensor[i]->asyncMeasureRange() )
        {
          --AlsCounter;
        }
      }

      //
      // Making an ambient light measurement.
      //
      else
      {
        // done with ambient light measurement
        if( ToFSensor[i]->asyncMeasureAmbientLight() )
        {
          AlsSensorId = nextAmbientSensor(AlsSensorId);
          AlsCounter  = AlsFreq;
        }
      }
    }

    //
    // Take a distance measurement.
    //
    else
    {
      ToFSensor[i]->asyncMeasureRange();
    }
  }
}

int nextAmbientSensor(int sensor)
{
  int   i;

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
  uint16_t  crosstalk;
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

void p(char *fmt, ...)
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

void errrsp(char *fmt, ... )
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

boolean serChkArgCnt(int nExpected)
{
  int   argCnt;

  argCnt = SerArgCnt - 1;

  if( argCnt != nExpected )
  {
    errrsp("requires %d args, got %d", nExpected, argCnt);
    return false;
  }

  return true;
}

int serParseOp(char *s)
{
  String  str(s);

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
    errrsp("unknown operator %s", s);
    return '?';
  }
}

boolean serParseNumber(char *sName, char *sVal,
                       long  nMin,  long nMax,
                       long &nVal)
{
  String  str = "";
  char   *s, c;

  if( (sVal == NULL) || (*sVal == 0) )
  {
    errrsp("no %s argument", *sName);
    return false;
  }

  for(s = sVal; *s != NULL; ++s)
  {
    c = *s;
    if( isDigit(c) )
    {
      str += c;
    }
    else
    {
      errrsp("%s value %s is not a number", sName, sVal);
      return false;
    }
  }
    
  nVal = str.toInt();

  if( (nVal < nMin) || (nVal > nMax) )
  {
    errrsp("%s value %s out-of-range", sName, sVal);
    return false;
  }

  return true;
}

boolean serParseSensorId(char *sVal, int sensor)
{
  long  n;

  if( !serParseNumber("sensor", sVal, LaeToFMuxMinChan, LaeToFMuxMaxChan, n) )
  {
    return false;
  }
  else if( ToFSensor[n]->isBlacklisted() )
  {
    errrsp("sensor %s not connected", sVal);
    return false;
  }
  else
  {
    sensor = (int)n;
    return true;
  }
}

boolean serParseTuneParam(char *sName, char *sVal,
                          long nCur, long nDft,
                          long nMin, long nMax,
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

void serExecCmd()
{
  char    cmdId;
  boolean bMode;

  if( strlen(SerArgs[SerCmdIdx]) > 1 )
  {
    errrsp("bad command");
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
      errrsp("unknown command");
      break;
  }

  SerContinuousMode = bMode;
}

void serExecHelp()
{
  rsp("%c %c %c %c %c %c %c %c %c",
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
  rsp("%s 10", SerArgs[SerCmdIdx]);
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

void serExecGetVersion()
{
  if( serChkArgCnt(LaeToFMuxSerCmdArgsGetVersion) )
  {
    rsp("%d", LAE_TOF_MUX_FW_VERSION);
  }
}

void serExecConfig()
{
  int   op;
  long  state;

  if( (SerArgCnt - 1) == 0 )
  {
    errrsp("no operator");
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
      AlsSensorId = AlsSensorId < 0? nextAmbientSensor(0): AlsSensorId;
    }
  }

  if( AlsSensorId < 0 )
  {
    rsp("%s", LaeToFMuxSerArgOff);
  }
  else
  {
    rsp("%s", LaeToFMuxSerArgOn);
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
  else if( !serParseSensorId(SerArgs[1], sensor) ) 
  {
    return;
  }
  else
  {
    ToFSensor[sensor]->getIdent(&ident);

    rsp("0x%02x %d.%d %d.%d, %d, %d",
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
      lux = LaeToFMuxArgLuxNoDev;
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

  Serial.print(LaeToFMuxSerEoR);
}

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
    errrsp("no operator");
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
      rsp("%u %u %u %u", rangeOffset, rangeCrossTalk, alsGain, alsIntPeriod);
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

  rsp("%u %u %u %u", rangeOffset, rangeCrossTalk, alsGain, alsIntPeriod);
}

void serExecProbe()
{
  if( !serChkArgCnt(LaeToFMuxSerCmdArgsProbe) )
  {
    return;
  }

  probe();

  rsp("%d", ToFNumConn);
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
    errrsp("%s bad output mode", str.c_str());
    return;
  }

  SerContinuousMode = true;

  rsp("%s", str.c_str());
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
