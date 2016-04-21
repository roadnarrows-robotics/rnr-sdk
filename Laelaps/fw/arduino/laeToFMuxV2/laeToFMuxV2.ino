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

VL6180x  *ToFSensor[LaeToFMuxNumOfChan];
int       AlsSensorId;
int       AlsFreqCnt;


//
// Response data
//
byte        RspBuf[LaeToFMuxMaxRspLen];         ///< response buffer
int         RspLen;                             ///< response length
const byte  RspErrorBuf[LaeToFMuxMaxRspLen] =   ///< response error pattern
{
  0xde, 0xad, 0xfa, 0xce, 0xba, 0xad, 0xf0, 0x0d, 0,
};


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

  Serial.begin(115200);
}

void loop()
{
  measureRanges();

  if( --AlsFreqCnt <= 0 )
  {
    measureAmbient();
    nextAmbientSensor();
  }

  if( Serial.available() > 0 )
  {
    serialRcvCmd();
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

#include <stdarg.h>

void p(char *fmt, ... )
{
  char buf[128]; // resulting string limited to 128 chars
  va_list args;

  va_start(args, fmt );
  vsnprintf(buf, 128, fmt, args);
  va_end(args);

  Serial.print(buf);
}

void serialRcvCmd()
{
  byte  cmd;

  cmd = Serial.read();

  switch( cmd )
  {
    case 'h':
      serPrintHelp();
      break;
    case 'l':
      serPrintSensors();
      break;
    case 'p':
      probe();
      break;
    case 'r':
      serPrintRanges();
      break;
    default:
      Serial.print("Commands are: 'l r h'\n");
      break;
  }
}

void serPrintHelp()
{
  Serial.print("\nHelp:\n");
  Serial.print("l  - list connected sensors\n");
  Serial.print("p  - (re)probe connected sensors\n");
  Serial.print("r  - print measured ranges\n");
  Serial.print("\n");
  Serial.print("h  - print help\n");
}

void serPrintSensors()
{
  int   i;

  for(i = 0; i < LaeToFMuxNumOfChan; ++i)
  {
    if( ToFSensor[i]->isBlacklisted() )
    {
      p("%d: not present\n", i);
    }
    else
    {
      p("%d: present\n", i);
    }
  }
}

void serPrintRanges()
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
    }

    p("%3d ", dist);
  }

  Serial.print("\n");
}
