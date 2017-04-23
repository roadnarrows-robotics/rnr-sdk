////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelasp
//
// Firmware:  laeToFMux
//
// File:      Vl6180.cpp
//
/*! \file
 *
 * \brief The VL6180 Time-of-Flight sensor class implementation.
 *
 * This code is based on the Arduino library freely available from Sparkfun.
 * See original comment block below.
 *
 * \sa https://github.com/sparkfun/ToF_Range_Finder_Sensor-VL6180/tree/master/Libraries/Arduino
 * \sa https://www.sparkfun.com/products/12785
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * 2015-2016  RoadNarrows
 * (http://www.RoadNarrows.com)
 * \n All Rights Reserved
 */
////////////////////////////////////////////////////////////////////////////////

/******************************************************************************
 * SparkFun_VL6180x.cpp
 * Library for VL6180x time of flight range finder.
 * Casey Kuhns @ SparkFun Electronics
 * 10/29/2014
 * https://github.com/sparkfun/SparkFun_ToF_Range_Finder-VL6180_Arduino_Library
 * 
 * The VL6180x by ST micro is a time of flight range finder that
 * uses pulsed IR light to determine distances from object at close
 * range.  The average range of a sensor is between 0-200mm
 * 
 * In this file are the functions in the VL6180x class
 * 
 * Resources:
 * This library uses the Arduino Wire.h to complete I2C transactions.
 * 
 * Development environment specifics:
 * 	IDE: Arduino 1.0.5
 * 	Hardware Platform: Arduino Pro 3.3V/8MHz
 * 	VL6180x Breakout Version: 1.0
 * **Updated for Arduino 1.6.4 5/2015**
 * 
 * This code is beerware. If you see me (or any other SparkFun employee) at the
 * local pub, and you've found our code helpful, please buy us a round!
 * 
 * Distributed as-is; no warranty is given.
 ******************************************************************************/

#include <SoftwareWire.h>

//
// Bridge between standard POSIX C/C++ Linux and Arduino constructs.
//
typedef byte byte_t;

#include "laeToFMux.h"  // interface between firmware and software.
#include "VL6180X.h"    // interface to sensor class


extern void p(const char *fmt, ...);

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Debugging Macros
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*! \brief Define to include debug hooks, undef to exclude.
 *
 * Warning: Lots of spontaneous output can and does interfere with firmware 
 * downloads. Nice.
 *
 * \{
 */
#undef DBG_ENABLE   ///< define/undef

//
// Debug sections.
//
#define DBG_SECT_INI    0x01    ///< debug initialization code section
#define DBG_SECT_TOF    0x02    ///< debug time-of-flight code section
#define DBG_SECT_ALS    0x04    ///< debug ambient light code section
#define DBG_SECT_ERR    0x08    ///< debug error handling code section

#define DBG_INIT_SENSOR 7       ///< define sensor to debug initialization [0-7]

#define DBG_MASK    DBG_SECT_TOF ///< define or'ed bits of DBG_SECT sections

#ifdef DBG_ENABLE

/*!
 * \brief Debug print string.
 */
#define DBG_PRINT(dbg, section, fmt, ...) \
  do \
  { \
    if( dbg && ((section) & DBG_MASK) ) \
    { \
      p("%s: " fmt, sectName(section), ##__VA_ARGS__); \
    } \
  } \
  while(false)

const char *sectName(uint8_t section)
{
  if( section & DBG_SECT_INI )
  {
    return "ini";
  }
  else if( section & DBG_SECT_TOF )
  {
    return "tof";
  }
  else if( section & DBG_SECT_ALS )
  {
    return "als";
  }
  else if( section & DBG_SECT_ERR )
  {
    return "err";
  }
  else
  {
    return "any";
  }
}

#else // disable

#define DBG_PRINT(dbg, section, ...)

#endif // DBG_ENABLE
/*
 * \}
 */

/*!
 * \brief Elapse delta time.
 *
 * \param t0  Some earlier, previous time.
 *
 * \return Milliseconds.
 */
unsigned long dt(unsigned long t0)
{
  unsigned long t1;

  t1 = millis();

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


//------------------------------------------------------------------------------
// Class VL6180x
//------------------------------------------------------------------------------

VL6180x::VL6180x(int sensorId, SoftwareWire &wire, uint8_t address) :
    m_nSensorId(sensorId), m_wire(wire), m_addr(address)
{
  m_bBlackListed  = false;
  m_bBusy         = false;
  m_uErrCnt       = 0;
  m_bDebug        = false;

  memset(&m_ident, 0, sizeof(VL6180xIdentification));

  m_range         = 0;
  m_lux           = 0.0;

  m_eAsyncState   = AsyncStateInit;

  m_bRangeNeedsTuning = false;
  m_bAlsNeedsTuning   = false;

  m_regRangeOffsetDft = 0;                          // factory part default 
  m_regRangeOffset    = 0;                          // default varies by part
  m_regRangeCrossTalk = VL6180X_RANGE_XTALK_DFT;    // cross-talk default
  m_regAlsGain        = VL6180X_AMBIENT_GAIN_DFT;   // als gain 1.0
  m_regAlsIntPeriod   = VL6180X_AMBIENT_INT_T_REC;  // als int time 100 msec
}

// This version uses the bootup register.
boolean VL6180x::waitForBootup(int tries)
{
  int   i;
  byte  bootstate;

  for(i = 0; i < tries; ++i)
  {
    bootstate = readReg8(VL6180X_FIRMWARE_BOOTUP);
    if( bootstate & 0x01 )
    {
      if( m_nSensorId == DBG_INIT_SENSOR )
      {
        DBG_PRINT(true, DBG_SECT_INI, "booted\n");
      }
      writeReg8(VL6180X_SYSTEM_FRESH_OUT_OF_RESET, 0); // clear reset
      return true;
    }
    else
    {
      delay(1);
    }
  }

  if( m_nSensorId == DBG_INIT_SENSOR )
  {
    DBG_PRINT(true, DBG_SECT_INI, "not booted\n");
  }

  return false;
}

#if 0
// This version uses the fresh_out_of_reset register.
boolean VL6180x::waitForBootup(int tries)
{
  int   i;
  byte  reset;

  for(i = 0; i < tries; ++i)
  {
    reset = readReg8(VL6180X_SYSTEM_FRESH_OUT_OF_RESET);
    if( reset & 0x01 )
    {
      if( m_nSensorId == DBG_INIT_SENSOR )
      {
        DBG_PRINT(true, DBG_SECT_INI, "fresh_out_of_reset\n");
      }
      writeReg8(VL6180X_SYSTEM_FRESH_OUT_OF_RESET, 0); // clear reset
      return true;
    }
    else
    {
      delay(1);
    }
  }

  if( m_nSensorId == DBG_INIT_SENSOR )
  {
    DBG_PRINT(true, DBG_SECT_INI, "not fresh_out_of_reset\n");
  }

  return false;
}
#endif // 0

boolean VL6180x::ping(int tries)
{
  int   i;
  byte  d;

  m_bBusy = true;

  for(i = 0; i < tries; ++i)
  {
    d = readReg8(VL6180X_IDENTIFICATION_MODEL_ID);

    if( (d != 0) && (d != 0xff) )
    {
      if( m_nSensorId == DBG_INIT_SENSOR )
      {
        DBG_PRINT(true, DBG_SECT_INI, "pinged, tries=%d\n", i+1);
      }
      m_bBusy = false;
      return true;
    }

    delay(10);
  }

  if( m_nSensorId == DBG_INIT_SENSOR )
  {
    DBG_PRINT(true, DBG_SECT_INI, "no response, tries=%d\\n", i);
  }

  m_bBusy = false;

  return false;
}

void VL6180x::initSensor()
{
  m_bBusy = true;

  // . . .
  // REGISTER_TUNING_SR03_270514_CustomerView.txt
  //
  // See: https://github.com/kirananto/ONEPLUS2RAZOR/tree/master/drivers/media/platform/msm/camera_v2/sensor/vl6180
  // . . .
  writeReg8(0x0207, 0x01);
  writeReg8(0x0208, 0x01);
  writeReg8(0x0096, 0x00);
  writeReg8(0x0097, 0xfd);
  writeReg8(0x00e3, 0x00);
  writeReg8(0x00e4, 0x04);
  writeReg8(0x00e5, 0x02);
  writeReg8(0x00e6, 0x01);
  writeReg8(0x00e7, 0x03);
  writeReg8(0x00f5, 0x02);
  writeReg8(0x00d9, 0x05);
  writeReg8(0x00db, 0xce);
  writeReg8(0x00dc, 0x03);
  writeReg8(0x00dd, 0xf8);
  writeReg8(0x009f, 0x00);
  writeReg8(0x00a3, 0x3c);
  writeReg8(0x00b7, 0x00);
  writeReg8(0x00bb, 0x3c);
  writeReg8(0x00b2, 0x09);
  writeReg8(0x00ca, 0x09);
  writeReg8(0x0198, 0x01);
  writeReg8(0x01b0, 0x17);
  writeReg8(0x01ad, 0x00);
  writeReg8(0x00ff, 0x05);
  writeReg8(0x0100, 0x05);
  writeReg8(0x0199, 0x05);
  writeReg8(0x01a6, 0x1b);
  writeReg8(0x01ac, 0x3e);
  writeReg8(0x01a7, 0x1f);
  writeReg8(0x0030, 0x00);

#if 0
  // . . .
  // Recommended : Public registers - See data sheet for more detail
  //
  // Note: These registers are set in writeSensorDefaults().
  // . . .

  // enables polling for New Sample ready when measurement completes
  writeReg8(VL6180X_SYSTEM_MODE_GPIO1, 0x10);

  // set the averaging sample period (compromise between lower noise and
  // increased execution time)
  writeReg8(VL6180X_READOUT_AVERAGING_SAMPLE_PERIOD, 0x30);

  // set the light and dark gain (upper nibble)
  // Note: dark gain should not be changed
  writeReg8(VL6180X_SYSALS_ANALOGUE_GAIN, 0x46);

  // set the # of range measurements after which auto calibration of system
  // is performed 
  writeReg8(VL6180X_SYSRANGE_VHV_REPEAT_RATE, 255);

  // set ALS integration time to 100ms
  writeReg8(VL6180X_SYSALS_INTEGRATION_PERIOD, 99);

  // perform a single temperature calibration of the ranging sensor
  writeReg8(VL6180X_SYSRANGE_VHV_RECALIBRATE, 0x01);

  // . . .
  // Optional: Public registers - See data sheet for more detail
  //
  // Note: These registers are set in writeSensorDefaults().
  // . . .
  
  // set default ranging inter-measurement period to 100ms
  writeReg8(VL6180X_SYSRANGE_INTERMEASUREMENT_PERIOD, 0x09);

  // set default ALS inter-measurement period to 500ms
  writeReg8(VL6180X_SYSALS_INTERMEASUREMENT_PERIOD, 0x31);

  // configures interrupt on New sample ready
  writeReg8(VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x24);
#endif // 0

  m_bBusy = false;
}

void VL6180x::writeSensorDefaults()
{
  m_bBusy = true;

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // System
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  // set GPIO interrupt active high when sample complete
  writeReg8(VL6180X_SYSTEM_MODE_GPIO1, 0x10);

  // set averaging sample period to reset-default
  writeReg8(VL6180X_READOUT_AVERAGING_SAMPLE_PERIOD, 0x30);

  // set firmware als result scaler to reset-default
  writeReg8(VL6180X_FIRMWARE_RESULT_SCALER, 0x01);

  //
  // Configure interrupt on ‘new als and range sample' ready threshold event.
  // Note: only new sample bit works, the rest is shit.
  //
  writeReg8(VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x24);


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Range Sensor
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
 
  // get factory calibrated range offset (part to part variance)
  m_regRangeOffsetDft = readReg8(VL6180X_SYSRANGE_PART_TO_PART_RANGE_OFFSET);

  // set cross-talk compensation to reset-default on initialization
  writeReg16(VL6180X_SYSRANGE_CROSSTALK_COMPENSATION_RATE, m_regRangeCrossTalk);

  // set ranging inter-measurement period to 100ms (0 == 10ms, step size == 10)
  writeReg8(VL6180X_SYSRANGE_INTERMEASUREMENT_PERIOD, 9);

  // set max range convergence time 50ms (reset-default is 0x31 = 49)
  writeReg8(VL6180X_SYSRANGE_MAX_CONVERGENCE_TIME, 50);

  // set range checks to reset-defaults of early convergence and S/N enables
  writeReg8(VL6180X_SYSRANGE_RANGE_CHECK_ENABLES, 0x10 | 0x01);

  // set early convergence estimate threshold. (value here from some notes)
  writeReg16(VL6180X_SYSRANGE_EARLY_CONVERGENCE_ESTIMATE, 0x7B);

  // set range distance thresholds in mm
  writeReg8(VL6180X_SYSRANGE_THRESH_HIGH, 200);
  writeReg8(VL6180X_SYSRANGE_THRESH_LOW,  5);

  // set auto calibration repeat rate (0 is off, >0 autocals after every n meas)
  writeReg8(VL6180X_SYSRANGE_VHV_REPEAT_RATE, 0);

  // perform a single temperature calibration (must be done when sensor stopped)
  writeReg8(VL6180X_SYSRANGE_VHV_RECALIBRATE, 0x01);

  // typical calibration time is less than 200usec
  delay(1);


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Ambient Light Sensor
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  // set default ALS continuous mode inter-measurement period to 500ms
  writeReg8(VL6180X_SYSALS_INTERMEASUREMENT_PERIOD, 49);

  // set the ALS gain to reset-default
  writeReg8(VL6180X_SYSALS_ANALOGUE_GAIN, (0x40 | m_regAlsGain));

  // set ALS integration time to 100ms
  writeReg16(VL6180X_SYSALS_INTEGRATION_PERIOD, m_regAlsIntPeriod-1);


  m_bBusy = false;
}

void VL6180x::readIdent()
{
  m_bBusy = true;

  m_ident.idModel = readReg8(VL6180X_IDENTIFICATION_MODEL_ID);
  m_ident.idModelRevMajor = readReg8(VL6180X_IDENTIFICATION_MODEL_REV_MAJOR);
  m_ident.idModelRevMinor = readReg8(VL6180X_IDENTIFICATION_MODEL_REV_MINOR);
  m_ident.idModuleRevMajor = readReg8(VL6180X_IDENTIFICATION_MODULE_REV_MAJOR);
  m_ident.idModuleRevMinor = readReg8(VL6180X_IDENTIFICATION_MODULE_REV_MINOR);

  m_ident.idDate = readReg16(VL6180X_IDENTIFICATION_DATE);
  m_ident.idTime = readReg16(VL6180X_IDENTIFICATION_TIME);

  m_bBusy = false;
}

void VL6180x::readTunes()
{
  m_regRangeOffset    = readReg8(VL6180X_SYSRANGE_PART_TO_PART_RANGE_OFFSET);
  m_regRangeCrossTalk = readReg16(VL6180X_SYSRANGE_CROSSTALK_COMPENSATION_RATE);

  m_regAlsGain = readReg8(VL6180X_SYSALS_ANALOGUE_GAIN);
  m_regAlsGain &= 0x2f;

  m_regAlsIntPeriod = readReg16(VL6180X_SYSALS_INTEGRATION_PERIOD);
  m_regAlsIntPeriod += 1;
}

#ifdef INCLUDE_EXTRAS
byte VL6180x::changeAddress(byte new_address)
{
  if( new_address == m_addr )
  {
    return m_addr;
  }

  else if( new_address > 127 )
  {
    return m_addr;
  }
   
  m_bBusy = true;

  writeReg8(VL6180X_I2C_SLAVE_DEVICE_ADDRESS, new_address);
   
  m_addr = readReg8(VL6180X_I2C_SLAVE_DEVICE_ADDRESS); 

  m_bBusy = false;

  return m_addr;
}

byte VL6180x::measureRange()
{
  byte  range;    // range raw value
  byte  status;   // range measurement status

  m_bBusy = true;

  if( m_bRangeNeedsTuning )
  {
    tuneRangeSensor(m_newRangeOffset, m_newRangeCrossTalk);
  }

  writeReg8(VL6180X_SYSRANGE_START, 0x01); //Start Single shot mode

  delay(20);

  writeReg8(VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07);

  range   = readReg8(VL6180X_RESULT_RANGE_VAL);
  status  = readReg8(VL6180X_RESULT_RANGE_STATUS);

  m_range = cvtRangeRawToDist(range, status);

  m_bBusy = false;

  return m_range;
}

float VL6180x::measureAmbientLight()
{
#ifdef LAE_USE_ALS
  unsigned int  alsRaw;
  byte          status;

  m_bBusy = true;

  if( m_bAlsNeedsTuning )
  {
    tuneAls(m_newAlsGain, m_newAlsIntPeriod);
  }

  // start ALS measurement 
  writeReg8(VL6180X_SYSALS_START, 0x01);

  delay(m_regAlsIntPeriod+10); // give it time... 

  writeReg8(VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07);

  // Retrieve the Raw ALS value from the sensor
  alsRaw = readReg16(VL6180X_RESULT_ALS_VAL);
  status = readReg8(VL6180X_RESULT_ALS_STATUS);
  
  // convert raw to lux
  m_lux = cvtAlsRawToLux(alsRaw, status);

  m_bBusy = false;

#else // !LAE_USE_ALS

  m_lux = 0;

#endif // LAE_USE_ALS

  return m_lux;
}
#endif // INCLUDE_EXTRAS

boolean VL6180x::asyncMeasureRange()
{
  boolean bExec;  // can [not] continue to execute on this pass
  boolean bDone;  // measurement cycle is [not] done
  byte    range;  // range raw value
  byte    status; // interrupt status register bits
  byte    error;  // error bits

  bExec = true;
  bDone = false;

  //
  // Execute asynchronous measurement a far as synchronously possible.
  //
  while( bExec )
  {
    switch( m_eAsyncState )
    {
      //
      // Initialize data for measurment.
      //
      case AsyncStateInit:
        DBG_PRINT(m_bDebug, DBG_SECT_TOF, "init\n");

        // mark sensor busy
        m_bBusy = true;

        // range tune parameters need updating
        if( m_bRangeNeedsTuning )
        {
          tuneRangeSensor(m_newRangeOffset, m_newRangeCrossTalk);
        }

        //
        // Clear interrupts. If device is stopped with bits uncleared, a new
        // measurement will never happen.
        //
        writeReg8(VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x01);

        // initialize state
        m_uAsyncTWait   = 40;
        m_uAsyncTStart  = millis();
        m_eAsyncState   = AsyncStateWaitForReady;
        break;

      //
      // Wait for the sensor to be ready.
      //
      case AsyncStateWaitForReady:
        DBG_PRINT(m_bDebug, DBG_SECT_TOF, "waitforready\n");

        // read range status register
        status = readReg8(VL6180X_RESULT_RANGE_STATUS);

        // sensor is ready
        if( status & 0x01 )
        {
          m_eAsyncState = AsyncStateStartMeas;
        }
        // keep waiting
        else if( dt(m_uAsyncTStart) <= m_uAsyncTWait )
        {
          bExec = false;
        }
        // timeout - take measurement anyway
        else
        {
          DBG_PRINT(m_bDebug, DBG_SECT_TOF|DBG_SECT_ERR,
              "waitforready: timeout\n");
          m_eAsyncState = AsyncStateStartMeas;
        }
        break;

      //
      // Start a measurement.
      //
      case AsyncStateStartMeas:
        DBG_PRINT(m_bDebug, DBG_SECT_TOF, "startmeas\n");

        // start measurement
        writeReg8(VL6180X_SYSRANGE_START, 0x01);

        // initialize state
        bExec           = false;
        m_uAsyncTWait   = 60;
        m_uAsyncTStart  = millis();
        m_eAsyncState   = AsyncStateWaitForResult;
        break;

      //
      // Wait for the measurement's result.
      //
      case AsyncStateWaitForResult:
        DBG_PRINT(m_bDebug, DBG_SECT_TOF, "waitforresult\n");

        // read gpio interrupt status 
        status = readReg8(VL6180X_RESULT_INTERRUPT_STATUS_GPIO);

        DBG_PRINT(m_bDebug, DBG_SECT_TOF, "waitforresult: status=0x%02x\n",
            status);

        // result ready
        if( status & 0x07 )
        {
          m_eAsyncState = AsyncStateDone;
        }
        // keep waiting
        else if( dt(m_uAsyncTStart) <= m_uAsyncTWait )
        {
          bExec = false;
        }
        // timeout - abort
        else
        {
          DBG_PRINT(m_bDebug, DBG_SECT_TOF|DBG_SECT_ERR,
              "waitforresult: timeout\n");
          m_eAsyncState = AsyncStateAbort;
        }
        break;

      //
      // Sensor measurement cycle has completed, retrieve the value.
      //
      case AsyncStateDone:
        DBG_PRINT(m_bDebug, DBG_SECT_TOF, "done: msec=%d\n",
            dt(m_uAsyncTStart));

        // get the range distance raw value
        range = readReg8(VL6180X_RESULT_RANGE_VAL);

        // get any error status
        error = readReg8(VL6180X_RESULT_RANGE_STATUS);
        error >>= 4;

        // clear interrupts
        writeReg8(VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07);

        // got a measurement as indicated by an interrupt status bit
        if( status & 0x04 )
        {
          // no errors
          if( error == 0x00 )
          {
            // sketchy out of range value
            if( range > VL6180X_RANGE_MAX )
            {
              range = VL6180X_RANGE_NO_OBJ;
            }
            // decrement error count
            if( m_uErrCnt > 0 )
            {
              --m_uErrCnt;
            }
          }

          // early convergence estimate, not an error, but rather no object
          //else if( error == 0x06 )
          //{
          //  range = VL6180X_RANGE_NO_OBJ;
          //}

          // some range sensing error result
          else
          {
            DBG_PRINT(m_bDebug, DBG_SECT_TOF|DBG_SECT_ERR,
                "done: error=0x%02x\n", error);
            range = VL6180X_RANGE_NO_OBJ;
          }
        }

        // failed to get a result
        else
        {
          range = VL6180X_RANGE_NO_OBJ;
        }

        m_range = range;

        DBG_PRINT(m_bDebug, DBG_SECT_TOF, "range=%d\n", m_range);

        bExec         = false;
        bDone         = true;
        m_eAsyncState = AsyncStateInit;
        m_bBusy       = false;
        break;

      //
      // Abort measurement.
      //
      case AsyncStateAbort:
        DBG_PRINT(m_bDebug, DBG_SECT_TOF|DBG_SECT_ERR, "abort after msec=%d\n",
            dt(m_uAsyncTStart));

        // clear interrupts
        writeReg8(VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07);

        ++m_uErrCnt;

        // Sensor is probably hung. Try to revive via firmware. There are no
        // power reset options for the sensor.
        if( m_uErrCnt > 10 )
        {
          DBG_PRINT(m_bDebug, DBG_SECT_ERR, "abort: reset %d\n", m_nSensorId);

          // maybe the i2c is screwed up
          if( !ping(1) )
          {
            DBG_PRINT(m_bDebug, DBG_SECT_ERR, "abort: restart i2c\n");
            m_wire.end();
            delay(10);
            m_wire.begin();
            delay(10);
          }
        }

        bExec         = false;
        bDone         = true;
        m_eAsyncState = AsyncStateInit;
        m_bBusy       = false;
        break;

      //
      // Unknown/error state.
      //
      default:
        m_eAsyncState = AsyncStateInit;
        break;
    }
  }

  return bDone;
}

boolean VL6180x::asyncMeasureAmbientLight()
{
#ifdef LAE_USE_ALS

  boolean   bExec;  // can [not] continue to execute on this pass
  boolean   bDone;  // measurement is [not] done
  uint16_t  raw;    // als raw value
  byte      status; // status register value

  bExec = true;
  bDone = false;

  //
  // Execute asynchronous measurement a far as synchronously possible.
  //
  while( bExec )
  {
    switch( m_eAsyncState )
    {
      //
      // Initialize data for measurment.
      //
      case AsyncStateInit:
        DBG_PRINT(m_bDebug, DBG_SECT_ALS, "init\n");

        // mark sensor busy
        m_bBusy = true;

        // ambient light tune parameters need updating
        if( m_bAlsNeedsTuning )
        {
          tuneAls(m_newAlsGain, m_newAlsIntPeriod);
        }

        // initialize state
        m_uAsyncTWait   = (unsigned long)((float)m_regAlsIntPeriod * 1.5);
        m_uAsyncTStart  = millis();
        m_eAsyncState   = AsyncStateWaitForReady;
        break;

      //
      // Wait for the sensor to be ready.
      //
      case AsyncStateWaitForReady:
        DBG_PRINT(m_bDebug, DBG_SECT_ALS, "wait ready\n");

        // read range status register
        status = readReg8(VL6180X_RESULT_ALS_STATUS);

        // sensor is ready
        if( status & 0x01 )
        {
          m_eAsyncState = AsyncStateStartMeas;
        }
        // keep waiting
        else if( dt(m_uAsyncTStart) <= m_uAsyncTWait )
        {
          bExec = false;
        }
        // timeout - abort
        else
        {
          m_eAsyncState = AsyncStateAbort;
        }
        break;

      //
      // Start a measurement.
      //
      case AsyncStateStartMeas:
        DBG_PRINT(m_bDebug, DBG_SECT_ALS, "start meas\n");

        writeReg8(VL6180X_SYSALS_START, 0x01);
        bExec         = false;
        m_eAsyncState = AsyncStateWaitForResult;
        break;

      //
      // Wait for the measurement's result.
      //
      case AsyncStateWaitForResult:
        // read gpio interrupt status 
        status = readReg8(VL6180X_RESULT_INTERRUPT_STATUS_GPIO);

        DBG_PRINT(m_bDebug, DBG_SECT_ALS, "wait meas, status=0x%02x\n",
            status);

        // result ready
        if( status & 0x20 )
        {
          m_eAsyncState = AsyncStateDone;
        }
        // keep waiting
        else if( dt(m_uAsyncTStart) <= m_uAsyncTWait )
        {
          bExec = false;
        }
        // timeout - abort
        else
        {
          m_eAsyncState = AsyncStateAbort;
        }
        break;

      //
      // Sensor measurement has completed, retrieve the value.
      //
      case AsyncStateDone:
        DBG_PRINT(m_bDebug, DBG_SECT_ALS, "done in msec=%d\n",
          dt(m_uAsyncTStart));

        // clear interrupts
        writeReg8(VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07);

        // get ambient lux and status values
        raw     = readReg16(VL6180X_RESULT_ALS_VAL);
        status  = readReg8(VL6180X_RESULT_ALS_STATUS);
  
        m_lux = cvtAlsRawToLux(raw, status);

        bExec         = false;
        bDone         = true;
        m_eAsyncState = AsyncStateInit;
        m_bBusy       = false;
        break;

      //
      // Abort measurement.
      //
      case AsyncStateAbort:
        DBG_PRINT(m_bDebug, DBG_SECT_ALS|DBG_SECT_ERR, "abort after msec=%d\n",
            dt(m_uAsyncTStart));

        // clear interrupts
        writeReg8(VL6180X_SYSTEM_INTERRUPT_CLEAR, 0x07);

        bExec         = false;
        bDone         = true;
        m_eAsyncState = AsyncStateInit;
        m_bBusy       = false;
        break;

      //
      // Unknown/error state.
      //
      default:
        m_eAsyncState = AsyncStateInit;
        break;
    }
  }

  return bDone;

#else // !LAE_USE_ALS

   m_lux = 0;

  return true;

#endif // LAE_USE_ALS
}

byte VL6180x::cvtRangeRawToDist(byte rangeRaw, byte rangeStatus)
{
  byte  range;

  rangeStatus >>= 4;
  
  if( rangeStatus == 0 )
  {
    range = rangeRaw;
  }
  // over/under flow (usually means no object)
  else
  {
    range = VL6180X_RANGE_NO_OBJ;
  }

  return range;
}

float VL6180x::cvtAlsRawToLux(uint16_t alsRaw, byte alsStatus)
{
  float   alsIntegrationPeriod;
  float   alsGain;
  float   lux;

  alsIntegrationPeriod = 100.0 / (float)m_regAlsIntPeriod;

  // gain enums to values
  switch( m_regAlsGain )
  {
    case GAIN_20:   alsGain = 20.0; break;
    case GAIN_10:   alsGain = 10.32; break;
    case GAIN_5:    alsGain = 5.21; break;
    case GAIN_2_5:  alsGain = 2.60; break;
    case GAIN_1_67: alsGain = 1.72; break;
    case GAIN_1_25: alsGain = 1.28; break;
    case GAIN_1:    alsGain = 1.01; break;
    case GAIN_40:   alsGain = 40.0; break;
    default:        alsGain = 1.0;  break;
  }

  // Calculate LUX from formula in AppNotes
  lux = (float)0.32 * ((float)alsRaw / alsGain) * alsIntegrationPeriod;

#if 0 // DBG code
  if( alsStatus & 0x10 )         // overflow
  {
    lux = -100.0;
  }
  else if( alsStatus & 0x20 )    // underflow
  {
    lux = -200.0;
  }
  else if( !(alsStatus & 0x01) ) // busy
  {
    lux = -300.0;
  }
#endif // DBG code

  return lux;
}

void VL6180x::markRangeForTuning(byte offset, uint16_t crosstalk)
{
  m_newRangeOffset    = offset;
  m_newRangeCrossTalk = crosstalk;
  m_bRangeNeedsTuning = true;
}

boolean VL6180x::tuneRangeSensor(byte offset, uint16_t crosstalk)
{
  if( offset != m_regRangeOffset )
  {
    writeReg8(VL6180X_SYSRANGE_PART_TO_PART_RANGE_OFFSET, offset);
    m_regRangeOffset = readReg8(VL6180X_SYSRANGE_PART_TO_PART_RANGE_OFFSET);
  }

  if( crosstalk != m_regRangeCrossTalk )
  {
    writeReg16(VL6180X_SYSRANGE_CROSSTALK_COMPENSATION_RATE, crosstalk);
    m_regRangeCrossTalk =
                    readReg16(VL6180X_SYSRANGE_CROSSTALK_COMPENSATION_RATE);
  }

  m_bRangeNeedsTuning = false;

  if( (offset == m_regRangeOffset) && (crosstalk == m_regRangeCrossTalk) )
  {
    return true;
  }
  else
  {
    return false;
  }
}

void VL6180x::markAlsForTuning(byte gain, uint16_t intPeriod)
{
  m_newAlsGain      = gain;
  m_newAlsIntPeriod = intPeriod;
  m_bAlsNeedsTuning = true;
}

boolean VL6180x::tuneAls(byte gain, uint16_t intPeriod)
{
  uint16_t    val;

  if( gain != m_regAlsGain )
  {
    writeReg8(VL6180X_SYSALS_ANALOGUE_GAIN, (byte)(0x40|gain));
    m_regAlsGain = readReg8(VL6180X_SYSALS_ANALOGUE_GAIN);
    m_regAlsGain &= 0x2f;
  }

  if( intPeriod == 0 )
  {
    intPeriod = 1;
  }

  if( intPeriod != m_regAlsIntPeriod )
  {
    val = intPeriod - 1;  // 0 is 1 msec, 1 is 2 msec, etc
    writeReg16(VL6180X_SYSALS_INTEGRATION_PERIOD, val);
    m_regAlsIntPeriod = readReg16(VL6180X_SYSALS_INTEGRATION_PERIOD);
    m_regAlsIntPeriod += 1;
  }

  m_bAlsNeedsTuning = false;

  if( (gain == m_regAlsGain) && (intPeriod == m_regAlsIntPeriod) )
  {
    return true;
  }
  else
  {
    return false;
  }
}

void VL6180x::whitelist()
{
  m_bBlackListed = false;
}

void VL6180x::blacklist()
{
  m_bBlackListed = true;
}

boolean VL6180x::isBlacklisted()
{
  return m_bBlackListed;
}

boolean VL6180x::isBusy()
{
  return m_bBusy;
}

void VL6180x::getIdent(VL6180xIdentification *pIdent)
{
  *pIdent = m_ident;
}

void VL6180x::getTunes(byte &offset, uint16_t &crosstalk,
                       byte &gain,   uint16_t &intPeriod)
{
  offset    = m_regRangeOffset;
  crosstalk = m_regRangeCrossTalk;
  gain      = m_regAlsGain;
  intPeriod = m_regAlsIntPeriod;
}

void VL6180x::getDefaultTunes(byte &offset, uint16_t &crosstalk,
                              byte &gain,   uint16_t &intPeriod)
{
  offset    = m_regRangeOffsetDft;
  crosstalk = VL6180X_RANGE_XTALK_DFT;
  gain      = VL6180X_AMBIENT_GAIN_DFT;
  intPeriod = VL6180X_AMBIENT_INT_T_REC;
}

void VL6180x::debug(boolean onoff)
{
  m_bDebug = onoff;
}

int VL6180x::getSensorId()
{
  return m_nSensorId;
}

byte VL6180x::getRange()
{
  return m_range;
}

float VL6180x::getAmbientLight()
{
  return m_lux;
}

byte VL6180x::readReg8(uint16_t regAddr)
{
  uint8_t n;
  byte    data;
  uint8_t rc;

  m_wire.beginTransmission(m_addr);     // sensor address
  m_wire.write((regAddr >> 8) & 0xFF);  // MSB of register address
  m_wire.write(regAddr & 0xFF);         // LSB of register address
  rc = m_wire.endTransmission(false);   // send address and register bytes

  if( rc == SOFTWAREWIRE_NO_ERROR )
  {
    n = m_wire.requestFrom(m_addr , 1); // request 1 byte back
    if( n >= 1 )
    {
      data = m_wire.read();             // read data from selected register
    }
    else
    {
      rc = SOFTWAREWIRE_OTHER; 
    }
  }

  if( rc != SOFTWAREWIRE_NO_ERROR )
  {
    data = 0;
    ++m_uErrCnt;

    DBG_PRINT(m_bDebug, DBG_SECT_ERR, "readReg8(%0x04x), errcnt=%u\n",
        regAddr, m_uErrCnt);
  }

  return data;
}

uint16_t VL6180x::readReg16(uint16_t regAddr)
{
  uint8_t   n;
  uint8_t   data_low;
  uint8_t   data_high;
  uint16_t  data;
  uint8_t   rc;

  m_wire.beginTransmission(m_addr);     // sensor address
  m_wire.write((regAddr >> 8) & 0xFF);  // MSB of register address
  m_wire.write(regAddr & 0xFF);         // LSB of register address
  rc = m_wire.endTransmission(false);   // send address and register bytes

  if( rc == SOFTWAREWIRE_NO_ERROR )
  {
    n = m_wire.requestFrom(m_addr , 2); // request 2 bytes back
    if( n >= 2 )
    {
      data_high = m_wire.read();          // read MSB from selected register
      data_low = m_wire.read();           // read LSB from selected register
      data = (data_high << 8)|data_low;
    }
    else
    {
      rc = SOFTWAREWIRE_OTHER; 
    }
  }

  if( rc != SOFTWAREWIRE_NO_ERROR )
  {
    data = 0;
    ++m_uErrCnt;

    DBG_PRINT(m_bDebug, DBG_SECT_ERR, "readReg16(%0x04x), errcnt=%u\n",
        regAddr, m_uErrCnt);
  }

  return data;
}

void VL6180x::writeReg8(uint16_t regAddr, byte data)
{
  uint8_t   rc;

  m_wire.beginTransmission(m_addr);     // sensor address
  m_wire.write((regAddr >> 8) & 0xFF);  // MSB of register address
  m_wire.write(regAddr & 0xFF);         // LSB of register address

  m_wire.write(data);                   // write register data

  rc = m_wire.endTransmission();        // send

  if( rc != SOFTWAREWIRE_NO_ERROR )
  {
    ++m_uErrCnt;

    DBG_PRINT(m_bDebug, DBG_SECT_ERR, "writeReg8(%0x04x), errcnt=%u\n",
        regAddr, m_uErrCnt);
  }
}

void VL6180x::writeReg16(uint16_t regAddr, uint16_t data)
{
  uint8_t   temp;
  uint8_t   rc;

  m_wire.beginTransmission(m_addr);     // sensor address
  m_wire.write((regAddr >> 8) & 0xFF);  // MSB of register address
  m_wire.write(regAddr & 0xFF);         // LSB of register address

  temp = (data >> 8) & 0xff;
  m_wire.write(temp);                   // write MSB register data
  temp = data & 0xff;
  m_wire.write(temp);                   // write LSB register data

  rc = m_wire.endTransmission();        // send

  if( rc != SOFTWAREWIRE_NO_ERROR )
  {
    ++m_uErrCnt;

    DBG_PRINT(m_bDebug, DBG_SECT_ERR, "writeReg16(%0x04x), errcnt=%u\n",
        regAddr, m_uErrCnt);
  }
}
