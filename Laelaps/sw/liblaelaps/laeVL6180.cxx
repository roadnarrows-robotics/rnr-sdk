////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Library:   liblaelaps
//
// File:      laeVL6180.cxx
//
/*! \file
 *
 * $LastChangedDate: 2016-04-11 13:03:57 -0600 (Mon, 11 Apr 2016) $
 * $Rev: 4381 $
 *
 * \brief Laelaps Time-of-Flight sensors. The ToFs are used as a virtual bumper
 * for close-in obstacle detection.
 *
 * The hardware changed between Laelaps v2.0 and subsequence v2.1+ versions.
 * See laeVL6180.h for details.
 *
 * This code is based on the Arduino library freely available from Sparkfun.
 * See original comment block below.
 *
 * \sa https://github.com/sparkfun/ToF_Range_Finder_Sensor-VL6180/tree/master/Libraries/Arduino
 * \sa https://www.sparkfun.com/products/12785
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2015-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
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

#include <inttypes.h>
#include <unistd.h>
#include <math.h>

#include <string>
#include <vector>
#include <map>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/i2c.h"

#include "Laelaps/laelaps.h"
#include "Laelaps/laeDesc.h"
#include "Laelaps/laeTune.h"
#include "Laelaps/laeUtils.h"
#include "Laelaps/laeDb.h"

#include "Laelaps/laeI2CMux.h"  // v2.0
#include "Laelaps/laeToFMux.h"  // v2.1+

#include "Laelaps/laeVL6180.h"

using namespace std;
using namespace laelaps;
using namespace sensor::vl6180;


//------------------------------------------------------------------------------
// Private Data
//------------------------------------------------------------------------------

/*!
 * \brief Ambient Light Sensor gain table.
 */
static double AlsGainAnalogTbl[GAIN_NumOf] =
{
  20.0,   // [GAIN_20]
  10.32,  // [GAIN_10]
  5.21,   // [GAIN_5]
  2.60,   // [GAIN_2_5]
  1.72,   // [GAIN_1_67]
  1.28,   // [GAIN_1_25]
  1.01,   // [GAIN_1]
  40.0    // [GAIN_40]
};

//------------------------------------------------------------------------------
// LaeRangeSensorInfo Class
//------------------------------------------------------------------------------

LaeVL6180SensorInfo::LaeVL6180SensorInfo()
{
  m_nIndex    = -1;
  m_nChan     = -1;
  m_fBeamDir  = 0.0;
  m_fDeadzone = 0.0;
}

LaeVL6180SensorInfo::LaeVL6180SensorInfo(int               nIndex,
                                         int               nChan,
                                         double            fBeamDir,
                                         double            fDeadzone,
                                         const std::string &strDesc)
{
  m_nIndex    = nIndex;
  m_nChan     = nChan;
  m_fBeamDir  = fBeamDir;
  m_fDeadzone = fDeadzone;
  m_strDesc   = strDesc;
}

LaeVL6180SensorInfo::LaeVL6180SensorInfo(const LaeVL6180SensorInfo &src)
{
  m_nIndex    = src.m_nIndex;
  m_nChan     = src.m_nChan;
  m_fBeamDir  = src.m_fBeamDir;
  m_fDeadzone = src.m_fDeadzone;
  m_strDesc   = src.m_strDesc;
}

LaeVL6180SensorInfo::~LaeVL6180SensorInfo()
{
}

LaeVL6180SensorInfo LaeVL6180SensorInfo::operator=(
                                                const LaeVL6180SensorInfo &rhs)
{
  m_nIndex    = rhs.m_nIndex;
  m_nChan     = rhs.m_nChan;
  m_fBeamDir  = rhs.m_fBeamDir;
  m_fDeadzone = rhs.m_fDeadzone;
  m_strDesc   = rhs.m_strDesc;

  return *this;
}

void LaeVL6180SensorInfo::getProps(int               &nIndex,
                                   int               &nChan,
                                   std::string       &strRadiationType,
                                   double            &fFoV,
                                   double            &fBeamDir,
                                   double            &fDeadzone,
                                   double            &fMin,
                                   double            &fMax,
                                   std::string       &strDesc)
{
  nIndex            = m_nIndex;
  nChan             = m_nChan;
  strRadiationType  = "infrared";
  fFoV              = degToRad(VL6180X_RANGE_FOV);
  fBeamDir          = m_fBeamDir;
  fDeadzone         = m_fDeadzone;
  fMin              = VL6180X_RANGE_MIN;
  fMax              = VL6180X_RANGE_MAX;
  strDesc           = m_strDesc;
}


//------------------------------------------------------------------------------
// LaeVL6180Mux Class
//------------------------------------------------------------------------------

LaeVL6180Mux::LaeVL6180Mux(LaeI2CMux     &mux,
                           int            nChan,
                           double         fBeamDir,
                           double         fDeadzone,
                           const string  &strNameId,
                           const string  &strDesc) :
    m_mux(mux), m_nChan(nChan), m_fBeamDir(fBeamDir), m_fDeadzone(fDeadzone),
    m_strNameId(strNameId), m_strDesc(strDesc)
{
  m_nErrorCnt       = 0;
  m_bBlackListed    = false;

  m_fAlsGain        = 1.0;  // gain with the highest dynamic range
  m_uAlsIntPeriod   = 50;   // datasheet recommends the forever 100 msec

  m_regRangeOffset    = (byte_t)0;
  m_regRangeCrossTalk = (u16_t)0;
  m_regAlsGain        = (byte_t)gainAnalogToEnum(m_fAlsGain);
  m_regAlsIntPeriod   = (u16_t)(m_uAlsIntPeriod - 1);

  m_fRange          = 0.0;
  m_fAmbientLight   = 0.0;

  pthread_mutex_init(&m_mutex, NULL);
}

LaeVL6180Mux::LaeVL6180Mux(const LaeVL6180Mux &src) :
    m_mux(src.m_mux), m_nChan(src.m_nChan),
    m_fBeamDir(src.m_fBeamDir), m_fDeadzone(src.m_fDeadzone),
    m_strNameId(src.m_strNameId)
{
  m_nErrorCnt         = src.m_nErrorCnt;
  m_bBlackListed      = src.m_bBlackListed;
  m_fAlsGain          = src.m_fAlsGain;
  m_uAlsIntPeriod     = src.m_uAlsIntPeriod;
  m_regRangeOffset    = src.m_regRangeOffset;
  m_regRangeCrossTalk = src.m_regRangeCrossTalk;
  m_regAlsGain        = src.m_regAlsGain;
  m_regAlsIntPeriod   = src.m_regAlsIntPeriod;
  m_fRange            = src.m_fRange;
  m_fAmbientLight     = src.m_fAmbientLight;

  pthread_mutex_destroy(&m_mutex);
}

LaeVL6180Mux::~LaeVL6180Mux()
{
}

int LaeVL6180Mux::initSensor(bool bForce)
{
  byte_t  reset;
  bool    bReinit = true;
  int     rc = LAE_OK;

  m_nErrorCnt     = 0;
  m_bBlackListed  = false;

  if( !bForce )
  {
    rc = readReg8(VL6180X_SYSTEM_FRESH_OUT_OF_RESET, reset);

    if( (rc == LAE_OK) && (reset == 0) )
    {
      LOGDIAG3("VL6180 sensor %s(%d): "
        "Not fresh out of reset: No initialization required.",
        m_strNameId.c_str(), m_nChan);
      bReinit = false;
    }
  }

  if( bReinit )
  {
    if( (rc = outOfResetInit()) == LAE_OK )
    {
      rc = writeDefaults();
    }
  }

  readShadowRegs();

  if( rc == LAE_OK )
  {
    LOGDIAG3("VL6180 sensor %s(%d): Initialized.",
      m_strNameId.c_str(), m_nChan);
  }

  else
  {
    m_bBlackListed = true;
    LOGERROR("VL6180 sensor %s(%d): Black listed: Failed to initialize.",
      m_strNameId.c_str(), m_nChan);
  }

  return rc;
}

int LaeVL6180Mux::outOfResetInit()
{ 
  int     rc;

  //
  // Required by datasheet.
  // http://www.st.com/st-web-ui/static/active/en/resource/technical/document/application_note/DM00122600.pdf
  //
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

  // 
  // Clear fresh reset bit. Default is 1 after boot/reset. Setting to 0 can
  // be used to detect reset condition.
  //
  rc = writeReg8(VL6180X_SYSTEM_FRESH_OUT_OF_RESET, 0);

  return rc;
}

int LaeVL6180Mux::writeDefaults()
{
  // Recommended settings from datasheet
  // http://www.st.com/st-web-ui/static/active/en/resource/technical/document/application_note/DM00122600.pdf

  // Disable interrupts.
  //writeReg8(VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO, 0);
  // Set GPIO1 high when sample complete
  writeReg8(VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO, (4 << 3)|(4));

  // Set GPIO1 high when sample complete. N/A since interrupts are disabled.
  writeReg8(VL6180X_SYSTEM_MODE_GPIO1, 0x10);

  // Set the readout averaging sample period.
  // RDK: Possible TUNE parameter.
  writeReg8(VL6180X_READOUT_AVERAGING_SAMPLE_PERIOD, 48);

  // Set the ALS analog gain.
  writeReg8(VL6180X_SYSALS_ANALOGUE_GAIN, (byte_t)(0x40 | m_regAlsGain));

  // Set auto calibration period (Max = 255)/(OFF = 0).
  writeReg8(VL6180X_SYSRANGE_VHV_REPEAT_RATE, 0xFF);

  // Perform a single temperature calibration
  writeReg8(VL6180X_SYSRANGE_VHV_RECALIBRATE, 0x01);

  // Set ALS integration time in msec.
  writeReg16(VL6180X_SYSALS_INTEGRATION_PERIOD, m_regAlsIntPeriod);

  // Optional settings from datasheet
  // http://www.st.com/st-web-ui/static/active/en/resource/technical/document/application_note/DM00122600.pdf
 
  // Set default ranging inter-measurement period to 100ms.
  // RDK: Used in interleaved mode which may be the optimal mode.
  writeReg8(VL6180X_SYSRANGE_INTERMEASUREMENT_PERIOD, 0x09);

  // Set default ALS inter-measurement period to 100ms.
  // RDK: Used in interleaved mode which may be the optimal mode.
  writeReg8(VL6180X_SYSALS_INTERMEASUREMENT_PERIOD, 0x0A);

  // Additional settings defaults from community
  // RDK Investigate
  writeReg8(VL6180X_SYSRANGE_MAX_CONVERGENCE_TIME, 0x32);
  writeReg8(VL6180X_SYSRANGE_RANGE_CHECK_ENABLES, 0x10 | 0x01);
  writeReg16(VL6180X_SYSRANGE_EARLY_CONVERGENCE_ESTIMATE, 0x7B );

  writeReg8(VL6180X_SYSRANGE_RANGE_IGNORE_VALID_HEIGHT, 255);

  writeReg8(VL6180X_FIRMWARE_RESULT_SCALER, 0x01);
}

int LaeVL6180Mux::tune(uint_t uRangeOffset, uint_t uRangeCrossTalk,
                       double fAlsGain, uint_t uAlsIntPeriod)
{
  byte_t  regRangeOffset;
  u16_t   regRangeCrossTalk;
  byte_t  regAlsGain;
  u16_t   regAlsIntPeriod;
  int     rc;

  //
  // Convert to new register values.
  //
  if( (int)uRangeOffset == VL6180X_FACTORY_DFT)
  {
    regRangeOffset = m_regRangeOffset;
  }
  else
  {
    regRangeOffset = (byte_t)cap((int)uRangeOffset,
                                  VL6180X_RANGE_OFFSET_MIN,
                                  VL6180X_RANGE_OFFSET_MAX);
  }

  if( (int)uRangeCrossTalk == VL6180X_FACTORY_DFT)
  {
    regRangeCrossTalk = m_regRangeCrossTalk;
  }
  else
  {
    regRangeCrossTalk = (u16_t)cap(uRangeCrossTalk,
                                    (uint_t)VL6180X_RANGE_XTALK_MIN,
                                    (uint_t)VL6180X_RANGE_XTALK_MAX);
  }

  // map analog to register enum, then find exact analog gain value
  regAlsGain = (byte_t)gainAnalogToEnum(fAlsGain);
  fAlsGain   = gainEnumToAnalog((vl6180x_als_gain)regAlsGain);

  uAlsIntPeriod = cap(uAlsIntPeriod, (uint_t)VL6180X_AMBIENT_INT_T_MIN,
                                     (uint_t)VL6180X_AMBIENT_INT_T_MAX);

  regAlsIntPeriod = (u16_t)(uAlsIntPeriod - 1);

  //
  // New offset value.
  //
  if( regRangeOffset != m_regRangeOffset )
  {
    rc = writeReg8(VL6180X_SYSRANGE_PART_TO_PART_RANGE_OFFSET, regRangeOffset);
    if( rc == LAE_OK )
    {
      m_regRangeOffset = regRangeOffset;
    }
    else
    {
      LOGDIAG3("Failed to write new range part-to-part "
                "offset register value %u.", regRangeOffset);
    }
  }

  //
  // New cross-talk value.
  //
  if( regRangeCrossTalk != m_regRangeCrossTalk )
  {
    rc = writeReg16(VL6180X_SYSRANGE_CROSSTALK_COMPENSATION_RATE,
                      regRangeCrossTalk);
    if( rc == LAE_OK )
    {
      m_regRangeOffset = regRangeOffset;
    }
    else
    {
      LOGDIAG3("Failed to write new range cross-talk compensation "
                "register value %u.", regRangeCrossTalk);
    }
  }

  //
  // New gain value.
  //
  if( regAlsGain != m_regAlsGain )
  {
    rc = writeReg8(VL6180X_SYSALS_ANALOGUE_GAIN, (byte_t)(0x40|regAlsGain));
    if( rc == LAE_OK )
    {
      m_fAlsGain   = fAlsGain;
      m_regAlsGain = regAlsGain;
    }
    else
    {
      LOGDIAG3("Failed to write new ALS gain register value %u.", regAlsGain);
    }
  }

  //
  // New integration period value.
  //
  if( regAlsIntPeriod != m_regAlsIntPeriod )
  {
    rc = writeReg16(VL6180X_SYSALS_INTEGRATION_PERIOD, regAlsIntPeriod);
    if( rc == LAE_OK )
    {
      m_uAlsIntPeriod   = uAlsIntPeriod;
      m_regAlsIntPeriod = regAlsIntPeriod;
    }
    else
    {
      LOGDIAG3("Failed to write new ALS integration period register value %u.",
          regAlsIntPeriod);
    }
  }

  // always return success for now
  return LAE_OK;
}

void LaeVL6180Mux::readShadowRegs()
{
  lock();

  readReg8(VL6180X_SYSRANGE_PART_TO_PART_RANGE_OFFSET, m_regRangeOffset);
  readReg16(VL6180X_SYSRANGE_CROSSTALK_COMPENSATION_RATE, m_regRangeCrossTalk);
  readReg8(VL6180X_SYSALS_ANALOGUE_GAIN, m_regAlsGain);
  readReg16(VL6180X_SYSALS_INTEGRATION_PERIOD, m_regAlsIntPeriod);

  //
  // Massage
  //
  m_regAlsGain      &= VL6180X_AMBIENT_GAIN_MASK;
  m_regAlsIntPeriod &= VL6180X_AMBIENT_INT_T_MASK;
  m_fAlsGain        = gainEnumToAnalog((vl6180x_als_gain)m_regAlsGain);
  m_uAlsIntPeriod = m_regAlsIntPeriod + 1;

  unlock();
}

void LaeVL6180Mux::readShadowRegs(byte_t &regRangeOffset,
                                  u16_t  &regRangeCrossTalk,
                                  byte_t &regAlsGain,
                                  u16_t  &regAlsIntPeriod)
{
  readShadowRegs();

  regRangeOffset    = m_regRangeOffset;
  regRangeCrossTalk = m_regRangeCrossTalk;
  regAlsGain        = m_regAlsGain;
  regAlsIntPeriod     = m_regAlsIntPeriod;
}

int LaeVL6180Mux::readId(struct VL6180xIdentification &id)
{
  lock();

  readReg8(VL6180X_IDENTIFICATION_MODEL_ID, id.idModel);
  readReg8(VL6180X_IDENTIFICATION_MODEL_REV_MAJOR, id.idModelRevMajor);
  readReg8(VL6180X_IDENTIFICATION_MODEL_REV_MINOR, id.idModelRevMinor);
  readReg8(VL6180X_IDENTIFICATION_MODULE_REV_MAJOR, id.idModuleRevMajor);
  readReg8(VL6180X_IDENTIFICATION_MODULE_REV_MINOR, id.idModuleRevMinor);

  readReg16(VL6180X_IDENTIFICATION_DATE, id.idDate);
  readReg16(VL6180X_IDENTIFICATION_TIME, id.idTime);

  unlock();

  return LAE_OK;
}

double LaeVL6180Mux::measureRange(u32_t msecWait)
{
  static byte_t startMeas             = 0x01; // start measurement
  static byte_t measDone              = 0x04; // measurement done bit
  static byte_t statusErrMask         = 0xf0; // status error bit mask
  static byte_t statusErrOverflowRaw  = 0xd0; // raw range overflow
  static byte_t statusErrOverflow     = 0xf0; // range overflow
  static byte_t clearInterrupts       = 0x07; // clear all interrupts

  double  fRange;       // working measured range
  byte_t  valRange;     // raw range value
  byte_t  valStatus;    // range sensor status
  u32_t   msec;         // milliseconds
  int     rc = LAE_OK;  // return code

  if( m_bBlackListed )
  {
    return VL6180X_ERR_MEAS;
  }

  lock();

  // auto-determine max wait time
  if( msecWait == VL6180X_T_AUTO )
  {
    msecWait = 15;
  }

  //
  // Wait for device and sensor to be ready.
  //
  msec = waitForSensorReady(VL6180X_RESULT_RANGE_STATUS, msecWait);

  if( msec > msecWait )
  {
    LOGDIAG3("VL6180 sensor %s(%d): Range sensor busy.",
      m_strNameId.c_str(), m_nChan);
    rc = -LAE_ECODE_TIMEDOUT;
  }

  //
  // Start single shot measurement.
  //
  if( rc == LAE_OK )
  {
    rc = writeReg8(VL6180X_SYSRANGE_START, startMeas);
  }

  //
  // Wait for range sensor measurement to complete.
  //
  if( rc == LAE_OK )
  {
    msec += waitForSensorMeasurement(msecWait-msec, measDone);

    //
    // Note:
    //  Typically, this timeout indicates that there a no objects in range.
    //  If it is logged, the too many errors will be printed. 
    //
    if( msec > msecWait )
    {
      LOGDIAG3("VL6180 sensor %s(%d): Range measurement timed out.",
        m_strNameId.c_str(), m_nChan);
      rc = -LAE_ECODE_TIMEDOUT;
    }
  }

  writeReg8(VL6180X_SYSTEM_INTERRUPT_CLEAR, clearInterrupts);

  //
  // Read range value (mm) and status.
  //
  if( rc == LAE_OK )
  {
    if( (rc = readReg8(VL6180X_RESULT_RANGE_VAL, valRange)) == LAE_OK )
    {
      //rc = readReg8(VL6180X_RESULT_RANGE_STATUS, valStatus);
      //fprintf(stderr, "DBG: range_status=0x%02x\n", val);
    }
  }

  //
  // Validate measurement and convert to SI units.
  //
  if( rc == LAE_OK )
  {
    // relevant bits
    //valStatus &= statusErrMask;

    // Sensor overflow error. Usually indicates no detected object.
    //if( (valStatus == statusErrOverflowRaw) ||
    //    (valStatus == statusErrOverflow) )
    //{
    //  fRange = VL6180X_RANGE_NO_OBJ;
    //}

    // other bits indicate sensor error
    //else if( valStatus )
    //{
    //  rc = -LAE_ECODE_BAD_VAL;
    //}

    // good
    //else
    //{
      // meters
      fRange = (double)valRange / 1000.0;

      // range sensor not trusted after maximum distance
      if( fRange > VL6180X_RANGE_MAX )
      {
        fRange = VL6180X_RANGE_NO_OBJ;
      } 
    //}
  }

  //
  // Save
  //
  if( rc == LAE_OK )
  {
    m_fRange    = fRange;
    m_nErrorCnt = 0;
    RtDb.m_range[m_nChan].m_fRange = m_fRange;
  }
  else if( rc == -LAE_ECODE_TIMEDOUT )
  {
    fRange   = VL6180X_RANGE_NO_OBJ;
    m_fRange = fRange;
    ++m_nErrorCnt;
    RtDb.m_range[m_nChan].m_fRange = m_fRange;
  }
  else
  {
    fRange = VL6180X_ERR_MEAS;
    ++m_nErrorCnt;
  }

  if( m_nErrorCnt > NSenseErrorsThreshold )
  {
    m_bBlackListed = true;
  }

  unlock();

  return fRange;
}

double LaeVL6180Mux::measureAmbientLight(u32_t msecWait)
{
  static byte_t startMeas     = 0x01; // start measurement
  static byte_t measDone      = 0x20; // measurement done bit
  static byte_t statusErrMask = 0xf0; // status error bit mask
  static double alsRes        = 0.32; // .32 lux/count (default without window)

  double  alsLux;       // working lux value
  u16_t   valAlsRaw;    // raw als value
  byte_t  valStatus;    // als sensor status
  u32_t   msec;         // milliseconds
  int     rc = LAE_OK;  // return code

  if( m_bBlackListed )
  {
    return VL6180X_ERR_MEAS;
  }

  lock();

  // auto-determine max wait time
  if( msecWait == VL6180X_T_AUTO )
  {
    msecWait = (u32_t)((double)m_uAlsIntPeriod * 1.2);
  }

  //
  // Wait for sensor to be ready.
  //
  msec = waitForSensorReady(VL6180X_RESULT_ALS_STATUS, msecWait);

  if( msec > msecWait )
  {
    LOGDIAG3("VL6180 sensor %s(%d): Ambient light sensor busy.",
      m_strNameId.c_str(), m_nChan);
    rc = -LAE_ECODE_TIMEDOUT;
  }

  //
  // Start single shot measurement.
  //
  if( rc == LAE_OK )
  {
    rc = writeReg8(VL6180X_SYSALS_START, startMeas);
  }

  //
  // Wait for ambient sensor measurement to complete.
  //
  if( rc == LAE_OK )
  {
    msec += waitForSensorMeasurement(msecWait-msec, measDone);

    if( msec > msecWait )
    {
      LOGDIAG3("VL6180 sensor %s(%d): Ambient light measurement timed out.",
        m_strNameId.c_str(), m_nChan);
      rc = -LAE_ECODE_TIMEDOUT;
    }
  }

  //
  // Read ALS raw value and status.
  //
  if( rc == LAE_OK )
  {
    if( (rc = readReg16(VL6180X_RESULT_ALS_VAL, valAlsRaw)) == LAE_OK )
    {
      rc = readReg8(VL6180X_RESULT_ALS_STATUS, valStatus);
    }
  }
  
  //
  // Validate measurement and convert to SI units.
  //
  if( rc == LAE_OK )
  {
    // relevant bits
    valStatus &= statusErrMask;

    // bits indicate sensor error
    if( valStatus )
    {
      rc = -LAE_ECODE_BAD_VAL;
    }

    // good
    else
    {
      // calculate LUX from formula in AppNotes
      alsLux = alsRes * ((double)valAlsRaw / m_fAlsGain) * 
                          100.0 / (double)(m_uAlsIntPeriod);
    }
  }

  //
  // Save
  //
  if( rc == LAE_OK )
  {
    m_fAmbientLight = alsLux;
    m_nErrorCnt     = 0;
    RtDb.m_range[m_nChan].m_fAmbientLight = m_fAmbientLight;
  }
  else
  {
    alsLux = VL6180X_ERR_MEAS;
    ++m_nErrorCnt;
  }

  if( m_nErrorCnt > NSenseErrorsThreshold )
  {
    m_bBlackListed = true;
  }

  unlock();

  return alsLux;
}

u32_t LaeVL6180Mux::waitForSensorReady(u16_t regStatus, u32_t msecWait)
{
  static byte_t statusReady = 0x01; // device and sensor is ready bit

  byte_t  val;
  u32_t   msec;

  //
  // Wait for sensor device to become available.
  //
  for(msec = 0; msec <= msecWait; ++msec)
  {
    // sensor not busy
    if( (readReg8(regStatus, val) == LAE_OK) && (val & statusReady) )
    {
      return msec;
    }

    // sleep 1 msec == 1000 usec
    else
    {
      usleep(1000);
    }
  }

  return msec;
}

u32_t LaeVL6180Mux::waitForSensorMeasurement(u32_t msecWait, byte_t bitDone)
{
  static byte_t clearInterrupts = 0x07;   // clear all interrupts

  byte_t  val;
  u32_t   msec;

  //
  // Wait for sensor device to become available.
  //
  for(msec = 0; msec <= msecWait; ++msec)
  {
    if( readReg8(VL6180X_RESULT_INTERRUPT_STATUS_GPIO, val) == LAE_OK )
    {
      // sensor not busy
      if( val & bitDone )
      {
      //  writeReg8(VL6180X_SYSTEM_INTERRUPT_CLEAR, clearInterrupts);
        return msec;
      }

      // sleep 1 msec == 1000 usec
      else
      {
        usleep(1000);
      }
    }
  }

  //fprintf(stderr, "DBG: status_gpio=0x%02x\n", val);
 
  return msec;
}

int LaeVL6180Mux::calibOffset(int    &nOffsetPre,
                              double &fAvgPre,
                              int    &nOffsetPost,
                              double &fAvgPost)
{
  u16_t  regOffset  = VL6180X_SYSRANGE_PART_TO_PART_RANGE_OFFSET;
  double fWhiteTgt  = 0.050;  // required white target distance
  double fRes       = 0.003;  // range plus/minus resolution is best possible
  int    minGood    = 10;     // minimum number of measurement to be valid
  int    maxIters   = 20;     // should >= min as per data sheet

  s8_t    offset;     // part-to-part offset in 2's compliment
  double  fRange;     // range measurement
  double  fSum;       // sum
  int     nMeasured;  // number of sucessful range measurements
  int     i;          // iterator
  int     rc;         // return code

  //
  // Read current, pre-calibrated offset register value.
  //
  if( (rc = readReg8(regOffset, (byte_t &)offset)) == LAE_OK )
  {
    m_regRangeOffset = (byte_t)offset;  // update shadow register
  }
  else
  {
    LOGDIAG3("Failed to read SYSRANGE_PART_TO_PART_RANGE_OFFSET (0x%04x) "
              "register. Cannot proceed with calibration.",
              regOffset);
    return rc;
  }

  //
  // Make a set of measurements to gather performance statistics.
  //
  for(i = 0, fSum = 0.0, nMeasured = 0; i<maxIters; ++i)
  {
    fRange = measureRange();
    if( (fRange >= 0.0) && (fRange <= VL6180X_RANGE_MAX) )
    {
      fSum += fRange;
      ++nMeasured;
    }
  }

  //
  // Not of enough good measurements to determine performance.
  //
  if( nMeasured < minGood )
  {
    LOGDIAG3("Made %d/%d range meassurements. A minimum of %d are required."
              "Cannot proceed with calibration.",
              nMeasured, minGood);
    return -LAE_ECODE_IO;
  }

  nOffsetPre  = (int)offset;
  fAvgPre     = fSum / (double)nMeasured;

  //
  // No part-to-part offset calibaration required.
  //
  if( (fAvgPre >= fWhiteTgt-fRes) && (fAvgPre <= fWhiteTgt+fRes) )
  {
    nOffsetPost = nOffsetPre;
    fAvgPost    = fAvgPre;
    return LAE_OK;
  }

  //
  // Clear offset register.
  //
  if( (rc = writeReg8(regOffset, 0)) == LAE_OK )
  {
    m_regRangeOffset = 0;  // update shadow register
  }
  else
  {
    LOGDIAG3("Failed to write SYSRANGE_PART_TO_PART_RANGE_OFFSET (0x%04x) "
              "register. Cannot proceed with calibration.",
              regOffset);
    return rc;
  }

  //
  // Make another set of measurements.
  //
  for(i = 0, fSum = 0.0, nMeasured = 0; i<maxIters; ++i)
  {
    fRange = measureRange();
    if( (fRange >= 0.0) && (fRange <= VL6180X_RANGE_MAX) )
    {
      fSum += fRange;
      ++nMeasured;
    }
  }

  //
  // Not of enough good measurements to determine calibration offset.
  //
  if( nMeasured < minGood )
  {
    LOGDIAG3("Made %d/%d range meassurements. A minimum of %d are required."
              "Cannot proceed with calibration.",
              nMeasured, minGood);
    return -LAE_ECODE_IO;
  }

  fAvgPost    = fSum / (double)nMeasured;
  nOffsetPost = (int)((fWhiteTgt - fAvgPost) * 1000.0);

  offset = (s8_t)(nOffsetPost);

  //
  // Write new offset value.
  //
  if( (rc = writeReg8(regOffset, (byte_t)offset)) == LAE_OK )
  {
    m_regRangeOffset = (byte_t)offset;  // update shadow register
  }
  else
  {
    LOGDIAG3("Failed to write new SYSRANGE_PART_TO_PART_RANGE_OFFSET (0x%04x) "
              "register. Calibration failed.",
              regOffset);
    return rc;
  }

  //
  // One final performance evaluation.
  //
  for(i = 0, fSum = 0.0, nMeasured = 0; i<maxIters; ++i)
  {
    fRange = measureRange();
    if( (fRange >= 0.0) && (fRange <= VL6180X_RANGE_MAX) )
    {
      fSum += fRange;
      ++nMeasured;
    }
  }

  //
  // Not of enough good measurements to determine performance.
  //
  if( nMeasured < minGood )
  {
    LOGWARN("Made %d/%d range meassurements. A minimum of %d are required."
              "Post performance evaluation is unknown.",
              nMeasured, minGood);
  }
  else
  {
    fAvgPost = fSum / (double)nMeasured;
  }

  return LAE_OK;
}

int LaeVL6180Mux::calibCrossTalk(int &nCrossTalk)
{
  u16_t  regComp    = VL6180X_SYSRANGE_CROSSTALK_COMPENSATION_RATE;
  u16_t  regRate    = VL6180X_RESULT_RANGE_RETURN_RATE;
  double fBlackTgt  = 0.100;  // required black target distance
  int    minGood    = 10;     // minimum number of measurement to be valid
  int    maxIters   = 20;     // should >= min as per data sheet

  byte_t  val;        // register value
  double  fRate;      // range return rate measurement
  double  fSumRate;   // return rate sum
  double  fAvgRate;   // return rate average
  double  fRange;     // range measurement
  double  fSumRange;  // range sum
  double  fAvgRange;  // range average
  int     nMeasured;  // number of sucessful range measurements
  int     i;          // iterator
  int     rc;         // return code

  //
  // Clear compensation register.
  //
  if( (rc = writeReg16(regComp, 0)) == LAE_OK )
  {
    m_regRangeCrossTalk = 0;  // update shadow register
  }
  else
  {
    LOGDIAG3("Failed to write SYSRANGE_CROSSTALK_COMPENSATION_RATE (0x%04x) "
              "register. Cannot proceed with calibration.",
              regComp);
    return rc;
  }

  //
  // Make set of measurements.
  //
  for(i = 0, fSumRange = 0.0, fSumRate = 0.0, nMeasured = 0; i<maxIters; ++i)
  {
    fRange = measureRange();
    rc = readReg8(regRate, val);

    if( (fRange >= 0.0) && (fRange <= VL6180X_RANGE_MAX) && (rc == LAE_OK) )
    {
      fSumRange += fRange;
      fSumRate  += (double)val;
      ++nMeasured;
    }
  }

  //
  // Not of enough good measurements to determine calibration offset.
  //
  if( nMeasured < minGood )
  {
    LOGDIAG3("Made %d/%d rate/range meassurements. "
            "A minimum of %d are required."
             "Cannot proceed with calibration.",
              nMeasured, minGood);
    return -LAE_ECODE_IO;
  }

  fAvgRange = fSumRange / (double)nMeasured;
  fAvgRate  = fSumRate / (double)nMeasured;

  val = (byte_t)(fAvgRate * (1.0 - fAvgRange/fBlackTgt));

  //
  // Write new compensation value.
  //
  if( (rc = writeReg16(regComp, val)) == LAE_OK )
  {
    m_regRangeCrossTalk = val;    // update shadow register
  }
  else
  {
    LOGDIAG3("Failed to write new SYSRANGE_CROSSTALK_COMPENSATION_RATE (0x%04x)"
              " register value. Calibration failed.",
              regComp);

    return rc;
  }

  nCrossTalk = (int)val & 0x0ff;

  return LAE_OK;
}

double LaeVL6180Mux::gainEnumToAnalog(vl6180x_als_gain eAlsGain)
{
  if( (eAlsGain >= 0) && (eAlsGain < GAIN_NumOf) )
  {
    return AlsGainAnalogTbl[eAlsGain];
  }
  else
  {
    return AlsGainAnalogTbl[GAIN_1];
  }
}

vl6180x_als_gain LaeVL6180Mux::gainAnalogToEnum(double fAlsGain)
{
  int       i;
  int       eGain;
  double    diff, minDiff;

  fAlsGain = fabs(fAlsGain);

  if( fAlsGain >= AlsGainAnalogTbl[GAIN_40] )
  {
    return GAIN_40;
  }
  else if( fAlsGain <= AlsGainAnalogTbl[GAIN_1] )
  {
    return GAIN_1;
  }

  eGain   = 0;  
  minDiff = fabs(AlsGainAnalogTbl[eGain] - fAlsGain);

  //
  // Find closest enum.
  //
  for(i=1; i<GAIN_NumOf; ++i)
  {
    diff = fabs(AlsGainAnalogTbl[i] - fAlsGain);
    if( diff < minDiff )
    {
      eGain   = i;
      minDiff = diff;
    }
  }

  return (vl6180x_als_gain)eGain;
}

int LaeVL6180Mux::readReg8(u16_t reg, byte_t &val)
{
  byte_t  req[2];
  int     n = 0;
  int     rc;

  req[n++] = (byte_t)((reg >> 8) & 0xff);
  req[n++] = (byte_t)(reg & 0xff);

  rc = m_mux.transact(m_nChan, VL6180X_ADDR, req, n, &val, 1, 0);

  if( rc != LAE_OK )
  {
    LOGDIAG3("VL6180 sensor %s(%d): Failed to read register 0x%04x.",
      m_strNameId.c_str(), m_nChan, reg);
  }

  return rc;
}

int LaeVL6180Mux::readReg16(u16_t reg, u16_t &val)
{
  byte_t  req[2];
  byte_t  rsp[2];
  int     n = 0;
  int     rc;

  req[n++] = (byte_t)((reg >> 8) & 0xff);
  req[n++] = (byte_t)(reg & 0xff);

  rc = m_mux.transact(m_nChan, VL6180X_ADDR, req, n, rsp, 2, 0);

  if( rc == LAE_OK )
  {
    val = ((u16_t)(rsp[0]) << 8) | (u16_t)rsp[1];
  }
  else
  {
    LOGDIAG3("VL6180 sensor %s(%d): Failed to read register 0x%04x.",
      m_strNameId.c_str(), m_nChan, reg);
  }

  return rc;
}

int LaeVL6180Mux::writeReg8(u16_t reg, byte_t val)
{
  byte_t  req[3];
  int     n = 0;
  int     k;
  int     rc;

  req[n++] = (byte_t)((reg >> 8) & 0xff);
  req[n++] = (byte_t)(reg & 0xff);
  req[n++] = val;

  if( (k = m_mux.write(m_nChan, VL6180X_ADDR, req, n)) == n )
  {
    rc = LAE_OK;
  }
  else
  {
    LOGDIAG3("VL6180 sensor %s(%d): Failed to write register 0x%04x.",
      m_strNameId.c_str(), m_nChan, reg);
    rc = -LAE_ECODE_IO;
  }

  return rc;
}

int LaeVL6180Mux::writeReg16(u16_t reg, u16_t val)
{
  byte_t  req[4];
  int     n = 0;
  int     k;
  int     rc;

  req[n++] = (byte_t)((reg >> 8) & 0xff);
  req[n++] = (byte_t)(reg & 0xff);
  req[n++] = (byte_t)((val >> 8) & 0xff);
  req[n++] = (byte_t)(val & 0xff);

  if( (k = m_mux.write(m_nChan, VL6180X_ADDR, req, n)) == n )
  {
    rc = LAE_OK;
  }
  else
  {
    LOGDIAG3("VL6180 sensor %s(%d): Failed to write register 0x%04x.",
      m_strNameId.c_str(), m_nChan, reg);
    rc = -LAE_ECODE_IO;
  }

  return rc;
}


//------------------------------------------------------------------------------
// LaeVL6180MuxArray Class
//------------------------------------------------------------------------------

LaeVL6180MuxArray::LaeVL6180MuxArray(laelaps::LaeI2C &i2cBus) :
    LaeRangeInterface(i2cBus), m_mux(i2cBus)
{
  m_nAlsIndex   = 0;
  m_nAlsCounter = AlsFreq;
}

LaeVL6180MuxArray::~LaeVL6180MuxArray()
{
  for(size_t i = 0; i < m_vecToF.size(); ++i)
  {
    delete m_vecToF[i];
  }
  m_vecToF.clear();

}

int LaeVL6180MuxArray::getInterfaceVersion(uint_t &uVerMajor,
                                           uint_t &uVerMinor,
                                           uint_t &uFwVer)
{
  uVerMajor = LAE_VER_MAJOR(RtDb.m_product.m_uProdHwVer);
  uVerMinor = LAE_VER_MINOR(RtDb.m_product.m_uProdHwVer);
  uFwVer    = 0;

  return laelaps::LAE_OK;
}

void LaeVL6180MuxArray::clearSensedData()
{
  for(size_t i = 0; i < m_vecToF.size(); ++i)
  {
    delete m_vecToF[i];
  }
  m_vecToF.clear();
}

int LaeVL6180MuxArray::configure(const LaeDesc &desc)
{
  LaeDesc::MapDescRangeSensor::const_iterator iter;
  VL6180xIdentification   ident;
  int                     rc;

  //
  // Time of Flight proximity sensors.
  //
  for(iter = desc.m_mapDescRangeSensor.begin();
      iter != desc.m_mapDescRangeSensor.end();
      ++iter)
  {
    m_vecToF.push_back(new LaeVL6180Mux(m_mux,
                              iter->second->m_nChan,
                              iter->second->m_fDir,
                              iter->second->m_fDeadzone,
                              iter->first,
                              iter->second->m_strDesc));
  }

  //
  // Try to initialize the sensors. But any failures will map only to warnings,
  // since we want the robot to still operate.
  //
  for(size_t i = 0; i < m_vecToF.size(); ++i)
  {
    if( (rc = m_vecToF[i]->readId(ident)) < 0 )
    {
      LOGWARN("VL6180 %s sensor: Failed to read identity.",
        m_vecToF[i]->getNameId().c_str());
    }

    if( (rc = m_vecToF[i]->initSensor(true)) < 0 )
    {
      LOGWARN("VL6180 %s sensor: Failed to initialize sensor.",
        m_vecToF[i]->getNameId().c_str());
    }

    if( rc == LAE_OK )
    {
      LOGDIAG3("VL6180 %s sensor connected:\n"
          "  Location: %s\n"
          "  Model:    %u v%u.%u\n"
          "  Module:   v%u.%u\n"
          "  Date:     %u %u",
        m_vecToF[i]->getNameId().c_str(),
        m_vecToF[i]->getDesc().c_str(),
        ident.idModel, ident.idModelRevMajor, ident.idModelRevMinor,
        ident.idModuleRevMajor, ident.idModuleRevMinor,
        ident.idDate, ident.idTime);

    }
  }

  return LAE_OK;
}

int LaeVL6180MuxArray::configure(const LaeTunes &tunes)
{
  string  strKey;
  int     nRangeOffset;
  int     nRangeCrossTalk;
  double  fAlsGain;
  int     nAlsIntPeriod;
  int     rc;

  for(size_t i = 0; i < m_vecToF.size(); ++i)
  {
    strKey = m_vecToF[i]->getNameId();

    tunes.getVL6180Params(strKey, nRangeOffset, nRangeCrossTalk,
                                  fAlsGain, nAlsIntPeriod);

    rc = m_vecToF[i]->tune((uint_t)nRangeOffset, (uint_t)nRangeCrossTalk,
                            fAlsGain, (uint_t)nAlsIntPeriod);

    if( rc == LAE_OK )
    {
      LOGDIAG3("VL6180 range sensor %s tuned.", strKey.c_str());
    }
    else
    {
      LOGERROR("VL6180 range sensor %s: Failed to tune sensor.",
          strKey.c_str());
      return rc;
    }
  }

  return LAE_OK;
}

int LaeVL6180MuxArray::reload(const LaeTunes &tunes)
{
  return configure(tunes);
}

void LaeVL6180MuxArray::exec()
{
  int     nNumSensors;
  double  fRange;
  double  fAmbient;

  // no sensors
  if( (nNumSensors = (int)m_vecToF.size()) == 0 )
  {
    return;
  }

  // take all range measurements
  if( --m_nAlsCounter > 0 )
  {
    for(int sensorIndex = 0; sensorIndex < nNumSensors; ++sensorIndex)
    {
      fRange = m_vecToF[sensorIndex]->measureRange(fAmbient);
      RtDb.m_range[sensorIndex].m_fRange = fRange;
    }
  }

  // take single ambient light measurement
  else
  {
    fAmbient = m_vecToF[m_nAlsIndex]->measureAmbientLight();
    RtDb.m_range[m_nAlsIndex].m_fAmbientLight = fAmbient;

    m_nAlsIndex   = (m_nAlsIndex + 1) % nNumSensors;
    m_nAlsCounter = AlsFreq;
  }
}

int LaeVL6180MuxArray::getRange(const std::string &strKey, double &fRange)
{
  double  fAmbient;

  for(size_t i = 0; i < m_vecToF.size(); ++i)
  {
    if( m_vecToF[i]->getNameId() == strKey )
    {
      m_vecToF[i]->getMeasurements(fRange, fAmbient);
      return LAE_OK;
    }
  }

  LOGERROR("VL6180 range sensor %s does not exist.", strKey.c_str());

  return -LAE_ECODE_BAD_VAL;
}
      
int LaeVL6180MuxArray::getRange(std::vector<std::string> &vecNames,
                                std::vector<double>      &vecRanges)
{
  double  fRange;
  double  fAmbient;

  vecNames.clear();
  vecRanges.clear();

  for(size_t i = 0; i < m_vecToF.size(); ++i)
  {
    m_vecToF[i]->getMeasurements(fRange, fAmbient);

    vecNames.push_back(m_vecToF[i]->getNameId());
    vecRanges.push_back(fRange);
  }

  return LAE_OK;
}
  
int LaeVL6180MuxArray::getAmbientLight(const std::string &strKey, double &fLux)
{
  double    fRange;

  for(size_t i = 0; i < m_vecToF.size(); ++i)
  {
    if( m_vecToF[i]->getNameId() == strKey )
    {
      m_vecToF[i]->getMeasurements(fRange, fLux);
      return LAE_OK;
    }
  }

  LOGERROR("VL6180 range sensor %s does not exist.", strKey.c_str());

  return -LAE_ECODE_BAD_VAL;
}
  
int LaeVL6180MuxArray::getAmbientLight(std::vector<std::string> &vecNames,
                                       std::vector<double>      &vecLux)
{
  double  fRange;
  double  fAmbient;

  vecNames.clear();
  vecLux.clear();

  for(size_t i = 0; i < m_vecToF.size(); ++i)
  {
    m_vecToF[i]->getMeasurements(fRange, fAmbient);

    vecNames.push_back(m_vecToF[i]->getNameId());
    vecLux.push_back(fAmbient);
  }

  return LAE_OK;
}

int LaeVL6180MuxArray::getSensorProps(const std::string &strKey,
                                      std::string       &strRadiationType,
                                      double            &fFoV,
                                      double            &fBeamDir,
                                      double            &fMin,
                                      double            &fMax)
{
  for(size_t i = 0; i < m_vecToF.size(); ++i)
  {
    if( m_vecToF[i]->getNameId() == strKey )
    {
      strRadiationType  = m_vecToF[i]->getRadiationType();
      fFoV              = m_vecToF[i]->getFoV();
      fBeamDir          = m_vecToF[i]->getBeamDir();
      m_vecToF[i]->getMinMax(fMin, fMax);

      return LAE_OK;
    }
  }

  LOGERROR("VL6180 range sensor %s does not exist.", strKey.c_str());

  return -LAE_ECODE_BAD_VAL;
}


//------------------------------------------------------------------------------
// LaeRangeMuxSubproc Class
//------------------------------------------------------------------------------

LaeRangeMuxSubproc::LaeRangeMuxSubproc(LaeI2C &i2cBus, uint_t addr) :
    LaeRangeInterface(i2cBus), m_addrSubProc(addr),
    m_vecRanges(ToFSensorMaxNumOf, 0.0),
    m_vecLux(ToFSensorMaxNumOf, 0.0)
{
  m_uFwVer = 0;

  pthread_mutex_init(&m_mutex, NULL);
}
  
LaeRangeMuxSubproc::~LaeRangeMuxSubproc()
{
  pthread_mutex_destroy(&m_mutex);
}
  
int LaeRangeMuxSubproc::getInterfaceVersion(uint_t &uVerMajor,
                                            uint_t &uVerMinor,
                                            uint_t &uFwVer)
{
  int   rc;

  rc = cmdGetFwVersion(m_uFwVer);

  uVerMajor = LAE_VER_MAJOR(RtDb.m_product.m_uProdHwVer);
  uVerMinor = LAE_VER_MINOR(RtDb.m_product.m_uProdHwVer);
  uFwVer    = m_uFwVer;

  return rc;
}

void LaeRangeMuxSubproc::clearSensedData()
{
  m_mapInfo.clear();

  for(size_t i = 0; i < m_vecRanges.size(); ++i)
  {
    m_vecRanges[i] = VL6180X_RANGE_NO_OBJ;
  }

  for(size_t i = 0; i < m_vecLux.size(); ++i)
  {
    m_vecLux[i] = 0.0;
  }
}

int LaeRangeMuxSubproc::configure(const LaeDesc &desc)
{
  LaeDesc::MapDescRangeSensor::const_iterator iter;
  SensorInfoMap::iterator iter2;
  uint_t                  uFwVer;
  VL6180xIdentification   ident;
  uint_t                  uRangeOffset;
  uint_t                  uRangeCrossTalk;
  double                  fAlsGain;
  uint_t                  uAlsIntPeriod;
  s8_t                    nOffset;
  int                     rc;

  //
  // Read firmware version, if necessary.
  //
  if( m_uFwVer == 0 )
  {
    if( (rc = cmdGetFwVersion(uFwVer)) == LAE_OK )
    {
      LOGDIAG2("ToFMux firmware version: %u.", uFwVer);
    }
    else
    {
      LOGERROR("Failed to read ToFMux firmware version.");
    }
  }
 
  //
  // Product Time of Flight proximity sensors.
  //
  for(iter = desc.m_mapDescRangeSensor.begin();
      iter != desc.m_mapDescRangeSensor.end();
      ++iter)
  {
    // map key - index assoc. (channel number is index for now)
    m_mapInfo[iter->first] = LaeVL6180SensorInfo(iter->second->m_nChan,
                                                 iter->second->m_nChan,
                                                 iter->second->m_fDir,
                                                 iter->second->m_fDeadzone,
                                                 iter->second->m_strDesc);
  }

  //
  // Get identity on each sensor.
  //
  for(iter2 = m_mapInfo.begin(); iter2 != m_mapInfo.end(); ++iter2)
  {
    rc = cmdGetIdent(iter2->first, ident);

    if( rc == LAE_OK )
    {
      LOGDIAG2("VL6180 %s(%d) sensor:\n"
          "  Location: %s\n"
          "  Model:    %u v%u.%u\n"
          "  Module:   v%u.%u\n"
          "  Date:     %u/%u",
        iter2->first.c_str(), iter2->second.m_nIndex,
        iter2->second.m_strDesc.c_str(),
        ident.idModel, ident.idModelRevMajor, ident.idModelRevMinor,
        ident.idModuleRevMajor, ident.idModuleRevMinor,
        ident.idDate, ident.idTime);
    }
    else
    {
      LOGERROR("VL6180 range %s sensor: Failed to read identify.",
          iter2->first.c_str());
    }

    rc = cmdGetTunes(iter2->first, uRangeOffset, uRangeCrossTalk,
                                    fAlsGain, uAlsIntPeriod);

    nOffset = (s8_t)(uRangeOffset & 0xff);

    if( rc == LAE_OK )
    {
      LOGDIAG2("VL6180 %s(%d) sensor: Tuning register values\n"
          "  ToF Offset:              %d(%u)\n"
          "  ToF CrossTalk:           %u\n"
          "  ALS Gain:                %.2lf\n"
          "  ALS Integration Period:  %u",
        iter2->first.c_str(), iter2->second.m_nIndex,
        nOffset, uRangeOffset, uRangeCrossTalk,
        fAlsGain, uAlsIntPeriod);
    }
  }

  return LAE_OK;
}

int LaeRangeMuxSubproc::configure(const LaeTunes &tunes)
{
  int     nRangeOffset;
  int     nRangeCrossTalk;
  double  fAlsGain;
  int     nAlsIntPeriod;
  int     rc;

  SensorInfoMap::iterator iter;

  for(iter = m_mapInfo.begin(); iter != m_mapInfo.end(); ++iter)
  {
    tunes.getVL6180Params(iter->first,  nRangeOffset, nRangeCrossTalk,
                                        fAlsGain, nAlsIntPeriod);

    //
    // Time-of-flight sensor tuning.
    //
    if( (nRangeOffset != VL6180X_FACTORY_DFT) &&
        (nRangeCrossTalk != VL6180X_FACTORY_DFT) )
    {
      rc = cmdTuneToFSensor(iter->first,  (uint_t)nRangeOffset,
                                          (uint_t)nRangeCrossTalk);

      if( rc -= LAE_OK )
      {
        LOGDIAG3("VL6180 range sensor %s: Time-of-flight sensor tuned.",
            iter->first.c_str());
      }
      else
      {
        LOGERROR("VL6180 range sensor %s: Failed to tune time-of-flight.",
            iter->first.c_str());
        return rc;
      }
    }
    
    //
    // Ambient light sensor tuning.
    //
    rc = cmdTuneAls(iter->first, fAlsGain, (uint_t)nAlsIntPeriod);

    if( rc == LAE_OK )
    {
      LOGDIAG3("VL6180 range sensor %s: Ambient light sensor tuned.",
            iter->first.c_str());
    }
    else
    {
      LOGERROR("VL6180 range sensor %s: Failed to tune ambient light sensor.",
            iter->first.c_str());
      return rc;
    }
  }

  return rc;
}

int LaeRangeMuxSubproc::reload(const LaeTunes &tunes)
{
  return configure(tunes);
}

void LaeRangeMuxSubproc::exec()
{
  cmdGetRanges();
  cmdGetAmbientLight();
}
      
int LaeRangeMuxSubproc::cmdGetFwVersion(uint_t &uVerNum)
{
  byte_t  cmd[LaeToFMuxI2CMaxCmdLen];
  byte_t  rsp[LaeToFMuxI2CMaxRspLen];
  size_t  lenCmd = 0;
  size_t  lenRsp = LaeToFMuxI2CRspLenGetVersion;
  int     rc;

  lock();

  cmd[lenCmd++] = LaeToFMuxI2CCmdIdGetVersion;

  rc = m_i2cBus.write_read(m_addrSubProc, cmd, lenCmd, rsp, lenRsp, TStd);

  if( rc == LAE_OK )
  {
    uVerNum = (uint_t)rsp[0];
    m_uFwVer = uVerNum;
    RtDb.m_product.m_uToFMuxFwVer = m_uFwVer;
  }

  unlock();

  return rc;
}
  
int LaeRangeMuxSubproc::cmdGetIdent(const std::string &strKey,
                                    VL6180xIdentification &ident)
{
  byte_t  cmd[LaeToFMuxI2CMaxCmdLen];
  byte_t  rsp[LaeToFMuxI2CMaxRspLen];
  size_t  lenCmd = 0;
  size_t  lenRsp = LaeToFMuxI2CRspLenGetIdent;
  byte_t  sensorIndex;
  u16_t   val_hi, val_lo;
  int     n;
  int     rc;

  SensorInfoMap::iterator pos;

  if( (pos = m_mapInfo.find(strKey)) == m_mapInfo.end() )
  {
    LOGERROR("VL6180 sensor %s not found.", strKey.c_str());
    return -LAE_ECODE_BAD_VAL;
  }

  sensorIndex = pos->second.m_nIndex;

  lock();

  cmd[lenCmd++] = LaeToFMuxI2CCmdIdGetIdent;
  cmd[lenCmd++] = sensorIndex;

  rc = m_i2cBus.write_read(m_addrSubProc, cmd, lenCmd, rsp, lenRsp, TStd);

  if( rc == LAE_OK )
  {
    n = 0;
    ident.idModel           = rsp[n++];
    ident.idModelRevMajor   = rsp[n++];
    ident.idModelRevMinor   = rsp[n++];
    ident.idModuleRevMajor  = rsp[n++];
    ident.idModuleRevMinor  = rsp[n++];

    val_hi = ((u16_t)rsp[n++]) << 8;
    val_lo = (u16_t)rsp[n++];
    ident.idDate = val_hi | val_lo;

    val_hi = ((u16_t)rsp[n++]) << 8;
    val_lo = (u16_t)rsp[n++];
    ident.idTime = val_hi | val_lo;
  }

  unlock();

  return rc;
}

int LaeRangeMuxSubproc::cmdGetRanges()
{
  byte_t  cmd[LaeToFMuxI2CMaxCmdLen];
  byte_t  rsp[LaeToFMuxI2CMaxRspLen];
  size_t  lenCmd = 0;
  size_t  lenRsp = LaeToFMuxI2CRspLenGetRanges;
  int     sensorIndex;
  byte_t  val;
  double  fRange;
  int     rc;

  lock();

  cmd[lenCmd++] = LaeToFMuxI2CCmdIdGetRanges;

  rc = m_i2cBus.write_read(m_addrSubProc, cmd, lenCmd, rsp, lenRsp, TStd);

  if( rc == LAE_OK )
  {
    for(sensorIndex = 0; sensorIndex < ToFSensorMaxNumOf; ++sensorIndex)
    {
      val = rsp[sensorIndex];
      if( val <= LaeToFMuxRangeMax )
      {
        fRange = (double)val * 0.001;   // mm to meters
      }
      else if( val == LaeToFMuxRangeNoObj )
      {
        fRange = VL6180X_RANGE_NO_OBJ;
      } 
      else if( val == LaeToFMuxRangeNoDev )
      {
        fRange = VL6180X_RANGE_MIN;
      }
      else
      {
        fRange = VL6180X_ERR_MEAS;
      }

      m_vecRanges[sensorIndex] = fRange;
    }
    RtDb.m_range[sensorIndex].m_fRange = fRange;
  }

  unlock();

  return rc;
}

int LaeRangeMuxSubproc::cmdGetRanges(std::vector<double> &vecRanges)
{
  int   rc;

  if( (rc = cmdGetRanges()) == LAE_OK )
  {
    vecRanges = m_vecRanges;
  }

  return rc;
}
    
int LaeRangeMuxSubproc::cmdGetAmbientLight()
{
  byte_t  cmd[LaeToFMuxI2CMaxCmdLen];
  byte_t  rsp[LaeToFMuxI2CMaxRspLen];
  size_t  lenCmd = 0;
  size_t  lenRsp = LaeToFMuxI2CRspLenGetLux;
  int     sensorIndex;
  int     n;
  u32_t   val;
  double  fLux;
  int     rc;

  lock();

  cmd[lenCmd++] = LaeToFMuxI2CCmdIdGetLux;

  rc = m_i2cBus.write_read(m_addrSubProc, cmd, lenCmd, rsp, lenRsp, TStd);

  if( rc == LAE_OK )
  {
    n = 0;

    for(sensorIndex = 0; sensorIndex < ToFSensorMaxNumOf; ++sensorIndex)
    {
      val = 0;
      for(int i = 0; i < 4; ++i)
      {
        val << 8;
        val |= rsp[n++];
      }

      fLux = (double)val * (double)LaeToFMuxI2CArgLuxScale;

      RtDb.m_range[sensorIndex].m_fAmbientLight = fLux;
    }
  }

  unlock();

  return rc;
}

int LaeRangeMuxSubproc::cmdGetAmbientLight(std::vector<double> &vecLux)
{
  int   rc;

  if( (rc = cmdGetAmbientLight()) == LAE_OK )
  {
    vecLux = m_vecLux;
  }

  return rc;
}
  
int LaeRangeMuxSubproc::cmdGetTunes(const std::string &strKey,
                                    uint_t            &uRangeOffset,
                                    uint_t            &uRangeCrossTalk,
                                    double            &fAlsGain,
                                    uint_t            &uAlsIntPeriod)
{
  byte_t  cmd[LaeToFMuxI2CMaxCmdLen];
  byte_t  rsp[LaeToFMuxI2CMaxRspLen];
  size_t  lenCmd = 0;
  size_t  lenRsp = LaeToFMuxI2CRspLenGetTunes;
  byte_t  sensorIndex;
  byte_t  val;
  uint_t  val_hi, val_lo;
  int     n;
  int     rc;

  SensorInfoMap::iterator pos;

  if( (pos = m_mapInfo.find(strKey)) == m_mapInfo.end() )
  {
    LOGERROR("VL6180 sensor %s not found.", strKey.c_str());
    return -LAE_ECODE_BAD_VAL;
  }

  sensorIndex = pos->second.m_nIndex;

  lock();

  cmd[lenCmd++] = LaeToFMuxI2CCmdIdGetTunes;
  cmd[lenCmd++] = sensorIndex;

  rc = m_i2cBus.write_read(m_addrSubProc, cmd, lenCmd, rsp, lenRsp, TStd);

  if( rc == LAE_OK )
  {
    n = 0;

    uRangeOffset    = rsp[n++];

    if( m_uFwVer >= 2 )
    {
      val_hi          = ((uint_t)rsp[n++]) << 8;
      val_lo          = (uint_t)rsp[n++];
      uRangeCrossTalk = val_hi | val_lo;
    }
    else
    {
      uRangeCrossTalk = rsp[n++];
    }

    val             = rsp[n++];
    fAlsGain        = LaeVL6180Mux::gainEnumToAnalog((vl6180x_als_gain)val);

    val_hi          = ((uint_t)rsp[n++]) << 8;
    val_lo          = (uint_t)rsp[n++];
    uAlsIntPeriod   = val_hi | val_lo;
  }

  unlock();

  return rc;
}

int LaeRangeMuxSubproc::cmdTuneToFSensor(const std::string &strKey,
                                         uint_t             uRangeOffset,
                                         uint_t             uRangeCrossTalk)
{
  byte_t  cmd[LaeToFMuxI2CMaxCmdLen];
  size_t  lenCmd = 0;
  byte_t  sensorIndex;
  byte_t  regOffset;
  u16_t   regCrossTalk;
  byte_t  val_hi, val_lo;
  int     n;
  int     rc;

  SensorInfoMap::iterator pos;

  if( (pos = m_mapInfo.find(strKey)) == m_mapInfo.end() )
  {
    LOGERROR("VL6180 sensor %s not found.", strKey.c_str());
    return -LAE_ECODE_BAD_VAL;
  }

  sensorIndex = pos->second.m_nIndex;

  lock();

  regOffset = (byte_t)(uRangeOffset, (uint_t)VL6180X_RANGE_OFFSET_MIN,
                                     (uint_t)VL6180X_RANGE_OFFSET_MAX);

  regCrossTalk = (u16_t)cap(uRangeCrossTalk, (uint_t)VL6180X_RANGE_XTALK_MIN,
                                             (uint_t)VL6180X_RANGE_XTALK_MAX);

  cmd[lenCmd++] = LaeToFMuxI2CCmdIdTuneToFSensor;
  cmd[lenCmd++] = sensorIndex;
  cmd[lenCmd++] = regOffset;

  if( m_uFwVer >= 2 )
  {
    val_hi = (byte_t)((regCrossTalk >> 8) & 0xff);
    val_lo = (byte_t)(regCrossTalk & 0xff);
    cmd[lenCmd++] = val_hi;
    cmd[lenCmd++] = val_lo;
  }
  else
  {
    cmd[lenCmd++] = regCrossTalk;
  }

  if( (n = m_i2cBus.write(m_addrSubProc, cmd, lenCmd)) == (int)lenCmd )
  {
    rc = LAE_OK;
  }
  else
  {
    rc = -LAE_ECODE_IO;
  }

  unlock();

  return rc;
}

int LaeRangeMuxSubproc::cmdTuneAls(const std::string &strKey,
                                   double             fAlsGain,
                                   uint_t             uAlsIntPeriod)
{
  byte_t  cmd[LaeToFMuxI2CMaxCmdLen];
  size_t  lenCmd = 0;
  byte_t  sensorIndex;
  byte_t  regAlsGain;
  byte_t  val_hi, val_lo;
  int     n;
  int     rc;

  SensorInfoMap::iterator pos;

  if( (pos = m_mapInfo.find(strKey)) == m_mapInfo.end() )
  {
    LOGERROR("VL6180 sensor %s not found.", strKey.c_str());
    return -LAE_ECODE_BAD_VAL;
  }

  sensorIndex = pos->second.m_nIndex;

  lock();

  regAlsGain = (byte_t)LaeVL6180Mux::gainAnalogToEnum(fAlsGain);

  uAlsIntPeriod = cap(uAlsIntPeriod, (uint_t)VL6180X_AMBIENT_INT_T_MIN,
                                     (uint_t)VL6180X_AMBIENT_INT_T_MAX);

  val_hi = (byte_t)((uAlsIntPeriod >> 8) & 0xff);
  val_lo = (byte_t)(uAlsIntPeriod & 0xff);

  cmd[lenCmd++] = LaeToFMuxI2CCmdIdTuneAls;
  cmd[lenCmd++] = sensorIndex;
  cmd[lenCmd++] = regAlsGain;
  cmd[lenCmd++] = val_hi;
  cmd[lenCmd++] = val_lo;

  if( (n = m_i2cBus.write(m_addrSubProc, cmd, lenCmd)) == (int)lenCmd )
  {
    rc = LAE_OK;
  }
  else
  {
    rc = -LAE_ECODE_IO;
  }

  unlock();

  return rc;
}

int LaeRangeMuxSubproc::getRange(const std::string &strKey, double &fRange)
{
  SensorInfoMap::iterator pos;

  if( (pos = m_mapInfo.find(strKey)) == m_mapInfo.end() )
  {
    LOGERROR("VL6180 sensor %s not found.", strKey.c_str());
    return -LAE_ECODE_BAD_VAL;
  }

  int sensorIndex = pos->second.m_nIndex;

  fRange = m_vecRanges[sensorIndex];

  return LAE_OK;
}
      
int LaeRangeMuxSubproc::getRange(std::vector<std::string> &vecNames,
                                 std::vector<double>      &vecRanges)
{
  SensorInfoMap::iterator iter;

  vecNames.clear();
  vecRanges.clear();

  for(iter = m_mapInfo.begin(); iter != m_mapInfo.end(); ++iter)
  {
    vecNames.push_back(iter->first);
    vecRanges.push_back(m_vecRanges[iter->second.m_nIndex]);
  }

  return LAE_OK;
}
  
int LaeRangeMuxSubproc::getAmbientLight(const std::string &strKey, double &fLux)
{
  SensorInfoMap::iterator pos;

  if( (pos = m_mapInfo.find(strKey)) == m_mapInfo.end() )
  {
    LOGERROR("VL6180 sensor %s not found.", strKey.c_str());
    return -LAE_ECODE_BAD_VAL;
  }

  int sensorIndex = pos->second.m_nIndex;

  fLux = m_vecLux[sensorIndex];

  return LAE_OK;
}
  
int LaeRangeMuxSubproc::getAmbientLight(std::vector<std::string> &vecNames,
                                        std::vector<double>      &vecLux)
{
  SensorInfoMap::iterator iter;

  vecNames.clear();
  vecLux.clear();

  for(iter = m_mapInfo.begin(); iter != m_mapInfo.end(); ++iter)
  {
    vecNames.push_back(iter->first);
    vecLux.push_back(m_vecLux[iter->second.m_nIndex]);
  }

}

int LaeRangeMuxSubproc::getSensorProps(const std::string &strKey,
                                       std::string       &strRadiationType,
                                       double            &fFoV,
                                       double            &fBeamDir,
                                       double            &fMin,
                                       double            &fMax)
{
  SensorInfoMap::iterator pos;

  if( (pos = m_mapInfo.find(strKey)) == m_mapInfo.end() )
  {
    LOGERROR("VL6180 range sensor %s not found.", strKey.c_str());
    return -LAE_ECODE_BAD_VAL;
  }

  int         nIndex;
  int         nChan;
  double      fDeadzone;
  std::string strDesc;

  pos->second.getProps(nIndex, nChan, strRadiationType, fFoV, fBeamDir,
                        fDeadzone, fMin, fMax, strDesc);

  return LAE_OK;
}


//------------------------------------------------------------------------------
// LaeRangeSensorGroup Class
//------------------------------------------------------------------------------

LaeRangeSensorGroup::LaeRangeSensorGroup(laelaps::LaeI2C &i2cBus) :
    m_i2cBus(i2cBus)
{
  m_interface            = new LaeRangeInterface(i2cBus); // no-op version
  m_bBlackListed         = false;
  RtDb.m_enable.m_bRange = true;
}

LaeRangeSensorGroup::~LaeRangeSensorGroup()
{
  if( m_interface != NULL )
  {
    delete m_interface;
  }
}

void LaeRangeSensorGroup::blacklist()
{
  m_bBlackListed         = true;
  RtDb.m_enable.m_bRange = false;
}

void LaeRangeSensorGroup::whitelist()
{
  m_bBlackListed         = false;
  RtDb.m_enable.m_bRange = true;
}

int LaeRangeSensorGroup::setInterface(uint_t uProdHwVer)
{
  if( m_interface != NULL )
  {
    delete m_interface;
    m_interface = NULL;
  }

  if( uProdHwVer >= LAE_VERSION(2, 1, 0) )
  {
    m_interface = new LaeRangeMuxSubproc(m_i2cBus);
  }
  else
  {
    m_interface = new LaeVL6180MuxArray(m_i2cBus);
  }

  return LAE_OK;
}

void LaeRangeSensorGroup::clearSensedData()
{
  m_interface->clearSensedData();
}

int LaeRangeSensorGroup::getInterfaceVersion(uint_t &uVerMajor,
                                             uint_t &uVerMinor,
                                             uint_t &uFwVer)
{
  // ignore blacklist
  return m_interface->getInterfaceVersion(uVerMajor, uVerMinor, uFwVer);
}

int LaeRangeSensorGroup::configure(const LaeDesc &desc)
{
  if( !isBlackListed() )
  {
    return m_interface->configure(desc);
  }
  else
  {
    return LAE_OK;
  }
}

int LaeRangeSensorGroup::configure(const LaeTunes &tunes)
{
  if( !isBlackListed() )
  {
    return m_interface->configure(tunes);
  }
  else
  {
    return LAE_OK;
  }
}

int LaeRangeSensorGroup::reload(const LaeTunes &tunes)
{
  if( !isBlackListed() )
  {
    return m_interface->reload(tunes);
  }
  else
  {
    return LAE_OK;
  }
}

int LaeRangeSensorGroup::getSensorProps(const string &strKey,
                                        string       &strRadiationType,
                                        double       &fFoV,
                                        double       &fBeamDir,
                                        double       &fMin,
                                        double       &fMax)
{
  // read-only data - nothing to blacklist
  return m_interface->getSensorProps(strKey, 
                                     strRadiationType,
                                     fFoV, fBeamDir,
                                     fMin, fMax);
}

int LaeRangeSensorGroup::getRange(const string &strKey, double &fRange)
{
  // shadowed data - nothing to blacklist
  return m_interface->getRange(strKey, fRange);
}

int LaeRangeSensorGroup::getRange(vector<string> &vecNames,
                                  vector<double> &vecRanges)
{
  // shadowed data - nothing to blacklist
  return m_interface->getRange(vecNames, vecRanges);
}

int LaeRangeSensorGroup::getAmbientLight(const string &strKey, double &fAmbient)
{
  // shadowed data - nothing to blacklist
  return m_interface->getAmbientLight(strKey, fAmbient);
}

int LaeRangeSensorGroup::getAmbientLight(vector<string> &vecNames,
                                         vector<double> &vecAmbient)
{
  // shadowed data - nothing to blacklist
  return m_interface->getAmbientLight(vecNames, vecAmbient);
}

void LaeRangeSensorGroup::exec()
{
  if( !isBlackListed() )
  {
    m_interface->exec();
  }
}
