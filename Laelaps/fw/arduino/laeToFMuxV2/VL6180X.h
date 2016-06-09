////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelasp
//
// Firmware:  laeToFMux
//
// File:      Vl6180.h
//
/*! \file
 *
 * $LastChangedDate: 2016-03-08 10:43:48 -0700 (Tue, 08 Mar 2016) $
 * $Rev: 4346 $
 *
 * \brief The VL6180 Time-of-Flight sensor class interface.
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
 * SparkFun_VL6180X.h
 * Library for VL6180x time of flight range finder.
 * Casey Kuhns @ SparkFun Electronics
 * 10/29/2014
 * https://github.com/sparkfun/SparkFun_ToF_Range_Finder-VL6180_Arduino_Library
 * 
 * The VL6180x by ST micro is a time of flight range finder that
 * uses pulsed IR light to determine distances from object at close
 * range.  The average range of a sensor is between 0-200mm
 * 
 * In this file are the function prototypes in the VL6180x class
 * 
 * Resources:
 * This library uses the Arduino Wire.h to complete I2C transactions.
 * 
 * Development environment specifics:
 * 	IDE: Arduino 1.0.5
 * 	Hardware Platform: Arduino Pro 3.3V/8MHz
 * 	VL6180x Breakout Version: 1.0
 
 **Updated for Arduino 1.6.4 5/2015**
 * 
 * Some settings and initial values come from code written by Kris Winer
 * VL6180X_t3 Basic Example Code
 * by: Kris Winer
 * date: September 1, 2014
 * license: Beerware - Use this code however you'd like. If you 
 * find it useful you can buy me a beer some time.
 * 
 * This code is beerware. If you see me (or any other SparkFun employee) at the
 * local pub, and you've found our code helpful, please buy us a round!
 * 
 * Distributed as-is; no warranty is given.
 ******************************************************************************/

#ifndef _VL6180X_h
#define _VL6180X_H

#include <Arduino.h>
#include <SoftwareWire.h>

//
// Sensor I2C 7-bit address.
//
#define VL6180X_I2C_ADDR_DFT      0x29      ///< \h_i2c default 7-bit address

#define VL6180X_FAILURE_RESET  -1

//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Registers
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
#define VL6180X_IDENTIFICATION_MODEL_ID              0x0000
#define VL6180X_IDENTIFICATION_MODEL_REV_MAJOR       0x0001
#define VL6180X_IDENTIFICATION_MODEL_REV_MINOR       0x0002
#define VL6180X_IDENTIFICATION_MODULE_REV_MAJOR      0x0003
#define VL6180X_IDENTIFICATION_MODULE_REV_MINOR      0x0004
#define VL6180X_IDENTIFICATION_DATE                  0x0006 //16bit value
#define VL6180X_IDENTIFICATION_TIME                  0x0008 //16bit value

#define VL6180X_SYSTEM_MODE_GPIO0                    0x0010
#define VL6180X_SYSTEM_MODE_GPIO1                    0x0011
#define VL6180X_SYSTEM_HISTORY_CTRL                  0x0012
#define VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO         0x0014
#define VL6180X_SYSTEM_INTERRUPT_CLEAR               0x0015
#define VL6180X_SYSTEM_FRESH_OUT_OF_RESET            0x0016
#define VL6180X_SYSTEM_GROUPED_PARAMETER_HOLD        0x0017

#define VL6180X_SYSRANGE_START                       0x0018
#define VL6180X_SYSRANGE_THRESH_HIGH                 0x0019
#define VL6180X_SYSRANGE_THRESH_LOW                  0x001A
#define VL6180X_SYSRANGE_INTERMEASUREMENT_PERIOD     0x001B
#define VL6180X_SYSRANGE_MAX_CONVERGENCE_TIME        0x001C
#define VL6180X_SYSRANGE_CROSSTALK_COMPENSATION_RATE 0x001E
#define VL6180X_SYSRANGE_CROSSTALK_VALID_HEIGHT      0x0021
#define VL6180X_SYSRANGE_EARLY_CONVERGENCE_ESTIMATE  0x0022
#define VL6180X_SYSRANGE_PART_TO_PART_RANGE_OFFSET   0x0024
#define VL6180X_SYSRANGE_RANGE_IGNORE_VALID_HEIGHT   0x0025
#define VL6180X_SYSRANGE_RANGE_IGNORE_THRESHOLD      0x0026
#define VL6180X_SYSRANGE_MAX_AMBIENT_LEVEL_MULT      0x002C
#define VL6180X_SYSRANGE_RANGE_CHECK_ENABLES         0x002D
#define VL6180X_SYSRANGE_VHV_RECALIBRATE             0x002E
#define VL6180X_SYSRANGE_VHV_REPEAT_RATE             0x0031

#define VL6180X_SYSALS_START                         0x0038
#define VL6180X_SYSALS_THRESH_HIGH                   0x003A
#define VL6180X_SYSALS_THRESH_LOW                    0x003C
#define VL6180X_SYSALS_INTERMEASUREMENT_PERIOD       0x003E
#define VL6180X_SYSALS_ANALOGUE_GAIN                 0x003F
#define VL6180X_SYSALS_INTEGRATION_PERIOD            0x0040

#define VL6180X_RESULT_RANGE_STATUS                  0x004D
#define VL6180X_RESULT_ALS_STATUS                    0x004E
#define VL6180X_RESULT_INTERRUPT_STATUS_GPIO         0x004F
#define VL6180X_RESULT_ALS_VAL                       0x0050
#define VL6180X_RESULT_HISTORY_BUFFER                0x0052 
#define VL6180X_RESULT_RANGE_VAL                     0x0062
#define VL6180X_RESULT_RANGE_RAW                     0x0064
#define VL6180X_RESULT_RANGE_RETURN_RATE             0x0066
#define VL6180X_RESULT_RANGE_REFERENCE_RATE          0x0068
#define VL6180X_RESULT_RANGE_RETURN_SIGNAL_COUNT     0x006C
#define VL6180X_RESULT_RANGE_REFERENCE_SIGNAL_COUNT  0x0070
#define VL6180X_RESULT_RANGE_RETURN_AMB_COUNT        0x0074
#define VL6180X_RESULT_RANGE_REFERENCE_AMB_COUNT     0x0078
#define VL6180X_RESULT_RANGE_RETURN_CONV_TIME        0x007C
#define VL6180X_RESULT_RANGE_REFERENCE_CONV_TIME     0x0080

#define VL6180X_READOUT_AVERAGING_SAMPLE_PERIOD      0x010A
#define VL6180X_FIRMWARE_BOOTUP                      0x0119
#define VL6180X_FIRMWARE_RESULT_SCALER               0x0120
#define VL6180X_I2C_SLAVE_DEVICE_ADDRESS             0x0212
#define VL6180X_INTERLEAVED_MODE_ENABLE              0x02A3

//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Range sensor values
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

#define VL6180X_RANGE_MIN     0     ///< minimum valid measured distance (mm)
#define VL6180X_RANGE_MAX     200   ///< maximum valid measured distance (mm)
#define VL6180X_RANGE_NO_OBJ  255   ///< no object detected special value

// part-to-part offset calibration (part to part default varies)
#define VL6180X_RANGE_OFFSET_MIN  0     ///< minimum tof offset
#define VL6180X_RANGE_OFFSET_MAX  0xff  ///< maximum tof offset

// cross-talk compensation calibration
#define VL6180X_RANGE_XTALK_MIN   0       ///< minimum tof cross-talk
#define VL6180X_RANGE_XTALK_MAX   0xffff  ///< maximum tof cross-talk
#define VL6180X_RANGE_XTALK_DFT   0       ///< factory default tof cross-talk

//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Ambient light sensor values
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

#define VL6180X_LUX_NO_LIGHT  0 ///< no light detected

/*!
 * \brief ALS gain enumeration.
 *
 * Data sheet shows gain values as binary list.
 */
enum vl6180x_als_gain
{
  GAIN_20 = 0,  ///< Actual ALS Gain of 20
  GAIN_10,      ///< Actual ALS Gain of 10.32
  GAIN_5,       ///< Actual ALS Gain of 5.21
  GAIN_2_5,     ///< Actual ALS Gain of 2.60
  GAIN_1_67,    ///< Actual ALS Gain of 1.72
  GAIN_1_25,    ///< Actual ALS Gain of 1.28
  GAIN_1 ,      ///< Actual ALS Gain of 1.01
  GAIN_40       ///< Actual ALS Gain of 40
};

// analog gain tuning
#define VL6180X_AMBIENT_GAIN_MIN  GAIN_20 ///< minimum als analog gain value
#define VL6180X_AMBIENT_GAIN_MAX  GAIN_40 ///< maximum als analog gain
#define VL6180X_AMBIENT_GAIN_DFT  GAIN_1  ///< factory default als analog gain

// integration period tuning
#define VL6180X_AMBIENT_INT_T_MIN   1     ///< minimum als int period (msec)
#define VL6180X_AMBIENT_INT_T_MAX   512   ///< maximum als int period (msec)
#define VL6180X_AMBIENT_INT_T_DFT   1     ///< factory default int period (1 ms)
#define VL6180X_AMBIENT_INT_T_REC   100   ///< recommended int period (msec)
#define VL6180X_AMBIENT_INT_T_MASK  0x1ff ///< als integration period mask

//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Other sensor values
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Sensor identification structure.
 */
struct VL6180xIdentification
{
  uint8_t idModel;
  uint8_t idModelRevMajor;
  uint8_t idModelRevMinor;
  uint8_t idModuleRevMajor;
  uint8_t idModuleRevMinor;
  uint16_t idDate;
  uint16_t idTime;
};


//------------------------------------------------------------------------------
// Class VL6180x
//------------------------------------------------------------------------------

/*!
 *
 */
class VL6180x
{
public:	
  /*!
   * \brief Asynchronous measurement execution states
   */
  enum AsyncState
  {
    AsyncStateInit,           ///< initialize measurement state
    AsyncStateWaitForReady,   ///< wait for sensor to be ready
    AsyncStateStartMeas,      ///< start measurement
    AsyncStateWaitForResult,  ///< wait for sensor measurement result
    AsyncStateAbort,          ///< abort measurement
    AsyncStateDone            ///< measurement made
  };

  /*!
   * \brief Default initialization constructor.
   *
   * \param sensorId  Sensor software id.
   * \param wire      Associated software I2C.
   * \param address   Sensor I2C 7-bit address.
   */
  VL6180x(int           sensorId,
          SoftwareWire &wire,
          uint8_t       address = VL6180X_I2C_ADDR_DFT);

  /*!
   * \brief Busy wait for sensor firmware to fully booted up.
   *
   * \brief tries   Number of tries.
   *
   * \return Returns true if sensor firmware booted, false otherwise.
   */
  boolean waitForBootup(int tries=1);

  /*!
   * \brief Ping sensor for existence on bus.
   *
   * \brief tries   Number of tries.
   *
   * \return Returns true if sensor responding, false otherwise.
   */
  boolean ping(int tries=1);

  /*!
   * \brief Initialize size as per ST datasheet.
   *
   * http://www.st.com/st-web-ui/static/active/en/resource/technical/document/application_note/DM00122600.pdf (Section 1.3).
   */
  void initSensor();

  /*!
   * \brief Set default value as per ST data sheet.
   *
   * http://www.st.com/st-web-ui/static/active/en/resource/technical/document/application_note/DM00122600.pdf (Section 9).
   */
  void writeSensorDefaults();

  /*!
   * \brief Read sensor identity and save.
   */
  void readIdent();

  /*!
   * \brief Read sensor exported tuning parameters.
   */
  void readTunes();

#ifdef INCLUDE_EXTRAS
  /*!
   * \brief Change address of sensor.
   *
   * Note: IT APPEARS THAT CHANGING THE ADDRESS IS NOT STORED IN NON-VOLATILE
   *       MEMORY. POWER CYCLING THE DEVICE REVERTS ADDRESS BACK TO 0X29.
   *
   * \return Return (new) address.
   */
  byte changeAddress(byte new_address);

  /*!
   * \brief Synchronously measure object's range.
   *
   * \return Returns distance (mm).
   */
  byte measureRange();

  /*!
   * \brief Synchronously measure ambient light.
   */
  float measureAmbientLight();
#endif // INCLUDE_EXTRAS

  /*!
   * \brief Asynchronously measure object's range.
   *
   * Example code snippet:
   * \verbatim 
   * byte dist;
   * while(true)
   * {
   *  if( asyncMeasureRange() )
   *  {
   *    dist = getRange();
   *    // .. do something.
   *  }
   * }
   * \endverbatim 
   *
   * \return Returns true when measurement is complete, false otherwise.
   */
  boolean asyncMeasureRange();

  /*!
   * \brief Asynchronously measure ambient light.
   *
   * Example code snippet:
   * \verbatim 
   * byte dist;
   * while(true)
   * {
   *  if( asyncMeasureAmbientLight() )
   *  {
   *    dist = getRange();
   *    // .. do something.
   *  }
   * }
   * \endverbatim 
   *
   * \return Returns true when measurement is complete, false otherwise.
   */
  boolean asyncMeasureAmbientLight();

  /*!
   * \brief Mark time-of-flight sensor for tuning.
   *
   * \param offset    ToF sensor part-to-part offset.
   * \param crosstalk ToF sensor cross-talk compensation.
   */
  void markRangeForTuning(byte offset, uint16_t crosstalk);
  
  /*!
   * \brief Tune time-of-flight range sensor.
   *
   * \param offset    ToF sensor part-to-part offset.
   * \param crosstalk ToF sensor cross-talk compensation.
   *
   * \return Returns true on success, false on failure.
   */
  boolean tuneRangeSensor(byte offset, uint16_t crosstalk);

  /*!
   * \brief Mark ambient light sensor for tuning.
   *
   * \param gain        Ambient light sensor analog gain enum.
   * \param intPeriod   Ambient light sensor integration period (msec).
   */
  void markAlsForTuning(byte gain, uint16_t intPeriod);

  /*!
   * \brief Tune ambient light sensor.
   *
   * \param gain        Ambient light sensor analog gain enum.
   * \param intPeriod   Ambient light sensor integration period (msec).
   *
   * \return Returns true on success, false on failure.
   */
  boolean tuneAls(byte gain, uint16_t intPeriod);


  //............................................................................
  // Attriube Member Functions
  //............................................................................

  /*!
   * \brief Get sensor's assigned id.
   *
   * \return Sensor id.
   */
  int getSensorId();

  /*!
   * \brief Whitelist this sensor.
   */
  void whitelist();

  /*!
   * \brief Blacklist this sensor.
   */
  void blacklist();

  /*!
   * \brief Test if this sensor is blacklisted.
   *
   * \return Returns true or false.
   */
  boolean isBlacklisted();

  /*!
   * \brief Test if this sensor is busy.
   *
   * \return Returns true or false.
   */
  boolean isBusy();

  /*!
   * \brief Get the identity of this sensor.
   *
   * \param [out] pIdent  Pointer to identity structure.
   */
  void getIdent(VL6180xIdentification *pIdent);

  /*!
   * \brief Get the current tuning parameters values.
   *
   * \param [out] offset      ToF sensor part-to-part offset.
   * \param [out] crosstalk   ToF sensor cross-talk compensation.
   * \param [out] gain        Ambient light sensor analog gain enum.
   * \param [out] intPeriod   Ambient light sensor integration period (msec).
   */
  void getTunes(byte &offset, uint16_t &crosstalk,
                byte &gain,   uint16_t &intPeriod);

  /*!
   * \brief Get the exported tuning factory defaults.
   *
   * \param [out] offset      ToF sensor part-to-part offset.
   * \param [out] crosstalk   ToF sensor cross-talk compensation.
   * \param [out] gain        Ambient light sensor analog gain enum.
   * \param [out] intPeriod   Ambient light sensor integration period (msec).
   */
  void getDefaultTunes(byte &offset, uint16_t &crosstalk,
                       byte &gain,   uint16_t &intPeriod);

  /*!
   * \brief Get measured range.
   *
   * \return Object distance (mm).
   */
  byte getRange();

  /*!
   * \brief Get measured ambient light.
   *
   * \return Illuminance (lux).
   */
  float getAmbientLight();

  /*!
   * \brief Read 8-bit value from sensor register.
   *
   * \param regAddr   Sensor's 16-bit register address.
   *
   * \return Read value.
   */
  byte readReg8(uint16_t regAddr);

  /*!
   * \brief Read 16-bit value from sensor register.
   *
   * \param regAddr   Sensor's 16-bit register address.
   *
   * \return Read value.
   */
  uint16_t readReg16(uint16_t regAddr);

  /*!
   * \brief Write 8-bit value to sensor register.
   *
   * \param regAddr   Sensor's 16-bit register address.
   * \param data      8-bit value.
   *
   * \return Read value.
   */
  void writeReg8(uint16_t regAddr, uint8_t data);

  /*!
   * \brief Write 16-bit value to sensor register.
   *
   * \param regAddr   Sensor's 16-bit register address.
   * \param data      16-bit value.
   *
   * \return Read value.
   */
  void writeReg16(uint16_t regAddr, uint16_t data);

protected:
  int           m_nSensorId;          ///< assigned sensor id
  SoftwareWire &m_wire;               ///< bound i2c bus interface
  int           m_addr;               ///< address of sensor
  boolean       m_bBlackListed;       ///< is [not] blacklisted
  boolean       m_bBusy;              ///< is [not] busy 

  // read values
  VL6180xIdentification m_ident;      ///< sensor identity
  byte                  m_range;      ///< last read range value
  float                 m_lux;        ///< last read ambient light value

  // asynchronous measurment state
  AsyncState    m_eAsyncState;        ///< current state
  unsigned long m_uAsyncTStart;       ///< measurement start clock time (msec)
  unsigned long m_uAsyncTWait;        ///< wait timeout (msec)
  
  // pending tuning operations
  byte      m_newRangeOffset;         ///< new pending range part-to-part offset
  byte      m_newRangeCrossTalk;      ///< new pending range cross-talk
  boolean   m_bRangeNeedsTuning;      ///< range sensor does [not] need tuning
  byte      m_newAlsGain;             ///< new pending ALS gain
  uint16_t  m_newAlsIntPeriod;        ///< new pending ALS itegration period
  boolean   m_bAlsNeedsTuning;        ///< ALS does [not] need tuning

  // shadow register values
  byte      m_regRangeOffsetDft;      ///< factory calibrated offset register
  byte      m_regRangeOffset;         ///< range part-to-part offset value
  uint16_t  m_regRangeCrossTalk;      ///< range cross-talk value
  byte      m_regAlsGain;             ///< ALS gain value
  uint16_t  m_regAlsIntPeriod;        ///< ALS itegration period value

  /*!
   * \brief Convert raw range value to disance.
   *
   * \param rangeRaw    Range's raw value.
   * \param rangeStatus Range measurement status.
   *
   * \return Converted distance (mm).
   */
  byte cvtRangeRawToDist(byte rangeRaw, byte rangeStatus);

  /*!
   * \brief Convert raw ALS value to lux.
   *
   * \param alsRaw    ALS's raw ambient light value.
   * \param alsStatus ALS measurement status.
   *
   * \return Converted lux.
   */
  float cvtAlsRawToLux(uint16_t alsRaw, byte alsStatus);

};  // class VL6180x

#endif // _VL6180X_H

