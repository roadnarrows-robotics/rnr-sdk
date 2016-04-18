////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// File:      laeVL6180.h
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
 *
 * \par Laelaps v2.0
 * The v2.0 hardware uses an i2c mutliplexer chip to toggle between the
 * connected sensors. When a sensor was enabled, it is electrically connected
 * to the odroid's /dev/i2c-3 bus. When disabled, the sensor was electrically
 * isolated.
 *
 * \verbatim
 *         i2c-3   0x70           0x29      0x29           0x29
 * odroid ------- i2cmux         VL6180_0, VL6180_2, ..., VL6180_7
 *                   |              ^         ^              ^
 *                   |    i2c-3     |         |              |
 *                   -----------------------------------------
 *                                   select
 *
 * \endverbatim
 *
 * \par Laelaps v2.1+
 * The v2.1+ hardware uses an Arduino compatible sub-processors. The
 * sub-processor is a slave i2c device to the odroid, and a i2c master to 8
 * independent soft i2c buses, one per each sensor.
 *
 * \verbatim
 *         i2c-3   0x71    soft-i2c-0   0x29
 * odroid ------- subproc ------------ VL6180_0
 *                   |
 *                   |     soft-i2c-1   0x29
 *                   |---------------- VL6180_1
 *                   |
 *                       ...
 *                   |
 *                   |     soft-i2c-7   0x29
 *                   |---------------- VL6180_7
 *
 * \endverbatim
 *
 * This code is based on the Arduino library freely available from Sparkfun.
 * See original comment block below.
 *
 * \sa https://github.com/sparkfun/ToF_Range_Finder_Sensor-VL6180/tree/master/Libraries/Arduino
 * \sa https://www.sparkfun.com/products/12785
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * (C) 2015-2016  RoadNarrows
 * (http://www.roadNarrows.com)
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

#ifndef _LAE_VL6180_H
#define _LAE_VL6180_H

#include <inttypes.h>
#include <pthread.h>

#include <string>
#include <vector>
#include <map>

#include "rnr/rnrconfig.h"

#include  "Laelaps/laelaps.h"
#include  "Laelaps/laeUtils.h"
#include  "Laelaps/laeDesc.h"
#include  "Laelaps/laeTune.h"
#include  "Laelaps/laeI2CMux.h"   // v2.0
#include  "Laelaps/laeToFMux.h"   // v2.1+

//
// Sensor I2C 7-bit address.
//
#define VL6180X_ADDR            0x29      ///< \h_i2c 7-bit address

//
// Proximity Sensor Specs
//
#define VL6180X_IR_LAMBDA       8.5e-07   ///< 850nm wavelength (m)
#define VL6180X_RANGE_MIN       0.0       ///< minimum range (m)
#define VL6180X_RANGE_MAX       0.2       ///< maximum range (m)
#define VL6180X_RANGE_FOV       40.0      ///< field of view (degrees)
#define VL6180X_RANGE_NO_OBJ    1000000.0 ///< no object detected

// part-to-part offset calibration (2's compliment)
#define VL6180X_RANGE_OFFSET_MIN  -128    ///< minimum tof offset
#define VL6180X_RANGE_OFFSET_MAX   127    ///< maximum tof offset

// cross-talk compensation calibration
#define VL6180X_RANGE_XTALK_MIN   0       ///< minimum tof cross-talk
#define VL6180X_RANGE_XTALK_MAX   255     ///< maximum tof cross-talk

//
// Ambient Light Sensor Specs
//
#define VL6180X_AMBIENT_RES       0xffff    ///< 16-bit ambient light resolution
#define VL6180X_AMBIENT_MIN       0.0       ///< minimum ambient light (lux)
#define VL6180X_AMBIENT_MAX       1000000.0 ///< maximum ambient light (lux)

// analog gain tuning
#define VL6180X_AMBIENT_GAIN_MIN    1.0   ///< minimum als analog gain
#define VL6180X_AMBIENT_GAIN_MAX    40.0  ///< maximum als analog gain
#define VL6180X_AMBIENT_GAIN_MASK   0x07  ///< als analog gain mask

// integration period tuning
#define VL6180X_AMBIENT_INT_T_MIN   1     ///< minimum als int. period (msec)
#define VL6180X_AMBIENT_INT_T_MAX   512   ///< maximum als int. period (msec)
#define VL6180X_AMBIENT_INT_T_REC   100   ///< recommended int. period (msec)
#define VL6180X_AMBIENT_INT_T_MASK 0x1ff  ///< als integration period mask

//
// Error measurement values.
//
#define VL6180X_ERR_MEAS    -1.0    ///< error meassurement

//
// Miscellanea
//
#define VL6180X_T_AUTO      0xffff  ///< auto-determine wait time
#define VL6180X_FACTORY_DFT -100000 ///< use factory default

//
// Registers
//
#define VL6180x_FAILURE_RESET  -1

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

namespace sensor
{
  namespace vl6180
  {
    /*!
     * \brief Ambient Light Sensor gain value enumeration.
     *
     * From data sheet shows gain values as binary list
     */
    enum vl6180x_als_gain
    {
      GAIN_20 = 0,  ///< actual ALS Gain of 20
      GAIN_10,      ///< actual ALS Gain of 10.32
      GAIN_5,       ///< actual ALS Gain of 5.21
      GAIN_2_5,     ///< actual ALS Gain of 2.60
      GAIN_1_67,    ///< actual ALS Gain of 1.72
      GAIN_1_25,    ///< actual ALS Gain of 1.28
      GAIN_1 ,      ///< actual ALS Gain of 1.01
      GAIN_40,      ///< actual ALS Gain of 40
      GAIN_NumOf    ///< number of gain enum values
    };
    
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


    // -------------------------------------------------------------------------
    // LaeVL6180SensorInfo Class
    // -------------------------------------------------------------------------

    /*!
     * \brief Container class to hold VL6180 time-of-flight sensor static
     * information.
     */
    class LaeVL6180SensorInfo
    {
    public:
      int                 m_nIndex;       ///< sensor index
      int                 m_nChan;        ///< multiplexed channel number
      double              m_fBeamDir;     ///< center of beam direction(radians)
      double              m_fDeadzone;    ///< sensor deadzone (m)
      std::string         m_strDesc;      ///< short description
    
      /*!
       * \brief Default constructor.
       */
      LaeVL6180SensorInfo();

      /*!
       * \brief Initialization constructor.
       *
       * \param nIndex    Sensor index. May be remapped if order of wired sensors
       *                  change.
       * \param nChan     Mulitplexed channel number.
       * \param fBeamDir  Center of IR beam direction (radians).
       * \param fDeadzone Sense deadzone range.
       * \param strDesc   Short description (e.g. location).
       */
      LaeVL6180SensorInfo(int               nIndex,
                          int               nChan,
                          double            fBeamDir,
                          double            fDeadzone,
                          const std::string &strDesc);
    
      /*!
       * \brief Copy constructor.
       *
       * \param src   Source object.
       */
      LaeVL6180SensorInfo(const LaeVL6180SensorInfo &src);
    
      /*!
       * \brief Destructor.
       */
      ~LaeVL6180SensorInfo();
    
      /*!
       * \brief Assignment operator
       *
       * \param rhs   Right hand side object.
       *
       * \reture *this
       */
      LaeVL6180SensorInfo operator=(const LaeVL6180SensorInfo &rhs);
    
      /*!
       * \brief Get sensor properties.
       *
       * \param [out] nIndex            Sensor index.
       * \param [out] nChan             Mulitplexed channel number.
       * \param [out] strRadiationType  Radiation type. 
       * \param [out] fFoV              Field of View (radians).
       * \param [out] fBeamDir          Center of IR beam direction (radians).
       * \param [out] fDeadzone         Sense deadzone range.
       * \param [out] fMin              Minimum range (meters).
       * \param [out] fMax              Maximum range (meters).
       * \param [out] strDesc           Short description (e.g. location).
       */
      void getProps(int               &nIndex,
                    int               &nChan,
                    std::string       &strRadiationType,
                    double            &fFoV,
                    double            &fBeamDir,
                    double            &fDeadzone,
                    double            &fMin,
                    double            &fMax,
                    std::string       &strDesc);
    
    protected:
    }; // class LaeVL6180SensorInfo


    // -------------------------------------------------------------------------
    // LaeVL6180Mux Class
    // -------------------------------------------------------------------------

    /*!
     * \brief VL6180 Time of Flight Class.
     *
     * The interface between the processor and the sensor is through an \h_i2c
     * mulitplexer.
     */
    class LaeVL6180Mux
    {
    public:
      /*!
       * \brief Consecutive sensed error count threshold.
       *
       * The sensor will be black listed if errors exceed this threshold.
       */
      static const int NSenseErrorsThreshold = 10;

      /*!
       * \brief Intialization constructor.
       *
       * \param mux       \h_i2c multiplexer switch.
       * \param nChan     Mulitplexed channel number.
       * \param fBeamDir  Center of IR beam direction (radians).
       * \param fDeadzone Sense deadzone range.
       * \param strNameId Name identifier.
       * \param strDesc   Short description (e.g. location).
       */
      LaeVL6180Mux(laelaps::LaeI2CMux &mux,
                   int                nChan,
                   double             fBeamDir,
                   double             fDeadzone,
                   const std::string  &strNameId="VL6180",
                   const std::string  &strDesc="Time-of-Flight range sensor");

      /*!
       * \brief Copy constructor.
       *
       * \param src   Source object.
       */
      LaeVL6180Mux(const LaeVL6180Mux &src);

      /*!
       * \brief Destructor.
       */
      virtual ~LaeVL6180Mux();

      /*!
       * \brief Initialize sensor with recommended settings and defaults.
       *
       * \note Higly recommended to call this method and sensor power-up and
       * after resets.
       *
       * \param bForce    Do [not] force re-initialization.
       *
       * \copydoc doc_return_std
       */
      int initSensor(bool bForce = false);

      /*!
       * \brief Write defaults to sensor.
       *
       * The defaults provide reasonable operation within the Laelaps
       * envirionment.
       *
       * \copydoc doc_return_std
       */
      int writeDefaults();

      /*!
       * \brief Read shadows register values and update derived data.
       */
      void readShadowRegs();

      /*!
       * \brief Read shadows register values, update derived data, and return
       * registers values.
       *
       * \param [out] regRangeOffset    Range part-to-part offset register value
       * \param [out] regRangeCrossTalk Range cross-talk register value.
       * \param [out] regAlsGain        ALS gain register value.
       * \param [out] regAlsIntPeriod   ALS itegration period register value.
       */
      void readShadowRegs(byte_t &regRangeOffset,
                          byte_t &regRangeCrossTalk,
                          byte_t &regAlsGain,
                          u16_t  &regAlsIntPeriod);

      /*!
       * \brief Tune sensor configuration.
       *
       * \param uRangeOffset    ToF sensor part-to-part offset.
       *                        If VL6180X_FACTORY_DFT then leave as is.
       * \param uRangeCrossTalk ToF sensor cross-talk compensation.
       *                        If VL6180X_FACTORY_DFT then leave as is.
       * \param fAlsGain        Ambient light sensor analog gain.
       * \param uAlsIntPeriod   Ambient light sensor integration period (msec).
       *
       * \copydoc doc_return_std
       */
      int tune(uint_t uRangeOffset, uint_t uRangeCrossTalk,
               double fAlsGain, uint_t uAlsIntPeriod);
      
      /*!
       * \brief Read sensor identification.
       *
       * \param [out] id  Identification structure.
       *
       * \copydoc doc_return_std
       */
      int readId(struct VL6180xIdentification &id);

      /*!
       * \brief Measure the sensed target range.
       *
       * Single shot mode.
       *
       * \param msecWait  Maximum time to wait for the measurement to complete
       *                  (milliseconds).
       *                  If the wait time is the special value VL6180X_T_AUTO,
       *                  then the wait time is auto-calculated base on current
       *                  sensor configuration.
       *
       * \return
       * On success, the measured object range (meters) is returned.\n
       * If no object is detected within sensor range,
       * \ref VL6180X_RANGE_NO_OBJ(1000000.0) is returned.\n
       * On error, \ref VL6180X_ERR_MEAS(-1.0) is returned.
       */
      double measureRange(u32_t msecWait=VL6180X_T_AUTO);

      /*!
       * \brief Measure the sensed ambient light illuminance.
       *
       * Single shot mode.
       *
       * \param msecWait  Maximum time to wait for the measurement to complete
       *                  (milliseconds).
       *                  If the wait time is the special value VL6180X_T_AUTO,
       *                  then the wait time is auto-calculated base on current
       *                  sensor configuration.
       *
       * \return On success, returns measured illuminance (lux).\n
       * On success, the measured illuminance (lux) is returned.\n
       * On error, \ref VL6180X_ERR_MEAS(-1.0) is returned.
       */
      double measureAmbientLight(u32_t msecWait=VL6180X_T_AUTO);
      
      /*!
       * \brief Get last sensed measurements.
       *
       * \parma [out] fRange        Sensed object range (meters).
       * \parma [out] fAmbienLight  Sensed ambient light (lux).
       */
      void getMeasurements(double &fRange, double &fAmbientLight)
      {
        fRange        = m_fRange;
        fAmbientLight = m_fAmbientLight;
      }

      /*!
       * \brief Get sensor assigned name id.
       *
       * \return String name.
       */
      std::string getNameId()
      {
        return m_strNameId;
      }

      /*!
       * \brief Get sensor short description.
       *
       * \return String name.
       */
      std::string getDesc()
      {
        return m_strDesc;
      }

      /*!
       * \brief Get sensor direction of the center of emitter IR beam.
       *
       * \return Radians.
       */
      double getBeamDir()
      {
        return m_fBeamDir;
      }

      /*!
       * \brief Get sensor field of view.
       *
       * \return Radians.
       */
      double getFoV()
      {
        return laelaps::degToRad(VL6180X_RANGE_FOV);
      }

      /*!
       * \brief Get sensor's minimum and maximum range.
       *
       * \return Meters, meters.
       */
      void getMinMax(double &fMin, double &fMax)
      {
        fMin = VL6180X_RANGE_MIN;
        fMax = VL6180X_RANGE_MAX;
      }

      /*!
       * \brief Get radiation type.
       *
       * \return String.
       */
      std::string getRadiationType()
      {
        return "infrared";
      }

      /*!
       * \brief Calibrate part-to-part offset.
       *
       * The offset is stored in register SYSRANGE_PART_TO_PART_RANGE_OFFSET
       * (0x0024).
       *
       * \sa Section 2.12.3 in datasheet for setup and procedures.
       *
       * \param [out] nOffsetPre  Current offset register value.
       * \param [out] fAvgPre     Average measured range on pre-calibrated
       *                          sensor.
       * \param [out] nOffsetPost Post-procedure offset value written to
       *                          offset register. If equal to nOffsetPre, 
       *                          no calibration was required.
       * \param [out] fAvgPost    Average measured range after new offset 
       *                          written.
       *
       * \copydoc doc_return_std
       */
      int calibOffset(int    &nOffsetPre,
                      double &fAvgPre,
                      int    &nOffsetPost,
                      double &fAvgPost);

      /*!
       * \brief Calibrate cross-talk compensation.
       *
       * The offset is stored in register SYSRANGE_CROSSTALK_COMPENSATION_RATE
       * (0x001e).
       *
       * \sa Section 2.12.4 in datasheet for setup and procedures.
       *
       * \param [out] nCrossTalk  Cross-talk compensation written to
       *                          compensation register.
       *
       * \copydoc doc_return_std
       */
      int calibCrossTalk(int &nCrossTalk);

      /*!
       * \brief Read an 8-bit value from a register.
       *
       * \param reg         16-bit register address.
       * \param [out] val   Read value.
       *
       * \copydoc doc_return_std
       */
      int readReg8(u16_t reg, byte_t &val);

      /*!
       * \brief Read a 16-bit value from a register.
       *
       * \param reg         16-bit register address.
       * \param [out] val   Read value.
       *
       * \copydoc doc_return_std
       */
      int readReg16(u16_t reg, u16_t &val);
    
      /*!
       * \brief Write an 8-bit value to a register.
       *
       * \param reg         16-bit register address.
       * \param [in] val    Write value.
       *
       * \copydoc doc_return_std
       */
      int writeReg8(u16_t reg, byte_t val);

      /*!
       * \brief Write a 16-bit value to a register.
       *
       * \param reg         16-bit register address.
       * \param [in] val    Write value.
       *
       * \copydoc doc_return_std
       */
      int writeReg16(u16_t reg, u16_t val);

      /*!
       * \brief Convert ambient light sensor gain register value enum to
       * analog gain.
       *
       * \param eAlsGain    Ambient light sensor gain enumeration.
       *
       * \return Analog gain.
       */
      static double gainEnumToAnalog(vl6180x_als_gain eAlsGain);

      /*!
       * \brief Convert ambient light sensor analog gain to associated register
       * value enum.
       *
       * The mapped enum with the minimum difference between the
       * target analog gain and the actual is chosen.
       *
       * \param fAlsGain    Ambient light sensor analog gain.
       *
       * \return Gain enum.
       */
      static vl6180x_als_gain gainAnalogToEnum(double fAlsGain);

    protected:
      laelaps::LaeI2CMux &m_mux;          ///< \h_i2c multiplexor
      int                 m_nChan;        ///< multiplexed channel number
      double              m_fBeamDir;     ///< center of beam direction(radians)
      double              m_fDeadzone;    ///< sensor deadzone (m)
      std::string         m_strNameId;    ///< name identifier of sensor
      std::string         m_strDesc;      ///< short description
      int                 m_nErrorCnt;    ///< consecutive error count
      bool                m_bBlackListed; ///< sensor is [not] black listed

      // tuning and calibration parameters
      double  m_fAlsGain;               ///< ambient light sensor analog gain 
      uint_t  m_uAlsIntPeriod;          ///< ALS integration period (msec)
      
      // shadow register values
      byte_t  m_regRangeOffset;         ///< range part-to-part offset register
      byte_t  m_regRangeCrossTalk;      ///< range cross-talk register
      byte_t  m_regAlsGain;             ///< ambient light sensor gain register
      u16_t   m_regAlsIntPeriod;        ///< ALS itegration period register

      // most recent measurements
      double  m_fRange;                 ///< range (m)
      double  m_fAmbientLight;          ///< ambient light (lux)

      // mutual exclusion
      pthread_mutex_t m_mutex;          ///< mutex
  
      /*!
       * \brief Lock the share resource.
       *
       * The lock()/unlock() primitives provide a thread safe mechanism.
       *
       * \par Context:
       * Any.
       */
      void lock()
      {
        pthread_mutex_lock(&m_mutex);
      }
  
      /*!
       * \brief Unlock the shared resource.
       *
       * \par Context:
       * Any.
       */
      void unlock()
      {
        pthread_mutex_unlock(&m_mutex);
      }

      /*!
       * \brief Initialize sensor out-of-reset.
       *
       * \note Recommend.
       *
       * \copydoc doc_return_std
       */
      int outOfResetInit();

      /*!
       * \brief Wait for sensor to be ready.
       *
       * \param regStatus Status register to check.
       * \param msecWait  Maximum time to wait for the measurement to complete
       *                  (milliseconds).
       *
       * \return
       * Returns the number of milliseconds waited. A value \h_gt msecWait
       * indicates a timeout.
       */
      u32_t waitForSensorReady(u16_t regStatus, u32_t msecWait);

      /*!
       * \brief Wait for sensor measurement to complete.
       *
       * \param msecWait  Maximum time to wait for the measurement to complete
       *                  (milliseconds).
       * \param bitDone   Bit to check for measurement completion.
       *
       * \return
       * Returns the number of milliseconds waited. A value \h_gt msecWait
       * indicates a timeout.
       */
      u32_t waitForSensorMeasurement(u32_t msecWait, byte_t bitDone);

    }; // class LaeVL6180Mux


    // -------------------------------------------------------------------------
    // LaeVL6180MuxArray Class
    // -------------------------------------------------------------------------
    
    /*!
     * \brief VL6180 Time of Flight Array Class.
     *
     * This class holds an set of LaeVL6180 objects to manage a VL6180 sensor
     * array.
     *
     * Used in 2.0 hardware.
     */
    class LaeVL6180MuxArray
    {
    public:
      static const int AlsFreq = 4; ///< take an ambient measurement cycle rate

      typedef std::vector<sensor::vl6180::LaeVL6180Mux*> VecToFSensors;
                ///< time-of-flight sensor vector type

      /*!
       * \brief Initialization constructor.
       *
       * \note The constructor should be kept light weight, since hardware
       * version determines if this class is used. See \ref LaeRangeMux.
       *
       * \param mux       \h_i2c multiplexer switch.
       */
      LaeVL6180MuxArray(laelaps::LaeI2C &i2cBus);

      /*!
       * \brief Destructor.
       */
      virtual ~LaeVL6180MuxArray();

      /*!
       * \brief Clear data.
       */
      void clear();

      /*!
       * \brief Configure sensor array from product description.
       *
       * \param desc    Product description.
       *
       * \copydoc doc_return_std
       */
      virtual int configure(const laelaps::LaeDesc &desc);

      /*!
       * \brief Configure sensor array from tuning parameters.
       *
       * \param tunes   Tuning parameters.
       *
       * \copydoc doc_return_std
       */
      virtual int configure(const laelaps::LaeTunes &tunes);

      /*!
       * \brief Reload configuration tuning parameters.
       *
       * \param tunes   Tuning parameters.
       *
       * \copydoc doc_return_std
       */
      virtual int reload(const laelaps::LaeTunes &tunes);

      /*!
       * \brief Execute task in one cycle to take measurements.
       *
       * \par Context:
       * LaeThreadRange thread instance.
       */
      virtual void exec();


      //........................................................................
      // Attribute Member Functions
      //........................................................................
  
      /*!
       * \brief Get a range measurement.
       *
       * \param strKey        Sensor's unique name (key).
       * \param [out] fRange  Sensed object range (meters).
       *
       * \copydoc doc_return_std
       */
      virtual int getRange(const std::string &strKey, double &fRange);
  
      /*!
       * \brief Get all sensor range measurements.
       *
       * \param [out] vecNames    Vector of sensor unique names.
       * \param [out] vecRanges   Vector of associated sensor measured ranges
       *                          (meters).
       *
       * \copydoc doc_return_std
       */
      virtual int getRange(std::vector<std::string> &vecNames,
                           std::vector<double>      &vecRanges);
  
      /*!
       * \brief Get an ambient light illuminance measurement.
       *
       * \param strKey          Sensor's unique name (key).
       * \param [out] fAmbient  Sensed ambient light (lux).
       *
       * \copydoc doc_return_std
       */
      virtual int getAmbientLight(const std::string &strKey, double &fAmbient);
  
      /*!
       * \brief Get all sensor ambient light illuminance measurements.
       *
       * \param strKey            Sensor's unique name (key).
       * \param [out] vecAmbient  Vector of associated sensor measured ambients
       *                          (lux).
       *
       * \copydoc doc_return_std
       */
      virtual int getAmbientLight(std::vector<std::string> &vecNames,
                                  std::vector<double> &vecAmbient);

      /*!
       * \brief Get range sensor properties.
       *
       * \param strKey                 Sensor's unique name id (key).
       * \param [out] strRadiationType Radiation type.
       * \param [out] fFoV             Field of View (radians).
       * \param [out] fBeamdir         Center of beam direction (radians).
       * \param [out] fMin             Minimum range (meters).
       * \param [out] fMax             Maximum range (meters).
       *
       * \copydoc doc_return_std
       */
      virtual int getSensorProps(const std::string &strKey,
                                 std::string       &strRadiationType,
                                 double            &fFoV,
                                 double            &fBeamDir,
                                 double            &fMin,
                                 double            &fMax);

    protected:
      laelaps::LaeI2CMux  m_mux;          ///< \h_i2c multiplexor
      VecToFSensors       m_vecToF;       ///< time of flight sensors
      int                 m_nAlsIndex;    ///< ambient light sensor index
      int                 m_nAlsCounter;  ///< when zero, make measurement

    }; // LaeVL6180MuxArray


    // -------------------------------------------------------------------------
    // LaeRangeMux Class
    // -------------------------------------------------------------------------

    /*!
     * \brief Multiplexed range sensors class.
     *
     * The interface between the processor and the sensors is through an \h_i2c
     * connected sub-processor. Since the sub-processor (mostly) hides the 
     * specific sensor hardware, this class is more general.
     *
     * Used in 2.1+ hardware.
     */
    class LaeRangeMux
    {
    public:
      //
      // Times
      //
      static const long TStd = 100;  ///< standard wait write_read (usec)

      //
      // Types
      //
      typedef std::map<std::string, LaeVL6180SensorInfo>  SensorInfoMap;

      /*!
       * \brief Initialization constructor.
       *
       * \note The constructor should be kept light weight, since hardware
       * version determines if this class is used. See \ref LaeVL6180MuxArray.
       *
       * \param i2cbus  Bound open \h_i2c bus instance.
       * \param addr    ToF multiplexor sub-processor I2C address.
       */
      LaeRangeMux(laelaps::LaeI2C &i2cBus,
                  uint_t addr=laelaps::LaeI2CAddrToFMux);
  
      /*!
       * \brief Destructor.
       */
      virtual ~LaeRangeMux();
  
      /*!
       * \brief Clear data.
       */
      void clear();

      /*!
       * \brief Configure sensor array from product description.
       *
       * \param desc    Product description.
       *
       * \copydoc doc_return_std
       */
      virtual int configure(const laelaps::LaeDesc &desc);

      /*!
       * \brief Configure sensor array from tuning parameters.
       *
       * \param tunes   Tuning parameters.
       *
       * \copydoc doc_return_std
       */
      virtual int configure(const laelaps::LaeTunes &tunes);

      /*!
       * \brief Reload configuration tuning parameters.
       *
       * \param tunes   Tuning parameters.
       *
       * \copydoc doc_return_std
       */
      virtual int reload(const laelaps::LaeTunes &tunes);

      /*!
       * \brief Execute one cycle to take measurements.
       *
       * \par Context:
       * LaeThreadRange thread instance.
       */
      virtual void exec();

      //........................................................................
      // ToFMux Sub-Processor Member Functions
      //........................................................................
      
      /*!
       * \brief Get the firmware version command.
       *
       * \param [out] uVerNum   Firmware version number.
       *
       * \copydoc doc_return_std
       */
      virtual int cmdGetFwVersion(uint_t &uVerNum);
  
      /*!
       * \brief Read sensor identification command.
       *
       * \param [out] id  Identification structure.
       *
       * \copydoc doc_return_std
       */
      int cmdGetIdent(const std::string &strKey, VL6180xIdentification &ident);
  
      /*!
       * \brief Read exported tuning parameters command.
       *
       * \param [out] uRangeOffset    ToF sensor part-to-part offset.
       * \param [out] uRangeCrossTalk ToF sensor cross-talk compensation.
       * \param [out] fAlsGain        Ambient light sensor analog gain.
       * \param [out] uAlsIntPeriod   Ambient light sensor integration period
       *                                (msec).
       *
       * \copydoc doc_return_std
       */
      int cmdGetTunes(const std::string &strKey,
                      uint_t            &uRangeOffset,
                      uint_t            &uRangeCrossTalk,
                      double            &fAlsGain,
                      uint_t            &uAlsIntPeriod);
  
      /*!
       * \brief Get measured object distances.
       *
       * \param [out] vecRanges   Vector of distances (mm).
       *
       * \copydoc doc_return_std
       */
      int cmdGetRanges(std::vector<double> &vecRanges);
    
      /*!
       * \brief Get measured ambient light illumination.
       *
       * \param [out] vecLux  Vector of illuminations (lux).
       *
       * \copydoc doc_return_std
       */
      int cmdGetAmbientLight(std::vector<double> &vecLux);
  
      /*!
       * \brief Tune time-of-flight range sensor command.
       *
       * \param uRangeOffset    ToF sensor part-to-part offset.
       *                        If VL6180X_FACTORY_DFT then leave as is.
       * \param uRangeCrossTalk ToF sensor cross-talk compensation.
       *                        If VL6180X_FACTORY_DFT then leave as is.
       *
       * \copydoc doc_return_std
       */
      int cmdTuneToFSensor(const std::string &strKey,
                           uint_t             uRangeOffset,
                           uint_t             uRangeCrossTalk);

      /*!
       * \brief Tune ambient light sensor command.
       *
       * \param fAlsGain        Ambient light sensor analog gain.
       * \param uAlsIntPeriod   Ambient light sensor integration period (msec).
       *
       * \copydoc doc_return_std
       */
      int cmdTuneAls(const std::string &strKey,
                     double             fAlsGain,
                     uint_t             uAlsIntPeriod);


      //........................................................................
      // Attribute Member Functions
      //........................................................................
  
      /*!
       * \brief Get a range measurement.
       *
       * \param strKey        Sensor's unique name (key).
       * \param [out] fRange  Sensed object range (meters).
       *
       * \copydoc doc_return_std
       */
      virtual int getRange(const std::string &strKey, double &fRange);
  
      /*!
       * \brief Get all sensor range measurements.
       *
       * \param [out] vecNames    Vector of sensor unique names.
       * \param [out] vecRanges   Vector of associated sensor measured ranges
       *                          (meters).
       *
       * \copydoc doc_return_std
       */
      virtual int getRange(std::vector<std::string> &vecNames,
                           std::vector<double>      &vecRanges);
  
      /*!
       * \brief Get an ambient light illuminance measurement.
       *
       * \param strKey          Sensor's unique name (key).
       * \param [out] fAmbient  Sensed ambient light (lux).
       *
       * \copydoc doc_return_std
       */
      virtual int getAmbientLight(const std::string &strKey, double &fAmbient);
  
      /*!
       * \brief Get all sensor ambient light illuminance measurements.
       *
       * \param [out] vecNames    Vector of sensor unique names.
       * \param [out] vecAmbient  Vector of associated sensor measured ambients
       *                          (lux).
       *
       * \copydoc doc_return_std
       */
      virtual int getAmbientLight(std::vector<std::string> &vecNames,
                                  std::vector<double> &vecAmbient);
  
      /*!
       * \brief Get range sensor properties.
       *
       * \param strKey                 Sensor's unique name id (key).
       * \param [out] strRadiationType Radiation type.
       * \param [out] fFoV             Field of View (radians).
       * \param [out] fBeamdir         Center of beam direction (radians).
       * \param [out] fMin             Minimum range (meters).
       * \param [out] fMax             Maximum range (meters).
       *
       * \copydoc doc_return_std
       */
      virtual int getSensorProps(const std::string &strKey,
                                 std::string       &strRadiationType,
                                 double            &fFoV,
                                 double            &fBeamDir,
                                 double            &fMin,
                                 double            &fMax);
      
    protected:
      // hardware
      laelaps::LaeI2C  &m_i2cBus;       ///< bound \h_i2c bus instance
      uint_t            m_addrSubProc;  ///< \h_i2c sub-processor address
      uint_t            m_uFwVer;       ///< firmware version number

      // sensor info indexed by key
      SensorInfoMap m_mapInfo;

      // shadow values
      std::vector<double>   m_vecRanges;  ///< measured distances (meters)
      std::vector<double>   m_vecLux;     ///< measured ambient light (lux)

      // mutual exclusion
      pthread_mutex_t m_mutex;          ///< mutex
  
      /*!
       * \brief Lock the share resource.
       *
       * The lock()/unlock() primitives provide a thread safe mechanism.
       *
       * \par Context:
       * Any.
       */
      void lock()
      {
        pthread_mutex_lock(&m_mutex);
      }
  
      /*!
       * \brief Unlock the shared resource.
       *
       * \par Context:
       * Any.
       */
      void unlock()
      {
        pthread_mutex_unlock(&m_mutex);
      }

      int cmdGetRanges();

      int cmdGetAmbientLight();

    }; // class LaeRangeMux


    // -------------------------------------------------------------------------
    // LaeRangeSensorGroup Class
    // -------------------------------------------------------------------------

    /*!
     * \brief Range sensor group class.
     *
     * Laelaps sensor hardware changed between v2.0 and v2.1.
     */
    class LaeRangeSensorGroup
    {
    public:
      LaeVL6180MuxArray m_interface_2_0;  ///< interface v2.0
      LaeRangeMux       m_interface_2_1;  ///< interface v2.1+

      /*!
       * \brief Default constructor.
       *
       * \param i2cBus  I2C interface.
       */
      LaeRangeSensorGroup(laelaps::LaeI2C &i2cBus) :
        m_interface_2_0(i2cBus), m_interface_2_1(i2cBus)
      {
      }

      /*!
       * \brief Destructor.
       */
      ~LaeRangeSensorGroup()
      {
      }

      /*!
       * \brief Clear data.
       */
      void clear();

      /*!
       * \brief Configure sensor group from product description.
       *
       * \param desc    Product description.
       *
       * \copydoc doc_return_std
       */
      virtual int configure(const laelaps::LaeDesc &desc);

      /*!
       * \brief Configure sensor group from tuning parameters.
       *
       * \param tunes   Tuning parameters.
       *
       * \copydoc doc_return_std
       */
      virtual int configure(const laelaps::LaeTunes &tunes);

      /*!
       * \brief Reload configuration tuning parameters.
       *
       * \param tunes   Tuning parameters.
       *
       * \copydoc doc_return_std
       */
      virtual int reload(const laelaps::LaeTunes &tunes);

      /*!
       * \brief Get range sensor properties.
       *
       * \param strKey                 Sensor's unique name id (key).
       * \param [out] strRadiationType Radiation type.
       * \param [out] fFoV             Field of View (radians).
       * \param [out] fBeamdir         Center of beam direction (radians).
       * \param [out] fMin             Minimum range (meters).
       * \param [out] fMax             Maximum range (meters).
       *
       * \copydoc doc_return_std
       */
      virtual int getSensorProps(const std::string &strKey,
                                 std::string       &strRadiationType,
                                 double            &fFoV,
                                 double            &fBeamDir,
                                 double            &fMin,
                                 double            &fMax);

      /*!
       * \brief Get a range measurement.
       *
       * \param strKey        Sensor's unique name (key).
       * \param [out] fRange  Sensed object range (meters).
       *
       * \copydoc doc_return_std
       */
      virtual int getRange(const std::string &strKey, double &fRange);
  
      /*!
       * \brief Get all sensor range measurements.
       *
       * \param [out] vecNames    Vector of sensor unique names.
       * \param [out] vecRanges   Vector of associated sensor measured ranges
       *                          (meters).
       *
       * \copydoc doc_return_std
       */
      virtual int getRange(std::vector<std::string> &vecNames,
                           std::vector<double>      &vecRanges);
  
      /*!
       * \brief Get an ambient light illuminance measurement.
       *
       * \param strKey          Sensor's unique name (key).
       * \param [out] fAmbient  Sensed ambient light (lux).
       *
       * \copydoc doc_return_std
       */
      virtual int getAmbientLight(const std::string &strKey, double &fAmbient);
  
      /*!
       * \brief Get all sensor ambient light illuminance measurements.
       *
       * \param strKey            Sensor's unique name (key).
       * \param [out] vecAmbient  Vector of associated sensor measured ambients
       *                          (lux).
       *
       * \copydoc doc_return_std
       */
      virtual int getAmbientLight(std::vector<std::string> &vecNames,
                                  std::vector<double> &vecAmbient);


    }; // class LaeRangeSensorGroup

  } // namespace vl6180
} // namespace sensor


#endif // _LAE_VL6180_H
