////////////////////////////////////////////////////////////////////////////////
//
// Package:   mousebot
//
// File:      mousebot.h
//
/*! \file
 *
 * $LastChangedDate: 2012-04-26 12:16:42 -0600 (Thu, 26 Apr 2012) $
 * $Rev: 1899 $
 *
 * \brief RoadNarrows MouseBot robot top-level header file.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 * \author Rob Shiely     (rob@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2010.  RoadNarrows LLC.
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 */
// Permission is hereby granted, without written agreement and without
// license or royalty fees, to use, copy, modify, and distribute this
// software and its documentation for any purpose, provided that
// (1) The above copyright notice and the following two paragraphs
// appear in all copies of the source code and (2) redistributions
// including binaries reproduces these notices in the supporting
// documentation.   Substantial modifications to this software may be
// copyrighted by their authors and need not follow the licensing terms
// described here, provided that the new terms are clearly indicated in
// all files where they apply.
//
// IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
// OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
// PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
// EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.
//
// THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
// "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
// PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef _MOUSEBOT_H
#define _MOUSEBOT_H

#include "rnr/rnrconfig.h"
#include "rnr/units.h"
#include "rnr/log.h"

#ifndef SWIG
C_DECLS_BEGIN
#endif // SWIG

// ---------------------------------------------------------------------------
// Defines and Types
// ---------------------------------------------------------------------------

/*!
 * \ingroup mousebot_ecodes
 *
 * \{
 */
#define MOUSEBOT_OK                       0 ///< not an error, success

#define MOUSEBOT_ECODE_GEN                1 ///< general, unspecified error
#define MOUSEBOT_ECODE_LIBKB              2 ///< libkorebot error
#define MOUSEBOT_ECODE_LIBKB_RECV         3 ///< libkorebot bad knet receive
#define MOUSEBOT_ECODE_LIBKB_SEND         4 ///< libkorebot bad knet send 
#define MOUSEBOT_ECODE_BAD_UNITS          5 ///< unsupported or bad units
#define MOUSEBOT_ECODE_BAD_VAL            6 ///< bad value
#define MOUSEBOT_ECODE_SYS                7 ///< system (errno) error
#define MOUSEBOT_ECODE_INTERNAL           8 ///< internal error (bug)

#define MOUSEBOT_ECODE_BADEC              9 ///< bad error code

#define MOUSEBOT_ECODE_NUMOF             10 ///< number of error codes
/*! \} */


// Limits
#define MOUSEBOT_INT_MAX      2147483647      ///< integer maximum
#define MOUSEBOT_INT_MIN      (-MOUSEBOT_INT_MAX-1) ///< integer minimum

/*!
 * \ingroup k3_dimensions
 *
 * \{
 */
// Dimensions
#define MOUSEBOT_BASE_MAX_RADIUS_MM   65      ///< maximum base radius (mm)
#define MOUSEBOT_BASE_MAX_HEIGHT_MM   70      ///< maximum base height (mm)
#define MOUSEBOT_BASE_WHEEL_BASE_MM   90      ///< wheel-to-wheel base (mm)
#define MOUSEBOT_BASE_WHEEL_DIAM_MM   41      ///< wheel diameter (mm)
/*! \} */

/*!
 * \ingroup k3_motor_prop
 *
 * \{
 */
// Motor and Odometrey Properties
#define MOUSEBOT_MOTOR_LEFT                 0 ///< left motor id
#define MOUSEBOT_MOTOR_RIGHT                1 ///< right motor id
#define MOUSEBOT_MOTOR_NUMOF                2 ///< total number of motors
#define MOUSEBOT_SPEED_MIN_ABS           2000
                ///< practical absolute minimum controlled raw spead
#define MOUSEBOT_SPEED_MAX_ABS          65535
                ///< practical absolute maximum controlled raw spead
#define MOUSEBOT_POS_PROF_MIN_SPEED     MOUSEBOT_SPEED_MIN_ABS
                ///< move to position profile minimum raw spead
#define MOUSEBOT_POS_PROF_MAX_SPEED     32767
                ///< move to position profile maximum raw spead
#define MOUSEBOT_POS_PROF_MIN_ACCEL        10
                ///< move to position profile minimum acceleration to position
#define MOUSEBOT_POS_PROF_MAX_ACCEL       255
                ///< move to position profile maximum acceleration to position
#define MOUSEBOT_OD_MIN        (MOUSEBOT_INT_MIN+1)
                ///< minimum odometer position negative raw value
#define MOUSEBOT_OD_MAX        (MOUSEBOT_INT_MAX-1)
                ///< maximum odometer position positve raw value
#define MOUSEBOT_OD_UM_PER_STEP            47 ///< micrometers per odometer step
#define MOUSEBOT_OD_NUMOF                   2
                ///< total number of odometers (one per motor)
/*! \} */

/*!
 * \ingroup k3_ir_prop
 *
 * \{
 */
// Infrared Sensors Properties (degrees as viewed from top, x-axis forward)
#define MOUSEBOT_IR_POS_BACK_LEFT        0  ///< back left 135\h_deg infrared sensor
#define MOUSEBOT_IR_POS_LEFT_90          1  ///< left 90\h_deg infrared sensor
#define MOUSEBOT_IR_POS_LEFT_45          2  ///< left 45\h_deg infrared sensor
#define MOUSEBOT_IR_POS_FRONT_LEFT       3  ///< front left 10\h_deg infrared sensor
#define MOUSEBOT_IR_POS_FRONT_RIGHT      4  ///< front right -10\h_deg infrared sensor
#define MOUSEBOT_IR_POS_RIGHT_45         5  ///< right -45\h_deg infrared sensor
#define MOUSEBOT_IR_POS_RIGHT_90         6  ///< right -90\h_deg infrared sensor
#define MOUSEBOT_IR_POS_BACK_RIGHT       7  ///< back right -135\h_deg infrared sensor
#define MOUSEBOT_IR_POS_BACK             8  ///< back 180\h_deg infrared sensor
#define MOUSEBOT_IR_POS_GROUND_RIGHT     9  ///< bottom right infrared sensor
#define MOUSEBOT_IR_POS_GROUND_LEFT     10  ///< bottom left infrared sensor
#define MOUSEBOT_IR_NUMOF               11  ///< total number of ir sensors
#define MOUSEBOT_IR_VAL_MIN              0  ///< minimum ir sensor raw value
#define MOUSEBOT_IR_VAL_MAX           4095  ///< maximum ir sensor value (12-bit)
#define MOUSEBOT_IR_GRP_FRONT            0  ///< front infrared group  [3, 4]
#define MOUSEBOT_IR_GRP_RIGHT            1  ///< right infrared group  [5, 6, 7]
#define MOUSEBOT_IR_GRP_BACK             2  ///< back infrared group   [0, 8, 7]
#define MOUSEBOT_IR_GRP_LEFT             3  ///< left infrared group   [0, 1, 2]
#define MOUSEBOT_IR_GRP_GROUND           4  ///< bottom infrared group [9, 10]
/*! \} */

/*!
 * \ingroup k3_uss_prop
 *
 * \{
 */
// UltraSonic Sensors Properties (degrees as viewed from top, x-axis forward)
#define MOUSEBOT_USS_POS_LEFT_90              0   ///< left 90\h_deg uss
#define MOUSEBOT_USS_POS_LEFT_45              1   ///< left 45\h_deg uss
#define MOUSEBOT_USS_POS_FRONT                2   ///< front 0\h_deg uss
#define MOUSEBOT_USS_POS_RIGHT_45             3   ///< right -45\h_deg uss
#define MOUSEBOT_USS_POS_RIGHT_90             4   ///< right -90\h_deg uss
#define MOUSEBOT_USS_NUMOF                    5   ///< total number of uss sensors
#define MOUSEBOT_USS_ECHO_MAX_MIN             1   ///< maximum minimum echoes/measure
#define MOUSEBOT_USS_ECHO_MAX_MAX             5   ///< maximum maximum echoes/measure
#define MOUSEBOT_USS_ECHO_MAX_DFT             3   ///< default maximum echoes/measure
#define MOUSEBOT_USS_DIST_MIN                20   ///< minimum echo distance (cm)
#define MOUSEBOT_USS_DIST_MAX               400   ///< maximum echo distance (cm)
#define MOUSEBOT_USS_AMPL_MAX     (MOUSEBOT_INT_MAX-1)  ///< maximum echo amplitude (?units)
#define MOUSEBOT_USS_TIME_MAX     (MOUSEBOT_INT_MAX-1)  ///< maximum echo time (? units)
#define MOUSEBOT_USS_NOISE_MAX    (MOUSEBOT_INT_MAX-1)  ///< maximum noise value (? units)
#define MOUSEBOT_USS_LEFT_90_EN_DFT       false
                                  ///< left 90\h_deg uss disabled by default
#define MOUSEBOT_USS_LEFT_45_EN_DFT       false
                                  ///< left 45\h_deg uss disabled by default
#define MOUSEBOT_USS_FRONT_EN_DFT          true
                                  ///< front 0\h_deg enabled by default
#define MOUSEBOT_USS_RIGHT_45_EN_DFT      false
                                  ///< right -45\h_deg uss disabled by default
#define MOUSEBOT_USS_RIGHT_90_EN_DFT      false
                                  ///< right -90\h_deg uss disabled by default
#define MOUSEBOT_USS_HAS_BODY_DFT          true   ///< upper body mounted by default
/*! \} */

// Firmware Operation Indices
#define MOUSEBOT_FWOP_INDEX_USS_MASK        0     ///< mask of enabled/disabled USSs
#define MOUSEBOT_FWOP_INDEX_USS_MAX_ECHOES  1     ///< max num of echoes/measurment
#define MOUSEBOT_FWOP_INDEX_NOT_USED_2      2     ///< not used
#define MOUSEBOT_FWOP_INDEX_NOT_USED_3      3     ///< not used
#define MOUSEBOT_FWOP_INDEX_IR_MASK         4     ///< mask of enabled/disabled IRs
#define MOUSEBOT_FWOP_INDEX_IR_GAIN_BRAIT   5     ///< Braitenberg IR gain
#define MOUSEBOT_FWOP_INDEX_UPPER_BODY      6     ///< plastic upper body [un]mounted

// Buffers Sizes
#define MOUSEBOT_BUF_MAX                  100     ///< command/response buffer maximum

/*!
 * \ingroup k3_defaults
 *
 * \{
 */
// Default units and values
#define MOUSEBOT_UNITS_SPEED_DFT    units_permil  ///< default speed units
#define MOUSEBOT_SPEED_MAX_FWD_DFT  1000
                ///< default forward maximum \h_permil speed
#define MOUSEBOT_SPEED_MAX_BWD_DFT -1000
                ///< default backwrd maximum \h_permil speed

#define MOUSEBOT_UNITS_OD_DFT       units_mm  ///< default odometery units
#define MOUSEBOT_OD_MAX_FWD_DFT     1000000
          ///< default forward maximum odometer position in mm
#define MOUSEBOT_OD_MAX_BWD_DFT    -1000000
          ///< default backward maximum (most negative) odometer position in mm

#define MOUSEBOT_UNITS_IR_DFT       units_raw ///< default sensor IR units
#define MOUSEBOT_UNITS_USS_DFT      units_mm  ///< default USS sensor distance units
/*! \} */

//
// Forward declarations
//
typedef struct knet_dev_s  *knet_dev_p; ///< pointer to knet_dev_t

/*!
 * \ingroup mousebot_types
 * Motor Configuration Structure
 */
typedef struct
{
  int m_nMaxBwdSpeed; ///< maximum backward current/raw speed limit \h_le 0
  int m_nMaxFwdSpeed; ///< maximum forward current/raw speed limit \h_ge 0
  int m_nMinOdometer; ///< minimum current/raw odometer position limit \h_le 0
  int m_nMaxOdometer; ///< maximum current/raw odometer position limit \h_ge 0
} MouseBotMotorCfg_T;
  
/*!
 * \ingroup mousebot_types
 * UltraSonic Sensor Configuration Structure
 */
typedef struct
{
  bool_t  m_bIsEnabled[MOUSEBOT_USS_NUMOF]; ///< array of enabled/disabled USS at pos
  uint_t  m_uMaxEchoes;               ///< maximum number of echoes/measure
  bool_t  m_bHasUpperBody;            ///< upper plastic body is [not] installed
} MouseBotUssCfg_T;
  
/*!
 * \ingroup mousebot_types
 * Robot Working Units Structure
 */
typedef struct
{
  units_t       m_unitsSpeed;         ///< motor speed units
  units_t       m_unitsOdometer;      ///< odometry distance units
  units_t       m_unitsIr;            ///< IR distance units
  units_t       m_unitsUss;           ///< USS distance units
} MouseBotAttrUnits_T;

/*!
 * \ingroup mousebot_types
 * UltraSonic Sense Returned Echo Structure
 */
typedef struct
{
  uint_t  m_uDistance;    ///< echo distance in current units
  uint_t  m_uAmplitude;   ///< echo signal strength (unknown units)
} MouseBotUssEcho_T;

/*!
 * \ingroup mousebot_types 
 * UltraSonic Sensor Measurement Structure
 */
typedef struct
{
  uint_t      m_uEchoCnt;       ///< number of returned echoes
  MouseBotUssEcho_T m_echo[MOUSEBOT_USS_ECHO_MAX_MAX];  ///< echo measurements
  uint_t      m_uMaxTimeTicks;  ///< max measurement time (unknown units)
  uint_t      m_uNoise;         ///< noise (unknown units)
} MouseBotUssMeas_T;

#ifndef SWIG

/*!
 * \ingroup mousebot_types
 * Khepera III Robot Structure
 */
typedef struct
{
  //
  // Devices
  //
  knet_dev_p    m_pDevDSPic;                    ///< dsPIC device handle
  knet_dev_p    m_pDevMot[MOUSEBOT_MOTOR_NUMOF];      ///< motor device handles

  //
  // Configuration and attributes
  //
  MouseBotMotorCfg_T  m_cfgMotor[MOUSEBOT_MOTOR_NUMOF];     ///< motor configurations
  MouseBotUssCfg_T    m_cfgUss;                       ///< USS configuration
  MouseBotAttrUnits_T m_attrUnits;                    ///< working units

  // 
  // Shadowed effector and sensor values in raw units (i.e. dog the robot)
  //
  int           m_dogSpeedLeft;                 ///< shadowed left motor speed
  int           m_dogSpeedRight;                ///< shadowed right motor speed
  int           m_dogSpeedRobot;                ///< shadowed robot center speed
  int           m_dogOdometer[MOUSEBOT_MOTOR_NUMOF];  ///< shadowed motor odometery
  uint_t        m_dogIrAmbient[MOUSEBOT_IR_NUMOF];    ///< shadowed ambient sensors
  uint_t        m_dogIrProximity[MOUSEBOT_IR_NUMOF];  ///< shadowed proximity sensors
  MouseBotUssMeas_T   m_dogUss[MOUSEBOT_USS_NUMOF];         ///< shadowed USS sensors
} MouseBot_T;


// ---------------------------------------------------------------------------
// Prototypes
// ---------------------------------------------------------------------------

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Utilities
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief Convert units to short symbol name.
 * \param u Units to look up.
 * \return Symbol name string.
 */
INLINE_IN_H const char *mousebotUnitsSym(units_t u)
{
  return u==units_raw? "": units_shortname(u);
}

extern const char *mousebotStrError(int ecode);

extern int k3LibKbToMouseBotEcode(int libkb_ecode);

extern void mousebotPrintBuf(FILE       *fp,
                       const char *sPreface,
                       byte_t      buf[],
                       const char *sFmt,
                       size_t      uCount,
                       size_t      uNLFreq,
                       uint_t      uCol);

#ifdef LOG
extern void mousebotLogBuf(const char *sPreface,
                     int         buf[],
                     size_t      uCount,
                     const char *sFmt);
#endif // LOG


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Initialization, Configuration, and Attribute Functions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

extern MouseBot_T *mousebotInit();

extern void mousebotExit(MouseBot_T *pMouseBot);

/*!
 * \brief Get the current motor speed units.
 * \param pMouseBot   Khepera3 robot instance.
 * \return Returns the current units. Returns units_undef on error.
 */
INLINE_IN_H units_t mousebotAttrGetUnitsSpeed(MouseBot_T *pMouseBot)
{
  return pMouseBot != NULL? pMouseBot->m_attrUnits.m_unitsSpeed: units_undef;
}

extern units_t mousebotAttrSetUnitsSpeed(MouseBot_T *pMouseBot, units_t unitsSpeed);

/*!
 * \brief Get the current motor odometer units.
 * \param pMouseBot   Khepera3 robot instance.
 * \return Returns the current units. Returns units_undef on error.
 */
INLINE_IN_H units_t mousebotAttrGetUnitsOdometer(MouseBot_T *pMouseBot)
{
  return pMouseBot != NULL? pMouseBot->m_attrUnits.m_unitsOdometer: units_undef;
}

extern units_t mousebotAttrSetUnitsOdometer(MouseBot_T *pMouseBot, units_t unitsOdometer);

/*!
 * \brief Get the current InfraRed sensor units.
 * \param pMouseBot   Khepera3 robot instance.
 * \return Returns the current units. Returns units_undef on error.
 */
INLINE_IN_H units_t mousebotAttrGetUnitsIr(MouseBot_T *pMouseBot)
{
  return pMouseBot != NULL? pMouseBot->m_attrUnits.m_unitsIr: units_undef;
}

extern units_t mousebotAttrSetUnitsIr(MouseBot_T *pMouseBot, units_t unitsIr);

/*!
 * \brief Get the current UltraSonic Sensor units.
 * \param pMouseBot   Khepera3 robot instance.
 * \return Returns the current units. Returns units_undef on error.
 */
INLINE_IN_H units_t mousebotAttrGetUnitsUss(MouseBot_T *pMouseBot)
{
  return pMouseBot != NULL? pMouseBot->m_attrUnits.m_unitsUss: units_undef;
}

extern units_t mousebotAttrSetUnitsUss(MouseBot_T *pMouseBot, units_t unitsUss);

extern int mousebotGetFwVer(MouseBot_T *pMouseBot, uint_t *puVersion, uint_t *puRevision);


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Motor Functions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

extern int mousebotMotorInit(MouseBot_T *pMouseBot, uint_t uWhich);

extern int mousebotMotorGetCfg(MouseBot_T   *pMouseBot,
                         uint_t        uWhich,
                         MouseBotMotorCfg_T *pMotorCfg);

extern int mousebotMotorSetCfg(MouseBot_T   *pMouseBot,
                         uint_t        uWhich,
                         MouseBotMotorCfg_T *pMotorCfg);

extern int mousebotMotorGetSpeeds(MouseBot_T *pMouseBot,
                            int        *pnSpeedLeft,
                            int        *pnSpeedRight);

extern int mousebotMotorSetSpeeds(MouseBot_T *pMouseBot, int nSpeedLeft, int nSpeedRight);

/*!
 * \brief Stop the Khepera III from moving.
 *
 * \param pMouseBot         Khepera3 robot instance.
 *
 * \copydoc doc_return_std
 */
INLINE_IN_H int mousebotMotorStop(MouseBot_T *pMouseBot)
{
  return mousebotMotorSetSpeeds(pMouseBot, 0, 0);
}

extern int mousebotMotorMoveTo(MouseBot_T *pMouseBot,
                         int         nSpeed,
                         int         nOdPosLeft,
                         int         nOdPosRight);

extern int mousebotMotorGetOdometry(MouseBot_T *pMouseBot,
                              int        *pnOdPosLeft,
                              int        *pnOdPosRight);

extern int mousebotMotorResetOdometry(MouseBot_T *pMouseBot);


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Infrared Sensor Functions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

extern int mousebotIrInit(MouseBot_T *pMouseBot);

extern int mousebotIrGetAmbient(MouseBot_T *pMouseBot, uint_t bufIr[]);

extern int mousebotIrGetProximity(MouseBot_T *pMouseBot, uint_t bufIr[]);

extern int mousebotIrGetProximityGroup(MouseBot_T *pMouseBot, int iGroup);


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// UltraSonic Sensor Functions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

extern int mousebotUssInit(MouseBot_T *pMouseBot);

/*!
 * \brief Get the current ultrasonic sensors configuration.
 * \param pMouseBot             Khepera3 robot instance.
 * \param [out] pUssCfg   Pointer to current USS configuration.
 * \copydoc doc_return_std
 */
INLINE_IN_H int mousebotUssGetCfg(MouseBot_T *pMouseBot,
                            MouseBotUssCfg_T *pUssCfg)
{
  *pUssCfg = pMouseBot->m_cfgUss;
  return MOUSEBOT_OK;
}

extern int mousebotUssSetCfg(MouseBot_T *pMouseBot, MouseBotUssCfg_T *pUssCfg);

extern int mousebotUssPing(MouseBot_T *pMouseBot, uint_t uUssPos, MouseBotUssMeas_T *pUssMeas);

#endif // SWIG


#ifndef SWIG
C_DECLS_END
#endif // SWIG


#endif // _MOUSEBOT_H
