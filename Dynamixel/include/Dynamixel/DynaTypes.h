////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   libDynamixel
//
// File:      DynaTypes.h
//
/*! \file
 *
 * $LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
 * $Rev: 3845 $
 *
 * \ingroup dyna_lib_hdrs
 *
 * \brief RoadNarrows Dynamixel Fundatmental Types.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011-2015.  RoadNarrows LLC.
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 */
/* 
 * @EuloBegin@
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
 * @EuloEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _DYNA_TYPES_H
#define _DYNA_TYPES_H

#include "rnr/rnrconfig.h"
#include "rnr/units.h"

#ifndef SWIG
C_DECLS_BEGIN
#endif // SWIG

#ifndef __cplusplus
typedef unsigned char bool;
#endif


/*!
 * \ingroup dyna_lib_types
 * \defgroup dyna_lib_types_base Dynamixel Fundamental Data Types
 *
 * \{
 */
/*! \} */

/*!
 * \ingroup dyna_lib_types_base
 * \brief Dynamixel Servo Specification Structure.
 */
typedef struct
{
  char   *m_sModelName;           ///< model name
  double  m_fWeight;              ///< weight (grams)
  double  m_fWidth;               ///< width (mm)
  double  m_fHeight;              ///< height (mm)
  double  m_fDepth;               ///< depth (mm)
  double  m_fResolution;          ///< resolution (degrees)
  double  m_fGearRedectionRatio;  ///< gear reduction ratio
  double  m_fStallTorque;         ///< max stall torque (kgf) at optimal power
  double  m_fMaxSpeed;            ///< max no-load speed (rpm) at optimal power
  double  m_fAngleMin;            ///< min rotation angle in servo mode (deg)
  double  m_fAngleMax;            ///< max rotation angle in servo mode (deg)
  uint_t  m_uSupportedModes;      ///< \ref dyna_servo_mode
  bool    m_bHas360Pos;           ///< does [not] have full 360 position info 
  double  m_fTempMin;             ///< minimum operational temperature (C)
  double  m_fTempMax;             ///< maximum operational temperature (C)
  double  m_fVoltMin;             ///< minimum operational voltage (V)
  double  m_fVoltMax;             ///< maximum operational voltage (V)
  uint_t  m_uCtlMethodUsed;       ///< \ref dyna_ctl_method
  uint_t  m_uRawPosMin;           ///< minimum raw position value (servo mode)
  uint_t  m_uRawPosMax;           ///< maximum raw position value (servo mode)
  uint_t  m_uRawPosModulo;        ///< raw position modulo
  uint_t  m_uRawSpeedMin;         ///< minimum raw speed magnitude value
  uint_t  m_uRawSpeedMax;         ///< maximum raw speed magnitude value
  uint_t  m_uRawTorqueMin;        ///< minimum raw torque value
  uint_t  m_uRawTorqueMax;        ///< maximum raw torque value
  uint_t  m_uRawTempMin;          ///< minimum raw temperature value
  uint_t  m_uRawTempMax;          ///< maximum raw temperature value
  uint_t  m_uRawVoltMin;          ///< minimum raw voltage value
  uint_t  m_uRawVoltMax;          ///< maximum raw voltage value
} DynaServoSpec_T;

/*!
 * \ingroup dyna_lib_types_base
 * \brief Dynamixel Servo Configuration Structure.
 *
 * Values exposed to applications are in current units.
 */
typedef struct
{
  uint_t  m_uModelNum;            ///< servo model number
  uint_t  m_uFwVer;               ///< firmware version
  int     m_nServoId;             ///< servo id 
  uint_t  m_uBaudRate;            ///< servo baud rate (actual)
  uint_t  m_uTRetDelay;           ///< response return delay time (raw or )
  uint_t  m_uLimCw;               ///< clockwise angle limit (deg
  uint_t  m_uLimCcw;              ///< counterclockwise angle limit
  uint_t  m_uLimTemp;             ///< maximum temperature limit
  uint_t  m_uLimVoltMin;          ///< minimum voltage limit
  uint_t  m_uLimVoltMax;          ///< maximum voltage limit
  uint_t  m_uLimTorqueMax;        ///< on power-up maximum torque limit
  uint_t  m_uSrl;                 ///< status return level
  bool_t  m_uAlarmLed;            ///< alarm led enable
  uint_t  m_uAlarmShutdown;       ///< alarm shutdown mask enable causes
  uint_t  m_uServoMode;           ///< servo mode \ref dyna_servo_mode
} DynaServoCfg_T;

/*!
 * \ingroup dyna_lib_types_base
 * \brief Dynamixel Control Method Structure.
 *
 * The Dynamixel servos have two types of position control method depending on
 * servo model. See \ref dyna_ctl_method.
 */
typedef struct
{
  uint_t  m_uCtlMethod;           ///< \ref dyna_ctl_method
  union
  {
    /// compliance control method parameters
    struct
    {
      uint_t    m_uCwMargin;      ///< clockwise compliance margin
      uint_t    m_uCwSlope;       ///< clockwise compliance slope
      uint_t    m_uCcwMargin;     ///< counterclockwise compliance margin
      uint_t    m_uCcwSlope;      ///< counterclockwise compliance slope
    } m_comp;

    // pid control method parameters
    struct
    {
      uint_t    m_uPGain;         ///< proportional constant gain
      uint_t    m_uIGain;         ///< integral constant gain
      uint_t    m_uDGain;         ///< derivative constant gain
    } m_pid;
  } m_params;                     ///< parameters to control methods
} DynaServoCtlMethod_T;

/*!
 * \ingroup dyna_lib_types_base
 * \brief Dynamixel Virtual Odometer Structure.
 *
 * The virtual odometer keeps track of the servo virtual position. It is most
 * usefull with servos configured in continuous mode.
 */
typedef struct
{
  int  m_nOdometer;     ///< current odometer reading (accumulator)
  //int  m_nOdModulo;     ///< odometer rollover value. NOT USED
  bool m_bOdEnabled;    ///< odometer mapping [not] enabled
  int  m_nOdDir;        ///< odometer direction: normal or reverse
  int  m_nOdGoalPos;    ///< odometer goal position
  int  m_nEncLastPos;   ///< last encoder position read from servo
  int  m_nEncZeroPt;    ///< servo encoder zero point position
} DynaServoOdometer_T;

/*!
 * \ingroup dyna_lib_types_base
 * \brief Dynamixel Servo State Structure.
 *
 * Values exposed to applications are in current units.
 */
typedef struct
{
  uint_t                m_uAlarms;        ///< current servo alarms and errors
  bool_t                m_bTorqueEnabled; ///< torque [not] enabled
  bool_t                m_bLed;           ///< led on/off
  DynaServoCtlMethod_T  m_ctlMethod;      ///< position control method
  uint_t                m_uGoalPos;       ///< goal position (encoder ticks)
  int                   m_nGoalSpeed;     ///< goal speed (raw)
  uint_t                m_uLimTorqueMax;  ///< maximum torque limit
  uint_t                m_uOverTorqueTh;  ///< set over torque cond. threshold
  uint_t                m_uClearTorqueTh; ///< clear over torque cond. threshold
  bool                  m_bOverTorqueCond;///< over torque condition state
  uint_t                m_uCurPos;        ///< current position (encoder ticks)
  int                   m_nCurSpeed;      ///< current speed (raw)
  int                   m_nCurLoad;       ///< current load
  uint_t                m_uCurVolt;       ///< current voltage
  uint_t                m_uCurTemp;       ///< current temperature
  bool_t                m_bIsMoving;      ///< servo is [not] moving
  DynaServoOdometer_T   m_od;             ///< servo virtual odometer
} DynaServoState_T;

/*!
 * \ingroup dyna_lib_type_base
 * \brief Linked Servos Configuration Info Structure.
 */
typedef struct
{
  uint_t  m_uLinkType;    ///< linked type \ref dyna_servo_link_type
  int     m_nServoIdMate; ///< linked mate's servo id
  bool    m_bRotReversed; ///< linked do [not] rotate in opposite directions
} DynaServoLink_T;

/*!
 * \ingroup dyna_lib_type_base
 * \brief Servo Proxy Agents Calls Structure.
 */
typedef struct
{
  /*! write servo goal position */
  int (*m_fnWriteGoalPos)(int nServoId, int nGoalPos, void *pUsrArg);
                 
  /*! write servo goal speed */
  int (*m_fnWriteGoalSpeed)(int nServoId, int nGoalSpeed, void *pUsrArg);

  /*! write servo goal speed and position */
  int (*m_fnWriteGoalSpeedPos)(int    nServoId,
                               int    nGoalSpeed,
                               int    nGoalPos,
                               void  *pUsrArg);
} DynaAgent_T;

/*!
 * \ingroup dyna_lib_types_base
 * \brief User registered function to start control of the position of this
 * servo while in continuous mode.
 *
 * \param nServoId    Servo id.
 * \param uOdGoalPos  Servo goal position in virtual odometer units.
 * \param pUserArg    User supplied argument passed unaltered to the function.
 *
 * \copydoc doc_return_std
 */
typedef int (*DynaServoStartPosCtlFunc_T)(int     nServoId,
                                          uint_t  uOdGoalPos,
                                          void   *pUserArg);

/*!
 * \ingroup dyna_lib_types_base
 * \brief User registered function to stop control of the position of this
 * servo while in continuous mode.
 *
 * \param nServoId    Servo id.
 * \param pUserArg    User supplied argument passed unaltered to the function.
 *
 * \copydoc doc_return_std
 */
typedef int (*DynaServoStopPosCtlFunc_T)(int nServoId, void *pUserArg);

/*!
 * \ingroup dyna_lib_types_base
 * \brief Dynamixel Working Units Structure.
 *
 * Values exposed to applications are in current units.
 *
 * \todo TODO Add support for units
 */
typedef struct 
{
  units_t unitsPos;             ///< servo position units
  units_t unitsSpeed;           ///< servo speed units
  units_t unitsTorque;          ///< servo torque units
  units_t unitsTemp;            ///< servo temperature units
  units_t unitsVolt;            ///< servo voltage units
  units_t unitsAmp;             ///< servo electric current units
} DynaUnits_T;

/*!
 * \ingroup dyna_lib_types_base 
 * Synchronous Write Tuple Structure
 */
typedef struct
{
  int       m_nServoId;     ///< servo id
  uint_t    m_uVal;         ///< write value
} DynaSyncWriteTuple_T;

/*!
 * \ingroup dyna_lib_types_base
 * \brief Position Tuple Structure
 */
typedef struct
{
  int       m_nServoId;     ///< servo id
  int       m_nPos;         ///< position
} DynaPosTuple_T;

/*!
 * \ingroup dyna_lib_types_base
 * \brief Synchronous Write Speed Tuple Structure
 */
typedef struct
{
  int       m_nServoId;     ///< servo id
  int       m_nSpeed;       ///< speed
} DynaSpeedTuple_T;

/*!
 * \ingroup dyna_lib_types_base
 * \brief Synchronous Write Speed-Position Tuple Structure
 */
typedef struct
{
  int       m_nServoId;     ///< servo id
  int       m_nSpeed;       ///< speed
  int       m_nPos;         ///< position
} DynaSpeedPosTuple_T;

/*!
 * \ingroup dyna_lib_types_base
 * \brief Real-value Tuple Structure
 */
typedef struct
{
  int       nServoId;     ///< servo id
  double    fVal;         ///< real value
} DynaRealTuple_T;

/*!
 * \ingroup dyna_lib_types_base
 * \brief Real-value Speed-Position 3-Tuple Structure
 */
typedef struct
{
  int       m_nServoId;     ///< servo id
  double    m_fSpeed;       ///< speed
  double    m_fPos;         ///< position
} DynaRealSpeedPosTuple_T;

/*!
 * \ingroup dyna_lib_types_base
 * \brief Servo Health Tuple Structure
 */
typedef struct
{
  int       m_nServoId;     ///< servo id
  uint_t    m_uAlarms;      ///< servo alarms
  int       m_nLoad;        ///< servo load
  uint_t    m_uVolts;       ///< servo volts
  uint_t    m_uTemp;        ///< servo temperature
} DynaHealthTuple_T;

/*!
 * \ingroup dyna_lib_types_base
 * \brief Servo Control Table Entry
 */
typedef struct
{
  uint_t      m_uAddr;      ///< control table entry address
  const char *m_sName;      ///< entry name
  uint_t      m_uSize;      ///< entry size 1 or 2 bytes
  uint_t      m_uMask;      ///< mask of entry
  bool_t      m_bSigned;    ///< entry is [not] signed integer
  const char *m_sFmt;       ///< alternal output format
} DynaCtlTblEntry_T;


#ifndef SWIG
C_DECLS_END
#endif // SWIG

#endif // _DYNA_TYPES_H
