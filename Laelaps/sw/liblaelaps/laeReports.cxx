////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Library:   liblaelaps
//
// File:      laeReports.cxx
//
/*! \file
 *
 * $LastChangedDate: 2016-04-08 15:42:39 -0600 (Fri, 08 Apr 2016) $
 * $Rev: 4380 $
 *
 * \brief Implementations of Laelaps requested and/or published report classes.
 *
 * \author Robin Knight     (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2015-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
/*
 * @EulaBegin@
 * 
 * Unless otherwise stated explicitly, all materials contained are copyrighted
 * and may not be used without RoadNarrows LLC's written consent,
 * except as provided in these terms and conditions or in the copyright
 * notice (documents and software) or other proprietary notice provided with
 * the relevant materials.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY 
 * MEMBERS/EMPLOYEES/CONTRACTORS OF ROADNARROWS OR DISTRIBUTORS OF THIS SOFTWARE
 * BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
 * DOCUMENTATION, EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHORS AND  ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////


#include <stdio.h>

#include <string>
#include <vector>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Laelaps/RoboClaw.h"

#include "Laelaps/laelaps.h"
#include "Laelaps/laeMotor.h"
#include "Laelaps/laeAlarms.h"
#include "Laelaps/laeDb.h"
#include "Laelaps/laeDesc.h"
#include "Laelaps/laeReports.h"
#include "Laelaps/laeRobot.h"

using namespace std;
using namespace laelaps;
using namespace motor::roboclaw;


// -----------------------------------------------------------------------------
// Class LaeRptMotorCtlrHealth
// -----------------------------------------------------------------------------

LaeRptMotorCtlrHealth::LaeRptMotorCtlrHealth()
{
  m_fTemperature  = 0.0;
  m_fVoltage      = 0.0;
  LaeAlarms::clearAlarms(m_alarms);
}

LaeRptMotorCtlrHealth::LaeRptMotorCtlrHealth(const LaeRptMotorCtlrHealth &src)
{
  m_strName       = src.m_strName;
  m_fTemperature  = src.m_fTemperature;
  m_fVoltage      = src.m_fVoltage;
  LaeAlarms::copyAlarms(src.m_alarms, m_alarms);
}

LaeRptMotorCtlrHealth &LaeRptMotorCtlrHealth::operator=(
                                              const LaeRptMotorCtlrHealth &rhs)
{
  m_strName       = rhs.m_strName;
  m_fTemperature  = rhs.m_fTemperature;
  m_fVoltage      = rhs.m_fVoltage;
  LaeAlarms::copyAlarms(rhs.m_alarms, m_alarms);

  return *this;
}


// -----------------------------------------------------------------------------
// Class LaeRptMotorHealth
// -----------------------------------------------------------------------------

LaeRptMotorHealth::LaeRptMotorHealth()
{
  m_fTemperature  = 0.0;
  m_fVoltage      = 0.0;
  m_fCurrent      = 0.0;
  LaeAlarms::clearAlarms(m_alarms);
}

LaeRptMotorHealth::LaeRptMotorHealth(const LaeRptMotorHealth &src)
{
  m_strName       = src.m_strName;
  m_fTemperature  = src.m_fTemperature;
  m_fVoltage      = src.m_fVoltage;
  m_fCurrent      = src.m_fCurrent;
  LaeAlarms::copyAlarms(src.m_alarms, m_alarms);
}

LaeRptMotorHealth &LaeRptMotorHealth::operator=(const LaeRptMotorHealth &rhs)
{
  m_strName       = rhs.m_strName;
  m_fTemperature  = rhs.m_fTemperature;
  m_fVoltage      = rhs.m_fVoltage;
  m_fCurrent      = rhs.m_fCurrent;
  LaeAlarms::copyAlarms(rhs.m_alarms, m_alarms);

  return *this;
}


// -----------------------------------------------------------------------------
// Class LaeRptRobotStatus
// -----------------------------------------------------------------------------

LaeRptRobotStatus::LaeRptRobotStatus(const LaeRptRobotStatus &src)
{
  // standard ROS industrial
  m_eRobotMode        = src.m_eRobotMode;
  m_eIsEStopped       = src.m_eIsEStopped;
  m_eAreDrivesPowered = src.m_eAreDrivesPowered;
  m_eIsMotionPossible = src.m_eIsMotionPossible;
  m_eIsInMotion       = src.m_eIsInMotion;
  m_eIsInError        = src.m_eIsInError;
  m_nErrorCode        = src.m_nErrorCode;

  // robot base extensions
  m_fBatterySoC       = src.m_fBatterySoC;
  m_bIsCharging       = src.m_bIsCharging;
  m_fCurrent          = src.m_fCurrent;
  m_fVoltage          = src.m_fVoltage;
  m_fTemperature      = src.m_fTemperature;
  m_eAuxBattEn        = src.m_eAuxBattEn;
  m_eAux5VEn          = src.m_eAux5VEn;
  LaeAlarms::copyAlarms(src.m_alarms, m_alarms);

  // motor extensions
  m_vecCtlrHealth     = src.m_vecCtlrHealth;
  m_vecMotorHealth    = src.m_vecMotorHealth;
}

LaeRptRobotStatus &LaeRptRobotStatus::operator=(const LaeRptRobotStatus &rhs)
{
  // standard ROS industrial
  m_eRobotMode        = rhs.m_eRobotMode;
  m_eIsEStopped       = rhs.m_eIsEStopped;
  m_eAreDrivesPowered = rhs.m_eAreDrivesPowered;
  m_eIsMotionPossible = rhs.m_eIsMotionPossible;
  m_eIsInMotion       = rhs.m_eIsInMotion;
  m_eIsInError        = rhs.m_eIsInError;
  m_nErrorCode        = rhs.m_nErrorCode;

  // robot base extensions
  m_fBatterySoC       = rhs.m_fBatterySoC;
  m_bIsCharging       = rhs.m_bIsCharging;
  m_fCurrent          = rhs.m_fCurrent;
  m_fVoltage          = rhs.m_fVoltage;
  m_fTemperature      = rhs.m_fTemperature;
  m_eAuxBattEn        = rhs.m_eAuxBattEn;
  m_eAux5VEn          = rhs.m_eAux5VEn;
  LaeAlarms::copyAlarms(rhs.m_alarms, m_alarms);

  // motor extensions
  m_vecCtlrHealth     = rhs.m_vecCtlrHealth;
  m_vecMotorHealth    = rhs.m_vecMotorHealth;

  return *this;
}

void LaeRptRobotStatus::clear()
{
  // standard ROS industrial
  m_eRobotMode        = LaeRobotModeUnknown;
  m_eIsEStopped       = LaeTriStateUnknown;
  m_eAreDrivesPowered = LaeTriStateUnknown;
  m_eIsMotionPossible = LaeTriStateUnknown;
  m_eIsInMotion       = LaeTriStateUnknown;
  m_eIsInError        = LaeTriStateUnknown;
  m_nErrorCode        = LAE_OK;

  // robot base extensions
  m_fBatterySoC       = 0.0;
  m_bIsCharging       = false;
  m_fCurrent          = 0.0;
  m_fVoltage          = 0.0;
  m_fTemperature      = 0.0;
  m_eAuxBattEn        = LaeTriStateUnknown;
  m_eAux5VEn          = LaeTriStateUnknown;
  LaeAlarms::clearAlarms(m_alarms);

  // motor extensions
  m_vecCtlrHealth.clear();
  m_vecMotorHealth.clear();
}

void LaeRptRobotStatus::generate(LaeRobot *pRobot)
{
  // ---
  // ROS Industrial
  // ---
  m_eRobotMode    = RtDb.m_robotstatus.m_eRobotMode;
  m_eIsEStopped   = RtDb.m_robotstatus.m_bIsEStopped?
                                            LaeTriStateTrue: LaeTriStateFalse;
  m_eAreDrivesPowered = RtDb.m_robotstatus.m_bAreMotorsPowered? 
                                            LaeTriStateTrue: LaeTriStateFalse;
  m_eIsInMotion = RtDb.m_robotstatus.m_bInMotion?
                                            LaeTriStateTrue: LaeTriStateFalse;

  // alarmed state
  if( RtDb.m_robotstatus.m_bAlarmState )
  {
    m_eIsInError = LaeTriStateTrue;
    m_nErrorCode = m_eIsEStopped == LaeTriStateTrue?
                                        -LAE_ECODE_ESTOP: -LAE_ECODE_ALARMED;
  }

  // unalarmed state
  else
  {
    m_eIsInError = LaeTriStateFalse;
    m_nErrorCode = LAE_OK;
  }

  m_eIsMotionPossible = pRobot->canMove()? LaeTriStateTrue: LaeTriStateFalse;

  // ---
  // Robot extended status
  // ---
  m_fBatterySoC   = RtDb.m_energy.m_fBatterySoC;
  m_bIsCharging   = RtDb.m_energy.m_bBatteryIsCharging;
  m_fCurrent      = RtDb.m_energy.m_fTotalCurrent;
  m_fVoltage      = RtDb.m_energy.m_fBatteryVoltage;

  m_fTemperature  = RtDb.m_robotstatus.m_fTempAvg;
  m_eAuxBattEn    = RtDb.m_enable.m_bAuxPortBatt?
                                      LaeTriStateFalse: LaeTriStateTrue;
  m_eAux5VEn      = RtDb.m_enable.m_bAuxPort5v?
                                      LaeTriStateFalse: LaeTriStateTrue;

  LaeAlarms::copyAlarms(RtDb.m_alarms.m_system, m_alarms);

  LaePlatform &base = pRobot->m_kin.getPlatform();

  //
  // Motor controller health extended status
  //
  for(int nCtlr = 0; nCtlr < LaeNumMotorCtlrs; ++nCtlr)
  {
    LaeRptMotorCtlrHealth  ctlrHealth;

    ctlrHealth.m_strName      = base.m_ctlr[nCtlr].m_strName;
    ctlrHealth.m_fTemperature = base.m_ctlr[nCtlr].m_fBoardTemp;
    ctlrHealth.m_fVoltage     = base.m_ctlr[nCtlr].m_fMainVolts;
    LaeAlarms::copyAlarms(RtDb.m_alarms.m_motorctlr[nCtlr],
                          ctlrHealth.m_alarms);

    m_vecCtlrHealth.push_back(ctlrHealth);
  }

  //
  // Motor health extended status
  //
  LaeMapPowertrain::iterator iter;       // kinematic chain iterator
  LaeMapPowertrain &mapPowertrains = pRobot->m_kin.getPowertrainMap();

  for(iter = mapPowertrains.begin(); iter != mapPowertrains.end(); ++iter)
  {
    LaeRptMotorHealth  motorHealth;
    LaePowertrain  &train = iter->second;
    int nMotorId = train.m_attr.m_nMotorId;

    motorHealth.m_strName      = train.m_strName;
    motorHealth.m_fTemperature = train.m_state.m_fTemp;
    motorHealth.m_fVoltage     = train.m_state.m_fVolts;
    LaeAlarms::copyAlarms(RtDb.m_alarms.m_motor[nMotorId],
                          motorHealth.m_alarms);

    m_vecMotorHealth.push_back(motorHealth);
  }
}


// -----------------------------------------------------------------------------
// Class LaeRptDynPowertrain
// -----------------------------------------------------------------------------


LaeRptDynPowertrain::LaeRptDynPowertrain()
{
  clear();
}

LaeRptDynPowertrain::LaeRptDynPowertrain(const LaeRptDynPowertrain &src)
{
  m_strName   = src.m_strName;
  m_nEncoder  = src.m_nEncoder;
  m_nSpeed    = src.m_nSpeed;
  m_fPosition = src.m_fPosition;
  m_fVelocity = src.m_fVelocity;
  m_fPe       = src.m_fPe;
  m_fTorque   = src.m_fTorque;
}


LaeRptDynPowertrain &LaeRptDynPowertrain::operator=(
                                                const LaeRptDynPowertrain &rhs)
{
  m_strName   = rhs.m_strName;
  m_nEncoder  = rhs.m_nEncoder;
  m_nSpeed    = rhs.m_nSpeed;
  m_fPosition = rhs.m_fPosition;
  m_fVelocity = rhs.m_fVelocity;
  m_fPe       = rhs.m_fPe;
  m_fTorque   = rhs.m_fTorque;

  return *this;
}

void LaeRptDynPowertrain::clear()
{
  m_strName.clear();
  m_nEncoder  = 0;
  m_nSpeed    = 0;
  m_fPosition = 0.0;
  m_fVelocity = 0.0;
  m_fPe       = 0.0;
  m_fTorque   = 0.0;
}

void LaeRptDynPowertrain::generate(int nMotorId)
{
  m_strName   = LaeDesc::KeyPowertrain[nMotorId];
  m_nEncoder  = RtDb.m_kin.m_powertrain[nMotorId].m_nEncoder;
  m_nSpeed    = RtDb.m_kin.m_powertrain[nMotorId].m_nSpeed;
  m_fPosition = RtDb.m_kin.m_powertrain[nMotorId].m_fPosition;
  m_fVelocity = RtDb.m_kin.m_powertrain[nMotorId].m_fVelocity;
  m_fPe       = RtDb.m_kin.m_powertrain[nMotorId].m_fPe;
  m_fTorque   = RtDb.m_kin.m_powertrain[nMotorId].m_fTorque;
}


// -----------------------------------------------------------------------------
// Class LaeRptDynamics
// -----------------------------------------------------------------------------

LaeRptDynamics::LaeRptDynamics()
{
  clear();
}

LaeRptDynamics::LaeRptDynamics(const LaeRptDynamics &src)
{
  m_pose          = src.m_pose;
  m_fOdometer     = src.m_fOdometer;
  m_fVelocity     = src.m_fVelocity;

  m_vecDynPowertrain = src.m_vecDynPowertrain;
}

LaeRptDynamics &LaeRptDynamics::operator=(const LaeRptDynamics &rhs)
{
  m_pose          = rhs.m_pose;
  m_fOdometer     = rhs.m_fOdometer;
  m_fVelocity     = rhs.m_fVelocity;
  m_vecDynPowertrain = rhs.m_vecDynPowertrain;

  return *this;
}

void LaeRptDynamics::clear()
{
  m_pose.clear();
  m_fOdometer = 0.0;
  m_fVelocity = 0.0;
  m_vecDynPowertrain.clear();
}

void LaeRptDynamics::generate(LaeRobot *pRobot)
{
  int                 nMotorId;
  LaeRptDynPowertrain pt;

  //
  // Platform dynamics
  //
  m_pose.m_x      = RtDb.m_kin.m_robot.m_x;
  m_pose.m_y      = RtDb.m_kin.m_robot.m_y;
  m_pose.m_theta  = RtDb.m_kin.m_robot.m_theta;
  m_fOdometer     = RtDb.m_kin.m_robot.m_fOdometer;
  m_fVelocity     = RtDb.m_kin.m_robot.m_fVelocity;
 
  //
  // Powertrain dynamics
  //
  for(nMotorId = 0; nMotorId < LaeMotorsNumOf; ++nMotorId)
  {
    pt.clear();
    pt.generate(nMotorId);
    m_vecDynPowertrain.push_back(pt);
  }
}
