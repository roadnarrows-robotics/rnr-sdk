////////////////////////////////////////////////////////////////////////////////
//
// Package:   Kuon
//
// Library:   libkuon
//
// File:      kuonRobot.cxx
//
/*! \file
 *
 * $LastChangedDate: 2014-04-09 15:58:07 -0600 (Wed, 09 Apr 2014) $
 * $Rev: 3638 $
 *
 * \brief Kuon Robot Class implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2013-2014  RoadNarrows
 * (http://www.RoadNarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
 * 
 * Permission is hereby granted, without written agreement and without
 * license or royalty fees, to use, copy, modify, and distribute this
 * software and its documentation for any purpose, provided that
 * (1) The above copyright notice and the following two paragraphs
 * appear in all copies of the source code and (2) redistributions
 * including binaries reproduces these notices in the supporting
 * documentation.   Substantial modifications to this software may be
 * copyrighted by their authors and need not follow the licensing terms
 * described here, provided that the new terms are clearly indicated in
 * all files where they apply.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
 * OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>

#include <string>
#include <vector>
#include <map>

#include "rnr/rnrconfig.h"
#include "rnr/units.h"
#include "rnr/log.h"

#include "Kuon/RS160DControl.h"

#include "Kuon/kuon.h"
#include "Kuon/kuonUtils.h"
#include "Kuon/kuonSpec.h"
#include "Kuon/kuonDesc.h"
#include "Kuon/kuonJoint.h"
#include "Kuon/kuonTraj.h"
#include "Kuon/kuonStatus.h"
#include "Kuon/kuonRobot.h"

using namespace std;
using namespace kuon;

/*!
 * \brief Test for no execute flag.
 *
 * Only works in KuonRobot methods.
 *
 * \return On true, return with KUON_OK.
 */
#define KUON_TRY_NO_EXEC() \
  do \
  { \
    if( m_bNoExec ) \
    { \
      return KUON_OK; \
    } \
  } while(0)

/*!
 * \brief Test for connection.
 *
 * Only works in KuonRobot methods.
 *
 * \return On failure, forces return from calling function with the appropriate
 * error code.
 */
#define KUON_TRY_CONN() \
  do \
  { \
    if( !isConnected() ) \
    { \
      LOGERROR("Robot is not connected."); \
      return -KUON_ECODE_NO_EXEC; \
    } \
  } while(0)

/*!
 * \brief Test for not estop.
 *
 * Only works in KuonRobot methods.
 *
 * \return On failure, forces return from calling function with the appropriate
 * error code.
 */
#define KUON_TRY_NOT_ESTOP() \
  do \
  { \
    if( m_bIsEStopped ) \
    { \
      LOGERROR("Robot is emergency stopped."); \
      return -KUON_ECODE_NO_EXEC; \
    } \
  } while(0)


// -----------------------------------------------------------------------------
// Class KuonRobot
// -----------------------------------------------------------------------------

const float KuonRobot::GovernorDft  = 0.20;
const float KuonRobot::BrakeDft     = 0.20;
const float KuonRobot::SlewDft      = 0.10;

KuonRobot::KuonRobot(bool bNoExec)
{
  // state
  m_bNoExec           = bNoExec;
  m_eRobotMode        = KuonRobotModeAuto;
  m_bIsEStopped       = false;
  m_bAlarmState       = false;
  m_fBattery          = 0.0;
  m_fBrake            = BrakeDft;
  m_fSlew             = SlewDft;
  m_nSetPtSpeedLeft   = 0;
  m_nSetPtSpeedRight  = 0;

  setGovernor(GovernorDft);

  // motor controllers i/f
  m_fdMotorCtlr0      = -1;   // front or rear (cannot tell with RS160D ctlrs)
  m_fdMotorCtlr1      = -1;   // front or rear (cannot tell with RS160D ctlrs)

  // asynchronous task control
  m_eAsyncTaskState   = KuonAsyncTaskStateIdle;
  m_rcAsyncTask       = KUON_OK;
  m_eAsyncTaskId      = AsyncTaskIdNone;
  m_pAsyncTaskArg     = NULL;
}

KuonRobot::~KuonRobot()
{
  disconnect();
}

int KuonRobot::connect(const std::string &strDevMotorCtlr0,
                       const std::string &strDevMotorCtlr1,
                       int                nBaudRateMotorCtlr)
{
  string  strDevName0;  // real device name 0
  string  strDevName1;  // real device name 1
  int     rc;           // return code

  //
  // Need a robot description before preceeding.
  //
  if( !m_descKuon.isDescribed() )
  {
    LOGERROR("Undefined Kuon description - "
             "don't know how to initialized properly.");
    return -KUON_ECODE_BAD_OP;
  }

  // get the real device names, not any symbolic links
  strDevName0 = getRealDeviceName(strDevMotorCtlr0);
  strDevName1 = getRealDeviceName(strDevMotorCtlr1);

  //
  // Open motor controller 0.
  //
  if( RS160DOpenConnection(strDevName0.c_str(), &m_fdMotorCtlr0) < 0 )
  {
    LOGERROR("%s: Failed to open motor controller 0.", strDevName0.c_str());
    rc = -KUON_ECODE_MOT_CTLR;
  }

  //
  // Configure motor controller 0.
  //
  else if( RS160DSetToSerial(m_fdMotorCtlr0) < 0 )
  {
    LOGERROR("%s: Failed to configure motor controller 0.",
        strDevName0.c_str());
    rc = -KUON_ECODE_MOT_CTLR;
  }

  //
  // Open motor controller 1.
  //
  if( RS160DOpenConnection(strDevName1.c_str(), &m_fdMotorCtlr1) < 0 )
  {
    LOGERROR("%s: Failed to open motor controller 1.", strDevName1.c_str());
    rc = -KUON_ECODE_MOT_CTLR;
  }

  // 
  // Configure motor controller 1.
  //
  else if( RS160DSetToSerial(m_fdMotorCtlr1) < 0 )
  {
    LOGERROR("%s: Failed to configure motor controller 1.",
        strDevName1.c_str());
    rc = -KUON_ECODE_MOT_CTLR;
  }

  // 
  // Convert fixed specifications to operational parameters.
  //
  else if( (rc = convertSpecs()) < 0 )
  {
    LOGERROR("Failed to convert product specifications to "
             "operational parameters.");
  }

  //
  // Configure for operation.
  //
  else if( (rc = configForOperation()) < 0 )
  {
    LOGERROR("Failed to configure for operation.");
  }

  // success
  else
  {
    LOGDIAG1("Connected to Kuon with controllers = (%s, %s).",
      strDevMotorCtlr0.c_str(), strDevMotorCtlr1.c_str());

    rc = KUON_OK;
  }

  // undo
  if( rc < 0 )
  {
    disconnect();
  }

  return rc;
}

int KuonRobot::disconnect()
{
  bool  bWasConn = false;

  if( isConnected() )
  {
    bWasConn = true;
    RS160DEStop(m_fdMotorCtlr0, m_fdMotorCtlr1);
  }

  if( m_fdMotorCtlr0 >= 0 )
  {
    RS160DClose(m_fdMotorCtlr0);
    m_fdMotorCtlr0 = -1;
  }

  if( m_fdMotorCtlr1 >= 0 )
  {
    RS160DClose(m_fdMotorCtlr1);
    m_fdMotorCtlr1 = -1;
  }

  if( bWasConn )
  {
    LOGDIAG1("Disconnected from Kuon.");
  }

  return KUON_OK;
}

int KuonRobot::estop()
{
  KUON_TRY_NO_EXEC();
  KUON_TRY_CONN();

  RS160DEStop(m_fdMotorCtlr0, m_fdMotorCtlr1);

  setBrake(1.0);

  m_bIsEStopped       = true;
  m_bAlarmState       = true;

  m_lastTrajBase.clear();

  LOGDIAG3("Kuon emergency stopped.");

  return KUON_OK;
}

int KuonRobot::freeze()
{
  KUON_TRY_NO_EXEC();
  KUON_TRY_CONN();

  m_lastTrajBase.clear();

  // stop 
  setSpeed(0.0, 0.0);

  // set 'parking' brake
  setBrake(1.0);

  LOGDIAG3("Kuon frozen at current position.");

  return KUON_OK;
}

int KuonRobot::release()
{
  KUON_TRY_NO_EXEC();
  KUON_TRY_CONN();

  m_lastTrajBase.clear();

  // stop 
  setSpeed(0.0, 0.0);

  // put in 'neutral'
  setBrake(0.0);

  LOGDIAG3("Kuon servo drives released.");

  return KUON_OK;
}

int KuonRobot::clearAlarms()
{
  KUON_TRY_CONN();

  return KUON_OK;
}

int KuonRobot::setBrake(float fBrake)
{
  int   raw;

  KUON_TRY_CONN();
  KUON_TRY_NOT_ESTOP();

  fBrake = (float)fcap(fBrake, 0.0, 1.0);

  raw = RS160D_MOTOR_BRAKE_MAX * fBrake;
  raw = icap(raw, RS160D_MOTOR_BRAKE_MIN, RS160D_MOTOR_BRAKE_MAX);

  RS160DAlterBraking(raw, m_fdMotorCtlr0, RS160D_MOTOR_LEFT_ID);
  RS160DAlterBraking(raw, m_fdMotorCtlr0, RS160D_MOTOR_RIGHT_ID);
  RS160DAlterBraking(raw, m_fdMotorCtlr1, RS160D_MOTOR_LEFT_ID);
  RS160DAlterBraking(raw, m_fdMotorCtlr1, RS160D_MOTOR_RIGHT_ID);

  m_fBrake = fBrake;

  m_bAreMotorsPowered = m_fBrake > 0.0? true: false;

  LOGDIAG3("Brakes set to %3.1f (raw=%d).", m_fBrake, raw);

  return KUON_OK;
}

int KuonRobot::setSlew(float fSlew)
{
  int   raw;

  KUON_TRY_CONN();
  KUON_TRY_NOT_ESTOP();

  fSlew = (float)fcap(fSlew, 0.0, 1.0);

  raw = RS160D_MOTOR_SLEW_MAX * fSlew;
  raw = icap(raw, RS160D_MOTOR_SLEW_MIN, RS160D_MOTOR_SLEW_MAX);

  RS160DAlterSlew(raw, m_fdMotorCtlr0, RS160D_MOTOR_LEFT_ID);
  RS160DAlterSlew(raw, m_fdMotorCtlr0, RS160D_MOTOR_RIGHT_ID);
  RS160DAlterSlew(raw, m_fdMotorCtlr1, RS160D_MOTOR_LEFT_ID);
  RS160DAlterSlew(raw, m_fdMotorCtlr1, RS160D_MOTOR_RIGHT_ID);

  m_fSlew = fSlew;

  LOGDIAG3("Power slew set to %3.1f (raw=%d).", m_fSlew, raw);

  return KUON_OK;
}

int KuonRobot::setSpeed(double fSpeedLeft, double fSpeedRight, units_t units)
{
  int   rawLeft;
  int   rawRight;

  KUON_TRY_CONN();
  KUON_TRY_NOT_ESTOP();

  rawLeft  = velocityToRawSpeed(fSpeedLeft,  units);
  rawRight = velocityToRawSpeed(fSpeedRight, units);

  rawLeft  = icap(rawLeft, m_nGovernSpeedMin, m_nGovernSpeedMax);
  rawRight = icap(rawRight, m_nGovernSpeedMin, m_nGovernSpeedMax);

  RS160DUpdateMotorSpeeds(-rawLeft, m_fdMotorCtlr0, RS160D_MOTOR_LEFT_ID);
  RS160DUpdateMotorSpeeds(rawRight, m_fdMotorCtlr0, RS160D_MOTOR_RIGHT_ID);
  RS160DUpdateMotorSpeeds(-rawLeft, m_fdMotorCtlr1, RS160D_MOTOR_LEFT_ID);
  RS160DUpdateMotorSpeeds(rawRight, m_fdMotorCtlr1, RS160D_MOTOR_RIGHT_ID);

  m_nSetPtSpeedLeft   = rawLeft;
  m_nSetPtSpeedRight  = rawRight;

  LOGDIAG3("Speed raw_left=%d, raw_right=%d.", rawLeft, rawRight);

  return KUON_OK;
}

int KuonRobot::move(KuonWheelTrajectoryPoint &trajectoryPoint)
{
  // TBD
  return KUON_OK;
}

int KuonRobot::velocityToRawSpeed(double fVelocity, units_t units)
{
  switch(units)
  {
    // [-1.0, 1.0]  of max servo speed
    case units_norm:
      return (int)(RS160D_MOTOR_SPEED_MAX * fcap(fVelocity, -1.0, 1.0));

    // % of max motor speed
    case units_percent:
      return (int)(RS160D_MOTOR_SPEED_MAX * 
                                        fcap(fVelocity, -100.0, 100.0)/100.0);

    // %% of max motor speed
    case units_permil:
      return (int)(RS160D_MOTOR_SPEED_MAX * 
                                      fcap(fVelocity, -1000.0, 1000.0)/1000.0);

    // raw units
    case units_raw:
      return (int)fcap(fVelocity,
                            RS160D_MOTOR_SPEED_MIN, RS160D_MOTOR_SPEED_MAX);

    // future
    case units_rad_per_s:
    case units_m_per_s:
      LOGWARN("%s velocity units not supported until encoders installed.",
          units_shortname(units));
      return 0;

    // bad units
    default:
      LOGWARN("%s velocity units not supported.", units_shortname(units));
      return 0;
  }
}

void KuonRobot::getVelocitySetPoints(double &fSpeedLeft,
                                     double &fSpeedRight,
                                     units_t units)
{
  switch(units)
  {
    // [-1.0, 1.0]  of max servo speed
    case units_norm:
      fSpeedLeft  = (double)m_nSetPtSpeedLeft  / (double)RS160D_MOTOR_SPEED_MAX;
      fSpeedRight = (double)m_nSetPtSpeedRight / (double)RS160D_MOTOR_SPEED_MAX;
      break;

    // % of max motor speed
    case units_percent:
      fSpeedLeft  = (double)m_nSetPtSpeedLeft  / (double)RS160D_MOTOR_SPEED_MAX;
      fSpeedRight = (double)m_nSetPtSpeedRight / (double)RS160D_MOTOR_SPEED_MAX;
      fSpeedLeft  *= 100.0;
      fSpeedRight *= 100.0;
      break;

    // %% of max motor speed
    case units_permil:
      fSpeedLeft  = (double)m_nSetPtSpeedLeft  / (double)RS160D_MOTOR_SPEED_MAX;
      fSpeedRight = (double)m_nSetPtSpeedRight / (double)RS160D_MOTOR_SPEED_MAX;
      fSpeedLeft  *= 1000.0;
      fSpeedRight *= 1000.0;
      break;

    // raw
    case units_raw:
    default:
      fSpeedLeft  = (double)m_nSetPtSpeedLeft;
      fSpeedRight = (double)m_nSetPtSpeedRight;
      break;
  }
}

int KuonRobot::getRobotStatus(KuonRobotStatus &robotStatus)
{
  MapRobotJoints::iterator  iter;       // kinematic chain iterator
  int                       nMotorId;   // motor id
  bool                      bIsMoving;  // robot is [not] moving
  KuonMotorHealth           health;     // motor health

  robotStatus.clear();

  robotStatus.m_eRobotMode        = m_eRobotMode;
  robotStatus.m_eIsEStopped       = m_bIsEStopped?
                                            KuonTriStateTrue: KuonTriStateFalse;
  robotStatus.m_eAreDrivesPowered = m_bAreMotorsPowered? 
                                            KuonTriStateTrue: KuonTriStateFalse;
  robotStatus.m_eIsInMotion = isInMotion()?
                                            KuonTriStateTrue: KuonTriStateFalse;

  // alarmed statue
  if( m_bAlarmState )
  {
    robotStatus.m_eIsMotionPossible = KuonTriStateFalse;
    robotStatus.m_eIsInError = KuonTriStateTrue;
    robotStatus.m_nErrorCode = m_bIsEStopped?
                                      -KUON_ECODE_ESTOP: -KUON_ECODE_ALARMED;
  }

  // unalarmed state
  else
  {
    robotStatus.m_eIsMotionPossible = m_bNoExec? 
                                            KuonTriStateFalse: KuonTriStateTrue;
    robotStatus.m_eIsInError = KuonTriStateFalse;
    robotStatus.m_nErrorCode = KUON_OK;
  }

  robotStatus.m_fGovernor = m_fGovernor;
  robotStatus.m_fBattery  = 0.0;


  for(iter = m_kinBase.begin(); iter != m_kinBase.end(); ++iter)
  {
    health.m_strName      = iter->second.m_strName;
    health.m_nMotorId     = iter->first;
    health.m_fTemperature = 0.0;
    health.m_fVoltage     = 0.0;
    health.m_uAlarms      = 0;

    robotStatus.m_vecMotorHealth.push_back(health);
  }

  return KUON_OK;
}

int KuonRobot::getJointState(KuonJointStatePoint &jointStatePoint)
{
  MapRobotJoints::iterator  iter;       // kinematic chain iterator
  int                       nMotorId;   // master servo id
  KuonRobotJoint           *pMotor;     // robotic motor joint
  KuonJointState            jointState; // working joint state
  byte_t                    uMask;      // working bit mask
  int                       i;          // working index

  KUON_TRY_CONN();

  jointStatePoint.clear();

  jointStatePoint.setKinematicChainName("base");

  //
  // Build joint state point.
  //
  for(iter = m_kinBase.begin(); iter != m_kinBase.end(); ++iter)
  {
    nMotorId  = iter->first;
    pMotor    = &(iter->second);

    // identifiers
    jointState.m_strName  = pMotor->m_strName;
    jointState.m_nMotorId = nMotorId;

    // positions (unknown without encoders)
    jointState.m_fPosition = 0.0;
    jointState.m_fOdometer = 0.0;
    jointState.m_nEncoder  = 0;

    // velocities
    jointState.m_fVelocity    = 0.0;
    jointState.m_fVelocityMps = 0.0;
    if( pMotor->m_nMotorIndex == RS160D_MOTOR_LEFT_ID )
    {
      jointState.m_nSpeed = m_nSetPtSpeedLeft;
    }
    else
    {
      jointState.m_nSpeed = m_nSetPtSpeedRight;
    }

    // torques and powers
    jointState.m_fEffort  = 0.0;
    jointState.m_fPe      = 0.0;
    jointState.m_fPm      = 0.0;

    // braking and power-up slewing
    jointState.m_fBrake   = m_fBrake;
    jointState.m_fSlew    = m_fSlew;

    // add state
    jointStatePoint.append(jointState);
  }

  return KUON_OK;
}

int KuonRobot::getTrajectoryState(KuonWheelTrajectoryFeedback &trajFeedback)
{
  // TBD
 
  trajFeedback.clear();

  return KUON_OK;
}

int KuonRobot::convertSpecs()
{
  KuonDescBase      *pDescBase;
  KuonSpecMotor_T   *pSpecMotor;
  KuonRobotJoint    *pMotor;
  int               i;
  int               rc;

  m_kinBase.clear();
  m_imapBase.clear();

  pDescBase = m_descKuon.getBaseDesc();

  //
  // Build up base kinematic chain.
  //
  for(i = 0; i < pDescBase->getNumMotors(); ++i)
  {
    // joint specification
    pSpecMotor = pDescBase->m_spec.getMotorSpecAt(i);

    // add robotic 'joint'
    if( (rc = addRobotJoint(pSpecMotor, m_kinBase, m_imapBase)) < 0 )
    {
      LOGERROR("Motor %d: Cannot add to kinematic chain.",
          pSpecMotor->m_nMotorId);
      return rc;
    }
  }
 
  return KUON_OK;
}

int KuonRobot::addRobotJoint(KuonSpecMotor_T *pSpecMotor,
                             MapRobotJoints  &kin,
                             IMapRobotJoints &imap)
{
  int               nMotorId;     // master servo id
  KuonRobotJoint    joint;        // robotic joint

  // master servo id associated with joint
  nMotorId = pSpecMotor->m_nMotorId;

  //
  // Populate joint data.
  //
  joint.m_strName             = pSpecMotor->m_strName;
  joint.m_nMotorId            = nMotorId;
  joint.m_nMotorCtlrId        = pSpecMotor->m_nMotorCtlrId;
  joint.m_nMotorIndex         = pSpecMotor->m_nMotorIndex;
  joint.m_nMotorDir           = pSpecMotor->m_nDir;
  joint.m_eJointType          = KuonJointTypeContinuous;
  joint.m_fGearRatio          = pSpecMotor->m_fGearRatio;
  joint.m_fTireRadius         = pSpecMotor->m_fTireRadius;
  joint.m_fTicksPerMotorRad   = 0.0;
  joint.m_fTicksPerWheelRad   = 0.0;

  //
  // Sanity checks.
  //

  //
  // Add to kinematic chain.
  //
  kin[nMotorId]         = joint;      // kinematic chain
  imap[joint.m_strName] = nMotorId;   // indirect map by joint name

  return KUON_OK;
}

int KuonRobot::configForOperation()
{
  m_eRobotMode        = KuonRobotModeAuto;
  m_bIsEStopped       = false;
  m_bAlarmState       = false;
  m_fBattery          = 0.0;
  m_fBrake            = BrakeDft;
  m_fSlew             = SlewDft;
  m_nSetPtSpeedLeft   = 0;
  m_nSetPtSpeedRight  = 0;

  setGovernor(GovernorDft);
  setBrake(BrakeDft);
  setSlew(SlewDft);
  setSpeed(0.0, 0.0);

  return KUON_OK;
}

int KuonRobot::createAsyncThread()
{
  int   rc;

  m_eAsyncTaskState = KuonAsyncTaskStateWorking;

  rc = pthread_create(&m_threadAsync, NULL, KuonRobot::asyncThread, (void*)this);
 
  if( rc == 0 )
  {
    rc = KUON_OK;
  }

  else
  {
    m_eAsyncTaskState = KuonAsyncTaskStateIdle;
    LOGSYSERROR("pthread_create()");
    m_rcAsyncTask   = -KUON_ECODE_SYS;
    m_eAsyncTaskId  = AsyncTaskIdNone;
    m_pAsyncTaskArg = NULL;
    rc = m_rcAsyncTask;
  }

  return rc;
}


void KuonRobot::cancelAsyncTask()
{
  MapRobotJoints::iterator  iter;

  if( m_eAsyncTaskState != KuonAsyncTaskStateIdle )
  {
    // cancel thread
    pthread_cancel(m_threadAsync);
    pthread_join(m_threadAsync, NULL);

    // cleanup
    switch( m_eAsyncTaskId )
    {
      default:
        break;
    }

    // clear state
    m_eAsyncTaskId    = AsyncTaskIdNone;
    m_pAsyncTaskArg   = NULL;
    m_rcAsyncTask     = -KUON_ECODE_INTR;
    m_eAsyncTaskState = KuonAsyncTaskStateIdle;
    LOGDIAG3("Async task canceled.");
  }
}

void *KuonRobot::asyncThread(void *pArg)
{
  KuonRobot *pThis = (KuonRobot *)pArg;
  int       oldstate;
  int       rc;

  pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, &oldstate);
  pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, &oldstate);
  //pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, &oldstate);

  LOGDIAG3("Async robot task thread created.");

  //
  // Execute asychronous task.
  //
  // For now, only calibrate task is supported asynchronously.
  //
  switch( pThis->m_eAsyncTaskId )
  {
    // Unknown task id.
    default:
      LOGERROR("Unknown async task id = %d.", (int)pThis->m_eAsyncTaskId);
      rc = -KUON_ECODE_BAD_VAL;
      break;
  }

  // freeze robot at current calibrated or aborted position.
  //pThis->freeze();  disable, so that goto zero pt in calibration finsishes

  pThis->m_eAsyncTaskId     = AsyncTaskIdNone;
  pThis->m_pAsyncTaskArg    = NULL;
  pThis->m_rcAsyncTask      = rc;
  pThis->m_eAsyncTaskState  = KuonAsyncTaskStateIdle;

  LOGDIAG3("Async robot task thread exited.");

  return NULL;
}
