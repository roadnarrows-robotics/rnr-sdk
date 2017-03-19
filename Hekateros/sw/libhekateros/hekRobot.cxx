////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekRobot.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-04-17 15:31:34 -0600 (Fri, 17 Apr 2015) $
 * $Rev: 3942 $
 *
 * \brief HekRobot - Hekateros Robot Class implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2011-2017. RoadNarrows LLC.\n
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
#include <unistd.h>
#include <pthread.h>
#include <math.h>

#include <string>
#include <vector>
#include <map>

#include "rnr/rnrconfig.h"
#include "rnr/units.h"
#include "rnr/log.h"
#include "rnr/i2c.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaServo.h"
#include "Dynamixel/DynaChain.h"
#include "Dynamixel/DynaBgThread.h"
#include "Dynamixel/DynaError.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekTune.h"
#include "Hekateros/hekUtils.h"
#include "Hekateros/hekOptical.h"
#include "Hekateros/hekMonitor.h"
#include "Hekateros/hekSpec.h"
#include "Hekateros/hekDesc.h"
#include "Hekateros/hekXmlTune.h"
#include "Hekateros/hekJoint.h"
#include "Hekateros/hekTraj.h"
#include "Hekateros/hekKin.h"
#include "Hekateros/hekState.h"
#include "Hekateros/hekRobot.h"

using namespace std;
using namespace hekateros;

/*!
 * \brief Define if heketeros has RN system board.
 */
#undef HEK_SYS_BOARD

/*!
 * \brief Test for no execute flag.
 *
 * Only works in HekRobot methods.
 *
 * \return On true, return with HEK_OK.
 */
#define HEK_TRY_NO_EXEC() \
  do \
  { \
    if( m_bNoExec ) \
    { \
      return HEK_OK; \
    } \
  } while(0)

/*!
 * \brief Convenience macro for trying to get a servo object from dynamixel
 * chain.
 *
 * Failure is considered a software bug since the chain has already be 
 * verified.
 *
 * Only works in HekRobot methods.
 *
 * \param [in] nServoId   Servo id.
 * \param [out] pServo    Pointer to associated servo object.
 *
 * \return On failure, forces return from calling function with the appropriate
 * error code.
 */
#define HEK_TRY_GET_SERVO(nServoId, pServo) \
  do \
  { \
    if( (pServo = m_pDynaChain->GetServo(nServoId)) == NULL ) \
    { \
      LOGERROR("BUG: Servo %d: Cannot find in dynamixel chain.", nServoId); \
      return -HEK_ECODE_INTERNAL; \
    } \
  } while(0)


/*!
 * \brief Test for connection.
 *
 * Only works in HekRobot methods.
 *
 * \return On failure, forces return from calling function with the appropriate
 * error code.
 */
#define HEK_TRY_CONN() \
  do \
  { \
    if( !isConnected() ) \
    { \
      LOGERROR("Robot is not connected."); \
      return -HEK_ECODE_NO_EXEC; \
    } \
  } while(0)

/*!
 * \brief Test for calibration.
 *
 * Only works in HekRobot methods.
 *
 * \return On failure, forces return from calling function with the appropriate
 * error code.
 */
#define HEK_TRY_CALIB() \
  do \
  { \
    if( m_eOpState != HekOpStateCalibrated ) \
    { \
      LOGERROR("Robot is not calibrated."); \
      return -HEK_ECODE_NO_EXEC; \
    } \
  } while(0)

/*!
 * \brief Test for not estop.
 *
 * Only works in HekRobot methods.
 *
 * \return On failure, forces return from calling function with the appropriate
 * error code.
 */
#define HEK_TRY_NOT_ESTOP() \
  do \
  { \
    if( m_bIsEStopped ) \
    { \
      LOGERROR("Robot is emergency stopped."); \
      return -HEK_ECODE_NO_EXEC; \
    } \
  } while(0)


// -----------------------------------------------------------------------------
// Class HekRobot
// -----------------------------------------------------------------------------

HekRobot::HekRobot(bool bNoExec)
{
  // state
  m_bNoExec           = bNoExec;
  m_eRobotMode        = HekRobotModeAuto;
  m_eOpState          = HekOpStateUncalibrated;
  m_bIsEStopped       = false;
  m_bAlarmState       = false;
  m_bAreServosPowered = false;
  m_bAtBalancedPos    = false;
  m_bAtParkedPos      = false;

  // dynamixel i/f
  m_pDynaComm         = NULL;
  m_pDynaChain        = NULL;
  m_pDynaBgThread     = NULL;

  // asynchronous task control and synchronization
  m_eAsyncTaskState   = HekAsyncTaskStateIdle;
  m_rcAsyncTask       = HEK_OK;
  m_eAsyncTaskId      = AsyncTaskIdNone;
  m_pAsyncTaskArg     = NULL;

  pthread_mutex_init(&m_mutex, NULL);
}

HekRobot::~HekRobot()
{
  disconnect();
  pthread_mutex_destroy(&m_mutex);
}

int HekRobot::connect(const string &strDevDynabus, int nBaudRateDynabus,
                      const string &strDevArduino, int nBaudRateArduino)
{
  string  strDevName; // real device name
  int     rc;         // return code

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Pre-connect requirements.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  //
  // Need a robot description before preceeding. The controlling application, 
  // typically a ROS node, provides the description. The desription is normally
  // the parsed data found in /etc/hekaeros/hekateros.conf.
  //
  if( !m_descHek.isDescribed() )
  {
    LOGERROR("Undefined Hekateros description - "
             "don't know how to initialized properly.");
    return -HEK_ECODE_BAD_OP;
  }
  

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Dynabus communication and configuration.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  // get the real device name, not a symbolic link
  strDevName = getRealDeviceName(strDevDynabus);

  // open (proxied) serial device to dynamixel bus
  m_pDynaComm = DynaComm::New(strDevName.c_str(), nBaudRateDynabus);

  if( m_pDynaComm == NULL )
  {
    LOGERROR("Failed to create dynamixel interface on '%s'@%d.",
        strDevName.c_str(), nBaudRateDynabus);
    return -HEK_ECODE_DYNA;
  }
  LOGDIAG2("Created dynamixel interface on '%s'@%d.",
        strDevName.c_str(), nBaudRateDynabus);

  // create dynamixel bus chain
  m_pDynaChain = new DynaChain(*m_pDynaComm);

  // scan Dynabus for all attached servos
  if( (rc = scanDynaBus(3)) < 0 )
  {
    LOGERROR("Hekateros dynamixel bus scan failed.");
    disconnect();
    return rc;
  }
  LOGDIAG2("Scanned for servos.");

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Build specifications, set tuning parameters, and configure.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  //
  // Convert fixed specifications to joint operation descriptions. The
  // specifications are compliled-in, product-specific link, joint, servo data.
  //
  // Required: product descriptions, scanned servos, and monitor object.
  //
  if( (rc = convertSpecs()) < 0 )
  {
    LOGERROR("Failed to convert product specifications to "
             "operations parameters.");
    disconnect();
    return rc;
  }
  LOGDIAG2("Converted product specifications to operations parameters.");

  //
  // Adjust tuning parameters from any defaults specified in the converted
  // joint descriptions.
  //
  // Required: joint descriptions and tuning paramaters.
  //
  adjustTuningFromSpecs();
  
  //
  // Override any tuning parameters from the optional, user-specified tuning
  // XML file.
  //
  HekXmlTune  xml;

  // parse tune XML file and set tuning parameter overrides
  xml.load(m_tunes, HekSysCfgPath, HekEtcTune);

  //
  // Configure servos EEPROM and RAM control tables for operation.
  //
  // Required: joint descriptions and scanned servos.
  //
  configServos();

  //
  // Mark all joints for calibration.
  //
  resetCalibStateForAllJoints(true);


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Kinodynmics thread.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
 
  // create active dynamics and kinematics chain
  m_pKin = new HekKinematics(*m_pDynaChain, m_jointsArm, m_tunes);

  // run kinematics thread at the given Hertz
  m_pKin->runThread(m_tunes.getKinematicsHz());

  LOGDIAG2("Kinematics thread started.");

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Health and safety monitoring thread.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  // get the real device name, not a symbolic link
  strDevName = getRealDeviceName(strDevArduino);

  // open all interfaces to monitoring hardware and create monitoring thread
  if( (rc = m_monitor.open(getVersionNum(), strDevName, nBaudRateArduino)) < 0 )
  {
    LOGERROR("Hekateros monitor interface failed to open on '%s'@%d.",
        strDevName.c_str(), nBaudRateArduino);
    disconnect();
    return rc;
  }
  LOGDIAG2("Hekateros monitor interface open on '%s'@%d.",
        strDevName.c_str(), nBaudRateArduino);

  m_bAlarmState = false;
  m_monitor.markAlarmCond(m_bAlarmState);

  // add chain to monitor 
  m_monitor.addServoChainToMonitor(m_pDynaChain);

  // add kinematics to monitor 
  m_monitor.addKinematicsToMonitor(m_pKin);

  // start robot monitoring health and safety thread
  m_monitor.start();

  LOGDIAG2("Monitor thread started.");


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Final touches
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  
  LOGDIAG1("Connected to Hekateros.");

  //
  // DBG DANGER, WILL ROBINSON
  //
  // Uncomment the next line to circumvent calibration step. Make sure arm
  // is at the zero point.
  // 
  //fauxcalibrate();
  //
  // DBG DANGER, WILL ROBINSON
  //

  return HEK_OK;
}

int HekRobot::disconnect()
{
  // close monitor 
  m_monitor.close();
  
  if( m_pKin != NULL )
  {
    delete m_pKin;
    m_pKin = NULL;
  }

  // delete dynamixel chain
  if( m_pDynaChain != NULL )
  {
    m_pDynaChain->EStop();  // == force release of arm motors
    delete m_pDynaChain;
    m_pDynaChain = NULL;
  }

  // close connection and delete dynamixel communication object
  if( m_pDynaComm != NULL )
  {
    if( m_pDynaComm->IsOpen() )
    {
      m_pDynaComm->Close();
    }
    delete m_pDynaComm;
    m_pDynaComm = NULL;
  }

  // reset robot state
  m_eRobotMode        = HekRobotModeAuto;
  m_eOpState          = HekOpStateUncalibrated;
  m_bIsEStopped       = false;
  m_bAlarmState       = false;
  m_bAreServosPowered = false;
  m_bAtBalancedPos    = false;
  m_bAtParkedPos      = false;

  LOGDIAG1("Disconnected from Hekateros.");
}

int HekRobot::calibrate(bool bForceRecalib)
{
  HekCalibStretch calib(*this);
  int             rc;

  HEK_TRY_NO_EXEC();
  HEK_TRY_CONN();

  // lock the arm
  freeze();

  // robot is now uncalibrated
  m_eOpState = HekOpStateUncalibrated;

  // mark relevant joints for recalibration
  resetCalibStateForAllJoints(bForceRecalib);

  // robot calibration started 
  m_eOpState = HekOpStateCalibrating;

  // now physically calibrate the arm
  if( (rc = calib.calibrate()) < 0 )
  {
    LOGERROR("Failed to calibrate Hekateros.");
    m_eOpState = HekOpStateUncalibrated;
    return rc;
  }

  //
  // Should be all calibrated at this point.
  //
  
  // synchronize robot operation state to collective joint operational states
  if( (m_eOpState = determineRobotOpState()) != HekOpStateCalibrated )
  {
    LOGERROR("BUG: Failed to calibrate Hekateros.");
    return -HEK_ECODE_INTERNAL;
  }

  // arm is calibrated
  gotoZeroPtPos();

  LOGDIAG1("Hekateros calibrated.");

  return HEK_OK;
}

int HekRobot::calibrateAsync(bool bForceRecalib)
{
  if( m_eAsyncTaskState != HekAsyncTaskStateIdle )
  {
    LOGERROR("Already executing asynchronous task.");
    return -HEK_ECODE_NO_RSRC;
  }
  else
  {
    m_eAsyncTaskId  = AsyncTaskIdCalibrate;
    m_pAsyncTaskArg = (void *)bForceRecalib;

    return createAsyncThread();
  }
}

void HekRobot::reload()
{
  HekXmlTune  xml;

  // parse user tune parameter overrides
  if( xml.load(m_tunes,HekSysCfgPath, HekEtcTune) == HEK_OK )
  {
    LOGDIAG2("User specified tuning parameters reloaded from XML file %s.",
            HekEtcTune);

    m_pKin->reload(m_tunes);
  }
}

int HekRobot::gotoBalancedPos()
{
  HekJointTrajectoryPoint   trajPoint;
  MapRobotJoints::iterator  iter;

  int             nMasterServoId;
  HekRobotJoint  *pJoint;
  int             rc;

  HEK_TRY_CONN();
  HEK_TRY_CALIB();
  HEK_TRY_NOT_ESTOP();

  //
  // Build trajoectory point.
  //
  for(iter = m_jointsArm.begin(); iter != m_jointsArm.end(); ++iter)
  {
    nMasterServoId  = iter->first;
    pJoint          = &(iter->second);

    switch( nMasterServoId )
    {
      case HekServoIdBase:
        trajPoint.append(pJoint->m_strName, pJoint->m_fBalPosRads,
            degToRad(60.0));
        break;
      case HekServoIdShoulderL:
        trajPoint.append(pJoint->m_strName, pJoint->m_fBalPosRads,
            degToRad(40.0));
        break;
      case HekServoIdElbow:
        trajPoint.append(pJoint->m_strName, pJoint->m_fBalPosRads,
            degToRad(35.0));
        break;
      case HekServoIdWristPitch:
        trajPoint.append(pJoint->m_strName, pJoint->m_fBalPosRads,
            degToRad(50.0));
        break;
      case HekServoIdWristRot:
        trajPoint.append(pJoint->m_strName, pJoint->m_fBalPosRads,
            degToRad(60.0));
        break;
      case HekServoIdGraboid:
        trajPoint.append(pJoint->m_strName, pJoint->m_fBalPosRads,
            degToRad(20.0));
        break;
      default:
        break;
    }
  }

  if( (rc = moveArm(trajPoint)) < 0 )
  {
    LOGERROR("Move to pre-defined balanced position failed.");
  }
  else
  {
    m_bAtBalancedPos = true;
    LOGDIAG2("Hekateros at balanced position.");
  }

  return rc;
}

int HekRobot::gotoParkedPos()
{
  HekJointTrajectoryPoint   trajPoint;
  MapRobotJoints::iterator  iter;

  int             nMasterServoId;
  HekRobotJoint  *pJoint;
  int             rc;

  HEK_TRY_CONN();
  HEK_TRY_CALIB();
  HEK_TRY_NOT_ESTOP();

  //
  // Build trajoectory point.
  //
  for(iter = m_jointsArm.begin(); iter != m_jointsArm.end(); ++iter)
  {
    nMasterServoId  = iter->first;
    pJoint          = &(iter->second);

    switch( nMasterServoId )
    {
      case HekServoIdBase:
        trajPoint.append(pJoint->m_strName, pJoint->m_fParkPosRads,
            degToRad(60.0));
        break;
      case HekServoIdShoulderL:
        trajPoint.append(pJoint->m_strName, pJoint->m_fParkPosRads,
            degToRad(40.0));
        break;
      case HekServoIdElbow:
        trajPoint.append(pJoint->m_strName, pJoint->m_fParkPosRads,
            degToRad(35.0));
        break;
      case HekServoIdWristPitch:
        trajPoint.append(pJoint->m_strName, pJoint->m_fParkPosRads,
            degToRad(50.0));
        break;
      case HekServoIdWristRot:
        trajPoint.append(pJoint->m_strName, pJoint->m_fParkPosRads,
            degToRad(60.0));
        break;
      case HekServoIdGraboid:
        trajPoint.append(pJoint->m_strName, pJoint->m_fParkPosRads,
            degToRad(20.0));
        break;
      default:
        break;
    }
  }

  if( (rc = moveArm(trajPoint)) < 0 )
  {
    LOGERROR("Move to pre-defined parked position failed.");
  }
  else
  {
    m_bAtParkedPos = true;
    LOGDIAG2("Hekateros at parked position.");
  }

  return rc;
}

int HekRobot::gotoZeroPtPos()
{
  HekJointTrajectoryPoint   trajPoint;
  MapRobotJoints::iterator  iter;

  int             nMasterServoId;
  HekRobotJoint  *pJoint;
  int             rc;

  HEK_TRY_CONN();
  HEK_TRY_CALIB();
  HEK_TRY_NOT_ESTOP();

  //
  // Build trajoectory point.
  //
  for(iter = m_jointsArm.begin(); iter != m_jointsArm.end(); ++iter)
  {
    nMasterServoId  = iter->first;
    pJoint          = &(iter->second);

    switch( nMasterServoId )
    {
      case HekServoIdBase:
        trajPoint.append(pJoint->m_strName, pJoint->m_fCalibPosRads,
            degToRad(60.0));
        break;
      case HekServoIdShoulderL:
        trajPoint.append(pJoint->m_strName, pJoint->m_fCalibPosRads,
            degToRad(30.0));
        break;
      case HekServoIdElbow:
        trajPoint.append(pJoint->m_strName, pJoint->m_fCalibPosRads,
            degToRad(25.0));
        break;
      case HekServoIdWristPitch:
        trajPoint.append(pJoint->m_strName, pJoint->m_fCalibPosRads,
            degToRad(50.0));
        break;
      case HekServoIdWristRot:
        trajPoint.append(pJoint->m_strName, pJoint->m_fCalibPosRads,
            degToRad(60.0));
        break;
      case HekServoIdGraboid:
        trajPoint.append(pJoint->m_strName, pJoint->m_fCalibPosRads,
            degToRad(20.0));
        break;
      default:
        break;
    }
  }

  if( (rc = moveArm(trajPoint)) < 0 )
  {
    LOGERROR("Move to pre-defined calibrated zero point position failed.");
  }
  else
  {
    LOGDIAG2("Hekateros at zero point position.");
  }

  return rc;
}

int HekRobot::openGripper()
{
  HekJointTrajectoryPoint   trajPoint;

  HekRobotJoint  *pJoint;
  int             rc;

  HEK_TRY_CONN();
  HEK_TRY_CALIB();
  HEK_TRY_NOT_ESTOP();

  if( (pJoint = getArmJoint(HekServoIdGraboid)) == NULL )
  {
    LOGERROR("No gripper end effector found.");
    return -HEK_ECODE_NO_RSRC;
  }

  trajPoint.append(pJoint->m_strName, pJoint->m_fMaxSoftLimitRads,
            degToRad(60.0));

  if( (rc = moveArm(trajPoint)) < 0 )
  {
    LOGERROR("Move to pre-defined gripper open position failed.");
  }
  else
  {
    LOGDIAG2("Hekateros gripper opened.");
  }

  return rc;
}

int HekRobot::closeGripper()
{
  HekJointTrajectoryPoint   trajPoint;

  HekRobotJoint  *pJoint;
  int             rc;

  HEK_TRY_CONN();
  HEK_TRY_CALIB();
  HEK_TRY_NOT_ESTOP();

  if( (pJoint = getArmJoint(HekServoIdGraboid)) == NULL )
  {
    LOGERROR("No gripper end effector found.");
    return -HEK_ECODE_NO_RSRC;
  }

  trajPoint.append(pJoint->m_strName, pJoint->m_fMinSoftLimitRads,
            degToRad(60.0));

  if( (rc = moveArm(trajPoint)) < 0 )
  {
    LOGERROR("Move to pre-defined gripper closed position failed.");
  }
  else
  {
    LOGDIAG2("Hekateros gripper closed.");
  }

  return rc;
}

int HekRobot::estop()
{
  HEK_TRY_NO_EXEC();
  HEK_TRY_CONN();

  m_pKin->estop();

  m_bIsEStopped       = true;
  m_bAreServosPowered = false;

  m_monitor.markEStopCond(m_bIsEStopped);
  m_monitor.markPoweredCond(m_bAreServosPowered);
 
  m_lastTrajArm.clear();

  m_bAlarmState = true;
  m_monitor.markAlarmCond(m_bAlarmState);

  LOGDIAG1("Hekateros emergency stopped.");

  return HEK_OK;
}

int HekRobot::freeze()
{
  HEK_TRY_NO_EXEC();
  HEK_TRY_CONN();

  m_bAreServosPowered = true;

  m_monitor.markPoweredCond(m_bAreServosPowered);

  m_lastTrajArm.clear();

  m_pKin->freeze();

  LOGDIAG2("Hekateros frozen at current position.");

  return HEK_OK;
}

int HekRobot::release()
{
  HEK_TRY_NO_EXEC();
  HEK_TRY_CONN();

  m_bAreServosPowered = false;

  m_monitor.markPoweredCond(m_bAreServosPowered);

  m_lastTrajArm.clear();

  m_pKin->release();

  LOGDIAG2("Hekateros servo drives released.");

  return HEK_OK;
}

int HekRobot::stop(const vector<string> &vecNames)
{
  int   n;

  HEK_TRY_NO_EXEC();
  HEK_TRY_CONN();
  HEK_TRY_NOT_ESTOP();

  m_lastTrajArm.clear();

  n = m_pKin->stop(vecNames);

  LOGDIAG3("%d joints stopped.", n);

  return n;
}

int HekRobot::clearAlarms()
{
  int             iter;       // servo iterator
  int             nServoId;   // servo id
  DynaServo      *pServo;     // servo

  HEK_TRY_CONN();

  release();

  //
  // Try to clear alarms for all servos.
  //
  for(nServoId = m_pDynaChain->IterStart(&iter);
      nServoId != DYNA_ID_NONE;
      nServoId = m_pDynaChain->IterNext(&iter))
  {
    pServo = m_pDynaChain->GetServo(nServoId);

    // over torque alarm zero's torque limit - reload from EEPROM
    pServo->ReloadMaxTorqueLimit();
  }

  //
  // Clear hekateros alarm led to provide visual feedback to the user. The
  // background check may relight the alarm led if alarms still persist.
  //
  m_bAlarmState = false;
  m_monitor.markAlarmCond(m_bAlarmState);

  return HEK_OK;
}

int HekRobot::moveArm(HekJointTrajectoryPoint &trajectoryPoint)
{
  int   rc;

  HEK_TRY_CONN();
  HEK_TRY_CALIB();
  HEK_TRY_NOT_ESTOP();

  lock();

  if( !m_bAreServosPowered )
  {
    m_bAreServosPowered = true;

    m_monitor.markPoweredCond(m_bAreServosPowered);
  }

  rc = m_pKin->move(trajectoryPoint);

  unlock();

  return rc >= 0? HEK_OK: rc;
}

int HekRobot::getRobotState(HekRobotState &robotState)
{
  static int      TuneStoppedSpeed = 9;   ///< stopped speed

  int             iter;           // servo iterator
  int             nServoId;       // servo id
  DynaServo      *pServo;         // servo
  bool            bIsMoving;      // robot is [not] moving
  HekServoHealth  health;         // servo health
  bool            bOldAlarmState; // old alarm state

  robotState.clear();

  bOldAlarmState  = m_bAlarmState;
  m_bAlarmState   = m_monitor.getAlarmCond();

  robotState.m_eRobotMode         = m_eRobotMode;
  robotState.m_eIsEStopped        = m_bIsEStopped?
                                            HekTriStateTrue: HekTriStateFalse;
  robotState.m_eIsCalibrated      = m_eOpState == HekOpStateCalibrated?
                                            HekTriStateTrue: HekTriStateFalse;
  robotState.m_eAreDrivesPowered  = m_bAreServosPowered? 
                                            HekTriStateTrue: HekTriStateFalse;
  robotState.m_eIsMotionPossible  = (m_eOpState == HekOpStateCalibrated) &&
                                      !m_bAlarmState && !m_bNoExec? 
                                            HekTriStateTrue: HekTriStateFalse;

  robotState.m_eIsInError = HekTriStateFalse;
  robotState.m_nErrorCode = HEK_OK;
  bIsMoving               = false;

  for(nServoId = m_pDynaChain->IterStart(&iter);
      nServoId != DYNA_ID_NONE;
      nServoId = m_pDynaChain->IterNext(&iter))
  {
    pServo = m_pDynaChain->GetServo(nServoId);

    health.m_nServoId     = nServoId;
    health.m_fTemperature = pServo->CvtRawTempToC(pServo->GetCurTemp());
    health.m_fVoltage     = pServo->CvtRawVoltToVolts(pServo->GetCurVolt());
    health.m_uAlarms      = pServo->GetAlarms();

    if( health.m_uAlarms != DYNA_ALARM_NONE )
    {
      robotState.m_eIsInError = HekTriStateTrue;
      robotState.m_nErrorCode = -HEK_ECODE_ALARMED;
    }

    if( iabs(pServo->GetCurSpeed()) > TuneStoppedSpeed )
    {
      bIsMoving = true;
    }

    robotState.m_vecServoHealth.push_back(health);
  }

  // estop overrides other alarms
  if( m_bIsEStopped )
  {
    robotState.m_eIsInError = HekTriStateTrue;
    robotState.m_nErrorCode = -HEK_ECODE_ESTOP;
  }

  // new alarm state other than estop. 
  else if( m_bAlarmState && (m_bAlarmState != bOldAlarmState) )
  {
    robotState.m_eIsInError = HekTriStateTrue;
    robotState.m_nErrorCode = -HEK_ECODE_ALARMED;
  }

  robotState.m_eIsInMotion = bIsMoving? HekTriStateTrue: HekTriStateFalse;

  return HEK_OK;
}

int HekRobot::getJointState(HekJointStatePoint &jointStatePoint)
{
  MapRobotJoints::iterator  iter;   // kinematic chain iterator
  int             nMasterServoId;   // master servo id
  HekRobotJoint  *pJoint;           // robotic joint
  DynaServo      *pServo;           // servo
  HekJointState   jointState;       // working joint state
  byte_t          uMask;            // working bit mask
  int             i;                // working index

  HEK_TRY_CONN();

  jointStatePoint.clear();

  jointStatePoint.setKinematicChainName("arm");

  //
  // Build joint state point.
  //
  for(iter = m_jointsArm.begin(); iter != m_jointsArm.end(); ++iter)
  {
    nMasterServoId  = iter->first;
    pJoint          = &(iter->second);

    HEK_TRY_GET_SERVO(nMasterServoId, pServo);

    // identifiers
    jointState.m_strName        = pJoint->m_strName;
    jointState.m_eOpState       = pJoint->m_eOpState;
    jointState.m_nMasterServoId = nMasterServoId;
    jointState.m_nSlaveServoId  = pJoint->m_nSlaveServoId;

    // joint dynamics
    m_pKin->getFilteredJointCurPosVel(jointState.m_strName,
                                      jointState.m_fPosition,
                                      jointState.m_fVelocity);

    // servo dynamics
    jointState.m_nOdPos   = pServo->GetOdometer();
    jointState.m_nEncPos  = pServo->OdometerToEncoder(jointState.m_nOdPos);
    jointState.m_nSpeed   = pServo->GetCurSpeed();
    jointState.m_fEffort  = (double)pServo->GetCurLoad();

    // optical limit switch states
    for(int i=0; i<HekOptLimitMaxPerJoint; ++i)
    {
      jointState.m_eOptSwitch[i] = m_monitor.getJointLimitTriState(pJoint, i);
    }

    // add state
    jointStatePoint.append(jointState);
  }

  return HEK_OK;
}

int HekRobot::getTrajectoryState(HekJointTrajectoryFeedback &trajectoryFeedback)
{
  HekJointStatePoint  jointStatePoint;
  int                 iActual;
  int                 i;
  int                 rc;

  trajectoryFeedback.clear();

  // last issued trajectory is the desired trajectory
  trajectoryFeedback[HekJointTrajectoryFeedback::TRAJ_DESIRED] = m_lastTrajArm;

  // retrieve joint state
  if( (rc = getJointState(jointStatePoint)) < 0 )
  {
    LOGERROR("Cannot retrieve joint state data.");
    return rc;
  }

  // copy joint state to actual trajectory
  iActual = HekJointTrajectoryFeedback::TRAJ_ACTUAL;
 
  for(i=0; i<jointStatePoint.getNumPoints(); ++i)
  {
    trajectoryFeedback[iActual].append(jointStatePoint[i].m_strName,
                                       jointStatePoint[i].m_fPosition,
                                       jointStatePoint[i].m_fVelocity);
  }

  return HEK_OK;
}

bool HekRobot::isInMotion()
{
  static int      TuneStoppedSpeed = 9;   ///< stopped speed

  int             iter;       // servo iterator
  int             nServoId;   // servo id
  DynaServo      *pServo;     // servo

  for(nServoId = m_pDynaChain->IterStart(&iter);
      nServoId != DYNA_ID_NONE;
      nServoId = m_pDynaChain->IterNext(&iter))
  {
    pServo = m_pDynaChain->GetServo(nServoId);

    if( iabs(pServo->GetCurSpeed()) > TuneStoppedSpeed )
    {
      return true;
    }
  }

  return false;
}

int HekRobot::scanDynaBus(int nMaxTries)
{
  static uint_t usecTry = 500000;

  int   nNumServosExpected;   // number of expected and required servos
  int   nNumServosScanned;    // number of scanned servos
  int   nNumServosMatched;    // number of scanned and matched servos
  int   nTries;               // scan attempts
  int   nServoId;             // servo id
  int   iter;                 // servo iterator
  int   rc;                   // return code

  nNumServosExpected = m_descHek.getNumServos();

  //
  // Scan hardware.
  //
  for(nTries=0; nTries<nMaxTries; ++nTries)
  {
    LOGDIAG3("Scanning Hekateros hardware - attempt %d of %d.",
        nTries+1, nMaxTries);

    // scan for servos
    nNumServosScanned = m_pDynaChain->AddNewServosByScan();

    //
    // Scanned at least the required number.
    //
    if( nNumServosScanned >= nNumServosExpected )
    {
      // check scanned ids against required
      for(nServoId = m_pDynaChain->IterStart(&iter), nNumServosMatched=0;
          nServoId != DYNA_ID_NONE;
          nServoId = m_pDynaChain->IterNext(&iter))
      {
        // matched
        if( m_descHek.hasServo(nServoId) )
        {
          ++nNumServosMatched;
        }

        // unmathced
        else
        {
          LOGWARN("Servo %d unexpected - uncontrolled.", nServoId);
        }
      }

      // servos matched
      if( nNumServosMatched == nNumServosExpected )
      {
        rc = HEK_OK;
        break;
      }

      // some required servos are missing
      else
      {
        LOGERROR("Match %d scanned servo ids, expected %d required matches.",
          nNumServosMatched, nNumServosExpected);
        rc = -HEK_ECODE_FORMAT;
      }
    }

    //
    // Scan failed to detect the minimum number of servos.
    //
    else
    {
      LOGERROR("Scanned %d servos, expected %d.",
          nNumServosScanned, nNumServosExpected);
      rc = -HEK_ECODE_NO_RSRC;
    }

    usleep(usecTry);
  }

  if( rc == HEK_OK )
  {
    LOGDIAG3("Hekateros dynamixel bus hardware scanned successfully.");
  }

  return rc;
}

int HekRobot::convertSpecs()
{
  HekDescArm       *pDescArm;
  HekDescEE        *pDescEE;
  HekSpecJoint_T   *pSpecJoint;
  HekSpecServo_T   *pSpecServo;
  int               nServoId;
  HekRobotJoint    *pJoint;
  int               i;
  int               rc;

  m_jointsArm.clear();
  m_jointsEquipDeck.clear();
  m_jointsAux.clear();
  m_imapJoints.clear();

  pDescArm = m_descHek.getArmDesc();
  pDescEE  = m_descHek.getEEDesc();

  //
  // Build up arm kinematic chain.
  //
  for(i = 0; i < pDescArm->m_spec.m_nDoF; ++i)
  {
    // joint specification
    pSpecJoint = pDescArm->m_spec.getJointSpecAt(i);

    // master servo id associated with joint
    nServoId  = pSpecJoint->m_nMasterServoId;
    pJoint    = &m_jointsArm[nServoId];

    // servo specification
    if( (pSpecServo = pDescArm->m_spec.getServoSpec(nServoId)) == NULL )
    {
      LOGERROR("Servo %d: Cannot find servo specification.", nServoId);
      return -HEK_ECODE_NO_EXEC;
    }

    // add rebotic joint
    rc = addRobotJoint(pSpecJoint, pSpecServo, m_jointsArm, m_imapJoints);
    if( rc < 0 )
    {
      LOGERROR("Servo %d: Cannot add to kinematic chain.", nServoId);
      return rc;
    }

    // add any optical limit switches
    if( (rc = m_monitor.addJointLimitsToMonitor(pSpecJoint, pJoint)) < 0 )
    {
      LOGERROR("Servo %d: Cannot add to optical limit(s).", nServoId);
      return rc;
    }
  }

  //
  // Add end effector joints to arm kinematic chain.
  //
  for(i = 0; i < pDescEE->m_spec.m_nDoF; ++i)
  {
    // joint specification
    pSpecJoint = pDescEE->m_spec.getJointSpecAt(i);

    // master servo id associated with joint
    nServoId  = pSpecJoint->m_nMasterServoId;
    pJoint    = &m_jointsArm[nServoId];

    // servo specification
    if( (pSpecServo = pDescEE->m_spec.getServoSpec(nServoId)) == NULL )
    {
      LOGERROR("Servo %d: Cannot find servo specification.", nServoId);
      return -HEK_ECODE_NO_EXEC;
    }

    // add rebotic joint
    rc = addRobotJoint(pSpecJoint, pSpecServo, m_jointsArm, m_imapJoints);
    if( rc < 0 )
    {
      LOGERROR("Servo %d: Cannot add to kinematic chain.", nServoId);
      return rc;
    }

    // add any optical limit switches
    if( (rc = m_monitor.addJointLimitsToMonitor(pSpecJoint, pJoint)) < 0 )
    {
      LOGERROR("Servo %d: Cannot add to optical limit(s).", nServoId);
      return rc;
    }
  }

  //
  // TODO: Add equipment deck and auxiliary effector chains here.
  //
 
  return HEK_OK;
}

int HekRobot::addRobotJoint(HekSpecJoint_T  *pSpecJoint,
                            HekSpecServo_T  *pSpecServo,
                            MapRobotJoints  &kin,
                            IMapRobotJoints &imap)
{
  int               nMasterServoId;     // master servo id
  DynaServo        *pServo;             // servo
  uint_t            uTicks;             // ticks
  double            fAngleMin;          // min angle(deg) in servo mode
  double            fAngleMax;          // max angle(deg) in servo mode
  double            fMaxRpm;            // max unloaded rpm
  double            fDegrees;           // working degrees
  double            fRadians;           // working radians
  HekRobotJoint     joint;              // robotic joint
  int               i;                  // working index

  // master servo id associated with joint
  nMasterServoId = pSpecJoint->m_nMasterServoId;

  // dynamixel servo object in dynamixel chain
  if( (pServo = m_pDynaChain->GetServo(nMasterServoId)) == NULL )
  {
    LOGERROR("Servo %d: Cannot find servo in dynamixel chain.", nMasterServoId);
    return -HEK_ECODE_NO_EXEC;
  }

  //
  // Dynamixel servo specification loaded during scan.
  //
  uTicks      = pServo->GetSpecification().m_uRawPosModulo;
  fAngleMin   = pServo->GetSpecification().m_fAngleMin;
  fAngleMax   = pServo->GetSpecification().m_fAngleMax;
  fMaxRpm     = pServo->GetSpecification().m_fMaxSpeed;

  //
  // Servo rotation range.
  //
  fDegrees = pSpecServo->m_bIsContinuous? 360.0: fAngleMax - fAngleMin;
  fRadians = degToRad(fDegrees);

  //
  // Populate joint data.
  //
  // Some data, such as joint rotation limits may be refined during
  // calibration.
  //
  joint.m_strName             = pSpecJoint->m_strName;
  joint.m_nMasterServoId      = nMasterServoId;
  joint.m_nSlaveServoId       = pSpecJoint->m_nSlaveServoId;
  joint.m_bIsServoContinuous  = pSpecServo->m_bIsContinuous;
  joint.m_nMasterServoDir     = pSpecServo->m_nDir;
  joint.m_eJointType          = pSpecJoint->m_eJointType;
  joint.m_fGearRatio          = pSpecJoint->m_fGearRatio;
  joint.m_fTicksPerServoRad   = (double)uTicks / fRadians; 
  joint.m_fTicksPerJointRad   = joint.m_fTicksPerServoRad *
                                  pSpecJoint->m_fGearRatio;
  joint.m_fMaxServoRadsPerSec = fMaxRpm * M_TAU / 60.0;
  joint.m_fMaxJointRadsPerSec = joint.m_fMaxServoRadsPerSec /
                                  joint.m_fGearRatio;
  joint.m_fMinPhyLimitRads    = degToRad(pSpecJoint->m_fMinPhyLimit);
  joint.m_fMaxPhyLimitRads    = degToRad(pSpecJoint->m_fMaxPhyLimit);
  joint.m_nMinPhyLimitOd      = (int)(joint.m_fTicksPerJointRad *
                                      joint.m_fMinPhyLimitRads);
  joint.m_nMaxPhyLimitOd      = (int)(joint.m_fTicksPerJointRad *
                                      joint.m_fMaxPhyLimitRads );
  joint.m_fMinSoftLimitRads   = joint.m_fMinPhyLimitRads;
  joint.m_fMaxSoftLimitRads   = joint.m_fMaxPhyLimitRads;
  joint.m_nMinSoftLimitOd     = joint.m_nMinPhyLimitOd;
  joint.m_nMaxSoftLimitOd     = joint.m_nMaxPhyLimitOd;
  joint.m_fCalibPosRads       = degToRad(pSpecJoint->m_fCalibPos);
  joint.m_fBalPosRads         = degToRad(pSpecJoint->m_fBalancedPos);
  joint.m_fParkPosRads        = degToRad(pSpecJoint->m_fParkedPos);
  joint.m_eLimitTypes         = pSpecJoint->m_eLimitTypes;

  joint.m_fOverTorqueThDft    = fcap(pSpecServo->m_fTorqueLimitPct,
                                    HekTuneOverTorqueThMin,
                                    HekTuneOverTorqueThMax) / 100.0;

  joint.m_eOpState            = HekOpStateUncalibrated;
  joint.m_bStopAtOptLimits    = false;

  for(i=0; i<HekOptLimitMaxPerJoint; ++i)
  {
    joint.m_byOptLimitMask[i] = pSpecJoint->m_limit[i].m_uBit;
  }

  //
  // Sanity checks.
  //
  joint.m_fCalibPosRads = fcap(joint.m_fCalibPosRads,
                                joint.m_fMinPhyLimitRads,
                                joint.m_fMaxPhyLimitRads);
  joint.m_fBalPosRads   = fcap(joint.m_fBalPosRads,
                                joint.m_fMinPhyLimitRads,
                                joint.m_fMaxPhyLimitRads);
  joint.m_fParkPosRads  = fcap(joint.m_fParkPosRads,
                                joint.m_fMinPhyLimitRads,
                                joint.m_fMaxPhyLimitRads);

  //
  // Add to kinematic chain.
  //
  kin[nMasterServoId]   = joint;            // kinematic chain
  imap[joint.m_strName] = nMasterServoId;   // indirect map by joint name

  return HEK_OK;
}

void HekRobot::adjustTuningFromSpecs()
{
  MapRobotJoints::iterator  iter;         // joint description iterator
  HekTunesJoint             tunesJoint;   // initialzed with default defaults
  string                    strJointName; // name of joint

  for(iter = m_jointsArm.begin(); iter != m_jointsArm.end(); ++iter)
  {
    strJointName = iter->second.m_strName;

    // per joint defaults from compiled specs
    tunesJoint.m_fOverTorqueTh = iter->second.m_fOverTorqueThDft;

    // add
    m_tunes.m_mapJointTunes[strJointName] = tunesJoint;
  }
}

void HekRobot::fauxcalibrate()
{
  MapRobotJoints::iterator  iter;

  freeze();

  for(iter = m_jointsArm.begin(); iter != m_jointsArm.end(); ++iter)
  {
    iter->second.m_eOpState = HekOpStateCalibrated;
  }
  m_pKin->resetServoOdometersForAllJoints();
  m_eOpState = HekOpStateCalibrated;
}

int HekRobot::configServos()
{
  int   rc;

  if( (rc = configEEPROMForAllServos(m_jointsArm)) < 0 )
  {
    LOGERROR("Failed to configure servo EEPROM for all servos.");
    return rc;
  }

  if( (rc = configRAMForAllServos(m_jointsArm)) < 0 )
  {
    LOGERROR("Failed to configure servo RAM for all servos.");
    return rc;
  }

  return HEK_OK;
}

int HekRobot::configEEPROMForAllServos(MapRobotJoints &kin)
{
  MapRobotJoints::iterator  iter;
  
  int     nServoId;
  int     rc;

  for(iter = kin.begin(); iter != kin.end(); ++iter)
  {
    nServoId = iter->second.m_nMasterServoId;

    if( (rc = configEEPROMForServo(nServoId, iter->second)) < 0 )
    {
      LOGERROR("Servo %d: Failed to configure servo EEPROM memory map.",
        nServoId);
      return rc;
    }

    nServoId = iter->second.m_nSlaveServoId;

    if( nServoId != DYNA_ID_NONE )
    {
      if( (rc = configEEPROMForServo(nServoId, iter->second)) < 0 )
      {
        LOGERROR("Servo %d: Failed to configure servo EEPROM memory map.",
          nServoId);
        return rc;
      }
    }
  }

  return HEK_OK;
}

int HekRobot::configEEPROMForServo(int nServoId, HekRobotJoint &joint)
{
  DynaServo  *pServo;
  uint_t      uPosMin;
  uint_t      uPosMax;
  uint_t      uTorqueMax;
  uint_t      uVal;
  uint_t      uMask;
  int         rc;

  // dynamixel servo object in dynamixel chain
  HEK_TRY_GET_SERVO(nServoId, pServo);

  uPosMin     = pServo->GetSpecification().m_uRawPosMin;
  uPosMax     = pServo->GetSpecification().m_uRawPosMax;
  uTorqueMax  = pServo->GetSpecification().m_uRawTorqueMax;

  // --
  // Ensure servo is in the correct servo mode.
  // --
  
  // get servo must be in continuous mode
  if( joint.m_bIsServoContinuous &&
      (pServo->GetServoMode() != DYNA_MODE_CONTINUOUS)) 
  {
    if( (rc = pServo->CfgWriteServoModeContinuous()) < 0 )
    {
      LOGERROR("Servo %d: Cannot configure EEPROM for continuous mode.",
          nServoId);
      return -HEK_ECODE_DYNA;
    }
  }

  // servo must be in servo mode
  else if( !joint.m_bIsServoContinuous && 
           (pServo->GetServoMode() == DYNA_MODE_CONTINUOUS) )
  {
    if( (rc = pServo->CfgWriteServoMode(uPosMin, uPosMax)) < 0 )
    {
      LOGERROR("Servo %d: Cannot configure EEPROM for servo mode.", nServoId);
      return -HEK_ECODE_DYNA;
    }
  }

  // --
  // Ensure maximum torque limit reload value is at the maximum.
  // --
  
  // always set maximum torque limit
  if( (rc = pServo->CfgReadMaxTorqueLimit(&uVal)) < 0 )
  {
    LOGWARN("Servo %d: Cannot read EEPROM maximum torque limit.", nServoId);
  }
  else if( uVal < uTorqueMax )
  {
    if( (rc = pServo->CfgWriteMaxTorqueLimit(uTorqueMax)) < 0 )
    {
      LOGWARN("Servo %d: Cannot configure EEPROM maximum torque limit.",
          nServoId);
    }
  }

  // --
  // Ensure alarm shutdown mask as the correct values.
  // --

  // for servos with no torque limit, disable over-load alarms
  uMask = DYNA_ALARM_VOLTAGE | DYNA_ALARM_ANGLE | DYNA_ALARM_TEMP |
            DYNA_ALARM_LOAD;

  if( (rc = pServo->CfgReadAlarmShutdownMask(&uVal)) < 0 )
  {
    LOGWARN("Servo %d: Cannot read EEPROM alarm shutdown mask.", nServoId);
  }
  else if( uVal != uMask )
  {
    if( (rc = pServo->CfgWriteAlarmShutdownMask(uMask)) < 0 )
    {
      LOGWARN("Servo %d: Cannot configure EEPROM alarm shutdown mask.",
          nServoId);
    }
  }

  return HEK_OK;
}

int HekRobot::configRAMForAllServos(MapRobotJoints &kin)
{
  MapRobotJoints::iterator  iter;
  int                       nServoId;
  int                       rc;

  for(iter = kin.begin(); iter != kin.end(); ++iter)
  {
    nServoId = iter->second.m_nMasterServoId;

    if( (rc = configRAMForServo(nServoId, iter->second)) < 0 )
    {
      LOGERROR("Servo %d: Failed to configure servo RAM memory map.",
        nServoId);
      return rc;
    }


    // no slave configuration
  }

  return HEK_OK;
}

int HekRobot::configRAMForServo(int nServoId, HekRobotJoint &joint)
{
  DynaServo  *pServo;
  uint_t      uTorqueMax;
  int         rc;

  // get dynamixel servo object in dynamixel chain
  HEK_TRY_GET_SERVO(nServoId, pServo);

  uTorqueMax  = pServo->GetSpecification().m_uRawTorqueMax;

  // --
  // Ensure running torque limit is at the maximum. 
  // --
 
  if( (rc = pServo->WriteMaxTorqueLimit(uTorqueMax)) < 0 )
  {
    LOGWARN("Servo %d: Cannot write RAM alarm shutdown mask.", nServoId);
  }

  // --
  // Disable servo drive on start up.
  // --

  if( (rc = pServo->WriteTorqueEnable(false)) < 0 )
  {
    LOGERROR("Servo %d: Cannot enable servo drive.", nServoId);
    return -HEK_ECODE_DYNA;
  }

  return HEK_OK;
}

void HekRobot::resetCalibStateForAllJoints(bool bForceRecalib)
{
  MapRobotJoints::iterator  iter;

  for(iter = m_jointsArm.begin(); iter != m_jointsArm.end(); ++iter)
  {
    if( bForceRecalib || (iter->second.m_eOpState != HekOpStateCalibrated) )
    {
      iter->second.m_eOpState = HekOpStateUncalibrated;
    }
  }
}

HekOpState HekRobot::determineRobotOpState()
{
  MapRobotJoints::iterator  iter;

  for(iter = m_jointsArm.begin(); iter != m_jointsArm.end(); ++iter)
  {
    if( iter->second.m_eOpState != HekOpStateCalibrated )
    {
      return HekOpStateUncalibrated;
    }
  }

  return HekOpStateCalibrated;
}

int HekRobot::createAsyncThread()
{
  int   rc;

  m_eAsyncTaskState = HekAsyncTaskStateWorking;
  m_rcAsyncTask     = HEK_OK;

  rc = pthread_create(&m_threadAsync, NULL, HekRobot::asyncThread, (void*)this);
 
  if( rc == 0 )
  {
    rc = HEK_OK;
  }

  else
  {
    m_eAsyncTaskState = HekAsyncTaskStateIdle;
    LOGSYSERROR("pthread_create()");
    m_rcAsyncTask   = -HEK_ECODE_SYS;
    m_eAsyncTaskId  = AsyncTaskIdNone;
    m_pAsyncTaskArg = NULL;
    rc = m_rcAsyncTask;
  }

  return rc;
}

void HekRobot::cancelAsyncTask()
{
  MapRobotJoints::iterator  iter;

  if( m_eAsyncTaskState != HekAsyncTaskStateIdle )
  {
    // cancel thread
    pthread_cancel(m_threadAsync);
    pthread_join(m_threadAsync, NULL);

    // cleanup
    switch( m_eAsyncTaskId )
    {
      case AsyncTaskIdCalibrate:
        freeze();
        for(iter = m_jointsArm.begin(); iter != m_jointsArm.end(); ++iter)
        {
          if( iter->second.m_eOpState != HekOpStateCalibrated )
          {
            iter->second.m_eOpState = HekOpStateUncalibrated;
            m_eOpState              = HekOpStateUncalibrated;
          }
        }
        break;
      default:
        break;
    }

    // clear state
    m_eAsyncTaskId    = AsyncTaskIdNone;
    m_pAsyncTaskArg   = NULL;
    m_rcAsyncTask     = -HEK_ECODE_INTR;
    m_eAsyncTaskState = HekAsyncTaskStateIdle;
    LOGDIAG3("Async task canceled.");
  }
}

HekAsyncTaskState HekRobot::getAsyncState()
{
  return m_eAsyncTaskState;
}

int HekRobot::getAsyncRc()
{
  return m_rcAsyncTask;
}

void *HekRobot::asyncThread(void *pArg)
{
  HekRobot *pThis = (HekRobot *)pArg;
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
    // Calibrate the robot. 
    case AsyncTaskIdCalibrate:
      {
        bool bForceRecalib = (bool)pThis->m_pAsyncTaskArg;
        rc = pThis->calibrate(bForceRecalib);
      }
      break;

    // Unknown task id.
    default:
      LOGERROR("Unknown async task id = %d.", (int)pThis->m_eAsyncTaskId);
      rc = -HEK_ECODE_BAD_VAL;
      break;
  }

  pThis->m_eAsyncTaskId     = AsyncTaskIdNone;
  pThis->m_pAsyncTaskArg    = NULL;
  pThis->m_rcAsyncTask      = rc;
  pThis->m_eAsyncTaskState  = HekAsyncTaskStateIdle;

  LOGDIAG3("Async robot task thread exited.");

  return NULL;
}
