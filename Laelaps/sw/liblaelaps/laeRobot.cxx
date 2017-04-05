////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Library:   liblaelaps
//
// File:      laeRobot.cxx
//
/*! \file
 *
 * \brief Laelaps Robot Class implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
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
#include <unistd.h>
#include <pthread.h>
#include <math.h>

#include <string>
#include <utility>
#include <vector>
#include <map>

#include "rnr/rnrconfig.h"
#include "rnr/units.h"
#include "rnr/i2c.h"
#include "rnr/log.h"

// common
#include "Laelaps/laelaps.h"
#include "Laelaps/laeUtils.h"

// hardware
#include "Laelaps/laeSysDev.h"
#include "Laelaps/RoboClaw.h"
#include "Laelaps/laeMotor.h"
#include "Laelaps/laeI2C.h"
#include "Laelaps/laeI2CMux.h"
#include "Laelaps/laeWatchDog.h"
#include "Laelaps/laeWd.h"
#include "Laelaps/laeVL6180.h"
#include "Laelaps/laeImu.h"
#include "Laelaps/laeCams.h"

// descriptions, tuning, and database
#include "Laelaps/laeTune.h"
#include "Laelaps/laeDesc.h"
#include "Laelaps/laeXmlTune.h"
#include "Laelaps/laeDb.h"

// control and application interface
#include "Laelaps/laeTraj.h"
#include "Laelaps/laePowertrain.h"
#include "Laelaps/laePlatform.h"
#include "Laelaps/laeKin.h"
#include "Laelaps/laeAlarms.h"
#include "Laelaps/laeReports.h"
#include "Laelaps/laeThreadAsync.h"
#include "Laelaps/laeThreadImu.h"
#include "Laelaps/laeThreadKin.h"
#include "Laelaps/laeThreadRange.h"
#include "Laelaps/laeThreadWd.h"

// the robot
#include "Laelaps/laeRobot.h"

using namespace std;
using namespace laelaps;
using namespace motor::roboclaw;
using namespace sensor::vl6180;
using namespace sensor::imu;

/*!
 * \brief Test for no execute flag.
 *
 * Only works in LaeRobot methods.
 *
 * \return On true, return with LAE_OK.
 */
#define LAE_TRY_NO_EXEC() \
  do \
  { \
    if( m_bNoExec ) \
    { \
      return LAE_OK; \
    } \
  } while(0)

/*!
 * \brief Test for connection.
 *
 * Only works in LaeRobot methods.
 *
 * \return On failure, forces return from calling function with the appropriate
 * error code.
 */
#define LAE_TRY_CONN() \
  do \
  { \
    if( !isConnected() ) \
    { \
      LOGERROR("Robot is not connected."); \
      return -LAE_ECODE_NO_EXEC; \
    } \
  } while(0)

/*!
 * \brief Test for not estop.
 *
 * Only works in LaeRobot methods.
 *
 * \return On failure, forces return from calling function with the appropriate
 * error code.
 */
#define LAE_TRY_NOT_ESTOP() \
  do \
  { \
    if( m_bIsEStopped ) \
    { \
      LOGERROR("Robot is emergency stopped."); \
      return -LAE_ECODE_NO_EXEC; \
    } \
  } while(0)


// -----------------------------------------------------------------------------
// Class LaeRobot
// -----------------------------------------------------------------------------

const double LaeRobot::GovernorDft = 1.0;

LaeRobot::LaeRobot(bool bNoExec) :
    m_i2cBus(), m_watchdog(m_i2cBus), m_range(m_i2cBus),
    m_threadImu(m_imu),
    m_threadKin(m_kin),
    m_threadRange(m_range),
    m_threadWatchDog(m_watchdog)
{
  int   nCtlr;

  // state
  m_bNoExec           = bNoExec;
  m_bIsConnected      = false;
  m_eRobotMode        = LaeRobotModeUnknown;
  m_bIsEStopped       = false;
  m_bAlarmState       = false;

  setGovernor(GovernorDft);

  syncDb();

  // asynchronous job 
  m_pAsyncJob         = NULL;
}

LaeRobot::~LaeRobot()
{
  disconnect();
}

int LaeRobot::connect()
{
  int     rc;           // return code

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Pre-connect requirements.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  //
  // Been here, did this.
  //
  if( m_bIsConnected )
  {
    LOGWARN("Laelaps already connected to hardware.");
    return LAE_OK;
  }

  //
  // Need a robot description before preceeding. The controlling application, 
  // typically a ROS node, provides the description. The desription is normally
  // the parsed data found in /etc/laelaps/laelaps.conf XML file.
  //
  if( !m_descLaelaps.isDescribed() )
  {
    LOGERROR("Undefined Laelaps description - "
             "don't know how to initialized properly.");
    return -LAE_ECODE_BAD_OP;
  }

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Initialize robot status.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  m_bIsConnected      = false;
  m_eRobotMode        = LaeRobotModeUnknown;
  m_bIsEStopped       = false;
  m_bAlarmState       = false;

  syncDb();

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Parse and set tuning parameters.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  //
  // Override any default tuning parameters from the optional, user-specified
  // tuning XML file.
  //
  LaeXmlTune  xml;

  // parse tune XML file and set tuning parameter overrides
  xml.load(m_tunes, LaeSysCfgPath, LaeEtcTune);

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Establish hardware connections.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  //
  // Laelaps peripherals I2C bus.
  //
  if( (rc = m_i2cBus.open(LaeDevI2C)) < 0 )
  {
    LOGSYSERROR("%s.", LaeDevI2C);
  }

  //
  // Connect to all standard built-in sensors.
  //
  if( rc == LAE_OK )
  {
    rc = connSensors();
  }

  //
  // Connect to the external watchdog arduino subprocessor.
  // Note:  The watchdog connection must preceed connection to the motor
  //        controllers.
  //
  if( rc == LAE_OK )
  {
    rc = connWatchDog();
  }

  //
  // Connect to motor controllers.
  //
  if( rc == LAE_OK )
  {
    rc = connMotorControllers(LaeDevMotorCtlrs, LaeBaudRateMotorCtlrs);
  }

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Initialize and Configure
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
 
  //
  // Configure hardware for operation.
  //
  if( rc == LAE_OK )
  {
    rc = configForOperation();
  }

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Start real-time, persistent threads.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
 
  if( rc == LAE_OK )
  {
    rc = startCoreThreads();
  }

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Finale
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
 
  // failure
  if( rc != LAE_OK )
  {
    LOGERROR("Failed to connect and/or initialize hardware.");
    disconnect();
  }

  // success
  else
  {
    m_bIsConnected      = true;
    m_eRobotMode        = LaeRobotModeAuto;
    m_bIsEStopped       = false;
    m_bAlarmState       = false;

    syncDb();

    freeze(); // place robot in a safe state

    LOGDIAG1("Connected to Laelaps.");
  }

  return rc;
}

int LaeRobot::disconnect()
{
  int   nCtlr;

  if( !isConnected() )
  {
    return LAE_OK;
  }

  //
  // Terminate all threads.
  //
  cancelAsyncJob();

  m_threadWatchDog.terminateThread();
  m_threadKin.terminateThread();
  m_threadImu.terminateThread();
  m_threadRange.terminateThread();

  m_kin.close();              // close motors
  m_imu.close();              // on usb device
  m_i2cBus.close();           // sensors/watchdog on i2c device

  m_range.clearSensedData();
  m_imu.clearSensedData();

  // reset robot state
  m_bIsConnected      = false;
  m_eRobotMode        = LaeRobotModeUnknown;
  m_bIsEStopped       = false;
  m_bAlarmState       = false;

  syncDb();

  LOGDIAG1("Disconnected from Laelaps.");

  return LAE_OK;
}

int LaeRobot::reload()
{
  LaeXmlTune  xml;    // xml parser
  int         rc;     // return code

  // reload tune XML file and set tuning parameter overrides
  xml.load(m_tunes, LaeSysCfgPath, LaeEtcTune);

  // re-tune Kinodynamics thread and hardware i/f
  if( (rc = m_threadKin.reload(m_tunes)) != LAE_OK )
  {
    LOGERROR("Failed to reload kinodynamics tune parameters.");
    return rc;
  }

  // re-tune IMU thread and hardware i/f
  else if( (rc = m_threadImu.reload(m_tunes)) != LAE_OK )
  {
    LOGERROR("Failed to reload IMU tune parameters.");
    return rc;
  }

  // re-tune Range thread and hardware i/f
  else if( (rc = m_threadRange.reload(m_tunes)) != LAE_OK )
  {
    LOGERROR("Failed to reload range sensing tune parameters.");
    return rc;
  }

  // re-tune WatchDog thread and hardware i/f
  else if( (rc = m_threadWatchDog.reload(m_tunes)) != LAE_OK )
  {
    LOGERROR("Failed to reload WatchDog tune parameters.");
    return rc;
  }

  if( rc == LAE_OK )
  {
    LOGDIAG2("Reloaded tuning parameters and reconfigured robot.");
  }

  return rc;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Action Methods.
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int LaeRobot::estop()
{
  int   rc;   // return code

  LAE_TRY_NO_EXEC();
  LAE_TRY_CONN();

  m_kin.estop();

  if( !m_kin.isEnabled() )
  {
    m_bIsEStopped = true;
    m_bAlarmState = true;

    syncDb();

    rc = LAE_OK;

    LOGDIAG3("Laelaps emergency stopped.");
  }
  else
  {
    rc = -LAE_ECODE_MOT_CTLR;

    LOGERROR("Failed to emergency stop.");
  }

  return rc;
}

int LaeRobot::resetEStop()
{
  int   rc;   // return code

  LAE_TRY_NO_EXEC();
  LAE_TRY_CONN();

  m_kin.resetEStop();

  if( m_kin.isEnabled() )
  {
    m_bIsEStopped = false;
    m_bAlarmState = false;

    syncDb();

    rc = LAE_OK;

    LOGDIAG3("Laelaps emergency stopped reset.");
  }
  else
  {
    rc = -LAE_ECODE_MOT_CTLR;

    LOGERROR("Failed to reset emergency stopped condition.");
  }

  return rc;
}

int LaeRobot::stop()
{
  return freeze();
}

int LaeRobot::freeze()
{
  int   rc;

  LAE_TRY_NO_EXEC();
  LAE_TRY_CONN();

  if( (rc = m_kin.stop()) == LAE_OK )
  {
    LOGDIAG3("Laelaps frozen at current position.");
  }

  return rc;
}

int LaeRobot::release()
{
  int   rc;

  LAE_TRY_NO_EXEC();
  LAE_TRY_CONN();

  // release motor drives 
  if( (rc = m_kin.release()) == LAE_OK )
  {
    LOGDIAG3("Laelaps motor drives released.");
  }

  return rc;
}

int LaeRobot::move(const LaeMapVelocity &velocity)
{
  int   rc;

  LAE_TRY_CONN();

  rc = m_kin.setGoalVelocities(velocity);

  return rc;
}

int LaeRobot::move(double fVelLinear, double fVelAngular)
{
  LAE_TRY_CONN();

  // TODO
  //if( (rc = m_kin.setX(velocity)) == LAE_OK )
  //{
  //}

 return LAE_OK;
} 

int LaeRobot::setDutyCycles(const LaeMapDutyCycle &duty)
{
  int   rc;

  LAE_TRY_CONN();

  rc = m_kin.setGoalDutyCycles(duty);

  return rc;
}

double LaeRobot::setGovernor(double fGovernor)
{
  m_fGovernor = fcap(fGovernor, 0.0, 1.0);

  return m_fGovernor;
}

double LaeRobot::incrementGovernor(double fDelta)
{
  return setGovernor(m_fGovernor+fDelta);
}

double LaeRobot::getGovernor()
{
  return m_fGovernor;
}

void LaeRobot::setRobotMode(LaeRobotMode eRobotMode)
{
  m_eRobotMode = eRobotMode;
  RtDb.m_robotstatus.m_eRobotMode = m_eRobotMode;
}

int LaeRobot::setAuxPower(const string &strName, LaeTriState eState)
{
  bool  bGoal, bResult;
  int   rc = LAE_OK;

  LAE_TRY_CONN();

  bGoal = eState == LaeTriStateEnabled? true: false;

  if( strName == "aux_batt_en" )
  {
    rc = m_watchdog.cmdEnableAuxPortBatt(bGoal);
    //bResult = bGoal? m_powerBatt.enable(): m_powerBatt.disable();
  }
  else if( strName == "aux_5v_en" )
  {
    rc = m_watchdog.cmdEnableAuxPort5V(bGoal);
    //bResult = bGoal? m_power5V.enable(): m_power5V.disable();
  }
  else
  {
    LOGERROR("%s: Unknown auxilliary port name.", strName.c_str());
    rc = -LAE_ECODE_BAD_VAL;
  }

  return rc;
}

int LaeRobot::clearAlarms()
{
  LAE_TRY_CONN();

  return LAE_OK;
}

int LaeRobot::getRobotStatus(LaeRptRobotStatus &rptBotStatus)
{
  rptBotStatus.clear();

  rptBotStatus.generate(this);

  return LAE_OK;
}

int LaeRobot::configDigitalPin(uint_t pin, uint_t dir)
{
  return m_watchdog.cmdConfigDPin(pin, dir);
}

int LaeRobot::readDigitalPin(uint_t pin, uint_t &val)
{
  return m_watchdog.cmdReadDPin(pin, val);
}

int LaeRobot::writeDigitalPin(uint_t pin, uint_t val)
{
  return m_watchdog.cmdWriteDPin(pin, val);
}

int LaeRobot::readAnalogPin(uint_t pin, uint_t &val)
{
  return m_watchdog.cmdReadAPin(pin, val);
}

int LaeRobot::getImu(double accel[], double gyro[], double rpy[], Quaternion &q)
{
  double mag[NumOfAxes];    // no magnetometer

  m_imu.getImuData(accel, gyro, mag, rpy, q);

  return LAE_OK;
}

int LaeRobot::getRangeSensorProps(const string &strKey,
                                  string       &strRadiationType,
                                  double       &fFoV,
                                  double       &fBeamDir,
                                  double       &fMin,
                                  double       &fMax)
{
  return m_range.getSensorProps(strKey, 
                                strRadiationType,
                                fFoV, fBeamDir,
                                fMin, fMax);
}

int LaeRobot::getRange(const string &strKey, double &fRange)
{
    return m_range.getRange(strKey, fRange);
}

int LaeRobot::getRange(vector<string> &vecNames, vector<double> &vecRanges)
{
  return m_range.getRange(vecNames, vecRanges);
}

int LaeRobot::getAmbientLight(const string &strKey, double &fAmbient)
{
  return m_range.getAmbientLight(strKey, fAmbient);
}

int LaeRobot::getAmbientLight(vector<string> &vecNames,
                              vector<double> &vecAmbient)
{
  return m_range.getAmbientLight(vecNames, vecAmbient);
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Reports
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int LaeRobot::getDynamics(LaeRptDynamics &rptDynamics)
{
  LAE_TRY_CONN();

  rptDynamics.clear();

  rptDynamics.generate(this);

  return LAE_OK;
}

int LaeRobot::getNavigationState(LaeSimplePathFeedback &pathFeedback)
{
  // TBD
 
  pathFeedback.clear();

  return LAE_OK;
}

int LaeRobot::getNavigationState(LaePathFeedback &pathFeedback)
{
  // TBD
 
  pathFeedback.clear();

  return LAE_OK;
}

LaePowertrain *LaeRobot::getPowertrain(const string &strName)
{
  return m_kin.getPowertrain(strName);
}

LaePowertrain *LaeRobot::getPowertrain(int nMotorId)
{
  return m_kin.getPowertrain(nMotorId);
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Attibute Methods.
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

bool LaeRobot::isConnected()
{
  return m_bIsConnected;
}

void LaeRobot::getVersion(int &nVerMajor, int &nVerMinor, int &nRevision)
{
  uint_t  uHwVer;

  uHwVer    = m_descLaelaps.getProdHwVer();
  nVerMajor = LAE_VER_MAJOR(uHwVer);
  nVerMinor = LAE_VER_MINOR(uHwVer);
  nRevision = LAE_VER_REV(uHwVer);
}

string LaeRobot::getVersionString()
{
  return m_descLaelaps.getProdHwVerString();
}

LaeDesc &LaeRobot::getLaelapsDesc()
{
  return m_descLaelaps;
}

int LaeRobot::getProdId()
{
  return m_descLaelaps.getProdId();
}

string LaeRobot::getProdName()
{
  return m_descLaelaps.getProdName();
}

string LaeRobot::getFullProdBrief()
{
  return m_descLaelaps.getProdBrief();
}

bool LaeRobot::isDescribed()
{
  return m_descLaelaps.isDescribed();
}

LaeRobotMode LaeRobot::getRobotMode()
{
  return m_eRobotMode;
}

bool LaeRobot::isEStopped()
{
  return m_bIsEStopped;
}

bool LaeRobot::areMotorsPowered()
{
  return m_kin.areMotorsPowered();
}

bool LaeRobot::isInMotion()
{
  return m_kin.isStopped()? false: true;
}

bool LaeRobot::isAlarmed()
{
  bool  bAlarmState = false;

  if( RtDb.m_alarms.m_system.m_uAlarms != LAE_ALARM_NONE )
  {
    bAlarmState = true;
  }
  if( isEStopped() )
  {
    bAlarmState = true;
  }

  return m_bAlarmState;
}

bool LaeRobot::canMove()
{
  // not safe to operate.
  if( !LaeAlarms::isSafeToOperate() )
  {
    return false;
  }
  // emergency stopped
  else if( m_bIsEStopped )
  {
    return false;
  }
  // motors are unpowered
  else if( !areMotorsPowered() )
  {
    return false;
  }
  // we have a go!
  else
  {
    return true;
  }
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Datatbase Methods.
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void LaeRobot::syncDb()
{
  RtDb.m_robotstatus.m_bIsConnected      = m_bIsConnected;
  RtDb.m_robotstatus.m_eRobotMode        = m_eRobotMode;
  RtDb.m_robotstatus.m_bIsEStopped       = m_bIsEStopped;
  RtDb.m_robotstatus.m_bAlarmState       = m_bAlarmState;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Hardare Connection and Configuration Methods.
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int LaeRobot::connSensors()
{
  string  strIdent;
  uint_t  uVerMajor, uVerMinor, uFwVer;
  int     rc;

  //
  // Connect to the CleanFlight Intertial Measrurement Unit.
  //
  if( (rc = m_imu.open(LaeDevIMU, LaeBaudRateIMU)) < 0 )
  {
    LOGERROR("%s: Failed to open IMU at %d baud.", LaeDevIMU, LaeBaudRateIMU);
  }

  else if( (rc = m_imu.readIdentity(strIdent)) < 0 )
  {
    LOGERROR("%s: Failed to read IMU identity.", LaeDevIMU);
  }

  if( rc == LAE_OK )
  {
    LOGDIAG2("Connected to IMU %s.", strIdent.c_str());
  }
  else
  {
    m_imu.blacklist();
    LOGERROR("IMU is blacklisted from the suite of robot sensors.");
    rc = LAE_OK;
  }

  //
  // Connect to the Range Sensor Group.
  //
  if( (rc = m_range.setInterface(m_descLaelaps.getProdHwVer())) < 0 )
  {
    LOGERROR("Failed to set range sensor group interface.");
  }

  else if((rc = m_range.getInterfaceVersion(uVerMajor, uVerMinor, uFwVer)) < 0)
  {
    LOGERROR("Failed to read range sensor group interface version.");
  }

  if( rc == LAE_OK )
  {
    m_range.clearSensedData();

    LOGDIAG2("Connected to Range Sensor Group v%u.%u, fwver=%u.",
        uVerMajor, uVerMinor, uFwVer);
  }
  else
  {
    m_range.blacklist();
    LOGERROR("Range sensors are blacklisted from the suite of robot sensors.");
    rc = LAE_OK;
  }

  return rc;
}

int LaeRobot::connWatchDog()
{
  uint_t  uFwVer;

  if( m_watchdog.cmdGetFwVersion(uFwVer) < 0 )
  {
    LOGWARN("WatchDog: Failed to get firmware version.");
  }
  else
  {
    LOGDIAG3("Connected to WatchDog sub-processor, fwver=%u.", uFwVer);

    // sync watchdog state with subprocessor
    m_watchdog.sync();
  }

  return LAE_OK;
}

int LaeRobot::connMotorControllers(const std::string &strDevMotorCtlrs,
                                   const int         nBaudRate)
{
  int     rc;

  rc = m_kin.open(strDevMotorCtlrs, nBaudRate,
                  LaeWd::enableMotorCtlrs, &m_watchdog);

  if( rc < 0 )
  {
    LOGERROR("%s: Failed to open at motor controllers at %d baud.",
        strDevMotorCtlrs.c_str(), nBaudRate);
  }

  LOGDIAG2("Created front and rear motor controller interfaces.");

  return rc;
}

int LaeRobot::configForOperation()
{
  int     rc;                 // return code

  //
  // Configure watchdog from product description
  //
  if( (rc = m_watchdog.configure(m_tunes)) != LAE_OK )
  {
    LOGERROR("Failed to configure WatchDog.");
  }

  //
  // Configure watchdog from tunable parameters
  //
  else if( (rc = m_watchdog.configure(m_tunes)) != LAE_OK )
  {
    LOGERROR("Failed to tune WatchDog.");
  }

  //
  // Configure kinodynamics from product description.
  //
  else if( (rc = m_kin.configure(m_descLaelaps)) != LAE_OK )
  {
    LOGERROR("Failed to configure kinodynamics.");
  }

  //
  // Configure kinodynamics from tunable parameters.
  //
  else if( (rc = m_kin.configure(m_tunes)) != LAE_OK )
  {
    LOGERROR("Failed to tune kinodynamics.");
  }

  //
  // Configure IMU from product description.
  //
  else if( (rc = m_imu.configure(m_descLaelaps)) != LAE_OK )
  {
    LOGERROR("Failed to configure IMU.");
  }

  //
  // Configure IMU from tunable parameters.
  //
  else if( (rc = m_imu.configure(m_tunes)) != LAE_OK )
  {
    LOGERROR("Failed to tune IMU.");
  }

  //
  // Configure range sensors from product description.
  //
  else if( (rc = m_range.configure(m_descLaelaps)) != LAE_OK )
  {
    LOGERROR("Failed to configure IMU.");
  }

  //
  // Configure range sensors from tunable parameters.
  //
  else if( (rc = m_range.configure(m_tunes)) != LAE_OK )
  {
    LOGERROR("Failed to configure range sensor group.");
  }

  //
  // Good
  //
  else
  {
    //
    // Enable power to top deck battery and regulated 5V power outputs.
    // Note: Moved to watchdog subprocessor.
    //
    //m_powerBatt.enable();
    //m_power5V.enable();

    rc = LAE_OK;
  }

  LOGDIAG2("Configured for operation.");

  return rc;
}

int LaeRobot::startCoreThreads()
{
  int     nPriority;
  double  fHz;
  int     rc;

  //
  // WatchDog thread.
  //
  nPriority = LaeThreadWd::ThreadWdPrioDft;
  fHz       = LaeThreadWd::optimizeHz(m_tunes.getWatchDogTimeout());

  if( (rc = startThread(&m_threadWatchDog, nPriority, fHz)) != LAE_OK )
  {
    return rc;
  }

  //
  // Time-of-Flight range sensors thread.
  //
  nPriority = LaeThreadRange::ThreadRangePrioDft;
  fHz       = m_tunes.getRangeHz();

  if( (rc = startThread(&m_threadRange, nPriority, fHz)) != LAE_OK )
  {
    return rc;
  }

  //
  // Inertia Measurement Unit thread.
  //
  nPriority = LaeThreadImu::ThreadImuPrioDft;
  fHz       = m_tunes.getImuHz();

  if( (rc = startThread(&m_threadImu, nPriority, fHz)) != LAE_OK )
  {
    return rc;
  }

  //
  // Kinodynamics thread.
  //
  nPriority = LaeThreadKin::ThreadKinPrioDft;
  fHz       = m_tunes.getKinematicsHz();

  if( (rc = startThread(&m_threadKin, nPriority, fHz)) != LAE_OK )
  {
    return rc;
  }

  return LAE_OK;
}

int LaeRobot::startThread(LaeThread *pThread, int nPriority, double fHz)
{
  string  strName;
  int     rc;

  strName = pThread->getThreadName();

  if( (rc = pThread->createThread(nPriority)) < 0 )
  {
    LOGERROR("%s thread: Failed to create.", strName.c_str());
  }
  else if( (rc = pThread->runThread(fHz)) < 0 )
  {
    LOGERROR("%s thread: Failed to start.", strName.c_str());
  }
  else
  {
    LOGDIAG2("%s thread started at %.3lfHz with priority %d.",
        strName.c_str(), fHz, nPriority); 
    rc = LAE_OK;
  }

  return rc;
}

int LaeRobot::runAsyncJob()
{
  int   rc;

  if( (rc = m_threadAsync.createThread(m_pAsyncJob)) == LAE_OK )
  {
    m_threadAsync.runThread();
  }

  return rc;
}

void LaeRobot::cancelAsyncJob()
{
  m_threadAsync.terminateThread();
}

LaeAsyncJob::JobState LaeRobot::getAsyncJobState()
{
  if( m_pAsyncJob != NULL )
  {
    return m_pAsyncJob->getState();
  }
  else
  {
    return LaeAsyncJob::JobStateNoJob;
  }
}

int LaeRobot::getAsyncJobRc()
{
  if( m_pAsyncJob != NULL )
  {
    m_pAsyncJob->getRc();
  }
  else
  {
    return -LAE_ECODE_NO_RSRC;
  }
}
