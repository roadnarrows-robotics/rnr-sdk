////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Library:   liblaelaps
//
// File:      laeKin.cxx
//
/*! \file
 *
 * $LastChangedDate: 2016-04-12 14:32:48 -0600 (Tue, 12 Apr 2016) $
 * $Rev: 4385 $
 *
 * \brief The Laelaps kinematics and dynamics class implemenation.
 *
 * \par Copyright:
 * (C) 2015-2016.  RoadNarrows
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
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

#include <sys/time.h>
#include <time.h>
#include <limits.h>
#include <unistd.h>
#include <stdio.h>
#include <pthread.h>
#include <sched.h>
#include <math.h>

#include <vector>
#include <map>

#include "rnr/rnrconfig.h"
#include "rnr/gpio.h"
#include "rnr/log.h"

#include "Laelaps/RoboClaw.h"

#include "Laelaps/laelaps.h"
#include "Laelaps/laeTune.h"
#include "Laelaps/laeUtils.h"
#include "Laelaps/laeSysDev.h"
#include "Laelaps/laeMotor.h"
#include "Laelaps/laeDb.h"
#include "Laelaps/laeTraj.h"
#include "Laelaps/laePowertrain.h"
#include "Laelaps/laePlatform.h"
#include "Laelaps/laeKin.h"

using namespace std;
using namespace laelaps;
using namespace motor::roboclaw;

//
// RDK TODO Put these parameters in tunes
//
static u32_t MoveAccel = 100000;
static u32_t StopDecel = 100000;

static int  SpeedReallyZero = 5; // motors speed is really zero

// ---------------------------------------------------------------------------
// LaeKinematics Class
// ---------------------------------------------------------------------------

LaeKinematics::LaeKinematics()
{
  int   nCtlr;    // controller id

  // motor controllers i/f
  for(nCtlr=0; nCtlr<LaeNumMotorCtlrs; ++nCtlr)
  {
    m_pMotorCtlr[nCtlr] = NULL;
  }

  // actions
  m_pAction = new LaeKinAction(*this);

  pthread_mutex_init(&m_mutex, NULL);

  m_fnEnableMotorCtlrs  = NULL;
  m_pEnableArg          = NULL;
  m_bIsEnabled          = false;
  m_bAreMotorsPowered   = false;
  m_bIsStopped          = true;
}

LaeKinematics::~LaeKinematics()
{
  int   nCtlr;

  if( m_pAction != NULL )
  {
    delete m_pAction;
    m_pAction = NULL;
  }

  // motor controllers i/f
  for(nCtlr=0; nCtlr<LaeNumMotorCtlrs; ++nCtlr)
  {
    if( m_pMotorCtlr[nCtlr] != NULL )
    {
      delete m_pMotorCtlr[nCtlr];
      m_pMotorCtlr[nCtlr] = NULL;
    }
  }

  close();

  pthread_mutex_destroy(&m_mutex);
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Basic I/O
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int LaeKinematics::open(const std::string &strDevMotorCtlrs,
                        const int         nBaudRate,
                        int (*fnEnable)(void *, bool),
                        void *pEnableArg)
{
  string  strDevName;   // real device names
  int     nCtlr;        // motor controller id

  m_fnEnableMotorCtlrs  = fnEnable;
  m_pEnableArg          = pEnableArg;
  m_bIsEnabled          = false;

  // Enable power to motor controllers
  enableMotorCtlrs();

  if( !m_bIsEnabled )
  {
    LOGERROR("Failed to enable power to motor controllers.");
    return -LAE_ECODE_IO;
  }

#ifdef LAE_DEPRECATED
  // Open the motor controller chip select via gpio
  if( m_csMotorCtlrs.open(LaeGpioMotorCtlrCs) < 0 )
  {
    LOGSYSERROR("Failed to open motor controller chip select on GPIO pin %d.",
        LaeGpioMotorCtlrCs);
    return -LAE_ECODE_MOT_CTLR;
  }

  LOGDIAG3("Opened motor controllers chip select signal on GPIO %d.",
        LaeGpioMotorCtlrCs);
#endif // LAE_DEPRECATED

  // Get the real device name, not any symbolic links.
  strDevName  = getRealDeviceName(strDevMotorCtlrs);

  // Open the motor controller.
  if( m_commMotorCtlrs.open(strDevName, nBaudRate) < 0 )
  {
    LOGERROR("%s: Failed to open motor controller port at %d baud.",
        strDevName.c_str(), nBaudRate);
    return -LAE_ECODE_MOT_CTLR;
  }

   LOGDIAG3("Open motor controllers serial communication on %s@%d.",
        strDevName.c_str(), nBaudRate);

  // 
  // Create front motor controller interface.
  //
  nCtlr = LaeMotorCtlrIdFront;
  m_pMotorCtlr[nCtlr] = 
            new RoboClaw(m_commMotorCtlrs, LaeMotorCtlrAddrFront, LaeKeyFront);
  m_pMotorCtlr[nCtlr]->setMotorDir(LaeMotorLeft, LaeMotorDirNormal);
  m_pMotorCtlr[nCtlr]->setMotorDir(LaeMotorRight, LaeMotorDirNormal);

  // 
  // Create rear motor controller interface.
  //
  nCtlr = LaeMotorCtlrIdRear;
  m_pMotorCtlr[nCtlr]  =
            new RoboClaw(m_commMotorCtlrs, LaeMotorCtlrAddrRear, LaeKeyRear);
  m_pMotorCtlr[nCtlr]->setMotorDir(LaeMotorLeft, LaeMotorDirNormal);
  m_pMotorCtlr[nCtlr]->setMotorDir(LaeMotorRight, LaeMotorDirNormal);

  m_bAreMotorsPowered = false;
  m_bIsStopped        = true;

  RtDb.m_robotstatus.m_bAreMotorsPowered  = m_bAreMotorsPowered;
  RtDb.m_robotstatus.m_bInMotion          = m_bIsStopped? false: true;

  LOGDIAG2("Front and rear motor controllers created on %s@%d.",
        strDevName.c_str(), nBaudRate);
  //LOGDIAG2("Front and rear motor controllers created on %s@%d, cs=%d.",
  //      strDevName.c_str(), nBaudRate, LaeGpioMotorCtlrCs);

  return LAE_OK;
}

int LaeKinematics::close()
{
  if( m_pAction != NULL )
  {
    m_pAction->terminate();
  }

#ifdef LAE_DEPRECATED
  m_csMotorCtlrs.close();     // gpio
#endif // LAE_DEPRECATED

  m_commMotorCtlrs.close();   // on serial device

  disableMotorCtlrs();

  if( m_bIsEnabled )
  {
    LOGWARN("Failed to disable power to motor controllers.");
  }

  return LAE_OK;
}

void LaeKinematics::enableMotorCtlrs()
{
  int   rc;

#ifdef LAE_DEPRECATED
  m_bIsEnabled = m_power.enable();
  return;
#endif // LAE_DEPRECATED

  //
  // Enable power to motor controllers (and motors).
  //
  if( m_fnEnableMotorCtlrs != NULL )
  {
    if( (rc = m_fnEnableMotorCtlrs(m_pEnableArg, true)) == LAE_OK )
    {
      m_bIsEnabled = true;
    }
  }

  //
  // In old versions of Laelaps, motors are always enabled.
  //
  else
  {
    m_bIsEnabled = true;
  }
}

void LaeKinematics::disableMotorCtlrs()
{
  int   rc;

#ifdef LAE_DEPRECATED
  m_bIsEnabled = m_power.disable();
  return;
#endif // LAE_DEPRECATED

  //
  // Disable power to motor controllers (and motors).
  //
  if( m_fnEnableMotorCtlrs != NULL )
  {
    if( (rc = m_fnEnableMotorCtlrs(m_pEnableArg, false)) == LAE_OK )
    {
      m_bIsEnabled        = false;
      m_bAreMotorsPowered = false;
      m_bIsStopped        = true;

      RtDb.m_robotstatus.m_bAreMotorsPowered  = m_bAreMotorsPowered;
      RtDb.m_robotstatus.m_bInMotion          = m_bIsStopped? false: true;
    }
  }

  //
  // In old versions of Laelaps, motors are always enabled. So only "disable"
  // in software.
  //
  else
  {
    m_bIsEnabled = false;
  }
}

void LaeKinematics::resyncComm()
{
  LOGDIAG3("Resynchronizing motor controller serial communication.");

  m_bIsEnabled = false;

  enableMotorCtlrs();

  stop();

  resetOdometers();
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Configuration
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int LaeKinematics::configure(const LaeDesc &desc)
{
  LaeDesc::MapDescPowertrain::const_iterator iter;
  LaePowertrain   train;
  int             rc = LAE_OK;

  //
  // Powertrains
  //
  for(iter = desc.m_mapDescPowertrain.begin();
      iter != desc.m_mapDescPowertrain.end();
      ++iter)
  {
    // configure powertrain kinematics
    train.configure(*iter->second);

    //
    // Sanity checks.
    //

    //
    // Add to kinematic chain.
    //
    m_kinPowertrains[train.m_strName] = train;
  }

  //
  // Robot platform
  //
  m_kinPlatform.configure(*desc.m_pDescBase);

  return rc;
}

int LaeKinematics::configure(const LaeTunes &tunes)
{
  int     nCtlr;              // motor controller id
  int     rc;                 // return code

  for(nCtlr = 0; nCtlr < LaeNumMotorCtlrs; ++nCtlr)
  {
    if( (rc = configureMotorController(tunes, nCtlr)) != LAE_OK )
    {
      break;
    }
  }

  m_kinPlatform.configure(tunes);

  return rc;
}

int LaeKinematics::configureMotorController(const LaeTunes &tunes, int nCtlr)
{
  bool    bNvmSave;           // do [not] save changes to ctlr's eeprom
  int     rc;                 // return code

  bNvmSave   = false;

  //
  // Make sure motor controllers are configured correctly.
  //
  // All operations are read, conditional writes.
  // Both controllers have identical config.
  //

  // battery cutoffs
  if( (rc = configMotorCtlrBatteryCutoffs(tunes, nCtlr, bNvmSave)) != LAE_OK )
  {
    return rc;
  }

  // logic cutoffs
  if( (rc = configMotorCtlrLogicCutoffs(tunes, nCtlr, bNvmSave)) != LAE_OK )
  {
    return rc;
  }

  // encoder modes
  if( (rc = configMotorCtlrEncoderModes(tunes, nCtlr, bNvmSave)) != LAE_OK )
  {
    return rc;
  }

  //
  // Configure the motor controller's powertrain pair.
  //
  if( (rc = configurePtp(tunes, nCtlr, bNvmSave)) != LAE_OK )
  {
    return rc;
  }

  //
  // If any settings, that can be saved, have changed, then save to
  // non-volatile EEPROM.
  //
  if( bNvmSave )
  {
    saveConfigToNvm(nCtlr);
  }

  //
  // Set controller state values.
  //
  resetMotorCtlrEncoders(nCtlr);

  return LAE_OK;
}

int LaeKinematics::configMotorCtlrBatteryCutoffs(const LaeTunes &tunes,
                                                 int            nCtlr,
                                                 bool           &bNvmSave)
{
  string strTag = LaeDesc::prettyMotorCtlrName(nCtlr,
                                    m_pMotorCtlr[nCtlr]->getAddress());

  double  fMin, fMax;         // working min,max values
  double  fMinMin, fMaxMax;   // working min min,max max values
  int     rc;                 // return code

  //
  // Read the main battery voltage cutoff settings (min, max) and set if
  // necessary.
  //
 
  LOGDIAG3("%s: Verifying main battery cutoff voltages...", strTag.c_str());

  //
  // Read current settings.
  //
  rc = m_pMotorCtlr[nCtlr]->cmdReadMainBattCutoffs(fMin, fMax);

  if( rc != OK )
  {
    LOGWARN("%s: Failed to read main battery cutoff voltages.", strTag.c_str());
    return LAE_OK;
  }

  LOGDIAG3("   Current settings: [%.1lf, %.1lf]", fMin, fMax);

  // Get supported full main battery range.
  m_pMotorCtlr[nCtlr]->getMainBattCutoffRange(fMinMin, fMaxMax);

  //
  // Change settings. The settings are always reset to defaults on power-up,
  // so cannot save to non-volatile memory.
  //
  if( (fMin != fMinMin) || (fMax != fMaxMax) )
  {
    rc = m_pMotorCtlr[nCtlr]->cmdSetMainBattCutoffs(fMinMin, fMaxMax);

    if( rc != OK )
    {
      LOGWARN("%s: Failed to set main battery cutoff voltages to "
            "[%.1lf, %.1lf].",
          strTag.c_str(), fMinMin, fMaxMax);
    }
    else
    {
      LOGDIAG3("   Changed settings: [%.1lf, %.1lf]", fMinMin, fMaxMax);
    }
  }

  //
  // The settings are good as configured.
  //
  else
  {
    LOGDIAG3("   Good.");
  }

  return LAE_OK;
}

int LaeKinematics::configMotorCtlrLogicCutoffs(const LaeTunes &tunes,
                                               int            nCtlr,
                                               bool           &bNvmSave)
{
  string strTag = LaeDesc::prettyMotorCtlrName(nCtlr,
                                    m_pMotorCtlr[nCtlr]->getAddress());

  double  fMin, fMax;         // working min,max values
  double  fMinMin, fMaxMax;   // working min min,max max values
  int     rc;                 // return code

  //
  // Read the logic 'battery' voltage cutoff settings (min, max) and set if
  // necessary.
  //
  // Currently, the LB-/LB+ terminals are not connected, and the logic uses
  // the main battery.
  //
 
  LOGDIAG3("%s: Verifying logic cutoff voltages...", strTag.c_str());

  //
  // Read current settings.
  //
  rc = m_pMotorCtlr[nCtlr]->cmdReadLogicCutoffs(fMin, fMax);

  if( rc != OK )
  {
    LOGWARN("%s: Failed to read logic cutoff voltages.", strTag.c_str());
    return LAE_OK;
  }

  LOGDIAG3("   Current settings: [%.1lf, %.1lf]", fMin, fMax);

  // Get supported full logic range.
  m_pMotorCtlr[nCtlr]->getLogicCutoffRange(fMinMin, fMaxMax);

  //
  // Change settings. The settings are always reset to defaults on power-up,
  // so cannot save to non-volatile memory.
  //
  if( (fMin != fMinMin) || (fMax != fMaxMax) )
  {
    rc = m_pMotorCtlr[nCtlr]->cmdSetLogicCutoffs(fMinMin, fMaxMax);

    if( rc != OK )
    {
      LOGWARN("%s: Failed to set logic cutoff voltages to [%.1lf, %.1lf].",
            strTag.c_str(), fMinMin, fMaxMax);
    }
    else
    {
      LOGDIAG3("   Changed settings: [%.1lf, %.1lf]", fMinMin, fMaxMax);
    }
  }

  //
  // The settings are good as configured.
  //
  else
  {
    LOGDIAG3("   Good.");
  }

  return LAE_OK;
}

int LaeKinematics::configMotorCtlrEncoderModes(const LaeTunes &tunes,
                                               int            nCtlr,
                                               bool           &bNvmSave)
{
  string strTag = LaeDesc::prettyMotorCtlrName(nCtlr,
                                    m_pMotorCtlr[nCtlr]->getAddress());

  byte_t  byMode1, byMode2;   // motor encoder modes
  int     rc;                 // return code

  //
  // Read both motors' encoder modes and set if necessary. Laelaps uses
  // quadrature encoding. This is a required configuration for operation. 
  //

  LOGDIAG3("%s: Verifying motor quadrature encoder modes...", strTag.c_str());

  //
  // Read current settings.
  //
  rc = m_pMotorCtlr[nCtlr]->cmdReadEncoderMode(byMode1, byMode2);

  if( rc != OK )
  {
    LOGERROR("%s: Failed to read motor encoder modes.", strTag.c_str());
    return -LAE_ECODE_MOT_CTLR;
  }

  LOGDIAG3("   Current settings: [0x%02x, 0x%02x]", byMode1, byMode2);

  //
  // Change settings. The new settings should also be saved to non-volatile
  // memory.
  //
  if( (byMode1 != ParamEncModeQuad) || (byMode2 != ParamEncModeQuad) )
  {
    rc = m_pMotorCtlr[nCtlr]->cmdSetEncoderMode2(ParamEncModeQuad,
                                                   ParamEncModeQuad);

    if( rc != OK )
    {
      LOGERROR("%s: Failed to set motor encoders to quadrature encoding "
            "[0x%02x, 0x%02x].",
            strTag.c_str(), ParamEncModeQuad, ParamEncModeQuad);
      return -LAE_ECODE_MOT_CTLR;
    }

    LOGDIAG3("   Changed settings: [0x%02x, 0x%02x]",
            ParamEncModeQuad, ParamEncModeQuad);

    bNvmSave = true;
  }

  //
  // The settings are good as configured.
  //
  else
  {
      LOGDIAG3("   Good.");
  }

  return LAE_OK;
}

int LaeKinematics::resetMotorCtlrEncoders(int nCtlr)
{
  string strTag = LaeDesc::prettyMotorCtlrName(nCtlr,
                                    m_pMotorCtlr[nCtlr]->getAddress());

  int rc;   // return code

  rc = m_pMotorCtlr[nCtlr]->cmdResetQEncoders();

  if( rc == OK )
  {
    LOGDIAG3("%s: Encoders reset.", strTag.c_str());
    return LAE_OK;
  }
  else
  {
    LOGWARN("%s: Failed to reset encoders.", strTag.c_str());
    return -LAE_ECODE_MOT_CTLR;
  }
}

int LaeKinematics::configurePtp(const LaeTunes  &tunes,
                                int             nCtlr,
                                bool            &bNvmSave)
{
  LaeMapPowertrain::iterator pos;

  int     nMotor;                               // motor index
  int     rc;                                   // return code

  string strKey = m_pMotorCtlr[nCtlr]->getNameId();

  for(nMotor = LaeMotorLeft; nMotor < LaeNumMotorsPerCtlr; ++nMotor)
  {
    strKey = LaePowertrain::toKey(nCtlr, nMotor);

    // - - - -
    // Update powertrain attributes with tuning parameters values.
    // - - - -
    if( (pos = m_kinPowertrains.find(strKey)) == m_kinPowertrains.end() )
    {
      LOGWARN("Cannot find powertrain %s - ignoring.", strKey.c_str());
      continue;
    }

    // configure parameters for powertrain kinodynamics
    pos->second.configure(tunes);

    // velocity PID
    if( (rc = configMotorVelocityPid(tunes, pos->second, bNvmSave)) != LAE_OK )
    {
      return rc;
    }

    // maximum ampere limit
    if( (rc = configMotorMaxAmpLimit(tunes, pos->second, bNvmSave)) != LAE_OK )
    {
      return rc;
    }

    //
    // RDK TODO 63,64 Read motor position PID
    // RDK TODO 61,62 Set motor position PID
  }

  return LAE_OK;
}

int LaeKinematics::configMotorVelocityPid(const LaeTunes &tunes,
                                          LaePowertrain  &powertrain,
                                          bool           &bNvmSave)
{
  string  strKey    = powertrain.m_strName;
  int     nCtlrId   = powertrain.m_attr.m_nMotorCtlrId;
  int     nMotorId  = powertrain.m_attr.m_nMotorId;
  int     nMotor    = powertrain.m_attr.m_nMotorIndex;
  byte_t  addr      = m_pMotorCtlr[nCtlrId]->getAddress();
  string  strTag    = LaeDesc::prettyMotorName(nCtlrId, addr, nMotorId);

  u32_t   uKp, uKi, uKd, uQpps;                 // existing PID settings.
  double  fTuneKp, fTuneKi, fTuneKd;            // tune PID parameters
  u32_t   uTuneKp, uTuneKi, uTuneKd, uTuneQpps; // converted tune target PID
  int     rc;                                   // return code

  //
  // Read motor's velocity PID parameters, and set if necessary.
  //
   
  LOGDIAG3("%s: Verifying velocity PID parameters...", strTag.c_str());

  //
  // Read current settings.
  //
  rc = m_pMotorCtlr[nCtlrId]->cmdReadVelocityPidConst(nMotor,
                                                      uKp, uKi, uKd, uQpps);

  if( rc != OK )
  {
    LOGWARN("%s: Failed to read velocity PID constants.", strTag.c_str());
    return LAE_OK;
  }

  fTuneKp = (double)uKp / (double)ParamVelPidCvt;
  fTuneKi = (double)uKi / (double)ParamVelPidCvt;
  fTuneKd = (double)uKd / (double)ParamVelPidCvt;

  LOGDIAG3("   Current settings: "
     "Kp=%lf (raw=0x%08x), Ki=%lf (raw=0x%08x), Kd=%lf (raw=0x%08x), Qpps=%u.",
     fTuneKp, uKp, fTuneKi, uKi, fTuneKd, uKd, uQpps);

  // Get the tune parameters in "natural" floating-point representation
  tunes.getVelPidKParams(strKey, fTuneKp, fTuneKi, fTuneKd);

  // Convert to motor controller speak.
  uTuneKp   = (u32_t)(fTuneKp * (double)ParamVelPidCvt);
  uTuneKi   = (u32_t)(fTuneKi * (double)ParamVelPidCvt);
  uTuneKd   = (u32_t)(fTuneKd * (double)ParamVelPidCvt);
  uTuneQpps = (u32_t)(powertrain.m_attr.m_uMaxQpps);

  //
  // Change settings. The new settings should also be saved to non-volatile
  // memory.
  //
  if( (uKp != uTuneKp) || (uKi != uTuneKi) || (uKd != uTuneKd) ||
          (uQpps != uTuneQpps) )
  {
    rc = m_pMotorCtlr[nCtlrId]->cmdSetVelocityPidConst(nMotor,
                                      uTuneKp, uTuneKi, uTuneKd, uTuneQpps);

    if( rc != OK)
    {
      LOGWARN("%s: Failed to set velocity PID constants.", strTag.c_str());
    }
    else
    {
      LOGDIAG3("   Changed settings: "
              "Kp=%lf (raw=0x%08x), Ki=%lf (raw=0x%08x), Kd=%lf (raw=0x%08x), "
              "Qpps=%u.",
            fTuneKp, uTuneKp, fTuneKi, uTuneKi, fTuneKd, uTuneKd, uTuneQpps);

      bNvmSave = true;
    }
  }

  //
  // The settings are good as configured.
  //
  else
  {
    LOGDIAG3("   Good.");
  }

  return LAE_OK;
}

int LaeKinematics::configMotorMaxAmpLimit(const LaeTunes &tunes,
                                          LaePowertrain  &powertrain,
                                          bool           &bNvmSave)
{
  string  strKey    = powertrain.m_strName;
  int     nCtlrId   = powertrain.m_attr.m_nMotorCtlrId;
  int     nMotorId  = powertrain.m_attr.m_nMotorId;
  int     nMotor    = powertrain.m_attr.m_nMotorIndex;
  byte_t  addr      = m_pMotorCtlr[nCtlrId]->getAddress();
  string  strTag    = LaeDesc::prettyMotorName(nCtlrId, addr, nMotorId);

  double  fCurMaxAmps, fMaxAmps;                // max. amp limits
  int     rc;                                   // return code

  //
  // Read motor's maximum current limit, and set if necessary.
  //
  // Note: Not limiting the amps can cause motor burn out.
  //
 
  LOGDIAG3("%s: Verifying maximum amp limit parameter...", strTag.c_str());

  //
  // Read current settings.
  //
  rc = m_pMotorCtlr[nCtlrId]->cmdReadMotorMaxCurrentLimit(nMotor, fCurMaxAmps);

  if( rc != OK )
  {
    LOGERROR("%s: Failed to read maximum current limit.", strTag.c_str());
    return -LAE_ECODE_MOT_CTLR;
  }

  LOGDIAG3("   Current settings: %.2lf amps.", fCurMaxAmps);

  if( RtDb.m_product.m_uProdHwVer == LAE_VERSION(2, 0, 0) )
  {
    fMaxAmps = LaeMotorMaxAmps;
  }
  else
  {
    fMaxAmps = LaeMotorMaxAmps_2_1;
  }

  //
  // Change settings. The new settings should also be saved to non-volatile
  // memory.
  //
  if( fCurMaxAmps != fMaxAmps )
  {
    rc = m_pMotorCtlr[nCtlrId]->cmdSetMotorMaxCurrentLimit(nMotor, fMaxAmps);

    if( rc != OK)
    {
      LOGERROR("%s: Failed to set maximum current limit to %.2lf.",
          strTag.c_str(), fMaxAmps);
      return -LAE_ECODE_MOT_CTLR;
    }
    else
    {
      LOGDIAG3("   Changed settings: %.2lf amps.", fMaxAmps);
      bNvmSave = true;
    }
  }

  //
  // The settings are good as configured.
  //
  else
  {
    LOGDIAG3("   Good.");
  }

  return LAE_OK;
}

int LaeKinematics::saveConfigToNvm(int nCtlrId)
{
  string strTag = LaeDesc::prettyMotorCtlrName(nCtlrId,
                                    m_pMotorCtlr[nCtlrId]->getAddress());

  int   rc;   // return code

  LOGDIAG3("%s: Saving new values to EEPROM.", strTag.c_str());

  rc = m_pMotorCtlr[nCtlrId]->cmdWriteSettingsToEEPROM();

  if( rc == OK )
  {
    LOGDIAG3("  Saved.");
  }
  else
  {
    LOGWARN("%s: Failed to write settings to EEPROM.", strTag.c_str());
  }

  return LAE_OK;
}

int LaeKinematics::reload(const LaeTunes &tunes)
{
  LaeMapPowertrain::iterator iter;    // kinematics chain iterator

  for(iter = m_kinPowertrains.begin(); iter != m_kinPowertrains.end(); ++iter)
  {
    iter->second.reload(tunes);
  }

  m_kinPlatform.reload(tunes);

  return LAE_OK;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Attribute Methods
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

RoboClaw *LaeKinematics::getMotorCtlr(const int nMotorCtlrId)
{
  if( (nMotorCtlrId >= 0) && (nMotorCtlrId < LaeNumMotorCtlrs) )
  {
    return m_pMotorCtlr[nMotorCtlrId];
  }
  else
  {
    return NULL;
  }
}

LaePowertrain *LaeKinematics::getPowertrain(const string &strName)
{
  if( m_kinPowertrains.find(strName) != m_kinPowertrains.end() )
  {
    return &m_kinPowertrains[strName];
  }
  else
  {
    return NULL;
  }
}

LaePowertrain *LaeKinematics::getPowertrain(int nCtlr, int nMotor)
{
  std::string strName = LaePowertrain::toKey(nCtlr, nMotor);

  if( m_kinPowertrains.find(strName) != m_kinPowertrains.end() )
  {
    return &m_kinPowertrains[strName];
  }
  else
  {
    return NULL;
  }
}

LaePowertrain *LaeKinematics::getPowertrain(int nMotorId)
{
  std::string strName = LaePowertrain::toKey(nMotorId);

  if( m_kinPowertrains.find(strName) != m_kinPowertrains.end() )
  {
    return &m_kinPowertrains[strName];
  }
  else
  {
    return NULL;
  }
}

int LaeKinematics::getPowertrainState(const std::string &strName,
                                      LaePowertrainState     &jointState)
{
  LaeMapPowertrain::iterator pos;

  if( (pos = m_kinPowertrains.find(strName)) != m_kinPowertrains.end() )
  {
    jointState = pos->second.m_state;
    return LAE_OK;
  }
  else
  {
    return -LAE_ECODE_BAD_VAL;
  }
}

bool LaeKinematics::isStopped()
{
  bool  bIsStopped;                 // all motors are [not] stopped
  LaeMapPowertrain::iterator iter;  // kinematics chain iterator

  for(iter = m_kinPowertrains.begin(), bIsStopped = true;
      (iter != m_kinPowertrains.end()) && bIsStopped;
      ++iter)
  {
    if( iabs(iter->second.m_state.m_nSpeed) > SpeedReallyZero )
    {
      bIsStopped = false;
    }
  }

  m_bIsStopped = bIsStopped;
  RtDb.m_robotstatus.m_bInMotion = m_bIsStopped? false: true;

  return m_bIsStopped;
}

bool LaeKinematics::isStopped(const std::string &strName)
{
  LaeMapPowertrain::iterator pos;

  if( (pos = m_kinPowertrains.find(strName)) != m_kinPowertrains.end() )
  {
    return iabs(pos->second.m_state.m_nSpeed) <= SpeedReallyZero;
  }
  else // non-existent joints don't move
  {
    return true;
  }
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Kinodynamic Methods
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int LaeKinematics::resetOdometers()
{
  int             nCtlr;    // motor controller id
  int             nMotor;   // motor index
  LaePowertrain  *pTrain;   // working powertrain
  int             n;        // number of powertrain odometers actually resetted

  lock();

  n = 0;

  if( m_bIsEnabled )
  {
    m_pAction->terminate();

    for(nCtlr = 0; nCtlr < LaeNumMotorCtlrs; ++nCtlr)
    {
      if( m_pMotorCtlr[nCtlr]->cmdResetQEncoders() == OK )
      {
        for(nMotor = 0; nMotor < LaeNumMotorsPerCtlr; ++nMotor)
        {
          if( (pTrain = getPowertrain(nCtlr, nMotor)) != NULL )
          {
            pTrain->resetOdometer();
            ++n;
          }
        }
      }
    }

    m_kinPlatform.resetOdometer();
  }

  unlock();

  return n;
}

void LaeKinematics::estop()
{
  int     nCtlr;    // motor controller id

  lock();

  if( m_bIsEnabled )
  {
    m_pAction->terminate();

    for(nCtlr = 0; nCtlr < LaeNumMotorCtlrs; ++nCtlr)
    {
      m_pMotorCtlr[nCtlr]->cmdEStop();
    }

    disableMotorCtlrs();
  }

  unlock();
}
  
void LaeKinematics::resetEStop()
{
  lock();

  enableMotorCtlrs();

  unlock();
}

int LaeKinematics::release()
{
  int   nCtlr;  // motor controller id
  int   rc;     // return code

  lock();

  if( m_bIsEnabled )
  {
    m_pAction->terminate();

    for(nCtlr = 0; nCtlr < LaeNumMotorCtlrs; ++nCtlr)
    {
      m_pMotorCtlr[nCtlr]->cmdDutyDrive2(0.0, 0.0);
    }

    m_bAreMotorsPowered = false;
    RtDb.m_robotstatus.m_bAreMotorsPowered = m_bAreMotorsPowered;

    rc = LAE_OK;
  }
  else
  {
    rc = -LAE_ECODE_BAD_OP;
  }

  unlock();

  return rc;
}
  
int LaeKinematics::stop()
{
  int   nCtlr;  // motor controller id
  int   rc;     // return code

  lock();

  if( m_bIsEnabled )
  {
    m_pAction->terminate();

    for(nCtlr = 0; nCtlr < LaeNumMotorCtlrs; ++nCtlr)
    {
      //m_pMotorCtlr[nCtlr]->cmdQStop();
      m_pMotorCtlr[nCtlr]->cmdStopWithDecel(StopDecel);
    }

    m_bAreMotorsPowered = true;
    RtDb.m_robotstatus.m_bAreMotorsPowered = m_bAreMotorsPowered;

    rc = LAE_OK;
  }
  else
  {
    rc = -LAE_ECODE_BAD_OP;
  }

  unlock();

  return rc;
}

int LaeKinematics::stop(const vector<string> &vecNames)
{
  vector<string>::const_iterator  iter; // joint name iterator

  LaeMapPowertrain::iterator  pos;    // kinematics chain position
  int                         n;      // number of actual joints stopped
  int                         nCtlr;  // motor controller id
  int                         nMotor; // motor index
  int                         rc;     // return code

  lock();

  n = 0;

  if( m_bIsEnabled )
  {
    m_pAction->terminate();

    for(iter = vecNames.begin(); iter != vecNames.end(); ++iter)
    {
      if( (pos = m_kinPowertrains.find(*iter)) != m_kinPowertrains.end() )
      {
        nCtlr  = pos->second.getMotorCtlrId();
        nMotor = pos->second.getMotorIndex();

        //if( m_pMotorCtlr[nCtlr]->cmdStop(nMotor) == OK )
        if( m_pMotorCtlr[nCtlr]->cmdStopWithDecel(nMotor, StopDecel) == OK )
        {
          ++n;
        }
      }
    }

    m_bAreMotorsPowered = true;
    RtDb.m_robotstatus.m_bAreMotorsPowered = m_bAreMotorsPowered;

    rc = LAE_OK;
  }
  else
  {
    rc = -LAE_ECODE_BAD_OP;
  }

  unlock();

  return n;
}

int LaeKinematics::stop(const string &strName)
{
  LaeMapPowertrain::iterator  pos;    // kinematics chain position
  int                         nCtlr;  // motor controller id
  int                         nMotor; // motor index
  int                         rc;     // return code

  lock();

  if( m_bIsEnabled )
  {
    m_pAction->terminate();

    if( (pos = m_kinPowertrains.find(strName)) != m_kinPowertrains.end() )
    {
      nCtlr  = pos->second.getMotorCtlrId();
      nMotor = pos->second.getMotorIndex();

      //rc = m_pMotorCtlr[nCtlr]->cmdStop(nMotor);
      rc = m_pMotorCtlr[nCtlr]->cmdStopWithDecel(nMotor, StopDecel);

      if( rc == OK )
      {
        m_bAreMotorsPowered = true;
        RtDb.m_robotstatus.m_bAreMotorsPowered = m_bAreMotorsPowered;
        rc = LAE_OK;
      }
      else
      {
        rc = -LAE_ECODE_MOT_CTLR;
      }
    }
    else
    {
      LOGWARN("Powertrain %s: Not found.", strName.c_str());
      rc = -LAE_ECODE_BAD_VAL;
    }
  }
  else
  {
    rc = -LAE_ECODE_BAD_OP;
  }

  unlock();

  return rc;
}

int LaeKinematics::waitForAllStop(double fSeconds)
{
  static const useconds_t WaitDt  = 100;  // usec wait between tests

  int nMaxTries = (int)ceil(fSeconds * MILLION / (double)WaitDt);

  for(int i = 0; i < nMaxTries; ++i)
  {
    if( isStopped() )
    {
      return LAE_OK;
    }
    usleep(WaitDt);
  }

  LOGERROR("Timed out waiting for %.2lf seconds for all joints to stop.",
      fSeconds);

  return -LAE_ECODE_TIMEDOUT;
}

int LaeKinematics::setGoalVelocities(const LaeMapVelocity &velocity)
{
  LaeKinActionVelocity *pAction;  // velocity action pointer
  int                   rc;       // return code

  lock();

  if( m_bIsEnabled )
  {
    //
    // Current action is not a velocity action. Terminate the action and
    // create an this new action.
    //
    if( m_pAction->getActionType() != LaeKinAction::ActionTypeVelocity )
    {
      m_pAction = LaeKinAction::replaceAction(*this, m_pAction,
                                            LaeKinAction::ActionTypeVelocity); 
    }

    pAction = (LaeKinActionVelocity *)m_pAction;

    //  update with new goals
    rc = pAction->update(velocity);

    if( pAction->isPlanningRequired() )
    {
      rc = pAction->plan();
    }

    if( pAction->isExecutionRequired() )
    {
      if( (rc = pAction->execute()) == LAE_OK )
      {
        m_bAreMotorsPowered = true;
        RtDb.m_robotstatus.m_bAreMotorsPowered = m_bAreMotorsPowered;
      }
    }
  }
  else
  {
    rc = -LAE_ECODE_BAD_OP;
  }

  unlock();

  return rc;
}

int LaeKinematics::setGoalDutyCycles(const LaeMapDutyCycle &duty)
{
  LaeKinActionDutyCycle  *pAction;  // duty cycle action pointer
  int                     rc;       // return code

  lock();

  if( m_bIsEnabled )
  {
    //
    // Current action is not a velocity action. Terminate the action and
    // create an this new action.
    //
    if( m_pAction->getActionType() != LaeKinAction::ActionTypeDutyCycle )
    {
      m_pAction = LaeKinAction::replaceAction(*this, m_pAction,
                                            LaeKinAction::ActionTypeDutyCycle); 
    }

    pAction = (LaeKinActionDutyCycle *)m_pAction;

    //  update with new goals
    rc = pAction->update(duty);

    if( pAction->isPlanningRequired() )
    {
      rc = pAction->plan();
    }

    if( pAction->isExecutionRequired() )
    {
      if( (rc = pAction->execute()) == LAE_OK )
      {
        m_bAreMotorsPowered = true;
        RtDb.m_robotstatus.m_bAreMotorsPowered = m_bAreMotorsPowered;
      }
    }
  }
  else
  {
    rc = -LAE_ECODE_BAD_OP;
  }

  unlock();

  return rc;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Kinodynamic Thread Operations
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int LaeKinematics::sense()
{
  int   rc;

  // sense real-time kinodynamics
  rc = senseDynamics();

  // add other sensor reads here (e.g. imu, gps, etc).

  return rc;
}

int LaeKinematics::senseDynamics()
{
  int             nCtlr;                        // motor controller id
  int             nMotor;                       // motor index
  double          amp[LaeNumMotorsPerCtlr];     // motor ampere draws
  uint_t          buflen[LaeNumMotorsPerCtlr];  // command buffer lengths
  s64_t           encoder;                      // motor encoder (quad pulses)
  s32_t           speed;                        // motor speed (qpps)
  bool            bIsStopped;                   // all motors are [not] stopped
  LaePowertrain  *pTrain;                       // working powertrain

  if( !m_bIsEnabled )
  {
    return LAE_OK;
  }

  bIsStopped = true;

  //
  // Sense real-time dynamics.
  //
  for(nCtlr = 0; nCtlr < LaeNumMotorCtlrs; ++nCtlr)
  {
    //
    // Board level sensing.
    //
    m_pMotorCtlr[nCtlr]->cmdReadMotorCurrentDraw(amp[0], amp[1]);
    m_pMotorCtlr[nCtlr]->cmdReadCmdBufLengths(buflen[0], buflen[1]);

    //
    // Motor level sensing.
    //
    for(nMotor = 0; nMotor < LaeNumMotorsPerCtlr; ++nMotor)
    {
      m_pMotorCtlr[nCtlr]->cmdReadQEncoder(nMotor, encoder);
      m_pMotorCtlr[nCtlr]->cmdReadQSpeed(nMotor, speed);

      if( (pTrain = getPowertrain(nCtlr, nMotor)) == NULL )
      {
        continue;
      }

      pTrain->updateStateDynamics(encoder, speed, amp[nMotor], buflen[nMotor]);

      if( iabs(speed) > SpeedReallyZero )
      {
        bIsStopped = false;
      }

      RtDb.m_motorctlr[nCtlr].m_fMotorCurrent[nMotor] = amp[nMotor];
    }
  }

  m_kinPlatform.updateStateDynamics(m_kinPowertrains);

  m_bIsStopped = bIsStopped;
  RtDb.m_robotstatus.m_bInMotion = m_bIsStopped? false: true;

  return LAE_OK;
}

int LaeKinematics::react()
{
  return LAE_OK; // RDK TODO
}

int LaeKinematics::plan()
{
  int     rc;

  // motors are not enabled
  if( !m_bIsEnabled )
  {
    return LAE_OK;
  }

  // planning requried, so plan
  else if( m_pAction->isPlanningRequired() )
  {
    rc = m_pAction->plan();
  }

  // nothing to plan
  else
  {
    rc = LAE_OK;
  }

  return rc;
}

int LaeKinematics::act()
{
  int     rc;

  // motors are not enabled
  if( !m_bIsEnabled )
  {
    rc = LAE_OK;
  }

  // action requried, so execute
  else if( m_pAction->isExecutionRequired() )
  {
    rc = m_pAction->execute();
  }

  // nothing to do
  else
  {
    rc = LAE_OK;
  }

  return rc;
}

int LaeKinematics::monitorHealth()
{
  int               nCtlr;          // motor controller id
  int               nMotor;         // motor index
  double            volts;          // main battery voltage
  double            temp;           // board temperature
  uint_t            status;         // board status
  LaePowertrain    *pTrain;         // pointer to joint

  if( !m_bIsEnabled )
  {
    return LAE_OK;
  }

  lock();

  // 
  // Read health data.
  //
  for(nCtlr = 0; nCtlr < LaeNumMotorCtlrs; ++nCtlr)
  {
    m_pMotorCtlr[nCtlr]->cmdReadMainBattVoltage(volts);
    m_pMotorCtlr[nCtlr]->cmdReadBoardTemperature(temp);
    m_pMotorCtlr[nCtlr]->cmdReadStatus(status);

    RtDb.m_motorctlr[nCtlr].m_uStatus         = status;
    RtDb.m_motorctlr[nCtlr].m_fTemperature    = temp;
    RtDb.m_motorctlr[nCtlr].m_fBatteryVoltage = volts;

    for(nMotor = 0; nMotor < LaeNumMotorsPerCtlr; ++nMotor)
    {
      if( (pTrain = getPowertrain(nCtlr, nMotor)) != NULL )
      {
        pTrain->updateHealth(volts, temp, status);
      }
    }

    m_kinPlatform.updateCtlrHealth(nCtlr, volts, temp, status);
  }

  m_kinPlatform.updateHealth(m_kinPowertrains);

  LOGDIAG4("Kinematics Thread: Monitored health of motor controllers.");

  unlock();

  return LAE_OK;
}

void LaeKinematics::exec()
{
  lock();

  //
  // Auto-Repair
  //
  // The motor controllers should be enabled, but the watchdog thread has
  // detected that they are not. This should not happen, so re-enable.
  //
  if( m_bIsEnabled && !RtDb.m_gpio.m_bMotorCtlrEn )
  {
    LOGERROR("Motor controllers have been unexpectedly disaabled. Re-enable.");
    unlock();
    resyncComm();
    lock();
  }

  // sense environment and dynamic state
  sense();

  // react to immediate stimulii
  react();

  // plan motions
  plan();

  // act on planned motions
  act();

  unlock();
}


// ---------------------------------------------------------------------------
// LaeKinAction
// ---------------------------------------------------------------------------

LaeKinAction *LaeKinAction::newAction(LaeKinematics    &kin,
                                      const ActionType eActionType)
{
  switch( eActionType )
  {
    case ActionTypeVelocity:
      return new LaeKinActionVelocity(kin);
    case ActionTypeDutyCycle:
      return new LaeKinActionDutyCycle(kin);
    case ActionTypeNavForDist:        // future
    case ActionTypeNavToPos:          // future
      return new LaeKinAction(kin);
    case ActionTypeIdle:
      return new LaeKinAction(kin);
    default:
      return new LaeKinAction(kin);
  }
}

LaeKinAction *LaeKinAction::replaceAction(LaeKinematics    &kin,
                                          LaeKinAction     *pAction,
                                          const ActionType eNewActionType)
{
  if( pAction != NULL )
  {
    pAction->terminate();
    delete pAction;
  }

  return newAction(kin, eNewActionType);
}


// ---------------------------------------------------------------------------
// LaeKinActionVelocity
// ---------------------------------------------------------------------------

LaeKinActionVelocity::LaeKinActionVelocity(LaeKinematics &kin)
    : LaeKinAction(kin, ActionTypeVelocity)
{
  clear();
}

int LaeKinActionVelocity::update(const LaeMapVelocity &velocity)
{
  LaeMapVelocity::const_iterator iter;
  LaeMapVelocity::iterator       pos;

  // set state to update
  m_eActionState = ActionStateUpdate;

  //
  // Iterate through key,velocity map.
  //
  for(iter = velocity.begin(); iter != velocity.end(); ++iter)
  {
    pos = m_mapGoalVel.find(iter->first);

    // velocity on a new action powertrain
    if( pos == m_mapGoalVel.end() )
    {
      m_mapGoalVel[iter->first] = iter->second;
      m_eActionState = ActionStatePlan;
    }

    // different velocity on an existing action powertrain
    else if( pos->second != iter->second )
    {
      pos->second = iter->second;
      m_eActionState = ActionStatePlan;
    }

    // else same velocity
    
    LOGDIAG3("Wheel %12s velocity = %lf rads/s.",
        iter->first.c_str(), iter->second);
  }

  // updated, but no planning and execution required
  if( m_eActionState == ActionStateUpdate )
  {
    m_eActionState = ActionStateIdle;
  }

  return LAE_OK;
}

int LaeKinActionVelocity::plan()
{
  int                       nCtlr;      // motor controller id
  int                       nMotor;     // motor index
  LaePowertrain            *pTrain;     // working powertrain
  LaeMapVelocity::iterator  goal;       // working goal
  int                       nSpeed;     // motor speed (qpps)

  if( !isPlanningRequired() )
  {
    return LAE_OK;
  }

  LaeKinAction::plan();

  //
  // Iterate through motors and convert velocities to speeds.
  //
  for(nCtlr = 0; nCtlr < LaeNumMotorCtlrs; ++nCtlr)
  {
    for(nMotor = 0; nMotor < LaeNumMotorsPerCtlr; ++nMotor)
    {
      if( (pTrain = m_kin.getPowertrain(nCtlr, nMotor)) == NULL )
      {
        continue;
      }

      goal = m_mapGoalVel.find(pTrain->getName());

      // converted goal velocity speed is target speed
      if( goal != m_mapGoalVel.end() )
      {
        nSpeed = goal->second / pTrain->m_attr.m_fWheelRadsPerPulse;
      }
      // current speed is target speed
      else
      {
        nSpeed = pTrain->m_state.m_nSpeed;
      }

      //
      // New speed
      //
      if( m_speed[nCtlr][nMotor] != nSpeed )
      {
        m_speed[nCtlr][nMotor] = nSpeed;
        m_eActionState = ActionStateExecute;
      }
    }
  }

  // planned, but no execution required
  if( m_eActionState != ActionStateExecute )
  {
    m_eActionState = ActionStateIdle;
  }
}

int LaeKinActionVelocity::execute()
{
  int       nCtlr;
  RoboClaw *pMotorCtlr;
  int       rc = LAE_OK;

  // no execution required
  if( !isExecutionRequired() )
  {
    return LAE_OK;
  }

  LaeKinAction::execute();

  // set motor speeds
  for(nCtlr = 0; nCtlr < LaeNumMotorCtlrs; ++nCtlr)
  {
    pMotorCtlr = m_kin.getMotorCtlr(nCtlr);

    //rc = pMotorCtlr->cmdQDrive2(m_speed[nCtlr][LaeMotorLeft],
    //                            m_speed[nCtlr][LaeMotorRight]);

    rc = pMotorCtlr->cmdQDriveWithAccel(
        m_speed[nCtlr][LaeMotorLeft],  MoveAccel,
        m_speed[nCtlr][LaeMotorRight], MoveAccel);

    if( rc != OK )
    {
      rc = -LAE_ECODE_MOT_CTLR;
      break;
    }
  }

  m_eActionState = ActionStateIdle;

  return rc;
}

int LaeKinActionVelocity::terminate()
{
  LaeMapVelocity::iterator  iter;
  int                       nCtlr;
  RoboClaw                 *pMotorCtlr;

  // already terminated
  if( hasTerminated() )
  {
    return LAE_OK;
  }

  //
  // Stop all motors.
  //
  // Note: Do not call m_kin.stop() here, since it calls this function.
  //
  for(nCtlr = 0; nCtlr < LaeNumMotorCtlrs; ++nCtlr)
  {
    pMotorCtlr = m_kin.getMotorCtlr(nCtlr);
    //pMotorCtlr->cmdQStop();
    pMotorCtlr->cmdStopWithDecel(StopDecel);
  }

  clear();

  LaeKinAction::terminate();

  return LAE_OK;
}

void LaeKinActionVelocity::clear()
{
  m_mapGoalVel.clear();

  for(int nCtlr = 0; nCtlr < LaeNumMotorCtlrs; ++nCtlr)
  {
    for(int nMotor = 0; nMotor < LaeNumMotorsPerCtlr; ++nMotor)
    {
      m_speed[nCtlr][nMotor] = 0;
    }
  }
}


// ---------------------------------------------------------------------------
// LaeKinActionDutyCycle
// ---------------------------------------------------------------------------

LaeKinActionDutyCycle::LaeKinActionDutyCycle(LaeKinematics &kin)
    : LaeKinAction(kin, ActionTypeDutyCycle)
{
}

int LaeKinActionDutyCycle::update(const LaeMapDutyCycle &duty)
{
  LaeMapDutyCycle::const_iterator iter;
  LaeMapDutyCycle::iterator       pos;

  // set state to update
  m_eActionState = ActionStateUpdate;

  //
  // Iterate through key,duty map.
  //
  for(iter = duty.begin(); iter != duty.end(); ++iter)
  {
    pos = m_mapGoalDutyCycle.find(iter->first);

    // duty on a new action powertrain
    if( pos == m_mapGoalDutyCycle.end() )
    {
      m_mapGoalDutyCycle[iter->first] = iter->second;
      m_eActionState = ActionStateExecute;
    }

    // different duty on an existing action powertrain
    else if( pos->second != iter->second )
    {
      pos->second = iter->second;
      m_eActionState = ActionStateExecute;
    }

    // else same duty
    
    LOGDIAG3("Motor %12s duty = %5.3lf.", iter->first.c_str(), iter->second);
  }

  // no planning and execution required
  if( m_eActionState == ActionStateUpdate )
  {
    m_eActionState = ActionStateIdle;
  }

  return LAE_OK;
}

int LaeKinActionDutyCycle::execute()
{
  int                       nCtlr;      // motor controller id
  int                       nMotor;     // motor index
  LaePowertrain            *pTrain;     // working powertrain
  LaeMapDutyCycle::iterator goal;       // working goal
  double                    fDuty[LaeNumMotorsPerCtlr]; // working duty cycles
  bool                      bDoExec;    // do [not] exec
  RoboClaw                 *pMotorCtlr; // motor controller
  int                       rc;         // return code

  // no execution required
  if( !isExecutionRequired() )
  {
    return LAE_OK;
  }

  LaeKinAction::execute();

  //
  // Iterate through motors and executre new duty cycle commands.
  //
  for(nCtlr = 0; nCtlr < LaeNumMotorCtlrs; ++nCtlr)
  {
    bDoExec = false;

    for(nMotor = 0; nMotor < LaeNumMotorsPerCtlr; ++nMotor)
    {
      if( (pTrain = m_kin.getPowertrain(nCtlr, nMotor)) == NULL )
      {
        continue;
      }

      goal = m_mapGoalDutyCycle.find(pTrain->getName());

      if( goal != m_mapGoalDutyCycle.end() )
      {
        fDuty[nMotor] = goal->second;
        bDoExec = true;
      }
      else
      {
        fDuty[nMotor] = 0.0;
      }
    }

    if( bDoExec )
    {
      pMotorCtlr = m_kin.getMotorCtlr(nCtlr);

      rc = pMotorCtlr->cmdDutyDrive2(fDuty[0], fDuty[1]);

      if( rc != OK )
      {
        rc = -LAE_ECODE_MOT_CTLR;
        break;
      }
    }
  }

  m_eActionState = ActionStateIdle;

  return rc;
}

int LaeKinActionDutyCycle::terminate()
{
  LaeMapDutyCycle::iterator  iter;
  int                       nCtlr;
  RoboClaw                  *pMotorCtlr;

  // already terminated
  if( hasTerminated() )
  {
    return LAE_OK;
  }

  //
  // Coast all motors.
  //
  for(nCtlr = 0; nCtlr < LaeNumMotorCtlrs; ++nCtlr)
  {
    pMotorCtlr = m_kin.getMotorCtlr(nCtlr);
    pMotorCtlr->cmdDutyDrive2(0.0, 0.0);
  }

  clear();

  LaeKinAction::terminate();

  return LAE_OK;
}

void LaeKinActionDutyCycle::clear()
{
  m_mapGoalDutyCycle.clear();
}
