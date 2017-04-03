////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Library:   liblaelaps
//
// File:      laeAlarms.cxx
//
/*! \file
 *
 * $LastChangedDate: 2016-04-08 15:42:39 -0600 (Fri, 08 Apr 2016) $
 * $Rev: 4380 $
 *
 * \brief Laelaps alarm monitoring class implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2016-2017. RoadNarrows LLC.\n
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

#include "rnr/rnrconfig.h"

#include  "Laelaps/laelaps.h"
#include  "Laelaps/laeSysDev.h"
#include  "Laelaps/RoboClaw.h"
#include  "Laelaps/laeMotor.h"
#include  "Laelaps/laeTune.h"
#include  "Laelaps/laeAlarms.h"
#include  "Laelaps/laeDb.h"

using namespace std;
using namespace laelaps;
using namespace motor::roboclaw;

// -----------------------------------------------------------------------------
// Class LaeAlarms
// -----------------------------------------------------------------------------

LaeAlarms::LaeAlarms()
{
}

LaeAlarms::~LaeAlarms()
{
}

bool LaeAlarms::isCritical()
{
  return RtDb.m_alarms.m_system.m_bIsCritical;
}

bool LaeAlarms::isBatteryCritical()
{
  return RtDb.m_alarms.m_battery.m_bIsCritical;
}

bool LaeAlarms::isSafeToOperate()
{
  // no critical alarms
  if( !RtDb.m_alarms.m_system.m_bIsCritical )
  {
    return true;
  }
  // only battery critical
  else if( (RtDb.m_alarms.m_system.m_uAlarms & ~LAE_ALARM_BATT) ==
                                                              LAE_ALARM_NONE )
  {
    return true;
  }
  // bad robot
  else
  {
    return false;
  }
}

void LaeAlarms::update()
{
  LaeAlarmInfo  sys;                // system alarms
  LaeAlarmInfo  subsys;             // subsystem alarms
  LaeAlarmInfo  mot1;               // motor 1 subsystem alarms
  LaeAlarmInfo  mot2;               // motor 2 subsystem alarms
  LaeAlarmInfo  sensors;            // sensors subsytem alarms
  u32_t         uStatus;            // subsystem status
  int           nCtlr;              // motor controller index
  int           nMotor1, nMotor2;   // motors 1 and 2 absolute indices

  // clear local copy of system alarms
  clearAlarms(sys);

  //
  // Determine motor controllers and motors alarm state.
  //
  for(nCtlr=0; nCtlr<LaeNumMotorCtlrs; ++nCtlr)
  {
    clearAlarms(subsys);
    clearAlarms(mot1);
    clearAlarms(mot2);

    uStatus = RtDb.m_motorctlr[nCtlr].m_uStatus;

    if( nCtlr == LaeMotorCtlrIdFront ) // front
    {
      nMotor1 = LaeMotorIdLF;
      nMotor2 = LaeMotorIdRF;
    }
    else  // rear
    {
      nMotor1 = LaeMotorIdLR;
      nMotor2 = LaeMotorIdRR;
    }

    // map controller status to controller alarms
    if( uStatus & ParamStatusErrTemp )
    {
      subsys.m_uAlarms |= LAE_ALARM_TEMP;
    }
    if( uStatus & ParamStatusErrMainBattHigh )
    {
      subsys.m_uAlarms |= LAE_ALARM_MOTCTLR_BATT_V_HIGH;
    }
    if( uStatus & ParamStatusErrLogicBattHigh )
    {
      subsys.m_uAlarms |= LAE_ALARM_MOTCTLR_LOGIC_V_HIGH;
    }
    if( uStatus & ParamStatusErrLogicBattLow )
    {
      subsys.m_uAlarms |= LAE_ALARM_MOTCTLR_LOGIC_V_LOW;
    }

    // map controller status to controller warnings
    if( uStatus & ParamStatusWarnMainBattHigh )
    {
      subsys.m_uWarnings |= LAE_WARN_MOTCTLR_BATT_V_HIGH;
    }
    if( uStatus & ParamStatusWarnMainBattLow )
    {
      subsys.m_uWarnings |= LAE_WARN_MOTCTLR_BATT_V_LOW;
    }
    if( uStatus & ParamStatusWarnTemp )
    {
      subsys.m_uWarnings |= LAE_WARN_TEMP;
    }

    // contoller critcal alarms
    if( subsys.m_uAlarms & LAE_CRIT_MOTCTLR )
    {
      subsys.m_bIsCritical = true;
    }

    // motor alarms
    if( uStatus & ParamStatusErrMot1Fault )
    {
      mot1.m_uAlarms |= LAE_ALARM_MOT_FAULT;
    }
    if( uStatus & ParamStatusErrMot2Fault )
    {
      mot2.m_uAlarms |= LAE_ALARM_MOT_FAULT;
    }

    // motor warnings
    if( uStatus & ParamStatusWarnMot1OverCur )
    {
      mot1.m_uWarnings |= LAE_WARN_MOT_OVER_CUR;
    }
    if( uStatus & ParamStatusWarnMot2OverCur )
    { 
      mot2.m_uWarnings |= LAE_WARN_MOT_OVER_CUR;
    }

    // motor critcal alarms
    if( mot1.m_uAlarms & LAE_CRIT_MOT )
    {
      mot1.m_bIsCritical = true;
    }
    if( mot2.m_uAlarms & LAE_CRIT_MOT )
    {
      mot2.m_bIsCritical = true;
    }

    // copy to database
    copyAlarms(subsys, RtDb.m_alarms.m_motorctlr[nCtlr]);
    copyAlarms(mot1, RtDb.m_alarms.m_motor[nMotor1]);
    copyAlarms(mot2, RtDb.m_alarms.m_motor[nMotor2]);

    // update system alarms from subsystem state
    updateSysAlarms(subsys, sys);
    updateSysAlarms(mot1, sys);
    updateSysAlarms(mot2, sys);
  }

  //
  // Determine battery alarm state 
  //
  clearAlarms(subsys);

  if( RtDb.m_energy.m_fBatterySoC < LAE_CRIT_BATT_SOC )
  {
    subsys.m_uAlarms |= LAE_ALARM_BATT;
    subsys.m_bIsCritical = true;
  }
  if( RtDb.m_energy.m_fBatterySoC < LAE_WARN_BATT_SOC )
  {
    subsys.m_uWarnings |= LAE_WARN_BATT;
  }

  // copy to database
  copyAlarms(subsys, RtDb.m_alarms.m_battery);

  // update system alarms from subsystem state
  updateSysAlarms(subsys, sys);

  //
  // Determine senors alarm state 
  //
  clearAlarms(subsys);

  if( !RtDb.m_enable.m_bImu )
  {
    subsys.m_uAlarms |= LAE_ALARM_IMU;
  }
  if( !RtDb.m_enable.m_bRange )
  {
    subsys.m_uAlarms |= LAE_ALARM_RANGE;
  }
  if( !RtDb.m_enable.m_bFCam )
  {
    subsys.m_uAlarms |= LAE_ALARM_FCAM;
  }

  // copy to database
  copyAlarms(subsys, RtDb.m_alarms.m_sensors);

  // update system alarms from subsystem state
  updateSysAlarms(subsys, sys);

  //
  // Other system alarms.
  //
  if( RtDb.m_robotstatus.m_bIsEStopped )
  {
    sys.m_uAlarms |= LAE_ALARM_ESTOP;
  }

  // copy system alarms to database
  copyAlarms(sys, RtDb.m_alarms.m_system);

  //fprintf(stderr, "DBG: isCrit=%d, alarms=0x%x, warns=0x%x\n", 
  //    RtDb.m_alarms.m_system.m_bIsCritical,
  //    RtDb.m_alarms.m_system.m_uAlarms,
  //    RtDb.m_alarms.m_system.m_uWarnings);
}

void LaeAlarms::updateSysAlarms(const LaeAlarmInfo &subsys, LaeAlarmInfo &sys)
{
  //
  // Any critically alarms subsystem propagates up to the system level.
  //
  if( subsys.m_bIsCritical )
  {
    sys.m_bIsCritical = true;
  }

  //
  // Subsytems alarms that propagate up to the system level.
  //
  if( subsys.m_uAlarms & LAE_ALARM_TEMP )
  {
    sys.m_uAlarms |= LAE_ALARM_TEMP;
  }
  if( subsys.m_uAlarms & LAE_ALARM_BATT )
  {
    sys.m_uAlarms |= LAE_ALARM_BATT;
  }
  if( subsys.m_uAlarms & LAE_ALARM_IMU )
  {
    sys.m_uAlarms |= LAE_ALARM_IMU;
  }
  if( subsys.m_uAlarms & LAE_ALARM_RANGE )
  {
    sys.m_uAlarms |= LAE_ALARM_RANGE;
  }
  if( subsys.m_uAlarms & LAE_ALARM_FCAM )
  {
    sys.m_uAlarms |= LAE_ALARM_FCAM;
  }

  //
  // Subsystem is [not] alarmed.
  //
  if( subsys.m_uAlarms != LAE_ALARM_NONE )
  {
    sys.m_uAlarms |= LAE_ALARM_SUBSYS;
  }

  //
  // Subsytems warnings that propagate up to the system level.
  //
  if( subsys.m_uWarnings & LAE_WARN_TEMP )
  {
    sys.m_uWarnings |= LAE_WARN_TEMP;
  }

  if( subsys.m_uWarnings & LAE_WARN_BATT )
  {
    sys.m_uWarnings |= LAE_WARN_BATT;
  }

  if( subsys.m_uWarnings != LAE_WARN_NONE )
  {
    sys.m_uWarnings |= LAE_WARN_SUBSYS;
  }
}

void LaeAlarms::clearAlarms(LaeAlarmInfo &info)
{
  info.m_bIsCritical  = false;
  info.m_uAlarms      = LAE_ALARM_NONE;
  info.m_uWarnings    = LAE_WARN_NONE;
}

void LaeAlarms::copyAlarms(const LaeAlarmInfo &src, LaeAlarmInfo &dst)
{
  dst.m_bIsCritical = src.m_bIsCritical;
  dst.m_uAlarms     = src.m_uAlarms;
  dst.m_uWarnings   = src.m_uWarnings;
}
