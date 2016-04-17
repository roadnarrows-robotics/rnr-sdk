////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   libDynamixel
//
// File:      DynaServoGeneric.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-03-13 13:28:02 -0600 (Fri, 13 Mar 2015) $
 * $Rev: 3890 $
 *
 * \brief Generic Dynamixel servo base class.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011-2015.  RoadNarrows LLC.
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

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <string.h>
#include <stdarg.h>
#include <iostream>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/new.h"
#include "rnr/units.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaTypes.h"
#include "Dynamixel/MX.h"

#include "Dynamixel/DynaError.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaServo.h"
#include "Dynamixel/DynaServoGeneric.h"
#include "Dynamixel/DynaOlio.h"

#include "DynaLibInternal.h"

using namespace std;


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Generic Servo EEPROM Control Table Information.
 */
static DynaCtlTblEntry_T GenericEEPROMCtlTblInfo[] =
{
  {0x00, "Model Number",              2, 0xffff,  false,  NULL},
  {0x02, "Firmware Version",          1, 0xff,    false,  "%u"},
  {0x03, "Servo Id",                  1, 0xff,    false,  "%u"},
  {0x04, "Baud Rate",                 1, 0xff,    false,  "%u"},
  {0x05, "Return Delay Time",         1, 0xff,    false,  "%u"},
  {0x06, "CW Angle Limit",            2, 0x3ff,   false,  "%u"},
  {0x08, "CCW Angle Limit",           2, 0x3ff,   false,  "%u"},
  {0x0b, "Highest Temperature Limit", 1, 0xff,    false,  "%u"},
  {0x0c, "Lowest Voltage Limit",      1, 0xff,    false,  "%u"},
  {0x0d, "Highest Voltage Limit",     1, 0xff,    false,  "%u"},
  {0x0e, "Maximum Torque",            2, 0x3ff,   false,  "%u"},
  {0x10, "Status Return Level",       1, 0xff,    false,  "%u"},
  {0x11, "Alarm LED",                 1, 0xff,    false,  "%u"},
  {0x12, "Alarm Shutdown",            1, 0xff,    false,  NULL}
};

/*!
 * \brief Generic Servo RAM Control Table Information.
 */
static DynaCtlTblEntry_T GenericRAMCtlTblInfo[] =
{
  {0x18, "Torque Enable",             1, 0xff,    false,  "%u"},
  {0x19, "LED",                       1, 0xff,    false,  "%u"},
  {0x1a, "CW Compliance Margin",      1, 0xff,    false,  "%u"},
  {0x1b, "CCW Compliance Margin",     1, 0xff,    false,  "%u"},
  {0x1c, "CW Compliance Slope",       1, 0xff,    false,  "%u"},
  {0x1d, "CCW Compliance Slope",      1, 0xff,    false,  "%u"},
  {0x1e, "Goal Position",             2, 0x3ff,   false,  "%u"},
  {0x20, "Goal Speed",                2, 0x7ff,   true,   "%d"},
  {0x22, "Torque Limit",              2, 0x3ff,   false,  "%u"},
  {0x24, "Current Position",          2, 0x3ff,   false,  "%u"},
  {0x26, "Current Speed",             2, 0x7ff,   true,   "%d"},
  {0x28, "Current Load",              2, 0x7ff,   true,   "%d"},
  {0x2a, "Current Voltage",           1, 0xff,    false,  "%u"},
  {0x2b, "Current Temperature",       1, 0xff,    false,  "%u"},
  {0x2c, "Registered",                1, 0xff,    false,  "%u"},
  {0x2e, "Moving",                    1, 0xff,    false,  "%u"},
  {0x2f, "Lock",                      1, 0xff,    false,  "%u"},
  {0x30, "Punch",                     2, 0x3ff,   false,  "%u"}
};

// ---------------------------------------------------------------------------
// DynaServoGeneric Class
// ---------------------------------------------------------------------------

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Constructors and Destructors
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

DynaServoGeneric::DynaServoGeneric(DynaComm &comm,
                                   int       nServoId,
                                   uint_t    uModelNum,
                                   uint_t    uFwVer) : 
                  DynaServo(comm, nServoId, uModelNum, uFwVer)
{
  Init();
  SyncData();
  CheckData();
}

DynaServoGeneric::~DynaServoGeneric()
{
  //Stop();
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Servo Proxy Agent Functions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int DynaServoGeneric::AgentWriteGoalPos(int nGoalOdPos)
{
  int   rc;

  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_IS_MASTER(this);
  DYNA_TRY_SERVO_HAS_POS_CTL(this);
  DYNA_TRY_SERVO_HAS_AGENT(this);

  m_state.m_od.m_nOdGoalPos = nGoalOdPos;
  rc = m_pAgent->m_fnWriteGoalPos(m_nServoId, nGoalOdPos, m_pAgentArg);

  return rc;
}

int DynaServoGeneric::AgentWriteGoalSpeed(int nGoalSpeed)
{
  int   rc;

  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_IS_MASTER(this);
  DYNA_TRY_SERVO_HAS_POS_CTL(this);
  DYNA_TRY_SERVO_HAS_AGENT(this);

  DYNA_TRY_EXPR( (iabs(nGoalSpeed) <= DYNA_SPEED_MAX_RAW),
                  DYNA_ECODE_BAD_VAL,
                  "Goal speed %d: Out of range.", nGoalSpeed);

  rc = m_pAgent->m_fnWriteGoalSpeed(m_nServoId, nGoalSpeed, m_pAgentArg);

  return rc;
}

int DynaServoGeneric::AgentWriteGoalSpeedPos(int nGoalSpeed, int nGoalOdPos)
{
  int   rc;

  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_IS_MASTER(this);
  DYNA_TRY_SERVO_HAS_POS_CTL(this);
  DYNA_TRY_SERVO_HAS_AGENT(this);

  DYNA_TRY_EXPR( (iabs(nGoalSpeed) <= DYNA_SPEED_MAX_RAW),
                  DYNA_ECODE_BAD_VAL,
                  "Goal speed %d: Out of range.", nGoalSpeed);

  m_state.m_od.m_nOdGoalPos = nGoalOdPos;

  rc = m_pAgent->m_fnWriteGoalSpeedPos(m_nServoId,
                                       nGoalSpeed, nGoalOdPos,
                                       m_pAgentArg);

  return rc;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Servo Move Functions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int DynaServoGeneric::MoveTo(int nGoalOdPos)
{
  int   rc;

  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_IS_MASTER(this);
  DYNA_TRY_SERVO_HAS_POS_CTL(this);

  if( HasAgent() )
  {
    rc = AgentWriteGoalPos(nGoalOdPos);
  }
  else
  {
    rc = WriteGoalPos(nGoalOdPos);
  }

  return rc;
}

int DynaServoGeneric::MoveAtSpeedTo(int nGoalSpeed, int nGoalOdPos)
{
  int   rc;   // return code

  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_IS_MASTER(this);
  DYNA_TRY_SERVO_HAS_POS_CTL(this);

  if( HasAgent() )
  {
    rc = AgentWriteGoalSpeedPos(nGoalSpeed, nGoalOdPos);
  }
  else if( (rc = WriteGoalSpeed(nGoalSpeed)) == DYNA_OK )
  {
    rc = WriteGoalPos(nGoalOdPos);
  }

  return rc;
}

int DynaServoGeneric::MoveAtSpeed(int nGoalSpeed)
{
  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_IS_MASTER(this);
  DYNA_TRY_SERVO_IN_MODE(this, DYNA_MODE_CONTINUOUS);

  return WriteGoalSpeed(nGoalSpeed);
}

int DynaServoGeneric::EStop()
{
  int nMaxTries = 3;
  int nTries;
  int rc;

  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_IS_MASTER(this);

  Stop();

  for(nTries=0; nTries<nMaxTries; ++nTries)
  {
    if( (rc = WriteTorqueEnable(false)) == DYNA_OK )
    {
      break;
    }
  }
  return rc;
}

int DynaServoGeneric::Stop()
{
  int     nCurPos;    // current position
  int     rc;         // return code

  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_IS_MASTER(this);

  if( HasAgent() )
  {
    rc = AgentWriteGoalSpeed(DYNA_SPEED_CONT_STOP);
  }

  else
  {
    switch( GetServoMode() )
    {
      //
      // In continuous mode: Set the goal speed to zero.
      //
      case DYNA_MODE_CONTINUOUS:
        rc = WriteGoalSpeed(DYNA_SPEED_CONT_STOP);
        break;

      //
      // In servo mode: Set the goal position to the current position (a 0 speed
      // value sets the speed to the default maximum).
      //
      case DYNA_MODE_SERVO:
      default:
        if( (rc = ReadCurPos(&nCurPos)) == DYNA_OK )
        {
          rc = WriteGoalPos(nCurPos);
        }
        break;
    }
  }

  return rc;
}

int DynaServoGeneric::Freeze()
{
  int     rc_tmp;     // temporary return code
  int     rc;         // return code

  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_IS_MASTER(this);

  rc = WriteTorqueEnable(true);

  rc_tmp = Stop();

  if( rc == DYNA_OK )
  {
    rc = rc_tmp;
  }

  return rc;
}

int DynaServoGeneric::Release()
{
  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_IS_MASTER(this);

  Stop();

  return WriteTorqueEnable(false);
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Servo Read/Write Functions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int DynaServoGeneric::CfgReadRotationLimits(uint_t *pCwLim, uint_t *pCcwLim)
{
  uint_t  uVal1, uVal2;   // working values
  int     rc;             // return code

  DYNA_TRY_COMM(m_comm);

  // Clockwise limit
  rc = m_comm.Read16(m_nServoId, DYNA_ADDR_LIM_CW_LSB, pCwLim);

  // Counter-clockwise limit
  if( rc == DYNA_OK )
  {
    rc = m_comm.Read16(m_nServoId, DYNA_ADDR_LIM_CCW_LSB, pCcwLim);
  }

  // Update configuration and state.
  if( rc == DYNA_OK )
  {
    m_cfg.m_uLimCw  = *pCwLim;
    m_cfg.m_uLimCcw = *pCcwLim;

    SetServoMode();

    if( IsLinkedMaster() )
    {
      rc = m_link.m_pServoMate->CfgReadRotationLimits(&uVal1, &uVal2);
    }
  }

  return rc;
}

int DynaServoGeneric::CfgWriteRotationLimits(uint_t uCwLim, uint_t uCcwLim)
{
  int   rc;   // return code

  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_IS_MASTER(this);

  DYNA_TRY_EXPR(((uCwLim >= m_spec.m_uRawPosMin) &&
                 (uCwLim <= m_spec.m_uRawPosMax)),
      DYNA_ECODE_BAD_VAL,
      "Clockwise limit %u: Out of range.", uCwLim);

  DYNA_TRY_EXPR(((uCcwLim >= m_spec.m_uRawPosMin) &&
                 (uCcwLim <= m_spec.m_uRawPosMax)),
      DYNA_ECODE_BAD_VAL,
      "Counterclockwise limit %u: Out of range.", uCcwLim);

  DYNA_TRY_EXPR((uCcwLim >= uCwLim), DYNA_ECODE_BAD_VAL,
      "Counterclockwise limit %u: Smaller than clockwise limit %u.",
      uCcwLim, uCwLim);

  //
  // Linked pair
  //
  if( IsLinkedMaster() )
  {
    // Clockwise limit
    rc = m_comm.vSyncWrite(DYNA_ADDR_LIM_CW_LSB, 2, 2,
                          m_nServoId, uCwLim,
                          m_link.m_pServoMate->GetServoId(), uCwLim);

    // Counter-clockwise limit
    if( rc == DYNA_OK )
    {
      m_cfg.m_uLimCw = uCwLim;
      m_link.m_pServoMate->m_cfg.m_uLimCw = uCwLim;

      rc = m_comm.vSyncWrite(DYNA_ADDR_LIM_CCW_LSB, 2, 2,
                          m_nServoId, uCcwLim,
                          m_link.m_pServoMate->GetServoId(), uCcwLim);

      if( rc == DYNA_OK )
      {
        m_cfg.m_uLimCcw = uCcwLim;
        m_link.m_pServoMate->m_cfg.m_uLimCcw = uCcwLim;
      }
    }
  }

  //
  // Solitary servo
  //
  else
  {
    // Clockwise limit
    rc = m_comm.Write16(m_nServoId, DYNA_ADDR_LIM_CW_LSB, uCwLim);

    // Counter-clockwise limit
    if( rc == DYNA_OK )
    {
      m_cfg.m_uLimCw = uCwLim;
      rc = m_comm.Write16(m_nServoId, DYNA_ADDR_LIM_CCW_LSB, uCcwLim);
    }

    // Update configuration and state.
    if( rc == DYNA_OK )
    {
      m_cfg.m_uLimCcw = uCcwLim;
      SetServoMode();
    }
  }

  return rc;
}

int DynaServoGeneric::CfgReadTemperatureLimit(uint_t *pTempLim)
{
  uint_t  uVal;   // working value
  int     rc;     // return code

  DYNA_TRY_COMM(m_comm);

  rc = m_comm.Read8(m_nServoId, DYNA_ADDR_LIM_TEMP_MAX, pTempLim);

  if( rc == DYNA_OK )
  {
    m_cfg.m_uLimTemp = *pTempLim;

    if( IsLinkedMaster() )
    {
      rc = m_link.m_pServoMate->CfgReadTemperatureLimit(&uVal);
    }
  }

  return rc;
}

int DynaServoGeneric::CfgWriteTemperatureLimit(uint_t uTempLim)
{
  int   rc;   // return code

  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_IS_MASTER(this);

  DYNA_TRY_EXPR(((uTempLim >= m_spec.m_uRawTempMin) &&
                 (uTempLim <= m_spec.m_uRawTempMax)), 
      DYNA_ECODE_BAD_VAL,
      "Temperature limit %u: Out of range.", uTempLim);

  //
  // Linked pair
  //
  if( IsLinkedMaster() )
  {
    rc = m_comm.vSyncWrite(DYNA_ADDR_LIM_TEMP_MAX, 1, 2,
                        m_nServoId, uTempLim,
                        m_link.m_pServoMate->GetServoId(), uTempLim);

    if( rc == DYNA_OK )
    {
      m_cfg.m_uLimTemp = uTempLim;
      m_link.m_pServoMate->m_cfg.m_uLimTemp = uTempLim;
    }
  }

  //
  // Solitary servo
  //
  else
  {
    rc = m_comm.Write8(m_nServoId, DYNA_ADDR_LIM_TEMP_MAX, uTempLim);

    if( rc == DYNA_OK )
    {
      m_cfg.m_uLimTemp = uTempLim;
    }
  }

  return rc;
}

int DynaServoGeneric::CfgReadVoltageLimits(uint_t *pMinVoltLim,
                                           uint_t *pMaxVoltLim)
{
  uint_t  uVal1, uVal2;   // working values
  int     rc;             // return code

  DYNA_TRY_COMM(m_comm);

  rc = m_comm.Read8(m_nServoId, DYNA_ADDR_LIM_VOLT_MIN, pMinVoltLim);

  if( rc == DYNA_OK )
  {
    rc = m_comm.Read8(m_nServoId, DYNA_ADDR_LIM_VOLT_MAX, pMaxVoltLim);
  }

  if( rc == DYNA_OK )
  {
    m_cfg.m_uLimVoltMin = *pMinVoltLim;
    m_cfg.m_uLimVoltMax = *pMaxVoltLim;

    if( IsLinkedMaster() )
    {
      rc = m_link.m_pServoMate->CfgReadVoltageLimits(&uVal1, &uVal2);
    }
  }

  return rc;
}

int DynaServoGeneric::CfgWriteVoltageLimits(uint_t uMinVoltLim,
                                            uint_t uMaxVoltLim)
{
  int   rc;   // return code

  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_IS_MASTER(this);

  DYNA_TRY_EXPR(((uMinVoltLim >= m_spec.m_uRawVoltMin) && 
                 (uMinVoltLim <= m_spec.m_uRawVoltMax)), 
      DYNA_ECODE_BAD_VAL,
      "Minimum voltage limit %u: Out of range.", uMinVoltLim);

  DYNA_TRY_EXPR(((uMaxVoltLim >= m_spec.m_uRawVoltMin) && 
                 (uMaxVoltLim <= m_spec.m_uRawVoltMax)), 
      DYNA_ECODE_BAD_VAL,
      "Maximum voltage limit %u: Out of range.", uMaxVoltLim);

  DYNA_TRY_EXPR((uMinVoltLim <= uMaxVoltLim), DYNA_ECODE_BAD_VAL,
      "Maximum voltage limit %u: Smaller than minimum voltage limit %u.",
      uMaxVoltLim, uMinVoltLim);

  //
  // Linked pair
  //
  if( IsLinkedMaster() )
  {
    rc = m_comm.vSyncWrite(DYNA_ADDR_LIM_VOLT_MIN, 1, 2,
                        m_nServoId, uMinVoltLim,
                        m_link.m_pServoMate->GetServoId(), uMinVoltLim);

    if( rc == DYNA_OK )
    {
      m_cfg.m_uLimVoltMin = uMinVoltLim;
      m_link.m_pServoMate->m_cfg.m_uLimVoltMin = uMinVoltLim;

      rc = m_comm.vSyncWrite(DYNA_ADDR_LIM_VOLT_MAX, 1, 2,
                        m_nServoId, uMaxVoltLim,
                        m_link.m_pServoMate->GetServoId(), uMaxVoltLim);
    }

    if( rc == DYNA_OK )
    {
      m_cfg.m_uLimVoltMax = uMaxVoltLim;
      m_link.m_pServoMate->m_cfg.m_uLimVoltMax = uMaxVoltLim;
    }
  }

  //
  // Solitary servo
  //
  else
  {
    rc = m_comm.Write8(m_nServoId, DYNA_ADDR_LIM_VOLT_MIN, uMinVoltLim);

    if( rc == DYNA_OK )
    {
      m_cfg.m_uLimVoltMin = uMinVoltLim;
      rc = m_comm.Write8(m_nServoId, DYNA_ADDR_LIM_VOLT_MAX, uMaxVoltLim);
    }

    if( rc == DYNA_OK )
    {
      m_cfg.m_uLimVoltMax = uMaxVoltLim;
    }
  }

  return rc;
}

int DynaServoGeneric::CfgReadMaxTorqueLimit(uint_t *pMaxTorqueLim)
{
  uint_t  uVal;   // working value
  int     rc;     // return code

  DYNA_TRY_COMM(m_comm);

  rc = m_comm.Read16(m_nServoId, DYNA_ADDR_LIM_TORQUE_MAX_ON_LSB,
                      pMaxTorqueLim);

  if( rc == DYNA_OK )
  {
    m_cfg.m_uLimTorqueMax = *pMaxTorqueLim;

    if( IsLinkedMaster() )
    {
      rc = m_link.m_pServoMate->CfgReadMaxTorqueLimit(&uVal);
    }
  }

  return rc;
}

int DynaServoGeneric::CfgWriteMaxTorqueLimit(uint_t uMaxTorqueLim)
{
  int   rc;   // return code

  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_IS_MASTER(this);

  DYNA_TRY_EXPR(((uMaxTorqueLim >= m_spec.m_uRawTorqueMin) && 
                 (uMaxTorqueLim <= m_spec.m_uRawTorqueMax)), 
      DYNA_ECODE_BAD_VAL,
      "Maximum torque limit %u: Out of range.", uMaxTorqueLim);

  //
  // Linked pair
  //
  if( IsLinkedMaster() )
  {
    rc = m_comm.vSyncWrite(DYNA_ADDR_LIM_TORQUE_MAX_ON_LSB, 2, 2,
                        m_nServoId, uMaxTorqueLim,
                        m_link.m_pServoMate->GetServoId(), uMaxTorqueLim);

    if( rc == DYNA_OK )
    {
      m_cfg.m_uLimTorqueMax = uMaxTorqueLim;
      m_link.m_pServoMate->m_cfg.m_uLimTorqueMax = uMaxTorqueLim;
    }
  }

  //
  // Solitary servo
  //
  else
  {
    rc = m_comm.Write16(m_nServoId, DYNA_ADDR_LIM_TORQUE_MAX_ON_LSB,
                      uMaxTorqueLim);

    if( rc == DYNA_OK )
    {
      m_cfg.m_uLimTorqueMax = uMaxTorqueLim;
    }
  }

  return rc;
}

int DynaServoGeneric::CfgReadAlarmShutdownMask(uint_t *pAlarmMask)
{
  uint_t  uVal;   // working value
  int     rc;     // return code

  DYNA_TRY_COMM(m_comm);

  rc = m_comm.Read8(m_nServoId, DYNA_ADDR_ALARM_SHUTDOWN, pAlarmMask);

  if( rc == DYNA_OK )
  {
    m_cfg.m_uAlarmShutdown = *pAlarmMask;

    if( IsLinkedMaster() )
    {
      rc = m_link.m_pServoMate->CfgReadAlarmShutdownMask(&uVal);
    }
  }

  return rc;
}

int DynaServoGeneric::CfgWriteAlarmShutdownMask(uint_t uAlarmMask)
{
  int   rc;   // return code

  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_IS_MASTER(this);

  uAlarmMask &= DYNA_ADDR_ALARM_SHUTDOWN_MASK;

  //
  // Linked pair
  //
  if( IsLinkedMaster() )
  {
    rc = m_comm.vSyncWrite(DYNA_ADDR_ALARM_SHUTDOWN, 1, 2,
                        m_nServoId, uAlarmMask,
                        m_link.m_pServoMate->GetServoId(), uAlarmMask);

    if( rc == DYNA_OK )
    {
      m_cfg.m_uAlarmShutdown = uAlarmMask;
      m_link.m_pServoMate->m_cfg.m_uAlarmShutdown = uAlarmMask;
    }
  }

  //
  // Solitary servo
  //
  else
  {
    rc = m_comm.Write8(m_nServoId, DYNA_ADDR_ALARM_SHUTDOWN, uAlarmMask);

    if( rc == DYNA_OK )
    {
      m_cfg.m_uAlarmShutdown = uAlarmMask;
    }
  }

  return rc;
}

int DynaServoGeneric::CfgReadServoMode(uint_t *pServoMode)
{
  uint_t  uCwLim;     // clockwise limit
  uint_t  uCcwLim;    // counterclockwise limit
  int     rc;         // return limit

  DYNA_TRY_COMM(m_comm);

  // mode set here
  rc = CfgReadRotationLimits(&uCwLim, &uCcwLim);

  *pServoMode = m_cfg.m_uServoMode;

  if( rc == DYNA_OK )
  {
    if( IsLinkedMaster() )
    {
      rc = m_link.m_pServoMate->CfgReadRotationLimits(&uCwLim, &uCcwLim);
    }
  }

  return rc;
}

int DynaServoGeneric::CfgWriteServoMode(uint_t uCwLim, uint_t uCcwLim)
{
  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_IS_MASTER(this);

  return CfgWriteRotationLimits(uCwLim, uCcwLim);
}

int DynaServoGeneric::CfgWriteServoModeContinuous()
{
  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_IS_MASTER(this);
  DYNA_TRY_SERVO_HAS_MODE(this, DYNA_MODE_CONTINUOUS);

  return CfgWriteRotationLimits(DYNA_CW_POS_CONT_MODE, DYNA_CCW_POS_CONT_MODE);
}

int DynaServoGeneric::ReadTorqueEnable(bool *pState)
{
  uint_t  uVal;   // working value
  bool    bVal;   // working boolean
  int     rc;     // return code

  DYNA_TRY_COMM(m_comm);

  rc = m_comm.Read8(m_nServoId, DYNA_ADDR_TORQUE_EN, &uVal);

  if( rc == DYNA_OK )
  {
    *pState = uVal==DYNA_TORQUE_EN_OFF? false: true;

    m_state.m_bTorqueEnabled = *pState;

    if( IsLinkedMaster() )
    {
      rc = m_link.m_pServoMate->ReadTorqueEnable(&bVal);
    }
  }

  return rc;
}

int DynaServoGeneric::WriteTorqueEnable(bool bState)
{
  uint_t  uVal;   // working value
  int     rc;     // return code

  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_IS_MASTER(this);

  uVal = bState? DYNA_TORQUE_EN_ON: DYNA_TORQUE_EN_OFF;

  //
  // Linked pair
  //
  if( IsLinkedMaster() )
  {
    rc = m_comm.vSyncWrite(DYNA_ADDR_TORQUE_EN, 1, 2,
                        m_nServoId, uVal,
                        m_link.m_pServoMate->GetServoId(),uVal);

    if( rc == DYNA_OK )
    {
      m_state.m_bTorqueEnabled = bState;
      m_link.m_pServoMate->m_state.m_bTorqueEnabled = bState;
    }
  }

  //
  // Solitary servo
  //
  else
  {
    rc = m_comm.Write8(m_nServoId, DYNA_ADDR_TORQUE_EN, uVal);

    if( rc == DYNA_OK )
    {
      m_state.m_bTorqueEnabled = bState;
    }
  }

  return rc;
}

int DynaServoGeneric::ReadLed(bool *pState)
{
  uint_t  uVal;   // working value
  bool    bVal;   // working boolean
  int     rc;     // return code

  DYNA_TRY_COMM(m_comm);

  rc = m_comm.Read8(m_nServoId, DYNA_ADDR_LED, &uVal);

  if( rc == DYNA_OK )
  {
    *pState = uVal==DYNA_LED_OFF? false: true;

    m_state.m_bLed = *pState;

    if( IsLinkedMaster() )
    {
      rc = m_link.m_pServoMate->ReadLed(&bVal);
    }
  }

  return rc;
}

int DynaServoGeneric::WriteLed(bool bState)
{
  uint_t  uVal;   // working value
  int     rc;     // return code

  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_IS_MASTER(this);

  uVal = bState? DYNA_LED_ON: DYNA_LED_OFF;

  //
  // Linked pair
  //
  if( IsLinkedMaster() )
  {
    rc = m_comm.vSyncWrite(DYNA_ADDR_LED, 1, 2,
                        m_nServoId, uVal,
                        m_link.m_pServoMate->GetServoId(), uVal);

    if( rc == DYNA_OK )
    {
      m_state.m_bLed = bState;
      m_link.m_pServoMate->m_state.m_bLed = bState;
    }
  }

  //
  // Solitary servo
  //
  else
  {
    rc = m_comm.Write8(m_nServoId, DYNA_ADDR_LED, uVal);

    if( rc == DYNA_OK )
    {
      m_state.m_bLed = bState;
    }
  }

  return rc;
}

int DynaServoGeneric::ReadControlMethod(DynaServoCtlMethod_T *pCtlMethod)
{
  int   rc;   // return code

  DYNA_TRY_COMM(m_comm);

  switch( m_spec.m_uCtlMethodUsed )
  {
    case DYNA_CTL_METHOD_COMPLIANCE:
      return ReadCtlMethodCompliance(pCtlMethod);
    case DYNA_CTL_METHOD_PID:
      return ReadCtlMethodPid(pCtlMethod);
    default:
      rc = -DYNA_ECODE_INTERNAL;
      DYNA_LOG_ERROR(rc, "Servo %d: Unknown control method %u.",
          m_nServoId, m_spec.m_uCtlMethodUsed);
      return rc;
  }
}

int DynaServoGeneric::WriteControlMethod(DynaServoCtlMethod_T &ctlMethod)
{
  int   rc;   // return code

  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_IS_MASTER(this);

  switch( m_spec.m_uCtlMethodUsed )
  {
    case DYNA_CTL_METHOD_COMPLIANCE:
      return WriteCtlMethodCompliance(ctlMethod);
    case DYNA_CTL_METHOD_PID:
      return WriteCtlMethodPid(ctlMethod);
    default:
      rc = -DYNA_ECODE_INTERNAL;
      DYNA_LOG_ERROR(rc, "Servo %d: Unknown control method %u.",
          m_nServoId, m_spec.m_uCtlMethodUsed);
      return rc;
  }
}

int DynaServoGeneric::ReadGoalPos(int *pGoalPos)
{
  uint_t  uVal;       // working value
  int     nValMate;   // working mate's value
  int     rc;         // return code

  DYNA_TRY_COMM(m_comm);

  if( m_cfg.m_uServoMode == DYNA_MODE_CONTINUOUS )
  {
    *pGoalPos = m_state.m_od.m_nOdGoalPos;
    rc = DYNA_OK;
  }
  else
  {
    rc = m_comm.Read16(m_nServoId, DYNA_ADDR_GOAL_POS_LSB, &uVal);

    if( rc == DYNA_OK )
    {
      m_state.m_uGoalPos = uVal;
      *pGoalPos = uVal;

      if( IsLinkedMaster() )
      {
        rc = m_link.m_pServoMate->ReadGoalPos(&nValMate);
      }
    }
  }

  return rc;
}

int DynaServoGeneric::WriteGoalPos(int nGoalOdPos)
{
  int       nEncPos;          // encoder position
  int       rc;               // return code

  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_IS_MASTER(this);

  nEncPos = OdometerToEncoder(nGoalOdPos);
  
  if( m_cfg.m_uServoMode == DYNA_MODE_SERVO )
  {
    DYNA_TRY_EXPR(((nEncPos >= m_spec.m_uRawPosMin) &&
                   (nEncPos <= m_spec.m_uRawPosMax)),
      DYNA_ECODE_BAD_VAL,
      "Servo %d: Goal odometer position %d (encoder=%d): Out of range.",
      m_nServoId, nGoalOdPos, nEncPos);
  }

  //
  // Linked pair
  //
  if( IsLinkedMaster() )
  {
    int   nGoalOdPosMate;   // mate's calculated goal odometer position
    int   nEncGoalPosMate;  // mate's calculated goal encoder position

    if( (rc = CalcMatesGoalPos(nGoalOdPos, &nGoalOdPosMate)) == DYNA_OK )
    {
      nEncGoalPosMate = m_link.m_pServoMate->OdometerToEncoder(nGoalOdPosMate);
      rc = m_comm.vSyncWrite(DYNA_ADDR_GOAL_POS_LSB, 2, 2,
                          m_nServoId, (uint_t)nEncPos,
                          m_link.m_pServoMate->GetServoId(), // RDK FIX ME
                          (uint_t)nEncGoalPosMate);

      if( rc == DYNA_OK )
      {
        m_state.m_uGoalPos = (uint_t)nEncPos;
        m_link.m_pServoMate->m_state.m_uGoalPos = (uint_t)nEncGoalPosMate;
      }
    }
  }

  //
  // Solitary servo
  //
  else
  {
    rc = m_comm.Write16(m_nServoId, DYNA_ADDR_GOAL_POS_LSB, (uint_t)nEncPos);

    if( rc == DYNA_OK )
    {
      m_state.m_uGoalPos = (uint_t)nEncPos;
    }
  }

  return rc;
}

int DynaServoGeneric::ReadGoalSpeed(int *pGoalSpeed)
{
  uint_t  uVal;     // working value
  int     nVal;     // working value
  int     rc;       // return code

  DYNA_TRY_COMM(m_comm);

  rc = m_comm.Read16(m_nServoId, DYNA_ADDR_GOAL_SPEED_LSB, &uVal);

  if( rc == DYNA_OK )
  {
    m_state.m_nGoalSpeed = UnpackGoalSpeed(uVal);
    *pGoalSpeed = m_state.m_nGoalSpeed;

    if( IsLinkedMaster() )
    {
      rc = m_link.m_pServoMate->ReadGoalSpeed(&nVal);
    }
  }

  return rc;
}

int DynaServoGeneric::WriteGoalSpeed(int nGoalSpeed)
{
  int     nGoalSpeedMate;   // mate's calculated speed
  uint_t  uVal1, uVal2;     // working values
  int     rc;               // return code

  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_IS_MASTER(this);

  DYNA_TRY_EXPR( (iabs(nGoalSpeed) <= DYNA_SPEED_MAX_RAW),
                  DYNA_ECODE_BAD_VAL,
                  "Goal speed %d: Out of range.", nGoalSpeed);

  uVal1 = PackGoalSpeed(nGoalSpeed);

  //
  // Linked pair
  //
  if( IsLinkedMaster() )
  {
    nGoalSpeedMate = CalcMatesGoalSpeed(nGoalSpeed);

    uVal2 = PackGoalSpeed(nGoalSpeedMate);

    rc = m_comm.vSyncWrite(DYNA_ADDR_GOAL_SPEED_LSB, 2, 2,
                        m_nServoId, uVal1,
                        m_link.m_pServoMate->GetServoId(), uVal2);

    if( rc == DYNA_OK )
    {
      m_state.m_nGoalSpeed = nGoalSpeed;
      m_link.m_pServoMate->m_state.m_nGoalSpeed = nGoalSpeedMate;
    }
  }

  //
  // Solitary servo
  //
  else
  {
    rc = m_comm.Write16(m_nServoId, DYNA_ADDR_GOAL_SPEED_LSB, uVal1);

    if( rc == DYNA_OK )
    {
      m_state.m_nGoalSpeed = nGoalSpeed;
    }
  }

  return rc;
}

int DynaServoGeneric::ReadMaxTorqueLimit(uint_t *pMaxTorqueLim)
{
  uint_t  uVal;   // working value
  int     rc;     // return code

  DYNA_TRY_COMM(m_comm);

  rc = m_comm.Read16(m_nServoId, DYNA_ADDR_LIM_TORQUE_MAX_LSB, pMaxTorqueLim);

  if( rc == DYNA_OK )
  {
    m_state.m_uLimTorqueMax = *pMaxTorqueLim;

    if( IsLinkedMaster() )
    {
      rc = m_link.m_pServoMate->ReadMaxTorqueLimit(&uVal);
    }
  }

  return rc;
}

int DynaServoGeneric::WriteMaxTorqueLimit(uint_t uMaxTorqueLim)
{
  int     rc;   // return code

  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_IS_MASTER(this);

  DYNA_TRY_EXPR(((uMaxTorqueLim >= m_spec.m_uRawTorqueMin) && 
                 (uMaxTorqueLim <= m_spec.m_uRawTorqueMax)), 
      DYNA_ECODE_BAD_VAL,
      "Maximum torque limit %u: Out of range.", uMaxTorqueLim);

  //
  // Linked pair
  //
  if( IsLinkedMaster() )
  {
    rc = m_comm.vSyncWrite(DYNA_ADDR_LIM_TORQUE_MAX_LSB, 2, 2,
                        m_nServoId, uMaxTorqueLim,
                        m_link.m_pServoMate->GetServoId(), uMaxTorqueLim);

    if( rc == DYNA_OK )
    {
      m_state.m_uLimTorqueMax = uMaxTorqueLim;
      m_link.m_pServoMate->m_state.m_uLimTorqueMax = uMaxTorqueLim;
    }
  }

  //
  // Solitary servo
  //
  else
  {
    rc = m_comm.Write16(m_nServoId, DYNA_ADDR_LIM_TORQUE_MAX_LSB,
                                    uMaxTorqueLim);

    if( rc == DYNA_OK )
    {
      m_state.m_uLimTorqueMax = uMaxTorqueLim;
    }
  }

  return rc;
}

int DynaServoGeneric::ReloadMaxTorqueLimit()
{
  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_IS_MASTER(this);

  return WriteMaxTorqueLimit(m_cfg.m_uLimTorqueMax);
}

int DynaServoGeneric::ReadCurPos(int *pCurOdPos)
{
  uint_t  uVal;   // working value
  int     nVal;   // working value
  int     rc;     // return code

  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_SERVO_HAS_POS_CTL(this);

  rc = m_comm.Read16(m_nServoId, DYNA_ADDR_CUR_POS_LSB, &uVal);

  if( rc == DYNA_OK )
  {
    m_state.m_uCurPos = uVal;
    
    UpdateOdometer((int)uVal);

    *pCurOdPos = GetOdometer();

    if( IsLinkedMaster() )
    {
      rc = m_link.m_pServoMate->ReadCurPos(&nVal);
    }
  }

  return rc;
}

int DynaServoGeneric::ReadCurSpeed(int *pCurSpeed)
{
  uint_t  uVal;   // working value
  int     nVal;   // working value
  int     rc;     // return code

  DYNA_TRY_COMM(m_comm);

  rc = m_comm.Read16(m_nServoId, DYNA_ADDR_CUR_SPEED_LSB, &uVal);

  if( rc == DYNA_OK )
  {

    m_state.m_nCurSpeed = UnpackCurSpeed(uVal);
    *pCurSpeed = m_state.m_nCurSpeed;

    if( IsLinkedMaster() )
    {
      rc = m_link.m_pServoMate->ReadCurSpeed(&nVal);
    }
  }

  return rc;
}

int DynaServoGeneric::ReadCurLoad(int *pCurLoad)
{
  uint_t  uVal;   // working value
  int     nVal;   // working value
  int     rc;     // return code

  DYNA_TRY_COMM(m_comm);

  rc = m_comm.Read16(m_nServoId, DYNA_ADDR_CUR_LOAD_LSB, &uVal);
  
  if( rc == DYNA_OK )
  {
    m_state.m_nCurLoad = UnpackCurLoad(uVal);
    *pCurLoad = m_state.m_nCurLoad;

    if( IsLinkedMaster() )
    {
      rc = m_link.m_pServoMate->ReadCurLoad(&nVal);
    }
  }

  return rc;
}

int DynaServoGeneric::ReadDynamics(int    *pCurPos,
                                   int    *pCurSpeed,
                                   int    *pCurLoad)
{
  int     rc, rc_tmp;                   // return codes

  DYNA_TRY_COMM(m_comm);

  rc = ReadCurPos(pCurPos);

  rc_tmp = ReadCurSpeed(pCurSpeed);

  if( rc == DYNA_OK )
  {
    rc = rc_tmp;
  }

  rc_tmp = ReadCurLoad(pCurLoad);

  if( rc == DYNA_OK )
  {
    rc = rc_tmp;
  }

  return rc;
}

int DynaServoGeneric::ReadHealth(uint_t *pAlarms,
                                 int    *pCurLoad,
                                 uint_t *pCurVolt,
                                 uint_t *pCurTemp)
{
  uint_t  uVal1;            // working value
  int     nVal2;            // woking value
  uint_t  uVal3, uVal4;     // working values
  int     rc, rc_tmp;       // return codes

  DYNA_TRY_COMM(m_comm);

  rc = ReadCurLoad(pCurLoad);
  
  rc_tmp = m_comm.Read8(m_nServoId, DYNA_ADDR_CUR_VOLT, pCurVolt);

  if( rc_tmp == DYNA_OK )
  {
    m_state.m_uCurVolt = *pCurVolt;
  }
  else if( rc == DYNA_OK )
  {
    rc = rc_tmp;
  }

  rc_tmp = m_comm.Read8(m_nServoId, DYNA_ADDR_CUR_TEMP_C, pCurTemp);

  if( rc_tmp == DYNA_OK )
  {
    m_state.m_uCurTemp = *pCurTemp;
  }
  else if( rc == DYNA_OK )
  {
    rc = rc_tmp;
  }

  if( rc == DYNA_OK )
  {
    if( IsLinkedMaster() )
    {
      rc = m_link.m_pServoMate->ReadHealth(&uVal1, &nVal2, &uVal3, &uVal4);
    }
  }

  m_state.m_uAlarms = m_comm.GetAlarms(); 
  *pAlarms = m_state.m_uAlarms;

  return rc;
}

int DynaServoGeneric::ReadIsMoving(bool *pState)
{
  uint_t  uVal;   // working value
  bool    bVal;   // working boolean
  int     rc;     // return code

  DYNA_TRY_COMM(m_comm);

  rc = m_comm.Read8(m_nServoId, DYNA_ADDR_IS_MOVING, &uVal);

  if( rc == DYNA_OK )
  {
    *pState = uVal==DYNA_IS_NOT_MOVING? false: true;

    m_state.m_bIsMoving = *pState;

    if( IsLinkedMaster() )
    {
      rc = m_link.m_pServoMate->ReadIsMoving(&bVal);
    }
  }

  return rc;
}

int DynaServoGeneric::Read(uint_t uAddr, uint_t *pVal)
{
  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_ADDR(uAddr);

  switch( uAddr )
  {
    case DYNA_ADDR_MODEL_NUM_LSB:
    case DYNA_ADDR_LIM_CW_LSB:
    case DYNA_ADDR_LIM_CCW_LSB:
    case DYNA_ADDR_LIM_TORQUE_MAX_ON_LSB:
    case DYNA_ADDR_GOAL_POS_LSB:
    case DYNA_ADDR_GOAL_SPEED_LSB:
    case DYNA_ADDR_LIM_TORQUE_MAX_LSB:
    case DYNA_ADDR_CUR_POS_LSB:
    case DYNA_ADDR_CUR_SPEED_LSB:
    case DYNA_ADDR_CUR_LOAD_LSB:
      return m_comm.Read16(m_nServoId, uAddr, pVal);

    case DYNA_ADDR_FWVER:
    case DYNA_ADDR_ID:
    case DYNA_ADDR_BAUD_RATE:
    case DYNA_ADDR_T_RET_DELAY:
    case DYNA_ADDR_LIM_TEMP_MAX:
    case DYNA_ADDR_LIM_VOLT_MIN:
    case DYNA_ADDR_LIM_VOLT_MAX:
    case DYNA_ADDR_SRL:
    case DYNA_ADDR_ALARM_LED:
    case DYNA_ADDR_ALARM_SHUTDOWN:
    case DYNA_ADDR_TORQUE_EN:
    case DYNA_ADDR_LED:
    case DYNA_ADDR_CW_COMP_MARGIN:
    case DYNA_ADDR_CCW_COMP_MARGIN:
    case DYNA_ADDR_CW_COMP_SLOPE:
    case DYNA_ADDR_CCW_COMP_SLOPE:
    case DYNA_ADDR_CUR_VOLT:
    case DYNA_ADDR_CUR_TEMP_C:
    case DYNA_ADDR_REG_INSTR:
    case DYNA_ADDR_IS_MOVING:
    case DYNA_ADDR_EEPROM_LOCK:
    case DYNA_ADDR_PUNCH_LSB:
    default:
      return m_comm.Read8(m_nServoId, uAddr, pVal);
  }
}

int DynaServoGeneric::Write(uint_t uAddr, uint_t uVal)
{
  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_ADDR(uAddr);

  switch( uAddr )
  {
    case DYNA_ADDR_MODEL_NUM_LSB:
    case DYNA_ADDR_LIM_CW_LSB:
    case DYNA_ADDR_LIM_CCW_LSB:
    case DYNA_ADDR_LIM_TORQUE_MAX_ON_LSB:
    case DYNA_ADDR_GOAL_POS_LSB:
    case DYNA_ADDR_GOAL_SPEED_LSB:
    case DYNA_ADDR_LIM_TORQUE_MAX_LSB:
    case DYNA_ADDR_CUR_POS_LSB:
    case DYNA_ADDR_CUR_SPEED_LSB:
    case DYNA_ADDR_CUR_LOAD_LSB:
      return m_comm.Write16(m_nServoId, uAddr, uVal);

    case DYNA_ADDR_FWVER:
    case DYNA_ADDR_ID:
    case DYNA_ADDR_BAUD_RATE:
    case DYNA_ADDR_T_RET_DELAY:
    case DYNA_ADDR_LIM_TEMP_MAX:
    case DYNA_ADDR_LIM_VOLT_MIN:
    case DYNA_ADDR_LIM_VOLT_MAX:
    case DYNA_ADDR_SRL:
    case DYNA_ADDR_ALARM_LED:
    case DYNA_ADDR_ALARM_SHUTDOWN:
    case DYNA_ADDR_TORQUE_EN:
    case DYNA_ADDR_LED:
    case DYNA_ADDR_CW_COMP_MARGIN:
    case DYNA_ADDR_CCW_COMP_MARGIN:
    case DYNA_ADDR_CW_COMP_SLOPE:
    case DYNA_ADDR_CCW_COMP_SLOPE:
    case DYNA_ADDR_CUR_VOLT:
    case DYNA_ADDR_CUR_TEMP_C:
    case DYNA_ADDR_REG_INSTR:
    case DYNA_ADDR_IS_MOVING:
    case DYNA_ADDR_EEPROM_LOCK:
    case DYNA_ADDR_PUNCH_LSB:
    default:
      return m_comm.Write8(m_nServoId, uAddr, uVal);
  }
}

bool DynaServoGeneric::Ping()
{
  DYNA_TRY_COMM(m_comm);
  return m_comm.Ping(m_nServoId);
}

int DynaServoGeneric::Reset()
{
  int   rc;

  DYNA_TRY_COMM(m_comm);

  if( (rc = m_comm.Reset(m_nServoId)) == DYNA_OK )
  {
    SyncData();
    CheckData();
  }
}

int DynaServoGeneric::SyncData()
{
  int   rc;

  rc = SyncCfg();
  rc = SyncState();

  return rc;
}

int DynaServoGeneric::SyncCfg()
{
  uint_t  uVal1;
  uint_t  uVal2;

  DYNA_TRY_COMM(m_comm);

  if( m_comm.Read8(m_nServoId, DYNA_ADDR_T_RET_DELAY, &uVal1) == DYNA_OK )
  {
    m_cfg.m_uTRetDelay = uVal1;
  }

  CfgReadRotationLimits(&uVal1, &uVal2);
  CfgReadTemperatureLimit(&uVal1);
  CfgReadVoltageLimits(&uVal1, &uVal2);
  CfgReadMaxTorqueLimit(&uVal1);

  if( m_comm.Read8(m_nServoId, DYNA_ADDR_SRL, &uVal1) == DYNA_OK )
  {
    m_cfg.m_uSrl = uVal1;
  }

  if( m_comm.Read8(m_nServoId, DYNA_ADDR_ALARM_LED, &uVal1) == DYNA_OK )
  {
    m_cfg.m_uAlarmLed = uVal1;
  }

  CfgReadAlarmShutdownMask(&uVal1);

  return DYNA_OK;
}

int DynaServoGeneric::SyncState()
{
  bool    bState;
  uint_t  uVal1, uVal3, uVal4, uVal5;
  int     nVal2;
  
  DYNA_TRY_COMM(m_comm);

  ReadTorqueEnable(&bState);
  ReadLed(&bState);
  ReadControlMethod(&m_state.m_ctlMethod);
  ReadGoalPos(&nVal2);
  ReadGoalSpeed(&nVal2);
  ReadMaxTorqueLimit(&uVal1);
  ReadCurPos(&nVal2);
  ReadCurSpeed(&nVal2);
  ReadHealth(&uVal1, &nVal2, &uVal3, &uVal4);
  ReadIsMoving(&bState);

  // reset odometer to align with encoder (odometer is also enabled)
  ResetOdometer(0, false);
  
  // log any servo alarms
  DYNA_LOG_SERVO_ALARMS(m_nServoId, m_state.m_uAlarms);

  return DYNA_OK;
}

void DynaServoGeneric::Dump()
{
  DumpCtlTbl("EEPROM",
                  GenericEEPROMCtlTblInfo, arraysize(GenericEEPROMCtlTblInfo));

  printf("\n");

  DumpCtlTbl("RAM", GenericRAMCtlTblInfo, arraysize(GenericRAMCtlTblInfo));
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Protected Interface
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

void DynaServoGeneric::Init()
{
  InitSpec();
  InitCfg();
  InitState();
}


void DynaServoGeneric::InitSpec()
{
  //
  // Fixed servo specification.
  //
  if( m_spec.m_sModelName != NULL )
  {
    delete[] m_spec.m_sModelName;
  }

  // physical and fixed features
  m_spec.m_sModelName           = newstr("Generic");
  m_spec.m_fWeight              = 100.0;
  m_spec.m_fWidth               = 40.0;
  m_spec.m_fHeight              = 50.0;
  m_spec.m_fDepth               = 40.0;
  m_spec.m_fResolution          = 0.29;
  m_spec.m_fGearRedectionRatio  = 193.0;
  m_spec.m_fStallTorque         = 40.0;
  m_spec.m_fMaxSpeed            = 85.0;
  m_spec.m_fAngleMin            = 0.0;
  m_spec.m_fAngleMax            = 300.0;
  m_spec.m_uSupportedModes      = DYNA_MODE_SERVO;
  m_spec.m_bHas360Pos           = false;
  m_spec.m_fTempMin             = -5.0;
  m_spec.m_fTempMax             = 80.0;
  m_spec.m_fVoltMin             = 12.0;
  m_spec.m_fVoltMax             = 18.5;

  // raw units
  m_spec.m_uRawPosMin           = DYNA_POS_MIN_RAW;
  m_spec.m_uRawPosMax           = DYNA_POS_MAX_RAW;
  m_spec.m_uRawPosModulo        = DYNA_POS_MODULO;
  m_spec.m_uRawSpeedMin         = DYNA_SPEED_MIN_RAW;
  m_spec.m_uRawSpeedMax         = DYNA_SPEED_MAX_RAW;
  m_spec.m_uRawTorqueMin        = DYNA_TORQUE_MIN_RAW;
  m_spec.m_uRawTorqueMax        = DYNA_TORQUE_MAX_RAW;
  m_spec.m_uRawTempMin          = DYNA_TEMP_MIN_RAW;
  m_spec.m_uRawTempMax          = DYNA_TEMP_MAX_RAW;
  m_spec.m_uRawVoltMin          = DYNA_VOLT_MIN_RAW;
  m_spec.m_uRawVoltMax          = DYNA_VOLT_MAX_RAW;
}

void DynaServoGeneric::InitCfg()
{
  //
  // Configuration defaults.
  //
  // Keep in 'natural' units (i.e. mostly in raw units), and convert to current
  // operational units when needed.
  //
  m_cfg.m_uTRetDelay          = DYNA_T_RET_DELAY_DFT_RAW;
  m_cfg.m_uLimCw              = DYNA_POS_MIN_RAW;
  m_cfg.m_uLimCcw             = DYNA_POS_MAX_RAW;
  m_cfg.m_uLimTemp            = DYNA_LIM_TEMP_DFT_C;
  m_cfg.m_uLimVoltMin         = DYNA_LIM_VOLT_MIN_DFT_RAW;
  m_cfg.m_uLimVoltMax         = DYNA_LIM_VOLT_MAX_DFT_RAW;
  m_cfg.m_uLimTorqueMax       = DYNA_LIM_TORQUE_MAX_ON_DFT_RAW;
  m_cfg.m_uSrl                = DYNA_SRL_RET_DFT;
  m_cfg.m_uAlarmLed           = DYNA_ALARM_LED_DFT;
  m_cfg.m_uAlarmShutdown      = DYNA_ALARM_DFT;  
  m_cfg.m_uServoMode          = DYNA_MODE_SERVO;
}

void DynaServoGeneric::InitState()
{
  //
  // State defaults.
  //
  // Keep in 'natural' units (i.e. mostly in raw units), and convert to current
  // operational units when needed.
  //
  m_state.m_uAlarms               = DYNA_ALARM_NONE; 
  m_state.m_bTorqueEnabled        = DYNA_TORQUE_EN_DFT;
  m_state.m_bLed                  = DYNA_LED_OFF;

  switch( m_spec.m_uCtlMethodUsed )
  {
    case DYNA_CTL_METHOD_COMPLIANCE:
      m_state.m_ctlMethod.m_params.m_comp.m_uCwMargin =
                                             DYNA_COMP_MARGIN_DFT_RAW; 
      m_state.m_ctlMethod.m_params.m_comp.m_uCwSlope  = 
                                              DYNA_COMP_SLOPE_TORQUE_DFT;
      m_state.m_ctlMethod.m_params.m_comp.m_uCcwMargin = 
                                              DYNA_COMP_MARGIN_DFT_RAW;
      m_state.m_ctlMethod.m_params.m_comp.m_uCcwSlope = 
                                              DYNA_COMP_SLOPE_TORQUE_DFT;
      break;
    case DYNA_CTL_METHOD_PID:
      m_state.m_ctlMethod.m_params.m_pid.m_uPGain  = DYNA_P_GAIN_DFT;
      m_state.m_ctlMethod.m_params.m_pid.m_uIGain  = DYNA_I_GAIN_DFT;
      m_state.m_ctlMethod.m_params.m_pid.m_uDGain  = DYNA_D_GAIN_DFT;
      break;
    default:
      DYNA_LOG_ERROR(-DYNA_ECODE_INTERNAL,
          "Servo %d: Unknown control method %u.",
          m_nServoId, m_spec.m_uCtlMethodUsed);
      break;
  }

  m_state.m_uGoalPos              = 0;
  m_state.m_nGoalSpeed            = 0;
  m_state.m_uLimTorqueMax         = DYNA_LIM_TORQUE_MAX_ON_DFT_RAW;
  m_state.m_uOverTorqueTh         = DYNA_LIM_TORQUE_MAX_ON_DFT_RAW + 1;
  m_state.m_uClearTorqueTh        = DYNA_LIM_TORQUE_MAX_ON_DFT_RAW + 1;
  m_state.m_bOverTorqueCond       = false;
  m_state.m_uCurPos               = 0;
  m_state.m_nCurSpeed             = 0;
  m_state.m_nCurLoad              = 0;
  m_state.m_uCurVolt              = 0;
  m_state.m_uCurTemp              = 0;
  m_state.m_bIsMoving             = false;
  m_state.m_od.m_bOdEnabled       = false;
}

void DynaServoGeneric::CheckData()
{
  /*! \todo TODO check synchronized data for consistencies */
}

int DynaServoGeneric::CalcMatesGoalPos(int nGoalOdPos, int *pGoalOdPosMate)
{
  DynaServo  *pServoMate;
  int         nEncPos;
  int         nGoalOdPosMate;
  uint_t      uVal;
  int         nVal;
  int         delta;
  int         rc;

  // linked mate
  pServoMate = m_link.m_pServoMate;

  // read latest current positions for master and mate
  rc = ReadCurPos(&nVal);

  DYNA_TRY_RC(rc, "Servo %d: Failed.", m_nServoId);

  // mates rotate in reverse directions to each other 
  if( m_link.m_bRotReversed )
  {
    delta = GetOdometer() - nGoalOdPos;

    nGoalOdPosMate = pServoMate->GetOdometer() + delta;

    nEncPos = OdometerToEncoder(nGoalOdPosMate);

    DYNA_TRY_EXPR(((nEncPos >= m_spec.m_uRawPosMin) &&
                   (nEncPos <= m_spec.m_uRawPosMax)),
        DYNA_ECODE_BAD_VAL,
        "Mate servo %d: Goal odometer position %d (encoder=%d): Out of range.",
        m_nServoId, nGoalOdPosMate, nEncPos);
  }

  // same rotation direction
  else
  {
    nGoalOdPosMate = nGoalOdPos;
  }

  if( rc == DYNA_OK )
  {
    *pGoalOdPosMate = nGoalOdPosMate;
  }

  return rc;
}

int DynaServoGeneric::ReadCtlMethodCompliance(DynaServoCtlMethod_T *pCtlMethod)
{
  DynaServoCtlMethod_T *pVal; // working value
  int                   rc;   // return code

  pCtlMethod->m_uCtlMethod = DYNA_CTL_METHOD_COMPLIANCE;

  rc = m_comm.Read8(m_nServoId, DYNA_ADDR_CW_COMP_MARGIN,
                            &(pCtlMethod->m_params.m_comp.m_uCwMargin));

  if( rc == DYNA_OK )
  {
    rc = m_comm.Read8(m_nServoId, DYNA_ADDR_CCW_COMP_MARGIN,
                            &(pCtlMethod->m_params.m_comp.m_uCcwMargin));
  }

  if( rc == DYNA_OK )
  {
    rc = m_comm.Read8(m_nServoId, DYNA_ADDR_CW_COMP_SLOPE,
                            &(pCtlMethod->m_params.m_comp.m_uCwSlope));
  }

  if( rc == DYNA_OK )
  {
    rc = m_comm.Read8(m_nServoId, DYNA_ADDR_CCW_COMP_SLOPE,
                            &(pCtlMethod->m_params.m_comp.m_uCcwSlope));
  }

  if( rc == DYNA_OK )
  {
    m_state.m_ctlMethod = *pCtlMethod;

    if( IsLinkedMaster() )
    {
      rc = m_link.m_pServoMate->ReadControlMethod(pVal);
    }
  }

  return rc;
}

int DynaServoGeneric::WriteCtlMethodCompliance(DynaServoCtlMethod_T &ctlMethod)
{
  uint_t      uCwMargin;    // new cw margin
  uint_t      uCcwMargin;   // new ccw margin
  uint_t      uCwSlope;     // new cw slope
  uint_t      uCcwSlope;    // new ccw slope
  DynaServo  *pServoMate;   // servo mate
  int         rc;           // return code

  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_IS_MASTER(this);

  uCwMargin   = ctlMethod.m_params.m_comp.m_uCwMargin;
  uCcwMargin  = ctlMethod.m_params.m_comp.m_uCcwMargin;
  uCwSlope    = ctlMethod.m_params.m_comp.m_uCwSlope;
  uCcwSlope   = ctlMethod.m_params.m_comp.m_uCcwSlope;

  DYNA_TRY_EXPR((ctlMethod.m_uCtlMethod == m_spec.m_uCtlMethodUsed),
        DYNA_ECODE_NOT_SUPP,
        "Control method %u.", ctlMethod.m_uCtlMethod);

  DYNA_TRY_EXPR(((uCwMargin >= DYNA_COMP_MARGIN_MIN_RAW) &&
                 (uCwMargin <= DYNA_COMP_MARGIN_MAX_RAW)),
        DYNA_ECODE_BAD_VAL,
        "CW compliance margin %u: Out of range.",
        uCwMargin);

  DYNA_TRY_EXPR(((uCcwMargin >= DYNA_COMP_MARGIN_MIN_RAW) &&
                 (uCcwMargin <= DYNA_COMP_MARGIN_MAX_RAW)),
        DYNA_ECODE_BAD_VAL,
        "CCW compliance margin %u: Out of range.",
        uCcwMargin);

  DYNA_TRY_EXPR(ChkComplianceSlope(uCwSlope),
        DYNA_ECODE_BAD_VAL,
        "CW compliance slope 0x%02x: Unknown value.",
        uCwSlope);

  DYNA_TRY_EXPR(ChkComplianceSlope(uCcwSlope),
        DYNA_ECODE_BAD_VAL,
        "CCW compliance slope 0x%02x: Unknown value.",
        uCcwSlope);

  //
  // Linked pair
  //
  if( IsLinkedMaster() )
  {
    pServoMate = m_link.m_pServoMate;

    rc = m_comm.vSyncWrite(DYNA_ADDR_CW_COMP_MARGIN, 1, 2,
                            m_nServoId, uCwMargin,
                            m_link.m_pServoMate->GetServoId(), uCwMargin);

    if( rc == DYNA_OK )
    {
      m_state.m_ctlMethod.m_params.m_comp.m_uCwMargin = uCwMargin;
      pServoMate->m_state.m_ctlMethod.m_params.m_comp.m_uCwMargin = uCwMargin;

      rc = m_comm.vSyncWrite(DYNA_ADDR_CCW_COMP_MARGIN, 1, 2,
                            m_nServoId, uCcwMargin,
                            m_link.m_pServoMate->GetServoId(), uCcwMargin);
    }

    if( rc == DYNA_OK )
    {
      m_state.m_ctlMethod.m_params.m_comp.m_uCcwMargin = uCcwMargin;
      pServoMate->m_state.m_ctlMethod.m_params.m_comp.m_uCcwMargin = uCcwMargin;

      rc = m_comm.vSyncWrite(DYNA_ADDR_CW_COMP_SLOPE, 1, 2,
                            m_nServoId, uCwSlope,
                            m_link.m_pServoMate->GetServoId(), uCwSlope);
    }

    if( rc == DYNA_OK )
    {
      m_state.m_ctlMethod.m_params.m_comp.m_uCwSlope = uCwSlope;
      pServoMate->m_state.m_ctlMethod.m_params.m_comp.m_uCwSlope = uCwSlope;

      rc = m_comm.vSyncWrite(DYNA_ADDR_CCW_COMP_SLOPE, 1, 2,
                            m_nServoId, uCcwSlope,
                            m_link.m_pServoMate->GetServoId(), uCcwSlope);
    }

    if( rc == DYNA_OK )
    {
      m_state.m_ctlMethod.m_params.m_comp.m_uCcwSlope = uCcwSlope;
      pServoMate->m_state.m_ctlMethod.m_params.m_comp.m_uCcwSlope = uCcwSlope;
    }
  }

  //
  // Solitary servo
  //
  else
  {
    rc = m_comm.Write8(m_nServoId, DYNA_ADDR_CW_COMP_MARGIN, uCwMargin);

    if( rc == DYNA_OK )
    {
      m_state.m_ctlMethod.m_params.m_comp.m_uCwMargin = uCwMargin;
      rc = m_comm.Write8(m_nServoId, DYNA_ADDR_CCW_COMP_MARGIN, uCcwMargin);
    }

    if( rc == DYNA_OK )
    {
      m_state.m_ctlMethod.m_params.m_comp.m_uCcwMargin = uCcwMargin;
      rc = m_comm.Write8(m_nServoId, DYNA_ADDR_CW_COMP_SLOPE, uCwSlope);
    }

    if( rc == DYNA_OK )
    {
      m_state.m_ctlMethod.m_params.m_comp.m_uCwSlope = uCwSlope;
      rc = m_comm.Write8(m_nServoId, DYNA_ADDR_CCW_COMP_SLOPE, uCcwSlope);
    }

    if( rc == DYNA_OK )
    {
      m_state.m_ctlMethod.m_params.m_comp.m_uCcwSlope = uCcwSlope;
    }
  }

  return rc;
}

int DynaServoGeneric::ReadCtlMethodPid(DynaServoCtlMethod_T *pCtlMethod)
{
  DynaServoCtlMethod_T *pVal; // working value
  int                   rc;   // return code

  pCtlMethod->m_uCtlMethod = DYNA_CTL_METHOD_PID;

  rc = m_comm.Read8(m_nServoId, DYNA_ADDR_P_GAIN,
                            &(pCtlMethod->m_params.m_pid.m_uPGain));

  if( rc == DYNA_OK )
  {
    rc = m_comm.Read8(m_nServoId, DYNA_ADDR_I_GAIN,
                            &(pCtlMethod->m_params.m_pid.m_uIGain));
  }

  if( rc == DYNA_OK )
  {
    rc = m_comm.Read8(m_nServoId, DYNA_ADDR_D_GAIN,
                            &(pCtlMethod->m_params.m_pid.m_uDGain));
  }

  if( rc == DYNA_OK )
  {
    m_state.m_ctlMethod = *pCtlMethod;

    if( IsLinkedMaster() )
    {
      rc = m_link.m_pServoMate->ReadControlMethod(pVal);
    }
  }

  return rc;
}

int DynaServoGeneric::WriteCtlMethodPid(DynaServoCtlMethod_T &ctlMethod)
{
  uint_t      uPGain;       // new p gain
  uint_t      uIGain;       // new i gain
  uint_t      uDGain;       // new d gain
  DynaServo  *pServoMate;   // servo mate
  int         rc;           // return code

  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_IS_MASTER(this);

  uPGain  = ctlMethod.m_params.m_pid.m_uPGain;
  uIGain  = ctlMethod.m_params.m_pid.m_uIGain;
  uDGain  = ctlMethod.m_params.m_pid.m_uDGain;

  DYNA_TRY_EXPR((ctlMethod.m_uCtlMethod == m_spec.m_uCtlMethodUsed),
        DYNA_ECODE_NOT_SUPP,
        "Control method %u.", ctlMethod.m_uCtlMethod);

  DYNA_TRY_EXPR(((uPGain >= DYNA_P_GAIN_MIN_RAW) &&
                 (uPGain <= DYNA_P_GAIN_MAX_RAW)),
        DYNA_ECODE_BAD_VAL,
        "P-Gain %u: Out of range.", uPGain);

  DYNA_TRY_EXPR(((uIGain >= DYNA_I_GAIN_MIN_RAW) &&
                 (uIGain <= DYNA_I_GAIN_MAX_RAW)),
        DYNA_ECODE_BAD_VAL,
        "I-Gain %u: Out of range.", uIGain);

  DYNA_TRY_EXPR(((uDGain >= DYNA_D_GAIN_MIN_RAW) &&
                 (uDGain <= DYNA_D_GAIN_MAX_RAW)),
        DYNA_ECODE_BAD_VAL,
        "D-Gain %u: Out of range.", uDGain);

  //
  // Linked pair
  //
  if( IsLinkedMaster() )
  {
    pServoMate = m_link.m_pServoMate;

    rc = m_comm.vSyncWrite(DYNA_ADDR_P_GAIN, 1, 2,
                            m_nServoId, uPGain,
                            m_link.m_pServoMate->GetServoId(), uPGain);

    if( rc == DYNA_OK )
    {
      m_state.m_ctlMethod.m_params.m_pid.m_uPGain = uPGain;
      pServoMate->m_state.m_ctlMethod.m_params.m_pid.m_uPGain = uPGain;

      rc = m_comm.vSyncWrite(DYNA_ADDR_I_GAIN, 1, 2,
                            m_nServoId, uIGain,
                            m_link.m_pServoMate->GetServoId(), uIGain);
    }

    if( rc == DYNA_OK )
    {
      m_state.m_ctlMethod.m_params.m_pid.m_uIGain = uIGain;
      pServoMate->m_state.m_ctlMethod.m_params.m_pid.m_uIGain = uIGain;

      rc = m_comm.vSyncWrite(DYNA_ADDR_D_GAIN, 1, 2,
                            m_nServoId, uDGain,
                            m_link.m_pServoMate->GetServoId(), uDGain);
    }

    if( rc == DYNA_OK )
    {
      m_state.m_ctlMethod.m_params.m_pid.m_uDGain = uDGain;
      pServoMate->m_state.m_ctlMethod.m_params.m_pid.m_uDGain = uDGain;
    }
  }

  //
  // Solitary servo
  //
  else
  {
    rc = m_comm.Write8(m_nServoId, DYNA_ADDR_P_GAIN, uPGain);

    if( rc == DYNA_OK )
    {
      m_state.m_ctlMethod.m_params.m_pid.m_uPGain = uPGain;
      rc = m_comm.Write8(m_nServoId, DYNA_ADDR_I_GAIN, uIGain);
    }

    if( rc == DYNA_OK )
    {
      m_state.m_ctlMethod.m_params.m_pid.m_uIGain = uIGain;
      rc = m_comm.Write8(m_nServoId, DYNA_ADDR_D_GAIN, uDGain);
    }

    if( rc == DYNA_OK )
    {
      m_state.m_ctlMethod.m_params.m_pid.m_uDGain = uDGain;
    }
  }

  return rc;
}

bool DynaServoGeneric::ChkComplianceSlope(uint_t uVal)
{
  switch( uVal )
  {
    case DYNA_COMP_SLOPE_TORQUE_1:
    case DYNA_COMP_SLOPE_TORQUE_2:
    case DYNA_COMP_SLOPE_TORQUE_3:
    case DYNA_COMP_SLOPE_TORQUE_4:
    case DYNA_COMP_SLOPE_TORQUE_5:
    case DYNA_COMP_SLOPE_TORQUE_6:
    case DYNA_COMP_SLOPE_TORQUE_7:
      return true;
    default:
      return false;
  }
}

uint_t DynaServoGeneric::PackGoalSpeed(int nGoalSpeed)
{
  if( m_cfg.m_uServoMode == DYNA_MODE_CONTINUOUS )
  {
    return (uint_t)( (iabs(nGoalSpeed) & DYNA_GOAL_SPEED_MAG_MASK) | 
             (nGoalSpeed<0? DYNA_GOAL_SPEED_DIR_CW: DYNA_GOAL_SPEED_DIR_CCW) );
  }
  else
  {
    return (uint_t)iabs(nGoalSpeed);
  }
}

int DynaServoGeneric::UnpackGoalSpeed(uint_t uVal)
{
  int nSpeed;

  nSpeed = (int)(uVal & DYNA_GOAL_SPEED_MAG_MASK);

  if( m_cfg.m_uServoMode == DYNA_MODE_CONTINUOUS )
  {
    if( (uVal & DYNA_GOAL_SPEED_DIR_MASK) == DYNA_GOAL_SPEED_DIR_CW )
    {
      nSpeed = -nSpeed;
    }
  }

  return nSpeed;
}

int DynaServoGeneric::UnpackCurSpeed(uint_t uVal)
{
  int nSpeed;

  nSpeed = (int)(uVal & DYNA_CUR_SPEED_MAG_MASK);

  if( (uVal & DYNA_CUR_SPEED_DIR_MASK) == DYNA_CUR_SPEED_DIR_CW )
  {
    nSpeed = -nSpeed;
  }

  return nSpeed;
}

int DynaServoGeneric::UnpackCurLoad(uint_t uVal)
{
  int nLoad;

  nLoad = (int)(uVal & DYNA_CUR_LOAD_MAG_MASK);

  if( (uVal & DYNA_CUR_LOAD_DIR_MASK) == DYNA_CUR_LOAD_DIR_CW )
  {
    nLoad = -nLoad;
  }

  return nLoad;
}

void DynaServoGeneric::SetServoMode()
{
  if( (m_spec.m_uSupportedModes & DYNA_MODE_CONTINUOUS) &&
      (m_cfg.m_uLimCw  == DYNA_CW_POS_CONT_MODE) &&
      (m_cfg.m_uLimCcw == DYNA_CCW_POS_CONT_MODE) )
  {
    m_cfg.m_uServoMode = DYNA_MODE_CONTINUOUS;
  }

  else
  {
    m_cfg.m_uServoMode = DYNA_MODE_SERVO;
  }
}
