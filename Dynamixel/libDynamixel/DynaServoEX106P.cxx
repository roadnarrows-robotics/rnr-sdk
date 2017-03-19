////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   libDynamixel
//
// File:      DynaServoEX106P.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-03-13 13:28:02 -0600 (Fri, 13 Mar 2015) $
 * $Rev: 3890 $
 *
 * \brief EX-106+ Dynamixel servo class.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
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
#include <stdlib.h>
#include <libgen.h>
#include <string.h>
#include <stdarg.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/new.h"
#include "rnr/units.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/EX.h"
#include "Dynamixel/DynaError.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaServo.h"
#include "Dynamixel/DynaServoGeneric.h"
#include "Dynamixel/DynaServoEX106P.h"

#include "DynaLibInternal.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief EX-106P Servo EEPROM Control Table Information.
 */
static DynaCtlTblEntry_T EX106PEEPROMCtlTblInfo[] =
{
  {0x00, "Model Number",              2, 0xffff,  false,  NULL},
  {0x02, "Firmware Version",          1, 0xff,    false,  "%u"},
  {0x03, "Servo Id",                  1, 0xff,    false,  "%u"},
  {0x04, "Baud Rate",                 1, 0xff,    false,  "%u"},
  {0x05, "Return Delay Time",         1, 0xff,    false,  "%u"},
  {0x06, "CW Angle Limit",            2, 0x3ff,   false,  "%u"},
  {0x08, "CCW Angle Limit",           2, 0x3ff,   false,  "%u"},
  {0x0a, "Drive Mode",                1, 0xff,    false,  "%u"},
  {0x0b, "Highest Temperature Limit", 1, 0xff,    false,  "%u"},
  {0x0c, "Lowest Voltage Limit",      1, 0xff,    false,  "%u"},
  {0x0d, "Highest Voltage Limit",     1, 0xff,    false,  "%u"},
  {0x0e, "Maximum Torque",            2, 0x3ff,   false,  "%u"},
  {0x10, "Status Return Level",       1, 0xff,    false,  "%u"},
  {0x11, "Alarm LED",                 1, 0xff,    false,  "%u"},
  {0x12, "Alarm Shutdown",            1, 0xff,    false,  NULL},
  {0x14, "Multi Turn Offset",         2, 0xffff,  true,   "%d"},
  {0x16, "Resolution Divider",        1, 0xff,    false,  "%u"}
};

/*!
 * \brief EX-106P Servo RAM Control Table Information.
 */
static DynaCtlTblEntry_T EX106PRAMCtlTblInfo[] =
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
  {0x30, "Punch",                     2, 0x3ff,   false,  "%u"},
  {0x38, "Sensed Current",            2, 0x3ff,   true,   "%d"}
};


// ---------------------------------------------------------------------------
// DynaServoEX106P Class
// ---------------------------------------------------------------------------

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Constructors and Destructors
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

DynaServoEX106P::DynaServoEX106P(DynaComm &comm,
                               int       nServoId,
                               uint_t    uModelNum,
                               uint_t    uFwVer) : 
                  DynaServoGeneric(comm)
{
  Init(nServoId, uFwVer); // this class
  SyncData();             // this class
  CheckData();            // this class
}

DynaServoEX106P::~DynaServoEX106P()
{
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Servo Read/Write Functions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int DynaServoEX106P::CfgReadDriveMode(bool *pIsMaster, bool *pIsNormal)
{
  uint_t            uVal;           // working value
  bool              bVal1, bVal2;   // working booleans
  DynaServoEX106P  *pServoMate;     // servo mate
  int               rc;             // return code

  DYNA_TRY_COMM(m_comm);

  rc = m_comm.Read8(m_nServoId, DYNA_EX106P_ADDR_DRIVE_MODE, &uVal);

  if( rc == DYNA_OK )
  {
    UnpackDriveMode(uVal, &m_cfgExt.m_bDriveModeIsMaster,
                          &m_cfgExt.m_bDriveModeIsNormal);

    *pIsMaster  = m_cfgExt.m_bDriveModeIsMaster;
    *pIsNormal  = m_cfgExt.m_bDriveModeIsNormal;

    if( IsLinkedMaster() )
    {
      pServoMate = (DynaServoEX106P *)(m_link.m_pServoMate);

      rc = pServoMate->CfgReadDriveMode(&bVal1, &bVal2);
    }
  }

  return rc;
}

int DynaServoEX106P::CfgWriteDriveMode(bool bIsMaster, bool bIsNormal)
{
  uint_t            uVal1, uVal2;   // working values
  DynaServoEX106P  *pServoMate;     // servo mate
  int               rc;             // return code

  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_IS_MASTER(this);

  uVal1 = PackDriveMode(bIsMaster, bIsNormal);

  //
  // Linked pair
  //
  if( IsLinkedMaster() )
  {
    DYNA_TRY_EXPR((bIsMaster), DYNA_ECODE_BAD_VAL,
      "Linked master servo %d: Drive mode must also be master.", m_nServoId);

    pServoMate = (DynaServoEX106P *)(m_link.m_pServoMate);

    uVal2 = PackDriveMode(false, bIsNormal);

    rc = m_comm.vSyncWrite(DYNA_EX106P_ADDR_DRIVE_MODE, 1, 2,
                        m_nServoId, uVal1,
                        pServoMate->GetServoId(), uVal2);

    if( rc == DYNA_OK )
    {
      m_cfgExt.m_bDriveModeIsMaster = true;
      m_cfgExt.m_bDriveModeIsNormal = bIsNormal;

      pServoMate->m_cfgExt.m_bDriveModeIsMaster = false;
      pServoMate->m_cfgExt.m_bDriveModeIsNormal = bIsNormal;
    }
  }

  //
  // Solitary servo
  //
  else
  {
    rc = m_comm.Write8(m_nServoId, DYNA_EX106P_ADDR_DRIVE_MODE, uVal1);

    if( rc == DYNA_OK )
    {
      m_cfgExt.m_bDriveModeIsMaster = bIsMaster;
      m_cfgExt.m_bDriveModeIsNormal = bIsNormal;
    }
  }

  return rc;
}

int DynaServoEX106P::ReadSensedCurrent(uint_t *pMilliAmps, uint_t *pTorqueDir)
{
  uint_t            uVal1, uVal2;   // working values
  DynaServoEX106P  *pServoMate;     // servo mate
  int               rc;             // return code

  DYNA_TRY_COMM(m_comm);

  rc = m_comm.Read16(m_nServoId, DYNA_EX106P_ADDR_CURRENT_LSB, &uVal1);

  if( rc == DYNA_OK )
  {
    UnpackSensedCurrent(uVal1, &m_stateExt.m_uSensedCurrentMilliAmps,
                               &m_stateExt.m_uSensedCurrentTorqueDir);

    *pMilliAmps  = m_stateExt.m_uSensedCurrentMilliAmps;
    *pTorqueDir  = m_stateExt.m_uSensedCurrentTorqueDir;

    if( IsLinkedMaster() )
    {
      pServoMate = (DynaServoEX106P *)(m_link.m_pServoMate);

      rc = pServoMate->ReadSensedCurrent(&uVal1, &uVal2);
    }
  }

  return rc;
}

int DynaServoEX106P::Read(uint_t uAddr, uint_t *pVal)
{
  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_ADDR(uAddr);

  switch( uAddr )
  {
    case DYNA_EX106P_ADDR_CURRENT_LSB:
      return m_comm.Read16(m_nServoId, uAddr, pVal);

    case DYNA_EX106P_ADDR_DRIVE_MODE:
      return m_comm.Read8(m_nServoId, uAddr, pVal);

    default:
      return DynaServoGeneric::Read(uAddr, pVal);
  }
}

int DynaServoEX106P::Write(uint_t uAddr, uint_t uVal)
{
  DYNA_TRY_COMM(m_comm);
  DYNA_TRY_ADDR(uAddr);

  switch( uAddr )
  {
    case DYNA_EX106P_ADDR_CURRENT_LSB:
      return m_comm.Write16(m_nServoId, uAddr, uVal);

    case DYNA_EX106P_ADDR_DRIVE_MODE:
      return m_comm.Write8(m_nServoId, uAddr, uVal);

    default:
      return DynaServoGeneric::Write(uAddr, uVal);
  }
}

int DynaServoEX106P::SyncCfg()
{
  bool    bVal1, bVal2;   // working booleans

  DYNA_TRY_COMM(m_comm);

  DynaServoGeneric::SyncCfg();

  CfgReadDriveMode(&bVal1, &bVal2);

  return DYNA_OK;
}

int DynaServoEX106P::SyncState()
{
  uint_t  uVal1, uVal2;   // working values
  
  DYNA_TRY_COMM(m_comm);

  DynaServoGeneric::SyncState();

  ReadSensedCurrent(&uVal1, &uVal2);

  return DYNA_OK;
}

void DynaServoEX106P::Dump()
{
  DumpCtlTbl("EEPROM",
                EX106PEEPROMCtlTblInfo, arraysize(EX106PEEPROMCtlTblInfo));

  printf("\n");

  DumpCtlTbl("RAM", EX106PRAMCtlTblInfo, arraysize(EX106PRAMCtlTblInfo));
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Protected Interface
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

void DynaServoEX106P::Init(int nServoId, uint_t uFwVer)
{
  DynaServo::Init(nServoId, DYNA_MODEL_NUM, uFwVer);

  InitSpec();   // this class
  InitCfg();    // this class
  InitState();  // this class
}

void DynaServoEX106P::InitSpec()
{
  //
  // Fixed servo specification.
  //
  if( m_spec.m_sModelName != NULL )
  {
    delete[] m_spec.m_sModelName;
  }

  // physical and fixed features
  m_spec.m_sModelName           = newstr("EX-106+");
  m_spec.m_fWeight              = DYNA_EX106P_SPEC_WEIGHT_G;
  m_spec.m_fWidth               = DYNA_EX106P_SPEC_WIDTH_MM;
  m_spec.m_fHeight              = DYNA_EX106P_SPEC_HEIGHT_MM;
  m_spec.m_fDepth               = DYNA_EX106P_SPEC_DEPTH_MM;
  m_spec.m_fResolution          = DYNA_EX106P_SPEC_POS_RES_DEG;
  m_spec.m_fGearRedectionRatio  = DYNA_EX106P_SPEC_GEAR_RATIO;
  m_spec.m_fStallTorque         = DYNA_EX106P_SPEC_STALL_TORQUE_KGF;
  m_spec.m_fMaxSpeed            = DYNA_EX106P_SPEC_MAX_SPEED_RPM;
  m_spec.m_fAngleMin            = DYNA_EX106P_SPEC_ANGLE_MIN_DEG;
  m_spec.m_fAngleMax            = DYNA_EX106P_SPEC_ANGLE_MAX_DEG;
  m_spec.m_uSupportedModes      = DYNA_EX106P_SPEC_MODES;
  m_spec.m_bHas360Pos           = DYNA_EX106P_SPEC_HAS_360_POS;
  m_spec.m_fTempMin             = DYNA_EX106P_SPEC_TEMP_MIN_C;
  m_spec.m_fTempMax             = DYNA_EX106P_SPEC_TEMP_MAX_C;
  m_spec.m_fVoltMin             = DYNA_EX106P_SPEC_VOLT_MIN_V;
  m_spec.m_fVoltMax             = DYNA_EX106P_SPEC_VOLT_MAX_V;
  m_spec.m_uCtlMethodUsed       = DYNA_EX106P_SPEC_CTL_METHOD;

  // raw units
  m_spec.m_uRawPosMin           = DYNA_EX106P_POS_MIN_RAW;
  m_spec.m_uRawPosMax           = DYNA_EX106P_POS_MAX_RAW;
  m_spec.m_uRawPosModulo        = DYNA_EX106P_POS_MODULO;
  m_spec.m_uRawSpeedMin         = DYNA_SPEED_MIN_RAW;
  m_spec.m_uRawSpeedMax         = DYNA_SPEED_MAX_RAW;
  m_spec.m_uRawTorqueMin        = DYNA_TORQUE_MIN_RAW;
  m_spec.m_uRawTorqueMax        = DYNA_TORQUE_MAX_RAW;
  m_spec.m_uRawTempMin          = DYNA_TEMP_MIN_RAW;
  m_spec.m_uRawTempMax          = DYNA_TEMP_MAX_RAW;
  m_spec.m_uRawVoltMin          = DYNA_VOLT_MIN_RAW;
  m_spec.m_uRawVoltMax          = DYNA_VOLT_MAX_RAW;
}

void DynaServoEX106P::InitCfg()
{
  DynaServoGeneric::InitCfg();

  m_cfgExt.m_bDriveModeIsMaster   = true;
  m_cfgExt.m_bDriveModeIsNormal = false;
}

void DynaServoEX106P::InitState()
{
  DynaServoGeneric::InitState();

  m_stateExt.m_uSensedCurrentMilliAmps  = 0;
  m_stateExt.m_uSensedCurrentTorqueDir  = DYNA_DIR_NONE;
}

uint_t DynaServoEX106P::PackDriveMode(bool bIsMaster, bool bIsNormal)
{
  uint_t  uVal;

  uVal = bIsMaster? DYNA_EX106P_DRIVE_MODE_MS_MASTER:
                    DYNA_EX106P_DRIVE_MODE_MS_SLAVE;

  uVal |= bIsNormal?  DYNA_EX106P_DRIVE_MODE_NR_NORM:
                      DYNA_EX106P_DRIVE_MODE_NR_REV;

  return uVal;
}

int DynaServoEX106P::UnpackDriveMode(uint_t  uVal,
                                     bool   *pIsMaster,
                                     bool   *pIsNormal)
{
  *pIsMaster = uVal & DYNA_EX106P_DRIVE_MODE_MS_SLAVE? false: true;

  *pIsNormal = uVal & DYNA_EX106P_DRIVE_MODE_NR_REV? false: true;

  return DYNA_OK;
}

int DynaServoEX106P::UnpackSensedCurrent(uint_t  uVal,
                                         uint_t *pMilliAmps,
                                         uint_t *pTorqueDir)
{
  *pMilliAmps = DYNA_EX106P_CURRENT_M_AMP(uVal) * DYNA_EX106P_CURRENT_RES_M_AMP;

  *pTorqueDir = DYNA_EX106P_CURRENT_TORQUE_DIR(uVal);

  return DYNA_OK;
}
