////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   libDynamixel
//
// File:      DynaServoMX12W.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-03-13 13:28:02 -0600 (Fri, 13 Mar 2015) $
 * $Rev: 3890 $
 *
 * \brief RoadNarrows MX-12W Dynamixel servo class.
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
#include "Dynamixel/MX.h"
#include "Dynamixel/DynaError.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaServo.h"
#include "Dynamixel/DynaServoGeneric.h"
#include "Dynamixel/DynaServoMX12W.h"

#include "DynaLibInternal.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief MX-12W Servo EEPROM Control Table Information.
 */
static DynaCtlTblEntry_T MX12WEEPROMCtlTblInfo[] =
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
  {0x12, "Alarm Shutdown",            1, 0xff,    false,  NULL},
  {0x14, "Multi Turn Offset",         2, 0xffff,  true,   "%d"},
  {0x16, "Resolution Divider",        1, 0xff,    false,  "%u"}
};

/*!
 * \brief MX-12W Servo RAM Control Table Information.
 */
static DynaCtlTblEntry_T MX12WRAMCtlTblInfo[] =
{
  {0x18, "Torque Enable",             1, 0xff,    false,  "%u"},
  {0x19, "LED",                       1, 0xff,    false,  "%u"},
  {0x1a, "D Gain",                    1, 0xff,    false,  "%u"},
  {0x1b, "I Gain",                    1, 0xff,    false,  "%u"},
  {0x1c, "P Gain",                    1, 0xff,    false,  "%u"},
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
  {0x49, "Goal Acceleration",         1, 0xff,    false,  "%u"}
};


// ---------------------------------------------------------------------------
// DynaServoMX12W Class
// ---------------------------------------------------------------------------

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Constructors and Destructors
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

DynaServoMX12W::DynaServoMX12W(DynaComm &comm,
                               int       nServoId,
                               uint_t    uModelNum,
                               uint_t    uFwVer) : 
                  DynaServoGeneric(comm)
{
  Init(nServoId, uFwVer); // this class
  SyncData();             // generic
  CheckData();            // generic
}

DynaServoMX12W::~DynaServoMX12W()
{
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Servo Read/Write Functions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void DynaServoMX12W::Dump()
{
  DumpCtlTbl("EEPROM", MX12WEEPROMCtlTblInfo, arraysize(MX12WEEPROMCtlTblInfo));

  printf("\n");

  DumpCtlTbl("RAM", MX12WRAMCtlTblInfo, arraysize(MX12WRAMCtlTblInfo));
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Protected Interface
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

void DynaServoMX12W::Init(int nServoId, uint_t uFwVer)
{
  DynaServo::Init(nServoId, DYNA_MODEL_NUM, uFwVer);

  InitSpec();   // this class
  InitCfg();    // generic
  InitState();  // generic
}

void DynaServoMX12W::InitSpec()
{
  //
  // Fixed servo specification.
  //
  if( m_spec.m_sModelName != NULL )
  {
    delete[] m_spec.m_sModelName;
  }

  // physical and fixed features
  m_spec.m_sModelName           = newstr("MX-12W");
  m_spec.m_fWeight              = DYNA_MX12W_SPEC_WEIGHT_G;
  m_spec.m_fWidth               = DYNA_MX12W_SPEC_WIDTH_MM;
  m_spec.m_fHeight              = DYNA_MX12W_SPEC_HEIGHT_MM;
  m_spec.m_fDepth               = DYNA_MX12W_SPEC_DEPTH_MM;
  m_spec.m_fResolution          = DYNA_MX12W_SPEC_POS_RES_DEG;
  m_spec.m_fGearRedectionRatio  = DYNA_MX12W_SPEC_GEAR_RATIO;
  m_spec.m_fStallTorque         = DYNA_MX12W_SPEC_STALL_TORQUE_KGF;
  m_spec.m_fMaxSpeed            = DYNA_MX12W_SPEC_MAX_SPEED_RPM;
  m_spec.m_fAngleMin            = DYNA_MX12W_SPEC_ANGLE_MIN_DEG;
  m_spec.m_fAngleMax            = DYNA_MX12W_SPEC_ANGLE_MAX_DEG;
  m_spec.m_uSupportedModes      = DYNA_MX12W_SPEC_MODES;
  m_spec.m_bHas360Pos           = DYNA_MX12W_SPEC_HAS_360_POS;
  m_spec.m_fTempMin             = DYNA_MX12W_SPEC_TEMP_MIN_C;
  m_spec.m_fTempMax             = DYNA_MX12W_SPEC_TEMP_MAX_C;
  m_spec.m_fVoltMin             = DYNA_MX12W_SPEC_VOLT_MIN_V;
  m_spec.m_fVoltMax             = DYNA_MX12W_SPEC_VOLT_MAX_V;
  m_spec.m_uCtlMethodUsed       = DYNA_MX12W_SPEC_CTL_METHOD;

  // raw units
  m_spec.m_uRawPosMin           = DYNA_MX12W_POS_MIN_RAW;
  m_spec.m_uRawPosMax           = DYNA_MX12W_POS_MAX_RAW;
  m_spec.m_uRawPosModulo        = DYNA_MX12W_POS_MODULO;
  m_spec.m_uRawSpeedMin         = DYNA_SPEED_MIN_RAW;
  m_spec.m_uRawSpeedMax         = DYNA_SPEED_MAX_RAW;
  m_spec.m_uRawTorqueMin        = DYNA_TORQUE_MIN_RAW;
  m_spec.m_uRawTorqueMax        = DYNA_TORQUE_MAX_RAW;
  m_spec.m_uRawTempMin          = DYNA_TEMP_MIN_RAW;
  m_spec.m_uRawTempMax          = DYNA_TEMP_MAX_RAW;
  m_spec.m_uRawVoltMin          = DYNA_VOLT_MIN_RAW;
  m_spec.m_uRawVoltMax          = DYNA_VOLT_MAX_RAW;
}
