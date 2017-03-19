////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   libDynamixel
//
// File:      DynaServoRX64.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
 * $Rev: 3845 $
 *
 * \brief RX-64 Dynamixel servo class.
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
#include "Dynamixel/RX.h"
#include "Dynamixel/DynaError.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaServo.h"
#include "Dynamixel/DynaServoGeneric.h"
#include "Dynamixel/DynaServoRX64.h"

#include "DynaLibInternal.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------


// ---------------------------------------------------------------------------
// DynaServoRX64 Class
// ---------------------------------------------------------------------------

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Constructors and Destructors
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

DynaServoRX64::DynaServoRX64(DynaComm &comm,
                               int       nServoId,
                               uint_t    uModelNum,
                               uint_t    uFwVer) : 
                  DynaServoGeneric(comm)
{
  Init(nServoId, uFwVer); // this class
  SyncData();             // generic
  CheckData();            // generic
}

DynaServoRX64::~DynaServoRX64()
{
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Servo Read/Write Functions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Protected Interface
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

void DynaServoRX64::Init(int nServoId, uint_t uFwVer)
{
  DynaServo::Init(nServoId, DYNA_MODEL_NUM, uFwVer);

  InitSpec();   // this class
  InitCfg();    // generic
  InitState();  // generic
}

void DynaServoRX64::InitSpec()
{
  //
  // Fixed servo specification.
  //
  if( m_spec.m_sModelName != NULL )
  {
    delete[] m_spec.m_sModelName;
  }

  // physical and fixed features
  m_spec.m_sModelName           = newstr("RX-64");
  m_spec.m_fWeight              = DYNA_RX64_SPEC_WEIGHT_G;
  m_spec.m_fWidth               = DYNA_RX64_SPEC_WIDTH_MM;
  m_spec.m_fHeight              = DYNA_RX64_SPEC_HEIGHT_MM;
  m_spec.m_fDepth               = DYNA_RX64_SPEC_DEPTH_MM;
  m_spec.m_fResolution          = DYNA_RX64_SPEC_POS_RES_DEG;
  m_spec.m_fGearRedectionRatio  = DYNA_RX64_SPEC_GEAR_RATIO;
  m_spec.m_fStallTorque         = DYNA_RX64_SPEC_STALL_TORQUE_KGF;
  m_spec.m_fMaxSpeed            = DYNA_RX64_SPEC_MAX_SPEED_RPM;
  m_spec.m_fAngleMin            = DYNA_RX64_SPEC_ANGLE_MIN_DEG;
  m_spec.m_fAngleMax            = DYNA_RX64_SPEC_ANGLE_MAX_DEG;
  m_spec.m_uSupportedModes      = DYNA_RX64_SPEC_MODES;
  m_spec.m_bHas360Pos           = DYNA_RX64_SPEC_HAS_360_POS;
  m_spec.m_fTempMin             = DYNA_RX64_SPEC_TEMP_MIN_C;
  m_spec.m_fTempMax             = DYNA_RX64_SPEC_TEMP_MAX_C;
  m_spec.m_fVoltMin             = DYNA_RX64_SPEC_VOLT_MIN_V;
  m_spec.m_fVoltMax             = DYNA_RX64_SPEC_VOLT_MAX_V;
  m_spec.m_uCtlMethodUsed       = DYNA_RX64_SPEC_CTL_METHOD;

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
