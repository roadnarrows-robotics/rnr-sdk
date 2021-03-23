////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekState.cxx
//
/*! \file
 *
 * $LastChangedDate: 2014-09-18 16:53:49 -0600 (Thu, 18 Sep 2014) $
 * $Rev: 3748 $
 *
 * \brief Hekateros Robot State classes implementations.
 *
 * \author Robin Knight     (robin.knight@roadnarrows.com)
 * \author Daniel Packard   (daniel@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2012-2017. RoadNarrows LLC.\n
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

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaComm.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekState.h"

using namespace std;
using namespace hekateros;


// -----------------------------------------------------------------------------
// Class HekServoHealth
// -----------------------------------------------------------------------------

HekServoHealth::HekServoHealth()
{
  m_nServoId      = DYNA_ID_NONE;
  m_fTemperature  = 0.0;
  m_fVoltage      = 0.0;
  m_uAlarms       = DYNA_ALARM_NONE;
}

HekServoHealth::HekServoHealth(const HekServoHealth &src)
{
  m_nServoId      = src.m_nServoId;
  m_fTemperature  = src.m_fTemperature;
  m_fVoltage      = src.m_fVoltage;
  m_uAlarms       = src.m_uAlarms;
}

HekServoHealth &HekServoHealth::operator=(const HekServoHealth &rhs)
{
  m_nServoId      = rhs.m_nServoId;
  m_fTemperature  = rhs.m_fTemperature;
  m_fVoltage      = rhs.m_fVoltage;
  m_uAlarms       = rhs.m_uAlarms;

  return *this;
}


// -----------------------------------------------------------------------------
// Class HekRobotState
// -----------------------------------------------------------------------------

HekRobotState::HekRobotState(const HekRobotState &src)
{
  m_eRobotMode        = src.m_eRobotMode;
  m_eIsCalibrated     = src.m_eIsCalibrated;
  m_eIsEStopped       = src.m_eIsEStopped;
  m_eAreDrivesPowered = src.m_eAreDrivesPowered;
  m_eIsMotionPossible = src.m_eIsMotionPossible;
  m_eIsInMotion       = src.m_eIsInMotion;
  m_eIsInError        = src.m_eIsInError;
  m_nErrorCode        = src.m_nErrorCode;
  m_vecServoHealth    = src.m_vecServoHealth;
}

HekRobotState &HekRobotState::operator=(const HekRobotState &rhs)
{
  m_eRobotMode        = rhs.m_eRobotMode;
  m_eIsCalibrated     = rhs.m_eIsCalibrated;
  m_eIsEStopped       = rhs.m_eIsEStopped;
  m_eAreDrivesPowered = rhs.m_eAreDrivesPowered;
  m_eIsMotionPossible = rhs.m_eIsMotionPossible;
  m_eIsInMotion       = rhs.m_eIsInMotion;
  m_eIsInError        = rhs.m_eIsInError;
  m_nErrorCode        = rhs.m_nErrorCode;
  m_vecServoHealth    = rhs.m_vecServoHealth;

  return *this;
}

void HekRobotState::clear()
{
  m_eRobotMode        = HekRobotModeUnknown;
  m_eIsCalibrated     = HekTriStateUnknown;
  m_eIsEStopped       = HekTriStateUnknown;
  m_eAreDrivesPowered = HekTriStateUnknown;
  m_eIsMotionPossible = HekTriStateUnknown;
  m_eIsInMotion       = HekTriStateUnknown;
  m_eIsInError        = HekTriStateUnknown;
  m_nErrorCode        = HEK_OK;
}
