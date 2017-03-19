////////////////////////////////////////////////////////////////////////////////
//
// Package:   Kuon
//
// Library:   libkuon
//
// File:      kuonStatus.cxx
//
/*! \file
 *
 * $LastChangedDate: 2014-04-04 17:05:03 -0600 (Fri, 04 Apr 2014) $
 * $Rev: 3629 $
 *
 * \brief Kuon Robot Status classes implementations.
 *
 * \author Robin Knight     (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2014-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
/*
 * @EulaBegin@
// Unless otherwise noted, all materials contained are copyrighted and may not
// be used except as provided in these terms and conditions or in the copyright
// notice (documents and software ) or other proprietary notice provided with
// the relevant materials.
//
//
// IN NO EVENT SHALL THE AUTHOR, ROADNARROWS, OR ANY MEMBERS/EMPLOYEES/
// CONTRACTORS OF ROADNARROWS OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
// PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
// EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.
//
// THE AUTHORS AND  ROADNARROWS SPECIFICALLY DISCLAIM ANY WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
// "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
// PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
//
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////


#include <stdio.h>

#include <string>
#include <vector>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Kuon/kuon.h"
#include "Kuon/kuonStatus.h"

using namespace std;
using namespace kuon;


// -----------------------------------------------------------------------------
// Class KuonMotorHealth
// -----------------------------------------------------------------------------

KuonMotorHealth::KuonMotorHealth()
{
  m_nMotorId      = KuonMotorIdNone;
  m_fTemperature  = 0.0;
  m_fVoltage      = 0.0;
  m_uAlarms       = KUON_ALARM_NONE;
}

KuonMotorHealth::KuonMotorHealth(const KuonMotorHealth &src)
{
  m_strName       = src.m_strName;
  m_nMotorId      = src.m_nMotorId;
  m_fTemperature  = src.m_fTemperature;
  m_fVoltage      = src.m_fVoltage;
  m_uAlarms       = src.m_uAlarms;
}

KuonMotorHealth KuonMotorHealth::operator=(const KuonMotorHealth &rhs)
{
  m_strName       = rhs.m_strName;
  m_nMotorId      = rhs.m_nMotorId;
  m_fTemperature  = rhs.m_fTemperature;
  m_fVoltage      = rhs.m_fVoltage;
  m_uAlarms       = rhs.m_uAlarms;

  return *this;
}


// -----------------------------------------------------------------------------
// Class KuonRobotStatus
// -----------------------------------------------------------------------------

KuonRobotStatus::KuonRobotStatus(const KuonRobotStatus &src)
{
  m_eRobotMode        = src.m_eRobotMode;
  m_eIsEStopped       = src.m_eIsEStopped;
  m_eAreDrivesPowered = src.m_eAreDrivesPowered;
  m_eIsMotionPossible = src.m_eIsMotionPossible;
  m_eIsInMotion       = src.m_eIsInMotion;
  m_eIsInError        = src.m_eIsInError;
  m_nErrorCode        = src.m_nErrorCode;
  m_fGovernor         = src.m_fGovernor;
  m_fBattery          = src.m_fBattery;
  m_vecMotorHealth    = src.m_vecMotorHealth;
}

KuonRobotStatus KuonRobotStatus::operator=(const KuonRobotStatus &rhs)
{
  m_eRobotMode        = rhs.m_eRobotMode;
  m_eIsEStopped       = rhs.m_eIsEStopped;
  m_eAreDrivesPowered = rhs.m_eAreDrivesPowered;
  m_eIsMotionPossible = rhs.m_eIsMotionPossible;
  m_eIsInMotion       = rhs.m_eIsInMotion;
  m_eIsInError        = rhs.m_eIsInError;
  m_nErrorCode        = rhs.m_nErrorCode;
  m_fGovernor         = rhs.m_fGovernor;
  m_fBattery          = rhs.m_fBattery;
  m_vecMotorHealth    = rhs.m_vecMotorHealth;

  return *this;
}

void KuonRobotStatus::clear()
{
  m_eRobotMode        = KuonRobotModeUnknown;
  m_eIsEStopped       = KuonTriStateUnknown;
  m_eAreDrivesPowered = KuonTriStateUnknown;
  m_eIsMotionPossible = KuonTriStateUnknown;
  m_eIsInMotion       = KuonTriStateUnknown;
  m_eIsInError        = KuonTriStateUnknown;
  m_nErrorCode        = KUON_OK;
  m_fGovernor         = 0.20;
  m_fBattery          = 0.0;
  m_vecMotorHealth.clear();
}
