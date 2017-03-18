////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Library:   liblaelaps
//
// File:      laePowertrain.cxx
//
/*! \file
 *
 * $LastChangedDate: 2016-02-15 15:44:49 -0700 (Mon, 15 Feb 2016) $
 * $Rev: 4320 $
 *
 * \brief Laelaps powertrain class implementations.
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

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Laelaps/RoboClaw.h"

#include "Laelaps/laelaps.h"
#include "Laelaps/laeTune.h"
#include "Laelaps/laeUtils.h"
#include "Laelaps/laeMotor.h"
#include "Laelaps/laeDesc.h"
#include "Laelaps/laeDb.h"
#include "Laelaps/laePowertrain.h"

using namespace std;
using namespace laelaps;
using namespace motor::roboclaw;


// -----------------------------------------------------------------------------
// Class LaePowertrainAttr
// -----------------------------------------------------------------------------

LaePowertrainAttr::LaePowertrainAttr()
{
  // (derived) attributes
  m_nMotorId            = LaeMotorIdNone;
  m_nMotorCtlrId        = LaeMotorCtlrIdNone;
  m_nMotorIndex         = 0;

  m_nMotorDir           = LaeMotorDirNormal;
  m_eJointType          = LaeJointTypeUnknown;

  m_uPulsesPerRev       = 0.0;
  m_fGearRatio          = 1.0;
  m_fMaxRps             = 0.0;
  m_uMaxQpps            = 1;
  m_fMotorRadsPerPulse  = 0.0;
  m_fWheelRadsPerPulse  = 0.0;

  m_fMaxAmps            = 0.0;
  m_fStallTorque        = 0.0;

  m_fTireRadius         = 0.0;
  m_fTireWidth          = 0.0;
  m_fMetersPerPulse     = 0.0;
  m_fMetersPerRadian    = 0.0;
}

LaePowertrainAttr::LaePowertrainAttr(const LaePowertrainAttr &src)
{
  // (derived) attributes
  m_nMotorId            = src.m_nMotorId;
  m_nMotorCtlrId        = src.m_nMotorCtlrId;
  m_nMotorIndex         = src.m_nMotorIndex;

  m_nMotorDir           = src.m_nMotorDir;
  m_eJointType          = src.m_eJointType;

  m_uPulsesPerRev       = src.m_uPulsesPerRev;
  m_fGearRatio          = src.m_fGearRatio;
  m_fMaxRps             = src.m_fMaxRps;
  m_uMaxQpps            = src.m_uMaxQpps;
  m_fMotorRadsPerPulse  = src.m_fMotorRadsPerPulse;
  m_fWheelRadsPerPulse  = src.m_fWheelRadsPerPulse;

  m_fMaxAmps            = src.m_fMaxAmps;
  m_fStallTorque        = src.m_fStallTorque;

  m_fTireRadius         = src.m_fTireRadius;
  m_fTireWidth          = src.m_fTireWidth;
  m_fMetersPerPulse     = src.m_fMetersPerPulse;
  m_fMetersPerRadian    = src.m_fMetersPerRadian;
}

LaePowertrainAttr LaePowertrainAttr::operator=(const LaePowertrainAttr &rhs)
{
  // (derived) attributes
  m_nMotorId            = rhs.m_nMotorId;
  m_nMotorCtlrId        = rhs.m_nMotorCtlrId;
  m_nMotorIndex         = rhs.m_nMotorIndex;

  m_nMotorDir           = rhs.m_nMotorDir;
  m_eJointType          = rhs.m_eJointType;

  m_uPulsesPerRev       = rhs.m_uPulsesPerRev;
  m_fMaxRps             = rhs.m_fMaxRps;
  m_uMaxQpps            = rhs.m_uMaxQpps;
  m_fMotorRadsPerPulse  = rhs.m_fMotorRadsPerPulse;
  m_fGearRatio          = rhs.m_fGearRatio;
  m_fWheelRadsPerPulse  = rhs.m_fWheelRadsPerPulse;

  m_fMaxAmps            = rhs.m_fMaxAmps;
  m_fStallTorque        = rhs.m_fStallTorque;

  m_fTireRadius         = rhs.m_fTireRadius;
  m_fTireWidth          = rhs.m_fTireWidth;
  m_fMetersPerPulse     = rhs.m_fMetersPerPulse;
  m_fMetersPerRadian    = rhs.m_fMetersPerRadian;

  return *this;
}

// -----------------------------------------------------------------------------
// Class LaePowertrainState
// -----------------------------------------------------------------------------

LaePowertrainState::LaePowertrainState()
{
  m_nEncoder  = 0;
  m_nSpeed    = 0;
  m_fTemp     = 0.0;
  m_fVolts    = 0.0;
  m_fAmps     = 0.0;
  m_uBufLen   = 0;
  m_uAlarms   = LaeMotorAlarmNone;
  m_uWarnings = LaeMotorWarnNone;

  m_fPosition = 0.0;
  m_fVelocity = 0.0;
  m_fTorque   = 0.0;
  m_fPe       = 0.0;
  m_fPm       = 0.0;
}

LaePowertrainState::LaePowertrainState(const LaePowertrainState &src)
{
  m_nEncoder  = src.m_nEncoder;
  m_nSpeed    = src.m_nSpeed;
  m_fTemp     = src.m_fTemp;
  m_fVolts    = src.m_fVolts;
  m_fAmps     = src.m_fAmps;
  m_uBufLen   = src.m_uBufLen;
  m_uAlarms   = src.m_uAlarms;
  m_uWarnings = src.m_uWarnings;

  m_fPosition = src.m_fPosition;
  m_fVelocity = src.m_fVelocity;
  m_fTorque   = src.m_fTorque;
  m_fPe       = src.m_fPe;
  m_fPm       = src.m_fPm;
}

LaePowertrainState LaePowertrainState::operator=(const LaePowertrainState &rhs)
{
  m_nEncoder  = rhs.m_nEncoder;
  m_nSpeed    = rhs.m_nSpeed;
  m_fTemp     = rhs.m_fTemp;
  m_fVolts    = rhs.m_fVolts;
  m_fAmps     = rhs.m_fAmps;
  m_uBufLen   = rhs.m_uBufLen;
  m_uWarnings = rhs.m_uWarnings;

  m_fPosition = rhs.m_fPosition;
  m_fVelocity = rhs.m_fVelocity;
  m_fTorque   = rhs.m_fTorque;
  m_fPe       = rhs.m_fPe;
  m_fPm       = rhs.m_fPm;

  return *this;
}


// -----------------------------------------------------------------------------
// Class LaePowertrain
// -----------------------------------------------------------------------------

LaePowertrain::LaePowertrain()
{
}

LaePowertrain::LaePowertrain(const LaePowertrain &src)
{
  m_strName = src.m_strName;
  m_attr    = src.m_attr;
  m_state   = src.m_state;
}

LaePowertrain LaePowertrain::operator=(const LaePowertrain &rhs)
{
  m_strName = rhs.m_strName;
  m_attr    = rhs.m_attr;
  m_state   = rhs.m_state;

  return *this;
}

int LaePowertrain::toMotorId(const int nCtlrId, const int nMotorIndex)
{
  switch( nCtlrId )
  {
    case LaeMotorCtlrIdFront:
      switch( nMotorIndex )
      {
        case LaeMotorLeft:
          return LaeMotorIdLF;
        case LaeMotorRight:
          return LaeMotorIdRF;
        default:
          break;
      }
      break;
    case LaeMotorCtlrIdRear:
      switch( nMotorIndex )
      {
        case LaeMotorLeft:
          return LaeMotorIdLR;
        case LaeMotorRight:
          return LaeMotorIdRR;
        default:
          break;
      }
      break;
    default:
      break;
  }

  return LaeMotorIdNone;
}

string LaePowertrain::toKey(const int nCtlrId, const int nMotorIndex)
{
  static const char *unknown = "";

  int nMotorId = LaePowertrain::toMotorId(nCtlrId, nMotorIndex);

  return nMotorId != LaeMotorIdNone? LaeDesc::KeyPowertrain[nMotorId]: unknown;
}

string LaePowertrain::toKey(const int nMotorId)
{
  static const char *unknown = "";

  if( (nMotorId >= 0) && (nMotorId < LaeMotorsNumOf) )
  {
    LaeDesc::KeyPowertrain[nMotorId];
  }
  else
  {
    return unknown;
  }
}

int LaePowertrain::configure(const LaeDescPowertrain &desc)
{
    // from description
    m_strName               = desc.m_strKey;
    m_attr.m_nMotorId       = desc.m_nMotorId;
    m_attr.m_nMotorCtlrId   = desc.m_nMotorCtlrId;
    m_attr.m_nMotorIndex    = desc.m_nMotorIndex;
    m_attr.m_eJointType     = desc.m_eJointType;
    m_attr.m_nMotorDir      = desc.m_nDir;
    m_attr.m_fGearRatio     = desc.m_fGearRatio;

    // from motor specs and derivations
    m_attr.m_uPulsesPerRev  = LaeQuadPulsesPerRev;
    m_attr.m_fMaxRps        = LaeMotorRatedMaxRpm / 60.0;
    m_attr.m_uMaxQpps       = (uint_t)(
                                      m_attr.m_fMaxRps *
                                      m_attr.m_fGearRatio *
                                      (double)m_attr.m_uPulsesPerRev);

    m_attr.m_fMotorRadsPerPulse = M_TAU / (double)m_attr.m_uPulsesPerRev;
    m_attr.m_fWheelRadsPerPulse = m_attr.m_fMotorRadsPerPulse /
                                          m_attr.m_fGearRatio;

    m_attr.m_fMaxAmps     = LaeMotorRatedAmps;
    m_attr.m_fStallTorque = LaeMotorStallTorque;

    return LAE_OK;
}

int LaePowertrain::configure(const LaeTunes &tunes)
{
  double  fTuneTireRadius, fTuneTireWidth;      // tire dimensions

  tunes.getTireDimParams(m_strName, fTuneTireRadius, fTuneTireWidth);

  m_attr.m_fTireRadius = fTuneTireRadius;
  m_attr.m_fTireWidth  = fTuneTireWidth;

  m_attr.m_fMetersPerPulse = m_attr.m_fWheelRadsPerPulse * m_attr.m_fTireRadius;
  m_attr.m_fMetersPerRadian = m_attr.m_fTireRadius;

  return LAE_OK;
}

int LaePowertrain::reload(const LaeTunes &tunes)
{
  return configure(tunes);
}

int LaePowertrain::resetOdometer()
{
  m_state.m_nEncoder  = 0;
  m_state.m_fPosition = 0.0;

  RtDb.m_kin.m_powertrain[m_attr.m_nMotorId].m_fPosition = m_state.m_fPosition;
}

int LaePowertrain::updateStateDynamics(s64_t   nEncoder,
                                       s32_t   nSpeed,
                                       double  fAmps,
                                       uint_t  uBufLen)
{
  m_state.m_nEncoder  = nEncoder;
  m_state.m_nSpeed    = nSpeed;

  if( fAmps <= motor::roboclaw::ParamAmpMax )
  {
    m_state.m_fAmps = fAmps;
  }

  // command buffer length (commands)
  m_state.m_uBufLen   = uBufLen;

  // angular position and velocity
  m_state.m_fPosition = (double)m_state.m_nEncoder *
                                        m_attr.m_fWheelRadsPerPulse;
  m_state.m_fVelocity = (double)m_state.m_nSpeed  *
                                        m_attr.m_fWheelRadsPerPulse;

  // power
  m_state.m_fPe = m_state.m_fAmps * m_state.m_fVolts;
  m_state.m_fPm = m_state.m_fPe * 1.0;  // TODO need efficiency curve

  // torque
  if( fabs(m_state.m_fVelocity) >= 0.0001 )
  {
    m_state.m_fTorque = m_state.m_fPm / m_state.m_fVelocity;
  }
  else if( m_state.m_fAmps >= (m_attr.m_fMaxAmps * 0.9) )
  {
    m_state.m_fTorque = m_attr.m_fStallTorque *
                              m_state.m_fAmps /
                              m_attr.m_fMaxAmps;
  }
  else
  {
    m_state.m_fTorque = 0.0;
  }

  m_state.m_fTorque = fcap(m_state.m_fTorque, -m_attr.m_fStallTorque,
                                              m_attr.m_fStallTorque);

  // update real-time database
  RtDb.m_kin.m_powertrain[m_attr.m_nMotorId].m_nEncoder  = m_state.m_nEncoder;
  RtDb.m_kin.m_powertrain[m_attr.m_nMotorId].m_nSpeed    = m_state.m_nSpeed;
  RtDb.m_kin.m_powertrain[m_attr.m_nMotorId].m_fPosition = m_state.m_fPosition;
  RtDb.m_kin.m_powertrain[m_attr.m_nMotorId].m_fVelocity = m_state.m_fVelocity;
  RtDb.m_kin.m_powertrain[m_attr.m_nMotorId].m_fPe       = m_state.m_fPe;
  RtDb.m_kin.m_powertrain[m_attr.m_nMotorId].m_fTorque   = m_state.m_fTorque;
}

int LaePowertrain::updateHealth(double fVolts, double fTemp, uint_t uCtlrStatus)
{
  m_state.m_fVolts  = fVolts;
  m_state.m_fTemp   = fTemp;

  m_state.m_uAlarms   = LaeMotorAlarmNone;
  m_state.m_uWarnings = LaeMotorWarnNone;

  //
  // Errors and/or warnings exist.
  //
  if( uCtlrStatus != ParamStatusNormal )
  {
    // emergency stopped
    if( uCtlrStatus & ParamStatusEStopped )
    {
      m_state.m_uAlarms |= LaeMotorAlarmEStop;
    }

    // temperature alarms
    if( uCtlrStatus & ParamStatusErrTemp )
    {
      m_state.m_uAlarms |= LaeMotorAlarmTemp;
    }

    // temperature warnings
    if( uCtlrStatus & ParamStatusWarnTemp )
    {
      m_state.m_uWarnings |= LaeMotorWarnTemp;
    }

    // voltage alarms
    if( uCtlrStatus & ( ParamStatusErrMainBattHigh |
                        ParamStatusErrLogicBattHigh |
                        ParamStatusErrLogicBattLow ) )
    {
      m_state.m_uAlarms |= LaeMotorAlarmVoltage;
    }

    // voltage warnings
    if( uCtlrStatus & ( ParamStatusWarnMainBattHigh |
                        ParamStatusWarnMainBattLow ) )
    {
      m_state.m_uWarnings |= LaeMotorWarnVoltage;
    }

    if( m_attr.m_nMotorIndex == Motor1 )
    {
      // motor 1 drive fault alarm
      if( uCtlrStatus & ParamStatusErrMot1Fault )
      {
        m_state.m_uAlarms |= LaeMotorAlarmFault;
      }

      // motor 1 over current warning
      if( uCtlrStatus & ParamStatusWarnMot1OverCur )
      {
        m_state.m_uWarnings |= LaeMotorWarnCurrent;
      }
    }
    else if(  m_attr.m_nMotorIndex == Motor2 )
    {
      // motor 2 drive fault alarm
      if( uCtlrStatus & ParamStatusErrMot2Fault )
      {
        m_state.m_uAlarms |= LaeMotorAlarmFault;
      }

      // motor 2 over current warning
      if( uCtlrStatus & ParamStatusWarnMot2OverCur )
      {
        m_state.m_uWarnings |= LaeMotorWarnCurrent;
      }
    }
  }

  // TODO torque calculation
  m_state.m_fTorque = 0.0;

  m_state.m_fPe = m_state.m_fAmps * m_state.m_fVolts;
  m_state.m_fPm = m_state.m_fPe * 1.0;  // TODO need efficiency curve
}
