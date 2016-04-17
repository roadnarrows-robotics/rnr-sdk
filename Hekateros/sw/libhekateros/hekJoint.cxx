////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekJoint.cxx
//
/*! \file
 *
 * $LastChangedDate: 2014-11-18 14:31:49 -0700 (Tue, 18 Nov 2014) $
 * $Rev: 3810 $
 *
 * \brief Hekateros joint classes implementations.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2013-2014  RoadNarrows
 * (http://www.RoadNarrows.com)
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
#include <unistd.h>

#include <string>
#include <map>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekTune.h"
#include "Hekateros/hekUtils.h"
#include "Hekateros/hekSpec.h"
#include "Hekateros/hekJoint.h"

using namespace std;
using namespace hekateros;


// -----------------------------------------------------------------------------
// Class HekRobotJoint
// -----------------------------------------------------------------------------

HekRobotJoint::HekRobotJoint()
{
  int   i;

  // (derived) specification
  m_nMasterServoId      = DYNA_ID_NONE;
  m_nSlaveServoId       = DYNA_ID_NONE;
  m_bIsServoContinuous  = false;
  m_nMasterServoDir     = DYNA_DIR_NONE;
  m_eJointType          = HekJointTypeUnknown;
  m_fGearRatio          = 1.0;
  m_fTicksPerServoRad   = 0.0;
  m_fTicksPerJointRad   = 0.0;

  // discovered limits and positions
  m_fMaxServoRadsPerSec = 0.0;
  m_fMaxJointRadsPerSec = 0.0;

  m_fMinPhyLimitRads    = 0.0;
  m_fMaxPhyLimitRads    = 0.0;
  m_nMinPhyLimitOd      = 0;
  m_nMaxPhyLimitOd      = 0;

  m_fMinSoftLimitRads   = 0.0;
  m_fMaxSoftLimitRads   = 0.0;
  m_nMinSoftLimitOd     = 0;
  m_nMaxSoftLimitOd     = 0;

  m_fCalibPosRads       = 0.0;
  m_fBalPosRads         = 0.0;
  m_fParkPosRads        = 0.0;

  m_eLimitTypes         = HekLimitTypeUnknown;
  m_fOverTorqueThDft    = HekTuneOverTorqueThDft;

  for(i=0; i<HekOptLimitMaxPerJoint; ++i)
  {
    m_byOptLimitMask[i] = 0x00;
  }

  // state
  m_eOpState            = HekOpStateUncalibrated;
  m_bStopAtOptLimits    = true;
}

HekRobotJoint::HekRobotJoint(const HekRobotJoint &src)
{
  int   i;

  // (derived) specification
  m_strName             = src.m_strName;
  m_nMasterServoId      = src.m_nMasterServoId;
  m_nSlaveServoId       = src.m_nSlaveServoId;
  m_bIsServoContinuous  = src.m_bIsServoContinuous;
  m_nMasterServoDir     = src.m_nMasterServoDir;
  m_eJointType          = src.m_eJointType;
  m_fGearRatio          = src.m_fGearRatio;
  m_fTicksPerServoRad   = src.m_fTicksPerServoRad;
  m_fTicksPerJointRad   = src.m_fTicksPerJointRad;

  // discovered limits and positions
  m_fMaxServoRadsPerSec = src.m_fMaxServoRadsPerSec;
  m_fMaxJointRadsPerSec = src.m_fMaxJointRadsPerSec;

  m_fMinPhyLimitRads    = src.m_fMinPhyLimitRads;
  m_fMaxPhyLimitRads    = src.m_fMaxPhyLimitRads;
  m_nMinPhyLimitOd      = src.m_nMinPhyLimitOd;
  m_nMaxPhyLimitOd      = src.m_nMaxPhyLimitOd;

  m_fMinSoftLimitRads   = src.m_fMinSoftLimitRads;
  m_fMaxSoftLimitRads   = src.m_fMaxSoftLimitRads;
  m_nMinSoftLimitOd     = src.m_nMinSoftLimitOd;
  m_nMaxSoftLimitOd     = src.m_nMaxSoftLimitOd;

  m_fCalibPosRads       = src.m_fCalibPosRads;
  m_fBalPosRads         = src.m_fBalPosRads;
  m_fParkPosRads        = src.m_fParkPosRads;

  m_eLimitTypes         = src.m_eLimitTypes;
  m_fOverTorqueThDft    = src.m_fOverTorqueThDft;

  for(i=0; i<HekOptLimitMaxPerJoint; ++i)
  {
    m_byOptLimitMask[i] = src.m_byOptLimitMask[i];
  }

  // state
  m_eOpState            = src.m_eOpState;
  m_bStopAtOptLimits    = src.m_bStopAtOptLimits;
}

HekRobotJoint::~HekRobotJoint()
{
}

HekRobotJoint HekRobotJoint::operator=(const HekRobotJoint &rhs)
{
  int     i;

  // (derived) specification
  m_strName             = rhs.m_strName;
  m_nMasterServoId      = rhs.m_nMasterServoId;
  m_nSlaveServoId       = rhs.m_nSlaveServoId;
  m_bIsServoContinuous  = rhs.m_bIsServoContinuous;
  m_nMasterServoDir     = rhs.m_nMasterServoDir;
  m_eJointType          = rhs.m_eJointType;
  m_fGearRatio          = rhs.m_fGearRatio;
  m_fTicksPerServoRad   = rhs.m_fTicksPerServoRad;
  m_fTicksPerJointRad   = rhs.m_fTicksPerJointRad;

  // discovered limits and positions
  m_fMaxServoRadsPerSec = rhs.m_fMaxServoRadsPerSec;
  m_fMaxJointRadsPerSec = rhs.m_fMaxJointRadsPerSec;

  m_fMinPhyLimitRads    = rhs.m_fMinPhyLimitRads;
  m_fMaxPhyLimitRads    = rhs.m_fMaxPhyLimitRads;
  m_nMinPhyLimitOd      = rhs.m_nMinPhyLimitOd;
  m_nMaxPhyLimitOd      = rhs.m_nMaxPhyLimitOd;

  m_fMinSoftLimitRads   = rhs.m_fMinSoftLimitRads;
  m_fMaxSoftLimitRads   = rhs.m_fMaxSoftLimitRads;
  m_nMinSoftLimitOd     = rhs.m_nMinSoftLimitOd;
  m_nMaxSoftLimitOd     = rhs.m_nMaxSoftLimitOd;

  m_fCalibPosRads       = rhs.m_fCalibPosRads;
  m_fBalPosRads         = rhs.m_fBalPosRads;
  m_fParkPosRads        = rhs.m_fParkPosRads;

  m_eLimitTypes         = rhs.m_eLimitTypes;
  m_fOverTorqueThDft    = rhs.m_fOverTorqueThDft;

  for(i=0; i<HekOptLimitMaxPerJoint; ++i)
  {
    m_byOptLimitMask[i] = rhs.m_byOptLimitMask[i];
  }

  // state
  m_eOpState            = rhs.m_eOpState;
  m_bStopAtOptLimits    = rhs.m_bStopAtOptLimits;

  return *this;
}


// -----------------------------------------------------------------------------
// Class HekJointState
// -----------------------------------------------------------------------------

HekJointState::HekJointState()
{
  m_eOpState        = HekOpStateUncalibrated;
  m_nMasterServoId  = DYNA_ID_NONE;
  m_nSlaveServoId   = DYNA_ID_NONE;
  m_fPosition       = 0.0;
  m_fVelocity       = 0.0;
  m_fEffort         = 0.0;
  m_nOdPos          = 0;
  m_nEncPos         = 0;
  m_nSpeed          = 0;

  for(int i=0; i<HekOptLimitMaxPerJoint; ++i)
  {
    m_eOptSwitch[i] = HekTriStateUnknown;
  }
}

HekJointState::HekJointState(const HekJointState &src)
{
  m_strName         = src.m_strName;
  m_eOpState        = src.m_eOpState;
  m_nMasterServoId  = src.m_nMasterServoId;
  m_nSlaveServoId   = src.m_nSlaveServoId;
  m_fPosition       = src.m_fPosition;
  m_fVelocity       = src.m_fVelocity;
  m_fEffort         = src.m_fEffort;
  m_nOdPos          = src.m_nOdPos;
  m_nEncPos         = src.m_nEncPos;
  m_nSpeed          = src.m_nSpeed;

  for(int i=0; i<HekOptLimitMaxPerJoint; ++i)
  {
    m_eOptSwitch[i] = src.m_eOptSwitch[i];
  }
}

HekJointState HekJointState::operator=(const HekJointState &rhs)
{
  m_strName         = rhs.m_strName;
  m_eOpState        = rhs.m_eOpState;
  m_nMasterServoId  = rhs.m_nMasterServoId;
  m_nSlaveServoId   = rhs.m_nSlaveServoId;
  m_fPosition       = rhs.m_fPosition;
  m_fVelocity       = rhs.m_fVelocity;
  m_fEffort         = rhs.m_fEffort;
  m_nOdPos          = rhs.m_nOdPos;
  m_nEncPos         = rhs.m_nEncPos;
  m_nSpeed          = rhs.m_nSpeed;

  for(int i=0; i<HekOptLimitMaxPerJoint; ++i)
  {
    m_eOptSwitch[i] = rhs.m_eOptSwitch[i];
  }

  return *this;
}


// -----------------------------------------------------------------------------
// Class HekJointStatePoint
// -----------------------------------------------------------------------------

static HekJointState  nojointstate;

bool HekJointStatePoint::hasJoint(const string &strJointName)
{
  vector<HekJointState>::iterator iter;

  for(iter = m_jointState.begin(); iter != m_jointState.end(); ++iter)
  {
    if( iter->m_strName == strJointName )
    {
      return true;
    }
  }
  return false;
}

HekJointState &HekJointStatePoint::operator[](const string &strJointName)
{
  vector<HekJointState>::iterator iter;

  for(iter = m_jointState.begin(); iter != m_jointState.end(); ++iter)
  {
    if( iter->m_strName == strJointName )
    {
      return *iter;
    }
  }
  return nojointstate;
}
