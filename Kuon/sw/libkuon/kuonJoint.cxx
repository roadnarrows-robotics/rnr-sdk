////////////////////////////////////////////////////////////////////////////////
//
// Package:   Kuon
//
// Library:   libkuon
//
// File:      kuonJoint.cxx
//
/*! \file
 *
 * $LastChangedDate: 2014-04-07 08:16:52 -0600 (Mon, 07 Apr 2014) $
 * $Rev: 3631 $
 *
 * \brief Kuon Robot Joint class implementations.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2014-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
/*
 * @EulaBegin@
 * 
 * Permission is hereby granted, without written agreement and without
 * license or royalty fees, to use, copy, modify, and distribute this
 * software and its documentation for any purpose, provided that
 * (1) The above copyright notice and the following two paragraphs
 * appear in all copies of the source code and (2) redistributions
 * including binaries reproduces these notices in the supporting
 * documentation.   Substantial modifications to this software may be
 * copyrighted by their authors and need not follow the licensing terms
 * described here, provided that the new terms are clearly indicated in
 * all files where they apply.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
 * OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
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

#include "Kuon/kuon.h"
#include "Kuon/kuonUtils.h"
#include "Kuon/kuonSpec.h"
#include "Kuon/kuonJoint.h"

using namespace std;
using namespace kuon;


// -----------------------------------------------------------------------------
// Class KuonRobotJoint
// -----------------------------------------------------------------------------

KuonRobotJoint::KuonRobotJoint()
{
  // (derived) specification
  m_nMotorId            = KuonMotorIdNone;
  m_nMotorCtlrId        = KuonMotorCtlrIdNone;
  m_nMotorIndex         = 0;
  m_nMotorDir           = KuonMotorDirUnknown;
  m_eJointType          = KuonJointTypeUnknown;
  m_fGearRatio          = 1.0;
  m_fTireRadius         = 1.0;
  m_fTicksPerMotorRad   = 0.0;
  m_fTicksPerWheelRad   = 0.0;

  // discovered limits and positions
}

KuonRobotJoint::KuonRobotJoint(const KuonRobotJoint &src)
{
  // (derived) specification
  m_strName             = src.m_strName;
  m_nMotorId            = src.m_nMotorId;
  m_nMotorCtlrId        = src.m_nMotorCtlrId;
  m_nMotorIndex         = src.m_nMotorIndex;
  m_nMotorDir           = src.m_nMotorDir;
  m_eJointType          = src.m_eJointType;
  m_fGearRatio          = src.m_fGearRatio;
  m_fTicksPerMotorRad   = src.m_fTicksPerMotorRad;
  m_fTicksPerWheelRad   = src.m_fTicksPerWheelRad;

  // discovered limits and positions
}

KuonRobotJoint::~KuonRobotJoint()
{
}

KuonRobotJoint KuonRobotJoint::operator=(const KuonRobotJoint &rhs)
{
  // (derived) specification
  m_strName             = rhs.m_strName;
  m_nMotorId            = rhs.m_nMotorId;
  m_nMotorCtlrId        = rhs.m_nMotorCtlrId;
  m_nMotorIndex         = rhs.m_nMotorIndex;
  m_nMotorDir           = rhs.m_nMotorDir;
  m_eJointType          = rhs.m_eJointType;
  m_fGearRatio          = rhs.m_fGearRatio;
  m_fTicksPerMotorRad   = rhs.m_fTicksPerMotorRad;
  m_fTicksPerWheelRad   = rhs.m_fTicksPerWheelRad;

  // discovered limits and positions

  return *this;
}


// -----------------------------------------------------------------------------
// Class KuonJointState
// -----------------------------------------------------------------------------

KuonJointState::KuonJointState()
{
  m_nMotorId        = KuonMotorIdNone;
  m_fPosition       = 0.0;
  m_fVelocity       = 0.0;
  m_fEffort         = 0.0;
  m_fOdometer       = 0.0;
  m_nEncoder        = 0;
  m_fVelocityMps    = 0.0;
  m_nSpeed          = 0;
  m_fPe             = 0.0;
  m_fPm             = 0.0;
  m_fBrake          = 0.0;
  m_fSlew           = 0.0;
}

KuonJointState::KuonJointState(const KuonJointState &src)
{
  m_strName         = src.m_strName;
  m_nMotorId        = src.m_nMotorId;
  m_fPosition       = src.m_fPosition;
  m_fVelocity       = src.m_fVelocity;
  m_fEffort         = src.m_fEffort;
  m_fOdometer       = src.m_fOdometer;
  m_nEncoder        = src.m_nEncoder;
  m_fVelocityMps    = src.m_fVelocityMps;
  m_nSpeed          = src.m_nSpeed;
  m_fPe             = src.m_fPe;
  m_fPm             = src.m_fPm;
  m_fBrake          = src.m_fBrake;
  m_fSlew           = src.m_fSlew;
}

KuonJointState KuonJointState::operator=(const KuonJointState &rhs)
{
  m_strName         = rhs.m_strName;
  m_nMotorId        = rhs.m_nMotorId;
  m_fPosition       = rhs.m_fPosition;
  m_fVelocity       = rhs.m_fVelocity;
  m_fEffort         = rhs.m_fEffort;
  m_fOdometer       = rhs.m_fOdometer;
  m_nEncoder        = rhs.m_nEncoder;
  m_fVelocityMps    = rhs.m_fVelocityMps;
  m_nSpeed          = rhs.m_nSpeed;
  m_fPe             = rhs.m_fPe;
  m_fPm             = rhs.m_fPm;
  m_fBrake          = rhs.m_fBrake;
  m_fSlew           = rhs.m_fSlew;

  return *this;
}


// -----------------------------------------------------------------------------
// Class KuonJointStatePoint
// -----------------------------------------------------------------------------

static KuonJointState  nojointstate;

bool KuonJointStatePoint::hasJoint(const string &strJointName)
{
  vector<KuonJointState>::iterator iter;

  for(iter = m_jointState.begin(); iter != m_jointState.end(); ++iter)
  {
    if( iter->m_strName == strJointName )
    {
      return true;
    }
  }
  return false;
}

KuonJointState &KuonJointStatePoint::operator[](const string &strJointName)
{
  vector<KuonJointState>::iterator iter;

  for(iter = m_jointState.begin(); iter != m_jointState.end(); ++iter)
  {
    if( iter->m_strName == strJointName )
    {
      return *iter;
    }
  }
  return nojointstate;
}
