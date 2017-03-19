////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekTune.cxx
//
/*! \file
 *
 * $LastChangedDate: 2014-10-11 20:13:57 +0000 (Sat, 11 Oct 2014) $
 * $Rev: 3782 $
 *
 * \brief Hekateros tuning implementation.
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

#include <string>
#include <map>

#include "rnr/rnrconfig.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekTune.h"
#include "Hekateros/hekUtils.h"

using namespace std;
using namespace hekateros;

//------------------------------------------------------------------------------
// HekTunesJoint Class
//------------------------------------------------------------------------------
  
HekTunesJoint::HekTunesJoint()
{
  m_fTolPos       = degToRad(HekTuneTolPosDft);
  m_fTolVel       = degToRad(HekTuneTolVelDft);
  m_fPidKp        = HekTunePidKpDft;
  m_fPidKi        = HekTunePidKiDft;
  m_fPidKd        = HekTunePidKdDft;
  m_fPidMaxDeltaV = degToRad(HekTunePidMaxDeltaVDft);
  m_fOverTorqueTh = HekTuneOverTorqueThDft / 100.0;
}

HekTunesJoint &HekTunesJoint::operator=(const HekTunesJoint &rhs)
{
  m_fTolPos       = rhs.m_fTolPos;
  m_fTolVel       = rhs.m_fTolVel;
  m_fPidKp        = rhs.m_fPidKp;
  m_fPidKi        = rhs.m_fPidKi;
  m_fPidKd        = rhs.m_fPidKd;
  m_fPidMaxDeltaV = rhs.m_fPidMaxDeltaV;
  m_fOverTorqueTh = rhs.m_fOverTorqueTh;
}


//------------------------------------------------------------------------------
// HekTunes Class
//------------------------------------------------------------------------------
  
HekTunes::HekTunes()
{
  m_fKinematicsHz       = HekTuneKinHzDft;
  m_fClearTorqueOffset  = HekTuneClearTorqueOffsetDft / 100.0;
  m_fVelDerate          = HekTuneVelDerateDft / 100.0;
  m_eTrajNorm           = HekTuneTrajNormDft;
  m_fTrajEpsilon        = degToRad(HekTuneTrajEpsilonDft);
}

void HekTunes::getToleranceParams(const std::string &strJointName,
                                  double &fTolPos, double &fTolVel) const
{
  MapJointTunes::const_iterator pos;

  if( (pos = m_mapJointTunes.find(strJointName)) != m_mapJointTunes.end() )
  {
    fTolPos = pos->second.m_fTolPos;
    fTolVel = pos->second.m_fTolVel;
  }
  else
  {
    fTolPos = degToRad(HekTuneTolPosDft);
    fTolVel = degToRad(HekTuneTolVelDft);
  }
}

void HekTunes::getPidKParams(const std::string &strJointName,
                             double &fKp, double &fKi, double &fKd) const
{
  MapJointTunes::const_iterator pos;

  if( (pos = m_mapJointTunes.find(strJointName)) != m_mapJointTunes.end() )
  {
    fKp = pos->second.m_fPidKp;
    fKi = pos->second.m_fPidKi;
    fKd = pos->second.m_fPidKd;
  }
  else
  {
    fKp = HekTunePidKpDft;
    fKi = HekTunePidKiDft;
    fKd = HekTunePidKdDft;
  }
}

double HekTunes::getPidMaxDeltaV(const std::string &strJointName) const
{
  MapJointTunes::const_iterator pos;

  if( (pos = m_mapJointTunes.find(strJointName)) != m_mapJointTunes.end() )
  {
    return pos->second.m_fPidMaxDeltaV;
  }
  else
  {
    return degToRad(HekTunePidMaxDeltaVDft);
  }
}

void HekTunes::getTorqueParams(const std::string &strJointName,
                               double &fOverTorqueTh,
                               double &fClearTorqueTh) const
{
  MapJointTunes::const_iterator pos;

  if( (pos = m_mapJointTunes.find(strJointName)) != m_mapJointTunes.end() )
  {
    fOverTorqueTh = pos->second.m_fOverTorqueTh;
  }
  else
  {
    fOverTorqueTh = HekTuneOverTorqueThDft / 100.0;
  }
  fClearTorqueTh = fOverTorqueTh * m_fClearTorqueOffset;
}
