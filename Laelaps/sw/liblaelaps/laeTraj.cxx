////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Library:   liblaelaps
//
// File:      laeTraj.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-06-01 13:57:21 -0600 (Mon, 01 Jun 2015) $
 * $Rev: 4009 $
 *
 * \brief Trajectory classes interfaces.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2015.  RoadNarrows
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

#include <sys/types.h>
#include <time.h>

#include <string>
#include <sstream>
#include <vector>
#include <map>

#include "rnr/rnrconfig.h"

#include "Laelaps/laelaps.h"
#include "Laelaps/laeUtils.h"
#include "Laelaps/laeTraj.h"


using namespace std;
using namespace laelaps;

// -----------------------------------------------------------------------------
// Class LaeSimplePathFeedback
// -----------------------------------------------------------------------------
  
LaeSimplePathFeedback::LaeSimplePathFeedback(const LaeSimplePathFeedback &src)
{
  for(int i = 0; i < TrajNumOf; ++i)
  {
    m_path[i] = src.m_path[i];
  }
}

LaeSimplePathFeedback LaeSimplePathFeedback::operator=(
                                              const LaeSimplePathFeedback &rhs)
{
  for(int i = 0; i < TrajNumOf; ++i)
  {
    m_path[i] = rhs.m_path[i];
  }
  return *this;
}

LaeSimplePath &LaeSimplePathFeedback::operator[](const size_t i)
{
  return i < TrajNumOf? m_path[i]: m_path[TrajDesired];
}

void LaeSimplePathFeedback::clear()
{
  for(int i = 0; i < TrajNumOf; ++i)
  {
    m_path[i].clear();
  }
}


// -----------------------------------------------------------------------------
// Class LaeWaypoint
// -----------------------------------------------------------------------------
  
LaeWaypoint::LaeWaypoint()
{
  m_timeStart       = StartImmediately;
  m_fVelocity       = 0.0;
  m_fAcceleration   = 0.0;
}

LaeWaypoint::LaeWaypoint(const LaePose    &pose,
                         const double      fVelocity,
                         const double      fAcceleration,
                         const string     &strName,
                         const timespec_t &timeStart)
    : m_pose(pose), m_strName(strName)
{
  m_timeStart     = timeStart;
  m_fVelocity     = fVelocity;
  m_fAcceleration = fAcceleration;
}

LaeWaypoint::LaeWaypoint(const LaeWaypoint &src)
    : m_pose(src.m_pose), m_strName(src.m_strName)
{
  m_timeStart     = src.m_timeStart;
  m_fVelocity     = src.m_fVelocity;
  m_fAcceleration = src.m_fAcceleration;
}

LaeWaypoint LaeWaypoint::operator=(const LaeWaypoint &rhs)
{
  m_strName       = rhs.m_strName;
  m_timeStart     = rhs.m_timeStart;
  m_pose          = rhs.m_pose;
  m_fVelocity     = rhs.m_fVelocity;
  m_fAcceleration = rhs.m_fAcceleration;

  return *this;
}

void LaeWaypoint::get(string      &strName,
                      timespec_t  &timeStart,
                      LaePose     &pose,
                      double      &fVelocity,
                      double      &fAcceleration)
{
  strName       = m_strName;
  timeStart     = m_timeStart;
  pose          = m_pose;
  fVelocity     = m_fVelocity;
  fAcceleration = m_fAcceleration;
}

void LaeWaypoint::clear()
{
  m_strName.clear();
  m_timeStart     = StartImmediately;
  m_pose.clear();
  m_fVelocity     = 0.0;
  m_fAcceleration = 0.0;
}


// -----------------------------------------------------------------------------
// Class LaePath
// -----------------------------------------------------------------------------

void LaePath::append(const LaePose     &pose,
                     const double      fVelocity,
                     const double      fAcceleration,
                     const std::string &strName,
                     const timespec_t  &timeStart)
{
  stringstream ss;

  if( strName.empty() )
  {
    ss << "wp_" << m_path.size();
  }
  else
  {
    ss << strName;
  }

  LaeWaypoint pt(pose, fVelocity, fAcceleration, ss.str(), timeStart);
  m_path.push_back(pt);
}


// -----------------------------------------------------------------------------
// Class LaePathFeedback
// -----------------------------------------------------------------------------
  
LaePathFeedback::LaePathFeedback(const LaePathFeedback &src)
{
  for(int i = 0; i < TrajNumOf; ++i)
  {
    m_path[i] = src.m_path[i];
  }
}

LaePathFeedback LaePathFeedback::operator=(const LaePathFeedback &rhs)
{
  for(int i = 0; i < TrajNumOf; ++i)
  {
    m_path[i] = rhs.m_path[i];
  }
  return *this;
}

LaePath &LaePathFeedback::operator[](const size_t i)
{
  return i < TrajNumOf? m_path[i]: m_path[TrajDesired];
}

void LaePathFeedback::clear()
{
  for(int i = 0; i < TrajNumOf; ++i)
  {
    m_path[i].clear();
  }
}
