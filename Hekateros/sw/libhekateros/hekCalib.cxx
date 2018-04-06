////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekCalib.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-01-15 12:45:22 -0700 (Thu, 15 Jan 2015) $
 * $Rev: 3857 $
 *
 * \brief HekCalib - Hekateros calibration abstract base class implementation.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2013-2018. RoadNarrows LLC.\n
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

#include <unistd.h>

#include "rnr/rnrconfig.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaServo.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekCalib.h"
#include "Hekateros/hekRobot.h"
#include "Hekateros/hekUtils.h"


using namespace std;
using namespace hekateros;

double HekCalib::moveWait(const string &strJointName,
                       const double  fJointGoalPos,
                       const double  fJointGoalVel)
{
  static const int  TuneMaxSecs  = 30;     // max of 30 seconds to complete move
  static const int  TuneWaituSec = 300000; // usec sleep between tests

  int nMaxIters = TuneMaxSecs * MILLION / TuneWaituSec; // max iterations

  double  fJointCurPos, fJointCurVel;

  LOGDIAG3("%s(): Move joint %s to goalpos=%.2lf at goalvel=%.3lf", LOGFUNCNAME,
      strJointName.c_str(), radToDeg(fJointGoalPos), radToDeg(fJointGoalVel));
  
  // start move
  m_robot.m_pKin->move(strJointName, fJointGoalPos, fJointGoalVel);

  //
  // Wait until move is completed, interrupted, or the maximum seconds reached.
  //
  for(int i=0; i<nMaxIters; ++i)
  {
    if( m_robot.m_pKin->isStopped(strJointName) )
    {
      LOGDIAG3("%s():  %s is stopped.", LOGFUNCNAME, strJointName.c_str());
      break;
    }

    m_robot.m_pKin->getJointCurPosVel(strJointName, fJointCurPos, fJointCurVel);

    LOGDIAG3("%s():  %s at curpos=%.2lf, curvel=%.3lf", LOGFUNCNAME,
          strJointName.c_str(), radToDeg(fJointCurPos), radToDeg(fJointCurVel));

    // sleep (and a pthread cancelation point)
    usleep(TuneWaituSec);
  }

  m_robot.m_pKin->getJointCurPosVel(strJointName, fJointCurPos, fJointCurVel);

  LOGDIAG3("%s(): Joint %s move ended at to curpos=%.2lf", LOGFUNCNAME,
          strJointName.c_str(), radToDeg(fJointCurPos));

  return fJointCurPos;
}

double HekCalib::moveToTorqueLimit(const string &strJointName,
                                const double  fJointGoalPos,
                                const double  fJointGoalVel)
{
  static const int  TuneMaxSecs  = 30;     // max of 30 seconds to complete move
  static const int  TuneWaituSec = 300000; // usec sleep between tests

  int nMaxIters = TuneMaxSecs * MILLION / TuneWaituSec; // max iterations

  double  fJointCurPos, fJointCurVel;

  LOGDIAG3("%s(): Move joint %s to goalpos=%.2lf at goalvel=%.3lf", LOGFUNCNAME,
      strJointName.c_str(), radToDeg(fJointGoalPos), radToDeg(fJointGoalVel));
  
  // start move
  m_robot.m_pKin->move(strJointName, fJointGoalPos, fJointGoalVel);

  //
  // Wait until move is completed, interrupted, or the maximum seconds reached.
  //
  for(int i=0; i<nMaxIters; ++i)
  {
    // physically blocked
    if( m_robot.m_pKin->hasOverTorqueCondition(strJointName) )
    {
      LOGDIAG3("%s():  %s in over torque condition.", LOGFUNCNAME,
          strJointName.c_str());
      break;
    }

    // joint kinematics stopped
    else if( m_robot.m_pKin->isStopped(strJointName) )
    {
      LOGDIAG3("%s():  %s is stopped.", LOGFUNCNAME, strJointName.c_str());
      break;
    }

    m_robot.m_pKin->getJointCurPosVel(strJointName, fJointCurPos, fJointCurVel);

    LOGDIAG3("%s():  %s at curpos=%.2lf, curvel=%.3lf", LOGFUNCNAME,
          strJointName.c_str(), radToDeg(fJointCurPos), radToDeg(fJointCurVel));

    // sleep (and a pthread cancelation point)
    usleep(TuneWaituSec);
  }

  m_robot.m_pKin->getJointCurPosVel(strJointName, fJointCurPos, fJointCurVel);

  LOGDIAG3("%s(): Joint %s move ended at to curpos=%.2lf", LOGFUNCNAME,
          strJointName.c_str(), radToDeg(fJointCurPos));

  return fJointCurPos;
}

double HekCalib::moveToLight(const string &strJointName,
                          const double  fJointGoalPos,
                          const double  fJointGoalVel,
                          byte_t        byMask)
{
  static const int  TuneMaxSecs  = 30;     // max of 30 seconds to complete move
  static const int  TuneWaituSec = 200000; // usec sleep between tests

  int nMaxIters = TuneMaxSecs * MILLION / TuneWaituSec; // max iterations

  double  fJointCurPos, fJointCurVel;
  byte_t  byLimits;

  LOGDIAG3("%s(): Move %s to goalpos=%.2lf at goalvel=%.3lf", LOGFUNCNAME,
      strJointName.c_str(), radToDeg(fJointGoalPos), radToDeg(fJointGoalVel));
  
  // start move
  m_robot.m_pKin->move(strJointName, fJointGoalPos, fJointGoalVel);

  // wait until move is completed or maximum seconds reached.
  for(int i=0; i<nMaxIters; ++i)
  {
    // physically blocked
    if( m_robot.m_pKin->hasOverTorqueCondition(strJointName) )
    {
      LOGDIAG3("%s():  %s in over torque condition.", LOGFUNCNAME,
            strJointName.c_str());
      break;
    }

    // joint kinematics stopped
    else if( m_robot.m_pKin->isStopped(strJointName) )
    {
      LOGDIAG3("%s():  %s is stopped.", LOGFUNCNAME, strJointName.c_str());
      break;
    }

    // optical limits
    byLimits = m_robot.m_monitor.getJointLimitBits();

    // light
    if( getLitOpticalLimits(byLimits, byMask) != 0 )
    {
      LOGDIAG3("%s():  %s reached the light.", LOGFUNCNAME,
            strJointName.c_str());
      break;
    }

    m_robot.m_pKin->getJointCurPosVel(strJointName, fJointCurPos, fJointCurVel);

    LOGDIAG3("%s():  %s at curpos=%.2lf, curvel=%.3lf", LOGFUNCNAME,
          strJointName.c_str(), radToDeg(fJointCurPos), radToDeg(fJointCurVel));

    // sleep (and a pthread cancelation point)
    usleep(TuneWaituSec);
  }

  m_robot.m_pKin->stop(strJointName);

  usleep(TuneWaituSec);

  byLimits = m_robot.m_monitor.getJointLimitBits();

  if( getLitOpticalLimits(byLimits, byMask) == 0 )
  {
    LOGWARN("%s(): %s stopped before finding the light.", LOGFUNCNAME,
      strJointName.c_str());
  }

  m_robot.m_pKin->getJointCurPosVel(strJointName, fJointCurPos, fJointCurVel);

  LOGDIAG3("%s(): Joint %s move ended at to curpos=%.2lf", LOGFUNCNAME,
          strJointName.c_str(), radToDeg(fJointCurPos));

  return fJointCurPos;
}

double HekCalib::moveToDark(const string &strJointName,
                            const double  fJointGoalPos,
                            const double  fJointGoalVel,
                            byte_t        byMask)
{
  static const int  TuneMaxSecs  = 30;     // max of 30 seconds to complete move
  static const int  TuneWaituSec = 200000; // usec sleep between tests

  int nMaxIters = TuneMaxSecs * MILLION / TuneWaituSec; // max iterations

  double  fJointCurPos, fJointCurVel;
  byte_t  byLimits;

  LOGDIAG3("%s(): Move %s to goalpos=%.2lf at goalvel=%.3lf", LOGFUNCNAME,
      strJointName.c_str(), radToDeg(fJointGoalPos), radToDeg(fJointGoalVel));
  
  // start move
  m_robot.m_pKin->move(strJointName, fJointGoalPos, fJointGoalVel);

  // wait until move is completed or maximum seconds reached.
  for(int i=0; i<nMaxIters; ++i)
  {
    // physically blocked
    if( m_robot.m_pKin->hasOverTorqueCondition(strJointName) )
    {
      LOGDIAG3("%s():  %s in over torque condition.", LOGFUNCNAME,
            strJointName.c_str());
      break;
    }

    // joint kinematics stopped
    else if( m_robot.m_pKin->isStopped(strJointName) )
    {
      LOGDIAG3("%s():  %s is stopped.", LOGFUNCNAME, strJointName.c_str());
      break;
    }

    // optical limits
    byLimits = m_robot.m_monitor.getJointLimitBits();

    // dark
    if( getDarkOpticalLimits(byLimits, byMask) != 0 )
    {
      LOGDIAG3("%s():  %s reached the dark.", LOGFUNCNAME,
            strJointName.c_str());
      break;
    }

    m_robot.m_pKin->getJointCurPosVel(strJointName, fJointCurPos, fJointCurVel);

    LOGDIAG3("%s():  %s at curpos=%.2lf, curvel=%.3lf", LOGFUNCNAME,
          strJointName.c_str(), radToDeg(fJointCurPos), radToDeg(fJointCurVel));

    // sleep (and a pthread cancelation point)
    usleep(TuneWaituSec);
  }

  m_robot.m_pKin->stop(strJointName);

  usleep(TuneWaituSec);

  byLimits = m_robot.m_monitor.getJointLimitBits();

  if( getDarkOpticalLimits(byLimits, byMask) == 0 )
  {
    LOGWARN("%s(): %s stopped before finding the dark.", LOGFUNCNAME,
      strJointName.c_str());
  }

  m_robot.m_pKin->getJointCurPosVel(strJointName, fJointCurPos, fJointCurVel);

  LOGDIAG3("%s(): Joint %s move ended at to curpos=%.2lf", LOGFUNCNAME,
          strJointName.c_str(), radToDeg(fJointCurPos));

  return fJointCurPos;
}
