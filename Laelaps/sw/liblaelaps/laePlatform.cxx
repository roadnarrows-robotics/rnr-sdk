////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Library:   liblaelaps
//
// File:      laePlatform.cxx
//
/*! \file
 *
 * \brief Laelaps robotic platform control and dynamics state implementation.
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

#include <math.h>

#include <string>
#include <vector>

#include "rnr/rnrconfig.h"

#include "Laelaps/RoboClaw.h"

#include "Laelaps/laelaps.h"
#include "Laelaps/laeDesc.h"
#include "Laelaps/laeTune.h"
#include "Laelaps/laeMotor.h"
#include "Laelaps/laeUtils.h"
#include "Laelaps/laeDb.h"
#include "Laelaps/laeTraj.h"
#include "Laelaps/laePowertrain.h"
#include "Laelaps/laePlatform.h"


using namespace std;
using namespace laelaps;
using namespace motor::roboclaw;


// -----------------------------------------------------------------------------
// Class LaeMotorCtlrState
// -----------------------------------------------------------------------------

void LaeMotorCtlrState::clear()
{
  m_fMainVolts  = 0.0;
  m_fBoardTemp  = 0.0;
  m_uStatus     = ParamStatusNormal;
}

LaeMotorCtlrState::LaeMotorCtlrState(const LaeMotorCtlrState &src)
{
  m_strName     = src.m_strName;
  m_fMainVolts  = src.m_fMainVolts;
  m_fBoardTemp  = src.m_fBoardTemp;
  m_uStatus     = src.m_uStatus;
}

LaeMotorCtlrState LaeMotorCtlrState::operator=(const LaeMotorCtlrState &rhs)
{
  m_strName     = rhs.m_strName;
  m_fMainVolts  = rhs.m_fMainVolts;
  m_fBoardTemp  = rhs.m_fBoardTemp;
  m_uStatus     = rhs.m_uStatus;

  return *this;
}


// -----------------------------------------------------------------------------
// Class LaePlatform
// -----------------------------------------------------------------------------

LaePlatform::LaePlatform() : m_strName("plaform")
{
  clear();

  m_ctlr[LaeMotorCtlrIdFront].m_strName = LaeKeyFront;
  m_ctlr[LaeMotorCtlrIdRear].m_strName  = LaeKeyRear;
}

LaePlatform::LaePlatform(const LaePlatform &src)
{
  int   nCtlr;
  int   i;

  m_strName       = src.m_strName;
  
  m_dimRobot      = src.m_dimRobot;
  m_dimBody       = src.m_dimBody;

  m_fWheelbase    = src.m_fWheelbase;
  m_fWheeltrack   = src.m_fWheeltrack;

  for(i = 0; i < LaeMotorsNumOf; ++i)
  {
    m_fPosLast[i] = src.m_fPosLast[i];
  }

  for(i = 0; i < HistSize; ++i)
  {
    m_pose[i] = src.m_pose[i];
  }

  m_fOdometer     = src.m_fOdometer;
  m_fVelocity     = src.m_fVelocity;

  m_fAmpsMotors   = src.m_fAmpsMotors;
  m_fVoltsAvg     = src.m_fVoltsAvg;
  m_fTempAvg      = src.m_fTempAvg;

  for(nCtlr = 0; nCtlr < LaeNumMotorCtlrs; ++nCtlr)
  {
    m_ctlr[nCtlr] = src.m_ctlr[nCtlr];
  }
}

LaePlatform LaePlatform::operator=(const LaePlatform &rhs)
{
  int   nCtlr;
  int   i;

  m_strName       = rhs.m_strName;
  
  m_dimRobot      = rhs.m_dimRobot;
  m_dimBody       = rhs.m_dimBody;

  m_fWheelbase    = rhs.m_fWheelbase;
  m_fWheeltrack   = rhs.m_fWheeltrack;

  for(i = 0; i < LaeMotorsNumOf; ++i)
  {
    m_fPosLast[i] = rhs.m_fPosLast[i];
  }

  for(i = 0; i < HistSize; ++i)
  {
    m_pose[i] = rhs.m_pose[i];
  }

  m_fOdometer     = rhs.m_fOdometer;
  m_fVelocity     = rhs.m_fVelocity;

  m_fAmpsMotors   = rhs.m_fAmpsMotors;
  m_fVoltsAvg     = rhs.m_fVoltsAvg;
  m_fTempAvg      = rhs.m_fTempAvg;

  for(nCtlr = 0; nCtlr < LaeNumMotorCtlrs; ++nCtlr)
  {
    m_ctlr[nCtlr] = rhs.m_ctlr[nCtlr];
  }

  return *this;
}

void LaePlatform::clear()
{
  int   i;

  m_dimRobot.clear();
  m_dimBody.clear();

  m_fWheelbase    = 0.0;
  m_fWheeltrack   = 0.0;

  for(i = 0; i < LaeMotorsNumOf; ++i)
  {
    m_fPosLast[i] = 0.0;
  }

  for(i = 0; i < HistSize; ++i)
  {
    m_pose[i].clear();
  }

  m_fOdometer     = 0.0;
  m_fVelocity     = 0.0;
  
  m_fAmpsMotors   = 0.0;
  m_fVoltsAvg     = 0.0;
  m_fTempAvg      = 0.0;

  m_ctlr[LaeMotorCtlrIdFront].clear();
  m_ctlr[LaeMotorCtlrIdRear].clear();
}

int LaePlatform::configure(const LaeDescBase &desc)
{
  // base kinematics
  m_dimBody     = desc.m_dimBody;
  m_dimRobot    = desc.m_dimRobot;
  m_fWheelbase  = desc.m_fWheelbase;
  m_fWheeltrack = desc.m_fWheeltrack;
}

int LaePlatform::configure(const LaeTunes &tunes)
{
  LaeDescBase desc;
  double      fTuneTireRadius, fTuneTireWidth;      // tire dimensions

  // assumes all tires are the same dimensions
  tunes.getTireDimParams(LaeKeyLeftFront, fTuneTireRadius, fTuneTireWidth);

  desc.calcDimensions(fTuneTireRadius, fTuneTireWidth);

  m_dimBody     = desc.m_dimBody;
  m_dimRobot    = desc.m_dimRobot;
  m_fWheelbase  = desc.m_fWheelbase;
  m_fWheeltrack = desc.m_fWheeltrack;

  return LAE_OK;
}

int LaePlatform::reload(const LaeTunes &tunes)
{
  return configure(tunes);
}

int LaePlatform::resetOdometer()
{
  int   i;

  clearPoses();

  for(i = 0; i < LaeMotorsNumOf; ++i)
  {
    m_fPosLast[i] = 0.0;
  }

  m_fOdometer = 0.0;
}

int LaePlatform::updateStateDynamics(const LaeMapPowertrain &mapPowertrains)
{
  static double ReallyZero = 1e-4;

  LaeMapPowertrain::const_iterator iter;

  int     nMotorId;                   // motor id
  double  fDeltaPos;                  // delta position last position (radians)
  double  fDeltaDist[LaeMotorsNumOf]; // delta wheel distances (meters)
  double  fDeltaOd;                   // delta odometer (meters)
  double  fDistL, fDistR;             // left and right average delta distance
  double  s;                          // delta arc length
  double  r;                          // radius of curvature
  double  c;                          // central angle
  double  dx, dy, da;                 // delta pose (m, m, radians)
  LaePose pose;                       // new pose

  // zero current velocity, delta odometer
  m_fVelocity = 0.0;
  fDeltaOd    = 0.0;

  //
  // Process powertrain kinodynamics to arrive at platform dynamics.
  //
  for(iter = mapPowertrains.begin(); iter != mapPowertrains.end(); ++iter)
  {
    const LaePowertrain &train = iter->second;

    nMotorId = train.m_attr.m_nMotorId;

    // delta position (radians) and delta distance (meters)
    fDeltaPos  = train.m_state.m_fPosition - m_fPosLast[nMotorId];
    m_fPosLast[nMotorId] = train.m_state.m_fPosition;
    fDeltaDist[nMotorId] = fDeltaPos * train.m_attr.m_fMetersPerRadian;

    // sum delta odometer distances
    fDeltaOd += fDeltaDist[nMotorId];

    // sum wheel velocity (radians/s) to linear velocity (meters/s)
    m_fVelocity += train.m_state.m_fVelocity * train.m_attr.m_fMetersPerRadian;
  }

  //
  // Average the odometer delta sum value and accumulate to attain meters 
  // traveled at the platform center of rotation. Note that this odometer is
  // the absolute distance traveled.
  //
  fDeltaOd = fabs(fDeltaOd / (double)LaeMotorsNumOf);
  m_fOdometer += fDeltaOd;

  //
  // Average the velocity sum. The velocity is calculated from the platform
  // center of rotation. Note that this velocity is the absolute value.
  //
  m_fVelocity = fabs(m_fVelocity / (double)LaeMotorsNumOf);

  //
  // Average the left and right side delta distances traveled.
  //
  fDistL = (fDeltaDist[LaeMotorIdLF] + fDeltaDist[LaeMotorIdLR]) / 2.0;
  fDistR = (fDeltaDist[LaeMotorIdRF] + fDeltaDist[LaeMotorIdRR]) / 2.0;

  //
  // Calculate the new absolute orientation.
  //
  // The new theta can be calculated directly from current odometry.
  //
  // Eq. s = r * c, where
  //
  // s = arc length (meters)
  // r = radius of curvature (meters) == wheel base
  // c = central angle (radians)
  //
  s = (fDistR + fDistL) / 2.0;
  c = (fDistR - fDistL) / m_fWheelbase;

  //
  // No linear component (i.e. spin or stopped).
  //
  if( fabs(s) <= ReallyZero )
  {
    dx = 0.0;
    dy = 0.0;
    da = c;
  }

  //
  // Degenerate curve: straight path.
  //
  // The arc length is a straigt line.
  //
  else if( fabs(c) <= ReallyZero )
  {
    dx = s;
    dy = 0.0;
    da = 0.0;
  }

  //
  // Circular arc path.
  //
  // Note: The following comments are nice, but the calculations simplify when
  // delta distances are used.
  //
  // The center of rotation is a pi/2 rotation from the robot frame of
  // reference. Rotating 90 degrees and translating the center to 0,0, we
  // can calculate the delta position. From the deltas, the current absolute
  // position is determined.
  //
  // Using trig identities:
  //   sin(-u) = -sin(u)
  //   cos(-u) =  cos(u)
  //   sin(u) = cos(theta)
  //   cos(u) = sin(theta)
  //
  // Let  u = pi/2 - theta
  //     -u = theta - pi/2 
  // Then:
  //   dx = r * (m.cos(theta[cur]-m.pi/2) - m.cos(theta[prev]-m.pi/2))
  //   dy = r * (m.sin(theta[cur]-m.pi/2) - m.sin(theta[prev]-m.pi/2))
  //
  else
  {
    r  = s / c;
    dx = r * sin(c);
    dy = r * -cos(c);
    da = c;
  }

  //
  // New current pose.
  //
  pose.m_x      = m_pose[IdxPrev].m_x + dx;
  pose.m_y      = m_pose[IdxPrev].m_y + dy;
  pose.m_theta  = fmod(m_pose[IdxPrev].m_theta + da, M_TAU);

  pushNewPose(pose);

  //
  // Save to real-time DB.
  //
  RtDb.m_kin.m_robot.m_x          = m_pose[IdxCur].m_x;
  RtDb.m_kin.m_robot.m_y          = m_pose[IdxCur].m_y;
  RtDb.m_kin.m_robot.m_theta      = m_pose[IdxCur].m_theta;
  RtDb.m_kin.m_robot.m_fOdometer  = m_fOdometer;
  RtDb.m_kin.m_robot.m_fVelocity  = m_fVelocity;

  return LAE_OK;
}

int LaePlatform::updateCtlrHealth(int    nCtlr,
                              double fVolts,
                              double fTemp,
                              uint_t uStatus)
{
  if( (nCtlr < 0) || (nCtlr >= LaeNumMotorCtlrs) )
  {
    LOGERROR("Motor controller id %d: Out-of-range.", nCtlr);
    return -LAE_ECODE_BAD_VAL;
  }

  m_ctlr[nCtlr].m_fMainVolts = fVolts;
  m_ctlr[nCtlr].m_fBoardTemp = fTemp;
  m_ctlr[nCtlr].m_uStatus    = uStatus;

  return LAE_OK;
}

int LaePlatform::updateHealth(const LaeMapPowertrain &mapPowertrains)
{
  LaeMapPowertrain::const_iterator iter;

  int   n;

  m_fAmpsMotors = 0.0;
  m_fVoltsAvg   = 0.0;
  m_fTempAvg    = 0.0;

  n = 0;

  for(iter = mapPowertrains.begin(); iter != mapPowertrains.end(); ++iter)
  {
    const LaePowertrain &train = iter->second;

    m_fAmpsMotors += train.m_state.m_fAmps;
    m_fVoltsAvg   += train.m_state.m_fVolts;
    m_fTempAvg    += train.m_state.m_fTemp;

    ++n;
  }

  m_fVoltsAvg /= (double)n;
  m_fTempAvg  /= (double)n;

  RtDb.m_robotstatus.m_fTempAvg = m_fTempAvg;

  return LAE_OK;
}

void LaePlatform::clearPoses()
{
  for(int i = 0; i < HistSize; ++i)
  {
    m_pose[i].clear();
  }
}

void LaePlatform::pushNewPose(const LaePose &pose)
{
  // save pose
  m_pose[IdxPrev] = m_pose[IdxCur];

  // clear new pose
  m_pose[IdxCur] = pose;
}
