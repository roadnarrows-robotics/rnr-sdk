////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekCalibStretch.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-02-11 15:38:58 -0700 (Wed, 11 Feb 2015) $
 * $Rev: 3868 $
 *
 * \brief HekCalibStretch - Hekateros calibration by stretching class
 * implementation.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2013-2017. RoadNarrows LLC.\n
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

#include "rnr/rnrconfig.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaServo.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekJoint.h"
#include "Hekateros/hekMonitor.h"
#include "Hekateros/hekCalib.h"
#include "Hekateros/hekCalibStretch.h"
#include "Hekateros/hekRobot.h"

using namespace std;
using namespace hekateros;

// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Convenience macro for trying to get a servo object from dynamixel
 * chain.
 *
 * Failure is considered a software bug since the chain has already be 
 * verified.
 *
 * Only works locally.
 *
 * \param [in] nServoId   Servo id.
 * \param [out] pServo    Pointer to associated servo object.
 *
 * \return On failure, forces return from calling function with the appropriate
 * error code.
 */
#define HEK_TRY_GET_SERVO(nServoId, pServo) \
  do \
  { \
    if( (pServo = m_robot.m_pDynaChain->GetServo(nServoId)) == NULL ) \
    { \
      LOGERROR("BUG: Servo %d: Cannot find in dynamixel chain.", nServoId); \
      return -HEK_ECODE_INTERNAL; \
    } \
  } while(0)

/*!
 * \brief Calibration order by master servo id.
 */
static const string CalibOrder[] =
{
  "grip",
  "wrist_pitch",
  "wrist_rot",
  "elbow",
  "shoulder",
  "base_rot"
};


// ---------------------------------------------------------------------------
// Class HekCalibStretch
// ---------------------------------------------------------------------------

int HekCalibStretch::calibrate()
{
  MapRobotJoints::iterator  pos;

  string          strJointName;
  HekRobotJoint  *pJoint;
  int             eLimitTypes;
  size_t          i;
  int             rc;

  // reset odometer to current arm pose
  m_robot.m_pKin->resetServoOdometersForAllJoints();

  //
  // Calibrate Hekateros joints in the specified calibration order.
  //
  for(i=0, rc=HEK_OK; (i<arraysize(CalibOrder)) && (rc >= 0); ++i)
  {
    strJointName = CalibOrder[i];

    // no joint in this product version with associated joint name
    if( (pJoint = m_robot.getArmJoint(strJointName)) == NULL )
    {
      continue;
    }

    // already calibrated
    if( pJoint->m_eOpState == HekOpStateCalibrated )
    {
      continue;
    }

    LOGDIAG2("\n----------- Calibrating Joint %s...", strJointName.c_str());

    eLimitTypes         = pJoint->m_eLimitTypes;
    pJoint->m_eOpState  = HekOpStateCalibrating;

    // one or more top dead center electronic switch limits
    if( eLimitTypes & HekLimitTypeElecTDC )
    {
      rc = calibrateJointTopDeadCenter(*pJoint);
    }

    // one or more electronic switch limiters
    else if( eLimitTypes & HekLimitTypeElec )
    {
      rc = calibrateJointByLimits(*pJoint);
    }

    // physical limits
    else if( eLimitTypes & HekLimitTypePhys )
    {
      rc = calibrateJointByTorqueLimits(*pJoint);
    }

    // no detectable limits - assume current position is the good position
    else if( eLimitTypes & HekLimitTypeNone )
    {
      rc = calibrateJointByTrust(*pJoint);
    }

    else if( eLimitTypes & HekLimitTypeAbsEnc )
    {
      LOGWARN("Joint %s: Absolute encoders are not supported yet.",
          strJointName.c_str());
    }

    // warning
    else
    {
      LOGWARN("Joint %s: Do not know how to calibrate this joint.",
          strJointName.c_str());
    }

    pJoint->m_eOpState =  rc == HEK_OK? HekOpStateCalibrated:
                                        HekOpStateUncalibrated;
  }

  return rc;
}

int HekCalibStretch::calibrateJointTopDeadCenter(HekRobotJoint &joint)
{
  // joint calibrating velocity (radians/second)
  static double TuneCalVel    = degToRad(20.0);

  // quick try in the minimum direction (radians)
  static double TuneMinAngle  = degToRad(20.0);

  string      strJointName;     // joint name
  int         nServoId;         // servo id
  bool        bIsReverse;       // odometer is [not] reversed

  double      fJointCurPos;     // user's starting/current position (radians)
  double      fJointCurVel;     // joint current velocity (radians/second) 
  double      fJointGoalPos;    // joint working goal position (radians)
  double      fJointCalibPos;   // joint calibrated position (radians)

  int         nServoCurPos;     // servo current position (odometer ticks)
  int         nServoCurSpeed;   // servo current speed (raw unitless)

  byte_t      byOptMask;        // optical limits mask
  byte_t      byOptBits;        // optical limits bits
  int         nEdge;            // which edge triggered

  int         i;                // working index
  int         rc = HEK_OK;      // return code

  strJointName  = joint.m_strName;
  nServoId      = joint.m_nMasterServoId;
  bIsReverse    = joint.m_nMasterServoDir == DYNA_DIR_CCW? false: true;

  LOGDIAG2("Joint %s (servo=%d): Calibrating by TDC optical limits.",
          strJointName.c_str(), joint.m_nMasterServoId);

  //
  // Only continuous joints support tdc limits
  //
  if( joint.m_eJointType != HekJointTypeContinuous )
  {
    LOGERROR("Joint %s (servo=%d): Not a continuous joint.",
          strJointName.c_str(), joint.m_nMasterServoId);
    return -HEK_ECODE_NO_EXEC;
  }

  //
  // Build optical limits bit mask.
  //
  for(i=0, byOptMask=0x00; i<HekOptLimitMaxPerJoint; ++i)
  {
    byOptMask |= joint.m_byOptLimitMask[i];
  }

  //
  // Sanity check
  //
  if( byOptMask == 0x00 )
  {
    LOGERROR("Joint %s (servo=%d): No optical limit bits set, spec correct?",
          strJointName.c_str(), nServoId);
    return -HEK_ECODE_INTERNAL;
  }

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Find joint top-dead-center position.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
 
  LOGDIAG2("Joint %s (servo=%d): "
           "Determining joint top-dead-center by optical limit(s).",
      strJointName.c_str(), nServoId);

  m_robot.m_pKin->getJointCurPosVel(strJointName, fJointCurPos, fJointCurVel);

  LOGDIAG2("Joint %s (servo=%d): Starting at uncalibrated %.2lf degrees.",
      strJointName.c_str(), nServoId, radToDeg(fJointCurPos));

  // Never auto-stop at TDC limits.
  joint.m_bStopAtOptLimits = false;

  byOptBits = getDarkOpticalLimits(m_robot.m_monitor.getJointLimitBits(),
                                   byOptMask);

  //
  // Starting in the light.  First try a small degree of rotation in the
  // minimum direction to find an optical limit dark edge.
  //
  if( byOptBits == 0x00 )
  {
    LOGDIAG2("Joint %s (servo=%d): "
           "Determining joint optical limit light-to-dark edge.",
      strJointName.c_str(), nServoId);

    m_robot.m_pKin->getJointCurPosVel(strJointName, fJointCurPos, fJointCurVel);

    fJointGoalPos = fJointCurPos - TuneMinAngle;
    nEdge         = 1;  // move will detect edge on maximum side

    LOGDIAG2("Joint %s (servo=%d): Move to the dark for <= %.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(fJointGoalPos));

    moveToDark(strJointName, fJointGoalPos, TuneCalVel, byOptMask);

    m_robot.m_pKin->getJointCurPosVel(strJointName, fJointCurPos, fJointCurVel);

    LOGDIAG2("Joint %s (servo=%d): Stopped at %.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(fJointCurPos));
  }

  byOptBits = getDarkOpticalLimits(m_robot.m_monitor.getJointLimitBits(),
                                   byOptMask);

  //
  // Did not find the optical limit. Do a complete spin in the opposite
  // direction to find a limit.
  //
  if( byOptBits == 0x00 )
  {
    LOGDIAG2("Joint %s (servo=%d): Did not find the dark. Reverse direction.",
        strJointName.c_str(), nServoId);

    m_robot.m_pKin->getJointCurPosVel(strJointName, fJointCurPos, fJointCurVel);

    fJointGoalPos = fJointCurPos + M_TAU;
    nEdge         = -1;   // move will detect edge on minimum side

    LOGDIAG2("Joint %s (servo=%d): "
            "Move to the dark for <= %.2lf degrees in the maximum direction.",
        strJointName.c_str(), nServoId, radToDeg(fJointGoalPos));

    moveToDark(strJointName, fJointGoalPos, TuneCalVel, byOptMask);

    m_robot.m_pKin->getJointCurPosVel(strJointName, fJointCurPos, fJointCurVel);

    LOGDIAG2("Joint %s (servo=%d): Stopped at %.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(fJointCurPos));
  }

  byOptBits = getDarkOpticalLimits(m_robot.m_monitor.getJointLimitBits(),
                                   byOptMask);

  //
  // Make sure that an optical limits has been found.
  //
  if( byOptBits == 0x00 )
  {
    LOGERROR("Joint %s (servo=%d): Did not find any optical limits.",
          strJointName.c_str(), nServoId);
    return -HEK_ECODE_NO_EXEC;
  }

  //
  // Fine tune center of optical limit.
  //
  for(i=0, byOptMask=0x00; i<HekOptLimitMaxPerJoint; ++i)
  {
    if( byOptBits & joint.m_byOptLimitMask[i] )
    {
      rc = fineTuneTDC(joint, joint.m_byOptLimitMask[i], nEdge, fJointCalibPos);
      break;
    }
  }

  if( rc < 0 )
  {
    LOGERROR("Joint %s (servo=%d): Fine tuning failed.",
          strJointName.c_str(), nServoId);
    return rc;
  }

  //
  // Move to 0 degrees the shortest way.
  //
  if( fJointCalibPos != 0.0 )
  {
    if( fJointCalibPos < M_PI )
    {
      fJointGoalPos = fJointCalibPos;
    }
    else
    {
      fJointGoalPos = M_TAU - fJointCalibPos;
    }

    LOGDIAG2("Joint %s (servo=%d): "
            "Move to the new zero point at %.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(fJointGoalPos));

    moveWait(strJointName, fJointGoalPos, TuneCalVel);

    m_robot.m_pKin->getJointCurPosVel(strJointName, fJointCurPos, fJointCurVel);

    LOGDIAG2("Joint %s (servo=%d): Stopped at %.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(fJointCurPos));
  }

  // 
  // Reset odometer to new zero degree position.
  //
  m_robot.m_pKin->resetServoOdometer(strJointName);

  LOGDIAG2("Joint %s (servo=%d): Odometer reset.",
        strJointName.c_str(), nServoId);

  //
  // Adjust limits and positions to adjusted zero degree point.
  //
  joint.m_fMinSoftLimitRads = 0.0;
  joint.m_fMaxSoftLimitRads = M_TAU;
  joint.m_nMinSoftLimitOd   = 0;
  joint.m_nMaxSoftLimitOd   = (int)(joint.m_fMaxSoftLimitRads *
                                    joint.m_fTicksPerJointRad);
 
  //
  // Finally move to the pre-defined calibration zero point position.
  //
  if( joint.m_fCalibPosRads != 0.0 )
  { 
    LOGDIAG2("Joint %s (servo=%d): Move to calibrated zero pt=%.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(joint.m_fCalibPosRads));

    moveWait(strJointName, joint.m_fCalibPosRads, TuneCalVel);

   m_robot.m_pKin->getJointCurPosVel(strJointName, fJointCurPos, fJointCurVel);

    LOGDIAG2("Joint %s (servo=%d): Stopped at %.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(fJointCurPos));
  }

  //
  // Calibrated.
  //
  m_robot.m_pKin->getServoCurPosSpeed(strJointName,
                                      nServoCurPos, nServoCurSpeed);

  LOGDIAG2("Joint %s (servo=%d): Calibrated by TDC optical limits:\n"
        "  Top dead center: %.02lfdeg (od=%d).",
      strJointName.c_str(), nServoId, 0.0, nServoCurPos);

  return HEK_OK;
}

int HekCalibStretch::calibrateJointByLimits(HekRobotJoint &joint)
{
  // joint calibrating velocity (radians/second)
  static double TuneCalVel      = degToRad(20.0);

  // additional angle margin (radians)
  static double TuneMarginAngle = degToRad(50.0);

  string      strJointName;     // joint name
  int         nServoId;         // servo id
  bool        bIsReverse;       // odometer is [not] reversed

  double      fJointCurPos;     // user's starting/current position (radians)
  double      fJointCurVel;     // joint current velocity (radians/second) 
  double      fJointGoalPos;    // joint working goal position (radians)
  double      fJointCalibPos;   // joint calibrated position (radians)

  int         nServoCurPos;     // servo current position (odometer ticks)
  int         nServoCurSpeed;   // servo current speed (raw unitless)

  byte_t      byOptMask;        // optical limits mask
  byte_t      byOptBits;        // optical limits bits
  HekOpticalLimit_T *pLimit;    // limit info associated with mask

  int         nDir;             // working direction
  double      fAngle1;          // working angle 1
  double      fAngle2;          // working angle 2

  int         i;                // working index
  int         rc = HEK_OK;      // return code

  //
  // Initialize fixed working data.
  //
  // Note: Only work with one optical limit per joint for now. Can extend if
  // needed.
  //
  strJointName  = joint.m_strName;
  nServoId      = joint.m_nMasterServoId;
  bIsReverse    = joint.m_nMasterServoDir == DYNA_DIR_CCW? false: true;
  byOptMask     = joint.m_byOptLimitMask[0];
  pLimit        = m_robot.m_monitor.getJointLimitInfo(byOptMask);

  LOGDIAG2("Joint %s (servo=%d): Calibrating by optical limits.",
          strJointName.c_str(), joint.m_nMasterServoId);

  //
  // Sanity check.
  //
  if( byOptMask == 0x00 )
  {
    LOGERROR("Joint %s (servo=%d): No optical limit bits set, spec correct?",
          strJointName.c_str(), nServoId);
    return -HEK_ECODE_INTERNAL;
  }

  // Disable optical limit auto-stopping for the joint during calibration.
  joint.m_bStopAtOptLimits = false;

  m_robot.m_pKin->getJointCurPosVel(strJointName, fJointCurPos, fJointCurVel);

  LOGDIAG2("Joint %s (servo=%d): Starting at %.2lf uncalibrated degrees.",
      strJointName.c_str(), nServoId, radToDeg(fJointCurPos));

  //
  // Get current optical limit values.
  //
  byOptBits = getDarkOpticalLimits(m_robot.m_monitor.getJointLimitBits(),
                                   byOptMask);

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Starting in the dark.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  if( byOptBits != 0x00 )
  {
    LOGDIAG2("Joint %s (servo=%d): "
           "Determining joint optical limit dark-to-light edge.",
      strJointName.c_str(), nServoId);

    // best guess direction based on configured parked position for this joint.
    nDir = joint.m_fParkPosRads < 0? 1: -1;

    //
    // Calculate the maximum of the minimum travel angles to reach an edge.
    //
    fAngle1 = pLimit->m_fMinBlackPos   - joint.m_fMinPhyLimitRads;
    fAngle2 = joint.m_fMaxPhyLimitRads - pLimit->m_fMaxBlackPos;

    if( fAngle2 > fAngle1 )
    {
      fAngle1 = fAngle2;
    }

    fAngle1 += TuneMarginAngle;

    m_robot.m_pKin->getJointCurPosVel(strJointName, fJointCurPos, fJointCurVel);

    fJointGoalPos = fJointCurPos + (double)nDir * fAngle1;

    LOGDIAG2("Joint %s (servo=%d): Move to light for <= %.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(fJointGoalPos));

    // Move to occlusion band edge.
    multiMoveToLight(strJointName, nDir, fJointGoalPos, TuneCalVel, byOptMask);

    m_robot.m_pKin->getJointCurPosVel(strJointName, fJointCurPos, fJointCurVel);

    LOGDIAG2("Joint %s (servo=%d): Stopped at %.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(fJointCurPos));

    // optical bits
    byOptBits = m_robot.m_monitor.getJointLimitBits();

    //
    // Did't find the light, probably guessed wrong direction and joint is at
    // a physical limit. Reverse direction.
    //
    if( !getLitOpticalLimits(byOptBits, byOptMask) )
    {
      LOGDIAG2("Joint %s (servo=%d): "
              "Did not find the light. Reverse direction.",
        strJointName.c_str(), nServoId);

      nDir = -nDir;

      m_robot.m_pKin->getJointCurPosVel(strJointName,
                                        fJointCurPos, fJointCurVel);

      fJointGoalPos = fJointCurPos + (double)nDir * fAngle1;

      LOGDIAG2("Joint %s (servo=%d): Move to light <= %.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(fJointGoalPos));

      multiMoveToLight(strJointName, nDir, fJointGoalPos, TuneCalVel,
                      byOptMask);

      m_robot.m_pKin->getJointCurPosVel(strJointName,
                                        fJointCurPos, fJointCurVel);

      LOGDIAG2("Joint %s (servo=%d): Stopped at %.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(fJointCurPos));
    }

    // optical bits
    byOptBits = m_robot.m_monitor.getJointLimitBits();

    //
    // Got an edge, perform final calibration fine tuning.
    //
    if( getLitOpticalLimits(byOptBits, byOptMask) )
    {
      m_robot.m_pKin->getJointCurPosVel(strJointName,
                                        fJointCurPos, fJointCurVel);

      LOGDIAG2("Joint %s (servo=%d): Got a light edge at %.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(fJointCurPos));

      return calibByLimitsFinal(joint, byOptMask, nDir);
    }

    //
    // Still no light. Give up.
    //
    else
    {
      LOGERROR("Joint %s (servo=%d): "
               "Could not find optical limit switch light.",
          strJointName.c_str(), nServoId);
      return -HEK_ECODE_NO_EXEC;
    }
  }


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Starting in the light.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  else
  {
    // best guess direction
    nDir = nServoId == HekServoIdShoulderL? -1: 1;

    //
    // Calculate the maximum angle to reach an edge.
    //
    fAngle1  = pLimit->m_fMaxBlackPos - pLimit->m_fMinBlackPos;
    fAngle1 += TuneMarginAngle;

    LOGDIAG2("Joint %s (servo=%d): "
           "Determining joint optical limit light-to-dark minimum edge.",
      strJointName.c_str(), nServoId);

    m_robot.m_pKin->getJointCurPosVel(strJointName, fJointCurPos, fJointCurVel);

    fJointGoalPos = fJointCurPos + (double)nDir * fAngle1;

    LOGDIAG2("Joint %s (servo=%d): Move to dark for <= %.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(fJointGoalPos));

    // Move to occlusion band edge.
    multiMoveToDark(strJointName, nDir, fJointGoalPos, TuneCalVel, byOptMask);

    m_robot.m_pKin->getJointCurPosVel(strJointName, fJointCurPos, fJointCurVel);

    LOGDIAG2("Joint %s (servo=%d): Stopped at %.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(fJointCurPos));

    // optical bits
    byOptBits = m_robot.m_monitor.getJointLimitBits();

    //
    // Got an edge, perform final calibration fine tuning.
    //
    if( getDarkOpticalLimits(byOptBits, byOptMask) )
    {
      m_robot.m_pKin->getJointCurPosVel(strJointName,
                                        fJointCurPos, fJointCurVel);

      LOGDIAG2("Joint %s (servo=%d): Got a dark edge at %.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(fJointCurPos));

      return calibByLimitsFinal(joint, byOptMask, nDir);
    }

    else
    {
      LOGERROR("Joint %s (servo=%d): "
             "Could not find optical limit occlusion edge.",
          strJointName.c_str(), nServoId);

      return -HEK_ECODE_NO_EXEC;
    }
  }

  LOGERROR("BUG: Joint %s (servo=%d): Should not be here.",
          strJointName.c_str(), nServoId);

  return -HEK_ECODE_INTERNAL;
}

int HekCalibStretch::calibByLimitsFinal(HekRobotJoint &joint,
                                        byte_t         byOptMask,
                                        int            nDir)
{
  // joint calibrating velocity (radians/second)
  static double TuneCalVel      = degToRad(20.0);

  // additional angle margin (radians)
  static double TuneMarginAngle = degToRad(10.0);

  string      strJointName;     // joint name
  int         nServoId;         // servo id
  bool        bIsReverse;       // odometer is [not] reversed

  HekOpticalLimit_T  *pLimit;       // limit info associated with mask

  double      fJointCurPos;     // user's starting/current position (radians)
  double      fJointCurVel;     // joint current velocity (radians/second) 
  double      fJointGoalPos;    // joint working goal position (radians)
  double      fJointCalibPos;   // joint calibrated position (radians)

  int         nServoCurPos;     // servo current position (odometer ticks)
  int         nServoCurSpeed;   // servo current speed (raw unitless)

  int         i;                // working index
  int         rc;               // return code

  strJointName  = joint.m_strName;
  nServoId      = joint.m_nMasterServoId;
  bIsReverse    = joint.m_nMasterServoDir == DYNA_DIR_CCW? false: true;
  pLimit        = m_robot.m_monitor.getJointLimitInfo(byOptMask);

  // fine tune edge
  if( (rc = fineTuneLimit(joint, byOptMask, nDir, fJointCalibPos)) < 0 )
  {
    LOGERROR("Joint %s (servo=%d): Fine tuning failed.",
          strJointName.c_str(), nServoId);
    return rc;
  }

  m_robot.m_pKin->getJointCurPosVel(strJointName, fJointCurPos, fJointCurVel);

  fJointGoalPos = fJointCurPos - fJointCalibPos;

  //
  // Special joint movements.
  //
  // Note: Slow down the elbow a tad to reduce hard stop at new zero point.
  //       Looks cleaner.
  //
  if( strJointName == "shoulder" )
  {
    string  strJointName2("elbow");
    double  fGearRatios = calcJointRatios(strJointName2, strJointName);
    m_robot.m_pKin->move(strJointName2, 0.0, 0.75 * fGearRatios * TuneCalVel);
  }

  LOGDIAG2("Joint %s (servo=%d): Move to the new zero point at %.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(fJointGoalPos));

  moveWait(strJointName, fJointGoalPos, TuneCalVel);

  m_robot.m_pKin->getJointCurPosVel(strJointName, fJointCurPos, fJointCurVel);

  LOGDIAG2("Joint %s (servo=%d): Stopped at %.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(fJointCurPos));

  m_robot.m_pKin->resetServoOdometer(strJointName);

  LOGDIAG2("Joint %s (servo=%d): Odometer reset.",
        strJointName.c_str(), nServoId);

  //
  // Adjust limits and positions.
  //
  joint.m_fMinSoftLimitRads = pLimit->m_fMinEdgePos;
  joint.m_fMaxSoftLimitRads = pLimit->m_fMaxEdgePos;
  joint.m_nMinSoftLimitOd   = (int)(joint.m_fMinSoftLimitRads *
                                        joint.m_fTicksPerJointRad);
  joint.m_nMaxSoftLimitOd   = (int)(joint.m_fMaxSoftLimitRads *
                                        joint.m_fTicksPerJointRad);

  // Enable optical limit stopping for joint during normal operation.
  joint.m_bStopAtOptLimits = true;
 
  LOGDIAG2("Joint %s (servo=%d): Calibrated by optical limits:\n"
          "  Physical limits: [%.2lfdeg (od=%d), %.2lfdeg (od=%d)]\n"
          "      Soft limits: [%.2lfdeg (od=%d), %.2lfdeg (od=%d)].",
      strJointName.c_str(), nServoId,
      radToDeg(joint.m_fMinPhyLimitRads),  joint.m_nMinPhyLimitOd,
      radToDeg(joint.m_fMaxPhyLimitRads),  joint.m_nMaxPhyLimitOd,
      radToDeg(joint.m_fMinSoftLimitRads), joint.m_nMinSoftLimitOd,
      radToDeg(joint.m_fMaxSoftLimitRads), joint.m_nMaxSoftLimitOd);

  return HEK_OK;
}

int HekCalibStretch::calibrateJointByTorqueLimits(HekRobotJoint &joint)
{
  // joint calibrating velocity (radians/second)
  static double TuneCalVel  = degToRad(20.0);

  // backoff angle margin (radians)
  static double TuneBackoff = degToRad(0.1);

  string      strJointName;     // joint name
  int         nServoId;         // servo id
  bool        bIsReverse;       // odometer is [not] reversed
  DynaServo  *pServo;           // servo control

  double      fMinLimit;        // joint minimum position
  double      fMaxLimit;        // joint maximum position
  double      fJointRange;      // joint range of motion

  double      fJointStartPos;   // user's starting position (radians)
  double      fJointCurPos;     // joint current position (radians)
  double      fJointCurVel;     // joint current velocity (radians/second) 
  double      fJointGoalPos;    // joint working goal position (radians)

  int         nServoPos;        // working servo position (odometer ticks);

  strJointName  = joint.m_strName;
  nServoId      = joint.m_nMasterServoId;
  bIsReverse    = joint.m_nMasterServoDir == DYNA_DIR_CCW? false: true;

  LOGDIAG2("Joint %s (servo=%d): Calibrating by torque limits.",
          strJointName.c_str(), joint.m_nMasterServoId);

  HEK_TRY_GET_SERVO(nServoId, pServo);

  // starting position
  m_robot.m_pKin->getJointCurPosVel(strJointName, fJointStartPos, fJointCurVel);
 
  LOGDIAG2("Joint %s (servo=%d): Starting at %.2lf uncalibrated degrees.",
      strJointName.c_str(), nServoId, radToDeg(fJointStartPos));

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Find minimum joint limit.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  LOGDIAG2("Joint %s (servo=%d): "
           "Determining minimum joint angle by torque limit.",
      joint.m_strName.c_str(), nServoId);

  fJointCurPos = fJointStartPos;

  //
  // A continuous mode servo must have its joint minimum limit within 360
  // degreees of joint rotation from the current position. Go to this endpoint
  // until the torque limit is found.
  //
  if( joint.m_bIsServoContinuous )
  {
    fJointGoalPos = -M_TAU + -fabs(fJointCurPos);
  }

  //
  // A servo mode servo has encoder endpoints that are known and finite. Go to
  // the appropriate endpoint until the torque limit is found.
  //
  else
  {
    if( (nServoPos = pServo->CalcOdometerAtEncMin()) > 0 )
    {
      nServoPos = pServo->CalcOdometerAtEncMax();
    }
    fJointGoalPos = m_robot.m_pKin->servoPosToJointPos(strJointName, nServoPos);
  }

  LOGDIAG2("Joint %s (servo=%d): Move to minimum >= %.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(fJointGoalPos));

  moveToTorqueLimit(strJointName, fJointGoalPos, TuneCalVel);

  // save minimum limit position
  m_robot.m_pKin->getJointCurPosVel(strJointName, fMinLimit, fJointCurVel);

  LOGDIAG2("Joint %s (servo=%d): Minimum reached at %.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(fMinLimit));

  //
  // Off load servo by moving back to starting calibration position.
  //
  LOGDIAG2("Joint %s (servo=%d): Move back to start at %.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(fJointStartPos));

  moveWait(strJointName, fJointStartPos, TuneCalVel);

  m_robot.m_pKin->getJointCurPosVel(strJointName, fJointCurPos, fJointCurVel);

  LOGDIAG2("Joint %s (servo=%d): Stopped at %.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(fJointCurPos));

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Find maximum limit.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  LOGDIAG2("Joint %s (servo=%d): "
           "Determining maximum joint angle by torque limit.",
      joint.m_strName.c_str(), nServoId);

  // current position
  m_robot.m_pKin->getJointCurPosVel(strJointName, fJointCurPos, fJointCurVel);
  
  //
  // A continuous mode servo must have its joint minimum limit within 360
  // degreees of joint rotation from the current position. Go to this endpoint
  // until the torque limit is found.
  //
  if( joint.m_bIsServoContinuous )
  {
    fJointGoalPos = M_TAU + fabs(fJointCurPos);
  }

  //
  // A servo mode servo has encoder endpoints that are known and finite. Go to
  // the appropriate endpoint until the torque limit is found.
  //
  else
  {
    if( (nServoPos = pServo->CalcOdometerAtEncMin()) < 0 )
    {
      nServoPos = pServo->CalcOdometerAtEncMax();
    }
    fJointGoalPos = m_robot.m_pKin->servoPosToJointPos(strJointName, nServoPos);
  }

  LOGDIAG2("Joint %s (servo=%d): Move to maximum <= %.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(fJointGoalPos));

  moveToTorqueLimit(strJointName, fJointGoalPos, TuneCalVel);

  // save maximum limit position
  m_robot.m_pKin->getJointCurPosVel(strJointName, fMaxLimit, fJointCurVel);

  LOGDIAG2("Joint %s (servo=%d): Maximum reached at %.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(fMaxLimit));

  //
  // Off load servo by moving back to starting calibration position
  //
  LOGDIAG2("Joint %s (servo=%d): Move back again to start at %.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(fJointStartPos));

  moveWait(strJointName, fJointStartPos, TuneCalVel);

  m_robot.m_pKin->getJointCurPosVel(strJointName, fJointCurPos, fJointCurVel);

  LOGDIAG2("Joint %s (servo=%d): Stopped at %.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(fJointCurPos));

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Set zero point and limits.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  //
  // Joint range of motion.
  //
  fJointRange = fMaxLimit - fMinLimit;

  //
  // Move to new zero point position.
  //
  fJointGoalPos = fMinLimit;

  LOGDIAG2("Joint %s (servo=%d): Move to the new zero point at %.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(fJointGoalPos));

  moveToTorqueLimit(strJointName, fJointGoalPos, TuneCalVel);

  m_robot.m_pKin->getJointCurPosVel(strJointName, fJointCurPos, fJointCurVel);

  LOGDIAG2("Joint %s (servo=%d): Stopped at %.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(fJointCurPos));

  // 
  // Reset odometer to new zero degree position.
  //
  m_robot.m_pKin->resetServoOdometer(strJointName);

  m_robot.m_pKin->getJointCurPosVel(strJointName, fJointCurPos, fJointCurVel);

  LOGDIAG2("Joint %s (servo=%d): Odometer reset.",
        strJointName.c_str(), nServoId);

  //
  // Adjust limits and positions to adjusted zero degree point.
  //
  joint.m_fMinPhyLimitRads  = fJointCurPos;
  joint.m_fMaxPhyLimitRads  = fJointCurPos + fJointRange;

  joint.m_nMinPhyLimitOd    = m_robot.m_pKin->jointPosToServoPos(
                                                    strJointName,
                                                    joint.m_fMinPhyLimitRads);
  joint.m_nMaxPhyLimitOd    = m_robot.m_pKin->jointPosToServoPos(
                                                    strJointName,
                                                    joint.m_fMaxPhyLimitRads);

  joint.m_fMinSoftLimitRads   = joint.m_fMinPhyLimitRads + TuneBackoff;
  joint.m_fMaxSoftLimitRads   = joint.m_fMaxPhyLimitRads - TuneBackoff;

  // hack - should be in spec
  if( nServoId == HekServoIdGraboid )
  {
    joint.m_fMaxSoftLimitRads = joint.m_fMaxPhyLimitRads - degToRad(1.0);
  }


  joint.m_nMinSoftLimitOd   = m_robot.m_pKin->jointPosToServoPos(
                                                    strJointName,
                                                  joint.m_fMinSoftLimitRads);
  joint.m_nMaxSoftLimitOd   = m_robot.m_pKin->jointPosToServoPos(
                                                  strJointName,
                                                  joint.m_fMaxSoftLimitRads);
  
  
  //
  // Finally move to pre-defined calibration zero point position.
  //
  if( joint.m_fCalibPosRads != 0.0 )
  { 
    LOGDIAG2("Joint %s (servo=%d): Move to calibrated zero pt=%.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(joint.m_fCalibPosRads));

    moveWait(strJointName, joint.m_fCalibPosRads, TuneCalVel);

    m_robot.m_pKin->getJointCurPosVel(strJointName, fJointCurPos, fJointCurVel);

    LOGDIAG2("Joint %s (servo=%d): Stopped at %.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(fJointCurPos));
  }

  //
  // Calibrated.
  //
  LOGDIAG2("Joint %s (servo=%d): Calibrated by torque:\n"
          "  Physical limits: [%.2lfdeg (od=%d), %.2lfdeg (od=%d)]\n"
          "      Soft limits: [%.2lfdeg (od=%d), %.2lfdeg (od=%d)].",
      joint.m_strName.c_str(), nServoId,
      radToDeg(joint.m_fMinPhyLimitRads),  joint.m_nMinPhyLimitOd,
      radToDeg(joint.m_fMaxPhyLimitRads),  joint.m_nMaxPhyLimitOd,
      radToDeg(joint.m_fMinSoftLimitRads), joint.m_nMinSoftLimitOd,
      radToDeg(joint.m_fMaxSoftLimitRads), joint.m_nMaxSoftLimitOd);

  return HEK_OK;
}

int HekCalibStretch::calibrateJointByTrust(HekRobotJoint &joint)
{
  string      strJointName;   // joint name
  int         nServoId;       // servo id
  bool        bIsReverse;     // odometer is [not] reversed

  double      fJointCurPos;   // joint current position (radians)
  double      fJointCurVel;   // joint current velocity (radians/second) 

  int         nServoCurPos;   // servo current position (odometer ticks)
  int         nServoCurSpeed; // servo current speed (raw unitless)

  strJointName  = joint.m_strName;
  nServoId      = joint.m_nMasterServoId;
  bIsReverse    = joint.m_nMasterServoDir == DYNA_DIR_CCW? false: true;

  LOGDIAG2("Joint %s (servo=%d): Calibrating by trust.",
          strJointName.c_str(), joint.m_nMasterServoId);

  // 
  // Reset odometer to the current position
  //
  m_robot.m_pKin->resetServoOdometer(strJointName);

  // starting position
  m_robot.m_pKin->getJointCurPosVel(strJointName, fJointCurPos, fJointCurVel);
  m_robot.m_pKin->getServoCurPosSpeed(strJointName,
                                        nServoCurPos,
                                        nServoCurSpeed);

  if( joint.m_eJointType == HekJointTypeContinuous )
  {
    LOGDIAG2("Joint %s (servo=%d): Calibrated by trust:\n"
            "  Top dead center: %.2lf deg (od=%d).",
      strJointName.c_str(), nServoId, radToDeg(fJointCurPos), nServoCurPos);
  }
  else
  {
    LOGDIAG2("Joint %s (servo=%d): Calibrated by trust:\n"
         "  Rotation [min, max] limits: [%.2lfdeg (od=%d), %.2lfdeg (od=%d)].",
            strJointName.c_str(), nServoId,
            radToDeg(joint.m_fMinPhyLimitRads), joint.m_nMinPhyLimitOd,
            radToDeg(joint.m_fMaxPhyLimitRads), joint.m_nMaxPhyLimitOd);
  }

  return HEK_OK;
}

int HekCalibStretch::fineTuneTDC(HekRobotJoint &joint,
                                 byte_t         byOptMask,
                                 int            nEdge,
                                 double        &fPosition)
{
  // joint calibrating velocity (radians/second)
  static double TuneCalVel  = degToRad(10.0);

  // tuning slop (radians)
  static double TuneFudge  = degToRad(5.0);

  string      strJointName; // joint name
  int         nServoId;     // servo id

  byte_t      byOptBits;        // optical limits bits
  HekOpticalLimit_T  *pLimit;   // limit info associated with mask
  double      fBandWidth;       // occlusion band width (radians)

  double      fJointStartPos;   // user's starting position (radians)
  double      fJointCurPos;     // joint current position (radians)
  double      fJointCurVel;     // joint current velocity (radians/second) 
  double      fJointGoalPos;    // joint working goal position (radians)

  double      fLitEdgePos0;     // joint lit edge position 0 (radians)
  double      fLitEdgePos1;     // joint lit edge position 1 (radians)

  int         rc = HEK_OK;      // return code

  strJointName  = joint.m_strName;
  nServoId      = joint.m_nMasterServoId;
  pLimit        = m_robot.m_monitor.getJointLimitInfo(byOptMask);

  LOGDIAG3("Joint %s (servo=%d): opt_mask=0x%02x, edge=%d.",
        strJointName.c_str(), nServoId, byOptMask, nEdge);

  LOGDIAG2("Joint %s (servo=%d): "
           "Occlusion band: max=%lfdeg, center=%lfdeg, min=%lfdeg.",
        strJointName.c_str(), nServoId,
        radToDeg(pLimit->m_fMaxEdgePos),
        radToDeg(pLimit->m_fCenterPos), 
        radToDeg(pLimit->m_fMinEdgePos));

  // width of occlusion band + some
  fBandWidth = pLimit->m_fMaxEdgePos - pLimit->m_fMinEdgePos + TuneFudge;

  // starting position
  m_robot.m_pKin->getJointCurPosVel(strJointName, fJointStartPos, fJointCurVel);

  //
  // Move to the light side of one of the occlusion band edges.
  // 
  fJointGoalPos = fJointStartPos + nEdge * fBandWidth;

  LOGDIAG2("Joint %s (servo=%d): "
      "Fine tune move for <= %.2lf degrees to find the first light edge.",
        strJointName.c_str(), nServoId, radToDeg(fJointGoalPos));

  moveToLight(strJointName, fJointGoalPos, TuneCalVel, byOptMask);

  // optical bits
  byOptBits = m_robot.m_monitor.getJointLimitBits();

  //
  // Found edge 0.
  //
  if( getLitOpticalLimits(byOptBits, byOptMask) != 0x00 )
  {
    // mark edge 0
    m_robot.m_pKin->getJointCurPosVel(strJointName, fLitEdgePos0, fJointCurVel);

    LOGDIAG2("Joint %s (servo=%d): Edge 0 at %.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(fLitEdgePos0));
  }

  //
  // Didn't find the light.
  //
  else
  {
    if( m_robot.m_pKin->hasOverTorqueCondition(strJointName) )
    {
      LOGERROR("Joint %s (servo=%d): Cannot calibrate: Joint is obstucted.",
          joint.m_strName.c_str(), nServoId);
      rc = -HEK_ECODE_COLLISION;
    }
    else
    {
      LOGERROR("Joint %s (servo=%d): Cannot calibrate: "
               "Did not find edge 0 of the occlusion band.",
          joint.m_strName.c_str(), nServoId);
      rc = -HEK_ECODE_NO_EXEC;
    }
  }

  //
  // Move in opposite direction to the light side of the other occlusion band
  // edge.
  //
  // Note: Even on any failures above, try to move back to a probably more
  // reasonable position.
  // 

  // first back into the dark
  LOGDIAG2("Joint %s (servo=%d): "
      "Move back into the occlusion band at %.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(fJointStartPos));

  moveToDark(strJointName, fJointStartPos, TuneCalVel, byOptMask);

  // current position in dark band
  m_robot.m_pKin->getJointCurPosVel(strJointName, fJointCurPos, fJointCurVel);

  LOGDIAG2("Joint %s (servo=%d): Stopped at %.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(fJointCurPos));

  //
  // Move to the opposite light side of one of the occlusion band edges.
  // 
  fJointGoalPos = fJointCurPos - nEdge * fBandWidth;

  LOGDIAG2("Joint %s (servo=%d): "
      "Fine tune move for %.2lf degrees to find the opposite light edge.",
        strJointName.c_str(), nServoId, radToDeg(fJointGoalPos));

  // then move to the opposite light
  moveToLight(strJointName, fJointGoalPos, TuneCalVel, byOptMask);

  // optical bits
  byOptBits = m_robot.m_monitor.getJointLimitBits();

  //
  // Found edge 1.
  //
  if( getLitOpticalLimits(byOptBits, byOptMask) != 0x00 )
  {
    // mark edge 1
    m_robot.m_pKin->getJointCurPosVel(strJointName, fLitEdgePos1, fJointCurVel);

    LOGDIAG2("Joint %s (servo=%d): Edge 1 at %.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(fLitEdgePos1));
  }

  //
  // Didn't find the light edge.
  //
  else
  {
    if( m_robot.m_pKin->hasOverTorqueCondition(strJointName) )
    {
      LOGERROR("Joint %s (servo=%d): Cannot calibrate: Joint is obstucted.",
          joint.m_strName.c_str(), nServoId);
      rc = (rc == HEK_OK)? -HEK_ECODE_COLLISION: rc;
    }
    else
    {
      LOGERROR("Joint %s (servo=%d): Cannot calibrate: "
               "Did not find edge 1 of occlusion band.",
          joint.m_strName.c_str(), nServoId);
      rc = (rc == HEK_OK)? -HEK_ECODE_NO_EXEC: rc;
    }
  }

  //
  // Move to center of occlusion band.
  // 
  if( rc == HEK_OK )
  {
    fJointGoalPos = (fLitEdgePos1 + fLitEdgePos0) / 2.0;

    LOGDIAG2("Joint %s (servo=%d): Move to TDC at %.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(fJointGoalPos));

    moveWait(strJointName, fJointGoalPos, TuneCalVel);

    m_robot.m_pKin->getJointCurPosVel(strJointName, fJointCurPos, fJointCurVel);

    fPosition = pLimit->m_fCenterPos; 

    LOGDIAG2("Joint %s (servo=%d): Fine tune TDC diff = %.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(fPosition));
  }

  return rc;
}

int HekCalibStretch::fineTuneLimit(HekRobotJoint &joint,
                                   byte_t         byOptMask,
                                   int            nDir,
                                   double        &fDeltaPos)
{
  // joint calibrating velocity (radians/second)
  static double TuneCalVel  = degToRad(5.0);

  // tuning slop (radians)
  static double TuneFudge  = degToRad(10.0);

  string      strJointName;     // joint name
  int         nServoId;         // servo id

  HekOpticalLimit_T  *pLimit;   // limit info associated with mask
  byte_t      byOptBits;        // optical limits bits

  double      fJointCurPos;     // joint current position (radians)
  double      fJointCurVel;     // joint current velocity (radians/second) 
  double      fJointGoalPos;    // joint working goal position (radians)
  double      fLightPos;        // occlusion band light position (radian)
  double      fDarkPos;         // occlusion band dark position (radian)
  double      fEdgePos;         // occlusion band edge position (radians)

  int         rc = HEK_OK;  // return code

  strJointName  = joint.m_strName;
  nServoId      = joint.m_nMasterServoId;
  pLimit        = m_robot.m_monitor.getJointLimitInfo(byOptMask);

  m_robot.m_pKin->stop(strJointName);

  // optical bits
  byOptBits = m_robot.m_monitor.getJointLimitBits();

  // need to start in the light
  if( !getLitOpticalLimits(byOptBits, byOptMask) )
  {
    nDir = -nDir;

    // current position
    m_robot.m_pKin->getJointCurPosVel(strJointName, fJointCurPos, fJointCurVel);

    fJointGoalPos = fJointCurPos + (double)nDir * TuneFudge;

    LOGDIAG2("Joint %s (servo=%d): "
      "Fine tune move for <= %.2lf degrees to find light edge.",
        strJointName.c_str(), nServoId, radToDeg(fJointGoalPos));

    moveToLight(strJointName, fJointGoalPos, TuneCalVel, byOptMask);

    m_robot.m_pKin->getJointCurPosVel(strJointName, fLightPos, fJointCurVel);

    LOGDIAG2("Joint %s (servo=%d): Stopped at first light at %.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(fLightPos));
  }

  // optical bits
  byOptBits = m_robot.m_monitor.getJointLimitBits();

  // sanity check
  if( getLitOpticalLimits(byOptBits, byOptMask) == 0x00 )
  {
    LOGERROR("Joint %s (servo=%d): Cannot find occlusion band edge.",
          strJointName.c_str(), nServoId);
    return -HEK_ECODE_NO_EXEC;
  }

  // move to dark
  nDir = -nDir;

  // current position
  m_robot.m_pKin->getJointCurPosVel(strJointName, fJointCurPos, fJointCurVel);

  fJointGoalPos = fJointCurPos + (double)nDir * TuneFudge;

  LOGDIAG2("Joint %s (servo=%d): "
      "Fine tune move for <= %.2lf degrees to find dark edge.",
        strJointName.c_str(), nServoId, radToDeg(fJointGoalPos));

  moveToDark(strJointName, fJointGoalPos, TuneCalVel, byOptMask);

  m_robot.m_pKin->getJointCurPosVel(strJointName, fDarkPos, fJointCurVel);

  LOGDIAG2("Joint %s (servo=%d): Stopped at first dark at %.2lf degrees.",
        strJointName.c_str(), nServoId, radToDeg(fDarkPos));

  // optical bits
  byOptBits = m_robot.m_monitor.getJointLimitBits();

  //
  // Didn't find the dark past the edge.
  //
  if( getDarkOpticalLimits(byOptBits, byOptMask) == 0x00 )
  {
    if( m_robot.m_pKin->hasOverTorqueCondition(strJointName) )
    {
      LOGERROR("Joint %s (servo=%d): Cannot calibrate: Joint is obstucted.",
          strJointName.c_str(), nServoId);
      rc = -HEK_ECODE_COLLISION;
    }
    else
    {
      LOGERROR("Joint %s (servo=%d): Cannot calibrate: "
               "Did not find the dark edge of occlusion band.",
          strJointName.c_str(), nServoId);
      rc = -HEK_ECODE_NO_EXEC;
    }
  }

  //
  // Set delta joint position as specified in spec. 
  //
  fEdgePos = (fLightPos + fDarkPos) / 2.0;
  if( nDir > 0 )
  {
    fDeltaPos = pLimit->m_fMaxEdgePos + fDarkPos - fEdgePos;
  }
  else
  {
    fDeltaPos = pLimit->m_fMinEdgePos + fEdgePos - fDarkPos;
  }

  return rc;
}

double HekCalibStretch::calcJointRatios(const string &strJointName1,
                                        const string &strJointName2)
{
  HekRobotJoint  *pJoint;       // working joint description
  double          fGearRatios;  // gear ratio 1 to gear ratio 2

  if( (pJoint = m_robot.getArmJoint(strJointName1)) != NULL )
  {
    fGearRatios = pJoint->m_fGearRatio;
    if( (pJoint = m_robot.getArmJoint(strJointName2)) != NULL )
    {
      fGearRatios /= pJoint->m_fGearRatio;
    }
    else
    {
      fGearRatios = 0.0;
    }
  }
  else
  {
    fGearRatios = 0.0;
  }

  return fGearRatios;
}

int HekCalibStretch::multiMoveToLight(const string &strJointName,
                                      int           nDir,
                                      double        fJointGoalPos,
                                      double        fJointGoalVel,
                                      byte_t        byMask)
{
  string  strJointName2;    // helper joint name
  double  fGearRatios;      // joint 2 gear ratio to joint 1 gear ratio
  double  fJointGoalPos2;   // helper joint goal position (radians)
  double  fJointGoalVel2;   // helper joint goal velocity (radians/second)

  // start helper joint moving is good direction
  if( strJointName == "shoulder" )
  {
    strJointName2 = "elbow";
    fGearRatios   = calcJointRatios(strJointName2, strJointName);
    if( nDir > 0 )
    {
      fJointGoalPos2 = m_robot.getArmJoint(strJointName2)->m_fMinSoftLimitRads;
    }
    else
    {
      fJointGoalPos2 = m_robot.getArmJoint(strJointName2)->m_fMaxSoftLimitRads;
    }
    fJointGoalVel2 = fGearRatios * fJointGoalVel;
    m_robot.m_pKin->move(strJointName2, fJointGoalPos2, fJointGoalVel2);
  }

  // block while joint moves to light
  moveToLight(strJointName, fJointGoalPos, fJointGoalVel, byMask);

  // stop helper joint
  if( strJointName2.size() > 0 )
  {
    m_robot.m_pKin->stop(strJointName2);
  }

  return HEK_OK;
}

int HekCalibStretch::multiMoveToDark(const string &strJointName,
                                     int           nDir,
                                     double        fJointGoalPos,
                                     double        fJointGoalVel,
                                     byte_t        byMask)
{
  string  strJointName2;    // helper joint name
  double  fGearRatios;      // joint 2 gear ratio to joint 1 gear ratio
  double  fJointGoalPos2;   // helper joint goal position (radians)
  double  fJointGoalVel2;   // helper joint goal velocity (radians/second)

  // start helper joint moving is good direction
  if( strJointName == "shoulder" )
  {
    strJointName2 = "elbow";
    fGearRatios   = calcJointRatios(strJointName2, strJointName);
    if( nDir > 0 )
    {
      fJointGoalPos2 = m_robot.getArmJoint(strJointName2)->m_fMinSoftLimitRads;
    }
    else
    {
      fJointGoalPos2 = m_robot.getArmJoint(strJointName2)->m_fMaxSoftLimitRads;
    }
    fJointGoalVel2 = fGearRatios * fJointGoalVel;
    m_robot.m_pKin->move(strJointName2, fJointGoalPos2, fJointGoalVel2);
  }

  // block while joint moves to light
  moveToDark(strJointName, fJointGoalPos, fJointGoalVel, byMask);

  // stop helper joint
  if( strJointName2.size() > 0 )
  {
    m_robot.m_pKin->stop(strJointName2);
  }

  return HEK_OK;
}

HekRobotJoint *HekCalibStretch::getHelpfulParentJoint(int nChildServoId)
{
  switch( nChildServoId )
  {
    case HekServoIdWristPitch:
      return &(m_robot.m_jointsArm[HekServoIdElbow]);
    case HekServoIdElbow:
      return &(m_robot.m_jointsArm[HekServoIdShoulderL]);
    default:
      return NULL;
  }
}
