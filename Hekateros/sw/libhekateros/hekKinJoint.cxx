////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekKinJoint.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-02-10 13:39:03 -0700 (Tue, 10 Feb 2015) $
 * $Rev: 3866 $
 *
 * \brief The Hekateros powered joint kinematics and dynamics class
 * implemenation.
 *
 * \copyright
 *   \h_copy 2014-2018. RoadNarrows LLC.\n
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

#include <sys/time.h>
#include <time.h>
#include <limits.h>
#include <unistd.h>
#include <stdio.h>
#include <pthread.h>

#include <vector>
#include <deque>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaError.h"
#include "Dynamixel/DynaTypes.h"
#include "Dynamixel/DynaPid.h"
#include "Dynamixel/DynaServo.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekTune.h"
#include "Hekateros/hekSpec.h"
#include "Hekateros/hekJoint.h"
#include "Hekateros/hekPid.h"
#include "Hekateros/hekKinJoint.h"
#include "Hekateros/hekUtils.h"

using namespace std;
using namespace hekateros;

// ---------------------------------------------------------------------------
// SyncMoveMsgs Class
// ---------------------------------------------------------------------------

SyncMoveMsgs::SyncMoveMsgs()
{
  for(int i=0; i<DYNA_ID_NUMOF; ++i)
  {
    m_tupSpeed[i].m_nServoId  = DYNA_ID_NONE;
    m_tupSpeed[i].m_nSpeed    = 0;
    m_tupOdPos[i].m_nServoId  = DYNA_ID_NONE;
    m_tupOdPos[i].m_nPos      = 0;
  }

  m_uSpeedTupCount = 0;
  m_uOdPosTupCount = 0;
}

void SyncMoveMsgs::clear()
{
  m_tupSpeed[0].m_nServoId  = DYNA_ID_NONE;
  m_tupSpeed[0].m_nSpeed    = 0;
  m_uSpeedTupCount          = 0;

  m_tupOdPos[0].m_nServoId  = DYNA_ID_NONE;
  m_tupOdPos[0].m_nPos      = 0;
  m_uOdPosTupCount          = 0;
}

int SyncMoveMsgs::addSpeed(int nServoId, int nServoSpeed)
{
  int   index;

  if( m_uSpeedTupCount < DYNA_ID_NONE )
  {
    index = (int)m_uSpeedTupCount++;
    m_tupSpeed[index].m_nServoId  = nServoId;
    m_tupSpeed[index].m_nSpeed    = nServoSpeed;
  }
  else
  {
    index = -1;
  }

  return index;
}

int SyncMoveMsgs::addOdPos(int nServoId, int nServoPos)
{
  int   index;

  if( m_uOdPosTupCount < DYNA_ID_NONE )
  {
    index = (int)m_uOdPosTupCount++;
    m_tupOdPos[index].m_nServoId  = nServoId;
    m_tupOdPos[index].m_nPos      = nServoPos;
  }
  else
  {
    index = -1;
  }

  return index;
}


// ---------------------------------------------------------------------------
// VelSpeedTuple Class
// ---------------------------------------------------------------------------

VelSpeedTuple::VelSpeedTuple()
{
  m_fJointVel   = 0.0;
  m_nServoSpeed = 0;
}

VelSpeedTuple::VelSpeedTuple(const double fJointVel, const int nServoSpeed)
{
  m_fJointVel   = fJointVel;
  m_nServoSpeed = nServoSpeed;
}

VelSpeedTuple::VelSpeedTuple(const VelSpeedTuple &src)
{
  m_fJointVel   = src.m_fJointVel;
  m_nServoSpeed = src.m_nServoSpeed;
}

VelSpeedTuple VelSpeedTuple::operator=(const VelSpeedTuple &rhs)
{
  m_fJointVel   = rhs.m_fJointVel;
  m_nServoSpeed = rhs.m_nServoSpeed;

  return *this;
}


// ---------------------------------------------------------------------------
// VelSpeedLookupTbl Class
// ---------------------------------------------------------------------------

VelSpeedLookupTbl::VelSpeedLookupTbl()
{
  m_fBucketSize = 1.0;
  m_nMaxIndex   = 0;
  m_nNumEntries = 0;
  m_bDbg        = false;
}

VelSpeedLookupTbl VelSpeedLookupTbl::operator=(const VelSpeedLookupTbl &rhs)
{
  m_fBucketSize = rhs.m_fBucketSize;
  m_nMaxIndex   = rhs.m_nMaxIndex;
  m_nNumEntries = rhs.m_nNumEntries;
  m_tbl         = rhs.m_tbl;
  m_strDbgId    = rhs.m_strDbgId;
  m_bDbg        = rhs.m_bDbg;
}

void VelSpeedLookupTbl::create(double fJointMaxVel, double fBucketSize)
{
  size_t        nElems;
  VelSpeedTuple noentry(0.0, NO_ENTRY);

  fJointMaxVel  = fabs(fJointMaxVel);
  m_fBucketSize = fabs(fBucketSize);

  nElems = (int)(fJointMaxVel / m_fBucketSize);
  if( (double)nElems * m_fBucketSize < fJointMaxVel )
  {
    ++nElems; // round up
  }
  ++nElems; // include zero entry

  m_tbl.insert(m_tbl.begin(), nElems, noentry);

  m_nMaxIndex = (int)m_tbl.size() - 1;

  m_tbl[0]            = VelSpeedTuple(0.0, 0);
  m_tbl[m_nMaxIndex]  = VelSpeedTuple(fJointMaxVel, DYNA_SPEED_MAX_RAW);

  m_nNumEntries = 2;

  if( m_bDbg )
  {
    fprintf(stderr, "DBG: %s(): %s: tbl_size=%zu, num_entries=%d\n",
      LOGFUNCNAME,
      m_strDbgId.c_str(), m_tbl.size(), m_nNumEntries);
  }
}
 
int VelSpeedLookupTbl::hash(double fJointVel)
{
  int index = (int)(fabs(fJointVel)/m_fBucketSize);
  return cap(index, 0, m_nMaxIndex);
}

void VelSpeedLookupTbl::update(double fJointVel, int nServoSpeed)
{
  int index = hash(fJointVel);

  if( (index > 0) && (index < m_nMaxIndex) )
  {
    int nOldCnt = m_nNumEntries;

    if( m_tbl[index].m_nServoSpeed == NO_ENTRY )
    {
      ++m_nNumEntries;
    }

    m_tbl[index].m_fJointVel    = fabs(fJointVel);
    m_tbl[index].m_nServoSpeed  = iabs(nServoSpeed);
    if( m_tbl[index].m_nServoSpeed < MIN_NON_ZERO_SPEED )
    {
      m_tbl[index].m_nServoSpeed = MIN_NON_ZERO_SPEED;
    }

    if( m_bDbg && (m_nNumEntries != nOldCnt) )
    {
      fprintf(stderr, "DBG: %s(): %s: "
                  "num_entries=%d, index=%d, vel=%lf, speed=%d\n",
        LOGFUNCNAME,
        m_strDbgId.c_str(), m_nNumEntries, index,
        radToDeg(m_tbl[index].m_fJointVel), m_tbl[index].m_nServoSpeed);
    }
  }
}

int VelSpeedLookupTbl::estimate(double fJointGoalVel)
{
  double  fJointAbsVel = fabs(fJointGoalVel);

  // zero is 0
  if( fJointAbsVel == 0.0 )
  {
    return m_tbl[0].m_nServoSpeed;
  }
  // maximum velocity
  else if( fJointAbsVel >= m_tbl[m_nMaxIndex].m_fJointVel )
  {
    return m_tbl[m_nMaxIndex].m_nServoSpeed;
  }

  double  fMinVel;
  int     nMinSpeed;
  double  fMaxVel;
  int     nMaxSpeed;

  int     index = hash(fJointAbsVel);

  // no entry at hashed location
  if( m_tbl[index].m_nServoSpeed == NO_ENTRY )
  {
    nMinSpeed = NO_ENTRY;
    nMaxSpeed = NO_ENTRY;
  }
  // go lucky - no interpolation needed
  else if( m_tbl[index].m_fJointVel == fJointAbsVel )
  {
    return m_tbl[index].m_nServoSpeed;
  }
  // hashed location is minimum neighbor
  else if( m_tbl[index].m_fJointVel < fJointAbsVel )
  {
    fMinVel   = m_tbl[index].m_fJointVel;
    nMinSpeed = m_tbl[index].m_nServoSpeed;
    nMaxSpeed = NO_ENTRY;
  }
  // hashed location is maximum neighbor
  else
  {
    nMinSpeed = NO_ENTRY;
    fMaxVel   = m_tbl[index].m_fJointVel;
    nMaxSpeed = m_tbl[index].m_nServoSpeed;
  }

  // no minimum neighbor - search downward
  if( nMinSpeed == NO_ENTRY )
  {
    for(int i = index-1; i >= 0; --i)
    {
      if( m_tbl[i].m_nServoSpeed == NO_ENTRY )
      {
        continue;
      }
      else if( m_tbl[i].m_fJointVel < fJointAbsVel )
      {
        fMinVel   = m_tbl[i].m_fJointVel;
        nMinSpeed = m_tbl[i].m_nServoSpeed;
        break;
      }
    }
  }
  if( nMinSpeed == NO_ENTRY )
  {
    return m_tbl[0].m_nServoSpeed;
  }

  // no maximum neighbor - search upward
  if( nMaxSpeed == NO_ENTRY )
  {
    for(int i = index+1; i <= m_nMaxIndex; ++i)
    {
      if( m_tbl[i].m_nServoSpeed == NO_ENTRY )
      {
        continue;
      }
      else if( m_tbl[i].m_fJointVel > fJointAbsVel )
      {
        fMaxVel   = m_tbl[i].m_fJointVel;
        nMaxSpeed = m_tbl[i].m_nServoSpeed;
        break;
      }
    }
  }
  if( nMaxSpeed == NO_ENTRY )
  {
    return m_tbl[m_nMaxIndex].m_nServoSpeed;
  }

  // interpolate
  return (int)((double)nMinSpeed + fabs(nMaxSpeed - nMinSpeed) * 
                      (fJointAbsVel - fMinVel) / (fMaxVel - fMinVel));
}

void VelSpeedLookupTbl::enableDebugging(const std::string &strId)
{
  m_strDbgId = strId;
  m_bDbg = true;
}

void VelSpeedLookupTbl::disableDebugging()
{
  m_bDbg = false;
}

void VelSpeedLookupTbl::dumpTable(const std::string &strId)
{
  fprintf(stderr, "DBG: %s: bucket_size=%.2lf deg, lookup table[%zu] =\n{\n",
      strId.c_str(), radToDeg(m_fBucketSize), m_tbl.size());

  for(int i=0; i<m_tbl.size(); ++i)
  {
    if( m_tbl[i].m_nServoSpeed != NO_ENTRY )
    {
      fprintf(stderr, "  [%3d]: vel=%7.3lf (%6.2lf deg), speed=%4d\n",
        i, m_tbl[i].m_fJointVel, radToDeg(m_tbl[i].m_fJointVel),
        m_tbl[i].m_nServoSpeed);
    }
  }
  fprintf(stderr, "}\n");
}

// ---------------------------------------------------------------------------
// HekKinJoint Class
// ---------------------------------------------------------------------------

const double HekKinJoint::SLOW_DERATE_DELTA_V = 1.0;

HekKinJoint::HekKinJoint() :
    m_pJoint(NULL), m_pServo(NULL),
    m_pid(HekTunePidKpDft, HekTunePidKiDft, HekTunePidKdDft),
    m_histPosIn(POS_WIN_SIZE, 0.0),
    m_histDtIn(DT_WIN_SIZE, 0.0),
    m_histVelIn(VEL_WIN_SIZE, 0.0),
    m_histTorqueIn(TORQUE_WIN_SIZE, 0.0)
{
  m_eState    = MoveStateIdle;

  pthread_mutex_init(&m_mutexSync, NULL);

  m_fTolPos     = degToRad(HekTuneTolPosDft);
  m_fTolVel     = degToRad(HekTuneTolVelDft);
  m_fStopVelTh  = SLOW_DERATE_DELTA_V * degToRad(HekTunePidMaxDeltaVDft);

  m_fJointRadPerTick  = 1.0;
  m_fJointGoalPos     = 0.0;
  m_nServoGoalPos     = 0;
  m_fJointCurPos      = 0.0;
  m_nServoCurPos      = 0;
  m_fJointPrevPos     = 0.0;
  m_nServoPrevPos     = 0;
  m_fJointPosOut      = 0.0;

  m_fDt     = 0.0;
  clock_gettime(CLOCK_REALTIME, &m_tsPrev);
  m_fDtAvg  = 0.0;

  m_fJointGoalVel   = 0.0;
  m_fJointCurVel    = 0.0;
  m_nServoCurSpeed  = 0;
  m_fVelDerate      = HekTuneVelDerateDft / 100.0;
  m_fJointVelOut    = 0.0;

  m_fJointTgtVel    = 0.0;
  m_nServoTgtSpeed  = 0;

  m_bOverTorqueCond = false;
  m_fOverTorqueTh   = 0.0;
  m_fClearTorqueTh  = 0.0;
  m_fTorqueOut      = 0.0;
}

HekKinJoint::HekKinJoint(HekRobotJoint  *pJoint,
                         DynaServo      *pServo,
                         const HekTunes &tunes) :
    m_pJoint(pJoint), m_pServo(pServo),
    m_histPosIn(POS_WIN_SIZE, 0.0),
    m_histDtIn(DT_WIN_SIZE, 0.0),
    m_histVelIn(VEL_WIN_SIZE, 0.0),
    m_histTorqueIn(TORQUE_WIN_SIZE, 0.0)
{
  double    fKp, fKi, fKd;
  double    fMaxDeltaV;
  double    fRawMaxTorque;

  m_eState    = MoveStateIdle;

  pthread_mutex_init(&m_mutexSync, NULL);

  // joint tolerance tuning parameters (normalized)
  tunes.getToleranceParams(pJoint->m_strName, m_fTolPos, m_fTolVel);

  // joint position and velocity PID tuning parameters
  tunes.getPidKParams(pJoint->m_strName, fKp, fKi, fKd);

  // pid constants
  setPidKParams(fKp, fKi, fKd);

  // joint pid maximum delta velocity tuning paramter
  fMaxDeltaV = tunes.getPidMaxDeltaV(m_pJoint->m_strName);

  // maximum PID output delta v (radians/second)
  setPidMaxDeltaVParam(fMaxDeltaV);

  // stop velocity maximum velocity
  m_fStopVelTh = SLOW_DERATE_DELTA_V * fMaxDeltaV;

  // position state
  m_fJointRadPerTick  = 1.0 / m_pJoint->m_fTicksPerJointRad;
  m_fJointGoalPos     = 0.0;
  m_nServoGoalPos     = 0;
  m_fJointCurPos      = 0.0;
  m_nServoCurPos      = 0;
  m_fJointPrevPos     = 0.0;
  m_nServoPrevPos     = 0;
  m_fJointPosOut      = 0.0;

  // clock
  m_fDt     = 0.0;
  clock_gettime(CLOCK_REALTIME, &m_tsPrev);
  m_fDtAvg  = 0.0;

  // velocity state
  m_fJointGoalVel   = 0.0;
  m_fJointCurVel    = 0.0;
  m_nServoCurSpeed  = 0;
  m_fVelDerate      = tunes.getVelocityDerate();  // tuning parameter (norm)
  m_fJointVelOut    = 0.0;

  m_fJointTgtVel    = 0.0;
  m_nServoTgtSpeed  = 0;

  // To debug any joint(s).
  //if( m_pJoint->m_strName == "base_rot")
  //{
  //  m_tblVelSpeed.enableDebugging(m_pJoint->m_strName);
  //}

  m_tblVelSpeed.create(m_pJoint->m_fMaxJointRadsPerSec, degToRad(5.0));

  // normalized joint torque tuning parameters
  tunes.getTorqueParams(pJoint->m_strName, m_fOverTorqueTh, m_fClearTorqueTh);

  // convert to raw theshold values 
  m_bOverTorqueCond = false;
  fRawMaxTorque     = (double)pServo->GetSpecification().m_uRawTorqueMax;
  m_fOverTorqueTh  *= fRawMaxTorque;
  m_fClearTorqueTh *= fRawMaxTorque;
  m_fTorqueOut      = 0.0;
}

HekKinJoint &HekKinJoint::operator=(const HekKinJoint &rhs)
{
  m_eState    = rhs.m_eState;
  m_pJoint    = rhs.m_pJoint;
  m_pServo    = rhs.m_pServo;

  m_fTolPos     = rhs.m_fTolPos;
  m_fTolVel     = rhs.m_fTolVel;
  m_fStopVelTh  = rhs.m_fStopVelTh;
  m_pid         = rhs.m_pid;

  m_fJointRadPerTick  = rhs.m_fJointRadPerTick;
  m_fJointGoalPos     = rhs.m_fJointGoalPos;
  m_nServoGoalPos     = rhs.m_nServoGoalPos;
  m_fJointCurPos      = rhs.m_fJointCurPos;
  m_nServoCurPos      = rhs.m_nServoCurPos;
  m_fJointPrevPos     = rhs.m_fJointPrevPos;
  m_nServoPrevPos     = rhs.m_nServoPrevPos;
  m_fJointPosOut      = rhs.m_fJointPosOut;
  m_histPosIn         = rhs.m_histPosIn;

  m_fDt       = rhs.m_fDt;
  m_tsPrev    = rhs.m_tsPrev;
  m_fDtAvg    = rhs.m_fDtAvg;
  m_histDtIn  = rhs.m_histDtIn;

  m_fJointGoalVel   = rhs.m_fJointGoalVel;
  m_fJointCurVel    = rhs.m_fJointCurVel;
  m_nServoCurSpeed  = rhs.m_nServoCurSpeed;
  m_tblVelSpeed     = rhs.m_tblVelSpeed;
  m_fJointVelOut    = rhs.m_fJointVelOut;
  m_histVelIn       = rhs.m_histVelIn;

  m_fJointTgtVel    = rhs.m_fJointTgtVel;
  m_nServoTgtSpeed  = rhs.m_nServoTgtSpeed;

  m_bOverTorqueCond = rhs.m_bOverTorqueCond;
  m_fOverTorqueTh   = rhs.m_fOverTorqueTh;
  m_fClearTorqueTh  = rhs.m_fClearTorqueTh;
  m_fTorqueOut      = rhs.m_fTorqueOut;
  m_histTorqueIn    = rhs.m_histTorqueIn;

  return *this;
}

void HekKinJoint::reload(const HekTunes &tunes)
{
  double    fKp, fKi, fKd;
  double    fMaxDeltaV;
  double    fRawMaxTorque;

  lock();

  // joint tolerance tuning parameters (normalized)
  tunes.getToleranceParams(m_pJoint->m_strName, m_fTolPos, m_fTolVel);

  // joint position and velocity PID tuning parameters
  tunes.getPidKParams(m_pJoint->m_strName, fKp, fKi, fKd);

  // set classic pid parameters
  setPidKParams(fKp, fKi, fKd);

  // joint pid maximum delta velocity tuning paramter
  fMaxDeltaV = tunes.getPidMaxDeltaV(m_pJoint->m_strName);

  // maximum PID output delta v (radians/second)
  setPidMaxDeltaVParam(fMaxDeltaV);

  // stop velocity maximum velocity
  m_fStopVelTh = SLOW_DERATE_DELTA_V * fMaxDeltaV;

  // velocity derate
  m_fVelDerate      = tunes.getVelocityDerate();

  // normalized joint torque tuning parameters
  tunes.getTorqueParams(m_pJoint->m_strName, m_fOverTorqueTh, m_fClearTorqueTh);

  // convert to raw theshold values 
  fRawMaxTorque     = (double)m_pServo->GetSpecification().m_uRawTorqueMax;
  m_fOverTorqueTh  *= fRawMaxTorque;
  m_fClearTorqueTh *= fRawMaxTorque;

  unlock();
}

int HekKinJoint::resetServoOdometer()
{
  uint_t  uEncZeroPt;   // encoder position mapping to new odometer zero pt
  bool    bIsReverse;   // odometer is [not] reverse to that of encoder
  int     rc;           // return code

  if( m_pServo == NULL )
  {
    return HEK_OK;
  }

  lock();

  // must be stopped
  nlstop();

  // read encoder position (note: should add libDynamixel function)
  if( (rc = m_pServo->Read(DYNA_ADDR_CUR_POS_LSB, &uEncZeroPt)) != DYNA_OK )
  {
    LOGERROR("Joint %s (servo=%d): Cannot read current encoder position.",
          m_pJoint->m_strName.c_str(), m_pJoint->m_nMasterServoId);
    rc = -HEK_ECODE_DYNA;
  }

  else
  {
    bIsReverse = m_pJoint->m_nMasterServoDir == DYNA_DIR_CCW? false: true;

    // reset soft odometer
    m_pServo->ResetOdometer((int)uEncZeroPt, bIsReverse);

    LOGDIAG3("Joint %s (servo=%d): Odometer reset at encoder=%u, reverse=%s.",
          m_pJoint->m_strName.c_str(), m_pJoint->m_nMasterServoId,
          uEncZeroPt, boolstr(bIsReverse));

    m_fJointGoalPos = 0.0;
    m_nServoGoalPos = 0;
    m_fJointCurPos  = 0.0;
    m_nServoCurPos  = 0;
    m_fJointPrevPos = 0.0;
    m_nServoPrevPos = 0;

    m_fJointGoalVel   = 0.0;
    m_fJointCurVel    = 0.0;
    m_nServoCurSpeed  = 0;

    m_fJointTgtVel    = 0.0;
    m_nServoTgtSpeed  = 0;

    rc = HEK_OK;
  }

  unlock();

  return rc;
}

int HekKinJoint::specifyMove(const double fGoalPos, const double fGoalVel)
{
  int   rc;

  lock();

  rc = nlspecifyMove(fGoalPos, fGoalVel);

  unlock();

  return rc;
}

int HekKinJoint::nlspecifyMove(const double fGoalPos, const double fGoalVel)
{
  // new goal position
  m_fJointGoalPos = fGoalPos;

  //
  // Check for goal position out of range condition and cap as necessary.
  //
  // Note:  Only check if the joint has been calibrated. If calibrated, assume
  //        a calibration routine is calling the function and "it knows what
  //        it is doing".
  //
  if( m_pJoint->m_eOpState == HekOpStateCalibrated )
  {
    if( m_pJoint->m_eJointType != HekJointTypeContinuous )
    {
      if( m_fJointGoalPos < m_pJoint->m_fMinSoftLimitRads )
      {
        LOGDIAG3("Joint %s (servo=%d): "
              "Goal position %.02lf < %.02lf degree limit - capping.",
          m_pJoint->m_strName.c_str(), m_pJoint->m_nMasterServoId,
          radToDeg(m_fJointGoalPos), radToDeg(m_pJoint->m_fMinSoftLimitRads));

        m_fJointGoalPos = m_pJoint->m_fMinSoftLimitRads;
      }
      else if( m_fJointGoalPos > m_pJoint->m_fMaxSoftLimitRads )
      {
        LOGDIAG3("Joint %s (servo=%d): "
              "Goal position %.02lf > %.02lf degree limit - capping.",
          m_pJoint->m_strName.c_str(), m_pJoint->m_nMasterServoId,
          radToDeg(m_fJointGoalPos), radToDeg(m_pJoint->m_fMaxSoftLimitRads));

        m_fJointGoalPos = m_pJoint->m_fMaxSoftLimitRads;
      }
    }
  }

  // joint position to odometer position
  m_nServoGoalPos = jointPosToServoPos(m_fJointGoalPos);

  //
  // Joint current and goal positions dictate sign of velocity. 
  //
  m_fJointGoalVel = fabs(fGoalVel);
  if( m_fJointCurPos > m_fJointGoalPos )
  {
    m_fJointGoalVel = -m_fJointGoalVel;
  }

  // derate
  m_fJointGoalVel *= m_fVelDerate;

  // new move
  //if( fabs(m_fJointGoalPos-m_fJointCurPos) > m_fTolPos )
  //{
    m_pid.specifySetPoint(m_fJointGoalPos, m_fJointGoalVel);
    m_eState = MoveStateNewMove;
  //}

  return HEK_OK;
}

int HekKinJoint::getGoalDir()
{
  int nSign = m_pServo->IsOdometerReversed()? -1: 1;
  
  return (m_nServoGoalPos-m_pServo->GetOdometer()) >= 0? nSign: -1 * nSign;
}
 
int HekKinJoint::getPlannedDir(int nServoTgtSpeed)
{
  return nServoTgtSpeed >= 0? 1: -1;
}

bool HekKinJoint::canMoveInGoalDir()
{
  return !m_bOverTorqueCond || (getGoalDir() == DYNA_GET_DIR(m_nServoCurLoad));
}

bool HekKinJoint::canMoveInPlannedDir(int nServoTgtSpeed)
{
  return !m_bOverTorqueCond ||
          (getPlannedDir(nServoTgtSpeed) == DYNA_GET_DIR(m_nServoCurLoad));
}

int HekKinJoint::senseDynamics(bool bIsControlling)
{
  int   nServoDejitPos;
  int   rc;

  if( m_pServo == NULL )
  {
    return HEK_OK;
  }

  lock();

  // save current servo position as the new previous position
  m_nServoPrevPos = m_nServoCurPos;

  // Read the servo dynamics.
  rc = m_pServo->ReadDynamics(&m_nServoCurPos,
                              &m_nServoCurSpeed,
                              &m_nServoCurLoad);

  // sanity check
  if( (m_pJoint->m_eOpState == HekOpStateCalibrated) &&
    (iabs(m_nServoCurPos-m_nServoPrevPos) >
                                        m_pJoint->m_fTicksPerServoRad * M_PI) )
  {
    LOGWARN("Large jump in servo position: prev_odpos=%d, cur_odpos=%d.",
            m_nServoPrevPos, m_nServoCurPos);
  }

  //
  // Transform.
  //
  if( rc == DYNA_OK )
  {
    // delta time
    m_fDt = delta_t();

    // running average of delta times (not used for now)
    //m_fDtAvg = averageDt(m_fDtAvg, m_fDt);

    // de-jitter servo odometer (encoder) position
    nServoDejitPos = dejitterServoPos(m_nServoCurPos, m_nServoPrevPos);

    // current joint position (radians)
    m_fJointPrevPos = m_fJointCurPos;
    m_fJointCurPos  = servoPosToJointPos(nServoDejitPos);

    // low-pass band filter current positions
    m_fJointPosOut = filterPositions(m_fJointPosOut, m_fJointCurPos);

    // current joint velocity (radians/second)
    m_fJointCurVel = jointVelocity(m_fJointCurPos, m_fJointPrevPos, m_fDt);
    
    // update joint instantaneous velocity-servo speed lookup table
    if( bIsControlling )
    {
      updateAssoc(m_fJointCurVel, m_nServoCurSpeed);
    }

    // low-pass band filter current velocities
    m_fJointVelOut = filterVelocities(m_fJointVelOut, m_fJointCurVel);

    // low-pass band filter torque (raw unitless load)
    m_fTorqueOut = filterTorques(m_fTorqueOut, m_nServoCurLoad);
    
    // set over torque limit condition
    if( fabs(m_fTorqueOut) > m_fOverTorqueTh )
    {
      if( !m_bOverTorqueCond && bIsControlling )
      {
        LOGWARN("Joint %s (servo=%d): Entered over-torque condition. "
                "Load=%lf.",
            m_pJoint->m_strName.c_str(), m_pServo->GetServoId(), m_fTorqueOut);
      }
      m_bOverTorqueCond = true;
    }

    // clear over torque limit condition
    else if( fabs(m_fTorqueOut) < m_fClearTorqueTh )
    {
      if( m_bOverTorqueCond && bIsControlling )
      {
        LOGDIAG2("Joint %s (servo=%d): Cleared over-torque condition.",
            m_pJoint->m_strName.c_str(), m_pServo->GetServoId());
      }
      m_bOverTorqueCond = false;
    }
  }

  unlock();

  return rc == DYNA_OK? HEK_OK: -HEK_ECODE_DYNA;
}

int HekKinJoint::react(bool bIsControlling,  SyncMoveMsgs &msgs)
{
  if( !bIsControlling )
  {
    return HEK_OK;
  }

  lock();

  //
  // Joint is slowing to a stop.
  //
  if( isStopping() )
  {
    nlslowToStop(msgs);
  }

  //
  // Joint has moved close enough to goal position, and is slow enough to stop.
  // Otherwise let the PID do its thing.
  //
  else if( isMovingToGoal() &&
          (fabs(m_fJointGoalPos-m_fJointCurPos) <= m_fTolPos) &&
          canStopNow() )
  {
    nlstop();
  }

  //
  // In over torque condition, apply torque control.
  //
  if( m_bOverTorqueCond )
  {
    nlapplyTorqueControl();
  }

  unlock();

  return HEK_OK;
}

HekKinJoint::MoveState HekKinJoint::planMotion(bool bIsControlling,
                                               SyncMoveMsgs &msgs)
{
  if( m_pServo == NULL )
  {
    return MoveStateIdle;
  }
  else if( !bIsControlling )
  {
    return m_eState;
  }

  lock();

  if( isMovingToGoal() )
  {
    // PID target velocity and corresponding target servo speed estimate
    m_fJointTgtVel    = m_pid.control(m_fJointCurPos, m_fJointCurVel, m_fDt);
    m_nServoTgtSpeed  = estimateServoTgtSpeed(m_fJointTgtVel);

    // still away from goal position
    if( (fabs(m_fJointGoalPos-m_fJointCurPos) > m_fTolPos) &&
        canMoveInPlannedDir(m_nServoTgtSpeed) )
    {
      // new move - make it happen
      if( m_eState == MoveStateNewMove )
      {
        addServoMsgEntries(msgs, m_nServoTgtSpeed, m_nServoGoalPos);
      }

      // subsequent changes of servo speed - only if sufficently different
      else if( fabs(m_fJointTgtVel-m_fJointCurVel) > m_fTolVel )
      {
        addServoMsgEntries(msgs, m_nServoTgtSpeed, m_nServoGoalPos);
      }
    }
  }

  unlock();

  return m_eState;
}

int HekKinJoint::act()
{
  lock();

  //
  // Actively moving joint and there is no over torque condition or the goal
  // direction will naturally ease the torque.
  //
  if( isMovingToGoal() && canMoveInGoalDir() )
  {
    nlmove();
  }

  //
  // In over torque condition, apply torque control.
  //
  else if( m_bOverTorqueCond )
  {
    nlapplyTorqueControl();
  }

  unlock();

  return HEK_OK;
}

int HekKinJoint::applyTorqueControl()
{
  int   rc;

  if( m_pServo == NULL )
  {
    return HEK_OK;
  }

  lock();

  rc = nlapplyTorqueControl();

  unlock();

  return rc;
}

int HekKinJoint::nlapplyTorqueControl()
{
  static const int        TuneMaxIters  = 5;    // max iterations to ease torque
  static const useconds_t TuneWait      = 100;  // wait between iterations

  int nSign;
  int nOffSpeed;
  int nOffPos;

  if( fabs(m_fTorqueOut) > m_fOverTorqueTh )
  {
    LOGDIAG4("Executing torque control for servo %d.", m_pServo->GetServoId());

    nSign     = DYNA_GET_DIR(m_nServoCurLoad);
    nOffSpeed = TORQUE_CTL_BACKOFF_SPEED;

    switch( m_pServo->GetServoMode() )
    {
      case DYNA_MODE_CONTINUOUS:
        nOffSpeed *= nSign;
        m_pServo->WriteGoalSpeed(nOffSpeed);
        for(int i = 0; i < TuneMaxIters; ++i)
        {
          usleep(TuneWait);
          m_pServo->ReadCurLoad(&m_nServoCurLoad);
          m_fTorqueOut = filterTorques(m_fTorqueOut, m_nServoCurLoad);
          if( fabs(m_fTorqueOut) <= m_fOverTorqueTh )
          {
            break;
          }
        }
        m_pServo->Stop();
        break;

      case DYNA_MODE_SERVO:
      default:
        nOffPos = nSign * TORQUE_CTL_BACKOFF_POS;
        m_pServo->WriteGoalSpeed(nOffSpeed);
        m_pServo->WriteGoalPos(nOffPos);
        break;
    }
  }

  return HEK_OK;
}

HekKinJoint::MoveState HekKinJoint::move()
{
  MoveState eState;

  if( m_pServo == NULL )
  {
    return MoveStateIdle;
  }

  lock();

  eState = nlmove();

  unlock();

  return eState;
}

HekKinJoint::MoveState HekKinJoint::nlmove()
{
  if( isMovingToGoal() )
  {
    // still away from goal position
    if( fabs(m_fJointGoalPos-m_fJointCurPos) > m_fTolPos )
    {
      // PID target velocity and corresponding target servo speed estimate
      m_fJointTgtVel    = m_pid.control(m_fJointCurPos, m_fJointCurVel, m_fDt);
      m_nServoTgtSpeed  = estimateServoTgtSpeed(m_fJointTgtVel);

      // new move - make it happen
      if( m_eState == MoveStateNewMove )
      {
        moveServo(m_nServoTgtSpeed, m_nServoGoalPos);
      }

      // subsequent changes of servo speed - only if sufficently different
      else if( fabs(m_fJointTgtVel-m_fJointCurVel) > m_fTolVel )
      {
        moveServo(m_nServoTgtSpeed, m_nServoGoalPos);
      }
    }

    // at goal position
    else
    {
      nlstop();
    }
  }

  return m_eState;
}

void HekKinJoint::moveServo(int  nServoGoalSpeed, int  nServoGoalPos)
{
  //
  // Annoying Dynamixel subtleties are outlined here.
  //
  // In continuous mode:
  //   When writing to the goal speed, the servo will immediately begin
  //   movement. Servo goal position has no effect. A value of 0 will
  //   stop the servo. The speeds are signed values.
  //
  // In servo mode:
  //    When writing the goal position, if different from the current
  //    position, the servo will immediately move at the goal speed
  //    loaded in its control table. So always write goal speed first,
  //    then goal position. The value of 0 cause the servo to go ape shit,
  //    moving at its maximum speed with no servo control. Avoid this value
  //    like the Ebola. To stop the servo, write the current position as
  //    its goal position. One final nit, no negative speeds.
  //
  if( m_pServo->GetServoMode() == DYNA_MODE_CONTINUOUS )
  {
    nServoGoalSpeed = cap(nServoGoalSpeed, -DYNA_SPEED_MAX_RAW,
                                            DYNA_SPEED_MAX_RAW);
    m_pServo->WriteGoalSpeed(nServoGoalSpeed);
  }
  else
  {
    nServoGoalSpeed = cap(iabs(nServoGoalSpeed), DYNA_SPEED_MIN_CTL,
                                                 DYNA_SPEED_MAX_RAW);
    m_pServo->WriteGoalSpeed(nServoGoalSpeed);
    if( m_eState == MoveStateNewMove )
    {
      m_pServo->WriteGoalPos(nServoGoalPos);
      m_eState = MoveStateMoving;
    }
  }
}

void HekKinJoint::addServoMsgEntries(SyncMoveMsgs &msgs,
                                     int          nServoTgtSpeed,
                                     int          nServoGoalPos)
{
  // 
  // See discussion in moveServo() for Dynamixel nuances.
  //
  if( m_pServo->GetServoMode() == DYNA_MODE_CONTINUOUS )
  {
    nServoTgtSpeed = cap(nServoTgtSpeed, -DYNA_SPEED_MAX_RAW,
                                          DYNA_SPEED_MAX_RAW);
    msgs.addSpeed(m_pServo->GetServoId(), nServoTgtSpeed);
  }
  else
  {
    m_nServoTgtSpeed = cap(iabs(nServoTgtSpeed), DYNA_SPEED_MIN_CTL,
                                                 DYNA_SPEED_MAX_RAW);
    msgs.addSpeed(m_pServo->GetServoId(), nServoTgtSpeed);
    if( m_eState == MoveStateNewMove )
    {
      msgs.addOdPos(m_pServo->GetServoId(), nServoGoalPos);
      m_eState = MoveStateMoving;
    }
  }
}

int HekKinJoint::stop()
{
  int   rc;

  if( m_pServo == NULL )
  {
    return -HEK_ECODE_DYNA;
  }

  lock();

  rc = nlstop();

  unlock();

  return rc;
}

int HekKinJoint::nlstop()
{
#ifdef HEK_KIN_EXEC_ALG_SYNC
  //
  // Test if moving too fast. If true, use a fast ramp down to slow to a
  // stopping speed. The ramp is defined by the maximum delta velocity tune
  // parameter.
  //
  // Note:  During calibration stop means stop.
  //
  if( !canStopNow() )
  {
    //fprintf(stderr, "DBG: %s() MOVING TO FAST TO STOP: "
    //                "curvel=%lf, stopvelth=%lf\n",
    //  LOGFUNCNAME, radToDeg(m_fJointCurVel), radToDeg(m_fStopVelTh));

    m_eState = MoveStateStopping;
    return HEK_OK;
  }
#endif // HEK_KIN_EXEC_ALG_SYNC

  m_pServo->Stop();

  if( m_pServo->ReadCurPos(&m_nServoCurPos) == DYNA_OK )
  {
    m_fJointCurPos = servoPosToJointPos(m_nServoCurPos);
  }

  m_fJointPrevPos   = m_fJointCurPos;
  m_nServoPrevPos   = m_nServoCurPos;

  m_fJointCurVel    = 0.0;
  m_nServoCurSpeed  = 0;

  m_fJointGoalPos   = m_fJointCurPos;
  m_nServoGoalPos   = m_nServoCurPos;
  m_fJointGoalVel   = 0.0;

  m_fJointTgtVel    = 0.0;
  m_nServoTgtSpeed  = 0;

  m_eState = MoveStateIdle;

  //fprintf(stderr, "DBG: STOP: %s\n", m_pJoint->m_strName.c_str());

  return HEK_OK;
}

int HekKinJoint::nlslowToStop(SyncMoveMsgs &msgs)
{
  double fJointStopVel;
  int    nServoStopSpeed;

  if( (fabs(m_fJointCurVel) <= m_fStopVelTh) )
  {
    //fprintf(stderr, "DBG: %s() SLOWED ENOUGH: curvel=%lf\n",
    //  LOGFUNCNAME, radToDeg(m_fJointCurVel));

    return nlstop();
  }
  else
  {
    if( m_fJointCurVel > 0.0 )
    {
      fJointStopVel = m_fJointCurVel - m_fStopVelTh;
    }
    else
    {
      fJointStopVel = m_fJointCurVel + m_fStopVelTh;
    }

    nServoStopSpeed  = estimateServoTgtSpeed(fJointStopVel);

    addServoMsgEntries(msgs, nServoStopSpeed, m_nServoGoalPos);

    //fprintf(stderr, "DBG: %s() SLOWING TO: curvel=%lf, stopvel=%lf\n",
    //  LOGFUNCNAME, radToDeg(m_fJointCurVel), radToDeg(fJointStopVel));

    return HEK_OK;
  }
}

int HekKinJoint::jointPosToServoPos(const double fPos)
{
  return (int)(fPos * m_pJoint->m_fTicksPerJointRad);
}

double HekKinJoint::servoPosToJointPos(const int nOdPos)
{ 
  return (double)nOdPos / m_pJoint->m_fTicksPerJointRad;
}

double HekKinJoint::jointVelocity(double fPos1, double fPos0, double fDt)
{
  return fDt > 0.0? (fPos1 - fPos0) / fDt: 0.0;
}

void HekKinJoint::updateAssoc(double fJointVel, int nServoSpeed)
{
  m_tblVelSpeed.update(fJointVel, nServoSpeed);
}

int HekKinJoint::estimateServoTgtSpeed(double fJointTgtVel)
{
  int nSign = (int)signof(fJointTgtVel);

  if( m_pServo->IsOdometerReversed() )
  {
    nSign = -nSign;
  }

  return nSign * m_tblVelSpeed.estimate(fJointTgtVel);
}

int HekKinJoint::estimateServoTgtSpeedSimple(double fJointTgtVel)
{
  double  r;
  int     nSpeedEst;

  r         = fJointTgtVel / m_pJoint->m_fMaxJointRadsPerSec;
  nSpeedEst = (int)(DYNA_SPEED_MAX_RAW * r);
  nSpeedEst = cap(nSpeedEst, 0, DYNA_SPEED_MAX_RAW) * getGoalDir();

  return nSpeedEst;
}

double HekKinJoint::delta_t()
{
  struct timespec tsNow;
  double          deltaTime;

  // now
  clock_gettime(CLOCK_REALTIME, &tsNow);

  // delta time in seconds
  deltaTime = hekateros::dt(tsNow, m_tsPrev);

  // new previous
  m_tsPrev = tsNow;

  return deltaTime;
}

double HekKinJoint::averageDt(double fDtAvg, double fDt)
{
  double  fDt0;
  double  fDtK;

  fDt0 = fDt / (double)DT_WIN_SIZE;
  fDtK = m_histDtIn.back();
  m_histDtIn.pop_back();
  m_histDtIn.push_front(fDt0);

  return fDtAvg - fDtK + fDt0;
}

int HekKinJoint::dejitterServoPos(int nServoCurPos, int nServoPrevPos)
{
  // Same position over two sense cycles - accept new position.
  if( nServoCurPos == nServoPrevPos )
  {
    return nServoCurPos;
  }

  // Sufficiently large jump in position, probably not jitter.
  else if( hekateros::iabs(nServoCurPos-nServoPrevPos) > 1 )
  {
    return nServoCurPos;
  }

  // Small change is position, treat as sampling and/or random jitter.
  else
  {
    return nServoPrevPos;
  }
}

double HekKinJoint::filterPositions(double fPosAvg, double fJointPos)
{
  double  fPos0;
  double  fPosK;

  fPos0 = fJointPos / (double)POS_WIN_SIZE;
  fPosK = m_histPosIn.back();
  m_histPosIn.pop_back();
  m_histPosIn.push_front(fPos0);

  return fPosAvg - fPosK + fPos0;
}

double HekKinJoint::filterVelocities(double fVelAvg, double fJointVel)
{
  double  fVel0;
  double  fVelK;

  fVel0 = fJointVel / (double)VEL_WIN_SIZE;
  fVelK = m_histVelIn.back();
  m_histVelIn.pop_back();
  m_histVelIn.push_front(fVel0);

  return fVelAvg - fVelK + fVel0;
}

double HekKinJoint::filterTorques(double fTorqueAvg, int nServoLoad)
{
  double  fTorque0;
  double  fTorqueK;

  fTorque0 = (double)nServoLoad / (double)TORQUE_WIN_SIZE;
  fTorqueK = m_histTorqueIn.back();
  m_histTorqueIn.pop_back();
  m_histTorqueIn.push_front(fTorque0);

  return fTorqueAvg - fTorqueK + fTorque0;
}


// ---------------------------------------------------------------------------
// HekKinJointWristRot Class
// ---------------------------------------------------------------------------

int HekKinJointWristRot::react(bool bIsControlling,  SyncMoveMsgs &msgs)
{
  if( !bIsControlling )
  {
    return HEK_OK;
  }

  lock();

  HekKinJoint::MoveState eStatePrev = m_eStateWristPitch;

  m_eStateWristPitch = getCoupledJointState();

  //
  // Wrist rotation is not moving and wrist pitch has transitioned from moving
  // to stop. Respecify wrist rotation goal move to avoid minor wrist rotation
  // drift.
  //
  if( isStopped() )
  {
    if( isCoupledJointStopped() && (eStatePrev != MoveStateIdle) )
    {
      //fprintf(stderr, "DBG: %s() isStopped - respecify move\n", LOGFUNCNAME);
      nlspecifyMove(m_fJointGoalPos, m_fJointGoalVel);
    }
  }

  //
  // Joint is slowing to a stop.
  //
  else if( isStopping() )
  {
    //fprintf(stderr, "DBG: %s() isStopping\n", LOGFUNCNAME);
    nlslowToStop(msgs);
  }

  //
  // Joint has moved close enough to goal position, and is slow enough to stop.
  // Otherwise let the PID do its thing.
  //
  else if( isMovingToGoal() &&
          (fabs(m_fJointGoalPos-m_fJointCurPos) <= m_fTolPos) &&
          canStopNow() )
  {
    nlstop();
    //fprintf(stderr, "DBG: %s() stop\n", LOGFUNCNAME);
  }

  //
  // In over torque condition, apply torque control.
  //
  if( m_bOverTorqueCond )
  {
    nlapplyTorqueControl();
  }

  unlock();

  return HEK_OK;
}

HekKinJoint::MoveState HekKinJointWristRot::planMotion(bool bIsControlling,
                                                       SyncMoveMsgs &msgs)
{
  double  fAdjustVel;
  int     fAdjustSpeed;

  if( m_pServo == NULL )
  {
    return MoveStateIdle;
  }
  else if( !bIsControlling )
  {
    return m_eState;
  }

  lock();

  m_eStateWristPitch = getCoupledJointState();

  if( isCoupledJointMoving() )
  {
    m_pKinJointWristPitch->getTgtVelSpeed(fAdjustVel, fAdjustSpeed);
  }
  else
  {
    fAdjustVel    = 0.0;
    fAdjustSpeed  = 0;
  }

  if( isMovingToGoal() || isCoupledJointMoving() )
  {
    // servo goal od position can change with coupled joints - recalculate
    m_nServoGoalPos = jointPosToServoPos(m_fJointGoalPos);

    if( isMovingToGoal() )
    {
      // PID target velocity 
      m_fJointTgtVel = m_pid.control(m_fJointCurPos, m_fJointCurVel, m_fDt);

      // estimate corresponding target servo speed
      m_nServoTgtSpeed  = estimateServoTgtSpeed(m_fJointTgtVel);
    }
    else
    {
      m_fJointTgtVel    = 0.0;
      m_nServoTgtSpeed  = 0;
    }

    m_nServoTgtSpeed  -= fAdjustSpeed;

    // still away from goal position
    if( ( (fabs(m_fJointGoalPos-m_fJointCurPos) > m_fTolPos) ||
          isCoupledJointMoving() ) &&
        canMoveInPlannedDir(m_nServoTgtSpeed) )
    {
      // new move - make it happen
      if( (m_eState == MoveStateNewMove) ||
          (m_eStateWristPitch == MoveStateNewMove) )
      {
        addServoMsgEntries(msgs, m_nServoTgtSpeed, m_nServoGoalPos);
      }

      // subsequent changes of servo speed - only if sufficently different
      else if( fabs(m_fJointTgtVel-m_fJointCurVel) > m_fTolVel )
      {
        addServoMsgEntries(msgs, m_nServoTgtSpeed, m_nServoGoalPos);
      }
    }
  }

  unlock();

  return m_eState;
}

int HekKinJointWristRot::act()
{
  lock();

  HekKinJoint::MoveState eStatePrev = m_eStateWristPitch;

  m_eStateWristPitch = getCoupledJointState();

  if( isStopped() && isCoupledJointStopped() &&
      (eStatePrev != MoveStateIdle) )
  {
    nlspecifyMove(m_fJointGoalPos, m_fJointGoalVel);
  }

  //
  // Actively moving wrist rotation and/or wrist pitch and there is no over
  // torque condition or the goal direction will naturally ease the torque.
  //
  if( isMovingToGoal() || isCoupledJointMoving() && canMoveInGoalDir() )
  {
    nlmove();
  }

  //
  // In over torque condition, apply torque control.
  //
  else if( m_bOverTorqueCond )
  {
    nlapplyTorqueControl();
  }

  unlock();

  return HEK_OK;
}

HekKinJoint::MoveState HekKinJointWristRot::nlmove()
{
  double  fAdjustVel;
  int     fAdjustSpeed;

  if( m_eStateWristPitch != MoveStateIdle )
  {
    m_pKinJointWristPitch->getTgtVelSpeed(fAdjustVel, fAdjustSpeed);
  }
  else
  {
    fAdjustVel    = 0.0;
    fAdjustSpeed  = 0;
  }

  if( isMovingToGoal() || isCoupledJointMoving() )
  {
    // still away from goal position
    if( (fabs(m_fJointGoalPos-m_fJointCurPos) > m_fTolPos) ||
        (m_eStateWristPitch != MoveStateIdle) )
    {
      // servo goal od position can change with coupled joints - recalculate
      m_nServoGoalPos = jointPosToServoPos(m_fJointGoalPos);

      if( isMovingToGoal() )
      {
        // PID target velocity 
        m_fJointTgtVel = m_pid.control(m_fJointCurPos, m_fJointCurVel, m_fDt);

        // estimate corresponding target servo speed
        m_nServoTgtSpeed  = estimateServoTgtSpeed(m_fJointTgtVel);
      }
      else
      {
        m_fJointTgtVel    = 0.0;
        m_nServoTgtSpeed  = 0;
      }

      m_nServoTgtSpeed  -= fAdjustSpeed;

      // new move - make it happen
      if( m_eState == MoveStateNewMove )
      {
        moveServo(m_nServoTgtSpeed, m_nServoGoalPos);
      }

      // subsequent changes of servo speed - only if sufficently different
      else if( fabs(m_fJointTgtVel-m_fJointCurVel) > m_fTolVel )
      {
        moveServo(m_nServoTgtSpeed, m_nServoGoalPos);
      }
    }

    // at goal position
    else
    {
      nlstop();
    }
  }

  return m_eState;
}

int HekKinJointWristRot::nlslowToStop(SyncMoveMsgs &msgs)
{
  double  fAdjustVel;
  int     fAdjustSpeed;
  double  fJointStopVel;
  int     nServoStopSpeed;

  if( (fabs(m_fJointCurVel) <= m_fStopVelTh) )
  {
    //fprintf(stderr, "DBG: %s() SLOWED ENOUGH: curvel=%lf\n",
    //  LOGFUNCNAME, radToDeg(m_fJointCurVel));

    return nlstop();
  }
  else
  {
    if( m_fJointCurVel > 0.0 )
    {
      fJointStopVel = m_fJointCurVel - m_fStopVelTh;
    }
    else
    {
      fJointStopVel = m_fJointCurVel + m_fStopVelTh;
    }

    nServoStopSpeed  = estimateServoTgtSpeed(fJointStopVel);

    if( isCoupledJointMoving() )
    {
      m_pKinJointWristPitch->getTgtVelSpeed(fAdjustVel, fAdjustSpeed);
    }
    else
    {
      fAdjustVel    = 0.0;
      fAdjustSpeed  = 0;
    }

    nServoStopSpeed -= fAdjustSpeed;

    addServoMsgEntries(msgs, nServoStopSpeed, m_nServoGoalPos);

    //fprintf(stderr, "DBG: %s() SLOWING TO: curvel=%lf, stopvel=%lf\n",
    //  LOGFUNCNAME, radToDeg(m_fJointCurVel), radToDeg(fJointStopVel));

    return HEK_OK;
  }
}

int HekKinJointWristRot::jointPosToServoPos(double fPos)
{
  int     nAdjustPos = 0, nAdjustSpeed = 0;
  int     nServoPos;

  if( m_pKinJointWristPitch != NULL )
  {
    m_pKinJointWristPitch->getServoCurPosSpeed(nAdjustPos, nAdjustSpeed);
  }

  nServoPos = (int)(fPos * m_pJoint->m_fTicksPerJointRad);

  return nServoPos + nAdjustPos;
}

double HekKinJointWristRot::servoPosToJointPos(int nOdPos)
{
  double  fAdjustPos = 0, fAdjustVel = 0;
  double  fJointPos;

  if( m_pKinJointWristPitch != NULL )
  {
    m_pKinJointWristPitch->getJointCurPosVel(fAdjustPos, fAdjustVel);
  }

  fJointPos = (double)nOdPos * m_fJointRadPerTick;

  return fJointPos - fAdjustPos;
}

void HekKinJointWristRot::updateAssoc(double fJointVel, int nServoSpeed)
{
  double  fAdjustPos = 0.0, fAdjustVel = 0.0;

  if( m_pKinJointWristPitch != NULL )
  {
    m_pKinJointWristPitch->getJointCurPosVel(fAdjustPos, fAdjustVel);
  }

  // pure wrist rotation move only
  if( (fAdjustVel == 0.0) && (getCoupledJointState() == MoveStateIdle) )
  {
    m_tblVelSpeed.update(fJointVel, nServoSpeed);
  }
}

HekKinJoint::MoveState HekKinJointWristRot::getCoupledJointState()
{
  if( m_pKinJointWristPitch != NULL )
  {
    return m_pKinJointWristPitch->getMoveState();
  }
  else
  {
    return MoveStateIdle;
  }
}
