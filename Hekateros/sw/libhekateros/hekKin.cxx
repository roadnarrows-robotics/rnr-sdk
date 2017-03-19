////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekKin.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-05-15 12:50:46 -0600 (Fri, 15 May 2015) $
 * $Rev: 3988 $
 *
 * \brief The Hekateros kinematics and dynamics class implemenation.
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

#include <sys/time.h>
#include <time.h>
#include <limits.h>
#include <unistd.h>
#include <stdio.h>
#include <pthread.h>
#include <sched.h>
#include <math.h>

#include <vector>
#include <map>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaError.h"
#include "Dynamixel/DynaServo.h"
#include "Dynamixel/DynaChain.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekTune.h"
#include "Hekateros/hekJoint.h"
#include "Hekateros/hekTraj.h"
#include "Hekateros/hekKinJoint.h"
#include "Hekateros/hekKin.h"
#include "Hekateros/hekUtils.h"

using namespace std;
using namespace hekateros;


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// HekKinematics Class
// ---------------------------------------------------------------------------

HekKinematics::HekKinematics(DynaChain      &dynaChain,
                             MapRobotJoints &kinJointDesc,
                             const HekTunes &tunes) :
    m_dynaChain(dynaChain)
{
  MapRobotJoints::iterator  iter;
  string                    strJointName;
  HekRobotJoint            *pJoint;
  int                       nServoId;
  DynaServo                *pServo;

  m_eState      = ThreadStateUninit;
  m_nNumJoints  = (int)kinJointDesc.size();
  m_nNumServos  = m_dynaChain.GetNumberInChain();
  m_fHz         = HekTuneKinHzMin;

  for(iter=kinJointDesc.begin(); iter!=kinJointDesc.end(); ++iter)
  {
    pJoint        = &iter->second;
    strJointName  = pJoint->getJointName();
    nServoId      = pJoint->getMasterServoId();
    pServo        = m_dynaChain.GetServo(nServoId);

    if( strJointName == "wrist_rot" )
    {
      m_kinChain[strJointName] = new HekKinJointWristRot(pJoint, pServo, tunes);
    }
    else
    {
      m_kinChain[strJointName] = new HekKinJoint(pJoint, pServo, tunes);
    }
  }

  //
  // Coupled wrist joints.
  //
  if( (m_kinChain.find("wrist_rot")   != m_kinChain.end()) &&
      (m_kinChain.find("wrist_pitch") != m_kinChain.end()) )
  {
    m_kinChain["wrist_rot"]->coupleJoint(m_kinChain["wrist_pitch"]);
  }

  buildTaskList();

#ifdef HEK_KIN_EXEC_ALG_INDIV
  m_iterTask        = m_taskList.begin();
#endif // HEK_KIN_EXEC_ALG_INDIV

  m_nHealthServoId  = m_dynaChain.IterStart(&m_iterHealth);

  pthread_mutex_init(&m_mutexSync, NULL);
  pthread_cond_init(&m_condSync, NULL);

  // create kinematics thread
  createThread();

  m_bIsControlling  = false;

#ifdef HEK_KIN_EXEC_ALG_SYNC
  m_bWaitOneCycle   = false;
#endif // HEK_KIN_EXEC_ALG_SYNC

  LOGDIAG3("Registered kinematic chain of %d joints and %d servos.",
      m_nNumJoints, m_nNumServos);
}

HekKinematics::~HekKinematics()
{
  KinChain::iterator  iter;    // kinematics chain iterator

  terminateThread();

  pthread_cond_destroy(&m_condSync);
  pthread_mutex_destroy(&m_mutexSync);

  for(iter = m_kinChain.begin(); iter != m_kinChain.end(); ++iter)
  {
    if( iter->second != NULL )
    {
      delete iter->second;
      iter->second = NULL;
    }
  }
  m_kinChain.clear();
}

// major hack
void HekKinematics::buildTaskList()
{
  m_taskList.empty();

  // 4DOF arms have no rotating base
  if( m_kinChain.find("base_rot") != m_kinChain.end() )
  {
    m_taskList.push_back("base_rot");
  }

  m_taskList.push_back("shoulder");
  m_taskList.push_back("elbow");
  m_taskList.push_back("wrist_pitch");

  // wrist rot coupled with pitch, keep close
  m_taskList.push_back("wrist_rot");

  // default gripper
  if( m_kinChain.find("grip") != m_kinChain.end() )
  {
    m_taskList.push_back("grip");
  }

#ifdef HEK_KIN_EXEC_ALG_INDIV
  // servo health task
  m_taskList.push_back("health");

  if( m_taskList.size() != m_kinChain.size()+1 )
  {
    LOGERROR("WTF: Task list not matching kinematic chain.");
  }
#endif // HEK_KIN_EXEC_ALG_INDIV
}

void HekKinematics::reload(const HekTunes &tunes)
{
  KinChain::iterator  iter;   // kinematics chain iterator

  for(iter = m_kinChain.begin(); iter != m_kinChain.end(); ++iter)
  {
    iter->second->reload(tunes);
  }

  setHz(tunes.m_fKinematicsHz);
}

int HekKinematics::resetServoOdometersForAllJoints()
{
  KinChain::iterator  iter;   // kinematics chain iterator
  int                 n;      // number of actual joints reset

  lock();

  for(iter = m_kinChain.begin(), n = 0; iter != m_kinChain.end(); ++iter)
  {
    if( iter->second->resetServoOdometer() == HEK_OK )
    {
      ++n;
    }
  }

  unlock();

  waitOneCycle();

  return n;
}

void HekKinematics::getJointCurPosVel(const std::string &strJointName,
                                      double            &fJointCurPos,
                                      double            &fJointCurVel)
{
  KinChain::iterator  pos;

  if( (pos = m_kinChain.find(strJointName)) != m_kinChain.end() )
  {
    pos->second->getJointCurPosVel(fJointCurPos, fJointCurVel);
  }
}

void HekKinematics::getFilteredJointCurPosVel(const std::string &strJointName,
                                              double            &fJointCurPos,
                                              double            &fJointCurVel)
{
  KinChain::iterator  pos;

  if( (pos = m_kinChain.find(strJointName)) != m_kinChain.end() )
  {
    pos->second->getFilteredJointCurPosVel(fJointCurPos, fJointCurVel);
  }
}

void HekKinematics::getServoCurPosSpeed(const std::string &strJointName,
                                        int               &nServoCurPos,
                                        int               &nServoCurSpeed)
{
  KinChain::iterator  pos;

  if( (pos = m_kinChain.find(strJointName)) != m_kinChain.end() )
  {
    pos->second->getServoCurPosSpeed(nServoCurPos, nServoCurSpeed);
  }
}

int HekKinematics::jointPosToServoPos(const std::string &strJointName,
                                      const double      fPos)
{
  KinChain::iterator  pos;

  if( (pos = m_kinChain.find(strJointName)) != m_kinChain.end() )
  {
    return pos->second->jointPosToServoPos(fPos);
  }
  else
  {
    return 0;
  }
}

double HekKinematics::servoPosToJointPos(const std::string &strJointName,
                                         const int          nOdPos)
{
  KinChain::iterator  pos;

  if( (pos = m_kinChain.find(strJointName)) != m_kinChain.end() )
  {
    return pos->second->servoPosToJointPos(nOdPos);
  }
  else
  {
    return 0.0;
  }
}

bool HekKinematics::isStopped(const std::string &strJointName)
{
  KinChain::iterator  pos;

  if( (pos = m_kinChain.find(strJointName)) != m_kinChain.end() )
  {
    return pos->second->isStopped();
  }
  else // non-existent joints don't move
  {
    return true;
  }
}

bool HekKinematics::hasOverTorqueCondition(const std::string &strJointName)
{
  KinChain::iterator  pos;

  if( (pos = m_kinChain.find(strJointName)) != m_kinChain.end() )
  {
    return pos->second->hasOverTorqueCondition();
  }
  else // non-existent joints have no torque issues
  {
    return false;
  }
}

int HekKinematics::resetServoOdometer(const string &strJointName)
{
  KinChain::iterator  pos;    // kinematics chain position
  int                 rc;     // return code

  if( (pos = m_kinChain.find(strJointName)) != m_kinChain.end() )
  {
    rc = pos->second->resetServoOdometer();
  }
  else
  {
    LOGWARN("Joint %s: Not found in kinematic chain.", strJointName.c_str());
    rc = -HEK_ECODE_BAD_VAL;
  }

  waitOneCycle();

  return rc;
}

void HekKinematics::estop()
{
  if( m_bIsControlling )
  {
    stop();
    waitForAllStop(1.0);
  }

  lock();

  m_bIsControlling = false;

  m_dynaChain.EStop();

  unlock();
}
  
void HekKinematics::freeze()
{
  m_bIsControlling = true;

  stop();

  waitForAllStop(5.0);
}
  
void HekKinematics::release()
{
  if( m_bIsControlling )
  {
    stop();
    waitForAllStop(5.0);
  }

  lock();

  m_bIsControlling = false;

  m_dynaChain.Release();

  unlock();
}
  
int HekKinematics::stop()
{
  KinChain::iterator  iter;

  if( !m_bIsControlling )
  {
    return -HEK_ECODE_BAD_OP;
  }

  for(iter=m_kinChain.begin(); iter!=m_kinChain.end(); ++iter)
  {
    iter->second->stop();
  }

  return HEK_OK;
}

int HekKinematics::stop(const vector<string> &vecJointNames)
{
  vector<string>::const_iterator  iter; // joint name iterator

  KinChain::iterator  pos;              // kinematics chain position
  int                 n;                // number of actual joints stopped

  if( !m_bIsControlling )
  {
    return -HEK_ECODE_BAD_OP;
  }

  for(iter = vecJointNames.begin(), n = 0; iter != vecJointNames.end(); ++iter)
  {
    if( (pos = m_kinChain.find(*iter)) != m_kinChain.end() )
    {
      if( pos->second->stop() == HEK_OK )
      {
        ++n;
      }
    }
  }
  return n;
}

int HekKinematics::stop(const string &strJointName)
{
  KinChain::iterator  pos;    // kinematics chain position
  int                 rc;     // return code

  if( !m_bIsControlling )
  {
    return -HEK_ECODE_BAD_OP;
  }

  if( (pos = m_kinChain.find(strJointName)) != m_kinChain.end() )
  {
    rc = pos->second->stop();
  }
  else
  {
    LOGWARN("Joint %s: Not found in kinematic chain.", strJointName.c_str());
    rc = -HEK_ECODE_BAD_VAL;
  }

  return rc;
}

int HekKinematics::waitForAllStop(double fSeconds)
{
  static const useconds_t WaitDt  = 100;  // usec wait between tests

  KinChain::iterator  iter;

  int nMaxTries = (int)ceil(fSeconds * MILLION / (double)WaitDt);
  int nMoveCnt;

  for(int i = 0; i < nMaxTries; ++i)
  {
    nMoveCnt = 0;

    for(iter = m_kinChain.begin(); iter != m_kinChain.end(); ++iter)
    {
      if( !iter->second->isStopped() )
      {
        ++nMoveCnt;
        break;
      }
    }

    if( nMoveCnt == 0 )
    {
      return HEK_OK;
    }

    usleep(WaitDt);
  }

  LOGERROR("Timed out waiting for %.2lf seconds for all joints to stop.",
      fSeconds);

  return -HEK_ECODE_TIMEDOUT;
}

int HekKinematics::move(HekJointTrajectoryPoint &trajectoryPoint)
{
  int                 i;                  // trajectory point iterator
  int                 nNumPts;            // number of actual points
  string              strJointName;       // joint name
  double              fJointGoalPos;      // goal joint position (radians)
  double              fJointGoalVel;      // goal joint velocity (rads/sec)
  double              fJointGoalAcc;      // goal joint acceleration (N/A)
  KinChain::iterator  pos;                // kinematics chain position

  lock();

  for(i=0, nNumPts=0; i<trajectoryPoint.getNumPoints(); ++i)
  {
    //
    // Joint point data: joint name, goal position, velocity, and acceleration.
    //
    trajectoryPoint[i].get(strJointName, fJointGoalPos,
                          fJointGoalVel, fJointGoalAcc);

    if( (pos = m_kinChain.find(strJointName)) != m_kinChain.end() )
    {
      pos->second->specifyMove(fJointGoalPos, fJointGoalVel);
      ++nNumPts;
    }
    else
    {
      LOGWARN("Joint %s: Not found in kinematic chain.", strJointName.c_str());
      continue;
    }
  }

  if( nNumPts > 0 )
  {
    m_bIsControlling = true;
  }

  unlock();

  return nNumPts;
}

int HekKinematics::move(const std::string &strJointName,
                        const double       fJointGoalPos,
                        const double       fJointGoalVel)
{
  KinChain::iterator  pos;    // kinematics chain position
  int                 rc;     // return code

  lock();

  if( (pos = m_kinChain.find(strJointName)) != m_kinChain.end() )
  {
    rc = pos->second->specifyMove(fJointGoalPos, fJointGoalVel);
    m_bIsControlling = true;
  }
  else
  {
    LOGWARN("Joint %s: Not found in kinematic chain.", strJointName.c_str());
    rc = -HEK_ECODE_BAD_VAL;
  }

  unlock();

  return rc;
}
 
void HekKinematics::sense()
{
  TaskList::iterator  iter;

  for(iter = m_taskList.begin(); iter != m_taskList.end(); ++iter)
  {
    m_kinChain[*iter]->senseDynamics(m_bIsControlling);
  }
}

void HekKinematics::react()
{
  TaskList::iterator  iter;

  for(iter = m_taskList.begin(); iter != m_taskList.end(); ++iter)
  {
    m_kinChain[*iter]->react(m_bIsControlling, m_msgs);
  }
}

void HekKinematics::sense_react()
{
  TaskList::iterator  iter;

  for(iter = m_taskList.begin(); iter != m_taskList.end(); ++iter)
  {
    m_kinChain[*iter]->senseDynamics(m_bIsControlling);
    m_kinChain[*iter]->react(m_bIsControlling, m_msgs);
  }
}

void HekKinematics::plan()
{
  TaskList::iterator  iter;

  for(iter = m_taskList.begin(); iter != m_taskList.end(); ++iter)
  {
    m_kinChain[*iter]->planMotion(m_bIsControlling, m_msgs);
  }
}

int HekKinematics::act()
{
  uint_t  uCount;
  int     rc;

  if( !m_bIsControlling )
  {
    return HEK_OK;
  }

  if( (uCount = m_msgs.getSpeedTupleCount()) > 0 )
  {
    rc = m_dynaChain.SyncWriteGoalSpeed(m_msgs.getSpeedTuples(), uCount);
  }

  if( (rc == DYNA_OK) && ((uCount = m_msgs.getOdPosTupleCount()) > 0) )
  {
    rc = m_dynaChain.SyncWriteGoalPos(m_msgs.getOdPosTuples(), uCount);
  }

  rc = rc < 0? -HEK_ECODE_DYNA: HEK_OK;

  return rc;
}

void HekKinematics::monitorHealth()
{
  DynaServo  *pServo;
  uint_t      uAlarms;
  int         nCurLoad;
  uint_t      uCurVolt;
  uint_t      uCurTemp;

  pServo = m_dynaChain.GetServo(m_nHealthServoId);

  if( (m_nHealthServoId = m_dynaChain.IterNext(&m_iterHealth)) == DYNA_ID_NONE )
  {
    m_nHealthServoId = m_dynaChain.IterStart(&m_iterHealth);
  }
  
  LOGDIAG4("Kinematics Thread: Monitoring health for servo %d.",
      pServo->GetServoId());

  pServo->ReadHealth(&uAlarms, &nCurLoad, &uCurVolt, &uCurTemp);
}

#ifdef HEK_KIN_EXEC_ALG_SYNC

void HekKinematics::exec()
{
  lock();

  // clear previous synchronous move messages
  m_msgs.clear();

  // sense-react on sensed state
  sense_react();

  // plan motions
  plan();

  // act on planned motions
  act();

  // monitor servo's health
  monitorHealth();

  // caller blocked waiting one execution cycle - signal to unblock
  if( m_bWaitOneCycle )
  {
    m_bWaitOneCycle = false;
    pthread_cond_signal(&m_condSync);
  }

  unlock();
}

#else // HEK_KIN_EXEC_ALG_INDIV

void HekKinematics::exec()
{
  lock();

  string strTask = *m_iterTask++;

  if( m_iterTask == m_taskList.end() )
  {
    m_iterTask = m_taskList.begin();
  }

#ifdef DBG
  if( !m_strTaskOneCycle.empty() )
  {
    fprintf(stderr, "DBG: %s(): task=%s\n", LOGFUNCNAME, strTask.c_str());
  }
#endif // DBG

  if( strTask == m_strTaskOneCycle )
  {
    m_strTaskOneCycle.clear();
    pthread_cond_signal(&m_condSync);
  }


  // health monitor task
  if( strTask == "health" )
  {
    monitorHealth();
  }

  // joint sense and control tasks
  else
  {
    HekKinJoint *pKinJoint = m_kinChain[strTask];

    pKinJoint->senseDynamics(m_bIsControlling);

    if( m_bIsControlling )
    {
      pKinJoint->act();
    }
  }

  unlock();
}

#endif // HEK_KIN_EXEC_ALG_SYNC

void *HekKinematics::thread(void *pArg)
{
  HekKinematics *pThis = (HekKinematics *)pArg;
  int           oldstate;
  int           rc;

  LOGDIAG3("Kinematics thread created - ready to run.");

  pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, &oldstate);
  pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, &oldstate);

  pThis->m_eState = ThreadStateReady;

  //
  // Loop forever until exit
  //
  while( (pThis->m_eState != ThreadStateExit) )
  {
    switch( pThis->m_eState )
    {
      //
      // Blocked on Ready "queue"
      //
      case ThreadStateReady:
        pThis->readyWait();

        // ready -> running 
        if( pThis->m_eState == ThreadStateRunning )
        {
#ifdef HEK_KIN_EXEC_ALG_SYNC
          LOGDIAG3("Kinematics thread started - running at %.1lf Hz.",
                pThis->m_fHz);
#else // HEK_KIN_EXEC_ALG_INDIV
          LOGDIAG3("Kinematics thread started - "
                   "running at %.1lf Hz, %lf seconds/task.",
                pThis->m_fHz, pThis->m_fTExec);
#endif // HEK_KIN_EXEC_ALG_SYNC
 
          // force immediately execution of first task
          clock_gettime(CLOCK_REALTIME, &pThis->m_tsSched);
        }
        break;

      //
      // Run wait,execute subcycle
      //
      case ThreadStateRunning:
        pThis->schedWait();
        if( pThis->m_eState == ThreadStateRunning )
        {
          pThis->exec();
        }
        break;

      //
      // Exit
      //
      case ThreadStateExit:
        break;

      default:
        LOGERROR("%d: Unexpected dynamixel background thread state.",
            pThis->m_eState);
        pThis->m_eState = ThreadStateExit;
        break;
    }
  }

  LOGDIAG3("Kinematics thread exited.");

  return NULL;
}

void HekKinematics::createThread()
{
  pthread_attr_t      attr;
  struct sched_param  parm;
  int                 rc;

  // Initialize thread attributes.
  pthread_attr_init(&attr);

  //
  // Set thread scheduling policy.
  // SCHED_FIFO:
  //  + preemptive priority scheduling
  //  + highest priority thread runs until it choses to block/sleep
  //  + when it unblock/wakes it goes to the end of the queue for its priority
  //
  pthread_attr_setschedpolicy(&attr, SCHED_FIFO);

  //
  // Get the system's maximum priority and set this thread to the
  // next-to-the-highest priority.
  parm.sched_priority = sched_get_priority_max(SCHED_FIFO) - 1;
  pthread_attr_setschedparam(&attr, &parm);

  // Create kinematics thread.
  rc = pthread_create(&m_thread, &attr, HekKinematics::thread, (void *)this);
 
  // wait for thread to initialize
  if( rc == 0 )
  {
    lock();
    pthread_cond_wait(&m_condSync, &m_mutexSync);
    unlock();
  }
  else
  {
    LOGSYSERROR("pthread_create()");
    m_eState = ThreadStateExit;
  }
}

void HekKinematics::setHz(const double fHz)
{
  m_fHz = fHz;

  if( m_fHz < HekTuneKinHzMin )
  {
    m_fHz = HekTuneKinHzMin;
  }

#ifdef HEK_KIN_EXEC_ALG_SYNC
  m_fTExec = 1.0 / m_fHz;
#else // !HEK_KIN_EXEC_ALG_INDIV
  m_fTExec = 1.0 / (m_fHz * (double)m_taskList.size());
#endif // HEK_KIN_EXEC_ALG_SYNC

  m_tsExecPeriod.tv_sec  = (long)floor(m_fTExec); 
  m_tsExecPeriod.tv_nsec = (long)fcap(
                                (m_fTExec-floor(m_fTExec)) * (double)BILLION,
                                0.0, (double)(BILLION-1) );
}

int HekKinematics::runThread(double fHz)
{
  int       rc;       // return code

  setHz(fHz);

  switch( m_eState )
  {
    case ThreadStateReady:
      changeState(ThreadStateRunning);
      rc = HEK_OK;
      break;

    case ThreadStateRunning:
    case ThreadStateExit:
    default:
      rc = -HEK_ECODE_GEN;
      LOGERROR("Kinematics thread in invalid state %d to run.", m_eState);
      break;
  } 

  return rc;
}

void HekKinematics::terminateThread()
{
  changeState(ThreadStateExit);
  pthread_join(m_thread, NULL);
}

#ifdef HEK_KIN_EXEC_ALG_SYNC
void HekKinematics::waitOneCycle()
{
  lock();

  m_bWaitOneCycle = true;

  pthread_cond_wait(&m_condSync, &m_mutexSync);

  unlock();
}

#else // HEK_KIN_EXEC_ALG_INDIV

void HekKinematics::waitOneCycle()
{
  lock();

  if( m_iterTask == m_taskList.begin() )
  {
    m_strTaskOneCycle = m_taskList.back();
  }
  else
  {
    TaskList::iterator  prev = m_iterTask - 1;
    m_strTaskOneCycle = *prev;
  }

#ifdef DBG
    fprintf(stderr, "DBG: %s(): taskonecycle=%s\n", LOGFUNCNAME,
          strTaskOneCycle.c_str());
#endif // DBG

  pthread_cond_wait(&m_condSync, &m_mutexSync);

  unlock();
}

#endif // HEK_KIN_EXEC_ALG_SYNC

void HekKinematics::readyWait()
{
  lock();

  // signal calling thread that created kinematics thread.
  pthread_cond_signal(&m_condSync);

  while( m_eState == ThreadStateReady )
  {
    pthread_cond_wait(&m_condSync, &m_mutexSync);
  }

  unlock();
}
  
void HekKinematics::timedWait(const struct timespec &tsTimeout)
{
  lock();

  pthread_cond_timedwait(&m_condSync, &m_mutexSync, &tsTimeout);

  unlock();
}
 
void HekKinematics::schedWait()
{
  static bool     bLogWarnings = false; // true to debug scheduling overloading
  struct timespec tsNow;                // now
  struct timespec tsSlip;               // any slippage

  // now
  clock_gettime(CLOCK_REALTIME, &tsNow);

  if( m_tsSched < tsNow )
  { 
    tsSlip = tsNow - m_tsSched;
  }
  else
  {
    tsSlip.tv_sec = tsSlip.tv_nsec = 0;
  }

  // block wait for next execute cycle
  if( tsNow < m_tsSched )
  {
    timedWait(m_tsSched);
    clock_gettime(CLOCK_REALTIME, &tsNow);
    m_tsSched = tsNow + m_tsExecPeriod;
  }

  // slipped by less than two execution cycles - try to make up the time
  else if( tsSlip < m_tsExecPeriod )
  {
    m_tsSched = tsNow + m_tsExecPeriod - tsSlip;
  }

  // slipped a bunch
  else
  {
    m_tsSched = tsNow + m_tsExecPeriod;

    // debug logging
    if( bLogWarnings )
    {
#ifdef HEK_KIN_EXEC_ALG_SYNC
      LOGWARN("Kinematics thread: "
          "Execution slipped by %ld.%09ld seconds.",
          tsSlip.tv_sec, tsSlip.tv_nsec);
#else // HEK_KIN_EXEC_ALG_INDIV
      LOGWARN("Kinematics thread: "
          "Scheduled task %s slipped by %ld.%09ld seconds.",
          m_iterTask->c_str(), tsSlip.tv_sec, tsSlip.tv_nsec);
#endif // HEK_KIN_EXEC_ALG_SYNC
    }
  }
}
