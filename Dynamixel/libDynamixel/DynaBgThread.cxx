////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   libDynamixel
//
// File:      DynaBgThread.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-07-08 12:19:29 -0600 (Wed, 08 Jul 2015) $
 * $Rev: 4024 $
 *
 * \brief Dynamixel background thread class.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * 
 * \copyright
 *   \h_copy 2011-2017. RoadNarrows LLC.\n
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
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <iostream>
#include <map>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/new.h"
#include "rnr/units.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/MX.h"

#include "Dynamixel/DynaError.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaServo.h"
#include "Dynamixel/DynaChain.h"
#include "Dynamixel/DynaBgThread.h"
#include "Dynamixel/DynaOlio.h"

#include "DynaLibInternal.h"

using namespace std;


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

// big switch
#undef  BG_TRACE_ENABLE

#ifdef BG_TRACE_ENABLE

#define BG_TRACE_FILE           "/tmp/bgtrace.log"  ///< trace file
#define BG_TRACE_SERVO_ALL      (-1)                ///< all servos filter
#define BG_TRACE_FILTER_POSCTL  0x01                ///< position control filter
#define BG_TRACE_FILTER_TORQCTL 0x02                ///< torque control filter
#define BG_TRACE_FILTER_DYNA    0x04                ///< dynamics topic filter
#define BG_TRACE_FILTER_HEALTH  0x08                ///< health topic filter
#define BG_TRACE_FILTER_ALL     0xff                ///< all topic filters

// trace filtering switches
#define BG_TRACE_SERVO        BG_TRACE_SERVO_ALL ///< all or specify servo id
#define BG_TRACE_FILTER       (BG_TRACE_FILTER_POSCTL|BG_TRACE_FILTER_TORQCTL)
                                                 ///< or'ed filter bits

static FILE *FpBgTrace = NULL;  ///< trace file pointer

/*!
 * \brief Trace macro.
 *
 * \param mask    Trace filter mask.
 * \param servoid Servo id.
 * \param prefix  Prefix string.
 * \param fmt     Format string.
 * \param ...     Variable format arguments.
 */
#define BG_TRACE(mask, servoid, prefix, fmt, ...) \
  do \
  { \
    if( (FpBgTrace != NULL) && (mask & BG_TRACE_FILTER) && \
        ((BG_TRACE_SERVO==BG_TRACE_SERVO_ALL) || (servoid==BG_TRACE_SERVO)) )\
    { \
      fprintf(FpBgTrace, prefix ": Servo: %d: " fmt, servoid, ##__VA_ARGS__); \
      fflush(FpBgTrace); \
    } \
  } while(0)

/*!
 * \brief Open trace file.
 */
#define BG_TRACE_OPEN() \
  do \
  { \
    if( (FpBgTrace = fopen(BG_TRACE_FILE, "w")) != NULL ) \
    { \
      fprintf(stderr, "BgThread tracing enabled. File: %s\n", BG_TRACE_FILE); \
      fprintf(FpBgTrace, "### Start Background Thread Tracing.\n\n"); \
      fflush(FpBgTrace); \
    } \
  } while(0)

/*!
 * \brief Close trace file.
 */
#define BG_TRACE_CLOSE() \
  do \
  { \
    if( FpBgTrace != NULL ) \
    { \
      fclose(FpBgTrace); \
    } \
  } while(0)

#else

/*! \brief No tracing output. */
#define BG_TRACE(mask, servoid, prefix, fmt, ...)

/*! \brief No tracing file open. */
#define BG_TRACE_OPEN()

/*! \brief No tracing file close. */
#define BG_TRACE_CLOSE()

#endif // BG_TRACE_ENABLE

/*!
 * \brief Calculate the difference between two time-of-day times.
 *
 * \param t1  Later time instance.
 * \param t0  Earlier time instance.
 *
 * \return Microsecond difference.
 */
static long dt_usec(struct timeval t1, struct timeval t0)
{
  long  dt;

  if( t1.tv_usec >= t0.tv_usec)
  {
    dt = (t1.tv_sec - t0.tv_sec) * 1000000 + (t1.tv_usec - t0.tv_usec);
  }
  else
  {
    dt = (t1.tv_sec - t0.tv_sec - 1) * 1000000 + 
      (1000000 + t1.tv_usec - t0.tv_usec);
  }

  return dt;
}


// ---------------------------------------------------------------------------
// Dynamixel Virtual Servo Class
// ---------------------------------------------------------------------------

DynaVServo::DynaVServo() :
    m_histTorqueIn(TORQUE_WIN_SIZE, 0.0)
{
  m_eState          = StateIdle;
  m_nTolerance      = 10;
  m_nOdGoalPos      = 0;
  m_nGoalSpeed      = GOAL_SPEED_DFT;
  m_bOverTorqueCond = false;
  m_bOverTorqueCtl  = false;
  m_fTorqueOut      = 0.0; 

  pthread_mutex_init(&m_mutexSync, NULL);
}

DynaVServo::DynaVServo(const DynaVServo &src)
{
  m_eState          = src.m_eState;
  m_pidPos          = src.m_pidPos;
  m_nTolerance      = src.m_nTolerance;
  m_nOdGoalPos      = src.m_nOdGoalPos;
  m_nGoalSpeed      = src.m_nGoalSpeed;
  m_bOverTorqueCond = src.m_bOverTorqueCond;
  m_bOverTorqueCtl  = src.m_bOverTorqueCtl;
  m_fTorqueOut      = src.m_fTorqueOut;
  m_histTorqueIn    = src.m_histTorqueIn;

  pthread_mutex_init(&m_mutexSync, NULL);
}

DynaVServo DynaVServo::operator=(const DynaVServo &rhs)
{
  m_eState          = rhs.m_eState;
  m_pidPos          = rhs.m_pidPos;
  m_nTolerance      = rhs.m_nTolerance;
  m_nOdGoalPos      = rhs.m_nOdGoalPos;
  m_nGoalSpeed      = rhs.m_nGoalSpeed;
  m_bOverTorqueCond = rhs.m_bOverTorqueCond;
  m_bOverTorqueCtl  = rhs.m_bOverTorqueCtl;
  m_fTorqueOut      = rhs.m_fTorqueOut;
  m_histTorqueIn    = rhs.m_histTorqueIn;

  return *this;
}

void DynaVServo::setToleranceInTicks(DynaServo *pServo, double fTolerance)
{
  double    fTicks;
  double    fAngleMin;
  double    fAngleMax;
  double    fDegrees;

  //
  // Servo rotation range.
  //
  if( pServo->GetServoMode() == DYNA_MODE_CONTINUOUS )
  {
    fDegrees = 360.0;
  }
  else
  {
    fAngleMin = pServo->GetSpecification().m_fAngleMin;
    fAngleMax = pServo->GetSpecification().m_fAngleMax;
    fDegrees  = fAngleMax - fAngleMin;
  }
  
  fTicks = (double)pServo->GetSpecification().m_uRawPosModulo;

  m_nTolerance = (int)(fTicks * fTolerance / fDegrees);
}

double DynaVServo::filterTorques(int nServoLoad)
{
  double  fTorque0;
  double  fTorqueK;

  fTorque0 = (double)nServoLoad / (double)TORQUE_WIN_SIZE;
  fTorqueK = m_histTorqueIn.back();
  m_histTorqueIn.pop_back();
  m_histTorqueIn.push_front(fTorque0);

  m_fTorqueOut = m_fTorqueOut - fTorqueK + fTorque0;

  return m_fTorqueOut;
}


// ---------------------------------------------------------------------------
// Dynamixel Backgroud Thread Class
// ---------------------------------------------------------------------------

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Public Interface
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

DynaBgThread::DynaBgThread(double fHz, double fTolerance)
{
  BG_TRACE_OPEN();

  pthread_mutex_init(&m_mutexSync, NULL);
  pthread_cond_init(&m_condSync, NULL);

  m_pServo      = NULL;
  m_pChain      = NULL;
  m_uNumServos  = 0;

  setTolerance(fTolerance);

  m_agent.m_fnWriteGoalPos      = WriteGoalPos;
  m_agent.m_fnWriteGoalSpeed    = WriteGoalSpeed;
  m_agent.m_fnWriteGoalSpeedPos = WriteGoalSpeedPos;

  m_fnUserCb  = NULL;
  m_pUserArg  = NULL;

  m_iterDynamics      = m_mapVServo.end();
  m_iterHealth        = m_mapVServo.end();
  m_nHealthServoId    = DYNA_ID_NONE;

  setHz(fHz);

  // create background thread
  createThread();
}

DynaBgThread::~DynaBgThread()
{
  UnregisterAgent();
  UnregisterUserCallback();

  if( m_eState != BgThreadStateZombie )
  {
    terminateThread();
  }

  pthread_cond_destroy(&m_condSync);
  pthread_mutex_destroy(&m_mutexSync);

  BG_TRACE_CLOSE();
}

void DynaBgThread::RegisterServoAgent(DynaServo *pServo)
{
  int         nServoId;
  DynaVServo  vServo;

  UnregisterAgent();

  if( pServo != NULL )
  {
    m_pServo      = pServo;
    m_pChain      = NULL;
    m_uNumServos  = 1;

    nServoId = pServo->GetServoId();

    m_mapVServo[nServoId] = vServo;

    m_mapVServo[nServoId].setToleranceInTicks(pServo, m_fTolerance);

    pServo->RegisterAgent(&m_agent, this);
  }

  setHz(m_fHz);

  m_iterDynamics      = m_mapVServo.begin();
  m_iterHealth        = m_mapVServo.begin();
  m_nHealthServoId    = m_iterHealth->first;

  LOGDIAG3("Registered servo: t_sched=%lu", m_TSched);
}

void DynaBgThread::RegisterChainAgent(DynaChain *pChain)
{
  int         nServoId;
  DynaServo  *pServo;
  int         iter;
  DynaVServo  vServo;

  UnregisterAgent();

  if( pChain != NULL )
  {
    m_pServo      = NULL;
    m_pChain      = pChain;
    m_uNumServos  = pChain->GetNumberInChain();

    for(nServoId = m_pChain->IterStart(&iter);
        nServoId != DYNA_ID_NONE;
        nServoId = m_pChain->IterNext(&iter))
    {
      if( (pServo = m_pChain->GetServo(nServoId)) != NULL )
      {
        m_mapVServo[nServoId] = vServo;
        m_mapVServo[nServoId].setToleranceInTicks(pServo, m_fTolerance);

        pServo->RegisterAgent(&m_agent, this);
      }
    }
  }

  setHz(m_fHz);

  m_iterDynamics      = m_mapVServo.begin();
  m_iterHealth        = m_mapVServo.begin();
  m_nHealthServoId    = m_iterHealth->first;

  LOGDIAG3("Registered chain: t_sched=%lu", m_TSched);
}

void DynaBgThread::UnregisterAgent()
{
  int         nServoId;
  DynaServo  *pServo;
  int         iter;

  if( m_pServo != NULL )
  {
    m_pServo->UnregisterAgent();
  }
  else if( m_pChain != NULL )
  {
    for(nServoId = m_pChain->IterStart(&iter);
        nServoId != DYNA_ID_NONE;
        nServoId = m_pChain->IterNext(&iter))
    {
      if( (pServo = m_pChain->GetServo(nServoId)) != NULL )
      {
        pServo->UnregisterAgent();
      }
    }
  }

  m_pServo      = NULL;
  m_pChain      = NULL;
  m_uNumServos  = 0;

  m_mapVServo.clear();
}

int DynaBgThread::Run()
{
  int         rc;   // return code

  switch( m_eState )
  {
    case BgThreadStateReady:
      gettimeofday(&m_tvSched, NULL);
      changeState(BgThreadStateRunning);
      LOGDIAG3("Background thread running.");
      rc = DYNA_OK;
      break;
    case BgThreadStateZombie:
      rc = -DYNA_ECODE_GEN;
      DYNA_LOG_ERROR(rc, "No background thread.");
      break;
    case BgThreadStateRunning:
    case BgThreadStatePaused:
    case BgThreadStateExit:
    default:
      rc = -DYNA_ECODE_GEN;
      DYNA_LOG_ERROR(rc, "Background thread in invalid state %d.", m_eState);
      break;
  } 

  return rc;
}

int DynaBgThread::Stop()
{
  int   rc;

  switch( m_eState )
  {
    case BgThreadStateReady:
    case BgThreadStateRunning:
    case BgThreadStatePaused:
      changeState(BgThreadStateReady);
      LOGDIAG3("Background thread stopped.");
      rc = DYNA_OK;
      break;
    case BgThreadStateZombie:
      rc = -DYNA_ECODE_GEN;
      DYNA_LOG_ERROR(rc, "No background thread.");
      break;
    case BgThreadStateExit:
    default:
      rc = -DYNA_ECODE_GEN;
      DYNA_LOG_ERROR(rc, "Background thread in invalid state %d.", m_eState);
      break;
  } 

  return rc;
}

int DynaBgThread::Pause()
{
  int   rc;

  switch( m_eState )
  {
    case BgThreadStateRunning:
    case BgThreadStatePaused:
      changeState(BgThreadStatePaused);
      LOGDIAG3("Background thread paused.");
      rc = DYNA_OK;
      break;
    case BgThreadStateZombie:
      rc = -DYNA_ECODE_GEN;
      DYNA_LOG_ERROR(rc, "No background thread.");
      break;
    case BgThreadStateReady:
    case BgThreadStateExit:
    default:
      rc = -DYNA_ECODE_GEN;
      DYNA_LOG_ERROR(rc, "Background thread in invalid state %d.", m_eState);
      break;
  } 

  return rc;
}

int DynaBgThread::Resume()
{
  int         rc;             // return code

  switch( m_eState )
  {
    case BgThreadStatePaused:
      gettimeofday(&m_tvSched, NULL);
      changeState(BgThreadStateRunning);
      LOGDIAG3("Background thread resumed.");
      rc = DYNA_OK;
      break;
    case BgThreadStateZombie:
      rc = -DYNA_ECODE_GEN;
      DYNA_LOG_ERROR(rc, "No background thread.");
      break;
    case BgThreadStateReady:
    case BgThreadStateRunning:
    case BgThreadStateExit:
    default:
      rc = -DYNA_ECODE_GEN;
      DYNA_LOG_ERROR(rc, "Background thread in invalid state %d.", m_eState);
      break;
  } 

  return rc;
}

int DynaBgThread::WriteGoalPos(int nServoId, int nOdGoalPos, void *pUserArg)
{
  DynaBgThread         *pThis = (DynaBgThread *)pUserArg;
  MapVServo::iterator   iter;
  DynaServo            *pServo;
  DynaVServo           *pVServo;
  int                   nOdCurPos;
  int                   nGoalSpeed;
  int                   rc;

  pServo = pThis->getRegisteredServo(nServoId);

  if( pServo == NULL )
  {
    rc = -DYNA_ECODE_NO_SERVO;
  }

  iter = pThis->m_mapVServo.find(nServoId);

  if( iter == pThis->m_mapVServo.end() )
  {
    rc = -DYNA_ECODE_NO_SERVO;
  }
  
  else
  {
    pVServo = &(iter->second);
    pVServo->lock();
    nOdCurPos = pServo->GetOdometer();
    pVServo->m_eState = DynaVServo::StateControl;
    pVServo->m_nOdGoalPos = nOdGoalPos;
    nGoalSpeed = pVServo->m_nGoalSpeed;

    pVServo->m_pidPos.SpecifySetPoint(nOdCurPos,
                                      nOdGoalPos,
                                      nGoalSpeed,
                                      pServo->GetCurSpeed(),
                                      pServo->IsOdometerReversed(),
                                      true);

    pVServo->unlock();

    BG_TRACE(BG_TRACE_FILTER_POSCTL|BG_TRACE_FILTER_TORQCTL, nServoId,
          "WriteGoalPos",
          "speedlimit=%d, goalpos=%d, curpos=%d, tol=%d.\n",
          nGoalSpeed,
          nOdGoalPos,
          nOdCurPos,
          pVServo->m_nTolerance);

    rc = DYNA_OK;
  }

  return rc;
}

int DynaBgThread::WriteGoalSpeed(int nServoId, int nGoalSpeed, void *pUserArg)
{
  DynaBgThread         *pThis = (DynaBgThread *)pUserArg;
  MapVServo::iterator   iter;
  DynaServo            *pServo;
  DynaVServo           *pVServo;
  int                   nOdCurPos;
  int                   nOdGoalPos;
  int                   rc;

  pServo = pThis->getRegisteredServo(nServoId);

  if( pServo == NULL )
  {
    rc = -DYNA_ECODE_NO_SERVO;
  }

  iter = pThis->m_mapVServo.find(nServoId);

  if( iter == pThis->m_mapVServo.end() )
  {
    rc = -DYNA_ECODE_NO_SERVO;
  }
  
  else if( nGoalSpeed == DYNA_SPEED_CONT_STOP )
  {
    pVServo = &(iter->second);

    pThis->stopPosCtl(pServo, pVServo);

    rc = DYNA_OK;
  }

  else
  {
    pVServo = &(iter->second);
    pVServo->lock();
    nOdCurPos = pServo->GetOdometer();
    pVServo->m_eState = DynaVServo::StateControl;
    pVServo->m_nGoalSpeed = nGoalSpeed;
    nOdGoalPos = pVServo->m_nOdGoalPos;

    pVServo->m_pidPos.SpecifySetPoint(nOdCurPos,
                                      nOdGoalPos,
                                      nGoalSpeed,
                                      pServo->GetCurSpeed(),
                                      pServo->IsOdometerReversed(),
                                      true);

    pVServo->unlock();

    BG_TRACE(BG_TRACE_FILTER_POSCTL|BG_TRACE_FILTER_TORQCTL, nServoId,
          "WriteGoalSpeed",
          "speedlimit=%d, goalpos=%d, curpos=%d, tol=%d.\n",
          nGoalSpeed,
          nOdGoalPos,
          nOdCurPos,
          pVServo->m_nTolerance);

    rc = DYNA_OK;
  }

  return rc;
}

int DynaBgThread::WriteGoalSpeedPos(int   nServoId,
                                    int   nGoalSpeed,
                                    int   nOdGoalPos,
                                    void *pUserArg)
{
  DynaBgThread         *pThis = (DynaBgThread *)pUserArg;
  MapVServo::iterator   iter;
  DynaServo            *pServo;
  DynaVServo           *pVServo;
  int                   nOdCurPos;
  int                   rc;

  pServo = pThis->getRegisteredServo(nServoId);

  if( pServo == NULL )
  {
    return -DYNA_ECODE_NO_SERVO;
  }

  iter = pThis->m_mapVServo.find(nServoId);

  if( iter == pThis->m_mapVServo.end() )
  {
    return -DYNA_ECODE_NO_SERVO;
  }
  
  pVServo = &(iter->second);

  pVServo->lock();

  switch( pServo->GetServoMode() )
  {
    case DYNA_MODE_CONTINUOUS:
      nOdCurPos = pServo->GetOdometer();
      pVServo->m_eState = DynaVServo::StateControl;
      pVServo->m_nOdGoalPos = nOdGoalPos;
      pVServo->m_nGoalSpeed = nGoalSpeed;

      pVServo->m_pidPos.SpecifySetPoint(nOdCurPos,
                                        nOdGoalPos,
                                        nGoalSpeed,
                                        pServo->GetCurSpeed(),
                                        pServo->IsOdometerReversed(),
                                        true);

      break;

    case DYNA_MODE_SERVO:
    default:
      pVServo->m_eState = DynaVServo::StateControl;
      pVServo->m_nOdGoalPos = nOdGoalPos;
      pVServo->m_nGoalSpeed = nGoalSpeed;
      if( pServo->WriteGoalSpeed(nGoalSpeed) == DYNA_OK )
      {
        pServo->WriteGoalPos(nOdGoalPos);
      }
      break;
  }

  pVServo->unlock();

  BG_TRACE(BG_TRACE_FILTER_POSCTL|BG_TRACE_FILTER_TORQCTL, nServoId,
          "WriteGoalPosSpeed",
          "speedlimit=%d, goalpos=%d, curpos=%d, tol=%d.\n",
          nGoalSpeed,
          nOdGoalPos,
          nOdCurPos,
          pVServo->m_nTolerance);

  return DYNA_OK;
}

void DynaBgThread::setHz(double fHz)
{
  if( fHz < HZ_EXEC_MIN )
  {
    fHz = HZ_EXEC_MIN;
  }

  m_fHz   = fHz;
  m_TExec = (long)(1.0/m_fHz * 1000000.0);

  if( m_mapVServo.size() > 0 )
  {
    m_TSched = m_TExec / m_mapVServo.size();
  }
  else
  {
    m_TSched = 5000000;
  }
  if( m_TSched  < T_EXEC_MIN )
  {
    m_TSched = T_EXEC_MIN;
  }

  LOGDIAG3("BGThread: Hz: %.3lf, Scheduled period: %ldusecs.", m_fHz, m_TSched);
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Protected Interface
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void DynaBgThread::changeState(DynaBgThread::BgThreadState eNewState)
{
  m_eState = eNewState;
  pthread_cond_signal(&m_condSync);
}

void DynaBgThread::readyWait()
{
  lock();
  while( m_eState == BgThreadStateReady )
  {
    pthread_cond_wait(&m_condSync, &m_mutexSync);
  }
  unlock();
}

void DynaBgThread::timeWait(long lMicroSecs)
{
  struct timespec tsTimeout;
  long int        lSecs;

  if( lMicroSecs <= 0 )
  {
    return;
  }

  clock_gettime(CLOCK_REALTIME, &tsTimeout);

  tsTimeout.tv_nsec = lMicroSecs * 1000;
  if( tsTimeout.tv_nsec > 1000000000 )
  {
    tsTimeout.tv_sec  += 1000000000;
    tsTimeout.tv_nsec -= 1000000000;
  }

  // wait with timeout
  lock();
  pthread_cond_timedwait(&m_condSync, &m_mutexSync, &tsTimeout);
  unlock();
}

DynaServo *DynaBgThread::getRegisteredServo(int nServoId)
{
  if( m_pChain != NULL )
  {
    return m_pChain->GetServo(nServoId);
  }
  else if( m_pServo != NULL )
  {
    return m_pServo->GetServoId() == nServoId? m_pServo: NULL;
  }
  else
  {
    return NULL;
  }
}

void DynaBgThread::sched(long lPeriod)
{
  static bool     bLogWarnings = false; // true to help tune 

  struct timeval  tvNow;        // now
  long            dt;           // delta time (microseconds)
  long            dtRemaining;  // remaining delta time to next scheduled event

  // now
  gettimeofday(&tvNow, NULL);

  // delta time between calls to this scheduler
  dt = dt_usec(tvNow, m_tvSched);

  // time remaining till next background thread event
  dtRemaining = lPeriod - dt;

  // wait
  if( dtRemaining > 0 )
  {
    timeWait(dtRemaining);
    gettimeofday(&tvNow, NULL);
    m_dTSched = dt_usec(tvNow, m_tvSched);
  }

  // slipped task period - don't wait
  else
  {
    // only log if really bad
    dtRemaining = fabs(dtRemaining);
    if( bLogWarnings && (dtRemaining >= lPeriod) )
    {
      LOGWARN("BGThread: Scheduled task slipped by %.2fmsec for servo %d",
        dtRemaining/1000.0, m_iterDynamics->first);
    }
    m_dTSched  = dt;
  }

  // save now
  m_tvSched = tvNow;
}

void DynaBgThread::exec()
{
  int         nServoId;
  DynaVServo *pVServo;
  DynaServo  *pServo;

  //
  // Get the next virtual servo.
  //
  if( (pVServo = getNextVServo(m_iterDynamics, nServoId)) == NULL )
  {
    return;
  }

  //
  // Get the associated registered servo.
  //
  if( (pServo = getRegisteredServo(nServoId)) == NULL )
  {
    // can't find servo - erase entry from vServo map
    m_mapVServo.erase(nServoId);
    m_uNumServos = (uint_t)m_mapVServo.size();
    return;
  }

  //
  // Execute servo dynamics monitoring and control.
  //
  execDynamics(pServo, pVServo);

  //
  // Monitor servo health.
  //
  if( nServoId == m_nHealthServoId )
  {
    monitorHealth(pServo, pVServo);
    getPrevVServo(m_iterHealth, m_nHealthServoId);
  }

  //
  // User callback
  //
  if( m_fnUserCb != NULL )
  {
    m_fnUserCb(m_pUserArg);
  }
}

void DynaBgThread::execDynamics(DynaServo *pServo, DynaVServo *pVServo)
{
  pVServo->lock();

  //
  // Read relavent servo state dynamics.
  //
  monitorDynamics(pServo, pVServo);

  //
  // Servo in torque overload condition, ease motor drive if possible.
  //
  if( pServo->GetState().m_bTorqueEnabled &&
      (pVServo->m_bOverTorqueCond || pVServo->m_bOverTorqueCtl) )
  {
    execTorqueCtl(pServo, pVServo);
  }

  //
  // Servo not driven or state is idle.
  //
  if( !pServo->GetState().m_bTorqueEnabled ||
      (pVServo->m_eState == DynaVServo::StateIdle) )
  {
    // do nothing
  }

  //
  // Stop servo.
  //
  else if( pVServo->m_nGoalSpeed == DYNA_SPEED_CONT_STOP )
  {
    stopPosCtl(pServo, pVServo);
  }

  //
  // Servo in torque overload condition and goal direction is in the same
  // direction as the load, which would exasperate the condition.
  //
  else if( pVServo->m_bOverTorqueCond &&
       (pVServo->getGoalDir(pServo) != DYNA_GET_DIR(pVServo->m_fTorqueOut)) )
  {
    //stopPosCtl(pServo, pVServo);
  }

  //
  // PID control servo to goal position.
  //
  else
  {
    execPosCtl(pServo, pVServo);
  }

  pVServo->unlock();
}

void DynaBgThread::execTorqueCtl(DynaServo *pServo, DynaVServo *pVServo)
{
  double      fTorqueAvg;     // filtered load
  uint_t      uOverTorqueTh;  // set over torque condition threshold
  uint_t      uClearTorqueTh; // clear over torque condition threshold
  int         nCurPos;        // current position
  int         sign;           // torque release direction sign 
  int         nGoalSpeed;     // goal speed
  int         nGoalPos;       // goal position

  LOGWARN("BGThread: Executing torque control for servo %d, torque=%.2lf.",
      pServo->GetServoId(), pVServo->m_fTorqueOut);

  pServo->GetSoftTorqueThresholds(uOverTorqueTh, uClearTorqueTh);

  fTorqueAvg = pVServo->m_fTorqueOut;

  if( (uint_t)fabs(fTorqueAvg) > uOverTorqueTh )
  {
    sign = DYNA_GET_DIR(fTorqueAvg);

    nGoalSpeed = DynaVServo::GOAL_SPEED_DFT;

    nCurPos   = pServo->GetOdometer();
    nGoalPos  = nCurPos + sign * 5;

    switch( pServo->GetServoMode() )
    {
      case DYNA_MODE_CONTINUOUS:
        nGoalSpeed = nGoalSpeed * sign;
        pServo->WriteGoalSpeed(nGoalSpeed);
        usleep(1000); // RDK hack
        stopMotion(pServo);
        break;

      case DYNA_MODE_SERVO:
      default:
        pServo->WriteGoalSpeed(nGoalSpeed);
        pServo->WriteGoalPos(nGoalPos);
        break;
    }

    pVServo->m_bOverTorqueCtl = true;

    BG_TRACE(BG_TRACE_FILTER_TORQCTL, pServo->GetServoId(), "TORQUECTL",
      "cur_load=%d, filtered_load=%.2lf, cur_od_pos=%d, "
      "goal_speed=%d, goal_od_pos=%d\n",
              nCurLoad, fTorqueAvg, nCurPos, nGoalSpeed, nGoalPos);
  }

  // stop servo, but don't touch current goals
  else
  {
    pVServo->m_bOverTorqueCtl = false;
    pServo->WriteGoalSpeed(DYNA_SPEED_CONT_STOP);
  }
}

void DynaBgThread::execPosCtl(DynaServo *pServo, DynaVServo *pVServo)
{
  int                   nOdGoalPos; // goal position
  int                   nGoalSpeed; // goal speed
  int                   nCurPos;    // current position
  int                   nCurSpeed;  // current speed
  int                   nErr;       // position difference error
  double                dt;         // delta time

  //
  // Once goal position is set, servo mode servos move on there own.
  //
  if( pServo->GetServoMode() == DYNA_MODE_SERVO )
  {
    nGoalSpeed  = pVServo->m_nGoalSpeed;
    nOdGoalPos  = pVServo->m_nOdGoalPos;

    // stop
    if( iabs(nGoalSpeed) == 0 )
    {
      stopPosCtl(pServo, pVServo);
    }

    pVServo->m_eState = DynaVServo::StateIdle;

    return;
  }

  else
  {
    nCurPos = pServo->GetOdometer();

    dt = (double)m_dTSched/1000000.0;

    nGoalSpeed  = (int)pVServo->m_pidPos.Control((double)nCurPos, dt);
    nErr        = (int)pVServo->m_pidPos.GetError();

    if( (iabs(nErr) < pVServo->m_nTolerance) ||
        (nGoalSpeed == DYNA_SPEED_CONT_STOP) )
    {
      stopPosCtl(pServo, pVServo);
    }
    else if( pServo->WriteGoalSpeed(nGoalSpeed) == DYNA_OK )
    {
      BG_TRACE(BG_TRACE_FILTER_POSCTL, pServo->GetServoId(), "MOVE",
                "goalspeed=%d, cur_od_pos=%d, goal_od_pos=%d, diff=%d\n",
                nGoalSpeed,
                nCurPos,
                pVServo->m_nOdGoalPos,
                nErr);
    }
  }
}

void DynaBgThread::stopPosCtl(DynaServo *pServo, DynaVServo *pVServo)
{
  int   nGoalOdPos;

  // stop servo movement
  stopMotion(pServo);

  nGoalOdPos            = pVServo->m_nOdGoalPos;
  pVServo->m_nOdGoalPos = pServo->GetOdometer();
  pVServo->m_eState     = DynaVServo::StateIdle;

  BG_TRACE(BG_TRACE_FILTER_POSCTL,
      pServo->GetServoId(), "STOP",
      "cur_od_pos=%d, goal_od_pos=%d\n",
      pVServo->m_nOdGoalPos, nOdGoalPos);
}

void DynaBgThread::stopMotion(DynaServo *pServo)
{
  int nCurOdPos;

  // stop servo
  switch( pServo->GetServoMode() )
  {
    //
    // In continuous mode: Set the goal speed to zero.
    //
    case DYNA_MODE_CONTINUOUS:
      pServo->WriteGoalSpeed(DYNA_SPEED_CONT_STOP);
      break;

    //
    // In servo mode: Set the goal position to the current position (a 0 speed
    // value sets the speed to the default maximum).
    //
    case DYNA_MODE_SERVO:
    default:
      if( pServo->ReadCurPos(&nCurOdPos) == DYNA_OK )
      {
        pServo->WriteGoalPos(nCurOdPos);
      }
      break;
  }
}

void DynaBgThread::monitorDynamics(DynaServo *pServo, DynaVServo *pVServo)
{
  int         nCurPos;
  int         nCurSpeed;
  int         nCurLoad;
  uint_t      uTorqueAvg;
  uint_t      uOverTorqueTh;
  uint_t      uClearTorqueTh;
  bool        bCurOverCond;

  // read the servo dynamics.
  if( pServo->ReadDynamics(&nCurPos, &nCurSpeed, &nCurLoad) >= 0 )
  {
    uTorqueAvg = (uint_t)fabs(pVServo->filterTorques(nCurLoad));

    pServo->GetSoftTorqueThresholds(uOverTorqueTh, uClearTorqueTh);

    bCurOverCond =  pServo->HasSoftTorqueOverCond();

    // set over torque limit condition
    if( uTorqueAvg > uOverTorqueTh )
    {
      pVServo->m_bOverTorqueCond = true;
      pServo->SetSoftTorqueOverCond(true);
    }

    // clear over torque limit condition
    else if( (uTorqueAvg < uClearTorqueTh) && bCurOverCond )
    {
      pVServo->m_bOverTorqueCond = false;
      pServo->SetSoftTorqueOverCond(false);
    }

    BG_TRACE(BG_TRACE_FILTER_DYNA, pServo->GetServoId(), "DYNAMICS",
        "pos=%d, speed=%d, cur_load=%d, filtered_load=%.2lf, overtorque=%s\n",
        nCurPos, nCurSpeed, nCurLoad, pVServo->m_fTorqueOut,
        (pServo->HasSoftTorqueOverCond()? "true": "false"));
  }
}

void DynaBgThread::monitorHealth(DynaServo *pServo, DynaVServo *pVServo)
{
  uint_t      uAlarms;
  int         nCurLoad;
  uint_t      uCurVolt;
  uint_t      uCurTemp;

  LOGDIAG4("BGThread: Monitoring health for servo %d.", pServo->GetServoId());

  pServo->ReadHealth(&uAlarms, &nCurLoad, &uCurVolt, &uCurTemp);

  //pVServo->filterTorques(nCurLoad);

  BG_TRACE(BG_TRACE_FILTER_HEALTH, pServo->GetServoId(), "HEALTH",
        "load=%d, voltage=%u, temperature=%u, alarms=0x%02x\n",
        nCurLoad, uCurVolt, uCurTemp, uAlarms);
}

void *DynaBgThread::bgThread(void *pArg)
{
  DynaBgThread  *pThis = (DynaBgThread *)pArg;
  int           rc;

  LOGDIAG3("Dynamixel background thread created.");

  //
  // Loop forever until exit
  //
  while( (pThis->m_eState != BgThreadStateExit) )
  {
    switch( pThis->m_eState )
    {
      case BgThreadStateReady:
      case BgThreadStatePaused:
        pThis->readyWait();
        LOGDIAG3("New state: %d: t_sched=%lu",
                  pThis->m_eState, pThis->m_TSched);
        break;
      case BgThreadStateRunning:
        if( pThis->m_uNumServos > 0 )
        {
          pThis->sched(pThis->m_TSched);
          if( pThis->m_eState == BgThreadStateRunning )
          {
            pThis->exec();
          }
        }
        else // nothing to do
        {
          // slowly
          pThis->sched(500000);
        }
        break;
      case BgThreadStateExit:
        break;
      default:
        LOGERROR("%d: Unexpected dynamixel background thread state.",
            pThis->m_eState);
        pThis->m_eState = BgThreadStateExit;
        break;
    }
  }

  pThis->m_eState = BgThreadStateZombie;

  LOGDIAG3("Dynamixel background thread exited.");

  return NULL;
}

void DynaBgThread::createThread()
{
  int   rc;

  m_eState = BgThreadStateReady;

  rc = pthread_create(&m_thread, NULL, DynaBgThread::bgThread, (void *)this);
 
  if( rc != 0 )
  {
    LOGSYSERROR("pthread_create()");
    m_eState = BgThreadStateZombie;
  }
}

void DynaBgThread::terminateThread()
{
  changeState(BgThreadStateExit);
  pthread_join(m_thread, NULL);
}
