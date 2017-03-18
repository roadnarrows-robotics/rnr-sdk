////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Library:   librnr_appkit
//
// File:      Thread.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-11-09 17:38:34 -0700 (Mon, 09 Nov 2015) $
 * $Rev: 4195 $
 *
 * \brief Thread base class implementation.
 *
 * \par Copyright
 *   \h_copy 2015-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
/*
 * @EulaBegin@
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

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/appkit/Time.h"
#include "rnr/appkit/Thread.h"

using namespace std;
using namespace rnr;


// ---------------------------------------------------------------------------
// Local
// ---------------------------------------------------------------------------

/*!
 * \brief Print out timespec value to stderr.
 *
 * \param what  What timespec.
 * \param tx    Timespec.
 */
static void prts(string what, const struct timespec &ts)
{
  fprintf(stderr, "%s = %ld.%09ld\n", what.c_str(), ts.tv_sec, ts.tv_nsec);
}


// ---------------------------------------------------------------------------
// Thread Class
// ---------------------------------------------------------------------------

Thread::Thread(const string &strThreadName) :
    m_strThreadName(strThreadName)
{
  m_eState      = ThreadStateUninit;
  m_fHz         = ThreadMinHz;

  pthread_mutex_init(&m_mutexSync, NULL);
  pthread_cond_init(&m_condSync, NULL);
}

Thread::~Thread()
{
  terminateThread();

  pthread_cond_destroy(&m_condSync);
  pthread_mutex_destroy(&m_mutexSync);
}

int Thread::createThread(int nPriority)
{
  pthread_attr_t      attr;
  int                 rc;

  //
  // Create thread with default attributes.
  //
  if( nPriority == ThreadPriorityDft )
  {
    rc = pthread_create(&m_thread, NULL, Thread::thread, (void *)this);
  }

  //
  // Create thread with a real-time priority.
  //
  else
  {
    setPriority(nPriority, attr);
    rc = pthread_create(&m_thread, &attr, Thread::thread, (void *)this);
  }

  // failure to create
  if( rc != 0 )
  {
    LOGSYSERROR("pthread_create()");
    m_eState = ThreadStateExit;
    return RC_ERROR;
  }

  //
  // Wait for thread to initialize.
  //
  lock();

  while( m_eState == ThreadStateUninit )
  {
    pthread_cond_wait(&m_condSync, &m_mutexSync);
  }

  unlock();

  return OK;
}

void Thread::setPriority(int nPriority, pthread_attr_t &attr)
{
  struct sched_param  parm;
  int                 prioMin, prioMax;
  double              f;

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
  // Get the system's minimum and maximum scheduling priorities for the given
  // scheduling policy.
  //
  prioMin = sched_get_priority_min(SCHED_FIFO);
  prioMax = sched_get_priority_max(SCHED_FIFO);

  //
  // Map target priority between system min, max.
  //
  if( nPriority < ThreadPriorityMin )
  {
    nPriority = ThreadPriorityMin;
  }
  else if( nPriority > ThreadPriorityMax )
  {
    nPriority = ThreadPriorityMax;
  }

  f = (double)nPriority/(double)(ThreadPriorityMax - ThreadPriorityMin + 1);

  nPriority = (int)(f * (double)(prioMax - prioMin + 1) + 0.5);

  if( nPriority < prioMin )
  {
    nPriority = prioMin;
  }
  else if( nPriority > prioMax )
  {
    nPriority = prioMax;
  }

  //
  // Set the priority for this thread.
  //
  parm.sched_priority = m_nPriority;
  pthread_attr_setschedparam(&attr, &parm);
}

int Thread::runThread(double fHz)
{
  int       rc;       // return code

  setHz(fHz);

  switch( m_eState )
  {
    case ThreadStateReady:
      changeState(ThreadStateStart);
      rc = OK;
      break;

    case ThreadStateStart:
    case ThreadStateRunning:
      break;

    case ThreadStateExit:
    default:
      rc = RC_ERROR;
      LOGERROR("%s thread in invalid state %d to run.",
          m_strThreadName.c_str(), m_eState);
      break;
  } 

  return rc;
}

int Thread::terminateThread()
{
  if( (m_eState == ThreadStateReady) ||
      (m_eState == ThreadStateStart) ||
      (m_eState == ThreadStateRunning) )
  {
    changeState(ThreadStateExit);
    pthread_join(m_thread, NULL);
  }
  return OK;
}

void Thread::setHz(const double fHz)
{
  lock();

  m_fHz = fHz;

  if( m_fHz < ThreadMinHz )
  {
    m_fHz = ThreadMinHz;
  }

  m_fTExec = 1.0 / m_fHz;

  m_tExecPeriod = m_fTExec;

  m_nSlipErrCnt = 0;

  //fprintf(stderr, "hz=%.2lf, m_fTExec=%lf\n", m_fHz, m_fTExec);
  //fprintf(stderr, "ExecPeriod=%lf\n", m_tExecPeriod.t());

  unlock();
}

void Thread::changeState(ThreadState eNewState)
{
  m_eState = eNewState;
  pthread_cond_signal(&m_condSync);
}

void Thread::timedWait(const struct timespec &tsTimeout)
{
  lock();

  pthread_cond_timedwait(&m_condSync, &m_mutexSync, &tsTimeout);

  unlock();
}
 
void Thread::readyBlock()
{
  lock();

  while( m_eState == ThreadStateReady )
  {
    pthread_cond_wait(&m_condSync, &m_mutexSync);
  }

  unlock();
}
  
void Thread::schedBlock()
{
  Time tNow;      // now
  Time tSlip;     // any slippage

  // mark now
  tNow.markNow();

  // 
  // Next scheduled execution cycle is in the future. Block wait for the next
  // cycle to begin.
  //
  if( tNow < m_tSched )
  {
    timedWait(m_tSched.ts());         // block
    tNow.markNow();                   // new now
    m_tSched = tNow + m_tExecPeriod;  // next scheduled time
    
    // leak
    if( m_nSlipErrCnt > 0 )
    {
      --m_nSlipErrCnt;
    }
  }

  //
  // Scheduled execution cycle is now.
  //
  else if( tNow == m_tSched )
  {
    m_tSched = tNow + m_tExecPeriod;     // next scheduled time

    if( m_nSlipErrCnt > 0 )
    {
      --m_nSlipErrCnt;
    }
  }

  //
  // Scheduled execution cycle slipped.
  //
  else
  {
    tSlip = tNow - m_tSched;

    //
    // Slipped by less than two execution cycles. Try to make up the time.
    //
    if( tSlip < m_tExecPeriod )
    {
      m_tSched = tNow + m_tExecPeriod - tSlip;  // next scheduled time
    }

    //
    // Slipped a bunch.
    //
    else
    {
      m_tSched = tNow + m_tExecPeriod;

      if( m_nSlipErrCnt < 1000 )
      {
        ++m_nSlipErrCnt;
      }

      // log moderated slippage
      if( m_nSlipErrCnt < 5 )
      {
        LOGWARN("%s thread: "
          "Execution slipped by %ld.%09ld seconds.",
          m_strThreadName.c_str(), tSlip.ts().tv_sec, tSlip.ts().tv_nsec);
      }
    }
  }
}

void Thread::transToReady()
{
}

void Thread::transToRunning()
{
}

void Thread::exec()
{
  Time tDelta;

  tDelta = m_tExecThisTimeStamp - m_tExecLastTimeStamp;

  //fprintf(stderr, "delta=%lf, this=%lf, last=%lf\n",
  //    tDelta.t(), m_tExecThisTimeStamp.t(), m_tExecLastTimeStamp.t());

  printf("Thread %s: [%ld.%09ld] delta seconds.\n",
          m_strThreadName.c_str(), tDelta.ts().tv_sec, tDelta.ts().tv_nsec);

  fflush(stdout);
}

void Thread::transToExit()
{
}

void *Thread::thread(void *pArg)
{
  Thread *pThis = (Thread *)pArg;
  int           oldstate;
  int           rc;

  pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, &oldstate);
  pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, &oldstate);

  // Change state and signal calling thread that created this thread.
  pThis->changeState(ThreadStateReady);

  // derived thread specific Ready state transition function
  pThis->transToReady();

  LOGDIAG3("%s thread created at priority %d - ready to run.",
      pThis->m_strThreadName.c_str(), pThis->m_nPriority);

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
        pThis->readyBlock();
        break;

      //
      // Start running thread to execute task.
      //
      case ThreadStateStart:
        LOGDIAG3("%s thread started - running at %.3lf Hz.",
                pThis->m_strThreadName.c_str(), pThis->m_fHz);
 
        // force immediately execution of first task
        pThis->m_tSched.markNow();

        pThis->m_tExecThisTimeStamp = pThis->m_tSched;
        pThis->m_tExecLastTimeStamp = pThis->m_tSched;

        pThis->changeState(ThreadStateRunning);

        // derived thread specific Running state transition function
        pThis->transToRunning();
        break;

      //
      // Run wait,execute subcycle
      //
      case ThreadStateRunning:
        pThis->schedBlock();
        if( pThis->m_eState == ThreadStateRunning )
        {
          pThis->m_tExecLastTimeStamp = pThis->m_tExecThisTimeStamp;
          pThis->m_tExecThisTimeStamp.markNow();

          pThis->lock();
          pThis->exec();
          pThis->unlock();
        }
        break;

      //
      // Exit
      //
      case ThreadStateExit:
        break;

      default:
        LOGERROR("%d: Unexpected thread state.", pThis->m_eState);
        pThis->m_eState = ThreadStateExit;
        break;
    }
  }

  pThis->transToExit();

  LOGDIAG3("%s thread terminated.", pThis->m_strThreadName.c_str());

  return NULL;
}
