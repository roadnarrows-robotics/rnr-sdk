////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Library:   liblaelaps
//
// File:      laeThread.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-09-05 10:00:50 -0700 (Sat, 05 Sep 2015) $
 * $Rev: 4074 $
 *
 * \brief The Laelaps thread base class implementation.
 *
 * \par Copyright
 *   \h_copy 2015-2018. RoadNarrows LLC.\n
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

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Laelaps/laelaps.h"
#include "Laelaps/laeUtils.h"
#include "Laelaps/laeThread.h"

using namespace std;
using namespace laelaps;


// ---------------------------------------------------------------------------
// Local
// ---------------------------------------------------------------------------

/*!
 * \brief Print out timespec value to stderr.
 *
 * \param what  What timespec.
 * \param tx    Timespec.
 */
static void prts(string what, struct timespec ts)
{
  fprintf(stderr, "%s = %ld.%09ld\n", what.c_str(), ts.tv_sec, ts.tv_nsec);
}


// ---------------------------------------------------------------------------
// LaeThread Class
// ---------------------------------------------------------------------------

const double LaeThread::ThreadMinHz = 0.001;  ///< once every 1000 seconds

LaeThread::LaeThread(const string &strThreadName) :
    m_strThreadName(strThreadName)
{
  m_eState      = ThreadStateUninit;
  m_fHz         = ThreadMinHz;

  pthread_mutex_init(&m_mutexSync, NULL);
  pthread_cond_init(&m_condSync, NULL);
}

LaeThread::~LaeThread()
{
  terminateThread();

  pthread_cond_destroy(&m_condSync);
  pthread_mutex_destroy(&m_mutexSync);
}

int LaeThread::createThread(int nPriority)
{
  pthread_attr_t      attr;
  struct sched_param  parm;
  double              f;
  int                 prioMin, prioMax;
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
  // Get the system's minimum and maximum scheduling priorities for the given
  // scheduling policy.
  //
  prioMin = sched_get_priority_min(SCHED_FIFO);
  prioMax = sched_get_priority_max(SCHED_FIFO);

  //
  // Map target priority between system min, max.
  //
  nPriority = cap(nPriority, ThreadPriorityMin, ThreadPriorityMax);
  f = (double)nPriority/(double)(ThreadPriorityMax - ThreadPriorityMin + 1);
  nPriority = (int)(f * (double)(prioMax - prioMin + 1) + 0.5);
  m_nPriority = cap(nPriority, prioMin, prioMax);

  //
  // Set the priority for this thread.
  //
  parm.sched_priority = m_nPriority;
  pthread_attr_setschedparam(&attr, &parm);

  // Create kinematics thread.
  rc = pthread_create(&m_thread, &attr, LaeThread::thread, (void *)this);
 
  // failure to create
  if( rc != 0 )
  {
    LOGSYSERROR("pthread_create()");
    m_eState = ThreadStateExit;
    return -LAE_ECODE_NO_EXEC;
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

  return LAE_OK;
}

int LaeThread::runThread(double fHz)
{
  int       rc;       // return code

  setHz(fHz);

  switch( m_eState )
  {
    case ThreadStateReady:
      changeState(ThreadStateStart);
      rc = LAE_OK;
      break;

    case ThreadStateStart:
    case ThreadStateRunning:
      break;

    case ThreadStateExit:
    default:
      rc = -LAE_ECODE_GEN;
      LOGERROR("%s thread in invalid state %d to run.",
          m_strThreadName.c_str(), m_eState);
      break;
  } 

  return rc;
}

int LaeThread::terminateThread()
{
  if( (m_eState == ThreadStateReady) ||
      (m_eState == ThreadStateStart) ||
      (m_eState == ThreadStateRunning) )
  {
    changeState(ThreadStateExit);
    pthread_join(m_thread, NULL);
  }
  return LAE_OK;
}

void LaeThread::setHz(const double fHz)
{
  lock();

  m_fHz = fHz;

  if( m_fHz < ThreadMinHz )
  {
    m_fHz = ThreadMinHz;
  }

  m_fTExec = 1.0 / m_fHz;

  //fprintf(stderr, "hz=%.2lf, m_fTExec=%lf\n", m_fHz, m_fTExec);

  m_tsExecPeriod.tv_sec  = (long)floor(m_fTExec); 
  m_tsExecPeriod.tv_nsec = (long)fcap(
                                (m_fTExec-floor(m_fTExec)) * (double)BILLION,
                                0.0, (double)(BILLION-1) );

  //prts("execperiod", m_tsExecPeriod);

  m_tsJitter    = m_tsExecPeriod;
  m_tsJitter.tv_nsec += 50000000;   // 5/100th of second

  m_nSlipErrCnt = 0;

  unlock();
}

void LaeThread::changeState(ThreadState eNewState)
{
  m_eState = eNewState;
  pthread_cond_signal(&m_condSync);
}

void LaeThread::timedWait(const struct timespec &tsTimeout)
{
  lock();

  pthread_cond_timedwait(&m_condSync, &m_mutexSync, &tsTimeout);

  unlock();
}
 
void LaeThread::readyBlock()
{
  lock();

  while( m_eState == ThreadStateReady )
  {
    pthread_cond_wait(&m_condSync, &m_mutexSync);
  }

  unlock();
}
  
void LaeThread::schedBlock()
{
  struct timespec tsNow;            // now
  struct timespec tsSlip = {0, 0};  // any slippage

  // now
  clock_gettime(CLOCK_REALTIME, &tsNow);

  //prts("now  ", tsNow);
  //prts("sched", m_tsSched);

  // 
  // Next scheduled execution cycle is in the future. Block wait for the next
  // cycle to begin.
  //
  if( tsNow < m_tsSched )
  {
    timedWait(m_tsSched);                   // block
    clock_gettime(CLOCK_REALTIME, &tsNow);  // now again
  }

  //
  // Determine any slip in task schedule.
  //
  if( tsNow > m_tsSched )
  {
    tsSlip = tsNow - m_tsSched;
  } 

  //
  // Slipped by less than an execution cycles. Try to make up the time.
  //
  if( tsSlip <= m_tsExecPeriod )
  {
    m_tsSched = tsNow + m_tsExecPeriod - tsSlip;  // next scheduled time
    if( m_nSlipErrCnt > 0 )
    {
      --m_nSlipErrCnt;
    }
  }

  //
  // Slipped by at least one full cycle, but within acceptable jitter.
  //
  else if( tsSlip < m_tsJitter )
  {
    m_tsSched = tsNow + m_tsExecPeriod;
  }

  //
  // Slipped badly by at least one full cycle.
  //
  else
  {
    m_tsSched = tsNow + m_tsExecPeriod;

    if( m_nSlipErrCnt < 1000 )
    {
      ++m_nSlipErrCnt;
    }

    // log moderated slippage
    if( m_nSlipErrCnt < 5 )
    {
      LOGWARN("%s thread: "
          "Execution slipped by %ld.%09ld seconds.",
          m_strThreadName.c_str(), tsSlip.tv_sec, tsSlip.tv_nsec);
    }
  }
}

void LaeThread::transToReady()
{
}

void LaeThread::transToRunning()
{
}

void LaeThread::exec()
{
  struct timespec tsDelta;

  tsDelta = m_tsExecThis - m_tsExecLast;

  printf("Thread %s: [%ld.%09ld] delta seconds.\n",
          m_strThreadName.c_str(), tsDelta.tv_sec, tsDelta.tv_nsec);

  fflush(stdout);
}

void LaeThread::transToExit()
{
}

void *LaeThread::thread(void *pArg)
{
  LaeThread *pThis = (LaeThread *)pArg;
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
        clock_gettime(CLOCK_REALTIME, &pThis->m_tsSched);
        pThis->m_tsExecThis = pThis->m_tsSched;
        pThis->m_tsExecLast = pThis->m_tsSched;

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
          pThis->m_tsExecLast = pThis->m_tsExecThis;
          clock_gettime(CLOCK_REALTIME, &pThis->m_tsExecThis);
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
