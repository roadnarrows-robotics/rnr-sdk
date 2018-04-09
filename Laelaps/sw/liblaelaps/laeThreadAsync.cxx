////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// File:      laeThreadAsync.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-10-19 10:38:52 -0600 (Mon, 19 Oct 2015) $
 * $Rev: 4153 $
 *
 * \brief Laelaps asynchronous thread class implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
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

#include <sys/types.h>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Laelaps/laelaps.h"
#include "Laelaps/laeUtils.h"

#include "Laelaps/laeThread.h"
#include "Laelaps/laeThreadAsync.h"

using namespace std;
using namespace laelaps;

//------------------------------------------------------------------------------
// LaeAsyncJob Class
//------------------------------------------------------------------------------
 
LaeAsyncJob::LaeAsyncJob(const string strJobName) : m_strJobName(strJobName)
{
  m_eJobState = JobStateCreated;
  m_bIsDone   = false;
  m_nJobRc    = LAE_OK;
}

LaeAsyncJob::~LaeAsyncJob()
{
}

void LaeAsyncJob::getReady()
{
  m_eJobState = JobStateReady;
}

void LaeAsyncJob::start()
{
  m_eJobState = JobStateRunning;
}

int LaeAsyncJob::run()
{
  m_nJobRc = -LAE_ECODE_NO_EXEC;

  return m_nJobRc;
}

void LaeAsyncJob::terminate()
{
  m_eJobState = JobStateTerminated;
  m_bIsDone   = true;
}

void LaeAsyncJob::abort(int rc)
{
  m_eJobState = JobStateTerminated;
  m_bIsDone   = true;
  m_nJobRc    = rc;
}


//------------------------------------------------------------------------------
// LaeThreadAsync Class
//------------------------------------------------------------------------------

const double       LaeThreadAsync::ThreadAsyncPrioDft = 25;
const double       LaeThreadAsync::ThreadAsyncHzDft   = 1.0;
const char* const  LaeThreadAsync::ThreadAsyncNameDft = "Async";

LaeThreadAsync::LaeThreadAsync() : LaeThread(ThreadAsyncNameDft)
{
  m_pJob = NULL;
}

LaeThreadAsync::~LaeThreadAsync()
{
  if( m_pJob != NULL )
  {
    if( !m_pJob->isDone() )
    {
      m_pJob->abort();
    }
  }
}

int LaeThreadAsync::createThread(LaeAsyncJob *pJob, int nPriority)
{
  int   rc;

  if( m_pJob == NULL )
  {
    LOGERROR("Creating %s thread with no job.", m_strThreadName.c_str());
    return -LAE_ECODE_BAD_OP;
  }

  m_pJob          = pJob;
  m_strThreadName = m_pJob->getJobName();

  if( (rc = LaeThread::createThread(nPriority)) != LAE_OK )
  {
    m_pJob->abort(rc);
    m_strThreadName = ThreadAsyncNameDft;
  }

  return rc;
}

int LaeThreadAsync::runThread(const double fHz)
{
  if( m_pJob == NULL )
  {
    LOGERROR("Running %s thread with no job.", m_strThreadName.c_str());
    return -LAE_ECODE_BAD_OP;
  }

  return LaeThread::runThread(fHz);
}

int LaeThreadAsync::terminateThread()
{
  int   rc;

  if( m_pJob != NULL )
  {
    if( !m_pJob->isDone() )
    {
      m_pJob->abort();
    }
  }

  rc = LaeThread::terminateThread();

  m_strThreadName = ThreadAsyncNameDft;

  return rc;
}

void LaeThreadAsync::transToReady()
{
  if( m_pJob != NULL )
  {
    m_pJob->getReady();
  }
}

void LaeThreadAsync::transToRunning()
{
  if( m_pJob != NULL )
  {
    m_pJob->start();
  }
}

void LaeThreadAsync::exec()
{
  int   rc;

  //
  // No job, the thread should not be running.
  //
  if( m_pJob == NULL )
  {
    changeState(ThreadStateExit);
  }

  //
  // Run (a slice) of the job.
  //
  rc = m_pJob->run();

  //
  // Failure - abort.
  //
  if( rc < 0 )
  {
    m_pJob->abort(rc);
    changeState(ThreadStateExit);
  }

  //
  // Normal completion - terminate.
  //
  else if( m_pJob->isDone() )
  {
    m_pJob->terminate();
    changeState(ThreadStateExit);
  }
}

void LaeThreadAsync::transToExit()
{
  if( m_pJob != NULL )
  {
    if( !m_pJob->isDone() )
    {
      m_pJob->abort();
    }
  }

  m_strThreadName = ThreadAsyncNameDft;
}
