////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// File:      laeThreadAsync.h
//
/*! \file
 *
 * $LastChangedDate: 2016-01-21 16:50:25 -0700 (Thu, 21 Jan 2016) $
 * $Rev: 4268 $
 *
 * \brief Laelaps asynchronouse thread class interface.
 *
 * Asynchronous threads are created on demand to execute jobs, asynchonous 
 * to the callers. An example is an ROS action server backend.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2015-2016  RoadNarrows
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

#ifndef _LAE_THREAD_ASYNC_H
#define _LAE_THREAD_ASYNC_H

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

/*!
 *  \brief The \h_laelaps namespace encapsulates all \h_laelaps related
 *  constructs.
 */
namespace laelaps
{
  //----------------------------------------------------------------------------
  // LaeAsyncJob Class
  //----------------------------------------------------------------------------
  
  /*!
   * \brief Asynchronous job base class.
   */
  class LaeAsyncJob
  {
  public:
    /*!
     * \brief Job states.
     */
    enum JobState
    {
      JobStateNoJob,      ///< no job
      JobStateCreated,    ///< job created, but not attached to thread
      JobStateReady,      ///< job attached to thread and ready to run
      JobStateRunning,    ///< job running
      JobStateTerminated  ///< job terminated
    };

    /*!
     * \brief Default constructor.
     */
    LaeAsyncJob(const std::string strJobName = "Job");

    /*!
     * \brief Destructor.
     */
    virtual ~LaeAsyncJob();
   
    /*!
     * \brief Get ready to run.
     *
     * Called after job is attached to the thread and is ready to run.
     */
    virtual void getReady();

    /*!
     * \brief Start.
     *
     * Called just prior to the attached thread starts execution.
     */
    virtual void start();

    /*!
     * \brief Run the job.
     *
     * The job can be time sliced, given that it is attached to a thread with
     * a built-in scheduler, or it can run to completion in a single execution
     * block.
     *
     * \copydoc doc_return_std
     */
    virtual int run();

    /*!
     * \brief Terminate the job normally.
     */
    virtual void terminate();

    /*!
     * \brief Abort the job.
     *
     * The abort is called when an error occurs or the thread is asynchronously
     * terminated.
     *
     * \param rc    Job error exit return code.
     */
    virtual void abort(int rc = -LAE_ECODE_INTR);

    /*!
     * \brief Get the current job state.
     *
     * \return State.
     */
    JobState getState()
    {
      return m_eJobState;
    }

    /*!
     * \brief Get the name of the job.
     *
     * \return String.
     */
    std::string getJobName()
    {
      return m_strJobName;
    }

    /*!
     * \brief Test if job has completed or should be aborted.
     *
     * \return Returns true or false.
     */
    bool isDone()
    {
      return m_bIsDone;
    }

    /*!
     * \brief Get job's return code.
     *
     * \return Returns return code.
     */
    int getRc()
    {
      return m_nJobRc;
    }

  protected:
    JobState      m_eJobState;    ///< job state
    std::string   m_strJobName;   ///< job name
    bool          m_bIsDone;      ///< job is [not] done
    int           m_nJobRc;       ///< job return code
  };

  //----------------------------------------------------------------------------
  // LaeThreadAsync Class
  //----------------------------------------------------------------------------
  
  /*!
   * Asynchronous thread class.
   */
  class LaeThreadAsync : public LaeThread
  {
  public:
    static const double       ThreadAsyncPrioDft = 25;  ///< default priority
    static const double       ThreadAsyncHzDft   = 1.0; ///< default run rate
    static const char* const  ThreadAsyncNameDft;       ///< default name

    /*!
     * \brief Default constructor.
     */
    LaeThreadAsync();

    /*!
     * \brief Destructor.
     */
    virtual ~LaeThreadAsync();

    /*!
     * \brief Create the thread.
     *
     * The thread remains blocked in the ready state until runThread() is
     * called.
     *
     * \param pJob        Asynchronous job.
     * \param nPriority   Thread OS scheduling priority.
     *
     * \par Context:
     * Calling thread.
     *
     * \copydoc doc_return_std
     */
    virtual int createThread(LaeAsyncJob *pJob,
                             int nPriority = ThreadAsyncPrioDft);

    /*!
     * \brief Run the thread.
     *
     * \par Valid Current State:
     * \ref ThreadStateReady
     *
     * \par New State:
     * \ref ThreadStateRunning
     *
     * \par Context:
     * Calling thread.
     *
     * \param fHZ   Thread run rate (Hertz).
     *
     * \copydoc doc_std_return
     */
    virtual int runThread(const double fHz = ThreadAsyncHzDft);

    /*!
     * \brief Terminate the thread.
     *
     * This function does not return until the thread actually terminates.
     *
     * \par Context:
     * Calling thread.
     *
     * \copydoc doc_return_std
     */
    virtual int terminateThread();

    /*!
     * \brief Get attached job.
     *
     * \return Returns pointer to job, or NULl if no job.
     */
    LaeAsyncJob *getJob()
    {
      return m_pJob;
    }

  protected:
    LaeAsyncJob *m_pJob;  ///< attached asynchronous job - owned by caller

    /*!
     * \brief Uninitialized to Ready state transition function.
     *
     * This function is called after entering the Ready state but prior to
     * being blocked waiting to be run.
     *
     * A hook for any derived thread class. A no-op for the base class.
     *
     * This function does not execute under the lock/unlock mutex.
     *
     * \par Context:
     * This thread.
     */
    virtual void transToReady();

    /*!
     * \brief Ready to Running state transition function.
     *
     * This function is called after entering the Running state but prior to
     * any run execution.
     *
     * A hook for any derived thread class. A no-op for the base class.
     *
     * This function does not execute under the lock/unlock mutex.
     *
     * \par Context:
     * This thread.
     */
    virtual void transToRunning();

    /*!
     * \brief Execute asynchronous task within scheduled cycle.
     *
     * This function executes under the lock/unlock mutex.
     *
     * \par Context:
     * This thread.
     */
    virtual void exec();

    /*!
     * \brief Any to Exit state transition function.
     *
     * This function is called after entering the Exit state but prior to
     * terminating the thread.
     *
     * A hook for any derived thread class. A no-op for the base class.
     *
     * This function does not execute under the lock/unlock mutex.
     *
     * \par Context:
     * This thread.
     */
    virtual void transToExit();

  }; // class LaeThreadAsync
  
} // namespace laelaps


#endif // _LAE_THREAD_ASYNC_H
