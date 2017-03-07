////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_appkit
//
// File:      Thread.h
//
/*! \file
 *
 * $LastChangedDate: 2015-11-09 17:38:34 -0700 (Mon, 09 Nov 2015) $
 * $Rev: 4195 $
 *
 * \brief Thread base class interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2015-2017  RoadNarrows
 * (http://www.RoadNarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _RNR_THREAD_H
#define _RNR_THREAD_H

#include <sys/types.h>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/appkit/Time.h"

namespace rnr
{

  //----------------------------------------------------------------------------
  // Thread Class
  //----------------------------------------------------------------------------
  
  /*!
   * Thread base class.
   */
  class Thread
  {
  public:
    //
    // Sceduling priority range. Values are mapped to system min,max values.
    //
    // Higher the number, higher priority the system will run the thread.
    //
    static const int ThreadPriorityDft  =  0; ///< default thread attributes
    static const int ThreadPriorityMin  =  1; ///< minimum scheduling priority
    static const int ThreadPriorityMax  = 99; ///< maximum scheduling priority

    static const double ThreadMinHz = 0.001;  ///< once every 1000 seconds

    /*!
     * \brief Kinematics thread states.
     */
    enum ThreadState
    {
      ThreadStateUninit,    ///< thread created but unitialized
      ThreadStateReady,     ///< thread created and blocked, ready to start
      ThreadStateStart,     ///< start to run 
      ThreadStateRunning,   ///< thread running
      ThreadStateExit       ///< thread exiting/exited
    };
  
    /*!
     * \brief Default constructor.
     */
    Thread(const std::string &strThreadName);

    /*!
     * \brief Destructor.
     */
    virtual ~Thread();

    /*!
     * \brief Create the thread.
     *
     * The thread remains blocked in the ready state until runThread() is
     * called.
     *
     * \param nPriority   Thread OS scheduling priority.
     *
     * \par Context:
     * Calling thread.
     *
     * \copydoc doc_return_std
     */
    virtual int createThread(int nPriority);

    /*!
     * \brief Run the thread.
     *
     * \par Valid Current State:
     * \ref ThreadStateReady
     *
     * \par New State:
     * \ref ThreadStateStart
     *
     * \par Context:
     * Calling thread.
     *
     * \param fHZ   Thread run rate (Hertz).
     *
     * \copydoc doc_std_return
     */
    virtual int runThread(const double fHz);

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
     * \brief Calculate thread new full cycle run rate.
     *
     * \param fHZ   Thread run rate (Hertz).
     */
    virtual void setHz(const double fHz);

    /*!
     * \brief Get assigned thread name.
     * 
     * \return Thread string name.
     */
    std::string getThreadName() const
    {
      return m_strThreadName;
    }

    /*!
     * \brief Get thread system scheduling priority.
     * 
     * \return Thread priority.
     */
    int getThreadPriority() const
    {
      return m_nPriority;
    }

    /*!
     * \brief Get thread run full cycle rate.
     * 
     * \return Thread hertz.
     */
    double getThreadHz() const
    {
      return m_fHz;
    }

  protected:
    // thread, state, and synchronization
    std::string     m_strThreadName;  ///< thread identifying name
    ThreadState     m_eState;         ///< thread state
    pthread_mutex_t m_mutexSync;      ///< synchonization mutex
    pthread_cond_t  m_condSync;       ///< synchonization condition
    pthread_t       m_thread;         ///< pthread identifier 

    // scheduler
    int       m_nPriority;          ///< thread OS scheduling priority
    double    m_fHz;                ///< thread cycle run rate (Hertz)
    double    m_fTExec;             ///< task execution cycle period (seconds)
    Time      m_tExecPeriod;        ///< task execution period (converted)
    Time      m_tSched;             ///< working scheduler time
    Time      m_tExecLastTimeStamp; ///< start of last execution time stamp
    Time      m_tExecThisTimeStamp; ///< start of this execution time stamp
    int       m_nSlipErrCnt;        ///< slipped error count leaky bucket

    /*!
     * \brief Lock the \h_i2c bus.
     *
     * The lock()/unlock() primitives provide safe multi-threading access.
     *
     * \par Context:
     * Any.
     */
    void lock()
    {
      pthread_mutex_lock(&m_mutexSync);
    }

  
    /*!
     * \brief Unlock the \h_i2c bus.
     *
     * The lock()/unlock() primitives provide safe multi-threading access.
     *
     * \par Context:
     * Any.
     */
    void unlock()
    {
      pthread_mutex_unlock(&m_mutexSync);
    }

    /*!
     * \brief Set real-time priority attributes of of the thread to be created.
     *
     * \param nPriority   Thread priority [1 (lowest) - 99 (highest)].
     * \param [out] attr  Thread attributes.
     */
    void setPriority(int nPriority, pthread_attr_t &attr);
    
    /*!
     * \brief Change the thread state.
     *
     * The thread, if blocked, will be immediately unblocked to examine the 
     * new state.
     *
     * \param eNewState   New state.
     *
     * \par Context:
     * Any.
     */
    void changeState(ThreadState eNewState);
  
    /*!
     * \brief Timed wait until state change or time out.
     *
     * \param tsTimeout   When timeout occurs in absolute time.
     *
     * \par Context:
     * Any.
     */
    void timedWait(const struct timespec &tsTimeout);
    
    /*!
     * \brief Block indefinitely while in the ready state.
     *
     * \par Context:
     * Any.
     */
    virtual void readyBlock();

    /*!
     * \brief Block the thread until the next subcycle task is to be run.
     *
     * \par Context:
     * This thread.
     */
    virtual void schedBlock();

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
     * \brief Execute task(s) within scheduled [sub]cycle.
     *
     * This function is called every cycle in the Running state.
     *
     * A hook for any derived thread class. A simple debug print execution for
     * the base class.
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

    /*!
     * \brief The thread.
     *
     * \param pArg  Thread argument (pointer to this object).
     * 
     * \return Returns NULL on thread exit.
     */
    static void *thread(void *pArg);
  };
  
} // namespace rnr


#endif // _RNR_THREAD_H
