////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   libDynamixel
//
// File:      DynaBgThread.h
//
/*! \file
 *
 * $LastChangedDate: 2015-07-08 12:19:29 -0600 (Wed, 08 Jul 2015) $
 * $Rev: 4024 $
 *
 * \brief Dynamixel background thread class declarations.
 *
 * The background thread performs several functions:
 * \li Position control for servos configured in continuous (wheel) mode and
 * provide 360\h_deg position data available. For example: EX-106P and the MX
 * series servos.
 * \li Background monitoring of the dynamics of registered servos. Dynamics
 * data are the current position, rotation speed and direction, and torque.
 * \li Background monitoring of the health of registered servos. Servo health
 * data are current alarms, temperature, voltage, and torque.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011-2015.  RoadNarrows
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

#ifndef _DYNA_BG_THREAD_H
#define _DYNA_BG_THREAD_H

#include <sys/time.h>
#include <pthread.h>

#include <map>
#include <deque>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaError.h"
#include "Dynamixel/DynaPidPos.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaServo.h"
#include "Dynamixel/DynaChain.h"

using namespace std;


// ---------------------------------------------------------------------------
// Virtual Servo Class
// ---------------------------------------------------------------------------

/*!
 * \brief Background Thread Virtual Servo Class
 */
class DynaVServo
{
public:
  /*!
   * \brief Torque sliding window size for low-pass band filtering of input
   * torques.
   */
  static const size_t TORQUE_WIN_SIZE = 10;

  /*!
   * \brief Servo dynamics position control state.
   */
  enum VServoState
  {
    StateIdle,      ///< no active position control
    StateBegin,     ///< start position control 
    StateControl    ///< in poistion control
  };

  static const int GOAL_SPEED_DFT = 100;  ///< default virutal goal speed

  /*
   * Virtual servo control variables.
   */
  VServoState m_eState;           ///< virtual servo state
  DynaPidPos  m_pidPos;           ///< servo position PID
  int         m_nTolerance;       ///< position \h_plusmn tolerance (ticks)
  int         m_nOdGoalPos;       ///< current goal position
  int         m_nGoalSpeed;       ///< current goal speed
  bool        m_bOverTorqueCond;  ///< is [not] in torque overload condition
  bool        m_bOverTorqueCtl;   ///< [not] in torque overload control
  double      m_fTorqueOut;       ///< low-pass filtered torque
  std::deque<double> m_histTorqueIn; ///< recent history of joint torques

  /*!
   * \brief Default constructor.
   */
  DynaVServo();

  /*!
   * \brief Copy constructor.
   *
   * \param src   Source object.
   */
  DynaVServo(const DynaVServo &src);

  /*!
   * \brief Destructor.
   */
  ~DynaVServo()
  {
    pthread_mutex_destroy(&m_mutexSync);
  }

  /*!
   * \brief Assignment operator
   *
   * \param rhs   Right hand side object.
   *
   * \return This object.
   */
  DynaVServo operator=(const DynaVServo &rhs);

  /*!
   * \brief Set position tolerance in encoder ticks.
   *
   * \param pServo    Pointer to servo.
   * \param fTolerance  Tolerance (degrees).
   */
  void setToleranceInTicks(DynaServo *pServo, double fTolerance);

  /*!
   * \brief Apply a low-pass band filter on the sensed torques (loads).
   *
   * The torques are interpreted from the unitless load values read from the
   * servo (and probably non-linear).
   *
   * A simple moving average (SMA) of load values with a window size of
   * \ref TORQUE_WIN_SIZE is used as the filter.
   *
   * \param nServoLoad  New servo load value (raw unitless).
   *
   * \return New torque average (raw unitless).
   */
  double filterTorques(int nServoLoad);
  
  /*!
   * \brief Get the current goal rotation direction.
   *
   * The direct is deterimed by the difference of current odometer value and
   * the goal position.
   *
   * Normally counter-clockwise is positive, clockwise is negative, but the
   * sence can be reversed in the odometer configuration.
   *
   * \param pServer   Pointer to servo.
   *
   * \return 1 or -1.
   */
  int getGoalDir(DynaServo *pServo)
  {
    int nSign = pServo->IsOdometerReversed()? -1: 1;

    return (m_nOdGoalPos-pServo->GetOdometer()) >= 0? nSign: -1*nSign;
  }

  /*!
   * \brief Lock the virtual servo.
   *
   * The calling thread will block while waiting for the mutex to become 
   * available. Once locked, the background thread will block when accessing
   * this vitural servo.
   *
   * The lock()/unlock() primitives provide a safe mechanism to modify the 
   * registered vServo data.
   *
   * \par Context:
   * Any.
   */
  void lock()
  {
    pthread_mutex_lock(&m_mutexSync);
  }

  /*!
   * \brief Unlock the virtual servo.
   *
   * The background thread will be available to run when accessing
   * this vitural servo.
   *
   * \par Context:
   * Any.
   */
  void unlock()
  {
    pthread_mutex_unlock(&m_mutexSync);
  }

protected:
  pthread_mutex_t m_mutexSync;      ///< synchonization mutex
};


// ---------------------------------------------------------------------------
// Dynamixel Background Thread Class
// ---------------------------------------------------------------------------

/*!
 * Dynamixel background thread.
 *
 * The DynaBgThread class supports background control and monitoring of
 * registered servos.
 */
class DynaBgThread
{
public:
  static const double HZ_EXEC_MIN   =  1.0; ///< minimum exec hertz
  static const double HZ_EXEC_DFT   = 50.0; ///< default exec hertz
  static const long   T_EXEC_MIN    = 100;  ///< task exec period min (100usec)
  static const double TOLERANCE_DFT =  1.0; ///< default pos. tolerance (deg)

  /*!
   * \brief Background thread states.
   */
  typedef enum
  {
    BgThreadStateZombie,    ///< zombie instance - no thread exists
    BgThreadStateReady,     ///< thread created and ready to run
    BgThreadStateRunning,   ///< thread running
    BgThreadStatePaused,    ///< thread paused
    BgThreadStateExit       ///< thread exiting
  } BgThreadState;

  /*!
   * \brief Default initializer constructor.
   *
   * \param fHz           Execution hertz.
   * \param fTolerance    Position tolerance (degrees).
   */
  DynaBgThread(double fHz         = HZ_EXEC_DFT,
               double fTolerance  = TOLERANCE_DFT);

  /*!
   * \brief Desctructor.
   */
  virtual ~DynaBgThread();

  /*!
   * \brief Register the Dynamixel servo for control and monitoring.
   *
   * The servo will replace the currently monitored servo or chain.
   *
   * If the thread is running, it should be paused or locked prior to calling
   * this function.
   *
   * The time periods are automatically recalculated.
   *
   * \param pServo    Dynamixel servo. May be NULL to deregister.
   */
  virtual void RegisterServoAgent(DynaServo *pServo);

  /*!
   * \brief Register the Dynamixel chain for control and monitoring.
   *
   * The chain will replace the currently monitored servo or chain.
   *
   * If the thread is running, it should be paused or locked prior to calling
   * this function.
   *
   * The time periods are automatically recalculated.
   *
   * \param pChain    Dynamixel chain. May be NULL to deregister.
   */
  virtual void RegisterChainAgent(DynaChain *pChain);

  /*!
   * \brief Unregister any chain or servo from control and monitoring.
   *
   * If the thread is running, it should be paused or locked prior to calling
   * this function.
   *
   * The time periods are automatically recalculated.
   *
   * \param pChain    Dynamixel chain. May be NULL to deregister.
   */
  virtual void UnregisterAgent();

  /*!
   * \brief Register user-defined callback function.
   *
   * \param fnUserCb    User callback function.
   * \param pUserArg    User callback argument.
   */
  virtual void RegisterUserCallback(void (*fnUserCb)(void*), void *pUserArg)
  {
    m_fnUserCb = fnUserCb;
    m_pUserArg = pUserArg;
  }

  /*!
   * \brief Unregister user-defined callback function.
   */
  virtual void UnregisterUserCallback()
  {
    m_fnUserCb = NULL;
    m_pUserArg = NULL;
  }

  /*!
   * \brief Run the background thread control and monitoring tasks.
   *
   * \par Valid Current States:
   * \ref BgThreadStateReady
   *
   * \par New State:
   * \ref BgThreadStateRunning
   *
   * \par Context:
   * Calling thread.
   *
   * \copydoc doc_std_return
   */
  virtual int Run();

  /*!
   * \brief Stop control and monitoring tasks.
   *
   * Any servos being actively position controlled will be erased from the 
   * position control map.
   *
   * \par Valid Current States:
   * \ref BgThreadStateReady
   * \ref BgThreadStateRunning
   * \ref BgThreadStatePaused
   *
   * \par New State:
   * \ref BgThreadStateReady
   *
   * \par Context:
   * Calling thread.
   *
   * \copydoc doc_std_return
   */
  virtual int Stop();

  /*!
   * \brief Pause control and monitoring tasks.
   *
   * The Pause()/Resume() actions provide a safe mechanism to modify the 
   * registered dynamixel chain and/or alter the time periods. When the thread
   * is resumed, the time periods are automatically recalculated since the mix
   * of servos in the chain could have changed.
   *
   * \par Valid Current States:
   * \ref BgThreadStateRunning
   * \ref BgThreadStatePaused
   *
   * \par New State:
   * \ref BgThreadStatePaused
   *
   * \par Context:
   * Calling thread.
   *
   * \copydoc doc_std_return
   */
  virtual int Pause();

  /*!
   * \brief Resume control and monitoring tasks.
   *
   * \par Valid Current States:
   * \ref BgThreadStatePaused
   *
   * \par New State:
   * \ref BgThreadStateRunning
   *
   * \par Context:
   * Calling thread.
   *
   * \copydoc doc_std_return
   */
  virtual int Resume();

  /*!
   * \brief Get current background thread state.
   *
   * \return DynaBgThread::BgThreadState
   */
  BgThreadState GetCurrentState()
  {
    return m_eState;
  }

  /*!
   * \brief Start controlling servo to rotate it to the given goal position.
   *
   * When the servo attains the given goal position within tolerance, the
   * servo will be stopped. The servo speed to the goal position is controlled
   * externally.
   *
   * The servo is expected to be in continuous mode and to support 306\h_deg
   * position information.
   *
   * \param nServoId    Servo id.
   * \param uGoalPos    Goal position in odometer virtual raw units. 
   * \param pUserArg    Pointer to this class instance.
   *
   * \copydoc doc_return_std
   */
  static int WriteGoalPos(int nServoId, int nOdGoalPos, void *pUserArg);

  static int WriteGoalSpeed(int nServoId, int nGoalSpeed, void *pUserArg);

  static int WriteGoalSpeedPos(int    nServoId,
                               int    nGoalSpeed,
                               int    nGoalPos,
                               void  *pUserArg);

  /*!
   * \brief Set new control and monitoring execution hertz rate.
   *
   * \param fHz       New execution hertz.
   */
  void setHz(double fHz);

  /*!
   * \brief Get control and monitoring hetrz rate.
   *
   * \return Execution hertz.
   */
  double getHz()
  {
    return m_fHz;
  }

  /*!
   * \brief Checks if servo is currently being controlled.
   *
   * \param nServoId    Servo Id.
   *
   * \return Returns true or false.
   */
  bool isServoBeingControlled(int nServoId)
  {
    MapVServo::iterator pos;

    if( (pos = m_mapVServo.find(nServoId)) != m_mapVServo.end() )
    {
      return pos->second.m_eState == DynaVServo::StateControl?  true: false;
    }
    else
    {
      return false;
    }
  }

protected:
  /*!
   * \brief Virtual Servo Associative Map Type.
   */
  typedef map<int,DynaVServo> MapVServo;

  // thread, state, and synchronization
  BgThreadState   m_eState;         ///< thread state
  pthread_mutex_t m_mutexSync;      ///< synchonization mutex
  pthread_cond_t  m_condSync;       ///< synchonization condition
  pthread_t       m_thread;         ///< pthread identifier 

  // the servos
  DynaServo      *m_pServo;         ///< registered dynamixel servo
  DynaChain      *m_pChain;         ///< or registered dynamixel chain
  MapVServo       m_mapVServo;      ///< virtual servo associative map
  uint_t          m_uNumServos;     ///< number of servos to monitor
  double          m_fTolerance;     ///< position \h_plusmn tolerance (degrees)

  // servo/chain agent
  DynaAgent_T     m_agent;          ///< servo/chain agent

  // user-defined registered callback
  void            (*m_fnUserCb)(void*); ///< user callback function
  void            *m_pUserArg;          ///< user callback argument

  // scheduler
  double          m_fHz;            ///< thread execution hertz
  long            m_TExec;          ///< full execution cycle \h_usec period
  struct timeval  m_tvSched;        ///< working scheduler time stamp
  long            m_TSched;         ///< scheduler \h_usec period
  long            m_dTSched;        ///< scheduler delta time

  // iterators
  MapVServo::iterator m_iterDynamics;   ///< dynamics iterator
  MapVServo::iterator m_iterHealth;     ///< health iterator
  int                 m_nHealthServoId; ///< current health monitoring servo id

  /*!
   * \brief Lock the background thread.
   *
   * The calling thread will block while waiting for the mutex to become 
   * available. Once locked, the background thread will block.
   *
   * The lock()/unlock() primitives provide a safe mechanism to modify the 
   * registered vServo data.
   *
   * \par Context:
   * Any.
   */
  void lock()
  {
    pthread_mutex_lock(&m_mutexSync);
  }


  /*!
   * \brief Unlock the background thread.
   *
   * The background thread will be available to run.
   *
   * \par Context:
   * Any.
   */
  void unlock()
  {
    pthread_mutex_unlock(&m_mutexSync);
  }

  /*!
   * \brief Change the background thread state.
   *
   * \param eNewState   New state.
   *
   * \par Context:
   * Calling thread or background thread.
   */
  void changeState(DynaBgThread::BgThreadState eNewState);

  /*!
   * \brief Wait indefinitely in ready state.
   *
   * \par Context:
   * Calling thread or background thread.
   */
  void readyWait();

  /*!
   * \brief Wait until state change or time out.
   *
   * \param lMicroSec   Maximum wait duration (microseconds).
   *
   * \par Context:
   * Calling thread or background thread.
   */
  void timeWait(long lMicroSecs);

  /*!
   * \brief Set position tolerance.
   *
   * The servo is considered at a goal position when it is within \h_plusmn of
   * the tolerance.
   */
  void setTolerance(double fTolerance)
  {
    m_fTolerance = fTolerance > 0.0? fTolerance: 0.0;
  }

  /*!
   * \brief Get registered servo.
   *
   * \param nServoId    Servo id.
   *
   * \return Returns pointer to servo on success. NULL on failure.
   */
  virtual DynaServo *getRegisteredServo(int nServoId);

  /*!
   * \brief Get the next virtual servo.
   *
   * \param [in,out] iter     Iterator at current position.
   * \param [out] nServoId    Servo id.
   *
   * \return Returns pointer to virtual servo on success. NULL on failure.
   */
  virtual DynaVServo *getNextVServo(MapVServo::iterator &iter, int &nServoId)
  {
    if( m_mapVServo.size() == 0 )
    {
      return NULL;
    }

    ++iter;

    if( iter == m_mapVServo.end() )
    {
      iter = m_mapVServo.begin();
    }

    nServoId = iter->first;
    return &(iter->second);
  }
  
  /*!
   * \brief Get the previous virtual servo.
   *
   * \param [in,out] iter     Iterator at current position.
   * \param [out] nServoId    Servo id.
   *
   * \return Returns pointer to virtual servo on success. NULL on failure.
   */
  virtual DynaVServo *getPrevVServo(MapVServo::iterator &iter, int &nServoId)
  {
    if( m_mapVServo.size() == 0 )
    {
      return NULL;
    }

    if( iter == m_mapVServo.begin() )
    {
      iter = m_mapVServo.end();
    }

    --iter;

    nServoId = iter->first;
    return &(iter->second);
  }
  
  /*!
   * \brief Schedule the Dynamixel background thread for the next task(s) to
   * perform.
   *
   * \par Context:
   * Background thread.
   */
  virtual void sched(long lPeriod);

  /*!
   * \brief Execute background tasks.
   *
   * \par Context:
   * Background thread.
   */
  virtual void exec();

  /*!
   * \brief Execute servo dynamics monitoring and control.
   *
   * \param pServo    Pointer to registered servo.
   * \param pVServo   Pointer to associated virtual servo.
   *
   * \par Context:
   * Background thread.
   */
  virtual void execDynamics(DynaServo *pServo, DynaVServo *pVServo);

  /*!
   * \brief Execute servo torque control.
   *
   * \param pServo    Pointer to registered servo.
   * \param pVServo   Pointer to associated virtual servo.
   *
   * \par Context:
   * Background thread.
   */
  virtual void execTorqueCtl(DynaServo *pServo, DynaVServo *pVServo);

  /*!
   * \brief Execute servo position control.
   *
   * \param pServo    Pointer to registered servo.
   * \param pVServo   Pointer to associated virtual servo.
   *
   * \par Context:
   * Background thread.
   */
  virtual void execPosCtl(DynaServo *pServo, DynaVServo *pVServo);

  /*!
   * \brief Stop servo position control.
   *
   * \param pServo    Pointer to registered servo.
   * \param pVServo   Pointer to associated virtual servo.
   *
   * \par Context:
   * Background thread.
   */
  virtual void stopPosCtl(DynaServo *pServo, DynaVServo *pVServo);

  /*!
   * \brief Stop servo motion.
   *
   * \param pServo    Pointer to registered servo.
   */
  virtual void stopMotion(DynaServo *pServo);

  /*!
   * \brief Execute servo dynamics monitoring background task.
   *
   * \param pServo    Pointer to registered servo.
   * \param pVServo   Pointer to associated virtual servo.
   *
   * \par Context:
   * Background thread.
   */
  virtual void monitorDynamics(DynaServo *pServo, DynaVServo *pVServo);

  /*!
   * \brief Execute servo health monitoring background task.
   *
   * \param pServo    Pointer to registered servo.
   * \param pVServo   Pointer to associated virtual servo.
   *
   * \par Context:
   * Background thread.
   */
  virtual void monitorHealth(DynaServo *pServo, DynaVServo *pVServo);

  /*!
   * \brief Dynamixel background thread.
   *
   * \warn Cannot have gui calls in this thread.
   *
   * \param pArg   Thread argument (point to DynaBgThread object).
   * 
   * \return Returns NULL on thread exit.
   */
  static void  *bgThread(void *pArg);

  /*!
   * \brief Create the background thread.
   *
   * \par Context:
   * Calling thread.
   */
  void createThread();
  
  /*!
   * \brief Terminate the background thread.
   *
   * This function does not return until the thread actually terminates.
   *
   * \par Context:
   * Calling thread.
   */
  void terminateThread();
};

#endif // _DYNA_BG_THREAD_H
