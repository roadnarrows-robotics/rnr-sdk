////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekKin.h
//
/*! \file
 *
 * $LastChangedDate: 2015-05-15 12:50:46 -0600 (Fri, 15 May 2015) $
 * $Rev: 3988 $
 *
 * \brief The Hekateros kinematics and dynamics class interface.
 *
 * The class instance starts a kinematics thread to sense kinematic chain
 * dynamics, control forward geometry kinematics, and monitor servo health.
 *
 * The kinematics includes all physical kinematic chains. The individual chains
 * are controlled by the higher-level interfaces such as MoveIt!
 *
 * The kinematics thread performs several functions:
 * \li Position, velocity, acceleration (future), and torque monitoring.
 * \li Goal joint position and velocity PID control.
 * \li Torque limiting override control.
 * \li Servo health monitoring.
 *
 * There are two alternative kinematics thread execution algorithms.
 *
 * \par Individual Move Execution Cycle
 * \ref HEK_KIN_EXEC_ALG_INDIV
 * \verbatim
 * get next task
 * if task is a joint task
 *   sense()
 *     sense dynamics (3 dynabus reads)
 *   act()
 *     control movement (1+ dynabus writes)
 *     limit torque if necessary (1+ dynabus writes)
 * else if task is to monitor
 *   monitor()
 *     health of one servo (3 dynabus reads)
 * block wait for next task time
 * \endverbatim
 *
 * \par Sync Move Execution Cycle
 * \ref HEK_KIN_EXEC_ALG_SYNC
 * \verbatim
 * sense()
 *   for each joint
 *     sense dynamics (3 dynabus reads)
 * react()
 *   for each joint
 *     stop motion if necessary (1 dynabus write)
 *     limit torque if necessary (1+ dynabus writes)
 * plan()
 *   for each joint
 *     plan motion and add to synchronous write messages
 * act()
 *   sync write (1 or 2 dynabus messages)
 * monitor()
 *   health of one servo (3 dynabus reads)
 * block wait for next cycle time
 * \endverbatim
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
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

#ifndef _HEK_KIN_H
#define _HEK_KIN_H

#include <sys/time.h>
#include <time.h>
#include <pthread.h>

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
#include "Hekateros/hekUtils.h"


namespace hekateros
{
  // ---------------------------------------------------------------------------
  // HekKinematics Class
  // ---------------------------------------------------------------------------
  
  /*!
   * Hekateros kinematics class.
   *
   * The HekKinematics class supports background control and monitoring of
   * registered joints and servos.
   */
  class HekKinematics
  {
  public:
    /*!
     * \brief Kinematics thread states.
     */
    enum ThreadState
    {
      ThreadStateUninit,    ///< thread created but unitialized
      ThreadStateReady,     ///< thread created and ready to run
      ThreadStateRunning,   ///< thread running
      ThreadStateExit       ///< thread exiting/exited
    };
  
    /*!
     * \brief Associative Map of kinematic joints.
     */
    typedef map<std::string, HekKinJoint *> KinChain;
  
    /*!
     * \brief Order list of tasks.
     */
    typedef vector<std::string> TaskList;
  
    /*!
     * \brief Default initializer constructor.
     *
     * \param dynaChain     Bound Dynamixel chain of all motors (servos).
     * \param kinJointDesc  Joint descriptions in kinematic chain to control.
     * \param tunes         Hekateros tuning parameters.
     */
    HekKinematics(DynaChain       &dynaChain,
                  MapRobotJoints  &kinJointDesc,
                  const HekTunes  &tunes);
  
    /*!
     * \brief Desctructor.
     */
    virtual ~HekKinematics();
  
    /*!
     * \brief Build ordered list of tasks to execute.
     *
     * \todo TODO: Currently this list is hard coded to the Hekateros family
     * of arms and end effectors. How to generalize?
     */
    virtual void buildTaskList();

    /*!
     * \brief Reload configuration tuning parameters for all joints.
     *
     * \param tunes     Hekateros tuning parameters.
     */
    virtual void reload(const HekTunes &tunes);

    /*!
     * \brief Get the current instantaneous joint position and velocity.
     *
     * \param [in] strJointName   Joint name.
     * \param [out] fJointCurPos  Current joint position (radians).
     * \param [out] fJointCurVel  Current joint velocity (radians/second).
     */
    virtual void getJointCurPosVel(const std::string &strJointName,
                                   double            &fJointCurPos,
                                   double            &fJointCurVel);

    /*!
     * \brief Get the smoothed (filtered) current joint position and velocity.
     *
     * \param [in] strJointName   Joint name.
     * \param [out] fJointCurPos  Current joint position (radians).
     * \param [out] fJointCurVel  Current joint velocity (radians/second).
     */
    virtual void getFilteredJointCurPosVel(const std::string &strJointName,
                                           double            &fJointCurPos,
                                           double            &fJointCurVel);

    /*!
     * \brief Get the current servo position and speed.
     *
     * \param [in] strJointName     Joint name.
     * \param [out] nServoCurPos    Current servo position (odometer ticks).
     * \param [out] nServoCurSpeed  Current servo velocity (raw unitless).
     */
    void getServoCurPosSpeed(const std::string &strJointName,
                             int               &nServoCurPos,
                             int               &nServoCurSpeed);

    /*!
     * \brief Convert joint position to the equivalent servo position.
     *
     * \param strJointName  Joint name.
     * \param fPos          Joint position (radians).
     *
     * \return Equivalent servo position (odometer ticks).
     */
    int jointPosToServoPos(const std::string &strJointName, const double fPos);

    /*!
     * \brief Convert servo position to the equivalent joint position.
     *
     * \param strJointName  Joint name.
     * \param nOdPos        Servo position (odometer ticks).
     *
     * \return Equivalent joint position (radians).
     */
    double servoPosToJointPos(const std::string &strJointName,
                              const int         nOdPos);

    /*!
     * \brief Test if joint is moving.
     *
     * \param strJointName  Joint name.
     *
     * \return Returns true if stopped, false otherwise.
     */
    virtual bool isStopped(const std::string &strJointName);

    /*!
     * \brief Test if joint is in an over torque condition.
     *
     * \param strJointName  Joint name.
     *
     * \return Returns true if in condition, false otherwise.
     */
    virtual bool hasOverTorqueCondition(const std::string &strJointName);

    /*!
     * \brief Reset all joints' master servos odometers to the current
     * respective encoder positions.
     *
     * \par Context:
     * Calling thread.
     *
     * \return Number of joint servos reset.
     */
    virtual int resetServoOdometersForAllJoints();

    /*!
     * \brief Reset joint's master servo odometer to the current encoder
     * position.
     *
     * \par Context:
     * Calling thread.
     *
     * \param strJointName  Name of joint with master servo to reset.
     *
     * \copydoc doc_return_std
     */
    virtual int resetServoOdometer(const std::string &strJointName);

    /*!
     * \brief Emergency stop the kinematics chain.
     *
     * All servos will stop driving, so chain may fall.
     *
     * \par Context:
     * Calling thread.
     */
    virtual void estop();
  
    /*!
     * \brief Freeze kinematics chain at the current position.
     *
     * The joint servos are still being driven. However, all active joint
     * motion control will cease.
     *
     * \par Context:
     * Calling thread.
     */
    virtual void freeze();
  
    /*!
     * \brief Release kinematics chain.
     *
     * Servos will stop driving, so the chain may fall. This call is assumed to 
     * be under control, so recalibration is not required. Typically, the chain
     * is released during manual repositioning or teaching.
     *
     * All active joint motion control will cease.
     *
     * \par Context:
     * Calling thread.
     */
    virtual void release();
  
    /*!
     * \brief Stop kinematics chain at the current position.
     *
     * The joint servos are still being driven. However, all active joint
     * motion control will cease.
     *
     * \par Context:
     * Calling thread.
     *
     * \return Number of joints stopped.
     */
    virtual int stop();

    /*!
     * \brief Stop the set of joints at the current position.
     *
     * The joint servos are still being driven. However, all active joint
     * motion control will cease.
     *
     * \par Context:
     * Calling thread.
     *
     * \param vecJointName  Vector list of joint names to stop.
     *
     * \return Number of joints stopped.
     */
    virtual int stop(const std::vector<std::string> &vecJointNames);
  
    /*!
     * \brief Stop one joint at the current position.
     *
     * The joint servos are still being driven. However, all active joint
     * motion control will cease.
     *
     * \par Context:
     * Calling thread.
     *
     * \param strJointName  Name of joint.
     *
     * \copydoc doc_return_std
     */
    virtual int stop(const std::string &strJointName);
  
    /*!
     * \brief Wait for all joints to stop.
     *
     * \par Context:
     * Calling thread.
     *
     * \param fSeconds  Maximum number of seconds to wait.
     *
     * \copydoc doc_return_std
     */
    virtual int waitForAllStop(double fSeconds);

    /*!
     * \brief Move kinematic chain through a trajectory point.
     *
     * \par Context:
     * Calling thread.
     *
     * \param trajectoryPoint   Trajectory end point.
     *
     * \return Number of joints with new move initiated.
     */
    virtual int move(HekJointTrajectoryPoint &trajectoryPoint);

    /*!
     * \brief Move single joint.
     *
     * \par Context:
     * Calling thread.
     *
     * \param strJointName    Joint name.
     * \param fJointGoalPos   Joint goal position (radians).
     * \param fJointGoalVel   Joint goal velocity (radians/second).
     *
     * \copydoc doc_std_return
     */
    virtual int move(const std::string &strJointName,
                     const double       fJointGoalPos,
                     const double       fJointGoalVel);

    /*!
     * \brief Set thread run rate and ancillary data.
     *
     * \param fHZ   Thread run rate (Hertz).
     */
    virtual void setHz(const double fHz);

    /*!
     * \brief Run the kinematics thread.
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
    int runThread(const double fHz = HekTuneKinHzDft);
    
    /*!
     * \brief Wait one full cycle.
     *
     * \par Context:
     * Calling thread.
     */
    void waitOneCycle();

  protected:
    // thread, state, and synchronization
    ThreadState     m_eState;         ///< thread state
    pthread_mutex_t m_mutexSync;      ///< synchonization mutex
    pthread_cond_t  m_condSync;       ///< synchonization condition
    pthread_t       m_thread;         ///< pthread identifier 
  
    // the kinematics and dynamics
    DynaChain      &m_dynaChain;      ///< dynamixel chain
    KinChain        m_kinChain;       ///< kinematic chain
    int             m_nNumJoints;     ///< number of joints to control
    int             m_nNumServos;     ///< number of servos to monitor
  
    // scheduler
    double          m_fHz;            ///< thread run rate (Hertz)
    double          m_fTExec;         ///< task execution period (seconds)
    struct timespec m_tsExecPeriod;   ///< task execution period (converted)
    struct timespec m_tsSched;        ///< working scheduler time stamp
  
    // tasks
    bool            m_bIsControlling; ///< [not] actively controlling joints
    TaskList        m_taskList;       ///< list of tasks to exec per cycle
#ifdef HEK_KIN_EXEC_ALG_SYNC
    SyncMoveMsgs    m_msgs;           ///< dynachain synchronous messages
    bool            m_bWaitOneCycle;  ///< caller is [not] waiting 1 exec cycle
#else // HEK_KIN_EXEC_ALG_INDIV
    TaskList::iterator  m_iterTask;       ///< task iterator
    std::string         m_strTaskOneCycle;///< task name one cycle away
#endif // HEK_KIN_EXEC_ALG_SYNC
    int             m_iterHealth;     ///< servo health iterator
    int             m_nHealthServoId; ///< health monitoring servo id
  
    /*!
     * \brief Lock the kinematics thread.
     *
     * The calling thread will block while waiting for the mutex to become 
     * available. Once locked, the kinematics thread will block.
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
     * \brief Unlock the kinematics thread.
     *
     * The kinematics thread will be available to run.
     *
     * \par Context:
     * Any.
     */
    void unlock()
    {
      pthread_mutex_unlock(&m_mutexSync);
    }
  
    /*!
     * \brief Sense the state of all joints.
     *
     * \par Context:
     * Kinematics thread.
     */
    virtual void sense();

    /*!
     * \brief React to any necessary time-critical joint events.
     *
     * \par Context:
     * Kinematics thread.
     */
    virtual void react();

    /*!
     * \brief Sense-react the state of all joints.
     *
     * \par Context:
     * Kinematics thread.
     */
    virtual void sense_react();

    /*!
     * \brief Plan motions for all joints.
     *
     * \par Context:
     * Kinematics thread.
     */
    virtual void plan();

    /*!
     * \brief Move all joints.
     *
     * \par Context:
     * Kinematics thread.
     */
    virtual int act();

    /*!
     * \brief Monitor servo health thread task.
     *
     * \par Context:
     * Kinematics thread.
     */
    virtual void monitorHealth();
  
    /*!
     * \brief Execute kinematics task(s).
     *
     * \par Context:
     * Kinematic thread.
     */
    virtual void exec();
  
    /*!
     * \brief The kinematics thread.
     *
     * \param pArg  Thread argument (point to HekKinematics object).
     * 
     * \return Returns NULL on thread exit.
     */
    static void *thread(void *pArg);
  
    /*!
     * \brief Create the kinematics thread.
     *
     * The thread remains blocked in the ready state until runThread() is
     * called.
     *
     * \par Context:
     * Calling thread.
     */
    void createThread();
  
    /*!
     * \brief Terminate the kinematics thread.
     *
     * This function does not return until the thread actually terminates.
     *
     * \par Context:
     * Calling thread.
     */
    void terminateThread();
  
    /*!
     * \brief Change the kinematics thread state.
     *
     * The thread, if blocked, will be immediately woken up.
     *
     * \param eNewState   New state.
     *
     * \par Context:
     * Calling thread or kinematics thread.
     */
    void changeState(ThreadState eNewState)
    {
      m_eState = eNewState;
      pthread_cond_signal(&m_condSync);
    }
  
    /*!
     * \brief Wait indefinitely while in the ready state.
     *
     * \par Context:
     * Calling thread or kinematics thread.
     */
    void readyWait();

    /*!
     * \brief Timed wait until state change or time out.
     *
     * \param lMicroSec   Maximum wait duration (microseconds).
     *
     * \par Context:
     * Calling thread or kinematics thread.
     */
    void timedWait(const struct timespec &tsTimeout);
    
    /*!
     * \brief Block kinematics thread until the next subcycle task is to be run.
     *
     * \par Context:
     * Kinematics thread.
     */
    void schedWait();
  };

} // namespace hekateros

#endif // _HEK_KIN_H
