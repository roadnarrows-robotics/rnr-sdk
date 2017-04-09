////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Library:   liblaelaps
//
// File:      laeKin.h
//
/*! \file
 *
 * \brief The Laelaps kinematics and dynamics class interface.
 *
 * The class instance starts a kinematics thread to sense kinematic chain
 * dynamics, control forward geometry kinematics, and monitor servo health.
 *
 * The kinematics includes all physical kinematic chains. The individual chains
 * are controlled by the higher-level interfaces such as MoveIt!
 *
 * The kinematics thread performs several functions:
 * \li Position, velocity, acceleration (future), and torque monitoring.
 * \li Goal motor position and velocity PID control.
 * \li Torque limiting override control.
 * \li Servo health monitoring.
 *
 * \par Kinedynamics Algorithm:
 * \verbatim
 * sense()
 *   for each powertrain
 *     sense dynamics (x reads)
 * react()
 *   for each powertrain
 *     stop motion if necessary (x writes)
 * plan()
 *   for each powertrain
 *     plan motion.
 * act()
 *   sync write (x writes)
 * monitor()
 *   health of motor controller with two motors  (x reads)
 * block wait for next cycle time
 * \endverbatim
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2015-2017. RoadNarrows LLC.\n
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

#ifndef _LAE_KIN_H
#define _LAE_KIN_H

#include <sys/time.h>
#include <time.h>
#include <pthread.h>

#include <vector>
#include <map>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Laelaps/RoboClaw.h"

#include "Laelaps/laelaps.h"
#include "Laelaps/laeUtils.h"
#include "Laelaps/laeDesc.h"
#include "Laelaps/laeTune.h"

#include "Laelaps/laeMotor.h"
#include "Laelaps/laeGpio.h"
#include "Laelaps/laeTraj.h"
#include "Laelaps/laePowertrain.h"
#include "Laelaps/laePlatform.h"

namespace laelaps
{
  //
  // Forward declaration.
  //
  class LaeKinAction;

  // ---------------------------------------------------------------------------
  // LaeKinematics Class
  // ---------------------------------------------------------------------------
  
  /*!
   * \brief Laelaps kinematics class.
   *
   * The LaeKinematics class supports control and monitoring of \h_laelaps
   * powertrains and base kinodynamics.
   */
  class LaeKinematics
  {
  public:
    /*!
     * \brief Default initializer constructor.
     */
    LaeKinematics();
  
    /*!
     * \brief Desctructor.
     */
    virtual ~LaeKinematics();
  
    /*!
     * \brief Open communication with the \h_laelaps motor controllers
     *
     * Motors controller serial interface support multi-drop, so one serial
     * device can support up to 8 motor controllers. A GPIO pin selects the
     * motor controller.
     *
     * \param strDevMotorCtlrs    Motor controllers serial device name.
     * \param nBaudRate           Motor controllers serial baudrate.
     * \param fnEnable            Motor controllers enable power function.
     * \param pEnableArg          Enable power function argument.
     *
     * \copydoc doc_return_std
     */
    int open(const std::string &strDevMotorCtlrs,
             const int nBaudRate,
             int (*fnEnable)(void *, bool) = NULL,
             void *pEnableArg = NULL);

    /*!
     * \brief Close communication.
     *
     * \copydoc doc_return_std
     */
    virtual int close();

    /*!
     * \brief Test if connection is open.
     *
     * \return Returns true or false.
     */
    virtual bool isOpen()
    {
      return m_commMotorCtlrs.isOpen();
    }

    /*!
     * \brief Test if motors are enabled.
     *
     * \return Returns true or false.
     */
    virtual bool isEnabled()
    {
      return m_bIsEnabled;
    }

    /*!
     * \brief Test if motors are powered.
     *
     * \return Returns true or false.
     */
    virtual bool areMotorsPowered()
    {
      return m_bAreMotorsPowered;
    }

    /*!
     * \brief Configure kinematics chains and data from product description.
     *
     * \param desc    Product description.
     *
     * \copydoc doc_return_std
     */
    virtual int configure(const LaeDesc &desc);

    /*!
     * \brief Configure kinematics for operation.
     *
     * \par Context:
     * Calling thread.
     *
     * \param tunes     Laelaps tuning parameters.
     *
     * \copydoc doc_return_std
     */
    virtual int configure(const LaeTunes &tunes);

    /*!
     * \brief Reload configuration tuning parameters.
     *
     * \par Context:
     * Calling thread.
     *
     * \param tunes     Laelaps tuning parameters.
     *
     * \copydoc doc_return_std
     */
    virtual int reload(const LaeTunes &tunes);
  
    /*!
     * \brief Get robot platform kinodynamics.
     *
     * \return Returns reference to robot platform kinodynamics object.
     */
    LaePlatform &getPlatform()
    {
      return m_kinPlatform;
    }

    /*!
     * \brief Get pointer to motor controller by name (key).
     *
     * \param nMotorCtlrId  Motor controller unique identifier.
     *
     * \return
     * On success, the pointer to motor controller object is returned.\n
     * On failure, NULL is returned.
     */
    motor::roboclaw::RoboClaw *getMotorCtlr(const int nMotorCtlrId);

    /*!
     * \brief Get map of all powertrain kinodynamics.
     *
     * \return Returns reference to map of powertrain kinodynamics objects.
     */
    LaeMapPowertrain &getPowertrainMap()
    {
      return m_kinPowertrains;
    }

    /*!
     * \brief Get pointer to powertrain by name (key).
     *
     * \param strName Powertrain name (key).
     *
     * \return
     * On success, the pointer to powertrain kinodynamics object is returned.\n
     * On failure, NULL is returned.
     */
    LaePowertrain *getPowertrain(const std::string &strName);

    /*!
     * \brief Get pointer to powertrain by controller id and motor index.
     *
     * \param nCtlr   Motor controller id.
     * \param nMotor  Motor index within motor controller.
     *
     * \return
     * On success, the pointer to powertrain kinodynamics object is returned.\n
     * On failure, NULL is returned.
     */
    LaePowertrain *getPowertrain(int nCtlr, int nMotor);

    /*!
     * \brief Get pointer to powertrain by motor id.
     *
     * \param nMotorId   Motor id.
     *
     * \return
     * On success, the pointer to powertrain kinodynamics object is returned.\n
     * On failure, NULL is returned.
     */
    LaePowertrain *getPowertrain(int nMotorId);

    /*!
     * \brief Get the current powertrain state.
     *
     * \par Context:
     * Calling thread.
     *
     * \param [in] strName  Powertrain unique name.
     * \param [out] state   Current powertrain state.
     *
     * \copydoc doc_return_std
     */
    virtual int getPowertrainState(const std::string  &strName,
                                   LaePowertrainState &state);

    /*!
     * \brief Test if all motors are stopped.
     *
     * \return Returns true if all are stopped, false otherwise.
     */
    virtual bool isStopped();

    /*!
     * \brief Test if a powertrain motor is stopped.
     *
     * \param strName   Powertrain unique name.
     *
     * \return Returns true if stopped, false otherwise.
     */
    virtual bool isStopped(const std::string &strName);

    /*!
     * \brief Reset all motors' odometers to the current respective encoder
     * positions.
     *
     * \par Context:
     * Calling thread.
     *
     * \return Number of motor odometers reset.
     */
    virtual int resetOdometers();

    /*!
     * \brief Emergency stop the robot.
     *
     * All motors will stop and power to motor controllers disabled.
     *
     * \par Context:
     * Calling thread.
     */
    virtual void estop();
  
    /*!
     * \brief Reset emergency stop condition.
     *
     * \par Context:
     * Calling thread.
     */
    virtual void resetEStop();

    /*!
     * \brief Freeze (stop) all motors at the current position.
     *
     * Synomonous with stop().
     *
     * \par Context:
     * Calling thread.
     *
     * \copydoc doc_return_std
     */
    virtual int freeze()
    {
      return stop();
    }
  
    /*!
     * \brief Release motors.
     *
     * Motors will stop, but are free to roll.
     *
     * \par Context:
     * Calling thread.
     *
     * \copydoc doc_return_std
     */
    virtual int release();
  
    /*!
     * \brief Stop all motors at the current position.
     *
     * The motor brakes, if available, are applied.
     *
     * \note The current RoboClaw motor controllers do not have electrical 
     * braking and the current Laelaps versions do not have mechanical braking.
     *
     * \par Context:
     * Calling thread.
     *
     * \copydoc doc_return_std
     */
    virtual int stop();

    /*!
     * \brief Stop the set of powertrain motors at the current position.
     *
     * \par Context:
     * Calling thread.
     *
     * \param vecName   Vector of unique powertrain names.
     *
     * \return Number of powertrain motors stopped.
     */
    virtual int stop(const std::vector<std::string> &vecNames);
  
    /*!
     * \brief Stop one powertrain motor at the current position.
     *
     * \par Context:
     * Calling thread.
     *
     * \param strName   Powertrain unique name.
     *
     * \copydoc doc_return_std
     */
    virtual int stop(const std::string &strName);
  
    /*!
     * \brief Wait for all motors to stop.
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
     * \brief Set new or updated motor velocity goals.
     *
     * The velocity actions are very simple actions (contrast with a 
     * multi-position task). Therefore, to reduce thread context switching and
     * increase responsiveness, after the new/updated velocity goals are set,
     * the plan and execute steps are immediately called.
     *
     * \par Context:
     * Calling thread.
     *
     * \param velocity  Map of powertrain velocities.\n
     *                  Keys are left_front, right_front, left_rear, and
     *                  right_rear.\n
     *                  Values are wheel shaft angular velocities
     *                  (radians/second).
     *
     * \copydoc doc_std_return
     */
    virtual int setGoalVelocities(const LaeMapVelocity &velocity);

    /*!
     * \brief Set new or updated motor duty cycle goals.
     *
     * The duty cycle actions are very simple actions (contrast with a 
     * multi-position task). Therefore, to reduce thread context switching and
     * increase responsiveness, after the new/updated duty goals are set,
     * the plan and execute steps are immediately called.
     *
     * \par Context:
     * Calling thread.
     *
     * \param duty    Map of powertrain duty cycles.\n
     *                  Keys are left_front, right_front, left_rear, and
     *                  right_rear.\n
     *                  Values are normalized duty cycle [-1.0, 1.0].
     *
     * \copydoc doc_std_return
     */
    virtual int setGoalDutyCycles(const LaeMapDutyCycle &duty);

    /*!
     * \brief Set new or updated robot twist velocity goals.
     *
     * The velocity actions are very simple actions (contrast with a 
     * multi-position task). Therefore, to reduce thread context switching and
     * increase responsiveness, after the new/updated velocity goals are set,
     * the plan and execute steps are immediately called.
     *
     * \par Context:
     * Calling thread.
     *
     * \param fVelLinear  Linear velocity (meters/second).
     * \param fVelAngular Angular velocity (radians/second).
     *
     * \copydoc doc_return_std
     */
    virtual int setGoalTwist(double fVelLinear, double fVelAngular);

    /*!
     * \brief Execute kinematics task(s).
     *
     * \par Context:
     * Kinematic thread.
     */
    virtual void exec();
  
    /*!
     * \brief Monitor kinematics health.
     *
     * \par Context:
     * Kinematics thread.
     *
     * \copydoc doc_return_std
     */
    virtual int monitorHealth();

  protected:
    // state
    bool      m_bIsEnabled;         ///< is [not] enabled
    bool      m_bAreMotorsPowered;  ///< motors are [not] powered
    bool      m_bIsStopped;         ///< all motors are [not] stopped
  
    // motor controllers
#ifdef LAE_DEPRECATED
    LaeMotorCtlrChipSelect          m_csMotorCtlrs;
                                        ///< motor controllers chip select 
    LaeMotorCtlrEnable              m_power;  ///< enable power in
#endif // LAE_DEPRECATED

    int (*m_fnEnableMotorCtlrs)(void *, bool);  ///< enable power function
    void *m_pEnableArg;                         ///< argument to enable function

    motor::roboclaw::RoboClawComm   m_commMotorCtlrs;
                                        ///< serial communication bus
    motor::roboclaw::RoboClaw      *m_pMotorCtlr[LaeNumMotorCtlrs];
                                        ///< RoboClaw motor controllers

    // the kinematics (abstracted) hardware interfaces
    LaePlatform       m_kinPlatform;    ///< robot platform kinematics
    LaeMapPowertrain  m_kinPowertrains; ///< robot powertrain kinematics

    // actions
    LaeKinAction   *m_pAction;        ///< current extended kinematics action

    // mutual exclusion
    pthread_mutex_t m_mutex;          ///< mutex
  
    /*!
     * \brief Lock the shared resource.
     *
     * The lock()/unlock() primitives provide a safe multi-threading.
     *
     * \par Context:
     * Any.
     */
    void lock()
    {
      pthread_mutex_lock(&m_mutex);
    }
  
    /*!
     * \brief Unlock the shared resource.
     *
     * \par Context:
     * Any.
     */
    void unlock()
    {
      pthread_mutex_unlock(&m_mutex);
    }

    /*!
     * \brief Attempt to resynchronize the serial communication between the
     * host and the motor controllers.
     */
    void resyncComm();

    /*!
     * \brief Configure motor controller.
     *
     * \par Context:
     * Calling thread.
     *
     * \param tunes           Laelaps tuning parameters.
     * \param nCtlr           Motor controller id.
     *
     * \copydoc doc_return_std
     */
    virtual int configureMotorController(const LaeTunes &tunes, int nCtlr);

    /*!
     * \brief Configure motor controller battery cutoff voltages.
     *
     * \par Context:
     * Calling thread.
     *
     * \param tunes           Laelaps tuning parameters.
     * \param nCtlr           Motor controller id.
     * \param [out] bNvmSave  Do [not] save configuration in motor contoller's 
     *                        non-volatile memory.
     *
     * \copydoc doc_return_std
     */
    virtual int configMotorCtlrBatteryCutoffs(const LaeTunes &tunes,
                                              int            nCtlr,
                                              bool           &bNvmSave);

    /*!
     * \brief Configure motor controller logic cutoff voltages.
     *
     * \par Context:
     * Calling thread.
     *
     * \param tunes           Laelaps tuning parameters.
     * \param nCtlr           Motor controller id.
     * \param [out] bNvmSave  Do [not] save configuration in motor contoller's 
     *                        non-volatile memory.
     *
     * \copydoc doc_return_std
     */
    virtual int configMotorCtlrLogicCutoffs(const LaeTunes &tunes,
                                            int            nCtlr,
                                            bool           &bNvmSave);

    /*!
     * \brief Configure motor controller encoder modes.
     *
     * \par Context:
     * Calling thread.
     *
     * \param tunes           Laelaps tuning parameters.
     * \param nCtlr           Motor controller id.
     * \param [out] bNvmSave  Do [not] save configuration in motor contoller's 
     *                        non-volatile memory.
     *
     * \copydoc doc_return_std
     */
    virtual int configMotorCtlrEncoderModes(const LaeTunes &tunes,
                                            int            nCtlr,
                                            bool           &bNvmSave);

    /*!
     * \brief Reset all motor controller encoders to zero.
     *
     * \par Context:
     * Calling thread.
     *
     * \param nCtlr Motor controller id.
     *
     * \copydoc doc_return_std
     */
    virtual int resetMotorCtlrEncoders(int nCtlr);

    /*!
     * \brief Configure powertrain pair.
     *
     * \par Context:
     * Calling thread.
     *
     * \param tunes           Laelaps tuning parameters.
     * \param nCtlr           Motor controller id.
     * \param [out] bNvmSave  Do [not] save configuration in motor contoller's 
     *                        non-volatile memory.
     *
     * \copydoc doc_return_std
     */
    virtual int configurePtp(const LaeTunes &tunes, int nCtlr, bool &bNvSave);

    /*!
     * \brief Configure motor velocity PID parameters.
     *
     * \par Context:
     * Calling thread.
     *
     * \param tunes           Laelaps tuning parameters.
     * \param powertrain      Powertrain kinodynmics object.
     * \param [out] bNvmSave  Do [not] save configuration in motor contoller's 
     *                        non-volatile memory.
     *
     * \copydoc doc_return_std
     */
    virtual int configMotorVelocityPid(const LaeTunes &tunes,
                                       LaePowertrain  &powertrain,
                                       bool           &bNvmSave);

    /*!
     * \brief Configure motor maximum ampere limit.
     *
     * \par Context:
     * Calling thread.
     *
     * \param tunes           Laelaps tuning parameters.
     * \param powertrain      Powertrain kinodynmics object.
     * \param strKey          Powertrain key.
     * \param [out] bNvmSave  Do [not] save configuration in motor contoller's 
     *                        non-volatile memory.
     *
     * \copydoc doc_return_std
     */
    virtual int configMotorMaxAmpLimit(const LaeTunes &tunes,
                                       LaePowertrain  &powertrain,
                                       bool           &bNvmSave);

    /*!
     * \brief Save configuration to motor controller's non-volatile memory.
     *
     * \par Context:
     * Calling thread.
     *
     * \param nCtlr   Motor controller id.
     *
     * \copydoc doc_return_std
     */
    virtual int saveConfigToNvm(int nCtlr);

    /*!
     * \brief Sense real-time state.
     *
     * \par Context:
     * Kinematics thread.
     *
     * \copydoc doc_return_std
     */
    virtual int sense();

    /*!
     * \brief Sense real-time kinodynamics.
     *
     * \par Context:
     * Kinematics thread.
     *
     * \copydoc doc_return_std
     */
    virtual int senseDynamics();

    /*!
     * \brief React to any necessary time-critical events.
     *
     * \par Context:
     * Kinematics thread.
     *
     * \copydoc doc_return_std
     */
    virtual int react();

    /*!
     * \brief Plan kinematic action.
     *
     * \par Context:
     * Kinematics thread.
     *
     * \copydoc doc_return_std
     */
    virtual int plan();

    /*!
     * \brief Act on current action.
     *
     * \par Context:
     * Kinematics thread.
     *
     * \copydoc doc_return_std
     */
    virtual int act();

    /*!
     * \brief Enable power to motor controllers.
     */
    virtual void enableMotorCtlrs();

    /*!
     * \brief Disable power to motor controllers.
     */
    virtual void disableMotorCtlrs();

  }; // LaeKinematics


  // ---------------------------------------------------------------------------
  // LaeKinAction Class
  // ---------------------------------------------------------------------------
  
  /*!
   * \brief Laelaps kinematics base action class.
   *
   * An action spans multiple kinematic control cycles. For example, navigating
   * to a goal position.
   *
   * The base class has no action.
   */
  class LaeKinAction
  {
  public:
    /*!
     * \brief Supported kinematic extended actions.
     */
    enum ActionType
    {
      ActionTypeIdle,       ///< idle action
      ActionTypeVelocity,   ///< move by angular velocities
      ActionTypeDutyCycle,  ///< move by motor duty cycles
      ActionTypeTwist,      ///< move robot by twist values
      ActionTypeNavForDist, ///< navigate for distances
      ActionTypeNavToPos    ///< navigate to positions
    };

    /*!
     * \brief Action states.
     */
    enum ActionState
    {
      ActionStateIdle,        ///< no action
      ActionStateUpdate,      ///< update action specific data
      ActionStatePlan,        ///< plan action
      ActionStateExecute,     ///< execute action
      ActionStateTerminated   ///< action terminated
    };

    /*!
     * \brief Default intialization constructor.
     *
     * \param kin       Bound kinemeatics driver.
     * \param eAction   Action type.
     */
    LaeKinAction(LaeKinematics &kin, ActionType eActionType=ActionTypeIdle) :
          m_kin(kin), m_eActionType(eActionType)
    {
      m_eActionState = ActionStateIdle;
    }

    /*!
     * \brief Destructor.
     */
    virtual ~LaeKinAction()
    {
    }

    /*!
     * \brief Archetype constructor to create action type instance.
     *
     * \param kin       Bound kinemeatics driver.
     * \param eAction   Action type to create.
     * 
     * \return Pointer to new allocated instance.
     */
    static LaeKinAction *newAction(LaeKinematics    &kin,
                                   const ActionType eActionType);

    /*!
     * \brief Archetype constructor to replace action type instance.
     *
     * \param kin       Bound kinemeatics driver.
     * \param pAction   Pointer to current action to be terminated and deleted.
     *                  The pointer is no long valid after this function 
     *                  returns.
     * \param eAction   New action type to create.
     * 
     * \return Pointer to new allocated instance.
     */
    static LaeKinAction *replaceAction(LaeKinematics    &kin,
                                       LaeKinAction     *pAction,
                                       const ActionType eNewActionType);

    /*!
     * \brief Plan next execution. 
     *
     * No hardware is touched.
     *
     * \copydoc doc_return_std
     */
    virtual int plan()
    {
      return LAE_OK;
    }

    /*!
     * \brief Execution [sub]action.
     *
     * Hardware is updated as needed.
     *
     * \copydoc doc_return_std
     */
    virtual int execute()
    {
      return LAE_OK;
    }

    /*!
     * \brief Terminate action.
     *
     * Hardware is updated as needed. The robot should be stopped.
     *
     * \copydoc doc_return_std
     */
    virtual int terminate()
    {
      m_eActionState = ActionStateTerminated;
      return LAE_OK;
    }

    /*!
     * \brief Test if action is idle.
     *
     * \return Returns true or false.
     */
    virtual bool isIdle()
    {
      return m_eActionState == ActionStateIdle;
    }

    /*!
     * \brief Test if action requires (re)planning.
     *
     * \return Returns true or false.
     */
    virtual bool isPlanningRequired()
    {
      return m_eActionState == ActionStatePlan;
    }

    /*!
     * \brief Test if more action requires (more) execution.
     *
     * \return Returns true or false.
     */
    virtual bool isExecutionRequired() const
    {
      return m_eActionState == ActionStateExecute;
    }

    /*!
     * \brief Test if action has been terminated.
     *
     * \note An action may not have any more tasks to perform, but has NOT
     * been terminated.
     *
     * \return Returns true or false.
     */
    virtual bool hasTerminated() const
    {
      return m_eActionState == ActionStateTerminated;
    }

    /*!
     * \brief Get the action type for this class.
     *
     * \return Action type enum.
     */
    virtual ActionType getActionType() const
    {
      return m_eActionType;
    }

    /*!
     * \brief Get the action state.
     *
     * \return Action state enum.
     */
    virtual ActionState getActionState() const
    {
      return m_eActionState;
    }

  protected:
    LaeKinematics &m_kin;           ///< bound kinematics driver
    ActionType    m_eActionType;    ///< action type enum
    ActionState   m_eActionState;   ///< action state.

  }; // LaeKinAction


  // ---------------------------------------------------------------------------
  // LaeKinActionVelocity Class
  // ---------------------------------------------------------------------------
  
  /*!
   * \brief Laelaps kinematics velocity action class.
   *
   * A velocity action only sets the velocities of a (sub)set of the powertrain
   * velocities. Powertrains not specified are kept at their current 
   * velocities.
   */
  class LaeKinActionVelocity : public LaeKinAction
  {
  public:
    /*!
     * \brief Default initialization constructor.
     *
     * \param kin       Bound kinemeatics driver.
     */
    LaeKinActionVelocity(LaeKinematics &kin);

    /*!
     * \brief Destructor.
     */
    virtual ~LaeKinActionVelocity()
    {
    }

    /*!
     * \brief Update with new velocities.
     *
     * \param velocity  Map of powertrain velocities.
     *
     * \copydoc doc_return_std
     */
    int update(const LaeMapVelocity &velocity);

    /*!
     * \brief Plan next execution. 
     *
     * If new velocities, the motor quad pulse per second are calculated.
     *
     * \copydoc doc_return_std
     */
    virtual int plan();

    /*!
     * \brief Execution new velocities.
     *
     * All motors with new velocities are written with new speeds.
     *
     * \copydoc doc_return_std
     */
    virtual int execute();

    /*!
     * \brief Terminate action.
     *
     * All motors are stopped.
     *
     * \copydoc doc_return_std
     */
    virtual int terminate();

  protected:
    LaeMapVelocity  m_mapGoalVel; ///< map of goal velocities
    s32_t           m_speed[LaeNumMotorCtlrs][LaeNumMotorsPerCtlr];
                                  ///< target motor speeds (qpps)

    /*!
     * \brief Clear goal.
     */
    void clear();

  }; // class LaeKinActionVelocity


  // ---------------------------------------------------------------------------
  // LaeKinActionDutyCycle Class
  // ---------------------------------------------------------------------------
  
  /*!
   * \brief Laelaps kinematics duty cycle action class.
   *
   * A velocity action only sets the duty cycles of a (sub)set of the powertrain
   * motors. Powertrains not specified are kept at their current duty cycles.
   */
  class LaeKinActionDutyCycle : public LaeKinAction
  {
  public:
    /*!
     * \brief Default initialization constructor.
     *
     * \param kin       Bound kinemeatics driver.
     */
    LaeKinActionDutyCycle(LaeKinematics &kin);

    /*!
     * \brief Destructor.
     */
    virtual ~LaeKinActionDutyCycle()
    {
    }

    /*!
     * \brief Update with new velocities.
     *
     * \param velocity  Map of powertrain velocities.
     *
     * \copydoc doc_return_std
     */
    int update(const LaeMapDutyCycle &dutycycle);

    /*!
     * \brief Execution new velocities.
     *
     * All motors with new velocities are written with new speeds.
     *
     * \copydoc doc_return_std
     */
    virtual int execute();

    /*!
     * \brief Terminate action.
     *
     * All motors are stopped.
     *
     * \copydoc doc_return_std
     */
    virtual int terminate();

  protected:
    LaeMapDutyCycle m_mapGoalDutyCycle; ///< map of goal duty cycles

    /*!
     * \brief Clear goal.
     */
    void clear();

  }; // class LaeKinActionDutyCycle


  // ---------------------------------------------------------------------------
  // LaeKinActionTwist Class
  // ---------------------------------------------------------------------------
  
  /*!
   * \brief Laelaps kinematics velocity action class.
   *
   * A velocity action only sets the velocities of a (sub)set of the powertrain
   * velocities. Powertrains not specified are kept at their current 
   * velocities.
   */
  class LaeKinActionTwist : public LaeKinAction
  {
  public:
    /*!
     * \brief Default initialization constructor.
     *
     * \param kin       Bound kinemeatics driver.
     */
    LaeKinActionTwist(LaeKinematics &kin);

    /*!
     * \brief Destructor.
     */
    virtual ~LaeKinActionTwist()
    {
    }

    /*!
     * \brief Update with new velocities.
     *
     * \param fVelLinear  Linear velocity (meters/second).
     * \param fVelAngular Angular velocity (radians/second).
     *
     * \copydoc doc_return_std
     */
    int update(double fVelLinear, double fVelAngular);

    /*!
     * \brief Plan next execution. 
     *
     * If new velocities, the motor quad pulse per second are calculated.
     *
     * \copydoc doc_return_std
     */
    virtual int plan();

    /*!
     * \brief Execution new velocities.
     *
     * All motors with new velocities are written with new speeds.
     *
     * \copydoc doc_return_std
     */
    virtual int execute();

    /*!
     * \brief Terminate action.
     *
     * All motors are stopped.
     *
     * \copydoc doc_return_std
     */
    virtual int terminate();

  protected:
    double    m_fGoalVelLinear;   ///< goal platform linear velocity
    double    m_fGoalVelAngular;  ///< goal platform angular velocity
    s32_t     m_speed[LaeNumMotorCtlrs][LaeNumMotorsPerCtlr];
                                  ///< target motor speeds (qpps)

    /*!
     * \brief Clear goal.
     */
    void clear();

  }; // class LaeKinActionVelocity

} // namespace laelaps

#endif // _LAE_KIN_H
