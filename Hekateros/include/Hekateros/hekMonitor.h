////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekRobot.h
//
/*! \file
 *
 * $LastChangedDate: 2015-05-01 14:30:09 -0600 (Fri, 01 May 2015) $
 * $Rev: 3974 $
 *
 * \brief Hekateros Monitor Class interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2014-2015  RoadNarrows
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

#ifndef _HEK_MONITOR_H
#define _HEK_MONITOR_H

/*!
 * \brief Define if Heketeros has RN system board with I/O expander.
 *
 * \note The original system board is deprecated. However a newer version may
 * be resurected at a future time, so keep the code.
 */
#undef HEK_HAS_SYS_BOARD

/*!
 * \brief Define if Hekateros has Arduino compatible I/O device.
 */
#define HEK_HAS_ARDUINO

#include <sys/types.h>
#include <sys/time.h>
#include <pthread.h>

#include "rnr/rnrconfig.h"
#include "rnr/units.h"

#ifdef HEK_HAS_SYS_BOARD
#include "rnr/i2c.h"
#endif // HEK_HAS_SYS_BOARD

#include <string>
#include <map>

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaServo.h"
#include "Dynamixel/DynaChain.h"
#include "Dynamixel/DynaBgThread.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekUtils.h"
#include "Hekateros/hekDesc.h"
#include "Hekateros/hekJoint.h"
#include "Hekateros/hekKin.h"
#include "Hekateros/hekState.h"
#include "Hekateros/hekUno.h"
#include "Hekateros/hekSysBoard.h"

namespace hekateros
{
  /*!
   * \brief \h_hek power, joint limits, and alarm monitoring class.
   */
  class HekMonitor
  { 
  public:
    /*!
     * \brief Map of optical limit switches for all joints in all kinematic
     * chains.
     *
     * \termblock
     * \term key: \termdata i/o expander bit mask \endterm
     * \term mapped type: \termdata joint optical limit data \endterm
     * \endtermblock
     */
    typedef map<int, HekJointOptical> MapOpticalLimits;

    /*!
     * \brief \h_hek internal monitor states.
     */
    enum MonStateId
    {
      MonStateIdInit,         ///< power on and/or (re)initializing
      MonStateIdRunNormal,    ///< running in normal mode
      MonStateIdRunAlarmed,   ///< running in alarmed mode
      MonStateIdHalting,      ///< halting
      MonStateIdHalted        ///< halted
    };

    /*!
     * \brief monitor thread states.
     */
    enum MonThStateId
    {
      MonThStateIdInit,     ///< S0
      MonThStateIdIdle,     ///< no work
      MonThStateIdRun,      ///< process thread work
      MonThStateIdExit      ///< exit/no thread
    };

    /*!
     * \brief Default constructor.
     */
    HekMonitor();

    /*!
     * \brief Destructor.
     */
    virtual ~HekMonitor();

    /*!
     * \brief Open resources to monitor hardware.
     *
     * Creates monitor thread.
     *
     * \param uHekHwVer         \h_hek hardware version.  
     * \param strDevArduino     Arduino serial device name.
     * \param nBaudRateArduino  Arduino baud rate.
     *
     * \copydoc doc_return_std
     */
    int open(uint_t             uHekHwVer,
             const std::string &strDevArduino     = HekDevArduino,
             int                nBaudRateArduino  = HekBaudRateArduino);

    /*!
     * \brief Close resources to monitor hardware.
     *
     * Cancels monitor thread.
     *
     * \copydoc doc_return_std
     */
    int close();


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Monitoring Thread Control
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Start monitoring added objects.
     *
     * \param fHz   Monitoring hertz.
     *
     * \copydoc doc_return_std
     */
    int start(double fHz=30.0);

    /*!
     * \brief Stop monitoring.
     *
     * \copydoc doc_return_std
     */
    int stop();


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Objects to Monitor
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Add servo chain to monitor for alarms and health.
     *
     * \warn Only add when monitor thread does not exist or is in the idle
     * state.
     *
     * \param pDynaChain  Pointer to dynamixel chain.
     */
    void addServoChainToMonitor(DynaChain *pDynaChain)
    {
      m_pDynaChain = pDynaChain;
    }

    /*!
     * \brief Add any joint limit switches to quick look-up map.
     *
     * \warn Only add when monitor thread does not exist or is in the idle
     * state.
     *
     * \param [in] pSpecJoint   Pointer to joint spcecification.
     * \param [in] pJoint       Pointer to associated master servo joint.
     *
     * \copydoc doc_return_std
     */
    int addJointLimitsToMonitor(HekSpecJoint_T *pSpecJoint,
                                HekRobotJoint  *pJoint);

    /*!
     * \brief Add kinematics for safety control.
     *
     * \warn Only add when monitor thread does not exist or is in the idle
     * state.
     *
     * \param pKin  Pointer to kinematics.
     */
    void addKinematicsToMonitor(HekKinematics *pKin)
    {
      m_pKin = pKin;
    }


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Monitoring State & Conditions
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Get the current monitor state.
     *
     * \return \ref MonStateId enum value.
     */
    MonStateId getMonState()
    {
      return m_eMonState;
    }

    /*!
     * \brief Mark alarm condition.
     *
     * \param bAlarm  \h_hek is [not] alarmed.
     */
    void markAlarmCond(bool bAlarmedCond);

    /*!
     * \brief Mark emergency stop condition.
     *
     * \param bEStopCond  \h_hek is [not] in emergency stop condition.  
     */
    void markEStopCond(bool bEStopCond);

    /*!
     * \brief Mark motors are powered condition.
     *
     * \param bPoweredCond  \h_hek servo motors are [not] powered.
     */
    void markPoweredCond(bool bPoweredCond)
    {
      m_bPoweredCond = bPoweredCond;
    }

    
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Basic I/O
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Read firmware version.
     *
     * \return Version number
     */
    byte_t readFwVersion();

    /*!
     * \brief Read joint limit switch states.
     *
     * \return Limit switch bit values.
     */
    byte_t readLimitSwitches();

    /*!
     * \brief Read End Effector GPIO states.
     *
     * \return GPIO bit values.
     */
    byte_t readEEGpio();

    /*!
     * \brief Read one End Effector GPIO pin state.
     *
     * \param byPin   GPIO pin.
     *
     * \copydoc doc_return_std
     */
    byte_t readEEGpioPin(byte_t byPin);

    /*!
     * \brief Write state to one End Effector GPIO pin.
     *
     * \param byPin   GPIO pin.
     * \param byVal   GPIO pin new value. 0 or 1.
     *
     * \copydoc doc_return_std
     */
    int writeEEGpioPin(byte_t byPin, byte_t byVal);

    /*!
     * \brief Configure End Effector GPIO pin direction.
     *
     * \param byPin   GPIO pin.
     * \param cDir    'i' == input, 'o' == output.
     *
     * \copydoc doc_return_std
     */
    int configEEGpioPin(byte_t byPin, char cDir);

    /*!
     * \brief Write alarm LED.
     *
     * \param bState  On/off state of LED.
     */
    void writeAlarmLED(bool bState);

    /*!
     * \brief Write CPU status LED.
     *
     * \param bState  On/off state of LED.
     */
    void writeStatusLED(bool bState)
    {
    }

    /*!
     * \brief Write system is halting.
     */
    void writeStartHaltingSystem()
    {
    }

    /*!
     * \brief Test interface.
     *
     * Diagnostics printed to stderr.
     */
    void testInterface();


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Attribute Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Get monitoring alarm condition.
     *
     * \return Returns true or false.
     */
    int getAlarmCond()
    {
      return m_bAlarmCond;
    }

    /*!
     * \brief Get monitoring emergency stop condition.
     *
     * \return Returns true or false.
     */
    int getEStopCond()
    {
      return m_bEStopCond;
    }

    /*!
     * \brief Get motors are powered condition.
     *
     * \return Returns true or false.
     */
    int getPoweredCond()
    {
      return m_bPoweredCond;
    }

    /*!
     * \brief Get joint limits information.
     *
     * \param byMask  Limit switch bit mask (index).
     *
     * \return Return pointer to limit switch information.
     */
    HekOpticalLimit_T *getJointLimitInfo(byte_t byMask)
    {
      return &(m_opticalLimits[byMask].m_limit);
    }

    /*!
     * \brief Get current limit switch bit values.
     *
     * \return Bits.
     */
    byte_t getJointLimitBits()
    {
      return m_byLimitBits;
    }

    /*!
     * \brief Get current End Effector auxilliary bit values.
     *
     * \return Bits.
     */
    byte_t getEEAuxBits()
    {
      return m_byEEAuxBits;
    }

    /*!
     * \brief Get current joint limit tri-state.
     *
     * \param pJoint  Joint.
     * \param nLimit  Joint limit switch index.
     *
     * \return Return \ref HekTriState value.
     */
    HekTriState getJointLimitTriState(HekRobotJoint *pJoint, int nLimit);
    
  protected:
    // state
    MonStateId        m_eMonState;      ///< \h_hek monitor state
    bool              m_bIsOpen;        ///< hardware is [not] open 
    bool              m_bEStopCond;     ///< estop condition
    bool              m_bAlarmCond;     ///< alarm condition
    bool              m_bPoweredCond;   ///< power to motors condition
    
    // objects monitored 
    DynaChain        *m_pDynaChain;     ///< dynamixel servo chain alarms
    MapOpticalLimits  m_opticalLimits;  ///< joint optical limits map
    HekKinematics    *m_pKin;           ///< kinematics 

    // monitor hardware interface
#if defined(HEK_HAS_ARDUINO)
    HekUno            m_hwif;           ///< arduino compatible subproc i/f
#elif defined(HEK_HAS_SYS_BOARD)
    HekSysBoard       m_hwif;           ///< system board w/ I/O expansion i/f
    // gpio
#endif // hardware interface

    byte_t            m_byLimitBits;    ///< limit bit state
    byte_t            m_byEEAuxBits;    ///< end effector auxilliary bit state

    // monitor thread control
    MonThStateId      m_eMonThState;    ///< monitoring thread state
    MonThStateId      m_eMonThStateOld; ///< monitoring thread old state
    double            m_fHz;            ///< monitoring hertz
    struct timespec   m_tPeriod;        ///< derived monitoring period
    struct timespec   m_tStart;         ///< start time for next monitor work
    pthread_mutex_t   m_mutexMon;       ///< monitor mutex
    pthread_cond_t    m_condMon;        ///< monitor condition
    pthread_t         m_threadMon;      ///< monitor pthread identifier 

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Monitor Thread Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Lock the monitor thread mutex.
     *
     * The calling thread will block while waiting for the mutex to become 
     * available. Once locked, the monitor thread will block.
     *
     * The lock()/unlock() primitives provide a safe mechanism safely access
     * monitor resources.
     *
     * \par Context:
     * Any.
     */
    void lock()
    {
      pthread_mutex_lock(&m_mutexMon);
    }


    /*!
     * \brief Unlock the monitor thread mutex.
     *
     * The monitor thread will be available to run.
     *
     * \par Context:
     * Any.
     */
    void unlock()
    {
      pthread_mutex_unlock(&m_mutexMon);
    }

    /*!
     * \brief Create the monitor thread.
     *
     * \par Context:
     * Calling thread.
     *
     * \copydoc doc_return_std
     */
    int createMonThread();

    /*!
     * \brief Destroy the monitor thread.
     *
     * \par Context:
     * Calling thread.
     */
    void destroyMonThread();

    /*!
     * \brief Signal monitor thread of new state.
     *
     * \param eNewState Monitor thread's new state.
     */
    void signalMonThread(MonThStateId eNewState);

    /*!
     * \brief Monitor thread block wait in idle state.
     */
    void idleWait();

    /*!
     * \brief Monitor thread block timed wait in run state.
     */
    void runWait();

    /*!
     * \brief Get thread state name string.
     *
     * \param eNewState Monitor thread's new state.
     *
     * \return State name string.
     */
    const char *getThStateName(MonThStateId eNewState);

    /*!
     * \brief Monitor thread.
     *
     * \param pArg   Thread argument (point to HekMonitor object).
     * 
     * \return Returns NULL on thread exit.
     */
     static void *monThread(void *pArg);

    /*!
     * \brief Monitor joint (optical) limits for state transitions.
     *
     * Any limited rotation joint will be stopped when an optical limit switch
     * transitions from lit to blocked.
     */
    void monJointLimits();

    /*!
     * \brief Monitor End Effector auxillary I/O.
     */
    void monEEAuxIO();

    /*!
     * \brief Monitor servos for alarms.
     */
    void monServoAlarms();
  };

} // namespace hekateros


#endif // _HEK_MONITOR_H
