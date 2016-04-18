////////////////////////////////////////////////////////////////////////////////
//
// Package:   Kuon
//
// Library:   libkuon
//
// File:      kuonRobot.h
//
/*! \file
 *
 * $LastChangedDate: 2014-04-09 16:45:20 -0600 (Wed, 09 Apr 2014) $
 * $Rev: 3639 $
 *
 * \brief Kuon Robot Class interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2013-2014  RoadNarrows
 * (http://www.RoadNarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
 * 
 * Permission is hereby granted, without written agreement and without
 * license or royalty fees, to use, copy, modify, and distribute this
 * software and its documentation for any purpose, provided that
 * (1) The above copyright notice and the following two paragraphs
 * appear in all copies of the source code and (2) redistributions
 * including binaries reproduces these notices in the supporting
 * documentation.   Substantial modifications to this software may be
 * copyrighted by their authors and need not follow the licensing terms
 * described here, provided that the new terms are clearly indicated in
 * all files where they apply.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
 * OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _KUON_ROBOT_H
#define _KUON_ROBOT_H

#include <pthread.h>

#include <string>
#include <map>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/units.h"

#include "Kuon/RS160DControl.h"

#include "Kuon/kuon.h"
#include "Kuon/kuonUtils.h"
#include "Kuon/kuonDesc.h"
#include "Kuon/kuonJoint.h"
#include "Kuon/kuonTraj.h"
#include "Kuon/kuonStatus.h"

namespace kuon
{
  /*!
   * \brief Kuon robotic manipulator plus accesories class.
   *
   * Includes up to 3 kinematic chains.
   */
  class KuonRobot
  { 
  public:
    static const float GovernorDft; ///< speed limit governor start-up default
    static const float BrakeDft;    ///< brake start-up default
    static const float SlewDft;     ///< slew start-up default

    /*!
     * \brief Map of robot joints.
     *
     * \termblock
     * \term key: \termdata motor id \endterm
     * \term mapped type: \termdata joint data \endterm
     * \endtermblock
     *
     * \note Joint order is critical. Ascending motor ids keeps map in order,
     * but if this cannot be guaranteed, then change strategy.
     */
    typedef std::map<int, KuonRobotJoint> MapRobotJoints;

    /*!
     * \brief Indirect map of robot joints.
     *
     * \termblock
     * \term key: \termdata joint name \endterm
     * \term mapped type: \termdata motor id \endterm
     * \endtermblock
     */
    typedef std::map<std::string, int> IMapRobotJoints;

    /*!
     * \brief Asynchronous task id.
     */
    enum AsyncTaskId
    {
      AsyncTaskIdNone,        ///< no task

      // add others here, as needed
    };

    /*!
     * \brief Default initialization constructor.
     *
     * \param bNoExec Do [not] execute arm physical movements. All commands
     *                and responses are supported but the lower level arm
     *                movement commands will not be issued.
     */
    KuonRobot(bool bNoExec=false);

    /*!
     * \brief Destructor.
     */
    virtual ~KuonRobot();


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Communication and Robot Initialization Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Connect to \h_kuon.
     *
     * \param strDevMotorCtlr0    Motor controller serial device name 0.
     * \param strDevMotorCtlr1    Motor controller serial device name 1.
     * \param nBaudRateMotorCtlr  Motor controllers baud rate.
     *
     * \copydoc doc_return_std
     */
    int connect(const std::string &strDevMotorCtlr0   = KuonDevMotorCtlr0,
                const std::string &strDevMotorCtlr1   = KuonDevMotorCtlr1,
                int                nBaudRateMotorCtlr = KuonBaudRateMotorCtlr);

    /*!
     * \brief Disconnect from \h_kuon.
     *
     * \copydoc doc_return_std
     */
    int disconnect();

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Canned Movements
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    // todo left 90, right 90, 180 uturn
   
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Fundamental Base Operations
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Emergency stop.
     *
     * All motor will stop driving, so arm or accessories may fall.
     *
     * \copydoc doc_return_std
     */
    int estop();

    /*!
     * \brief Freeze arm and accessories at current position.
     *
     * Motors are still being driven.
     *
     * \copydoc doc_return_std
     */
    int freeze();

    /*!
     * \brief Release arm and accessories.
     *
     * Motors will stop driving, so the arm may fall.
     *
     * \copydoc doc_return_std
     */
    int release();

    /*!
     * \brief Attempt to clear all motor alarms in all kinematic chains.
     *
     * \note Not all alarms are clearable (e.g. temperature).
     *
     * \copydoc doc_return_std
     */
    int clearAlarms();

    /*!
     * \brief Reset (clears) emergency stop condition.
     *
     * \note Motors are not re-powered until an move or freeze action is called.
     */
    void resetEStop()
    {
      m_bIsEStopped = false;
      m_bAlarmState = false;
    }

    /*!
     * \brief Set speed limit governor value.
     *
     * Governor is defined as:\n
     * speed = cap(set_speed, min_speed * governor, max_speed * governor)
     *
     * \param fGovernor   Governor value between [0.0, 1.0].
     *
     * \return Returns new governor value.
     */
    float setGovernor(float fGovernor)
    {
      m_fGovernor       = (float)fcap(fGovernor, 0.0, 1.0);
      m_nGovernSpeedMin = (int)(m_fGovernor * (float)RS160D_MOTOR_SPEED_MIN);
      m_nGovernSpeedMax = (int)(m_fGovernor * (float)RS160D_MOTOR_SPEED_MAX);

      return m_fGovernor;
    }

    /*!
     * \brief Increment/decrement speed limit governor value.
     *
     * Governor is defined as:\n
     * speed = set_speed * governor
     *
     * \param fDelta  Governor \h_plusmn delta.
     *
     * \return Returns new governor value.
     */
    float incrementGovernor(float fDelta)
    {
      return setGovernor(m_fGovernor+fDelta);
    }

    /*!
     * \brief Set robot's auto-brake value.
     *
     * The brake value is applied to all motors. In the current Kuon, rheostatic
     * motor braking is used. Future versions may use regenerative and/or
     * mechanical braking.
     *
     * Braking is automatically applied when motor speeds are 0.
     *
     * \param fBrake  Brake value between [0.0, 1.0] where 0.0 is coasting and
     *                1.0 is full brake.
     *
     * \copydoc doc_return_std
     */
    int setBrake(float fBrake);

    /*!
     * \brief Set robot's power slew value.
     *
     * The slew value is applied to all motors. It defines the power ramp up 
     * scale when accelerating. 
     *
     * Slewing is automatically applied on acceleration changes.
     *
     * \param fSlew   Slew value between [0.0, 1.0] where 0.0 is full power
     *                applied instantly with quickest response and 1.0 is the
     *                slowest ramp up with sloggish response but little back
     *                EMF generated.
     *
     * \copydoc doc_return_std
     */
    int setSlew(float fSlew);

    /*!
     * \brief Set robot's left and right motor speeds.
     *
     * \param fSpeedLeft  Left motor speeds in the given units.
     * \param fSpeedRight Right motor speeds in the given units.
     * \param units       Speed units. One of: \n
     *                    units_norm units_percent units_permil units_raw.
     *
     * \copydoc doc_return_std
     */
    int setSpeed(double fSpeedLeft,
                 double fSpeedRight,
                 units_t units=units_norm);

    /*!
     * \brief Move platform through trajectory point.
     *
     * TODO
     *
     * \param trajectoryPoint   Trajectory end point.
     *
     * \copydoc doc_return_std
     */
    int move(KuonWheelTrajectoryPoint &trajectoryPoint);

    /*!
     * \brief Convert velocity to raw motor speed value.
     *
     * \param fVelocity   Velocity.
     * \param units       Velocity units. One of:\n
     *                    units_norm [-1.0, 1.0]\n
     *                    units_percent [-100.0, 100.0]\n
     *                    units_permil [-1000.0, 1000.0]\n
     *                    units_raw [min, max]
     *
     * \return Raw speed value.
     */
    int velocityToRawSpeed(double fVelocity, units_t units=units_norm);


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Industrial Base Status and State
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Get the robot current status.
     *
     * \param [in,out] robotStatus  Robot status.
     *
     * \copydoc doc_return_std
     */
    int getRobotStatus(KuonRobotStatus &robotStatus);

    /*!
     * \brief Get the joint states of a kinematic chain.
     *
     * \param [in,out] jointStatePoint  Joint state point.
     *
     * \copydoc doc_return_std
     */
    int getJointState(KuonJointStatePoint &jointStatePoint);

    /*!
     * \brief Get trajectory feedback.
     *
     * TODO 
     * \param [in,out] trajectoryFeedback  Wheel state point.
     *
     * \copydoc doc_return_std
     */
    int getTrajectoryState(KuonWheelTrajectoryFeedback &trajFeedback);

    /*!
     * \brief Set robot's operational mode.
     *
     * \return eRobotMode Robot operation mode. See \ref KuonRobotMode.
     */
    void setRobotMode(KuonRobotMode eRobotMode)
    {
      m_eRobotMode = eRobotMode;
    }

    /*!
     * \brief Cancel any asynchronous task.
     *
     * \note There may be a little delay between canceling an async task
     * and the task actually stopping.
     */
    void cancelAsyncTask();

    /*!
     * \brief Get the current asynchronous task state.
     *
     * \return \ref KuonAsyncTaskState enum value.
     */
    KuonAsyncTaskState getAsyncState()
    {
      return m_eAsyncTaskState;
    }

    /*!
     * \brief Get the last asynchronous task return code.
     *
     * \return
     * Returns KUON_OK when task terminated successfully.
     * Otherwise, returns \h_lt 0 \ref kuon_ecodes.
     */
    int getAsyncRc()
    {
      return m_rcAsyncTask;
    }


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Attribute Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Test if connected to \h_kuon hardware.
     *
     * \return Returns true or false.
     */
    bool isConnected()
    {
      return (m_fdMotorCtlr0 >= 0) && (m_fdMotorCtlr1 >= 0);
    }

    /*!
     * \brief Get the \h_kuon robotic arm hardware version number.
     *
     * \param [out] nVerMajor     Major version number.
     * \param [out] nVerMinor     Minor version number.
     * \param [out] nVerRevision  Revision version number.
     */
    void getVersion(int &nVerMajor, int &nVerMinor, int &nRevision)
    {
      uint_t  uHwVer;

      uHwVer    = strToVersion(m_descKuon.getBaseDesc()->getProdHwVer());
      nVerMajor = KUON_VER_MAJOR(uHwVer);
      nVerMinor = KUON_VER_MINOR(uHwVer);
      nRevision = KUON_VER_REV(uHwVer);
    }

    /*!
     * \brief Get the \h_kuon robotic arm hardware version string.
     *
     * Version number strings are of the dotted form maj.min.rev.
     *
     * \return Version string.
     */
    std::string getVersion()
    {
      return m_descKuon.getBaseDesc()->getProdHwVer();
    }

    /*!
     * \brief Get the \h_kuon product description.
     *
     * \return Returns pointer to description.
     */
    KuonDesc *getKuonDesc()
    {
      return &m_descKuon;
    }

    /*!
     * \brief Convenience function to get this \h_kuon description's base
     * product id.
     *
     * \return Returns product id. See \ref KuonProdId.
     */
    int getProdId()
    {
      return m_descKuon.getBaseDesc()->getProdId();
    }
  
    /*!
     * \brief Convenience function to get this \h_kuon description's base
     * product name.
     *
     * \return Returns product name. See \ref KuonProdName.
     */
    std::string getProdName()
    {
      return m_descKuon.getBaseDesc()->getProdName();
    }
  
    /*!
     * \brief Get the \h_kuon full brief descirption.
     *
     * \return Returns product brief description.
     */
    std::string getFullProdBrief()
    {
      return m_descKuon.getFullProdBrief();
    }
  
    /*!
     * \brief Test if robot is fully described via configuration XML.
     *
     * \return Returns true or false.
     */
    int isDescribed()
    {
      return m_descKuon.isDescribed();
    }

    /*!
     * \brief Test if robot is current emergency stopped.
     *
     * \return Returns true or false.
     */
    int isEStopped()
    {
      return m_bIsEStopped;
    }

    /*!
     * \brief Test if robot motor are currently being driven (powered).
     *
     * \return Returns true or false.
     */
    int areMotorsPowered()
    {
      return m_bAreMotorsPowered;
    }

    /*!
     * \brief Test if any joint in any of the kinematic chains is moving.
     *
     * \note RS160D motor controller provide no feedback.
     *
     * \return Returns true or false.
     */
    bool isInMotion()
    {
      return (m_nSetPtSpeedLeft != 0) || (m_nSetPtSpeedRight != 0);
    }

    /*!
     * \brief Test if robot is alarmed.
     *
     * \return Returns true or false.
     */
    int isAlarmed()
    {
      return m_bAlarmState;
    }

    /*!
     * \brief Get current speed limit governor setting.
     *
     * \return Return value.
     */
    float getGovernor()
    {
      return m_fGovernor;
    }

    /*!
     * \brief Get current brake setting.
     *
     * \return Return value.
     */
    float getBrake()
    {
      return m_fBrake;
    }

    /*!
     * \brief Get current slew setting.
     *
     * \return Return value.
     */
    float getSlew()
    {
      return m_fSlew;
    }

    /*!
     * \brief Get the current left and right side motor velocity set points.
     *
     * \param [out] fSpeedLeft  Current left motors set velocity in units.
     * \param [out] fSpeedRight Current right motors set velocity in units.
     * \param units             Velocity units.
     */
    void getVelocitySetPoints(double &fSpeedLeft,
                              double &fSpeedRight,
                              units_t units=units_norm);


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Kinematic Access and Mapping Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Get robotic joint in arm+ee kinematic chain at the given index.
     *
     * \param index   Joint index.
     *
     * \return If found, returns pointer to joint. Otherwise returns NULL.
     */
    KuonRobotJoint *getBaseJointAt(int index)
    {
      return index < m_kinBase.size()? &m_kinBase.at(index): NULL;
    }

    /*!
     * \brief Get robotic joint in arm+ee kinematic chain.
     *
     * \param strName   Joint name (key).
     *
     * \return If found, returns pointer to joint. Otherwise returns NULL.
     */
    KuonRobotJoint *getBaseJoint(std::string &strName)
    {
      IMapRobotJoints::iterator  iter;

      if( (iter = m_imapBase.find(strName)) != m_imapBase.end() )
      {
        return &(m_kinBase[iter->second]);
      }
      else
      {
        return NULL;
      }
    }

    /*!
     * \brief Get robotic joint in arm+ee kinematic chain.
     *
     * \param nMotorId  Motor id (primary key).
     *
     * \return If found, returns pointer to joint. Otherwise returns NULL.
     */
    KuonRobotJoint *getBaseJoint(int nMotorId)
    {
      MapRobotJoints::iterator  iter;

      if( (iter = m_kinBase.find(nMotorId)) != m_kinBase.end() )
      {
        return &(iter->second);
      }
      else
      {
        return NULL;
      }
    }

  protected:
    // state
    bool            m_bNoExec;          ///< do [not] execute physical movements
    KuonDesc        m_descKuon;         ///< \h_kuon description
    KuonRobotMode   m_eRobotMode;       ///< robot operating mode
    bool            m_bIsEStopped;      ///< arm is [not] emergency stopped
    bool            m_bAlarmState;      ///< robot is [not] alarmed
    bool            m_bAreMotorsPowered;///< arm motor are [not] driven
    float           m_fGovernor;        ///< speed limit governor setting
    float           m_fBattery;         ///< battery energy level
    float           m_fBrake;           ///< motor braking
    float           m_fSlew;            ///< power slewing
    int             m_nGovernSpeedMin;  ///< minimum governed speed limit
    int             m_nGovernSpeedMax;  ///< maximum governed speed limit
    int             m_nSetPtSpeedLeft;  ///< left motors raw speed set point
    int             m_nSetPtSpeedRight; ///< right motors raw speed set point

    // RS160D motor controllers
    int               m_fdMotorCtlr0;   ///< motor controller 0 file descriptor
    int               m_fdMotorCtlr1;   ///< motor controller 1 file descriptor

    // joints and kinematics chains
    MapRobotJoints    m_kinBase;        ///< robot base kin
    IMapRobotJoints   m_imapBase;       ///< robot base indirect kin map

    // motion
    KuonWheelTrajectoryPoint m_lastTrajBase; ///< last trajectory point for base

    // asynchronous task control
    KuonAsyncTaskState  m_eAsyncTaskState;  ///< asynchronous task state
    int                 m_rcAsyncTask;      ///< last async task return code
    AsyncTaskId         m_eAsyncTaskId;     ///< asynchronous task id
    void               *m_pAsyncTaskArg;    ///< asynchronous argument
    pthread_t           m_threadAsync;      ///< async pthread identifier 


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Convert Static Specifications Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Convert specification(s) to operational parameters.
     *
     * The specifications must be defined and the connecton to the \h_kuon
     * hardware must be open.
     *
     * There are up to 3 kinematic chains, given the specifications and the
     * hardware:
     * \li arm + end effector
     * \li equipment deck effector
     * \li auxiliary effector
     *
     * \note A kinematic chain differs from the dynamixel chain. \h_kuon
     * supports one dynamixel bus in which all motor hang. A dynamixel chain
     * defines this communication bus. Motors on the bus can be associated with
     * any of the above three kinematic chains.
     *
     * \copydoc doc_return_std
     */
    int convertSpecs();

    /*!
     * \brief Add a joint to robot's kinematic chain.
     *
     * \param [in] pSpecJoint   Pointer to joint spcecification.
     * \param [in] pSpecMotor   Pointer to motor spcecification.
     * \param [out] kin         Modified kinematics chain of joints.
     * \param [out] imap        Indirect map of kinematic chain.
     *
     * \copydoc doc_return_std
     */
    int addRobotJoint(KuonSpecMotor_T *pSpecJoint,
                      MapRobotJoints  &kin,
                      IMapRobotJoints &imap);

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Hardware and Software Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Configure \h_kuon for normal operation.
     *
     * The arm, end effector, and accessory effectors are configured for
     * normal operation.
     *
     * \copydoc doc_return_std
     */
    int configForOperation();


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Asynchronous Operation Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Create the asynchronous thread.
     *
     * \par Context:
     * Calling thread.
     *
     * \copydoc doc_return_std
     */
    int createAsyncThread();

    /*!
     * \brief Asynchronous operation thread.
     *
     * \param pArg   Thread argument (point to KuonRobot object).
     * 
     * \return Returns NULL on thread exit.
     */
     static void *asyncThread(void *pArg);
  };

} // namespace kuon


#endif // _KUON_ROBOT_H
