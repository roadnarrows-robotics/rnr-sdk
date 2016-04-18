////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Library:   liblaelaps
//
// File:      laeReports.h
//
/*! \file
 *
 * $LastChangedDate: 2016-02-11 15:06:42 -0700 (Thu, 11 Feb 2016) $
 * $Rev: 4312 $
 *
 * \brief Interfaces of Laelaps requested and/or published report classes.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2015-2016.  RoadNarrows
 * (http://www.roadnarrows.com)
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

#ifndef _LAE_REPORTS_H
#define _LAE_REPORTS_H

#include <string>
#include <vector>

#include "rnr/rnrconfig.h"

#include "Laelaps/laelaps.h"
#include "Laelaps/laeAlarms.h"
#include "Laelaps/laeTraj.h"
#include "Laelaps/laePowertrain.h"

namespace laelaps
{
  //
  // Forward declarations.
  //
  class LaeRobot;

  // ---------------------------------------------------------------------------
  // Class LaeRptMotorCtlrHealth
  // ---------------------------------------------------------------------------
 
  /*!
   * \brief Robot motor health.
   */
  class LaeRptMotorCtlrHealth
  {
  public:
    std::string   m_strName;      ///< motor controller name
    double        m_fTemperature; ///< motor controller board temperature (C)
    double        m_fVoltage;     ///< motor controller main input voltage (V)
    LaeAlarmInfo  m_alarms;       ///< motor controller alarms and warnings

    /*!
     * \brief Default constructor.
     */
    LaeRptMotorCtlrHealth();

    /*!
     * \brief Copy constructor.
     */
    LaeRptMotorCtlrHealth(const LaeRptMotorCtlrHealth &src);

    /*!
     * \brief Destructor.
     */
    ~LaeRptMotorCtlrHealth()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns copy of this.
     */
    LaeRptMotorCtlrHealth operator=(const LaeRptMotorCtlrHealth &rhs);
  };


  // ---------------------------------------------------------------------------
  // Class LaeRptMotorHealth
  // ---------------------------------------------------------------------------
 
  /*!
   * \brief Robot motor health.
   */
  class LaeRptMotorHealth
  {
  public:
    std::string   m_strName;      ///< motor name
    double        m_fTemperature; ///< motor temperature (Celsius)
    double        m_fVoltage;     ///< motor voltage (volts)
    double        m_fCurrent;     ///< motor current draw (amperes)
    LaeAlarmInfo  m_alarms;       ///< motor alarms and warnings

    /*!
     * \brief Default constructor.
     */
    LaeRptMotorHealth();

    /*!
     * \brief Copy constructor.
     */
    LaeRptMotorHealth(const LaeRptMotorHealth &src);

    /*!
     * \brief Destructor.
     */
    ~LaeRptMotorHealth()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns copy of this.
     */
    LaeRptMotorHealth operator=(const LaeRptMotorHealth &rhs);
  };


  // ---------------------------------------------------------------------------
  // Class LaeRptRobotStatus
  // ---------------------------------------------------------------------------
 
  /*!
   * \brief Robot global status report.
   */
  class LaeRptRobotStatus
  {
  public:
    typedef std::vector<LaeRptMotorHealth>       VecMotorHealth;
    typedef std::vector<LaeRptMotorCtlrHealth>   VecMotorCtlrHealth;

    // ROS industrial
    LaeRobotMode    m_eRobotMode;         ///< robot operating mode
    LaeTriState     m_eIsEStopped;        ///< robot is [not] emergency stopped
    LaeTriState     m_eAreDrivesPowered;  ///< motor are [not] powered
    LaeTriState     m_eIsMotionPossible;  ///< motion is [not] possible
    LaeTriState     m_eIsInMotion;        ///< robot is [not] moving
    LaeTriState     m_eIsInError;         ///< robot is [not] in error condition
    int             m_nErrorCode;         ///< laelaps error code

    // robot base extensions
    double          m_fBatterySoC;    ///< battery state of charge (0%-100%)
    bool            m_bIsCharging;    ///< battery is [not] being charged
    double          m_fCurrent;       ///< estimated robot draw (A)
    double          m_fVoltage;       ///< average battery voltage (V)
    double          m_fTemperature;   ///< interior average temperature (C)
    LaeTriState     m_eAuxBattEn;     ///< auxilliary battery power enable
    LaeTriState     m_eAux5VEn;       ///< auxilliary 5V power enable
    LaeAlarmInfo    m_alarms;         ///< system alarms and warnings

    // motor extensions
    VecMotorCtlrHealth  m_vecCtlrHealth;  ///< motors controllers' health
    VecMotorHealth      m_vecMotorHealth; ///< motors' health

    /*!
     * \brief Default constructor.
     */
    LaeRptRobotStatus()
    {
      clear();
    }

    /*!
     * \brief Copy constructor.
     */
    LaeRptRobotStatus(const LaeRptRobotStatus &src);

    /*!
     * \brief Destructor.
     */
    ~LaeRptRobotStatus()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns copy of this.
     */
    LaeRptRobotStatus operator=(const LaeRptRobotStatus &rhs);

    /*!
     * \brief Clear report.
     */
    void clear();

    /*!
     * \brief Generate report.
     *
     * \param pRobot  Pointer to the robot.
     */
    void generate(LaeRobot *pRobot);

  }; // class LaeRptRobotStatus


  // ---------------------------------------------------------------------------
  // Class LaeRptDynPowertrain
  // ---------------------------------------------------------------------------

  /*!
   * \brief Powertrain dynamics subreport.
   */
  class LaeRptDynPowertrain
  {
  public:
    std::string m_strName;      ///< powertrain unique name (key)
    s64_t       m_nEncoder;     ///< motor encoder position (quad pulses)
    int         m_nSpeed;       ///< raw speed (qpps)
    double      m_fPosition;    ///< wheel angular position (radians)
    double      m_fVelocity;    ///< wheel angular velocity (radians/second)
    double      m_fPe;          ///< motor input electrical power (W)
    double      m_fTorque;      ///< wheel torque (N-m)

    /*!
     * \brief Default constructor.
     */
    LaeRptDynPowertrain();

    /*!
     * \brief Copy constructor.
     */
    LaeRptDynPowertrain(const LaeRptDynPowertrain &src);

    /*!
     * \brief Destructor.
     */
    ~LaeRptDynPowertrain()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns copy of this.
     */
    LaeRptDynPowertrain operator=(const LaeRptDynPowertrain &rhs);

    /*!
     * \brief Clear report.
     */
    void clear();

    /*!
     * \brief Generate report.
     *
     * \param nMotorId  Globally unique powertrain/motor id.
     */
    void generate(int nMotorId);

  }; // class LaeRptDynPowertrain


  // ---------------------------------------------------------------------------
  // Class LaeRptDynamics
  // ---------------------------------------------------------------------------

  /*!
   * \brief Laelaps dynamics report.
   */
  class LaeRptDynamics
  {
  public:
    typedef std::vector<LaeRptDynPowertrain>  VecDynPowertrain;

    // robot platform dynamics
    LaePose   m_pose;       ///< robot 2D pose (meters, meters, radians)
    double    m_fOdometer;  ///< robot odometer (meters)
    double    m_fVelocity;  ///< robot linear velocity v (meters/second)

    // powertrain dynamics
    VecDynPowertrain  m_vecDynPowertrain;   ///< powertrains' state

    /*!
     * \brief Default constructor.
     */
    LaeRptDynamics();

    /*!
     * \brief Copy constructor.
     */
    LaeRptDynamics(const LaeRptDynamics &src);

    /*!
     * \brief Destructor.
     */
    ~LaeRptDynamics()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns copy of this.
     */
    LaeRptDynamics operator=(const LaeRptDynamics &rhs);

    /*!
     * \brief Clear report.
     */
    void clear();

    /*!
     * \brief Generate report.
     *
     * \param pRobot  Pointer to the robot (not used).
     */
    void generate(LaeRobot *pRobot);

  }; // class LaeRptDynamics

} // namespace laelaps

#endif // _LAE_REPORTS_H
