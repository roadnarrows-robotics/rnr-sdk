////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// File:      laeBatt.h
//
/*! \file
 *
 * $LastChangedDate: 2016-01-21 16:50:25 -0700 (Thu, 21 Jan 2016) $
 * $Rev: 4268 $
 *
 * \brief Laelaps battery management and energy monitoring class interface.
 *
 * A class instance runs under the control of the WatchDog thread.
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

#ifndef _LAE_BATT_H
#define _LAE_BATT_H

#include "rnr/rnrconfig.h"

#include  "Laelaps/laelaps.h"

/*!
 *  \brief The \h_laelaps namespace encapsulates all \h_laelaps related
 *  constructs.
 */
namespace laelaps
{
  /*!
   * \brief Battery Management and System Energy Monitoring (BMSEM) Class.
   */
  class LaeBattery
  {
  public:
    /*!
     * \brief Default constructor.
     */
    LaeBattery();

    /*!
     * \brief Destructor.
     */
    virtual ~LaeBattery();

    /*!
     * \brief Get the most recently updated battery charging state.
     *
     * \return  The batteries are [not] being charged.
     */
    bool getChargingState()
    {
      return m_bIsCharging;
    }

    /*!
     * \brief Get the most recently sensed battery voltage.
     *
     * The battery voltage is the (weighted) average of all voltage sensors
     * from all batteries.
     *
     * \return The sensed battery average voltage (V).
     */
    double getBatteryVoltage()
    {
      return m_fBatteryVoltage;
    }

    /*!
     * \brief Get the most recently estimated battery state of charge.
     *
     * \return State of Charge (0.0% - 100.0%).
     */
    double getBatteryStateOfCharge()
    {
      return m_fBatterySoC;
    }

    /*!
     * \brief Get most recently sensed current pulled by all motors.
     *
     * \param Motor current (amps).
     */
    double getMotorCurrent()
    {
      return m_fMotorAmps;
    }

    /*!
     * \brief Get most recently calculated power used by all motors.
     *
     * \return Motor power (watts).
     */
    double getMotorPowerElec(const double fMotorWatts)
    {
      return m_fMotorWatts;
    }

    /*!
     * \brief Get most recently estimated total current pulled by the robot.
     *
     * \return System total current (amps).
     */
    double getTotalCurrent()
    {
      return  m_fTotalAmps;
    }

    /*!
     * \brief Get most recently estimated power consumed by robot.
     *
     * \return System total power (watts).
     */
    double getTotalPower()
    {
      return m_fTotalWatts;
    }

    /*!
     * \brief Calculate the present energy state of the robot motors.
     *
     * \note The motor controllers have battery voltage sensors, so the 
     * battery voltage is determined here also.
     */
    virtual void calcMotorEnergyState();

    /*!
     * \brief Calculate the present energy state of all logic curcuitry.
     *
     * Logic circuitry includes odroid, motor controllers, battery chargers,
     * IMU, watchdog sub-processor, sensors, and supporting hardware.
     */
    virtual void calcLogicEnergyState();

    /*!
     * \brief Estimate batteries State of Charge given the present energy state.
     *
     * \return SoC [0.0% - 100.0%]
     */
    virtual double estimateBatteryStateOfCharge();

    /*!
     * \brief Update the energy state for all of the robot's subsystems.
     */
    virtual void update();

  protected:
    bool    m_bIsCharging;      ///< the battery is [not] being charged
    double  m_fBatteryVoltage;  ///< sensed battery output voltage (V)
    double  m_fBatterySoC;      ///< battery state of charge (0% - 100%)
    double  m_fMotorAmps;       ///< sensed current for all motors (A)
    double  m_fMotorWatts;      ///< sensed power for all motors (watts)
    double  m_fLogicAmps;       ///< current consumed by logic circuitry (A)
    double  m_fLogicWatts;      ///< power consumed by logic circuitry (watts)
    double  m_fTotalAmps;       ///< total system current draw (A)
    double  m_fTotalWatts;      ///< total system power used (watts)
  }; // LaeBattery

} // namespace laelaps


#endif // _LAE_BATT_H
