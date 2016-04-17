////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekCalib.h
//
/*! \file
 *
 * $LastChangedDate: 2014-11-18 14:31:49 -0700 (Tue, 18 Nov 2014) $
 * $Rev: 3810 $
 *
 * \brief HekCalib - Hekateros calibration abstract base class interface.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2013.  RoadNarrows
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

#ifndef _HEK_CALIB_H
#define _HEK_CALIB_H

#include <string>

#include "rnr/rnrconfig.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaServo.h"

#include "Hekateros/hekateros.h"

namespace hekateros
{
  //
  // Forward declaraions.
  //
  class HekRobot;

  /*!
   * \brief Hekateros robotic manipulator calibration abstract base class.
   *
   * \param robot   Instance of \h_hek robot.
   */
  class HekCalib
  {
  public: 

    /*!
     * \breif Default initialization constructor.
     *
     * \param robot   Instance of \h_hek robot.
     */
    HekCalib(HekRobot &robot) : m_robot(robot)
    {
    }
  
    /*!
     * \breif Destructor.
     */
    virtual ~HekCalib()
    {
    }

    /*!
     * \brief Calibrate the \h_hek's robotic arm.
     *
     * \copydoc doc_return_std
     */
    virtual int calibrate() = 0;

    /*!
     * \brief Move joint to position.
     *
     * This call blocks until move is complete.
     *
     * \param strJointName    Joint name.
     * \param fJointGoalPos   Joint goal position (radians).
     * \param fJointGoalVel   Joint goal velocity (radians/second).
     *
     * \return New joint position (radians).
     */
    virtual double moveWait(const std::string &strJointName,
                            const double       fJointGoalPos,
                            const double       fJointGoalVel);

    /*!
     * \brief Move joint until torque limit is reached.
     *
     * This call blocks until move is complete.
     *
     * \param strJointName    Joint name.
     * \param fJointGoalPos   Joint goal position (radians).
     * \param fJointGoalVel   Joint goal velocity (radians/second).
     *
     * \return New joint position (radians) at over torque condition.
     */
    virtual double moveToTorqueLimit(const std::string &strJointName,
                                     const double       fJointGoalPos,
                                     const double       fJointGoalVel);

    /*!
     * \brief Move joint until unoccluded optical limit position is detected.
     *
     * This call blocks until move is complete.
     *
     * \param strJointName    Joint name.
     * \param fJointGoalPos   Joint goal position (radians).
     * \param fJointGoalVel   Joint goal velocity (radians/second).
     * \param byMask          Mask of limits to check.
     *
     * \return New joint position (radians) at first light.
     */
    virtual double moveToLight(const std::string &strJointName,
                               const double       fJointGoalPos,
                               const double       fJointGoalVel,
                               byte_t             byMask);

    /*!
     * \brief Move joint until occluded optical limit position is detected.
     *
     * This call blocks until move is complete.
     *
     * \param strJointName    Joint name.
     * \param fJointGoalPos   Joint goal position (radians).
     * \param fJointGoalVel   Joint goal velocity (radians/second).
     * \param byMask          Mask of limits to check.
     *
     * \return New joint position (radians) at first dark.
     */
    virtual double moveToDark(const std::string &strJointName,
                              const double       fJointGoalPos,
                              const double       fJointGoalVel,
                              byte_t             byMask);

  protected:
    HekRobot &m_robot;    ///< robot instance
  };

} // namespace hekaeros

#endif // _HEK_CALIB_H
