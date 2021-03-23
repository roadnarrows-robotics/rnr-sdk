////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Library:   liblaelaps
//
// File:      laePlatform.h
//
/*! \file
 *
 * $LastChangedDate: 2016-02-15 12:16:48 -0700 (Mon, 15 Feb 2016) $
 * $Rev: 4319 $
 *
 * \brief Laelaps robotic platform control and dynamics state interface.
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

#ifndef _LAE_PLATFORM_H
#define _LAE_PLATFORM_H

#include <string>
#include <vector>

#include "rnr/rnrconfig.h"

#include "Laelaps/RoboClaw.h"

#include "Laelaps/laelaps.h"
#include "Laelaps/laeTune.h"
#include "Laelaps/laeMotor.h"
#include "Laelaps/laeUtils.h"
#include "Laelaps/laeTraj.h"
#include "Laelaps/laePowertrain.h"

namespace laelaps
{
  // ---------------------------------------------------------------------------
  // Class LaeMotorCtlrState
  // ---------------------------------------------------------------------------

  /*!
   * \brief  Robot base motor controller board state.
   */
  class LaeMotorCtlrState
  {
  public:
    std::string   m_strName;      ///< motor controller name
    double        m_fMainVolts;   ///< average sensed battery voltage (V)
    double        m_fBoardTemp;   ///< average sensed interior tempature (C)
    uint_t        m_uStatus;      ///< board status

    /*!
     * \brief Default constructor.
     */
    LaeMotorCtlrState()
    {
      clear();
    }

    /*!
     * \brief Copy constructor.
     */
    LaeMotorCtlrState(const LaeMotorCtlrState &src);

    /*!
     * \brief Destructor.
     */
    ~LaeMotorCtlrState()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns reference to this.
     */
    LaeMotorCtlrState &operator=(const LaeMotorCtlrState &rhs);

    /*!
     * \brief Clear data sans name.
     */
    void clear();

  }; // class LaeMotorCtlrState


  // ---------------------------------------------------------------------------
  // Class LaePlatform
  // ---------------------------------------------------------------------------

  /*!
   * \brief  Robot platform control and state data class.
   */
  class LaePlatform
  {
  public:
    /*!
     * \brief Working indices and sizes.
     */
    enum Idx
    {
      IdxPrev   = 0,    ///< index of previous history
      IdxCur    = 1,    ///< index of current state
      HistSize  = 2     ///< size of history
    };

    std::string   m_strName;  ///< robot platform name

    // dimensions
    Dim       m_dimRobot;     ///< total dimensions of robot (TODO)
    Dim       m_dimBody;      ///< dimensions of robot body
    double    m_fWheelbase;   ///< wheelbase (m)
    double    m_fWheeltrack;  ///< wheeltrack (m)

    // dynamics
    double    m_fPosLast[LaeMotorsNumOf];
                                ///< last sensed wheel positions (radians)
    LaePose   m_pose[HistSize]; ///< robot pose (meters, meters, radians)
    double    m_fOdometer;      ///< robot odometer (meters)
    double    m_fVelocity;      ///< robot velocity (meters/second)

    // power
    double    m_fAmpsMotors;  ///< total draw from all motors (A)
    double    m_fVoltsAvg;    ///< average sensed battery voltage (V)
    double    m_fTempAvg;     ///< average sensed interior tempature (C)

    // motor controllers
    LaeMotorCtlrState m_ctlr[LaeNumMotorCtlrs]; ///< motor controllers' state

    /*!
     * \brief Default constructor.
     */
    LaePlatform();

    /*!
     * \brief Copy constructor.
     */
    LaePlatform(const LaePlatform &src);

    /*!
     * \brief Destructor.
     */
    ~LaePlatform()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns reference to this.
     */
    LaePlatform &operator=(const LaePlatform &rhs);

    /*!
     * \brief Clear data sans name.
     */
    void clear();

    /*!
     * \brief Get robot platform name.
     *
     * \return String.
     */
    std::string getPlatformName() const
    {
      return m_strName;
    }

    /*!
     * \brief Configure robot platform from product description.
     *
     * \param desc  Laelaps robot base product description data.
     *
     * \copydoc doc_return_std
     */
    virtual int configure(const LaeDescBase &desc);

    /*!
     * \brief Configure platform.
     *
     * \param tunes   Laelaps tuning parameters.
     *
     * \copydoc doc_return_std
     */
    virtual int configure(const LaeTunes &tunes);

    /*!
     * \brief Reload tuning parameters and re-configure.
     *
     * \param tunes   Laelaps tuning parameters.
     *
     * \copydoc doc_return_std
     */
    virtual int reload(const LaeTunes &tunes);

    /*!
     * \brief Reset odometry to zero.
     *
     * All related positioning data are reset to 0.
     *
     * \copydoc doc_return_std
     */
    virtual int resetOdometer();

    /*!
     * \brief Update robot platform state dynamics.
     *
     * \param mapPowertrains  Powertrain kinodynamics.
     *
     * \copydoc doc_return_std
     */
    virtual int updateStateDynamics(const LaeMapPowertrain &mapPowertrains);

    /*!
     * \brief Update robot platform motor controller health state.
     *
     * \param nCtlr       Motor controller id.
     * \param fVolts      Motor input voltage (V).
     * \param fTemp       Motor controller board voltage (V).
     * \param uCtlrStatus Motor controller status bits.
     *
     * \copydoc doc_return_std
     */
    virtual int updateCtlrHealth(int    nCtlr,
                                 double fVolts,
                                 double fTemp,
                                 uint_t uStatus);

    /*!
     * \brief Update robot platform health state.
     *
     * \param mapPowertrains  Powertrain kinodynamics.
     *
     * \copydoc doc_return_std
     */
    virtual int updateHealth(const LaeMapPowertrain &mapPowertrains);

    /*!
     * \brief Clear pose history.
     */
    virtual void clearPoses();

    /*!
     * \brief Push a new pose on the pose history.
     *
     * The history is aged first.
     *
     * \param pose  New pose.
     */
    virtual void pushNewPose(const LaePose &pose);

  };    // class LaePlatform

} // namespace laelaps


#endif // _LAE_PLATFORM_H
