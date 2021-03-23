////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekState.h
//
/*! \file
 *
 * $LastChangedDate: 2014-09-18 16:53:49 -0600 (Thu, 18 Sep 2014) $
 * $Rev: 3748 $
 *
 * \brief Hekateros Robot State classes interface.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2012-2017. RoadNarrows LLC.\n
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

#ifndef _HEK_STATE_H
#define _HEK_STATE_H

#include <string>
#include <vector>

#include "rnr/rnrconfig.h"

#include "Dynamixel/Dynamixel.h"

#include "Hekateros/hekateros.h"

namespace hekateros
{
  // ---------------------------------------------------------------------------
  // Class HekServoHealth
  // ---------------------------------------------------------------------------
 
  /*!
   * \brief Robot servo health.
   */
  class HekServoHealth
  {
  public:
    /*!
     * \brief Default constructor.
     */
    HekServoHealth();

    /*!
     * \brief Copy constructor.
     */
    HekServoHealth(const HekServoHealth &src);

    /*!
     * \brief Destructor.
     */
    ~HekServoHealth()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns copy of this.
     */
    HekServoHealth &operator=(const HekServoHealth &rhs);

    int     m_nServoId;       ///< servo id
    float   m_fTemperature;   ///< servo temperature (Celsius)
    float   m_fVoltage;       ///< servo voltage (volts)
    uint_t  m_uAlarms;        ///< servo alarms
  };


  // ---------------------------------------------------------------------------
  // Class HekRobotState
  // ---------------------------------------------------------------------------
 
  /*!
   * \brief Robot overall state.
   */
  class HekRobotState
  {
  public:
    typedef std::vector<HekServoHealth>   VecHealth;

    /*!
     * \brief Default constructor.
     */
    HekRobotState()
    {
      clear();
    }

    /*!
     * \brief Copy constructor.
     */
    HekRobotState(const HekRobotState &src);

    /*!
     * \brief Destructor.
     */
    ~HekRobotState()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns copy of this.
     */
    HekRobotState &operator=(const HekRobotState &rhs);

    /*!
     * \brief Clear data.
     */
    void clear();

    HekRobotMode    m_eRobotMode;         ///< robot operating mode
    HekTriState     m_eIsCalibrated;      ///< robot is [not] calibrated
    HekTriState     m_eIsEStopped;        ///< robot is [not] emergency stopped
    HekTriState     m_eAreDrivesPowered;  ///< servos are [not] powered
    HekTriState     m_eIsMotionPossible;  ///< motion is [not] possible
    HekTriState     m_eIsInMotion;        ///< robot is [not] moving
    HekTriState     m_eIsInError;         ///< robot is [not] in error condition
    int             m_nErrorCode;         ///< hekateros error code
    VecHealth       m_vecServoHealth;     ///< servos' health
  };

} // namespace hekateros

#endif // _HEK_STATE_H
