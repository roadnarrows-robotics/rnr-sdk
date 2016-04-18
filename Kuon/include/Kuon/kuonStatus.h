////////////////////////////////////////////////////////////////////////////////
//
// Package:   Kuon
//
// Library:   libkuon
//
// File:      kuonStatus.h
//
/*! \file
 *
 * $LastChangedDate: 2014-04-04 17:05:03 -0600 (Fri, 04 Apr 2014) $
 * $Rev: 3629 $
 *
 * \brief Kuon Robot Status classes interfaces.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2014.  RoadNarrows
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
// Unless otherwise noted, all materials contained are copyrighted and may not
// be used except as provided in these terms and conditions or in the copyright
// notice (documents and software ) or other proprietary notice provided with
// the relevant materials.
//
//
// IN NO EVENT SHALL THE AUTHOR, ROADNARROWS, OR ANY MEMBERS/EMPLOYEES/
// CONTRACTORS OF ROADNARROWS OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
// PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
// EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.
//
// THE AUTHORS AND  ROADNARROWS SPECIFICALLY DISCLAIM ANY WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
// "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
// PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _KUON_STATUS_H
#define _KUON_STATUS_H

#include <string>
#include <vector>

#include "rnr/rnrconfig.h"

#include "Kuon/kuon.h"

namespace kuon
{
  // ---------------------------------------------------------------------------
  // Class KuonMotorHealth
  // ---------------------------------------------------------------------------
 
  /*!
   * \brief Robot motor health.
   */
  class KuonMotorHealth
  {
  public:
    /*!
     * \brief Default constructor.
     */
    KuonMotorHealth();

    /*!
     * \brief Copy constructor.
     */
    KuonMotorHealth(const KuonMotorHealth &src);

    /*!
     * \brief Destructor.
     */
    ~KuonMotorHealth()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns copy of this.
     */
    KuonMotorHealth operator=(const KuonMotorHealth &rhs);

    std::string m_strName;      ///< motor name
    int         m_nMotorId;     ///< motor id
    float       m_fTemperature; ///< motor temperature (Celsius)
    float       m_fVoltage;     ///< motor voltage (volts)
    uint_t      m_uAlarms;      ///< motor alarms
  };


  // ---------------------------------------------------------------------------
  // Class KuonRobotStatus
  // ---------------------------------------------------------------------------
 
  /*!
   * \brief Robot global status.
   */
  class KuonRobotStatus
  {
  public:
    typedef std::vector<KuonMotorHealth>   VecHealth;

    /*!
     * \brief Default constructor.
     */
    KuonRobotStatus()
    {
      clear();
    }

    /*!
     * \brief Copy constructor.
     */
    KuonRobotStatus(const KuonRobotStatus &src);

    /*!
     * \brief Destructor.
     */
    ~KuonRobotStatus()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns copy of this.
     */
    KuonRobotStatus operator=(const KuonRobotStatus &rhs);

    /*!
     * \brief Clear data.
     */
    void clear();

    KuonRobotMode   m_eRobotMode;         ///< robot operating mode
    KuonTriState    m_eIsEStopped;        ///< robot is [not] emergency stopped
    KuonTriState    m_eAreDrivesPowered;  ///< motor are [not] powered
    KuonTriState    m_eIsMotionPossible;  ///< motion is [not] possible
    KuonTriState    m_eIsInMotion;        ///< robot is [not] moving
    KuonTriState    m_eIsInError;         ///< robot is [not] in error condition
    int             m_nErrorCode;         ///< kuon error code
    float           m_fGovernor;          ///< speed limiting governor (0-1)
    float           m_fBattery;           ///< current battery energy (Wh)
    VecHealth       m_vecMotorHealth;     ///< motors' health
  };

} // namespace kuon

#endif // _KUON_STATUS_H
