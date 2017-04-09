////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Library:   liblaelaps
//
// File:      laePowertrain.h
//
/*! \file
 *
 * \brief Laelaps powertrain class interfaces.
 *
 * There are four powertrains. A powertrain is defined as:\n
 * motor - encoder - gears - wheel - tire.
 *
 * Traditional actuated articulation is called a joint. Powertrains
 * are the near equivalent in Laelaps platforms.
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

#ifndef _LAE_POWERTRAIN_H
#define _LAE_POWERTRAIN_H

#include <string>
#include <vector>

#include "rnr/rnrconfig.h"

#include "Laelaps/laelaps.h"
#include "Laelaps/laeDesc.h"
#include "Laelaps/laeTune.h"

namespace laelaps
{
  // ---------------------------------------------------------------------------
  // Class LaePowertrainAttr
  // ---------------------------------------------------------------------------

  /*!
   * \brief  Powertrain attributes class.
   *
   * This class encapsulates the semi-fixed attribute data defining the
   * characteristics of a powertrain.
   */
  class LaePowertrainAttr
  {
  public:
    // (derived) attribute data
    int     m_nMotorId;           ///< motor id
    int     m_nMotorCtlrId;       ///< motor controller id
    int     m_nMotorIndex;        ///< motor controller unique motor index

    int     m_nMotorDir;          ///< motor normalized direction
    int     m_eJointType;         ///< joint type

    uint_t  m_uPulsesPerRev;      ///< encoder pulses per motor revolution
    double  m_fGearRatio;         ///< gear ratio
    double  m_fMaxRps;            ///< maximum rated output shaft rev/sec
    uint_t  m_uMaxQpps;           ///< maximum quadrature pulses/second @max rps
    double  m_fMotorRadsPerPulse; ///< motor radians per encoder pulse
    double  m_fWheelRadsPerPulse; ///< output shaft radians per encoder pulse

    double  m_fMaxAmps;           ///< maximum rated amps
    double  m_fStallTorque;       ///< stall torque @max amps

    double  m_fTireRadius;        ///< tire radius
    double  m_fTireWidth;         ///< tire width
    double  m_fMetersPerPulse;    ///< tire meters per encoder pulse
    double  m_fMetersPerRadian;   ///< tire meters per radian

    /*!
     * \brief Default constructor.
     */
    LaePowertrainAttr();

    /*!
     * \brief Copy constructor.
     */
    LaePowertrainAttr(const LaePowertrainAttr &src);

    /*!
     * \brief Destructor.
     */
    ~LaePowertrainAttr()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns copy of this.
     */
    LaePowertrainAttr operator=(const LaePowertrainAttr &rhs);
  }; // LaePowertrainAttr


  // ---------------------------------------------------------------------------
  // Class LaePowertrainState
  // ---------------------------------------------------------------------------

  /*!
   * \brief Powertrain state data class.
   *
   * This class encapsulates the current dynamic state of one powertrain.
   */
  class LaePowertrainState
  {
  public:
    // sensed state data
    s64_t       m_nEncoder;       ///< motor encoder position (quad pulses)
    int         m_nSpeed;         ///< raw speed (qpps)
    double      m_fTemp;          ///< motor temperature (C)
    double      m_fVolts;         ///< input motor voltage (V)
    double      m_fAmps;          ///< motor draw (A)
    uint_t      m_uBufLen;        ///< command queue length
    uint_t      m_uAlarms;        ///< motor alarms
    uint_t      m_uWarnings;      ///< motor warnings

    // derived state data
    double      m_fPosition;      ///< wheel angular position (radians)
    double      m_fVelocity;      ///< wheel angular velocity (radians/second)
    double      m_fTorque;        ///< wheel torque (N-m)
    double      m_fPe;            ///< motor input electrical power (W)
    double      m_fPm;            ///< motor output mechanical power (W)
   
    /*!
     * \brief Default constructor.
     */
    LaePowertrainState();

    /*!
     * \brief Copy constructor.
     */
    LaePowertrainState(const LaePowertrainState &src);

    /*!
     * \brief Destructor.
     */
    ~LaePowertrainState()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns copy of this.
     */
    LaePowertrainState operator=(const LaePowertrainState &rhs);
  }; // class LaePowertrainState


  // ---------------------------------------------------------------------------
  // Class LaePowertrain
  // ---------------------------------------------------------------------------

  /*!
   * \brief  Powertrain data class.
   *
   * This class encapsulates the fixed data and current state of one powertrain.
   */
  class LaePowertrain
  {
  public:
    std::string         m_strName;    ///< powertrain unique name (key)
    LaePowertrainAttr   m_attr;       ///< semi-fixed attribute data
    LaePowertrainState  m_state;      ///< dynamic state data

    /*!
     * \brief Default constructor.
     */
    LaePowertrain();

    /*!
     * \brief Copy constructor.
     */
    LaePowertrain(const LaePowertrain &src);

    /*!
     * \brief Destructor.
     */
    ~LaePowertrain()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns copy of this.
     */
    LaePowertrain operator=(const LaePowertrain &rhs);

    /*!
     * \brief Map motor controller id and motor index to motor id.
     *
     * \param nCtrlId       Motor controller id.
     * \param nMotorIndex   Motor controller motor index 
     *
     * \return Returns unique motor id.
     */
    static int toMotorId(const int nCtlrId, const int nMotorIndex);
    
    /*!
     * \brief Map motor controller id and motor index to powertrain name (key).
     *
     * \param nCtrlId       Motor controller id.
     * \param nMotorIndex   Motor controller motor index 
     *
     * \return Returns key on success, empty string if unknown.
     */
    static std::string toKey(const int nCtlrId, const int nMotorIndex);
    
    /*!
     * \brief Map motor id to powertrain name (key).
     *
     * \param nMotorId      Motor id.
     *
     * \return Returns key on success, empty string if unknown.
     */
    static std::string toKey(const int nMotorId);

    /*!
     * \brief Get this powertrain unique name (key).
     *
     * \return String.
     */
    std::string getName() const
    {
      return m_strName;
    }

    /*!
     * \brief Get motor id.
     *
     * \return Returns motor id.
     */
    int getMotorId() const
    {
      return m_attr.m_nMotorId;
    }

    /*!
     * \brief Get motor controller id.
     *
     * \return Returns controller id.
     */
    int getMotorCtlrId() const
    {
      return m_attr.m_nMotorCtlrId;
    }

    /*!
     * \brief Get motor controller motor index.
     *
     * \return Returns controller motor index.
     */
    int getMotorIndex() const
    {
      return m_attr.m_nMotorIndex;
    }

    /*!
     * \brief Configure powertrain from product description.
     *
     * \param desc  Laelaps powertrain product description data.
     *
     * \copydoc doc_return_std
     */
    virtual int configure(const LaeDescPowertrain &desc);

    /*!
     * \brief Configure powertrain from tune parameters.
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
     * \brief Update state dynamics.
     *
     * \param nEncoder  Current encoder position (quad pulses).
     * \param nSpeed    Current motor speed (qpps).
     * \param fAmps     Current motor draw (amperes).
     * \param uBufLen   Current command queue length.
     *
     * \copydoc doc_return_std
     */
    virtual int updateStateDynamics(s64_t   nEncoder,
                                    s32_t   nSpeed,
                                    double  fAmps,
                                    uint_t  uBufLen);

    /*!
     * \brief Update motor health state.
     *
     * \param fVolts      Motor input voltage (V).
     * \param fTemp       Motor controller board voltage (V).
     * \param uCtlrStatus Motor controller status bits.
     *
     * \copydoc doc_return_std
     */
    virtual int updateHealth(double fVolts, double fTemp, uint_t uCtlrStatus);

  }; // class LaePowertrain


  // ---------------------------------------------------------------------------
  // Other Types, Constants, Defines
  // ---------------------------------------------------------------------------

  /*!
   * \brief Map of powertrain kinodynamics.
   *
   * \termblock
   * \term key: \termdata powertrain unique name \endterm
   * \term mapped type: \termdata powertrain attribute and state data \endterm
   * \endtermblock
   */
  typedef std::map<std::string, LaePowertrain> LaeMapPowertrain;

} // namespace laelaps


#endif // _LAE_POWERTRAIN_H
