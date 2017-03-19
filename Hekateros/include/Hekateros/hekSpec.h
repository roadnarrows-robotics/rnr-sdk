////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekSpec.h
//
/*! \file
 *
 * $LastChangedDate: 2014-09-18 16:53:49 -0600 (Thu, 18 Sep 2014) $
 * $Rev: 3748 $
 *
 * \brief \h_hek product specification base classes.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2013-2017. RoadNarrows LLC.\n
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

#ifndef _HEK_SPEC_H
#define _HEK_SPEC_H

#include <string>
#include <vector>

#include "rnr/rnrconfig.h"

#include "Dynamixel/Dynamixel.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekOptical.h"

namespace hekateros
{
  //
  // Forward declarations.
  //
  class HekDesc;
  class HekRobot;

  /*!
   * \brief Supported joint types.
   */
  enum HekJointType
  {
    HekJointTypeUnknown   = 0,    ///< unknown/undefined joint type
    HekJointTypeFixed,            ///< fixed joint
    HekJointTypeRevolute,         ///< limited rotation
    HekJointTypeContinuous,       ///< continuous rotation
    HekJointTypeRevMimic,         ///< mimic rotation (e.g. fingers)

    HekJointTypeNumOf     = 4     ///< number of supported joint types
  };

  /*!
   * \brief Joint limit detection types.
   *
   * A joint may have more than one type.
   */
  enum HekLimitType
  {
    HekLimitTypeUnknown = 0x00,   ///< unknown/undefined joint type
    HekLimitTypeNone    = 0x01,   ///< no limit detection
    HekLimitTypePhys    = 0x02,   ///< physical
    HekLimitTypeElec    = 0x04,   ///< electronic
    HekLimitTypeElecTDC = 0x08,   ///< electronic top dead center
    HekLimitTypeAbsEnc  = 0x10,   ///< absolute encoder

    HekLimitTypeNumOf   = 5       ///< number of supported joint types
  };

  // ---------------------------------------------------------------------------
  // Struct HekSpecLink_T
  // ---------------------------------------------------------------------------
 
  /*!
   * \brief Robotic link specification.
   */
  struct HekSpecLink_T
  {
    /*!
     * \brief Assignment operator.
     * 
     * \note May add weights, cogs, etc.
     *
     * \param rhs   Right hand side object.
     *
     * \return this.
     */
    HekSpecLink_T operator=(const HekSpecLink_T &rhs)
    {
      m_strName     = rhs.m_strName;
      m_fLength     = rhs.m_fLength;

      return *this;
    }

    std::string m_strName;    ///< link name
    double      m_fLength;    ///< link length (mm)
  };


  // ---------------------------------------------------------------------------
  // Struct HekSpecJoint_T
  // ---------------------------------------------------------------------------
 
  /*!
   * \brief Robotic joint specification.
   */
  struct HekSpecJoint_T
  {
    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return this.
     */
    HekSpecJoint_T operator=(const HekSpecJoint_T &rhs)
    {
      int i;

      m_strName         = rhs.m_strName;
      m_nMasterServoId  = rhs.m_nMasterServoId;
      m_nSlaveServoId   = rhs.m_nSlaveServoId;
      m_eJointType      = rhs.m_eJointType;
      m_fGearRatio      = rhs.m_fGearRatio;
      m_fMinPhyLimit    = rhs.m_fMinPhyLimit;
      m_fMaxPhyLimit    = rhs.m_fMaxPhyLimit;
      m_eLimitTypes     = rhs.m_eLimitTypes;
      m_fCalibPos       = rhs.m_fCalibPos;
      m_fBalancedPos    = rhs.m_fBalancedPos;
      m_fParkedPos      = rhs.m_fParkedPos;
      m_nLinkParent     = rhs.m_nLinkParent;
      m_nLinkChild      = rhs.m_nLinkChild;

      for(i=0; i<HekOptLimitMaxPerJoint; ++i)
      {
        m_limit[i] = rhs.m_limit[i];
      }

      return *this;
    }

    std::string m_strName;          ///< joint name
    int         m_nMasterServoId;   ///< master servo id
    int         m_nSlaveServoId;    ///< linked slave servo id, if any
    int         m_eJointType;       ///< joint type
    double      m_fGearRatio;       ///< joint gear ratio
    double      m_fMinPhyLimit;     ///< joint minimum physical limit (degrees)
    double      m_fMaxPhyLimit;     ///< joint maximum physical limit (degrees)
    int         m_eLimitTypes;      ///< joint limit types
    HekOpticalLimit_T m_limit[HekOptLimitMaxPerJoint];
                                    ///< optical limits
    double      m_fCalibPos;        ///< joint calibrated position (degrees)
    double      m_fBalancedPos;     ///< joint balanced position (degrees)
    double      m_fParkedPos;       ///< joint parked position (degrees)
    int         m_nLinkParent;      ///< parent link index
    int         m_nLinkChild;       ///< child link index
  };


  // ---------------------------------------------------------------------------
  // Struct HekSpecServo_T
  // ---------------------------------------------------------------------------

  /*!
   * \brief Robotic servo specification.
   */
  struct HekSpecServo_T
  {
    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return this.
     */
    HekSpecServo_T operator=(const HekSpecServo_T &rhs)
    {
      m_nServoId        = rhs.m_nServoId;
      m_bMaster         = rhs.m_bMaster;
      m_bIsContinuous   = rhs.m_bIsContinuous;
      m_nDir            = rhs.m_nDir;
      m_fTorqueLimitPct = rhs.m_fTorqueLimitPct;
    }

    int     m_nServoId;       ///< servo id
    bool    m_bMaster;        ///< servo is [not] master in doulbe servo joints
    bool    m_bIsContinuous;  ///< servo should [not] be configured in 360 mode
    int     m_nDir;           ///< normalize cw/ccw direction.
    double  m_fTorqueLimitPct;///< torque limit(%). Set to 0 for no limit
  };


  // ---------------------------------------------------------------------------
  // Class HekSpec
  // ---------------------------------------------------------------------------

  /*!
   * \brief Robotic specification.
   */
  class HekSpec
  {
  public:
    /*!
     * \brief Constructor.
     */
    HekSpec();

    /*!
     * \brief Destructor.
     */
    ~HekSpec();

    /*!
     * \brief Set product fixed specification.
     *
     * \param eProdId \h_hek arm product id.
     * \param uHwVer  Hardware version.
     */
    int set(int eProdId, uint_t uHwVer);

    /*!
     * \brief Clear product fixed specification.
     */
    void clear();

    /*!
     * \brief Get specification's product family id.
     *
     * \return Returns family id.
     */
    int getProdFamily()
    {
      return m_eProdFamily;
    }
  
    /*!
     * \brief Get specification's product id.
     *
     * \return Returns product id. See \ref HekProdId.
     */
    int getProdId()
    {
      return m_eProdId;
    }
  
    /*!
     * \brief Get specification's product hardware version number.
     *
     * \return Returns version number.
     */
    uint_t getProdHwVer()
    {
      return m_uHwVer;
    }
  
    /*!
     * \brief Get specification's number of links.
     *
     * \return Returns number of links.
     */
    int getNumLinks()
    {
      return m_nNumLinks;
    }
  
    /*!
     * \brief Get specification's degrees of freedom.
     *
     * \return Returns DoF.
     */
    int getDoF()
    {
      return m_nDoF;
    }

    /*!
     * \brief Get specification's number of optical limits.
     *
     * \return Returns number of optical limits.
     */
    int getNumOpticalLimits()
    {
      return m_nNumOptLimits;
    }

    /*!
     * \brief Get specification's number of servos.
     *
     * \return Returns number of servos.
     */
    int getNumServos()
    {
      return m_nNumServos;
    }

    /*!
     * \brief Get link spec at the given index.
     *
     * \param index   Link index.
     *
     * \return If found, returns pointer to spec. Otherwise returns NULL.
     */
    HekSpecLink_T *getLinkSpecAt(int index)
    {
      return index < m_vecSpecLinks.size()? &m_vecSpecLinks.at(index): NULL;
    }

    /*!
     * \brief Get joint spec at the given index.
     *
     * \param index   Joint index.
     *
     * \return If found, returns pointer to spec. Otherwise returns NULL.
     */
    HekSpecJoint_T *getJointSpecAt(int index)
    {
      return index < m_vecSpecJoints.size()? &m_vecSpecJoints.at(index): NULL;
    }

    /*!
     * \brief Get joint spec associated joint name.
     *
     * \param strName   Joint name (primary key).
     *
     * \return If found, returns pointer to spec. Otherwise returns NULL.
     */
    HekSpecJoint_T *getJointSpec(std::string &strName);

    /*!
     * \brief Get joint spec associated with servo id.
     *
     * \param nServoId  Servo id (secondary key).
     *
     * \return If found, returns pointer to spec. Otherwise returns NULL.
     */
    HekSpecJoint_T *getJointSpec(int nServoId);

    /*!
     * \brief Get servo spec at the given index.
     *
     * \param index   Servo index.
     *
     * \return If found, returns pointer to spec. Otherwise returns NULL.
     */
    HekSpecServo_T *getServoSpecAt(int index)
    {
      return index < m_vecSpecServos.size()? &m_vecSpecServos.at(index): NULL;
    }

    /*!
     * \brief Get servo spec associated with servo id.
     *
     * \param nServoId    Servo id (key).
     *
     * \return If found, returns pointer to spec. Otherwise returns NULL.
     */
    HekSpecServo_T *getServoSpec(int nServoId);

    /*!
     * \brief Test if servo id is in the servo specifications.
     *
     * \return Returns true or false.
     */
    bool hasServo(int nServoId);

  protected:
    int                         m_eProdFamily;    ///< product family
    int                         m_eProdId;        ///< product id
    uint_t                      m_uHwVer;         ///< hardware version
    int                         m_nNumLinks;      ///< number of links
    int                         m_nDoF;           ///< degrees of freedom
    int                         m_nNumOptLimits;  ///< number of optical limits
    int                         m_nNumServos;     ///< number of servos
    std::vector<HekSpecLink_T>  m_vecSpecLinks;   ///< vector of link specs
    std::vector<HekSpecJoint_T> m_vecSpecJoints;  ///< vector of joint specs
    std::vector<HekSpecServo_T> m_vecSpecServos;  ///< vector of servo specs

    friend class HekDesc;
    friend class HekRobot;
  };
} // namespace hekateros


#endif // _HEK_SPEC_H
