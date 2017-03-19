////////////////////////////////////////////////////////////////////////////////
//
// Package:   Kuon
//
// Library:   libkuon
//
// File:      kuonSpec.h
//
/*! \file
 *
 * $LastChangedDate: 2014-04-04 17:05:03 -0600 (Fri, 04 Apr 2014) $
 * $Rev: 3629 $
 *
 * \brief \h_kuon product specification base classes.
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

#ifndef _KUON_SPEC_H
#define _KUON_SPEC_H

#include <string>
#include <vector>

#include "rnr/rnrconfig.h"

#include "Kuon/kuon.h"

namespace kuon
{
  //
  // Forward declarations.
  //
  class KuonDesc;
  class KuonRobot;

  /*!
   * \brief Supported joint/wheel types. Not really used for now.
   */
  enum KuonJointType
  {
    KuonJointTypeUnknown    = 0,  ///< unknown/undefined joint type
    KuonJointTypeFixed,           ///< fixed joint
    KuonJointTypeRevolute,        ///< limited rotation
    KuonJointTypeContinuous,      ///< continuous rotation
    KuonJointTypeRevMimic,        ///< mimic rotation (e.g. fingers)

    KuonJointTypeNumOf     = 4    ///< number of supported joint types
  };

  /*!
   * \brief Joint/wheel limit detection types. Not really used for now.
   *
   * A joint may have more than one type.
   */
  enum KuonLimitType
  {
    KuonLimitTypeUnknown = 0x00,   ///< unknown/undefined joint type
    KuonLimitTypeNone    = 0x01,   ///< no limit detection
    KuonLimitTypePhys    = 0x02,   ///< physical
    KuonLimitTypeElec    = 0x04,   ///< electronic
    KuonLimitTypeElecTDC = 0x08,   ///< electronic top dead center
    KuonLimitTypeAbsEnc  = 0x10,   ///< absolute encoder

    KuonLimitTypeNumOf   = 5       ///< number of supported joint types
  };


  // ---------------------------------------------------------------------------
  // Struct KuonSpecMotor_T
  // ---------------------------------------------------------------------------
 
  /*!
   * \brief Robotic motor specification.
   */
  struct KuonSpecMotor_T
  {
    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return this.
     */
    KuonSpecMotor_T operator=(const KuonSpecMotor_T &rhs)
    {
      int i;

      m_strName         = rhs.m_strName;
      m_nMotorId        = rhs.m_nMotorId;
      m_nMotorCtlrId    = rhs.m_nMotorCtlrId;
      m_nMotorIndex     = rhs.m_nMotorIndex;
      m_fGearRatio      = rhs.m_fGearRatio;
      m_fTireRadius     = rhs.m_fTireRadius;
      m_nDir            = rhs.m_nDir;

      return *this;
    }

    std::string m_strName;          ///< motor name
    int         m_nMotorId;         ///< robot unique robot motor id
    int         m_nMotorCtlrId;     ///< motor controller id
    int         m_nMotorIndex;      ///< motor controller unique motor index
    double      m_fGearRatio;       ///< motor gear ratio
    double      m_fTireRadius;      ///< tire radius (mm)
    int         m_nDir;             ///< normalize cw/ccw direction.
  };

  // ---------------------------------------------------------------------------
  // Class KuonSpec
  // ---------------------------------------------------------------------------

  /*!
   * \brief Robotic specification.
   */
  class KuonSpec
  {
  public:
    /*!
     * \brief Constructor.
     */
    KuonSpec();

    /*!
     * \brief Destructor.
     */
    ~KuonSpec();

    /*!
     * \brief Set product fixed specification.
     *
     * \param eProdId           \h_kuon arm product id.
     * \param uHwVer            Hardware version.
     * \param fFrontTireRadius  Radius of front tires (mm). If zero, then
     *                          factory defaults are used.
     * \param fRearTireRadius   Radius of rear tires (mm). If zero, then
     *                          factory defaults are used.
     */
    int set(int    eProdId,
            uint_t uHwVer,
            double fFrontTireRadius,
            double fRearTireRadius);

    /*!
     * \brief Clear product fixed specification.
     */
    void clear();

    /*!
     * \brief Get specification's product id.
     *
     * \return Returns product id. See \ref KuonProdId.
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
     * \brief Get specification's number of motors.
     *
     * \return Returns number of motors.
     */
    int getNumMotors()
    {
      return m_nNumMotors;
    }

    /*!
     * \brief Get motor spec at the given index.
     *
     * \param index   Motor index.
     *
     * \return If found, returns pointer to spec. Otherwise returns NULL.
     */
    KuonSpecMotor_T *getMotorSpecAt(int index)
    {
      return index < m_vecSpecMotors.size()? &m_vecSpecMotors.at(index): NULL;
    }

    /*!
     * \brief Get motor spec associated motor name.
     *
     * \param strName   Motor name (primary key).
     *
     * \return If found, returns pointer to spec. Otherwise returns NULL.
     */
    KuonSpecMotor_T *getMotorSpec(const std::string &strName);

    /*!
     * \brief Get motor spec associated with motor id.
     *
     * \param nMotorId  Motor id (secondary key).
     *
     * \return If found, returns pointer to spec. Otherwise returns NULL.
     */
    KuonSpecMotor_T *getMotorSpec(int nMotorId);

    /*!
     * \brief Test if motor id is in the motor specifications.
     *
     * \return Returns true or false.
     */
    bool hasMotor(int nMotorId)
    {
      return getMotorSpec(nMotorId) == NULL? false: true;
    }

    /*!
     * \brief Get front and rear tire radii (mm).
     *
     * \param [out] fFrontTireRadius    Front tires.
     * \param [out] fRearTireRadius     Rear tires.
     */
    void getTireSizes(double &fFrontTireRadius, double &fRearTireRadius);

  protected:
    int             m_eProdId;            ///< product id
    uint_t          m_uHwVer;             ///< hardware version
    int             m_nNumMotors;         ///< number of drive motors
    double          m_fFrontTireRadius;   ///< front tire radius (mm)
    double          m_fRearTireRadius;    ///< rear tire radius (mm)

    std::vector<KuonSpecMotor_T>  m_vecSpecMotors;  ///< vector of motor specs

    friend class KuonDesc;
    friend class KuonRobot;
  };
} // namespace kuon


#endif // _KUON_SPEC_H
