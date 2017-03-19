////////////////////////////////////////////////////////////////////////////////
//
// Package:   Kuon
//
// Library:   libkuon
//
// File:      kuonDesc.h
//
/*! \file
 *
 * $LastChangedDate: 2014-04-04 17:05:03 -0600 (Fri, 04 Apr 2014) $
 * $Rev: 3629 $
 *
 * \brief Kuon full robotic mobile platform descripition class interface.
 *
 * The description includes the base platform plus any supported sensor and
 * robotic payloads.
 *
 * Descriptions are a mash-up of factory fixed specificiations plus run-time
 * XML configuration.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2014-2017. RoadNarrows LLC.\n
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

#ifndef _KUON_DESC_H
#define _KUON_DESC_H

#include <math.h>
#include <vector>

#include "Kuon/kuon.h"
#include "Kuon/kuonDescBase.h"


namespace kuon
{
  //
  // Forward declaraions.
  //
  class KuonRobot;

  /*!
   * \brief Kuon robotic manipulator full description class.
   *
   * The description is determined from the XML etc configuration file. From
   * those description components, the compiled specifications are assigned.
   */
  class KuonDesc
  {
  public: 

    /*!
     * \breif Default constructor.
     */
    KuonDesc()
    {
    }
  
    /*!
     * \breif Destructor.
     */
    ~KuonDesc()
    {
    }
  
    /*
     * Set \h_kuon's robotic mobile base description.
     *
     * \param eProdId           \h_kuon base product id. See \ref KuonProdId.
     * \param strProdName       Product name. If string is empty, then the
     *                          default name from product id is used.
     * \param strProdBrief      Product brief. If string is empty, then the
     *                          default brief from product id is used.
     * \param strHwVer          Hardware version.
     * \param fFrontTireRadius  Radius of front tires (mm). If zero, then
     *                          factory defaults are used.
     * \param fRearTireRadius   Radius of rear tires (mm). If zero, then
     *                          factory defaults are used.
     */
    void setDesc(int                eProdId,
                 const std::string &strProdName="",
                 const std::string &strProdBrief="",
                 const std::string &strHwVer="1.0.0",
                 double             fFrontTireRadius=0.0,
                 double             fRearTireRadius=0.0)
    {
      m_descBase.setDesc(eProdId, strProdName, strProdBrief, strHwVer,
                         fFrontTireRadius, fRearTireRadius);
    }

    /*!
     * \brief Reset \h_kuon description to the "unitialized" values.
     */
    void resetDesc();

    /*!
     * \brief Mark \h_kuon hardware as fully described.
     *
     * The calling application context determines this state.
     *
     * \copydoc doc_return_std
     */
    int markAsDescribed();

    /*!
     * \brief Test if required descriptions are described.
     *
     * \return Returns true or false.
     */
    bool isDescribed()
    {
      return  m_bIsDescribed && (m_descBase.getProdId() != KuonProdIdUnknown)?
                        true: false;
    }

    /*!
     * \brief Get the \h_kuon base product description.
     *
     * \return Returns pointer to robotic base description.
     */
    KuonDescBase *getBaseDesc()
    {
      return &m_descBase;
    }
  
    /*!
     * \brief Convenience function to get this \h_kuon description's base
     * product id.
     *
     * \return Returns product id. See \ref KuonProdId.
     */
    int getProdId()
    {
      return m_descBase.getProdId();
    }
  
    /*!
     * \brief Get the \h_kuon full brief descirption.
     *
     * \return Returns string.
     */
    std::string getFullProdBrief()
    {
      return m_strFullBrief;
    }
  
    /*!
     * \brief Get the number of expected and required motors.
     *
     * \return Returns number of motors.
     */
    int getNumMotors()
    {
      return m_descBase.getNumMotors();
    }

    /*!
     * \brief Test if motor id is in the list of motors.
     *
     * \return Returns true or false.
     */
    bool hasMotor(int nMotorId)
    {
      return m_descBase.m_spec.hasMotor(nMotorId);
    }

  protected: 
    bool          m_bIsDescribed; ///< \h_kuon is [not] fully described
    KuonDescBase  m_descBase;     ///< \h_kuon robotic arm
    std::string   m_strFullBrief; ///< product with payload full brief

    friend class KuonRobot;
  };

} // namespace kuon

#endif // _KUON_DESC_H
