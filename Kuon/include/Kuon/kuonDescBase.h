////////////////////////////////////////////////////////////////////////////////
//
// Package:   Kuon
//
// Library:   libkuon
//
// File:      kuonDescBase.h
//
/*! \file
 *
 * $LastChangedDate: 2014-04-04 17:05:03 -0600 (Fri, 04 Apr 2014) $
 * $Rev: 3629 $
 *
 * \brief Kuon robotic base mobile platform description class interface.
 *
 * The base description does not include any payload descriptions.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2014.  RoadNarrows
 * (http://www.RoadNarrows.com)
 * \n All Rights Reserved
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

#ifndef _KUON_DESC_BASE_H
#define _KUON_DESC_BASE_H

#include <vector>

#include "rnr/rnrconfig.h"

#include "Kuon/kuon.h"
#include "Kuon/kuonSpec.h"
#include "Kuon/kuonProdBase.h"


namespace kuon
{
  //
  // Forward declarations
  //
  class KuonDesc;
  class KuonRobot;

  /*!
   * \brief Kuon robotic mobile base escription class.
   */
  class KuonDescBase
  {
  public: 

    /*!
     * \breif Default constructor.
     */
    KuonDescBase();
  
    /*!
     * \breif Destructor.
     */
    ~KuonDescBase()
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
                 double             fRearTireRadius=0.0);

    /*!
     * \brief Reset base description to the "unitialized" values.
     */
    void resetDesc();

    /*!
     * \brief Test if required base description is adequately described.
     *
     * \return Returns true or false.
     */
    bool isDescribed()
    {
      return (m_eProdId != KuonProdIdUnknown)? true: false;
    }

    /*!
     * \brief Get this base description's base product id.
     *
     * \return Returns product id. See \ref KuonProdId.
     */
    int getProdId()
    {
      return m_eProdId;
    }
  
    /*!
     * \brief Get this base description's name.
     *
     * \return Returns string.
     */
    std::string getProdName()
    {
      return m_strProdName;
    }
  
    /*!
     * \brief Get this base description's brief.
     *
     * \return Returns string.
     */
    std::string getProdBrief()
    {
      return m_strProdBrief;
    }
  
    /*!
     * \brief Get this base description's hardware version.
     *
     * \return Returns string.
     */
    std::string getProdHwVer()
    {
      return m_strProdHwVer;
    }
  
    /*!
     * \brief Get the number of expected and required motors.
     *
     * \return Returns number of motors.
     */
    int getNumMotors()
    {
      return m_spec.getNumMotors();
    }

    /*!
     * \brief Test if motor id is in the list of motors.
     *
     * \param nMotorId    Motor id.
     *
     * \return Returns true or false.
     */
    bool hasMotor(int nMotorId)
    {
      return m_spec.hasMotor(nMotorId);
    }

    /*!
     * \brief Get the \h_kuon product name string given the product id.
     *
     * \param eProdId Supported product id. See \ref KuonProdId.
     *
     * \return Returns product name string. An unidentified product id
     * returns "".
     */
    static const char *getProdName(int eProdId);
  
    /*!
     * \brief Get the \h_kuon product one-line brief description string given
     * the product id.
     *
     * \param eProdId Supported product id. See \ref KuonProdId.
     *
     * \return Returns product description string. An unidentified product id
     * returns "".
     */
    static const char *getProdBrief(int eProdId);
  
  protected: 
    int           m_eProdId;        ///< base product id
    std::string   m_strProdFamily;  ///< product name
    std::string   m_strProdName;    ///< product name
    std::string   m_strProdBrief;   ///< product brief
    std::string   m_strProdHwVer;   ///< product hardware version string
    uint_t        m_uProdHwVer;     ///< product hardware version number
    KuonSpec      m_spec;           ///< fixed specification

    friend class KuonDesc;
    friend class KuonRobot;
  };

} // namespace kuon

#endif // _KUON_DESC_BASE_H
