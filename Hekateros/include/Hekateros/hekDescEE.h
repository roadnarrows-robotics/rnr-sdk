////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekDescEE.h
//
/*! \file
 *
 * $LastChangedDate: 2014-09-18 16:53:49 -0600 (Thu, 18 Sep 2014) $
 * $Rev: 3748 $
 *
 * \brief HekDescEE - Hekateros end effector tool description class interface.
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

#ifndef _HEK_DESC_EE_H
#define _HEK_DESC_EE_H

#include <vector>

#include "Hekateros/hekateros.h"
#include "Hekateros/hekSpec.h"
#include "Hekateros/hekProdEE.h"


namespace hekateros
{
  //
  // Forward declarations
  //
  class HekDesc;
  class HekRobot;

  /*!
   * \brief Hekateros end effector tool description class.
   */
  class HekDescEE
  {
  public: 

    /*!
     * \breif Default constructor.
     */
    HekDescEE();
  
    /*!
     * \breif Destructor.
     */
    virtual ~HekDescEE()
    {
    }
  
    /*
     * Set \h_hek's end effector tool description.
     *
     * \param eProdId       \h_hek end effector product id.
     *                      See \ref HekEEProdId.
     * \param strProdName   Product name. If string is empty, then the default
     *                      name from product id is used.
     * \param strProdBrief  Product brief. If string is empty, then the default
     *                      brief from product id is used.
     * \param strHwVer      Hardware version.
     * \param nDoF          Degrees of Freedom. If 0, then the value
     *                      encoded in the product id is used.
     * \param eProdSize     Product size. If HekProdSizeUnknown, then the value
     *                      encoded in the product id is used.
     *                      See \ref HekProdSize.
     */
    void setDesc(int                eProdId,
                 const std::string &strProdName="",
                 const std::string &strProdBrief="",
                 const std::string &strHwVer="1.0.0",
                 int                nDoF=0,
                 int                eProdSize=HekProdSizeUnknown);

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
      return (m_eProdId != HekProdIdUnknown)? true: false;
    }

    /*!
     * \brief Get this end effector description's product id.
     *
     * \return Returns product id. See \ref HekEEProdId.
     */
    int getProdId()
    {
      return m_eProdId;
    }
  
    /*!
     * \brief Get this end effector description's name.
     *
     * \return Returns string.
     */
    std::string getProdName()
    {
      return m_strProdName;
    }
  
    /*!
     * \brief Get this end effector description's brief.
     *
     * \return Returns string.
     */
    std::string getProdBrief()
    {
      return m_strProdBrief;
    }
  
    /*!
     * \brief Get this end effector description's hardware version.
     *
     * \return Returns string.
     */
    std::string getProdHwVer()
    {
      return m_strProdHwVer;
    }
  
    /*!
     * \brief Get this end effector description's size.
     *
     * \return Returns product size. See \ref HekProdSize.
     */
    int getProdSize()
    {
      return m_eProdSize;
    }
  
    /*!
     * \brief Get this end effector description's degrees of freedom.
     *
     * \return Returns DoF.
     */
    int getDoF()
    {
      return m_nDoF;
    }

    /*!
     * \brief Get the number of expected and required servos.
     *
     * \return Returns number of servos.
     */
    int getNumServos()
    {
      return m_spec.getNumServos();
    }

    /*!
     * \brief Test if servo id is in the list of servos.
     *
     * \param nServoId    Servo id.
     *
     * \return Returns true or false.
     */
    bool hasServo(int nServoId)
    {
      return m_spec.hasServo(nServoId);
    }

  
    /*!
     * \brief Get the \h_hek end effector product name string given the
     * product id.
     *
     * \param eProdId   Supported product id. See \ref HekEEProdId.
     *
     * \return Returns product name string.
     * An unidentified product id returns "".
     */
    static const char *getProdName(int eProdId);
  
    /*!
     * \brief Get the \h_hek end effector product one-line brief description
     * string given the product id.
     *
     * \param eProdId   Supported product id. See \ref HekEEProdId.
     *
     * \return Returns product description string.
     * An unidentified product id returns "".
     */
    static const char *getProdBrief(int eProdId);
  
    /*!
     * \brief Get the \h_hek end effector size given the product id.
     *
     * \param eProdId   Supported product id. See \ref HekEEProdId.
     *
     * \return Returns product size value.
     * An unidentified product id returns HekProdSizeUknown.
     */
    static int getProdSize(int eProdId);

    /*!
     * \brief Get the \h_hek end effector degrees of freedom given the product
     * id.
     *
     * \param eProdId   Supported product id. See \ref HekEEProdId.
     *
     * \return Returns the degrees of freedom.
     */
    static int getDoF(int eProdId);

  protected: 
    int           m_eProdId;        ///< end effector product id
    std::string   m_strProdName;    ///< product name
    std::string   m_strProdBrief;   ///< product brief
    std::string   m_strProdHwVer;   ///< product hardware version
    uint_t        m_uProdHwVer;     ///< product hardware version number
    int           m_eProdSize;      ///< product size code
    int           m_nDoF;           ///< degrees of freedom
    HekSpec       m_spec;           ///< fixed specification

    friend class HekDesc;
    friend class HekRobot;
  };

} // namespace hekaeros


#endif // _HEK_DESC_EE_H
