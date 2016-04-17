////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekDesc.h
//
/*! \file
 *
 * $LastChangedDate: 2014-09-18 16:53:49 -0600 (Thu, 18 Sep 2014) $
 * $Rev: 3748 $
 *
 * \brief HekDesc - Hekateros full robotic manipulator descripition class
 * interface.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2012-2013.  RoadNarrows
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

#ifndef _HEK_DESC_H
#define _HEK_DESC_H

#include <math.h>
#include <vector>

#include "Hekateros/hekateros.h"
#include "Hekateros/hekDescArm.h"
#include "Hekateros/hekDescEE.h"


namespace hekateros
{
  //
  // Forward declaraions.
  //
  class HekRobot;

  /*!
   * \brief Hekateros robotic manipulator full description class.
   *
   * The description is determined from the XML etc configuration file. From
   * those description components, the compiled specifications are assigned.
   */
  class HekDesc
  {
  public: 

    /*!
     * \breif Default constructor.
     */
    HekDesc()
    {
    }
  
    /*!
     * \breif Destructor.
     */
    ~HekDesc()
    {
    }
  
    /*
     * Set \h_hek's robotic arm (base) description.
     *
     * \param eProdId       \h_hek base product id. See \ref HekProdId.
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
    void setArmDesc(int                eProdId,
                    const std::string &strProdName="",
                    const std::string &strProdBrief="",
                    const std::string &strHwVer="1.0.0",
                    int                nDoF=0,
                    int                eProdSize=HekProdSizeUnknown)
    {
      m_descArm.setDesc(eProdId, strProdName, strProdBrief, strHwVer,
                         nDoF, eProdSize);
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
    void setEEDesc(int                eEEProdId,
                   const std::string &strProdName="",
                   const std::string &strProdBrief="",
                   const std::string &strHwVer="1.0.0",
                   int                nDoF=0,
                   int                eProdSize=HekProdSizeUnknown)
    {
      m_descEE.setDesc(eEEProdId, strProdName, strProdBrief, strHwVer,
                        nDoF, eProdSize);
    }

    /*!
     * \brief Reset \h_hek description to the "unitialized" values.
     */
    void resetDesc();

    /*!
     * \brief Mark \h_hek hardware as fully described.
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
      return  m_bIsDescribed &&
             (m_descArm.getProdId() != HekProdIdUnknown) &&
             (m_descEE.getProdId()  != HekProdIdUnknown)? true: false;
    }

    /*!
     * \brief Get the \h_hek base product description.
     *
     * \return Returns pointer to robotic base description.
     */
    HekDescArm *getArmDesc()
    {
      return &m_descArm;
    }
  
    /*!
     * \brief Get the \h_hek end effector product description.
     *
     * \return Returns pointer to end effector description.
     */
    HekDescEE *getEEDesc()
    {
      return &m_descEE;
    }
  
    /*!
     * \brief Convenience function to get this \h_hek description's base
     * product id.
     *
     * \return Returns product id. See \ref HekProdId.
     */
    int getProdId()
    {
      return m_descArm.getProdId();
    }
  
    /*!
     * \brief Get the \h_hek full brief descirption.
     *
     * \return Returns string.
     */
    std::string getFullProdBrief()
    {
      return m_strFullBrief;
    }
  
    /*!
     * \brief Check if this \h_hek description has an end effector description.
     *
     * \return Returns true or false.
     */
    bool hasEndEffector()
    {
      return m_descEE.getProdId() != HekProdIdUnknown? true: false;
    }

    /*!
     * \brief Get this \h_hek description's total degrees of freedom.
     *
     * \return Returns DoF.
     */
    int getDoF()
    {
      return m_descArm.getDoF() + m_descEE.getDoF();
    }

    /*!
     * \brief Get the number of expected and required servos.
     *
     * \return Returns number of servos.
     */
    int getNumServos()
    {
      return m_descArm.getNumServos() + m_descEE.getNumServos();
    }

    /*!
     * \brief Test if servo id is in the list of servos.
     *
     * \return Returns true or false.
     */
    bool hasServo(int nServoId);

  protected: 
    bool              m_bIsDescribed; ///< \h_hek is [not] fully described
    HekDescArm        m_descArm;      ///< \h_hek robotic arm
    HekDescEE         m_descEE;       ///< \h_hek end effector tool
    //HekDescEquip      m_descEquip;     ///< \h_hek equipment deck
    //HekDescAux        m_descAux;      ///< \h_hek base auxilliary hardware
    std::string       m_strFullBrief; ///< product with accessories full brief

    friend class HekRobot;
  };

} // namespace hekaeros

#endif // _HEK_DESC_H
