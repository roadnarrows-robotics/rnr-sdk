////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekXmlCfg.h
//
/*! \file
 *
 * $LastChangedDate: 2015-08-13 19:23:07 -0600 (Thu, 13 Aug 2015) $
 * $Rev: 4065 $
 *
 * \brief HekXmlCfg - \h_hek XML configuration class interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2013-2016.  RoadNarrows LLC
 * (http://www.roadnarrows.com)
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

#ifndef _HEK_XML_CFG_H
#define _HEK_XML_CFG_H

#include <string>

#include "tinyxml.h"

#include "rnr/rnrconfig.h"
#include "rnr/appkit/Xml.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekDescArm.h"
#include "Hekateros/hekDescEE.h"
#include "Hekateros/hekDesc.h"


namespace hekateros
{
  //----------------------------------------------------------------------------
  // HekXmlCfg Class
  //----------------------------------------------------------------------------

  /*!
   * \brief HekXmlCfg \h_hek XML configuration class.
   */
  class HekXmlCfg : public rnr::Xml
  {
  public:
    /*!
     * \brief Default constructor.
     */
    HekXmlCfg() : 
      Xml("hekateros", HekXsiUrl, HekXslUrl),
      m_strMajElemArm("arm"),
      m_strMajElemEE("end_effector"),
      m_strAttrProdId("product_id"),
      m_strElemProdName("product_name"),
      m_strElemProdBrief("product_brief"),
      m_strElemProdHwVer("hw_version"),
      m_strElemProdDoF("dof"),
      m_strElemProdSize("size")
    {
    }

    /*!
     * \brief Destructor.
     */
    virtual ~HekXmlCfg()
    {
    }

    /*!
     * \brief Load XML file into DOM and set the \h_hek description.
     *
     * \param [out] desc      \h_hek robotic maniputator description.
     * \param strSearchPath   Search path of directory paths.
     * \param strXmlFileName  XML file name.
     * \param bAllInstances   Do [not] load and set all instances of XML files
     *                        found.
     *
     * \copydoc doc_return_std
     */
    virtual int load(HekDesc           &desc,
                     const std::string &strSearchPath=HekSysCfgPath,
                     const std::string &strXmlFileName=HekEtcCfg,
                     bool              bAllInstances=false);

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // I/O Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Load XML file into DOM.
     *
     * \param strXmlFileName  XML file path name.
     *
     * \copydoc doc_return_std
     */
    virtual int loadFile(const std::string &strXmlFileName=HekEtcCfg)
    {
      int   rc;

      rc = Xml::loadFile(strXmlFileName);

      return rc < 0? -HEK_ECODE_XML: HEK_OK;
    }

    /*!
     * \brief Load XML file into DOM and set the \h_hek description.
     *
     * \param desc            \h_hek robotic maniputator description.
     * \param strXmlFileName  XML file path name.
     *
     * \copydoc doc_return_std
     */
    virtual int loadFile(HekDesc           &desc,
                         const std::string &strXmlFileName=HekEtcCfg)
    {
      int   rc;

      if( (rc = Xml::loadFile(strXmlFileName)) == OK )
      {
        rc = setHekDescFromDOM(desc);
      }

      return rc < 0? -HEK_ECODE_XML: HEK_OK;
    }

    /*!
     * \brief Save DOM to XML file.
     *
     * \param strXmlFileName  XML file path name.
     *
     * \copydoc doc_return_std
     */
    virtual int saveFile(const std::string &strXmlFileName=HekEtcCfg)
    {
      int   rc;

      rc = Xml::saveFile(strXmlFileName);

      return rc < 0? -HEK_ECODE_XML: HEK_OK;
    }

    /*!
     * \brief Set DOM from \h_hek description and save XML file.
     *
     * \param desc            \h_hek robotic maniputator description.
     * \param strXmlFileName  XML file path name.
     *
     * \copydoc doc_return_std
     */
    virtual int saveFile(const HekDesc     &desc,
                         const std::string &strXmlFileName=HekEtcCfg)
    {
      int   rc;

      if( (rc = setDOMFromHekDesc(desc)) == HEK_OK )
      {
        rc = Xml::saveFile(strXmlFileName);
      }

      return rc < 0? -HEK_ECODE_XML: HEK_OK;
    }

    /*!
     * \brief Create a template \h_hek XML configuration file.
     *
     * Any current DOM is not accessed nor altered.
     *
     * \param strXmlFileName    XML file path name.
     *
     * \copydoc doc_return_std
     */
    virtual int createTemplateFile(const std::string &strXmlFileName=HekEtcCfg);

    /*!
     * \brief Set \h_hek description for DOM.
     *
     * \param desc    \h_hek robotic manipulator description.
     *
     * \copydoc doc_return_std
     */
    virtual int setHekDescFromDOM(HekDesc &desc);

    /*!
     * \brief Set the DOM from the \h_hek description.
     *
     * \param desc    \h_hek robotic manipulator description.
     *
     * \copydoc doc_return_std
     */
    virtual int setDOMFromHekDesc(const HekDesc &desc);

  protected:
    std::string m_strMajElemArm;      ///< robotic arm major element name
    std::string m_strMajElemEE;       ///< end effector major element name
    std::string m_strAttrProdId;      ///< product id attribute name
    std::string m_strElemProdName;    ///< product name element name
    std::string m_strElemProdBrief;   ///< product brief element name
    std::string m_strElemProdHwVer;   ///< product hardware version element name
    std::string m_strElemProdDoF;     ///< egrees of freedom element name
    std::string m_strElemProdSize;    ///< product size element name

    /*!
     * \brief Set \h_hek robotic base description from the DOM.
     *
     * \param pElemMaj  Pointer to major DOM base description element.
     * \param pDesc     Point to \h_hek robotic base description.
     *
     * \copydoc doc_return_std
     */
    virtual int setHekArmDescFromDOM(TiXmlElement *pElemMaj,
                                      HekDescArm *pDesc);

    /*!
     * \brief Set \h_hek end effector description for DOM.
     *
     * \param pElemMaj  Pointer to major DOM base description element.
     * \param pDesc     Point to \h_hek end effector description.
     *
     * \copydoc doc_return_std
     */
    virtual int setHekEEDescFromDOM(TiXmlElement *pElemMaj, HekDescEE *pDesc);

    /*!
     * \brief Convert string to \h_hek product size code.
     *
     * \param [in] str    String in hex, decimal, or octal format.
     * \param [out] val   Converted product size.
     *
     * \copydoc doc_return_std
     */
    int strToProdSizeCode(const std::string &str, int &val);

    /*!
     * \brief Convert product size to string equivalent.
     *
     * \param [in] val    Product size enumerate.
     * \param [out] str   Converted string equivalent.
     *
     * \copydoc doc_return_std
     */
    int prodSizeToStr(const int val, std::string &str);
  };

} // hekateros namespace

#endif // _HEK_XML_CFG_H
