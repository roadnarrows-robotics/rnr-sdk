////////////////////////////////////////////////////////////////////////////////
//
// Package:   Kuon
//
// Library:   libkuon
//
// File:      kuonXmlCfg.h
//
/*! \file
 *
 * $LastChangedDate: 2014-04-04 17:05:03 -0600 (Fri, 04 Apr 2014) $
 * $Rev: 3629 $
 *
 * \brief \h_kuon XML configuration class interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
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

#ifndef _KUON_XML_CFG_H
#define _KUON_XML_CFG_H

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/appkit/Xml.h"
#include "rnr/tinyxml/tinyxml.h"

#include "Kuon/kuon.h"
#include "Kuon/kuonDescBase.h"


namespace kuon
{
  //----------------------------------------------------------------------------
  // KuonXmlCfg Class
  //----------------------------------------------------------------------------

  /*!
   * \brief KuonXmlCfg \h_kuon XML configuration class.
   */
  class KuonXmlCfg : public rnr::Xml
  {
  public:
    /*!
     * \brief Default constructor.
     */
    KuonXmlCfg() : 
      Xml("kuon", KuonXsiUrl, KuonXslUrl),
      m_strMajElemBase("base"),
      m_strAttrProdId("product_id"),
      m_strElemProdName("product_name"),
      m_strElemProdBrief("product_brief"),
      m_strElemProdHwVer("hw_version"),
      m_strElemProdFrontTire("front_tires"),
      m_strElemProdRearTire("rear_tires")
    {
    }

    /*!
     * \brief Destructor.
     */
    virtual ~KuonXmlCfg()
    {
    }


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
    virtual int loadFile(const std::string &strXmlFileName=KuonEtcCfg)
    {
      int   rc;

      rc = Xml::loadFile(strXmlFileName);

      return rc < 0? -KUON_ECODE_XML: KUON_OK;
    }

    /*!
     * \brief Load XML file into DOM and set the \h_kuon description.
     *
     * \param desc            \h_kuon robotic maniputator description.
     * \param strXmlFileName  XML file path name.
     *
     * \copydoc doc_return_std
     */
    virtual int loadFile(KuonDesc          &desc,
                         const std::string &strXmlFileName=KuonEtcCfg)
    {
      int   rc;

      if( (rc = Xml::loadFile(strXmlFileName)) == OK )
      {
        rc = setKuonDescFromDOM(desc);
      }

      return rc < 0? -KUON_ECODE_XML: KUON_OK;
    }

    /*!
     * \brief Save DOM to XML file.
     *
     * \param strXmlFileName  XML file path name.
     *
     * \copydoc doc_return_std
     */
    virtual int saveFile(const std::string &strXmlFileName=KuonEtcCfg)
    {
      int   rc;

      rc = Xml::saveFile(strXmlFileName);

      return rc < 0? -KUON_ECODE_XML: KUON_OK;
    }

    /*!
     * \brief Set DOM from \h_kuon description and save XML file.
     *
     * \param desc            \h_kuon robotic maniputator description.
     * \param strXmlFileName  XML file path name.
     *
     * \copydoc doc_return_std
     */
    virtual int saveFile(const KuonDesc    &desc,
                         const std::string &strXmlFileName=KuonEtcCfg)
    {
      int   rc;

      if( (rc = setDOMFromKuonDesc(desc)) == KUON_OK )
      {
        rc = Xml::saveFile(strXmlFileName);
      }

      return rc < 0? -KUON_ECODE_XML: KUON_OK;
    }

    /*!
     * \brief Create a template \h_kuon XML configuration file.
     * root element.
     *
     * Any current DOM is not accessed nor altered.
     *
     * \param strXmlFileName    XML file path name.
     *
     * \copydoc doc_return_std
     */
    virtual int createTemplateFile(
                                const std::string &strXmlFileName=KuonEtcCfg);

    /*!
     * \brief Set \h_kuon description for DOM.
     *
     * \param desc    \h_kuon robotic manipulator description.
     *
     * \copydoc doc_return_std
     */
    virtual int setKuonDescFromDOM(KuonDesc &desc);

    /*!
     * \brief Set the DOM from the \h_kuon description.
     *
     * \param desc    \h_kuon robotic manipulator description.
     *
     * \copydoc doc_return_std
     */
    virtual int setDOMFromKuonDesc(const KuonDesc &desc);

  protected:
    std::string m_strMajElemBase;     ///< robotic base major element name
    std::string m_strAttrProdId;      ///< product id attribute name
    std::string m_strElemProdName;    ///< product name element name
    std::string m_strElemProdBrief;   ///< product brief element name
    std::string m_strElemProdHwVer;   ///< product hardware version element name
    std::string m_strElemProdFrontTire; ///< front tire radius element name
    std::string m_strElemProdRearTire;  ///< rear tire radius element name

    /*!
     * \brief Set \h_kuon robotic base platform description for DOM.
     *
     * \param pElemMaj  Pointer to major DOM base description element.
     * \param pDesc     Point to \h_kuon robotic base description.
     *
     * \copydoc doc_return_std
     */
    virtual int setKuonBaseDescFromDOM(TiXmlElement *pElemMaj,
                                       KuonDescBase *pDesc);
  };

} // namespace kuon

#endif // _KUON_XML_CFG_H
