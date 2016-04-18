////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Library:   liblaelaps
//
// File:      laeXmlCfg.h
//
/*! \file
 *
 * $LastChangedDate: 2016-01-21 16:50:25 -0700 (Thu, 21 Jan 2016) $
 * $Rev: 4268 $
 *
 * \brief \h_laelaps XML configuration class interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2015-2016.  RoadNarrows LLC
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

#ifndef _LAE_XML_CFG_H
#define _LAE_XML_CFG_H

#include <string>

#include "tinyxml.h"

#include "rnr/rnrconfig.h"
#include "rnr/Xml.h"

#include "Laelaps/laelaps.h"
#include "Laelaps/laeDesc.h"


namespace laelaps
{
  //----------------------------------------------------------------------------
  // LaeXmlCfg Class
  //----------------------------------------------------------------------------

  /*!
   * \brief LaeXmlCfg \h_laelaps XML configuration class.
   */
  class LaeXmlCfg : public rnr::Xml
  {
  public:
    /*!
     * \brief Default constructor.
     */
    LaeXmlCfg() : 
      Xml("laelaps", LaeXsiUrl, LaeXslUrl),

      m_strMajElemBase("base"),

      m_strAttrProdId("product_id"),
      m_strElemProdName("product_name"),
      m_strElemProdBrief("product_brief"),
      m_strElemProdFamily("product_family"),
      m_strElemProdModel("product_model"),
      m_strElemProdHwVer("hw_version")
    {
    }

    /*!
     * \brief Destructor.
     */
    virtual ~LaeXmlCfg()
    {
    }

    /*!
     * \brief Load XML file into DOM and set the \h_laelaps description.
     *
     * \param [out] desc      \h_laelaps robot description.
     * \param strSearchPath   Search path of directory paths.
     * \param strXmlFileName  XML file name.
     * \param bAllInstances   Do [not] load and set all instances of XML files
     *                        found.
     *
     * \copydoc doc_return_std
     */
    virtual int load(LaeDesc           &desc,
                     const std::string &strSearchPath,
                     const std::string &strXmlFileName,
                     bool               bAllInstances=false);

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
    virtual int loadFile(const std::string &strXmlFileName=LaeEtcCfg);

    /*!
     * \brief Load XML file into DOM and set the \h_laelaps description.
     *
     * \param desc            \h_laelaps robot description.
     * \param strXmlFileName  XML file path name.
     *
     * \copydoc doc_return_std
     */
    virtual int loadFile(LaeDesc &desc,
                         const std::string &strXmlFileName=LaeEtcCfg);

    /*!
     * \brief Save DOM to XML file.
     *
     * \param strXmlFileName  XML file path name.
     *
     * \copydoc doc_return_std
     */
    virtual int saveFile(const std::string &strXmlFileName=LaeEtcCfg);

    /*!
     * \brief Set DOM from \h_laelaps description and save XML file.
     *
     * \param desc            \h_laelaps robot description.
     * \param strXmlFileName  XML file path name.
     *
     * \copydoc doc_return_std
     */
    virtual int saveFile(const LaeDesc &desc,
                         const std::string  &strXmlFileName=LaeEtcCfg);

    /*!
     * \brief Create a template \h_laelaps XML configuration file.
     * root element.
     *
     * Any current DOM is not accessed nor altered.
     *
     * \param strXmlFileName    XML file path name.
     *
     * \copydoc doc_return_std
     */
    virtual int createTemplateFile(const std::string &strXmlFileName=LaeEtcCfg);

    /*!
     * \brief Set \h_laelaps description for DOM.
     *
     * \param desc            \h_laelaps robot description.
     *
     * \copydoc doc_return_std
     */
    virtual int setLaelapsDescFromDOM(LaeDesc &desc);

    /*!
     * \brief Set the DOM from the \h_laelaps description.
     *
     * \param desc            \h_laelaps robot description.
     *
     * \copydoc doc_return_std
     */
    virtual int setDOMFromLaelapsDesc(const LaeDesc &desc);

  protected:
    // <base product_id=ID> ...</base>
    std::string m_strMajElemBase;     ///< robotic base major element name
    std::string m_strAttrProdId;      ///< product id attribute name

    std::string m_strElemProdName;    ///< product name element name
    std::string m_strElemProdBrief;   ///< product brief element name
    std::string m_strElemProdFamily;  ///< product family element name
    std::string m_strElemProdModel;   ///< product model element name
    std::string m_strElemProdHwVer;   ///< product hardware version element name

    /*!
     * \brief Set \h_laelaps robotic base description for DOM.
     *
     * \param pElemMaj  Pointer to major DOM base description element.
     * \param desc      \h_laelaps robot description.
     *
     * \copydoc doc_return_std
     */
    virtual int setLaelapsBaseDescFromDOM(TiXmlElement *pElemMaj,
                                          LaeDesc  &desc);

    /*!
     * \brief Warn on unknown element.
     *
     * \param strElem   Element name.
     */
    void warnUnknownElem(const std::string &strElem)
    {
      setErrorMsg("%s: Element <%s> unknown - ignoring.",
            m_strXmlFileName.c_str(), strElem.c_str());
      LOGWARN("%s", m_bufErrMsg);
    }

  };

} // namespace laelaps

#endif // _LAE_XML_CFG_H
