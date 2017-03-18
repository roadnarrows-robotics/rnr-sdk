////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_appkit
//
// File:      Xml.h
//
/*! \file
 *
 * $LastChangedDate: 2014-04-01 15:50:36 -0600 (Tue, 01 Apr 2014) $
 * $Rev: 3627 $
 *
 * \brief XML base class.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2012-2017. RoadNarrows LLC.\n
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

#ifndef _RNR_XML_H
#define _RNR_XML_H

#include <string>

#include "rnr/tinyxml/tinyxml.h"

#include "rnr/rnrconfig.h"


namespace rnr
{
  /*! XML declaration fixed string */
  const char* const XmlDecl = "<?xml version=\"1.0\" encoding=\"utf-8\"?>";

  /*! XML schema instance fixed substring */
  const char* const XmlNsXsi =
    "xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"";

  const char* const XslDecl = 
    "<xsl:stylesheet version=\"1.0\" "
    "xmlns:xsl=\"http://www.w3.org/1999/XSL/Transform\">";

  //----------------------------------------------------------------------------
  // Xml Base Class
  //----------------------------------------------------------------------------

  /*!
   * \brief XML base class.
   */
  class Xml
  {
  public:
    /*!
     * \brief Initialization constructor.
     *
     * \param strRootElem   XML document top-level root element.
     * \param strXsiUrl     XML schema instance URL. Examples:\n
     *  "http://www.roadnarrows.com/xml/<em>pkg</em>/1.0/<em>name</em>.xsd"\n
     *  "file://<em>prefix</em>/share/<em>pkg</em>/<em>name</em>.xsd"\n
     *  "<em>name</em>.xsd"\n
     *                      Default: No schema.
     * \param strXslUrl     XML extended stylesheet language transformation
     *                      URL. Examples\n
     *  "http://www.roadnarrows.com/xml/<em>pkg</em>/1.0/<em>name</em>.xsl"\n
     *  "file://<em>prefix</em>/share/<em>pkg</em>/<em>name</em>.xsl"\n
     *  "<em>name</em>.xsl"\n
     *                      Default: No stylesheet.
     */
    Xml(const std::string &strRootElem,
        const std::string &strXsiUrl="",
        const std::string &strXslUrl="");

    /*!
     * \brief Destructor.
     */
    virtual ~Xml();


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // I/O Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Load XML file into DOM.
     *
     * \param strXmlFileName    XML file path name.
     *
     * \copydoc doc_return_std
     */
    virtual int loadFile(const std::string &strXmlFileName);

    /*!
     * \brief Save DOM to XML file.
     *
     * \param strXmlFileName    XML file path name.
     *
     * \copydoc doc_return_std
     */
    virtual int saveFile(const std::string &strXmlFileName);

    /*!
     * \brief Create XML file with the given major element name under the
     * root element.
     *
     * Any current DOM is not accessed nor altered.
     *
     * \param strXmlFileName    XML file path name.
     * \param strMajorElemName  XML major element name. A major element is an
     *                          XML direct child of the root element.
     *
     * \copydoc doc_return_std
     */
    virtual int createTemplateFile(const std::string &strXmlFileName,
                                   const std::string &strMajorElemName);

    /*!
     * \brief Check that the XML file exists and has read/write access.
     *
     * \param strXmlFileName    XML file path name.
     * \param bRequired         File is [not] required to exist. 
     *
     * \return Returns true if access permitted, false otherwise.
     */
    bool fileExists(const std::string &strXmlFileName, bool bRequired=false);


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Class Attribute Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Set XML document schema instance.
     *
     * \param strXsiUrl  URL of schema.
     */
    void setSchemaInstance(const std::string &strXsiUrl)
    {
      m_strXsiUrl = strXsiUrl;
    }

    /*!
     * \brief Set XML Extended Style Sheet.
     *
     * \param strXslUrl  URL of stylesheet.
     */
    void setStylesheet(const std::string &strXslUrl)
    {
      m_strXslUrl = strXslUrl;
    }

    /*!
     * \brief Get last load/save/create XML file name.
     *
     * \return File path name.
     */
    std::string getFileName()
    {
      return m_strXmlFileName;
    }

    /*!
     * \brief Check if DOM has been modified since last save.
     *
     * \return Returns true or false.
     */
    bool isModified()
    {
      return m_bModified;
    }

    /*!
     * \brief Set DOM modified state.
     *
     * \param bModified Modified state: true or false.
     */
    void setModifiedState(bool bModified)
    {
      m_bModified = bModified;
    }

    /*!
     * \brief Get last error message.
     *
     * \return String.
     */
    std::string getErrorMsg()
    {
      return m_bufErrMsg;
    }


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // DOM Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Get the major element in the DOM.
     *
     * \param strMajorElemName  XML major element name. A major element is an
     *                          XML direct child of the root element.
     *
     * \return On success, returns pointer to elemen.\n
     * On failure, NULL is returned.
     */
    TiXmlElement *getMajorElem(const std::string &strMajorElemName);

    /*!
     * \brief Get element's text.
     *
     * Leading and trailing white space is stripped.
     *
     * \param pElem   Pointer to DOM element object.
     *
     * \<elem\>
     * text...
     * \</elem\>
     *
     * \return Text string.
     */
    std::string elemText(TiXmlElement *pElem);

    /*!
     * \brief Get element's attribute value.
     *
     * \param pElem   Pointer to DOM element object.
     *
     * \return Attribute value string.
     */
    std::string elemAttr(TiXmlElement *pElem, const std::string &strAttrName);


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // String Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Convert string to integer.
     *
     * \param [in] str    String in hex, decimal, or octal format.
     * \param [out] val   Converted integer value.
     *
     * \copydoc doc_return_std
     */
    int strToInt(const std::string &str, int &val);

    /*!
     * \brief Convert integer to string.
     *
     * \param [in] val    Integer value.
     * \param [out] str   Converted string in hex, decimal, or octal format.
     * \param base        One of: 16 10 8.
     *
     * \copydoc doc_return_std
     */
    int intToStr(const int val, std::string &str, int base=10);

    /*!
     * \brief Convert string to double-precision floating-point number.
     *
     * \param [in] str    String in hex, decimal, or octal format.
     * \param [out] val   Converted double value.
     *
     * \copydoc doc_return_std
     */
    int strToDouble(const std::string &str, double &val);

    /*!
     * \brief Convert double to string.
     *
     * \param [in] val    Double value.
     * \param [out] str   Converted string in hex, decimal, or octal format.
     *
     * \copydoc doc_return_std
     */
    int doubleToStr(const double val, std::string &str);

  protected:
    std::string     m_strRootElemName;  ///< xml top-level root element name
    std::string     m_strXsiUrl;        ///< xml schema instance
    std::string     m_strXslUrl;        ///< xml style sheet

    std::string     m_strXmlFileName;   ///< xml file path name

    std::string     m_strXmlHead;       ///< xml document head
    std::string     m_strXmlTail;       ///< xml document tail

    TiXmlDocument   m_xmlDoc;           ///< parsed xml DOM
    TiXmlElement   *m_pElemRoot;        ///< top-level root element
    bool            m_bModified;        ///< xml [not] modified
    char            m_bufErrMsg[256];   ///< error message buffer

    /*!
     * \brief Make XML document head string.
     *
     * The XML head string contains XML declarations plus the root element
     * opening statement.
     */
    virtual void makeXmlHead();

    /*!
     * \brief Make XML document tail string.
     *
     * The XML tail string contains the root element closing statement.
     */
    virtual void makeXmlTail();

    /*!
     * \brief Set XML error message.
     *
     * \param sFmt  Format string.
     * \param ...   Format variable arguments.
     */
    void setErrorMsg(const char *sFmt, ...);
  };

} // rnr namespace

#endif // _RNR_XML_H
