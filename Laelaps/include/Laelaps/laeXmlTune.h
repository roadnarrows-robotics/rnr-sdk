////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Library:   liblaelaps
//
// File:      laeXmlTune.h
//
/*! \file
 *
 * $LastChangedDate: 2016-01-21 16:50:25 -0700 (Thu, 21 Jan 2016) $
 * $Rev: 4268 $
 *
 * \brief \h_laelaps XML tuning class interface.
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

#ifndef _LAE_XML_TUNE_H
#define _LAE_XML_TUNE_H

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/appkit/Xml.h"
#include "rnr/log.h"
#include "rnr/tinyxml/tinyxml.h"

#include "Laelaps/laelaps.h"
#include "Laelaps/laeTune.h"

namespace laelaps
{
  //----------------------------------------------------------------------------
  // LaeXmlTune Class
  //----------------------------------------------------------------------------

  /*!
   * \brief LaeXmlTune \h_laelaps XML tuning class.
   */
  class LaeXmlTune : public rnr::Xml
  {
  public:
    /*!
     * \brief Default constructor.
     */
    LaeXmlTune() : 
      Xml("laelaps", LaeXsiUrl, LaeXslUrl),
      m_strMajElemTuning("tuning"),
      m_strSecElemGlobal("global"),
      m_strSecElemBattery("battery"),
      m_strSecElemPowertrains("powertrains"),
      m_strSecElemRangeSensor("range_sensor")
    {
    }

    /*!
     * \brief Destructor.
     */
    virtual ~LaeXmlTune()
    {
    }

    /*!
     * \brief Load XML file into DOM and set the \h_laelaps tuning parameters.
     *
     * \param [out] tunes     \h_laelaps tuning.
     * \param strSearchPath   Search path of directory paths.
     * \param strXmlFileName  XML file name.
     * \param bAllInstances   Do [not] load and set all instances of XML files
     *                        found.
     *
     * \copydoc doc_return_std
     */
    virtual int load(LaeTunes          &tunes,
                     const std::string &strSearchPath=LaeSysCfgPath,
                     const std::string &strXmlFileName=LaeEtcTune,
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
    virtual int loadFile(const std::string &strXmlFileName=LaeEtcTune);

    /*!
     * \brief Load XML file into DOM and set the \h_laelaps tuning parameters.
     *
     * \param [out] tunes     \h_laelaps tuning.
     * \param strXmlFileName  XML file path name.
     *
     * \copydoc doc_return_std
     */
    virtual int loadFile(LaeTunes          &tunes,
                         const std::string &strXmlFileName=LaeEtcTune);

    /*!
     * \brief Save DOM to XML file.
     *
     * \param strXmlFileName  XML file path name.
     *
     * \copydoc doc_return_std
     */
    virtual int saveFile(const std::string &strXmlFileName=LaeEtcTune)
    {
      int rc = Xml::saveFile(strXmlFileName);

      return rc < 0? -LAE_ECODE_XML: LAE_OK;
    }

    /*!
     * \brief Set DOM from \h_laelaps tuning parameters and save XML file.
     *
     * \param [in] tunes      \h_laelaps tuning.
     * \param strXmlFileName  XML file path name.
     *
     * \copydoc doc_return_std
     */
    virtual int saveFile(const LaeTunes    &tunes,
                         const std::string &strXmlFileName=LaeEtcTune)
    {
      int   rc;

      if( (rc = setDOMFromTunes(tunes)) == LAE_OK )
      {
        rc = Xml::saveFile(strXmlFileName);
      }

      return rc < 0? -LAE_ECODE_XML: LAE_OK;
    }

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
    virtual int createTemplateFile(const std::string &strXmlFileName = 
                                                                  LaeEtcTune);

    /*!
     * \brief Set the tune parameters from the parsed DOM.
     *
     * \param [out] tunes   \h_laelaps tuning.
     *
     * \copydoc doc_return_std
     */
    virtual int setTunesFromDOM(LaeTunes &tunes);

    /*!
     * \brief Set the DOM from the \h_laelaps tune parameters.
     *
     * \param [in] tunes    \h_laelaps tuning.
     *
     * \copydoc doc_return_std
     */
    virtual int setDOMFromTunes(const LaeTunes &tunes);

  protected:
    // <tuning> ... </tuning>
    std::string m_strMajElemTuning;       ///< major element under 'laelaps'
    std::string m_strSecElemGlobal;       ///< global tuning section element
    std::string m_strSecElemBattery;      ///< battery section element name
    std::string m_strSecElemPowertrains;  ///< powertrain section element name
    std::string m_strSecElemRangeSensor;  ///< range sensor section elem name

    /*!
     * \brief Set the global tune parameters from the parsed DOM.
     *
     * \param pElemSec      Parent enclosing section element.
     * \param [out] tunes   \h_laelaps tuning.
     *
     * \copydoc doc_return_std
     */
    int setGlobalTunes(TiXmlElement *pElemSec, LaeTunes &tunes);

    /*!
     * \brief Set the global threads tune parameters from the parsed DOM.
     *
     * \param pElemSubSec   Parent enclosing subsection element.
     * \param [out] tunes   \h_laelaps tuning.
     *
     * \copydoc doc_return_std
     */
    int setGlobalThreadTunes(TiXmlElement *pElemSubSec, LaeTunes &tunes);

    /*!
     * \brief Set the global trajectory tune parameters from the parsed DOM.
     *
     * \param pElemSubSec   Parent enclosing subsection element.
     * \param [out] tunes   \h_laelaps tuning.
     *
     * \copydoc doc_return_std
     */
    int setGlobalTrajTunes(TiXmlElement *pElemSubSec, LaeTunes &tunes);

    /*!
     * \brief Set \h_laelaps battery tune parameters from the parsed DOM.
     *
     * \param pElemSec      Parent enclosing section element.
     * \param [out] tunes   \h_laelaps tuning.
     *
     * \copydoc doc_return_std
     */
    virtual int setBatteryTunes(TiXmlElement *pElemSec, LaeTunes &tunes);

    /*!
     * \brief Set \h_laelaps powertrain tuning parameters from the parsed DOM.
     *
     * \param pElemSec      Parent enclosing section element.
     * \param [out] tunes   \h_laelaps tuning.
     *
     * \copydoc doc_return_std
     */
    virtual int setPowertrainTunes(TiXmlElement *pElemSec, LaeTunes &tunes);

    /*!
     * \brief Set a powertrain pair's velocity PID tune parameters from the
     * parsed DOM.
     *
     * \param strLoc        Powertrain pair location (and key). 
     * \param pElemSubSec   Parent enclosing subsection element.
     * \param [out] tunes   \h_laelaps tuning.
     *
     * \copydoc doc_return_std
     */
    int setPowertrainVelPidTunes(const std::string &strLoc,
                                 TiXmlElement      *pElemSubSec,
                                 LaeTunes          &tunes);

    /*!
     * \brief Set a powertrain pair's tire tune parameters from the parsed DOM.
     *
     * \param strLoc        Powertrain pair location (and key). 
     * \param pElemSubSec   Parent enclosing subsection element.
     * \param [out] tunes   \h_laelaps tuning.
     *
     * \copydoc doc_return_std
     */
    int setPowertrainTireTunes(const std::string &strLoc,
                               TiXmlElement      *pElemSubSec,
                               LaeTunes          &tunes);

    /*!
     * \brief Set \h_laelaps range sensor tuning parameters from DOM.
     *
     * \param pElemSec      Parent enclosing section element.
     * \param [out] tunes   \h_laelaps tuning.
     *
     * \copydoc doc_return_std
     */
    virtual int setRangeSensorTunes(TiXmlElement *pElemSec, LaeTunes &tunes);

    /*!
     * \brief Set \h_laelaps VL6180 range sensor tuning parameters from DOM.
     *
     * \param pElemSec      Parent enclosing section element.
     * \param strAttrType   Type of range sensor.
     * \param strAttrLoc    Location of range sensor (key).
     * \param [out] tunes   \h_laelaps tuning.
     *
     * \copydoc doc_return_std
     */
    int setVL6180Tunes(TiXmlElement *pElemSec,
                       std::string  &strAttrType,
                       std::string  &strAttrLoc,
                       LaeTunes     &tunes);

    /*!
     * \brief Convert text to value with a minimum value.
     *
     * If the converted value is \h_lt the minimum, it will be set to the
     * minimum.
     *
     * If the text is empty or if a conversion error occurs, the value is not
     * set.
     *
     * \param strElem     XML element name.
     * \param strText     XML element text. \<elem\>TEXT\</elem\>
     * \param fMin        Minimum value allowed.
     * \param [out] fVal  Converted value.
     *
     * \copydoc doc_return_std
     */
    int strToDoubleWithMinimum(const std::string &strElem,
                               const std::string &strText,
                               const double      fMin,
                               double            &fVal);

    /*!
     * \brief Convert text to FPN value within minimum,maximum range.
     *
     * If the converted value is out of range, it will be set to the appropriate
     * boundry value.
     *
     * If the text is empty or if a conversion error occurs, the value is not
     * set.
     *
     * \param strElem     XML element name.
     * \param strText     XML element text. \<elem\>TEXT\</elem\>
     * \param fMin        Minimum value allowed.
     * \param fMax        Maximum value allowed.
     * \param [out] fVal  Converted value.
     *
     * \copydoc doc_return_std
     */
    int strToDoubleWithinRange(const std::string &strElem,
                               const std::string &strText,
                               const double      fMin,
                               const double      fMax,
                               double            &fVal);

    /*!
     * \brief Convert text to integer value within minimum,maximum range.
     *
     * If the converted value is out of range, it will be set to the appropriate
     * boundry value.
     *
     * If the text is empty or if a conversion error occurs, the value is not
     * set.
     *
     * \param strElem     XML element name.
     * \param strText     XML element text. \<elem\>TEXT\</elem\>
     * \param nMin        Minimum value allowed.
     * \param nMax        Maximum value allowed.
     * \param [out] nVal  Converted value.
     *
     * \copydoc doc_return_std
     */
    int strToIntWithinRange(const std::string &strElem,
                            const std::string &strText,
                            const int         nMin,
                            const int         nMax,
                            int               &nVal);

    /*!
     * \brief Convert text to norm enum.
     *
     * If the text is empty or if a conversion error occurs, the value is not
     * set.
     *
     * \param strElem     XML element name.
     * \param strText     XML element text. \<elem\>TEXT\</elem\>
     * \param [out] eNorm Converted value.
     *
     * \copydoc doc_return_std
     */
    int strToNorm(const std::string &strElem,
                  const std::string &strText,
                  LaeNorm           &eNorm);

    
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

} // laelaps namespace

#endif // _LAE_XML_TUNE_H
