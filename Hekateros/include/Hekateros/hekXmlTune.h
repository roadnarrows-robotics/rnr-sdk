////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekXmlTune.h
//
/*! \file
 *
 * $LastChangedDate: 2015-06-03 15:37:18 -0600 (Wed, 03 Jun 2015) $
 * $Rev: 4012 $
 *
 * \brief \h_hek XML tuning class interface.
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

#ifndef _HEK_XML_TUNE_H
#define _HEK_XML_TUNE_H

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/appkit/Xml.h"
#include "rnr/tinyxml/tinyxml.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekTune.h"
#include "Hekateros/hekJoint.h"

namespace hekateros
{
  //----------------------------------------------------------------------------
  // HekXmlTune Class
  //----------------------------------------------------------------------------

  /*!
   * \brief HekXmlTune \h_hek XML tuning class.
   */
  class HekXmlTune : public rnr::Xml
  {
  public:
    /*!
     * \brief Default constructor.
     */
    HekXmlTune() : 
      Xml("hekateros", HekXsiUrl, HekXslUrl),

      m_strMajElemTuning("tuning"),
      m_strSecElemGlobal("global"),
      m_strElemKinHz("kinematic_hz"),
      m_strElemClearTorqueOff("clear_torque_offset"),
      m_strElemVelDerate("velocity_derate"),
      m_strSubSecElemTraj("trajectory"),
      m_strElemTrajNorm("norm"),
      m_strElemTrajEpsilon("epsilon"),

      m_strSecElemJoint("joint"),
      m_strAttrJointId("name"),
      m_strElemTolPos("position_tolerance"),
      m_strElemTolVel("velocity_tolerance"),
      m_strElemOverTorqueTh("over_torque_threshold"),
      m_strSubSecElemPid("pid"),
      m_strElemPidKp("Kp"),
      m_strElemPidKi("Ki"),
      m_strElemPidKd("Kd"),
      m_strElemPidMaxDeltaV("max_delta_v")
    {
    }

    /*!
     * \brief Destructor.
     */
    virtual ~HekXmlTune()
    {
    }

    /*!
     * \brief Load XML file into DOM and set the \h_hek tuning parameters.
     *
     * \param [out] tunes     \h_hek tuning.
     * \param strSearchPath   Search path of directory paths.
     * \param strXmlFileName  XML file name.
     * \param bAllInstances   Do [not] load and set all instances of XML files
     *                        found.
     *
     * \copydoc doc_return_std
     */
    virtual int load(HekTunes          &tunes,
                     const std::string &strSearchPath=HekSysCfgPath,
                     const std::string &strXmlFileName=HekEtcTune,
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
    virtual int loadFile(const std::string &strXmlFileName=HekEtcTune);

    /*!
     * \brief Load XML file into DOM and set the \h_hek tuning parameters.
     *
     * \param [out] tunes     \h_hek tuning.
     * \param strXmlFileName  XML file path name.
     *
     * \copydoc doc_return_std
     */
    virtual int loadFile(HekTunes          &tunes,
                         const std::string &strXmlFileName=HekEtcTune)
    {
      int   rc;

      if( (rc = loadFile(strXmlFileName)) == HEK_OK )
      {
        rc = setTunesFromDOM(tunes);
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
    virtual int saveFile(const std::string &strXmlFileName=HekEtcTune)
    {
      int rc = Xml::saveFile(strXmlFileName);

      return rc < 0? -HEK_ECODE_XML: HEK_OK;
    }

    /*!
     * \brief Set DOM from \h_hek description and save XML file.
     *
     * \param [in] tunes      \h_hek tuning.
     * \param strXmlFileName  XML file path name.
     *
     * \copydoc doc_return_std
     */
    virtual int saveFile(const HekTunes    &tunes,
                         const std::string &strXmlFileName=HekEtcTune)
    {
      int   rc;

      if( (rc = setDOMFromHekTunes(tunes)) == HEK_OK )
      {
        rc = Xml::saveFile(strXmlFileName);
      }

      return rc < 0? -HEK_ECODE_XML: HEK_OK;
    }

    /*!
     * \brief Create a template \h_hek XML configuration file.
     * root element.
     *
     * Any current DOM is not accessed nor altered.
     *
     * \param strXmlFileName    XML file path name.
     *
     * \copydoc doc_return_std
     */
    virtual int createTemplateFile(const std::string &strXmlFileName = 
                                                                  HekEtcTune);

    /*!
     * \brief Set the tune parameters from the parsed DOM.
     *
     * \param [out] tunes   \h_hek tuning.
     *
     * \copydoc doc_return_std
     */
    virtual int setTunesFromDOM(HekTunes &tunes);

    /*!
     * \brief Set the DOM from the \h_hek tune parameters.
     *
     * \param [in] tunes    \h_hek tuning.
     *
     * \copydoc doc_return_std
     */
    virtual int setDOMFromHekTunes(const HekTunes &tunes);

  protected:
    std::string m_strMajElemTuning;   ///< major element under 'hekateros'

    std::string m_strSecElemGlobal;     ///< global tuning section element
    std::string m_strElemKinHz;         ///< kinematics thread hz
    std::string m_strElemClearTorqueOff;///< clear over torque cond. th offset
    std::string m_strElemVelDerate;     ///< derated velocity 
    std::string m_strSubSecElemTraj;    ///< trajectory subsection
    std::string m_strElemTrajNorm;      ///< distance norm
    std::string m_strElemTrajEpsilon;   ///< distance epsilon

    std::string m_strSecElemJoint;    ///< joint tuning section element.
    std::string m_strAttrJointId;     ///< joint id attribute name
    std::string m_strElemTolPos;      ///< position tolerance
    std::string m_strElemTolVel;      ///< position tolerance
    std::string m_strElemOverTorqueTh;///< over torque condition threshold
    std::string m_strSubSecElemPid;   ///< pos/vel pid subsection
    std::string m_strElemPidKp;       ///< pid proportional constant
    std::string m_strElemPidKi;       ///< pid integral constant
    std::string m_strElemPidKd;       ///< pid derivative constant
    std::string m_strElemPidMaxDeltaV;///< pid maximum delta v

    /*!
     * \brief Set the global tune parameters from the parsed DOM.
     *
     * \param pElemSec      Parent enclosing section element.
     * \param [out] tunes   \h_hek tuning.
     *
     * \copydoc doc_return_std
     */
    int setGlobalTunes(TiXmlElement *pElemSec, HekTunes &tunes);

    /*!
     * \brief Set a joint tune parameters from the parsed DOM.
     *
     * \param pElemSec      Parent enclosing section element.
     * \param [out] tunes   \h_hek tuning.
     *
     * \copydoc doc_return_std
     */
    int setJointTunes(TiXmlElement *pElemSec, HekTunes &tunes);

    /*!
     * \brief Set the global trajectory tune parameters from the parsed DOM.
     *
     * \param pElemSubSec   Parent enclosing subsection element.
     * \param [out] tunes   \h_hek tuning.
     *
     * \copydoc doc_return_std
     */
    int setGlobalTrajTunes(TiXmlElement *pElemSubSec, HekTunes &tunes);

    /*!
     * \brief Set a joint's PID tune parameters from the parsed DOM.
     *
     * \param strJointName      Joint name.
     * \param pElemSubSec       Parent enclosing subsection element.
     * \param [out] tunesJoint  \h_hek joint specific tuning.
     *
     * \copydoc doc_return_std
     */
    int setJointPidTunes(const std::string &strJointName,
                         TiXmlElement      *pElemSubSec,
                         HekTunesJoint      &tunesJoint);

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
     * \brief Convert text to value within minimum,maximum range.
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
                  HekNorm           &eNorm);

    
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

} // hekateros namespace

#endif // _HEK_XML_TUNE_H
