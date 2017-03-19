////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekXmlTune.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-06-03 15:37:18 -0600 (Wed, 03 Jun 2015) $
 * $Rev: 4012 $
 *
 * \brief \h_hek XML tuning class implementation.
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

#include <unistd.h>

#include <string>
#include <sstream>
#include <vector>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/appkit/Xml.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekUtils.h"
#include "Hekateros/hekTune.h"
#include "Hekateros/hekXmlTune.h"

using namespace std;
using namespace rnr;
using namespace hekateros;

//------------------------------------------------------------------------------
// HekXmlTune Class
//------------------------------------------------------------------------------

int HekXmlTune::load(HekTunes     &tunes,
                     const string &strSearchPath,
                     const string &strXmlFileName,
                     bool         bAllInstances)
{
  vector<string>  vecPath;          // vector of search paths
  string          fqname;           // fully qualified file name
  bool            bFoundInstance;   // [not] found instance
  size_t          i;                // working index
  int             rc;               // return code

  split(strSearchPath, ':', vecPath);

  bFoundInstance  = false;
  rc              = HEK_OK;

  for(i=0; i<vecPath.size(); ++i)
  {
    fqname = vecPath[i] + '/' + strXmlFileName;
    if( access(fqname.c_str(), F_OK) == 0 )
    {
      LOGDIAG3("Loading tune XML file: %s.", fqname.c_str());

      bFoundInstance = true;

      if( (rc = Xml::loadFile(fqname)) < 0 )
      {
        LOGERROR("Parse of tune parameters from XML file %s failed.",
            fqname.c_str());
        rc = -HEK_ECODE_XML;
      }
      else
      {
        rc = setTunesFromDOM(tunes);
      }

      if( rc == HEK_OK )
      {
        LOGDIAG2("Tuning parameters from XML file %s loaded.",
            fqname.c_str());
      }

      if( !bAllInstances )
      {
        break;
      }
    }
  }

  if( !bFoundInstance )
  {
    LOGDIAG2("Optional XML file %s not found - ignoring.",
            strXmlFileName.c_str());
  }

  return rc;
}

int HekXmlTune::loadFile(const std::string &strXmlFileName)
{
  int   rc;

  // test existence of optional tuning file
  if( access(strXmlFileName.c_str(), F_OK) < 0 )
  { 
    LOGDIAG2("Optional XML file %s does not exist - ignoring.",
            strXmlFileName.c_str());
    rc = -HEK_ECODE_XML;
  }

  // parse 
  else if( (rc = Xml::loadFile(strXmlFileName)) < 0 )
  {
    LOGERROR("Parse of tune parameters from XML file %s failed - ignoring.",
            strXmlFileName.c_str());
    rc = -HEK_ECODE_XML;
  }

  // success
  else
  {
    rc = HEK_OK;
  }

  return rc;
}

int HekXmlTune::createTemplateFile(const string &strXmlFileName)
{
  FILE       *fp;     // opened file pointer

  if( strXmlFileName.empty() )
  {
    setErrorMsg("No file name.");
    LOGERROR("%s", m_bufErrMsg);
    return -HEK_ECODE_XML;
  }

  m_strXmlFileName = strXmlFileName;

  // open file
  if( (fp = fopen(m_strXmlFileName.c_str(), "w+")) == NULL )
  {
    setErrorMsg("%s: %s(errno=%d).",
        m_strXmlFileName.c_str(), strerror(errno), errno);
    LOGERROR("%s", m_bufErrMsg);
    return -HEK_ECODE_XML;
  }

  makeXmlHead();
  makeXmlTail();

  //
  // XML head.
  //
  fprintf(fp, "  <!-- RoadNarrows Hekateros Tuning Configuration -->\n");
  fprintf(fp, "%s", m_strXmlHead.c_str());

  //
  // Robotic base major element.
  //
  fprintf(fp, "  <!-- Hekateros tuning -->\n");
  fprintf(fp, "  <%s>\n", m_strMajElemTuning.c_str());

  fprintf(fp, "    <!-- global tunes -->\n");
  fprintf(fp, "    <%s>\n", m_strSecElemGlobal.c_str());
  fprintf(fp, "      <%s>HZ</%s>\n",
      m_strElemKinHz.c_str(), m_strElemKinHz.c_str());
  fprintf(fp, "      <%s>OFFSET_PCT</%s>\n",
      m_strElemClearTorqueOff.c_str(), m_strElemClearTorqueOff.c_str());
  fprintf(fp, "      <%s>DERATE_PCT</%s>\n",
      m_strElemVelDerate.c_str(), m_strElemVelDerate.c_str());
  fprintf(fp, "      <%s>\n", m_strSubSecElemTraj.c_str());
  fprintf(fp, "        <%s>NORM</%s>\n",
      m_strElemTrajNorm.c_str(), m_strElemTrajNorm.c_str());
  fprintf(fp, "        <%s>E_DEG</%s>\n",
      m_strElemTrajEpsilon.c_str(), m_strElemTrajEpsilon.c_str());
  fprintf(fp, "      </%s>\n", m_strSubSecElemTraj.c_str());
  fprintf(fp, "    </%s>\n", m_strSecElemGlobal.c_str());

  fprintf(fp, "    <!-- joint tunes -->\n");
  fprintf(fp, "    <%s %s=\"JOINT_NAME\">\n",
      m_strSecElemJoint.c_str(), m_strAttrJointId.c_str());
  fprintf(fp, "      <%s>POS_TOL_DEG</%s>\n",
      m_strElemTolPos.c_str(), m_strElemTolPos.c_str());
  fprintf(fp, "      <%s>VEL_TOL_DEG_PER_SEC</%s>\n",
      m_strElemTolVel.c_str(), m_strElemTolVel.c_str());
  fprintf(fp, "      <%s>OVER_TORQUE_TH_PCT</%s>\n",
      m_strElemOverTorqueTh.c_str(), m_strElemOverTorqueTh.c_str());
  fprintf(fp, "      <%s>\n", m_strSubSecElemPid.c_str());
  fprintf(fp, "        <%s>FPN</%s>\n",
      m_strElemPidKp.c_str(), m_strElemPidKp.c_str());
  fprintf(fp, "        <%s>FPN</%s>\n",
      m_strElemPidKi.c_str(), m_strElemPidKi.c_str());
  fprintf(fp, "        <%s>FPN</%s>\n",
      m_strElemPidKd.c_str(), m_strElemPidKd.c_str());
  fprintf(fp, "        <%s>FPN</%s>\n",
      m_strElemPidMaxDeltaV.c_str(), m_strElemPidMaxDeltaV.c_str());
  fprintf(fp, "      </%s>\n", m_strSubSecElemPid.c_str());
  fprintf(fp, "    </%s>\n", m_strSecElemJoint.c_str());

  fprintf(fp, "  </%s>\n\n", m_strMajElemTuning.c_str());

  //
  // XML tail
  //
  fprintf(fp, "%s", m_strXmlTail.c_str());

  fclose(fp);

  LOGDIAG3("Created file %s.", m_strXmlFileName.c_str());

  return HEK_OK;
}

int HekXmlTune::setTunesFromDOM(HekTunes &tunes)
{
  TiXmlElement *pElem1, *pElem2;    // working xml elements
  const char   *sValue;             // working xml element name
  int           rc;                 // return code

  // root element
  if( m_pElemRoot == NULL )
  {
    setErrorMsg("Missing DOM and/or <%s> root element missing.",
       m_strRootElemName.c_str());
    LOGERROR("%s", m_bufErrMsg);
    return -HEK_ECODE_XML;
  }

  // search for major element
	for(pElem1 = m_pElemRoot->FirstChildElement(), rc = HEK_OK;
      (pElem1 != NULL) && (rc == HEK_OK);
      pElem1 = pElem1->NextSiblingElement())
  {
    // element name
    if( (sValue = pElem1->Value()) == NULL )
    {
      continue;
    }

    //
    // Tuning major element. Walk through child elements and convert.
    //
    else if( !strcasecmp(sValue, m_strMajElemTuning.c_str()) )
    {
      // search for section elements
	    for(pElem2 = pElem1->FirstChildElement();
          (pElem2 != NULL) && (rc == HEK_OK);
          pElem2 = pElem2->NextSiblingElement())
      {
        // child element name
        if( (sValue = pElem2->Value()) == NULL )
        {
          continue;
        }

        // global tuning section
        else if( !strcasecmp(sValue, m_strSecElemGlobal.c_str()) )
        {
          rc = setGlobalTunes(pElem2, tunes);
        }

        // joint X tuning section
        else if( !strcasecmp(sValue, m_strSecElemJoint.c_str()) )
        {
          rc = setJointTunes(pElem2, tunes);
        }

        // unknown
        else
        {
          warnUnknownElem(sValue);
        }
      }
    }
  }

  return HEK_OK;
}

int HekXmlTune::setDOMFromHekTunes(const HekTunes &tunes)
{
  // TODO
  return -HEK_ECODE_GEN;
}

int HekXmlTune::setGlobalTunes(TiXmlElement *pElemSec, HekTunes &tunes)
{
  TiXmlElement *pElem;    // working xml element
  const char   *sValue;   // working xml element name
  int           rc;       // return code

  //
  // Walk through child elements and convert.
  //
	for(pElem = pElemSec->FirstChildElement(), rc = HEK_OK;
      (pElem != NULL) && (rc == HEK_OK);
      pElem = pElem->NextSiblingElement())
  {
    // element name
    if( (sValue = pElem->Value()) == NULL )
    {
      continue;
    }

    // kinematic hz
    else if( !strcasecmp(sValue, m_strElemKinHz.c_str()) )
    {
      rc = strToDoubleWithMinimum(m_strElemKinHz, elemText(pElem),
                                  HekTuneKinHzMin, tunes.m_fKinematicsHz);
    }

    // clear over torque condition threshold offset
    else if( !strcasecmp(sValue, m_strElemClearTorqueOff.c_str()) )
    {
      rc = strToDoubleWithinRange(m_strElemClearTorqueOff, elemText(pElem),
                    HekTuneClearTorqueOffsetMin, HekTuneClearTorqueOffsetMax,
                    tunes.m_fClearTorqueOffset);

      // xml units are percentages - normalize
      if( rc == HEK_OK )
      {
        tunes.m_fClearTorqueOffset /= 100.0;
      }
    }

    // velocity derate
    else if( !strcasecmp(sValue, m_strElemVelDerate.c_str()) )
    {
      rc = strToDoubleWithinRange(m_strElemVelDerate, elemText(pElem),
                    HekTuneVelDerateMin, HekTuneVelDerateMax,
                    tunes.m_fVelDerate);

      // xml units are percentages - normalize
      if( rc == HEK_OK )
      {
        tunes.m_fVelDerate /= 100.0;
      }
    }

    // trajectory tune parameters
    else if( !strcasecmp(sValue, m_strSubSecElemTraj.c_str()) )
    {
      rc = setGlobalTrajTunes(pElem, tunes);
    }

    // unknown
    else
    {
      warnUnknownElem(sValue);
    }
  }

  if( rc == HEK_OK )
  {
    LOGDIAG3("%s: Hekateros global tune parameters set.",
      m_strXmlFileName.c_str());
  }

  return rc;
}

int HekXmlTune::setJointTunes(TiXmlElement *pElemSec, HekTunes &tunes)
{
  string        strJointName;   // joint name
  HekTunesJoint tunesJoint;     // initialzed with default defaults
  TiXmlElement *pElem;          // working xml element
  const char   *sValue;         // working xml element name
  int           rc;             // return code

  strJointName = elemAttr(pElemSec, m_strAttrJointId);

  if( strJointName.empty() )
  {
    setErrorMsg("%s: No %s attribute of <%s> found.",
      m_strXmlFileName.c_str(),
      m_strAttrJointId.c_str(),
      m_strSecElemJoint.c_str());
    LOGERROR("%s", m_bufErrMsg);
    return -HEK_ECODE_XML;
  }

  // joint tuning entry already present - use as the starting tuning values
  if( tunes.m_mapJointTunes.find(strJointName) != tunes.m_mapJointTunes.end() )
  {
    tunesJoint = tunes.m_mapJointTunes[strJointName];
  }
 
  //
  // Walk through child elements and convert.
  //
	for(pElem = pElemSec->FirstChildElement(), rc = HEK_OK;
      (pElem != NULL) && (rc == HEK_OK);
      pElem = pElem->NextSiblingElement())
  {
    // element name
    if( (sValue = pElem->Value()) == NULL )
    {
      continue;
    }

    // position tolerance
    else if( !strcasecmp(sValue, m_strElemTolPos.c_str()) )
    {
      rc = strToDoubleWithMinimum(m_strElemTolPos, elemText(pElem),
                                  HekTuneTolPosMin,
                                  tunesJoint.m_fTolPos);

      // xml units are degrees - convert to radians
      if( rc == HEK_OK )
      {
        tunesJoint.m_fTolPos = degToRad(tunesJoint.m_fTolPos);
      }
    }

    // velocity tolerance
    else if( !strcasecmp(sValue, m_strElemTolVel.c_str()) )
    {
      rc = strToDoubleWithMinimum(m_strElemTolVel, elemText(pElem),
                                  HekTuneTolVelMin,
                                  tunesJoint.m_fTolVel);

      // xml units are degrees/second - convert to radians/second
      if( rc == HEK_OK )
      {
        tunesJoint.m_fTolVel = degToRad(tunesJoint.m_fTolVel);
      }
    }

    // over torque threshold
    else if( !strcasecmp(sValue, m_strElemOverTorqueTh.c_str()) )
    {
      rc = strToDoubleWithinRange(m_strElemOverTorqueTh, elemText(pElem),
                    HekTuneOverTorqueThMin, HekTuneOverTorqueThMax,
                    tunesJoint.m_fOverTorqueTh);

      // xml units are percentages - normalize
      if( rc == HEK_OK )
      {
        tunesJoint.m_fOverTorqueTh /= 100.0;
      }
    }

    // position and velocity PID tune parameters
    else if( !strcasecmp(sValue, m_strSubSecElemPid.c_str()) )
    {
      rc = setJointPidTunes(strJointName, pElem, tunesJoint);
    }

    // unknown
    else
    {
      warnUnknownElem(sValue);
    }
  }

  if( rc == HEK_OK )
  {
    // add new joint tuning or overwrite existing
    tunes.m_mapJointTunes[strJointName] = tunesJoint;

    LOGDIAG3("%s: Hekateros joint %s tune parameters set.",
      m_strXmlFileName.c_str(), strJointName.c_str());
  }

  return rc;
}

int HekXmlTune::setGlobalTrajTunes(TiXmlElement *pElemSubSec, HekTunes &tunes)
{
  TiXmlElement *pElem;    // working xml element
  const char   *sValue;   // working xml element name
  int           rc;       // return code

  //
  // Walk through child elements and convert.
  //
	for(pElem = pElemSubSec->FirstChildElement(), rc = HEK_OK;
      (pElem != NULL) && (rc == HEK_OK);
      pElem = pElem->NextSiblingElement())
  {
    // element name
    if( (sValue = pElem->Value()) == NULL )
    {
      continue;
    }

    // norm
    else if( !strcasecmp(sValue, m_strElemTrajNorm.c_str()) )
    {
      strToNorm(m_strElemTrajNorm, elemText(pElem), tunes.m_eTrajNorm);
    }

    // epsilon
    else if( !strcasecmp(sValue, m_strElemTrajEpsilon.c_str()) )
    {
      rc = strToDoubleWithMinimum(m_strElemTrajEpsilon, elemText(pElem),
                                  HekTuneTrajEpsilonMin, tunes.m_fTrajEpsilon);

      // xml units are degrees - convert to radians
      if( rc == HEK_OK )
      {
        tunes.m_fTrajEpsilon = degToRad(tunes.m_fTrajEpsilon);
      }
    }

    // unknown
    else
    {
      warnUnknownElem(sValue);
    }
  }

  if( rc == HEK_OK )
  {
    LOGDIAG3("%s: Hekateros global trajectory tune parameters set.",
      m_strXmlFileName.c_str());
  }

  return rc;
}

int HekXmlTune::setJointPidTunes(const string &strJointName,
                                 TiXmlElement *pElemSubSec,
                                 HekTunesJoint &tunesJoint)
{
  TiXmlElement *pElem;    // working xml element
  const char   *sValue;   // working xml element name
  int           rc;       // return code

  //
  // Walk through child elements and convert.
  //
	for(pElem = pElemSubSec->FirstChildElement(), rc = HEK_OK;
      (pElem != NULL) && (rc == HEK_OK);
      pElem = pElem->NextSiblingElement())
  {
    // element name
    if( (sValue = pElem->Value()) == NULL )
    {
      continue;
    }

    // Kp
    else if( !strcasecmp(sValue, m_strElemPidKp.c_str()) )
    {
      rc = strToDoubleWithMinimum(m_strElemPidKp, elemText(pElem),
                                  HekTunePidKMin, tunesJoint.m_fPidKp);
    }

    // Ki
    else if( !strcasecmp(sValue, m_strElemPidKi.c_str()) )
    {
      rc = strToDoubleWithMinimum(m_strElemPidKi, elemText(pElem),
                                  HekTunePidKMin, tunesJoint.m_fPidKi);
    }

    // Kd
    else if( !strcasecmp(sValue, m_strElemPidKd.c_str()) )
    {
      rc = strToDoubleWithMinimum(m_strElemPidKd, elemText(pElem),
                                  HekTunePidKMin, tunesJoint.m_fPidKd);
    }

    // max_delta_v
    else if( !strcasecmp(sValue, m_strElemPidMaxDeltaV.c_str()) )
    {
      rc = strToDoubleWithMinimum(m_strElemPidMaxDeltaV, elemText(pElem),
                                  HekTunePidDeltaVNoMax,
                                  tunesJoint.m_fPidMaxDeltaV);

      // xml units are degrees - convert to radians
      if( rc == HEK_OK )
      {
        tunesJoint.m_fPidMaxDeltaV = degToRad(tunesJoint.m_fPidMaxDeltaV);
      }
    }

    // unknown
    else
    {
      warnUnknownElem(sValue);
    }
  }

  if( rc == HEK_OK )
  {
    LOGDIAG3("%s: Hekateros joint %s PID tune parameters set.",
      m_strXmlFileName.c_str(), strJointName.c_str());
  }

  return rc;
}

int HekXmlTune::strToDoubleWithMinimum(const string &strElem,
                                       const string &strText,
                                       const double fMin,
                                       double       &fVal)
{
  int   rc = HEK_OK;    // return code

  if( !strText.empty() )
  {
    if( (rc = strToDouble(strText, fVal)) < 0 )
    {
      setErrorMsg("%s: Element <%s> text \"%s\" not a FPN.",
            m_strXmlFileName.c_str(), strElem.c_str(), strText.c_str());
      LOGERROR("%s", m_bufErrMsg);
      rc = -HEK_ECODE_XML;
    }

    else if( fVal < fMin )
    {
      setErrorMsg("%s: Element <%s> value %lf < than minimum of %lf.",
            m_strXmlFileName.c_str(), strElem.c_str(), fVal, fMin);
      LOGWARN("%s", m_bufErrMsg);
      fVal = fMin;
    }
  }

  return rc;
}

int HekXmlTune::strToDoubleWithinRange(const string &strElem,
                                       const string &strText,
                                       const double fMin,
                                       const double fMax,
                                       double       &fVal)
{
  int   rc = HEK_OK;    // return code

  if( !strText.empty() )
  {
    if( (rc = strToDouble(strText, fVal)) < 0 )
    {
      setErrorMsg("%s: Element <%s> text \"%s\" not a FPN.",
            m_strXmlFileName.c_str(), strElem.c_str(), strText.c_str());
      LOGERROR("%s", m_bufErrMsg);
      rc = -HEK_ECODE_XML;
    }

    else if( fVal < fMin )
    {
      setErrorMsg("%s: Element <%s> value %lf < than minimum of %lf.",
            m_strXmlFileName.c_str(), strElem.c_str(), fVal, fMin);
      LOGWARN("%s", m_bufErrMsg);
      fVal = fMin;
    }

    else if( fVal > fMax )
    {
      setErrorMsg("%s: Element <%s> value %lf > than maximum of %lf.",
            m_strXmlFileName.c_str(), strElem.c_str(), fVal, fMax);
      LOGWARN("%s", m_bufErrMsg);
      fVal = fMax;
    }
  }

  return rc;
}

int HekXmlTune::strToNorm(const string &strElem,
                          const string &strText,
                          HekNorm      &eNorm)
{
  if( strText.empty() )
  {
    return HEK_OK;
  }
  else if( !strcasecmp(strText.c_str(), "L1") )
  {
    eNorm = HekNormL1;
    return HEK_OK;
  }
  else if( !strcasecmp(strText.c_str(), "L2") )
  {
    eNorm = HekNormL2;
    return HEK_OK;
  }
  else if( !strcasecmp(strText.c_str(), "Linf") )
  {
    eNorm = HekNormLinf;
    return HEK_OK;
  }
  else
  {
    setErrorMsg("%s: Element <%s> text \"%s\" not a recognized norm.",
            m_strXmlFileName.c_str(), strElem.c_str(), strText.c_str());
    LOGERROR("%s", m_bufErrMsg);
    return -HEK_ECODE_XML;
  }
}
