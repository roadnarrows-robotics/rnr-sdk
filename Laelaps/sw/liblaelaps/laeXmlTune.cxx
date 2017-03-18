////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Library:   liblaelaps
//
// File:      laeXmlTune.cxx
//
/*! \file
 *
 * $LastChangedDate: 2016-02-12 12:34:58 -0700 (Fri, 12 Feb 2016) $
 * $Rev: 4316 $
 *
 * \brief \h_laelaps XML tuning class implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2015-2017. RoadNarrows LLC.\n
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

#include "Laelaps/laelaps.h"
#include "Laelaps/laeUtils.h"
#include "Laelaps/laeMotor.h"
#include "Laelaps/laeTune.h"
#include "Laelaps/laeDesc.h"
#include "Laelaps/laeXmlTune.h"

using namespace std;
using namespace rnr;
using namespace laelaps;

//------------------------------------------------------------------------------
// LaeXmlTune Class
//------------------------------------------------------------------------------

int LaeXmlTune::load(LaeTunes     &tunes,
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
  rc              = LAE_OK;

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
        rc = -LAE_ECODE_XML;
      }
      else
      {
        rc = setTunesFromDOM(tunes);
      }

      if( rc == LAE_OK )
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

int LaeXmlTune::loadFile(const std::string &strXmlFileName)
{
  int   rc;

  // test existence of optional tuning file
  if( access(strXmlFileName.c_str(), F_OK) < 0 )
  { 
    LOGDIAG2("Optional XML file %s does not exist - ignoring.",
            strXmlFileName.c_str());
    rc = -LAE_ECODE_XML;
  }

  // parse 
  else if( (rc = Xml::loadFile(strXmlFileName)) < 0 )
  {
    LOGERROR("Parse of tune parameters from XML file %s failed - ignoring.",
            strXmlFileName.c_str());
    rc = -LAE_ECODE_XML;
  }

  // success
  else
  {
    rc = LAE_OK;
  }

  return rc;
}

int LaeXmlTune::loadFile(LaeTunes &tunes, const std::string &strXmlFileName)
{
  int   rc;

  if( (rc = loadFile(strXmlFileName)) == LAE_OK )
  {
  rc = setTunesFromDOM(tunes);
  }

  return rc < 0? -LAE_ECODE_XML: LAE_OK;
}

int LaeXmlTune::createTemplateFile(const string &strXmlFileName)
{
  FILE       *fp;     // opened file pointer

  if( strXmlFileName.empty() )
  {
    setErrorMsg("No file name.");
    LOGERROR("%s", m_bufErrMsg);
    return -LAE_ECODE_XML;
  }

  m_strXmlFileName = strXmlFileName;

  // open file
  if( (fp = fopen(m_strXmlFileName.c_str(), "w+")) == NULL )
  {
    setErrorMsg("%s: %s(errno=%d).",
        m_strXmlFileName.c_str(), strerror(errno), errno);
    LOGERROR("%s", m_bufErrMsg);
    return -LAE_ECODE_XML;
  }

  makeXmlHead();
  makeXmlTail();

  //
  // XML head.
  //
  fprintf(fp, "  <!-- RoadNarrows Laelaps Tuning Configuration -->\n");
  fprintf(fp, "%s", m_strXmlHead.c_str());

  //
  // Robotic base major element.
  //
  fprintf(fp, "  <!-- Laelaps tuning -->\n");
  fprintf(fp, "  <%s>\n", m_strMajElemTuning.c_str());

  fprintf(fp, "    <!-- TUNING ELEMENTS HERE -->\n");

  fprintf(fp, "  </%s>\n\n", m_strMajElemTuning.c_str());

  //
  // XML tail
  //
  fprintf(fp, "%s", m_strXmlTail.c_str());

  fclose(fp);

  LOGDIAG3("Created file %s.", m_strXmlFileName.c_str());

  return LAE_OK;
}

int LaeXmlTune::setTunesFromDOM(LaeTunes &tunes)
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
    return -LAE_ECODE_XML;
  }

  // search for major element
	for(pElem1 = m_pElemRoot->FirstChildElement(), rc = LAE_OK;
      (pElem1 != NULL) && (rc == LAE_OK);
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
    // <tuning> ... </tuning>
    //
    else if( !strcasecmp(sValue, m_strMajElemTuning.c_str()) )
    {
      // search for section elements
	    for(pElem2 = pElem1->FirstChildElement();
          (pElem2 != NULL) && (rc == LAE_OK);
          pElem2 = pElem2->NextSiblingElement())
      {
        // no child element name
        if( (sValue = pElem2->Value()) == NULL )
        {
          continue;
        }

        // <global> ... </global> section
        else if( !strcasecmp(sValue, m_strSecElemGlobal.c_str()) )
        {
          rc = setGlobalTunes(pElem2, tunes);
        }

        // <battery> ... </battery> section
        else if( !strcasecmp(sValue, m_strSecElemBattery.c_str()) )
        {
          rc = setBatteryTunes(pElem2, tunes);
        }

        // <powertrains location=LOC> ... <powertrains> section
        else if( !strcasecmp(sValue, m_strSecElemPowertrains.c_str()) )
        {
          rc = setPowertrainTunes(pElem2, tunes);
        }

        // <range_sensor type=TYPE location=LOC> ... <range_sensor> section
        else if( !strcasecmp(sValue, m_strSecElemRangeSensor.c_str()) )
        {
          rc = setRangeSensorTunes(pElem2, tunes);
        }

        // unknown
        else
        {
          warnUnknownElem(sValue);
        }
      }
    }
  }

  return LAE_OK;
}

int LaeXmlTune::setDOMFromTunes(const LaeTunes &tunes)
{
  // TODO
  return -LAE_ECODE_GEN;
}


// .............................................................................
// Global XML turning section
// .............................................................................

int LaeXmlTune::setGlobalTunes(TiXmlElement *pElemSec, LaeTunes &tunes)
{
  // sub-elements
  string strSubSecElemThreads("threads");
  string strSubSecElemTraj("trajectory");
  string strElemVelDerate("velocity_derate");
  string strElemWdTimeout("watchdog_timeout");

  TiXmlElement *pElem;    // working xml element
  const char   *sValue;   // working xml element name
  int           rc;       // return code

  //
  // Walk through child elements and convert.
  //
	for(pElem = pElemSec->FirstChildElement(), rc = LAE_OK;
      (pElem != NULL) && (rc == LAE_OK);
      pElem = pElem->NextSiblingElement())
  {
    // no element name
    if( (sValue = pElem->Value()) == NULL )
    {
      continue;
    }

    // <threads> ... </threads> subsection
    else if( !strcasecmp(sValue, strSubSecElemThreads.c_str()) )
    {
      rc = setGlobalThreadTunes(pElem, tunes);
    }

    // <velocity_derate> ... </velocity_derate>
    else if( !strcasecmp(sValue, strElemVelDerate.c_str()) )
    {
      rc = strToDoubleWithinRange(strElemVelDerate, elemText(pElem),
                    LaeTuneVelDerateMin, LaeTuneVelDerateMax,
                    tunes.m_fVelDerate);

      // xml units are percentages - normalize
      if( rc == LAE_OK )
      {
        tunes.m_fVelDerate /= 100.0;
      }
    }

    // <trajectory> ... </trajectory subsection
    else if( !strcasecmp(sValue, strSubSecElemTraj.c_str()) )
    {
      rc = setGlobalTrajTunes(pElem, tunes);
    }

    // <watchdog_timeout> ... </watchdog_timeout>
    else if( !strcasecmp(sValue, strElemWdTimeout.c_str()) )
    {
      rc = strToDoubleWithinRange(strElemWdTimeout, elemText(pElem),
                    LaeTuneWdTimeoutMin, LaeTuneWdTimeoutMax,
                    tunes.m_fWatchDogTimeout);

      // xml units are percentages - normalize
      if( rc == LAE_OK )
      {
        tunes.m_fVelDerate /= 100.0;
      }
    }
    // unknown
    else
    {
      warnUnknownElem(sValue);
    }
  }

  if( rc == LAE_OK )
  {
    LOGDIAG3("%s: Laelaps global tune parameters set.",
      m_strXmlFileName.c_str());
  }

  return rc;
}

int LaeXmlTune::setGlobalThreadTunes(TiXmlElement *pElemSubSec, LaeTunes &tunes)
{
  // sub-elements
  string strElemImuHz("imu_hz");
  string strElemKinHz("kinematics_hz");
  string strElemRangeHz("range_hz");

  TiXmlElement *pElem;    // working xml element
  const char   *sValue;   // working xml element name
  int           rc;       // return code

  //
  // Walk through child elements and convert.
  //
	for(pElem = pElemSubSec->FirstChildElement(), rc = LAE_OK;
      (pElem != NULL) && (rc == LAE_OK);
      pElem = pElem->NextSiblingElement())
  {
    // no element name
    if( (sValue = pElem->Value()) == NULL )
    {
      continue;
    }

    // <imu_hz> ... </imu_hz>
    else if( !strcasecmp(sValue, strElemImuHz.c_str()) )
    {
      rc = strToDoubleWithMinimum(strElemImuHz, elemText(pElem),
                                  LaeTuneThreadHzMin, tunes.m_fImuHz);
    }

    // <kinematics_hz> ... </kinematics_hz>
    else if( !strcasecmp(sValue, strElemKinHz.c_str()) )
    {
      rc = strToDoubleWithMinimum(strElemKinHz, elemText(pElem),
                                  LaeTuneThreadHzMin, tunes.m_fKinematicsHz);
    }

    // <range_hz> ... </range_hz>
    else if( !strcasecmp(sValue, strElemRangeHz.c_str()) )
    {
      rc = strToDoubleWithMinimum(strElemRangeHz, elemText(pElem),
                                  LaeTuneThreadHzMin, tunes.m_fRangeHz);
    }

    // unknown
    else
    {
      warnUnknownElem(sValue);
    }
  }

  if( rc == LAE_OK )
  {
    LOGDIAG3("%s: Laelaps global thread tune parameters set.",
      m_strXmlFileName.c_str());
  }

  return rc;
}

int LaeXmlTune::setGlobalTrajTunes(TiXmlElement *pElemSubSec, LaeTunes &tunes)
{
  // sub-elements
  string strElemTrajNorm("norm");
  string strElemTrajEpsilon("epsilon");

  TiXmlElement *pElem;    // working xml element
  const char   *sValue;   // working xml element name
  int           rc;       // return code

  //
  // Walk through child elements and convert.
  //
	for(pElem = pElemSubSec->FirstChildElement(), rc = LAE_OK;
      (pElem != NULL) && (rc == LAE_OK);
      pElem = pElem->NextSiblingElement())
  {
    // no element name
    if( (sValue = pElem->Value()) == NULL )
    {
      continue;
    }

    // <norm> ... </norm>
    else if( !strcasecmp(sValue, strElemTrajNorm.c_str()) )
    {
      strToNorm(strElemTrajNorm, elemText(pElem), tunes.m_eTrajNorm);
    }

    // <epsilon> ... </epsilon>
    else if( !strcasecmp(sValue, strElemTrajEpsilon.c_str()) )
    {
      rc = strToDoubleWithMinimum(strElemTrajEpsilon, elemText(pElem),
                                  LaeTuneTrajEpsilonMin, tunes.m_fTrajEpsilon);

      // xml units are degrees - convert to radians
      if( rc == LAE_OK )
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

  if( rc == LAE_OK )
  {
    LOGDIAG3("%s: Laelaps global trajectory tune parameters set.",
      m_strXmlFileName.c_str());
  }

  return rc;
}


// .............................................................................
// Battery XML Tuning Section
// .............................................................................

int LaeXmlTune::setBatteryTunes(TiXmlElement *pElemSec, LaeTunes &tunes)
{
  // sub-elements
  string strElemType("type");
  string strElemChem("chemistry");
  string strElemCap("capacity");
  string strElemCells("cells");
  string strElemMax("max");
  string strElemNominal("nominal");
  string strElemMin("min");

  TiXmlElement         *pElem;
  const char           *sValue;
  string                str;
  int                   iVal;
  double                fVal;
  int                   rc;

  //
  // Walk through child elements and convert.
  //
	for(pElem = pElemSec->FirstChildElement(), rc = LAE_OK;
      (pElem != NULL) && (rc == LAE_OK);
      pElem = pElem->NextSiblingElement())
  {
    // no element name
    if( (sValue = pElem->Value()) == NULL )
    {
      continue;
    }

    // <type> ... </type>
    else if( !strcasecmp(sValue, strElemType.c_str()) )
    {
      // Fixed 
      // tunes.m_battery.strType = elemText(pElem);
    }

    // <chemistry> ... </chemistry>
    else if( !strcasecmp(sValue, strElemChem.c_str()) )
    {
      // Fixed
      // tunes.m_battery.strChem = elemText(pElem);
    }

    // <capacity> ... </capacity>
    else if( !strcasecmp(sValue, strElemCap.c_str()) )
    {
      str = elemText(pElem);
      if( !str.empty() )
      {
        if( (rc = strToDouble(str, fVal)) < 0 )
        {
          setErrorMsg("%s: Element <%s> text \"%s\" not an number.",
            m_strXmlFileName.c_str(), strElemCap.c_str(),
            str.c_str());
          LOGERROR("%s", m_bufErrMsg);
          rc = -LAE_ECODE_XML;
        }
        else if( fVal != LaeTuneBattCapAh )
        {
          setErrorMsg("%s: Element <%s> value is fixed at %.1lfAh.",
            m_strXmlFileName.c_str(), strElemCap.c_str(), LaeTuneBattCapAh);
          LOGWARN("%s", m_bufErrMsg);
        }
      }
    }

    // <cells> ... </cells>
    else if( !strcasecmp(sValue, strElemCells.c_str()) )
    {
      str = elemText(pElem);
      if( !str.empty() )
      {
        if( (rc = strToInt(str, iVal)) < 0 )
        {
          setErrorMsg("%s: Element <%s> text \"%s\" not an number.",
            m_strXmlFileName.c_str(), strElemCells.c_str(),
            str.c_str());
          LOGERROR("%s", m_bufErrMsg);
          rc = -LAE_ECODE_XML;
        }
        else if( iVal != LaeTuneBattCells )
        {
          setErrorMsg("%s: Element <%s> value is fixed at %d cells.",
            m_strXmlFileName.c_str(), strElemCells.c_str(),
            LaeTuneBattCells);
          LOGWARN("%s", m_bufErrMsg);
        }
      }
    }

    // <nominal> ... </nominal>
    else if( !strcasecmp(sValue, strElemNominal.c_str()) )
    {
      str = elemText(pElem);
      if( !str.empty() )
      {
        if( (rc = strToDouble(str, fVal)) < 0 )
        {
          setErrorMsg("%s: Element <%s> text \"%s\" not an number.",
            m_strXmlFileName.c_str(), strElemNominal.c_str(),
            str.c_str());
          LOGERROR("%s", m_bufErrMsg);
          rc = -LAE_ECODE_XML;
        }
        else if( fVal != LaeTuneBattNominalV )
        {
          setErrorMsg("%s: Element <%s> value is fixed at %.1lfV.",
            m_strXmlFileName.c_str(), strElemNominal.c_str(),
            LaeTuneBattNominalV);
          LOGERROR("%s", m_bufErrMsg);
          return -LAE_ECODE_XML;
        }
      }
    }

    // <max> ... </max>
    else if( !strcasecmp(sValue, strElemMax.c_str()) )
    {
      rc = strToDoubleWithinRange(strElemMax, elemText(pElem),
                                  LaeTuneBattNominalV, LaeTuneBattMaxVMax,
                                  tunes.m_battery.m_fMaxV);
    }

    // <min> ... </min>
    else if( !strcasecmp(sValue, strElemMin.c_str()) )
    {
      rc = strToDoubleWithinRange(strElemMin, elemText(pElem),
                                  LaeTuneBattMinVMin, LaeTuneBattNominalV,
                                  tunes.m_battery.m_fMinV);
    }

    else
    {
      warnUnknownElem(sValue);
    }
  }

  if( rc == LAE_OK )
  {
    LOGDIAG3("%s: Laelaps battery tune parameters set.",
      m_strXmlFileName.c_str());
  }

  return rc;
}


// .............................................................................
// Powertrains XML Tuning Section
// .............................................................................

int LaeXmlTune::setPowertrainTunes(TiXmlElement *pElemSec, LaeTunes &tunes)
{
  // sub-elements
  string strAttrLoc("location");
  string strSubSecElemVelPid("velocity_pid");
  string strSubSecElemTires("tires");

  string        strLoc;   // powertrain location (and key)
  size_t        i;        // working index
  bool          bFound;   // [not] found
  TiXmlElement *pElem;    // working xml element
  const char   *sValue;   // working xml element name
  int           rc;       // return code

  strLoc = elemAttr(pElemSec, strAttrLoc);

  if( strLoc.empty() )
  {
    setErrorMsg("%s: No %s attribute of <%s> found.",
      m_strXmlFileName.c_str(),
      strAttrLoc.c_str(),
      m_strSecElemPowertrains.c_str());
    LOGERROR("%s", m_bufErrMsg);
    return -LAE_ECODE_XML;
  }

  for(i = 0, bFound = false; i < LaeNumMotorCtlrs; ++i)
  {
    if( !strcasecmp(strLoc.c_str(), LaeDesc::KeyMotorCtlr[i]) )
    {
      bFound = true;
      break;
    }
  }

  if( !bFound )
  {
    setErrorMsg("%s: Bad %s=\"%s\" attribute value of <%s> found.",
      m_strXmlFileName.c_str(),
      strAttrLoc.c_str(),
      strLoc.c_str(),
      m_strSecElemPowertrains.c_str());
    LOGERROR("%s", m_bufErrMsg);
    return -LAE_ECODE_XML;
  }

  // create entry 
  if( tunes.m_mapPtp.find(strLoc) == tunes.m_mapPtp.end() )
  {
    tunes.m_mapPtp[strLoc] = LaeTunesPowertrain();
  }

  //
  // Walk through child elements and convert.
  //
	for(pElem = pElemSec->FirstChildElement(), rc = LAE_OK;
      (pElem != NULL) && (rc == LAE_OK);
      pElem = pElem->NextSiblingElement())
  {
    // no element name
    if( (sValue = pElem->Value()) == NULL )
    {
      continue;
    }

    // <velocity_pid> ... </velocity_pid>
    else if( !strcasecmp(sValue, strSubSecElemVelPid.c_str()) )
    {
      rc = setPowertrainVelPidTunes(strLoc, pElem, tunes);
    }

    // <tires> ... </tires>
    else if( !strcasecmp(sValue, strSubSecElemTires.c_str()) )
    {
      rc = setPowertrainTireTunes(strLoc, pElem, tunes);
    }

    // unknown
    else
    {
      warnUnknownElem(sValue);
    }
  }

  if( rc == LAE_OK )
  {
    LOGDIAG3("%s: Laelaps %s powertrain tune parameters set.",
      m_strXmlFileName.c_str(), strLoc.c_str());
  }

  return rc;
}

int LaeXmlTune::setPowertrainVelPidTunes(const string &strLoc,
                                         TiXmlElement *pElemSubSec,
                                         LaeTunes     &tunes)
{
  // sub-elements
  string strElemVelPidKp("Kp");
  string strElemVelPidKi("Ki");
  string strElemVelPidKd("Kd");

  LaeTunesPowertrain  tunesPowertrain;  // powertrain tuning
  TiXmlElement       *pElem;            // working xml element
  const char         *sValue;           // working xml element name
  int                 rc;               // return code

  // tuning parameters
  if( tunes.m_mapPtp.find(strLoc) != tunes.m_mapPtp.end() )
  {
    tunesPowertrain = tunes.m_mapPtp[strLoc];
  }
  else
  {
    LOGERROR("Bug: Cannot find \%s\" powertrain tuning parameters.",
        strLoc.c_str());
    return -LAE_ECODE_INTERNAL;
  }
 
  //
  // Walk through child elements and convert.
  //
	for(pElem = pElemSubSec->FirstChildElement(), rc = LAE_OK;
      (pElem != NULL) && (rc == LAE_OK);
      pElem = pElem->NextSiblingElement())
  {
    // no element name
    if( (sValue = pElem->Value()) == NULL )
    {
      continue;
    }

    // <Kp> .. </Kp>
    else if( !strcasecmp(sValue, strElemVelPidKp.c_str()) )
    {
      rc = strToDoubleWithMinimum(strElemVelPidKp, elemText(pElem),
                                  LaeTuneVelPidKMin,
                                  tunesPowertrain.m_fVelPidKp);
    }

    // <Ki> ... </Ki>
    else if( !strcasecmp(sValue, strElemVelPidKi.c_str()) )
    {
      rc = strToDoubleWithMinimum(strElemVelPidKi, elemText(pElem),
                                  LaeTuneVelPidKMin,
                                  tunesPowertrain.m_fVelPidKi);
    }

    // <Kd> ... </Kd>
    else if( !strcasecmp(sValue, strElemVelPidKd.c_str()) )
    {
      rc = strToDoubleWithMinimum(strElemVelPidKd, elemText(pElem),
                                  LaeTuneVelPidKMin,
                                  tunesPowertrain.m_fVelPidKd);
    }

    // unknown
    else
    {
      warnUnknownElem(sValue);
    }
  }

  if( rc == LAE_OK )
  {
    // add back powertrain with new tuning parameters
    tunes.m_mapPtp[strLoc] = tunesPowertrain;

    LOGDIAG3("%s: Laelaps %s powertrain velocity PID tune parameters set.",
      m_strXmlFileName.c_str(), strLoc.c_str());
  }

  return rc;
}

int LaeXmlTune::setPowertrainTireTunes(const string &strLoc,
                                       TiXmlElement *pElemSubSec,
                                       LaeTunes     &tunes)
{
  // sub-elements
  string strElemTireRadius("radius");
  string strElemTireWidth("width");

  LaeTunesPowertrain  tunesPowertrain;  // powertrain tuning
  TiXmlElement       *pElem;            // working xml element
  const char         *sValue;           // working xml element name
  int                 rc;               // return code

  // tuning parameters
  if( tunes.m_mapPtp.find(strLoc) != tunes.m_mapPtp.end() )
  {
    tunesPowertrain = tunes.m_mapPtp[strLoc];
  }
  else
  {
    LOGERROR("Bug: Cannot find \%s\" powertrain tuning parameters.",
        strLoc.c_str());
    return -LAE_ECODE_INTERNAL;
  }
 
  //
  // Walk through child elements and convert.
  //
	for(pElem = pElemSubSec->FirstChildElement(), rc = LAE_OK;
      (pElem != NULL) && (rc == LAE_OK);
      pElem = pElem->NextSiblingElement())
  {
    // no element name
    if( (sValue = pElem->Value()) == NULL )
    {
      continue;
    }

    // <radius> .. </radius>
    else if( !strcasecmp(sValue, strElemTireRadius.c_str()) )
    {
      rc = strToDoubleWithMinimum(strElemTireRadius, elemText(pElem),
                                  LaeTuneTireDimMin,
                                  tunesPowertrain.m_fTireRadius);
    }

    // <width> ... </width>
    else if( !strcasecmp(sValue, strElemTireWidth.c_str()) )
    {
      rc = strToDoubleWithMinimum(strElemTireWidth, elemText(pElem),
                                  LaeTuneTireDimMin,
                                  tunesPowertrain.m_fTireWidth);
    }

    // unknown
    else
    {
      warnUnknownElem(sValue);
    }
  }

  if( rc == LAE_OK )
  {
    // add back powertrain with new tuning parameters
    tunes.m_mapPtp[strLoc] = tunesPowertrain;

    LOGDIAG3("%s: Laelaps %s powertrain tire dimension tune parameters set.",
      m_strXmlFileName.c_str(), strLoc.c_str());
  }

  return rc;
}


// .............................................................................
// Range Sensors XML Tuning Section
// .............................................................................

int LaeXmlTune::setRangeSensorTunes(TiXmlElement *pElemSec, LaeTunes &tunes)
{
  // sub-elements
  string strAttrType("type");
  string strAttrLoc("location");

  string strType;  // range sensor type (and key)
  string strLoc;   // range sensor location (and key)

  strType = elemAttr(pElemSec, strAttrType);
  strLoc  = elemAttr(pElemSec, strAttrLoc);

  if( strType.empty() )
  {
    setErrorMsg("%s: No %s attribute of <%s> found.",
      m_strXmlFileName.c_str(),
      strAttrLoc.c_str(),
      m_strSecElemRangeSensor.c_str());
    LOGERROR("%s", m_bufErrMsg);
    return -LAE_ECODE_XML;
  }

  else if( strcasecmp(strType.c_str(), "vl6180") )
  {
    setErrorMsg("%s: Bad %s=\"%s\" attribute value of <%s> found.",
      m_strXmlFileName.c_str(),
      strAttrType.c_str(),
      strType.c_str(),
      m_strSecElemRangeSensor.c_str());
    LOGERROR("%s", m_bufErrMsg);
    return -LAE_ECODE_XML;
  }

  else
  {
    return setVL6180Tunes(pElemSec, strType, strLoc, tunes);
  }
}

int LaeXmlTune::setVL6180Tunes(TiXmlElement *pElemSec,
                               string       &strType,
                               string       &strLoc,
                               LaeTunes     &tunes)
{
  // sub-elements
  string strAttrLoc("location");
  string strElemTofOffset("tof_offset");
  string strElemTofCrossTalk("tof_crosstalk");
  string strElemAlsGain("als_gain");
  string strElemAlsIntPeriod("als_int_period");
  string strValFactory("factory");

  LaeTunesVL6180  tunesSensor;  // VL6180 sensor tuning defaults
  size_t          i;            // working index
  bool            bFound;       // [not] found
  TiXmlElement   *pElem;        // working xml element
  const char     *sValue;       // working xml element name
  string          strText;        // working element text
  int             rc;           // return code

  if( strLoc.empty() )
  {
    setErrorMsg("%s: No %s attribute of <%s> found.",
      m_strXmlFileName.c_str(),
      strAttrLoc.c_str(),
      m_strSecElemRangeSensor.c_str());
    LOGERROR("%s", m_bufErrMsg);
    return -LAE_ECODE_XML;
  }

  for(i = 0, bFound = false; i < ToFSensorMaxNumOf; ++i)
  {
    if( !strcasecmp(strLoc.c_str(), LaeDesc::KeyRangeSensorMax[i]) )
    {
      bFound = true;
      break;
    }
  }

  if( !bFound )
  {
    setErrorMsg("%s: Bad %s=\"%s\" attribute value of <%s> found.",
      m_strXmlFileName.c_str(),
      strAttrLoc.c_str(),
      strLoc.c_str(),
      m_strSecElemRangeSensor.c_str());
    LOGERROR("%s", m_bufErrMsg);
    return -LAE_ECODE_XML;
  }

  // tuning parameters
  if( tunes.m_mapVL6180.find(strLoc) == tunes.m_mapVL6180.end() )
  {
    tunes.m_mapVL6180[strLoc] = tunesSensor;  // new sensor tunes
  }
  else
  {
    tunesSensor = tunes.m_mapVL6180[strLoc];  // existing sensor tunes
  }

  //
  // Walk through child elements and convert.
  //
	for(pElem = pElemSec->FirstChildElement(), rc = LAE_OK;
      (pElem != NULL) && (rc == LAE_OK);
      pElem = pElem->NextSiblingElement())
  {
    // no element name
    if( (sValue = pElem->Value()) == NULL )
    {
      continue;
    }

    // <tof_offset> ... </tof_offset>
    else if( !strcasecmp(sValue, strElemTofOffset.c_str()) )
    {
      strText = elemText(pElem);
      if( !strcasecmp(strText.c_str(), strValFactory.c_str()) )
      {
        tunesSensor.m_nTofOffset = LaeTuneVL6180TofOffsetDft;
      }
      else
      {
        rc = strToIntWithinRange(strElemTofOffset, strText,
                    LaeTuneVL6180TofOffsetMin, LaeTuneVL6180TofOffsetMax,
                    tunesSensor.m_nTofOffset);
      }
    }

    // <tof_crosstalk> ... </tof_crosstalk>
    else if( !strcasecmp(sValue, strElemTofCrossTalk.c_str()) )
    {
      strText = elemText(pElem);
      if( !strcasecmp(strText.c_str(), strValFactory.c_str()) )
      {
        tunesSensor.m_nTofCrossTalk = LaeTuneVL6180TofXTalkDft;
      }
      else
      {
        rc = strToIntWithinRange(strElemTofCrossTalk, strText,
                    LaeTuneVL6180TofXTalkMin, LaeTuneVL6180TofXTalkMax,
                    tunesSensor.m_nTofCrossTalk);
      }
    }

    // <als_gain> ... </als_gain>
    else if( !strcasecmp(sValue, strElemAlsGain.c_str()) )
    {
      strText = elemText(pElem);
      if( !strcasecmp(strText.c_str(), strValFactory.c_str()) )
      {
        tunesSensor.m_fAlsGain = LaeTuneVL6180AlsGainDft;
      }
      else
      {
        rc = strToDoubleWithinRange(strElemAlsGain, strText,
                    LaeTuneVL6180AlsGainMin, LaeTuneVL6180AlsGainMax,
                    tunesSensor.m_fAlsGain);
      }
    }

    // <als_int_period> ... </als_int_period>
    else if( !strcasecmp(sValue, strElemAlsIntPeriod.c_str()) )
    {
      strText = elemText(pElem);
      if( !strcasecmp(strText.c_str(), strValFactory.c_str()) )
      {
        tunesSensor.m_nAlsIntPeriod = LaeTuneVL6180AlsIntPeriodDft;
      }
      else
      {
        rc = strToIntWithinRange(strElemAlsIntPeriod, strText,
                    LaeTuneVL6180AlsIntPeriodMin, LaeTuneVL6180AlsIntPeriodMax,
                    tunesSensor.m_nAlsIntPeriod);
      }
    }

    // unknown
    else
    {
      warnUnknownElem(sValue);
    }
  }

  if( rc == LAE_OK )
  {
    tunes.m_mapVL6180[strLoc] = tunesSensor;  // update
    LOGDIAG3("%s: Laelaps %s VL6180 tune parameters set.",
      m_strXmlFileName.c_str(), strLoc.c_str());
  }

  return rc;
}

int LaeXmlTune::strToDoubleWithMinimum(const string &strElem,
                                       const string &strText,
                                       const double fMin,
                                       double       &fVal)
{
  int   rc = LAE_OK;    // return code

  if( !strText.empty() )
  {
    if( (rc = strToDouble(strText, fVal)) < 0 )
    {
      setErrorMsg("%s: Element <%s> text \"%s\" not a FPN.",
            m_strXmlFileName.c_str(), strElem.c_str(), strText.c_str());
      LOGERROR("%s", m_bufErrMsg);
      rc = -LAE_ECODE_XML;
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

int LaeXmlTune::strToDoubleWithinRange(const string &strElem,
                                       const string &strText,
                                       const double fMin,
                                       const double fMax,
                                       double       &fVal)
{
  int   rc = LAE_OK;    // return code

  if( !strText.empty() )
  {
    if( (rc = strToDouble(strText, fVal)) < 0 )
    {
      setErrorMsg("%s: Element <%s> text \"%s\" not a FPN.",
            m_strXmlFileName.c_str(), strElem.c_str(), strText.c_str());
      LOGERROR("%s", m_bufErrMsg);
      rc = -LAE_ECODE_XML;
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

int LaeXmlTune::strToIntWithinRange(const string &strElem,
                                    const string &strText,
                                    const int    nMin,
                                    const int    nMax,
                                    int          &nVal)
{
  int   rc = LAE_OK;    // return code

  if( !strText.empty() )
  {
    if( (rc = strToInt(strText, nVal)) < 0 )
    {
      setErrorMsg("%s: Element <%s> text \"%s\" Not a Number.",
            m_strXmlFileName.c_str(), strElem.c_str(), strText.c_str());
      LOGERROR("%s", m_bufErrMsg);
      rc = -LAE_ECODE_XML;
    }

    else if( nVal < nMin )
    {
      setErrorMsg("%s: Element <%s> value %d < than minimum of %d.",
            m_strXmlFileName.c_str(), strElem.c_str(), nVal, nMin);
      LOGWARN("%s", m_bufErrMsg);
      nVal = nMin;
    }

    else if( nVal > nMax )
    {
      setErrorMsg("%s: Element <%s> value %d > than maximum of %d.",
            m_strXmlFileName.c_str(), strElem.c_str(), nVal, nMax);
      LOGWARN("%s", m_bufErrMsg);
      nVal = nMax;
    }
  }

  return rc;
}

int LaeXmlTune::strToNorm(const string &strElem,
                          const string &strText,
                          LaeNorm      &eNorm)
{
  if( strText.empty() )
  {
    return LAE_OK;
  }
  else if( !strcasecmp(strText.c_str(), "L1") )
  {
    eNorm = LaeNormL1;
    return LAE_OK;
  }
  else if( !strcasecmp(strText.c_str(), "L2") )
  {
    eNorm = LaeNormL2;
    return LAE_OK;
  }
  else if( !strcasecmp(strText.c_str(), "Linf") )
  {
    eNorm = LaeNormLinf;
    return LAE_OK;
  }
  else
  {
    setErrorMsg("%s: Element <%s> text \"%s\" not a recognized norm.",
            m_strXmlFileName.c_str(), strElem.c_str(), strText.c_str());
    LOGERROR("%s", m_bufErrMsg);
    return -LAE_ECODE_XML;
  }
}
