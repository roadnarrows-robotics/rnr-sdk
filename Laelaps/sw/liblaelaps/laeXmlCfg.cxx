////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Library:   liblaelaps
//
// File:      laeXmlCfg.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-10-27 14:39:57 -0600 (Tue, 27 Oct 2015) $
 * $Rev: 4167 $
 *
 * \brief \h_laelaps XML configuration class implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2014.  RoadNarrows LLC
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

#include <unistd.h>

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/Xml.h"
#include "rnr/log.h"

#include "Laelaps/laelaps.h"
#include "Laelaps/laeUtils.h"
#include "Laelaps/laeMotor.h"
#include "Laelaps/laeDesc.h"
#include "Laelaps/laeXmlCfg.h"


using namespace std;
using namespace rnr;
using namespace laelaps;

int LaeXmlCfg::load(LaeDesc      &desc,
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

  desc.clear();

  for(i=0; i<vecPath.size(); ++i)
  {
    fqname = vecPath[i] + '/' + strXmlFileName;
    if( access(fqname.c_str(), F_OK) == 0 )
    {
      LOGDIAG3("Loading Laelaps description XML file: %s.", fqname.c_str());

      bFoundInstance = true;

      if( (rc = Xml::loadFile(fqname)) < 0 )
      {
        LOGERROR("Parse of Laelaps description from XML file %s failed.",
            fqname.c_str());
        rc = -LAE_ECODE_XML;
      }
      else
      {
        rc = setLaelapsDescFromDOM(desc);
      }

      if( rc == LAE_OK )
      {
        LOGDIAG2("Laelaps description from XML file %s loaded.",
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
    LOGERROR("XML file %s not found.", strXmlFileName.c_str());
    rc = -LAE_ECODE_XML;
  }

  return rc;
}

int LaeXmlCfg::loadFile(const string &strXmlFileName)
{
  int   rc;

  rc = Xml::loadFile(strXmlFileName);

  return rc < 0? -LAE_ECODE_XML: LAE_OK;
}

int LaeXmlCfg::loadFile(LaeDesc &desc, const string &strXmlFileName)
{
  int   rc;

  if( (rc = Xml::loadFile(strXmlFileName)) == OK )
  {
        rc = setLaelapsDescFromDOM(desc);
  }

  return rc < 0? -LAE_ECODE_XML: LAE_OK;
}

int LaeXmlCfg::saveFile(const string &strXmlFileName)
{
  int   rc;

  rc = Xml::saveFile(strXmlFileName);

  return rc < 0? -LAE_ECODE_XML: LAE_OK;
}

int LaeXmlCfg::saveFile(const LaeDesc &desc, const string &strXmlFileName)
{
  int   rc;

  if( (rc = setDOMFromLaelapsDesc(desc)) == LAE_OK )
  {
    rc = Xml::saveFile(strXmlFileName);
  }

  return rc < 0? -LAE_ECODE_XML: LAE_OK;
}

int LaeXmlCfg::createTemplateFile(const string &strXmlFileName)
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
  fprintf(fp, "  <!-- RoadNarrows Laelaps Top-Level Description -->\n");
  fprintf(fp, "%s", m_strXmlHead.c_str());

  //
  // Robotic base major element.
  //
  fprintf(fp, "  <!-- Laelaps robotic mobile base -->\n");
  fprintf(fp, "  <%s>\n", m_strMajElemBase.c_str());

  fprintf(fp, "    <!-- DESCRIPTION ELEMENTS HERE -->\n");

  fprintf(fp, "  </%s>\n\n", m_strMajElemBase.c_str());

  //
  // XML tail
  //
  fprintf(fp, "%s", m_strXmlTail.c_str());

  fclose(fp);

  LOGDIAG3("Created file %s.", m_strXmlFileName.c_str());

  return LAE_OK;
}

int LaeXmlCfg::setLaelapsDescFromDOM(LaeDesc &desc)
{
  TiXmlElement *pElem;
  const char   *sValue;
  const char   *sAttr;
  const char   *sText;
  int           n;
  int           rc;

  //
  // Subsection descriptions.
  //

  if( m_pElemRoot == NULL )
  {
    setErrorMsg("Missing DOM and/or <%s> root element missing.",
       m_strRootElemName.c_str());
    LOGERROR("%s", m_bufErrMsg);
    return -LAE_ECODE_XML;
  }

  // search secion elements
	for(pElem = m_pElemRoot->FirstChildElement();
      pElem != NULL;
      pElem = pElem->NextSiblingElement())
  {
    if( (sValue = pElem->Value()) == NULL )
    {
      continue;
    }

    // robotic base description
    if( !strcasecmp(sValue, m_strMajElemBase.c_str()) )
    {
      setLaelapsBaseDescFromDOM(pElem, desc);
    }
  }

  return LAE_OK;
}

int LaeXmlCfg::setLaelapsBaseDescFromDOM(TiXmlElement *pElemMaj, LaeDesc &desc)
{
  TiXmlElement *pElem;
  const char   *sValue;
  string        str;
  int           iVal;
  int           rc;

  if( (rc = strToInt(elemAttr(pElemMaj, m_strAttrProdId), iVal)) < 0 )
  {
    setErrorMsg("%s: No %s attribute of <%s> found or value not an integer.",
      m_strXmlFileName.c_str(),
      m_strAttrProdId.c_str(),
      m_strMajElemBase.c_str());
    LOGERROR("%s", m_bufErrMsg);
    return -LAE_ECODE_XML;
  }

  desc.m_eProdId = iVal;

  // child elements
	for(pElem = pElemMaj->FirstChildElement(), rc = LAE_OK;
      (pElem != NULL) && (rc == LAE_OK);
      pElem = pElem->NextSiblingElement())
  {
    if( (sValue = pElem->Value()) == NULL )
    {
      continue;
    }

    // product name
    else if( !strcasecmp(sValue, m_strElemProdName.c_str()) )
    {
      desc.m_strProdName = elemText(pElem);
    }

    // product brief
    else if( !strcasecmp(sValue, m_strElemProdBrief.c_str()) )
    {
      desc.m_strProdBrief = elemText(pElem);
    }

    // product family
    else if( !strcasecmp(sValue, m_strElemProdFamily.c_str()) )
    {
      desc.m_strProdFamily = elemText(pElem);
    }

    // product model
    else if( !strcasecmp(sValue, m_strElemProdModel.c_str()) )
    {
      desc.m_strProdModel = elemText(pElem);
    }

    // product hardware version
    else if( !strcasecmp(sValue, m_strElemProdHwVer.c_str()) )
    {
      desc.m_strProdHwVer = elemText(pElem);
      desc.m_uProdHwVer   = strToVersion(desc.m_strProdHwVer);
    }

    else
    {
      warnUnknownElem(sValue);
    }
  }

  LOGDIAG3("%s: Laelaps robotic base description set.",
      m_strXmlFileName.c_str());

  return LAE_OK;
}

int LaeXmlCfg::setDOMFromLaelapsDesc(const LaeDesc &desc)
{
  // TODO
  return -LAE_ECODE_GEN;
}
