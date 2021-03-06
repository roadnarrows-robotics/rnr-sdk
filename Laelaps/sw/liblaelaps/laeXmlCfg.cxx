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
 * \brief \h_laelaps XML configuration class implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright
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

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/appkit/Xml.h"

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
        rc = setDescFromDOM(desc);
      }

      if( rc == LAE_OK )
      {
        LOGDIAG2("Loaded Laelaps description from XML file %s.",
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
        rc = setDescFromDOM(desc);
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

  if( (rc = setDOMFromDesc(desc)) == LAE_OK )
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
  // Package options major element.
  //
  fprintf(fp, "  <!-- Laelaps package options -->\n");
  fprintf(fp, "  <%s>\n", m_strMajElemOptions.c_str());

  fprintf(fp, "    <!-- OPTION ELEMENTS HERE -->\n");

  fprintf(fp, "  </%s>\n\n", m_strMajElemOptions.c_str());

  //
  // XML tail
  //
  fprintf(fp, "%s", m_strXmlTail.c_str());

  fclose(fp);

  LOGDIAG3("Created file %s.", m_strXmlFileName.c_str());

  return LAE_OK;
}

int LaeXmlCfg::setDescFromDOM(LaeDesc &desc)
{
  TiXmlElement *pElem;
  const char   *sValue;
  const char   *sAttr;
  const char   *sText;
  int           n;
  int           rc;

  // root element
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
      setBaseDesc(pElem, desc);
    }

    // robotic package options descriptions
    else if( !strcasecmp(sValue, m_strMajElemOptions.c_str()) )
    {
      setOptionsDesc(pElem, desc);
    }

    else
    {
      warnUnknownElem(sValue);
    }
  }

  return LAE_OK;
}

int LaeXmlCfg::setBaseDesc(TiXmlElement *pElemMaj, LaeDesc &desc)
{
  // sub-elements
  string strAttrProdId("product_id");
  string strElemProdName("product_name");
  string strElemProdBrief("product_brief");
  string strElemProdFamily("product_family");
  string strElemProdModel("product_model");
  string strElemProdHwVer("hw_version");

  TiXmlElement *pElem;
  const char   *sValue;
  string        str;
  int           iVal;
  int           rc;

  if( (rc = strToInt(elemAttr(pElemMaj, strAttrProdId), iVal)) < 0 )
  {
    setErrorMsg("%s: No %s attribute of <%s> found or value not an integer.",
      m_strXmlFileName.c_str(),
      strAttrProdId.c_str(),
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
    else if( !strcasecmp(sValue, strElemProdName.c_str()) )
    {
      desc.m_strProdName = elemText(pElem);
    }

    // product brief
    else if( !strcasecmp(sValue, strElemProdBrief.c_str()) )
    {
      desc.m_strProdBrief = elemText(pElem);
    }

    // product family
    else if( !strcasecmp(sValue, strElemProdFamily.c_str()) )
    {
      desc.m_strProdFamily = elemText(pElem);
    }

    // product model
    else if( !strcasecmp(sValue, strElemProdModel.c_str()) )
    {
      desc.m_strProdModel = elemText(pElem);
    }

    // product hardware version
    else if( !strcasecmp(sValue, strElemProdHwVer.c_str()) )
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

int LaeXmlCfg::setOptionsDesc(TiXmlElement *pElemMaj, LaeDesc &desc)
{
  // sub-elements
  string strElemOptToF("tof_option");
  string strElemOptFCam("fcam_option");

  TiXmlElement *pElem;
  const char   *sValue;
  string        strText;

  // child elements
	for(pElem = pElemMaj->FirstChildElement();
      pElem != NULL;
      pElem = pElem->NextSiblingElement())
  {
    if( (sValue = pElem->Value()) == NULL )
    {
      continue;
    }

    // time-of-flight range sensor package
    else if( !strcasecmp(sValue, strElemOptToF.c_str()) )
    {
      strText = elemText(pElem);
      if( !strcasecmp(strText.c_str(), LaeDescOptions::PkgOptStd) )
      {
        desc.m_options.m_strPkgToF = LaeDescOptions::PkgOptStd;
      }
      else if( !strcasecmp(strText.c_str(), LaeDescOptions::PkgOptDeluxe) )
      {
        desc.m_options.m_strPkgToF = LaeDescOptions::PkgOptDeluxe;
      }
      else
      {
        LOGWARN("%s: unknown %s package option - assuming %s.",
            strText.c_str(), strElemOptToF.c_str(), LaeDescOptions::PkgOptStd);
        desc.m_options.m_strPkgToF = LaeDescOptions::PkgOptStd;
      }
    }

    // front camera sensor package
    else if( !strcasecmp(sValue, strElemOptFCam.c_str()) )
    {
      strText = elemText(pElem);
      if( !strcasecmp(strText.c_str(), LaeDescOptions::PkgOptStd) )
      {
        desc.m_options.m_strPkgFCam = LaeDescOptions::PkgOptStd;
      }
      else
      {
        LOGWARN("%s: unknown %s package option - assuming %s.",
            strText.c_str(), strElemOptFCam.c_str(), LaeDescOptions::PkgOptStd);
        desc.m_options.m_strPkgFCam = LaeDescOptions::PkgOptStd;
      }
    }

    else
    {
      warnUnknownElem(sValue);
    }
  }

  LOGDIAG3("%s: Laelaps package options set.", m_strXmlFileName.c_str());

  return LAE_OK;
}

int LaeXmlCfg::setDOMFromDesc(const LaeDesc &desc)
{
  // TODO
  return -LAE_ECODE_GEN;
}
