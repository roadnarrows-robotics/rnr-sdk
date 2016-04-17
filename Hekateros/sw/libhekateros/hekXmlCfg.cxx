////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekXmlCfg.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-04-24 13:06:48 -0600 (Fri, 24 Apr 2015) $
 * $Rev: 3959 $
 *
 * \brief HekXmlCfg - \h_hek XML configuration class implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2013-2015.  RoadNarrows LLC
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

#include "Hekateros/hekateros.h"
#include "Hekateros/hekDescArm.h"
#include "Hekateros/hekDescEE.h"
#include "Hekateros/hekDesc.h"
#include "Hekateros/hekUtils.h"
#include "Hekateros/hekXmlCfg.h"


using namespace std;
using namespace rnr;
using namespace hekateros;

int HekXmlCfg::load(HekDesc      &desc,
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
      LOGDIAG3("Loading Hekateros description XML file: %s.", fqname.c_str());

      bFoundInstance = true;

      if( (rc = Xml::loadFile(fqname)) < 0 )
      {
        LOGERROR("Parse of Hekateros description from XML file %s failed.",
            fqname.c_str());
        rc = -HEK_ECODE_XML;
      }
      else
      {
        rc = setHekDescFromDOM(desc);
      }

      if( rc == HEK_OK )
      {
        LOGDIAG2("Hekateros description from XML file %s loaded.",
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
    rc = -HEK_ECODE_XML;
  }

  return rc;
}


int HekXmlCfg::createTemplateFile(const string &strXmlFileName)
{
  FILE       *fp;

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
  fprintf(fp, "  <!-- RoadNarrows Hekateros Top-Level Configuration -->\n");
  fprintf(fp, "%s", m_strXmlHead.c_str());

  //
  // Robotic base major element.
  //
  fprintf(fp, "  <!-- Hekateros robotic base -->\n");
  fprintf(fp, "  <%s product_id=\"PRODID\">\n", m_strMajElemArm.c_str());

  fprintf(fp, "    <%s>PRODNAME</%s>\n",
      m_strElemProdName.c_str(), m_strElemProdName.c_str());

  fprintf(fp, "    <%s>\n    PRODBRIEF\n    </%s>\n",
      m_strElemProdBrief.c_str(), m_strElemProdBrief.c_str());

  fprintf(fp, "    <%s>PRODHWVER</%s>\n",
      m_strElemProdHwVer.c_str(), m_strElemProdHwVer.c_str());

  fprintf(fp, "    <%s>DOF</%s>\n",
      m_strElemProdDoF.c_str(), m_strElemProdDoF.c_str());

  fprintf(fp, "    <%s>PRODSIZE</%s>\n",
      m_strElemProdSize.c_str(), m_strElemProdSize.c_str());

  fprintf(fp, "  </%s>\n\n", m_strMajElemArm.c_str());

  //
  // End effector major element.
  //
  fprintf(fp, "  <!-- Hekateros end effector tool -->\n");
  fprintf(fp, "  <%s product_id=\"EEPRODID\">\n", m_strMajElemEE.c_str());

  fprintf(fp, "    <%s>PRODNAME</%s>\n",
      m_strElemProdName.c_str(), m_strElemProdName.c_str());

  fprintf(fp, "    <%s>\n    EEPRODBRIEF\n    </%s>\n",
      m_strElemProdBrief.c_str(), m_strElemProdBrief.c_str());

  fprintf(fp, "    <%s>EEPRODHWVER</%s>\n",
      m_strElemProdHwVer.c_str(), m_strElemProdHwVer.c_str());

  fprintf(fp, "    <%s>EEDOF</%s>\n",
      m_strElemProdDoF.c_str(), m_strElemProdDoF.c_str());

  fprintf(fp, "    <%s>EEPRODSIZE</%s>\n",
      m_strElemProdSize.c_str(), m_strElemProdSize.c_str());

  fprintf(fp, "  </%s>\n\n", m_strMajElemEE.c_str());


  //
  // XML tail
  //
  fprintf(fp, "%s", m_strXmlTail.c_str());

  fclose(fp);

  LOGDIAG3("Created file %s.", m_strXmlFileName.c_str());

  return HEK_OK;
}

int HekXmlCfg::setHekDescFromDOM(HekDesc &desc)
{
  HekDescArm   *pDescArm;
  HekDescEE    *pDescEE;
  TiXmlElement *pElem;
  const char   *sValue;
  const char   *sAttr;
  const char   *sText;
  int           n;
  int           rc;

  //
  // Subsection descriptions.
  //
  pDescArm  = desc.getArmDesc();
  pDescEE   = desc.getEEDesc();

  pDescArm->resetDesc();
  pDescEE->resetDesc();

  if( m_pElemRoot == NULL )
  {
    setErrorMsg("Missing DOM and/or <%s> root element missing.",
       m_strRootElemName.c_str());
    LOGERROR("%s", m_bufErrMsg);
    return -HEK_ECODE_XML;
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
    else if( !strcasecmp(sValue, m_strMajElemArm.c_str()) )
    {
      setHekArmDescFromDOM(pElem, pDescArm);
    }

    // end effecotr description
    else if( !strcasecmp(sValue, m_strMajElemEE.c_str()) )
    {
      setHekEEDescFromDOM(pElem, pDescEE);
    }
  }

  return HEK_OK;
}

int HekXmlCfg::setDOMFromHekDesc(const HekDesc &desc)
{
  // TODO
  return -HEK_ECODE_GEN;
}

int HekXmlCfg::setHekArmDescFromDOM(TiXmlElement *pElemMaj, HekDescArm *pDesc)
{
  TiXmlElement *pElem;
  const char   *sValue;
  string        str;
  int           eProdId;
  string        strProdName;
  string        strProdBrief;
  string        strHwVer;
  int           nDoF = 0;
  int           eProdSize = HekProdSizeUnknown;
  int           rc;

  if( (rc = strToInt(elemAttr(pElemMaj, m_strAttrProdId), eProdId)) < 0 )
  {
    setErrorMsg("%s: No %s attribute of <%s> found or value not an integer.",
      m_strXmlFileName.c_str(),
      m_strAttrProdId.c_str(),
      m_strMajElemArm.c_str());
    LOGERROR("%s", m_bufErrMsg);
    return -HEK_ECODE_XML;
  }

  // child elements
	for(pElem = pElemMaj->FirstChildElement();
      pElem != NULL;
      pElem = pElem->NextSiblingElement())
  {
    if( (sValue = pElem->Value()) == NULL )
    {
      continue;
    }

    // product name
    else if( !strcasecmp(sValue, m_strElemProdName.c_str()) )
    {
      strProdName = elemText(pElem);
    }

    // product brief
    else if( !strcasecmp(sValue, m_strElemProdBrief.c_str()) )
    {
      strProdBrief = elemText(pElem);
    }

    // product hardware version
    else if( !strcasecmp(sValue, m_strElemProdHwVer.c_str()) )
    {
      strHwVer = elemText(pElem);
    }

    // degrees of freedom
    else if( !strcasecmp(sValue, m_strElemProdDoF.c_str()) )
    {
      str = elemText(pElem);
      if( !str.empty() )
      {
        if( (rc = strToInt(str, nDoF)) < 0 )
        {
          setErrorMsg("%s: Element <%s> text \"%s\" not an integer.",
            m_strXmlFileName.c_str(), m_strElemProdDoF.c_str(), str.c_str());
          LOGERROR("%s", m_bufErrMsg);
          return -HEK_ECODE_XML;
        }
      }
    }

    // product size
    else if( !strcasecmp(sValue, m_strElemProdSize.c_str()) )
    {
      str = elemText(pElem);
      if( !str.empty() )
      {
        if( (rc = strToProdSizeCode(str, eProdSize)) < 0 )
        {
          setErrorMsg("%s: Element <%s> text \"%s\" not a recognized "
                      "product size.",
            m_strXmlFileName.c_str(), m_strElemProdSize.c_str(), str.c_str());
          LOGERROR("%s", m_bufErrMsg);
          return -HEK_ECODE_XML;
        }
      }
    }
  }

  pDesc->setDesc(eProdId, strProdName, strProdBrief, strHwVer, nDoF, eProdSize);

  LOGDIAG3("%s: Hekateros robotic base description set.",
      m_strXmlFileName.c_str());

  return HEK_OK;
}

int HekXmlCfg::setHekEEDescFromDOM(TiXmlElement *pElemMaj, HekDescEE *pDesc)
{
  TiXmlElement *pElem;
  const char   *sValue;
  string        str;
  int           eProdId;
  string        strProdName;
  string        strProdBrief;
  string        strHwVer;
  int           nDoF = 0;
  int           eProdSize = HekProdSizeUnknown;
  int           rc;

  if( (rc = strToInt(elemAttr(pElemMaj, m_strAttrProdId), eProdId)) < 0 )
  {
    setErrorMsg("%s: No %s attribute of <%s> found or value not an integer.",
      m_strXmlFileName.c_str(),
      m_strAttrProdId.c_str(),
      m_strMajElemArm.c_str());
    LOGERROR("%s", m_bufErrMsg);
    return -HEK_ECODE_XML;
  }

  // child elements
	for(pElem = pElemMaj->FirstChildElement();
      pElem != NULL;
      pElem = pElem->NextSiblingElement())
  {
    if( (sValue = pElem->Value()) == NULL )
    {
      continue;
    }

    // product name
    else if( !strcasecmp(sValue, m_strElemProdName.c_str()) )
    {
      strProdName = elemText(pElem);
    }

    // product brief
    else if( !strcasecmp(sValue, m_strElemProdBrief.c_str()) )
    {
      strProdBrief = elemText(pElem);
    }

    // product hardware version
    else if( !strcasecmp(sValue, m_strElemProdHwVer.c_str()) )
    {
      strHwVer = elemText(pElem);
    }

    // degrees of freedom
    else if( !strcasecmp(sValue, m_strElemProdDoF.c_str()) )
    {
      str = elemText(pElem);
      if( !str.empty() )
      {
        if( (rc = strToInt(str, nDoF)) < 0 )
        {
          setErrorMsg("%s: Element <%s> text \"%s\" not an integer.",
            m_strXmlFileName.c_str(), m_strElemProdDoF.c_str(), str.c_str());
          LOGERROR("%s", m_bufErrMsg);
          return -HEK_ECODE_XML;
        }
      }
    }

    // product size
    else if( !strcasecmp(sValue, m_strElemProdSize.c_str()) )
    {
      str = elemText(pElem);
      if( !str.empty() )
      {
        if( (rc = strToProdSizeCode(str, eProdSize)) < 0 )
        {
          setErrorMsg("%s: Element <%s> text \"%s\" not a recognized "
                      "product size.",
            m_strXmlFileName.c_str(), m_strElemProdSize.c_str(), str.c_str());
          LOGERROR("%s", m_bufErrMsg);
          return -HEK_ECODE_XML;
        }
      }
    }
  }

  pDesc->setDesc(eProdId, strProdName, strProdBrief, strHwVer, nDoF, eProdSize);

  LOGDIAG3("%s: Hekateros end effector description set.",
      m_strXmlFileName.c_str());

  return HEK_OK;
}

int HekXmlCfg::strToProdSizeCode(const string &str, int &val)
{
  int   rc = HEK_OK;

  if( str.empty() )
  {
    val = HekProdSizeUnknown;
  }
  else if( !strcmp(str.c_str(), HekProdSizeStrUnknown) )
  {
    val = HekProdSizeUnknown;
  }
  else if( !strcmp(str.c_str(), HekProdSizeStrStd) )
  {
    val = HekProdSizeStd;
  }
  else if( !strcmp(str.c_str(), HekProdSizeStrShort) )
  {
    val = HekProdSizeShort;
  }
  else if( !strcmp(str.c_str(), HekProdSizeStrLong) )
  {
    val = HekProdSizeLong;
  }
  else
  {
    LOGERROR("%s: Unknown product size code.", str.c_str());
    rc = -HEK_ECODE_XML;
  }

  return rc;
}

int HekXmlCfg::prodSizeToStr(const int val, string &str)
{
  int   rc = HEK_OK;

  switch(val)
  {
    case HekProdSizeUnknown:
      str = "?";
      break;
    case HekProdSizeStd:
      str = "N";
      break;
    case HekProdSizeShort:
      str = "S";
      break;
    case HekProdSizeLong:
      str = "L";
      break;
    default:
      LOGERROR("%c: Unknown product size code.", val);
      rc = -HEK_ECODE_XML;
      break;
  }

  return rc;
}
