////////////////////////////////////////////////////////////////////////////////
//
// Package:   Kuon
//
// Library:   libkuon
//
// File:      kuonXmlCfg.cxx
//
/*! \file
 *
 * $LastChangedDate: 2014-04-09 14:29:33 -0600 (Wed, 09 Apr 2014) $
 * $Rev: 3635 $
 *
 * \brief \h_kuon XML configuration class implementation.
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

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/appkit/Xml.h"

#include "Kuon/kuon.h"
#include "Kuon/kuonDescBase.h"
#include "Kuon/kuonDesc.h"
#include "Kuon/kuonXmlCfg.h"


using namespace std;
using namespace rnr;
using namespace kuon;


int KuonXmlCfg::createTemplateFile(const string &strXmlFileName)
{
  FILE       *fp;

  if( strXmlFileName.empty() )
  {
    setErrorMsg("No file name.");
    LOGERROR("%s", m_bufErrMsg);
    return -KUON_ECODE_XML;
  }

  m_strXmlFileName = strXmlFileName;

  // open file
  if( (fp = fopen(m_strXmlFileName.c_str(), "w+")) == NULL )
  {
    setErrorMsg("%s: %s(errno=%d).",
        m_strXmlFileName.c_str(), strerror(errno), errno);
    LOGERROR("%s", m_bufErrMsg);
    return -KUON_ECODE_XML;
  }

  makeXmlHead();
  makeXmlTail();

  //
  // XML head.
  //
  fprintf(fp, "  <!-- RoadNarrows Kuon Top-Level Configuration -->\n");
  fprintf(fp, "%s", m_strXmlHead.c_str());

  //
  // Robotic base major element.
  //
  fprintf(fp, "  <!-- Kuon robotic mobile platform -->\n");
  fprintf(fp, "  <%s product_id=\"PRODID\">\n", m_strMajElemBase.c_str());

  fprintf(fp, "    <%s>PRODNAME</%s>\n",
      m_strElemProdName.c_str(), m_strElemProdName.c_str());

  fprintf(fp, "    <%s>\n    PRODBRIEF\n    </%s>\n",
      m_strElemProdBrief.c_str(), m_strElemProdBrief.c_str());

  fprintf(fp, "    <%s>PRODHWVER</%s>\n",
      m_strElemProdHwVer.c_str(), m_strElemProdHwVer.c_str());

  fprintf(fp, "    <%s>FRONTTIRE</%s>\n",
      m_strElemProdFrontTire.c_str(), m_strElemProdFrontTire.c_str());

  fprintf(fp, "    <%s>REARTIRE</%s>\n",
      m_strElemProdRearTire.c_str(), m_strElemProdRearTire.c_str());

  fprintf(fp, "  </%s>\n\n", m_strMajElemBase.c_str());


  //
  // XML tail
  //
  fprintf(fp, "%s", m_strXmlTail.c_str());

  fclose(fp);

  LOGDIAG3("Created file %s.", m_strXmlFileName.c_str());

  return KUON_OK;
}

int KuonXmlCfg::setKuonDescFromDOM(KuonDesc &desc)
{
  KuonDescBase *pDescBase;
  TiXmlElement *pElem;
  const char   *sValue;
  const char   *sAttr;
  const char   *sText;
  int           n;
  int           rc;

  //
  // Subsection descriptions.
  //
  pDescBase  = desc.getBaseDesc();

  pDescBase->resetDesc();

  if( m_pElemRoot == NULL )
  {
    setErrorMsg("Missing DOM and/or <%s> root element missing.",
       m_strRootElemName.c_str());
    LOGERROR("%s", m_bufErrMsg);
    return -KUON_ECODE_XML;
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
      setKuonBaseDescFromDOM(pElem, pDescBase);
    }
  }

  return KUON_OK;
}

int KuonXmlCfg::setDOMFromKuonDesc(const KuonDesc &desc)
{
  // TODO
  return -KUON_ECODE_GEN;
}

int KuonXmlCfg::setKuonBaseDescFromDOM(TiXmlElement *pElemMaj,
                                       KuonDescBase *pDesc)
{
  TiXmlElement *pElem;
  const char   *sValue;
  string        str;
  int           eProdId;
  string        strProdName;
  string        strProdBrief;
  string        strHwVer;
  double        fFrontTire = 0.0;
  double        fRearTire  = 0.0;
  int           rc;

  if( (rc = strToInt(elemAttr(pElemMaj, m_strAttrProdId), eProdId)) < 0 )
  {
    setErrorMsg("%s: No %s attribute of <%s> found or value not an integer.",
      m_strXmlFileName.c_str(),
      m_strAttrProdId.c_str(),
      m_strMajElemBase.c_str());
    LOGERROR("%s", m_bufErrMsg);
    return -KUON_ECODE_XML;
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

    // front tires radius
    else if( !strcasecmp(sValue, m_strElemProdFrontTire.c_str()) )
    {
      str = elemText(pElem);
      if( !str.empty() )
      {
        if( (rc = strToDouble(str, fFrontTire)) < 0 )
        {
          setErrorMsg("%s: Element <%s> text \"%s\" not an number.",
            m_strXmlFileName.c_str(), m_strElemProdFrontTire.c_str(),
            str.c_str());
          LOGERROR("%s", m_bufErrMsg);
          return -KUON_ECODE_XML;
        }
      }
    }

    // rear tires radius
    else if( !strcasecmp(sValue, m_strElemProdRearTire.c_str()) )
    {
      str = elemText(pElem);
      if( !str.empty() )
      {
        if( (rc = strToDouble(str, fRearTire)) < 0 )
        {
          setErrorMsg("%s: Element <%s> text \"%s\" not an number.",
            m_strXmlFileName.c_str(), m_strElemProdRearTire.c_str(),
            str.c_str());
          LOGERROR("%s", m_bufErrMsg);
          return -KUON_ECODE_XML;
        }
      }
    }
  }

  pDesc->setDesc(eProdId, strProdName, strProdBrief, strHwVer,
                  fFrontTire, fRearTire);

  LOGDIAG3("%s: Kuon robotic base description set.",
      m_strXmlFileName.c_str());

  return KUON_OK;
}
