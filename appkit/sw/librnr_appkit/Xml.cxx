////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Library:   librnr_appkit
//
// File:      Xml.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-07-09 17:18:10 -0600 (Thu, 09 Jul 2015) $
 * $Rev: 4025 $
 *
 * \brief XML base class implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2012-2013.  RoadNarrows LLC
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
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

#include <stdio.h>
#include <stdarg.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include <sstream>
#include <string>

#include "tinyxml.h"

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/Xml.h"

using namespace std;
using namespace rnr;


//------------------------------------------------------------------------------
// Xml Base Class
//------------------------------------------------------------------------------

Xml::Xml(const string &strRootElem,
         const string &strXsiUrl,
         const string &strXslUrl) :
  m_strRootElemName(strRootElem),
  m_strXsiUrl(strXsiUrl),
  m_strXslUrl(strXslUrl)
{
  m_pElemRoot     = NULL;
  m_bModified     = false;
  m_bufErrMsg[0]  = 0;
}

Xml::~Xml()
{
}

int Xml::loadFile(const string &strXmlFileName)
{
  if( strXmlFileName.empty() )
  {
    setErrorMsg("No file name.");
    LOGERROR("%s", m_bufErrMsg);
    return RC_ERROR;
  }

  m_strXmlFileName = strXmlFileName;

  // load and parse xml file
	if( !m_xmlDoc.LoadFile(m_strXmlFileName.c_str()) )
  {
    setErrorMsg("%s[r%d,c%d]: %s.",
        m_strXmlFileName.c_str(), m_xmlDoc.ErrorRow(),
        m_xmlDoc.ErrorCol(), m_xmlDoc.ErrorDesc());
    LOGERROR("%s", m_bufErrMsg);
    return RC_ERROR;
  }

  // <rootelem> ... </rootelem>
  m_pElemRoot = m_xmlDoc.RootElement();

  if( m_pElemRoot == NULL )
  {
    setErrorMsg("Not a valid %s XML document: No root <%s> element found.",
      m_strXmlFileName.c_str(), m_strRootElemName.c_str());
    LOGERROR("%s", m_bufErrMsg);
    return RC_ERROR;
  }

  else if( strcasecmp(m_pElemRoot->Value(), m_strRootElemName.c_str()) )
  {
    setErrorMsg("Not a valid %s XML document: Root <%s> element not <%s>.",
      m_strXmlFileName.c_str(), m_pElemRoot->Value(),
      m_strRootElemName.c_str());
    LOGERROR("%s", m_bufErrMsg);
    return RC_ERROR;
  }

  setModifiedState(false);

  LOGDIAG3("XMl document %s loaded.", m_strXmlFileName.c_str());
  
  return OK;
}

int Xml::saveFile(const string &strXmlFileName)
{
  if( strXmlFileName.empty() )
  {
    setErrorMsg("No file name.");
    LOGERROR("%s", m_bufErrMsg);
    return RC_ERROR;
  }

  m_strXmlFileName = strXmlFileName;

  // save xml file
	if( !m_xmlDoc.SaveFile(m_strXmlFileName.c_str()) )
  {
    setErrorMsg("%s[r%d,c%d]: %s.",
        m_strXmlFileName.c_str(), m_xmlDoc.ErrorRow(),
        m_xmlDoc.ErrorCol(), m_xmlDoc.ErrorDesc());
    LOGERROR("%s", m_bufErrMsg);
    return RC_ERROR;
  }
  
  setModifiedState(false);

  LOGDIAG3("XMl document %s saved.", m_strXmlFileName.c_str());

  return OK;
}

int Xml::createTemplateFile(const string &strXmlFileName,
                            const string &strMajorElemName)
{
  FILE  *fp;

  if( strXmlFileName.empty() )
  {
    setErrorMsg("No file name.");
    LOGERROR("%s", m_bufErrMsg);
    return RC_ERROR;
  }

  m_strXmlFileName = strXmlFileName;

  // open file
  if( (fp = fopen(m_strXmlFileName.c_str(), "w+")) == NULL )
  {
    setErrorMsg("%s: %s(errno=%d).",
        m_strXmlFileName.c_str(), strerror(errno), errno);
    LOGERROR("%s", m_bufErrMsg);
    return RC_ERROR;
  }

  makeXmlHead();
  makeXmlTail();

  // create template of document
  fprintf(fp, "%s", m_strXmlHead.c_str());
  fprintf(fp, "<%s>\n</%s>\n",
      strMajorElemName.c_str(), strMajorElemName.c_str());
  fprintf(fp, "%s", m_strXmlTail.c_str());

  fclose(fp);

  LOGDIAG3("Created file %s.", m_strXmlFileName.c_str());

  return OK;
}

bool Xml::fileExists(const string &strXmlFileName, bool bRequired)
{
  bool bAccess;

  if( strXmlFileName.empty() )
  {
    setErrorMsg("No file name.");
    LOGERROR("%s", m_bufErrMsg);
    bAccess = false;
  }
  else
  {
    bAccess = access(strXmlFileName.c_str(), F_OK|R_OK|W_OK) == 0;
  }

  if( !bAccess && bRequired )
  {
    setErrorMsg("%s: %s(errno=%d).",
        strXmlFileName.c_str(), strerror(errno), errno);
    LOGERROR("%s", m_bufErrMsg);
  }

  return bAccess;
}

TiXmlElement *Xml::getMajorElem(const string &strMajorElemName)
{
  TiXmlElement  *pElem;
  const char    *sValue;

  if( strMajorElemName.empty() )
  {
    return NULL;
  }

  if( m_pElemRoot == NULL )
  {
    return NULL;
  }

  // major secion elements
	for(pElem = m_pElemRoot->FirstChildElement();
      pElem != NULL;
      pElem = pElem->NextSiblingElement())
  {
    sValue = pElem->Value();

    if( sValue == NULL )
    {
      continue;
    }
    else if( !strcasecmp(sValue, strMajorElemName.c_str()) )
    {
      return pElem;
    }
  }

  return NULL;
}

string Xml::elemText(TiXmlElement *pElem)
{
  std::string  str;
  const char  *s;

  if( (pElem != NULL) && ((s = pElem->GetText()) != NULL) )
  {
    str = s;
  }

  return str;
}

string Xml::elemAttr(TiXmlElement *pElem, const string &strAttrName)
{
  std::string  str;
  const char  *s;

  if( (pElem != NULL) &&
      !strAttrName.empty() &&
      ((s = pElem->Attribute(strAttrName.c_str())) != NULL) )
  {
    str = s;
  }

  return str;
}

int Xml::strToInt(const string &str, int &val)
{
  long long int val1; // must use 64-bit for linaro 32bit

  if( sscanf(str.c_str(), "%lli", &val1) != 1 )
  {
    LOGERROR("%s: Bad integer string representation.", str.c_str());
    return RC_ERROR;
  }

  val = (int)val1;

  return OK;
}

int Xml::intToStr(const int val, string &str, int base)
{
  char buf[64];

  switch( base )
  {
    case 16:
      snprintf(buf, sizeof(buf), "0x%x", val);
      break;
    case 10:
      snprintf(buf, sizeof(buf), "%d", val);
      break;
    case 8:
      snprintf(buf, sizeof(buf), "0%o", val);
      break;
    default:
      LOGERROR("%d: Unsupported base conversion.", base);
      return RC_ERROR;
  }

  buf[sizeof(buf)-1] = 0;
    
  str = buf;

  return OK;
}

int Xml::strToDouble(const string &str, double &val)
{
  if( sscanf(str.c_str(), "%lf", &val) != 1 )
  {
    LOGERROR("%s: Bad floating-point number string representation.",
        str.c_str());
    return RC_ERROR;
  }


  return OK;
}

int Xml::doubleToStr(const double val, string &str)
{
  char buf[64];

  snprintf(buf, sizeof(buf), "%lf", val);

  buf[sizeof(buf)-1] = 0;
    
  str = buf;

  return OK;
}

void Xml::makeXmlHead()
{
  stringstream  ssStylesheetStmt;
  stringstream  ssRootStmt;
  stringstream  ssHead;

  // no stylesheet
  if( m_strXslUrl.empty() )
  {
    ssStylesheetStmt << "";
  }

  // xslt stylesheet
  else
  {
    ssStylesheetStmt << "<?xml-stylestheet type=\"text/xsl\" href=\""
                     << m_strXslUrl
                     << "\"?>\n";
  }

  // no schema
  if( m_strXsiUrl.empty() )
  {
    ssRootStmt << "<" << m_strRootElemName << ">";
  }

  // validation schema
  else
  {
    ssRootStmt << "<" << m_strRootElemName << " " << XmlNsXsi << "\n"
            << "    xsi:noNamespaceSchemaLocation=\"" << m_strXsiUrl << "\">";
  }
 
  ssHead << XmlDecl << "\n"
         << ssStylesheetStmt.str() << "\n"
         << ssRootStmt.str() << ">\n\n";

  m_strXmlHead = ssHead.str();
}

void Xml::makeXmlTail()
{
  m_strXmlTail = "\n</" + m_strRootElemName + ">\n";
}

void Xml::setErrorMsg(const char *sFmt, ...)
{
  va_list         ap;

  // format error message
  va_start(ap, sFmt);
  vsnprintf(m_bufErrMsg, sizeof(m_bufErrMsg), sFmt, ap);
  m_bufErrMsg[sizeof(m_bufErrMsg)-1] = 0;
  va_end(ap);
}
