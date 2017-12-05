////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_appkit
//
// File:      CmdDef.cxx
//
/*! \file
 *
 * \brief Command line command definition class interface.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2016-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 *
 * \par License:
 * MIT
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
#include <unistd.h>
#include <stdlib.h>

#include <iostream>
#include <sstream>
#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/appkit/StringTheory.h"
#include "rnr/appkit/IOManip.h"
#include "rnr/appkit/CmdCore.h"
#include "rnr/appkit/CmdExtArg.h"
#include "rnr/appkit/CmdArgDef.h"
#include "rnr/appkit/CmdFormDef.h"
#include "rnr/appkit/CmdDef.h"

using namespace std;
using namespace rnr;
using namespace rnr::io;
using namespace rnr::cmd;

// -----------------------------------------------------------------------------
// Support
// -----------------------------------------------------------------------------

namespace rnr
{
  namespace cmd
  {
    static CmdFormDef noformdef;  ///< constant "no form def" form definition
  } // namespace cmd
} // namespace rnr


// -----------------------------------------------------------------------------
// CmdDef Class
// -----------------------------------------------------------------------------

CmdDef::CmdDef()
{
  m_nUid      = NoUid;
}

CmdDef::CmdDef(const CmdDef &src)
{
  m_nUid        = src.m_nUid;
  m_strName     = src.m_strName;
  m_strSyntax   = src.m_strSyntax;
  m_strSynopsis = src.m_strSynopsis;
  m_strLongDesc = src.m_strLongDesc;
  m_formDefs    = src.m_formDefs;
}

CmdDef::~CmdDef()
{
}

CmdDef &CmdDef::operator=(const CmdDef &rhs)
{
  m_nUid        = rhs.m_nUid;
  m_strName     = rhs.m_strName;
  m_strSyntax   = rhs.m_strSyntax;
  m_strSynopsis = rhs.m_strSynopsis;
  m_strLongDesc = rhs.m_strLongDesc;
  m_formDefs    = rhs.m_formDefs;
}

bool CmdDef::isDefined() const
{
  return  (m_nUid >= 0)             &&
          !m_strName.empty()        &&
          (numOfForms() > 0)        &&
          m_formDefs[0].isDefined();
}

void CmdDef::reset()
{
  for(size_t i = 0; i < m_formDefs.size(); ++i)
  {
    m_formDefs[i].reset();
  }
}

void CmdDef::setUid(const int uid)
{
  m_nUid = uid;
}

void CmdDef::setName(const string &strName)
{
  m_strName = strName;
}

void CmdDef::addHelp(const char *sSynopsis, const char *sLongDesc)
{
  string strSynopsis(sSynopsis != NULL? sSynopsis: "");
  string strLongDesc(sLongDesc != NULL? sLongDesc: "");

  addHelp(strSynopsis, strLongDesc);
}

void CmdDef::addHelp(const string &strSynopsis, const string &strLongDesc)
{
  m_strSynopsis = strSynopsis;
  m_strLongDesc = strLongDesc;
}

void CmdDef::pushForm(CmdFormDef &formdef)
{
  int nIndex = numOfForms();

  formdef.setParent(m_nUid);
  formdef.setIndex(nIndex);
  m_formDefs.push_back(formdef);
}

const CmdFormDef &CmdDef::at(const int nIndex) const
{
  if( (nIndex >= 0) && (nIndex < m_formDefs.size()) )
  {
    return m_formDefs[nIndex];
  }
  else
  {
    return noformdef;
  }
}

CmdFormDef &CmdDef::formAt(const int nIndex)
{
  if( (nIndex >= 0) && (nIndex < m_formDefs.size()) )
  {
    return m_formDefs[nIndex];
  }
  else
  {
    return noformdef;
  }
}

ostream &rnr::cmd::operator<<(ostream &os, const CmdDef &cmddef)
{
  string  sep;

  os << indent() << "{" << endl;

  os << deltaindent(2);

  os << indent() << "uid         = " << cmddef.m_nUid << endl;
  os << indent() << "name        = " << cmddef.m_strName << endl;
  os << indent() << "syntax      = " << cmddef.m_strSyntax << endl;
  os << indent() << "synopsis    = " << cmddef.m_strSynopsis << endl;
  os << indent() << "description = " << cmddef.m_strLongDesc << endl;
  os << indent() << "forms[" << cmddef.numOfForms() << "] = " << endl;
  os << indent() << "{" << endl;

  os << deltaindent(2);

  for(size_t i = 0; i < cmddef.m_formDefs.size(); ++i)
  {
    os << sep << cmddef.m_formDefs[i];

    if( sep.empty() )
    {
      sep = ",\n";
    }
  }
  os << endl;

  os << deltaindent(-2);

  os << indent() << "}" << endl;

  os << deltaindent(-2);

  os << indent() << "}";

  return os;
}
