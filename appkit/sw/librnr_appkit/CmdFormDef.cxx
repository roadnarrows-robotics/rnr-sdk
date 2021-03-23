////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_appkit
//
// File:      CmdFormDef.cxx
//
/*! \file
 *
 * \brief Command line command form definition class implementation.
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
#include <string>
#include <vector>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/appkit/IOManip.h"
#include "rnr/appkit/CmdCore.h"
#include "rnr/appkit/CmdArgDef.h"
#include "rnr/appkit/CmdFormDef.h"

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
    static CmdArgDef  noargdef; ///< constant "no arg def" argument definition

  } // namespace cmd
} // namespace rnr


// -----------------------------------------------------------------------------
// CmdFormDef Class
// -----------------------------------------------------------------------------

CmdFormDef::CmdFormDef()
{
  m_nCmdUid   = NoUid;
  m_nIndex    = NoIndex;
  m_nArgcReq  = 0;
  m_nArgcOpt  = 0;
}

CmdFormDef::CmdFormDef(const string &strSyntax) :
    m_strSyntax(strSyntax)
{
  m_nCmdUid   = NoUid;
  m_nIndex    = NoIndex;
  m_nArgcReq  = 0;
  m_nArgcOpt  = 0;
}

CmdFormDef::CmdFormDef(const CmdFormDef &src)
{
  m_nCmdUid   = src.m_nCmdUid;
  m_nIndex    = src.m_nIndex;
  m_strSyntax = src.m_strSyntax;
  m_argDefs   = src.m_argDefs;
  m_nArgcReq  = src.m_nArgcReq;
  m_nArgcOpt  = src.m_nArgcOpt;
}

CmdFormDef::~CmdFormDef()
{
}

CmdFormDef &CmdFormDef::operator=(const CmdFormDef &rhs)
{
  m_nCmdUid   = rhs.m_nCmdUid;
  m_nIndex    = rhs.m_nIndex;
  m_strSyntax = rhs.m_strSyntax;
  m_argDefs   = rhs.m_argDefs;
  m_nArgcReq  = rhs.m_nArgcReq;
  m_nArgcOpt  = rhs.m_nArgcOpt;

  return *this;
}

bool CmdFormDef::isDefined() const
{
  return  (m_nIndex >= 0)           &&
          !m_strSyntax.empty()      &&
          (m_argDefs.size() > 0)    &&
          m_argDefs[0].isDefined();
}

void CmdFormDef::reset()
{
  m_argDefs.clear();
  m_nArgcReq  = 0;
  m_nArgcOpt  = 0;
}

void CmdFormDef::setParent(const int nCmdUid)
{
  m_nCmdUid = nCmdUid;
}

void CmdFormDef::setIndex(const int nIndex)
{
  m_nIndex = nIndex;
}

void CmdFormDef::setSyntax(const string &strSyntax)
{
  m_strSyntax = strSyntax;
}

void CmdFormDef::pushArg(CmdArgDef &argdef)
{
  int nIndex = numOfArgs();

  argdef.setParent(m_nCmdUid, m_nIndex);
  argdef.setIndex(nIndex);
  m_argDefs.push_back(argdef);
}

const CmdArgDef &CmdFormDef::at(const int nIndex) const
{
  if( (nIndex >= 0) && (nIndex < m_argDefs.size()) )
  {
    return m_argDefs[nIndex];
  }
  else
  {
    return noargdef;
  }
}

CmdArgDef &CmdFormDef::argAt(const int nIndex)
{
  if( (nIndex >= 0) && (nIndex < m_argDefs.size()) )
  {
    return m_argDefs[nIndex];
  }
  else
  {
    return noargdef;
  }
}

CmdArgDef &CmdFormDef::lastArg()
{
  return m_argDefs.empty()? noargdef: m_argDefs.back();
}

ostream &rnr::cmd::operator<<(ostream &os, const CmdFormDef &formdef)
{
  string  sep;

  os << indent() << "{" << endl;

  os << deltaindent(2);

  os << indent() << "cmduid  = " << formdef.m_nCmdUid << endl;
  os << indent() << "index   = " << formdef.m_nIndex << endl;
  os << indent() << "syntax  = " << formdef.m_strSyntax << endl;
  os << indent() << "argc    = " << formdef.numOfArgs() << "(total)"
        << " " << formdef.numOfRequiredArgs() << "(required)"
        << " " << formdef.numOfOptionalArgs() << "(optional)"
        << endl;

  os << indent() << "args[" << formdef.numOfArgs() << "] =" << endl;
  os << indent() << "{" << endl;

  os << deltaindent(2);

  for(size_t i = 0; i < formdef.m_argDefs.size(); ++i)
  {
    os << sep << formdef.m_argDefs[i];
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

