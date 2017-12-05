////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_appkit
//
// File:      CmdExtArg.cxx
//
/*! \file
 *
 * \brief Command line extended argument implementation.
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
#include <ctype.h>

#include <iostream>
#include <sstream>
#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/appkit/LogBook.h"
#include "rnr/appkit/CmdCore.h"
#include "rnr/appkit/CmdExtArg.h"

using namespace std;
using namespace rnr;
using namespace rnr::cmd;

// -----------------------------------------------------------------------------
// CmdExtArg Class
// -----------------------------------------------------------------------------

CmdExtArg::CmdExtArg()
{
  m_nCmdUid       = NoUid;
  m_nFormIndex    = NoIndex;
  m_nArgIndex     = NoIndex;
  m_nArgInstance  = 0;
  m_eCvtType      = CvtTypeUndef;
  m_bCvtVal       = false;
  m_lCvtVal       = 0;
  m_fCvtVal       = 0.0;
}

CmdExtArg::CmdExtArg(const int    &nCmdUid,
                     const int    &nFormIndex,
                     const int    &nArgIndex,
                     const int    &nArgInstance,
                     const string &strArg) :
    m_nCmdUid(nCmdUid),
    m_nFormIndex(nFormIndex),
    m_nArgIndex(nArgIndex),
    m_nArgInstance(nArgInstance),
    m_strArg(strArg)
{
  m_eCvtType      = CvtTypeUndef;
  m_bCvtVal       = false;
  m_lCvtVal       = 0;
  m_fCvtVal       = 0.0;
}

CmdExtArg::CmdExtArg(const CmdExtArg &src)
{
  m_nCmdUid       = src.m_nCmdUid;
  m_nFormIndex    = src.m_nFormIndex;
  m_nArgIndex     = src.m_nArgIndex;
  m_nArgInstance  = src.m_nArgInstance;
  m_strArg        = src.m_strArg;
  m_eCvtType      = src.m_eCvtType;
  m_strCvtVal     = src.m_strCvtVal;
  m_bCvtVal       = src.m_bCvtVal;
  m_lCvtVal       = src.m_lCvtVal;
  m_fCvtVal       = src.m_fCvtVal;
}

CmdExtArg::~CmdExtArg()
{
}

CmdExtArg &CmdExtArg::operator=(const CmdExtArg &rhs)
{
  m_nCmdUid       = rhs.m_nCmdUid;
  m_nFormIndex    = rhs.m_nFormIndex;
  m_nArgIndex     = rhs.m_nArgIndex;
  m_nArgInstance  = rhs.m_nArgInstance;
  m_strArg        = rhs.m_strArg;
  m_eCvtType      = rhs.m_eCvtType;
  m_strCvtVal     = rhs.m_strCvtVal;
  m_bCvtVal       = rhs.m_bCvtVal;
  m_lCvtVal       = rhs.m_lCvtVal;
  m_fCvtVal       = rhs.m_fCvtVal;
}

void CmdExtArg::s(const string &strVal)
{
  m_strCvtVal = strVal;
  m_eCvtType  = CvtTypeString;
}

void CmdExtArg::e(const long eVal)
{
  m_strCvtVal = m_strArg;
  m_lCvtVal   = eVal;
  m_eCvtType  = CvtTypeEnum;
}

void CmdExtArg::b(const bool bVal)
{
  m_bCvtVal   = bVal;
  m_eCvtType  = CvtTypeBoolean;
}

void CmdExtArg::i(const long lVal)
{
  m_lCvtVal   = lVal;
  m_eCvtType  = CvtTypeInteger;
}

void CmdExtArg::f(const double fVal)
{
  m_fCvtVal   = fVal;
  m_eCvtType  = CvtTypeFpn;
}

bool CmdExtArg::operator==(const CmdExtArg &rval) const
{
  return (type() == rval.type()) && (arg() == rval.arg());
}

bool CmdExtArg::operator==(const std::string &rval) const
{
  return ((type() == CvtTypeString) || (type() == CvtTypeEnum)) &&
          (s() == rval);
}

bool CmdExtArg::operator==(const char* const &rval) const
{
  if( rval == NULL )
  {
    return false;
  }

  return ((type() == CvtTypeString) || (type() == CvtTypeEnum)) &&
          (strcmp(s().c_str(), rval) == 0);
}

bool CmdExtArg::operator==(const bool &rval) const
{
  return (type() == CvtTypeBoolean) && (b() == rval);
}

bool CmdExtArg::operator==(const long &rval) const
{
  return ((type() == CvtTypeEnum) || (type() == CvtTypeInteger)) &&
          (i() == rval);
}

bool CmdExtArg::operator==(const double &rval) const
{
  return (type() == CvtTypeFpn) && (f() == rval);
}

ostream &rnr::cmd::operator<<(ostream &os, const CmdExtArg &arg)
{
  switch( arg.type() )
  {
    case CmdExtArg::CvtTypeString:
      os << arg.s();
      break;
    case CmdExtArg::CvtTypeEnum:
      os << arg.s();  // string has priority over enum
      break;
    case CmdExtArg::CvtTypeBoolean:
      os << arg.b();
      break;
    case CmdExtArg::CvtTypeInteger:
      os << arg.i();
      break;
    case CmdExtArg::CvtTypeFpn:
      os << arg.f();
      break;
    case CmdExtArg::CvtTypeUndef:
    default:
      os << undefstring;
      break;
  }

  return os;
}

LogBook &rnr::cmd::operator<<(LogBook &log, const CmdExtArg &arg)
{
  stringstream ss;

  ss  << arg;
  log << ss.str();

  return log;
}
