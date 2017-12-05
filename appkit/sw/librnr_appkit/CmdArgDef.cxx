////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_appkit
//
// File:      CmdArgDef.cxx
//
/*! \file
 *
 * \brief Command line argument definition class implementation.
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
#include <locale>
#include <vector>
#include <set>
#include <map>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/appkit/StringTheory.h"
#include "rnr/appkit/IOManip.h"
#include "rnr/appkit/RegEx.h"
#include "rnr/appkit/LogBook.h"
#include "rnr/appkit/CmdCore.h"
#include "rnr/appkit/CmdExtArg.h"
#include "rnr/appkit/CmdArgDef.h"

using namespace std;
using namespace rnr;
using namespace rnr::str;
using namespace rnr::io;
using namespace rnr::cmd;


// -----------------------------------------------------------------------------
// Support
// -----------------------------------------------------------------------------

namespace rnr
{
  namespace cmd
  {
    static const string noliteral;  ///< no literal string

    /*!
     * \brief Argument type - string symbol lookup table.
     */
    static NameValuePair ArgTypeLookupTbl[] =
    {
      {ArgSymLiteral,     CmdArgDef::ArgTypeLiteral},
      {ArgSymWord,        CmdArgDef::ArgTypeWord},
      {ArgSymMultiWord,   CmdArgDef::ArgTypeMultiWord},
      {ArgSymIdentifier,  CmdArgDef::ArgTypeIdentifier},
      {ArgSymBoolean,     CmdArgDef::ArgTypeBoolean},
      {ArgSymInteger,     CmdArgDef::ArgTypeInteger},
      {ArgSymFpn,         CmdArgDef::ArgTypeFpn},
      {ArgSymFile,        CmdArgDef::ArgTypeFile},
      {ArgSymRegEx,       CmdArgDef::ArgTypeRegEx},
      {NULL,              CmdArgDef::ArgTypeUndef}
    };

    /*!
     * \brief Argument flag modifiers - name lookup table.
     */
    static NameValuePair ArgFlagLookupTbl[] =
    {
      {"FlagCommand",   CmdArgDef::FlagCommand},
      {"FlagOptional",  CmdArgDef::FlagOptional},
      {"FlagXorList",   CmdArgDef::FlagXorList},
      {NULL,            0}
    };

  } // namespace cmd
} // namespace rnr


// -----------------------------------------------------------------------------
// CmdArgDef Class
// -----------------------------------------------------------------------------

CmdArgDef::CmdArgDef()
{
  m_nCmdUid     = NoUid;
  m_nFormIndex  = -1;
  m_nIndex      = -1;
  m_eType       = ArgTypeUndef;
  m_uFlags      = 0;
}

CmdArgDef::CmdArgDef(const CmdArgDef &src)
{
  m_nCmdUid     = src.m_nCmdUid;
  m_nFormIndex  = src.m_nFormIndex;
  m_nIndex      = src.m_nIndex;
  m_strName     = src.m_strName;
  m_eType       = src.m_eType;
  m_literals    = src.m_literals;
  m_ranges      = src.m_ranges;
  m_re          = src.m_re;
  m_uFlags      = src.m_uFlags;
}

CmdArgDef::~CmdArgDef()
{
}

CmdArgDef &CmdArgDef::operator=(const CmdArgDef &rhs)
{
  m_nCmdUid     = rhs.m_nCmdUid;
  m_nFormIndex  = rhs.m_nFormIndex;
  m_nIndex      = rhs.m_nIndex;
  m_strName     = rhs.m_strName;
  m_eType       = rhs.m_eType;
  m_literals    = rhs.m_literals;
  m_ranges      = rhs.m_ranges;
  m_re          = rhs.m_re;
  m_uFlags      = rhs.m_uFlags;

  return *this;
}

bool CmdArgDef::isDefined() const
{
  return (m_nIndex >= 0) && !m_strName.empty() && (m_eType != ArgTypeUndef);
}

void CmdArgDef::setParent(const int nCmdUid, const int nFormIndex)
{
  m_nCmdUid     = nCmdUid;
  m_nFormIndex  = nFormIndex;
}

void CmdArgDef::setIndex(const int nIndex)
{
  m_nIndex = nIndex;
}

void CmdArgDef::setName(const string &strName)
{
  m_strName = strName;
}

void CmdArgDef::setType(const ArgType eType)
{
  m_eType = eType;
}

void CmdArgDef::addLiteralValue(const string &strValue)
{
  m_literals.push_back(strValue);
}

const string &CmdArgDef::literalAt(const int nIndex) const
{
  if( (nIndex >= 0) && (nIndex < m_literals.size()) )
  {
    return m_literals[nIndex];
  }
  else
  {
    return noliteral;
  }
}

void CmdArgDef::setRanges(const RangeVec &ranges)
{
  m_ranges = ranges;
}

bool CmdArgDef::inRange(const long value) const
{
  return inRange((double)value);
}

bool CmdArgDef::inRange(const double value) const
{
  // no minimum or maximum
  if( m_ranges.size() == 0 )
  {
    return true;
  }

  for(size_t i = 0; i < m_ranges.size(); ++i)
  {
    if( (value >= m_ranges[i].min) && (value <= m_ranges[i].max) )
    {
      return true;
    }
  }

  return false;
}

void CmdArgDef::setRegEx(const RegEx &re)
{
  m_re = re;
}

void CmdArgDef::orFlags(const unsigned uFlags)
{
  m_uFlags |= uFlags;
}

string CmdArgDef::constructLiteralList(const string sep) const
{
  stringstream  ss;
  string        pre;

  for(size_t i = 0; i < numOfLiterals(); ++i)
  {
    ss << pre << literalAt(i);
    pre = sep;
  }

  return ss.str();
}

string CmdArgDef::constructRangeList(const string sep) const
{
  stringstream  ss;
  string        pre;

  for(size_t i = 0; i < m_ranges.size(); ++i)
  {
    ss << pre << m_ranges[i].min << ":" << m_ranges[i].max;
    pre = sep;
  }

  return ss.str();
}

string CmdArgDef::constructSyntax() const
{
  stringstream  ss;
  string        sep;

  switch( m_eType )
  {
    case ArgTypeLiteral:
      ss << "{" << constructLiteralList(" | ") << "}";
      break;
    case ArgTypeWord:
    case ArgTypeMultiWord:
    case ArgTypeIdentifier:
    case ArgTypeBoolean:
    case ArgTypeFile:
      ss << "<" << m_strName << ":" << lookupArgSymbol(m_eType) << ">";
      break;
    case ArgTypeInteger:
    case ArgTypeFpn:
      ss << "<" << m_strName << ":" << lookupArgSymbol(m_eType);
      if( m_ranges.size() > 0 )
      {
        ss << "(" << constructRangeList() << ")";
      }
      ss << ">";
      break;
    case ArgTypeRegEx:
      ss << "<" << m_strName << ":" << lookupArgSymbol(m_eType);
      if( m_re.isValid() )
      {
        ss << "(" << m_re.getRegEx() << ")";
      }
      ss << ">";
      break;
    default:
      ss << "<" << m_strName << ">";
      break;
  }

  return ss.str();
}

double CmdArgDef::match(const string &strArg, const bool bIgnoreCase) const
{
  double  fWeight = 0.0;

  switch( m_eType )
  {
    //
    // Literal constant.
    //
    case ArgTypeLiteral:
      if( matchLiteral(strArg, bIgnoreCase) >= 0 )
      {
        fWeight = 1.0;
      }
      break;

    //
    // Contiguous block of non-whitespace characters.
    //
    case ArgTypeWord:
      if( strArg.find_first_of(" \t\r\n\v\f") == string::npos )
      {
        fWeight = 0.91;
      }
      break;
      
    //
    // Any string.
    //
    case ArgTypeMultiWord:
      fWeight = 0.90;
      break;

    //
    // C conforming identifier.
    //
    case ArgTypeIdentifier:
      if( isIdentifier(strArg) )
      {
        fWeight = 0.94;
      }
      break;

    //
    // Boolean.
    //
    case ArgTypeBoolean:
      {
        bool  val;
        int   rc;
        if( bIgnoreCase )
        {
          rc = tobool(lowercase(strArg), val);
        }
        else
        {
          rc = tobool(strArg, val);
        }

        if( rc == OK )
        {
          fWeight = 0.95;
        }
      }
      break;

    //
    // (Signed) integer in base 8, 10, or 16.
    //
    case ArgTypeInteger:
      {
        long  val;
        if( (tolong(strArg, val) == OK) && inRange(val) )
        {
          fWeight = 0.95;
        }
      }
      break;

    //
    // Floating-point number.
    // 
    case ArgTypeFpn:
      {
        double val;
        if( (todouble(strArg, val) == OK) && inRange(val) )
        {
          fWeight = 0.95;
        }
      }
      break;

    //
    // Regular expression.
    //
    case ArgTypeRegEx:
      if( m_re.isValid() && m_re.match(strArg) )
      {
        fWeight = 0.93;
      }
      break;

    //
    // Filename.
    //
    case ArgTypeFile:
      fWeight = 0.91;
      break;

    //
    // Bug.
    //
    case ArgTypeUndef:
    default:
      stringstream ss;
      ss
        << "Bug: "
        << "cmduid " << m_nCmdUid << ", "
        << "form " << m_nFormIndex << ", "
        << "arg " << m_nIndex << ": "
        << m_eType << ": Unknown type.";
      LOGERROR("%s", ss.str().c_str());
      break;
  } 

  return fWeight;
}

int CmdArgDef::matchLiteral(const string &strArg, const bool bIgnoreCase) const
{
  if( bIgnoreCase )
  {
    string strICaseArg = lowercase(strArg);

    for(size_t i = 0; i < numOfLiterals(); ++i)
    {
      if( strICaseArg == lowercase(literalAt(i)) )
      {
        return (int)i;
      }
    }
  }
  else
  {
    for(size_t i = 0; i < numOfLiterals(); ++i)
    {
      if( strArg == literalAt(i) )
      {
        return (int)i;
      }
    }
  }

  return -1;
}

CmdExtArg CmdArgDef::convert(const string &strArg, const bool bIgnoreCase) const
{
  CmdExtArg cvt(m_nCmdUid, m_nFormIndex, m_nIndex, 0, strArg);

  switch( m_eType )
  {
    // 
    // Converted literal value is the literal enum list index.
    //
    case ArgTypeLiteral:
      {
        long  val;

        if( (val = matchLiteral(strArg, bIgnoreCase)) >= 0 )
        {
          cvt.e(val);
        }
      }
      break;

    //
    // The converted value is the argument for these argument types.
    //
    case ArgTypeWord:
    case ArgTypeMultiWord:
    case ArgTypeIdentifier:
    case ArgTypeFile:
    case ArgTypeRegEx:
      cvt.s(strArg);
      break;

    //
    // Converted to boolean.
    //
    case ArgTypeBoolean:
      {
        bool    val;
        int     rc;

        if( bIgnoreCase )
        {
          rc = tobool(lowercase(strArg), val);
        }
        else
        {
          rc = tobool(strArg, val);
        }
        if( rc == OK )
        {
          cvt.b(val);
        }
      }
      break;

    //
    // Convert to integer.
    //
    case ArgTypeInteger:
      {
        long val;

        if( tolong(strArg, val) == OK )
        {
          cvt.i(val);
        }
      }
      break;

    //
    // Convert to floating-point number.
    //
    case ArgTypeFpn:
      {
        double val;

        if( todouble(strArg, val) == OK )
        {
          cvt.f(val);
        }
      }
      break;

    //
    // Unknown or uninitialized.
    //
    case ArgTypeUndef:
    default:
      break;
  }

  return cvt;
}

CmdArgDef::ArgType CmdArgDef::lookupArgType(const string strSymbol)
{
  for(int i = 0; ArgTypeLookupTbl[i].m_sName != NULL; ++i)
  {
    if( strSymbol == ArgTypeLookupTbl[i].m_sName )
    {
      return (CmdArgDef::ArgType)ArgTypeLookupTbl[i].m_nValue; 
    }
  }
  return CmdArgDef::ArgTypeUndef;
}

const string CmdArgDef::lookupArgSymbol(const CmdArgDef::ArgType eType)
{
  for(int i = 0; ArgTypeLookupTbl[i].m_sName != NULL; ++i)
  {
    if( eType == ArgTypeLookupTbl[i].m_nValue ) 
    {
      return ArgTypeLookupTbl[i].m_sName;
    }
  }
  return undefstring;
}

const string CmdArgDef::lookupFlagNames(const unsigned uFlags)
{
  stringstream  ss;
  string        sep;

  for(int i = 0; ArgFlagLookupTbl[i].m_sName != NULL; ++i)
  {
    if( uFlags & ArgFlagLookupTbl[i].m_nValue )
    {
      ss << sep << ArgFlagLookupTbl[i].m_sName;
      sep = "|";
    }
  }
  return ss.str();
}

ostream &rnr::cmd::operator<<(ostream &os, const CmdArgDef &argdef)
{
  os << indent() << "{" << endl;

  os << deltaindent(2);

  os << indent() << "cmduid      = " << argdef.m_nCmdUid << endl;
  os << indent() << "formindex   = " << argdef.m_nFormIndex << endl;
  os << indent() << "index       = " << argdef.m_nIndex << endl;
  os << indent() << "name        = " << argdef.m_strName << endl;
  os << indent() << "type        = "
    << argdef.m_eType << "("
    << argdef.lookupArgSymbol(argdef.m_eType) << ")" << endl;
  os << indent() << "flags       = 0x"
    << std::hex << argdef.m_uFlags << std::dec
    << "(" << argdef.lookupFlagNames(argdef.m_uFlags) << ")" << endl;
  os << indent() << "literals[" << argdef.m_literals.size() << "] = {"
    << argdef.constructLiteralList(", ")  << "}" << endl;
  os << indent() << "ranges[" << argdef.m_ranges.size() << "]   = ("
    << argdef.constructRangeList(", ")  << ")" << endl;
  os << indent() << "regex       = " << argdef.m_re << endl;

  os << deltaindent(-2);

  os << indent() << "}";

  return os;
}

