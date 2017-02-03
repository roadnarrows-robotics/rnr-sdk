////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_appkit
//
// File:      CommandLine.cxx
//
/*! \file
 *
 * \brief Command-Line parser class implementation
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 * (C) 2016-2017.  RoadNarrows LLC.
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
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
#include <vector>
#include <map>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/appkit/LogBook.h"
#include "rnr/appkit/ReadLine.h"
#include "rnr/appkit/CommandLine.h"

using namespace std;
using namespace rnr;
using namespace rnr::cmd;

// -----------------------------------------------------------------------------
// Private Implementation
// -----------------------------------------------------------------------------

/*!
 * \brief Debugging macros.
 */
#undef CL_ENABLE_DEBUG  ///< define to enable

#ifdef CL_ENABLE_DEBUG
static int dbg_calldepth_ = -1;

#define CL_CALL_DEPTH        dbg_calldepth_
#define CL_SET_CALL_DEPTH(n) dbg_calldepth_ = n
#define CL_PUSH_CALL_DEPTH() ++dbg_calldepth_
#define CL_POP_CALL_DEPTH()  --dbg_calldepth_

#define CL_DBG_CALL_IN(args, post) \
do \
{ \
  CL_PUSH_CALL_DEPTH(); \
  cerr << indent(dbg_calldepth_*2) << __func__ << "(" << args << ")" << post; \
} while(0)

#define CL_DBG_CALL_OUT1(res) \
do \
{ \
  cerr << indent(dbg_calldepth_*2) << "--> (" << res << ")" << endl; \
  CL_POP_CALL_DEPTH(); \
} while(0)

#define CL_DBG_CALL_OUT2(res) \
do \
{ \
  cerr << "--> (" << res << ")" << endl; \
  CL_POP_CALL_DEPTH(); \
} while(0)

#define CL_DBG(os) cerr << os

#else
#define CL_CALL_DEPTH
#define CL_SET_CALL_DEPTH(n)
#define CL_PUSH_CALL_DEPTH()
#define CL_POP_CALL_DEPTH()
#define CL_DBG_CALL_IN(args, post)
#define CL_DBG_CALL_OUT1(res)
#define CL_DBG_CALL_OUT2(res)
#define CL_DBG(os)
#endif // CL_ENABLE_DEBUG

namespace rnr
{
  namespace cmd
  {
    /*!
     * \brief Name - Value Pair entry structure
     */
    struct NameValuePair
    {
      const char* m_sName;    ///< name
      int         m_nValue;   ///< value
    };

    /*!
     * \brief Argument type - string symbol lookup table.
     */
    static NameValuePair ArgTypeLookupTbl[] =
    {
      {ArgLiteral,      CmdArgDef::TypeLiteral},
      {ArgWord,         CmdArgDef::TypeWord},
      {ArgInteger,      CmdArgDef::TypeInteger},
      {ArgFloat,        CmdArgDef::TypeFloat},
      {ArgQuotedString, CmdArgDef::TypeQuotedString},
      {ArgFile,         CmdArgDef::TypeFile},
      {ArgRegEx,        CmdArgDef::TypeRegEx},
      {NULL,            CmdArgDef::TypeUndef}
    };

    /*!
     * \brief Look up argument type, given argument type symbol.
     *
     * \param strSymbol Type Symbol.
     *
     * \return Type enum.
     */
    static CmdArgDef::ArgType lookupArgType(const string strSymbol)
    {
      for(int i = 0; ArgTypeLookupTbl[i].m_sName != NULL; ++i)
      {
        if( strSymbol == ArgTypeLookupTbl[i].m_sName )
        {
          return (CmdArgDef::ArgType)ArgTypeLookupTbl[i].m_nValue; 
        }
      }
      return CmdArgDef::TypeUndef;
    }

    /*!
     * \brief Look up argument symbol, given argument type.
     *
     * \param eType Type enum.
     *
     * \return Type symbol string.
     */
    static const string lookupArgSymbol(const CmdArgDef::ArgType eType)
    {
      for(int i = 0; ArgTypeLookupTbl[i].m_sName != NULL; ++i)
      {
        if( eType == ArgTypeLookupTbl[i].m_nValue ) 
        {
          return ArgTypeLookupTbl[i].m_sName;
        }
      }
      return "undef";
    }

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

    /*!
     * \brief Look up argument flags, given flags value.
     *
     * \param uFlags  Bit or'ed list of flags.
     *
     * \return String.
     */
    static const string lookupFlagNames(const unsigned uFlags)
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

    /*!
     * \brief Command usage syntax special characters.
     *
     * \param c Character to test.
     *
     * \return Returns true if c is special, false otherwise.
     */
    static inline bool isspecial(int c)
    {
      return( (c == '<') || (c == ':') || (c == '>') ||
              (c == '{') || (c == '|') || (c == '}') ||
              (c == '[') || (c == ']') );
    }

    /*!
     * \brief Test if c is a double quote character.
     */
    static inline bool isdquote(int c)
    {
      return(c == '"');
    }

    /*!
     * \brief Testif c is an escape character.
     */
    static inline bool isesc(int c)
    {
      return(c == '\\');
    }

    /*!
     * \brief Convert ascii character to binary character value.
     *
     * \param c ASCII character.
     *
     * \return Binary character.
     */
    static char tohex(int c)
    {
      if( (c >= '0') && (c <= '9') )
      {
        return (char)(c - '0');
      }
      else if( (c >= 'a') && (c <= 'f') )
      {
        return (char)(c - 'a');
      }
      else if( (c >= 'A') && (c <= 'F') )
      {
        return (char)(c - 'A');
      }
      else
      {
        return (char)0;
      }
    }

    /*!
     * \brief Convert boolean to "ok" or "not-ok".
     *
     * \param b   Boolean value.
     *
     * \return String.
     */
    static string okstr(bool b)
    {
      return b? "ok": "not-ok";
    }

    /*!
     * \brief Create indentation string.
     *
     * \param n Number of spaces.
     *
     * \return String.
     */
    static string indent(int n)
    {
      string s;
      while( n-- > 0 )
      {
        s.push_back(' ');
      }
      return s;
    }

    /*!
     * \brief Split string.
     *
     * \param [in] str        String to split.
     * \param [in] delem      Character delimiter.
     * \param [in,out] elems  Vector of split strings.
     */
    void split(const string &str, char delim, StringVec &elems)
    {
      stringstream  ss;
      string        item;

      ss.str(str);

      while(std::getline(ss, item, delim) )
      {
        elems.push_back(item);
      }
    }

    /*!
     * \brief Split string.
     *
     * \param [in] str        String to split.
     * \param [in] delem      Character delimiter.
     *
     * \return Vector of split strings.
     */
    StringVec split(const string &str, char delim)
    {
      std::vector<std::string> elems;

      split(str, delim, elems);

      return elems;
    }

    static const string emptystring;  ///< empty string
    static const string noliteral;    ///< "no literal" literal
    static CmdArgDef    noargdef;     ///< "no arg def" argument definition
    static CmdFormDef   noformdef;    ///< "no form def" form definition
    static CmdDef       nocmddef;     ///< "no cmd def" command definition
    static const string noprompt;     ///< "no prompt" prompt value


    // -------------------------------------------------------------------------
    // Public Convenience Functions
    // -------------------------------------------------------------------------

    int help(const CmdExec cmds[],
             size_t        uNumOfCmds,
             const string  &strCmdName,
             bool          bLongHelp)
    {
      bool    bAllCmds = strCmdName.empty()? true: false;
      int     len;
      int     cnt;
      size_t  i;
    
      for(i = 0, cnt = 0; i < uNumOfCmds; ++i)
      {
        if( bAllCmds || (strCmdName == cmds[i].m_sName) )
        {
          StringVec usages = split(cmds[i].m_sSyntax, '\n');
          if( bLongHelp )
          {
            len = (int)strlen(cmds[i].m_sName) + 2;
            len = 80 - 2 * len;
            printf("%s()%*s%s()\n\n", cmds[i].m_sName,
                len, "", cmds[i].m_sName);
            printf("Name\n  %s - %s\n\n", cmds[i].m_sName, cmds[i].m_sSynopsis);
            printf("Synopsis\n");
            for(size_t j = 0; j < usages.size(); ++j)
            {
              if( !usages[j].empty() )
              {
                printf("  %s\n", usages[j].c_str());
              }
            }
            printf("\n");
            if( cmds[i].m_sDesc != NULL )
            {
              printf("Description\n  %s\n\n", cmds[i].m_sDesc);
            }

            printf("    ---\n\n");
          }
          else
          {
            for(size_t j = 0; j < usages.size(); ++j)
            {
              if( !usages[j].empty() )
              {
                printf("%s\n", usages[j].c_str());
              }
            }
            printf("\n");
          }
    
          ++cnt;
        }

        if( !bAllCmds && (cnt == 1) )
        {
          break;
        }
      }
    
      if( bAllCmds )
      {
        printf("  %d commands\n", cnt);
      }
    
      return cnt;
    }

  } // namespace cmd
} // namespace rnr


// -----------------------------------------------------------------------------
// Token Class
// -----------------------------------------------------------------------------

Token::Token()
{
  m_uLineNum = 0;
  m_uLinePos = 0;
}

Token::Token(const Token &src)
{
  m_strValue = src.m_strValue;
  m_uLineNum = src.m_uLineNum;
  m_uLinePos = src.m_uLinePos;
}

Token::~Token()
{
}

Token &Token::operator=(const Token &rhs)
{
  m_strValue = rhs.m_strValue;
  m_uLineNum = rhs.m_uLineNum;
  m_uLinePos = rhs.m_uLinePos;

  return *this;
}

ostream &Token::oloc(ostream &os, const string &strLine)
{
  size_t  uPrev = os.width();

  os << *this << endl;
  os << strLine << endl;
  if( m_uLinePos > 0 )
  {
    os.width(m_uLinePos - 1);
  }
  os << '^' << endl;
  os.width(uPrev);

  return os;
}

ostream &rnr::cmd::operator<<(ostream &os, const Token &tok)
{
  if( tok.m_uLineNum > 0 )
  {
    os << tok.m_uLineNum << ":";
  }
  os << tok.m_uLinePos << ": "
    << "\"" << CommandLine::prettify(tok.m_strValue) << "\"";

  return os;
}


// -----------------------------------------------------------------------------
// CmdArgDef Class
// -----------------------------------------------------------------------------

CmdArgDef::CmdArgDef()
{
  m_nCmdUid     = CommandLine::NoUid;
  m_nFormIndex  = -1;
  m_nIndex      = -1;
  m_eType       = TypeUndef;
  m_uFlags      = 0;
}

CmdArgDef::CmdArgDef(const CmdArgDef &src)
{
  m_nCmdUid     = src.m_nCmdUid;
  m_nFormIndex  = src.m_nFormIndex;
  m_nIndex      = src.m_nIndex;
  m_strName     = src.m_strName;
  m_eType       = src.m_eType;
  m_values      = src.m_values;
  m_uFlags      = src.m_uFlags;
}

CmdArgDef::~CmdArgDef()
{
  //fprintf(stderr, "rdk: %s: %s\n", __func__, m_strName.c_str());
}

CmdArgDef &CmdArgDef::operator=(const CmdArgDef &rhs)
{
  m_nCmdUid     = rhs.m_nCmdUid;
  m_nFormIndex  = rhs.m_nFormIndex;
  m_nIndex      = rhs.m_nIndex;
  m_strName     = rhs.m_strName;
  m_eType       = rhs.m_eType;
  m_values      = rhs.m_values;
  m_uFlags      = rhs.m_uFlags;

  return *this;
}

bool CmdArgDef::isDefined() const
{
  return (m_nIndex >= 0) && !m_strName.empty() && (m_eType != TypeUndef);
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
  m_values.push_back(strValue);
}

const string &CmdArgDef::literalAt(const int nIndex) const
{
  if( (nIndex >= 0) && (nIndex < m_values.size()) )
  {
    return m_values[nIndex];
  }
  else
  {
    return noliteral;
  }
}

void CmdArgDef::orFlags(const unsigned uFlags)
{
  m_uFlags |= uFlags;
}

bool CmdArgDef::match(const string &strArg, bool bIgnoreCase) const
{
  bool  bHasMatch = false;

  switch( m_eType )
  {
    case CmdArgDef::TypeLiteral:
      for(size_t i = 0; i < numOfLiterals(); ++i)
      {
        if( strArg == literalAt(i) )
        {
          bHasMatch = true;
          break;
        }
      }
      break;

    case CmdArgDef::TypeWord:
      bHasMatch = true;
      break;
      
    case CmdArgDef::TypeQuotedString:
      bHasMatch = true;
      break;

    case CmdArgDef::TypeInteger:
      {
        long  val;
        if( CommandLine::strToLong(strArg, val) == CommandLine::Ok )
        {
          bHasMatch = true;
        }
      }
      break;

    case CmdArgDef::TypeFloat:
      {
        double val;
        if( CommandLine::strToDouble(strArg, val) == CommandLine::Ok )
        {
          bHasMatch = true;
        }
      }
      break;

    case CmdArgDef::TypeRegEx:
      bHasMatch = true;
      break; // TBD

    case CmdArgDef::TypeFile:
      bHasMatch = true;
      break; // TBD

    case CmdArgDef::TypeUndef:
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

  return bHasMatch;
}


ostream &rnr::cmd::operator<<(ostream &os, const CmdArgDef &argdef)
{
  int     nIndent = 14;   // TODO write a indent manip

  string  sp0(indent(nIndent));
  string  sp1(indent(nIndent+2));
  string  sep;

  os << sp0 << "{" << endl;
  os << sp1 << "cmduid    = " << argdef.m_nCmdUid << endl;
  os << sp1 << "formindex = " << argdef.m_nFormIndex << endl;
  os << sp1 << "index     = " << argdef.m_nIndex << endl;
  os << sp1 << "name      = " << argdef.m_strName << endl;
  os << sp1 << "type      = "
    << argdef.m_eType << "(" << lookupArgSymbol(argdef.m_eType) << ")" << endl;
  os << sp1 << "flags     = 0x" << std::hex << argdef.m_uFlags << std::dec
        << "(" << lookupFlagNames(argdef.m_uFlags) << ")" << endl;

  os << sp1 << "values[" << argdef.m_values.size() << "] = {";

  for(size_t i = 0; i < argdef.m_values.size(); ++i)
  {
    os << sep << argdef.m_values[i];
    if( sep.empty() )
    {
      sep = ", ";
    }
  }

  os << "}" << endl;

  os << sp0 << "}";

  return os;
}


// -----------------------------------------------------------------------------
// CmdFormDef Class
// -----------------------------------------------------------------------------

CmdFormDef::CmdFormDef()
{
  m_nCmdUid   = CommandLine::NoUid;
  m_nIndex    = -1;
  m_nArgcReq  = 0;
  m_nArgcOpt  = 0;
}

CmdFormDef::CmdFormDef(const string &strSyntax) :
    m_strSyntax(strSyntax)
{
  m_nCmdUid   = CommandLine::NoUid;
  m_nIndex    = -1;
  m_nArgcReq  = 0;
  m_nArgcOpt  = 0;
}

CmdFormDef::CmdFormDef(const CmdFormDef &src)
{
  m_nCmdUid   = src.m_nCmdUid;
  m_nIndex    = src.m_nIndex;
  m_strSyntax = src.m_strSyntax;
  m_argdefs   = src.m_argdefs;
  m_nArgcReq  = src.m_nArgcReq;
  m_nArgcOpt  = src.m_nArgcOpt;
}

CmdFormDef::~CmdFormDef()
{
  //fprintf(stderr, "rdk: %s: %d\n", __func__, m_nIndex);
}

CmdFormDef &CmdFormDef::operator=(const CmdFormDef &rhs)
{
  m_nCmdUid   = rhs.m_nCmdUid;
  m_nIndex    = rhs.m_nIndex;
  m_strSyntax = rhs.m_strSyntax;
  m_argdefs   = rhs.m_argdefs;
  m_nArgcReq  = rhs.m_nArgcReq;
  m_nArgcOpt  = rhs.m_nArgcOpt;
}

bool CmdFormDef::isDefined() const
{
  return  (m_nIndex >= 0)           &&
          !m_strSyntax.empty()      &&
          (m_argdefs.size() > 0)    &&
          m_argdefs[0].isDefined();
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
  m_argdefs.push_back(argdef);
}

const CmdArgDef &CmdFormDef::argAt(const int nIndex) const
{
  if( (nIndex >= 0) && (nIndex < m_argdefs.size()) )
  {
    return m_argdefs[nIndex];
  }
  else
  {
    return noargdef;
  }
}

CmdArgDef &CmdFormDef::argAt(const int nIndex)
{
  if( (nIndex >= 0) && (nIndex < m_argdefs.size()) )
  {
    return m_argdefs[nIndex];
  }
  else
  {
    return noargdef;
  }
}

CmdArgDef &CmdFormDef::lastArg()
{
  return m_argdefs.empty()? noargdef: m_argdefs.back();
}

ostream &rnr::cmd::operator<<(ostream &os, const CmdFormDef &formdef)
{
  int     nIndent = 8;    // TODO write a indent manip

  string  sp0(indent(nIndent));
  string  sp1(indent(nIndent+2));
  string  sep;

  os << sp0 << "{" << endl;
  os << sp1 << "cmduid  = " << formdef.m_nCmdUid << endl;
  os << sp1 << "index   = " << formdef.m_nIndex << endl;
  os << sp1 << "syntax  = " << formdef.m_strSyntax << endl;
  os << sp1 << "argc    = " << formdef.numOfArgs() << "(total)"
        << " " << formdef.numOfRequiredArgs() << "(required)"
        << " " << formdef.numOfOptionalArgs() << "(optional)"
        << endl;

  os << sp1 << "args[" << formdef.numOfArgs() << "] =" << endl;
  os << sp1 << "{" << endl;

  for(size_t i = 0; i < formdef.m_argdefs.size(); ++i)
  {
    os << sep << formdef.m_argdefs[i];
    if( sep.empty() )
    {
      sep = ",\n";
    }
  }
  os << endl;

  os << sp1 << "}" << endl;

  os << sp0 << "}";

  return os;
}

// -----------------------------------------------------------------------------
// CmdDef Class
// -----------------------------------------------------------------------------

CmdDef::CmdDef()
{
  m_nUid      = CommandLine::NoUid;
}

CmdDef::CmdDef(const CmdDef &src)
{
  m_nUid      = src.m_nUid;
  m_strName   = src.m_strName;
  m_formdefs  = src.m_formdefs;
}

CmdDef::~CmdDef()
{
  //fprintf(stderr, "rdk: ***  %s: %s %d\n", __func__, m_strName.c_str(), m_nUid);
}

CmdDef &CmdDef::operator=(const CmdDef &rhs)
{
  m_nUid      = rhs.m_nUid;
  m_strName   = rhs.m_strName;
  m_formdefs  = rhs.m_formdefs;
}

bool CmdDef::isDefined() const
{
  return  (m_nUid >= 0)             &&
          !m_strName.empty()        &&
          (numOfForms() > 0)        &&
          m_formdefs[0].isDefined();
}

void CmdDef::setUid(const int uid)
{
  m_nUid = uid;
}

void CmdDef::setName(const string &strName)
{
  m_strName = strName;
}

void CmdDef::pushForm(CmdFormDef &formdef)
{
  int nIndex = numOfForms();

  formdef.setParent(m_nUid);
  formdef.setIndex(nIndex);
  m_formdefs.push_back(formdef);
}

const CmdFormDef &CmdDef::formAt(const int nIndex) const
{
  if( (nIndex >= 0) && (nIndex < m_formdefs.size()) )
  {
    return m_formdefs[nIndex];
  }
  else
  {
    return noformdef;
  }
}

CmdFormDef &CmdDef::formAt(const int nIndex)
{
  if( (nIndex >= 0) && (nIndex < m_formdefs.size()) )
  {
    return m_formdefs[nIndex];
  }
  else
  {
    return noformdef;
  }
}

ostream &rnr::cmd::operator<<(ostream &os, const CmdDef &cmddef)
{
  int     nIndent = 4;    // TODO write a indent manip

  string  sp0(indent(nIndent));
  string  sp1(indent(nIndent+2));
  string  sep;

  os << sp0 << "{" << endl;

  os << sp1 << "uid      = " << cmddef.m_nUid << endl;
  os << sp1 << "name     = " << cmddef.m_strName << endl;
  os << sp1 << "forms[" << cmddef.numOfForms() << "] = " << endl;
  os << sp1 << "{" << endl;

  for(size_t i = 0; i < cmddef.m_formdefs.size(); ++i)
  {
    os << sep << cmddef.m_formdefs[i];

    if( sep.empty() )
    {
      sep = ",\n";
    }
  }
  os << endl;

  os << sp1 << "}" << endl;

  os << sp0 << "}";

  return os;
}


// -----------------------------------------------------------------------------
// CommandLine Class
// -----------------------------------------------------------------------------

//
// Log bookmarks.
//
static const string markExec("ExecMark");

//
// Log common prefix strings.
//
static const string    preExec("Execute: ");
static const string preCompile("Compile: ");
static const string   preParse("Parse:   ");
static const string   preInput("Input:   ");
static const string  preSelect("Select:  ");
static const string     preTry("Try:     ");
static const string     preFit("Fitness: ");
static const string preNoMatch("NoMatch: ");
static const string   preMatch("Match:   ");
static const string    preFail("Failure: ");

static const string preBug("Bug: ");

CommandLine::CommandLine(const string strName,
                         const string strPrompt,
                         bool         bUseRlLib,
                         bool         bIgnoreCase) :
    m_strName(strName),
    m_bIgnoreCase(bIgnoreCase),
    m_readline(strName, strPrompt, bUseRlLib),
    m_log("Command Line Log", 25)
{
  m_nUidCnt     = 0;
  m_bIsCompiled = false;

  pushPrompt(strPrompt);
}

CommandLine::~CommandLine()
{
}

bool CommandLine::isDefined() const
{
  return m_bIsCompiled;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Command Addition and Compile Methods
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int CommandLine::addCommand(const string strMultiFormSyntax)
{
  StringVec   syntaxForms;
  string      strName;
  int         uid;

  split(strMultiFormSyntax, '\n', syntaxForms);

  if( syntaxForms.size() == 0 || syntaxForms[0].empty() )
  {
    m_log << "No syntax forms specified." << eoe;
    LOGERROR("%s", getErrorStr().c_str());
    return NoUid;
  }

  // extract command name from the syntax
  strName = extractArgv0(syntaxForms[0]);

  if( strName.empty() )
  {
    m_log << "No valid command name specified: " << syntaxForms[0] << eoe;
    LOGERROR("%s", getErrorStr().c_str());
    return NoUid;
  }

  // find any existing command definition - may return nocmddef
  CmdDef &def = cmdAt(strName);

  //
  // New form of an existing command.
  //
  if( def.getName() == strName )
  {
    uid = def.getUid();
  }

  //
  // New command.
  //
  else
  {
    CmdDef newdef;

    // set command's unique id
    newdef.setUid(m_nUidCnt++);

    uid = newdef.getUid();

    // set command's name
    newdef.setName(strName);

    // add to map of commands 
    m_cmddefs[uid] = newdef;
  }

  CmdDef &cmddef = cmdAt(uid);

  //
  // Add all syntax forms.
  //
  for(size_t i = 0; i < syntaxForms.size(); ++i)
  {
    if( strName != extractArgv0(syntaxForms[i]) )
    {
      m_log << "Command name mismatch: Expected " << strName
            << ": " << syntaxForms[i] << eoe;
      LOGERROR("%s", getErrorStr().c_str());
      return NoUid;
    }

    CmdFormDef  formdef(syntaxForms[i]);

    // add command form
    cmddef.pushForm(formdef);
  }

  return uid;
}

int CommandLine::compile()
{
  CmdIter   iter;
  int       rc = Ok;

  CL_DBG_CALL_IN("", endl);

  if( m_log.hasBookMark(markExec) )
  {
    m_log.eraseToMark(markExec, LogBook::NEWEST);
  }
  else
  {
    m_log.clear();
  }

  m_log << bookmark(markExec) << preExec << __func__ << eoe;

  // no commands added
  if( m_nUidCnt == 0 )
  {
    m_log << "No commands added." << eoe;
    LOGERROR("%s", getErrorStr().c_str());
    rc = RC_ERROR;
  }

  // compile each command
  for(iter = m_cmddefs.begin(); (rc == Ok) && (iter != m_cmddefs.end()); ++iter)
  {
    CmdDef &cmddef = iter->second;

    rc = compile(cmddef);
  }

  // build readline generator
  if( rc == Ok )
  {
    rc = rlBuildReadLineGenerator();
  }

  // finalize
  if( rc == Ok )
  {
    // register readline generator callback
    m_readline.registerAltGenerator(rlGeneratorWrapper, this);

    m_bIsCompiled = true;

    m_log << "Ok" << eoe;
  }

  else
  {
    m_log << preFail << "Could not compile." << eoe;
  }

  CL_DBG_CALL_OUT1("rc=" << rc);

  return rc;
}

int CommandLine::compile(CmdDef &cmddef)
{
  StringVec   tokens;
  ssize_t     tokcnt;
  int         i;
  int         rc;

  CL_DBG_CALL_IN("cmddef.uid=" << cmddef.getUid(), endl);

  m_log << preCompile << "cmddef " << cmddef.getUid() << eoe;

  for(i = 0, rc = Ok; (i < cmddef.numOfForms()) && (rc == Ok); ++i)
  {
    CmdFormDef &form = cmddef.formAt(i);

    if( (tokcnt = tokenizeSyntax(form.getSyntax(), tokens)) < 0 )
    {
      rc = tokcnt;
    }

    else if( tokcnt == 0 )
    {
      m_log
        << preFail
        << "cmddef " << cmddef.m_nUid << ", "
        << "form " << form.getIndex() << ": No syntax specified." << eoe;
      LOGERROR("%s", getErrorStr().c_str());
      rc = EBadSyntax;
    }

    else
    {
      rc = parseSyntax(cmddef, form, tokens);
    }
  }

  CL_DBG_CALL_OUT1("rc=" << rc);
  CL_DBG(endl);

  return rc;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Command Line Interface Methods
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int CommandLine::readCommand(FILE *fp, int &uid, StringVec &argv)
{
  string  strLine;
  int     rc;

  if( m_log.hasBookMark(markExec) )
  {
    m_log.eraseToMark(markExec, LogBook::NEWEST);
  }
  else
  {
    m_log.clear();
  }

  m_log << bookmark(markExec) << preExec << __func__ << eoe;

  // read a line of input
  if( fileno(fp) == fileno(stdin) )
  {
    strLine = m_readline.rlreadLine();
  }
  else
  {
    strLine = m_readline.ireadLine(fp);
  }

  m_log << preInput << strLine << eoe;

  rc = processInput(strLine, uid, argv);

  if( rc == Ok )
  {
    m_log << "Ok" << eoe;
  }
  else
  {
    m_log << preFail << "Input not matched to any command." << eoe;
  }

  return rc;
}

void CommandLine::addToHistory(const StringVec &argv)
{
  m_readline.addToHistory(c14n(argv));
}

void CommandLine::pushPrompt(const string &strPrompt)
{
  m_prompts.push_back(strPrompt);
  m_readline.setPrompt(strPrompt);
}

void CommandLine::popPrompt()
{
  if( !m_prompts.empty() )
  {
    m_prompts.pop_back();
    m_readline.setPrompt(getPrompt());
  }
  else
  {
    m_readline.setPrompt(noprompt);
  }
}

const string &CommandLine::getPrompt() const
{
  if( !m_prompts.empty() )
  {
    return m_prompts.back();
  }
  else
  {
    return noprompt;
  }
}

int CommandLine::processInput(const string &strLine, int &uid, StringVec &argv)
{
  int     argc;
  int     rc;

  uid = NoUid;
  argv.clear();

  //
  // The line is empty - find out why?
  //
  if( strLine.empty() )
  {
    rc = checkReadResult();
  }

  //
  // Tokenize the input line.
  //
  else if( (argc = (int)tokenize(strLine, argv)) <= 0 )
  {
    // bad token - error string already set
    if( argc < 0 )
    {
      rc = EBadSyntax;
    }

    // blank line
    else
    {
      rc = checkReadResult();
    }
  }

  //
  // Otherwise match and validate the input arguments to the best form of a 
  // command definition.
  //
  else if( (rc = matchCommand(uid, argv)) == Ok )
  {
    // convert token vec to string vec
  }

  return rc;
}

int CommandLine::matchCommand(int &uid, const StringVec &argv)
{
  CmdIter         iter;
  vector<CmdIter> cmdCandidates;
  int             rc;

  uid  = NoUid;

  // invalid to call this function with no arguments - no match is possible
  if( argv.size() == 0 )
  {
    m_log << preBug << "No arguments to match." << eoe;
    LOGERROR("%s", getErrorStr().c_str());
    return EError;
  }

  // build candidates from command name (argv0)
  for(iter = m_cmddefs.begin(); iter != m_cmddefs.end(); ++iter)
  {
    // TODO: support ignore case
    if( iter->second.getName() == argv[0] )
    {
      cmdCandidates.push_back(iter);
    }
  }

  // no match
  if( cmdCandidates.size() == 0 )
  {
    m_log << preNoMatch << argv[0] << ": Command not found." << eoe;
    rc = EUnknownCmd;
  }

  // duplicates
  else if( cmdCandidates.size() > 1 )
  {
    m_log
      << preNoMatch
      << cmdCandidates.size() 
      << " duplicate commands definitions found for "
      << cmdCandidates.back()->second.getName()
      << eoe;
    rc = EAmbigCmd;
  }

  // got a single candiate - match on compiled syntx
  else
  {
    m_log
      << preSelect
      << cmdCandidates[0]->second.getName()
      << " command"
      << eoe;

    uid = cmdCandidates[0]->first;
    rc = matchCommand(cmdCandidates[0]->second, argv);
  }

  return rc;
}

int CommandLine::matchCommand(const CmdDef &cmddef, const StringVec &argv)
{
  double  fFitness, fMaxFitness;
  int     iBest, i;
  int     rc;

  fMaxFitness = 0.0;
  iBest       = -1;

  for(i = 0; i < cmddef.numOfForms(); ++i)
  {
    const CmdFormDef &form = cmddef.formAt(i);

    m_log << preTry << form.getSyntax() << eoe;

    if( (rc = matchCommandForm(cmddef, form, argv, fFitness)) == Ok )
    {
      if( fFitness > fMaxFitness )
      {
        fMaxFitness = fFitness;
        iBest       = i;
      }
    }

    m_log << preFit << fFitness << eoe;

    if( fMaxFitness >= 1.0 )
    {
      break;
    }
  }

  if( iBest >= 0 )
  {
    m_log << preMatch << cmddef.getName() << ", form " << iBest << "." << eoe;
    LOGDIAG2("Command %d,form %d match: Fitness %lf: %s", 
        cmddef.getUid(), iBest, fMaxFitness, c14n(argv).c_str());
    rc = Ok;
  }
  else if( rc == Ok )
  {
    m_log << preNoMatch << c14n(argv) << eoe;
    rc = EUnknownCmd;
  }

  return rc;
}


int CommandLine::matchCommandForm(const CmdDef     &cmddef,
                                  const CmdFormDef &form,
                                  const StringVec  &argv,
                                  double           &fFitness)
{
  int     in_argc = (int)argv.size();
  int     form_argc = form.numOfArgs();
  double  fWeight, fDecay;
  int     i, j;
  int     rc;

  fDecay   = 1.0;
  fFitness = 0.0;

  if( in_argc > form_argc )
  {
    m_log
      << preNoMatch
      << "command " << cmddef.getName() << ", "
      << "form " << form.getIndex() << ": "
      << "Too many arguments: "
      << in_argc << " specified, "
      << form_argc << " maximum."
      << eoe;
    return EBadSyntax;
  }

  else if( (in_argc - 1) < form.numOfRequiredArgs() )
  {
    m_log
      << preNoMatch
      << "command " << cmddef.getName() << ", "
      << "form " << form.getIndex() << ": "
      << "Missing required arguments: "
      << in_argc << " specified, "
      << form.numOfRequiredArgs()+1 << " required."
      << eoe;
    return EBadSyntax;
  }

  for(i = 0, j = 0, rc = Ok;
      (i < form_argc) && (j < in_argc) && (rc == Ok);
      ++i)
  {
    const CmdArgDef &argdef = form.argAt(i);

    if( argdef.match(argv[j], m_bIgnoreCase) )
    {
      rc = Ok;
    }
    else
    {
      m_log 
        << preNoMatch
        << "Token \"" << argv[j] << "\" at "
        << "command " << cmddef.getName() << ", "
        << "form " << argdef.getParentFormIndex() << ", "
        << "arg " << argdef.getName() << ": ";
      rc = EBadSyntax;
    }
    
    switch( argdef.getType() )
    {
      case CmdArgDef::TypeLiteral:
        fWeight = 1.0;
        if( rc != Ok )
        {
          m_log << "Not one of:";
          for(size_t iLit = 0; iLit < argdef.numOfLiterals(); ++iLit)
          {
            m_log << " " << argdef.literalAt(iLit);
          }
          m_log << ".";
        }
        break;
      case CmdArgDef::TypeWord:
      case CmdArgDef::TypeQuotedString:
        fWeight = 0.90;
        break; // nothing to match
      case CmdArgDef::TypeInteger:
        fWeight = 0.95;
        if( rc != Ok )
        {
          m_log << "Not an integer.";
        }
        break;
      case CmdArgDef::TypeFloat:
        fWeight = 0.95;
        if( rc != Ok )
        {
          m_log << "Not a float.";
        }
        break;
      case CmdArgDef::TypeRegEx:
        fWeight = 0.92;
        break; // TBD
      case CmdArgDef::TypeFile:
        fWeight = 0.90;
        break; // TBD
      case CmdArgDef::TypeUndef:
      default:
        m_log << argdef.m_eType << ": Bug: Unknown type.";
        rc = EError;
        break;
    }

    // good match
    if( rc == Ok )
    {
      if( fWeight == 1.0 )
      {
        fFitness += fWeight;
      }
      else
      {
        fFitness += fWeight * fDecay;
      }
      fDecay *= 0.9;
      ++j;
    }

    // bad match, but argument is optional, are there are more optionals.
    else if((argdef.getFlags() & CmdArgDef::FlagOptional) && (i < form_argc-1))
    {
      m_log << eoe; // record pending log entry
      rc = Ok;
    }

    // bad match
    else
    {
      m_log << eoe; // record pending log entry
      fFitness = 0.0;
    }
  }

  fFitness /= (double)in_argc;

  return rc;
}

int CommandLine::checkReadResult()
{
  // EOF
  if( m_readline.isEoF() )
  {
    m_log << "EOF" << eoe;
    return EEoF;
  }

  // file read error
  else if( m_readline.isFError() )
  {
    m_log << m_readline.getErrorStr() << eoe;
    return ERead;
  }

  // read successful
  else
  {
    return Ok;
  }
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// ReadLine Generator Methods
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int CommandLine::rlBuildReadLineGenerator()
{
  return Ok;
}

const string CommandLine::rlGeneratorWrapper(const string &strText,
                                             int          nIndex,
                                             const string &strContext,
                                             int          nStart,
                                             int          nEnd,
                                             unsigned     &uFlags,
                                             void         *pAppArg)
{
  if( pAppArg != NULL )
  {
    return ((CommandLine *)pAppArg)->rlGenerator(strText, nIndex, strContext,
                                                 nStart, nEnd, uFlags);
  }
  else
  {
    return emptystring;
  }
}

const string CommandLine::rlGenerator(const string &strText,
                                      int          nIndex,
                                      const string &strContext,
                                      int          nStart,
                                      int          nEnd,
                                      unsigned     &uFlags)
{
  vector<CmdArgDef*>  argdefs;
  StringVec           tabList;
  size_t              i;

  cerr  << "{text=" << strText
        << ", index=" << nIndex
        << ", context=" << strContext
        << ", start=" << nStart
        << ", end=" << nEnd
        << "}" << endl;

  rlArgDefs(strContext.substr(0, nStart), argdefs);

  for(i = 0; i < argdefs.size(); ++i)
  {
    cerr << *argdefs[i] << endl;
  }

  rlTabList(strText, argdefs, tabList, uFlags);

  return RlTabEnd;
}

void CommandLine::rlArgDefs(const string       &strSubtext,
                            vector<CmdArgDef*> &argdefs)
{
  vector<CmdArgDef*>  v[2];
  int                 tog0, tog1;
  StringVec           tokens;
  size_t              argc;
  size_t              i, j;

  argdefs.clear();

  tokenize(strSubtext, tokens);
    
  tog0 = 0, tog1 = 1;

  //
  // Seed with argv0 (command).
  //
  for(i = 0; i < numOfCmds(); ++i)
  {
    for(j = 0; j < m_cmddefs[i].numOfForms(); ++j)
    {
      CmdFormDef &form = m_cmddefs[i].formAt(j);
      v[tog0].push_back(&form.argAt(0));
    }
  }

  //
  // Walk across token arguments, paring unmatched argument defs from list.
  //
  for(argc = 0; argc < tokens.size(); ++argc)
  {
    v[tog1].clear();

    for(i = 0; i < v[tog0].size(); ++i)
    {
      CmdArgDef *p = v[tog0][i];

      if( p->match(tokens[argc], m_bIgnoreCase) )
      {
        CmdFormDef &form =
                m_cmddefs[p->getParentCmdUid()].formAt(p->getParentFormIndex());

        if( argc+1 < form.numOfArgs() )
        {
          v[tog1].push_back(&form.argAt(argc+1));
        }
      }
    }

    tog0 = tog1;
    tog1 = (tog1 + 1) % 2;
  }

  // copy to interface parameter
  for(i = 0; i < v[tog0].size(); ++i)
  {
    argdefs.push_back(v[tog0][i]);
  }
}

void CommandLine::rlTabList(const string       &strText,
                            vector<CmdArgDef*> &argdefs,
                            StringVec          &tabList,
                            unsigned           &uFlags)
{
  size_t        len;
  string        strLit;
  stringstream  ss;
  size_t        i, j;

  tabList.clear();

  uFlags = ReadLine::FlagTabNoDefault | ReadLine::FlagTabNoFilename;

  if( argdefs.size() == 0 )
  {
    return;
  }

  len = strText.size();

  for(i = 0; i < argdefs.size(); ++i)
  {
    switch( argdefs[i]->getType() )
    {
      case CmdArgDef::TypeLiteral:
        for(j = 0; j < argdefs[i]->numOfLiterals(); ++j)
        {
          strLit = argdefs[i]->literalAt(j);

          // TODO - case
          if( strText == strLit.substr(0, len) )
          {
            tabList.push_back(strLit);
          }
        }
        break;

      case CmdArgDef::TypeWord:
        if( len == 0 )
        {
        }
        break;
      
      case CmdArgDef::TypeQuotedString:
        if( len == 0 )
        {
        }
        break;

      case CmdArgDef::TypeInteger:
        if( len == 0 )
        {
          ss << "<" << argdefs[i]->getName() << ":"
           << lookupArgSymbol(argdefs[i]->getType()) << ">";
          tabList.push_back(ss.str());
        }
        break;

      case CmdArgDef::TypeFloat:
        if( len == 0 )
        {
        }
        break;

      case CmdArgDef::TypeRegEx:
        if( len == 0 )
        {
        }
        break;

      case CmdArgDef::TypeFile:
        if( len == 0 )
        {
        }
        uFlags &= ~ReadLine::FlagTabNoFilename;
        break;

      case CmdArgDef::TypeUndef:
      default:
        break;
    }
  } 
}

bool CommandLine::rlEq(const string &strCandidate, const string &strText)
{
  size_t n = strText.size();

  // TODO: support ignore case
  if( strCandidate.substr(0, n) == strText )
  {
    return true;
  }
  else
  {
    return false;
  }
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Attribute and Data Access Methods
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

const CmdDef &CommandLine::cmdAt(const int uid) const
{
  CmdConstIter  pos;

  if( (pos = m_cmddefs.find(uid)) != m_cmddefs.end() )
  {
    return pos->second;
  }
  else
  {
    return nocmddef;
  }
}

CmdDef &CommandLine::cmdAt(const int uid)
{
  CmdIter  pos;

  if( (pos = m_cmddefs.find(uid)) != m_cmddefs.end() )
  {
    return pos->second;
  }
  else
  {
    return nocmddef;
  }
}

const CmdDef &CommandLine::cmdAt(const string &strName) const
{
  CmdConstIter iter;

  for(iter = m_cmddefs.begin(); iter != m_cmddefs.end(); ++iter)
  {
    if( iter->second.getName() == strName )
    {
      return iter->second;
    }
  }

  return nocmddef;
}

CmdDef &CommandLine::cmdAt(const string &strName)
{
  CmdIter iter;

  for(iter = m_cmddefs.begin(); iter != m_cmddefs.end(); ++iter)
  {
    if( iter->second.getName() == strName )
    {
      return iter->second;
    }
  }

  return nocmddef;
}

const string &CommandLine::getErrorStr() const
{
  return m_log.lastText();
}

ostream &CommandLine::backtrace(ostream &os, bool bAll) const
{
  if( bAll )
  {
    return m_log.printLog(os);
  }
  else
  {
    return m_log.printToMark(os, markExec, LogBook::NEWEST);
  }
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Lexical Analyzer Methods
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

ssize_t CommandLine::tokenize(const string &strInput, StringVec &tokens)
{
  size_t    i, len;
  string    tok;

  CL_DBG_CALL_IN("\"" << strInput << "\"", endl);

  tokens.clear();

  len = strInput.length();

  for(i = 0; i < len; )
  {
    // find start of the next token
    while( (i < len) && isspace((int)strInput[i]) )
    {
      ++i;
    }

    // no more tokens
    if( i >= len ) 
    {
      break;
    }

    // new qouted string token
    if( isdquote(strInput[i]) )
    {
      i += lexQuotedString(strInput.substr(i), tokens);
    }

    // new contiguous word token
    else
    {
      i += lexWord(strInput.substr(i), tokens);
    }
  }

#ifdef CL_ENABLE_DEBUG
  {
    stringstream ss;
    string       sep;
    for(size_t i = 0; i < tokens.size(); ++i)
    {
      ss << sep << prettify(tokens[i]);
      sep = ", ";
    }
    CL_DBG_CALL_OUT1(ss.str());
  }
#endif // CL_ENABLE_DEBUG

  return (ssize_t)tokens.size();
}

string CommandLine::extractArgv0(const string &strSyntax)
{
  StringVec     tokens;

  tokenize(strSyntax, tokens);

  if( (tokens.size() > 0) && tokIdentifier(tokens[0]) )
  {
    return tokens[0];
  }

  else
  {
    return "";
  }
}

ssize_t CommandLine::tokenizeSyntax(const string &strSyntax, StringVec &tokens)
{
  size_t    i, len;
  string    tok;

  CL_DBG_CALL_IN("\"" << strSyntax << "\", tokens", endl);

  tokens.clear();

  len = strSyntax.length();

  for(i = 0; i < len; )
  {
    // find start of the next token
    while( (i < len) && isspace((int)strSyntax[i]) )
    {
      ++i;
    }

    // no more tokens
    if( i >= len ) 
    {
      break;
    }

    //  special character token
    if( isspecial(strSyntax[i]) )
    {
      tok = strSyntax[i++];
      pushToken(tok, tokens);
    }

    // new contiguous word token
    else
    {
      i += lexSyntaxWord(strSyntax.substr(i), tokens);
    }
  }

#ifdef CL_ENABLE_DEBUG
  {
    stringstream ss;
    string       sep;
    for(size_t i = 0; i < tokens.size(); ++i)
    {
      ss << sep << prettify(tokens[i]);
      sep = ", ";
    }
    CL_DBG_CALL_OUT1(ss.str());
  }
#endif // CL_ENABLE_DEBUG

  return (ssize_t)tokens.size();
}

size_t CommandLine::lexSyntaxWord(const string &str, StringVec &tokens)
{
  size_t    i, len;
  string    tok;

  len = str.length();

  while( (i < len) && !isspace((int)str[i]) && !isspecial(str[i]) )
  {
    tok.push_back(str[i++]);
  }

  pushToken(tok, tokens);

  return i;
}

size_t CommandLine::lexWord(const string &str, StringVec &tokens)
{
  size_t    i, len;
  string    tok;

  len = str.length();

  while( (i < len) && !isspace((int)str[i]) )
  {
    tok.push_back(str[i++]);
  }

  pushToken(tok, tokens);

  return i;
}

size_t CommandLine::lexQuotedString(const string &str, StringVec &tokens)
{
  size_t    i, len;
  bool      escseq;
  bool      eos;
  char      c;
  string    tok;

  len = str.length();

  if( isdquote(str[i]) )
  {
    tok.push_back(str[i++]);
    pushToken(tok, tokens);
  }
  else
  {
    return 0;
  }

  escseq  = false;
  eos     = false;

  while( (i < len) && !eos )
  {
    if( escseq )
    {
      switch( str[i] )
      {
        case 't':   // tab
          c = '\t';
          break;
        case 'n':   // newline
          c = '\n';
          break;
        case 'r':   // carriage return
          c = '\r';
          break;
        case 'v':   // vertical tab
          c = '\v';
          break;
        case 'f':   // formfeed
          c = '\f';
          break;
        case 'x':   // h[h]
          {
            int j = i + 1;
            if( (j < len) && !isdquote(str[j]) )
            {
              c = tohex(str[j]);
              ++i;
              ++j;
              if( (j < len) && !isdquote(str[j]) )
              {
                c <<= 4;
                c |= tohex(str[j]);
                ++i;
              }
            }
            else
            {
              c = 'x';
            }
          }
          break;
        default:
          c = str[i];
          break;
      }

      tok.push_back(c);
      escseq = false;
      ++i;
    }

    else
    {
      switch( str[i] )
      {
        case '\\':
          escseq = true;
          break;
        case '"':
          pushToken(tok, tokens);
          eos = true;
          break;
        default:
          tok.push_back(str[i++]);
          break;
      }
    }
  }

  if( (i < len) && isdquote(str[i]) )
  {
    tok.push_back(str[i++]);
    pushToken(tok, tokens);
  }

  return i;
}

void CommandLine::pushToken(string &tok, StringVec &tokens)
{
  tokens.push_back(tok);
  tok.clear();
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Extended Usage Syntax Parsing Methods
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int CommandLine::parseSyntax(CmdDef          &cmddef,
                             CmdFormDef      &form,
                             const StringVec &tokens)
{
  size_t  tokcnt  = tokens.size();
  size_t  pos     = 0;
  bool    bOk;
  int     rc;

  CL_DBG_CALL_IN("cmddef(uid=" << cmddef.getUid() << ")"
      << ", form(index=" << form.getIndex() << ")"
      << ", tokens", endl);

  m_log
    << preParse
    << "cmddef " << cmddef.getUid() << ", "
    << "form " << form.getIndex() << ": "
    << form.getSyntax()
    << eoe;

  // command
  if( !parseArgv0(cmddef, form, tokens, pos) )
  {
    m_log << preFail << "Parssing special argv0." << eoe;
    LOGERROR("%s", getErrorStr().c_str());
    bOk = false;
  }

  // required argmuents - may be none
  else if( !parseRequiredArgList(cmddef, form, tokens, pos) )
  {
    m_log << preFail << "Parsing required argument list." << eoe;
    LOGERROR("%s", getErrorStr().c_str());
    bOk = false;
  }

  // optional arguments - may be none
  else if( !parseOptionalArgList(cmddef, form, tokens, pos) )
  {
    m_log << preFail << "Parsing optional argument list." << eoe;
    LOGERROR("%s", getErrorStr().c_str());
    bOk = false;
  }

  // final checks
  else if( pos < tokcnt )
  {
    m_log << preFail
          << "Extraneous tokens found after optional arguments." << eoe;
    LOGERROR("%s", getErrorStr().c_str());
    bOk = false;
  }

  // good
  else
  {
    m_log << "Ok" << eoe;
    LOGDIAG2("Command(uid=%d, name=%s, form=%d, argc=%d) successfully parsed.",
        cmddef.getUid(), cmddef.getName().c_str(),
        form.getIndex(), form.numOfArgs());
    bOk = true;
  }

  rc = bOk? Ok: RC_ERROR;

  CL_DBG_CALL_OUT1("rc=" << rc << ", " << okstr(bOk) << ", pos=" << pos);

  return rc;
}

bool CommandLine::parseArgv0(CmdDef          &cmddef,
                             CmdFormDef      &form,
                             const StringVec &tokens,
                             size_t          &pos)
{
  bool  bOk;

  CL_DBG_CALL_IN("cmddef(uid=" << cmddef.getUid() << ")"
      << ", form(index=" << form.getIndex()
      << ", argc=" << form.numOfArgs() << ")"
      << ", tokens, pos=" << pos, endl);

  if( pos < tokens.size() )
  {
#ifdef MAYBE_THE_FUTURE
    if( peekEq("<", tokens, pos) )
    {
      bOk = parseVariableArg(cmddef, form, tokens, pos);
    }
    else
#endif // MAYBE_THE_FUTURE
    {
      bOk = parseLiteralArg(cmddef, form, tokens, pos);
    }
  }
  else
  {
    m_log << "Command argument not found." << eoe;
    LOGERROR("%s", getErrorStr().c_str());
    bOk = false;
  }

  if( bOk )
  {
    form.lastArg().orFlags(CmdArgDef::FlagCommand);

    // set command's name
    if( form.lastArg().getType() == CmdArgDef::TypeLiteral )
    {
      // literals are anonymous
      cmddef.setName(form.lastArg().literalAt(0));
    }
    else
    {
      // variables have required names
      cmddef.setName(form.lastArg().getName());
    }
  }

  CL_DBG_CALL_OUT1(okstr(bOk) << ", pos=" << pos);

  return bOk;
}

bool CommandLine::parseRequiredArgList(CmdDef          &cmddef,
                                       CmdFormDef      &form,
                                       const StringVec &tokens,
                                       size_t          &pos)
{
  bool  bOk = true;

  CL_DBG_CALL_IN("cmddef(uid=" << cmddef.getUid() << ")"
      << ", form(index=" << form.getIndex()
      << ", argc=" << form.numOfArgs() << ")"
      << ", tokens, pos=" << pos, endl);


  while( bOk && (pos < tokens.size()) )
  {
    // peek if start of optional arguments
    if( peekEq("[", tokens, pos) )
    {
      break;
    }
    
    // required argument
    else if( (bOk = parseArg(cmddef, form, tokens, pos)) )
    {
      form.m_nArgcReq++;
    }
  }

  CL_DBG_CALL_OUT1(okstr(bOk) << ", pos=" << pos);

  return bOk;
}

bool CommandLine::parseOptionalArgList(CmdDef          &cmddef,
                                       CmdFormDef      &form,
                                       const StringVec &tokens,
                                       size_t          &pos)
{
  bool  bOk = true;

  CL_DBG_CALL_IN("cmddef(uid=" << cmddef.getUid() << ")"
      << ", form(index=" << form.getIndex()
      << ", argc=" << form.numOfArgs() << ")"
      << ", tokens, pos=" << pos, endl);

  while( bOk && (pos < tokens.size()) )
  {
    // peek if start an optional arguments
    if( !peekEq("[", tokens, pos) )
    {
      break;
    }
    
    // "[ argument ]"
    if( tokEq(form.getSyntax(), "[", tokens, pos) &&
        parseArg(cmddef, form, tokens, pos) &&
        tokEq(form.getSyntax(), "]", tokens, pos) )
    {
      form.lastArg().orFlags(CmdArgDef::FlagOptional);
      form.m_nArgcOpt++;
    }
    else
    {
      bOk = false;
    }
  }

  CL_DBG_CALL_OUT1(okstr(bOk) << ", pos=" << pos);

  return bOk;
}

bool CommandLine::parseArg(CmdDef          &cmddef,
                           CmdFormDef      &form,
                           const StringVec &tokens,
                           size_t          &pos)
{
  bool  bOk;

  CL_DBG_CALL_IN("cmddef(uid=" << cmddef.getUid() << ")"
      << ", form(index=" << form.getIndex()
      << ", argc=" << form.numOfArgs() << ")"
      << ", tokens, pos=" << pos, endl);

  if( pos < tokens.size() )
  {
    if( peekEq("{", tokens, pos) )
    {
      bOk = parseXorListArg(cmddef, form, tokens, pos);
    }
    else if( peekEq("<", tokens, pos) )
    {
      bOk = parseVariableArg(cmddef, form, tokens, pos);
    }
    else
    {
      bOk = parseLiteralArg(cmddef, form, tokens, pos);
    }
  }
  else
  {
    m_log << "Command argument not found." << eoe;
    LOGERROR("%s", getErrorStr().c_str());
    bOk = false;
  }

  CL_DBG_CALL_OUT1(okstr(bOk) << ", pos=" << pos);

  return bOk;
}

bool CommandLine::parseXorListArg(CmdDef          &cmddef,
                                  CmdFormDef      &form,
                                  const StringVec &tokens,
                                  size_t          &pos)
{
  bool      bOk;
  CmdArgDef argdef;
  StringVec literals;

  CL_DBG_CALL_IN("cmddef(uid=" << cmddef.getUid() << ")"
      << ", form(index=" << form.getIndex()
      << ", argc=" << form.numOfArgs() << ")"
      << ", tokens, pos=" << pos, endl);

  form.pushArg(argdef);

  if( tokEq(form.getSyntax(), "{", tokens, pos) &&
      parseXorList(cmddef, form, tokens, pos, literals) &&
      tokEq(form.getSyntax(), "}", tokens, pos) )
  {
    stringstream  ss;
    int           n = form.numOfArgs() - 1;

    ss << "_" << n;

    form.lastArg().setName(ss.str());
    form.lastArg().setType(CmdArgDef::TypeLiteral);
    form.lastArg().orFlags(CmdArgDef::FlagXorList);

    for(size_t i = 0; i < literals.size(); ++i)
    {
      form.lastArg().addLiteralValue(literals[i]);
    }

    bOk = true;
  }
  else
  {
    bOk = false;
  }

  CL_DBG_CALL_OUT1(okstr(bOk) << ", pos=" << pos);

  return bOk;
}

bool CommandLine::parseVariableArg(CmdDef          &cmddef,
                                   CmdFormDef      &form,
                                   const StringVec &tokens,
                                   size_t          &pos)
{
  bool                bOk;
  CmdArgDef           argdef;
  string              strName;
  CmdArgDef::ArgType  eType;

  CL_DBG_CALL_IN("cmddef(uid=" << cmddef.getUid() << ")"
      << ", form(index=" << form.getIndex()
      << ", argc=" << form.numOfArgs() << ")"
      << ", tokens, pos=" << pos, endl);

  form.pushArg(argdef);

  bOk = tokEq(form.getSyntax(), "<", tokens, pos) &&
           parseIdentifier(cmddef, form, tokens, pos, strName);

  if( bOk )
  {
    if( peekEq(":", tokens, pos) )
    {
      bOk = tokEq(form.getSyntax(), ":", tokens, pos) &&
                parseType(cmddef, form, tokens, pos, eType);
    }
    else
    {
      eType = CmdArgDef::TypeWord;
    }
  }

  if( bOk )
  {
    bOk = tokEq(form.getSyntax(), ">", tokens, pos);
  }

  if( bOk )
  {
    form.lastArg().setName(strName);
    form.lastArg().setType(eType);
  }

  CL_DBG_CALL_OUT1(okstr(bOk) << ", pos=" << pos);

  return bOk;
}

bool CommandLine::parseLiteralArg(CmdDef          &cmddef,
                                  CmdFormDef      &form,
                                  const StringVec &tokens,
                                  size_t          &pos)
{
  bool      bOk;
  CmdArgDef argdef;
  string    strValue;

  CL_DBG_CALL_IN("cmddef(uid=" << cmddef.getUid() << ")"
      << ", form(index=" << form.getIndex()
      << ", argc=" << form.numOfArgs() << ")"
      << ", tokens, pos=" << pos, endl);

  form.pushArg(argdef);

  if( (bOk = parseLiteralValue(cmddef, form, tokens, pos, strValue)) )
  {
    stringstream  ss;
    int           n = form.numOfArgs() - 1;

    ss << "_" << n;

    form.lastArg().setName(ss.str());
    form.lastArg().setType(CmdArgDef::TypeLiteral);
    form.lastArg().addLiteralValue(strValue);
  }

  CL_DBG_CALL_OUT1(okstr(bOk) << ", pos=" << pos);

  return bOk;
}

bool CommandLine::parseXorList(CmdDef          &cmddef,
                               CmdFormDef      &form,
                               const StringVec &tokens,
                               size_t          &pos,
                               StringVec       &literals)
{
  bool    bOk   = true;
  bool    more = true;
  string  strValue;

  CL_DBG_CALL_IN("cmddef(uid=" << cmddef.getUid() << ")"
      << ", form(index=" << form.getIndex()
      << ", argc=" << form.numOfArgs() << ")"
      << ", tokens, pos=" << pos, endl);

  while( more && bOk && (pos < tokens.size()) )
  {
    if( peekEq("}", tokens, pos) )
    {
      m_log << "Expected literal value but found \"}\"." << eoe;
      LOGERROR("%s", getErrorStr().c_str());
      bOk = false;
    }

    else if( (bOk = parseLiteralValue(cmddef, form, tokens, pos, strValue)) )
    {
      literals.push_back(strValue);

      // peek if start of another xor option
      if( peekEq("|", tokens, pos) )
      {
        bOk = tokEq(form.getSyntax(), "|", tokens, pos);
      }
      else
      {
        more = false;
      }
    }
  }

  if( more && bOk )
  {
    m_log << "No literal value found." << eoe;
    LOGERROR("%s", getErrorStr().c_str());
    bOk = false;
  }

  CL_DBG_CALL_OUT1(okstr(bOk) << ", pos=" << pos);

  return bOk;
}

bool CommandLine::parseIdentifier(CmdDef          &cmddef,
                                  CmdFormDef      &form,
                                  const StringVec &tokens,
                                  size_t          &pos,
                                  string          &strIdent)
{
  bool    bOk = true;

  CL_DBG_CALL_IN("cmddef(uid=" << cmddef.getUid() << ")"
      << ", form(index=" << form.getIndex()
      << ", argc=" << form.numOfArgs() << ")"
      << ", tokens, pos=" << pos, endl);

  strIdent.clear();

  if( pos < tokens.size() )
  {
    strIdent = tokens[pos];

    if( (bOk = tokIdentifier(strIdent)) )
    {
      ++pos;
    }
  }
  else
  {
    m_log << "No identifier found." << eoe;
    LOGERROR("%s", getErrorStr().c_str());
    bOk = false;
  }

  CL_DBG_CALL_OUT2(okstr(bOk) << ", identifier=\"" << strIdent
      << "\", pos=" << pos);

  return bOk;
}

bool CommandLine::parseType(CmdDef             &cmddef,
                            CmdFormDef         &form,
                            const StringVec    &tokens,
                            size_t             &pos,
                            CmdArgDef::ArgType &eType)
{
  bool  bOk;

  CL_DBG_CALL_IN("cmddef(uid=" << cmddef.getUid() << ")"
      << ", form(index=" << form.getIndex()
      << ", argc=" << form.numOfArgs() << ")"
      << ", tokens, pos=" << pos, endl);

  eType = CmdArgDef::TypeUndef;

  if( pos < tokens.size() )
  {
    if( (eType = lookupArgType(tokens[pos])) != CmdArgDef::TypeUndef )
    {
      ++pos;
      bOk = true;
    }
    else
    {
      m_log << "Unknown variable type \"" << tokens[pos] << "\"." << eoe;
      LOGERROR("%s", getErrorStr().c_str());
      bOk = false;
    }
  }

  else
  {
    m_log << "No variable type found." << eoe;
    LOGERROR("%s", getErrorStr().c_str());
    bOk = false;
  }

  CL_DBG_CALL_OUT2(okstr(bOk) << ", type=" << eType << ", pos=" << pos);

  return bOk;
}

bool CommandLine::parseLiteralValue(CmdDef          &cmddef,
                                    CmdFormDef      &form,
                                    const StringVec &tokens,
                                    size_t          &pos,
                                    string          &strValue)
{
  bool  bOk;

  CL_DBG_CALL_IN("cmddef(uid=" << cmddef.getUid() << ")"
      << ", form(index=" << form.getIndex()
      << ", argc=" << form.numOfArgs() << ")"
      << ", tokens, pos=" << pos, endl);

  strValue.clear();

  if( pos < tokens.size() )
  {
    strValue = tokens[pos++];
    bOk = true;
  }
  else
  {
    m_log << "Literal value not found." << eoe;
    LOGERROR("%s", getErrorStr().c_str());
    bOk = false;
  }

  CL_DBG_CALL_OUT2(okstr(bOk) << ", value=\"" << strValue << "\", "
      << "pos=" << pos);

  return bOk;
}

bool CommandLine::tokEq(const string    &strInput,
                        const string    strCmp,
                        const StringVec &tokens,
                        size_t          &pos)
{
  bool  bOk;

  CL_DBG_CALL_IN("input, strcmp=\"" << strCmp << "\", "
      << "tokens, pos=" << pos, " ");

  if( (pos < tokens.size()) && (tokens[pos] == strCmp) )
  {
    ++pos;
    bOk = true;
  }
  else
  {
    m_log << "Expected \"" << strCmp << "\" token not found." << eoe;
    LOGERROR("%s", getErrorStr().c_str());
    bOk = false;
  }

  CL_DBG_CALL_OUT2(okstr(bOk) << ", pos=" << pos);

  return bOk;
}

bool CommandLine::peekEq(const string    strCmp,
                         const StringVec &tokens,
                         const size_t    pos)
{
  if( (pos < tokens.size()) && (tokens[pos] == strCmp) )
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool CommandLine::tokIdentifier(const string &str)
{
  bool    bOk = true;
  size_t  len;

  len = str.length();

  if( len == 0 )
  {
    m_log << "Empty identifier." << eoe;
    LOGERROR("%s", getErrorStr().c_str());
    bOk = false;
  }

  else if( !isalpha(str[0]) && (str[0] != '_') )
  {
    m_log
      << "Invalid '" << str[0] << "' at position " << 0
      << "' in identifier \"" << str << "\"." << eoe;
    LOGERROR("%s", getErrorStr().c_str());
    bOk = false;
  }

  for(size_t i = 1; bOk && (i < len); ++i)
  {
    if( !isalnum(str[i]) && (str[i] != '_') )
    {
      m_log
        << "Invalid '" << str[i] << "' at position " << i
        << "' in identifier \"" << str << "\"." << eoe;
      LOGERROR("%s", getErrorStr().c_str());
      bOk = false;
      break;
    }
  }

  return bOk;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Static Convenience Methods
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

string CommandLine::prettify(const string &str)
{
  string    realpurdy;
  bool      quoteit = false;

  for(size_t i = 0; i < str.length(); ++i)
  {
    switch( str[i] )
    {
      case '"':
        quoteit = true;
        realpurdy.append("\\\"");
        break;
      case ' ':
        quoteit = true;
        realpurdy.append(" ");
        break;
      case '\t':
        quoteit = true;
        realpurdy.append("\\t");
        break;
      case '\n':
        quoteit = true;
        realpurdy.append("\\n");
        break;
      case '\r':
        quoteit = true;
        realpurdy.append("\\r");
        break;
      case '\v':
        quoteit = true;
        realpurdy.append("\\v");
        break;
      case '\f':
        quoteit = true;
        realpurdy.append("\\f");
        break;
      case '\\':
        quoteit = true;
        realpurdy.append("\\");
        break;
      default:
        if( (str[i] < 0x21) || (str[i] > 0x7e) )
        {
          unsigned  hh = (unsigned)str[i];
          char      buf[8];

          sprintf(buf, "\\x%02x", hh);
          realpurdy.append(buf);
          quoteit = true;
        }
        else
        {
          realpurdy.push_back(str[i]);
        }
        break;
    }
  }

  if( quoteit )
  {
    realpurdy.insert(realpurdy.begin(), '"');
    realpurdy.insert(realpurdy.end(), '"');
  }

  return realpurdy;
}

string CommandLine::c14n(const StringVec &tokens)
{
  stringstream  ss;
  string        sep;

  for(size_t i = 0; i < tokens.size(); ++i)
  {
    ss << sep << prettify(tokens[i]);

    if( i == 0 )
    {
      sep = " ";
    }
  }

  return ss.str();
}

int CommandLine::strToLong(const string &str, long &val)
{
  char  sniff;

  return sscanf(str.c_str(), "%li%c", &val, &sniff) == 1? Ok: EBadSyntax;
}

int CommandLine::strToDouble(const string &str, double &val)
{
  char  sniff;

  return sscanf(str.c_str(), "%lf%c", &val, &sniff) == 1? Ok: EBadSyntax;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Output Methods and Operators
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

ostream &rnr::cmd::operator<<(ostream &os, const CommandLine &cl)
{
  int     nIndent = 0;    // TODO write a indent manip

  string  sp1(indent(nIndent+2));
  string  sep;

  CommandLine::CmdConstIter iter;

  os << "CommandLine " << cl.m_strName << endl;
  os << "{" << endl;

  os << sp1 << "name          = " << cl.m_strName << endl;
  os << sp1 << "have_readline = " << cl.m_readline.haveRlLib() << endl;
  os << sp1 << "use_readline  = " << cl.m_readline.useRlLib() << endl;
  os << sp1 << "prompt        = " << cl.getPrompt() << endl;
  os << sp1 << "ignorecase    = " << cl.m_bIgnoreCase << endl;
  os << sp1 << "cmddefs[" << cl.numOfCmds() << "] =" << endl;
  os << sp1 << "{" << endl;

  for(iter = cl.m_cmddefs.begin(); iter != cl.m_cmddefs.end(); ++iter)
  {
    os << sep << iter->second;
    if( sep.empty() )
    {
      sep = ",\n";
    }
  }
  os << endl;

  os << sp1 << "}" << endl;

  os << "}";

  return os;
}
