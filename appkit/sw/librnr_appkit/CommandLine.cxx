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
 * \brief Record error convenience macro.
 *
 * \param errString   String to record error message.
 * \param errStream   Error message insertion stream.
 */
#define CL_ERROR(errString, errStream) \
do \
{ \
  stringstream _ss; \
  _ss << errStream; \
  errString = _ss.str(); \
} while(0)

/*!
 * \brief Record and log error convenience macro.
 *
 * \param errString   String to record error message.
 * \param errStream   Error message insertion stream.
 */
#define CL_LOG_ERROR(errString, errStream) \
do \
{ \
  CL_ERROR(errString, errStream); \
  LOGERROR("%s", errString.c_str()); \
} while(0)

/*!
 * \brief Trace debugging.
 */
#undef CL_ENABLE_DEBUG

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
     * \brief Create indentaion string.
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

    static const string noliteral;  ///< "no literal" literal
    static CmdArgDef    noargdef;   ///< "no arg def" argument definition
    static CmdFormDef   noformdef;  ///< "no form def" form definition
    static CmdDef       nocmddef;   ///< "no cmd def" command definition
    static const string noprompt;   ///< "no prompt" prompt value


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
  m_nIndex  = -1;
  m_eType   = TypeUndef;
  m_uFlags  = 0;
}

CmdArgDef::CmdArgDef(const CmdArgDef &src)
{
  m_nIndex  = src.m_nIndex;
  m_strName = src.m_strName;
  m_eType   = src.m_eType;
  m_values  = src.m_values;
  m_uFlags  = src.m_uFlags;
}

CmdArgDef::~CmdArgDef()
{
  //fprintf(stderr, "rdk: %s: %s\n", __func__, m_strName.c_str());
}

CmdArgDef &CmdArgDef::operator=(const CmdArgDef &rhs)
{
  m_nIndex  = rhs.m_nIndex;
  m_strName = rhs.m_strName;
  m_eType   = rhs.m_eType;
  m_values  = rhs.m_values;
  m_uFlags  = rhs.m_uFlags;

  return *this;
}

bool CmdArgDef::isDefined() const
{
  return (m_nIndex >= 0) && !m_strName.empty() && (m_eType != TypeUndef);
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

ostream &rnr::cmd::operator<<(ostream &os, const CmdArgDef &argdef)
{
  string sep;

  os << "{";
  os << argdef.m_strName << ", ";
  os << argdef.m_eType << "(T" << lookupArgSymbol(argdef.m_eType) << "), ";
  os << "0x" << std::hex << argdef.m_uFlags << std::dec
        << "(" << lookupFlagNames(argdef.m_uFlags) << "), ";
  os << "{"; 

  for(size_t i = 0; i < argdef.m_values.size(); ++i)
  {
    os << sep << argdef.m_values[i];
    if( sep.empty() )
    {
      sep = ", ";
    }
  }

  os << "}";

  os << "}";

  return os;
}


// -----------------------------------------------------------------------------
// CmdFormDef Class
// -----------------------------------------------------------------------------

CmdFormDef::CmdFormDef()
{
  m_nIndex    = -1;
  m_nArgcReq  = 0;
  m_nArgcOpt  = 0;
}

CmdFormDef::CmdFormDef(const string &strSyntax) :
    m_strSyntax(strSyntax)
{
  m_nIndex    = -1;
  m_nArgcReq  = 0;
  m_nArgcOpt  = 0;
}

CmdFormDef::CmdFormDef(const CmdFormDef &src)
{
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
  string sp0(indent(8));
  string sp1(indent(10));
  string sp2(indent(12));
  string sep;

  os << sp0 << "{" << endl;

  os << sp1 << "syntax  = " << formdef.m_strSyntax << endl;
  os << sp1 << "argc    = " << formdef.numOfArgs() << "(total)"
        << " " << formdef.numOfRequiredArgs() << "(required)"
        << " " << formdef.numOfOptionalArgs() << "(optional)"
        << endl;
  os << sp1 << "args[" << formdef.numOfArgs() << "] =" << endl;
  os << sp1 << "{" << endl;

  for(size_t i = 0; i < formdef.m_argdefs.size(); ++i)
  {
    os << sep << sp2 << formdef.m_argdefs[i];
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
  string sp0(indent(4));
  string sp1(indent(6));
  string sep;

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

CommandLine::CommandLine(const string strName,
                         const string strPrompt,
                         bool         bUseRlLib,
                         bool         bIgnoreCase) :
    m_strName(strName),
    m_readline(strName, strPrompt, bUseRlLib),
    m_bIgnoreCase(bIgnoreCase)
{
  m_nUidCnt     = 0;
  m_bIsCompiled = false;

  pushPrompt(strPrompt);
}

CommandLine::~CommandLine()
{
  //fprintf(stderr, "rdk: %s: %s\n", __func__, m_strName.c_str());
}

bool CommandLine::isDefined() const
{
  return m_bIsCompiled;
}

int CommandLine::addCommand(const string strMultiFormSyntax)
{
  StringVec   syntaxForms;
  string      strName;
  int         uid;

  split(strMultiFormSyntax, '\n', syntaxForms);

  if( syntaxForms.size() == 0 || syntaxForms[0].empty() )
  {
    CL_LOG_ERROR(m_strError, "No syntax forms specified.");
    return NoUid;
  }

  // extract command name from the syntax
  strName = extractArgv0(syntaxForms[0]);

  if( strName.empty() )
  {
    CL_LOG_ERROR(m_strError, "No valid command name specified: "
        << syntaxForms[0]);
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
      CL_LOG_ERROR(m_strError, "Command name mismatch: Expected " << strName
          << ": " << syntaxForms[i]);
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

  // no commands added
  if( m_nUidCnt == 0 )
  {
    CL_LOG_ERROR(m_strError, "No commands added.");
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
    rc = buildReadLineGenerator();
  }

  // finalize
  if( rc == Ok )
  {
    // register readline generator callback
    m_readline.registerGenerator(rlGeneratorWrapper, this);

    m_bIsCompiled = true;
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

  for(i = 0, rc = Ok; (i < cmddef.numOfForms()) && (rc == Ok); ++i)
  {
    CmdFormDef &form = cmddef.formAt(i);

    if( (tokcnt = tokenizeSyntax(form.getSyntax(), tokens)) < 0 )
    {
      rc = tokcnt;
    }

    else if( tokcnt == 0 )
    {
      CL_LOG_ERROR(m_strError,
        "CmdDef " << cmddef.m_nUid << ": "
        << "form " << form.getIndex() << ": No syntax specified.");
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

int CommandLine::readCommand(int &uid, StringVec &argv)
{
  string  strLine;

  // read a line of input
  strLine = m_readline.rlreadLine();

  return processCommand(strLine, uid, argv);
}

int CommandLine::readCommand(FILE *fp, int &uid, StringVec &argv)
{
  string  strLine;

  // read a line of input
  if( fileno(fp) == fileno(stdin) )
  {
    strLine = m_readline.rlreadLine();
  }
  else
  {
    strLine = m_readline.ireadLine(fp);
  }

  return processCommand(strLine, uid, argv);
}

void CommandLine::addToHistory(const StringVec &argv)
{
  m_readline.addToHistory(c14n(argv));
}

int CommandLine::buildReadLineGenerator()
{
  return Ok;
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

char *CommandLine::rlGeneratorWrapper(string &strText,
                                      int    nState,
                                      string &strContext,
                                      void   *pAppArg)
{
  if( pAppArg != NULL )
  {
    return ((CommandLine *)pAppArg)->rlGenerator(strText, nState, strContext);
  }
  else
  {
    return NULL;
  }
}

char *CommandLine::rlGenerator(string &strText,
                               int     nState,
                               string &strContext)
{
  CL_DBG("{text=" << strText
      << ", state=" << nState
      << ", context=" << strContext
      << "}" << endl);

  return NULL;
}

ssize_t CommandLine::tokenize(const string &str, StringVec &tokens)
{
  size_t    i, len;
  string    tok;

  CL_DBG_CALL_IN("\"" << str << "\"", endl);

  tokens.clear();

  len = str.length();

  for(i = 0; i < len; )
  {
    // find start of the next token
    while( (i < len) && isspace((int)str[i]) )
    {
      ++i;
    }

    // no more tokens
    if( i >= len ) 
    {
      break;
    }

    // new qouted string token
    if( isdquote(str[i]) )
    {
      i += lexQuotedString(str.substr(i), tokens);
    }

    // new contiguous word token
    else
    {
      i += lexWord(str.substr(i), tokens);
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

ssize_t CommandLine::tokenizeSyntax(const string &str, StringVec &tokens)
{
  size_t    i, len;
  string    tok;

  CL_DBG_CALL_IN("\"" << str << "\", tokens", endl);

  tokens.clear();

  len = str.length();

  for(i = 0; i < len; )
  {
    // find start of the next token
    while( (i < len) && isspace((int)str[i]) )
    {
      ++i;
    }

    // no more tokens
    if( i >= len ) 
    {
      break;
    }

    //  special character token
    if( isspecial(str[i]) )
    {
      tok = str[i++];
      pushToken(tok, tokens);
    }

    // new contiguous word token
    else
    {
      i += lexSyntaxWord(str.substr(i), tokens);
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

void CommandLine::pushToken(string &tok, StringVec &tokens)
{
  tokens.push_back(tok);
  tok.clear();
}

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

  // command
  if( !parseArgv0(cmddef, form, tokens, pos) )
  {
    LOGERROR("Failed to parse special argv0.");
    bOk = false;
  }

  // required argmuents - may be none
  else if( !parseRequiredArgList(cmddef, form, tokens, pos) )
  {
    LOGERROR("Failed to parse required argument list.");
    bOk = false;
  }

  // optional arguments - may be none
  else if( !parseOptionalArgList(cmddef, form, tokens, pos) )
  {
    LOGERROR("Failed to parse optional argument list.");
    bOk = false;
  }

  // final checks
  else if( pos < tokcnt )
  {
    CL_LOG_ERROR(m_strError,
        "Extraneous tokens found after optional arguments.");
    bOk = false;
  }

  // good
  else
  {
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
    CL_LOG_ERROR(m_strError,"Command argument not found.");
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
    CL_LOG_ERROR(m_strError,"Command argument not found.");
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
      CL_LOG_ERROR(m_strError,"Expected literal value but found \"}\".");
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
    CL_LOG_ERROR(m_strError, "No literal value found.");
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
    CL_LOG_ERROR(m_strError, "No identifier found.");
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
      CL_LOG_ERROR(m_strError,
          "Unknown variable type \"" << tokens[pos] << "\".");
      bOk = false;
    }
  }

  else
  {
    CL_LOG_ERROR(m_strError, "No variable type found.");
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
    CL_LOG_ERROR(m_strError, "Literal value not found.");
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
    CL_LOG_ERROR(m_strError, "Expected \"" << strCmp << "\" token not found.");
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
    CL_LOG_ERROR(m_strError, "Empty identifier.");
    bOk = false;
  }

  else if( !isalpha(str[0]) && (str[0] != '_') )
  {
    CL_LOG_ERROR(m_strError,
            "Invalid '" << str[0] << "' at position " << 0
          << "' in identifier \"" << str << "\".");
    bOk = false;
  }

  for(size_t i = 1; bOk && (i < len); ++i)
  {
    if( !isalnum(str[i]) && (str[i] != '_') )
    {
      CL_LOG_ERROR(m_strError,
            "Invalid '" << str[i] << "' at position " << i
            << "' in identifier \"" << str << "\".");
      bOk = false;
      break;
    }
  }

  return bOk;
}

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

const string &CommandLine::getErrorStr() const
{
  return m_strError;
}

int CommandLine::processCommand(string &strLine, int &uid, StringVec &argv)
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

int CommandLine::matchCommand(int &uid, StringVec &argv)
{
  CmdIter         iter;
  vector<CmdIter> cmdCandidates;
  int             rc;

  uid  = NoUid;

  // invalid to call this function with no arguments - no match is possible
  if( argv.size() == 0 )
  {
    CL_LOG_ERROR(m_strError, "Bug: No arguments to match.");
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
    CL_ERROR(m_strError, "Unknown command: " << c14n(argv));
    rc = EUnknownCmd;
  }

  // duplicates
  else if( cmdCandidates.size() > 1 )
  {
    CL_ERROR(m_strError, cmdCandidates.size() 
        << " duplicate commands definitions found for "
        << cmdCandidates.back()->second.getName());
    rc = EAmbigCmd;
  }

  // got a single match on command name - validate
  else
  {
    uid = cmdCandidates[0]->first;
    rc = validateCommand(cmdCandidates[0]->second, argv);
  }

  return rc;
}

int CommandLine::validateCommand(const CmdDef &cmddef, StringVec &argv)
{
  double  fFitness, fMaxFitness;
  int     iBest, i;
  int     rc;

  fMaxFitness = 0.0;
  iBest       = -1;

  for(i = 0; i < cmddef.numOfForms(); ++i)
  {
    const CmdFormDef &form = cmddef.formAt(i);

    if( (rc = validateCommand(cmddef, form, argv, fFitness)) == Ok )
    {
      if( fFitness > fMaxFitness )
      {
        fMaxFitness = fFitness;
        iBest       = i;
      }
    }
  }

  if( iBest >= 0 )
  {
    LOGDIAG2("Command %d,form %d match: Fitness %lf: %s", 
        cmddef.getUid(), iBest, fMaxFitness, c14n(argv).c_str());
    rc = Ok;
  }
  else if( rc == Ok )
  {
    CL_ERROR(m_strError, "Unknown command: " << c14n(argv));
    rc = EUnknownCmd;
  }

  return rc;
}


int CommandLine::validateCommand(const CmdDef     &cmddef,
                                 const CmdFormDef &form,
                                 StringVec        &argv,
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
    CL_ERROR(m_strError, cmddef.getName() << ": Too many arguments: "
        << in_argc << " specified, "
        << form_argc << " maximum.");
    return EBadSyntax;
  }

  else if( (in_argc - 1) < form.numOfRequiredArgs() )
  {
    CL_ERROR(m_strError, cmddef.getName() << ": Missing required arguments: "
        << in_argc << " specified, "
        << form.numOfRequiredArgs()+1 << " required.");
    return EBadSyntax;
  }

  for(i = 0, j = 0, rc = Ok;
      (i < form_argc) && (j < in_argc) && (rc == Ok);
      ++i)
  {
    const CmdArgDef &argdef = form.argAt(i);

    switch( argdef.getType() )
    {
      case CmdArgDef::TypeLiteral:
        fWeight = 1.0;
        rc = validateArgLiteral(cmddef, argdef, argv[j]);
        break;
      case CmdArgDef::TypeWord:
      case CmdArgDef::TypeQuotedString:
        fWeight = 0.90;
        break; // nothing to validate
      case CmdArgDef::TypeInteger:
        fWeight = 0.95;
        rc = validateArgInteger(cmddef, argdef, argv[j]);
        break;
      case CmdArgDef::TypeFloat:
        fWeight = 0.95;
        rc = validateArgFloat(cmddef, argdef, argv[j]);
        break;
      case CmdArgDef::TypeUndef:
      default:
        CL_LOG_ERROR(m_strError, cmddef.getName() << ": Bug: "
                << argdef.m_eType << ": Unknown type.");
        rc = EError;
        break;
    }

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
    else if( argdef.getFlags() & CmdArgDef::FlagOptional )
    {
      rc = Ok;
    }
    else
    {
      fFitness = 0.0;
    }
  }

  fFitness /= (double)in_argc;

  return rc;
}

int CommandLine::validateArgLiteral(const CmdDef      &cmddef,
                                    const CmdArgDef   &argdef,
                                    const std::string &strValue)
{
  stringstream  ss;
  int           i;

  for(i = 0; i < argdef.numOfLiterals(); ++i)
  {
    const string &strLiteral = argdef.literalAt(i);

    // TODO: support ignore case
    if( strValue == strLiteral )
    {
      break;
    }

    ss << " " << strLiteral;
  }
        
  if( i < argdef.numOfLiterals() )
  {
    return Ok;
  }

  else
  {
    CL_ERROR(m_strError, cmddef.getName() << ": "
              << "<" << argdef.getName() << "> = "
              << strValue << ": Not one of:" << ss.str() << ".");

    return EBadSyntax;
  }
}

int CommandLine::validateArgInteger(const CmdDef      &cmddef,
                                    const CmdArgDef   &argdef,
                                    const std::string &strValue)
{
  long  val;
  int   rc;

  if( (rc = strToLong(strValue, val)) == Ok )
  {
    return Ok;
  }

  else
  {
    CL_ERROR(m_strError, cmddef.getName() << ": "
              << "<" << argdef.getName() << "> = "
              << strValue << ": Not an integer.");
    return rc;
  }
}

int CommandLine::validateArgFloat(const CmdDef      &cmddef,
                                  const CmdArgDef   &argdef,
                                  const std::string &strValue)
{
  double  val;
  int     rc;


  if( (rc = strToDouble(strValue, val)) == Ok )
  {
    return Ok;
  }

  else
  {
    CL_ERROR(m_strError, cmddef.getName() << ": "
              << "<" << argdef.getName() << "> = "
              << strValue << ": Not a float.");
    return rc;
  }
}

int CommandLine::checkReadResult()
{
  // EOF
  if( m_readline.isEoF() )
  {
    m_strError = "EOF";
    return EEoF;
  }

  // file read error
  else if( m_readline.isFError() )
  {
    m_strError = m_readline.getErrorStr();
    return ERead;
  }

  // read successful
  else
  {
    return Ok;
  }
}

ostream &rnr::cmd::operator<<(ostream &os, const CommandLine &cl)
{
  string sp1(indent(2));
  string sep;

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
