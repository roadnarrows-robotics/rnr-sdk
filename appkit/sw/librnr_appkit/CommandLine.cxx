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

#include "rnr/appkit/RegEx.h"
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
#define CL_SET_CALL_DEPTH(_n) dbg_calldepth_ = _n
#define CL_PUSH_CALL_DEPTH() ++dbg_calldepth_
#define CL_POP_CALL_DEPTH()  --dbg_calldepth_

/*!
 * \brief Enter function debug macro.
 * \param _args Stream of function arguments.
 * \param _post Stream of post call data.
 */
#define CL_DBG_CALL_IN(_args, _post) \
do \
{ \
  CL_PUSH_CALL_DEPTH(); \
  cerr << space(dbg_calldepth_*2) << __func__ << "(" << _args << ")" <<_post; \
} while(0)

/*!
 * \brief Enter parse function debug macro.
 * \param _cmddef   Command definition.
 * \param _formdef  Form definition.
 * \param _toks     Input tokens.
 * \param _pos      Parse position in tokens.
 * \param _post     Stream of post call data.
 */
#define CL_DBG_PARSE_CALL_IN(_cmddef, _formdef, _toks, _pos, _post) \
  CL_DBG_CALL_IN("cmddef(uid=" << (_cmddef).getUid() \
      << ", name=" << (_cmddef).getName() << ")"     \
      << ", form(index=" << (_formdef).getIndex()    \
      << ", argc=" << (_formdef).numOfArgs() << ")"  \
      << ", tokens=" << (_toks).size()               \
      << ", pos=" << (_pos), _post)

/*!
 * \brief Exit function debug macro with results on New Line.
 * \param _res    Results.
 */
#define CL_DBG_CALL_OUT_NL(_res) \
do \
{ \
  cerr << space(dbg_calldepth_*2) << "--> (" << _res << ")" << endl; \
  CL_POP_CALL_DEPTH(); \
} while(0)

/*!
 * \brief Exit function debug macro with results In Line.
 * \param _res    Results.
 */
#define CL_DBG_CALL_OUT_IL(_res) \
do \
{ \
  cerr << "--> (" << _res << ")" << endl; \
  CL_POP_CALL_DEPTH(); \
} while(0)

/*!
 * \brief Debug macro with no indentation.
 * \param _data   Any data stream.
 */
#define CL_DBG(_data) cerr << _data

#else
#define CL_CALL_DEPTH
#define CL_SET_CALL_DEPTH(_n)
#define CL_PUSH_CALL_DEPTH()
#define CL_POP_CALL_DEPTH()
#define CL_DBG_CALL_IN(_args, _post)
#define CL_DBG_PARSE_CALL_IN(_cmddef, _formdef, _toks, _pos, _post)
#define CL_DBG_CALL_OUT_NL(_res)
#define CL_DBG_CALL_OUT_IL(_res)
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
      return CmdArgDef::ArgTypeUndef;
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

    static const char *FalseHood[] =
    {
      "0", "false", "f", "off", "low", "disable", "no", NULL
    };

    static const char *TruthHood[] =
    {
      "1", "true", "t", "on", "high", "enable", "yes", NULL
    };

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
              (c == '[') || (c == ']') ||
              (c == '(') || (c == ')') );
    }

    /*!
     * \brief Test if c is a double quote character.
     */
    static inline bool isdquote(int c)
    {
      return(c == '"');
    }

    /*!
     * \brief Test if c is a open parenthesis character.
     */
    static inline bool isoparen(int c)
    {
      return(c == '(');
    }

    /*!
     * \brief Test if c is a close parenthesis character.
     */
    static inline bool iscparen(int c)
    {
      return(c == ')');
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
     * \brief Create space string.
     *
     * \param n Number of spaces.
     *
     * \return String.
     */
    static string space(unsigned n)
    {
      string s;
      while( n-- > 0 )
      {
        s.push_back(' ');
      }
      return s;
    }

    /*!
     * \brief Output stream indentation manipulator structure.
     */
    struct osManipIndent
    {
      /*
       * \brief Indentation change type enum.
       */
      enum change
      {
        NoChange,   ///< no change
        Relative,   ///< relative to current indentation level (delta)
        Absolute    ///< absolute
      };

      long    m_nIndent;  ///< new indentation value
      change  m_eChange;  ///< how the new indentationn is applied
      bool    m_bOut;     ///< do [not] output indentaion
    };

    /*!
     * \brief Set absolute indentation level.
     *
     * \param nIndent   Number of spaces to left indent.
     *
     * \return Indent manipulation object.
     */
    osManipIndent setindent(const long nIndent)
    {
      osManipIndent obj;

      obj.m_nIndent = nIndent;
      obj.m_eChange = osManipIndent::Absolute;
      obj.m_bOut    = false;

      return obj;
    }

    /*!
     * \brief Set relative delta indentation level.
     *
     * \param nDelta    Plus or minus delta from current indentation level.
     *
     * \return Indent manipulation object.
     */
    osManipIndent deltaindent(const long nDelta)
    {
      osManipIndent obj;

      obj.m_nIndent = nDelta;
      obj.m_eChange = osManipIndent::Relative;
      obj.m_bOut    = false;

      return obj;
    }

    /*!
     * \brief Left indent at current stream indentation level.
     *
     * \return Indent manipulation object.
     */
    osManipIndent indent()
    {
      osManipIndent obj;

      obj.m_nIndent = 0;
      obj.m_eChange = osManipIndent::NoChange;
      obj.m_bOut    = true;

      return obj;
    }

    /*!
     * \brief Insert indentation object into output stream.
     *
     * \param os  Output stream.
     * \param f   Object to insert.
     *
     * \return Reference to output stream.
     */
    ostream &operator<<(ostream &os, osManipIndent f)
    {
      const static int index = os.xalloc();

      long  level;

      switch( f.m_eChange )
      {
        case osManipIndent::Relative:
          level = os.iword(index) + f.m_nIndent;
          if( level < 0 )
          {
            level = 0;
          }
          os.iword(index) = level;
          break;
        case osManipIndent::Absolute:
          level = f.m_nIndent >= 0? f.m_nIndent: 0;
          os.iword(index) = level;
          break;
        case osManipIndent::NoChange:
        default:
          break;
      }

      if( f.m_bOut )
      {
        for(level = 0; level < os.iword(index); ++level)
        {
          os << ' ';
        }
      }

      return os;
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

    /*!
     * \brief Convert string to lower case.
     *
     * \param str String to convert.
     *
     * \return Lower case string.
     */
    string lowercase(const string &str)
    {
      std::locale loc;
      string      lower;

      for(size_t i = 0; i < str.size(); ++i)
      {
        lower.push_back(std::tolower(str[i], loc));
      }

      return lower;
    }

    /*!
     * \brief Find the Greatest Common SubString length.
     *
     * Prefix version starting at position 0.
     *
     * \param s   String 1.
     * \param t   String 2.
     *
     * \return Returns length of common substring.
     */
    size_t gcss(const string &s, const string &t)
    {
      size_t  i;

      for(i = 0; (i < s.size()) && (i < t.size()); ++i)
      {
        if( s[i] != t[i] )
        {
          break;
        }
      }

      return i;
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

    void help(ostream &os, const CmdDesc &desc, bool bLongHelp)
    {
      size_t  i;
    
      if( desc.m_sName == NULL )
      {
        os << "Error: Command has no name." << endl;
        return;
      }

      if( desc.m_sSyntax == NULL )
      {
        os << "Error: Command has no extended usage syntax." << endl;
        return;
      }

      StringVec usages = split(desc.m_sSyntax, '\n');

      if( bLongHelp )
      {
        ssize_t len;
        size_t  w;

        len = (ssize_t)strlen(desc.m_sName) + 2;
        len = 80 - 2 * len;
        os << desc.m_sName << "()";
        if( len > 1 )
        {
          w = os.width();
          os.width(len);
          os << "";
          os.width(w);
          os << desc.m_sName << "()";
        }
        os << "\n\n";

        os << "Name\n  " << desc.m_sName;
        if( desc.m_sSynopsis != NULL )
        {
          os << " - " << desc.m_sSynopsis;
        }
        os << "\n\n";

        os << "Synopsis\n";
        for(i = 0; i < usages.size(); ++i)
        {
          if( !usages[i].empty() )
          {
            os << "  " << usages[i] << "\n";
          }
        }
        os << "\n";

        if( desc.m_sLongDesc != NULL )
        {
          os << "Description\n";

          StringVec lines = split(desc.m_sLongDesc, '\n');
          size_t    wstart, wend, wlen; // word data
          size_t    cursor;             // output line cursor

          for(i = 0; i < lines.size(); ++i)
          {
            // indent
            os << "  ";
            cursor = 2;

            //
            // Output line, wrapping if necessary.
            //
            for(size_t j = 0; j < lines[i].size(); )
            {
              // whitespace
              while( isspace(lines[i][j]) && (j < lines[i].size()) )
              {
                // less than max output line length
                if( cursor < 80 )
                {
                  cout << ' ';
                  ++cursor;
                }
                // wrap
                else
                {
                  // wrap and indent
                  cout << "\n  ";
                  cursor = 2;
                }
                ++j;
              }

              if( j >= lines[i].size() )
              {
                break;
              }

              // word
              wstart = j;

              while( !isspace(lines[i][j]) && (j < lines[i].size()) )
              {
                ++j;
              }

              wend  = j;
              wlen  = wend - wstart;

              // wrap
              if( (cursor > 2) && (cursor+wlen >= 80) )
              {
                /// wrap and indent
                cout << "\n  ";
                cursor = 2;
              }

              cout << lines[i].substr(wstart, wlen);
              cursor += wlen;
            }

            cout << '\n';
          }
          cout << '\n';
        }
      }

      else
      {
        for(i = 0; i < usages.size(); ++i)
        {
          if( !usages[i].empty() )
          {
            os << usages[i] << "\n";
          }
        }
      }
    }

  } // namespace cmd
} // namespace rnr


// -----------------------------------------------------------------------------
// ExtArg Class
// -----------------------------------------------------------------------------

ExtArg::ExtArg()
{
  m_nCmdUid       = CommandLine::NoUid;
  m_nFormIndex    = CommandLine::NoIndex;
  m_nArgIndex     = CommandLine::NoIndex;
  m_nArgInstance  = 0;
  m_eCvtType      = CvtTypeUndef;
  m_bCvtVal       = false;
  m_lCvtVal       = 0;
  m_fCvtVal       = 0.0;
}

ExtArg::ExtArg(const int         &nCmdUid,
               const int         &nFormIndex,
               const int         &nArgIndex,
               const int         &nArgInstance,
               const std::string &strArg) :
    m_nCmdUid(nCmdUid), m_nFormIndex(nFormIndex), m_nArgIndex(nArgIndex),
    m_nArgInstance(nArgInstance), m_strArg(strArg)
{
  m_eCvtType      = CvtTypeUndef;
  m_bCvtVal       = false;
  m_lCvtVal       = 0;
  m_fCvtVal       = 0.0;
}

ExtArg::ExtArg(const ExtArg &src)
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

ExtArg::~ExtArg()
{
}

ExtArg &ExtArg::operator=(const ExtArg &rhs)
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

void ExtArg::s(const string &strVal)
{
  m_strCvtVal = strVal;
  m_eCvtType  = CvtTypeString;
}

void ExtArg::e(const long eVal)
{
  m_strCvtVal = m_strArg;
  m_lCvtVal   = eVal;
  m_eCvtType  = CvtTypeEnum;
}

void ExtArg::b(const bool bVal)
{
  m_bCvtVal   = bVal;
  m_eCvtType  = CvtTypeBoolean;
}

void ExtArg::i(const long lVal)
{
  m_lCvtVal   = lVal;
  m_eCvtType  = CvtTypeInteger;
}

void ExtArg::f(const double fVal)
{
  m_fCvtVal   = fVal;
  m_eCvtType  = CvtTypeFpn;
}

bool ExtArg::operator==(const ExtArg &rval) const
{
  return (type() == rval.type()) && (arg() == rval.arg());
}

bool ExtArg::operator==(const std::string &rval) const
{
  return ((type() == CvtTypeString) || (type() == CvtTypeEnum)) &&
          (s() == rval);
}

bool ExtArg::operator==(const char* const &rval) const
{
  if( rval == NULL )
  {
    return false;
  }

  return ((type() == CvtTypeString) || (type() == CvtTypeEnum)) &&
          (strcmp(s().c_str(), rval) == 0);
}

bool ExtArg::operator==(const bool &rval) const
{
  return (type() == CvtTypeBoolean) && (b() == rval);
}

bool ExtArg::operator==(const long &rval) const
{
  return ((type() == CvtTypeEnum) || (type() == CvtTypeInteger)) &&
          (i() == rval);
}

bool ExtArg::operator==(const double &rval) const
{
  return (type() == CvtTypeFpn) && (f() == rval);
}

ostream &rnr::cmd::operator<<(ostream &os, const ExtArg &arg)
{
  switch( arg.type() )
  {
    case ExtArg::CvtTypeString:
      os << arg.s();
      break;
    case ExtArg::CvtTypeEnum:
      os << arg.s();  // string has priority over enum
      break;
    case ExtArg::CvtTypeBoolean:
      os << arg.b();
      break;
    case ExtArg::CvtTypeInteger:
      os << arg.i();
      break;
    case ExtArg::CvtTypeFpn:
      os << arg.f();
      break;
    case ExtArg::CvtTypeUndef:
    default:
      os << "undef";
      break;
  }

  return os;
}

LogBook &rnr::cmd::operator<<(LogBook &log, const ExtArg &arg)
{
  stringstream ss;

  ss  << arg;
  log << ss.str();

  return log;
}


// -----------------------------------------------------------------------------
// Token Class
// -----------------------------------------------------------------------------

Token::Token()
{
  m_posStart = 0;
  m_posEnd   = 0;
}

Token::Token(const string &strValue, size_t posStart, size_t posEnd) :
  m_strValue(strValue), m_posStart(posStart), m_posEnd(posEnd)
{
}

Token::Token(const Token &src)
{
  m_strValue  = src.m_strValue;
  m_posStart  = src.m_posStart;
  m_posEnd    = src.m_posEnd;
}

Token::~Token()
{
}

Token &Token::operator=(const Token &rhs)
{
  m_strValue  = rhs.m_strValue;
  m_posStart  = rhs.m_posStart;
  m_posEnd    = rhs.m_posEnd;

  return *this;
}

ostream &Token::oloc(ostream &os, const string &strLine)
{
  size_t  w = os.width();

  os << *this << endl;
  os << strLine << endl;
  if( m_posStart > 0 )
  {
    os.width(m_posStart - 1);
  }
  os << '^' << endl;

  os.width(w);

  return os;
}

ostream &rnr::cmd::operator<<(ostream &os, const Token &tok)
{
  os << "'" << CommandLine::prettify(tok.value()) << "'";
  //os << " @" << tok.m_posStart << ":" << tok.m_posEnd;

  return os;
}

LogBook &rnr::cmd::operator<<(LogBook &log, const Token &tok)
{
  stringstream ss;

  ss  << tok;
  log << ss.str();

  return log;
}


// -----------------------------------------------------------------------------
// CmdArgDef Class
// -----------------------------------------------------------------------------

CmdArgDef::CmdArgDef()
{
  m_nCmdUid     = CommandLine::NoUid;
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

double CmdArgDef::match(const string &strArg, bool bIgnoreCase) const
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
      if( CommandLine::isIdentifier(strArg) )
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
          rc = CommandLine::strToBool(lowercase(strArg), val);
        }
        else
        {
          rc = CommandLine::strToBool(strArg, val);
        }

        if( rc == CommandLine::AOk )
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
        if( (CommandLine::strToLong(strArg, val) == CommandLine::AOk) &&
            inRange(val) )
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
        if( (CommandLine::strToDouble(strArg, val) == CommandLine::AOk) &&
            inRange(val) )
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

int CmdArgDef::matchLiteral(const string &strArg, bool bIgnoreCase) const
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

ExtArg CmdArgDef::convert(const string &strArg, bool bIgnoreCase) const
{
  ExtArg cvt(m_nCmdUid, m_nFormIndex, m_nIndex, 0, strArg);

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
          rc = CommandLine::strToBool(lowercase(strArg), val);
        }
        else
        {
          rc = CommandLine::strToBool(strArg, val);
        }
        if( rc == CommandLine::AOk )
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

        if( CommandLine::strToLong(strArg, val) == CommandLine::AOk )
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

        if( CommandLine::strToDouble(strArg, val) == CommandLine::AOk )
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

ostream &rnr::cmd::operator<<(ostream &os, const CmdArgDef &argdef)
{
  os << indent() << "{" << endl;

  os << deltaindent(2);

  os << indent() << "cmduid      = " << argdef.m_nCmdUid << endl;
  os << indent() << "formindex   = " << argdef.m_nFormIndex << endl;
  os << indent() << "index       = " << argdef.m_nIndex << endl;
  os << indent() << "name        = " << argdef.m_strName << endl;
  os << indent() << "type        = "
    << argdef.m_eType << "(" << lookupArgSymbol(argdef.m_eType) << ")" << endl;
  os << indent() << "flags       = 0x"
    << std::hex << argdef.m_uFlags << std::dec
    << "(" << lookupFlagNames(argdef.m_uFlags) << ")" << endl;
  os << indent() << "literals[" << argdef.m_literals.size() << "] = {"
    << argdef.constructLiteralList(", ")  << "}" << endl;
  os << indent() << "ranges[" << argdef.m_ranges.size() << "]   = ("
    << argdef.constructRangeList(", ")  << ")" << endl;
  os << indent() << "regex       = " << argdef.m_re << endl;

  os << deltaindent(-2);

  os << indent() << "}";

  return os;
}


// -----------------------------------------------------------------------------
// CmdFormDef Class
// -----------------------------------------------------------------------------

CmdFormDef::CmdFormDef()
{
  m_nCmdUid   = CommandLine::NoUid;
  m_nIndex    = CommandLine::NoIndex;
  m_nArgcReq  = 0;
  m_nArgcOpt  = 0;
}

CmdFormDef::CmdFormDef(const string &strSyntax) :
    m_strSyntax(strSyntax)
{
  m_nCmdUid   = CommandLine::NoUid;
  m_nIndex    = CommandLine::NoIndex;
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

void CmdFormDef::reset()
{
  m_argdefs.clear();
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
  m_argdefs.push_back(argdef);
}

const CmdArgDef &CmdFormDef::at(const int nIndex) const
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

  for(size_t i = 0; i < formdef.m_argdefs.size(); ++i)
  {
    os << sep << formdef.m_argdefs[i];
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

// -----------------------------------------------------------------------------
// CmdDef Class
// -----------------------------------------------------------------------------

CmdDef::CmdDef()
{
  m_nUid = CommandLine::NoUid;
}

CmdDef::CmdDef(const CmdDef &src)
{
  m_nUid      = src.m_nUid;
  m_strName   = src.m_strName;
  m_formdefs  = src.m_formdefs;
}

CmdDef::~CmdDef()
{
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

void CmdDef::reset()
{
  for(size_t i = 0; i < m_formdefs.size(); ++i)
  {
    m_formdefs[i].reset();
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

void CmdDef::pushForm(CmdFormDef &formdef)
{
  int nIndex = numOfForms();

  formdef.setParent(m_nUid);
  formdef.setIndex(nIndex);
  m_formdefs.push_back(formdef);
}

const CmdFormDef &CmdDef::at(const int nIndex) const
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
  string  sep;

  os << indent() << "{" << endl;

  os << deltaindent(2);

  os << indent() << "uid      = " << cmddef.m_nUid << endl;
  os << indent() << "name     = " << cmddef.m_strName << endl;
  os << indent() << "forms[" << cmddef.numOfForms() << "] = " << endl;
  os << indent() << "{" << endl;

  os << deltaindent(2);

  for(size_t i = 0; i < cmddef.m_formdefs.size(); ++i)
  {
    os << sep << cmddef.m_formdefs[i];

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
static const string    labelExec("Execute: ");
static const string  labelNoExec("NoExec:  ");
static const string labelCompile("Compile: ");
static const string   labelParse("Parse:   ");
static const string   labelInput("Input:   ");
static const string  labelSelect("Select:  ");
static const string     labelTry("Try:     ");
static const string     labelFit("Fitness: ");
static const string labelNoMatch("NoMatch: ");
static const string   labelMatch("Match:   ");
static const string  labelSyntax("Error:   ");
static const string    labelFail("Failure: ");
static const string   labelBlank("         ");

CommandLine::CommandLine(const string strName,
                         const string strPrompt,
                         bool         bUseRlLib,
                         bool         bIgnoreCase) :
    m_strName(strName),
    m_bIgnoreCase(bIgnoreCase),
    m_readline(strName, strPrompt, bUseRlLib),
    m_log("Command Line Log", 50)
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

  //
  // Split command syntax forms along newlines.
  //
  split(strMultiFormSyntax, '\n', syntaxForms);

  //
  // No syntax forms.
  //
  if( syntaxForms.size() == 0 || syntaxForms[0].empty() )
  {
    m_log << "No syntax forms specified." << eoe;
    LOGERROR("%s", getErrorStr().c_str());
    return NoUid;
  }

  //
  // New command.
  //
  CmdDef newdef;

  // set command's unique id
  newdef.setUid(m_nUidCnt++);

  uid = newdef.getUid();

  // add to map of commands 
  m_cmddefs[uid] = newdef;

  //
  // Add all syntax forms.
  //
  CmdDef &cmddef = cmdAt(uid);

  for(size_t i = 0; i < syntaxForms.size(); ++i)
  {
    CmdFormDef formdef(syntaxForms[i]);

    // add command form
    cmddef.pushForm(formdef);
  }

  m_bIsCompiled = false;

  return uid;
}

int CommandLine::removeCommand(const int uid)
{
  CmdIter pos;
  int     rc;

  if( uid == NoUid )
  {
    rc = EBadVal;
  }
  else if( (pos = m_cmddefs.find(uid)) == m_cmddefs.end() )
  {
    rc = EBadVal;
  }
  else
  {
    m_cmddefs.erase(pos);
    if( m_cmddefs.size() == 0 )
    {
      m_bIsCompiled = false;
    }
    m_log << "Command " << uid << " removed." << eoe;
    rc = AOk;
  }

  if( rc != AOk )
  {
    m_log << "No command with uid " << uid << " found." << eoe;
    LOGERROR("%s", getErrorStr().c_str());
  }

  return rc;
}

int CommandLine::removeCommand(const string &strCmd)
{
  CmdDef &cmddef = cmdAt(strCmd);

  return removeCommand(cmddef.getUid());
}

int CommandLine::removeAllCommands()
{
  m_cmddefs.clear();
  m_bIsCompiled = false;
  m_log << "All commands removed." << eoe;
  return AOk;
}

int CommandLine::compile()
{
  CmdIter   iter;
  int       rc = AOk;

  CL_DBG_CALL_IN("", endl);

  m_bIsCompiled = false;

  m_log.clear();
  m_log << bookmark(markExec) << labelExec << __func__ << eoe;

  //
  // No commands added.
  //
  if( m_nUidCnt == 0 )
  {
    m_log << labelFail << "No commands added." << eoe;
    LOGERROR("%s", getErrorStr().c_str());
    rc = EError;
  }

  //
  // Compile each command.
  //
  for(iter = m_cmddefs.begin();
      (rc == AOk) && (iter != m_cmddefs.end());
      ++iter)
  {
    CmdDef &cmddef = iter->second;

    if( !cmddef.isDefined() )
    {
      rc = compile(cmddef);
    }
  }

  //
  // Finalize compilation.
  //
  if( rc == AOk )
  {
    rc = finalize();
  }

  //
  // Build readline generator.
  //
  if( rc == AOk )
  {
    rc = rlBuildReadLineGenerator();
  }

  //
  // All good.
  //
  if( rc == AOk )
  {
    // register readline generator callback
    m_readline.registerAltGenerator(rlGeneratorWrapper, this);

    m_bIsCompiled = true;

    m_log.clear();
    m_log << bookmark(markExec) << labelExec << __func__ << eoe;
    m_log << labelCompile << "Compiled " << numOfCmds() << " commands." << eoe;
    m_log << labelExec << "Ok" << eoe;
  }

  CL_DBG_CALL_OUT_NL("rc=" << rc);

  return rc;
}

int CommandLine::compile(CmdDef &cmddef)
{
  TokenVec    tokens;
  ssize_t     tokcnt;
  int         i;
  int         rc;

  CL_DBG_CALL_IN("cmddef.uid=" << cmddef.getUid(), " ***" << endl);

  m_log.clear();
  m_log << bookmark(markExec) << labelExec << __func__ << eoe;
  m_log << labelCompile << "cmddef " << cmddef.getUid()
        << ": " << cmddef.getName() << eoe;

  // reset command definition to pre-compiled state
  cmddef.reset();

  for(i = 0, rc = AOk; (i < cmddef.numOfForms()) && (rc == AOk); ++i)
  {
    CmdFormDef &form = cmddef.formAt(i);

    if( (tokcnt = tokenizeSyntax(form.getSyntax(), tokens)) < 0 )
    {
      rc = tokcnt;
    }

    else if( tokcnt == 0 )
    {
      m_log << labelFail
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

  if( rc == AOk )
  {
    m_log << labelExec << "Ok" << eoe;
  }
  else
  {
    m_log << labelFail << "Could not compile." << eoe;
  }

  CL_DBG_CALL_OUT_NL("rc=" << rc);
  CL_DBG(endl);

  return rc;
}

int CommandLine::finalize()
{
  CmdIter iter, jter; // command definition iterators

  for(iter = m_cmddefs.begin(); iter != m_cmddefs.end(); ++iter)
  {
    CmdDef &cmddef_i = iter->second;
    
    jter = iter;

    for(++jter; jter != m_cmddefs.end(); ++jter)
    {
      CmdDef &cmddef_j = jter->second;

      if( cmddef_i.getName() == cmddef_j.getName() )
      {
        m_log << labelSyntax << "Duplicate command names found at uid's "
              << cmddef_i.getUid() << " and " << cmddef_j.getUid() << "."
              << eoe;
        LOGERROR("%s", getErrorStr().c_str());
        return EAmbigCmd;
      }
    }
  }

  return AOk;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Command Line Interface Methods
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int CommandLine::readCommand(FILE *fp, int &uid, int &iform, StringVec &argv)
{
  ExtArgVec extargv;
  int       rc;

  argv.clear();

  uid   = NoUid;
  iform = NoIndex;

  if( ((rc = readCommand(fp, extargv)) == AOk) && (extargv.size() > 0) )
  {
    uid   = extargv[0].uid();
    iform = extargv[0].formIndex();

    toVec(extargv, argv);
  }

  return rc;
}

int CommandLine::readCommand(FILE *fp, ExtArgVec &argv)
{
  string  strLine;
  int     rc;

  m_log.clear();
  m_log << bookmark(markExec) << labelExec << __func__ << eoe;

  argv.clear();

  //
  // Preliminary checks.
  //
  if( numOfCmds() == 0 )
  {
    m_log << labelNoExec << "No commands added to interface." << eoe;
    LOGERROR("%s", getErrorStr().c_str());
    return ENoExec;
  }
  else if( !isDefined() )
  {
    m_log << labelNoExec << "Commands not fully compiled." << eoe;
    LOGERROR("%s", getErrorStr().c_str());
    return ENoExec;
  }

  //
  // Read a line of input.
  //
  if( fileno(fp) == fileno(stdin) )
  {
    strLine = m_readline.rlreadLine();
  }
  else
  {
    strLine = m_readline.ireadLine(fp);
  }

  m_log << labelInput << "[" << getLineNum() << "] " << strLine << eoe;

  //
  // Process input, match to best command.
  //
  if( (rc = processInput(strLine, argv)) == AOk )
  {
    m_log << labelExec << "Ok" << eoe;

    if( argv.size() > 0 )
    {
      LOGDIAG3("Command %s(%d), form %d matched.",
        m_cmddefs[argv[0].uid()].getName(), argv[0].uid(), argv[0].formIndex());
    }
  }
  else
  {
    m_log << labelFail << "Input not matched to any command." << eoe;

    LOGDIAG3("No command matched to input.");
  }

  return rc;
}

void CommandLine::addToHistory(const StringVec &argv)
{
  m_readline.addToHistory(c14n(argv));
}

void CommandLine::addToHistory(const ExtArgVec &argv)
{
  StringVec v;

  toVec(argv, v);
  m_readline.addToHistory(c14n(v));
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

int CommandLine::processInput(const string &strLine,
                              ExtArgVec    &argv)
{
  TokenVec  tokens; // tokenized input arguments
  int       argc;   // argument count
  int       rc;     // return code

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
  else if( (argc = (int)tokenize(strLine, tokens)) < 0 )
  {
    // bad token - error string already set
    rc = EBadSyntax;
  }

  //
  // Line full of whitespace - again, find out why?
  //
  else if( argc == 0 )
  {
    rc = checkReadResult();
  }

  //
  // Otherwise match the input arguments to the best form of the best fit 
  // command definition.
  //
  else
  {
    rc = match(tokens, argv);
  }

  return rc;
}

int CommandLine::match(const TokenVec &tokens, ExtArgVec &argv)
{
  CmdIter   iter;             // command definition iterator

  double    fUltFitness;      // best fitness value
  int       uidUlt;           // best command unique id
  int       iformUlt;         // best form index

  double    fPenultFitness;   // second best fitness value
  int       uid2nd;           // seconds best command unique id
  int       iform2nd;         // second best form index

  ExtArgVec argvCmd;          // working command extended arguments
  double    fFitness;         // working fitness
  int       cnt;              // count of commands matching argv0
  int       rc;               // return code

  // This is considered a bug. 
  if( tokens.size() == 0 )
  {
    LOGERROR("Bug: No tokens.");
    return EError;
  }

  argvCmd.clear();

  fPenultFitness  = fUltFitness = 0.0;
  uid2nd          = uidUlt      = NoUid;
  iform2nd        = iformUlt    = NoIndex;

  cnt = 0;

  //
  // Find the best and second best command fits.
  //
  for(iter = m_cmddefs.begin(); iter != m_cmddefs.end(); ++iter)
  {
    rc = matchCommand(iter->second, tokens, argvCmd, fFitness);

    switch( rc )
    {
      case AOk:      // complete match
        ++cnt;
        break;
      case EArgv0:  // argv0 not matched
        break;
      default:      // argv0 matched, but args failed to match 
        ++cnt;
        continue;
    }

    // Found a new best.
    if( fFitness > fUltFitness )
    {
      fPenultFitness  = fUltFitness;
      uid2nd          = uidUlt;
      iform2nd        = iformUlt;

      argv        = argvCmd;
      fUltFitness = fFitness;
      uidUlt      = iter->second.getUid();
      iformUlt    = argv[0].formIndex();
    }

    // Found a new second best, but not a new best.
    else if( fFitness > fPenultFitness )
    {
      fPenultFitness  = fFitness;
      uid2nd          = iter->second.getUid();
      iform2nd        = argvCmd[0].formIndex();
    }
  }

  //
  // No interface match.
  //
  if( uidUlt == NoUid )
  {
    if( cnt > 0 )
    {
      m_log << labelNoMatch << c14n(tokens) << eoe;
    }
    else
    {
      m_log << labelSyntax << "Command " << tokens[0] << " not found." << eoe;
    }
    rc = EUnknownCmd;
  }

  //
  // Duplicate fitness.
  //
  else if( fUltFitness == fPenultFitness )
  {
    m_log << labelNoMatch
          << "Command '" << tokens[0] << "' ambiguous. Matches:"
          << eoe;
    m_log << labelBlank << cmdAt(uidUlt).formAt(iformUlt).getSyntax() << eoe;
    m_log << labelBlank << cmdAt(uid2nd).formAt(iform2nd).getSyntax() << eoe;
    rc = EAmbigCmd;
  }

  //
  // Got a unambiguous, matched command.
  //
  else
  {
    m_log << labelSelect << cmdAt(uidUlt).formAt(iformUlt).getSyntax() << eoe;
    rc = AOk;
  }

  return rc;
}

int CommandLine::matchCommand(const CmdDef   &cmddef,
                              const TokenVec &tokens,
                              ExtArgVec      &argv,
                              double         &fFitness)
{
  ExtArgVec argvForm;     // working form extended arguments
  double    fMaxFitness;  // working and best form fitness
  int       iBest;        // best form index
  int       i;            // working index
  int       rc;           // return code

  // This is considered a bug. 
  if( tokens.size() == 0 )
  {
    LOGERROR("Bug: No tokens.");
    return EError;
  }

  argv.clear();

  fFitness    = 0.0;
  fMaxFitness = 0.0;
  iBest       = NoIndex;

  //
  // If argv0 doesn't match, don't try to apply matching algorithm to this
  // command.
  //
  // Note: All command forms must have the same argv0 argument type and value.
  //
  if( cmddef.at(0).at(0).match(tokens[0].value()) == 0.0 )
  {
    return EArgv0;
  }

  for(i = 0; i < cmddef.numOfForms(); ++i)
  {
    const CmdFormDef &form = cmddef.at(i);

    m_log << labelTry << form.getSyntax() << eoe;

    if( (rc = matchCommandForm(form, tokens, argvForm, fFitness)) == AOk )
    {
      if( fFitness > fMaxFitness )
      {
        argv        = argvForm;
        fMaxFitness = fFitness;
        iBest       = i;
      }
    }

    m_log << labelFit << fFitness << eoe;

    if( fMaxFitness >= 1.0 )
    {
      break;
    }
  }

  //
  // Found the best command form that matches the input.
  //
  if( iBest >= 0 )
  {
    m_log << labelMatch
          << "Command '" << cmddef.getName()
          << "', form " << iBest << ": Fitness " << fMaxFitness
          << eoe;
    rc = AOk;
  }

  //
  // No fit.
  //
  else
  {
    m_log << labelNoMatch
          << "Command '" << cmddef.getName()
          << "', all forms."
          << eoe;
  }

  return rc;
}

int CommandLine::matchCommandForm(const CmdFormDef &form,
                                  const TokenVec   &tokens,
                                  ExtArgVec        &argv,
                                  double           &fFitness)
{
  int     argcToks = (int)tokens.size();  // number of input tokens
  int     argcForm = form.numOfArgs();    // form total number of arguments
  double  fWeight, fDecay;                // match weight and decay multiplier
  int     iArg;                           // working form argument index
  int     iTok;                           // working input tokens index
  int     rc;                             // return code

  argv.clear();

  fFitness = 0.0;
  fDecay   = 1.0;

  // command name (for logging)
  const string &strCmdName = m_cmddefs[form.getParentCmdUid()].getName();

  // 
  // Too many input arguments.
  //
  if( argcToks > argcForm )
  {
    m_log << labelNoMatch
          << "Command '" << strCmdName << "', "
          << "form " << form.getIndex() << ": "
          << "Too many arguments: "
          << argcToks << " specified, "
          << argcForm << " maximum."
          << eoe;
    return EBadSyntax;
  }

  //
  // Missing required input arguments.
  //
  else if( argcToks < form.numOfRequiredArgs() )
  {
    m_log << labelNoMatch
          << "Command '" << strCmdName << "', "
          << "form " << form.getIndex() << ": "
          << "Missing required arguments: "
          << argcToks << " specified, "
          << form.numOfRequiredArgs() << " required."
          << eoe;
    return EBadSyntax;
  }

  //
  // Match input arguments to form syntax.
  //
  for(iArg = 0, iTok = 0, rc = AOk;
      (iArg < argcForm) && (iTok < argcToks) && (rc == AOk);
      ++iArg)
  {
    const CmdArgDef &argdef = form.at(iArg);

    // try to match input token to argument sytax
    fWeight = argdef.match(tokens[iTok].value(), m_bIgnoreCase);

    // no match
    if( fWeight == 0.0 )
    {
      rc = EBadSyntax;

      // start log entry
      m_log << labelNoMatch
            << "Input argument " << tokens[iTok] << " doesn't match "
            << "command '" << strCmdName << "', "
            << "form " << form.getIndex() << ", "
            << (argdef.isOptional()? "(opt)": "")
            << "arg " << argdef.getIndex()
            << " definition."
            << eoe;
    
      //
      // Log reason of match failure.
      //
      switch( argdef.getType() )
      {
        case CmdArgDef::ArgTypeLiteral:
          m_log << labelBlank << "  Not one of: "
                << argdef.constructLiteralList() << "."
                << eoe;
        break;

        case CmdArgDef::ArgTypeWord:
        case CmdArgDef::ArgTypeMultiWord:
        case CmdArgDef::ArgTypeIdentifier:
        case CmdArgDef::ArgTypeBoolean:
        case CmdArgDef::ArgTypeFile:
          m_log << labelBlank << "  Not a "
                << "'" << lookupArgSymbol(argdef.getType()) << "' type."
                << eoe;
          break;

        case CmdArgDef::ArgTypeInteger:
        case CmdArgDef::ArgTypeFpn:
          m_log << labelBlank << "  Not a "
                << "'" << lookupArgSymbol(argdef.getType()) << "' type";
          if( argdef.getRanges().size() > 0 )
          {
            m_log << " or out of range " << argdef.constructRangeList();
          }
          m_log << "." << eoe;
          break;

        case CmdArgDef::ArgTypeRegEx:
          m_log << labelBlank << "  Failed to match re: "
                << "'" << argdef.getRegEx() << "'."
                << eoe;
          break;

        case CmdArgDef::ArgTypeUndef:
        default:
          LOGERROR("Bug: Unknown type %d.", argdef.getType());
          rc = EError;
          break;
      }
    }

    //
    // Good argument match.
    //
    if( rc == AOk )
    {
      m_log << labelMatch
            << "Input argument " << tokens[iTok] << " matches "
            << "cmd '" << strCmdName << "', "
            << "form " << form.getIndex() << ", "
            << "arg " << argdef.getIndex() << " "
            << "definition."
            << eoe;

      argv.push_back(argdef.convert(tokens[iTok].value(), m_bIgnoreCase));

      // don't decay perfection
      if( fWeight == 1.0 )
      {
        fFitness += fWeight;
      }
      else
      {
        fFitness += fWeight * fDecay;
      }

      fDecay *= 0.9;

      ++iTok;
    }

    //
    // Bad match, but argument is optional, and there are more optionals.
    //
    else if( argdef.isOptional() && (iArg < argcForm-1) )
    {
      rc = AOk;
    }

    //
    // Bad match.
    //
    else
    {
      fFitness = 0.0;
    }
  }

  // normalize fitness
  fFitness /= (double)argcToks;

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
    return AOk;
  }
}

void CommandLine::toVec(const ExtArgVec &v1, StringVec &v2)
{
  for(size_t i = 0; i < v1.size(); ++i)
  {
    v2.push_back(v1[i].arg());
  }
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Command Info and Argument Access Methods
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int CommandLine::numOfArgs(int uid, int iform) const
{
  return at(uid).at(iform).numOfArgs();
}

int CommandLine::numOfArgs(const ExtArg &arg) const
{
  return at(arg.uid()).at(arg.formIndex()).numOfArgs();
}

int CommandLine::numOfRequiredArgs(int uid, int iform) const
{
  return at(uid).at(iform).numOfRequiredArgs();
}

int CommandLine::numOfRequiredArgs(const ExtArg &arg) const
{
  return at(arg.uid()).at(arg.formIndex()).numOfRequiredArgs();
}

int CommandLine::numOfOptionalArgs(int uid, int iform) const
{
  return at(uid).at(iform).numOfOptionalArgs();
}

int CommandLine::numOfOptionalArgs(const ExtArg &arg) const
{
  return at(arg.uid()).at(arg.formIndex()).numOfOptionalArgs();
}

const string &CommandLine::getArgName(const ExtArg &arg) const
{
  // all terrain armored transport, of course
  const CmdArgDef &argdef= at(arg.uid()).at(arg.formIndex()).at(arg.argIndex());

  return argdef.getName();
}

CmdArgDef::ArgType CommandLine::getArgDefType(const ExtArg &arg) const
{
  const CmdArgDef &argdef= at(arg.uid()).at(arg.formIndex()).at(arg.argIndex());

  return argdef.getType();
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// ReadLine Generator Methods
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int CommandLine::rlBuildReadLineGenerator()
{
  return AOk;
}

const string CommandLine::rlGeneratorWrapper(void         *pAppArg,
                                             const string &strText,
                                             int           nIndex,
                                             const string &strContext,
                                             int           nStart,
                                             int           nEnd,
                                             unsigned     &uFlags)
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
                                      int           nIndex,
                                      const string &strContext,
                                      int           nStart,
                                      int           nEnd,
                                      unsigned     &uFlags)
{
  static vector<CmdArgDef*> argdefs;

  StringVec tabList;
  size_t    i;

  //cerr  << "{text=" << strText
  //      << ", index=" << nIndex
  //      << ", context=" << strContext
  //      << ", start=" << nStart
  //      << ", end=" << nEnd
  //      << "}" << endl;

  if( nIndex == 0 )
  {
    //cerr << "rebuild args" << endl;
    argdefs.clear();
    rlArgDefs(strContext.substr(0, nStart), argdefs);
  }

  rlTabList(strText, argdefs, tabList, uFlags);

  if( nIndex < tabList.size() )
  {
    return tabList[nIndex];
  }
  else
  {
    return RlTabEnd;
  }
}

void CommandLine::rlArgDefs(const string       &strSubtext,
                            vector<CmdArgDef*> &argdefs)
{
  CmdIter             iter;         // command definition iterator
  vector<CmdArgDef*>  v[2];         // work swap vectors of argument definitions
  int                 tog0, tog1;   // vector toggle source/destination indices
  StringVec           tokens;       // input tokens
  size_t              argc;         // token working argument count
  size_t              i, j;         // working indices

  argdefs.clear();

  tokenize(strSubtext, tokens);
    
  tog0 = 0, tog1 = 1;

  //
  // Seed with argv0 (command).
  //
  for(iter = m_cmddefs.begin(); iter != m_cmddefs.end(); ++iter)
  {
    for(j = 0; j < iter->second.numOfForms(); ++j)
    {
      CmdFormDef &form = iter->second.formAt(j);
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

      if( p->match(tokens[argc], m_bIgnoreCase) > 0.0 )
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
  size_t                len;          // length of input text
  set<string>           A;            // set with unique keys == values
  set<string>::iterator iter, jter;   // bidirectional set iterators
  string                str;          // working string
  size_t                i, j, min;    // working indices, minimum

  // clear TAB list of all existing candidates
  tabList.clear();

  // no filename completion
  uFlags = ReadLine::FlagTabNoFilename;

  // nothing to TAB complete
  if( argdefs.size() == 0 )
  {
    return;
  }

  // length of partial text to complete
  len = strText.size();

  //
  // Build tab completion super list.
  //
  // Note: Sets automatically handle duplicates.
  //
  for(i = 0; i < argdefs.size(); ++i)
  {
    switch( argdefs[i]->getType() )
    {
      // Find all literals that partially match text.
      case CmdArgDef::ArgTypeLiteral:
        for(j = 0; j < argdefs[i]->numOfLiterals(); ++j)
        {
          str = argdefs[i]->literalAt(j);

          if( rlPartialMatch(strText, str, len) )
          {
            A.insert(str);
          }
        }
        break;

      case CmdArgDef::ArgTypeBoolean:
        for(j = 0; FalseHood[j] != NULL; ++j)
        {
          if( rlPartialMatch(strText, FalseHood[j], len) )
          {
            // bit of hack. don't want 'false' and 'f' in TAB complition list
            if( FalseHood[j] != "f" )
            {
              A.insert(FalseHood[j]);
            }
          }
        }
        for(j = 0; TruthHood[j] != NULL; ++j)
        {
          if( rlPartialMatch(strText, TruthHood[j], len) )
          {
            // bit of hack. don't want 'true' and 't' in TAB complition list
            if( TruthHood[j] != "t" )
            {
              A.insert(TruthHood[j]);
            }
          }
        }
        break;

      // Argument types not posible to TAB complete, provide syntax if no text.
      case CmdArgDef::ArgTypeWord:
      case CmdArgDef::ArgTypeMultiWord:
      case CmdArgDef::ArgTypeIdentifier:
      case CmdArgDef::ArgTypeInteger:
      case CmdArgDef::ArgTypeFpn:
      case CmdArgDef::ArgTypeRegEx:
        if( len == 0 )
        {
          A.insert(argdefs[i]->constructSyntax());
          uFlags |= ReadLine::FlagTabNoDefault;
        }
        break;

      // Filename argument type, allow filename completion.
      case CmdArgDef::ArgTypeFile:
        uFlags &= ~ReadLine::FlagTabNoFilename;
        break;

      // Unknown.
      case CmdArgDef::ArgTypeUndef:
      default:
        break;
    }
  } 

  //
  // Find the greatest common prefix substring.
  //
  if( !(uFlags & ReadLine::FlagTabNoDefault) && (A.size() > 1) )
  {
    jter = A.begin();
    iter = jter++;

    size_t  min = iter->size();

    for(; (jter != A.end()) && (min > 0); ++iter, ++jter)
    {
      j = gcss(*iter, *jter);

      if( j < min )
      {
        min = j;
      }
    }

    // Common substring is longer than text. This is the only default TAB entry.
    if( min > len )
    {
      str = *A.begin();
      A.clear();
      A.insert(str.substr(0, min));
      uFlags |= ReadLine::FlagTabNoSpace;
    }
  }

  //
  // Copy set to vector list
  //
  for(iter = A.begin(); iter != A.end(); ++iter)
  {
    tabList.push_back(*iter);
  }

  // Disable default if multiple unique matches.
  if( tabList.size() > 1 )
  {
    uFlags |= ReadLine::FlagTabNoDefault;
  }
}

bool CommandLine::rlPartialMatch(const string &strText,
                                 const string strLiteral,
                                 size_t       uLen)
{
  if( m_bIgnoreCase )
  {
    return lowercase(strText) == lowercase(strLiteral.substr(0, uLen));
  }
  else
  {
    return strText == strLiteral.substr(0, uLen);
  }
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Attribute and Data Access Methods
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

bool CommandLine::hasCmd(const int uid) const
{
  return m_cmddefs.find(uid) != m_cmddefs.end();
}

bool CommandLine::hasCmd(const string &strName) const
{
  return at(strName).isDefined();
}

const CmdDef &CommandLine::at(const int uid) const
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

const CmdDef &CommandLine::at(const string &strName) const
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
  TokenVec  tokenz;
  ssize_t   cnt;
  ssize_t   i;

  tokens.clear();

  cnt = tokenize(strInput, tokenz);

  for(i = 0; i < cnt; ++i)
  {
    tokens.push_back(tokenz[i].value());
  }

  return cnt;
}

ssize_t CommandLine::tokenize(const string &strInput, TokenVec &tokens)
{
  size_t    len;      // length of input string
  ssize_t   cursor;   // cursor character position along input string
  size_t    start;    // start character position of token in input string

  CL_DBG_CALL_IN("\"" << strInput << "\"", endl);

  tokens.clear();

  len = strInput.length();

  for(cursor = 0; (cursor >= 0) && (cursor < len); )
  {
    // find start of the next token
    while( (cursor < len) && isspace((int)strInput[cursor]) )
    {
      ++cursor;
    }

    // no more tokens
    if( cursor >= len ) 
    {
      break;
    }

    // new qouted string token
    if( isdquote(strInput[cursor]) )
    {
      cursor = lexQuotedString(strInput, cursor, tokens);
    }

    // new contiguous word token
    else
    {
      cursor = lexWord(strInput, cursor, tokens);
    }
  }

#ifdef CL_ENABLE_DEBUG
  {
    stringstream ss;
    string       sep;
    for(size_t i = 0; i < tokens.size(); ++i)
    {
      ss << sep << tokens[i];
      sep = ", ";
    }
    CL_DBG_CALL_OUT_NL(ss.str());
  }
#endif // CL_ENABLE_DEBUG

  return (ssize_t)tokens.size();
}

string CommandLine::extractArgv0(const string &strSyntax)
{
  TokenVec  tokens;
  size_t    pos = 0;

  tokenize(strSyntax, tokens);

  if( (tokens.size() > 0) && tokIdentifier(tokens, pos) )
  {
    return tokens[0].value();
  }

  else
  {
    return emptystring;
  }
}

ssize_t CommandLine::tokenizeSyntax(const string &strSyntax, TokenVec &tokens)
{
  size_t    len;      // length of syntax string
  ssize_t   cursor;   // cursor character position along syntax string
  size_t    start;    // start character position of token in syntax string

  CL_DBG_CALL_IN("\"" << strSyntax << "\", tokens", endl);

  tokens.clear();

  len = strSyntax.length();

  for(cursor = 0; (cursor >= 0) && (cursor < len); )
  {
    // find start of the next token
    while( (cursor < len) && isspace((int)strSyntax[cursor]) )
    {
      ++cursor;
    }

    // no more tokens
    if( cursor >= len ) 
    {
      break;
    }

    // new parenthetical expression
    if( isoparen(strSyntax[cursor]) )
    {
      cursor = lexSyntaxParenExpr(strSyntax, cursor, tokens);
    }

    //  special character token
    else if( isspecial(strSyntax[cursor]) )
    {
      start = cursor++;
      pushToken(strSyntax, start, cursor, tokens);
    }

    // new contiguous word token
    else
    {
      cursor = lexSyntaxWord(strSyntax, cursor, tokens);
    }
  }

#ifdef CL_ENABLE_DEBUG
  {
    stringstream ss;
    string       sep;
    for(size_t i = 0; i < tokens.size(); ++i)
    {
      ss << sep << tokens[i];
      sep = ", ";
    }
    CL_DBG_CALL_OUT_NL(ss.str());
  }
#endif // CL_ENABLE_DEBUG

  return (ssize_t)tokens.size();
}

ssize_t CommandLine::lexSyntaxWord(const string &strSyntax,
                                   ssize_t       cursor,
                                   TokenVec     &tokens)
{
  size_t    len;
  size_t    start;

  len   = strSyntax.length();
  start = cursor;

  // find end of word
  while( (cursor < len) &&
         !isspace((int)strSyntax[cursor]) &&
         !isspecial(strSyntax[cursor]) )
  {
    ++cursor;
  }

  if( cursor > start )
  {
    pushToken(strSyntax, start, cursor, tokens);
  }
  else
  {
    m_log << labelSyntax << "No <word> found." << eoe;
    LOGERROR("%s", getErrorStr().c_str());
    cursor = EBadSyntax;
  }

  return cursor;
}

ssize_t CommandLine::lexSyntaxParenExpr(const string &strSyntax,
                                        ssize_t       cursor,
                                        TokenVec     &tokens)
{
  size_t    len;      // length of input string;
  size_t    start;    // start character position of token in input string
  int       pdepth;   // parenthesis depth
  bool      escseq;   // is [not] in an escape sequence
  bool      eop;      // is [not] at end of parenthetical
  char      c;        // working character
  string    value;    // working token value

  len     = strSyntax.length();
  start   = cursor;
  pdepth  = 0;
  escseq  = false;
  eop     = false;

  if( isoparen(strSyntax[cursor]) )
  {
    // '(' token
    start = cursor++;
    pushToken(strSyntax, start, cursor, tokens);

    start = cursor;
    ++pdepth;
  }
  else
  {
    m_log << labelSyntax << "No starting '(' found." << eoe;
    LOGERROR("%s", getErrorStr().c_str());
    return EBadSyntax;
  }

  while( (cursor >= 0) && (cursor < len) && !eop )
  {
    switch( strSyntax[cursor] )
    {
      case '\\':
        escseq = true;
        break;
      case '(':
        if( !escseq )
        {
          ++pdepth;
        }
        escseq = false;
        break;
      case ')':
        if( !escseq )
        {
          if( --pdepth == 0 )
          {
            eop = true;
          }
        }
        escseq = false;
        break;
      default:
        escseq = false;
        break;
    }

    if( !eop )
    {
      value.push_back(strSyntax[cursor++]);
    }
  }

  if( iscparen(strSyntax[cursor]) )
  {
    // expression token
    pushToken(strSyntax, start, cursor, tokens);

    // ')' token
    start = cursor++;
    pushToken(strSyntax, start, cursor, tokens);

    return cursor;
  }
  else
  {
    m_log << labelSyntax << "No ending '(' found." << eoe;
    LOGERROR("%s", getErrorStr().c_str());
    return EBadSyntax;
  }
}

ssize_t CommandLine::lexWord(const string &strInput,
                             ssize_t       cursor,
                             TokenVec     &tokens)
{
  size_t    len;
  size_t    start;

  len   = strInput.length();
  start = cursor;

  while( (cursor < len) && !isspace((int)strInput[cursor]) )
  {
    ++cursor;
  }

  if( cursor > start )
  {
    pushToken(strInput, start, cursor, tokens);
  }
  else
  {
    m_log << labelSyntax << "No <word> found." << eoe;
    cursor = EBadSyntax;
  }

  return cursor;
}

ssize_t CommandLine::lexQuotedString(const string &strInput,
                                     ssize_t       cursor,
                                     TokenVec     &tokens)
{
  size_t    len;      // length of input string;
  size_t    start;    // start character position of token in input string
  bool      escseq;   // is [not] in an escape sequence
  bool      eos;      // is [not] at end of string
  char      c;        // working converted character
  string    value;    // working token value

  len   = strInput.length();

  if( isdquote(strInput[cursor]) )
  {
    ++cursor;
    start = cursor;
  }
  else
  {
    m_log << labelSyntax << "No starting double qoute '\"' found." << eoe;
    return EBadSyntax;
  }

  escseq  = false;
  eos     = false;

  while( (cursor >= 0) && (cursor < len) && !eos )
  {
    // escape sequence
    if( escseq )
    {
      switch( strInput[cursor] )
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
            int i = cursor + 1;
            if( (i < len) && isxdigit(strInput[i]) )
            {
              c = tohex(strInput[i]);
              ++cursor;
              ++i;
              if( (i < len) && isxdigit(strInput[i]) )
              {
                c <<= 4;
                c |= tohex(strInput[i]);
                ++cursor;
              }
            }
            else
            {
              c = 'x';
            }
          }
          break;
        default:
          c = strInput[cursor];
          break;
      }

      value.push_back(c);
      escseq = false;
      ++cursor;
    }

    else
    {
      switch( strInput[cursor] )
      {
        case '\\':
          escseq = true;
          break;
        case '"':
          eos = true;
          break;
        default:
          value.push_back(strInput[cursor++]);
          break;
      }
    }
  }

  if( isdquote(strInput[cursor]) )
  {
    pushToken(strInput, start, cursor, tokens);

    ++cursor;   // get past quote

    return cursor;
  }
  else
  {
    m_log << labelSyntax << "No ending double qoute '\"' found." << eoe;
    return EBadSyntax;
  }
}

void CommandLine::pushToken(const string &strSource,
                            size_t        start,
                            ssize_t       cursor,
                            TokenVec     &tokens)
{
  Token tok(strSource.substr(start, cursor-start), start, cursor-1);
  tokens.push_back(tok);
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Extended Usage Syntax Parsing Methods
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int CommandLine::parseSyntax(CmdDef         &cmddef,
                             CmdFormDef     &form,
                             const TokenVec &tokens)
{
  size_t  tokcnt  = tokens.size();
  size_t  pos     = 0;
  bool    bOk     = true;
  int     rc;

  CL_DBG_PARSE_CALL_IN(cmddef, form, tokens, 0, endl);

  m_log << labelParse
        << "cmddef " << cmddef.getUid() << ", "
        << "form " << form.getIndex() << ": "
        << form.getSyntax()
        << eoe;

  // command
  if( !parseArgv0(cmddef, form, tokens, pos) )
  {
    m_log << labelFail << "Parssing special argv0." << eoe;
    bOk = false;
  }

  // required argmuents - may be none
  else if( !parseRequiredArgList(cmddef, form, tokens, pos) )
  {
    m_log << labelFail << "Parsing required argument list." << eoe;
    bOk = false;
  }

  // optional arguments - may be none
  else if( !parseOptionalArgList(cmddef, form, tokens, pos) )
  {
    m_log << labelFail << "Parsing optional argument list." << eoe;
    bOk = false;
  }

  // final checks
  else if( pos < tokcnt )
  {
    m_log << labelFail
          << "Extraneous tokens found after optional arguments."
          << eoe;
    bOk = false;
  }

  // all good
  if( bOk )
  {
    m_log << labelParse << "Ok" << eoe;
    LOGDIAG2("Command(uid=%d, name=%s, form=%d, argc=%d) successfully parsed.",
        cmddef.getUid(), cmddef.getName().c_str(),
        form.getIndex(), form.numOfArgs());
    rc = AOk;
  }

  // not so good - log error
  else
  {
    LOGERROR("%s", getErrorStr().c_str());
    rc = EBadSyntax;
  }

  CL_DBG_CALL_OUT_NL("rc=" << rc << ", " << okstr(bOk) << ", pos=" << pos);

  return rc;
}

bool CommandLine::parseArgv0(CmdDef         &cmddef,
                             CmdFormDef     &form,
                             const TokenVec &tokens,
                             size_t         &pos)
{
  bool    bOk;
  string  strName;

  CL_DBG_PARSE_CALL_IN(cmddef, form, tokens, pos, endl);

  if( pos < tokens.size() )
  {
    if( peekEq("<", tokens[pos]) )
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
    m_log << labelSyntax << "Command argument not found in form "
          << form.getIndex() << "."
          << eoe;
    LOGERROR("%s", getErrorStr().c_str());
    bOk = false;
  }

  if( bOk )
  {
    form.m_nArgcReq++;

    form.lastArg().orFlags(CmdArgDef::FlagCommand);

    // set command's name
    if( form.lastArg().getType() == CmdArgDef::ArgTypeLiteral )
    {
      // literals are anonymous, use the literal itself
      strName = form.lastArg().literalAt(0);
    }
    else
    {
      // variables have required names
      strName = form.lastArg().getName();
    }

    // assign command name
    if( cmddef.getName().empty() )
    {
      cmddef.setName(strName);
    }
    // all forms must have the same command name
    else if( cmddef.getName() != strName )
    {
      m_log << labelSyntax << "Expected command name '" << cmddef.getName()
            << "', but found '" << strName << "' in form " << form.getIndex()
            << "."
            << eoe;
      LOGERROR("%s", getErrorStr().c_str());
      bOk = false;
    }
  }

  CL_DBG_CALL_OUT_NL(okstr(bOk) << ", pos=" << pos);

  return bOk;
}

bool CommandLine::parseRequiredArgList(CmdDef         &cmddef,
                                       CmdFormDef     &form,
                                       const TokenVec &tokens,
                                       size_t         &pos)
{
  bool  bOk = true;

  CL_DBG_PARSE_CALL_IN(cmddef, form, tokens, pos, endl);

  while( bOk && (pos < tokens.size()) )
  {
    // peek if start of optional arguments
    if( peekEq("[", tokens[pos]) )
    {
      break;
    }
    
    // required argument
    else if( (bOk = parseArg(cmddef, form, tokens, pos)) )
    {
      form.m_nArgcReq++;
    }
  }

  CL_DBG_CALL_OUT_NL(okstr(bOk) << ", pos=" << pos);

  return bOk;
}

bool CommandLine::parseOptionalArgList(CmdDef         &cmddef,
                                       CmdFormDef     &form,
                                       const TokenVec &tokens,
                                       size_t         &pos)
{
  bool  bOk = true;

  CL_DBG_PARSE_CALL_IN(cmddef, form, tokens, pos, endl);

  while( bOk && (pos < tokens.size()) )
  {
    // peek if start an optional arguments
    if( !peekEq("[", tokens[pos]) )
    {
      break;
    }
    
    // "[ argument ]"
    if( tokEq("[", tokens, pos) &&
        parseArg(cmddef, form, tokens, pos) &&
        tokEq("]", tokens, pos) )
    {
      form.lastArg().orFlags(CmdArgDef::FlagOptional);
      form.m_nArgcOpt++;
    }
    else
    {
      bOk = false;
    }
  }

  CL_DBG_CALL_OUT_NL(okstr(bOk) << ", pos=" << pos);

  return bOk;
}

bool CommandLine::parseArg(CmdDef         &cmddef,
                           CmdFormDef     &form,
                           const TokenVec &tokens,
                           size_t         &pos)
{
  bool  bOk;

  CL_DBG_PARSE_CALL_IN(cmddef, form, tokens, pos, endl);

  if( pos < tokens.size() )
  {
    if( peekEq("{", tokens[pos]) )
    {
      bOk = parseXorListArg(cmddef, form, tokens, pos);
    }
    else if( peekEq("<", tokens[pos]) )
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
    m_log << labelSyntax << "Command argument not found." << eoe;
    LOGERROR("%s", getErrorStr().c_str());
    bOk = false;
  }

  CL_DBG_CALL_OUT_NL(okstr(bOk) << ", pos=" << pos);

  return bOk;
}

bool CommandLine::parseXorListArg(CmdDef         &cmddef,
                                  CmdFormDef     &form,
                                  const TokenVec &tokens,
                                  size_t         &pos)
{
  bool      bOk;
  CmdArgDef argdef;
  StringVec literals;

  CL_DBG_PARSE_CALL_IN(cmddef, form, tokens, pos, endl);

  form.pushArg(argdef);

  bOk = tokEq("{", tokens, pos) &&
        parseXorList(cmddef, form, tokens, pos, literals) &&
        tokEq("}", tokens, pos);

  if( bOk )
  {
    stringstream  ss;
    int           n = form.numOfArgs() - 1;

    ss << "_" << n;

    form.lastArg().setName(ss.str());
    form.lastArg().setType(CmdArgDef::ArgTypeLiteral);
    form.lastArg().orFlags(CmdArgDef::FlagXorList);

    for(size_t i = 0; i < literals.size(); ++i)
    {
      form.lastArg().addLiteralValue(literals[i]);
    }
  }

  CL_DBG_CALL_OUT_NL(okstr(bOk) << ", pos=" << pos);

  return bOk;
}

bool CommandLine::parseVariableArg(CmdDef         &cmddef,
                                   CmdFormDef     &form,
                                   const TokenVec &tokens,
                                   size_t         &pos)
{
  bool                bOk;
  CmdArgDef           argdef;
  string              strName;
  CmdArgDef::ArgType  eType;
  CmdArgDef::RangeVec ranges;
  RegEx               re;

  CL_DBG_PARSE_CALL_IN(cmddef, form, tokens, pos, endl);

  form.pushArg(argdef);

  bOk = tokEq("<", tokens, pos) &&
        parseIdentifier(cmddef, form, tokens, pos, strName);

  if( bOk )
  {
    // variable modifier
    if( peekEq(":", tokens[pos]) )
    {
      bOk = tokEq(":", tokens, pos) &&
            parseVarMod(cmddef, form, tokens, pos, eType, ranges, re);
    }
    // default
    else
    {
      eType = CmdArgDef::ArgTypeWord;
    }
  }

  bOk = bOk && tokEq(">", tokens, pos);

  if( bOk )
  {
    form.lastArg().setName(strName);
    form.lastArg().setType(eType);
    form.lastArg().setRanges(ranges);
    form.lastArg().setRegEx(re);
  }

  CL_DBG_CALL_OUT_NL(okstr(bOk) << ", pos=" << pos);

  return bOk;
}

bool CommandLine::parseLiteralArg(CmdDef         &cmddef,
                                  CmdFormDef     &form,
                                  const TokenVec &tokens,
                                  size_t         &pos)
{
  bool      bOk;
  CmdArgDef argdef;
  string    strValue;

  CL_DBG_PARSE_CALL_IN(cmddef, form, tokens, pos, endl);

  form.pushArg(argdef);

  bOk = parseLiteralValue(cmddef, form, tokens, pos, strValue);

  if( bOk )
  {
    stringstream  ss;
    int           n = form.numOfArgs() - 1;

    ss << "_" << n;

    form.lastArg().setName(ss.str());
    form.lastArg().setType(CmdArgDef::ArgTypeLiteral);
    form.lastArg().addLiteralValue(strValue);
  }

  CL_DBG_CALL_OUT_NL(okstr(bOk) << ", pos=" << pos);

  return bOk;
}

bool CommandLine::parseXorList(CmdDef         &cmddef,
                               CmdFormDef     &form,
                               const TokenVec &tokens,
                               size_t         &pos,
                               StringVec      &literals)
{
  bool    bOk   = true;
  bool    more = true;
  string  strValue;

  CL_DBG_PARSE_CALL_IN(cmddef, form, tokens, pos, endl);

  while( more && bOk && (pos < tokens.size()) )
  {
    if( peekEq("}", tokens[pos]) )
    {
      m_log << labelSyntax << "Expected literal value but found '}'." << eoe;
      LOGERROR("%s", getErrorStr().c_str());
      bOk = false;
    }

    else if( (bOk = parseLiteralValue(cmddef, form, tokens, pos, strValue)) )
    {
      literals.push_back(strValue);

      // peek if start of another xor option
      if( peekEq("|", tokens[pos]) )
      {
        bOk = tokEq("|", tokens, pos);
      }
      else
      {
        more = false;
      }
    }
  }

  if( more && bOk )
  {
    m_log << labelSyntax << "No literal value found." << eoe;
    LOGERROR("%s", getErrorStr().c_str());
    bOk = false;
  }

  CL_DBG_CALL_OUT_NL(okstr(bOk) << ", pos=" << pos);

  return bOk;
}

bool CommandLine::parseIdentifier(CmdDef         &cmddef,
                                  CmdFormDef     &form,
                                  const TokenVec &tokens,
                                  size_t         &pos,
                                  string         &strIdent)
{
  bool    bOk;

  CL_DBG_PARSE_CALL_IN(cmddef, form, tokens, pos, endl);

  strIdent.clear();

  if( pos < tokens.size() )
  {
    strIdent = tokens[pos].value();
    bOk = tokIdentifier(tokens, pos);
  }
  else
  {
    m_log << labelSyntax << "No identifier found." << eoe;
    LOGERROR("%s", getErrorStr().c_str());
    bOk = false;
  }

  CL_DBG_CALL_OUT_NL(okstr(bOk) << ", identifier='" << strIdent
      << "', pos=" << pos);

  return bOk;
}

bool CommandLine::parseVarMod(CmdDef              &cmddef,
                              CmdFormDef          &form,
                              const TokenVec      &tokens,
                              size_t              &pos,
                              CmdArgDef::ArgType  &eType,
                              CmdArgDef::RangeVec &ranges,
                              RegEx               &re)
{
  bool  bOk;

  CL_DBG_PARSE_CALL_IN(cmddef, form, tokens, pos, endl);

  bOk = parseVarType(cmddef, form, tokens, pos, eType);

  if( bOk )
  {
    if( peekEq("(", tokens[pos]) )
    {
      switch( eType )
      {
        case CmdArgDef::ArgTypeInteger:
        case CmdArgDef::ArgTypeFpn:
          bOk = tokEq("(", tokens, pos) &&
                parseVarRangeExpr(cmddef, form, tokens, pos, ranges) &&
                tokEq(")", tokens, pos);
          break;
        case CmdArgDef::ArgTypeRegEx:
          bOk = tokEq("(", tokens, pos) &&
                parseVarRegExpr(cmddef, form, tokens, pos, re) &&
                tokEq(")", tokens, pos);
          break;
        default:
          m_log << labelSyntax
                << "Unexpected '(' token found for argument type "
                << "'" << lookupArgSymbol(eType) << "'."
                << eoe;
          LOGERROR("%s", getErrorStr().c_str());
          bOk = false;
      }
    }
  }

  CL_DBG_CALL_OUT_NL(okstr(bOk) << ", type=" << eType << ", pos=" << pos);

  return bOk;
}

bool CommandLine::parseVarType(CmdDef             &cmddef,
                               CmdFormDef         &form,
                               const TokenVec     &tokens,
                               size_t             &pos,
                               CmdArgDef::ArgType &eType)
{
  bool  bOk;

  CL_DBG_PARSE_CALL_IN(cmddef, form, tokens, pos, " ");

  eType = CmdArgDef::ArgTypeUndef;

  if( pos < tokens.size() )
  {
    if((eType = lookupArgType(tokens[pos].value())) != CmdArgDef::ArgTypeUndef)
    {
      ++pos;
      bOk = true;
    }
    else
    {
      m_log << labelSyntax << "Unknown variable type '" << tokens[pos].value()
            << "'." << eoe;
      LOGERROR("%s", getErrorStr().c_str());
      bOk = false;
    }
  }

  else
  {
    m_log << labelSyntax << "No variable type found." << eoe;
    LOGERROR("%s", getErrorStr().c_str());
    bOk = false;
  }

  CL_DBG_CALL_OUT_IL(okstr(bOk) << ", type=" << eType << ", pos=" << pos);

  return bOk;
}

bool CommandLine::parseVarRangeExpr(CmdDef              &cmddef,
                                    CmdFormDef          &form,
                                    const TokenVec      &tokens,
                                    size_t              &pos,
                                    CmdArgDef::RangeVec &ranges)
{
  bool              bOk = true;
  StringVec         subranges;
  StringVec         minmax;
  CmdArgDef::range  r;

  CL_DBG_PARSE_CALL_IN(cmddef, form, tokens, pos, " ");

  ranges.clear();

  if( pos < tokens.size() )
  {
    split(tokens[pos].value(), ',', subranges);

    for(size_t i = 0; bOk && (i < subranges.size()); ++i)
    {
      minmax.clear();

      split(subranges[i], ':', minmax);

      if( minmax.size() == 0 )
      {
        m_log << labelSyntax << "No 'min[:max]' subrange found in "
              << "'" << tokens[pos].value() << "'.'"
              << eoe;
        bOk = false;
        break;
      }
      else if( minmax.size() > 2 )
      {
        m_log << labelSyntax << "Too many 'min:max:?' subrange values found in "
              << "'" << tokens[pos].value() << "'.'"
              << eoe;
        bOk = false;
        break;
      }

      if( strToDouble(minmax[0], r.min) != AOk )
      {
        m_log << labelSyntax << "Subrange min '" << minmax[0] << "' NaN in "
              << "'" << tokens[pos].value() << "'.'"
              << eoe;
        bOk = false;
        break;
      }

      if( minmax.size() == 2 )
      {
        if( strToDouble(minmax[1], r.max) != AOk )
        {
          m_log << labelSyntax << "Subrange max '" << minmax[1] << "' NaN in "
                << "'" << tokens[pos].value() << "'.'"
                << eoe;
          bOk = false;
          break;
        }
      }
      else
      {
        r.max = r.min;
      }

      // reverse ranges are ok, just swap order
      if( r.max < r.min )
      {
        double tmp;

        r.min = tmp;
        r.min = r.max;
        r.max = r.min;
      }

      ranges.push_back(r);
    }
  }

  // everything is cool, but no ranges specified
  if( bOk && (ranges.size() == 0) )
  {
    m_log << labelSyntax << "Range expression not found." << eoe;
    bOk = false;
  }

  if( bOk )
  {
    ++pos; // advance token parse position
  }
  else
  {
    LOGERROR("%s", getErrorStr().c_str());
  }

  CL_DBG_CALL_OUT_IL(okstr(bOk) << ", numranges=" << ranges.size()
      << ", pos=" << pos);

  return bOk;
}

bool CommandLine::parseVarRegExpr(CmdDef         &cmddef,
                                  CmdFormDef     &form,
                                  const TokenVec &tokens,
                                  size_t         &pos,
                                  RegEx          &re)
{
  bool  bOk;

  CL_DBG_PARSE_CALL_IN(cmddef, form, tokens, pos, " ");

  if( pos < tokens.size() )
  {
    if( m_bIgnoreCase )
    {
      re.setFlags(RegEx::ReFlagICase);
    }

    re = tokens[pos++].value();

    if( re.isValid() )
    {
      bOk = true;
    }
    else
    {
      m_log << labelSyntax << "Regular expression '"
            << "'" << re.getRegEx() << "': "
            << re.getErrorStr() << "."
            << eoe;
      LOGERROR("%s", getErrorStr().c_str());
      bOk = false;
    }
  }

  else
  {
    m_log << labelSyntax << "Regular expression not found." << eoe;
    LOGERROR("%s", getErrorStr().c_str());
    bOk = false;
  }

  CL_DBG_CALL_OUT_IL(okstr(bOk) << ", re='" << re.getRegEx() << "', pos="
      << pos);

  return bOk;
}

bool CommandLine::parseLiteralValue(CmdDef         &cmddef,
                                    CmdFormDef     &form,
                                    const TokenVec &tokens,
                                    size_t         &pos,
                                    string         &strValue)
{
  bool  bOk;

  CL_DBG_PARSE_CALL_IN(cmddef, form, tokens, pos, " ");

  strValue.clear();

  if( pos < tokens.size() )
  {
    strValue = tokens[pos++].value();
    bOk = true;
  }
  else
  {
    m_log << labelSyntax << "Literal value not found." << eoe;
    LOGERROR("%s", getErrorStr().c_str());
    bOk = false;
  }

  CL_DBG_CALL_OUT_IL(okstr(bOk) << ", value='" << strValue << "', "
      << "pos=" << pos);

  return bOk;
}

bool CommandLine::tokEq(const string   strCmp,
                        const TokenVec &tokens,
                        size_t         &pos)
{
  bool  bOk;

  CL_DBG_CALL_IN("input, strcmp=\"" << strCmp << "\", "
      << "tokens, pos=" << pos, " ");

  if( pos >= tokens.size() )
  {
    m_log << labelSyntax << "Token " << pos << " does not exist." << eoe;
    LOGERROR("%s", getErrorStr().c_str());
    bOk = false;
  }

  else if( tokens[pos].value() != strCmp )
  {
    m_log << labelSyntax << "Expected token '" << strCmp << "'"
          << ", but found " << tokens[pos]
          << eoe;
    LOGERROR("%s", getErrorStr().c_str());
    bOk = false;
  }

  else
  {
    ++pos;
    bOk = true;
  }

  CL_DBG_CALL_OUT_IL(okstr(bOk) << ", pos=" << pos);

  return bOk;
}

bool CommandLine::tokIdentifier(const TokenVec &tokens, size_t &pos)
{
  bool    bOk;

  CL_DBG_CALL_IN("tokens=" << tokens.size() << ", pos=" << pos, " ");

  bOk = isIdentifier(tokens[pos].value());

  if( bOk )
  {
    ++pos;
  }
  else
  {
    m_log << labelSyntax << "Invalid identifier " << tokens[pos] << eoe;
    LOGERROR("%s", getErrorStr().c_str());
  }

  CL_DBG_CALL_OUT_IL(okstr(bOk) << ", pos=" << pos);

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

string CommandLine::c14n(const TokenVec &tokens)
{
  stringstream  ss;
  string        sep;

  for(size_t i = 0; i < tokens.size(); ++i)
  {
    ss << sep << prettify(tokens[i].value());

    if( i == 0 )
    {
      sep = " ";
    }
  }

  return ss.str();
}

bool CommandLine::isIdentifier(const string &str)
{
  if( str.size() == 0 )
  {
    return false;
  }

  // start character
  if( !isalpha(str[0]) && (str[0] != '_') )
  {
    return false;
  }

  // subsequent characters
  for(size_t i = 1; i < str.size(); ++i)
  {
    if( !isalnum(str[i]) && (str[i] != '_') )
    {
      return false;
    }
  }

  return true;
}

int CommandLine::strToBool(const string &str, bool &val)
{
  size_t    i;

  for(i = 0; FalseHood[i] != NULL; ++i)
  {
    if( str == FalseHood[i] )
    {
      val = false;
      return AOk;
    }
  }

  for(i = 0; TruthHood[i] != NULL; ++i)
  {
    if( str == TruthHood[i] )
    {
      val = true;
      return AOk;
    }
  }

  return EBadSyntax;
}

int CommandLine::strToLong(const string &str, long &val)
{
  char  sniff;

  return sscanf(str.c_str(), "%li%c", &val, &sniff) == 1? AOk: EBadSyntax;
}

int CommandLine::strToDouble(const string &str, double &val)
{
  char  sniff;

  return sscanf(str.c_str(), "%lf%c", &val, &sniff) == 1? AOk: EBadSyntax;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Output Methods and Operators
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

ostream &rnr::cmd::operator<<(ostream &os, const CommandLine &cl)
{
  string  sep;

  CommandLine::CmdConstIter iter;

  os << setindent(0);

  os << "CommandLine " << cl.m_strName << endl;
  os << "{" << endl;

  os << deltaindent(2);

  os << indent() << "name          = " << cl.m_strName << endl;
  os << indent() << "have_readline = " << cl.m_readline.haveRlLib() << endl;
  os << indent() << "use_readline  = " << cl.m_readline.useRlLib() << endl;
  os << indent() << "prompt        = " << cl.getPrompt() << endl;
  os << indent() << "ignorecase    = " << cl.m_bIgnoreCase << endl;
  os << indent() << "cmddefs[" << cl.numOfCmds() << "] =" << endl;
  os << indent() << "{" << endl;

  os << deltaindent(2);

  for(iter = cl.m_cmddefs.begin(); iter != cl.m_cmddefs.end(); ++iter)
  {
    os << sep << iter->second;
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

  os << setindent(0);

  return os;
}
