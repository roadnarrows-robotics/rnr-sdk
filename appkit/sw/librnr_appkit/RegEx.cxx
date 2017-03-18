////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_appkit
//
// File:      RegEx.cxx
//
/*! \file
 *
 * \brief The Regular Expression class implementation.
 *
 * RegEx provides a wrapper around the regex C library calls.
 *
 * \note Generalized from dynashell_regex.cxx source found in RoadNarrows
 * Robotics Dynamixel SDK package.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2017-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
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


#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <regex.h>

#include <iostream>
#include <iomanip>
#include <cstdio>
#include <string>
#include <vector>

#include <rnr/rnrconfig.h>
#include <rnr/log.h>

#include "rnr/appkit/RegEx.h"

using namespace std;
using namespace rnr;


// ----------------------------------------------------------------------------
// Class RegEx
// ----------------------------------------------------------------------------

const char *RegEx::ReInvalid = "inre";

RegEx::RegEx(int nFlags)
{
  m_bIsValid = false;

  groomFlags(nFlags);
  setError(ReENoExpr);
}

RegEx::RegEx(const string &strRegEx, int nFlags)
{
  m_strRegEx  = strRegEx;
  m_bIsValid  = false;

  groomFlags(nFlags);

  compile();
}

RegEx::RegEx(const char *sRegEx, int nFlags)
{
  m_bIsValid = false;

  groomFlags(nFlags);

  if( (sRegEx != NULL) && (*sRegEx != 0) )
  {
    m_strRegEx = sRegEx;
    compile();
  }
  else
  {
    setError(ReENoExpr);
  }
}

RegEx::RegEx(const RegEx &src)
{
  m_strRegEx  = src.m_strRegEx;
  m_bIsValid  = false;

  groomFlags(src.m_nFlags);

  compile();
}

RegEx::~RegEx()
{
  freeReMem();
}

RegEx &RegEx::operator=(const RegEx &rhs)
{
  freeReMem();

  m_strRegEx = rhs.m_strRegEx;
 
  groomFlags(rhs.m_nFlags);

  compile();

  return *this;
}

RegEx &RegEx::operator=(const string &rhs)
{
  freeReMem();

  m_strRegEx = rhs;

  compile();

  return *this;
}
  
RegEx &RegEx::operator=(const char *rhs)
{
  freeReMem();

  if( (rhs != NULL) && (*rhs != 0) )
  {
    m_strRegEx = rhs;
    compile();
  }
  else
  {
    m_strRegEx.clear();
    setError(ReENoExpr);
  }

  return *this;
}

bool RegEx::match(const string &strInput, int nFlags) const
{
  if( !m_bIsValid )
  {
    return false;
  }
  else if( regexec(&m_regex, strInput.c_str(), 0, NULL, nFlags) == ReOk )
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool RegEx::match(const string &strInput, int nFlags)
{
  int rc; // return code

  if( !m_bIsValid )
  {
    setError(ReENotComp);
    return false;
  }
  else if( (rc = regexec(&m_regex, strInput.c_str(), 0, NULL, nFlags) == ReOk) )
  {
    setError(ReOk); // clear
    return true;
  }
  else
  {
    setError(rc);
    return false;
  }
}

bool RegEx::match(const char *sInput, int nFlags) const
{
  string strInput;

  if( sInput != NULL )
  {
    strInput = sInput;
  }

  return match(strInput, nFlags);
}

bool RegEx::match(const char *sInput, int nFlags)
{
  string strInput;

  if( sInput != NULL )
  {
    strInput = sInput;
  }

  return match(strInput, nFlags);
}

size_t RegEx::match(const string &strInput,
                    ReMatchVec   &matches,
                    const size_t uMaxSubMatches,
                    const int    nFlags) const
{
  regmatch_t  pos[uMaxSubMatches];
  const char *sIn;    // fixed input start pointer
  const char *s;      // working pointer
  int         flags;  // working matching behavior flags
  
  matches.clear();

  if( !m_bIsValid )
  {
    return matches.size();
  }

  sIn = s = strInput.c_str();
  flags = nFlags;

  while( regexec(&m_regex, s, uMaxSubMatches, pos, flags) == ReOk )
  {
    for(size_t i = 0; i < uMaxSubMatches; ++i)
    {
      ReMatch m;

      if( pos[i].rm_so == -1 )
      {
        break;
      }

      m.m_uStart   = pos[i].rm_so + (s - sIn);
      m.m_uEnd     = pos[i].rm_eo + (s - sIn);
      m.m_strMatch = strInput.substr(m.m_uStart, m.m_uEnd-m.m_uStart);
      m.m_uEnd    -= 1;

      //cerr << "DBG: " << i << ". "
      //    << "(" << m.m_uStart << "," << m.m_uEnd << ") "
      //    << "'" << m.m_strMatch << "'" << endl;

      matches.push_back(m);
    }

    flags |= REG_NOTBOL;

    s += pos[0].rm_eo;
  }

  return matches.size();
}

size_t RegEx::match(const string &strInput,
                    ReMatchVec   &matches,
                    const size_t uMaxSubMatches,
                    const int    nFlags)
{
  regmatch_t  pos[uMaxSubMatches];
  const char *sIn;    // fixed input start pointer
  const char *s;      // working pointer
  int         flags;  // working matching behavior flags
  int         rc;     // return code
  
  matches.clear();

  if( !m_bIsValid )
  {
    setError(ReENotComp);
    return matches.size();
  }

  sIn = s = strInput.c_str();
  flags = nFlags;

  while( (rc = regexec(&m_regex, s, uMaxSubMatches, pos, flags)) == ReOk )
  {
    for(size_t i = 0; i < uMaxSubMatches; ++i)
    {
      ReMatch m;

      if( pos[i].rm_so == -1 )
      {
        break;
      }

      m.m_uStart   = pos[i].rm_so + (s - sIn);
      m.m_uEnd     = pos[i].rm_eo + (s - sIn);
      m.m_strMatch = strInput.substr(m.m_uStart, m.m_uEnd-m.m_uStart);
      m.m_uEnd    -= 1;

      //cerr << "DBG: " << i << ". "
      //    << "(" << m.m_uStart << "," << m.m_uEnd << ") "
      //    << "'" << m.m_strMatch << "'" << endl;

      matches.push_back(m);
    }

    flags |= REG_NOTBOL;

    s += pos[0].rm_eo;
  }

  if( matches.size() == 0 )
  {
    setError(rc);
  }

  return matches.size();
}

size_t RegEx::match(const char   *sInput,
                    ReMatchVec   &matches,
                    const size_t uMaxSubMatches,
                    const int    nFlags) const
{
  string strInput;

  if( sInput != NULL )
  {
    strInput = sInput;
  }

  return match(strInput, matches, uMaxSubMatches, nFlags);
}

size_t RegEx::match(const char   *sInput,
                    ReMatchVec   &matches,
                    const size_t uMaxSubMatches,
                    const int    nFlags)
{
  string strInput;

  if( sInput != NULL )
  {
    strInput = sInput;
  }

  return match(strInput, matches, uMaxSubMatches, nFlags);
}

bool RegEx::compile()
{
  int rc;   // return code

  freeReMem();

  if( m_strRegEx.length() == 0 )
  {
    setError(ReENoExpr);
  }
  else if( (rc = regcomp(&m_regex, m_strRegEx.c_str(), m_nFlags)) == ReOk )
  {
    m_bIsValid = true;
    setError(ReOk); // not an error
  }
  else
  {
    setError(rc);
    LOGERROR("'%s': %s", m_strRegEx.c_str(), m_strReError.c_str());
  }

  return m_bIsValid;
}

void RegEx::setFlags(const int nFlags)
{
  int oldFlags = m_nFlags;

  groomFlags(nFlags);

  if( (m_nFlags != oldFlags) && !m_strRegEx.empty() )
  {
    compile();
  }
}

void RegEx::groomFlags(const int nFlags)
{
  m_nFlags  = nFlags & (ReFlagICase | ReFlagNewLine);
  m_nFlags |= REG_EXTENDED;
}

void RegEx::setError(const int nCode)
{
  m_nReCode = nCode;

  switch( m_nReCode )
  {
    case ReOk:
      m_strReError.clear();
      break;
    case ReENoExpr:
      m_strReError = "No pre-compiled regular expression";
      break;
    case ReENotComp:
      m_strReError = "Not compiled";
      break;
    default:
    {
      char buf[256];

      regerror(m_nReCode, &m_regex, buf, sizeof(buf));

      m_strReError = buf;
    }
    break;
  }
}

void RegEx::freeReMem()
{
  if( m_bIsValid )
  {
    regfree(&m_regex);
    m_bIsValid = false;
  }
}

ostream &rnr::operator<<(ostream &os, const rnr::RegEx &re)
{
  // invalid regular expression
  if( !re.isValid() )
  {
    os << RegEx::ReInvalid;
    return os;
  }

  string  strRe = re.getRegEx();

  os << "re\"";

  for(size_t i = 0; i < strRe.size(); ++i)
  {
    switch( strRe[i] )
    {
      case '\x1b':
        os << "\\e";
        break;
      case '\f':
        os << "\\f";
        break;
      case '\n':
        os << "\\n";
        break;
      case '\r':
        os << "\\r";
        break;
      case '\t':
        os << "\\t";
        break;
      case '\v':
        os << "\\v";
        break;
      case '"':
        os << "\\\"";
        break;
      default:
        os << strRe[i];
        break;
    }
  }

  os << "\"";

  return os;
}

istream &rnr::operator>>(istream &is, rnr::RegEx &re)
{
  // state
  string  strRe;
  char    expected  = 'r';
  bool    bHasRe    = false;
  bool    bInEsc    = false;

  char    c;

  // skip leading whitespace
  while( !is.fail() && !is.eof() && isspace(is.peek()) )
  {
    is.get(c); // eat
  }

  while( !bHasRe && !is.fail() && !is.eof() )
  {
    is.get(c);

    if( is.fail() || is.eof() )
    {
      break;
    }

    //
    // Step through expected state to get re prefix.
    //
    else if( expected != 0 )
    {
      if( c == expected )
      {
        switch( c )
        {
          case 'r':
            expected = 'e';
            break;
          case 'e':
            expected = '"';
            break;
          case '"':
          default:
            expected = 0;
            break;
        }
        strRe.push_back(c);
      }
      else
      {
        LOGERROR("%s: Got '%c', expected '%c' after '%s'. "
            "Required format: re\"REGEX\"",
            LOGFUNCNAME, c, expected, strRe.c_str());
        cin.setstate(ios::failbit);
      }
    }

    //
    // In regular expression escape sequence.
    //
    else if( bInEsc )
    {
      strRe.push_back(c);
      bInEsc = false;
    }
            
    //
    // Unescaped regular expression.
    //
    else
    {
      switch( c )
      {
        case '"':
          bHasRe = true;
          break;
        case '\\':
          strRe.push_back(c);
          bInEsc = true;
          break;
        case '\n':
        case '\r':
          LOGERROR("%s: Unexpected end-of-line after '%s'.",
            LOGFUNCNAME, strRe.c_str());
          cin.setstate(ios::failbit);
          break;
        default:
          if( isascii(c) && !iscntrl(c) )
          {
            strRe.push_back(c);
          }
          else
          {
            LOGERROR("%s: Unexpected non-ascii character 0x%02x after %s.",
              LOGFUNCNAME, c, strRe.c_str());
            cin.setstate(ios::failbit);
          }
          break;
      }
    }
  }

  if( cin.good() )
  {
    re = strRe.substr(3);
  }

  return is;
}
