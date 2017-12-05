////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_appkit
//
// File:      ReadLine.cxx
//
/*! \file
 *
 * \brief The ReadLine wrapper class implementaion.
 *
 * ReadLine provides a wrapper around the readline command-line C library.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2011-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 *
 * \par Licence:
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

#include <sys/types.h>
#include <unistd.h>
#include <errno.h>
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <cctype>
#include <locale>

#ifdef HAVE_READLINE
#include <readline/readline.h>
#include <readline/history.h>
#endif // HAVE_READLINE

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/appkit/StringTheory.h"
#include "rnr/appkit/ReadLine.h"

using namespace std;
using namespace rnr;
using namespace rnr::str;
using namespace rnr::cmd;


// -----------------------------------------------------------------------------
// Private Implementation
// -----------------------------------------------------------------------------

namespace rnr
{
  namespace cmd
  {
  } // namespace cmd
} // namespace rnr


// ----------------------------------------------------------------------------
// Class ReadLine
// ----------------------------------------------------------------------------

/*!
 * \brief Pointer to the only instance of ReadLine supported per application.
 */
ReadLine *ReadLine::ThisObj = NULL;

ReadLine::ReadLine(const string strName,
                   const string strPrompt,
                   bool         bUseRlLib) :
    m_strName(strName), m_strPrompt(strPrompt), m_bUseRlLib(bUseRlLib)
{
  m_uLineNum    = 0;
  m_bEoF        = false;
  m_bFError     = false;
  m_fnAppGen    = NULL;
  m_fnAltAppGen = NULL;
  m_pAppArg     = NULL;

#ifdef HAVE_READLINE
  //
  // Only one instance of this class can be instantiated since:
  //  1. The readline library provides no user argument(s) for callbacks to use.
  //  2. And more importantly, readline library uses global variables.
  //
  assert(ThisObj == NULL);

  // limitation of readline library which does not provide context feedback
  ThisObj = this;

  // tell readline completer that we want a crack at it first before default
  rl_attempted_completion_function = ReadLine::completionWrapper;

  // allow conditional parsing of the ~/.inputrc file
  rl_readline_name = m_strName.c_str();

#endif // HAVE_READLINE
}

ReadLine::~ReadLine()
{
  ThisObj = NULL;
}

void ReadLine::registerGenerator(AppGenFunc fnAppGen, void *pAppArg)
{
  m_fnAppGen    = fnAppGen;
  m_fnAltAppGen = NULL;
  m_pAppArg     = pAppArg;
}

void ReadLine::registerAltGenerator(AltAppGenFunc fnAltAppGen, void *pAppArg)
{
  m_fnAppGen    = NULL;
  m_fnAltAppGen = fnAltAppGen;
  m_pAppArg     = pAppArg;
}

void ReadLine::unregisterGenerator()
{
  m_fnAppGen    = NULL;
  m_fnAltAppGen = NULL;
  m_pAppArg     = NULL;

#ifdef HAVE_READLINE
  rl_attempted_completion_over  = 0;
  rl_completion_suppress_append = 0;
#endif // HAVE_READLINE
}

string &ReadLine::rlreadLine()
{
#ifdef HAVE_READLINE
  bool  bInteractive = isInteractive(stdin);

  // 
  // Use readline library.
  //
  if( m_bUseRlLib  && bInteractive )
  {
    clearStreamStatus();
    m_strLine.clear();

    char *sLine = readline(getPrompt().c_str());   // fixed at stdin

    if( sLine != NULL )
    {
      m_strLine = sLine;
      free(sLine);
      ++m_uLineNum;
    }

    else
    {
      setStreamStatus(stdin);
    }

    return m_strLine;
  }

  //
  // Interactive, but don't use the readline library.
  //
  else if( bInteractive )
  {
    return ireadLine(stdin);
  }

  //
  // Non-interactive.
  //
  else
  {
    return freadLine(stdin);
  }

#else // !HAVE_READLINE

  return ireadLine(stdin);

#endif // HAVE_READLINE
}

string &ReadLine::ireadLine(FILE *fp)
{
  // prompt
  if( isInteractive(fp) && !getPrompt().empty() )
  {
    fprintf(stdout, "%s", getPrompt().c_str());
    fflush(stdout);
  }

  return freadLine(fp);
}

string &ReadLine::freadLine(FILE *fp)
{
  static size_t   BufSize = 4096;

  char    bufLine[BufSize];
  size_t  n;

  clearStreamStatus();
  m_strLine.clear();

  if( fgets(bufLine, BufSize, fp) != NULL )
  {
    bufLine[BufSize-1] = 0;
    n = strlen(bufLine);

    // some characters were read
    if( n > 0 )
    {
      // got end of line
      if( bufLine[n-1] == '\n' )
      {
        bufLine[n-1] = 0;
        ++m_uLineNum;
      }

      // oops, input line trunctated - too long
      else
      {
        bufLine[n-1] = 0;
        ++m_uLineNum;
        LOGWARN("Input line too long. Exceeds %zu characters.", BufSize);
      }
    }

    // no i/o errors but also no characters read - odd
    else
    {
      setStreamStatus(fp);

      bufLine[0] = 0;
    }
  }

  // read failure
  else
  {
    setStreamStatus(fp);

    bufLine[0] = 0;
  }

  m_strLine = bufLine;

  trim(m_strLine);

  return m_strLine;
}

void ReadLine::addToHistory(const string &strLine)
{
#ifdef HAVE_READLINE
  string      str(strLine);
  HIST_ENTRY *pCurHist;

  trim(str);

  if( !str.empty() )
  {
    pCurHist = previous_history();

    if( (pCurHist == NULL) || strcmp(pCurHist->line, str.c_str()) )
    {
      add_history(str.c_str());
    }
  }
#endif // HAVE_READLINE
}

bool ReadLine::isInteractive(FILE *fp)
{
  return isatty(fileno(fp))? true: false;
}

const string &ReadLine::getErrorStr() const
{
  return m_strError;
}

void ReadLine::setStreamStatus(FILE *fp)
{
  m_bEoF    = feof(fp)?   true: false;
  m_bFError = ferror(fp)? true: false;

  if( m_bFError )
  {
    stringstream ss;

    if( errno > 0 )
    {
      ss << strerror(errno) << "(errno=" << errno << "}";
    }
    else
    {
      ss << "File read error.";
    }

    m_strError = ss.str();
  }
}

void ReadLine::clearStreamStatus()
{
  m_bEoF    = false;
  m_bFError = false;
  m_strError.clear();
}

char *ReadLine::dupstr(const char *s)
{
  size_t  size = sizeof(char) * (strlen(s) + 1);
  char *t = (char *)malloc(size);
  strcpy(t, s);
  return t;
}

size_t ReadLine::tokenize(const string &str, StringVec &tokens)
{
  size_t    i, len;
  string    tok;

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

    tok.clear();

    // find end of word
    while( (i < len) && !isspace((int)str[i]) )
    {
      tok.push_back(str[i++]);
    }

    tokens.push_back(tok);
  }

  return tokens.size();
}

size_t ReadLine::tokenize(char *s, char *tokv[], size_t tokmax)
{
  int       tokc = 0;

  while( tokc < (int)tokmax )
  {
    // find start of the next token
    while( *s && isspace((int)*s) )
    {
      s++;
    }

    // no more tokens
    if( *s == 0 )
    {
      return tokc;
    }

    // new token
    tokv[tokc++] = s;

    // find end of the token
    while( *s && !isspace((int)*s) )
    {
      s++;
    }

    // end of the line
    if( *s == 0 )
    {
      return tokc;
    }

    // null terminate
    *s++ = 0;
  }

  return (size_t)tokc;
}

int ReadLine::wc(const char *s)
{
  int   wc = 0;

  while( *s )
  {
    // find start of the next word
    while( *s && isspace((int)*s) )
    {
      s++;
    }

    // no more words
    if( *s == 0 )
    {
      break;
    }

    ++wc;

    // find end of the word
    while( *s && !isspace((int)*s) )
    {
      s++;
    }
  }

  return wc;
}

char **ReadLine::completionWrapper(const char *sText, int nStart, int nEnd)
{
  return ThisObj->completion(sText, nStart, nEnd);
}

char **ReadLine::completion(const char *sText, int nStart, int nEnd)
{
#ifdef HAVE_READLINE
  if( m_fnAltAppGen != NULL )
  {
    return altCompletion(sText, nStart, nEnd);
  }
  else
  {
    return rl_completion_matches(sText, generatorWrapper);
  }

#else
  return NULL;
#endif // HAVE_READLINE
}

char **ReadLine::altCompletion(const string strText, int nStart, int nEnd)
{
#ifdef HAVE_READLINE
  string          strContext(rl_line_buffer);
  int             nIndex;
  unsigned        uFlags;
  string          strMatch;
  vector<string>  matchList;
  size_t          i;
  char            **tabList;

  rl_attempted_completion_over  = 0;  // disable default filename TAB completion
  rl_completion_suppress_append = 0;  // enable default space (' ') after match

  // no flags
  uFlags = 0;

  for(nIndex = 0; ; ++nIndex)
  {
    strMatch = m_fnAltAppGen(m_pAppArg, strText, nIndex,
                             strContext, nStart, nEnd, uFlags);

    // end of completion generation
    if( strMatch.empty() )
    {
      if( uFlags & FlagTabNoFilename )
      {
        rl_attempted_completion_over = 1;
      }

      if( uFlags & FlagTabNoSpace )
      {
        rl_completion_suppress_append = 1;
      }
      break;
    }

    // regular completion
    else
    {
      // Empty string disables default. Undocumented.
      if( (nIndex == 0) && (uFlags & FlagTabNoDefault) )
      {
        matchList.push_back("");
      }
      matchList.push_back(strMatch);
    }
  }

  // no matches
  if( matchList.size() == 0 )
  {
    return NULL;
  }

  //
  // Malloc a tap-completion array.
  //
  // Note:  The readlline library frees all entries in this array along with
  //        the array itself. Undocumented.
  //
  i = sizeof(char*) * (matchList.size() + 1);

  tabList = (char**)malloc(i);

  // copy allocated char *'s
  for(i = 0; i < matchList.size(); ++i)
  {
    tabList[i] = dupstr(matchList[i]);
  }
  tabList[i] = NULL;

  return tabList;

#else // !HAVE_READLINE

  return NULL;

#endif // HAVE_READLINE
}

char *ReadLine::generatorWrapper(const char *sText, int nState)
{
#ifdef HAVE_READLINE
  // no registered generator match current path
  if( ThisObj->m_fnAppGen == NULL )
  {
    return NULL;
  }

  string strText(sText);
  string strContext(rl_line_buffer);

  // trim left and right whitespace
  trim(strContext);

  // first time through
  if( nState == ReadLine::FIRST )
  {
  }

  // call application-specific generator
  return ThisObj->m_fnAppGen(ThisObj->m_pAppArg, strText, nState, strContext);

#else // !HAVE_READLINE

  return NULL;

#endif // HAVE_READLINE
}
