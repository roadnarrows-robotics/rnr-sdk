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
 * ReadLine provides a wrapper around the readline command-line library.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011-2017  RoadNarrows
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
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
#include <algorithm> 
#include <functional> 
#include <cctype>
#include <locale>

#ifdef HAVE_READLINE
#include <readline/readline.h>
#include <readline/history.h>
#endif // HAVE_READLINE

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/appkit/ReadLine.h"

using namespace std;
using namespace rnr;
using namespace rnr::cmd;


// -----------------------------------------------------------------------------
// Private Implementation
// -----------------------------------------------------------------------------

namespace rnr
{
  namespace cmd
  {
    /*!
     * \brief Trim string in-place of leading whitespace.
     *
     * \param str String to trim.
     *
     * \return Left trimmed string.
     */
    static inline std::string &ltrim(std::string &str)
    {
      str.erase( str.begin(), std::find_if(str.begin(), str.end(),
                std::not1(std::ptr_fun<int, int>(std::isspace))) );
      return str;
    }
    
    /*!
     * \brief Trim string in-place of trailing whitespace.
     *
     * \param str String to trim.
     *
     * \return Right trimmed string.
     */
    static inline std::string &rtrim(std::string &str)
    {
      str.erase( std::find_if(str.rbegin(), str.rend(),
          std::not1(std::ptr_fun<int, int>(std::isspace))).base(), str.end() );
      return str;
    }
    
    /*!
     * \brief Trim string in-place of trailing and trailing whitespace.
     *
     * \param str String to trimmed.
     *
     * \return Trimmed string.
     */
    static inline std::string &trim(std::string &str)
    {
      return ltrim(rtrim(str));
    }

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
  //
  // Only one instance of this class can be instantiated since:
  //  1. The readline library provides no user argument for callbacks to use.
  //  2. Boost function/bind does work with non-static member functions AND
  //     readline C callback function pointers.
  //  3. And most importantly, readline library uses global variables.
  //
  assert(ThisObj == NULL);

  // limitation of readline library which does not provide context feedback
  ThisObj = this;

  m_bEoF      = false;
  m_bFError   = false;
  m_fnAppGen  = NULL;
  m_pAppArg   = NULL;

#ifdef HAVE_READLINE

  // tell readline completer that we want a crack at it first before default
  rl_attempted_completion_function = ReadLine::completionWrapper;

  // allow conditional parsing of the ~/.inputrc file
  rl_readline_name = m_strName.c_str();

#endif // HAVE_READLINE
}

ReadLine::~ReadLine()
{
  //fprintf(stderr, "rdk: %s %s\n", __func__, m_strName.c_str());
}

void ReadLine::registerGenerator(ReadLineGenFunc   fnAppGen,
                                 void              *pAppArg)
{
  m_fnAppGen  = fnAppGen;
  m_pAppArg   = pAppArg;
}

void ReadLine::unregisterGenerator()
{
  m_fnAppGen  = NULL;
  m_pAppArg   = NULL;
}

string &ReadLine::rlreadLine()
{
  bool  bInteractive = isInteractive(stdin);

  m_strLine.clear();

  if( m_bUseRlLib  && bInteractive )
  {
#ifdef HAVE_READLINE
    char *sLine = readline(getPrompt().c_str());   // fixed at stdin

    if( sLine != NULL )
    {
      m_strLine = sLine;
      trim(m_strLine);
      free(sLine);
    }

    else
    {
      setStreamStatus(stdin);
    }
#else
    ireadLine(stdin);
#endif // HAVE_READLINE
  }

  else if( bInteractive )
  {
    ireadLine(stdin);
  }

  else
  {
    freadLine(stdin);
  }

  return m_strLine;
}

string &ReadLine::ireadLine(FILE *fp)
{
  bool    bInteractive  = isInteractive(stdin);

  if( !getPrompt().empty() )
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

  if( fgets(bufLine, BufSize, fp) != NULL )
  {
    bufLine[BufSize-1] = 0;
    n = strlen(bufLine);
    if( (n > 0) && (bufLine[n-1] == '\n') )
    {
      bufLine[n-1] = 0;
    }
  }

  else
  {
    // set state
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

char *ReadLine::dupstr(const char *s)
{
  char *t = new char[strlen(s)+1];
  strcpy(t, s);
  return t;
}

string ReadLine::strip(const string &str)
{
  string  stripped(str);

  trim(stripped);

  return stripped;
}

string ReadLine::c14n(const string &str, size_t uLen)
{
  string  strc14n(str, uLen);
  size_t  pos;

  // strip leading and trailing whitespace
  trim(strc14n);

  // convert multiple whitespace sequences to a single blank
  // TODO need double quote processing
#if 0 // RDK
  if( strc14n.length() != 0 )
  {
    while( (pos = strc14n.find("  ")) != strc14n.npos )
    {
      str = strc14n.replace(pos, 2, " ");
    }
  }

  //cout << "  (" << str << ")" << endl;
#endif // RDK

  return strc14n;
}

int ReadLine::tokenize(char *s, char *tokv[], size_t tokmax)
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

  return tokc;
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
  return rl_completion_matches(sText, generatorWrapper);

#else
  return NULL;
#endif // HAVE_READLINE
}

char *ReadLine::generatorWrapper(const char *sText, int nState)
{
  // no registered generator match current path
  if( ThisObj->m_fnAppGen == NULL )
  {
    return NULL;
  }

  string strText(sText);
  string strContext(rl_line_buffer);

  trim(strContext);

  // first time through
  if( nState == ReadLine::FIRST )
  {
  }

  // call application-specific generator
  return ThisObj->m_fnAppGen(strText,
                             nState,
                             strContext,
                             ThisObj->m_pAppArg );
}
