////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Program:   dynashell
//
// File:      dynashell_readline.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
 * $Rev: 3845 $
 *
 * \brief The Dynamixel Shell ReadLine Class.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011-2015  RoadNarrows LLC.
 * \n All Rights Reserved
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
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#include <iostream>
#include <string>
#include <vector>

#ifdef HAVE_READLINE
#include <readline/readline.h>
#include <readline/history.h>
#endif // HAVE_READLINE

#include "dynashell_regex.h"
#include "dynashell_readline.h"

using namespace std;


// ----------------------------------------------------------------------------
// Class ReadLineEntry
// ----------------------------------------------------------------------------

/*!
 * \brief Default constructor.
 */
ReadLineEntry::ReadLineEntry()
{
  m_nUid      = ReadLine::NOT_REG;
  m_fnAppGen  = NULL;
  m_pAppArg   = NULL;
}

/*!
 * \brief Initialization constructor.
 *
 * \param strRegEx  Regular expression applied to current readline buffer state.
 * \param fnAppGen  Application-specific generator function.
 * \param pAppArg   Optional application argument generator function.
 */
ReadLineEntry::ReadLineEntry(const string         &strRegEx,
                             ReadLineAppGenFunc_T  fnAppGen,
                             void                 *pAppArg)
{
  m_nUid      = ReadLine::NOT_REG;
  m_regex     = strRegEx;
  m_fnAppGen  = fnAppGen;
  m_pAppArg   = pAppArg;
}

/*!
 * \brief Initialization constructor.
 *
 * \param sRegEx    Regular expression applied to current readline buffer state.
 * \param fnAppGen  Application-specific generator function.
 * \param pAppArg   Optional application argument generator function.
 */
ReadLineEntry::ReadLineEntry(const char           *sRegEx,
                             ReadLineAppGenFunc_T  fnAppGen,
                             void                 *pAppArg)
{
  m_nUid      = ReadLine::NOT_REG;
  m_regex     = sRegEx;
  m_fnAppGen  = fnAppGen;
  m_pAppArg   = pAppArg;
}

/*!
 * \brief Copy constructor.
 *
 * \param src     ReadLineEntry to copy.
 */
ReadLineEntry::ReadLineEntry(const ReadLineEntry &src)
{
  m_nUid      = src.m_nUid;
  m_regex     = src.m_regex;
  m_fnAppGen  = src.m_fnAppGen;
  m_pAppArg   = src.m_pAppArg;
}

/*!
 * \brief Default destructor.
 */
ReadLineEntry::~ReadLineEntry()
{
}


// ----------------------------------------------------------------------------
// Class ReadLine
// ----------------------------------------------------------------------------

/*!
 * \brief Initialization constructor.
 */
ReadLine::ReadLine(const string &strName)
{
  if( strName.length() > 0 )
  {
    m_sName = dupstr(strName.c_str());
  }
  else
  {
    m_sName = dupstr("none");
  }

  m_nUidCounter = 0;

#ifdef HAVE_READLINE
  // Allow conditional parsing of the ~/.inputrc file.
  rl_readline_name = m_sName;

  // Tell the completer that we want a crack first.
  rl_attempted_completion_function = CompletionWrap;
#endif // HAVE_READLINE

  // limitation of readline library which does not provide context feedback
  ReadLineThis = this;
}

/*!
 * \brief Initialization constructor.
 */
ReadLine::ReadLine(const char *sName)
{
  if( sName != NULL )
  {
    m_sName = dupstr(sName);
  }
  else
  {
    m_sName = dupstr("none");
  }

  m_nUidCounter = 0;

#ifdef HAVE_READLINE
  // Allow conditional parsing of the ~/.inputrc file.
  rl_readline_name = m_sName;

  // Tell the completer that we want a crack first.
  rl_attempted_completion_function = CompletionWrap;
#endif // HAVE_READLINE

  // limitation of readline library which does not provide context feedback
  ReadLineThis = this;
}

/*!
 * \brief Destructor
 */
ReadLine::~ReadLine()
{
  if( m_sName != NULL )
  {
    delete[] m_sName;
  }
}

/*!
 * \brief Register application-specific tab-completion generator associated.
 *
 * \param sRegEx    Regular expression applied to current readline buffer state.
 * \param fnAppGen  Application-specific generator function.
 * \param pAppArg   Optional application argument generator function.
 *
 * \return
 * On successful registration, a unique id \h_ge 0 is returned.\n
 * On regular expression evalution failure, the generator is not registered and
 * ReadLine::NOT_REG is returned.
 */
int ReadLine::RegisterGenerator(const char           *sRegEx,
                                ReadLineAppGenFunc_T  fnAppGen,
                                void                 *pAppArg)
{
  string        str;
  ReadLineEntry entry(sRegEx, fnAppGen, pAppArg);

  if( entry.IsValid() )
  {
    entry.m_nUid = m_nUidCounter;
    m_vecGenerators.push_back(entry);
    //fprintf(stderr, "DBG: %d: regex(%s)\n", m_nUidCounter, sRegEx);
    return m_nUidCounter++;
  }
  else
  {
    return NOT_REG;
  }
}

/*!
 * \brief Unregister application-specific generator associated with path.
 *
 * \param strPath   Unique path string.
 */
void ReadLine::UnregisterGenerator(int nUid)
{
  VecAppEntry::iterator iter;

  for(iter = m_vecGenerators.begin();
      iter != m_vecGenerators.begin();
      ++iter)
  {
    if( iter->GetUid() == nUid )
    {
      m_vecGenerators.erase(iter);
      return;
    }
  }
}

/*! 
 * \brief Read one input line from the given input stream.
 *
 * The freadline functions will read one line of input from the given input
 * file pointer using the prompt string to prompt the user. If the prompt is
 * NULL or an empty string then no prompt is issued. The line returned is
 * allocated, so the caller must free it when finished.
 *
 * The line returned has the final newline removed, so only the text of the
 * line remains.
 *
 * \param fp      File pointer to input steam.
 * \param sPrompt Optional user prompt string.
 *
 * \return If no errors occurred and EOF is not encountered, an allocated,
 * null-terminated line buffer is return. Else NULL is return.
 */
char *ReadLine::fReadLine(FILE *fp, const char *sPrompt)
{
  static size_t   BufSize = 4096;

  char   *bufLine;
  size_t  n;

  bufLine = new char[BufSize];

  if( sPrompt && *sPrompt )
  {
    fprintf(stdout, "%s", sPrompt);
    fflush(stdout);
  }

  if( fgets(bufLine, BufSize, fp) == NULL )
  {
    delete[] bufLine;
    return NULL;
  }

  bufLine[BufSize-1] = 0;
  n = strlen(bufLine);
  if( (n > 0) && (bufLine[n-1] == '\n') )
  {
    bufLine[n-1] = 0;
  }

  return bufLine;
}

/*!
 * \brief Add line to history.
 *  
 *  The line is added if the line is not empty and does not match the last
 *  command.
 *
 *  \param  Line to add.
 */
void ReadLine::AddToHistory(const char *sLine)
{
#ifdef HAVE_READLINE
  string      str = sLine;
  HIST_ENTRY *pCurHist;

  str =  strip(str);

  if( !str.empty() )
  {
    pCurHist = previous_history();

    if( (pCurHist == NULL) || strcmp(pCurHist->line, str.c_str()) )
    {
      add_history(sLine);
    }
  }
#endif // HAVE_READLINE
}

/*!
 * \brief Duplicate string.
 *
 * \param s   Null-terminated string to dup.
 *
 * \return Duplicated, allocated char *.
 */
char *ReadLine::dupstr(const char *s)
{
  char *t = new char[strlen(s)+1];
  strcpy(t, s);
  return t;
}

/*!
 * \brief Strip string of leading and trailing white space.
 *
 * \param [in] str    Input string to strip.
 *
 * \return Stripped string.
 */
string ReadLine::strip(string &str)
{
  string  t = str;
  int     i;

  // strip leading blanks
  for(i=0; i<t.length(); ++i)
  {
    if( !isspace((int)t[i]) )
    {
      break;
    }
  }

  if( i != 0 )
  {
    t = t.substr(i);
  }

  // strip trailing blanks
  for(i=t.length()-1; i>0; --i)
  {
    if( !isspace((int)t[i]) )
    {
      break;
    }
  }

  if( i != t.length()-1 )
  {
    t = t.substr(0, i+1);
  }

  return t;
}

/*!
 * \brief Strip string of leading and trailing white space.
 *
 * A null character '\0' is inserted after the last non-white space character.
 *
 * \param [in,out] s  Null-terminated char*.
 *
 * \return Pointer to first non-white space character in s or to null-terminator
 * if no non-white space characters are found. 
 */
char *ReadLine::strip(char *s)
{
  char *left, *right;

  for(left = s; isspace((int)*left); left++) ;
    
  if( *left == 0 )
  {
    return left;
  }

  right = left + strlen(left) - 1;

  while( (right > left) && isspace((int)*right) )
  {
    right--;
  }

  *++right = '\0';

  return left;
}

/*!
 * \brief Canonicalization of a string.
 *
 * \note c14n is an cute abbreviation where 14 represents the number of letters
 * between the c and n.
 *
 * \param s       Null-terminated string to canonicalize.
 * \param uLen    (Sub)length of string to canonicalize.
 *
 * \return Return string holding canonical form.
 */
string ReadLine::c14n(const char *s, size_t uLen)
{
  string  str(s, uLen);
  size_t  pos;

  // strip leading and trailing whitespace
  str = ReadLine::strip(str);

  // convert multiple whitespace sequences to a single blank
  if( str.length() != 0 )
  {
    while( (pos = str.find("  ")) != str.npos )
    {
      str = str.replace(pos, 2, " ");
    }
  }

  //cout << "  (" << str << ")" << endl;

  return str;
}

/*!
 * \brief Tokenize input.
 *
 * \param [in,out] s  Input string to tokenize.
 * \param [out] tokv  Array of tokens (pointer to locations in s).
 * \param tokmax      Maximum number of tokens.
 *
 * \return Number of tokens.
 */
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

/*!
 * \brief Count the words in the string.
 *
 * \param str   String to count.
 *
 * \return Number of words.
 */
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

/*! 
 * \brief Command completion callback function wrapper.
 *
 * Attempt to complete on the contents of text. The start and end bound the
 * region of rl_line_buffer that contains the word to complete.  Text is
 * the word to complete.  We can use the entire contents of rl_line_buffer
 * in case we want to do some simple parsing.
 *
 * \param sText   Text string to complete as a command.
 * \param nStart  Start index of text string in line.
 * \param nEnd    End index of text string in line.
 *  
 * \return Array of matches, NULL if none.
 */
char **ReadLine::CompletionWrap(const char *sText, int nStart, int nEnd)
{
  return ReadLineThis->Completion(sText, nStart, nEnd);
}

/*! 
 * \brief Command completion callback function.
 *
 * \param sText   Text string to complete as a command.
 * \param nStart  Start index of text string in line.
 * \param nEnd    End index of text string in line.
 *  
 * \return Array of matches, NULL if none.
 */
char **ReadLine::Completion(const char *sText, int nStart, int nEnd)
{
#ifdef HAVE_READLINE
  MatchGenerator(nStart);

  return rl_completion_matches(sText, GeneratorWrap);

#else
  return NULL;
#endif // HAVE_READLINE
}

/*!
 * \brief Generator wrapper.
 *
 * Calls the matched, registered application-specific generator.
 *
 * \param sText     Partial text string to complete.
 * \param nState    Generator state. If FIRST,then initialize any statics.
 *
 * \return If a first/next match is made, return allocated completed match.\n
 * Otherwise return NULL.
 */

char *ReadLine::GeneratorWrap(const char *sText, int nState)
{
  // no registered generator match current path
  if( ReadLineThis->m_posMatched == ReadLineThis->m_vecGenerators.end() )
  {
    return NULL;
  }

  // first time through
  if( nState == ReadLine::FIRST )
  {
    ReadLineThis->m_uTextLen = strlen(sText);
  }

  // call application-specific generator
  return ReadLineThis->m_posMatched->m_fnAppGen(
                              ReadLineThis->m_posMatched->GetUid(),
                              sText,
                              ReadLineThis->m_uTextLen,
                              nState,
                              ReadLineThis->m_strContext.c_str(),
                              ReadLineThis->m_posMatched->m_pAppArg);
}

/*!
 * \brief Find application-specific generator associated with the first 
 * characters in the readline buffer.
 *
 * \param nLen  Length in readline buffer.
 */
void ReadLine::MatchGenerator(int nLen)
{
  m_posMatched = m_vecGenerators.end();

  m_strContext.clear();

#ifdef HAVE_READLINE

  if( nLen > 0 )
  {
    m_strContext = ReadLine::c14n(rl_line_buffer, (size_t)nLen);
  }
  else
  {
    m_strContext = "";
  }

  //fprintf(stderr, "\nDBG: matching '%s' ", m_strContext.c_str());

  for(m_posMatched = m_vecGenerators.begin();
      m_posMatched != m_vecGenerators.end();
      ++m_posMatched)
  {
    //fprintf(stderr, "DBG: %d: regex(%s) ",
    //    m_posMatched->GetUid(), m_posMatched->m_regex.GetRegEx());

    if( m_posMatched->m_regex.Match(m_strContext) )
    {
      //fprintf(stderr, "matched regex(%s)",  m_posMatched->m_regex.GetRegEx());
      break;
    }
  }
  //fprintf(stderr, "\n");
#endif // HAVE_READLINE
}

/*!
 * \brief Pointer to the only instance of readline supported per application.
 */
ReadLine *ReadLine::ReadLineThis;
