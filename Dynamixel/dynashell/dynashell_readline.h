////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Program:   dynashell
//
// File:      dynashell_readline.h
//
/*! \file
 *
 * $LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
 * $Rev: 3845 $
 *
 * \brief The Dynamixel Shell ReadLine Class.
 *
 * \note Define HAVE_READLINE to enable interface with libreadline.
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

#ifndef _DYNASHELL_READLINE_H
#define _DYNASHELL_READLINE_H

#include <sys/types.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#include <iostream>
#include <string>
#include <vector>

#ifdef HAVE_READLINE
#include <readline/readline.h>
#include <readline/history.h>
#endif // HAVE_READLINE

#include "dynashell_regex.h"

using namespace std;


// ----------------------------------------------------------------------------
// Types
// ----------------------------------------------------------------------------

/*!
 * \brief Application-specific tab completion generator function type.
 *
 * \param nUid      Registered unique id.
 * \param sText     Partial text string to complete.
 * \param uTextLen  Length of text.
 * \param nState    Generator state. If FIRST,then initialize any statics.
 * \param sContext  Generator context (i.e. canonical path).
 * \param pAppArg   Generator function optional application argument.
 *
 * \return If a first/next match is made, return allocated completed match.\n
 * Otherwise return NULL.
 */
typedef char *(*ReadLineAppGenFunc_T)(int         nUid,
                                      const char *sText,
                                      size_t      uTextLen,
                                      int         nState,
                                      const char *sContext,
                                      void       *pAppArg);


// ----------------------------------------------------------------------------
// Class ReadLineEntry
// ----------------------------------------------------------------------------

class ReadLineEntry
{
public:
  ReadLineEntry();

  ReadLineEntry(const string         &strRegEx,
                ReadLineAppGenFunc_T  fnAppGen,
                void                 *pAppArg);

  ReadLineEntry(const char           *sRegEx,
                ReadLineAppGenFunc_T  fnAppGen,
                void                 *pAppArg);

  ReadLineEntry(const ReadLineEntry &src);

  ~ReadLineEntry();

  /*!
   * \brief Check if entry is valid.
   *
   * \return Returns true or false.
   */
  bool IsValid()
  {
    return m_regex.IsValid();
  }

  /*!
   * \brief Get the unique id associated with this entry.
   *
   * \return Unique id.
   */
  int GetUid()
  {
    return m_nUid;
  }

protected:
  int                   m_nUid;     ///< unique id
  RegEx                 m_regex;    ///< applicatin matching regular expression
  ReadLineAppGenFunc_T  m_fnAppGen; ///< application-specific generator
  void                 *m_pAppArg;  ///< application-specific argument

  friend class ReadLine;
};


// ----------------------------------------------------------------------------
// Class ReadLine
// ----------------------------------------------------------------------------

/*!
 * \brief ReadLine class provides a c++ wrapper around the readline c library.
 *
 * \par Features:
 * \li line input
 * \li tab completion
 * \li history
 *
 * If libreadline is not available, then only line input is available.
 */
class ReadLine
{
public:

  static const int  FIRST = 0;      ///< first state
  static const int  NOT_REG = -1;   ///< not registered return value
  static ReadLine  *ReadLineThis;   ///< static pointer this single instance

  ReadLine(const string &strName);

  ReadLine(const char *sName);

  virtual ~ReadLine();

  /*!
   * \brief Register application-specific tab-completion generator associated.
   *
   * \param strRegEx  Regular expression applied to current readline buffer
   *                  state.
   * \param fnAppGen  Application-specific generator function.
   * \param pAppArg   Optional application argument generator function.
   *
   * \return
   * On successful registration, a unique id \h_ge 0 is returned.\n
   * On regular expression evalution failure, the generator is not registered
   * and ReadLine::NOT_REG is returned.
   */
  int RegisterGenerator(const string          strRegEx,
                        ReadLineAppGenFunc_T  fnAppGen,
                        void                 *pAppArg)
  {
    return RegisterGenerator(strRegEx.c_str(), fnAppGen, pAppArg);
  }

  int RegisterGenerator(const char           *sRegEx,
                        ReadLineAppGenFunc_T  fnAppGen,
                        void                 *pAppArg);

  void UnregisterGenerator(int nUid);

  /*!
   * \brief Interactively read a line of input from standard input.
   *
   * If the readline library feature is enabled, then the input is controlled
   * by the readline() call along with history and tab completion.
   *
   * \param sPrompt   Optional user prompt string.
   *
   * \return If no errors occurred and EOF is not encountered, an allocated,
   * null-terminated line buffer is return. Else NULL is return.
   */
  char *iReadLine(const char *sPrompt)
  {
#ifdef HAVE_READLINE
    return readline(sPrompt);          // fixed at stdin
#else
    retrun fReadLine(stdin, sPrompt);  // so then so are we
#endif // HAVE_READLINE
  }

  static char *fReadLine(FILE *fp, const char *sPrompt);

  void AddToHistory(const char *sInput);


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Built-In Tab Completion Generators
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  
  /*!
   * \brief File name tab completion generator.
   *
   * \param sText     Partial text string to complete.
   * \param nState    Generator state. If FIRST,then initialize any statics.
   *
   * \return
   * If a first/next match is made, return allocated completed match.\n
   * Otherwise return NULL.
   */
  static char *FileCompletionGenerator(const char *sText, int nState)
  {
#ifdef HAVE_READLINE
    return rl_filename_completion_function(sText, nState);
#else
    return NULL;
#endif // HAVE_READLINE
  }

  /*!
   * \brief User name tab completion generator.
   *
   * \param sText     Partial text string to complete.
   * \param nState    Generator state. If FIRST,then initialize any statics.
   *
   * \return
   * If a first/next match is made, return allocated completed match.\n
   * Otherwise return NULL.
   */
  static char *UserCompletionGenerator(const char *sText, int nState)
  {
#ifdef HAVE_READLINE
    return rl_username_completion_function(sText, nState);
#else
    return NULL;
#endif // HAVE_READLINE
  }


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Utilities
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \brief Duplicate string.
   *
   * \param str   String to dup.
   *
   * \return Duplicated, allocated char *.
   */
  static char *dupstr(const string &str)
  {
    return ReadLine::dupstr(str.c_str());
  }

  static char *dupstr(const char *s);

  static string strip(string &str);

  static char *strip(char *s);

  /*!
   * \brief Canonicalization of a string.
   *
   * \param s       Null-terminated string to canonicalize.
   * \param uLen    (Sub)length of string to canonicalize.
   *
   * \return Return string holding canonical form.
   */
  static string c14n(const string &str, size_t nLen)
  {
    return ReadLine::c14n(str.c_str(), nLen);
  }

  static string c14n(const char *s, size_t nLen);

  static int tokenize(char *s, char *tokv[], size_t tokmax);

  /*!
   * \brief Count the words in the string.
   *
   * \param str   String to count.
   *
   * \return Number of words.
   */
  static int wc(const string &str)
  {
    return ReadLine::wc(str.c_str());
  }

  static int wc(const char *s);

protected:
  /*!
   * \brief Internal registered generator map type.
   */
  typedef vector<ReadLineEntry> VecAppEntry;

  char                 *m_sName;          ///< readline name
  int                   m_nUidCounter;    ///< unique id counter
  VecAppEntry           m_vecGenerators;  ///< map of generators
  VecAppEntry::iterator m_posMatched;     ///< matched entry position
  string                m_strContext;     ///< current readline buffer context
  size_t                m_uTextLen;       ///< text length of text to generator

  static char **CompletionWrap(const char *sText, int nStart, int nEnd);

  char **Completion(const char *sText, int nStart, int nEnd);

  static char *GeneratorWrap(const char *sText, int nState);

  void MatchGenerator(int nEnd);
};


#endif // _DYNASHELL_READLINE_H
