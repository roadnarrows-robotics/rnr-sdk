////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_appkit
//
// File:      ReadLine.h
//
/*! \file
 *
 * \brief The thin ReadLine wrapper class interface.
 *
 * ReadLine provides a wrapper around the readline command-line C library.
 *
 * \note Define HAVE_READLINE to compile interface with the libreadline
 * library.
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

#ifndef _RNR_READ_LINE_H
#define _RNR_READ_LINE_H

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


#include "rnr/appkit/StringTheory.h"

/*!
 * \brief RoadNarrows Robotics Command Namespace
 */
namespace rnr
{
  namespace cmd
  {
    static const char *RlTabEnd = ""; ///< no more matches value (empty string)

    // -------------------------------------------------------------------------
    // Class ReadLine
    // -------------------------------------------------------------------------
    
    /*!
     * \brief ReadLine class provides a C++ wrapper around the readline C
     * library.
     *
     * \par Features:
     * \li line input processing
     * \li tab completion
     * \li history
     *
     * If libreadline is not available, then only simple line input processing
     * is available.
     */
    class ReadLine
    {
    public:
      static const int  FIRST = 0;  ///< first state
    
      /*!
       * \brief ReadLine flag modifiers.
       */
      enum RlFlags
      {
        FlagTabNoMods     = 0x0000, ///< no TAB modifiers
        FlagTabNoDefault  = 0x0001, ///< no default TAB completion match
        FlagTabNoFilename = 0x0002, ///< no filename TAB completion attempt
        FlagTabNoSpace    = 0x0004  ///< no space(' ') after TAB completion
      };

      /*!
       * \brief Application-specific TAB completion generator function type.
       *
       * The generator is called multiple times on a TAB completion event.
       * The call is made through the readline library rl_completion_matches()
       * C function. On the first call, nState equals ReadLine::FIRST. This
       * allows for any necessary initaliazation of (static) application
       * specific state variables. On subsequent calls, nState \h_gt 0. 
       *
       * For each matched candidate, a malloc'd string must be returned. The
       * readline library automatically frees the memory allocation after use.
       *
       * When all candidates are exhausted, return NULL to signal no more
       * matches to the readline library calling function.
       *
       * ReadLine Support Methods:
       *  - ReadLine::dupstr() - string allocation.
       *  - ReadLine::tokenize() - simple, whitespace token generator.
       *  - ReadLine::wc() count whitespace separated words.
       *
       * \param pAppArg     Generator function argument.
       * \param strText     Partial text string to complete.
       * \param nState      Generator state. If FIRST(0), initialize any
       *                    state statics.
       * \param strContext  Generator context (i.e. stripped input buffer).
       *
       * \return
       * If a first/next match is made, return allocated completed match.\n
       * Otherwise return NULL.
       */
      typedef char *(*AppGenFunc)(void               *pAppArg,
                                  const std::string  &strText,
                                  int                 nState,
                                  const std::string  &strContext);
    
      /*!
       * \brief Alternative application-specific TAB completion generator
       * function type.
       *
       * This is a somewhat more powerful TAB completion function that remains
       * entirely in C++. The calling function is ReadLine::completion()
       * which, in turn, is called by the readline library through the 
       * set rl_attempted_completion_function variable.
       *
       * The generator is called multiple times on a TAB completion event.
       * On the first call, nIndex is equal to 0. Subsequent calls, nIndex is
       * incremented by one. The function may safely use this index to sequence
       * through any application-specific data.
       *
       * Return Values:
       * -# match: Any non-empty, non-whitespace string.
       * -# empty: An empty string "" terminates TAB completion generation.
       *           RlTabEnd is equivalent to the empty string.
       *
       * Tab Modifier Flags:
       *  - FlagTabNoDefault:
       *    If this flag is set at index 0 and a match is returned, then
       *    a "no default" readline library value will be set. Otherwise the
       *    this first match  is the default.
       *  - FlagTabNoFilename:
       *    If this flag is set at the end of TAB completion (an empty string
       *    is returned), then readline library default filename completion
       *    action is disabled. (See rl_attempted_completion_over.)
       *    The readline library default action is only invoked if no strings
       *    are generated.
       *
       * Flags only apply to the current TAB completion event and are reset
       * to defaults there after.
       *
       * ReadLine Support Methods: \ref See AppGenFunc.
       *
       * \param pAppArg       Generator function argument.
       * \param strText       Partial text string to complete.
       * \param nIndex        Match candidate index starting from 0.
       * \param strContext    Generator context (i.e. input buffer).
       * \param nStart        Starting index in context of text.
       * \param nEnd          Ending index in context of the position
       *                      immediately after the end of text. If nStart
       *                      equals nEnd, then empty text.
       * \param [out] uFlags  TAB completion modifier flags.
       *
       * \return Match or empty string.
       */
      typedef const std::string (*AltAppGenFunc)(void              *pAppArg,
                                                 const std::string &strText,
                                                 int                nIndex,
                                                 const std::string &strContext,
                                                 int                nStart,
                                                 int                nEnd,
                                                 unsigned          &uFlags);
    
      /*!
       * \brief Default initialization constructor.
       *
       * \param strName   Name of readline. Used to allow for conditional
       *                  parsing of the ~/.inputrc file.
       * \param strPrompt Command line primary prompt string. Prompting occurs
       *                  only in interactive mode.
       * \param bUseRlLib Use the readline library. Note that readline must
       *                  be available and the input is interactive. Otherwise
       *                  a simple command-line interface will be used.
       */
      ReadLine(const std::string strName    = "",
               const std::string strPrompt  = "> ",
               bool              bUseRlLib  = true);
    
      /*!
       * \brief Destructor.
       */
      virtual ~ReadLine();
    
      /*!
       * \brief Register application-specific tab-completion generator.
       *
       * The registered function will be called through the readline library
       * rl_completion_matches() C function.
       *
       * \param fnAppGen  Application-specific generator function.
       * \param pAppArg   Optional application argument generator function.
       */
      void registerGenerator(AppGenFunc fnAppGen, void *pAppArg);
    
      /*!
       * \brief Register alternate application-specific tab-completion
       * generator.
       *
       * The registered function will be called through the
       * ReadLine::completion C++ method.
       *
       * \param fnAltAppGen Alt application-specific generator function.
       * \param pAppArg     Optional application argument generator function.
       */
      void registerAltGenerator(AltAppGenFunc fnAltAppGen, void *pAppArg);

      /*!
       * \brief Unregister application-specific generator associated with path.
       */
      void unregisterGenerator();
    
      /*!
       * \brief Interactively read a line of input from standard input (stdin).
       *
       * The readline library function readline() will be called iff:
       *  - The readline library is available (HAVE_READLINE).
       *  - The application enabled the ReadLine instance to use the readline.
       *  - library (bUseRlLib).
       *  - The input stdin is interactive (i.e. bound to a terminal or tty).
       *
       * Otherwise, ireadLine() is used.
       *
       * The readline() library call enables TAB completion, line editing, and
       * history recording/recall.
       *
       * If in interactive mode, the user will be prompted with the current
       * prompt string. Any prompt is written to standard output (stdout).
       *
       * \return
       * Returns the string of the line read, stripped of leading and trailing
       * whitespace. If the string is empty, then one of:
       *  - The line was all whitespace.
       *  - EOF was reached. Use \ref isEoF() to test this condition.
       *  - A file read error occurred. Use \ref isFError() to test this
       *    conition.
       */
      std::string &rlreadLine();
    
      /*!
       * \brief Interactively read a line of input from standard input (stdin).
       *
       * The readline library is not used.
       *
       * If in interactive mode, the user will be prompted with the currently
       * set prompt string. Any prompt is written to standard output (stdout).
       *
       * \note The \ref rlreadLine() call is the preferred method since it
       * automatically determines the best underlining function/method to call. 
       *
       * \return
       * Returns the string of the line read, stripped of leading and trailing
       * whitespace. If the string is empty, then one of:
       *  - The line was all whitespace.
       *  - EOF was reached. Use \ref isEoF() to test this condition.
       *  - A file read error occurred. Use \ref isFError() to test this
       *    conition.
       */
      std::string &ireadLine()
      {
        return ireadLine(stdin);
      }
    
      /*!
       * \brief Interactively read a line of input from an input stream.
       *
       * The readline library is not used.
       *
       * If in interactive mode, the user will be prompted with the currently
       * set prompt string. Any prompt is written to standard output (stdout).
       *
       * \param fp  Input file stream pointer.
       *
       * \return
       * Returns the string of the line read, stripped of leading and trailing
       * whitespace. If the string is empty, then one of:
       *  - The line was all whitespace.
       *  - EOF was reached. Use \ref isEoF() to test this condition.
       *  - A file read error occurred. Use \ref isFError() to test this
       *    conition.
       */
      std::string &ireadLine(FILE *fp);
    
      /*! 
       * \brief Read one input line from the given input stream.
       *
       * The input stream may be attached to a terminal, pipe, or file. No
       * prompting will occur.
       *
       * \param fp  Input file stream pointer.
       *
       * \return
       * Returns the string of the line read, stripped of leading and trailing
       * whitespace. If the string is empty, then one of:
       *  - The line was all whitespace.
       *  - EOF was reached. Use \ref isEoF() to test this condition.
       *  - A file read error occurred. Use \ref isFError() to test this
       *    conition.
       */
      std::string &freadLine(FILE *fp);
    
      /*
       * \brief Add input to readline history.
       *
       * The line is added if the line is not empty and does not match the last
       * command.
       *
       * \param strLine   Input line to add.
       */
      void addToHistory(const std::string &strLine);
    
      /*!
       * \brief Get this instance of readline name.
       */
      const std::string &getName() const
      {
        return m_strName;
      }

      /*!
       * \brief Test if have readline library.
       *
       * \return Returns true or false.
       */
      bool haveRlLib() const
      {
#ifdef HAVE_READLINE
        return true;
#else // !HAVE_READLINE
        return false;
#endif // HAVE_READLINE
      }

      /*!
       * \brief Test if the readline library is enabled.
       *
       * \return Returns true or false.
       */
      bool useRlLib() const
      {
        return m_bUseRlLib;
      }

      /*!
       * \brief Test if input file pointer is interactive.
       *
       * A file input stream is interactive if it is bound to a terminal or
       * tty.
       *
       * \return Returns true or false.
       */
      bool isInteractive(FILE *fp);
    
      /*!
       * \brief Set the prompt string.
       *
       * \param strPrompt   New prompt string.
       */
      void setPrompt(const std::string &strPrompt)
      {
        m_strPrompt = strPrompt;
      }

      /*!
       * \brief Get the current prompt string.
       *
       * \return String.
       */
      const std::string &getPrompt() const
      {
        return m_strPrompt;
      }

      /*!
       * \brief Get last read line.
       *
       * \return String.
       */
      const std::string &getLastRead() const
      {
        return m_strLine;
      }

      /*!
       * \brief Get the line number of the last read line.
       *
       * \return Line number.
       */
      size_t getLineNum() const
      {
        return m_uLineNum;
      }

      /*!
       * \brief Set the current line number.
       *
       * \param Line number.
       */
      void setLineNum(const size_t uLineNum)
      {
        m_uLineNum = uLineNum;
      }

      /*!
       * \brief Reset the line number to zero.
       */
      void resetLineNum()
      {
        m_uLineNum = 0;
      }

      /*!
       * \brief Test if the last read operation resulted in an end-of-file 
       * condition.
       *
       * \return Returns true or false.
       */
      bool isEoF() const
      {
        return m_bEoF;
      }

      /*!
       * \brief Test if the last read operation resulted in an I/O error
       * condition.
       *
       * \return Returns true or false.
       */
      bool isFError() const
      {
        return m_bFError;
      }

      /*!
       * \brief Get the most recently set error string.
       *
       * \return Error string.
       */
      const std::string &getErrorStr() const;

    
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Built-In Tab Completion Generators
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      
      /*!
       * \brief Filename TAB completion generator.
       *
       * A completion generator for filenames.
       *
       * \param sText     Partial text string to complete.
       * \param nState    Generator state. If FIRST,then initialize any statics.
       *
       * \return
       * If a first/next match is made, return allocated completed match.\n
       * Otherwise return NULL.
       */
      static char *filenameCompletionGenerator(const char *sText, int nState)
      {
    #ifdef HAVE_READLINE
        return rl_filename_completion_function(sText, nState);
    #else // !HAVE_READLINE
        return NULL;
    #endif // HAVE_READLINE
      }
    
      /*!
       * \brief Username TAB completion generator.
       *
       * A completion generator for usernames.
       *
       * \param sText     Partial text string to complete.
       * \param nState    Generator state. If FIRST,then initialize any statics.
       *
       * \return
       * If a first/next match is made, return allocated completed match.\n
       * Otherwise return NULL.
       */
      static char *usernameCompletionGenerator(const char *sText, int nState)
      {
    #ifdef HAVE_READLINE
        return rl_username_completion_function(sText, nState);
    #else // !HAVE_READLINE
        return NULL;
    #endif // HAVE_READLINE
      }
    

      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Utilities
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    
      /*!
       * \brief Duplicate string.
       *
       * The readline library requires allocated char*'s.
       *
       * \param str   String to dup.
       *
       * \return Duplicated, allocated char*.
       */
      static char *dupstr(const std::string &str)
      {
        return ReadLine::dupstr(str.c_str());
      }
    
      /*!
       * \brief Duplicate string.
       *
       * The readline library requires allocated char*'s.
       *
       * \param s   Null-terminated string to dup.
       *
       * \return Duplicated, allocated char *.
       */
      static char *dupstr(const char *s);
    
      /*!
       * \brief Tokenize the given string.
       *
       * Tokens are contiguous non-whitespace character sequences, separated by
       * either the start-of-string, whitespace, or the end-of-string.
       *
       * \param [in] str      Input string to generate tokens.
       * \param [out] tokens  Vector of tokens.
       *
       * \return Number of tokens.
       */
      static size_t tokenize(const std::string &str, str::StringVec &tokens);
    
      /*!
       * \brief In-place string tokenizer.
       *
       * Tokens are contiguous non-whitespace character sequences, separated by
       * either the start-of-string, whitespace, or the end-of-string.
       *
       * The '\0' character is inserted at the end of each detected word in s.
       * No memory allocation is used.
       *
       * \param [in,out] s  Input/output null-terminated string to tokenize.
       * \param [out] tokv  Array of tokens (pointer to locations in s).
       * \param [in] tokmax Maximum number of tokens.
       *
       * \return Number of tokens.
       */
      size_t tokenize(char *s, char *tokv[], size_t tokmax);

      /*!
       * \brief Count the words in the string.
       *
       * \param str   String to count.
       *
       * \return Number of words.
       */
      static int wc(const std::string &str)
      {
        return ReadLine::wc(str.c_str());
      }
    
      /*!
       * \brief Count the words in the string.
       *
       * \param s   Null-terminated string to count.
       *
       * \return Number of words.
       */
      static int wc(const char *s);
    
    protected:
      static ReadLine *ThisObj;     ///< static pointer to this single instance
    
      std::string     m_strName;    ///< readline name
      std::string     m_strPrompt;  ///< prompt string
      bool            m_bUseRlLib;  ///< [do not] use readline library
      std::string     m_strLine;    ///< last read line
      size_t          m_uLineNum;   ///< line number
      bool            m_bEoF;       ///< last file op end of file condition
      bool            m_bFError;    ///< last file op file error condition
      std::string     m_strError;   ///< error string
      AppGenFunc      m_fnAppGen;   ///< application-specific generator
      AltAppGenFunc   m_fnAltAppGen;///< alternate app-specific generator
      void           *m_pAppArg;    ///< application-specific argument

      /*! 
       * \brief Command completion callback function wrapper.
       * 
       * Wraps method ReadLine::completion().
       *
       * \param sText   Text string to complete as a command.
       * \param nStart  Start index of text string in line.
       * \param nEnd    End index of text string in line.
       *  
       * \return Array of matches, NULL if none.
       */
      static char **completionWrapper(const char *sText, int nStart, int nEnd);
    
      /*! 
       * \brief Command completion callback function.
       *
       * Attempt to complete on the contents of text. The start and end bound
       * the region of rl_line_buffer that contains the word to complete. Text
       * is the word to complete.  We can use the entire contents of
       * rl_line_buffer to provide context.
       *
       * \param sText   Text string to complete as a command.
       * \param nStart  Start index of text string in line.
       * \param nEnd    End index of text string in line.
       *  
       * \return Array of matches, NULL if none.
       */
      char **completion(const char *sText, int nStart, int nEnd);
    
      /*! 
       * \brief Command completion callback function.
       *
       * If an alternative application-specific generator is registered, this
       * method is used to provide the requisite C++ interface.
       *
       * Attempt to complete on the contents of text. The start and end bound
       * the region of rl_line_buffer that contains the word to complete. Text
       * is the word to complete.  We can use the entire contents of
       * rl_line_buffer to provide context.
       *
       * This method also provides the indexing for the registered generator.
       *
       * \param sText   Text string to complete as a command.
       * \param nStart  Start index of text string in line.
       * \param nEnd    End index of text string in line.
       *  
       * \return Array of matches, NULL if none.
       */
      char **altCompletion(const std::string strText, int nStart, int nEnd);

      /*!
       * \brief Generator wrapper.
       *
       * Calls the registered application-specific generator.
       *
       * \param sText     Partial text string to complete.
       * \param nState    Generator state. If FIRST,then initialize any statics.
       *
       * \return
       * If a first/next match is made, return allocated completed match.\n
       * Otherwise return NULL.
       */
      static char *generatorWrapper(const char *sText, int nState);

      /*!
       * \brief Set stream status for the last read operation.
       *
       * \param fp  Input file pointer.
       */
      void setStreamStatus(FILE *fp);

      /*!
       * \brief Clear stream status prior to next read operation.
       */
      void clearStreamStatus();
    };

  } // namespace cmd
} // namespace rnr


#endif // _RNR_READ_LINE_H
