////////////////////////////////////////////////////////////////////////////////
//
// Package:     RoadNarrows Robotics ROS Chess Engine Package
//
// Link:        https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:     librnr_appkit
//
// File:        ReadLine.h
//
/*! \file
 *
 * \brief The thin ReadLine wrapper class interface.
 *
 * ReadLine provides a wrapper around the readline command-line library.
 *
 * \note Define HAVE_READLINE to enable interface with libreadline.
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

#ifndef _READ_LINE_H
#define _READ_LINE_H

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


namespace rnr
{
  namespace cmd
  {
    /*!
     * \brief Application-specific tab completion generator function type.
     *
     * \param strText     Partial text string to complete.
     * \param nState      Generator state. If FIRST,then initialize any statics.
     * \param strContext  Generator context (i.e. canonical path).
     * \param pAppArg     Generator function optional argument.
     *
     * \return
     * If a first/next match is made, return allocated completed match.\n
     * Otherwise return NULL.
     */
    typedef char *(*ReadLineGenFunc)(std::string &strText,
                                     int         nState,
                                     std::string &strContext,
                                     void       *pAppArg);
    
    
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
      static const int  FIRST   =  0;   ///< first state
      static const int  NOT_REG = -1;   ///< not registered return value
    
      /*!
       * \brief Default initialization constructor.
       *
       * \param strName   Name of readline. Used to allow for conditional
       *                  parsing of the ~/.inputrc file.
       * \param strPrompt Command line primary prompt string. Prompting occurs
       *                  only in interactive mode.
       * \param bUseRlLib Use the readline library. Note that readline must
       *                  be available and in interactive mode. Otherwise a
       *                  simple command-line interface will be used.
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
      void registerGenerator(ReadLineGenFunc fnAppGen, void *pAppArg);
    
      /*!
       * \brief Unregister application-specific generator associated with path.
       */
      void unregisterGenerator();
    
      /*!
       * \brief Interactively read a line of input from standard input (stdin).
       *
       * The readline library call readline() will be used iff:
       *  - The readline library is available (HAVE_READLINE).
       *  - The application enabled the ReadLine instance to use the readline.
       *  - library (bUseRlLib).
       *  - stdin is interactive (i.e. bound to a terminal or tty).
       *
       * Otherwise, ireadLine() is used.
       *
       * The readline() library call enables tab completion, line editing, and
       * history recording/recall.
       *
       * If in interactive mode, the user will be prompted with the currently
       * set prompt string. Any prompt is written to standard output (stdout).
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
    
      const std::string &getName() const
      {
        return m_strName;
      }

      bool haveRlLib() const
      {
#ifdef HAVE_READLINE
        return true;
#else
        return false;
#endif // HAVE_READLINE
      }

      bool useRlLib() const
      {
        return m_bUseRlLib;
      }

      bool isInteractive(FILE *fp);
    
      void setPrompt(const std::string &strPrompt)
      {
        m_strPrompt = strPrompt;
      }

      const std::string &getPrompt() const
      {
        return m_strPrompt;
      }

      bool isEoF() const
      {
        return m_bEoF;
      }

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
      static char *dupstr(const std::string &str)
      {
        return ReadLine::dupstr(str.c_str());
      }
    
      /*!
       * \brief Duplicate string.
       *
       * \param s   Null-terminated string to dup.
       *
       * \return Duplicated, allocated char *.
       */
      static char *dupstr(const char *s);
    
      /*!
       * \brief Strip copy of string of leading and trailing white space.
       *
       * \param [in] str    Input string to strip.
       *
       * \return Stripped copy.
       */
      static std::string strip(const std::string &str);
    
      /*!
       * \brief Canonicalization of a string.
       *
       * \note c14n is an cute abbreviation where 14 represents the number of
       * letters between the 'c' and 'n' in the word "canonicalization".
       *
       * \param str     String to canonicalize.
       * \param uLen    (Sub)length of string to canonicalize.
       *
       * \return Return copy of string holding canonical form.
       */
      static std::string c14n(const std::string &str, size_t uLen);
    
      /*!
       * \brief Tokenize input.
       *
       * \param [in,out] s  Input string to tokenize.
       * \param [out] tokv  Array of tokens (pointer to locations in s).
       * \param tokmax      Maximum number of tokens.
       *
       * \return Number of tokens.
       */
      static int tokenize(char *s, char *tokv[], size_t tokmax);
    
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
       * \param s   Null terminate string to count.
       *
       * \return Number of words.
       */
      static int wc(const char *s);
    
    protected:
      static ReadLine  *ThisObj;   ///< static pointer to this single instance
    
      std::string       m_strName;    ///< readline name
      std::string       m_strPrompt;  ///< prompt string
      bool              m_bUseRlLib;  ///< [do not] use readline library
      std::string       m_strLine;    ///< last read line
      bool              m_bEoF;       ///< last file op end of file condition
      bool              m_bFError;    ///< last file op file error condition
      std::string       m_strError;   ///< error string
      ReadLineGenFunc   m_fnAppGen;   ///< application-specific generator
      void             *m_pAppArg;    ///< application-specific argument

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
      static char **completionWrapper(const char *sText, int nStart, int nEnd);
    
      /*! 
       * \brief Command completion callback function.
       *
       * \param sText   Text string to complete as a command.
       * \param nStart  Start index of text string in line.
       * \param nEnd    End index of text string in line.
       *  
       * \return Array of matches, NULL if none.
       */
      char **completion(const char *sText, int nStart, int nEnd);
    
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
      static char *generatorWrapper(const char *sText, int nState);

      void setStreamStatus(FILE *fp);
    };

  } // namespace cmd
} // namespace rnr


#endif // _READ_LINE_H
