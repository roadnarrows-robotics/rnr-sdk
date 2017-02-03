////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_appkit
//
// File:      CommandLine.h
//
/*! \file
 *
 * \brief Command-Line parser class interface
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

#ifndef _COMMAND_LINE_H
#define _COMMAND_LINE_H

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

#include <iostream>
#include <string>
#include <vector>
#include <map>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/appkit/LogBook.h"
#include "rnr/appkit/ReadLine.h"


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Doxygen "macros"
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \defgroup doc_return_cl doc_return_cl
 * \{
\return
On success, Ok(0) is returned.
On error, a CommandLine error code (\< 0) is returned.
See \ref getErroStr() and backtrace() to retrieve or print error(s).
 * \}
 *
 * \defgroup doc_return_ssize doc_return_ssize
 * \{
\return
On success, returns size (count).
On error, a CommandLine error code (\< 0) is returned.
See \ref getErroStr() and backtrace() to retrieve or print error(s).
 * \}
 */


/*!
 * \brief RoadNarrows Robotics Command
 */
namespace rnr
{
  namespace cmd
  {
    /*!
     * \brief Useful types.
     */
    typedef std::vector<std::string>  StringVec;  ///< vector of strings type
    typedef int (*CmdExecFunc)(const StringVec&); ///< cmd exec function type
  
    /*!
     * \brief User available command execution structure.
     * 
     * This structure is provided as a convenience type for application
     * development. It is external to the core CommandLine functionality.
     */
    struct CmdExec
    {
      const char  *m_sName;       ///< command name
      const char  *m_sSyntax;     ///< parsable command extended usage syntax
      const char  *m_sSynopsis;   ///< short command synopsis
      const char  *m_sDesc;       ///< long command description
      CmdExecFunc  m_fnExec;      ///< command execution function
    };
  
    /*!
     * \brief Print help for commands.
     *
     * \param cmds        Array of commands.
     * \param uNumOfCmds  Number of commands.
     * \param strCmdName  If not an empty string, the command to print help.
     *                    Otherwise all commands help is printed.
     * \param bLongHelp   Print long or short help.
     */
    int help(const CmdExec      cmds[],
             size_t             uNumOfCmds,
             const std::string  &strCmdName,
             bool               bLongHelp = true);

    /*!
     * \brief Variable argument types.
     */
    static const char *ArgLiteral       = "literal";  ///< literal constant
    static const char *ArgWord          = "word";     ///< contiguous [^\s] seq
    static const char *ArgInteger       = "int";      ///< integer (long)
    static const char *ArgFloat         = "float";    ///< float (double)
    static const char *ArgQuotedString  = "quoted-string"; ///< "c..."
    static const char *ArgFile          = "file";     ///< file path
    static const char *ArgRegEx         = "re";       ///< regular expression
  

    //--------------------------------------------------------------------------
    // Token Class
    //--------------------------------------------------------------------------
  
    /*!
     * \brief Parsed token class.
     */
    class Token
    {
    public:
      std::string m_strValue; ///< token string value
      // TODO replace m_uLineNum,Pos with m_uStartPos, m_uEndPos
      size_t      m_uLineNum; ///< line number (always 0 if interactive mode)
      size_t      m_uLinePos; ///< start to token value in line position

      /*!
       * \brief Default construtor.
       */
      Token();
      
      /*!
       * \brief Copy construtor.
       *
       * \param src Source object.
       */
      Token(const Token &src);

      /*!
       * \brief Destructor.
       */
      virtual ~Token();

      /*!
       * \brief Assignment operator.
       *
       * \param rhs   Right-hand side object.
       *
       * \return *this
       */
      Token &operator=(const Token &rhs);
  

      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Output Methods and Operators
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

      /*!
       * \brief Ouput token and location of token in line to stream.
       *
       * \verbatim
       * [linenum:]linepos "token"
       * line of text where token was generated
       *                    ^
       * \endverbatim
       *
       * \param os        Output stream.
       * \param strLine   Line whence token was generated.
       *
       * \return Reference to output stream.
       */
      std::ostream &oloc(std::ostream &os, const std::string &strLine);

      /*!
       * \brief Insert object into output stream.
       *
       * \param os  Output stream.
       * \param tok Object to insert.
       *
       * \return Reference to output stream.
       */
      friend std::ostream &operator<<(std::ostream &os, const Token &tok);
    };

    //
    // Types
    //
    typedef std::vector<Token> TokenVec;  ///< vector of tokens type


    //--------------------------------------------------------------------------
    // CmdArgDef Class
    //--------------------------------------------------------------------------
  
    /*!
     * \brief Command argument compiled definition class.
     */
    class CmdArgDef
    {
    public:
      /*!
       * \brief Argument type enumeration.
       */
      enum ArgType
      {
        TypeUndef   = 0,  ///< undefined argument type
        TypeLiteral,      ///< literal constant
        TypeWord,         ///< any non-whitespace contiguous character sequence
        TypeInteger,      ///< integer (long)
        TypeFloat,        ///< floating point number (double)
        TypeQuotedString, ///< escape sequence supported "c..." dqouted string
        TypeFile,         ///< file path
        TypeRegEx         ///< regular expression
      };
  
      /*!
       * \brief Argument flag modifiers.
       */
      enum ArgFlags
      {
        FlagCommand   = 0x0001,   ///< argument is the command
        FlagOptional  = 0x0002,   ///< argument is optional
        FlagXorList   = 0x0004    ///< argument has a mutually exclusive list
      };

      /*!
       * \brief Default construtor.
       */
      CmdArgDef();
  
      /*!
       * \brief Copy construtor.
       *
       * \param src Source object.
       */
      CmdArgDef(const CmdArgDef &src);

      /*!
       * \brief Destructor.
       */
      virtual ~CmdArgDef();
  
      /*!
       * \brief Assignment operator.
       *
       * \param rhs   Right-hand side object.
       *
       * \return *this
       */
      CmdArgDef &operator=(const CmdArgDef &rhs);
  

      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Public Attribute Methods
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

      /*
       * \brief Test if argument definition is sufficiently defined.
       *
       * \return Returns true or false.
       */
      bool isDefined() const;

      /*!
       * \brief Get parent command's unique id.
       *
       * \return Unique id.
       */
      int getParentCmdUid() const
      {
        return m_nCmdUid;
      }

      /*!
       * \brief Get parent command form's index.
       *
       * \return Zero based index.
       */
      int getParentFormIndex() const
      {
        return m_nFormIndex;
      }

      /*!
       * \brief Get argument's command line index.
       *
       * \return Zero based index..
       */
      int getIndex() const
      {
        return m_nIndex;
      }

      /*!
       * \brief Get argument's name.
       *
       * \return Name string.
       */
      const std::string &getName() const
      {
        return m_strName;
      }

      /*!
       * \brief Get argument's type.
       *
       * \return Type enum.
       */
      ArgType getType() const
      {
        return m_eType;
      }

      /*!
       * \brief Get the number of mutually exclusive literals.
       *
       * \return Number.
       */
      int numOfLiterals() const
      {
        return (int)m_values.size();
      }

      /*!
       * \brief Get literal value at index.
       *
       * \return String value.
       */
      const std::string &literalAt(const int nIndex) const;

      /*!
       * \brief Get argument's modifier flags.
       *
       * \return Bit-or'ed flags.
       */
      unsigned getFlags() const
      {
        return m_uFlags;
      }


      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Pattern Matching Methods
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

      /*!
       * \brief Match argument string against argument definition pattern.
       *
       * \param strArg      Argument value.
       * \param bIgnoreCase Do [not] ignore case when applying pattern matching.
       *
       * \return Returns true or false.
       */
      bool match(const std::string &strArg, bool bIgnoreCase = false) const;


      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Output Methods and Operators
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

      /*!
       * \brief Insert object into output stream.
       *
       * \param os      Output stream.
       * \param argdef  Object to insert.
       *
       * \return Reference to output stream.
       */
      friend std::ostream &operator<<(std::ostream    &os,
                                      const CmdArgDef &argdef);
  
      //
      // Friends
      //
      friend class CmdFormDef;
      friend class CmdDef;
      friend class CommandLine;

    protected:
      int         m_nCmdUid;    ///< parent command's unique id
      int         m_nFormIndex; ///< parent command's form index
      int         m_nIndex;     ///< argument index
      std::string m_strName;    ///< argument name
      ArgType     m_eType;      ///< argument type
      StringVec   m_values;     ///< argument values
      unsigned    m_uFlags;     ///< argument modifiers

      /*!
       * \brief Set parent's command and form id's.
       *
       * \param nCmdUid     Command unique id.
       * \param nFormIndex  Zero based form index within command.
       */
      void setParent(const int nCmdUid, const int nFormIndex);

      /*!
       * \brief Set arguments's command line index.
       *
       * \param nIndex  Zero based index.
       */
      void setIndex(const int nIndex);

      /*!
       * \brief Set argument's name.
       *
       * \param strName Name string.
       */
      void setName(const std::string &strName);

      /*!
       * \brief Set argument's type.
       *
       * \param eType Type enum.
       */
      void setType(const ArgType eType);

      /*!
       * \brief Add literal value to list of argument values.
       *
       * \param strValue    Value to add.
       */
      void addLiteralValue(const std::string &strValue);

      /*!
       * \brief Or flags into argment modifier flags.
       *
       * \param uFlags  Bit list of flags.
       */
      void orFlags(const unsigned uFlags);
    };
  
    //
    // Types
    //
    typedef std::vector<CmdArgDef> ArgDefVec;  ///< vector of argument defs


    //--------------------------------------------------------------------------
    // CmdFormDef Class
    //--------------------------------------------------------------------------

    class CmdFormDef
    {
    public:
      /*!
       * \brief Default construtor.
       */
      CmdFormDef();
  
      /*!
       * \brief Initialization construtor.
       */
      CmdFormDef(const std::string &strSyntax);
  
      /*!
       * \brief Copy construtor.
       *
       * \param src Source object.
       */
      CmdFormDef(const CmdFormDef &src);

      /*!
       * \brief Destructor.
       */
      virtual ~CmdFormDef();
  
      /*!
       * \brief Assignment operator.
       *
       * \param rhs   Right-hand side object.
       *
       * \return *this
       */
      CmdFormDef &operator=(const CmdFormDef &rhs);
  

      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Public Attribute Methods
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

      /*
       * \brief Test if form definition is sufficiently defined.
       *
       * \return Returns true or false.
       */
      bool isDefined() const;

      /*!
       * \brief Get parent command's unique id.
       *
       * \return Unique id.
       */
      int getParentCmdUid() const
      {
        return m_nCmdUid;
      }

      /*!
       * \brief Get forms's index.
       *
       * \return Zero based index.
       */
      int getIndex() const
      {
        return m_nIndex;
      }

      /*!
       * \brief Get form's extended usage syntax.
       */
      std::string getSyntax() const
      {
        return m_strSyntax;
      }

      /*!
       * \brief Get form's argument at index.
       *
       * \return Command argument definition reference.
       */
      const CmdArgDef &argAt(const int nIndex) const;

      /*!
       * \brief Get the total number of arguments.
       *
       * The first argument (argv0) is the command.
       *
       * \return Number of arguments.
       */
      int numOfArgs() const
      {
        return (int)m_argdefs.size();
      }

      /*!
       * \brief Get the total number of required arguments.
       *
       * Required arguments start at argument index 1.
       *
       * \return Number of arguments.
       */
      int numOfRequiredArgs() const
      {
        return m_nArgcReq;
      }

      /*!
       * \brief Get the total number of optional arguments.
       *
       * Optional arguments start at argument index 1+numOfRequiredArgs(().
       *
       * \return Number of arguments.
       */
      int numOfOptionalArgs() const
      {
        return m_nArgcOpt;
      }
  

      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Output Methods and Operators
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

      /*!
       * \brief Insert object into output stream.
       *
       * \param os      Output stream.
       * \param formdef Object to insert.
       *
       * \return Reference to output stream.
       */
      friend std::ostream &operator<<(std::ostream     &os,
                                      const CmdFormDef &formdef);

      //
      // Friends
      //
      friend class CmdDef;
      friend class CommandLine;

    protected:
      int         m_nCmdUid;    ///< parent command's unique id
      int         m_nIndex;     ///< forms index
      std::string m_strSyntax;  ///< command extened usage syntax
      ArgDefVec   m_argdefs;    ///< command argument definitions
      int         m_nArgcReq;   ///< number of required arguments
      int         m_nArgcOpt;   ///< number of optional arguments

      /*!
       * \brief Set parent's command id.
       *
       * \param nCmdUid     Command unique id.
       */
      void setParent(const int nCmdUid);

      /*!
       * \brief Set forms's index.
       *
       * \param nIndex  Zero based index.
       */
      void setIndex(const int nIndex);

      /*!
       * \brief Set command's extended usage syntax.
       *
       * \param strSyntax   Syntax string.
       */
      void setSyntax(const std::string &strSyntax);

      /*!
       * \brief Push argument to end of argument list.
       *
       * \param argdef Command argument definition.
       */
      void pushArg(CmdArgDef &argdef);

      /*!
       * \brief Get form's argument at index.
       *
       * \return Command argument definition reference.
       */
      CmdArgDef &argAt(const int nIndex);

      /*!
       * \brief Get the last pushed argument.
       *
       * \return Argument definition reference.
       */
      CmdArgDef &lastArg();
    };

    //
    // Types
    //
    typedef std::vector<CmdFormDef> CmdFormDefVec;  ///< vector of command forms


    //--------------------------------------------------------------------------
    // CmdDef Class
    //--------------------------------------------------------------------------
  
    /*!
     * \brief Command compiled definition class.
     */
    class CmdDef
    {
    public:
      /*!
       * \brief Default construtor.
       */
      CmdDef();
  
      /*!
       * \brief Copy construtor.
       *
       * \param src Source object.
       */
      CmdDef(const CmdDef &src);

      /*!
       * \brief Destructor.
       */
      virtual ~CmdDef();
  
      /*!
       * \brief Assignment operator.
       *
       * \param rhs   Right-hand side object.
       *
       * \return *this
       */
      CmdDef &operator=(const CmdDef &rhs);
  

      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Public Attribute Methods
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

      /*
       * \brief Test if command definition is sufficiently defined.
       *
       * \return Returns true or false.
       */
      bool isDefined() const;

      /*!
       * \brief Get command's unique id.
       *
       * \return Unique id.
       */
      int getUid() const
      {
        return m_nUid;
      }

      /*!
       * \brief Get command's name.
       *
       * \return Name string.
       */
      const std::string &getName() const
      {
        return m_strName;
      }

      /*!
       * \brief Get the total number command forms.
       *
       * \return Number of forms.
       */
      int numOfForms() const
      {
        return (int)m_formdefs.size();
      }

      /*!
       * \brief Get command form at index.
       *
       * \param nIndex  Form's index.
       *
       * \return Command form definition reference.
       */
      const CmdFormDef &formAt(const int nIndex) const;

      /*!
       * \brief Insert object into output stream.
       *
       * \param os      Output stream.
       * \param cmddef  Object to insert.
       *
       * \return Reference to output stream.
       */
      friend std::ostream &operator<<(std::ostream &os, const CmdDef &cmddef);

      //
      // Friends
      //
      friend class CommandLine;

    protected:
      int           m_nUid;       ///< command unique id
      std::string   m_strName;    ///< command name
      CmdFormDefVec m_formdefs;   ///< vector of command forms
 
      /*!
       * \brief Set command's unique id.
       *
       * \param uid     Unique id.
       */
      void setUid(const int uid);

      /*!
       * \brief Set command's name.
       *
       * \param strName Name string.
       */
      void setName(const std::string &strName);

      /*!
       * \brief Push new command form to end of form list.
       *
       * \param formdef   Command form definition.
       */
      void pushForm(CmdFormDef &formdef);

      /*!
       * \brief Get command form at index.
       *
       * \param nIndex  Form's index.
       *
       * \return Command form definition reference.
       */
      CmdFormDef &formAt(const int nIndex);
    };
  
  
    //--------------------------------------------------------------------------
    // CommandLine Class
    //--------------------------------------------------------------------------
  
    /*!
     * \brief CommandLine class.
     */
    class CommandLine
    {
    public:
      /*!
       * \brief Error return codes.
       */
      static const int EError       = -1;  ///< general, unspecified error
      static const int EEoF         = -2;  ///< end of file
      static const int ERead        = -3;  ///< read error
      static const int EAmbigCmd    = -4;  ///< ambiguous command
      static const int EUnknownCmd  = -5;  ///< unknown, unmatched command
      static const int EBadSyntax   = -6;  ///< bad syntax
  
      static const int Ok           =  OK; ///< (0) no error, success, good

      static const int NoUid = -1;  ///< no unique id

      /*!
       * \brief Default initialization constructor.
       *
       * \param strName     Name of command set and entry into readline's
       *                    conditional parsing of the ~/.inputrc file.
       * \param strPrompt   Command line primary prompt string. Prompting occurs
       *                    only in interactive mode.
       * \param bUseRlLib   Use the readline library. Note that readline must
       *                    be available and in interactive mode. Otherwise a
       *                    simple command-line interface will be used.
       * \param bIgnoreCase Ignore case on comands and arguments.
       */
      CommandLine(const std::string strName     = "cl",
                  const std::string strPrompt   = "> ",
                  bool              bUseRlLib   = true,
                  bool              bIgnoreCase = false);
    
      /*!
       * \brief Destructor.
       */
      virtual ~CommandLine();
  
      /*
       * \brief Test if command line is sufficiently defined.
       *
       * \return Returns true or false.
       */
      bool isDefined() const;
  

      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Public Command Addition and Compile Methods
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

      /*!
       * \brief Add command to command line interface.
       *
       * The command syntax may specify multiple command forms, each separated
       * by a newline '\n' character. The command named in each form must be
       * the identical.
       *
       * This is a (somewhat) light-weight function. A call to compile() is
       * necessary to finalized command syntax processing.
       *
       * \return
       * On success, returns command's assigned unique id.
       * Otherwise NoUid is returned.
       */
      virtual int addCommand(const std::string strMultiFormSyntax);
  
      /*!
       * \brief Compile all added commands.
       *
       * Compiling essentially parses the command extended usage syntax and
       * sets the internal data for:
       *  - readline tab completion
       *  - input to command pattern matching
       *  - first-level command validation
       *
       *  \copydoc doc_return_cl
       */
      virtual int compile();
  
      /*!
       * \brief Compile a command.
       *
       * \sa CommandLine::compile()
       *
       * \param cmddef  Command definition.
       *
       *  \copydoc doc_return_cl
       */
      virtual int compile(CmdDef &cmddef);
  

      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Public Command Line Interface Methods
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  
      /*!
       * \brief Read an input line stdin and match to best compiled command.
       *
       * \param [out] uid   Matched command unique id.
       * \param [out] argv  Vector of arguments, with argv[0] being the
       *                    command name.
       *
       *  \copydoc doc_return_cl
       */
      virtual int readCommand(int &uid, StringVec &argv)
      {
        return readCommand(stdin, uid, argv);
      }

      /*!
       * \brief Read an input line file fp and match to best compiled command.
       *
       * \param fp          Input file pointer.
       * \param [out] uid   Matched command unique id.
       * \param [out] argv  Vector of arguments, with argv[0] being the
       *                    command name.
       *
       *  \copydoc doc_return_cl
       */
      virtual int readCommand(FILE *fp, int &uid, StringVec &argv);
  
      /*!
       * \brief Add command to history.
       *
       * \note Only available if readline is available and enabled, and
       * input is interactive.
       *
       * \param argv  Command in argv vector.
       */
      virtual void addToHistory(const StringVec &argv);
  
      /*!
       * \brief Push prompt string onto stack of prompts. 
       *
       * If in interactive mode, the user will be prompted with the prompt
       * string found at the top of the prompt stack. If the stack or prompt
       * string is empty, no prompt will be displayed. Any prompt is written
       * to standard output (stdout).
       *
       * \param strPrompt   Prompt string.
       */
      void pushPrompt(const std::string &strPrompt);

      /*!
       * \brief Pop prompt string from stack of prompts. 
       *
       * The new top prompt string will be used for user prompting.
       */
      void popPrompt();

      /*!
       * \brief Get the current prompt string.
       *
       * \return String.
       */
      const std::string &getPrompt() const;


      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Public Input Processing Methods
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

      /*!
       * \brief Lexically analyze string to generate a series of tokens.
       *
       * Tokens are separated by whitespace.
       *
       * Each tokens is either:
       * * A contiguous sequence of non-whitespace characters.
       * * An escapable sequence of characters delineated by double quotes '"'.
       *
       * \verbatim
       * <token-list>    ::= <token>
       *                   | <token-list> <token>
       *
       * <token>         ::= <word>
       *                   | <quoted-string>
       *
       * <word>          ::= {NONWHITE_CHAR}+
       *
       * <qouted-string> ::= '"' {ESCAPABLE_CHAR}* '"'
       * \endverbatim
       *
       * \param [in] strInput Input string to analyze.
       * \param [out] tokens  Generated string tokens.
       *
       *  \copydoc doc_return_ssize
       */
      virtual ssize_t tokenize(const std::string &strInput, StringVec &tokens);
  

      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Public Attribute and Data Access Methods
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

      /*!
       * \brief Get command's name.
       *
       * \return Name string.
       */
      const std::string &getName() const
      {
        return m_strName;
      }

      /*!
       * \brief Get command with unique id.
       *
       * \param uid   Command unique id.
       *
       * \return Command definition reference.
       */
      const CmdDef &cmdAt(const int uid) const;

      /*!
       * \brief Get command with name.
       *
       * \param strName Command name.
       *
       * \return Command definition reference.
       */
      const CmdDef &cmdAt(const std::string &strName) const;

      /*!
       * \brief Get the total number of added commands.
       *
       * \return Number of commands.
       */
      int numOfCmds() const
      {
        return (int)m_cmddefs.size();
      }

      /*!
       * \brief Get the most recent error.
       *
       * \return Error string.
       */
      const std::string &getErrorStr() const;

      /*!
       * \brief Insert trace and error log backtrace into output stream.
       *
       * \param os    Output stream.
       * \param bAll  If true, backtrace all of log. Otherwise backtrace to
       *              last major execution mark.
       *
       * \return Reference to output stream.
       */
      std::ostream &backtrace(std::ostream &os, bool bAll = false) const;
  

      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Output Methods and Operators
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

      /*!
       * \brief Insert object into output stream.
       *
       * \param os  Output stream.
       * \param cl  Object to insert.
       *
       * \return Reference to output stream.
       */
      friend std::ostream &operator<<(std::ostream &os, const CommandLine &cl);
  

      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Static Convenience Methods
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

      /*!
       * \brief Prettify string.
       *
       * Binary characters are converted to ascii escape hex sequences. Control
       * characters are converted to standard ascii escape sequences.
       *
       * \param str String to make pretty.
       *
       * \return Prettified version of string.
       */
      static std::string prettify(const std::string &str);

      /*!
       * \brief Canonicalization of a string.
       *
       * \note The name c14n is an cute abbreviation where 14 represents the
       * number of letters between the 'c' and 'n' in the word
       * "canonicalization".
       *
       * \param tokens  String tokens to canonicalize.
       *
       * \return Return string holding canonical form.
       */
      static std::string c14n(const StringVec &tokens);

      /*!
       * \brief Convert string to a long integer.
       *
       * \param [in] str    String in hex, decimal, or octal format.
       * \param [out] val   Converted long value.
       *
       * \return Returns Ok if a valid long, EBadSyntax otherwise.
       */
       static int strToLong(const std::string &str, long &val);

      /*!
       * \brief Convert string to a double-precision floating-point number.
       *
       * \param [in] str    String in hex, decimal, or octal format.
       * \param [out] val   Converted double value.
       *
       * \return Returns Ok if a valid double, EBadSyntax otherwise.
       */
       static int strToDouble(const std::string &str, double &val);

    protected:
      /*!
       * Useful types.
       */
      typedef std::map<unsigned, CmdDef>  CmdDefMap;    ///< command map type
      typedef CmdDefMap::iterator         CmdIter;      ///< cmd iterator type
      typedef CmdDefMap::const_iterator   CmdConstIter; ///< cmd const iter type
      typedef StringVec                   PromptStack;  ///< prompt stack type

      std::string m_strName;      ///< name of this command line
      bool        m_bIgnoreCase;  ///< do [not] ignore case on commands
      int         m_nUidCnt;      ///< unique id counter
      bool        m_bIsCompiled;  ///< has [not] been successfully compiled
      CmdDefMap   m_cmddefs;      ///< map of added command definitions
      PromptStack m_prompts;      ///< stack of prompt strings
      ReadLine    m_readline;     ///< readline interface
      LogBook     m_log;          ///< trace and error log

      /*!
       * \brief Get command with the given unique id.
       *
       * \param uid   Command unique id.
       *
       * \return Command definition reference.
       */
      CmdDef &cmdAt(const int uid);

      /*!
       * \brief Get command with the given name.
       *
       * \param strName Command name.
       *
       * \return Command definition reference.
       */
      CmdDef &cmdAt(const std::string &strName);
  

      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Lexical Analyzer Methods
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

      /*!
       * \brief Extract command name from extended usage syntax 
       *
       * \param strSyntax   Syntax string.
       *
       * \return Comamnd name (argv0).
       */
      std::string extractArgv0(const std::string &strSyntax);

      /*!
       * \brief Lexically analyze extened usage syntax string to generate a
       * series of tokens.
       *
       * Tokens are separated by either whitespace or syntax special characters.
       *
       * \param [in] strSyntax  Input string to analyze.
       * \param [out] tokens    Generated tokens.
       *
       *  \copydoc doc_return_ssize
       */
      virtual ssize_t tokenizeSyntax(const std::string &strSyntax,
                                     StringVec         &tokens);

      /*!
       * \brief Syntax word lexical analyzer.
       *
       * A syntax word is define as any contiguous sequence of non-whitespace,
       * non syntax special symbol characters.
       *
       * Any generated token is place at the end of the vector.
       *
       * \param [in] str        Input string to analyze.
       * \param [in,out] tokens Vector of generated tokens. 
       *
       * \return Returns the length of the token. Zero (0) indicates no token.
       */
      virtual size_t lexSyntaxWord(const std::string &str, StringVec &tokens);

      /*!
       * \brief Word lexical analyzer.
       *
       * A word is define as any contiguous sequence of non-whitespace
       * characters.
       *
       * Any generated token is place at the end of the vector.
       *
       * \param [in] str        Input string to analyze.
       * \param [in,out] tokens Vector of generated tokens. 
       *
       * \return Returns the length of the token. Zero (0) indicates no token.
       */
      virtual size_t lexWord(const std::string &str, StringVec &tokens);

      /*!
       * \brief Quoted string lexical analyzer.
       *
       * Syntax: "..."
       *
       * The unescaped double quotes are not included in the token. Escape
       * sequences "\<c>" and "\<h><h>" are recognized and processed.
       *
       * Any generated token is place at the end of the vector.
       *
       * \param [in] str        Input string to analyze.
       * \param [in,out] tokens Vector of generated tokens. 
       *
       * \return Returns the length of the token. Zero (0) indicates either
       * no token or an empty string ("").
       */
      virtual size_t lexQuotedString(const std::string &str, StringVec &tokens);

      /*!
       * \brief Push token to the end of the generated tokens.
       *
       * \param tok             Token to push.
       * \param [in,out] tokens Vector of lexical tokens.
       */
      void pushToken(std::string &tok, StringVec &tokens);
  

      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Extended Usage Syntax Parsing Methods
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      
      /*!
       * \brief Parse command syntax.
       *
       * The parsed, generated internal data are used for readline tab
       * completion, input - command pattern matching, and first-level
       * validation.
       *
       * The parseSyntax() function [in]directly calls the CommandLine::parse
       * family of member functions. Each function takes, at a minimum, four
       * parameters:
       *  Parameter | Description
       *  --------- | -----------
       *  cmddef    | Definition of an added command. It may be modified.
       *  form      | Definition of a command form. It may be modified.
       *  tokens    | Lexical tokens generated from the extended usage syntax.
       *  pos       | The parse cursor position. On input, it specifies \
       *              the starting position in the tokens vector. On a \
       *              succesful parse, the cursor position is advanced \
       *              to the first token after the relevant syntax block.
       *
       *
       * \par Syntax:
       * \verbatim
       * <command> ::= <argv0> [<required-arg-list>] [<optional-arg-list>]
       * \endverbatim
       *
       * \param [in,out] cmddef An added command object.
       * \param [in,out] form   Command form definition. 
       * \param [in] tokens     Lexical tokens generated from the syntax usage.
       *
       *  \copydoc doc_return_cl
       */
      virtual int parseSyntax(CmdDef          &cmddef,
                              CmdFormDef      &form,
                              const StringVec &tokens);

      /*!
       * \brief Parse argument 0 (command) syntax.
       *
       * Argv0 is a special argument that defines the command. Currently, only
       * literal arguments are valid. If expanded to include variables, then
       * tab completion and command matching would be interesting.
       *
       * \par Syntax:
       * \verbatim
       * <argv0> ::= <literal-arg>
       * \endverbatim
       *
       * \param [in,out] cmddef An added command object.
       * \param [in,out] form   Command form definition. 
       * \param [in] tokens     Lexical tokens generated from the syntax usage.
       * \param [in,out] pos    The parse cursor position. \sa parseSyntax().
       *
       * \return Returns true on a successful parse, false otherwise.
       */
      bool parseArgv0(CmdDef          &cmddef,
                      CmdFormDef      &form,
                      const StringVec &tokens,
                      size_t          &pos);
      
      /*!
       * \brief Parse required argument list syntax.
       *
       * \par Syntax:
       * \verbatim
       * <required-arg-list> ::= <arg>
       *                       | <required-arg-list> <arg>
       * \endverbatim
       *
       * \param [in,out] cmddef An added command object.
       * \param [in,out] form   Command form definition. 
       * \param [in] tokens     Lexical tokens generated from the syntax usage.
       * \param [in,out] pos    The parse cursor position. \sa parseSyntax().
       *
       * \return Returns true on a successful parse, false otherwise.
       */
      bool parseRequiredArgList(CmdDef          &cmddef,
                                CmdFormDef      &form,
                                const StringVec &tokens,
                                size_t          &pos);
  
      /*!
       * \brief Parse optional argument list syntax.
       *
       * \par Syntax:
       * \verbatim
       * <optional-arg-list> ::= '[' <arg> ']'
       *                       | <optional-arg-list> '[' <arg> ']'
       * \endverbatim
       *
       * \param [in,out] cmddef An added command object.
       * \param [in,out] form   Command form definition. 
       * \param [in] tokens     Lexical tokens generated from the syntax usage.
       * \param [in,out] pos    The parse cursor position. \sa parseSyntax().
       *
       * \return Returns true on a successful parse, false otherwise.
       */
      bool parseOptionalArgList(CmdDef          &cmddef,
                                CmdFormDef      &form,
                                const StringVec &tokens,
                                size_t          &pos);

      /*!
       * \brief Parse argument syntax.
       *
       * \par Syntax:
       * \verbatim
       * <arg> ::= <xor-list-arg>
       *         | <variable-arg>
       *         | <literal-arg>
       * \endverbatim
       *
       * \param [in,out] cmddef An added command object.
       * \param [in,out] form   Command form definition. 
       * \param [in] tokens     Lexical tokens generated from the syntax usage.
       * \param [in,out] pos    The parse cursor position. \sa parseSyntax().
       *
       * \return Returns true on a successful parse, false otherwise.
       */
      bool parseArg(CmdDef          &cmddef,
                    CmdFormDef      &form,
                    const StringVec &tokens,
                    size_t          &pos);
      
      /*!
       * \brief Parse mutually exclusive argument values syntax.
       *
       * A CmdArgDef object is added to the CmdDef
       *
       * \par Syntax:
       * \verbatim
       * <xor-list-arg> ::= '{' <xor-list> '}'
       * \endverbatim
       *
       * \param [in,out] cmddef An added command object.
       * \param [in,out] form   Command form definition. 
       * \param [in] tokens     Lexical tokens generated from the syntax usage.
       * \param [in,out] pos    The parse cursor position. \sa parseSyntax().
       *
       * \return Returns true on a successful parse, false otherwise.
       */
      bool parseXorListArg(CmdDef          &cmddef,
                           CmdFormDef      &form,
                           const StringVec &tokens,
                           size_t          &pos);
      
      /*!
       * \brief Parse variable argument syntax.
       *
       * A CmdArgDef object is added to the CmdDef
       *
       * \par Syntax:
       * \verbatim
       * <variable-arg> ::= '<' <identity> '>'              (* word default *)
       *                  | '<' <identity> ':' <type> '>'
       * \endverbatim
       *
       * \param [in,out] cmddef An added command object.
       * \param [in,out] form   Command form definition. 
       * \param [in] tokens     Lexical tokens generated from the syntax usage.
       * \param [in,out] pos    The parse cursor position. \sa parseSyntax().
       *
       * \return Returns true on a successful parse, false otherwise.
       */
      bool parseVariableArg(CmdDef          &cmddef,
                            CmdFormDef      &form,
                            const StringVec &tokens,
                            size_t          &pos);

      /*!
       * \brief Parse literal, fixed-valued argument syntax.
       *
       * A CmdArgDef object is added to the CmdDef
       *
       * \par Syntax:
       * \verbatim
       * <literal-arg> ::= <literal>
       * \endverbatim
       *
       * \param [in,out] cmddef An added command object.
       * \param [in,out] form   Command form definition. 
       * \param [in] tokens     Lexical tokens generated from the syntax usage.
       * \param [in,out] pos    The parse cursor position. \sa parseSyntax().
       *
       * \return Returns true on a successful parse, false otherwise.
       */
      bool parseLiteralArg(CmdDef          &cmddef,
                           CmdFormDef      &form,
                           const StringVec &tokens,
                           size_t          &pos);

      /*!
       * \brief Parse mutually exclusive list.
       *
       * \par Syntax:
       * \verbatim
       * <xor-list> ::= <literal>
       *              | <xor-list> '|' <literal>
       * \endverbatim
       *
       * \param [in,out] cmddef An added command object.
       * \param [in,out] form   Command form definition. 
       * \param [in] tokens     Lexical tokens generated from the syntax usage.
       * \param [in,out] pos    The parse cursor position. \sa parseSyntax().
       * \param [out] literals  Vector of literals.
       *
       * \return Returns true on a successful parse, false otherwise.
       */
      bool parseXorList(CmdDef          &cmddef,
                        CmdFormDef      &form,
                        const StringVec &tokens,
                        size_t          &pos,
                        StringVec       &literals);

      /*!
       * \brief Parse identifier.
       *
       * \par Syntax:
       * \verbatim
       * <identifier> ::= ALPHA {ALPHANUMERIC}*
       *                | '_' {ALPHANUMERIC}*
       * \endverbatim
       *
       * \param [in,out] cmddef An added command object.
       * \param [in,out] form   Command form definition. 
       * \param [in] tokens     Lexical tokens generated from the syntax usage.
       * \param [in,out] pos    The parse cursor position. \sa parseSyntax().
       * \param [out] strIdent  Identity string.
       *
       * \return Returns true on a successful parse, false otherwise.
       */
      bool parseIdentifier(CmdDef          &cmddef,
                           CmdFormDef      &form,
                           const StringVec &tokens,
                           size_t          &pos,
                           std::string     &strIdent);
      
      /*!
       * \brief Parse variable type.
       *
       * \par Syntax:
       * \verbatim
       * <type> ::= literal
       *          | word
       *          | int
       *          | float
       *          | quoted-string
       *          | file
       * \endverbatim
       *
       * \param [in,out] cmddef An added command object.
       * \param [in,out] form   Command form definition. 
       * \param [in] tokens     Lexical tokens generated from the syntax usage.
       * \param [in,out] pos    The parse cursor position. \sa parseSyntax().
       * \param [out] eType     Type enum.
       *
       * \return Returns true on a successful parse, false otherwise.
       */
      bool parseType(CmdDef             &cmddef,
                     CmdFormDef         &form,
                     const StringVec    &tokens,
                     size_t             &pos,
                     CmdArgDef::ArgType &eType);

      /*!
       * \brief Parse literal value.
       *
       * \par Syntax:
       * \verbatim
       * <literal> ::= {NON_SPECIAL_CHAR}+
       * \endverbatim
       *
       * \param [in,out] cmddef An added command object.
       * \param [in,out] form   Command form definition. 
       * \param [in] tokens     Lexical tokens generated from the syntax usage.
       * \param [in,out] pos    The parse cursor position. \sa parseSyntax().
       * \param [out] strValue  Value string.
       *
       * \return Returns true on a successful parse, false otherwise.
       */
      bool parseLiteralValue(CmdDef          &cmddef,
                             CmdFormDef      &form,
                             const StringVec &tokens,
                             size_t          &pos,
                             std::string     &strValue);

      /*!
       * \brief Test if token at position is equal to string.
       *
       * If equal, the position cursor is advanced on position. If not equal,
       * an error message will be logged.
       *
       * \param [in] strInput String origin of tokens.
       * \param [in] strCmp   String to compare token against.
       * \param [in] tokens   Lexical tokens generated from the syntax usage.
       * \param [in,out] pos  The position in the token vector.
       *                      Advanced 1 position if equal.
       *
       * \return Returns true on equal, false otherwise.
       */
      bool tokEq(const std::string &strInput,
                 const std::string strCmp,
                 const StringVec   &tokens,
                 size_t            &pos);

      /*!
       * \brief Peek if token at position is equal to string.
       *
       * The position cursor is not advance nor any error message logged.
       *
       * \param [in] strCmp String to compare token against.
       * \param [in] tokens Lexical tokens generated from the syntax usage.
       * \param [in] pos    The position in the token vector.
       *
       * \return Returns true on equal, false otherwise.
       */
      bool peekEq(const std::string strCmp,
                  const StringVec   &tokens,
                  const size_t      pos);
      
      /*!
       * \brief Test if string has valid identifier syntax.
       *
       * \param [in] str    String to test.
       *
       * \return Returns true or false.
       */
      bool tokIdentifier(const std::string &str);
  

      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Command Line Interface Support Methods
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

      /*!
       * \brief Process a line of input.
       *
       * The line is tokenized and matched to the command with the best fit.
       *
       * \param [in] strLine  Line of input.
       * \param [out] uid     Best matched command unique id.
       * \param [out] argv    Vector of arguments.
       *
       *  \copydoc doc_return_cl
       */
      int processInput(const std::string &strLine, int &uid, StringVec &argv);

      /*!
       * \brief Match best command against input line argument list.
       *
       * \param [out] uid     Best matched command unique id.
       * \param [in] argv     Vector of arguments.
       *
       *  \copydoc doc_return_cl
       */
      int matchCommand(int &uid, const StringVec &argv);

      /*!
       * \brief Match best command form against input line argument list.
       *
       * \param [in] cmddef   Compiled command definition.
       * \param [in] argv     Vector of arguments.
       *
       *  \copydoc doc_return_cl
       */
      int matchCommand(const CmdDef &cmddef, const StringVec &argv);

      /*!
       * \brief Match command form against input line argument list.
       *
       * \param [in] cmddef     Compiled command definition.
       * \param [in] form       Compiled command form definition.
       * \param [in] argv       Vector of arguments.
       * \param [out] fFitness  Fitness of match [0.0 - 1.0].
       *
       *  \copydoc doc_return_cl
       */
      int matchCommandForm(const CmdDef     &cmddef,
                           const CmdFormDef &form,
                           const StringVec  &argv,
                           double           &fFitness);

      /*!
       * \brief Check read input result.
       *
       *  \copydoc doc_return_cl
       */
      int checkReadResult();
  

      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // ReadLine Generator Methods
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

      /*!
       * \brief Perform any necessary pre-processing to prepare for command line
       * TAB completion.
       *
       * \note Nothing is needed for the CommandLine base class.
       *
       *  \copydoc doc_return_cl
       */
      virtual int rlBuildReadLineGenerator();

      /*!
       *
       * See \ref ReadLine::AltAppGenFunc for description of a ReadLine
       * generator.
       *
       * \param [out] uFlags  TAB completion modifier flags.
       */
      static const std::string rlGeneratorWrapper(const std::string &strText,
                                                  int               nIndex,
                                                  const std::string &strContext,
                                                  int               nStart,
                                                  int               nEnd,
                                                  unsigned          &uFlags,
                                                  void              *pAppArg);

      virtual const std::string rlGenerator(const std::string &strText,
                                            int               nIndex,
                                            const std::string &strContext,
                                            int               nStart,
                                            int               nEnd,
                                            unsigned          &uFlags);

      void rlArgDefs(const std::string       &strSubtext,
                     std::vector<CmdArgDef*> &argdefs);

      void rlTabList(const std:: string      &strText,
                     std::vector<CmdArgDef*> &argdefs,
                     StringVec               &tabList,
                     unsigned                &uFlags);


      bool rlEq(const std::string &strCandidate,
                        const std::string &strText);

    }; // class CommandLine
  
  } // namespace cmd
} // namespace rnr

#endif // _COMMAND_LINE_H
