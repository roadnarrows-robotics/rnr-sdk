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
 * \brief Command line interface class interface.
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

#ifndef _RNR_COMMAND_LINE_H
#define _RNR_COMMAND_LINE_H

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

#include <iostream>
#include <string>
#include <vector>
#include <map>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/appkit/StringTheory.h"
#include "rnr/appkit/RegEx.h"
#include "rnr/appkit/LogBook.h"
#include "rnr/appkit/ReadLine.h"
#include "rnr/appkit/Token.h"
#include "rnr/appkit/CmdCore.h"
#include "rnr/appkit/CmdExtArg.h"
#include "rnr/appkit/CmdArgDef.h"
#include "rnr/appkit/CmdFormDef.h"
#include "rnr/appkit/CmdDef.h"


/*!
 * \brief RoadNarrows Robotics
 */
namespace rnr
{
  /*!
   * \brief Commands
   */
  namespace cmd
  {
    //
    // Forward declarations
    //
    class CommandLine;

    //--------------------------------------------------------------------------
    // Types and Values
    //--------------------------------------------------------------------------

    /*!
     * \brief Command execution function type, variant 1.
     *
     * With this variant, a vector of command line strings argument are
     * provided to the function.
     *
     * The arguments have been validated against the associated extended syntax.
     *
     * This function type is provided as a convenience for application
     * development. It is external to the core CommandLine functionality.
     *
     * \param [in] argv Vector of string arguments produced from a successful
     *                  return from the relevant readCommand() call. The argv[0]
     *                  argument is the command name.
     *
     * \return User defined return code.
     */
    typedef int (*CmdExec1Func)(const str::StringVec &argv);
  
    /*!
     * \brief Command execution function type, variant 2.
     *
     * With this variant, a vector of command line extended arguments are
     * provided to the function. Each extended argument provide matched command
     * context. Moreover, each argument has been converted to its basic type.
     *
     * The arguments have been validated against the associated extended syntax.
     *
     * This function type is provided as a convenience for application
     * development. It is external to the core CommandLine functionality.
     *
     * \param [in] argv Vector of extended arguments produced from a successful
     *                  return from the relevant readCommand() call. The argv[0]
     *                  argument is the command name.
     *
     * \return User defined return code.
     */
    typedef int (*CmdExec2Func)(const CmdExtArgVec &argv);

    /*!
     * \brief Command execution function type, variant 3.
     *
     * This is identical to variant 2, but with the additional parameter 
     * referencing the common-line interface specification.
     *
     * \param [in] cli  Command-line interface.
     * \param [in] argv Vector of extended arguments produced from a successful
     *                  return from the relevant readCommand() call. The argv[0]
     *                  argument is the command name.
     *
     * \return User defined return code.
     */
    typedef int (*CmdExec3Func)(CommandLine &cli, const CmdExtArgVec &argv);


    //--------------------------------------------------------------------------
    // CmdExec Classs
    //--------------------------------------------------------------------------

    /*!
     * \brief Helper class to hold a command line execution function.
     */
    struct CmdExec
    {
    public:
      /*!
       * \brief Default constructor.
       */
      CmdExec()
      {
        m_uid     = NoUid;
        m_variant = VariantUndef;
        m_exec.p  = NULL;
      }

      /*!
       * \brief Variant 1 constructor.
       *
       * \param uid Associated command's unique id.
       * \param fn  Variant 1 execution function.
       */
      CmdExec(int uid, CmdExec1Func fn)
      {
        m_uid       = uid;
        m_variant   = Variant1;
        m_exec.fn1  = fn;
      }

      /*!
       * \brief Variant 2 constructor.
       *
       * \param uid Associated command's unique id.
       * \param fn  Variant 2 execution function.
       */
      CmdExec(int uid, CmdExec2Func fn)
      {
        m_uid       = uid;
        m_variant   = Variant2;
        m_exec.fn2  = fn;
      }

      /*!
       * \brief Variant 3 constructor.
       *
       * \param uid Associated command's unique id.
       * \param fn  Variant 3 execution function.
       */
      CmdExec(int uid, CmdExec3Func fn)
      {
        m_uid       = uid;
        m_variant   = Variant3;
        m_exec.fn3  = fn;
      }

      /*!
       * \brief Copy constructor.
       *
       * \param src   Source object.
       */
      CmdExec(const CmdExec &src)
      {
        m_uid     = src.m_uid;
        m_variant = src.m_variant;
        m_exec    = src.m_exec;
      }

      /*!
       * \brief Execute a comamnd with the given arguments.
       *
       * \param argv  The list of string arguments.
       *
       * \return
       * Returns a \ref cmd_ecodes on failure to find the associated command or
       * its execution function. Otherwise, returns the user-define execution
       * function return value.
       */
      int execute(const str::StringVec &argv);

      /*!
       * \brief Execute a comamnd with the given arguments.
       *
       * \param cli   Reference to the command-line interface.
       * \param argv  The list of extended arguments.
       *
       * \return
       * Returns a \ref cmd_ecodes on failure to find the associated command or
       * its execution function. Otherwise, returns the user-define execution
       * function return value.
       */
      int execute(CommandLine &cli, const CmdExtArgVec &argv);

      /*!
       * \brief Get associated command's unique id.
       *
       * \return Unique id.
       */
      int getUid() const
      {
        return m_uid;
      }

      /*!
       * \brief Insert object into output stream.
       *
       * \param os  Output stream.
       * \param obj Object to insert.
       *
       * \return Reference to output stream.
       */
      friend std::ostream &operator<<(std::ostream &os, const CmdExec &obj);

    protected:
      /*!
       * \brief Execution function variant enumeration.
       */
      enum Variant
      {
        VariantUndef,   ///< undefined variant
        Variant1,       ///< variant 1
        Variant2,       ///< variant 2
        Variant3        ///< variant 3
      };

      /*!
       * \brief Union of all variant execution functions.
       */
      union FnVar
      {
        void *        p;    ///< pointer
        CmdExec1Func  fn1;  ///< execution function variant 1
        CmdExec2Func  fn2;  ///< execution function variant 2
        CmdExec3Func  fn3;  ///< execution function variant 3
      };

      int     m_uid;      ///< command unique id associated with this execution
      Variant m_variant;  ///< function variant enum
      FnVar   m_exec;     ///< function execution variant
    };


    //--------------------------------------------------------------------------
    // DataSect Classs
    //--------------------------------------------------------------------------

    /*!
     * \brief Command line interface data section class.
     *
     * The cli can support multiple data sections, each delineated by a unique
     * namespace.
     *
     * If a data deallocator function is specifed, the section data will be
     * automatically deleted when the command line data section is is deleted.
     */
    class DataSect
    {
    public:
      /*! data section data deallocator function type. */
      typedef void (*DeallocFunc)(void *);

      std::string m_strNs;      ///< data section namespace
      void *      m_pData;      ///< pointer to section data
      DeallocFunc m_fnDealloc;  ///< data deallocator function

      /*!
       * \brief Default constructor.
       */
      DataSect();

      /*!
       * \brief Initialization constructor.
       *
       * \param ns      ///< data section namespace
       * \param pData   ///< pointer to section data
       * \param fn      ///< data deallocator function
       */
      DataSect(const std::string &ns, void *pData, DeallocFunc fn = NULL);

      /*!
       * \brief Destructor.
       */
      ~DataSect();

      void set(const std::string &ns, void *pData, DeallocFunc fn = NULL);

      /*!
       * \brief Return namespace.
       *
       * \return String.
       */
      const std::string &ns() const
      {
        return m_strNs;
      }

      /*!
       * \brief Return section data.
       *
       * \return Void pointer.
       */
      void *data()
      {
        return m_pData;
      }

      /*!
       * \brief Return section data.
       *
       * \return Constant void pointer.
       */
      const void *data() const
      {
        return m_pData;
      }

      /*!
       * \brief Return deallocator.
       *
       * \return Pointer to function.
       */
      DeallocFunc deallocator()
      {
        return m_fnDealloc;
      }

      /*!
       * \brief Return deallocator.
       *
       * \return Constant pointer to function.
       */
      const DeallocFunc deallocator() const
      {
        return m_fnDealloc;
      }

      /*!
       * \brief Insert object into output stream.
       *
       * \param os  Output stream.
       * \param obj Object to insert.
       *
       * \return Reference to output stream.
       */
      friend std::ostream &operator<<(std::ostream &os, const DataSect &obj);

    }; // class DataSect


    //--------------------------------------------------------------------------
    // Types and Data
    //--------------------------------------------------------------------------

    //
    // Command definitions map.
    //
    typedef std::map<unsigned, CmdDef>  CmdDefMap;    ///< command map type
    typedef CmdDefMap::iterator         CmdDefIter;   ///< cmd iterator type
    typedef CmdDefMap::const_iterator   CmdDefCIter;  ///< cmd const iter type

    //
    // Command execution map.
    //
    typedef std::map<unsigned, CmdExec> CmdExecMap;   ///< exec map type
    typedef CmdExecMap::iterator        CmdExecIter;  ///< exec iterator type
    typedef CmdExecMap::const_iterator  CmdExecCIter; ///< exec const iter type

    //
    // Data Section map.
    //
    typedef std::map<std::string, DataSect> DataSectMap;   ///< section map type
    typedef DataSectMap::iterator           DataSectIter;  ///< iterator type
    typedef DataSectMap::const_iterator     DataSectCIter; ///< const iter type

    typedef str::StringVec  PromptStack;  ///< prompt stack type


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
  
      /*!
       * \brief Test if command-line interface is ok to continue.
       *
       * \return Returns true or false.
       */
      bool ok() const
      {
        return !((DataSectCore*)m_dataSects.at(DataSectNsCore).data())->m_bQuit;
      }

      /*!
       * \brief Mark command-line interface to quit.
       */
      void quit()
      {
        ((DataSectCore *)m_dataSects[DataSectNsCore].data())->m_bQuit = true;
      }

      /*!
       * \brief Test if backtracing is enabled.
       *
       * \return Returns true or false.
       */
      bool getBtEnable() const
      {
        return
          ((DataSectCore*)m_dataSects.at(DataSectNsCore).data())->m_bBacktrace;
      }

      /*!
       * \brief Enable/disable backtracing.
       *
       * \param bEnable Enable(true) or Disable(false).
       */
      void setBtEnable(bool bEnable)
      {
        ((DataSectCore*)m_dataSects.at(DataSectNsCore).data())->m_bBacktrace =
          bEnable;
      }


      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Public Command Addition and Compile Methods
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

      /*!
       * \defgroup doc_addcommand
       * \{
       * \brief Add a command to the command line interface.
       *
       * The command syntax may specify multiple command forms, each separated
       * by a newline '\\n' character. The command named in each form must be
       * the identical.
       *
       * This is a light-weight function. A call to compile() is necessary to
       * finalized command syntax processing.
       *
       * \param desc        Command description (see \ref CmdDesc).
       * \param strSyntax   One or more extended usage syntax separated by
       *                    newline characters.
       * \param fnExec      Command execution function of variant 1, 2, or 3.
       *
       * \return
       * On success, returns command's assigned unique id.
       * Otherwise NoUid is returned.
       * \} 
       */
      /*! \copydoc doc_addcommand */
      virtual int addCommand(const CmdDesc &desc);

      /*! \copydoc doc_addcommand */
      virtual int addCommand(const CmdDesc &desc, CmdExec1Func fnExec);

      /*! \copydoc doc_addcommand */
      virtual int addCommand(const CmdDesc &desc, CmdExec2Func fnExec);

      /*! \copydoc doc_addcommand */
      virtual int addCommand(const CmdDesc &desc, CmdExec3Func fnExec);

      /*! \copydoc doc_addcommand */
      virtual int addCommand(const std::string strSyntax);

      /*! \copydoc doc_addcommand */
      virtual int addCommand(const std::string strSyntax, CmdExec1Func fnExec);

      /*! \copydoc doc_addcommand */
      virtual int addCommand(const std::string strSyntax, CmdExec2Func fnExec);

      /*! \copydoc doc_addcommand */
      virtual int addCommand(const std::string strSyntax, CmdExec3Func fnExec);
  
      /*!
       * \brief Remove command from command line interface.
       *
       * No re-comiple is necessary.
       *
       * \param uid   Command unique id.
       *
       *  \copydoc doc_return_cl
       */
      virtual int removeCommand(const int uid);
  
      /*!
       * \brief Remove command from command line interface.
       *
       * No re-comiple is necessary.
       *
       * \param strName Command name.
       *
       *  \copydoc doc_return_cl
       */
      virtual int removeCommand(const std::string &strName);
  
      /*!
       * \brief Remove all commands from command line interface.
       *
       *  \copydoc doc_return_cl
       */
      virtual int removeAllCommands();
  
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
  

      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Public Command Line Data Section Methods
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

      /*!
       * \brief Add a data section to the command-line interface.
       * 
       * When the section is removed or the interface is deleted, the section 
       * data is automatically deleted if a deallocation function is specified.
       * Otherwise, the user has responsibility to delete any data.
       *
       * \param ns    Namespace of data section.
       * \param pData Pointer to data.
       * \param fn    Deallocator function.
       *
       *  \copydoc doc_return_cl
       */
      int addDataSection(const std::string     &ns,
                         void                  *pData,
                         DataSect::DeallocFunc fn = NULL);

      /*!
       * \brief Remove a data section from the command-line interface.
       *
       * If a deallocation function was specified, the section data is also
       * deleted. Otherwise, the data remains intact.
       *
       * \param ns    Namespace of data section to remove.
       *
       *  \copydoc doc_return_cl
       */
      int removeDataSection(const std::string &ns);

      /*!
       * \brief Get the section data under the specified namespace.
       *
       * \param ns    Namespace of data section to retrieve.
       *
       * \return Returns pointer to the data if it exist. NULL is returned if
       * no namespace is found, or no data has been specified.
       */
      void *getDataSection(const std::string &ns);

      /*!
       * \brief Test if the namespace is a command-line interface reserved name.
       *
       * \param ns    Namespace.
       *
       * \return Returns true or false.
       */
      bool isReservedDataSection(const std::string &ns) const;


      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Public Command Line Interface Methods
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  
      /*!
       * \brief Read an input line from stdin and match to the best compiled
       * command.
       *
       * Simple string argument version.
       *
       * On success, no arguments indicate an empty input line. Otherwise,
       * the command name argv[0] is guaranteed to be present in the vector. 
       *
       * \param [out] uid   Matched command unique id.
       * \param [out] iform Matched command form index.
       * \param [out] argv  Vector of arguments, with argv[0] being the
       *                    command name.
       *
       *  \copydoc doc_return_cl
       */
      virtual int readCommand(int &uid, int &iform, str::StringVec &argv)
      {
        return readCommand(stdin, uid, iform, argv);
      }

      /*!
       * \brief Read an input line from stdin and match to the best compiled
       * command.
       *
       * Extended argument version.
       *
       * Each extended argument contains matched command context of:
       *  - the command's unique id and definition
       *  - form definition index
       *  - argument definition index
       *  - argument instance number for (future) argument repetition
       *
       * On success, no arguments indicate an empty input line. Otherwise,
       * the command name argv[0] is guaranteed to be present in the vector. 
       *
       * Extended arguments gives access to the compiled, matched command
       * definition data such as number of required, optional, and total
       * arguments, plus argument specific info such as name, type, etc. 
       *
       * \param [out] uid   Matched command unique id.
       * \param [out] argv  Vector of extended arguments, with argv[0] being the
       *                    command name argument.
       *
       *  \copydoc doc_return_cl
       */
      virtual int readCommand(CmdExtArgVec &argv)
      {
        return readCommand(stdin, argv);
      }

      /*!
       * \brief Read an input line from file fp and match to the best compiled
       * command.
       *
       * Simple string argument version.
       *
       * On success, no arguments indicate an empty input line. Otherwise,
       * the command name argv[0] is guaranteed to be present in the vector. 
       *
       * \param       fp    Input file pointer.
       * \param [out] uid   Matched command unique id.
       * \param [out] iform Matched command form index.
       * \param [out] argv  Vector of arguments, with argv[0] being the
       *                    command name.
       *
       *  \copydoc doc_return_cl
       */
      virtual int readCommand(FILE           *fp,
                              int            &uid,
                              int            &iform,
                              str::StringVec &argv);
  
      /*!
       * \brief Read an input line from file fp and match to the best compiled
       * command.
       *
       * Extended argument version.
       *
       * Each extended argument contains matched command context of:
       *  - the command's unique id and definition
       *  - form definition index
       *  - argument definition index
       *  - argument instance number for (future) argument repetition
       *
       * On success, no arguments indicate an empty input line. Otherwise,
       * the command name argv[0] is guaranteed to be present in the vector. 
       *
       * Extended arguments gives access to the compiled, matched command
       * definition data such as number of required, optional, and total
       * arguments, plus argument specific info such as name, type, etc. 
       *
       * \param fp          Input file pointer.
       * \param [out] argv  Vector of extended arguments, with argv[0] being the
       *                    command name argument.
       *
       *  \copydoc doc_return_cl
       */
      virtual int readCommand(FILE *fp, CmdExtArgVec &argv);

      bool kbhit()
      {
        return kbhit(stdin);
      }

      bool kbhit(FILE *fp);

      /*!
       * \brief Execute a comamnd with the given arguments.
       *
       * \param argv  The list of string arguments.
       *
       * \return
       * Returns a \ref cmd_ecodes on failure to find the associated command or
       * its execution function. Otherwise, returns the user-define execution
       * function return value.
       */
      virtual int execute(const str::StringVec &argv);

      /*!
       * \brief Execute a comamnd with the given arguments.
       *
       * \param argv  The list of extended arguments.
       *
       * \return
       * Returns a \ref cmd_ecodes on failure to find the associated command or
       * its execution function. Otherwise, returns the user-define execution
       * function return value.
       */
      virtual int execute(const CmdExtArgVec &argv);

      /*!
       * \brief Add command to history.
       *
       * Simple string argument version.
       *
       * \note Only available if readline is available and enabled, and
       * input is interactive.
       *
       * \param argv  Command in argv vector.
       */
      virtual void addToHistory(const str::StringVec &argv);
  
      /*!
       * \brief Add command to history.
       *
       * Extended argument version.
       *
       * \note Only available if readline is available and enabled, and
       * input is interactive.
       *
       * \param argv  Command in argv vector.
       */
      virtual void addToHistory(const CmdExtArgVec &argv);
  
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

      /*!
       * \brief Get the line number of the last read line.
       *
       * \return Line number.
       */
      size_t getLineNum() const
      {
        return m_readline.getLineNum();
      }

      /*!
       * \brief Set the current line number.
       *
       * \param Line number.
       */
      void setLineNum(const size_t uLineNum)
      {
        m_readline.setLineNum(uLineNum);
      }

      /*!
       * \brief Reset the line number to zero.
       */
      void resetLineNum()
      {
        m_readline.resetLineNum();
      }
  

      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Public Command Form Info and Argument Access Methods
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

      /*!
       * \brief Get the total number of arguments.
       *
       * The first argument (argv0) is the command.
       *
       * \param uid   Matched command unique id.
       * \param iform Matched command form index.
       *
       * \return Number of arguments.
       */
      int numOfArgs(int uid, int iform) const;

      /*!
       * \brief Get the total number of arguments of matched command.
       *
       * The extended argument contains matched command context.
       *
       * \param arg   Extended argument.
       *
       * \return Number of arguments.
       */
      int numOfArgs(const CmdExtArg &arg) const;

      /*!
       * \brief Get the total number of required arguments.
       *
       * The required number includes the command argv0.
       *
       * Required arguments start at argument index 0.
       *
       * \param uid   Matched command unique id.
       * \param iform Matched command form index.
       *
       * \return Number of required arguments.
       */
      int numOfRequiredArgs(int uid, int iform) const;

      /*!
       * \brief Get the total number of required arguments of matched command.
       *
       * The extended argument contains matched command context.
       *
       * The required number includes the command argv0.
       *
       * Required arguments start at argument index 0.
       *
       * \param arg   Extended argument.
       *
       * \return Number of required arguments.
       */
      int numOfRequiredArgs(const CmdExtArg &arg) const;

      /*!
       * \brief Get the total number of optional arguments.
       *
       * Optional arguments start at argument index numOfRequiredArgs().
       *
       * \param uid   Matched command unique id.
       * \param iform Matched command form index.
       *
       * \return Number of optional arguments.
       */
      int numOfOptionalArgs(int uid, int iform) const;

      /*!
       * \brief Get the total number of arguments of matched command.
       *
       * The extended argument contains matched command context.
       *
       * \param arg   Extended argument.
       *
       * \return Number of optional arguments.
       */
      int numOfOptionalArgs(const CmdExtArg &arg) const;

      /*!
       * \brief Get the argument name.
       *
       * The extended argument contains matched command context.
       *
       * \param arg   Extended argument.
       *
       * \return Returns argument name on success, empty string on failure.
       */
      const std::string &getArgName(const CmdExtArg &arg) const;

      /*!
       * \brief Get the argument type.
       *
       * The extended argument contains matched command context.
       *
       * \note Argument type differs from argument converted type. The former
       * specifies the argument syntax, while the later specifies converted
       * basic type such as string, int, etc.
       *
       * \param arg   Extended argument.
       *
       * \return Returns argument type.
       */
      CmdArgDef::ArgType getArgDefType(const CmdExtArg &arg) const;


      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Public Input Processing Methods
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

      /*!
       * \brief Lexically analyze input string to generate a series of tokens.
       *
       * Tokens are separated by whitespace.
       *
       * Each tokens is either:
       * * A contiguous sequence of non-whitespace characters.
       * * An escapable sequence of characters delineated by double quotes '"'.
       *
       * ~~~~~~~~
       * <token-list>    ::= <token>
       *                   | <token-list> <token>
       *
       * <token>         ::= <word>
       *                   | <quoted-string>
       *
       * <word>          ::= {NONWHITE_CHAR}+
       *
       * <qouted-string> ::= '"' {ESCAPABLE_CHAR}* '"'
       * ~~~~~~~~
       *
       * \param [in] strInput Input string to analyze.
       * \param [out] tokens  Generated tokens.
       *
       *  \copydoc doc_return_ssize
       */
      virtual ssize_t tokenizeInput(const std::string &strInput,
                                    TokenVec          &tokens);

      /*!
       * \brief Lexically analyze input string to generate a series of string
       * tokens.
       *
       * See \ref tokenize()
       *
       * \param [in] strInput Input string to analyze.
       * \param [out] tokens  Generated string tokens.
       *
       *  \copydoc doc_return_ssize
       */
      virtual ssize_t tokenizeInput(const std::string &strInput,
                                    str::StringVec    &tokens);
  

      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Public Attribute and Data Access Methods
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

      /*!
       * \brief Get command line interface's name.
       *
       * \return Name string.
       */
      const std::string &getName() const
      {
        return m_strName;
      }

      /*!
       * \brief Test if command exists.
       *
       * \param uid   Command unique id.
       *
       * \return Returns true or false.
       */
      bool hasCmd(const int uid) const;

      /*!
       * \brief Test if command exists.
       *
       * \param strName Command name.
       *
       * \return Returns true or false.
       */
      bool hasCmd(const std::string &strName) const;

      /*!
       * \brief Get the command definition with the unique id.
       *
       * \param uid   Command unique id.
       *
       * \return Command definition reference.
       */
      const CmdDef &at(const int uid) const;

      /*!
       * \brief Get command definition with the argv0 name.
       *
       * \param strName Command name.
       *
       * \return Command definition reference.
       */
      const CmdDef &at(const std::string &strName) const;

      /*!
       * \brief Index operator.
       *
       * Get the command definition with the unique id.
       *
       * \param uid   Command unique id.
       *
       * \return Command definition reference.
       */
      const CmdDef &operator[](const int uid) const;

      /*!
       * \brief Index operator.
       *
       * Get the command definition with the unique id.
       *
       * \param strName Command name.
       *
       * \return Command definition reference.
       */
      const CmdDef &operator[](const std::string &strName) const;

      /*!
       * \brief Get the total number of added commands.
       *
       * \return Number of commands.
       */
      int numOfCmds() const
      {
        return (int)m_cmdDefs.size();
      }

      /*!
       * \brief Return a constant iterator pointing to the first element of
       * the list of command definitions.
       *
       * \return Constant Iterator.
       */
      CmdDefCIter begin() const
      {
        return m_cmdDefs.begin();
      }

      /*!
       * \brief Return a constant iterator referring to the past-the-end
       * element of the list of command definitions.
       *
       * \return Constant Iterator.
       */
      CmdDefCIter end() const
      {
        return m_cmdDefs.end();
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
      std::ostream &backtrace(std::ostream &os, const bool bAll = false) const;


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
       * \brief Canonicalization of a string.
       *
       * \note The name c14n is an cute abbreviation where 14 represents the
       * number of letters between the 'c' and 'n' in the word
       * "canonicalization".
       *
       * \param tokens  Tokens to canonicalize.
       *
       * \return Return string holding canonical form.
       */
      static std::string c14n(const TokenVec &tokens);

    protected:
      std::string m_strName;      ///< name of this command line
      bool        m_bIgnoreCase;  ///< do [not] ignore case on commands
      int         m_nUidCnt;      ///< unique id counter
      bool        m_bIsCompiled;  ///< has [not] been successfully compiled
      CmdDefMap   m_cmdDefs;      ///< map of added command definitions
      CmdExecMap  m_cmdExecs;     ///< map of added command executions
      PromptStack m_prompts;      ///< stack of prompt strings
      ReadLine    m_readline;     ///< readline interface
      LogBook     m_log;          ///< trace and error log
      DataSectMap m_dataSects;    ///< data sections

      /*!
       * \brief Get modifiable command definition with the given unique id.
       *
       * Protected version of at().
       *
       * \param uid   Command unique id.
       *
       * \return Command definition reference.
       */
      CmdDef &cmdAt(const int uid);

      /*!
       * \brief Get modifiable command definition with the given argv0 name.
       *
       * Protected version of at().
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
       * \brief Lexically analyze extened usage syntax string to generate a
       * series of tokens.
       *
       * Tokens are separated by either whitespace, parenthetical blocks,
       * or syntax special characters.
       *
       * \param [in]  strSyntax Syntax string to analyze.
       * \param [out] tokens    Generated tokens.
       *
       *  \copydoc doc_return_ssize
       */
      virtual ssize_t tokenizeSyntax(const std::string &strSyntax,
                                     TokenVec          &tokens);

      /*!
       * \brief Syntax word lexical analyzer.
       *
       * A syntax word is defined as any contiguous sequence of non-whitespace,
       * non-syntax-special symbol characters.
       *
       * On success, the generated <word> token is placed at the end of the
       * vector.
       *
       * \param [in]     strSyntax  Syntax string to analyze.
       * \param [in]     cursor     Character position along syntax string.
       * \param [in,out] tokens     Vector of generated tokens. 
       *
       * \return
       * On success, returns new cursor position.
       * Otherwise a negative error code is returned.
       */
      virtual ssize_t lexSyntaxWord(const std::string &strSyntax,
                                    ssize_t            cursor,
                                    TokenVec          &tokens);

      /*!
       * \brief Syntax parenthetical expression lexical analyzer.
       *
       * A syntax parenthetical expression isdefined as any contiguous sequence
       * block of character delineated by a balanced pair of parentheses. A
       * backslash escaped parenthesis does to factor into the paentheses
       * countiog.
       *
       * On success,the generated tokens '(', <paren-tok>, and ')' are placed
       * at the end of the vector.
       *
       * \par Syntax:
       * ~~~~~~~~
       * <paren-expr-def> ::= '(' <paren-tok> ')'
       * ~~~~~~~~
       *
       * \param [in]     strSyntax  Syntax string to analyze.
       * \param [in]     cursor     Character position along syntax string.
       * \param [in,out] tokens     Vector of generated tokens. 
       *
       * \return
       * On success, returns new cursor position.
       * Otherwise a negative error code is returned.
       */
      virtual ssize_t lexSyntaxParenExpr(const std::string &strSyntax,
                                         ssize_t            cursor,
                                         TokenVec          &tokens);

      /*!
       * \brief Word lexical analyzer.
       *
       * A word is defined as any contiguous sequence of non-whitespace
       * characters.
       *
       * On success, the generated <word> token is placed at the end of the
       * vector.
       *
       * \param [in]     strInput   Input string to analyze.
       * \param [in]     cursor     Character position along input string.
       * \param [in,out] tokens     Vector of generated tokens. 
       *
       * \return
       * On success, returns new cursor position.
       * Otherwise a negative error code is returned.
       */
      virtual ssize_t lexWord(const std::string &strInput,
                              ssize_t            cursor,
                              TokenVec          &tokens);

      /*!
       * \brief Quoted string lexical analyzer.
       *
       * The unescaped double quotes are not included in the token. Escape
       * sequences interpreted.
       *
       * On success, the generated <interp-char-seq> token is placed at the end
       * of the vector.
       *
       * \par Syntax:
       * ~~~~~~~~
       * <quoted-string> ::= '"' <interp-char-seq> '"'
       *
       * <interp-char-seq> ::= <interp-char>
       *                     | <interp-char-seq> <interp-char>
       *
       * <interp-char> ::= NON_BACKSLASH_CHAR
       *                |  <interp-esc-char>
       *
       * <interp-esc-char> ::= '\' CHAR
       *                     | '\' 'x' HEXDIGIT
       *                     | '\' 'x' HEXDIGIT HEXDIGIT
       * ~~~~~~~~
       *
       * \param [in]     strInput   Input string to analyze.
       * \param [in]     cursor     Character position along input string.
       * \param [in,out] tokens     Vector of generated tokens. 
       *
       * \return
       * On success, returns new cursor position.
       * Otherwise a negative error code is returned.
       */
      virtual ssize_t lexQuotedString(const std::string &strInput,
                                      ssize_t            cursor,
                                      TokenVec          &tokens);

      /*!
       * \brief Log lexigraphical token.
       *
       * \param strSource       Source string of tokens.
       * \param start           Start token character position in source.
       * \param cursor          Cursor position in source.
       * \param [in,out] tokens Vector of lexical tokens.
       * \param bLoc            If true, then include line number and token
       *                        characater positions.
       */
      virtual void logLexToken(const std::string &strSource,
                               const size_t      start,
                               const ssize_t     cursor,
                               TokenVec          &tokens,
                               const bool        bLoc = false);

      /*!
       * \brief Push token to the end of the generated tokens.
       *
       * \param strSource       Source string of tokens.
       * \param start           Start token character position in source.
       * \param cursor          Cursor position in source.
       * \param [in,out] tokens Vector of lexical tokens.
       */
      void pushToken(const std::string &strSource,
                     const size_t      start,
                     const ssize_t     cursor,
                     TokenVec          &tokens);
  

      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Command Addition and Compile Methods
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

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

      /*!
       * \brief Finalize command compilation.
       *
       *  \copydoc doc_return_cl
       */
      int finalize();
  

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
       *
       *  Parameter | Description
       *  --------- | -----------
       *  cmddef    | Definition of an added command. It may be modified.
       *  form      | Definition of a command form. It may be modified.
       *  tokens    | Lexical tokens generated from the extended usage syntax.
       *  pos       | The parse cursor position. On input, it specifies
       *  ^         |  the starting position in the tokens vector. On a
       *  ^         | succesful parse, the cursor position is advanced
       *  ^         | to the first token after the relevant syntax block.
       *
       *
       * \par Syntax:
       * ~~~~~~~~
       * <command> ::= <argv0> [<required-arg-list>] [<optional-arg-list>]
       * ~~~~~~~~
       *
       * \param [in,out] cmddef Command definition.
       * \param [in,out] form   Command form definition. 
       * \param [in]     tokens Lexical tokens generated from the syntax usage.
       *
       *  \copydoc doc_return_cl
       */
      virtual int parseSyntax(CmdDef         &cmddef,
                              CmdFormDef     &form,
                              const TokenVec &tokens);

      /*!
       * \brief Parse argument 0 (command name) syntax.
       *
       * Argv0 is a special argument that defines the command name. Only
       * literal and variable arguments types are valid.
       *
       * \par Syntax:
       * ~~~~~~~~
       * <argv0> ::= <literal-arg>
       *           | <variable-arg>
       * ~~~~~~~~
       *
       * \param [in,out] cmddef Command definition.
       * \param [in,out] form   Command form definition. 
       * \param [in]     tokens Lexical tokens generated from the syntax usage.
       * \param [in,out] pos    The parse cursor position. \sa parseSyntax().
       *
       * \return Returns true on a successful parse, false otherwise.
       */
      bool parseArgv0(CmdDef         &cmddef,
                      CmdFormDef     &form,
                      const TokenVec &tokens,
                      size_t         &pos);
      
      /*!
       * \brief Parse required argument list syntax.
       *
       * \par Syntax:
       * ~~~~~~~~
       * <required-arg-list> ::= <arg>
       *                       | <required-arg-list> <arg>
       * ~~~~~~~~
       *
       * \param [in,out] cmddef Command definition.
       * \param [in,out] form   Command form definition. 
       * \param [in]     tokens Lexical tokens generated from the syntax usage.
       * \param [in,out] pos    The parse cursor position. \sa parseSyntax().
       *
       * \return Returns true on a successful parse, false otherwise.
       */
      bool parseRequiredArgList(CmdDef         &cmddef,
                                CmdFormDef     &form,
                                const TokenVec &tokens,
                                size_t         &pos);
  
      /*!
       * \brief Parse optional argument list syntax.
       *
       * \par Syntax:
       * ~~~~~~~~
       * <optional-arg-list> ::= '[' <arg> ']'
       *                       | <optional-arg-list> '[' <arg> ']'
       * ~~~~~~~~
       *
       * \param [in,out] cmddef Command definition.
       * \param [in,out] form   Command form definition. 
       * \param [in]     tokens Lexical tokens generated from the syntax usage.
       * \param [in,out] pos    The parse cursor position. \sa parseSyntax().
       *
       * \return Returns true on a successful parse, false otherwise.
       */
      bool parseOptionalArgList(CmdDef         &cmddef,
                                CmdFormDef     &form,
                                const TokenVec &tokens,
                                size_t         &pos);

      /*!
       * \brief Parse argument syntax.
       *
       * \par Syntax:
       * ~~~~~~~~
       * <arg> ::= <xor-list-arg>
       *         | <variable-arg>
       *         | <literal-arg>
       * ~~~~~~~~
       *
       * \param [in,out] cmddef Command definition.
       * \param [in,out] form   Command form definition. 
       * \param [in]     tokens Lexical tokens generated from the syntax usage.
       * \param [in,out] pos    The parse cursor position. \sa parseSyntax().
       *
       * \return Returns true on a successful parse, false otherwise.
       */
      bool parseArg(CmdDef         &cmddef,
                    CmdFormDef     &form,
                    const TokenVec &tokens,
                    size_t         &pos);
      
      /*!
       * \brief Parse mutually exclusive argument values syntax.
       *
       * A CmdArgDef object is added to the CmdDef
       *
       * \par Syntax:
       * ~~~~~~~~
       * <xor-list-arg> ::= '{' <xor-list> '}'
       * ~~~~~~~~
       *
       * \param [in,out] cmddef Command definition.
       * \param [in,out] form   Command form definition. 
       * \param [in]     tokens Lexical tokens generated from the syntax usage.
       * \param [in,out] pos    The parse cursor position. \sa parseSyntax().
       *
       * \return Returns true on a successful parse, false otherwise.
       */
      bool parseXorListArg(CmdDef         &cmddef,
                           CmdFormDef     &form,
                           const TokenVec &tokens,
                           size_t         &pos);
      
      /*!
       * \brief Parse variable argument syntax.
       *
       * A CmdArgDef object is added to the CmdDef
       *
       * \par Syntax:
       * ~~~~~~~~
       * <variable-arg> ::= '<' <identifier> '>'
       *                  | '<' <identifier> ':' <var-modifier> '>'
       * ~~~~~~~~
       *
       * \param [in,out] cmddef Command definition.
       * \param [in,out] form   Command form definition. 
       * \param [in]     tokens Lexical tokens generated from the syntax usage.
       * \param [in,out] pos    The parse cursor position. \sa parseSyntax().
       *
       * \return Returns true on a successful parse, false otherwise.
       */
      bool parseVariableArg(CmdDef         &cmddef,
                            CmdFormDef     &form,
                            const TokenVec &tokens,
                            size_t         &pos);

      /*!
       * \brief Parse literal, fixed-valued argument syntax.
       *
       * A CmdArgDef object is added to the CmdDef
       *
       * \par Syntax:
       * ~~~~~~~~
       * <literal-arg> ::= <literal>
       * ~~~~~~~~
       *
       * \param [in,out] cmddef Command definition.
       * \param [in,out] form   Command form definition. 
       * \param [in]     tokens Lexical tokens generated from the syntax usage.
       * \param [in,out] pos    The parse cursor position. \sa parseSyntax().
       *
       * \return Returns true on a successful parse, false otherwise.
       */
      bool parseLiteralArg(CmdDef         &cmddef,
                           CmdFormDef     &form,
                           const TokenVec &tokens,
                           size_t         &pos);

      /*!
       * \brief Parse mutually exclusive list.
       *
       * \par Syntax:
       * ~~~~~~~~
       * <xor-list> ::= <literal>
       *              | <xor-list> '|' <literal>
       * ~~~~~~~~
       *
       * \param [in,out] cmddef   Command definition.
       * \param [in,out] form     Command form definition. 
       * \param [in]     tokens   Lexical tokens generated from syntax usage.
       * \param [in,out] pos      The parse cursor position. \sa parseSyntax().
       * \param [out]    literals Vector of literals.
       *
       * \return Returns true on a successful parse, false otherwise.
       */
      bool parseXorList(CmdDef         &cmddef,
                        CmdFormDef     &form,
                        const TokenVec &tokens,
                        size_t         &pos,
                        str::StringVec &literals);

      /*!
       * \brief Parse identifier.
       *
       * \par Syntax:
       * ~~~~~~~~
       * <identifier> ::= ALPHA {ALPHANUMERIC}*
       *                | '_' {ALPHANUMERIC}*
       * ~~~~~~~~
       *
       * \param [in,out] cmddef   Command definition.
       * \param [in,out] form     Command form definition. 
       * \param [in]     tokens   Lexical tokens generated from syntax usage.
       * \param [in,out] pos      The parse cursor position. \sa parseSyntax().
       * \param [out]    strIdent Identity string.
       *
       * \return Returns true on a successful parse, false otherwise.
       */
      bool parseIdentifier(CmdDef         &cmddef,
                           CmdFormDef     &form,
                           const TokenVec &tokens,
                           size_t         &pos,
                           std::string    &strIdent);
      
      /*!
       * \brief Parse variable modifier.
       *
       * \par Syntax:
       * ~~~~~~~~
       * <var-modifier> ::= <var-type>
       *                  | <var-type> <var-paren-expr>
       *
       * <var-paren-expr> ::= '(' <range-expr> ')'
       *                    | '(' <regex> ')'
       * ~~~~~~~~
       *
       * \param [in,out] cmddef Command definition.
       * \param [in,out] form   Command form definition. 
       * \param [in]     tokens Lexical tokens generated from the syntax usage.
       * \param [in,out] pos    The parse cursor position. \sa parseSyntax().
       * \param [out]    eType  Type enum.
       * \param [out]    ranges Vector of ranges.
       * \param [out]    re     Regular expression.
       *
       * \return Returns true on a successful parse, false otherwise.
       */
      bool parseVarMod(CmdDef              &cmddef,
                       CmdFormDef          &form,
                       const TokenVec      &tokens,
                       size_t              &pos,
                       CmdArgDef::ArgType  &eType,
                       CmdArgDef::RangeVec &ranges,
                       RegEx               &re);

      /*!
       * \brief Parse variable type.
       *
       * \par Syntax:
       * ~~~~~~~~
       * <var-type> ::= word | multiword | identifier | bool |
       *                int | fpn | re | file
       * ~~~~~~~~
       *
       * \param [in,out] cmddef Command definition.
       * \param [in,out] form   Command form definition. 
       * \param [in]     tokens Lexical tokens generated from the syntax usage.
       * \param [in,out] pos    The parse cursor position. \sa parseSyntax().
       * \param [out]    eType  Type enum.
       *
       * \return Returns true on a successful parse, false otherwise.
       */
      bool parseVarType(CmdDef             &cmddef,
                        CmdFormDef         &form,
                        const TokenVec     &tokens,
                        size_t             &pos,
                        CmdArgDef::ArgType &eType);

      /*!
       * \brief Parse variable range expression.
       *
       * \par Syntax:
       * ~~~~~~~~
       * <range-expr> ::= <range>
       *                | <range-expr> ',' <range>
       *
       * <range> ::= NUMBER
       *           | NUMBER ':' NUMBER
       * ~~~~~~~~
       *
       * \param [in,out] cmddef Command definition.
       * \param [in,out] form   Command form definition. 
       * \param [in]     tokens Lexical tokens generated from the syntax usage.
       * \param [in,out] pos    The parse cursor position. \sa parseSyntax().
       * \param [out]    ranges Vector of ranges.
       *
       * \return Returns true on a successful parse, false otherwise.
       */
      bool parseVarRangeExpr(CmdDef              &cmddef,
                             CmdFormDef          &form,
                             const TokenVec      &tokens,
                             size_t              &pos,
                             CmdArgDef::RangeVec &ranges);

      /*!
       * \brief Parse variable regular expression.
       *
       * \par Syntax:
       * ~~~~~~~~
       * <regex> ::= REGEX
       * ~~~~~~~~
       *
       * \param [in,out] cmddef Command definition.
       * \param [in,out] form   Command form definition. 
       * \param [in]     tokens Lexical tokens generated from the syntax usage.
       * \param [in,out] pos    The parse cursor position. \sa parseSyntax().
       * \param [out]    re     Regular expression.
       *
       * \return Returns true on a successful parse, false otherwise.
       */
      bool parseVarRegExpr(CmdDef         &cmddef,
                           CmdFormDef     &form,
                           const TokenVec &tokens,
                           size_t         &pos,
                           RegEx          &re);

      /*!
       * \brief Parse literal value.
       *
       * \par Syntax:
       * ~~~~~~~~
       * <literal> ::= {NON_SPECIAL_CHAR}+
       * ~~~~~~~~
       *
       * \param [in,out] cmddef   Command definition.
       * \param [in,out] form     Command form definition. 
       * \param [in]     tokens   Lexical tokens generated from syntax usage.
       * \param [in,out] pos      The parse cursor position. \sa parseSyntax().
       * \param [out]    strValue Value string.
       *
       * \return Returns true on a successful parse, false otherwise.
       */
      bool parseLiteralValue(CmdDef         &cmddef,
                             CmdFormDef     &form,
                             const TokenVec &tokens,
                             size_t         &pos,
                             std::string    &strValue);

      /*!
       * \brief Test if token at position is equal to string.
       *
       * If equal, the position cursor is advanced on position. If not equal,
       * an error message will be logged.
       *
       * \param [in]     strCmp String to compare token against.
       * \param [in]     tokens Lexical tokens generated from the syntax usage.
       * \param [in,out] pos    The position in the token vector.
       *                        Advanced 1 position if equal.
       *
       * \return Returns true on equal, false otherwise.
       */
      bool tokEq(const std::string strCmp, const TokenVec &tokens, size_t &pos);

      /*!
       * \brief Peek if token is equal to string.
       *
       * The position cursor is not advance nor any error message logged.
       *
       * \param [in] strCmp String to compare token against.
       * \param [in] token  Lexical token.
       *
       * \return Returns true on equal, false otherwise.
       */
      bool peekEq(const std::string strCmp, const Token &token) const
      {
        return token.value() == strCmp;
      }
      
      /*!
       * \brief Test if string has valid identifier syntax.
       *
       * \param [in]     tokens Lexical tokens generated from the syntax usage.
       * \param [in,out] pos    The position in the token vector.
       *                        Advanced 1 position if valid identifier.
       *
       * \return Returns true or false.
       */
      bool tokIdentifier(const TokenVec &tokens, size_t &pos);
  

      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Command Line Interface Support Methods
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

      /*!
       * \brief Process a line of input.
       *
       * The line is tokenized and matched to the command with the best fit.
       *
       * \param [in]  strLine   Line of input.
       * \param [out] argv      Vector of extended arguments, with argv[0] being
       *                        the command name argument.
       *
       *  \copydoc doc_return_cl
       */
      int processInput(const std::string &strLine, CmdExtArgVec &argv);

      /*!
       * \brief Match the input tokens to the compiled commands to find the
       * best fit.
       *
       * \param [in]  tokens  Vector of input tokens.
       * \param [out] argv    Vector of extended arguments, with argv[0] being
       *                      the command name argument.
       *
       *  \copydoc doc_return_cl
       */
      int match(const TokenVec &tokens, CmdExtArgVec &argv);

      /*!
       * \brief Match best command form against input line argument list.
       *
       * \param [in]  cmddef    Compiled command definition.
       * \param [in]  tokens    Vector of arguments.
       * \param [out] argv      Vector of extended arguments, with argv[0] being
       *                        the command name argument.
       * \param [out] fFitness  Fitness of match [0.0 - 1.0].
       *
       *  \copydoc doc_return_cl
       */
      int matchCommand(const CmdDef   &cmddef,
                       const TokenVec &tokens,
                       CmdExtArgVec   &argv,
                       double         &fFitness);

      /*!
       * \brief Match command form against input line argument list.
       *
       * \param [in]  form      Compiled command form definition.
       * \param [in]  tokens    Vector of arguments.
       * \param [out] argv      Vector of extended arguments, with argv[0] being
       *                        the command name argument.
       * \param [out] fFitness  Fitness of match [0.0 - 1.0].
       *
       *  \copydoc doc_return_cl
       */
      int matchCommandForm(const CmdFormDef &form,
                           const TokenVec   &tokens,
                           CmdExtArgVec     &argv,
                           double           &fFitness);

      /*!
       * \brief Check read input result.
       *
       *  \copydoc doc_return_cl
       */
      int checkReadResult();
  
      /*!
       * \brief Convert extended argument vector to string argument vector.
       *
       * \param [in]  v1  Extended argument vector.
       * \param [out] v2  String argument vector.
       */
      void toVec(const CmdExtArgVec &v1, str::StringVec &v2);


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
       * \brief Static TAB completion generator wrapper.
       *
       * See \ref ReadLine::AltAppGenFunc for description of a ReadLine
       * generator.
       *
       * \param pAppArg       Pointer to this CommandLine object.
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
      static const std::string rlGeneratorWrapper(void              *pAppArg,
                                                  const std::string &strText,
                                                  int               nIndex,
                                                  const std::string &strContext,
                                                  int               nStart,
                                                  int               nEnd,
                                                  unsigned          &uFlags);

      /*!
       * \brief TAB completion generator.
       *
       * See \ref ReadLine::AltAppGenFunc for description of a ReadLine
       * generator.
       *
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
      virtual const std::string rlGenerator(const std::string &strText,
                                            int               nIndex,
                                            const std::string &strContext,
                                            int               nStart,
                                            int               nEnd,
                                            unsigned          &uFlags);

      /*!
       * \brief Build list of all command argument definitions that match
       * completed subtext.
       *
       * \param [in] strSubtext Complete subtext string.
       * \param [out] argdefs   List of matched argument definitions.
       */
      void rlArgDefs(const std::string       &strSubtext,
                     std::vector<CmdArgDef*> &argdefs);

      /*!
       * \brief Build TAB completion list and set appropriate readline
       * modifiers.
       *
       * \param [in] strText  Partial text string to complete.
       * \param [in] argdefs  List of matched argument definitions.
       * \param [out] tabList List of TAB completion matches.
       * \param [out] uFlags  TAB completion modifier flags.
       */
      void rlTabList(const std::string       &strText,
                     std::vector<CmdArgDef*> &argdefs,
                     str::StringVec          &tabList,
                     unsigned                &uFlags);

      /*!
       * \brief Match partial text agains literal string.
       *
       * Ignore case is taken into account when strings are compared.
       *
       * \param [in] strText    Partial text string to complete.
       * \param [in] strLiteral Literal text string to compare.
       * \param [in] uLen       Number of characters to match.
       *
       * \return Returns true on partial match, false otherwise.
       */
      bool rlPartialMatch(const std::string &strText,
                          const std::string strLiteral,
                          const size_t      uLen);

    }; // class CommandLine

  } // namespace cmd
} // namespace rnr

#endif // _RNR_COMMAND_LINE_H
