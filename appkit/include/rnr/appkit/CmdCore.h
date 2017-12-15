////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_appkit
//
// File:      CmdCore.h
//
/*! \file
 *
 * \brief Command line core data types.
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

#ifndef _RNR_CMD_CORE_H
#define _RNR_CMD_CORE_H

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <ctype.h>

#include <iostream>
#include <string>

#include <rnr/rnrconfig.h>

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
    /*!
     * \brief User available command description structure.
     * 
     * This structure is provided as a convenience for application development.
     * It is external to the core CommandLine functionality.
     */
    struct CmdDesc
    {
      std::string name;       ///< command name
      std::string syntax;     ///< parsable command extended usage syntax
      std::string synopsis;   ///< short command synopsis
      std::string longdesc;   ///< long command description
    };

    const int AOk         =  OK; ///< (0) A-Ok, no error, success, good

    /*!
     * \defgroup cmd_ecodes Command Error Codes
     * \{
     */
    const int EError      = -1;   ///< general, unspecified error
    const int EEoF        = -2;   ///< end of file
    const int ERead       = -3;   ///< read error
    const int EAmbigCmd   = -4;   ///< ambiguous command
    const int EUnknownCmd = -5;   ///< unknown, unmatched command
    const int EBadSyntax  = -6;   ///< bad syntax
    const int ENoExec     = -7;   ///< cannot execute
    const int EArgv0      = -8;   ///< not this command argv0
    const int ENoOp       = -9;   ///< no operation
    const int EBadVal     = -10;  ///< bad value

    const int NumOfECodes = 10;   ///< number of error codes.
    /*! \} */

    /*!
     * \brief Special values.
     */
    const int NoUid       = -1;  ///< no unique id
    const int NoIndex     = -1;  ///< no index

    /*!
     * \brief Variable argument symbol names.
     */
    const char* const ArgSymLiteral     = "literal";    ///< literal constant
    const char* const ArgSymWord        = "word";       ///< non-whitespace seq
    const char* const ArgSymMultiWord   = "multiword";  ///< any sequence
    const char* const ArgSymIdentifier  = "identifier"; ///< identifier
    const char* const ArgSymBoolean     = "bool";       ///< boolean (bool)
    const char* const ArgSymInteger     = "int";        ///< integer (long)
    const char* const ArgSymFpn         = "fpn";        ///< fpn (double)
    const char* const ArgSymFile        = "file";       ///< file path
    const char* const ArgSymRegEx       = "re";         ///< regular expression

    //
    // Special string values.
    //
    extern const std::string emptystring; ///< "" empty string
    extern const std::string undefstring; ///< "undef" string
    
    /*!
     * \brief Reserved command line data section namespaces.
     */
    const char* const DataSectNsCore  = "core";   ///< core data section ns
    const char* const DataSectNsOS    = "os";     ///< OS data section ns
    const char* const DataSectNsNet   = "net";    ///< network data section ns

    /*!
     * \brief Core data section type.
     */
    struct DataSectCore
    {
      bool  m_bQuit;        ///< command-line should [not] quit
      bool  m_bBacktrace;   ///< do [not] backtrace command line parsing

      /*!
       * \brief Default contructor.
       */
      DataSectCore()
      {
        m_bQuit       = false;
        m_bBacktrace  = false;
      }

      /*!
       * \brief Deallocate (delete) allocated (new) core data section.
       *
       * \param p   Pointer to data section object.
       */
       static void dealloc(void *p);
    };

    /*!
     * \brief Handy Dandy Name-Value Pair entry structure.
     */
    struct NameValuePair
    {
      const char* m_sName;    ///< name
      int         m_nValue;   ///< value
    };

    /*!
     * \brief Test if string is a valid identifier.
     *
     * Identifiers adhere to the C syntax.
     *
     * \param [in] str    String to test.
     *
     * \return Returns true or false.
     */
    bool isIdentifier(const std::string &str);

  } // namespace cmd
} // namespace rnr

#endif // _RNR_CMD_CORE_H
