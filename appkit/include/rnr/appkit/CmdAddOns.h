////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_appkit
//
// File:      CmdAddOns.h
//
/*! \file
 *
 * \brief Command line interface command add-ons interface.
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

#ifndef _RNR_CMD_ADD_ONS_H
#define _RNR_CMD_ADD_ONS_H

#include <unistd.h>
#include <stdlib.h>

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/appkit/CmdDef.h"
#include "rnr/appkit/CommandLine.h"

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
     * \brief Add-Ons
     */
    namespace addons
    {
      //------------------------------------------------------------------------
      // Help Command Add-On
      //------------------------------------------------------------------------
  
      /*!
       * \brief Command help section identifiers.
       */
      enum HelpSect
      {
        HelpSectAll,    ///< all commond help sections
        HelpSectName,   ///< command name help section
        HelpSectBrief,  ///< command brief description help section
        HelpSectUsage   ///< command usage (synopsis) help section
      };
  
      /*!
       * \brief Print help for a command convenience function.
       *
       * \param os          Output stream.
       * \param cmddef      Command definition.
       * \param section     Help section.
       *
       * \return OK(0) on success, negative value on failure.
       */
      extern int printCmdHelp(std::ostream   &os,
                              const CmdDef   &cmddef,
                              const HelpSect section = HelpSectAll);
      
      /*!
       * \brief Print help for a command convenience function.
       *
       * \param os          Output stream.
       * \param desc        Command description.
       * \param section     Help section.
       *
       * \return OK(0) on success, negative value on failure.
       */
      extern int printCmdHelp(std::ostream   &os,
                              const CmdDesc  &desc,
                              const HelpSect section = HelpSectAll);
  
      /*!
       * \brief Print help for a command convenience function.
       *
       * \param os          Output stream.
       * \param strName     Command name.
       * \param strSyntax   Command usage syntax.
       * \param strSynopsis Command short synopsis.
       * \param strLongDesc Command long description.
       * \param section     Help section.
       *
       * \return OK(0) on success, negative value on failure.
       */
      extern int printCmdHelp(std::ostream      &os,
                              const std::string &strName,
                              const std::string &strSyntax,
                              const std::string &strSynopsis,
                              const std::string &strLongDesc,
                              const HelpSect    section = HelpSectAll);
  
      /*!
       * \brief Add the core 'help' command to the command-line interface.
       *
       * Syntax: help [{--brief | -b | --list | -l | --usage | -u}] [<cmd>]
       *
       * The help command will print help for the given help sections for the
       * given command names to cout.
       *
       * Any relevant command data is applied to the command line interface
       * "core" data section.
       *
       * The name of the command may be overridded, but the syntax and
       * semantics cannot.
       *
       * \param cli     Command-line interface.
       * \param strName Name of the command.
       *
       * \return  On success, returns command's assigned unique id.
       *          Otherwise NoUid is returned.
       */
      int addHelpCommand(CommandLine &cli, const std::string &strName = "help");
  
  
      //------------------------------------------------------------------------
      // Quit Command Add-On
      //------------------------------------------------------------------------

      /*!
       * \brief Add the core 'quit' command to the command-line interface.
       *
       * Syntax: quit
       *
       * The quit command will disable anymore execution of the command line.
       * It is up to the application to terminate by querying the command line
       * ok() method.
       *
       * Any relevant command data is applied to the command line interface
       * "core" data section.
       *
       * The name of the command may be overridded, but the syntax and
       * semantics cannot.
       *
       * \param cli     Command-line interface.
       * \param strName Name of the command.
       *
       * \return  On success, returns command's assigned unique id.
       *          Otherwise NoUid is returned.
       */
      int addQuitCommand(CommandLine &cli, const std::string &strName = "quit");


      //------------------------------------------------------------------------
      // Backtrace Enable Command Add-On
      //------------------------------------------------------------------------

      /*!
       * \brief Add the core 'bt' command to the command-line interface.
       *
       * Syntax: bt <onoff>
       *
       * The bt command enables/disables backtracing of input line parsing.
       * The trace is written to cerr.
       *
       * Any relevant command data is applied to the command line interface
       * "core" data section.
       *
       * The name of the command may be overridded, but the syntax and
       * semantics cannot.
       *
       * \param cli     Command-line interface.
       * \param strName Name of the command.
       *
       * \return  On success, returns command's assigned unique id.
       *          Otherwise NoUid is returned.
       */
      int addBtEnableCommand(CommandLine &cli,
                             const std::string &strName = "bt");

    } // namespace addons
  } // namespace cmd
} // namespace rnr

#endif // _RNR_CMD_ADD_ONS_H
