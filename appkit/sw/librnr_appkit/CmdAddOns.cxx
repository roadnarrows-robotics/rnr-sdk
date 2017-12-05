////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_appkit
//
// File:      CmdAddOns.cxx
//
/*! \file
 *
 * \brief Command line interface command add-ons implementation.
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

#include <unistd.h>
#include <stdlib.h>

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <set>
#include <map>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/appkit/StringTheory.h"
#include "rnr/appkit/CmdCore.h"
#include "rnr/appkit/CmdDef.h"
#include "rnr/appkit/CommandLine.h"
#include "rnr/appkit/CmdAddOns.h"

using namespace std;
using namespace rnr;
using namespace rnr::str;
using namespace rnr::cmd;
using namespace rnr::cmd::addons;

/*!
 * \brief RoadNarrows Robotics
 */
namespace rnr
{
  namespace cmd
  {
    namespace addons
    {
      //------------------------------------------------------------------------
      // Private
      //------------------------------------------------------------------------

      static string atname("@N@"); ///< string to replace by command name


      //------------------------------------------------------------------------
      // Help Command Add-On
      //------------------------------------------------------------------------

      /*!
       * \brief Execute 'help' command.
       *
       * \par Syntax:
       * help [{--list | -l | --synopsis | -s | --usage | -u}] [<cmd>]
       *
       * \todo Support ellipses ... syntax.
       *
       * \param cli   Command line interface.
       * \param argv  Extended command line arguments.
       *
       * \return OK(0) on success, negative value on failure.
       */
      static int execHelp(CommandLine &cli, const CmdExtArgVec &argv)
      {
        HelpSect section = HelpSectAll;         // default help option
  
        std::set<string>            setOfCmds;  // set of commands for help
        std::set<string>::iterator  iterHelp;   // help commands iterator
        size_t                      i;
      
        //
        // Process Input arguments.
        //
        for(i = 1; i < argv.size(); ++i)
        {
          if( (argv[i] == "--brief") || (argv[i] == "-b") )
          {
            section = HelpSectBrief;
          }
          else if( (argv[i] == "--list") || (argv[i] == "-l") )
          {
            section = HelpSectName;
          }
          else if( (argv[i] == "--usage") || (argv[i] == "-u") )
          {
            section = HelpSectUsage;
          }
          else
          {
            setOfCmds.insert(argv[i].s());
          }
        }
      
        //
        // All commands.
        //
        if( setOfCmds.empty() )
        {
          CmdDefCIter iterDefs;
  
          // alphbetize
          for(iterDefs = cli.begin(); iterDefs != cli.end(); ++iterDefs)
          {
            setOfCmds.insert(iterDefs->second.getName());
          }
        }
  
        //
        // Print help for command(s).
        //
        for(iterHelp = setOfCmds.begin();
            iterHelp != setOfCmds.end();
            ++iterHelp)
        {
          const string &cmdName = *iterHelp;
          const CmdDef &cmdDef  = cli.at(cmdName);
  
          if( cmdDef.getUid() == NoUid )
          {
            cout << "Error: help: No help for command '" << cmdName << "'."
              << endl;
            return RC_ERROR;
          }
  
          printCmdHelp(cout, cmdDef, section);
  
          switch( section )
          {
            case HelpSectName:
              cout << " ";
              break;
            case HelpSectBrief:
            case HelpSectUsage:
              cout << endl;
              break;
            case HelpSectAll:
            default:
              cout << "    ---" << endl << endl;
              break;
          }
        }
  
        switch( section )
        {
          case HelpSectName:
            cout << endl;
            break;
          case HelpSectBrief:
          case HelpSectUsage:
          case HelpSectAll:
          default:
            cout << "  " << setOfCmds.size() << " commands" << endl;
            break;
        }
      
        return OK;
      }
  
      int printCmdHelp(ostream        &os,
                       const CmdDef   &cmddef,
                       const HelpSect section)
      {
        return printCmdHelp(os, cmddef.getName(), cmddef.getSyntax(),
                         cmddef.getSynopsis(), cmddef.getLongDesc(),
                         section);
      }
  
      int printCmdHelp(ostream &os, const CmdDesc &desc, const HelpSect section)
      {
        return printCmdHelp(os,
                        desc.name, desc.syntax, desc.synopsis, desc.longdesc,
                        section);
      }
  
      int printCmdHelp(ostream        &os,
                       const string   &strName,
                       const string   &strSyntax,
                       const string   &strSynopsis,
                       const string   &strLongDesc,
                       const HelpSect section)
      {
        size_t  i;
      
        if( strName.empty() )
        {
          os << "Error: Command has no name." << endl;
          return RC_ERROR;
        }
  
        if( strSyntax.empty() )
        {
          os << "Error: Command has no extended usage syntax." << endl;
          return RC_ERROR;
        }
  
        StringVec usages = split(strSyntax, '\n');
  
        switch( section )
        {
          case HelpSectName:
            os << strName;
            break;
  
          case HelpSectBrief:
            os << strName << " - " << strSynopsis << endl;
            break;
  
          case HelpSectUsage:
            for(i = 0; i < usages.size(); ++i)
            {
              if( !usages[i].empty() )
              {
                os << usages[i] << "\n";
              }
            }
            break;
  
          case HelpSectAll:
          default:
            ssize_t len;
            size_t  w;
  
            len = (ssize_t)strName.size() + 2;
            len = 80 - 2 * len;
            os << strName << "()";
            if( len > 1 )
            {
              w = os.width();
              os.width(len);
              os << "";
              os.width(w);
              os << strName << "()";
            }
            os << "\n\n";
  
            os << "Name\n  " << strName;
            os << " - " << strSynopsis;
            os << "\n\n";
  
            os << "Synopsis\n";
            for(i = 0; i < usages.size(); ++i)
            {
              if( !usages[i].empty() )
              {
                os << "  " << usages[i] << "\n";
              }
            }
            os << "\n";
  
            if( !strLongDesc.empty() )
            {
              os << "Description\n";
  
              StringVec lines = split(strLongDesc, '\n');
              size_t    wstart, wend, wlen; // word data
              size_t    cursor;             // output line cursor
  
              for(i = 0; i < lines.size(); ++i)
              {
                // indent
                os << "  ";
                cursor = 2;
  
                //
                // Output line, wrapping if necessary.
                //
                for(size_t j = 0; j < lines[i].size(); )
                {
                  // whitespace
                  while( isspace(lines[i][j]) && (j < lines[i].size()) )
                  {
                    // less than max output line length
                    if( cursor < 80 )
                    {
                      cout << ' ';
                      ++cursor;
                    }
                    // wrap
                    else
                    {
                      // wrap and indent
                      cout << "\n  ";
                      cursor = 2;
                    }
                    ++j;
                  }
  
                  if( j >= lines[i].size() )
                  {
                    break;
                  }
  
                  // word
                  wstart = j;
  
                  while( !isspace(lines[i][j]) && (j < lines[i].size()) )
                  {
                    ++j;
                  }
  
                  wend  = j;
                  wlen  = wend - wstart;
  
                  // wrap
                  if( (cursor > 2) && (cursor+wlen >= 80) )
                  {
                    /// wrap and indent
                    cout << "\n  ";
                    cursor = 2;
                  }
  
                  cout << lines[i].substr(wstart, wlen);
                  cursor += wlen;
                }
  
                cout << '\n';
              }
              cout << '\n';
            }
            break;
        }
  
        return OK;
      }
  
      int addHelpCommand(CommandLine &cli, const string &strName)
      {
        string  cmd(strName);
        int     uid;

        if( cmd.empty() )
        {
          cmd = "help";
        }

        CmdDesc desc =
        {
          .name = cmd,

          .syntax = replace(atname, cmd, 
            "@N@ [{--brief | -b | --list | -l | --usage | -u}] [<cmd>]"),

          .synopsis = "Print command help.",

          .longdesc =
            "Print command help. "
            "If a <cmd> is specified, then only help for that command is "
            "printed. Otherwise all command help is printed.\n\n"
            "Mandotory arguments to long options are mandatory for short "
            "options too.\n\n"
            "-b, --brief  Print only command brief descriptions.\n"
            "-l, --list   Print only command names\n"
            "-u, --usage  Print only command usages (syntax)."
        };

        uid = cli.addCommand(desc, execHelp);
      
        return uid;
      }
      

      //------------------------------------------------------------------------
      // Quit Command Add-On
      //------------------------------------------------------------------------

      /*!
       * \brief Execute 'quit' command.
       *
       * \par Syntax:
       * quit
       *
       * \param cli   Command line interface.
       * \param argv  Extended command line arguments.
       *
       * \return OK(0) on success, negative value on failure.
       */
      static int execQuit(CommandLine &cli, const CmdExtArgVec &argv)
      {
        cli.quit();
        return OK;
      }
  
      int addQuitCommand(CommandLine &cli, const string &strName)
      {
        string  cmd(strName);
        int     uid;

        if( cmd.empty() )
        {
          cmd = "quit";
        }

        CmdDesc desc =
        {
          .name = cmd,

          .syntax = replace(atname, cmd, "@N@"),

          .synopsis = replace(atname, cmd, "@N@ command-line interface.")
        };

        uid = cli.addCommand(desc, execQuit);
      
        return uid;
      }


      //------------------------------------------------------------------------
      // Backtrace Enable Command Add-On
      //------------------------------------------------------------------------

      /*!
       * \brief Execute 'backtrace' command.
       *
       * \par Syntax:
       * bt <onoff>
       *
       * \param cli   Command line interface.
       * \param argv  Extended command line arguments.
       *
       * \return OK(0) on success, negative value on failure.
       */
      static int execBtEnable(CommandLine &cli, const CmdExtArgVec &argv)
      {
        cli.setBtEnable(argv[1].b());
        return OK;
      }

      int addBtEnableCommand(CommandLine &cli, const string &strName)
      {
        string  cmd(strName);
        int     uid;

        if( cmd.empty() )
        {
          cmd = "bt";
        }

        CmdDesc desc =
        {
          .name = cmd,

          .syntax = replace(atname, cmd, "@N@ <onoff:bool>"),

          .synopsis = "Enable/disable backtracing of input line parsing.",

          .longdesc =
            "Enable/disable backtracing of input line parsing."
            "The <onoff> argument is a boolean string whose conversion is "
            "either true(enable) and false(disable)."
        };

        uid = cli.addCommand(desc, execBtEnable);
      
        return uid;
      }

    } // namespace addons
  } // namespace cmd
} // namespace rnr

