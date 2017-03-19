////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Program:   dynashell
//
// File:      dynashell_cmd.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
 * $Rev: 3845 $
 *
 * \brief The Dynamixel Shell Command Base and Derived Core Classes.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2011-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
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


#include <sys/time.h>
#include <sys/types.h>
#include <stdio.h>
#include <unistd.h>
#include <ctype.h>

#include <cstring>
#include <iostream>
#include <fstream>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaServo.h"
#include "Dynamixel/DynaChain.h"
#include "Dynamixel/DynaBgThread.h"

#include "dynashell.h"
#include "dynashell_cmd.h"
#include "dynashell_util.h"

using namespace std;


// -----------------------------------------------------------------------------
// DynaShellCmdNULL Class
// -----------------------------------------------------------------------------

/*!
 * \brief NULL command.
 */
class DynaShellCmdNULL : public DynaShellCmd
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdNULL() : DynaShellCmd(0, 0)
  {
    m_sCmdName      = "";
    m_sCmdHelpArgs  = "";
    m_sCmdHelpDesc  = "";
  }

  /*!
   * \brief NULL
   *
   * \param shell   Dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   */
  void Exec(DynaShell &shell, int argc, char *argv[])
  {
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdHelp Class
// -----------------------------------------------------------------------------

/*!
 * \brief Help command.
 */
class DynaShellCmdHelp : public DynaShellCmd
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdHelp() : DynaShellCmd(0, 2)
  {
    m_sCmdName      = "help";
    m_sCmdHelpBrief = "Print command help.";
    m_sCmdHelpArgs  = "[<cmd> [object]]";
    m_sCmdHelpDesc  = "Print list of commands or command long help. "
                      "If no arguments are "
                      "specified, then only the list of commands is printed. "
                      "If only the <cmd> argument is specified, and <cmd> is "
                      "of the form:\n"
                      "  verb object [qualifiers]\n"
                      "then a list of the command objects is printed. "
                      "Otherwise, the long help is printed for the given "
                      "command.\n"
                      "  <cmd>     Root command name (verb).\n"
                      "  <object>  Sub-command name (object).";

  }

  /*!
   * \brief Print help.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   */
  void Exec(DynaShell &shell, int argc, char *argv[])
  {
    ShCmdNode   cmdNode;

    TRY( ChkArgCnt(shell, argc) );

    // print root command list
    if( argc == 0 )
    {
      //PrintHelpWordList(shell, shell.GetRootCmds());
      PrintHelpSummaryList(shell, shell.GetRootCmds());
      printf("\n");
      return;
    }

    cmdNode = MatchHelp(shell, argc, argv, shell.GetRootCmds());

    // print command object list
    if( cmdNode.IsMap() )
    {
      // PrintHelpWordList(shell, *cmdNode.GetMap());
      PrintHelpSummaryList(shell, *cmdNode.GetMap());
      printf("\n");
    }

    // print long help.
    else if( cmdNode.IsCmd() )
    {
      cmdNode.GetCmd()->PrintHelp();
      printf("\n");
    }

    // print error 1
    else if( argc == 1 )
    {
      shell.Error("help: Unknown command \"%s\".", argv[0]);
    }

    // or print error 2
    else
    {
      shell.Error("help: Unknown command \"%s %s\".", argv[0], argv[1]);
    }
  }

  /*!
   * \brief Command tab completion generator.
   *
   * Completes <group> [object]
   *
   * \param shell     Dynamixel shell.
   * \param sText     Partial text string to complete.
   * \param uTextLen  Length of text.
   * \param nState    Generator state. If FIRST, then initialize any statics.
   * \param sContext  Generator context (i.e. canonical command path).
   *
   * \return
   * If a first/next match is made, return allocated completed match.\n
   * Otherwise return NULL.
   */
  virtual char *TabCompletion(DynaShell   &shell,
                              const char  *sText,
                              size_t       uTextLen,
                              int          nState,
                              const char  *sContext)
  {
    size_t      uCmdLen = strlen(m_sPubName);
    const char *sSubContext;

    // this case should not happen
    if( strlen(sContext) < uCmdLen )
    {
      return NULL;
    }

    // "help"
    else if( strlen(sContext) == uCmdLen )
    {
      sSubContext = "";
    }

    // "help group "
    else
    {
      sSubContext = sContext+uCmdLen+1;
    }

    return shell.CmdListGenerator(0, sText, uTextLen, nState, sSubContext);
  }


protected:
  /*!
   * \brief Print help list of words.
   *
   * \param shell   Dynamixel shell.
   * \param cmdMap  Command map to print.
   */
  void PrintHelpWordList(DynaShell &shell, ShCmdMap &cmdMap)
  {
    ShCmdMap::iterator  iter;

    for(iter=cmdMap.begin(); iter!=cmdMap.end(); iter++)
    {
      printf("%s ", iter->first);
    }

    printf("\n");
  }

  /*!
   * \brief Print help summary list.
   *
   * \param shell   Dynamixel shell.
   * \param cmdMap  Command map to print.
   */
  void PrintHelpSummaryList(DynaShell &shell, ShCmdMap &cmdMap)
  {
    ShCmdMap::iterator  iter;

    for(iter=cmdMap.begin(); iter!=cmdMap.end(); iter++)
    {
      printf("%15s - %s\n", iter->first, iter->second.GetMapBrief().c_str());
    }
  }

  /*!
   * \brief Match command.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   * \param cmdMap  Command map to search.
   *
   * \return Shell command node.
   */
  ShCmdNode &MatchHelp(DynaShell &shell,
                       int        argc,
                       char      *argv[],
                       ShCmdMap  &cmdMap)
  {
    static ShCmdNode    cmdNull;

    ShCmdMap::iterator  iter1, iter2;
    char               *sPattern;
    const char         *sCmdName1, *sCmdName2;
    size_t              n;
  
    if( argc == 0 )
    {
      return cmdNull;
    }

    sPattern = argv[0];

    n = strlen(sPattern);

    // find command
    for(iter1=cmdMap.begin(); iter1!=cmdMap.end(); iter1++)
    {
      sCmdName1 = iter1->first;
      if( !strncmp(sPattern, sCmdName1, n) )
      {
        break;
      }
    }

    // no matching command found
    if( iter1 == cmdMap.end() )
    {
      return cmdNull;
    }

    // verify uniqueness
    for(iter2=cmdMap.begin(); iter2!=cmdMap.end(); iter2++)
    {
      if( iter2 == iter1 )
      {
        continue;
      }

      sCmdName2 = iter2->first;

      // another match 
      if( !strncmp(sPattern, sCmdName2, n) )
      {
        shell.Error("help: %s: (Sub)Command not unique: Matches %s %s ...",
            sPattern, sCmdName1, sCmdName2);
        return cmdNull;
      }
    }

    if( argc == 1 )
    {
      return iter1->second;
    }
    else if( (argc == 2) && iter1->second.IsMap() )
    {
      return MatchHelp(shell, argc-1, argv+1, *iter1->second.GetMap());
    }
    else
    {
      return cmdNull;
    }
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdQuit Class
// -----------------------------------------------------------------------------

/*!
 * \brief Quit command.
 */
class DynaShellCmdQuit : public DynaShellCmd
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdQuit() : DynaShellCmd(0, 0)
  {
    m_sCmdName      = "quit";
    m_sCmdHelpBrief = "Quit (exit) dynashell.";
    m_sCmdHelpArgs  = "";
    m_sCmdHelpDesc  = "";
  }

  /*!
   * \brief Quit shell.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   */
  void Exec(DynaShell &shell, int argc, char *argv[])
  {
    shell.m_bRun = false;
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdExit Class
// -----------------------------------------------------------------------------

/*!
 * \brief Exit command.
 */
class DynaShellCmdExit : public DynaShellCmd
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdExit() : DynaShellCmd(0, 0)
  {
    m_sCmdName      = "exit";
    m_sCmdHelpBrief = "Exit (quit) dynashell.";
    m_sCmdHelpArgs  = "";
    m_sCmdHelpDesc  = "";
  }

  /*!
   * \brief Exit shell.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   */
  void Exec(DynaShell &shell, int argc, char *argv[])
  {
    shell.m_bRun = false;
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdLog Class
// -----------------------------------------------------------------------------

/*!
 * \brief Log command
 */
class DynaShellCmdLog : public DynaShellCmd
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdLog() : DynaShellCmd(1, 1)
  {
    m_sCmdName      = "log";
    m_sCmdHelpBrief = "Set the logging level.";
    m_sCmdHelpArgs  = "<level>";
    m_sCmdHelpDesc  = "Set the logging level.\n"
                      "  <level>  0 | off | 1 | error | 2 | diag1 | ...";
  }

  /*!
   * \brief Set diagnotics logging level.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   */
  void Exec(DynaShell &shell, int argc, char *argv[])
  {
    int   nLevel;

    TRY( ChkArgCnt(shell, argc) );

    if( !strcmp(argv[0], "off") )
    {
      nLevel = 0;
    }
    else if( !strcmp(argv[0], "error") )
    {
      nLevel = 1;
    }
    else if( !strncmp(argv[0], "diag", 3) )
    {
      TRY( ToInt(shell, argv[0]+4, &nLevel) );
      nLevel++;
    }
    else
    {
      TRY( ToInt(shell, argv[0], &nLevel) );
    }

    LOG_SET_THRESHOLD(nLevel);
  }

  /*!
   * \brief Command tab completion generator.
   *
   * Completes <log_level>
   *
   * \param shell     Dynamixel shell.
   * \param sText     Partial text string to complete.
   * \param uTextLen  Length of text.
   * \param nState    Generator state. If FIRST, then initialize any statics.
   * \param sContext  Generator context (i.e. canonical command path).
   *
   * \return
   * If a first/next match is made, return allocated completed match.\n
   * Otherwise return NULL.
   */
  virtual char *TabCompletion(DynaShell   &shell,
                              const char  *sText,
                              size_t       uTextLen,
                              int          nState,
                              const char  *sContext)
  {
    // log level symbolic names table.
    static const char *logLevelSymTbl[] =
    {
      "off", "error", "diag1", "diag2", "diag3", "diag4", "diag5", "diag6"
    }; 

    const char  *s;

    if( (ReadLine::wc(sContext) - ReadLine::wc(m_sPubName)) != 0 )
    {
      return NULL;
    }

    //
    // New command to complete - initialize.
    //
    if( nState == ReadLine::FIRST )
    {
      m_nTabIndex = 0;
    }

    // <log_level>
    while( m_nTabIndex < arraysize(logLevelSymTbl) )
    {
      s = logLevelSymTbl[m_nTabIndex++];

      if( !strncmp(s, sText, uTextLen) )
      {
        return ReadLine::dupstr(s);
      }
    }

    // no more matches
    return NULL;
  }

protected:
  int m_nTabIndex;    ///< tab completion: keyword index

};


// -----------------------------------------------------------------------------
// DynaShellCmdWait Class
// -----------------------------------------------------------------------------

/*!
 * \brief Wait command.
 */
class DynaShellCmdWait : public DynaShellCmd
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdWait() : DynaShellCmd(1, 1)
  {
    m_sCmdName      = "wait";
    m_sCmdHelpBrief = "Delay for a specified amount of time.";
    m_sCmdHelpArgs  = "<sec>";
    m_sCmdHelpDesc  = "Block waiting for <sec> seconds.\n"
                      "  <sec>  Floating-point wait period in seconds.";
  }

  /*!
   * \brief Wait the given number of seconds.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   */
  void Exec(DynaShell &shell, int argc, char *argv[])
  {
    double          fWait;
    struct timeval  timeout;

    TRY( ChkArgCnt(shell, argc) );
    TRY( ToDouble(shell, argv[0], &fWait) );

    timeout.tv_sec   = (long)fWait;
    timeout.tv_usec  = (long)((fWait - (double)timeout.tv_sec) * 1000000.0);

    select(0, NULL, NULL, NULL, &timeout);
  }
};


// -----------------------------------------------------------------------------
// DynaShellCmdScript Class
// -----------------------------------------------------------------------------

/*!
 * \brief Script command.
 * \par Usage: script <em>file</em>
 */
class DynaShellCmdScript : public DynaShellCmd
{
public:

  /*
   * \brief Default constructor.
   */
  DynaShellCmdScript() : DynaShellCmd(1, 1)
  {
    m_sCmdName      = "script";
    m_sCmdHelpBrief = "Run a dynamixel shell script.";
    m_sCmdHelpArgs  = "<file>";
    m_sCmdHelpDesc  = "Run a file containing dynamixel shell commands.\n"
                      "  <file>   Shell text file name.";
  }

  /*!
   * \brief Execute script.
   *
   * \param shell   Dynamixel shell.
   * \param argc    Command argument count.
   * \param argv    Array of arguments.
   */
  void Exec(DynaShell &shell, int argc, char *argv[])
  {
    char   *sScriptFile;
    FILE   *fp;

    TRY( ChkArgCnt(shell, argc) );

    sScriptFile = argv[0];

    if( access(sScriptFile, F_OK|R_OK) != 0 )
    {
      shell.Error("%s: %s.", sScriptFile, strerror(errno));
    }
    else if( (fp = fopen(sScriptFile, "r")) == NULL )
    {
      shell.Error("%s: %s.", sScriptFile, strerror(errno));
    }
    else
    {
      shell.ScriptPush(sScriptFile, fp);
      shell.Ok();
    }
  }

  /*!
   * \brief Command tab completion generator.
   *
   * Completes <script_file>>
   *
   * \param shell     Dynamixel shell.
   * \param sText     Partial text string to complete.
   * \param uTextLen  Length of text.
   * \param nState    Generator state. If FIRST, then initialize any statics.
   * \param sContext  Generator context (i.e. canonical command path).
   *
   * \return
   * If a first/next match is made, return allocated completed match.\n
   * Otherwise return NULL.
   */
  virtual char *TabCompletion(DynaShell   &shell,
                              const char  *sText,
                              size_t       uTextLen,
                              int          nState,
                              const char  *sContext)
  {
    return ReadLine::FileCompletionGenerator(sText, nState);
  }
};


// -----------------------------------------------------------------------------
// Public Interface.
// -----------------------------------------------------------------------------

/*!
 * \brief Publish shell core commands to shell.
 *
 * The core set includes the common shell base commands idependent of 
 * application.
 *
 * \brief shell   Dynamixel shell.
 */
void PublishShellCoreCommands(DynaShell &shell)
{
  shell.PublishCommand(NULL, new DynaShellCmdHelp());
  shell.PublishCommand(NULL, new DynaShellCmdQuit());
  //shell.PublishCommand(NULL, new DynaShellCmdExit());
  shell.PublishCommand(NULL, new DynaShellCmdLog());
  shell.PublishCommand(NULL, new DynaShellCmdWait());
  shell.PublishCommand("run", new DynaShellCmdScript());
}
