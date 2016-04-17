////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Program:   dynashell
//
// File:      dynashell_main.c
//
/*! \file
 *
 * $LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
 * $Rev: 3845 $
 *
 * \brief RoadNarrows simple Dynamixel shell using the RoadNarrows Dynamixel
 * library.
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

#include <sys/select.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <libgen.h>
#include <ctype.h>
#include <errno.h>

#include "rnr/rnrconfig.h"
#include "rnr/opts.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"

#include "version.h"

#include "dynashell.h"
#include "dynashell_cmd.h"
#include "dynashell_util.h"


// 
// The command with option and argument values.
//
static char  *Argv0;                            ///< the command
static char  *OptsSerDevUri = NULL;             ///< the serial device URI
static int    OptsBaudRate  = 1000000;          ///< serial baudrate
static char  *OptsScript    = NULL;             ///< script file
static bool   OptsXTrace    = false;            ///< trace script
static bool   OptsSilent    = false;            ///< silence output


/*!
 * \brief Program information.
 */
static OptsPgmInfo_T AppPgmInfo =
{
  // usage_args
  NULL,

  // synopsis
  "Dynamixel simple shell.",

  // long_desc = 
  "The %P provides a simple command-line shell to the Dynamixel interface "
  "provided by the library libDynamixel.",

  // diagnostics
  NULL
};

/*!
 * \brief Command line options information.
 */
static OptsInfo_T AppOptsInfo[] =
{
  // -u, --uri <device_uri>
  {
    "uri",                      // long_opt
    'u',                        // short_opt
    required_argument,          // has_arg
    false,                      // has_default
    &OptsSerDevUri,             // opt_addr
    OptsCvtArgStr,              // fn_cvt
    OptsFmtStr,                 // fn_fmt
    "<device_uri>",             // arg_name
    "(Proxied) serial device interface. Syntax:\n"  // opt desc
    "  [botsense://[hostname][:port]]/device",
  },

  // -b, --baudrate <baud>
  {
    "baudrate",                 // long_opt
    'b',                        // short_opt
    required_argument,          // has_arg
    true,                       // has_default
    &OptsBaudRate,              // opt_addr
    OptsCvtArgInt,              // fn_cvt
    OptsFmtInt,                 // fn_fmt
    "<baud>",                   // arg_name
    "Serial interface baudrate.", // opt desc
  },

  // -s, --script <file>
  {
    "script",                   // long_opt
    's',                        // short_opt
    required_argument,          // has_arg
    false,                      // has_default
    &OptsScript,                // opt_addr
    OptsCvtArgStr,              // fn_cvt
    OptsFmtStr,                 // fn_fmt
    "<file>",                   // arg_name
    "Shell script filename.",   // opt desc
  },

  // -x, --xtrace
  {
    "xtrace",                   // long_opt
    'x',                        // short_opt
    no_argument,                // has_arg
    true,                       // has_default
    &OptsXTrace,                // opt_addr
    OptsCvtArgBool,             // fn_cvt
    OptsFmtBool,                // fn_fmt
    NULL,                       // arg_name
    "Trace shell scripts.",     // opt desc
  },

  // -s, --silent
  {
    "silent",                   // long_opt
    OPTS_NO_SHORT,              // short_opt
    no_argument,                // has_arg
    true,                       // has_default
    &OptsSilent,                // opt_addr
    OptsCvtArgBool,             // fn_cvt
    OptsFmtBool,                // fn_fmt
    NULL,                       // arg_name
    "Silence non-error responses.", // opt desc
  },

  {NULL, }
};

//
// Global Data
//

/*!
 * \brief Main command-line argument initialization.
 *
 * \param shell   Dynamixel shell.
 * \param argc    Command-line argument count.
 * \param argv    Command-line argument list.
 *
 * \par Exits:
 * Program terminates on conversion error.
 */
static void MainInitArgs(DynaShell &shell, int argc, char *argv[])
{
  FILE  *fp;

  // name of this process
  Argv0 = basename(argv[0]);

  // parse input options
  argv = OptsGet(Argv0, &PkgInfo, &AppPgmInfo, AppOptsInfo, true,
                 &argc, argv);

  if( OptsSerDevUri != NULL )
  {
    shell.m_pDynaComm = DynaComm::New(OptsSerDevUri, OptsBaudRate);

    if( shell.m_pDynaComm == NULL )
    {
      OptsInvalid(Argv0, "Failed to create interface on %s@%d.",
          OptsSerDevUri, OptsBaudRate);
    }

    shell.m_pDynaChain = new DynaChain(*shell.m_pDynaComm);

    // create background thread and register chain - leave in ready state
    if( shell.m_pDynaBgThread == NULL )
    {
      shell.m_pDynaBgThread = new DynaBgThread();
      shell.m_pDynaBgThread->RegisterChainAgent(shell.m_pDynaChain);
    }
  }

  if( OptsScript != NULL )
  { 
    if( access(OptsScript, F_OK|R_OK) != 0 )
    {
      OptsInvalid(Argv0, "%s: %s.", OptsScript, strerror(errno));
    }
    else if( (fp = fopen(OptsScript, "r")) == NULL )
    {
      OptsInvalid(Argv0, "%s: %s.", OptsScript, strerror(errno));
    }
    else
    {
      shell.ScriptPush(OptsScript, fp);
      shell.m_bIsInteractive = false;
    }
  }

  shell.m_bXTrace = OptsXTrace;
  shell.m_bSilent = OptsSilent;
}

/*!
 * \brief Main initialization.
 *
 * \param shell   Dynamixel shell.
 * \param argc    Command-line argument count.
 * \param argv    Command-line argument list.
 *
 * \par Exits:
 * Program terminates on conversion error.
 */
static void MainInit(DynaShell &shell, int argc, char *argv[])
{
  MainInitArgs(shell, argc, argv);

  shell.PublishMap("bg", "Commands to control the vServo background thread.");
  shell.PublishMap("clear", "Commands to clear run-time state.");
  shell.PublishMap("dump", "Dump memory contents.");
  shell.PublishMap("get", "Commands to get servo class object parameters.");
  shell.PublishMap("load", "Commands to load files.");
  shell.PublishMap("read", "Commands to read from the physical servos.");
  shell.PublishMap("run", "Commands to run scripts.");
  shell.PublishMap("save", "Commands to save files.");
  shell.PublishMap("set", "Commands to set servo class object parameters.");
  shell.PublishMap("write", "Commands to write to the physical servos.");

  PublishShellCoreCommands(shell);
  PublishShellInterfaceCommands(shell);
  PublishShellServoCommands(shell);
  //PublishShellTrainCommands(shell);
}

/*!
 * \brief Main clean-up on exiting.
 *
 * \param shell   Dynamixel shell.
 */
static void MainFini(DynaShell &shell)
{
}

/*!
 * \brief Example main.
 *
 * \param argc  Command-line argument count.
 * \param argv  Command-line argument list.
 *
 * \par Exit Status:
 * Program exits with 0 success, \h_gt 0 on failure.
 */
int main(int argc, char *argv[])
{
  DynaShell   shell;
  int         rc;

  MainInit(shell, argc, argv);

  rc = shell.Run();

  MainFini(shell);

  return rc == DYNA_OK? 0: 8;
}
