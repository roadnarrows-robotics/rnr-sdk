////////////////////////////////////////////////////////////////////////////////
//
// Package:   gpio
//
// Program:   gpioexport   
//
// File:      gpioexport.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-04-08 17:22:10 -0600 (Wed, 08 Apr 2015) $
 * $Rev: 3913 $
 *
 * \brief Create GPIO exported interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2015  RoadNarrows
 * (http://www.RoadNarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/opts.h"
#include "rnr/pkg.h"

#include "rnr/gpio.h"

#include "version.h"

using namespace std;

/*!
 * \ingroup cmds
 * \defgroup gpioexport gpioexport
 * \{
 */

#define APP_EC_OK       0   ///< success exit code
#define APP_EC_ARGS     2   ///< command-line options/arguments error exit code
#define APP_EC_EXEC     4   ///< execution exit code

static char  *Argv0;              ///< the command
static char  *OptsMode  = NULL;   ///< permissions

static int    ArgsGpio;           ///< gpio number 

/*!
 * \brief Program information.
 */
static OptsPgmInfo_T PgmInfo =
{
  // usage_args
  "<gpio>",

  // synopsis
  "Create GPIO exported interface.",

  // long_desc = 
  "The %P command creates a GPIO exported interface for the specified GPIO "
  "pin number.\n\n"
  "NOTE: This command requires root privileges.",

  // diagnostics
  NULL
};

/*!
 * \brief Command line options information.
 */
static OptsInfo_T OptsInfo[] =
{
  // -m, --mode
  //{
  //  "mode",               // long_opt
  //  'm',                  // short_opt
  //  required_argument,    // has_arg
  //  false,                // has_default
  //  &OptsMode,            // opt_addr
  //  OptsCvtArgStr,        // fn_cvt
  //  OptsFmtStr,           // fn_fmt
  //  "<MODE>",             // arg_name
  //                        // opt desc
  //  "Change GPIO exported interface permissions."
  //},

  {NULL, }
};

static int strToInt(const string &str, int &val)
{
  long long int val1; // must use 64-bit for arm 32-bit compilers

  if( sscanf(str.c_str(), "%lli", &val1) != 1 )
  {
    return RC_ERROR;
  }

  val = (int)val1;

  return OK;
}

/*!
 * \brief Main initialization.
 *
 * \param argc    Command-line argument count.
 * \param argv    Command-line argument list.
 *
 * \par Exits:
 * Program terminates on conversion error.
 */
static void mainInit(int argc, char *argv[])
{
  // name of this process
  Argv0 = basename(argv[0]);

  // parse input options
  argv = OptsGet(Argv0, &PkgInfo, &PgmInfo, OptsInfo, true, &argc, argv);

  if( argc == 0 )
  {
    fprintf(stderr, "%s: No GPIO pin number <gpio> specified.\n", Argv0);
    fprintf(stderr, "Try '%s --help' for more information.\n", Argv0);
    exit(APP_EC_ARGS);
  }

  else if( strToInt(argv[0], ArgsGpio) < 0 )
  {
    fprintf(stderr, "%s: '%s': Bad GPIO number.\n", Argv0, argv[0]);
    fprintf(stderr, "Try '%s --help' for more information.\n", Argv0);
    exit(APP_EC_ARGS);
  }
}

/*!
 * \brief Main.
 *
 * \param argc    Command-line argument count.
 * \param argv    Command-line argument list.
 *
 * \return Returns 0 on succes, non-zero on failure.
 */
int main(int argc, char* argv[])
{
  char  cmd[MAX_PATH]; 
  char  dir[MAX_PATH]; 
  
  mainInit(argc, argv);

  if( gpioExport(ArgsGpio) < 0 )
  {
    return APP_EC_EXEC;
  }

  gpioMakeDirname(ArgsGpio, dir, sizeof(dir));

  // RDK gpiox, value, direction
  if( OptsMode != NULL )
  {
    snprintf(cmd, sizeof(cmd), "/bin/chmod --recursive %s %s", OptsMode, dir);
    cmd[sizeof(cmd)-1] = 0;
    if( system(cmd) < 0 )
    {
      LOGERROR("%s: Command failed.", cmd);
      return APP_EC_EXEC;
    }
  }

  return APP_EC_OK;
}

/*!
 * \}
 */
