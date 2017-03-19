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
 * \copyright
 *   \h_copy 2015-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
/*
 * @EulaBegin@
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>

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

#define NO_ARG  -1                ///< no argument

static int    ArgsGpioNum;        ///< gpio number 
static int    ArgsGpioDir;        ///< gpio direction 
static int    ArgsGpioEdge;       ///< gpio edge trigger type
static mode_t Permissions;        ///< gpio permissions

/*!
 * \brief Program information.
 */
static OptsPgmInfo_T PgmInfo =
{
  // usage_args
  "<gpio> [{in|out} [{none|rising|falling|both}]]",

  // synopsis
  "Create GPIO exported interface.",

  // long_desc = 
  "The %P command creates a GPIO exported interface for the specified GPIO "
  "number. Optionally, the direction and edge of the GPIO can also be "
  "specified.\n\n"
  "Arguments:\n"
  "  <gpio>         Exported GPIO number.\n"
  "  <direction>    GPIO direction. One of: in out.\n"
  "                   Default: System defined.\n"
  "  <edge>         Input GPIO trigger. One of: none rising falling both.\n"
  "                   Default: System defined. N/A for output GPIO."
  "\n\n"
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
  {
    "mode",               // long_opt
    'm',                  // short_opt
    required_argument,    // has_arg
    false,                // has_default
    &OptsMode,            // opt_addr
    OptsCvtArgStr,        // fn_cvt
    OptsFmtStr,           // fn_fmt
    "<MODE>",             // arg_name
                          // opt desc
    "Change GPIO exported interface permissions. "
    "Format: 0[0-7]+ for user,group,other."
  },

  {NULL, }
};

/*!
 * \brief Exit program on bad command-line values.
 */
static void badCmdExit()
{
  fprintf(stderr, "Try '%s --help' for more information.\n", Argv0);
  exit(APP_EC_ARGS);
}

/*!
 * \brief Convert string to integer.
 */
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
 * \brief Convert string to file permissions.
 */
static int strToMode(const string &str, mode_t &mode)
{
  int   val;

  mode = 0;

  if( strToInt(str, val) < 0 )
  {
    return RC_ERROR;
  }

  // negative
  if( val < 0 )
  {
    return RC_ERROR;
  }
  // [0-7]
  else if( val <= 7 )
  {
    mode = (mode_t)((val << 6) & S_IRWXU);
  }
  // [0-7][0-7]
  else if( val <= 077 )
  {
    mode = (mode_t)((val << 3) & (S_IRWXU|S_IRWXG));
  }
  // [0-7][0-7][0-7]
  else if( val <= 0777 )
  {
    mode = (mode_t)(val & (S_IRWXU|S_IRWXG|S_IRWXO));
  }
  // out-of-range error
  else
  {
    return RC_ERROR;
  }

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
 *
 * \return
 * Exit code.
 */
static void mainInit(int argc, char *argv[])
{
  // name of this process
  Argv0 = basename(argv[0]);

  // parse input options
  argv = OptsGet(Argv0, &PkgInfo, &PgmInfo, OptsInfo, true, &argc, argv);

  if( OptsMode != NULL )
  {
    if( strToMode(OptsMode, Permissions) < 0 )
    {
      fprintf(stderr, "%s: '%s': Bad permissions mode.\n", Argv0, OptsMode);
      badCmdExit();
    }
  }

  if( argc == 0 )
  {
    fprintf(stderr, "%s: No GPIO pin number <gpio> specified.\n", Argv0);
    badCmdExit();
  }

  else if( strToInt(argv[0], ArgsGpioNum) < 0 )
  {
    fprintf(stderr, "%s: '%s': Bad GPIO number.\n", Argv0, argv[0]);
    badCmdExit();
  }

  if( argc >= 2 )
  {
    if( !strcasecmp(argv[1], GPIO_DIR_IN_STR) )
    {
      ArgsGpioDir = GPIO_DIR_IN;
    }
    else if( !strcasecmp(argv[1], GPIO_DIR_OUT_STR) )
    {
      ArgsGpioDir = GPIO_DIR_OUT;
    }
    else
    {
      fprintf(stderr, "%s: '%s': Bad GPIO direction.\n", Argv0, argv[1]);
      badCmdExit();
    }
  }
  else
  {
    ArgsGpioDir = NO_ARG;
  }

  if( argc >= 3 )
  {
    if( !strcasecmp(argv[2], GPIO_EDGE_NONE_STR) )
    {
      ArgsGpioEdge = GPIO_EDGE_NONE;
    }
    else if( !strcasecmp(argv[2], GPIO_EDGE_RISING_STR) )
    {
      ArgsGpioEdge = GPIO_EDGE_RISING;
    }
    else if( !strcasecmp(argv[2], GPIO_EDGE_FALLING_STR) )
    {
      ArgsGpioEdge = GPIO_EDGE_FALLING;
    }
    else if( !strcasecmp(argv[2], GPIO_EDGE_BOTH_STR) )
    {
      ArgsGpioEdge = GPIO_EDGE_BOTH;
    }
    else
    {
      fprintf(stderr, "%s: '%s': Bad GPIO edge.\n", Argv0, argv[2]);
      badCmdExit();
    }
  }
  else
  {
    ArgsGpioEdge = NO_ARG;
  }

  if( argc >= 4 )
  {
    fprintf(stderr, "%s: '%s...': What is this?.\n", Argv0, argv[3]);
    badCmdExit();
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
  mainInit(argc, argv);

  if( gpioExport(ArgsGpioNum) < 0 )
  {
    return APP_EC_EXEC;
  }

  if( ArgsGpioDir != NO_ARG )
  {
    if( gpioSetDirection(ArgsGpioNum, ArgsGpioDir) < 0 )
    {
      return APP_EC_EXEC;
    }
  }

  if( ArgsGpioEdge != NO_ARG )
  {
    if( gpioSetEdge(ArgsGpioNum, ArgsGpioEdge) < 0 )
    {
      return APP_EC_EXEC;
    }
  }

  if( OptsMode != NULL )
  {
    char  gpioPath[MAX_PATH]; 
    char  path[MAX_PATH]; 
  
    gpioMakeDirname(ArgsGpioNum, gpioPath, sizeof(gpioPath));

    sprintf(path, "%s/%s", gpioPath, "value");

    if( chmod(path, Permissions) < 0 )
    {
      LOGSYSERROR("chmod(%s, %o)", path, Permissions);
      return APP_EC_EXEC;
    }
  }

  return APP_EC_OK;
}

/*!
 * \}
 */
