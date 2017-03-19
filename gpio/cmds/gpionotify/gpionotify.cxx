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
 *   \h_copy 2016-2017. RoadNarrows LLC.\n
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
#include <fcntl.h> 
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

static char    *Argv0;                ///< the command
static bool_t   OptsMonitor = false;  ///< do [not] keep listening for events
static double   OptsTimeout = 0.0;    ///< event timeout (none)

static int    ArgsGpioNum;        ///< gpio number 

/*!
 * \brief Program information.
 */
static OptsPgmInfo_T PgmInfo =
{
  // usage_args
  "<gpio>",

  // synopsis
  "Block wait for GPIO value to change.",

  // long_desc = 
  "The %P command block waits for the value of the inoput GPIO associated with "
  "the <gpio> exported number to change. "
  "When the GPIO value changes or a timeout "
  "occurs, the value of \"0\" or \"1\" is printed to stdout. On error, the "
  "value \"-1\" is printed to stdout, and the program terminates.\n"
  "Note:\n"
  "  The exported GPIO interface must be configured to trigger on an "
  "edge transition.\n"
  "Examples:\n"
  "  gpionotify 19\n"
  "  gpionotify --timeout=1.5 --monitor 172"
  "",

  // diagnostics
  NULL
};

/*!
 * \brief Command line options information.
 */
static OptsInfo_T OptsInfo[] =
{
  // -m, --monitor
  {
    "monitor",      // long_opt
    'm',            // short_opt
    no_argument,    // has_arg
    true,           // has_default
    &OptsMonitor,   // opt_addr
    OptsCvtArgBool, // fn_cvt
    OptsFmtBool,    // fn_fmt
    NULL,           // arg_name
                    // opt desc
    "Keep listening for events. Default will terminate after one trigger."
  },

  // -t, --timeout
  {
    "timeout",          // long_opt
    't',                // short_opt
    required_argument,  // has_arg
    true,               // has_default
    &OptsTimeout,       // opt_addr
    OptsCvtArgFloat,    // fn_cvt
    OptsFmtFloat,       // fn_fmt
    "<seconds>",        // arg_name
                        // opt desc
    "Set wait timeout in seconds. A value of 0.0 means no timeout."
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

  for(int i = 2; i<argc; ++i)
    printf(".%s.\n", argv[i]);

}

int main(int argc, char *argv[])
{
  int   fd;
  int   rc;

  // parse command line
  mainInit(argc, argv);

  // open sys/class/gpio exported value 
  if( (fd = gpioOpen(ArgsGpioNum)) < 0 )
  {
    return APP_EC_EXEC;
  }

  // read once, to purge existing gpio triggers
  gpioQuickRead(fd);

  // now monitor events
  do
  {
    rc = gpioNotify(fd, OptsTimeout);

    fprintf(stdout, "%d\n", rc);
    fflush(stdout);

  } while( OptsMonitor && (rc >= 0) );

  return rc < -1? APP_EC_EXEC: APP_EC_OK;
}
