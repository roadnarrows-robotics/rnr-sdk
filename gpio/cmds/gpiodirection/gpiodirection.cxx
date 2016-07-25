////////////////////////////////////////////////////////////////////////////////
//
// Package:   gpio
//
// Program:   gpiodirection   
//
// File:      gpiodirection.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-04-08 17:22:10 -0600 (Wed, 08 Apr 2015) $
 * $Rev: 3913 $
 *
 * \brief Set GPIO pin direction.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2015-2016.  RoadNarrows
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
 * \defgroup gpiodirection gpiodirection
 * \{
 */

#define APP_EC_OK       0   ///< success exit code
#define APP_EC_ARGS     2   ///< command-line options/arguments error exit code
#define APP_EC_EXEC     4   ///< execution exit code

static char  *Argv0;                    ///< the command
static char  *OptsMethod  = (char *)"sysfs";  ///< system file system 
static int    ArgsGpio;                 ///< gpio number 
static int    ArgsDirection;            ///< gpio direction 

/*!
 * \brief Program information.
 */
static OptsPgmInfo_T PgmInfo =
{
  // usage_args
  "<gpio> {in | out}",

  // synopsis
  "Set GPIO pin direction.",

  // long_desc = 
  "The %P command sets the direction for the GPIO associated with the given "
  "<gpio> exported number."
  "An input (in) direction allows for the value of the GPIO to be read. "
  "An output (out) direction allows for writing of values to the GPIO.",

  // diagnostics
  NULL
};

/*!
 * \brief Command line options information.
 */
static OptsInfo_T OptsInfo[] =
{
#ifdef MMAP_GPIO
  // -m, --method
  {
    "method",             // long_opt
    'm',                  // short_opt
    required_argument,    // has_arg
    true,                 // has_default
    &OptsMethod,          // opt_addr
    OptsCvtArgStr,        // fn_cvt
    OptsFmtStr,           // fn_fmt
    "<method>",           // arg_name
                          // opt desc
    "GPIO access method. One of: sysfs mmap."
  },
#endif // MMAP_GPIO

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

  if( argc == 1 )
  {
    fprintf(stderr, "%s: No GPIO direction specified.\n", Argv0);
    fprintf(stderr, "Try '%s --help' for more information.\n", Argv0);
    exit(APP_EC_ARGS);
  }
  else if( !strcmp(argv[1], GPIO_DIR_IN_STR) )
  {
    ArgsDirection = GPIO_DIR_IN;
  }
  else if( !strcmp(argv[1], GPIO_DIR_OUT_STR) )
  {
    ArgsDirection = GPIO_DIR_OUT;
  }
  else
  {
    fprintf(stderr, "%s: '%s': Bad direction.\n", Argv0, argv[1]);
    fprintf(stderr, "Try '%s --help' for more information.\n", Argv0);
    exit(APP_EC_ARGS);
  }
}

/*!
 * \brief Set GPIO directiion.
 *
 * Method: sysfs
 *
 * \return Application exit code.
 */
static int sysfsSetDirection()
{
  int   ec;

  if( gpioSetDirection(ArgsGpio, ArgsDirection) < 0 )
  {
    ec = APP_EC_EXEC;
  }
  else
  {
    ec = APP_EC_EXEC;
  }

  return ec;
}

#ifdef MMAP_GPIO
/*!
 * \brief Set GPIO directiion.
 *
 * Method: mmap
 *
 * \return Application exit code.
 */
static int mmapSetDirection()
{
  int   ec;

  if(  mmapGpioMap() < 0 )
  {
    return APP_EC_EXEC;
  }

  if( mmapGpioSetDirection(ArgsGpio, ArgsDirection) < 0 )
  {
    ec = APP_EC_EXEC;
  }
  else
  {
    ec = APP_EC_OK;
  }

  mmapGpioUnmap();

  return ec;
}
#endif // MMAP_GPIO

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
  int   ec;

  mainInit(argc, argv);

  if( !strcmp(OptsMethod, "sysfs") )
  {
    ec = sysfsSetDirection();
  }
#ifdef MMAP_GPIO
  else if( !strcmp(OptsMethod, "mmap") )
  {
    ec = mmapSetDirection();
  }
#endif // MMAP_GPIO
  else
  {
    fprintf(stderr,"%s: Unknown GPIO access method.", OptsMethod);
    ec = APP_EC_ARGS;
  }

  return ec;
}

/*!
 * \}
 */
