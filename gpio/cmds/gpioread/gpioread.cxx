////////////////////////////////////////////////////////////////////////////////
//
// Package:   gpio
//
// Program:   gpioread   
//
// File:      gpioread.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-04-09 15:03:13 -0600 (Thu, 09 Apr 2015) $
 * $Rev: 3916 $
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
#include <unistd.h>
#include <time.h>

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
 * \defgroup gpioread gpioread
 * \{
 */

#define APP_EC_OK       0   ///< success exit code
#define APP_EC_ARGS     2   ///< command-line options/arguments error exit code
#define APP_EC_EXEC     4   ///< execution exit code

static char    *Argv0;                  ///< the command
static char    *OptsMethod  = (char *)"sysfs";  ///< system file system 
static int      OptsCount     = 1;      ///< number of reads
static double   OptsInterval  = 1.0;    ///< read interval in seconds
static bool_t   OptsVerbose   = false;  ///< permissions

static int    ArgsGpio;                 ///< gpio number 

/*!
 * \brief Program information.
 */
static OptsPgmInfo_T PgmInfo =
{
  // usage_args
  "<gpio>",

  // synopsis
  "Read GPIO pin value.",

  // long_desc = 
  "The %P command reads the current GPIO pin value (0 or 1) for the specified "
  "GPIO pin number.",

  // diagnostics
  NULL
};

/*!
 * \brief Command line options information.
 */
static OptsInfo_T OptsInfo[] =
{
  // -c, --count
  {
    "count",              // long_opt
    'c',                  // short_opt
    required_argument,    // has_arg
    true,                 // has_default
    &OptsCount,           // opt_addr
    OptsCvtArgInt,        // fn_cvt
    OptsFmtInt,           // fn_fmt
    "<count>",            // arg_name
                          // opt desc
    "Number of times to read the GPIO pin. A zero value means forever."
  },

  // -i, --interval
  {
    "interval",           // long_opt
    'i',                  // short_opt
    required_argument,    // has_arg
    true,                 // has_default
    &OptsInterval,        // opt_addr
    OptsCvtArgFloat,      // fn_cvt
    OptsFmtFloat,         // fn_fmt
    "<time>",             // arg_name
                          // opt desc
    "Wait interval seconds between reads specified as a floating-point number."
  },

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

  // -v, --verbose
  {
    "verbose",            // long_opt
    'v',                  // short_opt
    no_argument,          // has_arg
    true,                 // has_default
    &OptsVerbose,         // opt_addr
    OptsCvtArgBool,       // fn_cvt
    OptsFmtBool,          // fn_fmt
    NULL,                 // arg_name
                          // opt desc
    "Print read value(s) in verbose mode."
  },

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

static void printValue(int val)
{
  struct timespec tsNow;

  if( OptsVerbose )
  {
    clock_gettime(CLOCK_REALTIME, &tsNow);
    printf("[%ld.%09ld] gpio %d = ", tsNow.tv_sec, tsNow.tv_nsec, ArgsGpio);
  }

  printf("%d\n", val);
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

  if( OptsCount < 0 )
  {
    OptsCount = -OptsCount;
  }

  if( OptsInterval < 0.0 )
  {
    OptsInterval = -OptsInterval;
  }

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
 * \brief Read GPIO value(s).
 *
 * Method: sysfs
 *
 * \return Application exit code.
 */
static int sysfsRead()
{
  int         fd;
  bool        bForever;
  int         nReads;
  int         val;
  uint_t      usec;

  if( (fd = gpioOpen(ArgsGpio)) < 0 )
  {
    return APP_EC_EXEC;
  }

  bForever  = OptsCount == 0? true: false;
  nReads    = 0;
  usec      = (uint_t)(OptsInterval * 1000000);
  
  while( bForever || (nReads < OptsCount) )
  {
    if( (nReads > 0) && (usec > 0) )
    {
      usleep(usec);
    }

    if( (val = gpioQuickRead(fd)) < 0 )
    {
      gpioClose(fd);
      return APP_EC_EXEC;
    }

    printValue(val);

    ++nReads;
  }

  gpioClose(fd);

  return APP_EC_OK;
}

/*!
 * \brief Read GPIO value(s).
 *
 * Method: mmap
 *
 * \return Application exit code.
 */
static int mmapRead()
{
  bool        bForever;
  int         nReads;
  int         val;
  uint_t      usec;

  if(  mmapGpioMap() < 0 )
  {
    return APP_EC_EXEC;
  }

  bForever  = OptsCount == 0? true: false;
  nReads    = 0;
  usec      = (uint_t)(OptsInterval * 1000000);
  
  while( bForever || (nReads < OptsCount) )
  {
    if( (nReads > 0) && (usec > 0) )
    {
      usleep(usec);
    }

    if( (val = mmapGpioRead(ArgsGpio)) < 0 )
    {
      mmapGpioUnmap();
      return APP_EC_EXEC;
    }

    printValue(val);

    ++nReads;
  }

  mmapGpioUnmap();

  return APP_EC_OK;
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
  int   ec;

  mainInit(argc, argv);

  if( !strcmp(OptsMethod, "sysfs") )
  {
    ec = sysfsRead();
  }
  else if( !strcmp(OptsMethod, "mmap") )
  {
    ec = mmapRead();
  }

  return ec;
}

/*!
 * \}
 */
