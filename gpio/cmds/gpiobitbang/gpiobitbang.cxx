////////////////////////////////////////////////////////////////////////////////
//
// Package:   gpio
//
// Program:   gpiobitbang   
//
// File:      gpiobitbang.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-04-08 17:22:10 -0600 (Wed, 08 Apr 2015) $
 * $Rev: 3913 $
 *
 * \brief Bit bang pattern out GPIO pin.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2015-2016  RoadNarrows
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
 * \defgroup gpiobitbang gpiobitbang
 * \{
 */

#define APP_EC_OK       0   ///< success exit code
#define APP_EC_ARGS     2   ///< command-line options/arguments error exit code
#define APP_EC_EXEC     4   ///< execution exit code

static char    *Argv0;                          ///< the command
static char    *OptsMethod  = (char *)"sysfs";  ///< system file system 
static int      OptsIbd     = 1000;             ///< inter-bit delay
static int      ArgsGpio;                       ///< gpio number 
static byte_t  *ArgsPattern;                    ///< bit pattern to write
static size_t   PatByteCount;                   ///< pattern byte count
static size_t   PatBitCount;                    ///< pattern bit count


/*!
 * \brief Program information.
 */
static OptsPgmInfo_T PgmInfo =
{
  // usage_args
  "<gpio> byte [byte...]",

  // synopsis
  "Bit-bang bit pattern out GPIO pin.",

  // long_desc = 
  "The %P command writes the specified bit pattern to the GPIO at the given "
  "exported number. "
  "A bit value of 0 sets the pin low. "
  "A bit value of 1 set the pin high."
  "The byte pattern is specified as a series of byte arguments of any of the "
  "format:\n"
  "  0xH[H]     hexidecimal      examples: 0x5        0x3C     0xff\n"
  "  0bB[B...]  binary           examples: 0b00000101 0b001111 0b11111111\n"
  "  D[D...]    unsigned integer examples: 5          60       255",

  // diagnostics
  NULL
};

/*!
 * \brief Command line options information.
 */
static OptsInfo_T OptsInfo[] =
{
  // -i, --ibd
  {
    "ibd",                // long_opt
    'i',                  // short_opt
    required_argument,    // has_arg
    true,                 // has_default
    &OptsIbd,             // opt_addr
    OptsCvtArgInt,        // fn_cvt
    OptsFmtInt,           // fn_fmt
    "<usec>",             // arg_name
                          // opt desc
    "Inter-bit delay in microseconds. A 0 value means no delay."
  },

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

static int strToBits(const string &str, byte_t *p)
{
  string  s;
  int     bit;
  size_t  i;
  int     val = 0;

  if( str.substr(0, 2) == "0b" )
  {
    s = str.substr(2, str.size()-2);
    bit = 0x80;
    for(i=0; i<8 & i<s.size(); ++i)
    {
      if( s[i] == '1' )
      {
        val |= bit;
      }
      else if( s[i] != '0' )
      {
        return RC_ERROR;
      }
      bit >>= 1;
    }
  }
  else
  {
    if( sscanf(str.c_str(), "%i", &val) != 1 )
    {
      return RC_ERROR;
    }
  }

  *p = (byte_t)val;

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
  int     i;
  byte_t *p;

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
    fprintf(stderr, "%s: No bit pattern specified.\n", Argv0);
    fprintf(stderr, "Try '%s --help' for more information.\n", Argv0);
    exit(APP_EC_ARGS);
  }

  PatByteCount  = argc - 1;
  PatBitCount   = 8 * PatByteCount;
  ArgsPattern   = (byte_t *)malloc(sizeof(byte_t) * PatByteCount);

  for(i=1, p=ArgsPattern; i<argc; ++i, ++p)
  {
    if( strToBits(argv[i], p) < 0 )
    {
      fprintf(stderr, "%s: '%s': Bad bit pattern %d.\n", Argv0, argv[i], i);
      fprintf(stderr, "Try '%s --help' for more information.\n", Argv0);
      exit(APP_EC_ARGS);
    }
  }
}

/*!
 * \brief Bit bang bit pattern.
 *
 * Method: sysfs
 *
 * \return Application exit code.
 */
static int sysfsBitBang()
{
  int   fd;
  int   ec;

  if( (fd = gpioOpen(ArgsGpio)) < 0 )
  {
    return APP_EC_EXEC;
  }

  if( gpioBitBang(fd, ArgsPattern, PatBitCount, OptsIbd) < 0 )
  {
    ec = APP_EC_EXEC;
  }
  else
  {
    ec = APP_EC_OK;
  }

  gpioClose(fd);

  return ec;
}

#ifdef MMAP_GPIO
/*!
 * \brief Bit bang bit pattern.
 *
 * Method: mmap
 *
 * \return Application exit code.
 */
static int mmapBitBang()
{
  int   ec;

  if(  mmapGpioMap() < 0 )
  {
    return APP_EC_EXEC;
  }

  if( mmapGpioBitBang(ArgsGpio, ArgsPattern, PatBitCount, OptsIbd) < 0 )
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
  int   pin;
  int   ec;

  mainInit(argc, argv);


  printf("Bit banging pattern to GPIO %d.\n", ArgsGpio);
  printf("  Pattern: ");
  for(size_t i=0; i<PatByteCount; ++i)
  {
    printf("%02x ", ArgsPattern[i]);
  }
  printf("\n");
  printf("  Ibd:     %u usec\n", OptsIbd);

  if( !strcmp(OptsMethod, "sysfs") )
  {
    ec = sysfsBitBang();
  }
#ifdef MMAP_GPIO
  else if( !strcmp(OptsMethod, "mmap") )
  {
    ec = mmapBitBang();
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
