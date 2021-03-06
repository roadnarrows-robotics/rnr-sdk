////////////////////////////////////////////////////////////////////////////////
//
// Package:   gpio
//
// Program:   gpioprobe   
//
// File:      gpioprobe.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-04-09 15:03:13 -0600 (Thu, 09 Apr 2015) $
 * $Rev: 3916 $
 *
 * \brief Bit bang pattern out GPIO pin.
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
 * \defgroup gpioprobe gpioprobe
 * \{
 */

#define APP_EC_OK       0   ///< success exit code
#define APP_EC_ARGS     2   ///< command-line options/arguments error exit code
#define APP_EC_EXEC     4   ///< execution exit code

static char    *Argv0;                          ///< the command
static char    *OptsMethod  = (char *)"sysfs";  ///< system file system 
static int      ArgsGpio;                       ///< gpio number 

/*!
 * \brief Program information.
 */
static OptsPgmInfo_T PgmInfo =
{
  // usage_args
  "<gpio>",

  // synopsis
  "Safely probe GPIO settings and state.",

  // long_desc = 
  "The %P command probes the settings and state of GPIO exported number.",

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
  // name of this process
  Argv0 = basename(argv[0]);

  // parse input options
  argv = OptsGet(Argv0, &PkgInfo, &PgmInfo, OptsInfo, true, &argc, argv);

  if( argc == 0 )
  {
    fprintf(stderr, "%s: No exported GPIO/header pin number specified.\n",
        Argv0);
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
 * \brief Probe GPIO state.
 *
 * Method: sysfs
 *
 * \param [out] pInfo   Pointer to GPIO info.
 *
 * \return Application exit code.
 */
static int sysfsProbe(gpio_info_t *pInfo)
{
  int   ec;

  if( gpioProbe(ArgsGpio, pInfo) < 0 )
  {
    ec = APP_EC_EXEC;
  }
  else
  {
    ec = APP_EC_OK;
  }

  return ec;
}

static int sysfsProbeAndPrint()
{
  gpio_info_t     info;
  const char     *s;
  int             ec;

  ec = sysfsProbe(&info);

  if( ec != APP_EC_OK )
  {
    fprintf(stderr, "Error: sysfs GPIO Probe failed.\n");
    return ec;
  }

  printf("GPIO Info\n");
  printf("  exported gpio number: %d\n", info.gpio);
  if( info.pin > 0 )
  {
    printf("  external pin number:  %d\n", info.pin);
  }
  else
  {
    printf("  external pin number:  N/A\n");
  }

  switch( info.dir )
  {
    case GPIO_DIR_IN:
      s = GPIO_DIR_IN_STR;
      break;
    case GPIO_DIR_OUT:
      s = GPIO_DIR_OUT_STR;
      break;
    default:
      s = "?";
      break;
  }
  printf("  direction:            %s\n", s);

  switch( info.edge )
  {
    case GPIO_EDGE_NONE:
      s = GPIO_EDGE_NONE_STR;
      break;
    case GPIO_EDGE_RISING:
      s = GPIO_EDGE_RISING_STR;
      break;
    case GPIO_EDGE_FALLING:
      s = GPIO_EDGE_FALLING_STR;
      break;
    case GPIO_EDGE_BOTH:
      s = GPIO_EDGE_BOTH_STR;
      break;
    default:
      s = "?";
      break;
  }
  printf("  edge:                 %s\n", s);

  printf("  pull:                 N/A\n");
  printf("  value:                %d\n", info.value);
}

#ifdef MMAP_GPIO
/*!
 * \brief Probe GPIO state.
 *
 * Method: mmap
 *
 * \param [out] pInfo   Pointer to GPIO info.
 *
 * \return Application exit code.
 */
static int mmapProbe(mmap_gpio_info_t *pInfo)
{
  int   ec;

  if( mmapGpioMap() < 0 )
  {
    return APP_EC_EXEC;
  }

  if( mmapGpioProbe(ArgsGpio, pInfo) < 0 )
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

static int mmapProbeAndPrint()
{
  mmap_gpio_info_t  info;
  const char       *s;
  int               ec;

  ec = mmapProbe(&info);

  if( ec != APP_EC_OK )
  {
    fprintf(stderr, "Error: mmapped GPIO Probe failed.\n");
    return ec;
  }

  printf("GPIO Info\n");
  printf("  exported gpio number: %d\n", info.gpio);
  if( info.pin > 0 )
  {
    printf("  external pin number:  %d\n", info.pin);
  }
  else
  {
    printf("  external pin number:  N/A\n");
  }

  printf("  mmap base address:    0x%lx\n", info.base);
  printf("  mmap channel offset:  0x%x\n", info.channel);
  printf("  mmap bit:             %d\n", info.bit);

  switch( info.dir )
  {
    case GPIO_DIR_IN:
      s = GPIO_DIR_IN_STR;
      break;
    case GPIO_DIR_OUT:
      s = GPIO_DIR_OUT_STR;
      break;
    default:
      s = "?";
      break;
  }
  printf("  direction:            %s\n", s);

  switch( info.edge )
  {
    case GPIO_EDGE_NONE:
      s = GPIO_EDGE_NONE_STR;
      break;
    case GPIO_EDGE_RISING:
      s = GPIO_EDGE_RISING_STR;
      break;
    case GPIO_EDGE_FALLING:
      s = GPIO_EDGE_FALLING_STR;
      break;
    case GPIO_EDGE_BOTH:
      s = GPIO_EDGE_BOTH_STR;
      break;
    default:
      s = "?";
      break;
  }
  printf("  edge:                 %s\n", s);

  switch( info.pull )
  {
    case GPIO_PULL_UP:
      s = GPIO_PULL_UP_STR;
      break;
    case GPIO_PULL_DN:
      s = GPIO_PULL_DN_STR;
      break;
    default:
      s = GPIO_PULL_DS_STR;
      break;
  }
  printf("  pull:                 %s\n", s);

  printf("  value:                %d\n", info.value);

  return APP_EC_OK;
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
    ec = sysfsProbeAndPrint();
  }
#ifdef MMAP_GPIO
  else if( !strcmp(OptsMethod, "mmap") )
  {
    ec = mmapProbeAndPrint();
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
