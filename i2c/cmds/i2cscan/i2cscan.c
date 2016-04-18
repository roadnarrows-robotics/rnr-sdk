////////////////////////////////////////////////////////////////////////////////
//
// Package:   RCB3
//
// File:      i2cscan.c
//
/*! \file
 *
 * $LastChangedDate: 2009-09-09 09:44:12 -0600 (Wed, 09 Sep 2009) $
 * $Rev: 130 $
 *
 * \brief I2C Bus Scan.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2007-2009.  RoadNarrows LLC.
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 */
// Permission is hereby granted, without written agreement and without
// license or royalty fees, to use, copy, modify, and distribute this
// software and its documentation for any purpose, provided that
// (1) The above copyright notice and the following two paragraphs
// appear in all copies of the source code and (2) redistributions
// including binaries reproduces these notices in the supporting
// documentation.   Substantial modifications to this software may be
// copyrighted by their authors and need not follow the licensing terms
// described here, provided that the new terms are clearly indicated in
// all files where they apply.
//
// IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
// OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
// PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
// EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.
//
// THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
// "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
// PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
//
////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>
#include <libgen.h>
#include <unistd.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/opts.h"

#include "rnr/i2c-dev.h"
#include "rnr/i2c.h"

#include "version.h"

// The Command
static char *Argv0;                         ///< command name

//
// Command Option Values
//
static int    OptVerbose  = 0;              ///< verbose option
static char  *OptDevName  = "/dev/i2c/0";   ///< i2c bus device option
static int    OptDevFd    = -1;             ///< opened i2c bus device fd option

/*!
 *  \brief Program Information
 */
static OptsPgmInfo_T I2CScanPgmInfo =
{
  NULL,

  .synopsis = "I2C Bus Scan",

  .long_desc = 
    "The %P command scans the specified I2C Bus for all attached devices. "
    "The I2C addresses of all found devices are written to stdout in ASCII hex "
    "format.",

  .diagnostics = 
    "Exit status is 0 if devices are found, 1 otherwise. Exit tatus is >= 2 if "
    "error(s) are encountered."
};

/*!
 * \brief Command Line Options Information
 */
static OptsInfo_T I2CScanOptsInfo[] =
{
  // I2C device name
  {
    .long_opt   = "device", 
    .short_opt  = 'd', 
    .has_arg    = required_argument,
    .has_default= true,
    .opt_addr   = &OptDevName,
    .fn_cvt     = OptsCvtArgStr,
    .fn_fmt     = OptsFmtStr,
    .arg_name   = "<device>",
    .opt_desc   = "I2C device."
  },

  // opend I2C file descriptor
  {
    .long_opt   = "fd",
    .short_opt  = OPTS_NO_SHORT,
    .has_arg    = required_argument,
    .opt_addr   = &OptDevFd,
    .fn_cvt     = OptsCvtArgInt,
    .fn_fmt     = OptsFmtInt,
    .arg_name   = "<n>",
    .opt_desc   = "Opened I2C device file descriptor."
  },

  // verbose printing
  {
    .long_opt   = "verbose",
    .short_opt  = 'v',
    .has_arg    = no_argument,
    .has_default= true,
    .opt_addr   = &OptVerbose,
    .fn_fmt     = OptsFmtBool,
    .opt_desc   = "Set print verbosity."
  },

  {NULL, }
};

/*!
 * \brief Found scanned device callback.
 *
 * \param pI2C    Pointer to \h_i2c handle.
 * \param addr    Slave device address.
 * \param context User provided context.
 *
 * \return Returns 1.
 */
static int scanCallback(i2c_t *pI2C, i2c_addr_t addr, void *context)
{
  printf("0x%02x ", addr);
  return 1;
}

/*!
 * \brief Execute \h_i2c slave device scan.
 *
 * \param pI2C    Pointer to \h_i2c handle.
 *
 * \return Returns number of devices found \h_ge 0 on success, -1 on failure.
 */
static int execScan(i2c_t *pI2C)
{
  int n = 0;

  if( OptVerbose )
  {
    printf("%s scanning...\n", Argv0);
    printf(" scanned devices: ");
  }
  n = i2c_scan(pI2C, scanCallback, NULL);

  if(n > 0)
  {
    printf("\n");
  }

  if( OptVerbose )
  {
    printf(" number found:    %d\n", n);
  }

  return n;
}

/*!
 * \brief Command initialization.
 *
 * \param argc    Command-line argument count.
 * \param argv    Command-line arguments.
 * \param pI2C    Pointer to \h_i2c handle.
 */
static void MainInit(int argc, char *argv[], i2c_t *pI2C)
{
  // Name of this process
  Argv0 = basename(argv[0]);

  // Get the environment
  //EnvGet();

  // Parse input arguments
  argv = OptsGet(Argv0, &PkgInfo, &I2CScanPgmInfo, I2CScanOptsInfo, true,
                 &argc, argv);

  // Opened device specified
  if( OptDevFd >= 0 )
  {
    pI2C->fd    = OptDevFd;
    pI2C->addr  = (ushort_t)(-1);
  }

  // I2C Bus device specified
  else
  {
    if( OptDevName == NULL || OptDevName[0] == 0 )
    {
      OptDevName = "/dev/i2c/0";
    }

    if( OptVerbose )
    {
      printf("I2C device: %s\n\n", OptDevName);
    }
  
    if( i2c_open(pI2C, OptDevName) < 0 )
    {
     LOGSYSERROR("%s: Failed to open.", OptDevName);
     exit(EC_ERROR);
    }
  }
}

/*!
 * \brief i2cscan main()
 *
 * \param argc  Count of command-line options and arguments.
 * \param argv  Array of command-line options and arguments.
 *
 * \return Exit value.
 */
int main(int argc, char *argv[])
{
  i2c_t i2c;
  int   rc;

  MainInit(argc, argv, &i2c);

  rc = execScan(&i2c);

  return rc > 0? 0: 1;
}

/*!
\page i2cscan I2CSCAN(1)

\section NAME
i2cscan - I<sup>2</sup>C Bus Scan

\section SYNOPSIS
i2cscan [OPTIONS]

\section DESCRIPTION
The i2cscan command scans the specified I<sup>2</sup>C Bus for all attached
devices. The I<sup>2</sup>C addresses of all found devices are written to
\p stdout in ASCII hex format.


\section OPTIONS RNR_OPTIONS 
\verbatim
  -d, --device=<device>       I2C device.
                                DEFAULT: /dev/i2c/0
      --fd=<n>                Opened I2C device file descriptor.
  -v, --verbose               Set print verbosity.
                                DEFAULT: false
  RNR_OPTIONS                 Standard set of options provided by librnr.
\endverbatim

\section DIAGNOSTICS
Exit status is 0 if devices are found, 1 otherwise.
Exit status is >= 2 if error(s) are encountered.

\section SEE_ALSO
\ref i2ccheck,
\ref i2cread,
\ref i2ctrans,
\ref i2cwrite

\section AUTHOR
Robin Knight (robin.knight@roadnarrows.com)

\section COPYRIGHT
(C) 2007.  RoadNarrows LLC.
(http://www.roadnarrows.com)
\n All Rights Reserved
*/
