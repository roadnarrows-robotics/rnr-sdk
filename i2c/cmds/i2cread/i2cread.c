////////////////////////////////////////////////////////////////////////////////
//
// Package:   RCB3
//
// File:      i2cread.c
//
/*! \file
 *
 * $LastChangedDate: 2011-09-12 16:08:28 -0600 (Mon, 12 Sep 2011) $
 * $Rev: 1279 $
 *
 * \brief Read an I2C device.
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

// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

// The Command
static char *Argv0;                             ///< command namve

//
// Command Option Values
//
static int    OptVerbose      = 0;              ///< verbose option
static char  *OptDevName      = "/dev/i2c/0";   ///< i2c bus device option
static int    OptDevFd        = -1;             ///< opened i2c bus dev option
static int    OptI2CAddr      = -1;             ///< slave device address option
static int    OptI2CReadCount = 1;              ///< read byte count option

/*!
 *  \brief Program Information
 */
static OptsPgmInfo_T I2CReadPgmInfo =
{
  .usage_args = "--address <addr>",

  .synopsis = "Read <count> bytes from an I2C device.",

  .long_desc = 
    "The %P command attempts to read <count> bytes from the specified I2C "
    "device. All read bytes are printed to stdout in ASCII hex format.",

  .diagnostics = 
    "Exit status is 0 if <count> bytes are read, 1 otherwise. "
    "Exit status is >=2 if error(s) are encountered."
};

/*!
 * \brief Command Line Options Information
 */
static OptsInfo_T I2CReadOptsInfo[] =
{
  // I2C device address
  {
    .long_opt   = "address", 
    .short_opt  = 'a', 
    .has_arg    = required_argument,
    .opt_addr   = &OptI2CAddr,
    .fn_cvt     = OptsCvtArgInt,
    .fn_fmt     = OptsFmtInt,
    .arg_name   = "<addr>",
    .opt_desc   = "I2C device address. REQUIRED"
  },

  // I2C device read count
  {
    .long_opt   = "count", 
    .short_opt  = 'c', 
    .has_arg    = required_argument,
    .has_default= true,
    .opt_addr   = &OptI2CReadCount,
    .fn_cvt     = OptsCvtArgInt,
    .fn_fmt     = OptsFmtInt,
    .arg_name   = "<count>",
    .opt_desc   = "I2C device read byte count"
  },

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
 * \brief Execute \h_i2c slave device read.
 *
 * \param pI2C    Pointer to \h_i2c handle.
 * \param addr    Slave device address.
 * \param readlen Number of bytes to read.
 *
 * \return Returns number of bytes read \h_ge 0 on success, -1 on failure.
 */
static int execRead(i2c_t *pI2C, int addr, int readlen)
{
  int     i;
  byte_t  readbuf[1024];

  if( readlen > sizeof(readbuf) )
  {
    readlen = sizeof(readbuf);
    fprintf(stderr, 
        "%s: Warning: %d: I2C read length too long. Resetting to %u]\n",
        Argv0, readlen, (uint_t)sizeof(readbuf));
  }

  if( OptVerbose )
  {
    printf("%s Reading 0x%02x for %d bytes...\n", Argv0, (byte_t)addr, readlen);
  }

  readlen = i2c_read(pI2C, (i2c_addr_t)addr, readbuf, (uint_t)readlen);

  if( readlen < 0 )
  {
    LOGSYSERROR("i2c_read()");
    return RC_ERROR;
  }

  if( OptVerbose )
  {
    printf("Read %d bytes: ", readlen);
  }
  for(i=0; i<readlen; ++i)
  {
    printf("0x%02x ", readbuf[i]);
  }
  if( (readlen > 0) || OptVerbose )
  {
    printf("\n");
  }

  return readlen;
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
  argv = OptsGet(Argv0, &PkgInfo, &I2CReadPgmInfo, I2CReadOptsInfo, true,
                 &argc, argv);

  // Final option checks
  if( OptI2CAddr < 0x00 )
  {
    fprintf(stderr, "%s: Address option required\n", Argv0);
    exit(EC_BAD_OPT);
  }
  else if( (OptI2CAddr < I2C_ADDR_DEV_LOW) || (OptI2CAddr > I2C_ADDR_DEV_HIGH) )
  {
    fprintf(stderr, "%s: Address out of range: 0x%x\n", Argv0, OptI2CAddr);
    exit(EC_BAD_OPT);
  }

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
 * \brief i2cread main()
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

  rc = execRead(&i2c, OptI2CAddr, OptI2CReadCount);

  return rc > 0? 0: (rc == 0? 1: 2);
}

/*!
\page i2cread I2CREAD(1)

\section NAME
i2cread - Read bytes from an I<sup>2</sup>C Bus device

\section SYNOPSIS
i2cread [OPTIONS] --address \<addr\>

\section DESCRIPTION
The i2cread command attempts to read \<count\> bytes from the specified
I<sup>2</sup> device.
The bytes read are printed to \p stdout in ASCII hex format.

\section OPTIONS RNR_OPTIONS 
\verbatim
  -a, --address=<addr>        I2C device 7-bit address. REQUIRED.
  -c, --count=<count>         Read count bytes.
                                DEFAULT: 1
  -d, --device=<device>       I2C device.
                                DEFAULT: /dev/i2c/0
      --fd=<n>                Opened I2C device file descriptor.
  -v, --verbose               Set print verbosity.
                                DEFAULT: false
  RNR_OPTIONS                 Standard set of options provided by librnr.
\endverbatim

\section DIAGNOSTICS
Exit status is 0 if \<count\> bytes are read, 1 otherwise.
Exit status is \>= 2 if error(s) are encountered.

\section SEE_ALSO
\ref i2ccheck,
\ref i2cscan,
\ref i2ctrans,
\ref i2cwrite

\section AUTHOR
Robin Knight (robin.knight@roadnarrows.com)

\section COPYRIGHT
(C) 2007.  RoadNarrows LLC.
(http://www.roadnarrows.com)
\n All Rights Reserved
*/
