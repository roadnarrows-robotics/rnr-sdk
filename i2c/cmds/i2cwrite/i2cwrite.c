////////////////////////////////////////////////////////////////////////////////
//
// Package:   RCB3
//
// File:      i2cwrite.c
//
/*! \file
 *
 * $LastChangedDate: 2011-09-12 16:08:28 -0600 (Mon, 12 Sep 2011) $
 * $Rev: 1279 $
 *
 * \brief Write bytes to a device on the I2C Bus.
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
static char *Argv0;                       ///< command name

//
// Command Option Values
//
static int    OptVerbose  = 0;            ///< verbose option
static char  *OptDevName  = "/dev/i2c/0"; ///< i2c bus device option
static int    OptDevFd    = -1;           ///< opened i2c buse device fd option
static int    OptI2CAddr  = -1;           ///< slave i2c device address
static byte_t ArgWriteBuf[1024];          ///< write buffer argument
static int    ArgWriteLen = 0;            ///< write byte count arguement

/*!
 *  \brief Program Information
 */
static OptsPgmInfo_T I2CWritePgmInfo =
{
  .usage_args = "byte1 [byte2 ...]",

  .synopsis = "Write bytes to a device on an I2C Bus.",

  .long_desc = 
    "The %P command write the bytes arguments to an I2C Bus for a device "
    "with the specified address.",

  .diagnostics = 
    "Exit status is 0 if the bytes are successfully written. "
    "Exist status is >= 2 if error(s) are encountered.",
};

/*!
 * \brief Command Line Options Information
 */
static OptsInfo_T I2CWriteOptsInfo[] =
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
 * \brief Execute \h_i2c slave device write.
 *
 * \param pI2C      Pointer to \h_i2c handle.
 * \param addr      Slave device address.
 * \param writebuf  Buffer to write.
 * \param writelen  Number of write bytes.
 *
 * \return Returns \h_ge 0 on success, -1 on failure.
 */
static int execWrite(i2c_t *pI2C, int addr, byte_t writebuf[], int writelen)
{
  int   n;

  if( OptVerbose )
  {
    printf("%s: i2cdev 0x%02x: writting %u bytes...\n",
        Argv0, (byte_t)addr, writelen);
  }

  n = i2c_write(pI2C, (i2c_addr_t)addr, writebuf, (uint_t)writelen); 

  if( n < 0 )
  {
    LOGSYSERROR("i2c_write()");
    return RC_ERROR;
  }

  if( OptVerbose )
  {
    printf("Wrote %d bytes\n", n);
  }

  if( n < writelen )
  {
    LOGERROR("wrote only %d bytes of %d", n, writelen);
  }

  return n;
}

/*!
 * \brief Convert string to unsigned integer.
 *
 * \param s           String.
 * \param [out] pVal  Pointer to converted value.
 *
 * \return Returns OK on success, RC_ERROR on error.
 */
static int StrToUnsigned(const char *s, uint_t *pVal)
{
  long    lVal;
  char   *sEnd;

  if( (s == NULL) || (*s == 0) )
  {
    return RC_ERROR;
  }
  
  lVal = strtol(s, &sEnd, 0);

  if( *sEnd != 0 )
  {
    return RC_ERROR;
  }
  else if( lVal < 0 )
  {
    return RC_ERROR;
  }
  else
  {
    *pVal = (uint_t)lVal;
    return OK;
  }
}

/*!
 * \brief Convert string to byte.
 *
 * \param s             String.
 * \param [out] pByte   Pointer to converted value.
 *
 * \return Returns OK on success, RC_ERROR on error.
 */
static int StrToByte(const char *s, byte_t *pByte)
{
  uint_t  uVal;

  if( StrToUnsigned(s, &uVal) != OK )
  {
    return RC_ERROR;
  }
  else if( uVal > 0xff )
  {
    return RC_ERROR;
  }
  else
  {
    *pByte = (byte_t)uVal;
    return OK;
  }
}

/*!
 * \brief Get write bytes from command-line arguments.
 *
 * \param argc    Command-line argument count.
 * \param argv    Command-line arguments.
 *
 * \return Exits on error.
 */
static void ArgsGet(int argc, char *argv[])
{
  int     i;
  size_t  max = sizeof(ArgWriteBuf);  

  if( argc == 0 )
  {
    printf("%s Error: At least one byte must be specified\n", Argv0);
    exit(EC_ERROR);
  }
  else if( argc > max )
  {
    fprintf(stderr, 
        "%s: Warning: %d: I2C write. Only %u bytes will be written.\n",
        Argv0, argc, (uint_t)max);
  }

  for(i=0; i<argc && i<max; ++i)
  {
    if( StrToByte(argv[i], ArgWriteBuf+i) != OK )
    {
      printf("%s Error: byte %d = '%s': bad byte value\n", Argv0, i, argv[i]);
      exit(EC_ERROR);
    }
  }

  ArgWriteLen = i;
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
  argv = OptsGet(Argv0, &PkgInfo, &I2CWritePgmInfo, I2CWriteOptsInfo, true,
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

  // Convert command-line arguments
  ArgsGet(argc, argv);

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
 * \brief i2cwrite main()
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

  rc = execWrite(&i2c, OptI2CAddr, ArgWriteBuf, ArgWriteLen);

  return rc == ArgWriteLen? 0: (rc >= 0? 1: 2);
}

/*!
\page i2cwrite I2CWRITE(1)

\section NAME
i2cwrite - Write bytes to a device on a I<sup>2</sup>C Bus

\section SYNOPSIS
i2cwrite [OPTIONS] byte1 [byte2 ...]

\section DESCRIPTION
Write bytes to a device on the I<sup>2</sup>C Bus.

\section OPTIONS RNR_OPTIONS 
\verbatim
  -a, --address=<addr>        I2C device 7-bit address. REQUIRED.
  -d, --device=<device>       I2C device.
                                DEFAULT: /dev/i2c/0
      --fd=<n>                Opened I2C device file descriptor.
  -v, --verbose               Set print verbosity.
                                DEFAULT: false
  RNR_OPTIONS                 Standard set of options provided by librnr.
\endverbatim

\section DIAGNOSTICS
Exit status is 0 if the bytes are successfully written.
Exist status is \>= 2 if error(s) are encountered.

\section SEE_ALSO
\ref i2ccheck
\ref i2cread
\ref i2cscan
\ref i2ctrans

\section AUTHOR
Robin Knight (robin.knight@roadnarrows.com)

\section COPYRIGHT
(C) 2007.  RoadNarrows LLC.
(http://www.roadnarrows.com)
\n All Rights Reserved
*/
