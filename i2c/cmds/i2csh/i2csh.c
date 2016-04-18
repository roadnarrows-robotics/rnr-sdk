////////////////////////////////////////////////////////////////////////////////
//
// Package:   RCB3
//
// File:      i2csh.c
//
/*! \file
 *
 * $LastChangedDate: 2009-09-09 09:44:12 -0600 (Wed, 09 Sep 2009) $
 * $Rev: 130 $
 *
 * \brief Simple I2C Bus Command-Line Shell.
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

#ifdef HAVE_READLINE
#include <readline/readline.h>
#include <readline/history.h>
#endif // HAVE_READLINE

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/opts.h"

#include "rnr/i2c.h"

#include "version.h"

// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

#define RC_QUIT (RC_ERROR-1)  ///< quit shell return code

//
// types
//

/*!
 * \brief Command execution function type.
 */
typedef int (execfunc_t)(i2c_t *, int, const char *[]);

/*!
 * \brief Shell Command Info.
 */
typedef struct
{
  const char  *m_sCmd;          ///< input-line full command name
  execfunc_t  *m_fnExec;        ///< assoc. exec. function
  const char  *m_sHelpBrief;    ///< command help
  const char  *m_sHelpArgs;     ///< command help argments
} shcmd_t;

// The Command
static char *Argv0;                       ///< command name

static int  OptVerbose  = 0;              ///< verbose option
static char *OptDevName = "/dev/i2c/0";   ///< i2c bus device option

/*!
 *  \brief Program Information
 */
static OptsPgmInfo_T I2CShPgmInfo =
{
  .synopsis   = "I2C Simple Raw Shell",
  .long_desc  =
    "The I2C Shell (%P) provides a simple interactive interface to devices "
    "attached to an I2C Bus."
};

/*!
 * \brief Command Line Options Information
 */
static OptsInfo_T I2CShOptsInfo[] =
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


// ---------------------------------------------------------------------------
// Support
// ---------------------------------------------------------------------------

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
 * \brief Make array of arguments separated with white-space.
 *
 * \param sLine         Input line.
 * \param [out] pArgc   Number of parsed arguments
 *
 * \return Returns pointer to array of strings.
 */
static const char **makeargs(char *sLine, int *pArgc)
{
  static const char *args[256];
  char              *s;
  char              *sDelim = " \t\n\r";
  int                i = 0;

  for(s=strtok(sLine, sDelim); s!=NULL && i<256; s=strtok(NULL, sDelim))
  {
    args[i++] = s;
  }
  *pArgc = i;
  return args;
}

#ifndef HAVE_READLINE
/*!
 * \brief Read an input line from stdin.
 *
 * This function is used only if there is no readline functionality.
 *
 * \param prompt  User prompt string.
 *
 * \return Returns read line buffer.
 */
static char *readline(const char *prompt)
{
  static char linebuf[512]; // real readline() uses malloc's so adjust
  int         bIsBlank;
  char       *s;

  if( prompt && *prompt )
  {
    printf("%s", prompt);
  }

  if( fgets(linebuf, (int)sizeof(linebuf), stdin) == NULL )
  {
    return 0;
  }

  linebuf[sizeof(linebuf)-1] = 0;

  bIsBlank = 1;
  for(s=linebuf; *s && bIsBlank; ++s)
  {
    if( !isspace(*s) )
    {
      bIsBlank = 0;
    }
  }

  if( bIsBlank )
  {
    return NULL;
  }
  else
  {
    return linebuf;
  }
}
#endif // HAVE_READLINE


// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------

/*!
 * \brief Execute \h_i2c slave device read.
 *
 * read \<addr\> \<readlength\>
 *
 * \param pI2C    Pointer to \h_i2c handle.
 * \param nArgc   Number of input arguments.
 * \param sArgv   Array of input argument strings.
 *
 * \return Returns OK on success, RC_ERROR on error.
 */
static int execRead(i2c_t *pI2C, int nArgc, const char *sArgv[])
{
  int     i;
  byte_t  addr;
  byte_t  readbuf[256];
  uint_t  readlen;
  int     rc;

  if( nArgc != 3 )
  {
    printf("Error: %s requires 2 arguments\n", sArgv[0]);
    return RC_ERROR;
  }

  LOGDIAG1CALL(_TPTR(pI2C), _TINT(nArgc), _TSTR(sArgv[0]), _TSTR(sArgv[1]),
              _TSTR(sArgv[2]));

  // address
  if( StrToByte(sArgv[1], &addr) != OK )
  {
    printf("Error: %s: bad I2C address\n", sArgv[1]);
    return RC_ERROR;
  }
  else if( addr > 0x7f )
  {
    printf("Error: 0x%02x: I2C address out of range [0,0x7f]\n", addr);
    return RC_ERROR;
  }

  // read length
  if( StrToUnsigned(sArgv[2], &readlen) != OK )
  {
    printf("Error: %s: bad read length\n", sArgv[2]);
    return RC_ERROR;
  }
  else if( readlen > sizeof(readbuf) )
  {
    printf("Error: %d: I2C read length out of range [0,%u]\n",
        readlen, (uint_t)sizeof(readbuf));
    return RC_ERROR;
  }

  // echo out parsed command prior to execution
  if( OptVerbose )
  {
    printf("command:       %s\n", sArgv[0]);
    printf(" address:      0x%02x\n", addr);
    printf(" read length:  %d\n", readlen);
  }

  rc = i2c_read(pI2C, addr, readbuf, readlen);
  if( rc < 0 )
  {
    LOGSYSERROR("i2c_read()");
    return RC_ERROR;
  }

  if( OptVerbose )
  {
    printf("response:\n");
    printf(" read:        ");
  }

  readlen = (uint_t)rc;

  for(i=0; i<readlen; ++i)
  {
    printf("0x%02x ", readbuf[i]);
  }
  printf("\n");
  if( OptVerbose )
  {
    printf(" bytes read:  %u\n", readlen);
  }

  return OK;
}

/*!
 * \brief Execute \h_i2c slave device write.
 *
 * write \<addr\> \<wbyte0\> [\<wbyte1\> ...]
 *
 * \param pI2C    Pointer to \h_i2c handle.
 * \param nArgc   Number of input arguments.
 * \param sArgv   Array of input argument strings.
 *
 * \return Returns OK on success, RC_ERROR on error.
 */
static int execWrite(i2c_t *pI2C, int nArgc, const char *sArgv[])
{
  int     i, j;
  byte_t  addr;
  byte_t  writebuf[256];
  uint_t  writelen;
  int     rc;

  if( nArgc < 3 )
  {
    printf("Error: %s requires at least 2 arguments\n", sArgv[0]);
    return RC_ERROR;
  }

  LOGDIAG1CALL(_TPTR(pI2C), _TINT(nArgc), _TSTR(sArgv[0]), _TSTR(sArgv[1]),
              _TSTR(sArgv[2]));

  // address
  if( StrToByte(sArgv[1], &addr) != OK )
  {
    printf("Error: %s: bad I2C address\n", sArgv[1]);
    return RC_ERROR;
  }
  else if( addr > 0x7f )
  {
    printf("Error: 0x%02x: I2C address out of range [0,0x7f]\n", addr);
    return RC_ERROR;
  }

  // write bytes
  for(i=2, j=0; i<nArgc && j<sizeof(writebuf); ++i, ++j)
  {
    if( StrToByte(sArgv[i], writebuf+j) != OK )
    {
      printf("Error: argv[%d]='%s': bad byte value\n", i, sArgv[i]);
      return RC_ERROR;
    }
  }
  writelen = (uint_t)j;

  // echo out parsed command prior to execution
  if( OptVerbose )
  {
    printf("command:       %s\n", sArgv[0]);
    printf(" address:      0x%02x\n", addr);
    printf(" write length: %d\n", writelen);
    printf(" write:        ");
    for(i=0; i<writelen; ++i)
    {
      printf("0x%02x ", writebuf[i]);
    }
    printf("\n");
  }

  rc = i2c_write(pI2C, addr, writebuf, writelen); 

  if( rc < 0 )
  {
    LOGSYSERROR("i2c_write()");
    return RC_ERROR;
  }

  if( OptVerbose )
  {
    printf("response:\n");
    printf(" bytes written: ");
  }
  printf("%d\n", writelen);

  return OK;
}

/*!
 * \brief Execute \h_i2c slave device write/read transaction.
 *
 * transaction \<addr\> \<wbyte0\> [\<wbyte1\> ...] \<readlength\>
 *
 * \param pI2C    Pointer to \h_i2c handle.
 * \param nArgc   Number of input arguments.
 * \param sArgv   Array of input argument strings.
 *
 * \return Returns OK on success, RC_ERROR on error.
 */
static int execTransaction(i2c_t *pI2C, int nArgc, const char *sArgv[])
{
  int     i, j;
  byte_t  addr;
  byte_t  writebuf[256];
  uint_t  writelen;
  byte_t  readbuf[256];
  uint_t  readlen;
  int     rc;

  if( nArgc < 4 )
  {
    printf("Error: %s requires at least 3 arguments\n", sArgv[0]);
    return RC_ERROR;
  }

  LOGDIAG1CALL(_TPTR(pI2C), _TINT(nArgc), _TSTR(sArgv[0]), _TSTR(sArgv[1]),
              _TSTR(sArgv[2]), _TSTR(sArgv[3]));

  // address
  if( StrToByte(sArgv[1], &addr) != OK )
  {
    printf("Error: %s: bad I2C address\n", sArgv[1]);
    return RC_ERROR;
  }
  else if( addr > 0x7f )
  {
    printf("Error: 0x%02x: I2C address out of range [0,0x7f]\n", addr);
    return RC_ERROR;
  }

  // write bytes
  for(i=2, j=0; i<nArgc-1 && j<sizeof(writebuf); ++i, ++j)
  {
    if( StrToByte(sArgv[i], writebuf+j) != OK )
    {
      printf("Error: argv[%d]='%s': bad byte value\n", i, sArgv[i]);
      return RC_ERROR;
    }
  }
  writelen = (uint_t)j;

  // read length
  if( StrToUnsigned(sArgv[nArgc-1], &readlen) != OK )
  {
    printf("Error: %s: bad read length\n", sArgv[nArgc-1]);
    return RC_ERROR;
  }
  else if( readlen > sizeof(readbuf) )
  {
    printf("Error: %d: I2C read length out of range [0,%u]\n",
        readlen, (uint_t)sizeof(readbuf));
    return RC_ERROR;
  }

  // echo out parsed command prior to execution
  if( OptVerbose )
  {
    printf("command:       %s\n", sArgv[0]);
    printf(" address:      0x%02x\n", addr);
    printf(" write length: %d\n", writelen);
    printf(" write:        ");
    for(i=0; i<writelen; ++i)
    {
      printf("0x%02x ", writebuf[i]);
    }
    printf("\n");
    printf(" read length:  %d\n", readlen);
  }

  rc = i2c_transfer(pI2C, addr, writebuf, writelen, readbuf, readlen);

  if( rc < 0 )
  {
    LOGSYSERROR("i2c_transfer()");
    return RC_ERROR;
  }

  if( OptVerbose )
  {
    printf("response:\n");
    printf(" read: ");
  }
  for(i=0; i<readlen; ++i)
  {
    printf("0x%02x ", readbuf[i]);
  }
  printf("\n");

  return OK;
}

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
 * scan
 *
 * \param pI2C    Pointer to \h_i2c handle.
 * \param nArgc   Number of input arguments.
 * \param sArgv   Array of input argument strings.
 *
 * \return Returns OK on success, RC_ERROR on error.
 */
static int execScan(i2c_t *pI2C, int nArgc, const char *sArgv[])
{
  int n;

  if( nArgc > 1 )
  {
    printf("Error: %s takes no arguments\n", sArgv[0]);
    return RC_ERROR;
  }

  LOGDIAG1CALL(_TPTR(pI2C), _TINT(nArgc), _TSTR(sArgv[0]));

  if( OptVerbose )
  {
    printf("command:         %s\n", sArgv[0]);
    printf("response:\n");
    printf(" scanned devices: ");
  }
  n = i2c_scan(pI2C, scanCallback, NULL);
  printf("\n");

  if( OptVerbose )
  {
    printf(" number found:    %d\n", n);
  }

  return OK;
}

/*!
 * \brief Execute \h_i2c slave device check.
 *
 * check \<addr\>
 *
 * \param pI2C    Pointer to \h_i2c handle.
 * \param nArgc   Number of input arguments.
 * \param sArgv   Array of input argument strings.
 *
 * \return Returns OK on success, RC_ERROR on error.
 */
static int execCheck(i2c_t *pI2C, int nArgc, const char *sArgv[])
{
  byte_t  addr;
  int     rc;

  if( nArgc != 2 )
  {
    printf("Error: %s requires 1 argument\n", sArgv[0]);
    return RC_ERROR;
  }

  LOGDIAG1CALL(_TPTR(pI2C), _TINT(nArgc), _TSTR(sArgv[0]), _TSTR(sArgv[1]));

  // address
  if( StrToByte(sArgv[1], &addr) != OK )
  {
    printf("Error: %s: bad I2C address\n", sArgv[1]);
    return RC_ERROR;
  }
  else if( addr > 0x7f )
  {
    printf("Error: 0x%02x: I2C address out of range [0,0x7f]\n", addr);
    return RC_ERROR;
  }

  if( OptVerbose )
  {
    printf("command:      %s\n", sArgv[0]);
  }
  rc = i2c_exists(pI2C, addr);
  if( rc < 0 )
  {
    LOGSYSERROR("i2c_exists()");
    return RC_ERROR;
  }

  if( OptVerbose )
  {
    printf("response:\n");
    printf(" 0x%02X", addr);
  }

  if( rc )
  {
    printf("device found\n");
  }
  else
  {
    printf("device not found\n");
  }

  return OK;
}

/*!
 * \brief Execute enabling/disabling verbose printing.
 *
 * \param pI2C    Pointer to \h_i2c handle.
 * \param nArgc   Number of input arguments.
 * \param sArgv   Array of input argument strings.
 *
 * \return Returns OK on success, RC_ERROR on error.
 */
static int execVerbose(i2c_t *pI2C, int nArgc, const char *sArgv[])
{
  //printf("execExists(%s, %d, ...)\n", sArgv[0], nArgc);

  if( nArgc != 2 )
  {
    printf("Error: %s requires 1 argument\n", sArgv[0]);
    return RC_ERROR;
  }

  if( !strcmp(sArgv[1], "on") )
  {
    OptVerbose = 1;
    printf("on\n");
    return OK;
  }
  else if( !strcmp(sArgv[1], "off") )
  {
    OptVerbose = 0;
    printf("off\n");
    return OK;
  }
  else
  {
    printf("Error: %s: unknown option\n", sArgv[1]);
    return RC_ERROR;
  }
}

static shcmd_t I2CCmds[];     ///< forward declaration of shell commands

/*!
 * \brief Execute shell help.
 *
 * \param pI2C    Pointer to \h_i2c handle.
 * \param nArgc   Number of input arguments.
 * \param sArgv   Array of input argument strings.
 *
 * \return Returns OK on success, RC_ERROR on error.
 */
static int execHelp(i2c_t *pI2C, int nArgc, const char *sArgv[])
{
  shcmd_t *pCmd;

  //printf("execHelp(%s, %d, ...)\n", sArgv[0], nArgc);

  for(pCmd=I2CCmds; pCmd->m_sCmd!=NULL; pCmd++)
  {
    if( pCmd->m_sHelpArgs != NULL )
    {
      printf("%s %s\n", pCmd->m_sCmd, pCmd->m_sHelpArgs);
    }
    printf("  %s\n", pCmd->m_sHelpBrief);
  }
  return OK;
}

/*!
 * \brief Execute quit shell.
 *
 * \param pI2C    Pointer to \h_i2c handle.
 * \param nArgc   Number of input arguments.
 * \param sArgv   Array of input argument strings.
 *
 * \return Returns RC_QUIT.
 */
static int execQuit(i2c_t *pI2C, int nArgc, const char *sArgv[])
{
  //printf("execQuit(%s, %d, ...)\n", sArgv[0], nArgc);
  i2c_close(pI2C);

  return RC_QUIT;
}

/*!
 * \brief Shell commands.
 */
static shcmd_t I2CCmds[] =
{
  { "read",
    execRead, 
    "read I2C device",
    "<addr> <readlength>"
  },

  { "write",
    execWrite, 
    "write I2C device",
    "<addr> <wbyte0> [<wbyte1> ...]"
  },

  { "transaction",
    execTransaction, 
    "I2C write/read transaction",
    "<addr> <wbyte0> [<wbyte1> ...] <readlength>"
  },

  { "check",
    execCheck, 
    "check if device exists on I2C Bus",
    "<addr>"
  },

  { "scan",
    execScan, 
    "scan I2C Bus for all connected devices",
    ""
  },

  { "verbose",
    execVerbose, 
    "print verbosity",
    "{on|off}"
  },

  { "help",
    execHelp, 
    "print help",
    ""
  },

  { "quit",
    execQuit, 
    "quit shell",
    ""
  },

  {NULL, }
};


// ---------------------------------------------------------------------------
// Execution Control
// ---------------------------------------------------------------------------

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
  argv = OptsGet(Argv0, &PkgInfo, &I2CShPgmInfo, I2CShOptsInfo, true,
                 &argc, argv);

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

/*!
 * \brief Shell main loop.
 *
 * \param pI2C    Pointer to \h_i2c handle.
 */
static void MainLoop(i2c_t *pI2C)
{
  int           bDoQuit = 0;
  char         *sInput;
  int           nArgc;
  const char  **sArgv;
  //int         i;
  int           bDidCmd;
  shcmd_t      *pCmd;
  int           rc;

  while( !bDoQuit )
  {
    sInput = readline("i2csh> ");
    if( sInput == NULL )
    {
      continue;
    }
    //printf("'%s'\n", sInput);

    sArgv = makeargs(sInput, &nArgc);
    //for(i=0; i<nArgc; ++i)
    //{
    //  printf("sArgv[%d]='%s'\n", i, sArgv[i]);
    //}

    if( nArgc == 0 )
    {
      continue;
    }

    for(pCmd=I2CCmds, bDidCmd=0; pCmd->m_sCmd!=NULL && !bDidCmd; pCmd++)
    {
      if( !strncmp(sArgv[0], pCmd->m_sCmd, strlen(sArgv[0])) )
      {
        sArgv[0] = pCmd->m_sCmd;
        rc = pCmd->m_fnExec(pI2C, nArgc, sArgv);
        bDidCmd = 1;
        if( rc == RC_QUIT )
        {
          bDoQuit = 1;
        }
      }
    }

    if( !bDidCmd )
    {
      printf("Error: Unknown command: %s\n", sArgv[0]);
    }
#ifdef HAVE_READLINE
     free(sInput);
#endif // HAVE_READLINE
  }
}

/*!
 * \brief i2csh main()
 *
 * \param argc  Count of command-line options and arguments.
 * \param argv  Array of command-line options and arguments.
 *
 * \return Exit value.
 */
int main(int argc, char *argv[])
{
  i2c_t i2c;

  MainInit(argc, argv, &i2c);

  printf("I2C Raw Shell\n");
  printf("-------------\n");
  printf("(partial command matching valid; enter 'help' for help)\n\n");

  MainLoop(&i2c);

  return 0;
}

/*!
\page i2csh I2CSH(1)

\section NAME
i2csh - Simple I<sup>2</sup>C Bus command-line shell

\section SYNOPSIS
i2csh [OPTIONS]

\section DESCRIPTION
The I2C Shell (i2csh) provides a simple interactive interface to devices
attached to an I<sup>2</sup>C Bus.

\section OPTIONS RNR_OPTIONS 
\verbatim
  -d, --device=<device>       I2C device.
                                DEFAULT: /dev/i2c/0
  -v, --verbose               Set print verbosity.
                                DEFAULT: false
  RNR_OPTIONS                 Standard set of options provided by librnr.
\endverbatim

\section AUTHOR
Robin Knight (robin.knight@roadnarrows.com)

\section COPYRIGHT
(C) 2007.  RoadNarrows LLC.
(http://www.roadnarrows.com)
\n All Rights Reserved
*/
