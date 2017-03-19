////////////////////////////////////////////////////////////////////////////////
//
// Package:   CogniBoost
//
// Library:   libCogniBoost
//
// File:      cbLibError.c
//
/*! \file
 *
 * $LastChangedDate: 2011-10-19 15:05:47 -0600 (Wed, 19 Oct 2011) $
 * $Rev: 1398 $
 *
 * \brief Error and logging handling routines.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 * \author Brent Wilkins  (brent@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2011-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
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
#include <libgen.h>
#include <string.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/new.h"
#include "rnr/units.h"

#include "CogniBoost/CogniBoost.h"
#include "CogniBoost/libCogniBoost.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------
#define CB_RESERVED_ESTR "Reserved CogniBoost error string"

/*!
 * \brief Package CogniBoost Error Code String Table.
 *
 * Table is indexed by error codes (see \ref cb_ecodes). Keep
 * in sync.
 */
static const char *cbEcodeStrTbl[] =
{
  "Ok",                                         ///< [CB_OK]
  "Response Error",                             ///< [CB_RSP_ECODE_GEN]
  "Response Error: Bad packet",                 ///< [CB_RSP_ECODE_BAD_PKT]
  "Response Error: Checksum failed",            ///< [CB_RSP_ECODE_CHKSUM]
  "Response Error: Message too short",          ///< [CB_RSP_ECODE_MSG_SMALL]
  "Response Error: Message too long",           ///< [CB_RSP_ECODE_MSG_BIG]
  "Response Error: Bad or corrupted message",   ///< [CB_RSP_ECODE_BAD_MSG]
  "Response Error: Receive buffer overflow."    ///< [CB_RSP_ECODE_OVERFLOW]
  "Response Error: Upload aborted",             ///< [CB_RSP_ECODE_ABORT]
  "Response Error: Unknown or invalid command", ///< [CB_RSP_ECODE_CMD_ID_INVAL]
  "Response Error: Invalid command syntax",     ///< [CB_RSP_ECODE_CMD_INVAL]
  "Response Error: Invalid argument syntax",    ///< [CB_RSP_ECODE_ARG_INVAL]
  "Response Error: Argument out of range",      ///< [CB_RSP_ECODE_ARG_RANGE]
  "Response Error: Wrong number of arguments",  ///< [CB_RSP_ECODE_ARG_CNT]
  "Response Error: Execution error",            ///< [CB_RSP_ECODE_EXEC]
  "Response Error: Operation not permitted",    ///< [CB_RSP_ECODE_PERM]
  CB_RESERVED_ESTR                              ///< [RESERVED]
  CB_RESERVED_ESTR                              ///< [RESERVED]
  CB_RESERVED_ESTR                              ///< [RESERVED]
  CB_RESERVED_ESTR                              ///< [RESERVED]
  CB_RESERVED_ESTR                              ///< [RESERVED]
  CB_RESERVED_ESTR                              ///< [RESERVED]
  CB_RESERVED_ESTR                              ///< [RESERVED]
  CB_RESERVED_ESTR                              ///< [RESERVED]
  "Error",                                      ///< [CB_LIB_ECODE_GEN]
  "System call failed",                         ///< [CB_LIB_ECODE_SYS]
  "Invalid/missing response error code",        ///< [CB_LIB_ECODE_RSP_BADEC]
  "Not supported (yet)",                        ///< [CB_LIB_ECODE_NOTSUP]
  "No device"                                   ///< [CB_LIB_ECODE_BUSY]
  "Operation timed out",                        ///< [CB_LIB_ECODE_TIMEDOUT]
  "No resources availbable"                     ///< [CB_LIB_ECODE_NO_RSRC]
  "Resource busy",                              ///< [CB_LIB_ECODE_BUSY]
  "Cannot execute"                              ///< [CB_LIB_ECODE_EXEC]
  "Bad I/O",                                    ///< [CB_LIB_ECODE_IO]
  "Bad read",                                   ///< [CB_LIB_ECODE_READ]
  "Bad write",                                  ///< [CB_LIB_ECODE_WRITE]
  "Cannot synchronize",                         ///< [CB_LIB_ECODE_SYNC]
  "Bad receive",                                ///< [CB_LIB_ECODE_RECV]
  "Bad send",                                   ///< [CB_LIB_ECODE_SEND]
  "Bad packet header",                          ///< [CB_LIB_ECODE_BAD_HDR]
  "Bad or corrupted message",                   ///< [CB_LIB_ECODE_BAD_MSG]
  "Bad or partial response",                    ///< [CB_LIB_ECODE_BAD_RSP]
  "Checksum failed",                            ///< [CB_LIB_ECODE_CHKSUM]
  "Invalid argument",                           ///< [CB_LIB_ECODE_INVAL]
  "Argument too small",                         ///< [CB_LIB_ECODE_TOO_SMALL]
  "Argument too big",                           ///< [CB_LIB_ECODE_TOO_BIG]
  "Argument out of range",                      ///< [CB_LIB_ECODE_RANGE]
  "Software bug!",                              ///< [CB_LIB_ECODE_BUG]
  "Invalid error code"                          ///< [CB_LIB_ECODE_BADEC]
};

// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Get the error string describing the CogniBoost error code.
 *
 * The absolute value of the error code is taken prior retrieving the string.
 * An unknown or out-of-range error code will be mapped to
 * \ref CB_ECODE_BADEC.
 *
 * \param ecode CogniBoost error code.
 *
 * \return Returns the appropriate error code string.
 */
const char *cbStrError(int ecode)
{
  if( ecode < 0 )
  {
    ecode = -ecode;
  }

  if( ecode >= arraysize(cbEcodeStrTbl) )
  {
    ecode = CB_LIB_ECODE_BADEC;
  }
  return cbEcodeStrTbl[ecode];
}

/*!
 * \brief Pretty print a byte buffer to opened file stream.
 *
 * \param fp        File pointer.
 * \param sPreface  Optional buffer preface string (set to NULL for no preface).
 * \param buf       Buffer to print.
 * \param sFmt      Buffer entry format string.
 * \param uCount    Number of entries to print.
 * \param uNLFreq   Newline frequency (set to 0 for no newlines).
 * \param uCol      Column alignment number.
 */
void cbPrintBuf(FILE       *fp,
                const char *sPreface,
                byte_t      buf[],
                const char *sFmt,
                size_t      uCount,
                size_t      uNLFreq,
                uint_t      uCol)
{
  size_t  i;

  if( sPreface && *sPreface )
  {
    fprintf(fp, "%s", sPreface);
  }

  for(i=0; i<uCount; ++i)
  {
    if( (uNLFreq > 0) && ((i % uNLFreq) == 0) && (i != 0) )
    {
      fprintf(fp, "\n%*s", uCol, "");
    }
    fprintf(fp, sFmt, buf[i]);
  }
  fprintf(fp, "\n");
}

#ifdef LOG
/*! 
 * \brief Log integer data.
 *
 * Print diagnostic logging of the contents of a buffer of bytes.
 *
 * \param sPreface  Buffer preface string.
 * \param buf       Buffer.
 * \param uCount    Number of entries to log.
 * \param sFmt      Buffer entry format string.
 */
void cbLogBuf(const char *sPreface, int buf[], size_t uCount, const char *sFmt)
{
  FILE   *fp;   // log file pointer
  size_t  i;    // working index

  fp = LOG_GET_LOGFP();

  fprintf(fp, "%sDiag%d: %s", LOG_PREFACE, LOG_GET_THRESHOLD()-1, sPreface);
  for(i=0; i<uCount; ++i)
  {
    if( (i % 8) == 0 )
    {
      fprintf(fp, "\n");
    }
    fprintf(fp, sFmt, buf[i]);
  }
  fprintf(fp, "\n");
}
#endif // LOG

