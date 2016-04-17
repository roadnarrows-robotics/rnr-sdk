////////////////////////////////////////////////////////////////////////////////
//
// Package:   BotSense
//
// Library:   libbotsense
//
// File:      bsLibError.c
//
/*! \file
 *
 * $LastChangedDate: 2010-09-25 09:06:47 -0600 (Sat, 25 Sep 2010) $
 * $Rev: 605 $
 *
 * \brief Error and logging handling routines.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2009-2010.  RoadNarrows LLC.
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
#include <libgen.h>
#include <string.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "botsense/BotSense.h"
#include "botsense/libBotSense.h"

#include "bsLibInternal.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief \h_botsense Error Code String Table.
 *
 * Table is indexed by \h_botsense error codes (see \ref bs_ecodes). Keep
 * in sync.
 */
static const char *bsEcodeStrTbl[] =
{
  "Ok",                                     ///< [BS_OK]

  "Error",                                  ///< [BS_ECODE_GEN]
  "Receive error",                          ///< [BS_ECODE_BAD_RECV]
  "Send error",                             ///< [BS_ECODE_BAD_SEND]
  "Failed to resynchronize",                ///< [BS_ECODE_BAD_RESYNC]
  "Bad message header",                     ///< [BS_ECODE_MSG_BAD_HDR]
  "Message fragment",                       ///< [BS_ECODE_MSG_FRAG]
  "Message too big",                        ///< [BS_ECODE_MSG_TOO_BIG]
  "Buffer too small",                       ///< [BS_ECODE_BUF_TOO_SMALL]
  "Message corrupted",                      ///< [BS_ECODE_BAD_MSG]
  "Invalid value",                          ///< [BS_ECODE_BAD_VAL]
  "Bad or unknown transaction id",          ///< [BS_ECODE_MSG_BAD_TID]
  "Bad virtual connection handle",          ///< [BS_ECODE_BAD_VCONN_HND]
  "Virtual connection not found",           ///< [BS_ECODE_NO_VCONN]
  "Unknown request",                        ///< [BS_ECODE_UNKNOWN_REQ]
  "No proxied device",                      ///< [BS_ECODE_NO_DEV]
  "No interface module",                    ///< [BS_ECODE_NO_MOD]
  "Bad interface module",                   ///< [BS_ECODE_BAD_MOD]
  "No resources available",                 ///< [BS_ECODE_NO_RSRC]
  "Resource busy",                          ///< [BS_ECODE_BUSY]
  "Operation timed out",                    ///< [BS_ECODE_TIMEDOUT]
  "Cannot execute",                         ///< [BS_ECODE_NO_EXEC]
  "Connection failed",                      ///< [BS_ECODE_SERVER_CONN_FAIL]
  "Connection denied",                      ///< [BS_ECODE_SERVER_CONN_DENY]
  "Bad or unknown client",                  ///< [BS_ECODE_SERVER_BAD_CLIENT]
  "System error",                           ///< [BS_ECODE_SYS]
  "Internal error",                         ///< [BS_ECODE_INTERNAL]
  "Invalid error code"                      ///< [BS_ECODE_BADEC]
};


// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Get the error string describing the \h_botsense error code.
 *
 * The absolute value of the error code is taken prior retrieving the string.
 * An unknown or out-of-range error code will be mapped to
 * \ref BS_ECODE_BADEC.
 *
 * \param ecode \h_botsense error code.
 *
 * \return Returns the appropriate error code string.
 */
const char *bsStrError(int ecode)
{
  if( ecode < 0 )
  {
    ecode = -ecode;
  }

  if( ecode >= arraysize(bsEcodeStrTbl) )
  {
    ecode = BS_ECODE_BADEC;
  }
  return bsEcodeStrTbl[ecode];
}
