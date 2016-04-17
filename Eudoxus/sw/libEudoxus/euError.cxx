////////////////////////////////////////////////////////////////////////////////
//
// Package:   Eudoxus
//
// Library:   libEudoxus
//
// File:      euError.c
//
/*! \file
 *
 * $LastChangedDate: 2015-11-30 15:41:20 -0700 (Mon, 30 Nov 2015) $
 * $Rev: 4226 $
 *
 * \brief Error and logging handling routines.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2012-2015.  RoadNarrows LLC
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
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
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#include "Eudoxus/euConf.h"

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <string.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Eudoxus/Eudoxus.h"
#include "Eudoxus/euUtils.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

namespace eu
{
  /*!
   * \brief Eudoxus Error Code String Table.
   *
   * Table is indexed by Eudoxus error codes (see \ref eu_ecodes). Keep
   * in sync.
   */
  static const char *euEcodeStrTbl[] =
  {
    "Ok",                                     ///< [EU_OK]
  
    "Error",                                  ///< [EU_ECODE_GEN]
    "System error",                           ///< [EU_ECODE_SYS]
    "Internal error",                         ///< [EU_ECODE_INTERNAL]
    "Invalid value",                          ///< [EU_ECODE_BAD_VAL]
    "Too big",                                ///< [EU_ECODE_MSG_TOO_BIG]
    "Too small",                              ///< [EU_ECODE_BUF_TOO_SMALL]
    "Out of range",                           ///< [EU_ECODE_RANGE]
    "Invalid operation",                      ///< [EU_ECODE_BAD_OP]
    "Operation timed out",                    ///< [EU_ECODE_TIMEDOUT]
    "Device not found",                       ///< [EU_ECODE_NO_DEV]
    "No resources available",                 ///< [EU_ECODE_NO_RSRC]
    "Resource busy",                          ///< [EU_ECODE_BUSY]
    "Cannot execute",                         ///< [EU_ECODE_NO_EXEC]
    "No premissions",                         ///< [EU_ECODE_PERM]
    "Formatting error",                       ///< [EU_ECODE_BAD_FMT]
    "OpenNI error",                           ///< [EU_ECODE_NI]
    "PCL error",                              ///< [EU_ECODE_pcl]
    "GStreamer error",                        ///< [EU_ECODE_gst]
  
    "Invalid error code"                      ///< [EU_ECODE_BADEC]
  };
} // namespace eu


// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

namespace eu
{
  /*!
   * \brief Get the error string describing the Eudoxus error code.
   *
   * The absolute value of the error code is taken prior retrieving the string.
   * An unknown or out-of-range error code will be mapped to
   * \ref EU_ECODE_BADEC.
   *
   * \param ecode Eudoxus error code.
   *
   * \return Returns the appropriate error code string.
   */
  const char *getStrError(int ecode)
  {
    if( ecode < 0 )
    {
      ecode = -ecode;
    }
  
    if( ecode >= arraysize(euEcodeStrTbl) )
    {
      ecode = EU_ECODE_BADEC;
    }
    return euEcodeStrTbl[ecode];
  }

} // namespace eu
