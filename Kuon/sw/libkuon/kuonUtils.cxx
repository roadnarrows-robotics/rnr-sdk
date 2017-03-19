////////////////////////////////////////////////////////////////////////////////
//
// Package:   Kuon
//
// Library:   libkuon
//
// File:      kuonUtils.cxx
//
//
/*! \file
 *
 * $LastChangedDate: 2014-04-04 17:05:03 -0600 (Fri, 04 Apr 2014) $
 * $Rev: 3629 $
 *
 * \brief Kuon utilities.
 *
 * \author Daniel Packard (daniel@roadnarrows.com)
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2012-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
/*
 * @EulaBegin@
 * 
 * Permission is hereby granted, without written agreement and without
 * license or royalty fees, to use, copy, modify, and distribute this
 * software and its documentation for any purpose, provided that
 * (1) The above copyright notice and the following two paragraphs
 * appear in all copies of the source code and (2) redistributions
 * including binaries reproduces these notices in the supporting
 * documentation.   Substantial modifications to this software may be
 * copyrighted by their authors and need not follow the licensing terms
 * described here, provided that the new terms are clearly indicated in
 * all files where they apply.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
 * OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////


#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <libgen.h>
#include <stdio.h>
#include <errno.h>

#include <string>
#include <sstream>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Kuon/kuon.h"
#include "Kuon/kuonUtils.h"

using namespace std;


//------------------------------------------------------------------------------
// Private Interface
//------------------------------------------------------------------------------

/*!
 * \ingroup libkuon
 * \brief \h_kuon Error Code String Table.
 *
 * Table is indexed by \h_kuon error codes (see \ref kuon_ecodes). Keep
 * in sync.
 */
static const char *EcodeStrTbl[] =
{
  "Ok",                                     ///< [KUON_OK]

  "Error",                                  ///< [KUON_ECODE_GEN]
  "System error",                           ///< [KUON_ECODE_SYS]
  "Internal error",                         ///< [KUON_ECODE_INTERNAL]
  "Bad value",                              ///< [KUON_ECODE_BAD_VAL]
  "Too big",                                ///< [KUON_ECODE_TOO_BIG]
  "Too small",                              ///< [KUON_ECODE_TOO_SMALL]
  "Value out-of-range",                     ///< [KUON_ECODE_RANGE]
  "Invalid operation",                      ///< [KUON_ECODE_BAD_OP]
  "Operation timed out",                    ///< [KUON_ECODE_TIMEDOUT]
  "Device not found",                       ///< [KUON_ECODE_NO_DEV]
  "No resource available",                  ///< [KUON_ECODE_NO_RSRC]
  "Resource busy",                          ///< [KUON_ECODE_BUSY]
  "Cannot execute",                         ///< [KUON_ECODE_NO_EXEC]
  "Permissions denied",                     ///< [KUON_ECODE_PERM]
  "Motor error",                            ///< [KUON_ECODE_MOTOR]
  "Motor controller error",                 ///< [KUON_ECODE_MOT_CTLR]
  "Battery error",                          ///< [KUON_ECODE_BATT]
  "Bad format",                             ///< [KUON_ECODE_FORMAT]
  "BotSense error",                         ///< [KUON_ECODE_BOTSENSE]
  "File not found",                         ///< [KUON_ECODE_NO_FILE]
  "XML error",                              ///< [KUON_ECODE_XML]
  "Robot is in an alarmed state",           ///< [KUON_ECODE_ALARMED]
  "Operation interrupted",                  ///< [KUON_ECODE_INTR]
  "Robotic link(s) movement obstructed",    ///< [KUON_ECODE_COLLISION]
  "Robot emergency stopped",                ///< [KUON_ECODE_ESTOP]

  "Invalid error code"                      ///< [KUON_ECODE_BADEC]
};


//------------------------------------------------------------------------------
// Public Interface
//------------------------------------------------------------------------------

const char *kuon::getStrError(const int ecode)
{
  int ec = ecode >= 0 ? ecode : -ecode;

  if( ec >= arraysize(EcodeStrTbl) )
  {
    ec = KUON_ECODE_BADEC;
  }

  return EcodeStrTbl[ec];
}

uint_t kuon::strToVersion(const string &str)
{
  int   nMajor = 0;
  int   nMinor = 0;
  int   nRevision = 0;

  sscanf(str.c_str(), "%d.%d.%d", &nMajor, &nMinor, &nRevision);

  return KUON_VERSION(nMajor, nMinor, nRevision);
}

string kuon::getRealDeviceName(const string &strDevName)
{
  char    buf[MAX_PATH+1];
  ssize_t len;

  //
  // Symbolic link.
  //
  if( (len = readlink(strDevName.c_str(), buf, MAX_PATH)) > 0 )
  {
    buf[len] = 0;

    // absollute path
    if( buf[0] == '/' )
    {
      string strRealDevName(buf);
      return strRealDevName;
    }

    // relative path
    else
    {
      char          s[strDevName.size()+1];
      stringstream  ss;

      strcpy(s, strDevName.c_str());

      char *sDirName = dirname(s);

      ss << sDirName << "/" << buf;

      return ss.str();
    }
  }

  //
  // Real device.
  //
  else
  {
    return strDevName;
  }
}
