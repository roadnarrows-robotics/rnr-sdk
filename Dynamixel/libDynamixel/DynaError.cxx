////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   libDynamixel
//
// File:      DynaError.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-03-13 13:28:02 -0600 (Fri, 13 Mar 2015) $
 * $Rev: 3890 $
 *
 * \brief Error and logging handling routines.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2011-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
/*
 * @EulaBegin@
 * 
 * Unless otherwise stated explicitly, all materials contained are copyrighted
 * and may not be used without RoadNarrows LLC's written consent,
 * except as provided in these terms and conditions or in the copyright
 * notice (documents and software) or other proprietary notice provided with
 * the relevant materials.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY 
 * MEMBERS/EMPLOYEES/CONTRACTORS OF ROADNARROWS OR DISTRIBUTORS OF THIS SOFTWARE
 * BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
 * DOCUMENTATION, EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHORS AND  ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <string.h>

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/new.h"
#include "rnr/units.h"

#include "Dynamixel/dxl/dxl.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaError.h"
#include "Dynamixel/DynaComm.h"

#include "DynaLibInternal.h"

using namespace std;


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Package Dynamixel Error Code String Table.
 *
 * Table is indexed by error codes (see \ref dyna_ecodes). Keep
 * in sync.
 */
static const char *DynaEcodeStrTbl[] =
{
  "Ok",                                     ///< [DYNA_OK]

  "Error",                                  ///< [DYNA_ECODE_GEN]
  "System error",                           ///< [DYNA_ECODE_SYS]
  "Internal error",                         ///< [DYNA_ECODE_INTERNAL]
  "Invalid error code",                     ///< [DYNA_ECODE_BADEC]
  "Invalid units",                          ///< [DYNA_ECODE_BAD_UNITS]
  "Invalid value",                          ///< [DYNA_ECODE_BAD_VAL]
  "No servo",                               ///< [DYNA_ECODE_NO_SERVO]
  "Serial device error",                    ///< [DYNA_ECODE_BAD_DEV]
  "Not open",                               ///< [DYNA_ECODE_BADF]
  "Communication error",                    ///< [DYNA_ECODE_ECOMM]
  "Transmit packet failure",                ///< [DYNA_ECODE_TX_FAIL]
  "Receive packet failure",                 ///< [DYNA_ECODE_RX_FAIL]
  "Transmit packet error",                  ///< [DYNA_ECODE_TX_ERROR]
  "Receive packet time out",                ///< [DYNA_ECODE_RX_TIMEOUT]
  "Received corrupted packet",              ///< [DYNA_ECODE_RX_BAD_PKT]
  "Servo errored condition",                ///< [DYNA_ECODE_ESERVO]
  "Resource not available",                 ///< [DYNA_ECODE_RSRC]
  "feature/function no supported",          ///< [DYNA_ECODE_NOT_SUPP]
  "Linked servos error",                    ///< [DYNA_ECODE_LINKED]
  "Operation not permitted on slave servo", ///< [DYNA_ECODE_SLAVE]
  "BotSense proxy error",                   ///< [DYNA_ECODE_BOTSENSE]
  "Shell parse error",                      ///< [DYNA_ECODE_PARSE]
  "Shell run-time error",                   ///< [DYNA_ECODE_RUNTIME]
  "Cannot execute"                          ///< [DYNA_ECODE_NOEXEC]
};


// ---------------------------------------------------------------------------
// Internal Interface 
// ---------------------------------------------------------------------------

int DynaMapDxlToEcode(int nDxlError)
{
  switch( nDxlError )
  {
    case DXL_COMM_TXFAIL:
      return DYNA_ECODE_TX_FAIL;
    case DXL_COMM_RXFAIL:
      return DYNA_ECODE_RX_FAIL;
    case DXL_COMM_TXERROR:
      return DYNA_ECODE_TX_ERROR;
    case DXL_COMM_RXTIMEOUT:
      return DYNA_ECODE_RX_TIMEOUT;
    case DXL_COMM_RXCORRUPT:
      return DYNA_ECODE_RX_BAD_PKT;
    default:
      return DYNA_ECODE_ECOMM;
  }
}


// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

const char *DynaStrError(int ecode)
{
  if( ecode < 0 )
  {
    ecode = -ecode;
  }

  if( ecode >= arraysize(DynaEcodeStrTbl) )
  {
    ecode = DYNA_ECODE_BADEC;
  }
  return DynaEcodeStrTbl[ecode];
}

void DynaPrintBuf(FILE       *fp,
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

void DynaLogServoAlarms(int nServoId, uint_t uAlarms)
{
  FILE   *fp;
  string  strAlarms;

  fp = LOG_GET_LOGFP();

  fprintf(fp, "%sDiag%d: ", LOG_PREFACE, LOG_GET_THRESHOLD()-1);
  fprintf(fp, "Servo %d alarms: ", nServoId);

  if( uAlarms == DYNA_ALARM_NONE )
  {
    fprintf(fp, "no alarms;\n");
    return;
  }

  else
  {
    strAlarms = DynaComm::GetAlarmsString(uAlarms);
    fprintf(fp, "%s\n", strAlarms.c_str());
  }
}

void DynaLogBuf(const char *sPreface,
                byte_t      buf[],
                size_t      uCount,
                const char *sFmt)
{
  FILE   *fp;   // log file pointer
  size_t  i;    // working index

  fp = LOG_GET_LOGFP();

  fprintf(fp, "%sDiag%d: %s", LOG_PREFACE, LOG_GET_THRESHOLD()-1, sPreface);
  for(i=0; i<uCount; ++i)
  {
    if( (i % 16) == 0 )
    {
      fprintf(fp, "\n");
    }
    fprintf(fp, sFmt, buf[i]);
  }
  fprintf(fp, "\n");
}

#endif // LOG
