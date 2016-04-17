////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   libDynamixel
//
// File:      DynaComm.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
 * $Rev: 3845 $
 *
 * \brief RoadNarrows Dynamixel bus communications abstract class base class.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011-2015.  RoadNarrows LLC.
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
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

#include <unistd.h>
#include <string.h>
#include <pthread.h>

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/uri.h"
#include "rnr/shm.h"
#include "rnr/log.h"

#include "botsense/BotSense.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaError.h"

#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaCommBotSense.h"
#include "Dynamixel/DynaCommSerial.h"

#include "DynaLibInternal.h"

using namespace std;


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Dynamixel Bus Communication Status String Table.
 */
static const char *DynaBusStatusStrTbl[] =
{
  "Transmit packet success",                ///< [COMM_TXSUCCESS]
  "Receive packet success",                 ///< [COMM_RXSUCCESS]
  "Transmit packet failure",                ///< [COMM_TXFAIL]
  "Receive packet failure",                 ///< [COMM_RXFAIL]
  "Transmit error",                         ///< [COMM_TXERROR]
  "Waiting to receive packet",              ///< [COMM_RXWAITING]
  "Timed out waiting to receive packet",    ///< [COMM_RXTIMEOUT]
  "Received packet corrupted",              ///< [COMM_RXCORRUPT]
  "Unknown communication status"            ///< [?]
};

/*!
 * \brief Dynamixel Servo Alarms String Table.
 */
static const char *DynaServoAlarmStrTbl[] =
{
  "Input voltage out of range",             ///< DYNA_ALARM_VOLTAGE
  "Angle position out of range",            ///< DYNA_ALARM_ANGLE
  "Over-temperature",                       ///< DYNA_ALARM_TEMP
  "Command out of range",                   ///< DYNA_ALARM_CMD_RANGE
  "Checksum",                               ///< DYNA_ALARM_CHECKSUM
  "Load out of torque range",               ///< DYNA_ALARM_LOAD
  "Instruction code"                        ///< DYNA_ALARM_INSTRUCTION
};

/*!
 * \brief Dynamixel Servo Alarms Short String Table.
 */
static const char *DynaServoAlarmShortStrTbl[] =
{
  "VOLT",             ///< DYNA_ALARM_VOLTAGE
  "POS",              ///< DYNA_ALARM_ANGLE
  "TEMP",             ///< DYNA_ALARM_TEMP
  "CMD",              ///< DYNA_ALARM_CMD_RANGE
  "CS",               ///< DYNA_ALARM_CHECKSUM
  "TORQ",             ///< DYNA_ALARM_LOAD
  "INST"              ///< DYNA_ALARM_INSTRUCTION
};

/*!
 * \brief Baud rate to baud number enumeration map.
 */
static const int DynaBaudMap[DYNA_BAUDNUM_NUMOF][2] =
{
  {1000000,   DYNA_BAUDNUM_1000000},
  {500000,    DYNA_BAUDNUM_500000},
  {400000,    DYNA_BAUDNUM_400000},
  {250000,    DYNA_BAUDNUM_250000},
  {200000,    DYNA_BAUDNUM_200000},
  {115200,    DYNA_BAUDNUM_115200},
  {57600,     DYNA_BAUDNUM_57600},
  {19200,     DYNA_BAUDNUM_19200},
  {9600,      DYNA_BAUDNUM_9600},
  {2250000,   DYNA_BAUDNUM_EXT_2250000},
  {2500000,   DYNA_BAUDNUM_EXT_2500000},
  {3000000,   DYNA_BAUDNUM_EXT_3000000}
};


// ---------------------------------------------------------------------------
// DynaComm Class
// ---------------------------------------------------------------------------

/*! \brief Shared memory mutex key: roaddyna */
const key_t DynaComm::ShmKey = 0x70add12a;

DynaComm::DynaComm()
{ 
  m_sDevUri         = NULL;
  m_nBaudRate       = 0;
  m_bIsOpen         = false;
  m_uAlarms         = DYNA_ALARM_NONE;

  shm_mutex_init(ShmKey, &m_mutexComm);
}

DynaComm::DynaComm(const char *sUri, int nBaudRate)
{ 
  m_sDevUri     = newstr(sUri);
  m_nBaudRate   = nBaudRate;
  m_bIsOpen     = false;
  m_uAlarms     = DYNA_ALARM_NONE;

  shm_mutex_init(ShmKey, &m_mutexComm);
}

DynaComm::~DynaComm()
{
  if( m_sDevUri != NULL )
  {
    delete[] m_sDevUri;
  }

  shm_mutex_destroy(&m_mutexComm);
}

DynaComm *DynaComm::New(const char *sUri, int nBaudRate)
{
  Uri_T      *pUri;         // parsed URI
  const char *sHostName;    // parsed/default hostname
  int         nPortNum;     // parsed/default IP port number
  DynaComm   *pDynaComm;    // new derived Dynamixel bus communication object
  int         rc;           // return code

  pUri      = NULL;
  pDynaComm = NULL;

  //
  // No uri string.
  //
  if( sUri == NULL )
  {
    DYNA_LOG_ERROR(DYNA_ECODE_BAD_VAL, "No URI specified.");
  }

  //
  // Bad baud rate.
  //
  else if( (rc = DynaComm::BaudRateToNum(nBaudRate)) < 0 )
  {
    DYNA_LOG_ERROR(rc, "%d: Unsupported baudrate.", nBaudRate);
  }

  //
  // Bad uri parse.
  //
  else if( (pUri = UriParseNew(sUri)) == NULL )
  {
    DYNA_LOG_ERROR(DYNA_ECODE_BAD_VAL, "Failed to parse URI \"%s\".", sUri);
  }

  //
  // Required device path in uri not specified.
  //
  else if( pUri->m_sPath == NULL )
  {
    DYNA_LOG_ERROR(DYNA_ECODE_BAD_VAL,
        "URI \"%s\" does not specify device path.", sUri);
  }

  //
  // Direct connect serial Dynamixel bus.
  //
  else if( pUri->m_sScheme == NULL )
  {
    pDynaComm = new DynaCommSerial(pUri->m_sPath, nBaudRate);
  }

  //
  // BotSense proxied Dynamixel bus.
  //
  else if( !strcasecmp(pUri->m_sScheme, BSPROXY_URI_SCHEME) )
  {
    sHostName = pUri->m_sHostName;
    nPortNum  = pUri->m_nPortNum;

    if( sHostName == NULL )
    {
      sHostName = BSPROXY_URI_HOSTNAME_DFT; 
    }

    if( nPortNum == URI_PORT_NONE )
    {
      nPortNum = BSPROXY_LISTEN_PORT_DFT;
    }

    pDynaComm = new DynaCommBotSense(pUri->m_sPath, nBaudRate,
                                      sHostName, nPortNum);
  }

  //
  // File scheme is also direct connect, but only locally.
  //
  else if( !strcasecmp(pUri->m_sScheme, URI_SCHEME_FILE) )
  {
    if( (pUri->m_sHostName == NULL) || 
        !strcasecmp(pUri->m_sHostName, URI_LOCAL_HOST) )
    {
      pDynaComm = new DynaCommSerial(pUri->m_sPath, nBaudRate);
    }
    else
    {
      DYNA_LOG_ERROR(DYNA_ECODE_BAD_VAL,
        "URI \"%s\" remote file device access not supported - "
        "use botsense proxy services.", sUri);
    }
  }

  //
  // Bad scheme.
  //
  else
  {
    DYNA_LOG_ERROR(DYNA_ECODE_BAD_VAL, "URI \"%s\" unknown scheme.", sUri);
  }

  if( pUri != NULL )
  {
    UriDelete(pUri);
  }

  //
  // Final checks
  //
  if( (pDynaComm != NULL) && !pDynaComm->IsOpen() )
  {
    DYNA_LOG_ERROR(DYNA_ECODE_BAD_DEV,
        "Failed to open the device \"%s\" at %d baud.", sUri, nBaudRate);
    delete pDynaComm;
    pDynaComm = NULL;
  }

  return pDynaComm;
}

const char *DynaComm::GetBusStatusString(uint_t uBusStatus)
{
  if( uBusStatus < 0 )
  {
    uBusStatus = -uBusStatus;
  }

  if( uBusStatus >= arraysize(DynaBusStatusStrTbl) )
  {
    uBusStatus = arraysize(DynaBusStatusStrTbl)  - 1;
  }
  return DynaBusStatusStrTbl[uBusStatus];
}

string DynaComm::GetAlarmsString(const uint_t  uAlarms,
                                 const string &strSep)
{
  string  strAlarms;
  size_t  i;
  uint_t  bit;

  if( uAlarms == DYNA_ALARM_NONE )
  {
    return "OK";
  }

  for(i=0, bit=0x01; i<arraysize(DynaServoAlarmStrTbl); ++i, bit<<=1)
  {
    if( !strAlarms.empty() )
    {
      strAlarms += strSep;
    }

    if( bit & uAlarms )
    {
      strAlarms += DynaServoAlarmStrTbl[i];
    }
  }

  return strAlarms;
}

string DynaComm::GetAlarmsShortString(const uint_t  uAlarms,
                                      const string &strSep)
{
  string  strAlarms;
  size_t  i;
  uint_t  bit;

  if( uAlarms == DYNA_ALARM_NONE )
  {
    return "OK";
  }

  for(i=0, bit=0x01; i<arraysize(DynaServoAlarmShortStrTbl); ++i, bit<<=1)
  {
    if( !strAlarms.empty() )
    {
      strAlarms += strSep;
    }

    if( bit & uAlarms )
    {
      strAlarms += DynaServoAlarmShortStrTbl[i];
    }
  }

  return strAlarms;
}

int DynaComm::BaudRateToNum(int nBaudRate)
{
  int   i;

  for(i=0; i<arraysize(DynaBaudMap); ++i)
  {
    if( DynaBaudMap[i][0] == nBaudRate )
    {
      return DynaBaudMap[i][1];
    }
  }
  return -DYNA_ECODE_BAD_VAL;
}

int DynaComm::BaudNumToRate(int nBaudNum)
{
  int   i;

  for(i=0; i<arraysize(DynaBaudMap); ++i)
  {
    if( DynaBaudMap[i][1] == nBaudNum )
    {
      return DynaBaudMap[i][0];
    }
  }
  return -DYNA_ECODE_BAD_VAL;
}

int DynaComm::BaudRateAt(int nIndex)
{
  if( (nIndex >= 0) && (nIndex < arraysize(DynaBaudMap)) )
  {
    return DynaBaudMap[nIndex][0];
  }
  else
  {
    return 0;
  }
}

int DynaComm::BaudNumAt(int nIndex)
{
  if( (nIndex >= 0) && (nIndex < arraysize(DynaBaudMap)) )
  {
    return DynaBaudMap[nIndex][1];
  }
  else
  {
    return 0;
  }
}

int DynaComm::vSyncWrite(uint_t uAddr, uint_t uValSize, uint_t uCount, ...)
{
  DynaSyncWriteTuple_T  tuples[DYNA_ID_NUMOF];
  va_list               ap;
  int                   i;

  va_start(ap, uCount);

  for(i=0; i<uCount && i<DYNA_ID_NUMOF; ++i)
  {
    tuples[i].m_nServoId = va_arg(ap, int);
    tuples[i].m_uVal     = va_arg(ap, uint_t);
  }

  va_end(ap);

  return SyncWrite(uAddr, uValSize, tuples, (uint_t)i);
}
