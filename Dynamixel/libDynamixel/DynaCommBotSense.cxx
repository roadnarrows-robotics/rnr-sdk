////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   libDynamixel
//
// File:      DynaCommBotSense.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-03-04 12:38:36 -0700 (Wed, 04 Mar 2015) $
 * $Rev: 3873 $
 *
 * \brief RoadNarrows Dynamixel bus communication over BotSense IP proxy
 * interface class.
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
#include <ctype.h>
#include <pthread.h>

#include "rnr/rnrconfig.h"
#include "rnr/uri.h"
#include "rnr/shm.h"
#include "rnr/log.h"

#include "botsense/BotSense.h"
#include "botsense/libBotSense.h"
#include "botsense/bsDyna.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaError.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaCommBotSense.h"

#include "DynaLibInternal.h"


// ---------------------------------------------------------------------------
// Private Interface
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// DynaCommBotSense Class
// ---------------------------------------------------------------------------

DynaCommBotSense::DynaCommBotSense() : DynaComm()
{
  m_sSerialDevName    = NULL;
  m_sBsProxyHostName  = newstr("localhost");
  m_nBsProxyIPPort    = BSPROXY_LISTEN_PORT_DFT;
  m_pBsClient         = bsClientNew("Dynamixel");
  m_hndBsVConn        = BSPROXY_VCONN_UNDEF;
  m_bBsTrace          = false;
}

DynaCommBotSense::DynaCommBotSense(const char *sSerialDevName,
                                   int         nBaudRate,
                                   const char *sBsProxyHostName,
                                   int         nBsProxyIPPort) : DynaComm()
{
  Uri_T uri;

  // fill in URI structure
  uri.m_sScheme   = (char *)BSPROXY_URI_SCHEME;
  uri.m_sUserInfo = NULL;
  uri.m_sHostName = (char *)sBsProxyHostName;
  uri.m_nPortNum  = nBsProxyIPPort;
  uri.m_sPath     = (char *)sSerialDevName;
  uri.m_sQuery    = NULL;

  m_sDevUri           = UriStrNew(&uri);
  m_nBaudRate         = nBaudRate;

  m_sSerialDevName    = newstr(sSerialDevName);
  m_sBsProxyHostName  = newstr(sBsProxyHostName);
  m_nBsProxyIPPort    = nBsProxyIPPort;
  m_pBsClient         = bsClientNew("Dynamixel");
  m_hndBsVConn        = BSPROXY_VCONN_UNDEF;
  m_bBsTrace          = false;

  Open();
}

DynaCommBotSense::~DynaCommBotSense()
{
  Close();

  bsClientDelete(m_pBsClient);

  if( m_sBsProxyHostName != NULL )
  {
    delete[] m_sBsProxyHostName;
  }

  if( m_sSerialDevName != NULL )
  {
    delete[] m_sSerialDevName;
  }
}

int DynaCommBotSense::Open(const char *sSerialDevName,
                           int         nBaudRate,
                           const char *sBsProxyHostName,
                           int         nBsProxyIPPort)
{
  // close if open
  Close();

  if( m_sSerialDevName != NULL )
  {
    delete[] m_sSerialDevName;
  }

  if( m_sBsProxyHostName != NULL )
  {
    delete[] m_sBsProxyHostName;
  }

  m_sSerialDevName    = newstr(sSerialDevName);
  m_nBaudRate         = nBaudRate;
  m_sBsProxyHostName  = newstr(sBsProxyHostName);
  m_nBsProxyIPPort    = nBsProxyIPPort;
  m_hndBsVConn        = BSPROXY_VCONN_UNDEF;

  // (re)open
  return Open();
}

int DynaCommBotSense::Open()
{
  int nSerialIndex;
  int nBaudNum;
  int rc;

  // close if open
  Close();

  // check proxied serial device name
  if( (m_sSerialDevName == NULL) || (*m_sSerialDevName == 0) )
  {
    rc = -DYNA_ECODE_BAD_VAL;
    DYNA_LOG_ERROR(rc, "Unspecified serial device.");
  }

  // check proxy server host name
  else if( (m_sBsProxyHostName == NULL) || (*m_sBsProxyHostName == 0) )
  {
    rc = -DYNA_ECODE_BOTSENSE;
    DYNA_LOG_ERROR(rc, "Unspecified BotSense server host name.");
  }

  // connect to bsProxy server
  else if((rc = 
        bsServerConnect(m_pBsClient, m_sBsProxyHostName, m_nBsProxyIPPort)) < 0)
  {
    LOGERROR("bsProxy server %s:%d: %s.",
        m_sBsProxyHostName, m_nBsProxyIPPort, bsStrError(rc));
    rc = -DYNA_ECODE_BOTSENSE;
  }

  // open proxied serial device
  else if( (m_hndBsVConn = bsDynaOpen(m_pBsClient,
                                      m_sSerialDevName,
                                      m_nBaudRate,
                                      m_bBsTrace)) < 0 )
  {
    rc = m_hndBsVConn;
    m_hndBsVConn = BSPROXY_VCONN_UNDEF;
    DYNA_LOG_ERROR(rc, "Failed to open %s@%d on bsProxy server %s:%d.",
        m_sSerialDevName, m_nBaudRate, m_sBsProxyHostName, m_nBsProxyIPPort);
  }

  // success
  else
  {
    m_bIsOpen = true;
    rc = DYNA_OK;
    LOGDIAG3("Dynamixel bus communication opened on %s@%d "
            "via bsProxy server %s:%d.",
        m_sSerialDevName, m_nBaudRate, m_sBsProxyHostName, m_nBsProxyIPPort);
  }

  return rc;
}

int DynaCommBotSense::Close()
{
  if( m_bIsOpen )
  {
    bsDynaClose(m_pBsClient, m_hndBsVConn);
    bsServerDisconnect(m_pBsClient);
    m_hndBsVConn = BSPROXY_VCONN_UNDEF;
    m_bIsOpen    = false;
  }
  return DYNA_OK;
}

int DynaCommBotSense::SetBaudRate(int nNewBaudRate)
{
  int   rc;   // return code

  if( m_bIsOpen )
  {
    rc = bsDynaSetBaudRate(m_pBsClient, m_hndBsVConn, nNewBaudRate);

    DYNA_TRY_RC(rc, "SetBaudRate(%d)", nNewBaudRate);
  }

  m_nBaudRate = nNewBaudRate;

  return DYNA_OK;
}

int DynaCommBotSense::SetHalfDuplexCtl(int                nSignal,
                                       HalfDuplexTxFunc_T fnEnableTx,
                                       HalfDuplexRxFunc_T fnEnableRx)
{
  int   rc;   // return code

  if( m_bIsOpen )
  {
    // Note: do not know how got generalize this for non-builtin signal support.
    rc = bsDynaSetHalfDuplexCtl(m_pBsClient, m_hndBsVConn, nSignal);

    DYNA_TRY_RC(rc, "SetHalfDuplexCtl(%d)", nSignal);
  }

  return DYNA_OK;
}

int DynaCommBotSense::SetMsgTracing(bool bEnabled)
{
  m_bBsTrace = bEnabled;

  return DYNA_OK;
}

int DynaCommBotSense::Read8(int nServoId, uint_t uAddr, byte_t *pVal)
{
  int     rc;

  DYNA_TRY_COMM(*this);

  shm_mutex_lock(&m_mutexComm);

  rc = bsDynaRead8(m_pBsClient, m_hndBsVConn,
                    nServoId, uAddr, pVal, &m_uAlarms);

  shm_mutex_unlock(&m_mutexComm);

  if( rc == BS_OK )
  {
    rc = DYNA_OK;
  }
  else
  {
    rc = -DYNA_ECODE_BOTSENSE;
    DYNA_LOG_ERROR(rc, "Read8(%d, 0x%02x)", nServoId, uAddr);
  }

  return rc;
}

int DynaCommBotSense::Write8(int nServoId, uint_t uAddr, byte_t byVal)
{
  int   rc;

  DYNA_TRY_COMM(*this);

  shm_mutex_lock(&m_mutexComm);

  rc = bsDynaWrite8(m_pBsClient, m_hndBsVConn,
                    nServoId, uAddr, byVal, &m_uAlarms);

  shm_mutex_unlock(&m_mutexComm);
  
  if( rc == BS_OK )
  {
    rc = DYNA_OK;
  }
  else
  {
    rc = -DYNA_ECODE_BOTSENSE;
    DYNA_LOG_ERROR(rc, "Write8(%d, 0x%02x, 0x%02x)", nServoId, uAddr, byVal);
  }

  return rc;
}

int DynaCommBotSense::Read16(int nServoId, uint_t uAddr, ushort_t *pVal)
{
  int   rc;

  DYNA_TRY_COMM(*this);

  shm_mutex_lock(&m_mutexComm);

  rc = bsDynaRead16(m_pBsClient, m_hndBsVConn,
                        nServoId, uAddr, pVal, &m_uAlarms);

  shm_mutex_unlock(&m_mutexComm);

  if( rc == BS_OK )
  {
    rc = DYNA_OK;
  }
  else
  {
    rc = -DYNA_ECODE_BOTSENSE;
    DYNA_LOG_ERROR(rc, "Read16(%d, 0x%02x)", nServoId, uAddr);
  }

  return rc;
}

int DynaCommBotSense::Write16(int nServoId, uint_t uAddr, ushort_t huVal)
{
  int   rc;

  DYNA_TRY_COMM(*this);

  shm_mutex_lock(&m_mutexComm);

  rc = bsDynaWrite16(m_pBsClient, m_hndBsVConn,
                        nServoId, uAddr, huVal, &m_uAlarms);

  shm_mutex_unlock(&m_mutexComm);

  if( rc == BS_OK )
  {
    rc = DYNA_OK;
  }
  else
  {
    rc = -DYNA_ECODE_BOTSENSE;
    DYNA_LOG_ERROR(rc, "Write16(%d, 0x%02x, 0x%04x)", nServoId, uAddr, huVal);
  }

  return rc;
}

int DynaCommBotSense::SyncWrite(uint_t                uAddr,
                                uint_t                uValSize,
                                DynaSyncWriteTuple_T  tuples[],
                                uint_t                uCount)
{
  int rc;

  DYNA_TRY_COMM(*this);

  shm_mutex_lock(&m_mutexComm);

  rc = bsDynaSyncWrite(m_pBsClient, m_hndBsVConn,
                          uAddr, uValSize, tuples, uCount);

  shm_mutex_unlock(&m_mutexComm);
  
  if( rc == BS_OK )
  {
    rc = DYNA_OK;
  }
  else
  {
    rc = -DYNA_ECODE_BOTSENSE;
    DYNA_LOG_ERROR(rc, "SyncWrite(0x%02x, %d, tuples, %d)",
        uAddr, uValSize, uCount);
  }

  return rc;
}

bool DynaCommBotSense::Ping(int nServoId)
{
  bool_t  bPong;
  int     rc;

  DYNA_TRY_COMM(*this);

  shm_mutex_lock(&m_mutexComm);

  rc = bsDynaPing(m_pBsClient, m_hndBsVConn, nServoId, &bPong);

  shm_mutex_unlock(&m_mutexComm);

  if( rc == BS_OK )
  {
    return bPong? true: false;
  }
  else
  {
    rc = -DYNA_ECODE_BOTSENSE;
    DYNA_LOG_ERROR(rc, "Ping(%d)", nServoId);
    return false;
  }
}

int DynaCommBotSense::Reset(int nServoId)
{
  int rc;

  DYNA_TRY_COMM(*this);

  shm_mutex_lock(&m_mutexComm);

  rc = bsDynaReset(m_pBsClient, m_hndBsVConn, nServoId);

  shm_mutex_unlock(&m_mutexComm);
  
  if( rc == BS_OK )
  {
    rc = DYNA_OK;
  }
  else
  {
    rc = -DYNA_ECODE_BOTSENSE;
    DYNA_LOG_ERROR(rc, "Reset(%d)", nServoId);
  }

  return rc;
}

