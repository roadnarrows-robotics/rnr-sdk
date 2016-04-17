////////////////////////////////////////////////////////////////////////////////
//
// Package:   CogniBoost
//
// Library:   libCogniBoost
//
// File:      cbLibDev.c
//
/*! \file
 *
 * $LastChangedDate: 2011-10-19 15:05:47 -0600 (Wed, 19 Oct 2011) $
 * $Rev: 1398 $
 *
 * \brief CogniBoost device I/O and attribute functions.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Brent Wilkins  (brent@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011.  RoadNarrows LLC.
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
#include <errno.h>
#include <pthread.h>
#include <limits.h>
#include <sys/select.h>
#include <sys/time.h>

#include "rnr/rnrconfig.h"
#include "rnr/new.h"
#include "rnr/log.h"
#include "rnr/netmsgs.h"
#include "rnr/serdev.h"
#include "rnr/sock.h"

#include "CogniBoost/CogniMem.h"
#include "CogniBoost/CogniBoost.h"
#include "CogniBoost/CogniBoostProto.h"
#include "CogniBoost/CogniBoostMsgs.h"
#include "CogniBoost/libCogniBoost.h"

#include "cbLibInternal.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

// ...........................................................................
// Defines and Types
// ...........................................................................

/*!
 * \brief Convenience macro to produce a buffer (offset, size) 2-tuple.
 *
 * The offset allows libCogniBoost to pack the header in front of the buffer
 * without doing any unnecessary copies, while the buffer size is decremented 
 * to account for the header bytes to be packed.
 *
 * \param buf   Buffer.
 */ 
#define CB_BUF_MSG(buf) \
  (buf)+CB_PKT_HDR_SIZE, sizeof(buf)-(size_t)CB_PKT_HDR_SIZE

/*!
 * \brief Convenience macro to produce msgname, msgid 2-tuple.
 *
 * \param msgid     Message id.
 */
#define CB_MSG_NAME_ID(msgid)   cbGetMsgName((msgid)), (msgid)

/*!
 * \brief Log packet send/receive event.
 *
 * \param hnd     CogniBoost device handle.
 * \param sEvent  Event description string.
 * \param pPktHdr Packet header.
 * \param uMsgId  Message Id.
 */
#ifdef LOG
#define CB_LOG_PKT(hnd, sEvent, pPktHdr, uMsgId) \
  do \
  { \
    if( LOGABLE(LOG_LEVEL_DIAG2) ) \
    { \
      cbLogPkt(hnd, sEvent, pPktHdr, uMsgId); \
    } \
  } while(0)
#else
#define CB_LOG_PKT(hnd, sEvent, pPktHdr, uMsgId)
#endif // LOG

//
// Forward Declarations
//
static void cbCacheForget(cbHnd_T hnd, cbTid_T uTid);

// ...........................................................................
// Helpful Utilities
// ...........................................................................

/*!
 * \brief Pull message id out of packed message buffer.
 *
 * \param bufMsg    Packed message buffer.
 * \param uMsgLen   Message length (bytes).
 *
 * \return
 * On success, returns the CogniBoost unique message id.\n
 * On failure, returns NMMSG_ID_NONE.
 */
static inline uint_t cbGetMsgId(byte_t bufMsg[], size_t uMsgLen)
{
  return (uint_t)nmGetITVMsgId(bufMsg, uMsgLen, CB_MSG_ENDIAN);
}

/*!
 * \brief Get the message name.
 *
 * \param uMsgId    Message id.
 *
 * \return
 * Returns message name if it can be determined. Otherwise returns "unknown".
 */
static inline const char *cbGetMsgName(uint_t uMsgId)
{
  static const char       *sUnknown = "unknown";
  const NMMsgDef_T        *pMsgDef;

  pMsgDef = cbLookupMsgDef((cbMsgId_T)uMsgId);
  return pMsgDef!=NULL? pMsgDef->m_sMsgName: sUnknown;
}

#ifdef LOG
/*!
 * \brief Log packet send/receive event.
 *
 * \param hnd     CogniBoost device handle.
 * \param sEvent  Event description string.
 * \param pPktHdr Packet header.
 * \param uMsgId  Message Id.
 */
static void cbLogPkt(cbHnd_T       hnd,
                     const char   *sEvent,
                     cbPktHdr_T   *pPktHdr,
                     uint_t        uMsgId)
{
  FILE        *fp;
  const char  *sMsgName;

  fp        = LOG_GET_LOGFP();
  sMsgName  = cbGetMsgName(uMsgId);

  fprintf(fp, "Device %d: %s: "
              "{magic=0x%02x, tid=%d, msglen=%d, msg=%s(msgid=%u)}",
              hnd->m_fd, sEvent,
              pPktHdr->m_hdrMagic, pPktHdr->m_hdrTid, pPktHdr->m_hdrMsgLen,
              CB_MSG_NAME_ID(uMsgId));
}
#endif // LOG


// ...........................................................................
// Device Data Functions
// ...........................................................................

/*!
 * \brief Create a new connected CogniBoost device data instance.
 *
 * \param sDeviceName   Serial device name.
 * \param nBaudRate     Serial baud rate.
 * \param fd            Open CogniBoost file descriptor.
 * \param bTrace        Do [not] trace messages.
 *
 * \return Returns pointer to new client on success. Else returns NULL.
 */
static cbDevice_T *cbDeviceNew(const char *sDeviceName,
                               int         nBaudRate,
                               int         fd,
                               bool_t      bTrace)
{
  cbDevice_T     *pDevice = NEW(cbDevice_T);
  size_t          i;

  pDevice->m_sDeviceName  = new_strdup(sDeviceName);
  pDevice->m_nBaudRate    = nBaudRate;
  pDevice->m_eConnType    = cbConnTypeDirect;
  pDevice->m_fd           = fd;
  pDevice->m_bTrace       = bTrace? true: false;
  pDevice->m_uTidCounter  = 0;

  pthread_mutex_init(&(pDevice->m_mutexTrans), NULL);
      
  for(i=0; i<arraysize(pDevice->m_tblTransCache); ++i)
  {
    pDevice->m_tblTransCache[i] = NULL;
  }

  memset(&(pDevice->m_cbIds), 0, sizeof(cbIdentities_T));

  return pDevice;
}

/*!
 * \brief Delete device data.
 *
 * \warning The client should disconnect prior to deletion.
 *
 * \param hnd       CogniBoost device handle.
 */
void cbDeviceDelete(cbHnd_T hnd)
{
  size_t i;

  if( hnd == CB_HND_NONE )
  {
    return;
  }

  for(i=0; i<arraysize(hnd->m_tblTransCache); ++i)
  {
    cbCacheForget(hnd, (cbTid_T)i);
  }

  pthread_mutex_destroy(&(hnd->m_mutexTrans));

  delete((void *)(hnd->m_sDeviceName));

  delete(hnd);
}


// ...........................................................................
// Transaction Mutual Exclusion Functions
// ...........................................................................

/*!
 * \brief Lock device mutual exclusion.
 *
 * \param hnd       CogniBoost device handle.
 */
static inline void cbLock(cbHnd_T hnd)
{
  int rc;

  if( (rc = pthread_mutex_lock(&(hnd->m_mutexTrans))) != 0 ) 
  { 
    errno = rc;
    CB_LOG_SYSERROR(hnd, CB_LIB_ECODE_SYS, "pthread_mutex_lock().");
  }
}

/*!
 * \brief Unlock device mutual exclusion.
 *
 * \param hnd       CogniBoost device handle.
 */
static inline void cbUnlock(cbHnd_T hnd)
{
  int rc;

  if( (rc = pthread_mutex_unlock(&(hnd->m_mutexTrans))) != 0 ) 
  { 
    errno = rc;
    CB_LOG_SYSERROR(hnd, CB_LIB_ECODE_SYS, "pthread_mutex_unlock().");
  }
}

/*!
 * \brief Try to lock device mutual exclusion.
 *
 * \param hnd       CogniBoost device handle.
 *
 * \return
 * Return true if lock was acquired. Otherwise false if already locked.
 */
static inline bool_t cbTryLock(cbHnd_T hnd)
{
  return pthread_mutex_trylock(&(hnd->m_mutexTrans)) == 0? true: false;
}


// ...........................................................................
// Transaction Caching Functions
// ...........................................................................

/*!
 * \brief Mark the start of a transaction in the transaction cache.
 *
 * A client can be multi-threaded, and so the responses from a CogniBoost server
 * may come out of order with respect to the client thread.
 *
 * \param hnd       CogniBoost device handle.
 * \param uTid      Transaction id.
 */
static void cbCacheMark(cbHnd_T hnd, cbTid_T uTid)
{
  cbTransInfo_T *pInfo;

  cbLock(hnd);

  pInfo = hnd->m_tblTransCache[uTid & CB_HDR_TID_MASK];

  if( pInfo != NULL )
  {
    delete(pInfo->m_pPktHdr);
    delete(pInfo->m_pBuf);
    delete(pInfo);
  }

  pInfo = NEW(cbTransInfo_T);

  pInfo->m_pPktHdr  = NULL;
  pInfo->m_pBuf     = NULL;
  pInfo->m_bCached  = false;

  hnd->m_tblTransCache[uTid & CB_HDR_TID_MASK] = pInfo;

  cbUnlock(hnd);
}

/*!
 * \brief Delete any cached transaction state from the cache.
 *
 * \param hnd       CogniBoost device handle.
 * \param uTid      Transaction id.
 */
static void cbCacheForget(cbHnd_T hnd, cbTid_T uTid)
{
  cbTransInfo_T *pInfo;

  cbLock(hnd);

  pInfo = hnd->m_tblTransCache[uTid & CB_HDR_TID_MASK];

  if( pInfo != NULL )
  {
    delete(pInfo->m_pPktHdr);
    delete(pInfo->m_pBuf);
    delete(pInfo);
    hnd->m_tblTransCache[uTid & CB_HDR_TID_MASK] = NULL;
  }

  cbUnlock(hnd);
}

/*!
 * \brief Cache a transaction response.
 *
 * Caching a response occurs when this client thread receives a response
 * for another client thread.
 *
 * \param hnd           CogniBoost device handle.
 * \param uTid          Transaction id.
 * \param [in] pPktHdr  Response packet header.
 * \param [in] bufRspMsg   Packed response message buffer.
 */
static void cbCacheSave(cbHnd_T     hnd,
                        cbTid_T     uTid,
                        cbPktHdr_T *pPktHdr,
                        byte_t      bufRspMsg[])
{
  cbTransInfo_T  *pInfo;
  size_t          uMsgLen;

  cbLock(hnd);

  pInfo = hnd->m_tblTransCache[uTid & CB_HDR_TID_MASK];

  delete(pInfo->m_pPktHdr);
  delete(pInfo->m_pBuf);

  uMsgLen = (size_t)(pPktHdr->m_hdrMsgLen);

  pInfo->m_pPktHdr  = new_memdup(sizeof(cbPktHdr_T), pPktHdr);
  pInfo->m_pBuf     = new_memdup(uMsgLen, bufRspMsg);
  pInfo->m_bCached  = true;

  LOGDIAG3("Device %d: Saved response to cache: tid=%u, msg=%s(msgid=%u).",
      hnd->m_fd, uTid, CB_MSG_NAME_ID(cbGetMsgId(bufRspMsg, uMsgLen)));

  cbUnlock(hnd);
}

/*!
 * \brief Load cached response from transaction cache.
 *
 * Caching a response occurs when this client thread receives a response
 * for another client thread.
 *
 * \param hnd           CogniBoost device handle.
 * \param uTid          Transaction id.
 * \param [out] pPktHdr Response packet header.
 * \param [out] bufRspMsg  Packed response message buffer.
 * \param sizeRsp       Size of response buffer.
 * 
 * \return
 * Returns \h_lt 0 error code if a response was cached but an error occurred.\n
 * Returns = 0 if no response found in cache.\n
 * Returns \h_gt 0 if a good cached response was loaded.
 */
static int cbCacheLoad(cbHnd_T      hnd,
                       cbTid_T      uTid,
                       cbPktHdr_T  *pPktHdr,
                       byte_t       bufRspMsg[],
                       size_t       sizeRsp)
{
  cbTransInfo_T  *pInfo;            // cached information
  size_t          uRspMsgLen;       // response message len
  int             rc = 0;           // return code

  cbLock(hnd);

  pInfo = hnd->m_tblTransCache[uTid & CB_HDR_TID_MASK];

  if( (pInfo == NULL) || !pInfo->m_bCached )
  {
    rc = 0;
  }

  else
  {
    uRspMsgLen = (size_t)(pPktHdr->m_hdrMsgLen);
    if( uRspMsgLen > sizeRsp )
    {
      CB_LOG_ERROR(hnd, CB_LIB_ECODE_TOO_BIG,
        "Message length=%zu > buffer size=%zu.", uRspMsgLen, sizeRsp);
      rc = -CB_LIB_ECODE_TOO_BIG;
    }

    // load from cache
    else
    {
      *pPktHdr = *(pInfo->m_pPktHdr);
      memcpy(bufRspMsg, pInfo->m_pBuf, uRspMsgLen);

      LOGDIAG3("Device %d: Loaded response from cache: "
               "tid=%u, msg=%s(msgid=%u).",
        hnd->m_fd, uTid, CB_MSG_NAME_ID(cbGetMsgId(bufRspMsg, uRspMsgLen)));

      rc = 1;
    }
  }

  cbUnlock(hnd);

  return rc;
}


// ...........................................................................
// Communication Support Functions
// ...........................................................................

/*!
 * \brief Atomically get the next available transaction id.
 *
 * \param hnd       CogniBoost device handle.
 *
 * return Returns tid.
 */
static cbTid_T cbNextTid(cbHnd_T hnd)
{
  cbTid_T uTid;

  cbLock(hnd);

  uTid = hnd->m_uTidCounter;
  hnd->m_uTidCounter = (uTid + 1) & CB_HDR_TID_MASK;

  cbUnlock(hnd);

  return uTid;
}

/*!
 * \brief Pack a \h_cogniboost packet header into a buffer.
 *
 * \param hnd             CogniBoost device handle.
 * \param [in] pPktHdr    Pointer to packet header structure.
 * \param [out] buf       Output message buffer.
 * \param bufSize         Size of output buffer.
 *
 * \return 
 * On success, returns the number of bytes packed.
 * On error, returns \h_lt 0.
 */
static inline int cbPackPktHdr(cbHnd_T hnd,
                               cbPktHdr_T  *pPktHdr,
                               byte_t       buf[],
                               size_t       bufSize)
{
  int   n = 0;

  CHK_EXPR(hnd, (bufSize >= CB_PKT_HDR_SIZE), CB_LIB_ECODE_TOO_SMALL,
              "Buffer size=%zu < packet size=%zu.",
              bufSize, CB_PKT_HDR_SIZE);

  n += nmPackU8(pPktHdr->m_hdrMagic,   buf+n, bufSize-(size_t)n, CB_MSG_ENDIAN);
  n += nmPackU8(pPktHdr->m_hdrTid,     buf+n, bufSize-(size_t)n, CB_MSG_ENDIAN);
  n += nmPackU16(pPktHdr->m_hdrMsgLen, buf+n, bufSize-(size_t)n, CB_MSG_ENDIAN);

  return n;
}

/*!
 * \brief Unpack a \h_cogniboost packet header from a buffer.
 *
 * \param hnd             CogniBoost device handle.
 * \param [in] buf        Input message buffer.
 * \param bufSize         Size of input buffer.
 * \param [out] pPktHdr   Pointer to packet header structure.
 *
 * \return 
 * On success, returns the number of bytes unpacked.
 * On error, returns \h_lt 0.
 */
static inline int cbUnpackPktHdr(cbHnd_T      hnd,
                                 byte_t       buf[],
                                 size_t       bufSize,
                                 cbPktHdr_T  *pPktHdr)
{
  int   n = 0;

  CHK_EXPR(hnd, (bufSize >= CB_PKT_HDR_SIZE),
                CB_LIB_ECODE_TOO_SMALL, "buffer size=%zu < packet size=%u",
                bufSize, CB_PKT_HDR_SIZE);

  n += nmUnpackU8(buf+n, bufSize-(size_t)n, &pPktHdr->m_hdrMagic,
                          CB_MSG_ENDIAN);
  n += nmUnpackU8(buf+n, bufSize-(size_t)n, &pPktHdr->m_hdrTid,
                          CB_MSG_ENDIAN);
  n += nmUnpackU16(buf+n, bufSize-(size_t)n, &pPktHdr->m_hdrMsgLen,
                          CB_MSG_ENDIAN);

  return n;
}


// ...........................................................................
// Device I/O Functions
// ...........................................................................

/*!
 * \brief Read bytes from CogniBoost device.
 *
 *  Read up to <em>count</em> bytes into <em>buf</em> from the connection.
 *  This call is non-blocking if the timeout value <em>usec</em> is greater than
 *  zero. Otherwise the read can block indefinitely.
 *
 *  Note that on return the bytes read can be less than <em>count</em>.
 *
 * \param hnd       CogniBoost device handle.
 * \param [out] buf Destination buffer.
 * \param count     Number of bytes to read.
 * \param usec      Timeout in microseconds.\n
 *                  If <em>usec</em> \h_gt 0, an upper timeout limit is placed
 *                  on the read.
 *                  If <em>usec</em> == 0, then the read will block indefinitely
 *                  until <em>count</em> bytes are read or an I/O error has
 *                  ocurred.
 *
 * \return
 * On success, returns \h_ge 0 number of bytes read.\n
 * On error, returns \h_lt 0 error code.
 */
static int cbDeviceRead(cbHnd_T hnd, byte_t buf[], size_t count, uint_t usec)
{
  ssize_t         n;

  LOGDIAG5CALL(_TINT(hnd->m_fd), _TPTR(buf), _TUINT(count), _TUINT(usec));

  n = SerDevRead(hnd->m_fd, buf, count, usec);

  if( n < 0 )
  {
    CB_LOG_SYSERROR(hnd, CB_LIB_ECODE_READ, "SerDevRead().");
    return -CB_LIB_ECODE_READ;
  }

  else if( (size_t)n != count )
  {
    CB_LOG_ERROR(hnd, CB_LIB_ECODE_TIMEDOUT, "SerDevRead().");
  }

  LOGDIAG4("%s(): Device %d: %zd bytes read.", LOGFUNCNAME, hnd->m_fd, n);

  return (int)n;
}

/*!
 * \brief Write bytes to CogniBoost device.
 *
 *  Write up to <em>count</em> bytes from <em>buf</em> to the connection.
 *  This call is non-blocking if the timeout value <em>usec</em> is greater than
 *  zero. Otherwise the read can block indefinitely.
 *
 *  Note that the number of bytes written can be less than the <em>count</em>.
 *
 * \param hnd       CogniBoost device handle.
 * \param buf       Destination buffer.
 * \param count     Number of bytes to read.
 * \param usec      Timeout in microseconds.\n
 *                  If <em>usec</em> \h_gt 0, an upper timeout limit is placed
 *                  on the read.
 *                  If <em>usec</em> == 0, then the read will block indefinitely
 *                  until <em>count</em> bytes are read or an I/O error has
 *                  ocurred.
 *
 * \return
 * On success, returns \h_ge 0 number of bytes written.\n
 * On error, returns \h_lt 0 error code.
 */
static int cbDeviceWrite(cbHnd_T hnd, byte_t buf[], size_t count, uint_t usec)
{
  ssize_t         n;

  LOGDIAG5CALL(_TINT(hnd->m_fd), _TPTR(buf), _TUINT(count), _TUINT(usec));

  n = SerDevWrite(hnd->m_fd, buf, count, usec);

  if( n < 0 )
  {
    CB_LOG_SYSERROR(hnd, CB_LIB_ECODE_WRITE, "SerDevWrite().");
    return -CB_LIB_ECODE_READ;
  }

  else if( (size_t)n != count )
  {
    CB_LOG_ERROR(hnd, CB_LIB_ECODE_TIMEDOUT, "SerDevWrite().");
  }

  LOGDIAG4("%s(): Device %d: %zd bytes written.", LOGFUNCNAME, hnd->m_fd, n);

  return (int)n;
}

/*!
 * \brief Flush input of <em>count</em> bytes received from device.
 *
 * \param hnd       CogniBoost device handle.
 * \param count     Number of bytes to flush.
 */
static void cbDeviceFlushInput(cbHnd_T hnd, size_t count)
{
  byte_t  buf[1024];
  size_t  block;
  int     n;

  while( count > 0 )
  {
    block = sizeof(buf) >= count? count: sizeof(buf);
    n = cbDeviceRead(hnd, buf, block, CB_TUNE_T_READ);
    if( n <= 0 )
    {
      return;
    }
    count -= (size_t)n;
  }
}

/*!
 * \brief Resync client with server.
 *
 *  Lost message alignment with the server (if this is possible). Perfomr a
 *  series of flush-request cycles to try to get back in sync.
 *
 * \param hnd       CogniBoost device handle.
 */
static void cbResync(cbHnd_T hnd)
{
  int   nMaxTries = 5;
  int   nTries;

  for(nTries=0; nTries<nMaxTries; ++nTries)
  {
    cbDeviceFlushInput(hnd, CB_PKT_MAX_SIZE);
    // TODO
  }
  CB_LOG_ERROR(hnd, CB_LIB_ECODE_SYNC, "");
}

/*!
 * \brief Send a request packet to the server.
 *
 * \param hnd             CogniBoost device handle.
 * \param uTidReq         Transaction id.
 * \param eReqMsgId       Request message id.
 * \param [in] bufReqPkt  Packet buffer with the prepacked request message.
 *                        The buffer has empty head space for the packet header.
 * \param uReqMsgLen      Length of request message (prepacked in packet).
 *
 * \return 
 * On success, returns the length of the sent request packet.\n
 * \copydoc doc_return_ecode
 */
static int cbSendReq(cbHnd_T      hnd,
                     cbTid_T      uTidReq,
                     cbMsgId_T    eReqMsgId,
                     byte_t       bufReqPkt[],
                     size_t       uReqMsgLen)
{
  cbPktHdr_T  hdrReq;       // request packet header
  int         nReqPktLen;
  int         n;

  CHK_EXPR(hnd, (uReqMsgLen >= CB_MSG_MIN_SIZE), CB_LIB_ECODE_TOO_SMALL,
      "Request %s(%d) length=%zu < message minimum=%zu.",
      CB_MSG_NAME_ID(eReqMsgId), uReqMsgLen, CB_MSG_MIN_SIZE);

  CHK_EXPR(hnd, (uReqMsgLen <= CB_MSG_MAX_SIZE), CB_LIB_ECODE_TOO_BIG,
      "Request %s(%d) length=%zu > message maximum=%zu.",
      CB_MSG_NAME_ID(eReqMsgId), uReqMsgLen, CB_MSG_MAX_SIZE);
  
  // fill request packet header
  hdrReq.m_hdrMagic   = (byte_t)CB_HDR_MAGIC;
  hdrReq.m_hdrTid     = (byte_t)uTidReq;
  hdrReq.m_hdrMsgLen  = (ushort_t)uReqMsgLen;

  // packet size
  nReqPktLen = CB_PKT_HDR_SIZE + (int)uReqMsgLen;

  // prepend header to request buffer
  n = cbPackPktHdr(hnd, &hdrReq, bufReqPkt, CB_PKT_HDR_SIZE);

  CHK_ECODE(hnd, n,
      "Failed to pack request packet header into packet for message %s(%d).",
      CB_MSG_NAME_ID(eReqMsgId));

  // send the message to the server
  n = cbDeviceWrite(hnd, bufReqPkt, (size_t)nReqPktLen, CB_TUNE_T_WRITE);

  CHK_EXPR(hnd, (n == nReqPktLen), CB_LIB_ECODE_SEND,
      "Failed to send request packet for message %s(%d).",
      CB_MSG_NAME_ID(eReqMsgId));

  // log successful request sent event
  CB_LOG_PKT(hnd, "REQ", &hdrReq, eReqMsgId);

  return nReqPktLen;
}

/*!
 * \brief Read response message from server.
 *
 * \param hnd             CogniBoost device handle.
 * \param [out] pPktHdr   Response packet header.
 * \param [out] bufRspMsg Buffer with packed response message.
 * \param sizeRspBuf      Size of response buffer.
 *
 * \return 
 * On success, returns the length of received response packet.\n
 * \copydoc doc_return_ecode
 */
static int cbRecvRsp(cbHnd_T      hnd,
                     cbPktHdr_T  *pPktHdr,
                     byte_t       bufRspMsg[],
                     size_t       sizeRspBuf)

{
  size_t      uRspMsgLen;
  int         nRspPktLen;
  byte_t      bufHdr[CB_PKT_HDR_SIZE];
  int         n;

  // read response header
  n = cbDeviceRead(hnd, bufHdr, CB_PKT_HDR_SIZE, CB_TUNE_T_READ);

  CHK_EXPR(hnd, (n == CB_PKT_HDR_SIZE), CB_LIB_ECODE_RECV,
        "Failed to receive response packet header, %d bytes received.", n);

  // unpack packet header
  n = cbUnpackPktHdr(hnd, bufHdr, (size_t)CB_PKT_HDR_SIZE, pPktHdr);

  CHK_ECODE(hnd, n, "Received bad response packet header, discarding.");

  uRspMsgLen  = (size_t)(pPktHdr->m_hdrMsgLen);
  nRspPktLen  = CB_PKT_HDR_SIZE + (int)uRspMsgLen;

  // validate magic
  if( pPktHdr->m_hdrMagic != CB_HDR_MAGIC )
  {
    CB_LOG_ERROR(hnd, CB_RSP_ECODE_BAD_PKT,
      "Response packet header magic=0x%02x is bad, resyncing.",
      pPktHdr->m_hdrMagic);
    cbResync(hnd);
    return -CB_RSP_ECODE_BAD_PKT;
  }

  // message too short
  else if( uRspMsgLen < CB_MSG_MIN_SIZE )
  {
    CB_LOG_ERROR(hnd, CB_LIB_ECODE_TOO_SMALL,
      "Response packet header message length=%zu < message minimum=%zu, "
      "discarding.",
      uRspMsgLen, CB_MSG_MIN_SIZE);
    cbDeviceFlushInput(hnd, uRspMsgLen);
    return -CB_LIB_ECODE_TOO_SMALL;
  }

  // message too long
  else if( uRspMsgLen > sizeRspBuf )
  {
    CB_LOG_ERROR(hnd, CB_LIB_ECODE_TOO_BIG,
      "Response packet header message length=%zu > buffer size=%zu, "
      "discarding.",
      uRspMsgLen, sizeRspBuf);
    cbDeviceFlushInput(hnd, uRspMsgLen);
    return -CB_LIB_ECODE_TOO_BIG;
  }

  // read response message
  n = cbDeviceRead(hnd, bufRspMsg, uRspMsgLen, CB_TUNE_T_READ);

  CHK_EXPR(hnd, (n == (int)uRspMsgLen), CB_LIB_ECODE_RECV,
      "Failed to receive response message, %d bytes received.", n);

  // log successful response received event
  CB_LOG_PKT(hnd, "RSP", pPktHdr, cbGetMsgId(bufRspMsg, uRspMsgLen));

  return nRspPktLen;
}


// ---------------------------------------------------------------------------
// Internal Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Open the device to a CogniBoost.
 *
 * \param sDeviceName   Serial device name.
 * \param nBaudRate     Serial baud rate.
 * \param bTrace        Do [not] trace messages.
 *
 * \return
 * On success, returns a new CogniBoost device handle.
 * On failure, \ref CB_HND_NONE is returned.
 */
cbHnd_T cbOpen(const char *sDeviceName, int nBaudRate, bool_t bTrace)
{
  int fd;     // file descriptor

  // invalid device name specified
  if ( (sDeviceName == NULL ) || (*sDeviceName == 0) )
  {
    LOGERROR("%s(): No device name specified.", LOGFUNCNAME);
    errno = EINVAL;
    return CB_HND_NONE;
  }

  // check baudrate value
  switch( nBaudRate )
  {
    case CB_PARAM_BAUD_RATE_115200:
    case CB_PARAM_BAUD_RATE_230400:
    case CB_PARAM_BAUD_RATE_500000:
    case CB_PARAM_BAUD_RATE_1000000:
    case CB_PARAM_BAUD_RATE_2000000:
    case CB_PARAM_BAUD_RATE_3000000:
    case CB_PARAM_BAUD_RATE_4000000:
      break;
    default:
      LOGERROR("%s(): Invalid baud rate=%d.", LOGFUNCNAME, nBaudRate);
      errno = EINVAL;
      return CB_HND_NONE;
  }
 
  // open serial device baudrate/8-N-1, no flow control
  fd = SerDevOpen(sDeviceName,  // device name 
                  nBaudRate,    // baud rate
                  8,            // byte size
                  'N',          // parity
                  1,            // stop bits
                  false,        // Do [not] use hardware flow control
                  false);       // Do [not] use software flow control

  if( fd < 0 )
  {
    LOGSYSERROR("%s(): Failed to open device \"%s\".",
            LOGFUNCNAME, sDeviceName);
    return CB_HND_NONE;
  }

  LOGDIAG3("Opened device %d to %s@%d.", fd, nBaudRate, sDeviceName);

  return cbDeviceNew(sDeviceName, nBaudRate, fd, bTrace);
}

/*!
 * \brief Close the CogniBoost device.
 *
 * The device handle is deallocated and is no longer valid.
 *  
 * \param hnd       CogniBoost device handle.
 *
 * \copydoc doc_std_return
 */
int cbClose(cbHnd_T hnd)
{
  CHK_DEV(hnd);

  SerDevClose(hnd->m_fd);

  LOGDIAG3("Closed device %d to %s.", hnd->m_fd, hnd->m_sDeviceName);

  cbDeviceDelete(hnd);

  return CB_OK;
}

/*!
 * \brief Execute a low-level request-response transaction with the CogniBoost
 * server.
 *
 * \note The request and response buffer can be the same buffer if the packed 
 * request contents does not need to be preserved.
 *
 * \warning There must be exactly \ref CB_PKT_HDR_SIZE bytes at the front
 * of the request buffer available to pack the \h_botsense header. (Reduces the
 * number of buffer copies.)
 *
 * \par Request Format:
 * packet ::= pkthdr msg\n
 * msg ::=  msghdr msgbody
 *
 * \par Response Format:
 * packet ::= pkthdr msg\n
 * msg ::=  msghdr msgbody
 *
 * \param hnd             CogniBoost device handle.
 * \param eReqMsgId       Request message id.
 * \param [in] bufReqPkt  Packet buffer with the prepacked request message.
 *                        The buffer has empty head space for the packet header.
 * \param uReqMsgLen      Length of packed request message.
 * \param eRspMsgId       Expected response message id.
 * \param [out] bufRspMsg Buffer with received packed response message. 
 * \param sizeRspBuf      Size of response buffer (number of bytes).
 *
 * \return 
 * On success, returns the length of the response message.\n
 * \copydoc doc_return_ecode
 */
int cbTransLow(cbHnd_T    hnd,
               cbMsgId_T  eReqMsgId,
               byte_t     bufReqPkt[],
               size_t     uReqMsgLen,
               cbMsgId_T  eRspMsgId,
               byte_t     bufRspMsg[],
               size_t     sizeRspBuf)
{
  cbTid_T       uTidReq;      // request transaction id
  cbPktHdr_T    hdrRsp;       // response packet header
  cbMsgId_T     eRcvMsgId;    // received message id
  cbRspErr_T    msgRspErr;    // common error response message
  bool_t        bPending;     // resposne is [not] pending
  int           nRspMsgLen;   // response message length
  uint_t        uRspECode;    // response error message error code
  int           n;            // number of bytes/return code

  CHK_DEV(hnd);

  CHK_EXPR(hnd, (sizeRspBuf >= CB_MSG_MIN_SIZE),
      CB_LIB_ECODE_TOO_SMALL, "Response buffer size=%zu < message minimum=%zu.",
      sizeRspBuf, CB_MSG_MIN_SIZE);

  // this request's transaction id
  uTidReq = cbNextTid(hnd);

  // mark request for tracking
  cbCacheMark(hnd, uTidReq);

  //
  // send request
  //
  cbLock(hnd);

  n = cbSendReq(hnd, uTidReq, eReqMsgId, bufReqPkt, uReqMsgLen);

  cbUnlock(hnd);

  CHK_ECODE(hnd, n, "Failed to send request %s(%d).",
      CB_MSG_NAME_ID(eReqMsgId));

  //
  // receive response
  //
  bPending = true;

  while( bPending )
  {
    // check transaction cache for any cached response.
    n = cbCacheLoad(hnd, uTidReq, &hdrRsp, bufRspMsg, sizeRspBuf);

    CHK_ECODE(hnd, n, "Cached response failed for request %s(%d).",
        CB_MSG_NAME_ID(eReqMsgId));

    // found a good, cached response
    if( n > 0 )
    {
      cbCacheForget(hnd, uTidReq);
      bPending = false;
    }

    // nothing cached, receive a response
    else
    {
      cbLock(hnd);

      n = (int)cbRecvRsp(hnd, &hdrRsp, bufRspMsg, sizeRspBuf);
      
      cbUnlock(hnd);

      CHK_ECODE(hnd, n, "Failed to receive response %s(%d).",
          CB_MSG_NAME_ID(eRspMsgId));

      // response was for this request
      if( uTidReq == hdrRsp.m_hdrTid )
      {
        cbCacheForget(hnd, uTidReq);
        bPending = false;
      }

      // response is for another thread - cache it
      else
      {
        cbCacheSave(hnd, uTidReq, &hdrRsp, bufRspMsg);
      }
    }
  }

  nRspMsgLen  = (int)hdrRsp.m_hdrMsgLen;
  eRcvMsgId   = cbGetMsgId(bufRspMsg, (size_t)n);

  // received common error response
  if( eRcvMsgId == cbMsgIdRspErr )
  {
    n = cbUnpackRspErr(bufRspMsg, (size_t)n, &msgRspErr, hnd->m_bTrace);

    CHK_ECODE(hnd, n, "Failed to unpack error response message.");

    uRspECode = (uint_t)(msgRspErr.m_ecode);

    // unknown response error code
    if( uRspECode >= CB_RSP_ECODE_NUMOF )
    {
      uRspECode = CB_LIB_ECODE_RSP_BADEC;
    }

    CB_LOG_ERROR(hnd, (int)uRspECode, "%s.", msgRspErr.m_emsg);

    return -(int)uRspECode;
  }

  CHK_EXPR(hnd, (eRcvMsgId == eRspMsgId), CB_LIB_ECODE_RECV,
        "Received unexpected message %s(%d), "
        "expected message %s(%d).",
        CB_MSG_NAME_ID(eRcvMsgId), CB_MSG_NAME_ID(eRspMsgId));

  return nRspMsgLen;
}

/*!
 * \brief Execute a request-response transaction with the CogniBoost server.
 *
 * \param hnd             CogniBoost device handle.
 * \param eReqMsgId       Request message id.
 * \param [in]pReqStruct  Pointer to request message-specific structure with
 *                        initialized data. NULL if no message body.
 * \param eRspMsgId       Response message id.
 * \param [out]pRspStruct Pointer to response message-specific structure. The
 *                        structure will be filled in with the field values
 *                        from the response message. NULL if no message body.
 *
 * \copydoc doc_return_std
 */ 
int cbTrans(cbHnd_T     hnd,
            cbMsgId_T   eReqMsgId,
            void       *pReqStruct,
            cbMsgId_T   eRspMsgId,
            void       *pRspStruct)
{
  byte_t    bufPkt[CB_PKT_MAX_SIZE];  // request/response packet buffer
  int       n;                        // number of bytes/return code

  CHK_DEV(hnd);

  // pack request
  n = cbPackMsg(eReqMsgId, pReqStruct, CB_BUF_MSG(bufPkt), hnd->m_bTrace);

  CHK_NM_ECODE(hnd, n, "Failed to pack message %s(%d).",
              CB_MSG_NAME_ID(eReqMsgId));

  n = cbTransLow(hnd, eReqMsgId, bufPkt, (size_t)n,
                      eRspMsgId, CB_BUF_MSG(bufPkt));

  CHK_ECODE(hnd, n, "Transaction failed.");

  // unpack response
  n = cbUnpackMsg(eRspMsgId, CB_BUF_MSG(bufPkt), pRspStruct, hnd->m_bTrace);

  CHK_NM_ECODE(hnd, n, "Failed to unpack message %s(%d).",
              CB_MSG_NAME_ID(eRspMsgId));

  return CB_OK;
}


// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------
 
/*!
 * \brief Get CogniBoost host device name.
 *
 * \param hnd       CogniBoost device handle.
 *
 * \return
 * On success, returns the device name string.\n
 * On failure, returns NULL.
 */
const char *cbAttrGetDeviceName(cbHnd_T hnd)
{
  return hnd != CB_HND_NONE? hnd->m_sDeviceName: NULL;
}

/*!
 * \brief Get CogniBoost host baud rate.
 *
 * \param hnd       CogniBoost device handle.
 *
 * \return 
 * On success, returns the baud rate.\n
 * \copydoc doc_return_ecode
 */
int cbAttrGetBaudRate(cbHnd_T hnd)
{
  return hnd != CB_HND_NONE? hnd->m_nBaudRate: -CB_LIB_ECODE_NO_DEV;
}

/*!
 * \brief Get CogniBoost host device file descriptor.
 *
 * \param hnd       CogniBoost device handle.
 *
 * \return 
 * On success, returns the file descriptor to the client open serial device.
 * \copydoc doc_return_ecode
 */
int cbAttrGetFd(cbHnd_T hnd)
{
  return hnd != CB_HND_NONE? hnd->m_fd: -CB_LIB_ECODE_NO_DEV;
}

/*!
 * \brief Get host message trace state.
 *
 * \param hnd       CogniBoost device handle.
 *
 * \return 
 * Returns true (tracing enabled) or false (tracing disabled).
 */
bool_t cbAttrGetTraceState(cbHnd_T hnd)
{
  return hnd != CB_HND_NONE? hnd->m_bTrace: false;
}

/*!
 * \brief Set host message trace state.
 *
 * \param hnd       CogniBoost device handle.
 * \param bTrace    Enable (true) / disable (false) message tracing.
 *
 * \copydoc doc_std_return
 */
int cbAttrSetTraceState(cbHnd_T hnd, bool_t bEnable)
{
  CHK_DEV(hnd);

  hnd->m_bTrace = bEnable? true: false;

  return CB_OK;
}

/*!
 * \brief Set host diagnostics logging threshold.
 *
 * \param hnd       CogniBoost device handle.
 * \param nLevel    New logging threshold level.
 *
 * \copydoc doc_std_return
 */
int cbAttrSetLogging(cbHnd_T hnd, int nLevel)
{
  CHK_DEV(hnd);
  LOG_SET_THRESHOLD(nLevel);
  return CB_OK;
}
