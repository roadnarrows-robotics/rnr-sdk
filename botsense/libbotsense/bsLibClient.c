////////////////////////////////////////////////////////////////////////////////
//
// Package:   BotSense
//
// Library:   libbsclient
//
// File:      bsLibClient.c
//
/*! \file
 *
 * $LastChangedDate: 2012-11-25 10:42:41 -0700 (Sun, 25 Nov 2012) $
 * $Rev: 2544 $
 *
 * \brief Client base functions.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2009-2017. RoadNarrows LLC.\n
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
#include <errno.h>
#include <pthread.h>
#include <limits.h>
#include <sys/select.h>
#include <sys/time.h>

#include "rnr/rnrconfig.h"
#include "rnr/new.h"
#include "rnr/log.h"
#include "rnr/netmsgs.h"

#include "botsense/BotSense.h"
#include "botsense/libBotSense.h"
#include "botsense/bsProxyMsgs.h"

#include "bsLibInternal.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

PRAGMA_IGNORED(sign-conversion)
/*!
 * \brief FD_SET() wrapper with no annoying warnings.
 * \param fd    File descriptor to add to set.
 * \param pset  Pointer to fd set.
 */
static inline void fdset_nowarn(int fd, fd_set *pset)
{
  FD_SET(fd, pset);
}
PRAGMA_WARNING(sign-conversion)


// ...........................................................................
// Transaction Mutual Exclusion Functions
// ...........................................................................

/*!
 * \brief Lock client's transaction mutual exclusion.
 *
 * \param pClient   \h_botsense client.
 */
static inline void bsTransLock(BsClient_T *pClient)
{
  int rc;

  if( (rc = pthread_mutex_lock(&(pClient->m_mutexTrans))) != 0 ) 
  { 
    errno = rc;
    BSCLIENT_LOG_SYSERROR(pClient, BS_ECODE_SYS, "pthread_mutex_lock()");
  }
}

/*!
 * \brief Unlock client's transaction mutual exclusion.
 *
 * \param pClient   \h_botsense client.
 */
static inline void bsTransUnlock(BsClient_T *pClient)
{
  int rc;

  if( (rc = pthread_mutex_unlock(&(pClient->m_mutexTrans))) != 0 ) 
  { 
    errno = rc;
    BSCLIENT_LOG_SYSERROR(pClient, BS_ECODE_SYS, "pthread_mutex_unlock()");
  }
}

/*!
 * \brief Try to lock client's transaction mutual exclusion.
 *
 * \param pClient   \h_botsense client.
 *
 * \return
 * Return true if lock acquired. Otherwise false if already locked.
 */
static inline bool_t bsTransTryLock(BsClient_T *pClient)
{
  return pthread_mutex_trylock(&(pClient->m_mutexTrans)) == 0? true: false;
}


// ...........................................................................
// Transaction Caching Functions
// ...........................................................................

/*!
 * \brief Mark the start of a client's server transaction in the transaction
 * cache.
 *
 * A client can be multi-threaded, and so the responses from the server
 * may come out of order with respect to the client thread.
 *
 * \param pClient   \h_botsense client.
 * \param uTid      Transaction id.
 */
static void bsTransMark(BsClient_T *pClient, BsTid_T uTid)
{
  BsTransInfo_T *pInfo;

  bsTransLock(pClient);

  pInfo = pClient->m_tblTransCache[uTid & BSPROXY_TID_MASK];

  if( pInfo != NULL )
  {
    delete(pInfo->m_pMsgHdr);
    delete(pInfo->m_pBuf);
    delete(pInfo);
  }

  pInfo = NEW(BsTransInfo_T);

  pInfo->m_pMsgHdr  = NULL;
  pInfo->m_pBuf     = NULL;
  pInfo->m_bCached  = false;

  pClient->m_tblTransCache[uTid & BSPROXY_TID_MASK] = pInfo;

  bsTransUnlock(pClient);
}

/*!
 * \brief Delete any cached transaction state from the cache.
 *
 * \param pClient   \h_botsense client.
 * \param uTid      Transaction id.
 */
static void bsTransForget(BsClient_T *pClient, BsTid_T uTid)
{
  BsTransInfo_T *pInfo;

  bsTransLock(pClient);

  pInfo = pClient->m_tblTransCache[uTid & BSPROXY_TID_MASK];

  if( pInfo != NULL )
  {
    delete(pInfo->m_pMsgHdr);
    delete(pInfo->m_pBuf);
    delete(pInfo);
    pClient->m_tblTransCache[uTid & BSPROXY_TID_MASK] = NULL;
  }

  bsTransUnlock(pClient);
}

/*!
 * \brief Cache a transaction response.
 *
 * Cashing a response occurs when this client thread receives a response
 * fo another client thread.
 *
 * \param pClient       \h_botsense client.
 * \param uTid          Transaction id.
 * \param [in] pMsgHdr  Response message header.
 * \param [in] bufRsp   Packed response message buffer.
 */
static void bsTransCacheRsp(BsClient_T      *pClient,
                            BsTid_T          uTid,
                            BsProxyMsgHdr_T *pMsgHdr,
                            byte_t           bufRsp[])
{
  BsTransInfo_T  *pInfo;
  size_t          uBodyLen;

  bsTransLock(pClient);

  pInfo = pClient->m_tblTransCache[uTid & BSPROXY_TID_MASK];

  delete(pInfo->m_pMsgHdr);
  delete(pInfo->m_pBuf);

  uBodyLen = (size_t)(pMsgHdr->m_hdrBodyLen);

  pInfo->m_pMsgHdr  = new_memdup(sizeof(BsProxyMsgHdr_T), pMsgHdr);
  pInfo->m_pBuf     = uBodyLen>0? new_memdup(uBodyLen, bufRsp): NULL;
  pInfo->m_bCached  = true;

  LOGDIAG3("%s: Tid=%u, MsgId=%u cached.",
      bsClientAttrGetName(pClient), uTid, pMsgHdr->m_hdrMsgId);

  bsTransUnlock(pClient);
}

/*!
 * \brief Load cached response from transaction cache.
 *
 * Cashing a response occurs when this client thread receives a response
 * fo another client thread.
 *
 * \param pClient       \h_botsense client.
 * \param uTid          Transaction id.
 * \param [out] pMsgHdr Response message header.
 * \param [out] bufRsp  Packed response message buffer.
 * \param sizeRsp       Size of response buffer.
 * 
 * \return
 * Returns \h_lt 0 error code if a response was cached but an error occurred.\n
 * Returns = 0 if no response found in cache.\n
 * Returns \h_gt 0 if a good cached response was loaded.
 */
static int bsTransLoadCached(BsClient_T      *pClient,
                             BsTid_T          uTid,
                             BsProxyMsgHdr_T *pMsgHdr,
                             byte_t           bufRsp[],
                             size_t           sizeRsp)
{
  BsTransInfo_T  *pInfo;
  size_t          uBodyLen;
  int             rc = 0;

  bsTransLock(pClient);

  pInfo = pClient->m_tblTransCache[uTid & BSPROXY_TID_MASK];

  if( (pInfo == NULL) || !pInfo->m_bCached )
  {
    rc = 0;
  }

  else
  {
    uBodyLen = (size_t)(pMsgHdr->m_hdrBodyLen);
    if( uBodyLen > sizeRsp )
    {
      BSCLIENT_LOG_ERROR(pClient, BS_ECODE_MSG_TOO_BIG,
        "body_len=%zu > buf_size=%zu.", uBodyLen, sizeRsp);
      rc = -BS_ECODE_MSG_TOO_BIG;
    }

    // load from cache
    else
    {
      *pMsgHdr = *(pInfo->m_pMsgHdr);
      if( uBodyLen > 0 )
      {
        memcpy(bufRsp, pInfo->m_pBuf, uBodyLen);
      }

      LOGDIAG3("%s: Tid=%u, MsgId=%u loaded from cached.",
        bsClientAttrGetName(pClient), uTid, pMsgHdr->m_hdrMsgId);

      rc = 1;
    }
  }

  bsTransUnlock(pClient);

  return rc;
}

/*!
 * \brief Atomically get the next available transaction id.
 *
 * \param pClient   \h_botsense client.
 *
 * return Returns tid.
 */
static BsTid_T bsNextTid(BsClient_T *pClient)
{
  BsTid_T uTid;

  bsTransLock(pClient);

  uTid = pClient->m_uTidCounter;
  pClient->m_uTidCounter = (uTid + 1) & BSPROXY_TID_MASK;

  bsTransUnlock(pClient);

  return uTid;
}


// ...........................................................................
// Timer Utilities
// ...........................................................................

/*! 
 * \brief Mark the current time. Resolution is microseconds.
 *
 * \param pTvMark   Pointer to timeval structure to be populated with
 *                  the current system time in seconds and useconds.
 */
static inline void timer_mark(struct timeval  *pTvMark)
{
  if( gettimeofday(pTvMark, NULL) != OK )
  {
    LOGSYSERROR("gettimeofday()");
    timerclear(pTvMark);
  }
}

/*! 
 * \brief Calculate the elapsed time between the given time mark and this call.
 *
 * \param pTvMark   Pointer to timeval holding time mark.
 *
 * \return 
 * Number of microseconds elasped. If the marked time is invalid or the current
 * time cannot be ascertained, UINT_MAX is returned.
 */
static uint_t timer_elapsed(struct timeval *pTvMark)
{
  struct timeval  tvEnd, tvDelta;

  timer_mark(&tvEnd);

  if( !timerisset(pTvMark) || !timerisset(&tvEnd) )
  {
    return UINT_MAX;
  }

  tvDelta.tv_sec = tvEnd.tv_sec - pTvMark->tv_sec;
  if( tvEnd.tv_usec < pTvMark->tv_usec )
  {
    tvDelta.tv_sec--;
    tvEnd.tv_usec += 1000000;
  }
  tvDelta.tv_usec = tvEnd.tv_usec - pTvMark->tv_usec;

  return (uint_t)(tvDelta.tv_sec * 1000000 + tvDelta.tv_usec);
}


// ...........................................................................
// Connection I/O Functions
// ...........................................................................

/*!
 * \brief Pack \h_botsense client message header.
 *
 * \param pClient         \h_botsense client.
 * \param [in] pMsgHdr    Pointer to message header structure.
 * \param [out] buf       Output message buffer.
 * \param bufSize         Size of output buffer.
 *
 * \return 
 * On success, returns the number of bytes packed.
 * On error, returns \h_lt 0.
 */
static inline int bsClientPackMsgHdr(BsClient_T      *pClient,
                                     BsProxyMsgHdr_T *pMsgHdr,
                                     byte_t           buf[],
                                     size_t           bufSize)
{
  BSCLIENT_TRY_EXPR(pClient, (bufSize >= BSPROXY_MSG_HDR_LEN),
                    BS_ECODE_BUF_TOO_SMALL, "bufSize=%zu < %u",
                    bufSize, BSPROXY_MSG_HDR_LEN);

  return bsPackMsgHdr(pMsgHdr, buf, bufSize);
}

/*!
 * \brief Unpack \h_botsense client message header.
 *
 * \param pClient         \h_botsense client.
 * \param [in] buf        Input message buffer.
 * \param bufSize         Size of input buffer.
 * \param [out] pMsgHdr   Pointer to message header structure.
 *
 * \return 
 * On success, returns the number of bytes unpacked.
 * On error, returns \h_lt 0.
 */
static inline int bsClientUnpackMsgHdr(BsClient_T      *pClient,
                                       byte_t           buf[],
                                       size_t           bufSize,
                                       BsProxyMsgHdr_T *pMsgHdr)
{
  BSCLIENT_TRY_EXPR(pClient, (bufSize >= BSPROXY_MSG_HDR_LEN),
                    BS_ECODE_BUF_TOO_SMALL, "bufSize=%zu < %u",
                    bufSize, BSPROXY_MSG_HDR_LEN);

  return bsUnpackMsgHdr(buf, bufSize, pMsgHdr);
}

/*!
 * \brief Read bytes from socket.
 *
 *  Read up to <em>count</em> bytes into <em>buf</em> from the socket.
 *  This call is non-blocking if the timeout value <em>usec</em> is greater than
 *  zero. Otherwise the read can block indefinitely.
 *
 *  Note that on return the bytes read can be less than <em>count</em>.
 *
 * \param pClient   \h_botsense client.
 * \param sd        Socket descriptor.
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
static int bsClientRead(BsClient_T *pClient,
                        int         sd,
                        byte_t      buf[],
                        size_t      count,
                        uint_t      usec)
{
  bool_t          bNonBlocking;
  struct timeval  tstart;
  uint_t          tdelta;
  fd_set          rset;
  struct timeval  timeout;
  int             nFd;
  ssize_t         n;
  int             nBytes = 0;

  LOGDIAG4CALL(_TINT(sd), _TPTR(buf), _TUINT(count), _TUINT(usec));

  bNonBlocking = usec > 0? true: false;

  while( nBytes < count )
  {
    FD_ZERO(&rset);
    fdset_nowarn(sd, &rset);

    // wait for input with timeout
    if( bNonBlocking )
    {
      timer_mark(&tstart);

      // (re)load timeout (gets munged after each select())
      timeout.tv_sec  = (time_t)(usec / 1000000);
      timeout.tv_usec = (time_t)(usec % 1000000);

      nFd = select(sd+1, &rset, NULL, NULL, &timeout);
    }

    // block indefinitely for input
    else 
    {
      nFd = select(sd+1, &rset, NULL, NULL, NULL);
    }

    // system error occurred on select - interpret
    if( nFd < 0 )
    {
      switch(errno)
      {
        case EAGAIN:        // non-blocking timeout
        case EINTR:         // read was interrupted
          break;
        default:
          BSCLIENT_LOG_SYSERROR(pClient, BS_ECODE_BAD_RECV,
              "select(%d,...)", sd);
          return -BS_ECODE_BAD_RECV;
      }
    }

    // select() timeout occurred
    else if( nFd == 0 )
    {
      LOGDIAG4("select() on read timed out.");
      break;
    }

    // data available from serial device 
    n = read(sd, buf+nBytes, count-(size_t)nBytes);

    // system error occurred on read - interpret
    if( n < 0 )
    {
      switch(errno)
      {
        case EAGAIN:        // non-blocking timeout
        case EINTR:         // read was interrupted
          n = 0;
          break;
        default:
          BSCLIENT_LOG_SYSERROR(pClient, BS_ECODE_BAD_RECV,
              "select(%d,...)", sd);
          return -BS_ECODE_BAD_RECV;
      }
    }

    // got some data
    nBytes += (int)n;

    // all of the requested bytes have been read
    if( nBytes == count )
    {
      break;
    }

    // determine time left for non-blocking read timeout
    if( bNonBlocking && ((size_t)nBytes < count) )
    {
      tdelta = timer_elapsed(&tstart);
      if( tdelta >= usec )
      {
        LOGDIAG4("%s() timed out.", LOGFUNCNAME);
        break;
      }
      else
      {
        usec -= tdelta;
      }
    }
  }

  LOGDIAG4("%s(): %d bytes read.", LOGFUNCNAME, nBytes);

  return nBytes;
}

/*!
 * \brief Write bytes to socket.
 *
 *  Write up to <em>count</em> bytes from <em>buf</em> to the socket.
 *  This call is non-blocking if the timeout value <em>usec</em> is greater than
 *  zero. Otherwise the read can block indefinitely.
 *
 *  Note that the number of bytes written can be less than the <em>count</em>.
 *
 * \param pClient   \h_botsense client.
 * \param sd        Socket descriptor.
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
static int bsClientWrite(BsClient_T *pClient,
                         int         sd,
                         byte_t      buf[],
                         size_t      count,
                         uint_t      usec)
{
  bool_t          bNonBlocking;
  struct timeval  tstart;
  uint_t          tdelta;
  fd_set          wset;
  struct timeval  timeout;
  int             nFd;
  ssize_t         n;
  int             nBytes = 0;

  LOGDIAG4CALL(_TINT(sd), _TPTR(buf), _TUINT(count), _TUINT(usec));

  bNonBlocking = usec > 0? true: false;

  while( nBytes < count )
  {
    FD_ZERO(&wset);
    fdset_nowarn(sd, &wset);

    // wait for input with timeout
    if( bNonBlocking )
    {
      timer_mark(&tstart);

      // (re)load timeout (gets munged after each select())
      timeout.tv_sec  = (time_t)(usec / 1000000);
      timeout.tv_usec = (time_t)(usec % 1000000);

      nFd = select(sd+1, NULL, &wset, NULL, &timeout);
    }

    // block indefinitely for input
    else 
    {
      nFd = select(sd+1, NULL, &wset, NULL, NULL);
    }

    // system error occurred on select - interpret
    if( nFd < 0 )
    {
      switch(errno)
      {
        case EAGAIN:        // non-blocking timeout
        case EINTR:         // write was interrupted
          break;
        default:
          BSCLIENT_LOG_SYSERROR(pClient, BS_ECODE_BAD_SEND,
              "select(%d,...)", sd);
          return -BS_ECODE_BAD_RECV;
      }
    }

    // select() timeout occurred
    else if( nFd == 0 )
    {
      LOGDIAG4("select() on write timed out.");
      break;
    }

    // data available from serial device 
    n = write(sd, buf+nBytes, count-(size_t)nBytes);

    // system error occurred on write - interpret
    if( n < 0 )
    {
      switch(errno)
      {
        case EAGAIN:        // non-blocking timeout
        case EINTR:         // write was interrupted
          n = 0;
          break;
        default:
          BSCLIENT_LOG_SYSERROR(pClient, BS_ECODE_BAD_SEND,
              "select(%d,...)", sd);
          return -BS_ECODE_BAD_RECV;
      }
    }

    // sent some data
    nBytes += (int)n;

    // all of the requested bytes have been written
    if( nBytes == count )
    {
      break;
    }

    // determine time left for non-blocking write timeout
    if( bNonBlocking && ((size_t)nBytes < count) )
    {
      tdelta = timer_elapsed(&tstart);
      if( tdelta >= usec )
      {
        LOGDIAG4("%s() timed out.", LOGFUNCNAME);
        break;
      }
      else
      {
        usec -= tdelta;
      }
    }
  }

  LOGDIAG4("%s(): %d bytes written.", LOGFUNCNAME, nBytes);

  return nBytes;
}

/*!
 * \brief Flush input of <em>count</em> bytes.
 *
 * \param pClient   \h_botsense client.
 * \param count     Number of bytes to flush.
 */
static void bsFlushInput(BsClient_T *pClient, size_t count)
{
  int     sd;
  byte_t  buf[1024];
  size_t  block;
  int     n;

  sd = SocketAttrGetSd(pClient->m_pSocket);

  while( count > 0 )
  {
    block = sizeof(buf) >= count? count: sizeof(buf);
    n = bsClientRead(pClient, sd, buf, block, BSCLIENT_T_FLUSH);
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
 * \param pClient   \h_botsense client.
 */
static void bsResync(BsClient_T *pClient)
{
  int   nMaxTries = 5;
  int   nTries;
  int   rc;

  for(nTries=0; nTries<nMaxTries; ++nTries)
  {
    bsFlushInput(pClient, BSPROXY_MSG_MAX_LEN);
    if( (rc = bsServerReqLoopback(pClient, "resync")) == BS_OK )
    {
      return;
    }
  }
  BSCLIENT_LOG_ERROR(pClient, BS_ECODE_BAD_RESYNC, "");
}

/*!
 * \brief Send a request message to the server.
 *
 * \param pClient       \h_botsense client.
 * \param [in] pMsgHdr  Filled-in request message header.
 * \param [in] bufReq   Buffer with empty header bytes plus packed request
 *                      message body.
 * \param bufSize       Size of request buffer.
 *
 * \return Returns number of bytes sent on success.\n
 * Else returns \h_lt 0 error code.
 */
static int bsClientSendReq(BsClient_T      *pClient,
                           BsProxyMsgHdr_T *pMsgHdr,
                           byte_t           bufReq[],
                           size_t           bufSize)
{
  int         sd;
  size_t      uBodyLen;
  uint_t      uMsgId;
  int         nBytes;
  int         n;

  sd        = SocketAttrGetSd(pClient->m_pSocket);
  uMsgId    = pMsgHdr->m_hdrMsgId;
  uBodyLen  = (size_t)(pMsgHdr->m_hdrBodyLen);
  nBytes    = BSPROXY_MSG_HDR_LEN + (int)uBodyLen;

  _BS_LOG_MSGHDR(pClient, "Tx Req", pMsgHdr);

  BSCLIENT_TRY_EXPR(pClient, (bufSize >= BSPROXY_MSG_HDR_LEN),
      BS_ECODE_BUF_TOO_SMALL, "MsgId=%u: buf_size=%zu", bufSize);

  BSCLIENT_TRY_EXPR(pClient, (uBodyLen <= BSPROXY_MSG_BODY_MAX),
      BS_ECODE_MSG_TOO_BIG, "MsgId=%u: Message body_len=%zu > max=%zu",
      uMsgId, uBodyLen, BSPROXY_MSG_BODY_MAX);

  // prepend header to request buffer
  n = bsClientPackMsgHdr(pClient, pMsgHdr, bufReq, BSPROXY_MSG_HDR_LEN);

  BSCLIENT_TRY_ECODE(pClient, n,
      "MsgId=%u: Failed to pack message header", uMsgId);

  bsTransLock(pClient);

  // send the message to the server
  n = bsClientWrite(pClient, sd, bufReq, (size_t)nBytes,
                                                    pClient->m_uReqTimeout);

  bsTransUnlock(pClient);

  BSCLIENT_TRY_EXPR(pClient, (n == nBytes), BS_ECODE_BAD_SEND,
      "MsgId=%u: Failed to send message", uMsgId);

  LOGDIAG3("MsgId=%u: Sent %d byte request.", uMsgId, nBytes);

  return nBytes;
}

/*!
 * \brief Read response message from server.
 *
 * \param pClient         \h_botsense client.
 * \param [out] pMsgHdr   Response message header.
 * \param [out] bufRsp    Buffer with packed response message body.
 * \param bufSize         Size of response buffer.
 *
 * \return Returns \h_lt 0 on error,
 * or \h_gt 0 number of bytes for successful read.
 */
static int bsClientRecvRsp(BsClient_T      *pClient,
                           BsProxyMsgHdr_T *pMsgHdr,
                           byte_t           bufRsp[],
                           size_t           bufSize)

{
  int         sd;
  size_t      uBodyLen;
  uint_t      uMsgId;
  int         nBytes;             
  byte_t      bufHdr[BSPROXY_MSG_HDR_LEN];
  int         n;

  // socket file descriptor
  sd = SocketAttrGetSd(pClient->m_pSocket);

  bsTransLock(pClient);

  // read response header
  n = bsClientRead(pClient, sd, bufHdr, BSPROXY_MSG_HDR_LEN,
                                      pClient->m_uRspTimeout);

  bsTransUnlock(pClient);

  BSCLIENT_TRY_EXPR(pClient, (n == BSPROXY_MSG_HDR_LEN), BS_ECODE_BAD_RECV,
        "Failed to receive response message header, %d bytes received.", n);

  // unpack message header
  n = bsClientUnpackMsgHdr(pClient, bufHdr, (size_t)BSPROXY_MSG_HDR_LEN,
                            pMsgHdr);

  BSCLIENT_TRY_ECODE(pClient, n,   
        "Received bad response message header, discarding.");

  _BS_LOG_MSGHDR(pClient, "Rx Rsp", pMsgHdr);

  uBodyLen  = (size_t)(pMsgHdr->m_hdrBodyLen);
  uMsgId    = pMsgHdr->m_hdrMsgId;
  nBytes    = BSPROXY_MSG_HDR_LEN + (int)uBodyLen;

  // validate magic
  if( pMsgHdr->m_hdrMagic != BSPROXY_MSG_MAGIC )
  {
    BSCLIENT_LOG_ERROR(pClient, BS_ECODE_MSG_BAD_HDR,
      "MsgId=%u: " 
      "Received bad magic 0x%04x in response message header, resyncing.",
      uMsgId, pMsgHdr->m_hdrMagic);
    bsResync(pClient);
    return -BS_ECODE_MSG_BAD_HDR;
  }

  // message too long
  if( uBodyLen > bufSize )
  {
    BSCLIENT_LOG_ERROR(pClient, BS_ECODE_MSG_TOO_BIG,
      "MsgId=%u: Message body_len=%zu > buf_size=%zu, discarding.",
      uMsgId, uBodyLen, bufSize);
    bsFlushInput(pClient, uBodyLen);
    return -BS_ECODE_MSG_TOO_BIG;
  }

  // read response body
  if( uBodyLen > 0 )
  {
    // read response body
    n = bsClientRead(pClient, sd, bufRsp, uBodyLen, pClient->m_uRspTimeout);

    BSCLIENT_TRY_EXPR(pClient, (n == (int)uBodyLen), BS_ECODE_BAD_RECV,
      "MsgId=%u: Failed to receive response message body.", uMsgId);
  }

  LOGDIAG3("MsgId=%u: Received %d byte response.", uMsgId, nBytes);

  return nBytes;
}


// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Get client name.
 *
 * \param pClient   \h_botsense client.
 *
 * \return 
 * On success, returns client name string. Else returns NULL.
 */
const char *bsClientAttrGetName(BsClient_P pClient)
{
  return pClient!=NULL? pClient->m_sClientName: NULL;
}

/*!
 * \brief Get client request (write) and response (read) timeouts.
 *
 * \param pClient           \h_botsense client.
 * \param [out] pReqTimeout Request (write) timeout in seconds.
 * \param [out] pRspTimeout Response (read) timeout in seconds.
 */
void bsClientAttrGetTimeouts(BsClient_P  pClient,
                             uint_t     *pReqTimeout,
                             uint_t     *pRspTimeout)
{
  if( pClient != NULL )
  {
    *pReqTimeout = pClient->m_uReqTimeout / 1000000;
    *pRspTimeout = pClient->m_uRspTimeout / 1000000;
  }
}

/*!
 * \brief Set client request (write) and response (read) timeouts.
 *
 * A timeout value of 0 means block forever until i/o operation is complete.
 *
 * \param pClient           \h_botsense client.
 * \param [in] pReqTimeout  Request (write) timeout in seconds.
 * \param [in] pRspTimeout  Response (read) timeout in seconds.
 */
void bsClientAttrSetTimeouts(BsClient_P pClient,
                             uint_t     uReqTimeout,
                             uint_t     uRspTimeout)
{
  if( pClient != NULL )
  {
    pClient->m_uReqTimeout = uReqTimeout * 1000000;
    pClient->m_uRspTimeout = uRspTimeout * 1000000;
  }
}

/*!
 * \brief Get client virtual connection trace state.
 *
 * \param pClient   \h_botsense client.
 * \param hndVConn  Virtual connection handle.
 *
 * \return 
 * Returns true or false.
 */
bool_t bsClientAttrGetTraceState(BsClient_P pClient, BsVConnHnd_T hndVConn)
{
  if( pClient == NULL )
  {
    return false;
  }
  else if( hndVConn == BSPROXY_VCONN_SERVER )
  {
    return pClient->m_bTraceServer;
  }
  else if( bsClientAttrHasVConn(pClient, hndVConn) )
  {
    return bsGetVConn(pClient, hndVConn)->m_bTrace;
  }
  else
  {
    return false;
  }
}

/*!
 * \brief Get the number of active virtual connections for this client.
 *
 * \param pClient   \h_botsense client.
 *
 * \return 
 * Number of virtual connections.
 */
int bsClientAttrGetVConnCount(BsClient_P pClient)
{
  if( pClient == NULL )
  {
    return 0;
  }
  else
  {
    return pClient->m_nVConnCount;
  }
}

/*!
 * \brief Test if client has a virtual connection identified by the handle.
 *
 * \param pClient   \h_botsense client.
 * \param hndVConn  Virtual connection handle.
 *
 * \return 
 * Returns true or false.
 */
bool_t bsClientAttrHasVConn(BsClient_P pClient, BsVConnHnd_T hndVConn)
{
  int   index;

  if( pClient == NULL )
  {
    return false;
  }
  else if( !BSCLIENT_IS_VCONN_HANDLE(hndVConn) )
  {
    return false;
  }
  else if( (index = pClient->m_tblHndIndex[hndVConn]) == BSPROXY_VCONN_UNDEF )
  {
    return false;
  }
  else if( pClient->m_tblVConn[index] == NULL )
  {
    return false;
  }
  else
  {
    return true;
  }
}

/*!
 * \brief Get client virtual connection device name.
 *
 * \param pClient   \h_botsense client.
 * \param hndVConn  Virtual connection handle.
 *
 * \return
 * On success, returns device name string. Else returns NULL.
 */
const char *bsClientAttrGetDevName(BsClient_P pClient, BsVConnHnd_T hndVConn)
{
  if( pClient == NULL )
  {
    return NULL;
  }
  else if( hndVConn == BSPROXY_VCONN_SERVER )
  {
    return "server";
  }
  else if( !bsClientAttrHasVConn(pClient, hndVConn) )
  {
    return NULL;
  }
  else
  {
    return pClient->m_tblVConn[hndVConn]->m_sDevName;
  }
}

/*!
 * \brief Get client virtual connection interface module name.
 *
 * \param pClient   \h_botsense client.
 * \param hndVConn  Virtual connection handle.
 *
 * \return
 * On success, returns interface module name string. Else returns NULL.
 */
const char *bsClientAttrGetModName(BsClient_P pClient, BsVConnHnd_T hndVConn)
{
  if( pClient == NULL )
  {
    return NULL;
  }
  else if( hndVConn == BSPROXY_VCONN_SERVER )
  {
    return "server";
  }
  else if( !bsClientAttrHasVConn(pClient, hndVConn) )
  {
    return NULL;
  }
  else
  {
    return pClient->m_tblVConn[hndVConn]->m_sModName;
  }
}

/*!
 * \brief Set client's diagnostics logging threshold.
 *
 * \param pClient   \h_botsense client.
 * \param nLevel    New logging threshold level.
 */
void bsClientAttrSetLogging(BsClient_P pClient, int nLevel)
{
  LOG_SET_THRESHOLD(nLevel);
}

/*!
 * \brief Get client's connection state.
 *
 * \param pClient           \h_botsense client.
 * \param [out] pConnState  Pointer to connection state.
 */
void bsClientAttrGetConnState(BsClient_P           pClient,
                              BsClientConnState_T *pConnState)
{
  static BsClientConnState_T  noconn = {false, ""};

  Socket_T *pSocket = pClient->m_pSocket;

  if( (pSocket == NULL) || !SocketStateIsOpen(pSocket) )
  {
    *pConnState = noconn;
  }

  else
  {
    pConnState->m_bIsConnected    = true;
    pConnState->m_sServerHostName = SocketAttrGetRemoteName(pSocket);
  }
}

/*!
 * \brief Fill in message header.
 *
 * \param pClient       \h_botsense client.
 * \param hndVConn      Virtual connection handle.
 * \param uMsgId        Virtual connection unique message id.
 * \param uBodyLen      Message body length.
 * \param [out] pMsgHdr Filled message header.
 */
void bsClientFillMsgHdr(BsClient_P       pClient,
                        int              hndVConn,
                        uint_t           uMsgId,
                        size_t           uBodyLen,
                        BsProxyMsgHdr_T *pMsgHdr)
{
  pMsgHdr->m_hdrMagic   = (ushort_t)BSPROXY_MSG_MAGIC;
  pMsgHdr->m_hdrTid     = (byte_t)bsNextTid(pClient);
  pMsgHdr->m_hdrVConn   = (byte_t)hndVConn;
  pMsgHdr->m_hdrMsgId   = (ushort_t)uMsgId;
  pMsgHdr->m_hdrBodyLen = (ushort_t)uBodyLen;
}

/*!
 * \brief Get the message name.
 *
 * For each (virtual connection, message id) 2-tuple, there can be a known
 * name string (provided the id is valid and an application provides the
 * information).
 *
 * \param pClient   \h_botsense client.
 * \param hndVConn  Virtual connection handle.
 * \param uMsgId    Message id.
 *
 * \return
 * Returns message name if it can be determined. Otherwise returns "unknown".
 */
const char *bsClientGetMsgName(BsClient_P   pClient,
                               BsVConnHnd_T hndVConn,
                               uint_t       uMsgId)
{
  static const char       *sUnknown = "unknown";
  const NMMsgDef_T        *pMsgDef;
  BsVConn_T               *pVConn;
  const BsClientAppInfo_T *pAppInfo;

  if( hndVConn == BSPROXY_VCONN_SERVER )
  {
    pMsgDef = BsProxyLookupMsgDef((BsProxyMsgId_T)uMsgId);
    return pMsgDef!=NULL? pMsgDef->m_sMsgName: sUnknown;
  }
  else if( !bsClientAttrHasVConn(pClient, hndVConn) )
  {
    return sUnknown;
  }
  else if( (pVConn = bsGetVConn(pClient, hndVConn)) == NULL )
  {
    return sUnknown;
  }
  else if( (pAppInfo = pVConn->m_pAppInfo) == NULL ) 
  {
    return sUnknown;
  }
  else if( pAppInfo->fnGetMsgName == NULL )
  {
    return sUnknown;
  }
  else
  {
    return pAppInfo->fnGetMsgName(pClient, hndVConn, uMsgId);
  }
}

#ifdef LOG
/*!
 * \brief Log [pre/un]packed message header.
 *
 * \param pClient   \h_botsense client.
 * \param sPreface  Preface string.
 * \param pMsgHdr   Pointer to message header structure.
 */
void bsClientLogMsgHdr(BsClient_P       pClient,
                       const char      *sPreface,
                       BsProxyMsgHdr_T *pMsgHdr)
{
  FILE        *fp;
  const char  *sMsgName;

  fp = LOG_GET_LOGFP();

  if( pMsgHdr != NULL )
  {
    sMsgName = bsClientGetMsgName(pClient,
                                  (BsVConnHnd_T)pMsgHdr->m_hdrVConn,
                                  (uint_t)pMsgHdr->m_hdrMsgId);

    fprintf(fp, "%s MsgHdr = {\n", sPreface);
    fprintf(fp, "  Magic:   0x%04x\n",  pMsgHdr->m_hdrMagic);
    fprintf(fp, "  Tid:     %u\n",      (uint_t)(pMsgHdr->m_hdrTid));
    fprintf(fp, "  VConn:   %u\n",      (uint_t)(pMsgHdr->m_hdrVConn));
    fprintf(fp, "  MsgId:   %u %s \n",  pMsgHdr->m_hdrMsgId, sMsgName);
    fprintf(fp, "  BodyLen: %u\n",      pMsgHdr->m_hdrBodyLen);
    fprintf(fp, "}\n");
  }
  else
  {
    fprintf(fp, "%s MsgHdr: (null)\n", sPreface);
  }
}
#endif // LOG

/*!
 * \brief Execute a request - response transaction with the server.
 *
 * \note The request and response buffer can be the same buffer if packed 
 * request contents does not need to be preserved.
 *
 * \warning There must be exactly \ref BSPROXY_MSG_HDR_LEN bytes at the front
 * of the request buffer available to pack the \h_botsense header. (Reduces the
 * number of buffer copies.)
 *
 * \par Request Format:
 *  msghdr msgbody
 *
 * \par Response Format:
 *  msghdr msgbody
 *
 * \param pClient       \h_botsense client.
 * \param hndVConn      Virtual connection handle.
 * \param uReqMsgId     Virtual connection unique request message id.
 * \param [in] bufReq   Buffer with empty header plus packed request
 *                      message body.
 * \param uReqBodyLen   Length of packed request message body.
 * \param uRspMsgId     Virtual connection unique expected response message id.
 * \param [out] bufRsp  Buffer with received packed response message body. 
 * \param sizeRspBuf    Size of response buffer (number of bytes).
 *
 * \return 
 * On success, returns \h_ge 0 the number of response body bytes received.\n
 * \copydoc doc_return_ecode
 */
int bsClientTrans(BsClient_P  pClient,
                  int         hndVConn,
                  uint_t      uReqMsgId,
                  byte_t      bufReq[],
                  size_t      uReqBodyLen,
                  uint_t      uRspMsgId,
                  byte_t      bufRsp[],
                  size_t      sizeRspBuf)
{
  BsProxyMsgHdr_T   hdrReq;       // request message header
  BsProxyMsgHdr_T   hdrRsp;       // response message header
  BsTid_T           tidReq;       // request transaction id
  BsProxyRspErr_T   msgRspErr;    // common error response message
  bool_t            bPending;     // resposne is [not] pending
  int               n;            // number of bytes/return code

  // fill request header
  bsClientFillMsgHdr(pClient, hndVConn, uReqMsgId, uReqBodyLen, &hdrReq);

  // this request's transaction id
  tidReq = hdrReq.m_hdrTid;

  // mark request for tracking
  bsTransMark(pClient, tidReq);

  // send request
  n = bsClientSendReq(pClient, &hdrReq, bufReq,
                            BSPROXY_MSG_HDR_LEN+uReqBodyLen);

  BSCLIENT_TRY_ECODE(pClient, n,
      "MsgId=%u: Failed to send request.", uReqMsgId);

  // log successful request sent event
  _BS_LOG_REQ(pClient, &hdrReq);

  bPending = true;

  // receive response
  while( bPending )
  {
    // check transaction cache for any cached response.
    n = bsTransLoadCached(pClient, tidReq, &hdrRsp, bufRsp, sizeRspBuf);

    BSCLIENT_TRY_ECODE(pClient, n,
        "MsgId=%u: Cached response failed for request.", uReqMsgId);

    // found a good, cached response
    if( n > 0 )
    {
      bsTransForget(pClient, tidReq);
      bPending = false;
    }

    // receive a response
    else
    {
      n = (int)bsClientRecvRsp(pClient, &hdrRsp, bufRsp, sizeRspBuf);

      BSCLIENT_TRY_ECODE(pClient, n,
        "MsgId=%u: Receive response failed for request.", uReqMsgId);

      // response was for this request
      if( tidReq == hdrRsp.m_hdrTid )
      {
        bsTransForget(pClient, tidReq);
        bPending = false;
      }

      // response is for another thread - cache it
      else
      {
        bsTransCacheRsp(pClient, tidReq, &hdrRsp, bufRsp);
      }
    }
  }

  n = (int)hdrRsp.m_hdrBodyLen;

  // received common error response
  if( (hdrRsp.m_hdrVConn == BSPROXY_VCONN_SERVER) &&
      (hdrRsp.m_hdrMsgId == BsProxyMsgIdRspErr) )
  {
    n = BsProxyUnpackRspErr(bufRsp, (size_t)n, &msgRspErr, false);

    BSCLIENT_TRY_ECODE(pClient, n,
        "MsgId=%u: Failed to unpack error response message body.",
        hdrRsp.m_hdrMsgId);

    n = -(int)(msgRspErr.m_ecode);

    BSCLIENT_LOG_ERROR(pClient, n, "%s", msgRspErr.m_emsg);

    return n;
  }

  // received application-specific response
  else
  {
    BSCLIENT_TRY_EXPR(pClient, (hdrRsp.m_hdrMsgId == uRspMsgId),
        BS_ECODE_MSG_BAD_HDR,
        "MsgId=%u: Unexpected response message id, expected MsgId=%u.",
        hdrRsp.m_hdrMsgId, uRspMsgId);
  }

  // log response received event
  _BS_LOG_RSP(pClient, &hdrRsp);

  return n;
}

/*!
 * \brief Create a new unconnected proxied client.
 *
 * \param sClientName   Proxied client (robot)'s name.
 *
 * \return Returns pointer to new client on success. Else returns NULL.
 */
BsClient_T *bsClientNew(const char *sClientName)
{
  BsClient_T     *pClient = NEW(BsClient_T);
  size_t          i;

  pClient->m_sClientName  = new_strdup(sClientName);
  pClient->m_pSocket      = NULL;

  pthread_mutex_init(&(pClient->m_mutexTrans), NULL);

  pClient->m_uReqTimeout  = BSCLIENT_T_WRITE;
  pClient->m_uRspTimeout  = BSCLIENT_T_READ;
  pClient->m_uTidCounter  = 0;
  pClient->m_bTraceServer = false;
      
  for(i=0; i<arraysize(pClient->m_tblTransCache); ++i)
  {
    pClient->m_tblTransCache[i] = NULL;
  }

  pClient->m_nVConnCount = 0;

  for(i=0; i<arraysize(pClient->m_tblHndIndex); ++i)
  {
    pClient->m_tblHndIndex[i] = BSPROXY_VCONN_UNDEF;
  }

  for(i=0; i<arraysize(pClient->m_tblVConn); ++i)
  {
    pClient->m_tblVConn[i] = NULL;
  }

  return pClient;
}

/*!
 * \brief Delete a proxied client.
 *
 * \warning The client should disconnect prior to deletion.
 *
 * \param pClient   \h_botsense proxied client.
 */
void bsClientDelete(BsClient_P pClient)
{
  size_t i;

  if( pClient == NULL )
  {
    return;
  }

  if( SocketStateIsOpen(pClient->m_pSocket) )
  {
    BSCLIENT_LOG_WARN(pClient, BS_ECODE_GEN,
        "Connection still open, auto-disconnecting.");
    (void)bsServerDisconnect(pClient);
  }

  for(i=0; i<arraysize(pClient->m_tblTransCache); ++i)
  {
    bsTransForget(pClient, (BsTid_T)i);
  }

  bsVConnClearAll(pClient);

  pthread_mutex_destroy(&(pClient->m_mutexTrans));

  delete((void *)(pClient->m_sClientName));

  delete(pClient);
}


// ---------------------------------------------------------------------------
// Internal Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Remove and delete all of a client's vConnections.
 *
 * \param pClient   \h_botsense proxied client.
 */ 
void bsVConnClearAll(BsClient_T *pClient)
{
  size_t  nHnd;
  int     index;

  for(nHnd=0; nHnd<arraysize(pClient->m_tblHndIndex); ++nHnd)
  {
    index = (int)pClient->m_tblHndIndex[nHnd];
    if( index != BSPROXY_VCONN_UNDEF )
    {
      bsVConnRemove(pClient, (BsVConnHnd_T)nHnd);
      bsVConnDelete(pClient, index);
    }
  }
}

/*!
 * \brief Add a created vConnection to the client's tblHndIndex table.
 *
 * \note The new vConnection should have been created and reserved prior
 * to adding.
 *
 * \param pClient   \h_botsense client.
 * \param hndVConn  New virtual connection proxied device handle.
 * \param index     Internal vConnection index.
 *
 * \copydoc doc_return_std
 */
int bsVConnAdd(BsClient_T  *pClient,
               BsVConnHnd_T hndVConn,
               int          index)
{
  int   i;
  int   rc;

  BSCLIENT_TRY_EXPR(pClient, BSCLIENT_IS_VCONN_HANDLE(hndVConn),
      BS_ECODE_INTERNAL, "VConn=%d: out-of-range.", hndVConn);

  BSCLIENT_TRY_EXPR(pClient, (index < BSPROXY_VCONN_CLIENT_MAX),
      BS_ECODE_INTERNAL, "index=%d: out-of-range.", index);

  bsTransLock(pClient);

  i = (int)pClient->m_tblHndIndex[hndVConn];

  if( i != BSPROXY_VCONN_UNDEF )
  {
    BSCLIENT_LOG_ERROR(pClient, BS_ECODE_INTERNAL,
      "VConn=%d: index=%d: handle already assigned: "
      "Internal vconn tables corrupted.", hndVConn, i);
    rc = -BS_ECODE_INTERNAL;
  }

  else if( pClient->m_tblVConn[index] == NULL )
  {
    BSCLIENT_LOG_ERROR(pClient, BS_ECODE_INTERNAL,
      "VConn=%d: index=%d: No vConnection present: "
      "Internal vconn tables corrupted.", hndVConn, index);
    rc = -BS_ECODE_INTERNAL;
  }

  else
  {
    pClient->m_tblVConn[index]->m_hndVConn = hndVConn;
    pClient->m_tblHndIndex[hndVConn] = (byte_t)index;
    pClient->m_nVConnCount++;
    rc = BS_OK;
  }

  bsTransUnlock(pClient);

  return rc;
}

/*!
 * \brief Remove a vConnection from the client's tblHndIndex table.
 *
 * \note The vConnection stills needs to be deleted.
 *
 * \param pClient   \h_botsense client.
 * \param hndVConn  New virtual connection proxied device handle.
 *
 * \return
 * On success, returns the removed, but still reserved, internal vConnection
 * index.\n
 * \copydoc doc_return_ecode
 */
int bsVConnRemove(BsClient_T *pClient, BsVConnHnd_T hndVConn)
{
  int   index;
  int   rc;

  BSCLIENT_TRY_EXPR(pClient, BSCLIENT_IS_VCONN_HANDLE(hndVConn),
      BS_ECODE_INTERNAL, "VConn=%d: out-of-range.", hndVConn);

  bsTransLock(pClient);

  index = pClient->m_tblHndIndex[hndVConn];

  if( index == BSPROXY_VCONN_UNDEF )
  {
    BSCLIENT_LOG_ERROR(pClient, BS_ECODE_INTERNAL,
      "VConn=%d: vConnection not present in table.", hndVConn);
    rc = -BS_ECODE_INTERNAL;
  }

  else
  {
    pClient->m_tblVConn[index]->m_hndVConn = BSPROXY_VCONN_UNDEF;
    pClient->m_tblHndIndex[hndVConn] = BSPROXY_VCONN_UNDEF;
    pClient->m_nVConnCount--;

    rc = index;
  }

  bsTransUnlock(pClient);

  return rc;
}

/*!
 * \brief Reserve a new client vConnection (and proxied device) in the
 * client's tblVConn table.
 *
 * \note The new vConnection still needs to be added.
 *
 * \param pClient   \h_botsense client.
 * \param sDevName  Proxied device file path name.
 * \param sModName  Interface module file path name.
 * \param pAppInfo  Static application-specific information and callbacks.
 * \param bTrace    Do [not] enable message tracing.
 *
 * \return
 * On success, returns reserved internal vConnection index.\n
 * \copydoc doc_return_ecode
 */
int bsVConnNew(BsClient_T              *pClient,
               const char              *sDevName,
               const char              *sModName,
               const BsClientAppInfo_T *pAppInfo,
               bool_t                   bTrace)
{
  int         index;
  BsVConn_T  *pVConn;
  int         rc = BS_OK;

  bsTransLock(pClient);

  if( bsClientAttrGetVConnCount(pClient) >= BSPROXY_VCONN_CLIENT_MAX )
  {
    BSCLIENT_LOG_ERROR(pClient, BS_ECODE_NO_RSRC,
        "Client has the maximum=%d virtual connections.",
        BSPROXY_VCONN_CLIENT_MAX);
    rc = -BS_ECODE_NO_RSRC;
  }

  else
  {
    for(index=0; index<BSPROXY_VCONN_CLIENT_MAX; ++index)
    {
      if( pClient->m_tblVConn[index] == NULL )
      {
        break;
      }
    }

    if( index >= BSPROXY_VCONN_CLIENT_MAX )
    {
      BSCLIENT_LOG_ERROR(pClient, BS_ECODE_INTERNAL,
                      "Internal vconn tables corrupted.");
      rc = -BS_ECODE_INTERNAL;
    }
  }

  if( rc == BS_OK )
  {
    pVConn = NEW(BsVConn_T);

    pVConn->m_hndVConn  = BSPROXY_VCONN_UNDEF;
    pVConn->m_bTrace    = bTrace;
    pVConn->m_sDevName  = new_strdup(sDevName);
    pVConn->m_sModName  = new_strdup(sModName);
    pVConn->m_pAppInfo  = pAppInfo;

    // reserve slot
    pClient->m_tblVConn[index] = pVConn;
    
    rc = index;
  }

  bsTransUnlock(pClient);

  return rc;
}

/*!
 * \brief Delete a vConnection (and proxied device) from the client's
 * tblVConn table.
 *
 * \note The vConnection should be removed first prior to deletion. 
 *
 * \param pClient   \h_botsense client.
 * \param index     Internal vConnection index.
 *
 * \copydoc doc_return_std
 */
int bsVConnDelete(BsClient_T *pClient, int index)
{
  BsVConn_T  *pVConn;
  int         rc;

  BSCLIENT_TRY_EXPR(pClient, (index < BSPROXY_VCONN_CLIENT_MAX),
      BS_ECODE_INTERNAL, "index=%d: out-of-range.", index);

  bsTransLock(pClient);

  pVConn = pClient->m_tblVConn[index];

  if( pVConn == NULL )
  {
    BSCLIENT_LOG_ERROR(pClient, BS_ECODE_INTERNAL,
      "index=%u: vConnection not present in table.", index);
    rc = -BS_ECODE_INTERNAL;
  }

  else
  {
    delete((char *)(pVConn->m_sDevName));
    delete((char *)(pVConn->m_sModName));
    delete(pVConn);

    // free slot
    pClient->m_tblVConn[index] = NULL;

    rc = BS_OK;
  }

  bsTransUnlock(pClient);

  return rc;
}
