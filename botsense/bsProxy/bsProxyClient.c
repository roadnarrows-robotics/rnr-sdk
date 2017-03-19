////////////////////////////////////////////////////////////////////////////////
//
// Package:   BotSense
//
// Program:   bsProxy
//
// File:      bsProxyClient.c
//
/*! \file
 *
 * $LastChangedDate: 2010-08-20 11:36:38 -0600 (Fri, 20 Aug 2010) $
 * $Rev: 568 $
 *
 * \brief \h_botsense bsProxy client routines.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2007-2017. RoadNarrows LLC.\n
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
#include <string.h>
#include <stdarg.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/new.h"
#include "rnr/sock.h"
#include "rnr/netmsgs.h"

#include "botsense/BotSense.h"
#include "botsense/bsProxyMsgs.h"

#include "bsProxy.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

//
// Local data
//
static pthread_mutex_t  BsClientBusyMutex;  ///< busy mutex
static pthread_cond_t   BsClientBusyCond;   ///< busy condition

/*!
 * \brief Convert pointer to client to client handle.
 * 
 * \param pClient   \h_botsense client.
 */
#define CLIENT_HND(pClient) \
  ServerClientSd2Hnd(SocketAttrGetSd(pClient->m_pClientSock))

//.............................................................................
// Mutual Exclusion Functions
//.............................................................................

/*!
 * \brief Lock client's global busy mutual exclusion.
 */
static inline void ClientLockBusy()
{
  int rc;

  if( (rc = pthread_mutex_lock(&BsClientBusyMutex)) != 0 ) 
  { 
    errno = rc;
    LOGSYSERROR("pthread_mutex_lock()");
  }
}

/*!
 * \brief Unlock client's global busy mutual exclusion.
 */
static inline void ClientUnlockBusy()
{
  int rc;

  if( (rc = pthread_mutex_unlock(&BsClientBusyMutex)) != 0 ) 
  { 
    errno = rc;
    LOGSYSERROR("pthread_mutex_unlock()");
  }
}

/*!
 * \brief Try to lock client's global busy mutual exclusion.
 *
 * \return
 * Returns true if lock is acquired. Otherwise returns false if mutex already
 * locked.
 */
static inline bool_t ClientBusyTryLock()
{
  return pthread_mutex_trylock(&BsClientBusyMutex) == 0? true: false;
}

/*!
 * \brief Broadcast that a client has been freed or deleted.
 * 
 * A broadcast will unblock all threads currently blocked on the busy condition
 * variable. Only those threads waiting on the client whose condition has
 * changed will run. If multiple threads are waiting on the same client, then
 * only one is schedule to run.
 */
static inline void ClientBroadcastNotBusy()
{
  int rc;

  if( (rc = pthread_cond_broadcast(&BsClientBusyCond)) != 0 )
  { 
    errno = rc;
    LOGSYSERROR("pthread_cond_broadcast()");
  }
}

/*!
 * \brief Wait on a client to become free.
 */
static inline void ClientWaitNotBusy()
{
  int rc;

  if( (rc = pthread_cond_wait(&BsClientBusyCond, &BsClientBusyMutex)) != 0 )
  { 
    errno = rc;
    LOGSYSERROR("pthread_cond_wait()");
  }
}


//.............................................................................
// Basic I/O Functions
//.............................................................................

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

/*!
 * \brief Read count bytes from socket.
 *
 * \param pClient     \h_botsense client.
 * \param [out] buf   Destination buffer.
 * \param uCount      Number of bytes to read.
 *
 * \return Returns <0 on error, 0 on time out, or \h_ge 0 for the number of
 * bytes read.
 */
static ssize_t ClientRead(BsProxyClientCtl_T *pClient,
                          byte_t              buf[],
                          size_t              uCount)
{
  uint_t          usec = BSPROXY_TUNE_T_RECV;
  struct timeval  tstart;
  uint_t          tdelta;
  struct timeval  timeout;
  fd_set          rset;
  int             sd;
  int             nFd;
  ssize_t         n;
  ssize_t         nBytes = 0;

  LOGDIAG4CALL(_TSTR(ClientThisHasName(pClient)), _TPTR(buf), _TUINT(uCount));

  // client is disconnected or in an unrecoverable state
  if( pClient->m_eClientState == BsProxyClientStateZombie )
  {
    return -BS_ECODE_SERVER_BAD_CLIENT;
  }

  // socket descriptor
  sd = SocketAttrGetSd(pClient->m_pClientSock);

  //
  // Read the data until either 1) count bytes are read, 2) a time out occurs,
  // or 3) an error occurs.
  //
  while( nBytes < uCount )
  {
    FD_ZERO(&rset);
    fdset_nowarn(sd, &rset);

    // mark now
    timer_mark(&tstart);

    // (re)load timeout (gets munged after each select())
    timeout.tv_sec  = (time_t)(usec / 1000000);
    timeout.tv_usec = (time_t)(usec % 1000000);

    // wait to bytes to be available to be read
    nFd = select(sd+1, &rset, NULL, NULL, &timeout);

    // system error occurred on select - interpret
    if( nFd < 0 )
    {
      switch(errno)
      {
        case EAGAIN:        // non-blocking timeout
        case EINTR:         // read was interrupted
          break;
        default:            // non-recoverable error
          BSPROXY_LOG_SYSERROR(ServerClientSd2Hnd(sd), "select(%d,...)", sd);
          return -BS_ECODE_BAD_RECV;
      }
    }

    // select() timeout occurred
    else if( nFd == 0 )
    {
      LOGDIAG4("select() on read timed out.");
      break;
    }

    // read the available data from the socket
    n = read(sd, buf+nBytes, uCount-(size_t)nBytes);

    // system error occurred on read - interpret
    if( n < 0 )
    {
      switch(errno)
      {
        case EAGAIN:        // non-blocking timeout
        case EINTR:         // read was interrupted
          n = 0;
          break;
        default:            // non-recoverable error
          BSPROXY_LOG_SYSERROR(ServerClientSd2Hnd(sd), "select(%d,...)", sd);
          return -BS_ECODE_BAD_RECV;
      }
    }

    // got some data
    nBytes += n;

    // all of the requested bytes have been read
    if( nBytes == uCount )
    {
      break;
    }

    // determine time left for the non-blocking read timeout
    if( nBytes < uCount )
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

  LOGDIAG4("%s(): %zd bytes read.", LOGFUNCNAME, nBytes);

  return nBytes;
}

/*!
 * \brief Write count bytes to socket.
 *
 * \param pClient     \h_botsense client.
 * \param [in] buf    Source buffer.
 * \param uCount      Number of bytes to write.
 *
 * \return Returns <0 on error, 0 on time out, or \h_ge 0 for the number of
 * bytes written.
 */
static ssize_t ClientWrite(BsProxyClientCtl_T  *pClient,
                           byte_t               buf[],
                           size_t               uCount)
{
  uint_t          usec = BSPROXY_TUNE_T_SEND;
  struct timeval  tstart;
  uint_t          tdelta;
  struct timeval  timeout;
  fd_set          wset;
  int             sd;
  int             nFd;
  ssize_t         n;
  ssize_t         nBytes = 0;

  LOGDIAG4CALL(_TSTR(ClientThisHasName(pClient)), _TPTR(buf), _TUINT(uCount));

  // client is disconnected or in an unrecoverable state
  if( pClient->m_eClientState == BsProxyClientStateZombie )
  {
    return -BS_ECODE_SERVER_BAD_CLIENT;
  }

  // socket descriptor
  sd = SocketAttrGetSd(pClient->m_pClientSock);

  //
  // Write the data until either 1) count bytes are written, 2) a time out
  // occurs, or 3) an error occurs.
  //
  while( nBytes < uCount )
  {
    FD_ZERO(&wset);
    fdset_nowarn(sd, &wset);

    // mark now
    timer_mark(&tstart);

    // (re)load timeout (gets munged after each select())
    timeout.tv_sec  = (time_t)(usec / 1000000);
    timeout.tv_usec = (time_t)(usec % 1000000);

    nFd = select(sd+1, NULL, &wset, NULL, &timeout);

    // system error occurred on select - interpret
    if( nFd < 0 )
    {
      switch(errno)
      {
        case EAGAIN:        // non-blocking timeout
        case EINTR:         // write was interrupted
          break;
        default:            // non-recoverable error
          BSPROXY_LOG_SYSERROR(ServerClientSd2Hnd(sd), "select(%d,...)", sd);
          return -BS_ECODE_BAD_SEND;
      }
    }

    // select() timeout occurred
    else if( nFd == 0 )
    {
      LOGDIAG4("select() on write timed out.");
      break;
    }

    // socket is available for writing
    n = write(sd, buf+nBytes, uCount-(size_t)nBytes);

    // system error occurred on write - interpret
    if( n < 0 )
    {
      switch(errno)
      {
        case EAGAIN:        // non-blocking timeout
        case EINTR:         // write was interrupted
          n = 0;
          break;
        default:            // non-recoverable error
          BSPROXY_LOG_SYSERROR(ServerClientSd2Hnd(sd), "select(%d,...)", sd);
          return -BS_ECODE_BAD_SEND;
      }
    }

    // wrote some data
    nBytes += n;

    // all of the requested bytes have been written
    if( nBytes == uCount )
    {
      break;
    }

    // determine time left for non-blocking write timeout
    if( nBytes < uCount )
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

  LOGDIAG4("%s(): %zd bytes written.", LOGFUNCNAME, nBytes);

  return nBytes;
}

/*!
 * \brief Resync to client message stream.
 *
 * If a client receieve stream becomes unsynchronized, the message headers
 * are aliased to some offset into the stream. To resync, bsProxy searches
 * the stream for the magic bytes in the message header. Once found, the
 * reset of the header is read.
 *
 * \param pClient       \h_botsense client.
 * \param [out] bufHdr  Destination header buffer.
 *
 * \return Returns <0 on error or \h_ge 0 number of bytes for successful read.
 */
static ssize_t ClientResync(BsProxyClientCtl_T *pClient, byte_t bufHdr[])
{
  enum          ReSyncState_T     {GetMagicHi, GetMagicLo, GetRoH};
  byte_t        byMagicHi       = (byte_t)((BSPROXY_MSG_MAGIC >> 8) & 0xff);
  byte_t        byMagicLo       = (byte_t)(BSPROXY_MSG_MAGIC & 0xff);
  size_t        uCount          = BSPROXY_MSG_HDR_LEN;
  size_t        n               = 0;
  enum ReSyncState_T eCurState  = GetMagicHi;

  byte_t  buf[BSPROXY_MSG_HDR_LEN];
  ssize_t nBytes;
  int     i;

  //
  // Search stream for message header.
  //
  while( (nBytes = ClientRead(pClient, buf, uCount)) > 0 )
  {
    for(i=0; i<nBytes; ++i)
    {
      switch( eCurState )
      {
        case GetMagicHi:      // get some high magic
          if( buf[i] == byMagicHi )
          {
            bufHdr[n++] = buf[i];
            eCurState = GetMagicLo;
          }
          break;
        case GetMagicLo:      // now get some low magic
          if( buf[i] == byMagicLo )
          {
            bufHdr[n++] = buf[i];
            eCurState = GetRoH;
          }
          else    // false magic
          {
            n = 0;
            eCurState = GetMagicHi;
          }
          break;
        case GetRoH:        // get rest of header
          bufHdr[n++] = buf[i];
          break;
      }
    }

    uCount = BSPROXY_MSG_HDR_LEN - n;  

    if( uCount == 0 )
    {
      nBytes = BSPROXY_MSG_HDR_LEN;
      break;
    }
  }

  return nBytes;
}

/*!
 * \brief Flush receive stream.
 *
 * \param pClient \h_botsense client.
 * \param uCount  Number of bytes to flush.
 */
static void ClientFlushInput(BsProxyClientCtl_T *pClient, size_t uCount)
{
  byte_t  buf[256];
  ssize_t k = 0;
  size_t  n;

  do
  {
    uCount -= (size_t)k;
    n = sizeof(buf) < uCount? sizeof(buf): uCount;
  } while( (n > 0) && ((k = ClientRead(pClient, buf, n)) > 0) );
}


// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

// ...........................................................................
// Client Global Initialization and Concurrency Control
// ...........................................................................

/*!
 * \brief The \h_botsense bsProxy server one-time client control initialization.
 */
void ClientOneTimeInit()
{
  // create mutex
  pthread_mutex_init(&BsClientBusyMutex, NULL);

  // create condition
  pthread_cond_init(&BsClientBusyCond, NULL);
}

/*!
 * \brief Acquire client, locking it from other threads.
 *
 * The calling thread is blocked until the client is availble or has been
 * deleted.
 *
 * \param hndClient   Client handle.
 *
 * \return On success, returns pointer to locked client control block.
 * On failure, NULL is returned.
 */
BsProxyClientCtl_T *ClientAcquire(BsProxyClientHnd_T hndClient)
{
  BsProxyClientCtl_T *pClient;

  ClientLockBusy();

  while( ((pClient = ServerGetClient(hndClient)) != NULL) && pClient->m_bBusy )
  {
    ClientWaitNotBusy();
  }

  if( pClient )
  {
    pClient->m_bBusy = true;
  }

  ClientUnlockBusy();

  return pClient;
}

/*!
 * \brief Release the locked client.
 *
 * A broadcast is sent to all blocking threads on the freed event.
 *
 * \param hndClient   Client handle.
 */
void ClientRelease(BsProxyClientHnd_T hndClient)
{
  BsProxyClientCtl_T *pClient;

  ClientLockBusy();

  if( (pClient = ServerGetClient(hndClient)) != NULL )
  {
    pClient->m_bBusy = false;
  }

  ClientBroadcastNotBusy();

  ClientUnlockBusy();
}


// ...........................................................................
// Client Create and Destroy
// ...........................................................................

/*!
 * \brief Create new client control structure.
 *
 * \param pSockClient   Accecpted client socket.
 *
 * \return Returns a newly allocated and intialized client control block.
 */
BsProxyClientCtl_T *ClientNew(Socket_T *pSockClient)
{
  BsProxyClientCtl_T  *pClient;

  pClient                 = NEW(BsProxyClientCtl_T);
  pClient->m_sClientName  = new_strdup(SocketAttrGetRemoteName(pSockClient));
  pClient->m_pClientSock  = pSockClient;
  pClient->m_eClientState = BsProxyClientStateNominal;
  pClient->m_uRefCnt      = 0;
  pClient->m_bServerTrace = false;

  return pClient;
}
  
/*!
 * \brief Delete a client with the server.
 *
 * \param pClient   \h_botsense client to delete.
 */
void ClientDelete(BsProxyClientCtl_T *pClient)
{
  if( pClient != NULL )
  {
    SocketClose(pClient->m_pClientSock);
    SocketDelete(pClient->m_pClientSock);
    delete((char *)pClient->m_sClientName);
    delete(pClient);
  }
}


// ...........................................................................
// Client Attribute Sets and Gets
// ...........................................................................

/*!
 * \brief Set client's state.
 *
 * \param pClient   \h_botsense client.
 * \param eNewState New client state.
 */
void ClientSetState(BsProxyClientCtl_T *pClient, BsProxyClientState_T eNewState)
{
  static const char *sZombie = "(zombie)";

  const char *sState;
  char       *sName;
  size_t      n;

  if( pClient == NULL )
  {
    return;
  }

  else if( pClient->m_eClientState == eNewState )
  {
    return;
  }

  switch( eNewState )
  {
    case BsProxyClientStateZombie:    ///< disconnected but not deleted
      sState = "disconnected";
      n = strlen(pClient->m_sClientName) + strlen(sZombie) + 2;
      sName = NEWSTR(n);
      sprintf(sName, "%s %s", pClient->m_sClientName, sZombie);
      delete((char *)pClient->m_sClientName);
      pClient->m_sClientName = sName;
      break;
    case BsProxyClientStateNominal:   ///< normal operation
      sState = "nominal";
      break;
    case BsProxyClientStateReSync:    ///< resyncing server with client
      sState = "resyncing";
      break;
    default:
      return;
  }

  pClient->m_eClientState = eNewState;

  LOGDIAG2("%s: %s: %s.", ServerHasName(), ClientThisHasName(pClient), sState);
}


// ...........................................................................
// Client Receive Request Functions
// ...........................................................................

/*!
 * \brief Receive client request header.
 *
 * Sanity checks are performed on the header.
 *
 * \param pClient       \h_botsense client.
 * \param [out] pMsgHdr Received, unpacked message header.
 *
 * \return Returns \h_lt 0 on error or \h_ge 0 number of bytes for successful
 * received.
 */
static int ClientRecvHdr(BsProxyClientCtl_T *pClient, BsProxyMsgHdr_T *pMsgHdr)
{
  byte_t  bufHdr[BSPROXY_MSG_HDR_LEN];  // header buffer
  int     nBytes;                       // bytes read        
  size_t  uBodyLen;                     // length of message body

  //
  // Read the message header. The method depends on the client's state.
  //  Nominal   Normal operation
  //  Resync    Need to find the start of a header in stream, then read header
  //  Zombie    Client is disconnecting, simply return error.
  //
  switch( pClient->m_eClientState )
  {
    case BsProxyClientStateNominal: ///< normal operation
      nBytes = (int)ClientRead(pClient, bufHdr, BSPROXY_MSG_HDR_LEN);
      break;
    case BsProxyClientStateReSync:  ///< resyncing server with client
      nBytes = (int)ClientResync(pClient, bufHdr);
      break;
    case BsProxyClientStateZombie:  ///< disconnected/bad state, but not deleted
    default:
      return -BS_ECODE_SERVER_BAD_CLIENT;
  }

  // receive error
  if( nBytes < 0 )
  {
    return -BS_ECODE_BAD_RECV;
  }

  // nothing to receive
  else if( nBytes == 0 )
  {
    return 0;
  }

  // received header fragment, enter resync state
  else if( nBytes < BSPROXY_MSG_HDR_LEN )
  {
    BSPROXY_LOG_ERROR(CLIENT_HND(pClient), BS_ECODE_MSG_FRAG,
        "Received %d bytes, expected %d byte header.",
        nBytes, BSPROXY_MSG_HDR_LEN);
    ClientSetState(pClient, BsProxyClientStateReSync);
    return 0;
  }

  // unpack message header
  else if( bsUnpackMsgHdr(bufHdr, (size_t)nBytes, pMsgHdr) < 0 )
  {
    BSPROXY_LOG_ERROR(CLIENT_HND(pClient), BS_ECODE_MSG_BAD_HDR,
        "Received bad message header.");
    ClientSetState(pClient, BsProxyClientStateReSync);
    return 0;
  }

  // validate magic
  else if( pMsgHdr->m_hdrMagic != BSPROXY_MSG_MAGIC )
  {
    BSPROXY_LOG_ERROR(CLIENT_HND(pClient), BS_ECODE_MSG_BAD_HDR,
      "Received bad magic 0x%04x in message header.",
      pMsgHdr->m_hdrMagic);
    ClientSetState(pClient, BsProxyClientStateReSync);
    return 0;
  }

  //
  // The message body is too long.
  //
  else if( pMsgHdr->m_hdrBodyLen > BSPROXY_MSG_BODY_MAX )
  {
    uBodyLen = (size_t)pMsgHdr->m_hdrBodyLen;
    BSPROXY_LOG_ERROR(CLIENT_HND(pClient), BS_ECODE_MSG_TOO_BIG,
        "Received body length=%zu, flushing.", uBodyLen);
    ClientFlushInput(pClient, uBodyLen);
    return 0;
  }

  else
  {
    return nBytes;
  }
}

/*!
 * \brief Receive client request message body.
 *
 * Sanity checks are performed on the message.
 *
 * \param pClient       \h_botsense client.
 * \param [out] buf     Received, packed message body.
 * \param uBodyLen      Message body length.
 *
 * \return Returns \h_lt 0 on error or \h_ge 0 number of bytes for successful
 * received.
 */
static int ClientRecvBody(BsProxyClientCtl_T *pClient,
                          byte_t              buf[],
                          size_t              uBodyLen)
{
  int     nBytes;       // bytes read        

  // read failed
  if( (nBytes = (int)ClientRead(pClient, buf, uBodyLen)) < 0 )
  {
    BSPROXY_LOG_ERROR(CLIENT_HND(pClient), BS_ECODE_BAD_RECV,
        "Failed to receive client message body.");
    return -BS_ECODE_BAD_RECV;
  }

  // read a message fragment
  else if( nBytes < (ssize_t)uBodyLen )
  {
    BSPROXY_LOG_ERROR(CLIENT_HND(pClient), BS_ECODE_MSG_FRAG,
        "Received length=%zd, expected length=%zu.", nBytes, uBodyLen);
    ClientSetState(pClient, BsProxyClientStateReSync);
    nBytes = 0;
  }

  return nBytes;
}

/*!
 * \brief Receive a request message from client.
 *
 * \param hndClient     \h_botsense client handle.
 * \param [out] pMsgHdr Request message header.
 * \param [out] addrBuf Pointer to allocated input receive message body buffer.
 *
 * \return
 * Returns \h_lt 0 on client non-recoverable error,\n
 * 0 for discarded message,\n
 * \h_gt 0 number of bytes for successful read.
 */
int ClientRecvReq(BsProxyClientHnd_T  hndClient,
                  BsProxyMsgHdr_T    *pMsgHdr,
                  byte_t            **addrBuf)
{
  BsProxyClientCtl_T *pClient;      // BotSense client
  size_t              uBodyLen;     // expected message body length
  byte_t             *pBuf = NULL;  // allocated request packed message body
  int                 nBytesHdr;    // read message header bytes/return code
  int                 nBytesBody;   // read message body bytes/return code
  int                 nMsgLen;      // total message length/return code

  *addrBuf  = NULL;

  // lock client
  if( (pClient = ClientAcquire(hndClient)) == NULL )
  {
    return -BS_ECODE_SERVER_BAD_CLIENT;
  }

  // receive message header
  if( (nBytesHdr = ClientRecvHdr(pClient, pMsgHdr)) <= 0 )
  {
    nMsgLen = nBytesHdr;    // return code
  }

  // received a good header, now receive any message body 
  else
  {
    // (reenter) nominal state
    if( pClient->m_eClientState != BsProxyClientStateNominal )
    {
      ClientSetState(pClient, BsProxyClientStateNominal);
    }

    BSPROXY_LOG_REQ(hndClient, pMsgHdr);

    // specified message body length
    uBodyLen = (size_t)pMsgHdr->m_hdrBodyLen;

    //
    // Get the message body.
    //
    if( uBodyLen > 0 )
    {
      // allocate buffer
      pBuf = (byte_t *)new(uBodyLen);

      // receive 
      if( (nBytesBody = ClientRecvBody(pClient, pBuf, uBodyLen)) <= 0 )
      {
        delete(pBuf);
        pBuf = NULL;
        nMsgLen = nBytesBody;   // return code
      }

      // success
      else
      {
        nMsgLen = nBytesHdr + nBytesBody;
      }
    }

    //
    // No message body
    //
    else
    {
      pBuf = NULL;
      nMsgLen = nBytesHdr;
    }
  }

  // set allocated buffer
  *addrBuf = pBuf;

  if( nMsgLen > 0 )
  {
    LOGDIAG3("%s: %s: received message length=%d",
            ServerHasName(), ClientThisHasName(pClient), nMsgLen);
  }

  // unlock client
  ClientRelease(hndClient);

  return nMsgLen;
}


// ...........................................................................
// Client Send Reponse Functions
// ...........................................................................

/*!
 * \brief Send an ok response to the client.
 *
 * \param hndClient   \h_botsense client handle.
 * \param uTid        Request-response transaction id.
 *
 * \copydoc doc_return_std
 */
int ClientSendOkRsp(BsProxyClientHnd_T hndClient, BsTid_T uTid)
{
  byte_t bufRsp[BSPROXY_MSG_HDR_LEN];

  //
  // Send the ok message.
  // Note:  Even though this is a server defined message, there is no message
  //        body, so use the lower level send function directly. 
  //
  return ClientSendRsp(hndClient, BSPROXY_VCONN_SERVER, uTid,
                        BsProxyMsgIdRspOk, bufRsp, (size_t)0);
}

/*!
 * \brief Send an error response to the client.
 *
 * \param hndClient   \h_botsense client handle.
 * \param uTid        Request-response transaction id.
 * \param nECode      \copydoc doc_param_ecode
 * \param sErrFmt     Error format string.
 * \param ...         Format string variable arguments.
 *
 * \copydoc doc_return_std
 */
int ClientSendErrorRsp(BsProxyClientHnd_T hndClient,
                       BsTid_T            uTid,
                       int                nECode,
                       const char        *sErrFmt,
                       ...)
{
  va_list ap;
  int     rc;

  va_start(ap, sErrFmt);
  rc = ClientSendVErrorRsp(hndClient, uTid, nECode, sErrFmt,ap);
  va_end(ap);
  return rc;
}

/*!
 * \brief Send va_list error response to the client.
 *
 * \param hndClient   \h_botsense client handle.
 * \param uTid        Request-response transaction id.
 * \param nECode      \copydoc doc_param_ecode
 * \param sErrFmt     Error format string.
 * \param ap          Format string va_list.
 *
 * \copydoc doc_return_std
 */
int ClientSendVErrorRsp(BsProxyClientHnd_T  hndClient,
                        BsTid_T             uTid,
                        int                 nECode,
                        const char         *sErrFmt,
                        va_list             ap)
{
  BsProxyRspErr_T  msgRsp;

  // Fill in formatted error string as the response message body.
  vsnprintf(msgRsp.m_emsg, BSPROXY_RSPERR_EMSG_LEN+1, sErrFmt, ap);
  msgRsp.m_emsg[BSPROXY_RSPERR_EMSG_LEN] = 0;

  // set error code
  if( nECode < 0 )
  {
    nECode = -nECode;
  }
  msgRsp.m_ecode = (byte_t)nECode;

  // send error message
  ClientSendServerRsp(hndClient, uTid, BsProxyMsgIdRspErr, &msgRsp);

  return -nECode;
}

/*!
 * \brief Send a server-terminated response message to client.
 *
 * The response message header must contain the information except for the
 * body length which will be automatically calculated.
 *
 * \param hndClient     \h_botsense client handle.
 * \param uTid          Request-response transaction id.
 * \param uMsgId        BsProxyMsgIdRsp<em>x</em> response message id.
 * \param [in] pMsgRsp  Response message populated structure. NULL if no body.
 *
 * \return Returns number of bytes written on success.\n
 * \copydoc doc_return_ecode.
 */
int ClientSendServerRsp(BsProxyClientHnd_T  hndClient,
                        BsTid_T             uTid,
                        BsProxyMsgId_T      uMsgId,
                        void               *pMsgRsp)
{
  const NMMsgDef_T *pMsgDef;
  byte_t            bufRsp[BSPROXY_MSG_MAX_LEN];
  int               n;

  // find the bsProxy server message definition
  pMsgDef = BsProxyLookupMsgDef(uMsgId);

  if( pMsgDef == NULL )
  {
    BSPROXY_LOG_ERROR(hndClient, BS_ECODE_INTERNAL,
        "Cannot find message definition for MsgId=%u.", uMsgId);
    return -BS_ECODE_INTERNAL;
  }
  
  //
  // Pack response message body
  //
  if( pMsgRsp != NULL )
  {
    n = BsProxyPackMsg(uMsgId, pMsgRsp, BSPROXY_BUF_BODY(bufRsp),
                       ClientGetTraceState(hndClient));

    if( n < 0 )
    {
      BSPROXY_LOG_NMERROR(hndClient, n,
        "Failed to pack message body for MsgId=%u.", uMsgId);
      return -BS_ECODE_INTERNAL;
    }
  }

  //
  // No response message body
  //
  else
  {
    n = 0;
  }

  return ClientSendRsp(hndClient, BSPROXY_VCONN_SERVER, uTid, uMsgId,
                        bufRsp, (size_t)n);
}

/*!
 * \brief Send module-specific response to the client.
 *
 * \param hndClient   \h_botsense client handle.
 * \param hndVConn    Virtual connection handle.
 * \param uTid        Request-response transaction id.
 * \param uMsgId      Response message id.
 * \param bufRsp      Packed repsonse message body.
 * \param uRspSize    Size of response in buffer (number of bytes).
 *
 * \copydoc doc_return_std
 */
int ClientSendRsp(BsProxyClientHnd_T  hndClient,
                  BsVConnHnd_T        hndVConn,
                  BsTid_T             uTid,
                  BsMsgId_T           uMsgId,
                  byte_t              bufRsp[],
                  size_t              uRspSize)
{
  BsProxyClientCtl_T *pClient;
  BsProxyMsgHdr_T     msgHdr;
  size_t              nBytes;
  int                 n;

  // message header
  msgHdr.m_hdrMagic   = (ushort_t)BSPROXY_MSG_MAGIC;
  msgHdr.m_hdrTid     = (byte_t)uTid;
  msgHdr.m_hdrVConn   = (byte_t)hndVConn;
  msgHdr.m_hdrMsgId   = (ushort_t)uMsgId;
  msgHdr.m_hdrBodyLen = (ushort_t)uRspSize;

  n = bsPackMsgHdr(&msgHdr, bufRsp, BSPROXY_MSG_HDR_LEN);

  if( n < 0 )
  {
    BSPROXY_LOG_ERROR(hndClient, n,
        "Failed to pack message header for MsgId=%u.", uMsgId);
    return -BS_ECODE_INTERNAL;
  }

  nBytes = (size_t)n + uRspSize;

  if( (pClient = ClientAcquire(hndClient)) == NULL )
  {
    BSPROXY_LOG_ERROR(hndClient, BS_ECODE_SERVER_BAD_CLIENT,
        "Failed to acquire Client=%d.", hndClient);
    return -BS_ECODE_SERVER_BAD_CLIENT;
  }

  // send message to client
  n = (int)ClientWrite(pClient, bufRsp, nBytes);

  ClientRelease(hndClient);

  if( n < 0 )
  {
    BSPROXY_LOG_SYSERROR(hndClient,
        "Failed to send message, MsgId=%u", uMsgId);
    return -BS_ECODE_BAD_SEND;
  }

  // success
  else
  {
    BSPROXY_LOG_RSP(hndClient, &msgHdr);
    return n;
  }
}
