////////////////////////////////////////////////////////////////////////////////
//
// Package:   BotSense
//
// Program:   bsProxy
//
// File:      bsProxy.c
//
/*! \file
 *
 * $LastChangedDate: 2010-09-13 10:25:05 -0600 (Mon, 13 Sep 2010) $
 * $Rev: 581 $
 *
 * \brief \h_botsense proxy IP server.
 *
 * \sa \ref man_bsproxy
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 * (C) 2007-2010.  RoadNarrows LLC.
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

#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <string.h>
#include <limits.h>
#include <errno.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/opts.h"
#include "rnr/dlistvoid.h"
#include "rnr/dliststr.h"
#include "rnr/new.h"
#include "rnr/sock.h"
#include "rnr/sockset.h"

#include "botsense/bsProxyMsgs.h"

#include "version.h"

#include "bsProxy.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

const char    *Argv0;         ///< command base name

//
// The bsProxy Options.
//
static char        *BsOptListenIFDft    = NULL;   ///< default listen interface
static DListVoid_T *BsOptDListListenIF  = NULL;   ///< list of listen i/f's
static DListStr_T  *BsOptDListLibPath   = NULL;   ///< list of lib search paths

//
// Forward declarations.
//
static int OptsCvtArgInterface(const char *argv0, const char *sOptName,
                                char *optarg, void *pOptVal);
static int OptsCvtArgLibPath(const char *argv0, const char *sOptName,
                             char *optarg, void *pOptVal);

/*!
 * Program Information
 */
static OptsPgmInfo_T BsProxyPgmInfo =
{
  .synopsis   = "BotSense IP Proxy Server.",
  .long_desc  =
    "The BotSense Proxy Server (%P) provides an IP interace to any proxied "
    "(pseudo) device. %P is multi-threaded by device to optimize client "
    "responsiveness. For each (pseudo) device, there must exist a dynamically "
    "linked library that %P will load as needed, plus a client side library. "
    "The two libraries typically communicate through a defined set of NetMsgs "
    "network messages. The Message Exchange Patterns (MEP) support by %P "
    "are client request - server response and server output streaming "
    "(e.g. video).\n"
    "\n"
    "A wide range of RoadNarrows support robots use %P."
};

/*!
 * Command Line Options Information.
 */
static OptsInfo_T BsProxyOptsInfo[] =
{
  // -i, --interface  Proxy IP Port Number
  {
    .long_opt   = "interface", 
    .short_opt  = 'i',
    .has_arg    = required_argument,
    .opt_addr   = &BsOptListenIFDft,
    .has_default= true,
    .fn_cvt     = OptsCvtArgInterface,
    .fn_fmt     = OptsFmtStr,
    .arg_name   = "[<addr>][:<port>]",
    .opt_desc   = "Server passive socket interface. The server will listen "
                  "on this interface to accept new incoming connection "
                  "requests."
  },

  // -L, --libpath  Library serach path
  {
    .long_opt   = "libpath", 
    .short_opt  = 'L',
    .has_arg    = required_argument,
    .opt_addr   = NULL,
    .has_default= false,
    .fn_cvt     = OptsCvtArgLibPath,
    .fn_fmt     = OptsFmtStr,
    .arg_name   = "<dir>",
    .opt_desc   = "Add library directory path to search for dynamic linked "
                  "library plugins."
  },

  {NULL, }
};

/*!
 * \brief Log Proxy Server Specific Error.
 *
 * \param ecode     \h_botsense error code.
 * \param efmt      Error output format string literal.
 * \param ...       Error variable arguments.    
 */
#define SERVER_LOG_ERROR(ecode, efmt, ...) \
  LOGERROR("Server %s:%s(ecode=%d): " efmt, \
      ServerHasName(), \
      bsStrError(ecode), (ecode>=0? ecode: -ecode), \
      ##__VA_ARGS__)

/*!
 * \brief Log Proxy Server Specific System Error.
 *
 * \param efmt      Error output format string litteral.
 * \param ...       Error variable arguments.    
 */
#define SERVER_LOG_SYSERROR(efmt, ...) \
  LOGERROR("Server %s: %s(ecode=%d): %s(errno=%d)" efmt, \
          ServerHasName(), \
          bsStrError(BS_ECODE_SYS), BS_ECODE_SYS, \
          strerror(errno), errno, ##__VA_ARGS__)

/*!
 * \brief Lock server's mutual exclusion.
 */
static inline void ServerLock()
{
  int rc;

  if( (rc = pthread_mutex_lock(&BsServerCtl->m_mutex)) != 0 )
  { 
    errno = rc;
    SERVER_LOG_SYSERROR("pthread_mutex_lock()");
  }
}

/*!
 * \brief Unlock server's  mutual exclusion.
 */
static inline void ServerUnlock()
{
  int rc;

  if( (rc = pthread_mutex_unlock(&BsServerCtl->m_mutex)) != 0 )
  { 
    errno = rc;
    SERVER_LOG_SYSERROR("pthread_mutex_unlock()");
  }
}

/*!
 * \brief Try to lock global mutual exclusion.
 *
 * \return
 * Returns true if lock is acquired. Otherwise returns false if mutex already
 * locked.
 */
static inline bool_t ServerTryLock()
{
  return pthread_mutex_trylock(&BsServerCtl->m_mutex) == 0? true: false;
}

/*!
 * \brief Register a new client with the server.
 *
 * \param pSockClient   Accecpted client socket.
 *
 * \return Returns client handle.
 */
static BsProxyClientHnd_T BsProxyClientRegister(Socket_T *pSockClient)
{
  BsProxyServerCtl_T *pServer;
  BsProxyClientCtl_T *pClient;
  int                 sd;

  pClient = ClientNew(pSockClient);

  // get the server instance
  pServer = ServerGet();

  //
  // The client handle is also the opened socket descriptor.
  //
  sd = SocketAttrGetSd(pSockClient);

  //
  // Add client socket to server's socket set. The client socket is added
  // only to the active read set, not the write set.
  //
  SockSetAdd(pServer->m_pServerSockSet, pClient->m_pClientSock, true, false);

  pServer->m_pServerRegClient[sd] = pClient;
  pServer->m_uServerRegClientCount++;

  return ServerClientSd2Hnd(sd);
}

/*!
 * \brief Unregister a client with the server.
 *
 * Okay, this is messy. A client may disconnect from the server at any time,
 * leaving many virtual connections open. The server thread could be processing
 * open and close requests on this zombie client while device threads could be
 * sending responses on the same client. Hopefully, all will work out with only
 * a few diagnotics messages printed.
 *
 * \par Execution Context:
 * Server thread (main) 

 * \param hndClient     \h_botsense client handle.
 */
static void BsProxyClientUnregister(BsProxyClientHnd_T hndClient)
{
  BsProxyServerCtl_T *pServer;
  BsProxyClientCtl_T *pClient;
  BsVConnHnd_T        hndVConn;
  BsProxyVConn_T     *pVConn;

  //
  // Lock the client.
  //
  if( (pClient = ClientAcquire(hndClient)) == NULL )
  {
    return;
  }

  // mark client zombie state (stop thread actions on this client)
  ClientSetState(pClient, BsProxyClientStateZombie);

  // get the server instance
  pServer = ServerGet();

  //
  // Loop through the device virtual connections, closing all opened connections
  // associated with this client.
  //
  for(hndVConn=BSPROXY_VCONN_MOD_MIN;
      hndVConn<=BSPROXY_VCONN_MOD_MAX;
      ++hndVConn)
  {
    if( (pVConn = VConnGet(hndVConn)) != NULL )
    {
      // pointers point to the same client allocated object
      if( pVConn->m_hndClient == hndClient )
      {
        VConnClose(hndClient, hndVConn);
        pClient->m_uRefCnt--;
      }
    }
  }

  //
  // Remove the client from server reqistered set.
  //
  SockSetRemove(pServer->m_pServerSockSet, pClient->m_pClientSock);
  pServer->m_pServerRegClient[ServerClientHnd2Sd(hndClient)] = NULL;
  pServer->m_uServerRegClientCount--;

  //
  // Now delete the client (don't need to release since the client no longer
  // exists).
  //
  ClientDelete(pClient);
}

/*!
 * \brief Delete a server control block.
 *
 * \param pServer   \h_botsense server.
 */
static void BsProxyServerDelete(BsProxyServerCtl_T *pServer)
{
  BsProxyClientHnd_T  hndClient;

  if( pServer != NULL )
  {
    for(hndClient=0; hndClient<BSPROXY_FD_SETSIZE; ++hndClient)
    {
      if( pServer->m_pServerRegClient[ServerClientHnd2Sd(hndClient)] != NULL )
      {
        BsProxyClientUnregister(hndClient);
      }
    }
    SocketDelete(pServer->m_pServerSockListener);
    SockSetDelete(pServer->m_pServerSockSet);
    delete((char *)pServer->m_sServerName);
    pthread_mutex_destroy(&pServer->m_mutex);
    delete(pServer);
  }
}

/*!
 * \brief Create new server control block.
 *
 * \param sPrefix         Server name prefix.
 * \param sLocalHostName  Local host interface to bind to.
 * \param nPort           Server listener TCP port number.
 *
 * \return Returns a newly allocated and intialized server control block.
 */
static BsProxyServerCtl_T *BsProxyServerNew(const char *sPrefix,
                                            const char *sLocalHostName,
                                            int         nPort)
{
  BsProxyServerCtl_T *pServer;
  char               *bufName;
  int                 rc = BS_OK;

  bufName = NEWSTR(strlen(sPrefix)+16);
  sprintf(bufName, "%s_%d", sPrefix, nPort);

  pServer                           = NEW(BsProxyServerCtl_T);
  pServer->m_sServerName            = bufName;
  pServer->m_nServerPort            = nPort;
  pServer->m_pServerSockSet         = SockSetNew();
  pServer->m_pServerSockListener    = SocketOpenTcpListener(nPort,
                                                            sLocalHostName);
  pServer->m_uServerRegClientCount  = 0;

  // create server mutex
  pthread_mutex_init(&pServer->m_mutex, NULL);

  if( pServer->m_pServerSockListener == NULL )
  {
    LOGERROR("%s: Failed to open server listener socket.",
                pServer->m_sServerName);
    rc = -BS_ECODE_SYS;
  }

  else
  {
    rc = SockSetAdd(pServer->m_pServerSockSet,  // server client socket set
                    pServer->m_pServerSockListener, // server listener socket
                    true,                       // add listener to active read
                    true);                      // add listener to active write

    if( rc < 0 )
    {
      LOGERROR("%s: Failed to add server listener socket to set.",
                pServer->m_sServerName);
      rc = -BS_ECODE_INTERNAL;
    }
  }

  //
  // Error clean up.
  //
  if( rc < 0 )
  {
    BsProxyServerDelete(pServer);
    pServer = NULL;
  }

  return pServer;
}

/*!
 * \brief Dispatch client request.
 *
 * The client request is queued in the appropriate service thread.
 *
 * \note This dispatcher owns the allocated message buffer until it is 
 * successfully queue.
 *
 * \param hndClient   \h_botsense client handle.
 * \param pMsgHdr     Received, unpacked request message header.
 * \param bufReq      Received, packed allocated message body.
 */
static void BsProxyDispatch(BsProxyClientHnd_T  hndClient,
                            BsProxyMsgHdr_T    *pMsgHdr,
                            byte_t              bufReq[])
{
  BsProxyVConn_T *pVConn;
  BsVConnHnd_T    hndVConn;
  BsTid_T         uTid;
  BsMsgId_T       uMsgId;
  int             rc;

  hndVConn= (BsVConnHnd_T)pMsgHdr->m_hdrVConn;
  uTid    = (BsTid_T)pMsgHdr->m_hdrTid;
  uMsgId  = (BsMsgId_T)pMsgHdr->m_hdrMsgId;

  //
  // Check if handle is valid.
  //
  if( !BSPROXY_CHK_VCONN_HND(hndVConn) )
  {
    delete(bufReq);
    BSPROXY_SEND_ERROR_RSP(hndClient, hndVConn, uTid,
          BS_ECODE_BAD_VCONN_HND, "VConn=%d", hndVConn);
    return;
  }

  //
  // Lock the virtual connection.
  //
  else if( (pVConn = VConnAcquire(hndVConn)) == NULL )
  {
    delete(bufReq);
    BSPROXY_SEND_ERROR_RSP(hndClient, hndVConn, uTid,
          BS_ECODE_NO_VCONN, "VConn=%d", hndVConn);
    return;
  }

  //
  // Queue the request
  //
  else
  {
    rc = ThQueue(pVConn->m_pThCtl,
                hndClient, hndVConn, uTid, uMsgId,
                bufReq, (size_t)pMsgHdr->m_hdrBodyLen);

    // release the virtual connection
    VConnRelease(hndVConn);

    if( rc < 0 )
    {
      delete(bufReq);
      BSPROXY_SEND_ERROR_RSP(hndClient, hndVConn, uTid,
          BS_ECODE_NO_RSRC, "MsgId=%u", uMsgId);
    }
  }
}

/*!
 * \brief Server mainloop.
 *
 * Processes all client registrations, requests, and unregistrations.
 *
 * \param pServer   \h_botsense server.
 *
 * \copydoc doc_return_std
 */
static int BsProxyServer(BsProxyServerCtl_T *pServer)
{
  struct timeval      tvSelect;       // select() timeout
  Socket_T           *pSockClient;    // new client socket
  BsProxyClientCtl_T *pClient;        // client
  Socket_T           *pSock;          // working socket 
  SockSetIter_T       iterSockSet;    // socket set iterator
  int                 nCntSd;         // number of active sockets
  int                 sdListener;     // socket descriptor
  int                 sd;             // socket descriptor
  BsProxyClientHnd_T  hndClient;      // client handle
  int                 nMsgLen;        // message length/return code
  BsProxyMsgHdr_T     msgHdr;         // request message header
  byte_t             *pBuf;           // allocated message body

  LOGDIAG1("%s: Listening on interface %d", ServerHasName(), ServerHasPort());

  sdListener = SocketAttrGetSd(pServer->m_pServerSockListener);

  tvSelect.tv_sec   = BSPROXY_TUNE_T_SELECT / 1000000;
  tvSelect.tv_usec  = BSPROXY_TUNE_T_SELECT % 1000000;

  //
  // Loop forever for client events.
  //
  while( true )
  {
    // block w/o timout on select() with active sockets
    nCntSd = SockSetSelect(pServer->m_pServerSockSet, &tvSelect);

    //
    // One or more clients I/O operations are available.
    //
    if( nCntSd > 0 )
    {
      //
      // Iterate through read events.
      // 
      for(pSock=SockSetIterFirst(pServer->m_pServerSockSet,
                                 SOCK_SET_SELECTED, SOCK_IO_READ,
                                 &iterSockSet);
          pSock!=NULL;
          pSock=SockSetIterNext(&iterSockSet))
      {
        // socket descriptor
        sd = SocketAttrGetSd(pSock);

        //
        // New bsProxy client.
        //
        if( sd == sdListener )
        {
          // accept new, non-blocking connection
          pSockClient = SocketAccept(pServer->m_pServerSockListener, true);

          if( pSockClient == NULL )
          {
            SERVER_LOG_ERROR(BS_ECODE_GEN, "accept() failed.");
            return -BS_ECODE_GEN;
          }

          // maximum number of clients exceeded
          if( (pServer->m_uServerRegClientCount >= BSPROXY_REG_CLIENT_MAX) ||
              (sd >= BSPROXY_FD_SETSIZE) )
          {
            SERVER_LOG_ERROR(BS_ECODE_SERVER_CONN_DENY,
              "%s: Connection denied, maximum %d clients are "
              "already registered.",
              SocketAttrGetRemoteName(pSockClient), BSPROXY_REG_CLIENT_MAX);
            SocketClose(pSockClient);
            SocketDelete(pSockClient);
          }

          // register accepted client with this server
          else
          {
            hndClient = BsProxyClientRegister(pSockClient);
            LOGDIAG1("%s: %s: Connection accepted, client registered.",
                  ServerHasName(), ClientHasName(hndClient));
            LOGDIAG2("%s: %u registered clients.", 
                  ServerHasName(), pServer->m_uServerRegClientCount);
          }
        }

        //
        // A registered bsProxy client sent a request.
        //
        else
        {
          // map socket descriptor to handle
          hndClient = ServerClientSd2Hnd(sd);

          // get the (unlocked) client
          pClient = ServerGetClient(hndClient);

          if( pClient == NULL )
          {
            SERVER_LOG_ERROR(BS_ECODE_SERVER_BAD_CLIENT,
              "%d: No registered client for select().", sd);
            close(sd);
            continue;
          }

          // receive client request
          nMsgLen = ClientRecvReq(sd, &msgHdr, &pBuf);

          // unrecoverable connection error or client has disconnected
          if( nMsgLen <= 0 )
          {
            LOGDIAG1("%s: %s: Connnection closed, client unregistered.",
              ServerHasName(), ClientHasName(hndClient));
            BsProxyClientUnregister(hndClient);
            delete(pBuf);
            LOGDIAG2("%s: %u registered clients.", 
                  ServerHasName(), pServer->m_uServerRegClientCount);
          }

          // dispatch the client request
          else if( nMsgLen > 0 )
          {
            BsProxyDispatch(hndClient, &msgHdr, pBuf);
          }
        }
      }

      //
      // Clear any write events. For now, nothing to do.
      //
      for(pSock=SockSetIterFirst(pServer->m_pServerSockSet,
                                 SOCK_SET_SELECTED, SOCK_IO_WRITE,
                                 &iterSockSet);
          pSock!=NULL;
          pSock=SockSetIterNext(&iterSockSet))
      {
        sd = SocketAttrGetSd(pSock);
      }
    }

    //
    // Timed out. Check client connections.
    // 
    // Note:  This is a sanity check. When a client disconnects, typically a
    //        read event with zero bytes will occur. The select() will detect
    //        this event and will be processed in the above code.
    //
    else if( nCntSd == 0 )
    {
      for(pSock=SockSetIterFirst(pServer->m_pServerSockSet,
                                 SOCK_SET_ACTIVE, SOCK_IO_READ,
                                 &iterSockSet);
          pSock!=NULL;
          pSock=SockSetIterNext(&iterSockSet))
      {
        // socket descriptor
        sd = SocketAttrGetSd(pSock);

        if( (sd != sdListener) && SocketStateIsErrored(pSock) )
        {
          hndClient = ServerClientSd2Hnd(sd);
          LOGDIAG1("%s: %s: Connnection closed, client unregistered.",
              ServerHasName(), ClientHasName(hndClient));
          BsProxyClientUnregister(hndClient);
          LOGDIAG2("%s: %u registered clients.", 
                  ServerHasName(), pServer->m_uServerRegClientCount);
        }
      }
    }

    //
    // Internal error.
    //
    else
    {
      SERVER_LOG_ERROR(BS_ECODE_INTERNAL, "select() failed.");
      return -BS_ECODE_INTERNAL;
    }
  }

  return BS_OK;
}

/*!
 * \brief Get and set environment.
 */
static void MainEnv()
{
}

/*!
 * \brief Get configuration.
 */
static void MainCfg()
{
}

/*!
 * \brief Convert command-line listen interface option string to 
 * network name/number and port number.
 *
 * \par Option Argument Syntax:
 *    [addr][:port]
 *
 * \param argv0         Command name.
 * \param sOptName      Option name.
 * \param optarg        Parsed option argument to convert (optional).
 * \param[out] pOptVal  Pointer to converted option value (not used).
 *
 * \exception OptsInvalid()
 *
 * \return If returns, then returns BS_OK.
 */
static int OptsCvtArgInterface(const char *argv0,
                               const char *sOptName,
                               char       *optarg,
                               void       *pOptVal)
{
  char               *sInet;
  char               *sSepField;
  char               *sPort;
  int                 nPort;
  BsProxyListenIF_T  *pIF;

  sInet = new_strdup(optarg);

  sSepField = strchr(sInet, ':');

  if( sSepField )
  {
    *sSepField  = 0;
    sPort       = sSepField+1;
    nPort       = (int)atol(sPort);
    if( (nPort <= 0) || (nPort > 0xffff) )
    {
      OptsInvalid(Argv0, "'%s': Invalid '%s' argument port value.",
          optarg, sOptName);
    }
  }
  else
  {
    nPort = BSPROXY_LISTEN_PORT_DFT;
  }

  if( *sInet == 0 )
  {
    delete(sInet);
    sInet = NULL;
  }

  pIF = NEW(BsProxyListenIF_T);
  pIF->m_sAddr = sInet;
  pIF->m_nPort = nPort;
  DListVoidAppend(BsOptDListListenIF, pIF);

  return BS_OK;
}

/*!
 * \brief Append command-line library search path option to list of search
 * paths.
 *
 * \par Option Argument Syntax:
 *    dir
 *
 * \param argv0         Command name.
 * \param sOptName      Option name.
 * \param optarg        Parsed option argument to convert (optional).
 * \param[out] pOptVal  Pointer to converted option value (not used).
 *
 * \exception OptsInvalid()
 *
 * \return If returns, then returns BS_OK.
 */
static int OptsCvtArgLibPath(const char *argv0,
                             const char *sOptName,
                             char       *optarg,
                             void       *pOptVal)
{
  DListVoidAppend(BsOptDListLibPath, optarg);
  return BS_OK;
}

/*!
 * \brief Main initialization.
 *
 * Parses command-line options and arguments and environment variables to
 * configure the BotsSense Proxy server.
 *
 * \param argc  Count of command-line options and arguments.
 * \param argv  Array of command-line options and arguments.
 */
static void MainInit(int argc, char *argv[])
{
  BsProxyListenIF_T  *pIF;
  char                buf[32];

  Argv0 = basename(argv[0]);

  sprintf_s(buf, sizeof(buf), ":%d", BSPROXY_LISTEN_PORT_DFT);
  BsOptListenIFDft    = new_strdup(buf);
  BsOptDListListenIF  = DListVoidNew(NULL, NULL);
  BsOptDListLibPath   = DListStrNew(NULL, NULL);

  // Parse input options
  argv = OptsGet(Argv0, &PkgInfo, &BsProxyPgmInfo,
                    BsProxyOptsInfo, true, &argc, argv);

  // The environment
  MainEnv();

  // The configuration file(s)
  MainCfg();

  //
  // Final checks and tweaks.
  //
  if( DListVoidCount(BsOptDListListenIF) == 0 )
  {
    pIF = NEW(BsProxyListenIF_T);
    pIF->m_sAddr = NULL;
    pIF->m_nPort = BSPROXY_LISTEN_PORT_DFT;
    DListVoidAppend(BsOptDListListenIF, pIF);
  }

  if( DListVoidCount(BsOptDListListenIF) > 1 )
  {
    LOGDIAG2("Warning: Currently only one listen interface is supported. "
             "First I/F will be used.");
  }
}


// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief The server control block. 
 *
 * \note In a future bsProxy implementation, multiple server instance threads
 * could be running in the same application. But currently there is only one
 * server per application instance.
 */
BsProxyServerCtl_T *BsServerCtl;

/*!
 * \brief bsProxy main()
 *
 * \param argc  Count of command-line options and arguments.
 * \param argv  Array of command-line options and arguments.
 *
 * \return Exit value.
 */
int main(int argc, char *argv[])
{
  BsProxyListenIF_T  *pIF;
  int                 rc;

  // Process command-line, environment, and configuration file data.
  MainInit(argc, argv);

  LOGDIAG1("%s v%s %s", Argv0, PkgInfo.m_sPkgVersion, PkgInfo.m_sPkgTimeStamp);

  // Interface module control one-time initialization.
  ModOneTimeInit(BsOptDListLibPath);

  // Service thread control one-time initialization.
  ThOneTimeInit();

  // Client control one-time initialization.
  ClientOneTimeInit();

  // Virtual connection control one-time initialization.
  VConnOneTimeInit();

  //
  // FUTURE:  If multiple listen interfaces are specified, then fork extra
  //          copies of bsProxy here.
  //
 
  //
  // Create a server control block to listen on the first interface.
  //
  pIF = (BsProxyListenIF_T *)DListStrGetDataAt(BsOptDListListenIF, 0);
  BsServerCtl = BsProxyServerNew(Argv0, pIF->m_sAddr, pIF->m_nPort);

  if( BsServerCtl == NULL )
  {
    return EC_ERROR;
  }

  //
  // Open the server-terminated virtual connection. The server service thread
  // is started.
  //
  if( (rc = VConnOpenServer()) < 0 )
  {
    return EC_ERROR;
  }

  LOGDIAG1("BotSense IP Proxy Server %s started", ServerHasName());

  // Run.
  rc = BsProxyServer(BsServerCtl);

  return rc == BS_OK? 0: EC_ERROR; 
}
