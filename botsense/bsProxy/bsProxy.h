////////////////////////////////////////////////////////////////////////////////
//
// Package:   BotSense
//
// Program:   bsProxy
//
// File:      bsProxy.h
//
/*! \file
 *
 * $LastChangedDate: 2010-08-23 09:53:11 -0600 (Mon, 23 Aug 2010) $
 * $Rev: 570 $
 *
 * \brief \h_botsense bsProxy IP server declarations.
 *
 * \todo Add support for botsense configuration XML files.
 * \todo Add fork-exec'ing to support multiple interfaces.
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

#ifndef _BSPROXY_H
#define _BSPROXY_H

#include <sys/time.h>
#include <sys/select.h>
#include <stdarg.h>
#include <pthread.h>
#include <errno.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/dlistvoid.h"
#include "rnr/dliststr.h"
#include "rnr/sock.h"
#include "rnr/sockset.h"
#include "rnr/netmsgs.h"

#include "botsense/BotSense.h"
#include "botsense/libBotSense.h"
#include "botsense/bsProxyModIF.h"
#include "botsense/bsProxyMsgs.h"


//-----------------------------------------------------------------------------
// Data Types and Defines
//-----------------------------------------------------------------------------

/*!
 * Compiled Default Tunables.
 *
 * \note These can be overriden in the configuration XML file.
 */
/*!
 * \ingroup bsserver
 * \defgroup bssrv_tunes Default Tunables
 *
 * \h_botsense bsProxy Tunables
 *
 * \note These can be overriden in the configuration XML file.
 *
 * \{
 */
#define BSPROXY_TUNE_T_RECV   1000000   ///< 1.0s receive time out
#define BSPROXY_TUNE_T_SEND    500000   ///< 0.5s send time out
#define BSPROXY_TUNE_T_SELECT 5000000   ///< 5.0s send select time out
/*! \} */

//
// Server socket set configuration.
//
#ifndef FD_SETSIZE
#define BSPROXY_FD_SETSIZE  256           ///< server socket set size
#else
#define BSPROXY_FD_SETSIZE  FD_SETSIZE    ///< server socket set size
#endif

//
// Forward declarations
//
typedef struct _bsproxy_th_ctl  *BsProxyThCtl_P;  ///< service thread control
typedef struct _bsproxy_th_req  *BsProxyThReq_P;  ///< service thread request

typedef int BsProxyClientHnd_T;    ///< bsProxy server client handle


/*!
 * Interface Module Exported Interface Control Block Structure Type
 */
typedef struct
{
  const char         *m_sModUri;      ///< module Uniform Resource Id
  void               *m_dllHandle;    ///< dynamic library loader handle
  uint_t              m_uRefCnt;      ///< vconn reference count for this module
  BsModInitFunc_P     m_fnModInit;    ///< module init
  BsModExitFunc_P     m_fnModExit;    ///< module exit
  BsModOpenFunc_P     m_fnModOpen;    ///< device open
  BsModCloseFunc_P    m_fnModClose;   ///< device close
  BsModRequestFunc_P  m_fnModRequest; ///< module request
  BsModTraceFunc_P    m_fnModTrace;   ///< tracing
  BsModInfoFunc_P     m_fnModInfo;    ///< static info
} BsProxyModIF_T;

/*!
 * \brief Service thread states.
 */
typedef enum
{
  BsProxyThStateUninit,      ///< thread is not fully initialized
  BsProxyThStateInit,        ///< thread is initialized
  BsProxyThStateRunning,     ///< thread is running (nominal state)
  BsProxyThStateExit         ///< thread is exiting
} BsProxyThState_T;

/*!
 * \h_botsense Service Thread Control Block Structure Type
 */
typedef struct _bsproxy_th_ctl
{
  const char       *m_sDevUri;      ///< proxied device URI or server
  uint_t            m_uRefCnt;      ///< vconn reference count for this thread
  BsProxyThState_T  m_eState;       ///< thread state
  pthread_mutex_t   m_mutexSync;    ///< synchronization mutex used by condition
  pthread_cond_t    m_condSync;     ///< synchronization condition
  DListVoid_T      *m_queue;        ///< thread request queue
  pthread_t         m_thread;       ///< the service thread

  /*!
   * \brief Service thread request handler.
   *
   * \param hndClient   \h_botsense client handle.
   * \param hndVConn    Virtual connection handle.
   * \param uTid        Request-response transaction id.
   * \param uMsgId      Request message id.
   * \param [in] bufReq Packed request message body buffer.   
   * \param uReqLen     Length of packed request (number of bytes).
   *
   * \copydoc doc_return_std
   */
  int (*m_fnRequest)(BsProxyClientHnd_T hndClient,
                     BsVConnHnd_T       hndVConn,
                     BsTid_T            uTid,
                     BsMsgId_T          uMsgId,
                     byte_t             bufReq[],
                     size_t             uReqLen);
} BsProxyThCtl_T;

/*!
 * \h_botsense Proxy Client States
 */
typedef enum
{
  BsProxyClientStateInit,     ///< initialization state
  BsProxyClientStateNominal,  ///< normal operation
  BsProxyClientStateReSync,   ///< resyncing server with client
  BsProxyClientStateZombie,   ///< disconnected or fatal, not deleted
} BsProxyClientState_T;

/*!
 * \h_botsense Proxy Client Control Block Structure
 */
typedef struct
{
  bool_t                m_bBusy;        ///< client is [not] busy
  const char           *m_sClientName;  ///< client's (remote) name
  Socket_T             *m_pClientSock;  ///< client opened TCP socket
  BsProxyClientState_T  m_eClientState; ///< client state
  uint_t                m_uRefCnt;      ///< vconn ref count for this client
  bool_t                m_bServerTrace; ///< do [not] trace server messages
} BsProxyClientCtl_T;

/*!
 * \h_botsense Proxy Server Control Block Structure
 */
typedef struct
{
  const char         *m_sServerName;            ///< server's name
  int                 m_nServerPort;            ///< server's listener port
  SockSet_T          *m_pServerSockSet;         ///< server socket set
  Socket_T           *m_pServerSockListener;    ///< server listener socket
  uint_t              m_uServerRegClientCount;  ///< num of reg. clients
  BsProxyClientCtl_T *m_pServerRegClient[BSPROXY_FD_SETSIZE];
                                                ///< registered clients
  pthread_mutex_t     m_mutex;                  ///< server mutex lock
} BsProxyServerCtl_T;

/*!
 * \h_botsense Virtual Connection Structure
 */
typedef struct
{
  bool_t              m_bBusy;        ///< virtual connection is [not] busy
  BsVConnHnd_T        m_hndVConn;     ///< self reference
  BsProxyClientHnd_T  m_hndClient;    ///< proxied client handle
  BsProxyThCtl_T     *m_pThCtl;       ///< service thread
  BsProxyModIF_T     *m_pModIF;       ///< interface module I/F
  int                 m_rd;           ///< module resource descriptor
} BsProxyVConn_T;

/*!
 * \h_botsense Service Thread Request Structure Type
 */
typedef struct _bsproxy_th_req
{
  BsProxyClientHnd_T  m_hndClient;    ///< proxied client handle
  BsVConnHnd_T        m_hndVConn;     ///< virtual connection handle
  BsTid_T             m_uTid;         ///< request-response transaction id
  BsMsgId_T           m_uMsgId;       ///< message id
  byte_t             *m_bufReq;       ///< packed request message buffer
  size_t              m_uReqLen;      ///< request buffer length
} BsProxyThReq_T;

/*!
 * \h_botsense Server Listen Address, Port Pair Type
 */
typedef struct
{
  char *m_sAddr;          ///< listen local address or hostname
  int   m_nPort;          ///< listen port number
} BsProxyListenIF_T;



//-----------------------------------------------------------------------------
// Data
//-----------------------------------------------------------------------------

// The server control block.
extern BsProxyServerCtl_T *BsServerCtl;


//-----------------------------------------------------------------------------
// Logging and Support Macros
//-----------------------------------------------------------------------------

/*!
 * \brief Log Proxy Server Warning.
 *
 * \param hndClient Client handle.
 * \param ecode     \h_botsense error code.
 * \param wfmt      Warning output format string literal.
 * \param ...       Warning variable arguments.    
 */
#define BSPROXY_LOG_WARN(hndClient, ecode, wfmt, ...) \
  LOGDIAG2("Warning: %s: %s: %s(ecode=%d): " wfmt, \
      ServerHasName(), ClientHasName(hndClient), \
      bsStrError(ecode), ((ecode)>=0? (ecode): -(ecode)), \
      ##__VA_ARGS__)

/*!
 * \brief Log Proxy Server Error.
 *
 * \param hndClient Client handle.
 * \param ecode     \h_botsense error code.
 * \param efmt      Error output format string literal.
 * \param ...       Error variable arguments.    
 */
#define BSPROXY_LOG_ERROR(hndClient, ecode, efmt, ...) \
  LOGERROR("%s: %s: %s(ecode=%d): " efmt, \
      ServerHasName(), ClientHasName(hndClient), \
      bsStrError(ecode), (ecode>=0? ecode: -ecode), \
      ##__VA_ARGS__)

/*!
 * \brief Log Proxy Server NetMsgs (Un)Packing Error.
 *
 * \param hndClient Client handle.
 * \param nmecode   NetMsgs error code.
 * \param efmt      Error output format string literal.
 * \param ...       Error variable arguments.    
 */
#define BSPROXY_LOG_NMERROR(hndClient, nmecode, efmt, ...) \
  BSPROXY_LOG_ERROR(hndClient, BS_ECODE_BAD_MSG, "%s(nmecode=%d): " efmt, \
      nmStrError(nmecode), (nmecode>=0? nmecode: -nmecode), \
      ##__VA_ARGS__)

/*!
 * \brief Log Proxy Server System Error.
 *
 * \param hndClient Client handle.
 * \param efmt      Error output format string litteral.
 * \param ...       Error variable arguments.    
 */
#define BSPROXY_LOG_SYSERROR(hndClient, efmt, ...) \
  LOGERROR("%s: %s: %s(ecode=%d): %s(errno=%d)" efmt, \
          ServerHasName(), ClientHasName(hndClient), \
          bsStrError(BS_ECODE_SYS), BS_ECODE_SYS, \
          strerror(errno), errno, ##__VA_ARGS__)

/*!
 * \brief Log \h_botsense Error and Send Error Response.
 *
 * \param hndClient Client handle.
 * \param hndVConn  Requestor's virtual connection handle.
 * \param uTid      Request-Response transaction id.
 * \param ecode     \h_botsense error code.
 * \param efmt      Error output format string literal.
 * \param ...       Error variable arguments.    
 */
#define BSPROXY_SEND_ERROR_RSP(hndClient, hndVConn, uTid, ecode, efmt, ...) \
  do \
  { \
    BSPROXY_LOG_ERROR(hndClient, ecode, "req_vconn=%u: " efmt, \
        hndVConn, ##__VA_ARGS__); \
    ClientSendErrorRsp(hndClient, uTid, ecode, efmt, ##__VA_ARGS__); \
  } while(0)

/*!
 * \brief Log NetMsgs (Un)Packing Error and Send Error Response.
 *
 * \param hndClient Client handle.
 * \param hndVConn  Requestor's virtual connection handle.
 * \param uTid      Request-Response transaction id.
 * \param nmecode   NetMsgs error code.
 * \param efmt      Error output format string literal.
 * \param ...       Error variable arguments.    
 */
#define BSPROXY_SEND_NMERROR_RSP(hndClient, hndVConn, uTid, nmecode, efmt, ...) \
  do \
  { \
    BSPROXY_LOG_NMERROR(hndClient, nmecode, "req_vconn=%u: " efmt, \
        hndVConn, ##__VA_ARGS__); \
    ClientSendErrorRsp(hndClient, uTid, BS_ECODE_BAD_MSG, efmt, \
                          ##__VA_ARGS__); \
  } while(0)

/*!
 * \brief Log System Error and Send Error Response.
 *
 * \param hndClient Client handle.
 * \param hndVConn  Requestor's virtual connection handle.
 * \param uTid      Request-Response transaction id.
 * \param efmt      Error output format string literal.
 * \param ...       Error variable arguments.    
 */
#define BSPROXY_SEND_SYSERROR_RSP(hndClient, hndVConn, uTid, efmt, ...) \
  do \
  { \
    BSPROXY_LOG_SYSERROR(hndClient, "req_vconn=%u: " efmt, \
        hndVConn, ##__VA_ARGS__); \
    ClientSendErrorRsp(hndClient, uTid, BS_ECODE_SYS, \
        "%s(errno=%d): vconn=%u:" efmt, \
        strerror(errno), errno, hndVConn, ##__VA_ARGS__); \
  } while(0)


/*!
 * \brief Log client request.
 *
 * \param hndClient Client handle.
 * \param pMsgHdr   Request message header.
 */
#ifdef LOG
#define BSPROXY_LOG_REQ(hndClient, pMsgHdr) \
  do \
  { \
    if( LOGABLE(LOG_LEVEL_DIAG2) ) \
    { \
      LOGDIAG2("%s: %s:", ServerHasName(), ClientHasName(hndClient)); \
      BsProxyLogMsgHdr("Rx Req", pMsgHdr); \
    } \
  } while(0)
#else
#define BSPROXY_LOG_REQ(hndClient, pMsgHdr)
#endif // LOG

/*!
 * \brief Log client request.
 *
 * \param hndClient Client handle.
 * \param pMsgHdr   Request message header.
 */
#ifdef LOG
#define BSPROXY_LOG_RSP(hndClient, pMsgHdr) \
  do \
  { \
    if( LOGABLE(LOG_LEVEL_DIAG2) ) \
    { \
      LOGDIAG2("%s: %s:", ServerHasName(), ClientHasName(hndClient)); \
      BsProxyLogMsgHdr("Tx Rsp", pMsgHdr); \
    } \
  } while(0)
#else
#define BSPROXY_LOG_RSP(hndClient, pMsgHdr)
#endif // LOG

/*!
 * Check if handle is in virtual connection range.
 *
 * The special server handle is included.
 *
 * \sa bsProxyMsg.h
 *
 * \param hndVConn  Virtual connection handle.
 *
 * \return true or false
 */
#define BSPROXY_CHK_VCONN_HND(hndVConn) \
  (((int)(hndVConn) >= BSPROXY_VCONN_MIN) && \
   ((int)(hndVConn) <= BSPROXY_VCONN_MAX))

/*!
 * Check if handle is in virtual connection device module range.
 *
 * \sa bsProxyMsg.h
 *
 * \param hndVConn  Virtual connection handle.
 *
 * \return true or false
 */
#define BSPROXY_CHK_MOD_VCONN_HND(hndVConn) \
  (((int)(hndVConn) >= BSPROXY_VCONN_MOD_MIN) && \
   ((int)(hndVConn) <= BSPROXY_VCONN_MOD_MAX))


//-----------------------------------------------------------------------------
// BotSense Utilities
//-----------------------------------------------------------------------------

/*! 
 * \brief Mark the current time. Resolution is microseconds.
 *
 * \param pTvMark   Pointer to timeval structure to be populated with
 *                  the current system seconds and useconds.
 */
static inline void timer_mark(struct timeval  *pTvMark)
{
  if( gettimeofday(pTvMark, NULL) != OK )
  {
    LOGSYSERROR("gettimeofday()");
    timerclear(pTvMark);
  }
}

extern uint_t timer_elapsed(struct timeval *pTvMark);

/*!
 * \brief Get the \h_botsense server.
 *
 * \note In the future, if bsProxy is expanded to be multi-threaded multi-server
 * then this call will return the correct server given the caller's context.
 *
 * \return Pointer to the server's control block.
 */
INLINE_IN_H BsProxyServerCtl_T *ServerGet()
{
  return BsServerCtl;
}

/*!
 * \brief Get the \h_botsense server's official name.
 *
 * \return Null-terminated server name string.
 */
INLINE_IN_H const char *ServerHasName()
{
  return ServerGet()->m_sServerName;
}

/*!
 * \brief Get the \h_botsense server's listening address.
 *
 * \return Null-terminated server address string.
 */
INLINE_IN_H const char *ServerHasAddr()
{
  return SocketAttrGetLocalName(ServerGet()->m_pServerSockListener);
}

/*!
 * \brief Get the \h_botsense server's listening port.
 *
 * \return Port.
 */
INLINE_IN_H int ServerHasPort()
{
  return ServerGet()->m_nServerPort;
}

/*!
 * \brief Convert the \h_botsense server client handle to client socket
 * descriptor.
 *
 * \note Currently, the handle and socket descriptor are one and the same. But
 * this mapping could change in the future.
 *
 * \param hndClient   \h_botsense client handle.
 *
 * \return Client socket descriptor.
 */
INLINE_IN_H int ServerClientHnd2Sd(BsProxyClientHnd_T hndClient)
{
  return (int)hndClient;
}

/*!
 * \brief Convert the \h_botsense server client socket descriptor to client
 * handle.
 *
 * \note Currently, the handle and socket descriptor are one and the same. But
 * this mapping could change in the future.
 *
 * \param sd    \h_botsense client socket descriptor.
 *
 * \return Client handle.
 */
INLINE_IN_H BsProxyClientHnd_T ServerClientSd2Hnd(int sd)
{
  return (BsProxyClientHnd_T)sd;
}

/*!
 * \brief Get the \h_botsense server client.
 *
 * \note The client is not locked.
 *
 * \param hndClient   \h_botsense client handle.
 *
 * \return \h_botsense client control block.
 */
INLINE_IN_H BsProxyClientCtl_T *ServerGetClient(BsProxyClientHnd_T hndClient)
{
  int sd = ServerClientHnd2Sd(hndClient);

  if( (sd >= 0) && (sd < BSPROXY_FD_SETSIZE) )
  {
    return ServerGet()->m_pServerRegClient[sd];
  }
  else
  {
    return NULL;
  }
}

/*!
 * \brief Get this \h_botsense client official name.
 *
 * \warning The client is not locked. Use in a safe context.
 *
 * \param pClient   \h_botsense client.
 *
 * \return Null-terminated client name string.
 */
INLINE_IN_H const char *ClientThisHasName(BsProxyClientCtl_T *pClient)
{
  return pClient != NULL? pClient->m_sClientName: "(none)";
}

/*!
 * \brief Get the \h_botsense client official name.
 *
 * \warning The client is not locked. Use in a safe context.
 *
 * \param hndClient   \h_botsense client handle.
 *
 * \return Null-terminated client name string.
 */
INLINE_IN_H const char *ClientHasName(BsProxyClientHnd_T hndClient)
{
  return ClientThisHasName(ServerGetClient(hndClient));
}

/*!
 * \brief Get the \h_botsense client's server-terminated message trace state.
 *
 * \warning The client is not locked. Use in a safe context.
 *
 * \param hndClient   \h_botsense client handle.
 *
 * \return Trace state.
 */
INLINE_IN_H bool_t ClientGetTraceState(BsProxyClientHnd_T hndClient)
{
  BsProxyClientCtl_T *pClient = ServerGetClient(hndClient);
  
  return pClient != NULL? pClient->m_bServerTrace: false;
}

/*!
 * \brief Set the \h_botsense client's server-terminated message trace state.
 *
 * \warning The client is not locked. Use in a safe context.
 *
 * \param hndClient   \h_botsense client handle.
 * \param bTrace      New trace state.
 */
INLINE_IN_H void ClientSetTraceState(BsProxyClientHnd_T hndClient,
                                     bool_t             bTrace)
{
  BsProxyClientCtl_T *pClient = ServerGetClient(hndClient);
  
  if( pClient != NULL )
  {
    pClient->m_bServerTrace = bTrace;
  }
}

#ifdef LOG
extern void BsProxyLogMsgHdr(const char *sPreface, BsProxyMsgHdr_T *pMsgHdr);
#endif // LOG


//-----------------------------------------------------------------------------
// BotSense Client Prototypes
//-----------------------------------------------------------------------------

extern void ClientOneTimeInit();

extern BsProxyClientCtl_T *ClientAcquire(BsProxyClientHnd_T hndClient);

extern void ClientRelease(BsProxyClientHnd_T hndClient);

extern BsProxyClientCtl_T *ClientNew(Socket_T *pSockClient);

extern void ClientDelete(BsProxyClientCtl_T *pClient);

extern void ClientSetState(BsProxyClientCtl_T   *pClient,
                           BsProxyClientState_T  eNewState);

extern int ClientRecvReq(BsProxyClientHnd_T   hndClient,
                         BsProxyMsgHdr_T     *pMsgHdr,
                         byte_t             **addrBuf);

extern int ClientSendOkRsp(BsProxyClientHnd_T hndClient, BsTid_T uTid);

extern int ClientSendErrorRsp(BsProxyClientHnd_T  hndClient,
                              BsTid_T             uTid,
                              int                 nECode,
                              const char         *sErrFmt,
                              ...);

extern int ClientSendVErrorRsp(BsProxyClientHnd_T hndClient,
                               BsTid_T            uTid,
                               int                nECode,
                               const char        *sErrFmt,
                               va_list            ap);

extern int ClientSendServerRsp(BsProxyClientHnd_T   hndClient,
                               BsTid_T              uTid,
                               BsProxyMsgId_T       uMsgId,
                               void                *pMsgRsp);

extern int ClientSendRsp(BsProxyClientHnd_T hndClient,
                         BsVConnHnd_T       hndVConn,
                         BsTid_T            uTid,
                         BsMsgId_T          uMsgId,
                         byte_t             bufRsp[],
                         size_t             uRspSize);


//-----------------------------------------------------------------------------
// BotSense Virtual Connection Prototypes
//-----------------------------------------------------------------------------

extern void VConnOneTimeInit();

extern BsProxyVConn_T *VConnNew();

extern void VConnDelete(BsProxyVConn_T *pVConn);

extern BsProxyVConn_T *VConnGet(BsVConnHnd_T hndVConn);

extern int VConnOpenServer();

extern int VConnOpenDev(BsProxyClientHnd_T  hndClient,
                        const char         *sDevName,
                        const char         *sModName,
                        byte_t              argbuf[],
                        size_t              uArgLen,
                        bool_t              bTrace);

extern int VConnClose(BsProxyClientHnd_T hndClient, BsVConnHnd_T hndVConn);

extern BsProxyVConn_T *VConnAcquire(BsVConnHnd_T hndVConn);

extern void VConnRelease(BsVConnHnd_T hndVConn);


//-----------------------------------------------------------------------------
// BotSense Interface Module Prototypes
//-----------------------------------------------------------------------------

extern void ModOneTimeInit(DListStr_T *pDListLibPath);

extern char *ModNewModUri(const char *sModName);

extern BsProxyModIF_T *ModGetLoadedIF(const char *sModUri);

extern BsProxyModIF_T *ModLoad(const char *sModUri);

extern void ModUnload(BsProxyModIF_T *pModIF);


//-----------------------------------------------------------------------------
// BotSense Service Thread Prototypes
//-----------------------------------------------------------------------------

extern BsProxyThReq_T *ThReqNew(BsProxyClientHnd_T  hndClient,
                                BsVConnHnd_T        hndVConn,
                                BsTid_T             uTid,
                                BsMsgId_T           uMsgId,
                                byte_t              bufReq[],
                                size_t              uReqLen);

extern void ThReqDelete(BsProxyThReq_T *pThReq);

extern int ThQueue(BsProxyThCtl_T      *pThCtl,
                   BsProxyClientHnd_T   hndClient,
                   BsVConnHnd_T         hndVConn,
                   BsTid_T              uTid,
                   BsMsgId_T            uMsgId,
                   byte_t               bufReq[],
                   size_t               uReqLen);

extern BsProxyThReq_T *ThDequeue(BsProxyThCtl_T *pThCtl);

extern void ThOneTimeInit();

extern char *ThNewDevUri(const char *sDevName);

extern BsProxyThCtl_T *ThCreateDevThread(const char *sDevUri);

extern BsProxyThCtl_T *ThCreateServerThread();

extern void ThDestroyThread(BsProxyThCtl_T *pThCtl);


//-----------------------------------------------------------------------------
// BotSense Server Request Thread Prototypes
//-----------------------------------------------------------------------------

extern int ServerRequest(BsProxyClientHnd_T hndClient,
                         BsVConnHnd_T       hndVConn,
                         BsTid_T            uTid,
                         BsMsgId_T          uMsgId,
                         byte_t             bufReq[],
                         size_t             uReqLen);


#endif // _BSPROXY_H
