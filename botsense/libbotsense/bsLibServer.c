////////////////////////////////////////////////////////////////////////////////
//
// Package:   BotSense
//
// Library:   libbsclient
//
// File:      bsLibServer.c
//
/*! \file
 *
 * $LastChangedDate: 2010-08-20 11:36:38 -0600 (Fri, 20 Aug 2010) $
 * $Rev: 568 $
 *
 * \brief Server-Client connection, control, and information functions.
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

#include "rnr/rnrconfig.h"
#include "rnr/sock.h"
#include "rnr/new.h"
#include "rnr/log.h"

#include "botsense/BotSense.h"
#include "botsense/libBotSense.h"
#include "botsense/bsProxyMsgs.h"

#include "bsLibInternal.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

#ifdef LOG
/*!
 * \brief Log virtual connection open event.
 * \param pClient   \h_botsense client.
 * \param index     Internal vConnection index.
 */
#define _BS_LOG_VCONN_OPEN(pClient, index) \
  do \
  { \
    if( LOGABLE(LOG_LEVEL_DIAG2) ) \
    { \
      bsLogVConnOpenEvent(pClient, index); \
    } \
  } while(0)

/*!
 * \brief Log virtual connection open event.
 *
 * \param pClient   \h_botsense client.
 * \param index     Internal vConnection index.
 */
static void bsLogVConnOpenEvent(BsClient_T *pClient, int index)
{
  FILE                    *fp;
  BsVConn_T               *pVConn;
  const BsClientAppInfo_T *pAppInfo;

  fp        = LOG_GET_LOGFP();
  pVConn    = pClient->m_tblVConn[index];

  if( pVConn == NULL )
  {
    fprintf(fp, "%s: internal data corruption.\n", pClient->m_sClientName);
    return;
  }

  pAppInfo  = pVConn->m_pAppInfo;

  fprintf(fp, "%s: Opened Virtual Connection\n", pClient->m_sClientName); 
  fprintf(fp, "{\n");
  fprintf(fp, "  VConn:      %d\n", pVConn->m_hndVConn);
  fprintf(fp, "  Device:     %s\n", pVConn->m_sDevName);
  fprintf(fp, "  I/F Module: %s\n", pVConn->m_sModName);
  if( pAppInfo != NULL )
  {
    fprintf(fp, "  Application = {\n");
    if( pAppInfo->app_name != NULL )
    {
      fprintf(fp, "    Name:       %s\n", pAppInfo->app_name);
    }
    if( pAppInfo->brief != NULL )
    {
      fprintf(fp, "    Brief:      %s\n", pAppInfo->brief);
    }
    if( pAppInfo->version != NULL )
    {
      fprintf(fp, "    Version:    %s\n", pAppInfo->version);
    }
    if( pAppInfo->date != NULL )
    {
      fprintf(fp, "    Date:       %s\n", pAppInfo->date);
    }
    if( pAppInfo->maintainer != NULL )
    {
      fprintf(fp, "    Maintainer: %s\n", pAppInfo->maintainer);
    }
    fprintf(fp, "  }\n");
  }
  fprintf(fp, "}\n");
}

#else

/*!
 * \brief Do not log virtual connection open event.
 * \param pClient   \h_botsense client.
 * \param hndVConn  Virtual connection handle.
 */
#define _BS_LOG_VCONN_OPEN(pClient, hndVConn)

#endif // LOG


// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Connect to the bsProxy server.
 * 
 * \param pClient         \h_botsense client.
 * \param sServerHostName Server's hostname in either a network name or a 
 *                        dotted IP address.
 * \param ipServerPort    Server's listen port number.
 * 
 * \copydoc doc_return_std
 */
int bsServerConnect(BsClient_P    pClient,
                    const char   *sServerHostName,
                    int           ipServerPort)
{
  Socket_T  *pSocket;

  // close socket if already opened
  if( SocketStateIsOpen(pClient->m_pSocket) )
  {
    bsServerDisconnect(pClient);
  }

  // make TCP connection
  pSocket = SocketOpenTcpConnection(sServerHostName, ipServerPort);

  if( pSocket == NULL )
  {
    BSCLIENT_LOG_SYSERROR(pClient, BS_ECODE_SERVER_CONN_FAIL,
        "%s:%d", sServerHostName, ipServerPort);
    return -BS_ECODE_SERVER_CONN_FAIL;
  }

  // set socket for non-block I/O
  SocketAttrSetNonBlocking(pSocket);

  LOGDIAG2("%s: Connected to server at %s.\n",
      pClient->m_sClientName, SocketAttrGetRemoteName(pSocket));

  pClient->m_pSocket = pSocket;

  return BS_OK;
} 

/*!
 * \brief Disconnect from the bsProxy server.
 * 
 * \param pClient   \h_botsense client.
 * 
 * \copydoc doc_return_std
 */
int bsServerDisconnect(BsClient_P pClient)
{
  if( SocketStateIsOpen(pClient->m_pSocket) )
  {
    LOGDIAG2("%s: Disconnected from server at %s.\n",
        pClient->m_sClientName, SocketAttrGetRemoteName(pClient->m_pSocket));
    SocketClose(pClient->m_pSocket);
    SocketDelete(pClient->m_pSocket);
    pClient->m_pSocket = NULL;
    bsVConnClearAll(pClient);
  }

  return BS_OK;
}

/*!
 * \brief Request server to loopback the requested message data.
 * 
 * \param pClient               \h_botsense client.
 * \param [in,out] sLoopbackMsg Null-terminated message string to loopback.
 * 
 * \copydoc doc_return_std
 */
int bsServerReqLoopback(BsClient_P pClient, char sLoopbackMsg[])
{
  static BsProxyMsgId_T msgIdReq = BsProxyMsgIdReqLoopback;
  static BsProxyMsgId_T msgIdRsp = BsProxyMsgIdRspLoopback;

  BsProxyReqLoopback_T  msgReq;                   // request message 
  BsProxyRspLoopback_T  msgRsp;                   // response message
  byte_t                buf[BSPROXY_MSG_MAX_LEN]; // req/rsp buffer
  bool_t                bTrace;                   // do [not] trace messages
  int                   n;                        // number of bytes/return code
  size_t                m;                        // loopback cdata length

  // trace state
  bTrace = bsClientAttrGetTraceState(pClient, BSPROXY_VCONN_SERVER);

  //
  // Set request message values.
  //
  m = strlen(sLoopbackMsg) + 1;
  if( sizeof(msgReq.m_cdata) < m )
  {
    m = sizeof(msgReq.m_cdata);
  }

  strcpy_s(msgReq.m_cdata, m, sLoopbackMsg);

  //
  // Pack request.
  //
  n = BsProxyPackReqLoopback(&msgReq, BSPROXY_BUF_BODY(buf), bTrace);

  // check packing return code
  BSCLIENT_TRY_NM_ECODE(pClient, n, "MsgId=%u", msgIdReq);

  //
  // Execute request-response transaction.
  //
  n = bsClientTrans(pClient,  BSPROXY_VCONN_SERVER,
                    msgIdReq, buf, (size_t)n,
                    msgIdRsp, buf, sizeof(buf));

  // check transaction return code
  BSCLIENT_TRY_ECODE(pClient, n, "MsgId=%u: Transaction failed.", msgIdReq);

  //
  // Unpack response.
  //
  n = BsProxyUnpackRspLoopback(buf, (size_t)n, &msgRsp, bTrace);

  // check unpack return code
  BSCLIENT_TRY_NM_ECODE(pClient, n, "MsgId=%u", msgIdRsp);

  //
  // Set return values from response.
  //
  strcpy_s(sLoopbackMsg, m, msgRsp.m_cdata);

  return BS_OK;
}

/*!
 * \brief Request server to set the server's logging level.
 * 
 * \param pClient         \h_botsense client.
 * \param nLogLevel       Log level.
 * 
 * \copydoc doc_return_std
 */
int bsServerReqSetLogging(BsClient_P pClient, int nLogLevel)
{
  static BsProxyMsgId_T msgIdReq = BsProxyMsgIdReqSetLogging;
  static BsProxyMsgId_T msgIdRsp = BsProxyMsgIdRspOk;

  BsProxyReqSetLogging_T  msgReq;                   // request message 
  byte_t                  buf[BSPROXY_MSG_MAX_LEN]; // req/rsp buffer
  bool_t                  bTrace;                   // do [not] trace messages
  int                     n;                        // num. of bytes/return code

  // trace state
  bTrace = bsClientAttrGetTraceState(pClient, BSPROXY_VCONN_SERVER);

  //
  // Set request message values.
  //
  msgReq.m_level = nLogLevel;

  //
  // Pack request.
  //
  n = BsProxyPackReqSetLogging(&msgReq, BSPROXY_BUF_BODY(buf), bTrace);

  // check packing return code
  BSCLIENT_TRY_NM_ECODE(pClient, n, "MsgId=%u", msgIdReq);

  //
  // Execute request-response transaction.
  //
  n = bsClientTrans(pClient,  BSPROXY_VCONN_SERVER,
                    msgIdReq, buf, (size_t)n,
                    msgIdRsp, buf, sizeof(buf));

  // check transaction return code
  BSCLIENT_TRY_ECODE(pClient, n, "MsgId=%u: Transaction failed.", msgIdReq);

  //
  // Unpack response. No response body.
  //

  //
  // Set return values from response. No response values.
  //

  return BS_OK;
}

/*!
 * \brief Request server to return the server's version string.
 * 
 * \param pClient       \h_botsense client.
 * \param [out] bufVer  Output null-terminated string buffer.
 * \param bufSize       Size of output buffer (chars).
 * 
 * \copydoc doc_return_std
 */
int bsServerReqGetVersion(BsClient_P pClient, char bufVer[], size_t bufSize)
{
  static BsProxyMsgId_T msgIdReq = BsProxyMsgIdReqGetVersion;
  static BsProxyMsgId_T msgIdRsp = BsProxyMsgIdRspGetVersion;

  BsProxyRspGetVersion_T  msgRsp;                   // response message
  byte_t                  buf[BSPROXY_MSG_MAX_LEN]; // req/rsp buffer
  bool_t                  bTrace;                   // do [not] trace messages
  int                     n;                        // num. of bytes/return code

  // trace state
  bTrace = bsClientAttrGetTraceState(pClient, BSPROXY_VCONN_SERVER);

  //
  // Set request message values. No request values.
  //

  //
  // Pack request. No request body.
  //

  //
  // Execute request-response transaction.
  //
  n = bsClientTrans(pClient,  BSPROXY_VCONN_SERVER,
                    msgIdReq, buf, (size_t)0,
                    msgIdRsp, buf, sizeof(buf));

  // check transaction return code
  BSCLIENT_TRY_ECODE(pClient, n, "MsgId=%u: Transaction failed.", msgIdReq);

  //
  // Unpack response.
  //
  n = BsProxyUnpackRspGetVersion(buf, (size_t)n, &msgRsp, bTrace);

  // check unpack return code
  BSCLIENT_TRY_NM_ECODE(pClient, n, "MsgId=%u", msgIdRsp);

  //
  // Set return values from response.
  //
  strcpy_s(bufVer, bufSize, msgRsp.m_version);

  return BS_OK;
}

/*!
 * \brief Request server to enable/disable message tracing on a virtual
 * connection.
 * 
 * \param pClient         \h_botsense client.
 * \param hndVConn        Virtual connection handle to set tracing state.
 * \param bNewTrace       Message new tracing enable(true)/disable(false) state.
 * 
 * \copydoc doc_return_std
 */
int bsServerReqMsgTrace(BsClient_P   pClient,
                        BsVConnHnd_T hndVConn,
                        bool_t       bNewTrace)
{
  static BsProxyMsgId_T msgIdReq = BsProxyMsgIdReqMsgTrace;
  static BsProxyMsgId_T msgIdRsp = BsProxyMsgIdRspOk;

  BsProxyReqMsgTrace_T  msgReq;                   // request message 
  byte_t                buf[BSPROXY_MSG_MAX_LEN]; // req/rsp buffer
  bool_t                bTrace;                   // do [not] trace messages
  int                   n;                        // number of bytes/return code

  // trace state
  bTrace = bsClientAttrGetTraceState(pClient, BSPROXY_VCONN_SERVER);

  //
  // Set request message values.
  //
  msgReq.m_vconn = (byte_t)hndVConn;
  msgReq.m_trace  = bNewTrace;

  //
  // Pack request.
  //
  n = BsProxyPackReqMsgTrace(&msgReq, BSPROXY_BUF_BODY(buf), bTrace);

  // check packing return code
  BSCLIENT_TRY_NM_ECODE(pClient, n, "MsgId=%u", msgIdReq);

  //
  // Execute request-response transaction.
  //
  n = bsClientTrans(pClient,  BSPROXY_VCONN_SERVER,
                    msgIdReq, buf, (size_t)n,
                    msgIdRsp, buf, sizeof(buf));

  // check transaction return code
  BSCLIENT_TRY_ECODE(pClient, n, "MsgId=%u: Transaction failed.", msgIdReq);

  //
  // Unpack response. No response body.
  //

  //
  // Set return values from response. No response values.
  //

  return BS_OK;
}

/*!
 * \brief Request server to establish a virtual connection to the device end
 * point.
 *
 * The device is open if not already opened by another virtual connection.
 * Otherwise it is attached to this vconn.
 *
 * The interface module is dynamically loaded into the server and provides
 * the set of services for the client application communicating with the device.
 * 
 * \param pClient     \h_botsense client.
 * \param sDevName    Device path name.
 * \param sModName    Interface module path name.
 * \param argbuf      Packed buffer of module-specific open parameters.
 * \param arglen      Number of packed bytes in argument buffer.
 * \param pAppInfo    Application-specific information and callbacks (optional).
 *                    Set to NULL if no info. Set any member to NULL to ignore
 *                    that value.
 * \param bInitTrace  Initial message tracing enable(true)/disable(false) state.
 * 
 * \return
 * On success, the virtual connection handle is returned.\n
 * \copydoc doc_return_ecode
 */
int bsServerReqOpenDev(BsClient_P               pClient,
                       const char              *sDevName,
                       const char              *sModName,
                       byte_t                   argbuf[],
                       size_t                   arglen,
                       const BsClientAppInfo_T *pAppInfo,
                       bool_t                   bInitTrace)
{
  static BsProxyMsgId_T msgIdReq = BsProxyMsgIdReqDevOpen;
  static BsProxyMsgId_T msgIdRsp = BsProxyMsgIdRspDevOpen;

  BsProxyReqDevOpen_T   msgReq;                   // request message 
  BsProxyRspDevOpen_T   msgRsp;                   // response message
  byte_t                buf[BSPROXY_MSG_MAX_LEN]; // req/rsp buffer
  bool_t                bTrace;                   // do [not] trace messages
  int                   index;                    // internal vconn index
  int                   n;                        // number of bytes/return code
  int                   rc;                       // return code

  //
  // Parameter checks.
  //
  BSCLIENT_TRY_EXPR(pClient, (strlen(sDevName)<=BSPROXY_REQDEVOPEN_DEVNAME_LEN),
      BS_ECODE_BAD_VAL,
      "%zu > %u", strlen(sDevName), BSPROXY_REQDEVOPEN_DEVNAME_LEN);

  BSCLIENT_TRY_EXPR(pClient, (strlen(sModName)<=BSPROXY_REQDEVOPEN_MODNAME_LEN),
      BS_ECODE_BAD_VAL,
      "%zu > %u", strlen(sModName), BSPROXY_REQDEVOPEN_MODNAME_LEN);

  BSCLIENT_TRY_EXPR(pClient, (arglen<=BSPROXY_REQDEVOPEN_ARGBUF_LEN),
      BS_ECODE_BAD_VAL,
      "%zu > %u", arglen, BSPROXY_REQDEVOPEN_ARGBUF_LEN);

  //
  // Create and reserve new client vConnection.
  //
  if((index=bsVConnNew(pClient, sDevName, sModName, pAppInfo, bInitTrace)) < 0)
  {
    return index;
  }

  // server-ended trace state
  bTrace = bsClientAttrGetTraceState(pClient, BSPROXY_VCONN_SERVER);

  //
  // Set request message values.
  //
  msgReq.m_trace = bInitTrace;
  strcpy(msgReq.m_devname, sDevName);
  strcpy(msgReq.m_modname, sModName);
  memcpy(msgReq.m_argbuf.u.m_buf, argbuf, arglen);
  msgReq.m_argbuf.m_count = arglen;

  //
  // Pack request.
  //
  if( (n = BsProxyPackReqDevOpen(&msgReq, BSPROXY_BUF_BODY(buf), bTrace)) < 0 )
  {
    BSCLIENT_LOG_NM_ERROR(pClient, n, "MsgId=%u", msgIdReq);
    rc = -BS_ECODE_BAD_MSG;
  }

  //
  // Execute request-response transaction.
  //
  else if( (n = bsClientTrans(pClient,  BSPROXY_VCONN_SERVER,
                              msgIdReq, buf, (size_t)n,
                              msgIdRsp, buf, sizeof(buf))) < 0 )
  {
    BSCLIENT_LOG_ERROR(pClient, n, "MsgId=%u: Transaction failed.", msgIdReq);
    rc = n;
  }

  //
  // Unpack response.
  //
  else if( (n = BsProxyUnpackRspDevOpen(buf, (size_t)n, &msgRsp, bTrace)) < 0 )
  {
    BSCLIENT_LOG_NM_ERROR(pClient, n, "MsgId=%u", msgIdRsp);
    rc = -BS_ECODE_BAD_MSG;
  }

  //
  // Success
  //
  else
  {
    bsVConnAdd(pClient, msgRsp.m_vconn, index);
    _BS_LOG_VCONN_OPEN(pClient, index);
    rc = (int)msgRsp.m_vconn;
  }

  //
  // Error clean up
  //
  if( rc < 0 )
  {
    bsVConnDelete(pClient, index);
  }

  return rc;
}

/*!
 * \brief Request server to close a client's vitual connection.
 * 
 * \param pClient     \h_botsense client.
 * \param hndVConn    Handle to virtual connection to close.
 * 
 * \copydoc doc_return_std
 */
int bsServerReqCloseDev(BsClient_P pClient, BsVConnHnd_T hndVConn)
{
  static BsProxyMsgId_T msgIdReq = BsProxyMsgIdReqDevClose;
  static BsProxyMsgId_T msgIdRsp = BsProxyMsgIdRspOk;

  BsProxyReqDevClose_T  msgReq;                   // request message 
  byte_t                buf[BSPROXY_MSG_MAX_LEN]; // req/rsp buffer
  bool_t                bTrace;                   // do [not] trace messages
  int                   n;                        // number of bytes/return code
  int                   index;                    // internal vconn index

  //
  // Parameter checks.
  //
  BSCLIENT_TRY_EXPR(pClient, BSCLIENT_HAS_VCONN(pClient, hndVConn),
      BS_ECODE_BAD_VAL, "VConn=%d", hndVConn);


  // trace state
  bTrace = bsClientAttrGetTraceState(pClient, BSPROXY_VCONN_SERVER);

  //
  // Set request message values.
  //
  msgReq.m_vconn = (byte_t)hndVConn;

  //
  // Pack request.
  //
  n = BsProxyPackReqDevClose(&msgReq, BSPROXY_BUF_BODY(buf), bTrace);

  // check packing return code
  BSCLIENT_TRY_NM_ECODE(pClient, n, "MsgId=%u", msgIdReq);

  //
  // Execute request-response transaction.
  //
  n = bsClientTrans(pClient,  BSPROXY_VCONN_SERVER,
                    msgIdReq, buf, (size_t)n,
                    msgIdRsp, buf, sizeof(buf));

  // check transaction return code
  BSCLIENT_TRY_ECODE(pClient, n, "MsgId=%u: Transaction failed.", msgIdReq);

  //
  // Unpack response. No response body.
  //

  //
  // Set return values from response. No response values.
  //

  //
  // Remove and delete client vConnection.
  //
  if( (index = bsVConnRemove(pClient, hndVConn)) >= 0 )
  {
    bsVConnDelete(pClient, index);
  }

  LOGDIAG2("%s: Virtual connecton %u closed.",
            pClient->m_sClientName, (uint_t)hndVConn);

  return BS_OK;
}

/*!
 * \brief Request server to retrieve the server's list of virtual connection
 * handles for this client.
 * 
 * \param pClient             \h_botsense client.
 * \param [out] pVecHandles   Vector of handles.
 * 
 * \copydoc doc_return_std
 */
int bsServerReqGetVConnList(BsClient_P      pClient,
                            BsVecHandles_T *pVecHandles)
{
  static BsProxyMsgId_T msgIdReq = BsProxyMsgIdReqGetVConnList;
  static BsProxyMsgId_T msgIdRsp = BsProxyMsgIdRspGetVConnList;

  BsProxyRspGetVConnList_T  msgRsp;                   // response message
  byte_t                    buf[BSPROXY_MSG_MAX_LEN]; // req/rsp buffer
  bool_t                    bTrace;                   // do [not] trace messages
  int                       n;                        // num bytes/return code
  int                       i;                        // working index

  // trace state
  bTrace = bsClientAttrGetTraceState(pClient, BSPROXY_VCONN_SERVER);

  //
  // Set request message values. No request values.
  //

  //
  // Pack request. No request body.
  //

  //
  // Execute request-response transaction.
  //
  n = bsClientTrans(pClient,  BSPROXY_VCONN_SERVER,
                    msgIdReq, buf, (size_t)0,
                    msgIdRsp, buf, sizeof(buf));

  // check transaction return code
  BSCLIENT_TRY_ECODE(pClient, n, "MsgId=%u: Transaction failed.", msgIdReq);

  //
  // Unpack response.
  //
  n = BsProxyUnpackRspGetVConnList(buf, (size_t)n, &msgRsp, bTrace);

  // check unpack return code
  BSCLIENT_TRY_NM_ECODE(pClient, n, "MsgId=%u", msgIdRsp);

  //
  // Set return values from response.
  //
  for(i=0; (i<msgRsp.m_vconn.m_count) && (i<BSPROXY_VCONN_CLIENT_MAX); ++i)
  {
    pVecHandles->m_vecHnd[i] = (BsVConnHnd_T)msgRsp.m_vconn.u.m_buf[i];
  }
  pVecHandles->m_uCount = msgRsp.m_vconn.m_count;

  return BS_OK;
}

/*!
 * \brief Request server to retrieve the server's information for a given
 * virtual connection.
 * 
 * \param pClient           \h_botsense client.
 * \param hndVConn          Virtual connection handle.
 * \param [out] pVConnInfo  Proxied device info.
 * 
 * \copydoc doc_return_std
 */
int bsServerReqGetVConnInfo(BsClient_P     pClient,
                            BsVConnHnd_T   hndVConn,
                            BsVConnInfo_T *pVConnInfo)
{
  static BsProxyMsgId_T msgIdReq = BsProxyMsgIdReqGetVConnInfo;
  static BsProxyMsgId_T msgIdRsp = BsProxyMsgIdRspGetVConnInfo;

  BsProxyReqGetVConnInfo_T  msgReq;                   // request message 
  BsProxyRspGetVConnInfo_T  msgRsp;                   // response message
  byte_t                    buf[BSPROXY_MSG_MAX_LEN]; // req/rsp buffer
  bool_t                    bTrace;                   // do [not] trace messages
  int                       n;                        // num bytes/return code

  //
  // Parameter checks.
  //
  BSCLIENT_TRY_EXPR(pClient, BSCLIENT_HAS_VCONN(pClient, hndVConn),
      BS_ECODE_BAD_VAL, "VConn=%d", hndVConn);


  // trace state
  bTrace = bsClientAttrGetTraceState(pClient, BSPROXY_VCONN_SERVER);

  //
  // Set request message values.
  //
  msgReq.m_vconn = (byte_t)hndVConn;

  //
  // Pack request.
  //
  n = BsProxyPackReqGetVConnInfo(&msgReq, BSPROXY_BUF_BODY(buf), bTrace);

  // check packing return code
  BSCLIENT_TRY_NM_ECODE(pClient, n, "MsgId=%u", msgIdReq);

  //
  // Execute request-response transaction.
  //
  n = bsClientTrans(pClient,  BSPROXY_VCONN_SERVER,
                    msgIdReq, buf, (size_t)n,
                    msgIdRsp, buf, sizeof(buf));

  // check transaction return code
  BSCLIENT_TRY_ECODE(pClient, n, "MsgId=%u: Transaction failed.", msgIdReq);

  //
  // Unpack response.
  //
  n = BsProxyUnpackRspGetVConnInfo(buf, (size_t)n, &msgRsp, bTrace);

  // check unpack return code
  BSCLIENT_TRY_NM_ECODE(pClient, n, "MsgId=%u", msgIdRsp);

  //
  // Set return values from response.
  //
  pVConnInfo->m_vconn = msgRsp.m_vconn;
  pVConnInfo->m_rd = msgRsp.m_rd;
  strcpy_s(pVConnInfo->m_client, sizeof(pVConnInfo->m_client),
      msgRsp.m_client);
  strcpy_s(pVConnInfo->m_devuri, sizeof(pVConnInfo->m_devuri), msgRsp.m_devuri);
  strcpy_s(pVConnInfo->m_moduri, sizeof(pVConnInfo->m_moduri), msgRsp.m_moduri);
  strcpy_s(pVConnInfo->m_modver, sizeof(pVConnInfo->m_modver), msgRsp.m_modver);
  strcpy_s(pVConnInfo->m_moddate, sizeof(pVConnInfo->m_moddate),
      msgRsp.m_moddate);

  return BS_OK;
}
