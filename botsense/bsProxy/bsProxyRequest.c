////////////////////////////////////////////////////////////////////////////////
//
// Package:   BotSense
//
// Program:   bsProxy
//
// File:      bsProxyRequest.c
//
/*! \file
 *
 * $LastChangedDate: 2010-08-20 11:36:38 -0600 (Fri, 20 Aug 2010) $
 * $Rev: 568 $
 *
 * \brief \h_botsense bsProxy server-terminated requests.
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/new.h"

#include "botsense/BotSense.h"
#include "botsense/bsProxyModIF.h"
#include "botsense/bsProxyMsgs.h"

#include "bsProxy.h"
#include "version.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Service client's request to perform a loopback.
 *
 * \par Execution Context:
 * Server service thread.
 *
 * \param hndClient   \h_botsense client handle.
 * \param hndVConn    Virtual connection handle.
 * \param uTid        Request-response transaction id.
 * \param uMsgId      Request message id.
 * \param [in] bufReq Packed request message body buffer.   
 * \param uReqLen     Length of packed request (number of bytes).
 * \param bTrace      Do [not] trace this request-repsonse.
 *
 * \copydoc doc_return_std
 */
static int ReqLoopback(BsProxyClientHnd_T hndClient,
                       BsVConnHnd_T       hndVConn,
                       BsTid_T            uTid,
                       BsMsgId_T          uMsgId,
                       byte_t             bufReq[],
                       size_t             uReqLen,
                       bool_t             bTrace)
{
  static BsProxyMsgId_T uMsgIdRsp = BsProxyMsgIdRspLoopback;

  BsProxyReqLoopback_T  msgReq;
  BsProxyRspLoopback_T  msgRsp;
  int                   rc;

  //
  // Unpack client request.
  //
  rc = BsProxyUnpackReqLoopback(bufReq, uReqLen, &msgReq, bTrace);

  if( rc < 0 )
  {
    BSPROXY_SEND_NMERROR_RSP(hndClient, hndVConn, uTid, rc, "MsgId=%u", uMsgId);
    return -BS_ECODE_BAD_MSG;
  }

  //
  // Execute client request.
  //
  msgReq.m_cdata[BSPROXY_REQLOOPBACK_CDATA_LEN] = 0;

  // print to standard error
  fprintf(stderr, "%s: %s\n", ClientHasName(hndClient), msgReq.m_cdata);

  //
  // Fill in response.
  //
  strcpy_s(msgRsp.m_cdata, BSPROXY_RSPLOOPBACK_CDATA_LEN+1, msgReq.m_cdata);

  //
  // Send server response.
  //
  rc = ClientSendServerRsp(hndClient, uTid, uMsgIdRsp, &msgRsp);

  return rc;
}

/*!
 * \brief Service client's request to set the server's logging level.
 *
 * \par Execution Context:
 * Server service thread.
 *
 * \param hndClient   \h_botsense client handle.
 * \param hndVConn    Virtual connection handle.
 * \param uTid        Request-response transaction id.
 * \param uMsgId      Request message id.
 * \param [in] bufReq Packed request message body buffer.   
 * \param uReqLen     Length of packed request (number of bytes).
 * \param bTrace      Do [not] trace this request-repsonse.
 *
 * \copydoc doc_return_std
 */
static int ReqSetLogging(BsProxyClientHnd_T hndClient,
                         BsVConnHnd_T       hndVConn,
                         BsTid_T            uTid,
                         BsMsgId_T          uMsgId,
                         byte_t             bufReq[],
                         size_t             uReqLen,
                         bool_t             bTrace)
{
  BsProxyReqSetLogging_T  msgReq;
  int                     rc;

  //
  // Unpack client request.
  //
  rc = BsProxyUnpackReqSetLogging(bufReq, uReqLen, &msgReq, bTrace);

  if( rc < 0 )
  {
    BSPROXY_SEND_NMERROR_RSP(hndClient, hndVConn, uTid, rc, "MsgId=%u", uMsgId);
    return -BS_ECODE_BAD_MSG;
  }

  //
  // Execute client request.
  //
  LOG_SET_THRESHOLD(msgReq.m_level);

  //
  // Send server response.
  //
  return ClientSendOkRsp(hndClient, uTid);
}

/*!
 * \brief Service client's request to set server or client terminated message
 * tracing.
 *
 * \par Execution Context:
 * Server service thread.
 *
 * \param hndClient   \h_botsense client handle.
 * \param hndVConn    Virtual connection handle.
 * \param uTid        Request-response transaction id.
 * \param uMsgId      Request message id.
 * \param [in] bufReq Packed request message body buffer.   
 * \param uReqLen     Length of packed request (number of bytes).
 * \param bTrace      Do [not] trace this request-repsonse.
 *
 * \copydoc doc_return_std
 */
static int ReqMsgTrace(BsProxyClientHnd_T hndClient,
                       BsVConnHnd_T       hndVConn,
                       BsTid_T            uTid,
                       BsMsgId_T          uMsgId,
                       byte_t             bufReq[],
                       size_t             uReqLen,
                       bool_t             bTrace)
{
  BsProxyReqMsgTrace_T  msgReq;
  BsProxyVConn_T       *pVConn;
  int                   rc;

  //
  // Unpack client request.
  //
  rc = BsProxyUnpackReqMsgTrace(bufReq, uReqLen, &msgReq, bTrace);

  if( rc < 0 )
  {
    BSPROXY_SEND_NMERROR_RSP(hndClient, hndVConn, uTid, rc, "MsgId=%u", uMsgId);
    return -BS_ECODE_BAD_MSG;
  }

  //
  // Execute client request.
  //
  if( msgReq.m_vconn == BSPROXY_VCONN_SERVER )
  {
    ClientSetTraceState(hndClient, msgReq.m_trace);
  }
  else if( (pVConn = VConnAcquire(msgReq.m_vconn)) == NULL )
  {
    BSPROXY_SEND_ERROR_RSP(hndClient, hndVConn, uTid, BS_ECODE_NO_VCONN,
        "VConn=%d", msgReq.m_vconn);
    return -BS_ECODE_NO_VCONN;
  }
  else
  {
    pVConn->m_pModIF->m_fnModTrace(msgReq.m_vconn, msgReq.m_trace);
    VConnRelease(msgReq.m_vconn);
  }

  //
  // Send server response.
  //
  return ClientSendOkRsp(hndClient, uTid);
}

/*!
 * \brief Service client's request to get the server's version.
 *
 * \par Execution Context:
 * Server service thread.
 *
 * \param hndClient   \h_botsense client handle.
 * \param hndVConn    Virtual connection handle.
 * \param uTid        Request-response transaction id.
 * \param uMsgId      Request message id.
 * \param [in] bufReq Packed request message body buffer.   
 * \param uReqLen     Length of packed request (number of bytes).
 * \param bTrace      Do [not] trace this request-repsonse.
 *
 * \copydoc doc_return_std
 */
static int ReqGetVersion(BsProxyClientHnd_T hndClient,
                         BsVConnHnd_T       hndVConn,
                         BsTid_T            uTid,
                         BsMsgId_T          uMsgId,
                         byte_t             bufReq[],
                         size_t             uReqLen,
                         bool_t             bTrace)
{
  static BsProxyMsgId_T uMsgIdRsp = BsProxyMsgIdRspGetVersion;

  BsProxyRspGetVersion_T  msgRsp;
  int                     rc;

  //  Get the version strings.
  sprintf_s(msgRsp.m_version, BSPROXY_RSPGETVERSION_VERSION_LEN+1,
            "bsProxy %s %s", PKG_VERSION, PKG_TIMESTAMP);

  //
  // Send server response.
  //
  rc = ClientSendServerRsp(hndClient, uTid, uMsgIdRsp, &msgRsp);

  return rc;
}

/*!
 * \brief Service client's request to open a proxied device virtual connection.
 *
 * \par Execution Context:
 * Server service thread.
 *
 * \param hndClient   \h_botsense client handle.
 * \param hndVConn    Virtual connection handle.
 * \param uTid        Request-response transaction id.
 * \param uMsgId      Request message id.
 * \param [in] bufReq Packed request message body buffer.   
 * \param uReqLen     Length of packed request (number of bytes).
 * \param bTrace      Do [not] trace this request-repsonse.
 *
 * \copydoc doc_return_std
 */
static int ReqDevOpen(BsProxyClientHnd_T hndClient,
                      BsVConnHnd_T       hndVConn,
                      BsTid_T            uTid,
                      BsMsgId_T          uMsgId,
                      byte_t             bufReq[],
                      size_t             uReqLen,
                      bool_t             bTrace)
{
  static BsProxyMsgId_T uMsgIdRsp = BsProxyMsgIdRspDevOpen;

  BsProxyReqDevOpen_T   msgReq;
  BsProxyRspDevOpen_T   msgRsp;
  BsProxyClientCtl_T   *pClient;
  int                   n;
  int                   rc;

  //
  // Unpack client request.
  //
  rc = BsProxyUnpackReqDevOpen(bufReq, uReqLen, &msgReq, bTrace);

  if( rc < 0 )
  {
    BSPROXY_SEND_NMERROR_RSP(hndClient, hndVConn, uTid, rc, "MsgId=%u", uMsgId);
    return -BS_ECODE_BAD_MSG;
  }

  //
  // Execute client request.
  //

  //
  // Acquire the client, locking it from other threads. If it cannot be
  // acquired, the client has a bad connection or is disconnected and is
  // (being) unregistered. Don't send response message.
  //
  if( (pClient = ClientAcquire(hndClient)) == NULL )
  {
    return -BS_ECODE_SERVER_BAD_CLIENT;
  }

  //
  // Open a new virtual connection.
  //
  n = VConnOpenDev(hndClient, msgReq.m_devname, msgReq.m_modname,
                 msgReq.m_argbuf.u.m_buf, (size_t)msgReq.m_argbuf.m_count,
                 msgReq.m_trace);

  if( n >= 0 )
  {
    pClient->m_uRefCnt++;
  }

  // Unlock the client. Response message may now be sent on this client.
  ClientRelease(hndClient);

  if( n < 0 )
  {
    BSPROXY_SEND_ERROR_RSP(hndClient, hndVConn, uTid, n,
        "DevName=%s, ModName=%s", msgReq.m_devname, msgReq.m_modname);
    return -n;
  }

  //
  // Fill in response.
  //
  msgRsp.m_vconn = (byte_t)n;

  //
  // Send server response.
  //
  rc = ClientSendServerRsp(hndClient, uTid, uMsgIdRsp, &msgRsp);

  return rc;
}

/*!
 * \brief Service client's request to close a device virtual connection.
 *
 * \par Execution Context:
 * Server service thread.
 *
 * \param hndClient   \h_botsense client handle.
 * \param hndVConn    Virtual connection handle.
 * \param uTid        Request-response transaction id.
 * \param uMsgId      Request message id.
 * \param [in] bufReq Packed request message body buffer.   
 * \param uReqLen     Length of packed request (number of bytes).
 * \param bTrace      Do [not] trace this request-repsonse.
 *
 * \copydoc doc_return_std
 */
static int ReqDevClose(BsProxyClientHnd_T hndClient,
                       BsVConnHnd_T       hndVConn,
                       BsTid_T            uTid,
                       BsMsgId_T          uMsgId,
                       byte_t             bufReq[],
                       size_t             uReqLen,
                       bool_t             bTrace)
{
  BsProxyReqDevClose_T  msgReq;
  BsProxyClientCtl_T   *pClient;
  int                   rc;

  //
  // Unpack client request.
  //
  rc = BsProxyUnpackReqDevClose(bufReq, uReqLen, &msgReq, bTrace);

  if( rc < 0 )
  {
    BSPROXY_SEND_NMERROR_RSP(hndClient, hndVConn, uTid, rc, "MsgId=%u", uMsgId);
    return -BS_ECODE_BAD_MSG;
  }

  //
  // Execute client request.
  //

  //
  // Acquire the client, locking it from other threads. If it cannot be
  // acquired, the client has a bad connection or is disconnected and is (being)
  // unregistered. Don't send response message.
  //
  if( (pClient = ClientAcquire(hndClient)) == NULL )
  {
    return -BS_ECODE_SERVER_BAD_CLIENT;
  }

  if( (rc = VConnClose(hndClient, msgReq.m_vconn)) == BS_OK )
  {
    pClient->m_uRefCnt--;
  }

  // Unlock the client. Response message may now be sent on this client.
  ClientRelease(hndClient);

  if( rc < 0 )
  {
    BSPROXY_SEND_ERROR_RSP(hndClient, hndVConn, uTid, rc,
        "VConn=%d", (int)msgReq.m_vconn);
    return -rc;
  }

  //
  // Send server response.
  //
  return ClientSendOkRsp(hndClient, uTid);
}

/*!
 * \brief Service client's request to get the list of all of the client's
 * opened virtual connection handles.
 *
 * \par Execution Context:
 * Server service thread.
 *
 * \param hndClient   \h_botsense client handle.
 * \param hndVConn    Virtual connection handle.
 * \param uTid        Request-response transaction id.
 * \param uMsgId      Request message id.
 * \param [in] bufReq Packed request message body buffer.   
 * \param uReqLen     Length of packed request (number of bytes).
 * \param bTrace      Do [not] trace this request-repsonse.
 *
 * \copydoc doc_return_std
 */
static int ReqGetVConnList(BsProxyClientHnd_T hndClient,
                           BsVConnHnd_T       hndVConn,
                           BsTid_T            uTid,
                           BsMsgId_T          uMsgId,
                           byte_t             bufReq[],
                           size_t             uReqLen,
                           bool_t             bTrace)
{
  static BsProxyMsgId_T uMsgIdRsp = BsProxyMsgIdRspGetVConnList;

  BsProxyRspGetVConnList_T  msgRsp;
  BsProxyVConn_T           *pVConn;
  int                       n, i;
  int                       rc;

  //
  // Execute client request.
  //

  //
  // Loop through the device virtual connections.
  //
  for(i=0, n=BSPROXY_VCONN_MOD_MIN; n<=BSPROXY_VCONN_MOD_MAX; ++n)
  {
    if( (pVConn = VConnAcquire(n)) != NULL )
    {
      // pointers point to the same client allocated object
      if( pVConn->m_hndClient == hndClient )
      {
        msgRsp.m_vconn.u.m_buf[i++] = (byte_t)n;
      }
      VConnRelease(n);
    }
  }

  msgRsp.m_vconn.m_count = (size_t)i;

  //
  // Send server response.
  //
  rc = ClientSendServerRsp(hndClient, uTid, uMsgIdRsp, &msgRsp);

  return rc;
}

/*!
 * \brief Service client's request to get a virtual connection's information.
 *
 * \par Execution Context:
 * Server service thread.
 *
 * \param hndClient   \h_botsense client handle.
 * \param hndVConn    Virtual connection handle.
 * \param uTid        Request-response transaction id.
 * \param uMsgId      Request message id.
 * \param [in] bufReq Packed request message body buffer.   
 * \param uReqLen     Length of packed request (number of bytes).
 * \param bTrace      Do [not] trace this request-repsonse.
 *
 * \copydoc doc_return_std
 */
static int ReqGetVConnInfo(BsProxyClientHnd_T hndClient,
                           BsVConnHnd_T       hndVConn,
                           BsTid_T            uTid,
                           BsMsgId_T          uMsgId,
                           byte_t             bufReq[],
                           size_t             uReqLen,
                           bool_t             bTrace)
{
  static BsProxyMsgId_T uMsgIdRsp = BsProxyMsgIdRspGetVConnInfo;

  BsProxyReqGetVConnInfo_T  msgReq;
  BsProxyRspGetVConnInfo_T  msgRsp;
  BsProxyVConn_T           *pVConn;
  const BsModInfo_T        *pModInfo;
  int                       rc;

  //
  // Unpack client request.
  //
  rc = BsProxyUnpackReqGetVConnInfo(bufReq, uReqLen, &msgReq, bTrace);

  if( rc < 0 )
  {
    BSPROXY_SEND_NMERROR_RSP(hndClient, hndVConn, uTid, rc, "MsgId=%u", uMsgId);
    return -BS_ECODE_BAD_MSG;
  }

  //
  // Execute client request.
  //

  if( (pVConn = VConnAcquire(msgReq.m_vconn)) != NULL )
  {
    msgRsp.m_vconn  = (byte_t)pVConn->m_hndVConn;
    msgRsp.m_rd     = pVConn->m_rd;
    strcpy_s(msgRsp.m_client, BSPROXY_RSPGETVCONNINFO_CLIENT_LEN+1,
                ClientHasName(hndClient));
    strcpy_s(msgRsp.m_devuri, BSPROXY_RSPGETVCONNINFO_DEVURI_LEN+1,
                pVConn->m_pThCtl->m_sDevUri);
    strcpy_s(msgRsp.m_moduri, BSPROXY_RSPGETVCONNINFO_MODURI_LEN+1,
                pVConn->m_pModIF->m_sModUri);

    pModInfo = pVConn->m_pModIF->m_fnModInfo();

    strcpy_s(msgRsp.m_modver, BSPROXY_RSPGETVCONNINFO_MODVER_LEN+1,
                pModInfo->version);
    strcpy_s(msgRsp.m_moddate, BSPROXY_RSPGETVCONNINFO_MODDATE_LEN+1,
                pModInfo->date);

    VConnRelease(msgReq.m_vconn);
  }

  //
  // Send server response.
  //
  rc = ClientSendServerRsp(hndClient, uTid, uMsgIdRsp, &msgRsp);

  return rc;
}


// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Server service thread request handler.
 *
 * \par Execution Context:
 * Server service thread.
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
int ServerRequest(BsProxyClientHnd_T hndClient,
                  BsVConnHnd_T       hndVConn,
                  BsTid_T            uTid,
                  BsMsgId_T          uMsgId,
                  byte_t             bufReq[],
                  size_t             uReqLen)

{
  bool_t bServerTrace = ClientGetTraceState(hndClient);

  switch( uMsgId )
  {
    case BsProxyMsgIdReqLoopback:
      return ReqLoopback(hndClient, hndVConn, uTid, uMsgId, bufReq, uReqLen,
                        bServerTrace);

    case BsProxyMsgIdReqSetLogging:
      return ReqSetLogging(hndClient, hndVConn, uTid, uMsgId, bufReq, uReqLen,
                            bServerTrace);

    case BsProxyMsgIdReqMsgTrace:
      return ReqMsgTrace(hndClient, hndVConn, uTid, uMsgId, bufReq, uReqLen,
                          bServerTrace);

    case BsProxyMsgIdReqGetVersion:
      return ReqGetVersion(hndClient, hndVConn, uTid, uMsgId, bufReq, uReqLen,
                          bServerTrace);

    case BsProxyMsgIdReqDevOpen:
      return ReqDevOpen(hndClient, hndVConn, uTid, uMsgId, bufReq, uReqLen,
                          bServerTrace);

    case BsProxyMsgIdReqDevClose:
      return ReqDevClose(hndClient, hndVConn, uTid, uMsgId, bufReq, uReqLen,
                          bServerTrace);

    case BsProxyMsgIdReqGetVConnList:
      return ReqGetVConnList(hndClient, hndVConn, uTid, uMsgId, bufReq, uReqLen,
                              bServerTrace);

    case BsProxyMsgIdReqGetVConnInfo:
      return ReqGetVConnInfo(hndClient, hndVConn, uTid, uMsgId, bufReq, uReqLen,
                              bServerTrace);

    default:
      BSPROXY_SEND_ERROR_RSP(hndClient, hndVConn, uTid, BS_ECODE_UNKNOWN_REQ,
          "MsgId=%u", uMsgId);
      return -BS_ECODE_UNKNOWN_REQ;
  }
}
