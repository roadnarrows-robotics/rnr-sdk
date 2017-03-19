////////////////////////////////////////////////////////////////////////////////
//
// Package:   BotSense
//
// Module:    bsNull
// Library:   libbsclient_null
// File:      bsNullClient.c
//
/*! \file
 *
 * $LastChangedDate: 2010-08-20 11:36:38 -0600 (Fri, 20 Aug 2010) $
 * $Rev: 568 $
 *
 * \brief \h_botsense bsProxy client proxied /dev/null device used for testing.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2010-2017. RoadNarrows LLC.\n
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
#include "rnr/log.h"
#include "rnr/netmsgs.h"

#include "botsense/BotSense.h"
#include "botsense/libBotSense.h"
#include "botsense/bsNull.h"
#include "botsense/bsNullMsgs.h"

// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief \h_botsense client application information.
 */
static BsClientAppInfo_T bsNullAppInfo = 
{
  .app_name     = "libbsclient_null",
  .brief        = "The /dev/null proxied device.",
  .version      = "1.0.0",
  .date         = "2010.07.21",
  .maintainer   = "RaodNarrows LLC",
  .license      = "(C) 2010 RoadNarrows LLC. All rights reserved.",

  .fnGetMsgName = bsNullGetMsgName
};

// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Get the DevNull message name.
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
const char *bsNullGetMsgName(BsClient_P   pClient,
                               BsVConnHnd_T hndVConn,
                               uint_t       uMsgId)
{
  const NMMsgDef_T  *pMsgDef;

  pMsgDef = BsNullLookupMsgDef((BsNullMsgId_T)uMsgId);

  return pMsgDef!=NULL? pMsgDef->m_sMsgName: "unknown";
}

/*!
 * \brief Request proxy server to establish a virtual connection to the
 * /dev/null device.
 *
 * \param pClient     \h_botsense client.
 * \param bInitTrace  Initial message tracing enable(true)/disable(false) state.
 * 
 * \return
 * On success, the virtual connection handle is returned.\n
 * \copydoc doc_return_ecode
 */
int bsNullReqOpen(BsClient_P pClient, bool_t bInitTrace)
{
  int     hnd;                      // vconn handle / return code

  //
  // Execute server transaction (returns handle \h_ge 0 on success).
  //
  hnd = bsServerReqOpenDev(pClient, BS_NULL_DEV_NAME, BS_NULL_SERVER_MOD,
                          NULL, (size_t)0, &bsNullAppInfo, bInitTrace);

  // check transaction return code
  BSCLIENT_TRY_ECODE(pClient, hnd, "bsServerReqOpenDev(dev='%s') failed.",
                      BS_NULL_DEV_NAME);

  // return handle
  return hnd;
}

/*!
 * \brief Request proxy server to close client's proxied DevNull device vitual
 * connection.
 * 
 * \param pClient     \h_botsense client.
 * \param hndVConn    Handle to virtual connection to close.
 * 
 * \copydoc doc_return_std
 */
int bsNullReqClose(BsClient_P pClient, BsVConnHnd_T hndVConn)
{
  return bsServerReqCloseDev(pClient, hndVConn);
}

/*!
 * \brief Proxied request to write to /dev/null.
 *
 * \param pClient     \h_botsense client.
 * \param hndVConn    Handle to virtual connection to close.
 * \param [in] wbuf   Write buffer.
 * \param uWriteLen   Number of bytes to write.
 *
 * \return
 * Returns number of bytes written on success.\n
 * \copydoc doc_return_ecode
 */
int bsNullReqWrite(BsClient_P   pClient,
                   BsVConnHnd_T hndVConn,
                   byte_t       wbuf[],
                   size_t       uWriteLen)
{
  static BsNullMsgId_T  msgIdReq = BsNullMsgIdReqWrite;
  static BsNullMsgId_T  msgIdRsp = BsNullMsgIdRspWrite;

  BsNullReqWrite_T  msgReq;                   // request message 
  BsNullRspWrite_T  msgRsp;                   // response message 
  byte_t            buf[BSPROXY_MSG_MAX_LEN]; // req/rsp buffer
  bool_t            bTrace;                   // do [not] trace messages
  int               n;                        // number of bytes/return code

  //
  // Parameter checks.
  //
  BSCLIENT_TRY_EXPR(pClient, BSCLIENT_HAS_VCONN(pClient, hndVConn),
      BS_ECODE_BAD_VAL, "VConn=%d", hndVConn);

  BSCLIENT_TRY_EXPR(pClient,
      (uWriteLen <= (size_t)BSNULL_REQWRITE_WRITEBUF_LEN), BS_ECODE_BAD_VAL,
      "write_len=%zu > max_len=%zu",
      uWriteLen, (size_t)BSNULL_REQWRITE_WRITEBUF_LEN);

  // trace state
  bTrace = bsClientAttrGetTraceState(pClient, hndVConn);

  //
  // Set request message values.
  //
  memcpy(msgReq.m_writebuf.u.m_buf, wbuf, uWriteLen);
  msgReq.m_writebuf.m_count = uWriteLen;

  //
  // Pack request.
  //
  n = BsNullPackReqWrite(&msgReq, BSPROXY_BUF_BODY(buf), bTrace);

  // check packing return code
  BSCLIENT_TRY_NM_ECODE(pClient, n, "MsgId=%u", msgIdReq);

  //
  // Execute request-response transaction.
  //
  n = bsClientTrans(pClient,  hndVConn,
                    msgIdReq, buf, (size_t)n,
                    msgIdRsp, buf, sizeof(buf));

  // check transaction return code
  BSCLIENT_TRY_ECODE(pClient, n, "MsgId=%u: Transaction failed.", msgIdReq);

  //
  // Unpack response.
  //
  n = BsNullUnpackRspWrite(buf, (size_t)n, &msgRsp, bTrace);

  // check unpack return code
  BSCLIENT_TRY_NM_ECODE(pClient, n, "MsgId=%u", msgIdRsp);

  //
  // Set return values from response.
  //
  return (int)msgRsp.m_byteswritten;
}
