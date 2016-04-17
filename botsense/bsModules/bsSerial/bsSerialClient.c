////////////////////////////////////////////////////////////////////////////////
//
// Package:   BotSense
//
// Module:    bsSerial
// Library:   libbsclient_serial
// File:      bsSerialClient.c
//
/*! \file
 *
 * $LastChangedDate: 2010-08-20 11:36:38 -0600 (Fri, 20 Aug 2010) $
 * $Rev: 568 $
 *
 * \brief \h_botsense bsProxy client proxied RS-232 serial device library.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2009-2010.  RoadNarrows LLC.
 * (http://www.roadnarrows.com)\n
 * All Rights Reserved
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
#include "rnr/serdev.h"

#include "botsense/BotSense.h"
#include "botsense/libBotSense.h"
#include "botsense/bsSerial.h"
#include "botsense/bsSerialMsgs.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief \h_botsense client application information.
 */
static BsClientAppInfo_T bsSerialAppInfo = 
{
  .app_name     = "libbsclient_serial",
  .brief        = "Raw R2-232 serial proxied device.",
  .version      = "2.0.0",
  .date         = "2010.07.14",
  .maintainer   = "RaodNarrows LLC",
  .license      = "(C) 2010 RoadNarrows LLC. All rights reserved.",

  .fnGetMsgName = bsSerialGetMsgName
};

// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Get the serial message name.
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
const char *bsSerialGetMsgName(BsClient_P   pClient,
                               BsVConnHnd_T hndVConn,
                               uint_t       uMsgId)
{
  const NMMsgDef_T  *pMsgDef;

  pMsgDef = BsSerialLookupMsgDef((BsSerialMsgId_T)uMsgId);

  return pMsgDef!=NULL? pMsgDef->m_sMsgName: "unknown";
}

/*!
 * \brief Request proxy server to establish a virtual connection to an RS-232
 * serial device.
 *
 * \param pClient     \h_botsense client.
 * \param sDevName    Proxied serial device name (e.g. /dev/ttyS0).
 * \param nBaudRate   Baud rate.
 * \param nByteSize   Bytes size in bits 5...8.
 * \param cParity     Parity. One of: 'N', 'E', 'O'
 * \param nStopBits   Number of stop bits 1, 2
 * \param bRtsCts     Do [not] use hardware flow control.
 * \param bXonXoff    Do [not] use software flow control.
 * \param bInitTrace  Initial message tracing enable(true)/disable(false) state.
 * 
 * \return
 * On success, the virtual connection handle is returned.\n
 * \copydoc doc_return_ecode
 */
int bsSerialReqOpen(BsClient_P  pClient,
                    const char *sDevName,
                    int         nBaudRate,
                    int         nByteSize,
                    int         cParity,
                    int         nStopBits,
                    bool_t      bRtsCts,
                    bool_t      bXonXoff,
                    bool_t      bInitTrace)
{
  static BsSerialMsgId_T  msgIdReq = BsSerialMsgIdReqOpenArgs;
  
  BsSerialReqOpenArgs_T   msgReq;                   // specific open args
  byte_t                  buf[BSPROXY_MSG_MAX_LEN]; // specific open args buffer
  bool_t                  bTrace;                   // do [not] trace messages
  int                     n;                        // num of bytes/return code

  // trace state
  bTrace = bsClientAttrGetTraceState(pClient, BSPROXY_VCONN_SERVER);
  
  //
  // Set specific open argument values.
  //
  msgReq.m_baudrate = (uint_t)nBaudRate;
  msgReq.m_bytesize = (byte_t)nByteSize;
  msgReq.m_parity   = (char)cParity;
  msgReq.m_stopbits = (byte_t)nStopBits;
  msgReq.m_rtscts   = bRtsCts;
  msgReq.m_xonxoff  = bXonXoff;

  //
  // Pack specific open arguments (returns bytes packed \h_ge 0 on success).
  //
  n = BsSerialPackReqOpenArgs(&msgReq, buf, sizeof(buf), bTrace);

  // check packing return code
  BSCLIENT_TRY_NM_ECODE(pClient, n, "MsgId=%u", msgIdReq);


  //
  // Execute server transaction (returns handle \h_ge 0 on success).
  //
  n = bsServerReqOpenDev(pClient, sDevName, BS_SER_SERVER_MOD, buf, (size_t)n,
                         &bsSerialAppInfo, bInitTrace);

  // check transaction return code
  BSCLIENT_TRY_ECODE(pClient, n, "bsServerReqOpenDev(dev='%s') failed.",
      sDevName);

  // return handle
  return n;
}

/*!
 * \brief Request proxy server to close client's proxied serial device vitual
 * connection.
 * 
 * \param pClient     \h_botsense client.
 * \param hndVConn    Handle to virtual connection to close.
 * 
 * \copydoc doc_return_std
 */
int bsSerialReqClose(BsClient_P pClient, BsVConnHnd_T hndVConn)
{
  return bsServerReqCloseDev(pClient, hndVConn);
}

/*!
 * \brief Proxied request to read from an RS-232 serial device.
 *
 * \param pClient     \h_botsense client.
 * \param hndVConn    Handle to virtual connection to close.
 * \param uReadLen    Number of bytes to read.
 * \param [out] rbuf  Read buffer.
 *
 * \return
 * Returns number of bytes read on success.\n
 * \copydoc doc_return_ecode
 */
int bsSerialReqRead(BsClient_P    pClient,
                    BsVConnHnd_T  hndVConn,
                    size_t        uReadLen,
                    byte_t        rbuf[])
{
  static BsSerialMsgId_T  msgIdReq = BsSerialMsgIdReqRead;
  static BsSerialMsgId_T  msgIdRsp = BsSerialMsgIdRspRead;

  BsSerialReqRead_T     msgReq;                   // request message 
  BsSerialRspRead_T     msgRsp;                   // response message 
  byte_t                buf[BSPROXY_MSG_MAX_LEN]; // req/rsp buffer
  bool_t                bTrace;                   // do [not] trace messages
  int                   n;                        // number of bytes/return code

  //
  // Parameter checks.
  //
  BSCLIENT_TRY_EXPR(pClient, BSCLIENT_HAS_VCONN(pClient, hndVConn),
      BS_ECODE_BAD_VAL, "VConn=%d", hndVConn);

  BSCLIENT_TRY_EXPR(pClient, (uReadLen <= (size_t)BSSERIAL_RSPREAD_READBUF_LEN),
      BS_ECODE_BAD_VAL, "read_len=%zu > max_len=%zu",
      uReadLen, (size_t)BSSERIAL_RSPREAD_READBUF_LEN);

  // trace state
  bTrace = bsClientAttrGetTraceState(pClient, hndVConn);

  //
  // Set request message values.
  //
  msgReq.m_readlen = (byte_t)uReadLen;

  //
  // Pack request.
  //
  n = BsSerialPackReqRead(&msgReq, BSPROXY_BUF_BODY(buf), bTrace);

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
  n = BsSerialUnpackRspRead(buf, (size_t)n, &msgRsp, bTrace);

  // check unpack return code
  BSCLIENT_TRY_NM_ECODE(pClient, n, "MsgId=%u", msgIdRsp);

  //
  // Set return values from response.
  //
  if( uReadLen > msgRsp.m_readbuf.m_count )
  {
    uReadLen = msgRsp.m_readbuf.m_count;
  }
  memcpy(rbuf, msgRsp.m_readbuf.u.m_buf, uReadLen);

  return (int)uReadLen;
}

/*!
 * \brief Proxied request to write to a RS-232 serial device.
 *
 * \param pClient       \h_botsense client.
 * \param hndVConn      Handle to virtual connection to close.
 * \param [in] wbuf     Write buffer.
 * \param uWriteLen     Number of bytes to write.
 *
 * \return
 * Returns number of bytes written on success.\n
 * \copydoc doc_return_ecode
 */
int bsSerialReqWrite(BsClient_P   pClient,
                     BsVConnHnd_T hndVConn,
                     byte_t       wbuf[],
                     size_t       uWriteLen)
{
  static BsSerialMsgId_T  msgIdReq = BsSerialMsgIdReqWrite;
  static BsSerialMsgId_T  msgIdRsp = BsSerialMsgIdRspWrite;

  BsSerialReqWrite_T    msgReq;                   // request message 
  BsSerialRspWrite_T    msgRsp;                   // response message 
  byte_t                buf[BSPROXY_MSG_MAX_LEN]; // req/rsp buffer
  bool_t                bTrace;                   // do [not] trace messages
  int                   n;                        // number of bytes/return code

  //
  // Parameter checks.
  //
  BSCLIENT_TRY_EXPR(pClient, BSCLIENT_HAS_VCONN(pClient, hndVConn),
      BS_ECODE_BAD_VAL, "VConn=%d", hndVConn);

  BSCLIENT_TRY_EXPR(pClient,
      (uWriteLen <= (size_t)BSSERIAL_REQWRITE_WRITEBUF_LEN), BS_ECODE_BAD_VAL,
      "write_len=%zu > max_len=%zu",
      uWriteLen, (size_t)BSSERIAL_REQWRITE_WRITEBUF_LEN);

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
  n = BsSerialPackReqWrite(&msgReq, BSPROXY_BUF_BODY(buf), bTrace);

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
  n = BsSerialUnpackRspWrite(buf, (size_t)n, &msgRsp, bTrace);

  // check unpack return code
  BSCLIENT_TRY_NM_ECODE(pClient, n, "MsgId=%u", msgIdRsp);

  //
  // Set return values from response.
  //
  return (int)msgRsp.m_byteswritten;
}

/*!
 * \brief Proxied request to execute a write-read transaction on a RS-232
 * serial device.
 *
 * \param pClient       \h_botsense client.
 * \param hndVConn      Handle to virtual connection to close.
 * \param [in] wbuf     Write buffer.
 * \param uWriteLen     Number of bytes to write.
 * \param uReadLen      Number of bytes to read.
 * \param [out] rbuf    Read buffer.
 *
 * \return
 * Returns number of bytes read on success.\n
 * \copydoc doc_return_ecode
 */
int bsSerialReqTrans(BsClient_P   pClient,
                     BsVConnHnd_T hndVConn,
                     byte_t       wbuf[],
                     size_t       uWriteLen,
                     size_t       uReadLen,
                     byte_t       rbuf[])
{
  static BsSerialMsgId_T  msgIdReq = BsSerialMsgIdReqTrans;
  static BsSerialMsgId_T  msgIdRsp = BsSerialMsgIdRspRead;

  BsSerialReqTrans_T    msgReq;                   // request message 
  BsSerialRspRead_T     msgRsp;                   // response message 
  byte_t                buf[BSPROXY_MSG_MAX_LEN]; // req/rsp buffer
  bool_t                bTrace;                   // do [not] trace messages
  int                   n;                        // number of bytes/return code

  //
  // Parameter checks.
  //
  BSCLIENT_TRY_EXPR(pClient, BSCLIENT_HAS_VCONN(pClient, hndVConn),
      BS_ECODE_BAD_VAL, "VConn=%d", hndVConn);

  BSCLIENT_TRY_EXPR(pClient,
      (uWriteLen <= (size_t)BSSERIAL_REQTRANS_WRITEBUF_LEN), BS_ECODE_BAD_VAL,
      "write_len=%zu > max_len=%zu",
      uWriteLen, (size_t)BSSERIAL_REQTRANS_WRITEBUF_LEN);

  BSCLIENT_TRY_EXPR(pClient, (uReadLen <= (size_t)BSSERIAL_RSPREAD_READBUF_LEN),
      BS_ECODE_BAD_VAL, "read_len=%zu > max_len=%zu",
      uReadLen, (size_t)BSSERIAL_RSPREAD_READBUF_LEN);

  // trace state
  bTrace = bsClientAttrGetTraceState(pClient, hndVConn);

  //
  // Set request message values.
  //
  memcpy(msgReq.m_writebuf.u.m_buf, wbuf, uWriteLen);
  msgReq.m_writebuf.m_count = uWriteLen;
  msgReq.m_readlen = (byte_t)uReadLen;

  //
  // Pack request.
  //
  n = BsSerialPackReqTrans(&msgReq, BSPROXY_BUF_BODY(buf), bTrace);

  // check packing return code
  BSCLIENT_TRY_NM_ECODE(pClient, n, "MsgId=%u", msgIdReq);

  //
  // Execute request-response server transaction.
  //
  n = bsClientTrans(pClient,  hndVConn,
                    msgIdReq, buf, (size_t)n,
                    msgIdRsp, buf, sizeof(buf));

  // check transaction return code
  BSCLIENT_TRY_ECODE(pClient, n, "MsgId=%u: Transaction failed.", msgIdReq);

  //
  // Unpack response.
  //
  n = BsSerialUnpackRspRead(buf, (size_t)n, &msgRsp, bTrace);

  // check unpack return code
  BSCLIENT_TRY_NM_ECODE(pClient, n, "MsgId=%u", msgIdRsp);

  //
  // Set return values from response.
  //
  if( uReadLen > msgRsp.m_readbuf.m_count )
  {
    uReadLen = msgRsp.m_readbuf.m_count;
  }
  memcpy(rbuf, msgRsp.m_readbuf.u.m_buf, uReadLen);

  return (int)uReadLen;
}
