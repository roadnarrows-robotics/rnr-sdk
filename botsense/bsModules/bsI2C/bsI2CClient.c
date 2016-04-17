////////////////////////////////////////////////////////////////////////////////
//
// Package:   BotSense
//
// Module:    bsI2C
// Library:   libbsclient_i2c
// File:      bsI2CClient.c
//
/*! \file
 *
 * $LastChangedDate: 2010-08-20 11:36:38 -0600 (Fri, 20 Aug 2010) $
 * $Rev: 568 $
 *
 * \brief \h_botsense bsProxy client proxied \h_i2c bus device library.
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
#include "rnr/i2c.h"

#include "botsense/BotSense.h"
#include "botsense/libBotSense.h"
#include "botsense/bsI2C.h"
#include "botsense/bsI2CMsgs.h"

// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief \h_botsense client application information.
 */
static BsClientAppInfo_T bsI2CAppInfo = 
{
  .app_name     = "libbsclient_i2c",
  .brief        = "Raw I2C Bus proxied device.",
  .version      = "2.0.0",
  .date         = "2010.07.20",
  .maintainer   = "RaodNarrows LLC",
  .license      = "(C) 2010 RoadNarrows LLC. All rights reserved.",

  .fnGetMsgName = bsI2CGetMsgName
};

// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Get the \h_i2c message name.
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
const char *bsI2CGetMsgName(BsClient_P   pClient,
                               BsVConnHnd_T hndVConn,
                               uint_t       uMsgId)
{
  const NMMsgDef_T  *pMsgDef;

  pMsgDef = BsI2CLookupMsgDef((BsI2CMsgId_T)uMsgId);

  return pMsgDef!=NULL? pMsgDef->m_sMsgName: "unknown";
}

/*!
 * \brief Request proxy server to establish a virtual connection to an \h_i2c
 * bus device.
 *
 * \param pClient     \h_botsense client.
 * \param sDevName    Proxied \h_i2c bus device name (e.g. /dev/i2c-0).
 * \param bInitTrace  Initial message tracing enable(true)/disable(false) state.
 * 
 * \return
 * On success, the virtual connection handle is returned.\n
 * \copydoc doc_return_ecode
 */
int bsI2CReqOpen(BsClient_P  pClient,
                 const char *sDevName,
                 bool_t      bInitTrace)
{
  int     hnd;                      // vconn handle / return code

  //
  // Execute server transaction (returns handle \h_ge 0 on success).
  //
  hnd = bsServerReqOpenDev(pClient, sDevName, BS_I2C_SERVER_MOD,
                            NULL, (size_t)0, &bsI2CAppInfo, bInitTrace);

  // check transaction return code
  BSCLIENT_TRY_ECODE(pClient, hnd, "bsServerReqOpenDev(dev='%s') failed.",
      sDevName);

  // return handle
  return hnd;
}

/*!
 * \brief Request proxy server to close client's proxied \h_i2c device vitual
 * connection.
 * 
 * \param pClient     \h_botsense client.
 * \param hndVConn    Handle to virtual connection to close.
 * 
 * \copydoc doc_return_std
 */
int bsI2CReqClose(BsClient_P pClient, BsVConnHnd_T hndVConn)
{
  return bsServerReqCloseDev(pClient, hndVConn);
}

/*!
 * \brief Proxied request to read from a device attached to the \h_i2c bus.
 *
 * \param pClient     \h_botsense client.
 * \param hndVConn    Handle to virtual connection to close.
 * \param i2cAddr     Address of attached device on the \h_i2c bus.
 * \param uReadLen    Number of bytes to read.
 * \param [out] rbuf  Read buffer.
 *
 * \return
 * Returns number of bytes read on success.\n
 * \copydoc doc_return_ecode
 */
int bsI2CReqRead(BsClient_P   pClient,
                 BsVConnHnd_T hndVConn,
                 i2c_addr_t   i2cAddr,
                 size_t       uReadLen,
                 byte_t       rbuf[])
{
  static BsI2CMsgId_T msgIdReq = BsI2CMsgIdReqRead;
  static BsI2CMsgId_T msgIdRsp = BsI2CMsgIdRspRead;

  BsI2CReqRead_T    msgReq;                   // request message 
  BsI2CRspRead_T    msgRsp;                   // response message 
  byte_t            buf[BSPROXY_MSG_MAX_LEN]; // req/rsp buffer
  bool_t            bTrace;                   // do [not] trace messages
  int               n;                        // number of bytes/return code

  //
  // Parameter checks.
  //
  BSCLIENT_TRY_EXPR(pClient, BSCLIENT_HAS_VCONN(pClient, hndVConn),
      BS_ECODE_BAD_VAL, "VConn=%d", hndVConn);

  BSCLIENT_TRY_EXPR(pClient, (uReadLen <= (size_t)BSI2C_RSPREAD_READBUF_LEN),
      BS_ECODE_BAD_VAL, "read_len=%zu > max_len=%zu",
      uReadLen, (size_t)BSI2C_RSPREAD_READBUF_LEN);

  // trace state
  bTrace = bsClientAttrGetTraceState(pClient, hndVConn);

  //
  // Set request message values.
  //
  msgReq.m_addr     = (ushort_t)i2cAddr;
  msgReq.m_readlen  = (byte_t)uReadLen;

  //
  // Pack request.
  //
  n = BsI2CPackReqRead(&msgReq, BSPROXY_BUF_BODY(buf), bTrace);

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
  n = BsI2CUnpackRspRead(buf, (size_t)n, &msgRsp, bTrace);

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
 * \brief Proxied request to write to a device attached to the \h_i2c bus.
 *
 * \param pClient     \h_botsense client.
 * \param hndVConn    Handle to virtual connection to close.
 * \param i2cAddr     Address of attached device on the \h_i2c bus.
 * \param [in] wbuf   Write buffer.
 * \param uWriteLen   Number of bytes to write.
 *
 * \return
 * Returns number of bytes written on success.\n
 * \copydoc doc_return_ecode
 */
int bsI2CReqWrite(BsClient_P   pClient,
                  BsVConnHnd_T hndVConn,
                  i2c_addr_t   i2cAddr,
                  byte_t       wbuf[],
                  size_t       uWriteLen)
{
  static BsI2CMsgId_T msgIdReq = BsI2CMsgIdReqWrite;
  static BsI2CMsgId_T msgIdRsp = BsI2CMsgIdRspWrite;

  BsI2CReqWrite_T   msgReq;                   // request message 
  BsI2CRspWrite_T   msgRsp;                   // response message 
  byte_t            buf[BSPROXY_MSG_MAX_LEN]; // req/rsp buffer
  bool_t            bTrace;                   // do [not] trace messages
  int               n;                        // number of bytes/return code

  //
  // Parameter checks.
  //
  BSCLIENT_TRY_EXPR(pClient, BSCLIENT_HAS_VCONN(pClient, hndVConn),
      BS_ECODE_BAD_VAL, "VConn=%d", hndVConn);

  BSCLIENT_TRY_EXPR(pClient,
      (uWriteLen <= (size_t)BSI2C_REQWRITE_WRITEBUF_LEN), BS_ECODE_BAD_VAL,
      "write_len=%zu > max_len=%zu",
      uWriteLen, (size_t)BSI2C_REQWRITE_WRITEBUF_LEN);

  // trace state
  bTrace = bsClientAttrGetTraceState(pClient, hndVConn);

  //
  // Set request message values.
  //
  msgReq.m_addr = (ushort_t)i2cAddr;
  memcpy(msgReq.m_writebuf.u.m_buf, wbuf, uWriteLen);
  msgReq.m_writebuf.m_count = uWriteLen;

  //
  // Pack request.
  //
  n = BsI2CPackReqWrite(&msgReq, BSPROXY_BUF_BODY(buf), bTrace);

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
  n = BsI2CUnpackRspWrite(buf, (size_t)n, &msgRsp, bTrace);

  // check unpack return code
  BSCLIENT_TRY_NM_ECODE(pClient, n, "MsgId=%u", msgIdRsp);

  //
  // Set return values from response.
  //
  return (int)msgRsp.m_byteswritten;
}

/*!
 * \brief Proxied request to execute a write-read transaction to a device
 * attached to the \h_i2c bus.
 *
 * \param pClient       \h_botsense client.
 * \param hndVConn      Handle to virtual connection to close.
 * \param i2cAddr       Address of attached device on the \h_i2c bus.
 * \param [in] wbuf     Write buffer.
 * \param uWriteLen     Number of bytes to write.
 * \param uReadLen      Number of bytes to read.
 * \param [out] rbuf    Read buffer.
 *
 * \return
 * Returns number of bytes read on success.\n
 * \copydoc doc_return_ecode
 */
int bsI2CReqTrans(BsClient_P   pClient,
                  BsVConnHnd_T hndVConn,
                  i2c_addr_t   i2cAddr,
                  byte_t       wbuf[],
                  size_t       uWriteLen,
                  size_t       uReadLen,
                  byte_t       rbuf[])
{
  static BsI2CMsgId_T msgIdReq = BsI2CMsgIdReqTrans;
  static BsI2CMsgId_T msgIdRsp = BsI2CMsgIdRspRead;

  BsI2CReqTrans_T   msgReq;                   // request message 
  BsI2CRspRead_T    msgRsp;                   // response message 
  byte_t            buf[BSPROXY_MSG_MAX_LEN]; // req/rsp buffer
  bool_t            bTrace;                   // do [not] trace messages
  int               n;                        // number of bytes/return code

  //
  // Parameter checks.
  //
  BSCLIENT_TRY_EXPR(pClient, BSCLIENT_HAS_VCONN(pClient, hndVConn),
      BS_ECODE_BAD_VAL, "VConn=%d", hndVConn);

  BSCLIENT_TRY_EXPR(pClient,
      (uWriteLen <= (size_t)BSI2C_REQTRANS_WRITEBUF_LEN), BS_ECODE_BAD_VAL,
      "write_len=%zu > max_len=%zu",
      uWriteLen, (size_t)BSI2C_REQTRANS_WRITEBUF_LEN);

  BSCLIENT_TRY_EXPR(pClient, (uReadLen <= (size_t)BSI2C_RSPREAD_READBUF_LEN),
      BS_ECODE_BAD_VAL, "read_len=%zu > max_len=%zu",
      uReadLen, (size_t)BSI2C_RSPREAD_READBUF_LEN);

  // trace state
  bTrace = bsClientAttrGetTraceState(pClient, hndVConn);

  //
  // Set request message values.
  //
  msgReq.m_addr = (ushort_t)i2cAddr;
  memcpy(msgReq.m_writebuf.u.m_buf, wbuf, uWriteLen);
  msgReq.m_writebuf.m_count = uWriteLen;
  msgReq.m_readlen = (byte_t)uReadLen;

  //
  // Pack request.
  //
  n = BsI2CPackReqTrans(&msgReq, BSPROXY_BUF_BODY(buf), bTrace);

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
  n = BsI2CUnpackRspRead(buf, (size_t)n, &msgRsp, bTrace);

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
 * \brief Proxied request to scan the \h_i2c bus for all attached devices.
 *
 * \param pClient       \h_botsense client.
 * \param hndVConn      Handle to virtual connection to close.
 * \param [out] bufScan Scan buffer.
 * \param sizeScanBuf   Scan buffer size (maximum number of address elements).
 *
 * \return
 * Returns number of devices discovered \h_ge 0 from scan on success.\n
 * \copydoc doc_return_ecode
 */
int bsI2CReqScan(BsClient_P   pClient,
                 BsVConnHnd_T hndVConn,
                 i2c_addr_t   bufScan[],
                 size_t       sizeScanBuf)
{
  static BsI2CMsgId_T msgIdReq = BsI2CMsgIdReqScan;
  static BsI2CMsgId_T msgIdRsp = BsI2CMsgIdRspScan;

  BsI2CRspScan_T    msgRsp;                   // response message 
  byte_t            buf[BSPROXY_MSG_MAX_LEN]; // req/rsp buffer
  bool_t            bTrace;                   // do [not] trace messages
  int               i;                        // working index
  int               n;                        // number of bytes/return code

  //
  // Parameter checks.
  //
  BSCLIENT_TRY_EXPR(pClient, BSCLIENT_HAS_VCONN(pClient, hndVConn),
      BS_ECODE_BAD_VAL, "VConn=%d", hndVConn);

  // trace state
  bTrace = bsClientAttrGetTraceState(pClient, hndVConn);

  //
  // Execute request-response transaction.
  //
  n = bsClientTrans(pClient,  hndVConn,
                    msgIdReq, NULL, (size_t)0,
                    msgIdRsp, buf, sizeof(buf));

  // check transaction return code
  BSCLIENT_TRY_ECODE(pClient, n, "MsgId=%u: Transaction failed.", msgIdReq);

  //
  // Unpack response.
  //
  n = BsI2CUnpackRspScan(buf, (size_t)n, &msgRsp, bTrace);

  // check unpack return code
  BSCLIENT_TRY_NM_ECODE(pClient, n, "MsgId=%u", msgIdRsp);

  //
  // Set return values from response.
  //
  n = sizeScanBuf < (size_t)msgRsp.m_scan.m_count?
                        (int)sizeScanBuf: (int)msgRsp.m_scan.m_count;

  for(i=0; i<n; ++i)
  {
    bufScan[i] = (i2c_addr_t)msgRsp.m_scan.u.m_buf[i];
  }

  return n;
}
