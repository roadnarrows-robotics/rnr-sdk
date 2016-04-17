////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Module:    bsDyna
//
// Library:   libbsclient_dyna
//
// File:      bsDynaClient.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
 * $Rev: 3845 $
 *
 * \brief BotSense bsProxy client proxied Dynamixel device.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2012-2015.  RoadNarrows LLC.
 * (http://www.roadnarrows.com)\n
 * All Rights Reserved
 */
/*
 * @EulaBegin@
 * 
 * Unless otherwise stated explicitly, all materials contained are copyrighted
 * and may not be used without RoadNarrows LLC's written consent,
 * except as provided in these terms and conditions or in the copyright
 * notice (documents and software) or other proprietary notice provided with
 * the relevant materials.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY 
 * MEMBERS/EMPLOYEES/CONTRACTORS OF ROADNARROWS OR DISTRIBUTORS OF THIS SOFTWARE
 * BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
 * DOCUMENTATION, EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHORS AND  ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
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
#include "botsense/bsProxyMsgs.h"
#include "botsense/bsDyna.h"
#include "botsense/bsDynaMsgs.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief \h_botsense client application information.
 */
static BsClientAppInfo_T bsDynaAppInfo = 
{
  "libbsclient_dyna",
  "The proxied Dynamixel servo chain.",
  "1.1.0",
  "2015.01.09",
  "RoadNarrows LLC",
  "(C) 2012-2015 RoadNarrows LLC. All rights reserved.",
  bsDynaGetMsgName
};

// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

const char *bsDynaGetMsgName(BsClient_P   pClient,
                             BsVConnHnd_T hndVConn,
                             uint_t       uMsgId)
{
  const NMMsgDef_T  *pMsgDef;

  pMsgDef = BsDynaLookupMsgDef((BsDynaMsgId_T)uMsgId);

  return pMsgDef!=NULL? pMsgDef->m_sMsgName: "unknown";
}

int bsDynaOpen(BsClient_P  pClient,
               const char *sDevName,
               int         nBaudRate,
               bool_t      bInitTrace)
{
  static BsDynaMsgId_T    msgIdReq = BsDynaMsgIdReqOpenArgs;
  
  BsDynaReqOpenArgs_T     msgReq;                   // specific open args
  byte_t                  buf[BSPROXY_MSG_MAX_LEN]; // specific open args buffer
  bool_t                  bTrace;                   // do [not] trace messages
  int                     n;                        // num of bytes/return code

  // trace state
  bTrace = bsClientAttrGetTraceState(pClient, BSPROXY_VCONN_SERVER);
  
  //
  // Set specific open argument values.
  //
  msgReq.m_baudrate = (uint_t)nBaudRate;

  //
  // Pack specific open arguments (returns bytes packed \h_ge 0 on success).
  //
  n = BsDynaPackReqOpenArgs(&msgReq, buf, sizeof(buf), bTrace);

  // check packing return code
  BSCLIENT_TRY_NM_ECODE(pClient, n, "MsgId=%u", msgIdReq);


  //
  // Execute server transaction (returns handle \h_ge 0 on success).
  //
  n = bsServerReqOpenDev(pClient, sDevName, BS_DYNA_SERVER_MOD, buf, (size_t)n,
                         &bsDynaAppInfo, bInitTrace);

  // check transaction return code
  BSCLIENT_TRY_ECODE(pClient, n, "bsServerReqOpenDev(dev='%s') failed.",
      sDevName);

  // return handle
  return n;
}

int bsDynaClose(BsClient_P pClient, BsVConnHnd_T hndVConn)
{
  return bsServerReqCloseDev(pClient, hndVConn);
}

int bsDynaSetBaudRate(BsClient_P    pClient,
                      BsVConnHnd_T  hndVConn,
                      int           nBaudRate)
{
  static BsDynaMsgId_T  msgIdReq = BsDynaMsgIdReqSetBaudRate;
  static BsProxyMsgId_T msgIdRsp = BsProxyMsgIdRspOk;

  BsDynaReqSetBaudRate_T  msgReq;                   // response message 
  byte_t                  buf[BSPROXY_MSG_MAX_LEN]; // req/rsp buffer
  bool_t                  bTrace;                   // do [not] trace messages
  int                     n;                        // num of bytes/return code

  //
  // Parameter checks.
  //
  BSCLIENT_TRY_EXPR(pClient, BSCLIENT_HAS_VCONN(pClient, hndVConn),
      BS_ECODE_BAD_VAL, "VConn=%d", hndVConn);

  // trace state
  bTrace = bsClientAttrGetTraceState(pClient, hndVConn);

  //
  // Set request message values.
  //
  msgReq.m_baudrate = (uint_t)nBaudRate;

  //
  // Pack request.
  //
  n = BsDynaPackReqSetBaudRate(&msgReq, BSPROXY_BUF_BODY(buf), bTrace);

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

  LOGDIAG3("%s: SetBaudRate: %d", bsClientAttrGetName(pClient), nBaudRate);

  return BS_OK;
}

int bsDynaRead8(BsClient_P    pClient,
                BsVConnHnd_T  hndVConn,
                int           nServoId,
                uint_t        uAddr,
                byte_t       *pVal,
                uint_t       *pAlarms)
{
  static BsDynaMsgId_T  msgIdReq = BsDynaMsgIdReqRead8;
  static BsDynaMsgId_T  msgIdRsp = BsDynaMsgIdRspRead8;

  BsDynaReqRead8_T      msgReq;                   // response message 
  BsDynaRspRead8_T      msgRsp;                   // response message 
  byte_t                buf[BSPROXY_MSG_MAX_LEN]; // req/rsp buffer
  bool_t                bTrace;                   // do [not] trace messages
  int                   n;                        // num of bytes/return code

  //
  // Parameter checks.
  //
  BSCLIENT_TRY_EXPR(pClient, BSCLIENT_HAS_VCONN(pClient, hndVConn),
      BS_ECODE_BAD_VAL, "VConn=%d", hndVConn);

  // trace state
  bTrace = bsClientAttrGetTraceState(pClient, hndVConn);

  //
  // Set request message values.
  //
  msgReq.m_servo_id = (byte_t)nServoId;
  msgReq.m_addr     = (byte_t)uAddr;

  //
  // Pack request.
  //
  n = BsDynaPackReqRead8(&msgReq, BSPROXY_BUF_BODY(buf), bTrace);

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
  n = BsDynaUnpackRspRead8(buf, (size_t)n, &msgRsp, bTrace);

  // check unpack return code
  BSCLIENT_TRY_NM_ECODE(pClient, n, "MsgId=%u", msgIdRsp);

  //
  // Set return values from response.
  //
  *pAlarms  = (uint_t)msgRsp.m_alarms;
  *pVal     = msgRsp.m_val;

  LOGDIAG3("%s: Read8: 0x%02x --> 0x%02x.",
      bsClientAttrGetName(pClient), uAddr, *pVal);

  return BS_OK;
}

int bsDynaRead16(BsClient_P    pClient,
                 BsVConnHnd_T  hndVConn,
                 int           nServoId,
                 uint_t        uAddr,
                 ushort_t     *pVal,
                 uint_t       *pAlarms)
{
  static BsDynaMsgId_T  msgIdReq = BsDynaMsgIdReqRead16;
  static BsDynaMsgId_T  msgIdRsp = BsDynaMsgIdRspRead16;

  BsDynaReqRead16_T     msgReq;                   // response message 
  BsDynaRspRead16_T     msgRsp;                   // response message 
  byte_t                buf[BSPROXY_MSG_MAX_LEN]; // req/rsp buffer
  bool_t                bTrace;                   // do [not] trace messages
  int                   n;                        // num of bytes/return code

  //
  // Parameter checks.
  //
  BSCLIENT_TRY_EXPR(pClient, BSCLIENT_HAS_VCONN(pClient, hndVConn),
      BS_ECODE_BAD_VAL, "VConn=%d", hndVConn);

  // trace state
  bTrace = bsClientAttrGetTraceState(pClient, hndVConn);

  //
  // Set request message values.
  //
  msgReq.m_servo_id = (byte_t)nServoId;
  msgReq.m_addr     = (byte_t)uAddr;

  //
  // Pack request.
  //
  n = BsDynaPackReqRead16(&msgReq, BSPROXY_BUF_BODY(buf), bTrace);

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
  n = BsDynaUnpackRspRead16(buf, (size_t)n, &msgRsp, bTrace);

  // check unpack return code
  BSCLIENT_TRY_NM_ECODE(pClient, n, "MsgId=%u", msgIdRsp);

  //
  // Set return values from response.
  //
  *pAlarms  = (uint_t)msgRsp.m_alarms;
  *pVal     = msgRsp.m_val;

  LOGDIAG3("%s: Read16: 0x%02x --> 0x%04x.",
      bsClientAttrGetName(pClient), uAddr, *pVal);

  return BS_OK;
}

int bsDynaWrite8(BsClient_P    pClient,
                 BsVConnHnd_T  hndVConn,
                 int           nServoId,
                 uint_t        uAddr,
                 byte_t        byVal,
                 uint_t       *pAlarms)
{
  static BsDynaMsgId_T  msgIdReq = BsDynaMsgIdReqWrite8;
  static BsDynaMsgId_T  msgIdRsp = BsDynaMsgIdRspWrite8;

  BsDynaReqWrite8_T     msgReq;                   // response message 
  BsDynaRspWrite8_T     msgRsp;                   // response message 
  byte_t                buf[BSPROXY_MSG_MAX_LEN]; // req/rsp buffer
  bool_t                bTrace;                   // do [not] trace messages
  int                   n;                        // num of bytes/return code

  //
  // Parameter checks.
  //
  BSCLIENT_TRY_EXPR(pClient, BSCLIENT_HAS_VCONN(pClient, hndVConn),
      BS_ECODE_BAD_VAL, "VConn=%d", hndVConn);

  // trace state
  bTrace = bsClientAttrGetTraceState(pClient, hndVConn);

  //
  // Set request message values.
  //
  msgReq.m_servo_id = (byte_t)nServoId;
  msgReq.m_addr     = (byte_t)uAddr;
  msgReq.m_val      = byVal;

  //
  // Pack request.
  //
  n = BsDynaPackReqWrite8(&msgReq, BSPROXY_BUF_BODY(buf), bTrace);

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
  n = BsDynaUnpackRspWrite8(buf, (size_t)n, &msgRsp, bTrace);

  // check unpack return code
  BSCLIENT_TRY_NM_ECODE(pClient, n, "MsgId=%u", msgIdRsp);

  //
  // Set return values from response.
  //
  *pAlarms  = (uint_t)msgRsp.m_alarms;

  LOGDIAG3("%s: Write8: 0x%02x <-- 0x%02x.",
      bsClientAttrGetName(pClient), uAddr, byVal);

  return BS_OK;
}

int bsDynaWrite16(BsClient_P    pClient,
                  BsVConnHnd_T  hndVConn,
                  int           nServoId,
                  uint_t        uAddr,
                  ushort_t      huVal,
                  uint_t       *pAlarms)
{
  static BsDynaMsgId_T  msgIdReq = BsDynaMsgIdReqWrite16;
  static BsDynaMsgId_T  msgIdRsp = BsDynaMsgIdRspWrite16;

  BsDynaReqWrite16_T    msgReq;                   // response message 
  BsDynaRspWrite16_T    msgRsp;                   // response message 
  byte_t                buf[BSPROXY_MSG_MAX_LEN]; // req/rsp buffer
  bool_t                bTrace;                   // do [not] trace messages
  int                   n;                        // num of bytes/return code

  //
  // Parameter checks.
  //
  BSCLIENT_TRY_EXPR(pClient, BSCLIENT_HAS_VCONN(pClient, hndVConn),
      BS_ECODE_BAD_VAL, "VConn=%d", hndVConn);

  // trace state
  bTrace = bsClientAttrGetTraceState(pClient, hndVConn);

  //
  // Set request message values.
  //
  msgReq.m_servo_id = (byte_t)nServoId;
  msgReq.m_addr     = (byte_t)uAddr;
  msgReq.m_val      = huVal;

  //
  // Pack request.
  //
  n = BsDynaPackReqWrite16(&msgReq, BSPROXY_BUF_BODY(buf), bTrace);

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
  n = BsDynaUnpackRspWrite16(buf, (size_t)n, &msgRsp, bTrace);

  // check unpack return code
  BSCLIENT_TRY_NM_ECODE(pClient, n, "MsgId=%u", msgIdRsp);

  //
  // Set return values from response.
  //
  *pAlarms  = (uint_t)msgRsp.m_alarms;

  LOGDIAG3("%s: Write16: 0x%02x <-- 0x%04x.",
      bsClientAttrGetName(pClient), uAddr, huVal);

  return BS_OK;
}

int bsDynaSyncWrite(BsClient_P            pClient,
                    BsVConnHnd_T          hndVConn,
                    uint_t                uAddr,
                    uint_t                uDataSize,
                    DynaSyncWriteTuple_T  tuples[],
                    uint_t                uCount)
{
  static BsDynaMsgId_T  msgIdReq = BsDynaMsgIdReqSyncWrite;
  static BsProxyMsgId_T msgIdRsp = BsProxyMsgIdRspOk;

  BsDynaReqSyncWrite_T  msgReq;                   // response message 
  byte_t                buf[BSPROXY_MSG_MAX_LEN]; // req/rsp buffer
  bool_t                bTrace;                   // do [not] trace messages
  int                   n;                        // num of bytes/return code

  //
  // Parameter checks.
  //
  BSCLIENT_TRY_EXPR(pClient, BSCLIENT_HAS_VCONN(pClient, hndVConn),
      BS_ECODE_BAD_VAL, "VConn=%d", hndVConn);

  // trace state
  bTrace = bsClientAttrGetTraceState(pClient, hndVConn);

  //
  // Set request message values.
  //
  msgReq.m_addr       = (byte_t)uAddr;
  msgReq.m_data_size  = (byte_t)uDataSize;

  for(n=0; (n<(int)uCount) && (n<DYNA_ID_NUMOF); ++n)
  {
    msgReq.m_tuples.u.m_buf[n].m_servo_id = (byte_t)tuples[n].m_nServoId;
    msgReq.m_tuples.u.m_buf[n].m_val      = (ushort_t)tuples[n].m_uVal;
  }
  msgReq.m_tuples.m_count = (size_t)n;

  //
  // Pack request.
  //
  n = BsDynaPackReqSyncWrite(&msgReq, BSPROXY_BUF_BODY(buf), bTrace);

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

  LOGDIAG3("%s: SyncWrite: %u tuples to 0x%02x",
      bsClientAttrGetName(pClient), uCount, uAddr);

  return BS_OK;
}

int bsDynaPing(BsClient_P    pClient,
               BsVConnHnd_T  hndVConn,
               int           nServoId,
               bool_t       *pPong)
{
  static BsDynaMsgId_T  msgIdReq = BsDynaMsgIdReqPing;
  static BsDynaMsgId_T  msgIdRsp = BsDynaMsgIdRspPing;

  BsDynaReqPing_T       msgReq;                   // response message 
  BsDynaRspPing_T       msgRsp;                   // response message 
  byte_t                buf[BSPROXY_MSG_MAX_LEN]; // req/rsp buffer
  bool_t                bTrace;                   // do [not] trace messages
  int                   n;                        // num of bytes/return code

  //
  // Parameter checks.
  //
  BSCLIENT_TRY_EXPR(pClient, BSCLIENT_HAS_VCONN(pClient, hndVConn),
      BS_ECODE_BAD_VAL, "VConn=%d", hndVConn);

  // trace state
  bTrace = bsClientAttrGetTraceState(pClient, hndVConn);

  //
  // Set request message values.
  //
  msgReq.m_servo_id = (byte_t)nServoId;

  //
  // Pack request.
  //
  n = BsDynaPackReqPing(&msgReq, BSPROXY_BUF_BODY(buf), bTrace);

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
  n = BsDynaUnpackRspPing(buf, (size_t)n, &msgRsp, bTrace);

  // check unpack return code
  BSCLIENT_TRY_NM_ECODE(pClient, n, "MsgId=%u", msgIdRsp);

  *pPong = msgRsp.m_pong? true: false;

  return BS_OK;
}

int bsDynaReset(BsClient_P    pClient,
               BsVConnHnd_T  hndVConn,
               int           nServoId)
{
  static BsDynaMsgId_T  msgIdReq = BsDynaMsgIdReqReset;
  static BsProxyMsgId_T msgIdRsp = BsProxyMsgIdRspOk;

  BsDynaReqReset_T      msgReq;                   // response message 
  byte_t                buf[BSPROXY_MSG_MAX_LEN]; // req/rsp buffer
  bool_t                bTrace;                   // do [not] trace messages
  int                   n;                        // num of bytes/return code

  //
  // Parameter checks.
  //
  BSCLIENT_TRY_EXPR(pClient, BSCLIENT_HAS_VCONN(pClient, hndVConn),
      BS_ECODE_BAD_VAL, "VConn=%d", hndVConn);

  // trace state
  bTrace = bsClientAttrGetTraceState(pClient, hndVConn);

  //
  // Set request message values.
  //
  msgReq.m_servo_id = (byte_t)nServoId;

  //
  // Pack request.
  //
  n = BsDynaPackReqReset(&msgReq, BSPROXY_BUF_BODY(buf), bTrace);

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

  return BS_OK;
}

int bsDynaSetHalfDuplexCtl(BsClient_P    pClient,
                           BsVConnHnd_T  hndVConn,
                           int           nSignal)
{
  static BsDynaMsgId_T  msgIdReq = BsDynaMsgIdReqSetHalfDuplexCtl;
  static BsProxyMsgId_T msgIdRsp = BsProxyMsgIdRspOk;

  BsDynaReqSetHalfDuplexCtl_T msgReq;               // response message 
  byte_t                      buf[BSPROXY_MSG_MAX_LEN]; // req/rsp buffer
  bool_t                      bTrace;               // do [not] trace messages
  int                         n;                    // num of bytes/return code

  //
  // Parameter checks.
  //
  BSCLIENT_TRY_EXPR(pClient, BSCLIENT_HAS_VCONN(pClient, hndVConn),
      BS_ECODE_BAD_VAL, "VConn=%d", hndVConn);

  // trace state
  bTrace = bsClientAttrGetTraceState(pClient, hndVConn);

  //
  // Set request message values.
  //
  msgReq.m_signal = (uint_t)nSignal;

  //
  // Pack request.
  //
  n = BsDynaPackReqSetHalfDuplexCtl(&msgReq, BSPROXY_BUF_BODY(buf), bTrace);

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

  LOGDIAG3("%s: SetHalfDuplexCtl: %d", bsClientAttrGetName(pClient), nSignal);

  return BS_OK;
}
