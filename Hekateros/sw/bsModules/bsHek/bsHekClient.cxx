////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Module:    bsHek
// Library:   libbsclient_hek
// File:      bsHekClient.cxx
//
/*! \file
 *
 * $LastChangedDate: 2012-09-19 14:47:46 -0600 (Wed, 19 Sep 2012) $
 * $Rev: 2298 $
 *
 * \brief BotSense bsProxy client proxied Hekateros device.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2012-2017. RoadNarrows LLC.\n
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
#include "botsense/bsProxyMsgs.h"
#include "botsense/bsHek.h"
#include "botsense/bsHekMsgs.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief \h_botsense client application information.
 */
static BsClientAppInfo_T bsHekAppInfo = 
{
  "libbsclient_hek",
  "The proxied Hekateros servo chain.",
  "1.0.0",
  "2012.05.12",
  "RoadNarrows LLC",
  "(C) 2012 RoadNarrows LLC. All rights reserved.",
  bsHekGetMsgName
};

int bsHekCvtStateVec(BsClient_P           pClient,
                     byte_t               buf[],
                     int                  n,
                     DynaSpeedPosTuple_T  vecState[],
                     size_t               uNumServos,
                     bool_t               bTrace)
{
  static BsHekMsgId_T   msgIdRsp = BsHekMsgIdRspState;
  BsHekRspState_T       msgRsp;
  size_t                i;

  n = BsHekUnpackRspState(buf, (size_t)n, &msgRsp, bTrace);

  // check unpack return code
  BSCLIENT_TRY_NM_ECODE(pClient, n, "MsgId=%u", msgIdRsp);

  for(i=0; i<uNumServos && i<msgRsp.m_state.m_count; ++i)
  {
    vecState[i].m_nServoId  = (int)msgRsp.m_state.u.m_buf[i].m_servo_id;
    vecState[i].m_nSpeed    = (int)msgRsp.m_state.u.m_buf[i].m_goal_speed;
    vecState[i].m_nPos      = (int)msgRsp.m_state.u.m_buf[i].m_goal_pos;
  }

  LOGDIAG3("%s: Rsp: State: %u servos", bsClientAttrGetName(pClient), i);

  return (int)i;
}


// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Get the Hekateros message name.
 *
 * Each (virtual connection, message id) 2-tuple provides a unique server
 * mapping that can be used associate a name string to the message
 * (provided the id is valid and an application provides the information).
 *
 * \param pClient   \h_botsense client.
 * \param hndVConn  Virtual connection handle.
 * \param uMsgId    Message id.
 *
 * \return
 * Returns message name if it can be determined. Otherwise returns "unknown".
 */
const char *bsHekGetMsgName(BsClient_P   pClient,
                            BsVConnHnd_T hndVConn,
                            uint_t       uMsgId)
{
  const NMMsgDef_T  *pMsgDef;

  pMsgDef = BsHekLookupMsgDef((BsHekMsgId_T)uMsgId);

  return pMsgDef!=NULL? pMsgDef->m_sMsgName: "unknown";
}

/*!
 * \brief Request proxy server to establish a virtual connection to the
 * Hekateros servo chain using the USB2Hekateros serial dongle.
 *
 * \param pClient     \h_botsense client.
 * \param sDevName    Proxied USB serial device name (e.g. /dev/ttyUSB0).
 * \param nBaudRate   Baud rate.
 * \param bInitTrace  Initial message tracing enable(true)/disable(false) state.
 * 
 * \return
 * On success, the virtual connection handle is returned.\n
 * \copydoc doc_return_bs_ecode
 */
int bsHekOpen(BsClient_P  pClient,
              const char *sDevName,
              int         nBaudRate,
              bool_t      bInitTrace)
{
  static BsHekMsgId_T     msgIdReq = BsHekMsgIdReqOpenArgs;
  
  BsHekReqOpenArgs_T      msgReq;                   // specific open args
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
  n = BsHekPackReqOpenArgs(&msgReq, buf, sizeof(buf), bTrace);

  // check packing return code
  BSCLIENT_TRY_NM_ECODE(pClient, n, "MsgId=%u", msgIdReq);


  //
  // Execute server transaction (returns handle \h_ge 0 on success).
  //
  n = bsServerReqOpenDev(pClient, sDevName, BS_HEK_SERVER_MOD, buf, (size_t)n,
                         &bsHekAppInfo, bInitTrace);

  // check transaction return code
  BSCLIENT_TRY_ECODE(pClient, n, "bsServerReqOpenDev(dev='%s') failed.",
      sDevName);

  // return handle
  return n;
}

/*!
 * \brief Request proxy server to close client's proxied Hekateros servo chain
 * vitual connection.
 * 
 * \param pClient     \h_botsense client.
 * \param hndVConn    Handle to virtual connection to close.
 * 
 * \copydoc doc_return_bs_std
 */
int bsHekClose(BsClient_P pClient, BsVConnHnd_T hndVConn)
{
  return bsServerReqCloseDev(pClient, hndVConn);
}

/*!
 * \brief Proxied request to get the Hekateros version.
 * \param pClient             \h_botsense client.
 * \param hndVConn            Handle to virtual connection to close.
 * \param [out] bufVersion    Version buffer.
 * \param sizeBuf             Size of buffer.         
 *
 * \return doc_return_bs_std
 */
int bsHekGetVersion(BsClient_P    pClient,
                    BsVConnHnd_T  hndVConn,
                    char          bufVersion[],
                    size_t        sizeBuf)
{
  static BsHekMsgId_T   msgIdReq = BsHekMsgIdReqGetVersion;
  static BsHekMsgId_T   msgIdRsp = BsHekMsgIdRspGetVersion;

  BsHekRspGetVersion_T  msgRsp;                   // response message 
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

  //
  // Pack request.
  //

  // check packing return code
  BSCLIENT_TRY_NM_ECODE(pClient, n, "MsgId=%u", msgIdReq);

  //
  // Execute request-response transaction.
  //
  n = bsClientTrans(pClient,  hndVConn,
                    msgIdReq, buf, (size_t)0,
                    msgIdRsp, buf, sizeof(buf));

  // check transaction return code
  BSCLIENT_TRY_ECODE(pClient, n, "MsgId=%u: Transaction failed.", msgIdReq);

  //
  // Unpack response.
  //
  n = BsHekUnpackRspGetVersion(buf, (size_t)n, &msgRsp, bTrace);

  // check unpack return code
  BSCLIENT_TRY_NM_ECODE(pClient, n, "MsgId=%u", msgIdRsp);

  //
  // Set return values from response.
  //
  strncpy(bufVersion, msgRsp.m_version, sizeBuf);
  bufVersion[sizeBuf-1] = 0;

  LOGDIAG3("%s: Hekateros Version: %s.\n",
      bsClientAttrGetName(pClient), bufVersion);

  return BS_OK;
}

/*!
 * \brief Proxied request to move the given servos at speed to goal positions.
 *
 * \param pClient             \h_botsense client.
 * \param hndVConn            Handle to virtual connection to close.
 * \param nServoId            Servo id.
 * \param uAddr               Control table address.
 * \param [out] pVal          Value read.
 * \param [out] pAlarms       Current servo alarms, if any.
 *
 * \copydoc doc_return_bs_std
 */
int bsHekMoveAtSpeedTo(BsClient_P          pClient,
                       BsVConnHnd_T        hndVConn,
                       DynaSpeedPosTuple_T vecMove[],
                       size_t              uNumServos)
{
  static BsHekMsgId_T   msgIdReq = BsHekMsgIdReqMoveAtSpeedTo;
  static BsHekMsgId_T   msgIdRsp = BsHekMsgIdRspState;

  BsHekReqMoveAtSpeedTo_T msgReq;                   // response message 
  byte_t                  buf[BSPROXY_MSG_MAX_LEN]; // req/rsp buffer
  bool_t                  bTrace;                   // do [not] trace messages
  int                     n;                        // num of bytes/return code
  size_t                  i;

  //
  // Parameter checks.
  //
  BSCLIENT_TRY_EXPR(pClient, BSCLIENT_HAS_VCONN(pClient, hndVConn),
      BS_ECODE_BAD_VAL, "VConn=%d", hndVConn);

  if( uNumServos <= 0 )
  {
    return 0;
  }

  // trace state
  bTrace = bsClientAttrGetTraceState(pClient, hndVConn);

  //
  // Set request message values.
  //
  for(i=0; i<uNumServos; ++i)
  {
    msgReq.m_move.u.m_buf[i].m_servo_id   = (byte_t)vecMove[i].m_nServoId;
    msgReq.m_move.u.m_buf[i].m_goal_speed = (short)vecMove[i].m_nSpeed;
    msgReq.m_move.u.m_buf[i].m_goal_pos   = (short)vecMove[i].m_nPos;
  }
  msgReq.m_move.m_count = uNumServos;

  //
  // Pack request.
  //
  n = BsHekPackReqMoveAtSpeedTo(&msgReq, BSPROXY_BUF_BODY(buf), bTrace);

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
  // Unpack response and set return values from response.
  //
  return bsHekCvtStateVec(pClient, buf, n, vecMove, uNumServos, bTrace);

  return BS_OK;
}

/*!
 * \brief Proxied request to Hekateros servos states.
 *
 * \param pClient             \h_botsense client.
 * \param hndVConn            Handle to virtual connection to close.
 * \param nServoId            Servo id.
 * \param [out] vecState      Vector of servo states.
 * \param uNumServos          Maximum number of servo states to return.
 *
 * \copydoc doc_return_bs_std
 */
int bsHekGetState(BsClient_P          pClient,
                  BsVConnHnd_T        hndVConn,
                  DynaSpeedPosTuple_T vecState[],
                  size_t              uNumServos)
{
  static BsHekMsgId_T   msgIdReq = BsHekMsgIdReqGetState;
  static BsHekMsgId_T   msgIdRsp = BsHekMsgIdRspState;

  byte_t                buf[BSPROXY_MSG_MAX_LEN]; // req/rsp buffer
  bool_t                bTrace;                   // do [not] trace messages
  int                   n;                        // num of bytes/return code

  //
  // Parameter checks.
  //
  BSCLIENT_TRY_EXPR(pClient, BSCLIENT_HAS_VCONN(pClient, hndVConn),
      BS_ECODE_BAD_VAL, "VConn=%d", hndVConn);

  if( uNumServos <= 0 )
  {
    return 0;
  }

  // trace state
  bTrace = bsClientAttrGetTraceState(pClient, hndVConn);

  //
  // Set request message values.
  //

  //
  // Pack request.
  //

  //
  // Execute request-response transaction.
  //
  n = bsClientTrans(pClient,  hndVConn,
                    msgIdReq, buf, (size_t)0,
                    msgIdRsp, buf, sizeof(buf));

  // check transaction return code
  BSCLIENT_TRY_ECODE(pClient, n, "MsgId=%u: Transaction failed.", msgIdReq);

  //
  // Unpack response and set return values from response.
  //
  return bsHekCvtStateVec(pClient, buf, n, vecState, uNumServos, bTrace);
}

/*!
 * \brief Proxied request to Hekateros to freeze arm at current location.
 *
 * \param pClient             \h_botsense client.
 * \param hndVConn            Handle to virtual connection to close.
 * \param nServoId            Servo id.
 * \param [out] vecState      Vector of servo states.
 * \param uNumServos          Maximum number of servo states to return.
 *
 * \copydoc doc_return_bs_std
 */
int bsHekFreeze(BsClient_P          pClient,
                BsVConnHnd_T        hndVConn,
                DynaSpeedPosTuple_T vecState[],
                size_t              uNumServos)
{
  static BsHekMsgId_T   msgIdReq = BsHekMsgIdReqFreeze;
  static BsHekMsgId_T   msgIdRsp = BsHekMsgIdRspState;

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

  //
  // Pack request.
  //

  //
  // Execute request-response transaction.
  //
  n = bsClientTrans(pClient,  hndVConn,
                    msgIdReq, buf, (size_t)0,
                    msgIdRsp, buf, sizeof(buf));

  // check transaction return code
  BSCLIENT_TRY_ECODE(pClient, n, "MsgId=%u: Transaction failed.", msgIdReq);

  //
  // Unpack response and set return values from response.
  //
  return bsHekCvtStateVec(pClient, buf, n, vecState, uNumServos, bTrace);
}

/*!
 * \brief Proxied request to Hekateros make an emergency stop.
 *
 * \param pClient             \h_botsense client.
 * \param hndVConn            Handle to virtual connection to close.
 * \param nServoId            Servo id.
 * \param [out] vecState      Vector of servo states.
 * \param uNumServos          Maximum number of servo states to return.
 *
 * \copydoc doc_return_bs_std
 */
int bsHekEStop(BsClient_P          pClient,
               BsVConnHnd_T        hndVConn,
               DynaSpeedPosTuple_T vecState[],
               size_t              uNumServos)
{
  static BsHekMsgId_T   msgIdReq = BsHekMsgIdReqEStop;
  static BsHekMsgId_T   msgIdRsp = BsHekMsgIdRspState;

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

  //
  // Pack request.
  //

  //
  // Execute request-response transaction.
  //
  n = bsClientTrans(pClient,  hndVConn,
                    msgIdReq, buf, (size_t)0,
                    msgIdRsp, buf, sizeof(buf));

  // check transaction return code
  BSCLIENT_TRY_ECODE(pClient, n, "MsgId=%u: Transaction failed.", msgIdReq);

  //
  // Unpack response and set return values from response.
  //
  return bsHekCvtStateVec(pClient, buf, n, vecState, uNumServos, bTrace);
}

/*!
 * \brief Proxied request to Hekateros to calibrate arm.
 *
 * \param pClient             \h_botsense client.
 * \param hndVConn            Handle to virtual connection to close.
 * \param nServoId            Servo id.
 * \param [out] vecState      Vector of servo states.
 * \param uNumServos          Maximum number of servo states to return.
 *
 * \copydoc doc_return_bs_std
 */
int bsHekCalibrate(BsClient_P          pClient,
                   BsVConnHnd_T        hndVConn,
                   DynaSpeedPosTuple_T vecState[],
                   size_t              uNumServos)
{
  static BsHekMsgId_T   msgIdReq = BsHekMsgIdReqCalibrate;
  static BsHekMsgId_T   msgIdRsp = BsHekMsgIdRspState;

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

  //
  // Pack request.
  //

  //
  // Execute request-response transaction.
  //
  n = bsClientTrans(pClient,  hndVConn,
                    msgIdReq, buf, (size_t)0,
                    msgIdRsp, buf, sizeof(buf));

  // check transaction return code
  BSCLIENT_TRY_ECODE(pClient, n, "MsgId=%u: Transaction failed.", msgIdReq);

  //
  // Unpack response and set return values from response.
  //
  return bsHekCvtStateVec(pClient, buf, n, vecState, uNumServos, bTrace);
}

/*!
 * \brief Proxied request to get Hekateros servos health.
 *
 * \param pClient             \h_botsense client.
 * \param hndVConn            Handle to virtual connection to close.
 * \param vecServoIds         Vector of servo ids.
 * \param [out] vecHealth     Vector of servo health.
 * \param uNumServos          Number of servos.
 *
 * \copydoc doc_return_bs_std
 */
int bsHekGetHealth(BsClient_P          pClient,
                   BsVConnHnd_T        hndVConn,
                   int                 vecServoIds[],
                   DynaHealthTuple_T   vecHealth[],
                   size_t              uNumServos)
{
  static BsHekMsgId_T   msgIdReq = BsHekMsgIdReqGetHealth;
  static BsHekMsgId_T   msgIdRsp = BsHekMsgIdRspGetHealth;

  BsHekReqGetHealth_T     msgReq;                   // request message 
  BsHekRspGetHealth_T     msgRsp;                   // response message 
  byte_t                  buf[BSPROXY_MSG_MAX_LEN]; // req/rsp buffer
  bool_t                  bTrace;                   // do [not] trace messages
  int                     i;
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
  for(i=0; i<uNumServos; ++i)
  {
    msgReq.m_servo_id.u.m_buf[i] = (byte_t)vecServoIds[i];
  }
  msgReq.m_servo_id.m_count = uNumServos;

  //
  // Pack request.
  //
  n = BsHekPackReqGetHealth(&msgReq, BSPROXY_BUF_BODY(buf), bTrace);

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
  n = BsHekUnpackRspGetHealth(buf, (size_t)n, &msgRsp, bTrace);

  // check unpack return code
  BSCLIENT_TRY_NM_ECODE(pClient, n, "MsgId=%u", msgIdRsp);

  //
  // Set return values from response.
  //
  for(i=0; i<uNumServos && i<msgRsp.m_health.m_count; ++i)
  {
    vecHealth[i].m_nServoId = (int)msgRsp.m_health.u.m_buf[i].m_servo_id;
    vecHealth[i].m_uAlarms  = (uint_t)msgRsp.m_health.u.m_buf[i].m_alarms;
    vecHealth[i].m_nLoad    = (int)msgRsp.m_health.u.m_buf[i].m_load;
    vecHealth[i].m_uVolts   = (uint_t)msgRsp.m_health.u.m_buf[i].m_volts;
    vecHealth[i].m_uTemp    = (uint_t)msgRsp.m_health.u.m_buf[i].m_temp;
  }

  LOGDIAG3("%s: Rsp: Health: %u servos", bsClientAttrGetName(pClient), i);

  return i;
}

/*!
 * \brief Proxied request to get Hekateros servos health.
 *
 * \param pClient             \h_botsense client.
 * \param hndVConn            Handle to virtual connection to close.
 * \param vecServoIds         Vector of servo ids.
 * \param uNumServos          Number of servos.
 *
 * \copydoc doc_return_bs_std
 */
int bsHekClearAlarms(BsClient_P          pClient,
                     BsVConnHnd_T        hndVConn,
                     int                 vecServoIds[],
                     size_t              uNumServos)
{
  static BsHekMsgId_T   msgIdReq = BsHekMsgIdReqClearAlarms;
  static BsProxyMsgId_T msgIdRsp = BsProxyMsgIdRspOk;

  BsHekReqClearAlarms_T   msgReq;                   // request message 
  byte_t                  buf[BSPROXY_MSG_MAX_LEN]; // req/rsp buffer
  bool_t                  bTrace;                   // do [not] trace messages
  int                     i;
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
  for(i=0; i<uNumServos; ++i)
  {
    msgReq.m_servo_id.u.m_buf[i]   = (byte_t)vecServoIds[i];
  }
  msgReq.m_servo_id.m_count = uNumServos;

  //
  // Pack request.
  //
  n = BsHekPackReqClearAlarms(&msgReq, BSPROXY_BUF_BODY(buf), bTrace);

  //
  // Execute request-response transaction.
  //
  n = bsClientTrans(pClient,  hndVConn,
                    msgIdReq, buf, (size_t)n,
                    msgIdRsp, buf, sizeof(buf));

  // check transaction return code
  BSCLIENT_TRY_ECODE(pClient, n, "MsgId=%u: Transaction failed.", msgIdReq);

  LOGDIAG3("%s: Alarms cleared: %u servos", bsClientAttrGetName(pClient), i);

  return BS_OK;
}
