////////////////////////////////////////////////////////////////////////////////
//
// Package:   mousebot
//
// Module:    bsMouseBot
// Library:   libbsclient_mousebot
// File:      bsMouseBotClient.c
//
/*! \file
 *
 * $LastChangedDate: 2012-04-25 18:46:22 -0600 (Wed, 25 Apr 2012) $
 * $Rev: 1896 $
 *
 * \brief \h_botsense bsProxy client proxied MouseBot robot device.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 * \author Rob Shiely 	  (rob@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
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
#include "botsense/bsProxyMsgs.h"
#include "botsense/bsMouseBot.h"
#include "botsense/bsMouseBotMsgs.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief \h_botsense client application information.
 */
static BsClientAppInfo_T bsMouseBotAppInfo = 
{
  .app_name     = "libbsclient_mousebot",
  .brief        = "The MouseBot proxied robot.",
  .version      = "1.0.0",
  .date         = "2011.01.10",
  .maintainer   = "RaodNarrows LLC",
  .license      = "(C) 2011 RoadNarrows LLC. All rights reserved.",

  .fnGetMsgName = bsMouseBotGetMsgName
};

// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Get the MouseBot message name.
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
const char *bsMouseBotGetMsgName(BsClient_P   pClient,
                           BsVConnHnd_T hndVConn,
                           uint_t       uMsgId)
{
  const NMMsgDef_T  *pMsgDef;

  pMsgDef = BsMouseBotLookupMsgDef((BsMouseBotMsgId_T)uMsgId);

  return pMsgDef!=NULL? pMsgDef->m_sMsgName: "unknown";
}

/*!
 * \brief Request proxy server to establish a virtual connection to the
 * MouseBot robot.
 *
 * \param pClient     \h_botsense client.
 * \param bInitTrace  Initial message tracing enable(true)/disable(false) state.
 * 
 * \return
 * On success, the virtual connection handle is returned.\n
 * \copydoc doc_return_bs_ecode
 */
int bsMouseBotReqOpen(BsClient_P pClient, 
                  const char* dev1, 
                  const char* dev2,
                  bool_t server, 
                  bool_t bInitTrace)
{
  int     hnd;                      // vconn handle / return code
  size_t uSize;

  //
  // Execute server transaction (returns handle \h_ge 0 on success).
  //
  byte_t buf[BSPROXY_MSG_MAX_LEN];
  BsMouseBotReqOpenParams_T openParams;
  

  strcpy( openParams.m_FrontAddr, dev1);
  strcpy( openParams.m_RearAddr, dev2);
  openParams.m_server = server;

  uSize = BsMouseBotPackReqOpenParams( &openParams,
                                      buf,
                                      sizeof(buf),
                                      bInitTrace );

  hnd = bsServerReqOpenDev(pClient,BS_MOUSEBOT_DEV_NAME, BS_MOUSEBOT_SERVER_MOD,
                          buf, uSize, &bsMouseBotAppInfo, bInitTrace);

  // check transaction return code
  BSCLIENT_TRY_ECODE(pClient, hnd, "bsServerReqOpenDev(dev='%s') failed.",
                      BS_MOUSEBOT_DEV_NAME);

  // return handle
  return hnd;
}

/*!
 * \brief Request proxy server to close client's proxied MouseBot robot
 * vitual connection.
 * 
 * \param pClient     \h_botsense client.
 * \param hndVConn    Handle to virtual connection to close.
 * 
 * \copydoc doc_return_bs_std
 */
int bsMouseBotReqClose(BsClient_P pClient, BsVConnHnd_T hndVConn)
{
  return bsServerReqCloseDev(pClient, hndVConn);
}

/*!
 * \brief Proxied request to set robot's motor speeds.
 *
 * \param pClient     \h_botsense client.
 * \param hndVConn    Handle to virtual connection to close.
 * \param nSpeedLeft  Left motor speed in current units.
 * \param nSpeedRight Right motor speed in current units.
 *
 * \copydoc doc_return_bs_std
 */
int bsMouseBotReqSetMotorSpeeds(BsClient_P   pClient,
                          BsVConnHnd_T hndVConn,
                          int          nSpeedLeft,
                          int          nSpeedRight)
{
  static BsMouseBotMsgId_T    msgIdReq = BsMouseBotMsgIdReqSetMotorSpeeds;
  static BsProxyMsgId_T msgIdRsp = BsProxyMsgIdRspOk;

  BsMouseBotReqSetMotorSpeeds_T msgReq;                   // request message 
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
  msgReq.m_speedleft  = nSpeedLeft;
  msgReq.m_speedright = nSpeedRight;

  //
  // Pack request.
  //
  n = BsMouseBotPackReqSetMotorSpeeds(&msgReq, BSPROXY_BUF_BODY(buf), bTrace);

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
  // Set return values from response.
  //
  LOGDIAG3("%s: SetSpeeds: (%d,%d).", bsClientAttrGetName(pClient),
                                nSpeedLeft, nSpeedRight);

  return BS_OK;
}


/*!
 * \brief Proxied request to alter robots brake rates.
 *
 * \param pClient     \h_botsense client.
 * \param hndVConn    Handle to virtual connection to close.
 * \param nbrakeFL    Change front left wheel brake rate.
 * \param nbrakeFR    Change front right wheel brake rate.
 * \param nbrakeRL    Change rear left wheel brake rate.
 * \param nbrakeRR    Change rear right wheel brake rate.

 *
 * \copydoc doc_return_bs_std
 */
int bsMouseBotReqAlterBrake(BsClient_P   pClient,
                          BsVConnHnd_T hndVConn,
                          int          nbrakeFL,
                          int          nbrakeFR,
                          int          nbrakeRL,
                          int          nbrakeRR)
{
  static BsMouseBotMsgId_T    msgIdReq = BsMouseBotMsgIdReqAlterBrake;
  static BsProxyMsgId_T   msgIdRsp = BsProxyMsgIdRspOk;

  BsMouseBotReqAlterBrake_T   msgReq;                   // request message 
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
  msgReq.m_brakeFL  = nbrakeFL;
  msgReq.m_brakeFR  = nbrakeFR;
  msgReq.m_brakeRL  = nbrakeRL;
  msgReq.m_brakeRR  = nbrakeRR;

  //
  // Pack request.
  //
  n = BsMouseBotPackReqAlterBrake(&msgReq, BSPROXY_BUF_BODY(buf), bTrace);

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
  // Set return values from response.
  //
  LOGDIAG3("%s: SetBrake: (%d,%d,%d,%d).", bsClientAttrGetName(pClient),
                                nbrakeFL, nbrakeFR, nbrakeRL, nbrakeRR);

  return BS_OK;
}

/*!
 * \brief Proxied request to alter robots slew rates.
 *
 * \param pClient     \h_botsense client.
 * \param hndVConn    Handle to virtual connection to close.
 * \param nslewFL    Change front left wheel brake rate.
 * \param nslewFR    Change front right wheel brake rate.
 * \param nslewRL    Change rear left wheel brake rate.
 * \param nslewRR    Change rear right wheel brake rate.

 *
 * \copydoc doc_return_bs_std
 */
int bsMouseBotReqAlterSlew(BsClient_P   pClient,
                          BsVConnHnd_T hndVConn,
                          int          nslewFL,
                          int          nslewFR,
                          int          nslewRL,
                          int          nslewRR)
{
  static BsMouseBotMsgId_T    msgIdReq = BsMouseBotMsgIdReqAlterSlew;
  static BsProxyMsgId_T   msgIdRsp = BsProxyMsgIdRspOk;

  BsMouseBotReqAlterSlew_T    msgReq;                   // request message 
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
  msgReq.m_slewFL  = nslewFL;
  msgReq.m_slewFR  = nslewFR;
  msgReq.m_slewRL  = nslewRL;
  msgReq.m_slewRR  = nslewRR;

  //
  // Pack request.
  //
  n = BsMouseBotPackReqAlterSlew(&msgReq, BSPROXY_BUF_BODY(buf), bTrace);

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
  // Set return values from response.
  //
  LOGDIAG3("%s: SetSlew: (%d,%d,%d,%d).", bsClientAttrGetName(pClient),
                                nslewFL, nslewFR, nslewRL, nslewRR);

  return BS_OK;
}


//DHP - to add later
#if 0
/*!
 * \brief Proxied request to get the current motor speeds.
 *
 * \param pClient             \h_botsense client.
 * \param hndVConn            Handle to virtual connection to close.
 * \param [out] pnSpeedLeft   Left motor speed in the current units.
 * \param [out] pnSpeedRight  Right motor speed in the current units.
 *
 * \copydoc doc_return_bs_std
 */
int bsMouseBotReqGetMotorSpeeds(BsClient_P     pClient,
                          BsVConnHnd_T   hndVConn,
                          int           *pnSpeedLeft,
                          int           *pnSpeedRight)
{
  static BsMouseBotMsgId_T    msgIdReq = BsMouseBotMsgIdReqGetMotorSpeeds;
  static BsMouseBotMsgId_T    msgIdRsp = BsMouseBotMsgIdRspGetMotorSpeeds;

  BsMouseBotRspGetMotorSpeeds_T msgRsp;                   // response message 
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
  // Execute request-response transaction.
  //
  n = bsClientTrans(pClient,  hndVConn,
                    msgIdReq, buf, 0,
                    msgIdRsp, buf, sizeof(buf));

  // check transaction return code
  BSCLIENT_TRY_ECODE(pClient, n, "MsgId=%u: Transaction failed.", msgIdReq);

  //
  // Unpack response.
  //
  n = BsMouseBotUnpackRspGetMotorSpeeds(buf, (size_t)n, &msgRsp, bTrace);

  // check unpack return code
  BSCLIENT_TRY_NM_ECODE(pClient, n, "MsgId=%u", msgIdRsp);

  //
  // Set return values from response.
  //
  *pnSpeedLeft  = msgRsp.m_speedleft;
  *pnSpeedRight = msgRsp.m_speedright;

  LOGDIAG3("%s: GetSpeeds: (%d,%d).", bsClientAttrGetName(pClient),
                                *pnSpeedLeft, *pnSpeedRight);

  return BS_OK;
}
#endif

/*!
 * \brief Proxied request to stop robot's motors.
 *
 * \param pClient     \h_botsense client.
 * \param hndVConn    Handle to virtual connection to close.
 *
 * \copydoc doc_return_bs_std
 */
int bsMouseBotReqStop(BsClient_P pClient, BsVConnHnd_T hndVConn)
{
  static BsMouseBotMsgId_T    msgIdReq = BsMouseBotMsgIdReqStop;
  static BsProxyMsgId_T msgIdRsp = BsProxyMsgIdRspOk;

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
  // Execute request-response transaction.
  //
  n = bsClientTrans(pClient,  hndVConn,
                    msgIdReq, buf, 0,
                    msgIdRsp, buf, sizeof(buf));

  // check transaction return code
  BSCLIENT_TRY_ECODE(pClient, n, "MsgId=%u: Transaction failed.", msgIdReq);

  //
  // Set return values from response.
  //
  LOGDIAG3("%s: Stopped.", bsClientAttrGetName(pClient));

  return BS_OK;
}

// DHP -- need to get firmware version? 
#if 0 
/*!
 * \brief Proxied request to get the firmware version.
 *
 * \param pClient             \h_botsense client.
 * \param hndVConn            Handle to virtual connection to close.
 * \param [out] puVersion     Version number.
 * \param [out] puRevision    Revesion number.
 *
 * \copydoc doc_return_bs_std
 */
int bsMouseBotReqGetFwVer(BsClient_P     pClient,
                    BsVConnHnd_T   hndVConn,
                    uint_t        *puVersion,
                    uint_t        *puRevision)
{
  static BsMouseBotMsgId_T    msgIdReq = BsMouseBotMsgIdReqGetFwVer;
  static BsMouseBotMsgId_T    msgIdRsp = BsMouseBotMsgIdRspGetFwVer;

  BsMouseBotRspGetFwVer_T       msgRsp;                   // response message 
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
  // Execute request-response transaction.
  //
  n = bsClientTrans(pClient,  hndVConn,
                    msgIdReq, buf, 0,
                    msgIdRsp, buf, sizeof(buf));

  // check transaction return code
  BSCLIENT_TRY_ECODE(pClient, n, "MsgId=%u: Transaction failed.", msgIdReq);

  //
  // Unpack response.
  //
  n = BsMouseBotUnpackRspGetFwVer(buf, (size_t)n, &msgRsp, bTrace);

  // check unpack return code
  BSCLIENT_TRY_NM_ECODE(pClient, n, "MsgId=%u", msgIdRsp);

  //
  // Set return values from response.
  //
  *puVersion  = msgRsp.m_version;
  *puRevision = msgRsp.m_revision;

  LOGDIAG3("%s: GetFwVer: (%d,%d).", bsClientAttrGetName(pClient),
                                *puVersion, *puRevision);

  return BS_OK;
}
#endif
