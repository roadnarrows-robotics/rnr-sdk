////////////////////////////////////////////////////////////////////////////////
//
// Package:   mousebot
//
// File:      bsMouseBot.h
//
/*! \file
 *
 * $LastChangedDate: 2012-04-26 12:16:42 -0600 (Thu, 26 Apr 2012) $
 * $Rev: 1899 $
 *
 * \brief \h_botsense bsProxy client library MouseBot robot interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2012.  RoadNarrows LLC.
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

#ifndef _BSMOUSEBOT_H
#define _BSMOUSEBOT_H

#include "rnr/rnrconfig.h"

#include "botsense/BotSense.h"
#include "botsense/libBotSense.h"

#include "Kuon/mousebot.h"

#ifndef SWIG
C_DECLS_BEGIN
#endif // SWIG

#define BS_MOUSEBOT_DEV_NAME    "/mousebot"             
                                ///< MouseBot robot device

#define BS_MOUSEBOT_SERVER_MOD  "libbsserver_mousebot"  
                                ///< server plugin dll module

#define BS_MOUSEBOT_CLIENT_LIB  "libbsclient_mousebot"  
                                ///< client app library


extern const char *bsMouseBotGetMsgName(BsClient_P   pClient,
                                  BsVConnHnd_T hndVConn,
                                  uint_t       uMsgId);

extern int bsMouseBotReqOpen( BsClient_P  pClient, 
                          const char* dev1,
                          const char* dev2,
                          bool_t server,
                          bool_t bInitTrace);

extern int bsMouseBotReqClose(BsClient_P pClient, BsVConnHnd_T hndVConn);

extern int bsMouseBotReqSetMotorSpeeds(BsClient_P   pClient,
                                 BsVConnHnd_T hndVConn,
                                 int          nSpeedLeft,
                                 int          nSpeedRight);

extern int bsMouseBotReqGetMotorSpeeds(BsClient_P     pClient,
                                 BsVConnHnd_T   hndVConn,
                                 int           *pnSpeedLeft,
                                 int           *pnSpeedRight);

extern int bsMouseBotReqAlterBrake(BsClient_P       pClient,
                                 BsVConnHnd_T   hndVConn,
                                 int            pnbrakeFL,
                                 int            pnbrakeFR,
                                 int            pnbrakeRL,
                                 int            pnbrakeRR);

extern int bsMouseBotReqAlterSlew(BsClient_P       pClient,
                                 BsVConnHnd_T  hndVConn,
                                 int           pnslewFL,
                                 int           pnslewFR,
                                 int           pnslewRL,
                                 int           pnslewRR);


extern int bsMouseBotReqStop(BsClient_P pClient, BsVConnHnd_T hndVConn);

extern int bsMouseBotReqGetMotorCfg(BsClient_P     pClient,
                              BsVConnHnd_T   hndVConn,
                              uint_t         uWhich,
                              MouseBotMotorCfg_T  *pMotorCfg);

extern int bsMouseBotReqSetMotorCfg(BsClient_P     pClient,
                              BsVConnHnd_T   hndVConn,
                              uint_t         uWhich,
                              MouseBotMotorCfg_T  *pMotorCfg);

extern int bsMouseBotReqGetFwVer(BsClient_P     pClient,
                           BsVConnHnd_T   hndVConn,
                           uint_t        *puVersion,
                           uint_t        *puRevision);

#ifndef SWIG
C_DECLS_END
#endif // SWIG

#endif // _BSMOUSEBOT_H
