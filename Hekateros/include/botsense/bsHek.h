////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// File:      bsHek.h
//
/*! \file
 *
 * $LastChangedDate: 2014-09-18 16:53:49 -0600 (Thu, 18 Sep 2014) $
 * $Rev: 3748 $
 *
 * \brief \h_botsense bsProxy client library Hekateros interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2011-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
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

#ifndef _BSHEK_H
#define _BSHEK_H

#include "rnr/rnrconfig.h"

#include "botsense/BotSense.h"
#include "botsense/libBotSense.h"

#include "Dynamixel/DynaTypes.h"
#include "Hekateros/hekateros.h"

#ifndef SWIG
C_DECLS_BEGIN
#endif // SWIG

#define BS_HEK_SERVER_MOD  "libbsserver_hek"     ///< server plugin dll module
#define BS_HEK_CLIENT_LIB  "libbsclient_hek"     ///< client app library

extern const char *bsHekGetMsgName(BsClient_P   pBsClient,
                                   BsVConnHnd_T hndVConn,
                                   uint_t       uMsgId);

extern int bsHekOpen(BsClient_P  pBsClient,
                     const char *sDevName,
                     int         nBaudRate,
                     bool_t      bInitTrace);

extern int bsHekClose(BsClient_P pBsClient, BsVConnHnd_T hndBsVConn);

extern int bsHekGetVersion(BsClient_P   pBsClient,
                           BsVConnHnd_T hndBsVConn,
                           char         bufVersion[],
                           size_t       sizeBuf);

extern int bsHekMoveAtSpeedTo(BsClient_P          pClient,
                              BsVConnHnd_T        hndVConn,
                              DynaSpeedPosTuple_T vecMove[],
                              size_t              uNumServos);

extern int bsHekGetState(BsClient_P          pClient,
                         BsVConnHnd_T        hndVConn,
                         DynaSpeedPosTuple_T vecState[],
                         size_t              uNumServos);

extern int bsHekFreeze(BsClient_P          pClient,
                       BsVConnHnd_T        hndVConn,
                       DynaSpeedPosTuple_T vecState[],
                       size_t              uNumServos);

extern int bsHekEStop(BsClient_P          pClient,
                      BsVConnHnd_T        hndVConn,
                      DynaSpeedPosTuple_T vecState[],
                      size_t              uNumServos);

extern int bsHekCalibrate(BsClient_P          pClient,
                          BsVConnHnd_T        hndVConn,
                          DynaSpeedPosTuple_T vecState[],
                          size_t              uNumServos);

extern int bsHekGetHealth(BsClient_P        pClient,
                          BsVConnHnd_T      hndVConn,
                          int               vecServoIds[],
                          DynaHealthTuple_T vecHealth[],
                          size_t            uNumServos);

extern int bsHekClearAlarms(BsClient_P    pClient,
                            BsVConnHnd_T  hndVConn,
                            int           vecServoIds[],
                            size_t        uNumServos);


#ifndef SWIG
C_DECLS_END
#endif // SWIG

#endif // _BSHEK_H
