////////////////////////////////////////////////////////////////////////////////
//
// Package:   kuon
//
// File:      bsKuon.h
//
/*! \file
 *
 * $LastChangedDate: 2011-01-13 11:05:37 -0700 (Thu, 13 Jan 2011) $
 * $Rev: 657 $
 *
 * \brief \h_botsense bsProxy client library Kuon robot interface.
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

#ifndef _BSKUON_H
#define _BSKUON_H

#include "rnr/rnrconfig.h"

#include "botsense/BotSense.h"
#include "botsense/libBotSense.h"

#include "Kuon/kuon.h"

#ifndef SWIG
C_DECLS_BEGIN
#endif // SWIG

#define BS_KUON_DEV_NAME    "/kuon"               ///< Kuon robot device

#define BS_KUON_SERVER_MOD  "libbsserver_kuon"    ///< server plugin dll module
#define BS_KUON_CLIENT_LIB  "libbsclient_kuon"    ///< client app library

/*!
 * \ingroup kuon_types
 * Motor Configuration Structure
 */
typedef struct
{
  int m_nMaxBwdSpeed; ///< maximum backward current/raw speed limit \h_le 0
  int m_nMaxFwdSpeed; ///< maximum forward current/raw speed limit \h_ge 0
  int m_nMinOdometer; ///< minimum current/raw odometer position limit \h_le 0
  int m_nMaxOdometer; ///< maximum current/raw odometer position limit \h_ge 0
} KuonMotorCfg_T;
  
/*!
 * \ingroup kuon_types
 * Motor IMU Structure
 */
typedef struct
{
  float   m_fAccX;    ///< acceleration in the x direction
  float   m_fAccY;    ///< acceleration in the x direction
  float   m_fAccZ;    ///< acceleration in the x direction
  float   m_fGyrX;    ///< rotation about the x-axis
  float   m_fGyrY;    ///< rotation about the y-axis
  float   m_fGyrZ;    ///< rotation about the z-axis
  float   m_fTemp;    ///< temperature
} KuonImu_T;


extern const char *bsKuonGetMsgName(BsClient_P   pClient,
                                  BsVConnHnd_T hndVConn,
                                  uint_t       uMsgId);

extern int bsKuonReqOpen( BsClient_P  pClient, 
                          const char* dev1,
                          const char* dev2,
                          bool_t server,
                          bool_t bInitTrace);

extern int bsKuonReqClose(BsClient_P pClient, BsVConnHnd_T hndVConn);

extern int bsKuonReqSetMotorSpeeds(BsClient_P   pClient,
                                 BsVConnHnd_T hndVConn,
                                 int          nSpeedLeft,
                                 int          nSpeedRight);

extern int bsKuonReqGetMotorSpeeds(BsClient_P     pClient,
                                 BsVConnHnd_T   hndVConn,
                                 int           *pnSpeedLeft,
                                 int           *pnSpeedRight);

extern int bsKuonReqAlterBrake(BsClient_P       pClient,
                                 BsVConnHnd_T   hndVConn,
                                 int            pnbrakeFL,
                                 int            pnbrakeFR,
                                 int            pnbrakeRL,
                                 int            pnbrakeRR);

extern int bsKuonReqAlterSlew(BsClient_P       pClient,
                                 BsVConnHnd_T  hndVConn,
                                 int           pnslewFL,
                                 int           pnslewFR,
                                 int           pnslewRL,
                                 int           pnslewRR);


extern int bsKuonReqReadImu(BsClient_P    pClient,
                            BsVConnHnd_T  hndVConn,
                            KuonImu_T    *pImu);

extern int bsKuonReqReadImuDecoupAngles(BsClient_P    pClient,
                                        BsVConnHnd_T  hndVConn,
                                        float        *pHeading,
                                        float        *pPitch,
                                        float        *pRoll);

extern int bsKuonReqZeroOutImuGyros(BsClient_P pClient, BsVConnHnd_T hndVConn);

extern int bsKuonReqStop(BsClient_P pClient, BsVConnHnd_T hndVConn);

extern int bsKuonReqGetMotorCfg(BsClient_P     pClient,
                              BsVConnHnd_T   hndVConn,
                              uint_t         uWhich,
                              KuonMotorCfg_T  *pMotorCfg);

extern int bsKuonReqSetMotorCfg(BsClient_P     pClient,
                                BsVConnHnd_T   hndVConn,
                                uint_t         uWhich,
                                KuonMotorCfg_T  *pMotorCfg);

extern int bsKuonReqGetFwVer(BsClient_P     pClient,
                             BsVConnHnd_T   hndVConn,
                             uint_t        *puVersion,
                             uint_t        *puRevision);

#ifndef SWIG
C_DECLS_END
#endif // SWIG

#endif // _BSKUON_H
