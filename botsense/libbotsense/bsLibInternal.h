////////////////////////////////////////////////////////////////////////////////
//
// Package:   BotSense
//
// Library:   libbsclient
//
// File:      bsLibInternal.h
//
/*! \file
 *
 * $LastChangedDate: 2012-11-25 10:42:41 -0700 (Sun, 25 Nov 2012) $
 * $Rev: 2544 $
 *
 * \brief The libBotSense internal declarations.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2009-2017. RoadNarrows LLC.\n
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

#ifndef _BSLIBINTERNAL_H
#define _BSLIBINTERNAL_H

#include <errno.h>
#include <pthread.h>

#include "rnr/rnrconfig.h"
#include "rnr/sock.h"

#include "botsense/BotSense.h"
#include "botsense/libBotSense.h"

//
// Default Time values in microseconds.
// Note: Defaults can be overridden by client attribute functions or by
// configuration file(s) (future).
//
#define BSCLIENT_T_READ  10000000     ///< 10 second read timeout default
#define BSCLIENT_T_WRITE  5000000     ///<  5 second write timeout default
#define BSCLIENT_T_FLUSH  5000000     ///<  5 second flush timeout default


/*!
 * \brief Cashed Transaction Information Structure
 */
typedef struct
{
  bool_t           m_bCached;     ///< this transaction is [not] cached
  BsProxyMsgHdr_T *m_pMsgHdr;     ///< cached message header
  byte_t          *m_pBuf;        ///< cached packed message body buffer
} BsTransInfo_T;

/*!
 * \brief Client Virtual Connection Info Structure
 */
typedef struct
{
  BsVConnHnd_T             m_hndVConn;  ///< virtual connection handle
  bool_t                   m_bTrace;    ///< do [not] trace this vconn msgs
  const char              *m_sDevName;  ///< device path name
  const char              *m_sModName;  ///< interface module path name
  const BsClientAppInfo_T *m_pAppInfo;  ///< app-specific info (optional)
} BsVConn_T;

/*!
 * \brief The Client Structure Type
 */
typedef struct _bsClientStruct
{
  const char         *m_sClientName;      ///< client name
  Socket_T           *m_pSocket;          ///< IP socket to server
  pthread_mutex_t     m_mutexTrans;       ///< client transaction mutex
  uint_t              m_uReqTimeout;      ///< client request timeout
  uint_t              m_uRspTimeout;      ///< client response timeout
  uint_t              m_uTidCounter;      ///< transaction id counter
  bool_t              m_bTraceServer;     ///< do [not] trace server-ended msgs
  BsTransInfo_T      *m_tblTransCache[BSPROXY_TID_NUMOF];
                                          ///< transaction cache
  int                 m_nVConnCount;      ///< number of active v. connections
  byte_t              m_tblHndIndex[BSPROXY_VCONN_MOD_NUMOF];
                                          ///< handle to vconn index table
  BsVConn_T          *m_tblVConn[BSPROXY_VCONN_CLIENT_MAX];
                                          ///< table of virtual connections
} BsClient_T;


// ---------------------------------------------------------------------------
// Internal Defines, Macros, Types and Structures
// ---------------------------------------------------------------------------

/*!
 * \brief Log [pre/un]packed message header.
 *
 * \param pClient   \h_botsense client.
 * \param sPreface  Preface string.
 * \param pMsgHdr   Pointer to message header.
 */
#ifdef LOG
#define _BS_LOG_MSGHDR(pClient, sPreface, pMsgHdr) \
  do \
  { \
    if( LOGABLE(LOG_LEVEL_DIAG3) ) \
    { \
      bsClientLogMsgHdr(pClient, sPreface, pMsgHdr); \
    } \
  } while(0)
#else
#define _BS_LOG_MSGHDR(pClient, sPreface, pMsgHdr)
#endif // LOG

/*!
 * \brief Log successful request recieved event.
 *
 * \param pClient   \h_botsense client.
 * \param pMsgHdr   Pointer to request header.
 */
#define _BS_LOG_REQ(pClient, pMsgHdr) \
  LOGDIAG3("%s: MsgId=%d sent.", \
      bsClientAttrGetName(pClient), (pMsgHdr)->m_hdrMsgId)

/*!
 * \brief Log successful repsonse recieved event.
 *
 * \param pClient   \h_botsense client.
 * \param pMsgHdr   Pointer to response header.
 */
#define _BS_LOG_RSP(pClient, pMsgHdr) \
  LOGDIAG3("%s: MsgId=%d received.", \
      bsClientAttrGetName(pClient), (pMsgHdr)->m_hdrMsgId)


// ---------------------------------------------------------------------------
// Internal Prototypes
// ---------------------------------------------------------------------------

/*!
 * Get the client vConnection given the handle.
 *
 * \param pClient   \h_botsense client.
 * \param hndVConn  New virtual connection proxied device handle.
 *
 * \return On success, returns vConnection, else returns NULL.
 */
INLINE_IN_H BsVConn_T *bsGetVConn(BsClient_T *pClient, BsVConnHnd_T hndVConn)
{
  if( BSCLIENT_IS_VCONN_HANDLE(hndVConn) && 
      (pClient->m_tblHndIndex[hndVConn] != BSPROXY_VCONN_UNDEF) )
  {
    return pClient->m_tblVConn[pClient->m_tblHndIndex[hndVConn]];
  }
  else
  {
    return NULL;
  }
}

extern void bsVConnClearAll(BsClient_T *pClient);

extern int bsVConnAdd(BsClient_T  *pClient,
                      BsVConnHnd_T hndVConn,
                      int          index);

extern int bsVConnRemove(BsClient_T *pClient, BsVConnHnd_T hndVConn);

extern int bsVConnNew(BsClient_T               *pClient,
                      const char               *sDevName,
                      const char               *sModName,
                      const BsClientAppInfo_T  *pAppInfo,
                      bool_t                    bTrace);

extern int bsVConnDelete(BsClient_T *pClient, int index);


#endif // _BSLIBINTERNAL_H
