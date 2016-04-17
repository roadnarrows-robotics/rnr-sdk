////////////////////////////////////////////////////////////////////////////////
//
// Package:   CogniBoost
//
// File:      libCogniBoost.h
//
/*! \file
 *
 *  $LastChangedDate: 2011-10-19 15:05:47 -0600 (Wed, 19 Oct 2011) $
 *  $Rev: 1398 $
 *
 * \brief CogniBoost host library libCogniBoost internal functions and
 * declarations.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 * \author Brent Wilkins  (brent@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011  RoadNarrows LLC.
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
///////////////////////////////////////////////////////////////////////////////

#ifndef _CB_LIB_INTERNAL_H
#define _CB_LIB_INTERNAL_H

#include <errno.h>
#include <pthread.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/serdev.h"

#include "CogniBoost/CogniMem.h"
#include "CogniBoost/CogniBoost.h"
#include "CogniBoost/CogniBoostProto.h"
#include "CogniBoost/libCogniBoost.h"

// ----------------------------------------------------------------------------
// Internal Defines, Macros, Types and Structures
// ---------------------------------------------------------------------------

/*!
 * \brief Check if <em>CogniBoost</em> return value is (not) an error
 *
 * If the check fails, an appropriate error is logged and the calling 
 * function immediately returns with the value -ecode.
 *
 *
 * \param hnd     CogniBoost handle.
 * \param ecode   \h_cogniboost error code.
 * \param efmt    Error output format string literal.
 * \param ...     Error variable arguments. 
 */
#define CHK_ECODE(hnd, ecode, efmt, ...) \
do \
{ \
  if( (ecode) < 0 ) \
  { \
    CB_LOG_ERROR(hnd, ecode, efmt, ##__VA_ARGS__); \
    return (ecode); \
  } \
} while(0)

/*!
 * \brief Log NetMsgs error.
 *
 * \param hnd     CogniBoost handle.
 * \param nmecode NetMsgs error code.
 * \param efmt    Error output formatted string literal.
 * \param ...     Error variable arguments.
 */
#define CB_LOG_NM_ERROR(hnd, nmecode, efmt, ...) \
  CB_LOG_ERROR(hnd, CB_LIB_ECODE_BAD_MSG, "%s(nmecode=%d): " efmt, \
        nmStrError(nmecode), (nmecode>=0? nmecode: -nmecode), ##__VA_ARGS__)

/*!
 * \brief Check if NetMsgs (un)packing return value is not an error (\<0)
 *
 * If the check fails, an appropriate error is logged and the calling 
 * function immediately returns with the value -CB_RSP_ECODE_BAD_MSG.
 *
 * \param hnd     CogniBoost handle.
 * \param nmecode NetMsgs error code.
 * \param efmt    Error output format string literal.
 * \param ...     Error variable arguments. 
 */
#define CHK_NM_ECODE(hnd, nmecode, efmt, ...) \
do \
{ \
  if( (nmecode) < 0 ) \
  { \
    CB_LOG_NM_ERROR(hnd, nmecode, efmt, ##__VA_ARGS__); \
    return (-CB_RSP_ECODE_BAD_MSG); \
  } \
} while(0)

/*!
 * \brief Check if expression evaluates to true.
 *
 * If the check is false, an appropriate error is logged and the calling 
 * function immediately returns with the appropriate CogniBoost error code.
 *
 * \param hnd     CogniBoost handle.
 * \param expr    Expression to evaluate
 * \param ecode   CogniBoost error code.
 * \param efmt    Error output format string literal.
 * \param ...     Variable arguments.
 */
#define CHK_EXPR(hnd, expr, ecode, efmt, ...) \
do \
{ \
  if ( !(expr) ) \
  { \
    CB_LOG_ERROR(hnd, ecode, "%s: " efmt, #expr, ##__VA_ARGS__); \
    return (ecode > 0) ? -(ecode) : ecode; \
  } \
} while(0)
 
/*!
 * \brief Check if CogniBoost client device handle is valid and open.
 *
 * If the check is false, an appropriate error is logged and the calling 
 * function immediately returns with the appropriate CogniBoost error code.
 *
 * \param hnd     CogniBoost handle.
 */
#define CHK_DEV(hnd) \
do \
{ \
  if ( ((hnd) == CB_HND_NONE) || (cbAttrGetFd((hnd)) < 0) ) \
  { \
    CB_LOG_ERROR(hnd, CB_LIB_ECODE_NO_DEV, "No CogniMem device open."); \
    return -CB_LIB_ECODE_NO_DEV; \
  } \
} while(0)

/*!
 *  \brief Cached transaction structure.
 */
typedef struct
{
  bool_t        m_bCached;      ///< this transaction is [not] cached
  cbPktHdr_T   *m_pPktHdr;      ///< cached packet header
  byte_t       *m_pBuf;         ///< cached packed message body
} cbTransInfo_T;

/*!
 * \brief  CogniBoost Device Structure Type
 *
 * \todo Virtualize to support both direct and proxied connections.
 */
typedef struct _cbDeviceStruct
{
  const char     *m_sDeviceName;    ///< serial USB device name
  int             m_nBaudRate;      ///< serial USB device baud rate
  cbConnType_T    m_eConnType;      ///< connection type
  int             m_fd;             ///< file descriptor to CogniBoost device
  bool_t          m_bTrace;         ///< do [not] trace messages
  cbIdentities_T  m_cbIds;          ///< Cogniboost identities
  pthread_mutex_t m_mutexTrans;     ///< client transaction mutex
  uint_t          m_uTidCounter;    ///< transaction id counter
  cbTransInfo_T  *m_tblTransCache[CB_HDR_TID_NUMOF]; ///< transaction cache
} cbDevice_T;



// ---------------------------------------------------------------------------
// Internal Prototypes
// ---------------------------------------------------------------------------

extern cbHnd_T cbOpen(const char *sDeviceName, int nBaudRate, bool_t bTrace);

extern int cbClose(cbHnd_T hnd);

extern int cbTransLow(cbHnd_T   hnd,
                      cbMsgId_T eReqMsgId,
                      byte_t    bufReqPkt[],
                      size_t    uReqMsgLen,
                      cbMsgId_T eRspMsgId,
                      byte_t    bufRspMsg[],
                      size_t    sizeRspBuf);

extern int cbTrans(cbHnd_T     hnd,
                   cbMsgId_T   eReqMsgId,
                   void       *pReqStruct,
                   cbMsgId_T   eRspMsgId,
                   void       *pRspStruct);

extern void timer_mark(struct timeval *pTvMark);

extern uint_t timer_elapsed(struct timeval *pTvMark);

#ifdef LOG
extern void bsLogBuf(const char *sBufName, byte_t buf[], size_t uCount);

extern void bsLogAsciiBuf(const char *sBufName, byte_t buf[], size_t uCount);
#endif // LOG


#endif // _CB_LIB_INTERNAL_H
