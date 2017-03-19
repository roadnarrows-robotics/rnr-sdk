////////////////////////////////////////////////////////////////////////////////
//
// Package:   BotSense
//
// File:      libBotSense.h
//
/*! \file
 *
 * $LastChangedDate: 2012-11-25 10:42:41 -0700 (Sun, 25 Nov 2012) $
 * $Rev: 2544 $
 *
 * \brief \h_botsense client library declarations.
 *
 * \note This file must be swig-able to generate a python extension module.
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

#ifndef _BSCLIENT_H
#define _BSCLIENT_H

#include "errno.h"

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "botsense/BotSense.h"


#ifndef SWIG
C_DECLS_BEGIN
#endif

// ---------------------------------------------------------------------------
// Types and Defines
// ---------------------------------------------------------------------------

/*!
 * \brief Pointer to client structure forward declaration.
 */
typedef struct _bsClientStruct *BsClient_P;

/*!
 * \brief \h_botsense client library information and callbacks to 
 * application-specific data.
 *
 * When an application (library) establishes an application-specific virtual
 * connection, this static information and callbacks are passed to the client
 * library.
 *
 * The information and callbacks are optional. The \h_botsense library will
 * ignore any member value that is set to NULL or if the pointer to the info
 * is NULL (everything is ignored).
 *
 */
typedef struct
{
  const char  *app_name;          ///< application name
  const char  *brief;             ///< brief one-line description
  const char  *version;           ///< dotted version x.y.z[-app]
  const char  *date;              ///< version date
  const char  *maintainer;        ///< maintainer/owner
  const char  *license;           ///< short license/copyright statement

  /*!
   * \brief Get the message name callback function.
   *
   * \param pClient   \h_botsense client.
   * \param hndVConn  Virtual connection handle.
   * \param uMsgId    Message id associated with virtual connection interface.
   *
   * \return
   * Returns static null-terminated string.
   */
  const char *(*fnGetMsgName)(BsClient_P   pClient,
                              BsVConnHnd_T hndVConn,
                              uint_t       uMsgId);
} BsClientAppInfo_T;

/*!
 * \h_botsense Client Connection State Structure Type
 */
typedef struct
{
  bool_t      m_bIsConnected;     ///< client is [not] connected to server
  const char *m_sServerHostName;  ///< server host name
} BsClientConnState_T;

/*!
 * \h_botsense Server Vector of Client Virtual Connection Handles Type
 */
typedef struct
{
  size_t        m_uCount;                           ///< vector length
  BsVConnHnd_T  m_vecHnd[BSPROXY_VCONN_CLIENT_MAX]; ///< vector of handles
} BsVecHandles_T;


/*!
 * Maximum size (including null) of a virtual connection information string.
 */
#define BSCLIENT_INFO_STR_MAX_SIZE   256

/*!
 * \h_botsense Server Proxied Device Information Type
 */
typedef struct
{
  byte_t  m_vconn;                                ///< virtual connection handle
  int     m_rd;                                   ///< resource descriptor
  char    m_client[BSCLIENT_INFO_STR_MAX_SIZE];   ///< client name
  char    m_devuri[BSCLIENT_INFO_STR_MAX_SIZE];   ///< device URI
  char    m_moduri[BSCLIENT_INFO_STR_MAX_SIZE];   ///< i/f module URI
  char    m_modver[BSCLIENT_INFO_STR_MAX_SIZE];   ///< i/f module version
  char    m_moddate[BSCLIENT_INFO_STR_MAX_SIZE];  ///< i/f module date
} BsVConnInfo_T;

/*!
 * \brief Log Warning.
 *
 * \param pClient   \h_botsense client.
 * \param ecode     \h_botsense error code.
 * \param wfmt      Warning output format string literal.
 * \param ...       Warning variable arguments.    
 */
#define BSCLIENT_LOG_WARN(pClient, ecode, wfmt, ...) \
  LOGDIAG3("Warning: %s: %s(): %s(ecode=%d): " wfmt, \
      bsClientAttrGetName(pClient), LOGFUNCNAME, \
      bsStrError(ecode), ((ecode)>=0? (ecode): -(ecode)), \
      ##__VA_ARGS__)

/*!
 * \brief Log Error.
 *
 * \param pClient   \h_botsense client.
 * \param ecode     \h_botsense error code.
 * \param efmt      Error output format string literal.
 * \param ...       Error variable arguments.    
 */
#define BSCLIENT_LOG_ERROR(pClient, ecode, efmt, ...) \
  LOGERROR("%s: %s(): %s(ecode=%d): " efmt, \
      bsClientAttrGetName(pClient), LOGFUNCNAME, \
      bsStrError(ecode), (ecode>=0? ecode: -ecode), \
      ##__VA_ARGS__)

/*!
 * \brief Log NetMsgs Error.
 *
 * \param pClient   \h_botsense client.
 * \param nmecode   NetMsgs error code.
 * \param efmt      Error output format string literal.
 * \param ...       Error variable arguments.    
 */
#define BSCLIENT_LOG_NM_ERROR(pClient, nmecode, efmt, ...) \
  BSCLIENT_LOG_ERROR(pClient, BS_ECODE_BAD_MSG, "%s(nmecode=%d): " efmt, \
        nmStrError(nmecode), (nmecode>=0? nmecode: -nmecode), ##__VA_ARGS__)

/*!
 * \brief Log System Error.
 *
 * \param pClient   \h_botsense client.
 * \param ecode     \h_botsense error code.
 * \param efmt      Error output format string litteral.
 * \param ...       Error variable arguments.    
 */
#define BSCLIENT_LOG_SYSERROR(pClient, ecode, efmt, ...) \
  LOGERROR("%s: %s(): %s(ecode=%d): %s(errno=%d)" efmt, \
          bsClientAttrGetName(pClient), LOGFUNCNAME, \
          bsStrError(ecode), ((ecode)>=0? (ecode): -(ecode)), \
          strerror(errno), errno, ##__VA_ARGS__)

/*!
 * \brief Check if \h_botsense return value is not an error (\h_lt 0).
 *
 * If the check is false, an appropriate error is logged and the calling
 * function is immediately exited by invoking a <b>return</b> with the \h_lt 0
 * error code.
 *
 * \param pClient   \h_botsense client.
 * \param ecode     \h_botsense error code.
 * \param efmt      Error output format string literal.
 * \param ...       Error variable arguments.    
 */
#define BSCLIENT_TRY_ECODE(pClient, ecode, efmt, ...) \
  do \
  { \
    if( (ecode) < 0 ) \
    { \
      BSCLIENT_LOG_ERROR(pClient, ecode, efmt, ##__VA_ARGS__); \
      return (ecode); \
    } \
  } while(0)

/*!
 * \brief Check if NetMsgs (un)packing return value is not an error (\h_lt 0).
 *
 * If the check is false, an appropriate error is logged and the calling
 * function is immediately exited by invoking a <b>return</b> with the \h_lt 0
 * error code.
 *
 * \param pClient   \h_botsense client.
 * \param nmecode   NetMsgs error code.
 * \param efmt      Error output format string literal.
 * \param ...       Error variable arguments.    
 */
#define BSCLIENT_TRY_NM_ECODE(pClient, nmecode, efmt, ...) \
  do \
  { \
    if( (nmecode) < 0 ) \
    { \
      BSCLIENT_LOG_NM_ERROR(pClient, nmecode, efmt, ##__VA_ARGS__); \
      return -BS_ECODE_BAD_MSG; \
    } \
  } while(0)

/*!
 * \brief Check if expression evaluates to true.
 *
 * If the check is false, an appropriate error is logged and the calling
 * function is immediately exited by invoking a <b>return</b> with the \h_lt 0
 * error code.
 *
 * \param pClient   \h_botsense client.
 * \param expr      Validation expression.
 * \param ecode     \h_botsense error code.
 * \param efmt      Error output format string literal.
 * \param ...       Error variable arguments.    
 */
#define BSCLIENT_TRY_EXPR(pClient, expr, ecode, efmt, ...) \
  do \
  { \
    if( !(expr) ) \
    { \
      BSCLIENT_LOG_ERROR(pClient, ecode, "%s: " efmt, #expr, ##__VA_ARGS__); \
      return (ecode)>0? -(ecode): (ecode); \
    } \
  } while(0)

/*!
 * \brief Test if the handle is in the valid client virtual connection range.
 *
 * \param hnd   Virtual connection handle.
 */
#define BSCLIENT_IS_VCONN_HANDLE(hnd) \
  (((int)(hnd) >= BSPROXY_VCONN_MOD_MIN) && \
   ((int)(hnd) <= BSPROXY_VCONN_MOD_MAX))

/*!
 * \brief Tests if the handle is valid and there is an established 
 * virtual connection.
 *
 * \param pClient   \h_botsense client.
 * \param hnd       Virtual connection handle.
 */
#define BSCLIENT_HAS_VCONN(pClient, hnd) \
  (BSCLIENT_IS_VCONN_HANDLE(hnd) && bsClientAttrHasVConn(pClient, hnd))



// ---------------------------------------------------------------------------
// Prototypes
// ---------------------------------------------------------------------------

//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Utility Prototypes
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

extern const char *bsStrError(int nECode);

extern int bsPackMsgHdr(BsProxyMsgHdr_T *pMsgHdr, byte_t buf[], size_t bufSize);

extern int bsUnpackMsgHdr(byte_t           buf[],
                          size_t           bufSize,
                          BsProxyMsgHdr_T *pMsgHdr);

#ifdef LOG

extern void bsLogBuf(const char *sBufName, byte_t buf[], size_t uCount);

extern void bsLogAsciiBuf(const char *sBufName, byte_t buf[], size_t uCount);

#endif // LOG


//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Client-Specific Prototypes
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

extern BsClient_P bsClientNew(const char *sClientName);

extern void bsClientDelete(BsClient_P pClient);

extern int bsClientTrans(BsClient_P pClient,
                         int        hndVConn,
                         uint_t     uReqMsgId,
                         byte_t     bufReq[],
                         size_t     uReqBodyLen,
                         uint_t     uRspMsgId,
                         byte_t     bufRsp[],
                         size_t     sizeRspBuf);

extern const char *bsClientAttrGetName(BsClient_P pClient);

extern void bsClientAttrGetTimeouts(BsClient_P  pClient,
                                    uint_t     *pReqTimeout,
                                    uint_t     *pRspTimeout);

extern void bsClientAttrSetTimeouts(BsClient_P pClient,
                                    uint_t     uReqTimeout,
                                    uint_t     uRspTimeout);

extern bool_t bsClientAttrGetTraceState(BsClient_P   pClient,
                                        BsVConnHnd_T hndVConn);
      
extern int bsClientAttrGetVConnCount(BsClient_P pClient);

extern bool_t bsClientAttrHasVConn(BsClient_P pClient, BsVConnHnd_T hndVConn);

extern const char *bsClientAttrGetDevName(BsClient_P   pClient,
                                          BsVConnHnd_T hndVConn);

extern const char *bsClientAttrGetModName(BsClient_P   pClient,
                                          BsVConnHnd_T hndVConn);

extern void bsClientAttrSetLogging(BsClient_P pClient, int nLevel);

extern void bsClientAttrGetConnState(BsClient_P           pClient,
                                     BsClientConnState_T *pConnState);

extern void bsClientFillMsgHdr(BsClient_P       pClient,
                               BsVConnHnd_T     hndVConn,
                               uint_t           uMsgId,
                               size_t           uBodyLen,
                               BsProxyMsgHdr_T *pMsgHdr);

extern const char *bsClientGetMsgName(BsClient_P   pClient,
                                      BsVConnHnd_T hndVConn,
                                      uint_t       uMsgId);

#ifdef LOG

extern void bsClientLogMsgHdr(BsClient_P       pClient,
                              const char      *sPreface,
                              BsProxyMsgHdr_T *pMsgHdr);

#endif // LOG


//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 
// Client - Server (bsProxy) Connection, Control, and Info Requests Prototypes
//. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . 

extern int bsServerConnect(BsClient_P  pClient,
                           const char *sServerHostName,
                           int         ipPortServer);

extern int bsServerDisconnect(BsClient_P pClient);

extern int bsServerReqGetVersion(BsClient_P pClient,
                                 char       buf[],
                                 size_t     sizeBuf);

extern int bsServerReqLoopback(BsClient_P pClient, char sLoopbackMsg[]);

extern int bsServerReqSetLogging(BsClient_P pClient, int nLogLevel);

extern int bsServerReqMsgTrace(BsClient_P   pClient,
                               BsVConnHnd_T hndVConn,
                               bool_t       bTrace);

extern int bsServerReqOpenDev(BsClient_P               pClient,
                              const char              *sDevName,
                              const char              *sModName,
                              byte_t                   argbuf[],
                              size_t                   arglen,
                              const BsClientAppInfo_T *pAppInfo,
                              bool_t                   bTrace);

extern int bsServerReqCloseDev(BsClient_P pClient, BsVConnHnd_T hndVConn);

extern int bsServerReqGetVConnList(BsClient_P      pClient,
                                   BsVecHandles_T *pVecHandles);

extern int bsServerReqGetVConnInfo(BsClient_P     pClient,
                                   BsVConnHnd_T   hndVConn,
                                   BsVConnInfo_T *pVConnInfo);

#ifndef SWIG
C_DECLS_END
#endif


#endif // _BSCLIENT_H
