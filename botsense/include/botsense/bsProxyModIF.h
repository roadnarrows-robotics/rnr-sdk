////////////////////////////////////////////////////////////////////////////////
//
// Package:   BotSense
//
// File:      bsProxyModIF.h
//
/*! \file
 *
 * $LastChangedDate: 2010-08-20 11:36:38 -0600 (Fri, 20 Aug 2010) $
 * $Rev: 568 $
 *
 * \brief \h_botsense bsProxy Dynamically Linked Library module interface.
 *
 * The client side assumes responsibility for "doing the right thing". So,
 * bsProxy does minimal consistency checking.
 *
 * A bsProxy module must be thread safe. The bsProxy device thread provides
 * context control and sequencing.
 * 
 * The bsProxy provides a unique handle to identify a specific
 * device-module instance (virtual connection).
 *
 * A bsProxy interface module:
 * \li Typically supports simultaneous multiple device-module instances.
 * \li Should support different devices of the same class simultaneously.
 *    (e.g. /dev/ttyS0 and /dev/ttyUSB0).
 * \li Should support different handles to the same device-module association.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2010.  RoadNarrows LLC.
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

#ifndef _BSPROXYMODIF_H
#define _BSPROXYMODIF_H

#include "rnr/rnrconfig.h"
#include "rnr/netmsgs.h"

#include "botsense/BotSense.h"

C_DECLS_BEGIN


// ---------------------------------------------------------------------------
// Exported Interface Module Symbols
// ---------------------------------------------------------------------------

/*!
 * \brief Required Module Symbols. See below for function prototypes.
 */
#define BSMOD_SYM_INIT      bsModInit     ///< module initialization function
#define BSMOD_SYM_EXIT      bsModExit     ///< module deinit exit function
#define BSMOD_SYM_OPEN      bsModOpen     ///< open device function
#define BSMOD_SYM_CLOSE     bsModClose    ///< close device function
#define BSMOD_SYM_REQUEST   bsModRequest  ///< process client request function
#define BSMOD_SYM_TRACE     bsModTrace    ///< enable/disable tracing function
#define BSMOD_SYM_INFO      bsModInfo     ///< get module static info function

/*!
 * \brief Convert expanded macro parameter to string literal.
 * \param x Parameter.
 */
#define _S_EXP(x) _S_LIT(x)

/*!
 * \brief Convert macro parameter to string literal.
 * \param x Parameter.
 */
#define _S_LIT(x) #x

/*!
 * \brief Required Module Symbols Strings.
 */
#define BSMOD_SYM_INIT_S    _S_EXP(BSMOD_SYM_INIT)    ///< init function string
#define BSMOD_SYM_EXIT_S    _S_EXP(BSMOD_SYM_EXIT)    ///< exit function string
#define BSMOD_SYM_OPEN_S    _S_EXP(BSMOD_SYM_OPEN)    ///< open function string
#define BSMOD_SYM_CLOSE_S   _S_EXP(BSMOD_SYM_CLOSE)   ///< close function string
#define BSMOD_SYM_REQUEST_S _S_EXP(BSMOD_SYM_REQUEST) ///< request func string
#define BSMOD_SYM_TRACE_S   _S_EXP(BSMOD_SYM_TRACE)   ///< tracing func string
#define BSMOD_SYM_INFO_S    _S_EXP(BSMOD_SYM_INFO)    ///< info function string


// ---------------------------------------------------------------------------
// Interface Module Data Types and Helper Macros
// ---------------------------------------------------------------------------

/*!
 * \brief Standard bsProxy static interface module information structure type.
 */
typedef struct
{
  const char  *mod_name;    ///< module load name sans OS dependent substrings
  const char  *brief;       ///< brief one-line description
  const char  *version;     ///< dotted version x.y.z[-app[-a.b.c]]
  const char  *date;        ///< version date
  const char  *maintainer;  ///< maintainer/owner
  const char  *license;     ///< short license/copyright statement
} BsModInfo_T;

/*!
 * \brief Useful indirect indexing of handle to resource instance structure.
 *
 * A module's implementation is not required to use this data structure and the
 * associated functions. However, it can save developer's time in many 
 * instances when many handle values must be are mapped to few resources.
 */
typedef struct
{
  byte_t    m_vecIndex[BSPROXY_VCONN_MOD_NUMOF];  ///< vecIndex[handle] -> index
  void    **m_vecRsrc;                            ///< vecRsrc[index] -> rsrc
  uint_t    m_uMaxResources;                      ///< maximum resource count
  uint_t    m_uInUseCount;                        ///< resources in-use count
} BsModRsrcTbl_T;

/*!
 * \brief Test if the handle is in the valid module virtual connection range.
 *
 * \param hndVConn  Virtual connection handle.
 */
#define BSMOD_IS_VCONN_HANDLE(hndVConn) \
  (((int)(hndVConn) >= BSPROXY_VCONN_MOD_MIN) && \
   ((int)(hndVConn) <= BSPROXY_VCONN_MOD_MAX))

/*!
 * \brief Test if the resource table resource handle slot is free.
 *
 * \param pRsrcTbl  Pointer to resource table.
 * \param hndVConn  Virtual connection handle.
 */
#define BSMOD_RSRC_IS_FREE(pRsrcTbl, hndVConn) \
  (BSMOD_IS_VCONN_HANDLE(hndVConn) && \
  (pRsrcTbl)->m_vecIndex[hndVConn] == (byte_t)BSPROXY_VCONN_UNDEF)

/*!
 * \brief Test if the resource table resource handle slot is in-use.
 *
 * \param pRsrcTbl  Pointer to resource table.
 * \param hndVConn  Virtual connection handle.
 */
#define BSMOD_RSRC_IS_INUSE(pRsrcTbl, hndVConn) \
  (BSMOD_IS_VCONN_HANDLE(hndVConn) && \
  (pRsrcTbl)->m_vecIndex[hndVConn] != (byte_t)BSPROXY_VCONN_UNDEF)

/*!
 * \brief Get the resource index given the handle.
 *
 * \param pRsrcTbl  Pointer to resource table.
 * \param hndVConn  Virtual connection handle.
 *
 * \return On success, returns \h_ge 0 index. Else returns -1.
 */
#define BSMOD_RSRC_INDEX(pRsrcTbl, hndVConn) \
  ( BSMOD_IS_VCONN_HANDLE(hndVConn)? \
      ((pRsrcTbl)->m_vecIndex[hndVConn] != (byte_t)BSPROXY_VCONN_UNDEF? \
        (int)((pRsrcTbl)->m_vecIndex[hndVConn]): -1): \
      -1 \
  )

/*!
 * \brief Get the resource given the handle.
 *
 * \param pRsrcTbl  Pointer to resource table.
 * \param hndVConn  Virtual connection handle.
 *
 * \return On success, returns pointer to resource. Else returns NULL.
 */
#define BSMOD_RSRC(pRsrcTbl, hndVConn) \
  ( BSMOD_IS_VCONN_HANDLE(hndVConn)? \
      ((pRsrcTbl)->m_vecIndex[hndVConn] != (byte_t)BSPROXY_VCONN_UNDEF? \
        ((pRsrcTbl)->m_vecRsrc[(pRsrcTbl)->m_vecIndex[hndVConn]]): NULL): \
      NULL \
  )
  
/*!
 * \brief Get the resouce table current in-use count.
 *
 * \param pRsrcTbl  Pointer to resource table.
 */
#define BSMOD_RSRC_INUSE_COUNT(pRsrcTbl) ((pRsrcTbl)->m_uInUseCount)

/*!
 * \brief What to iterate over.
 */
typedef enum
{
  BsModIterOverDevUri,      ///< iterator over device URI
  BsModIterOverModUri,      ///< iterator over module URI
  BsModIterOverVConn        ///< iterator over virtual connections (no pattern)
} BsModIterOver_T;

/*!
 * \brief Module Iterator Type.
 */
typedef struct
{
  BsModIterOver_T   m_eOver;      ///< iterator over enum
  BsVConnHnd_T      m_hndVConn;   ///< virtual connection handle
  char             *m_sDevUri;    ///< device URI
  char             *m_sModUri;    ///< module URI
  int               m_rd;         ///< module's resource descriptor
} BsModIter_T;

/*!
 * \brief Interface Module callbacks to bsProxy services type.
 */
typedef struct
{
  /*!
   * \brief Send module-specific repsonse callback function.
   *
   * \param hndVConn  Virtual connection handle.
   * \param uTid      Request-Response transaction id.
   * \param uMsgId    Response message id.
   * \param bufRsp    Packed repsonse message body.
   * \param uRspLen   Length of response in buffer (number of bytes).
   */
  void (*m_cbSendRsp)(BsVConnHnd_T hndVConn,
                      BsTid_T      uTid,
                      BsMsgId_T    uMsgId,
                      byte_t       bufRsp[],
                      size_t       uRspLen);

  /*!
   * \brief Send generic ok repsonse callback function.
   *
   * \param hndVConn  Virtual connection handle.
   * \param uTid      Request-Response transaction id.
   */
  void (*m_cbSendOkRsp)(BsVConnHnd_T hndVConn, BsTid_T uTid);

  /*!
   * \brief Send generic error repsonse callback function.
   *
   * \param hndVConn  Virtual connection handle.
   * \param uTid      Request-Response transaction id.
   * \param nECode    \copydoc doc_param_ecode
   * \param sErrFmt   Error format string.
   * \param ...       Variable arguments to format string.
   */
  void (*m_cbSendErrorRsp)(BsVConnHnd_T  hndVConn,
                           BsTid_T       uTid,
                           int           nECode,
                           const char   *sErrFmt,
                           ...);

  /*!
   * \brief Allocatae a new module resource table of fixed size.
   *
   * The resource table is indexed by the virtual connection handle.
   *
   * \param nMaxResources   Maximum number of simultaneous resources supported.
   *
   * \return Pointer to allocated resource block.
   */
  BsModRsrcTbl_T *(*m_cbModRsrcTblNew)(int nMaxResources);

  /*!
   * \brief Delete an allocated resource table.
   *
   * \warning Module-specific resources should be freed up prior to calling this
   * function.
   *
   * \param pRsrcTbl  Pointer to resource table.
   */
  void (*m_cbModRsrcTblDelete)(BsModRsrcTbl_T *pRsrcTbl);
 
  /*!
   * \brief Add a new resource to the resource table.
   *
   * \param pRsrcTbl  Pointer to resource table.
   * \param hndVConn  Virtual connection handle.
   * \param pRsrc     Pointer to allocated resource associated with handle.
   *
   * \copydoc doc_return_std
   */
  int (*m_cbModRsrcAdd)(BsModRsrcTbl_T *pRsrcTbl,
                        BsVConnHnd_T    hndVConn,
                        void           *pRsrc);

  /*!
   * \brief Remove a resource from the resource table.
   *
   * The resource is not deleted.
   *
   * \param pRsrcTbl  Pointer to resource table.
   * \param hndVConn  Virtual connection handle.
   *
   * \return On success, returns pointer to removed resource. On failure, NULL
   * is returned.
   */
  void *(*m_cbModRsrcRemove)(BsModRsrcTbl_T *pRsrcTbl, BsVConnHnd_T hndVConn);

  /*!
   * \brief Copy the device URI associated with the virtual connection.
   *
   * \param hndVConn    Virtual connection handle.
   * \param [out] dest  Destination buffer.
   * \param n           Size of buffer.
   *
   * \return Pointer to destination buffer dest.
   */
  const char *(*m_fnGetDevUri)(BsVConnHnd_T hndVConn, char dest[], size_t n);

  /*!
   * \brief Start an iterator over the virtual connections matching the given 
   * pattern.
   *
   * If a virtual connection match is found, an iterator is allocated and its
   * data is filled with the first set of information.
   *
   * The caller must delete the iterator by calling \ref m_cbModIterNext()
   * until there are no more matches (auto-delete) or by calling \ref 
   * m_cbModIterDelete().
   *
   * \param eOver     What pattern to iterator over (see \ref BsModIterOver_T).
   * \param sPattern  Device or module URI pattern string.
   *
   * \return On success, returns the pointer to the allocacted module
   * iterator.\n
   * On failure, NULL is returned.
   */
  BsModIter_T *(*m_cbModIterFirst)(BsModIterOver_T eOver, const char *sPattern);

  /*!
   * \brief Get the next set of information given the iterator state.
   *
   * If a virtual connection match is found, an iterator data is updated.
   *
   * If no match is found, the iterator is auto-deleted.
   *
   * \param pIter     Initialized, allocated iterator.
   *
   * \return On success, returns the pointer to the iterator.\n
   * On failure, NULL is returned.
   */
  BsModIter_T *(*m_cbModIterNext)(BsModIter_T *pIter);

  /*!
   * \brief Delete the module iterator.
   *
   * \param pIter   Module iterator.
   */
   void (*m_cbModIterDelete)(BsModIter_T *pIter);

  // load a library - server keeps track - shares handle between modules
  //                  client get symbols it needs
} BsModProxyCb_T;


// ---------------------------------------------------------------------------
// Interface Module Logging, Error and Exception Responses
// ---------------------------------------------------------------------------

/*!
 * \brief Log Interface Module Warning.
 *
 * \param hndVConn  Virtual connection handle.
 * \param ecode     \h_botsense error code.
 * \param wfmt      Warning output format string literal.
 * \param ...       Warning variable arguments.    
 */
#define BSMOD_LOG_WARN(hndVConn, ecode, wfmt, ...) \
  LOGDIAG3("Warning: VConn=%d: %s(): %s(ecode=%d): " wfmt, \
      hndVConn, LOGFUNCNAME, \
      bsStrError(ecode), ((ecode)>=0? (ecode): -(ecode)), \
      ##__VA_ARGS__)

/*!
 * \brief Log Interface Module Error.
 *
 * \param hndVConn  Virtual connection handle.
 * \param ecode     \h_botsense error code.
 * \param efmt      Error output format string literal.
 * \param ...       Error variable arguments.    
 */
#define BSMOD_LOG_ERROR(hndVConn, ecode, efmt, ...) \
  LOGERROR("VConn=%d: %s(): %s(ecode=%d): " efmt, \
      hndVConn, LOGFUNCNAME, \
      bsStrError(ecode), (ecode>=0? ecode: -ecode), \
      ##__VA_ARGS__)

/*!
 * \brief Log Interface Module NetMsgs (Un)Packing Error.
 *
 * \param hndVConn  Virtual connection handle.
 * \param nmecode   NetMsgs error code.
 * \param efmt      Error output format string literal.
 * \param ...       Error variable arguments.    
 */
#define BSMOD_LOG_NMERROR(hndVConn, nmecode, efmt, ...) \
  BSMOD_LOG_ERROR(hndVConn, BS_ECODE_BAD_MSG, "%s(nmecode=%d): " efmt, \
      nmStrError(nmecode), (nmecode>=0? nmecode: -nmecode), \
      ##__VA_ARGS__)

/*!
 * \brief Log Interface Module System Error.
 *
 * \param hndVConn  Virtual connection handle.
 * \param efmt      Error output format string litteral.
 * \param ...       Error variable arguments.    
 */
#define BSMOD_LOG_SYSERROR(hndVConn, efmt, ...) \
  LOGERROR("VConn=%d: %s(): %s(ecode=%d): %s(errno=%d): " efmt, \
          hndVConn, LOGFUNCNAME, \
          bsStrError(BS_ECODE_SYS), BS_ECODE_SYS, \
          strerror(errno), errno, ##__VA_ARGS__)

/*!
 * \brief Log \h_botsense Error and Send Error Response.
 *
 * \param pCb       Pointer to server callbacks.
 * \param hndVConn  Virtual connection handle.
 * \param uTid      Request-Response transaction id.
 * \param ecode     \h_botsense error code.
 * \param efmt      Error output format string literal.
 * \param ...       Error variable arguments.    
 */
#define BSMOD_SEND_ERROR_RSP(pCb, hndVConn, uTid, ecode, efmt, ...) \
  do \
  { \
    BSMOD_LOG_ERROR(hndVConn, ecode, efmt, ##__VA_ARGS__); \
    (pCb)->m_cbSendErrorRsp(hndVConn, uTid, ecode, efmt, ##__VA_ARGS__); \
  } while(0)

/*!
 * \brief Log NetMsgs (Un)Packing Error and Send Error Response.
 *
 * \param pCb       Pointer to server callbacks.
 * \param hndVConn  Virtual connection handle.
 * \param uTid      Request-Response transaction id.
 * \param nmecode   NetMsgs error code.
 * \param efmt      Error output format string literal.
 * \param ...       Error variable arguments.    
 */
#define BSMOD_SEND_NMERROR_RSP(pCb, hndVConn, uTid, nmecode, efmt, ...) \
  do \
  { \
    BSMOD_LOG_NMERROR(hndVConn, nmecode, efmt, ##__VA_ARGS__); \
    (pCb)->m_cbSendErrorRsp(hndVConn, uTid, BS_ECODE_BAD_MSG, efmt, \
                          ##__VA_ARGS__); \
  } while(0)

/*!
 * \brief Log System Error and Send Error Response.
 *
 * \param pCb       Pointer to server callbacks.
 * \param hndVConn  Virtual connection handle.
 * \param uTid      Request-Response transaction id.
 * \param efmt      Error output format string literal.
 * \param ...       Error variable arguments.    
 */
#define BSMOD_SEND_SYSERROR_RSP(pCb, hndVConn, uTid, efmt, ...) \
  do \
  { \
    BSMOD_LOG_SYSERROR(hndVConn, efmt, ##__VA_ARGS__); \
    (pCb)->m_cbSendErrorRsp(hndVConn, uTid, BS_ECODE_SYS, \
        "%s(errno=%d): " efmt, \
        strerror(errno), errno, ##__VA_ARGS__); \
  } while(0)

/*!
 * \brief Check if handle is within the range of module handles.
 *
 * If the check is false, an appropriate error is logged and the calling
 * function is immediately exited by invoking a <b>return</b> with the
 * appropriate \h_lt 0 error code.
 *
 * \param hndVConn  Virtual connection handle.
 */
#define BSMOD_TRY_VCONN_HND_RANGE(hndVConn) \
  do \
  { \
    if( !BSMOD_IS_VCONN_HANDLE(hndVConn) ) \
    { \
      BSMOD_LOG_ERROR(hndVConn, BS_ECODE_BAD_VCONN_HND, \
          "Module vconn handle out-of-range."); \
      return -BS_ECODE_BAD_VCONN_HND; \
    } \
  } while(0)


// ---------------------------------------------------------------------------
// Module Exported Interface
// ---------------------------------------------------------------------------

/*!
 * \brief Initialize module.
 *
 * Called once after module is loaded.
 *
 * Global module initialization and resource allocation should be done here.
 *
 * \param sModUri     Expanded, canonical module path name.
 * \param pCallbacks  Pointer to a set of module -\> bsProxy core callback
 *                    functions.
 *
 * \copydoc doc_return_std
 */
typedef int (BsModInitFunc_T)(const char           *sModUri,
                              const BsModProxyCb_T *pCallbacks);
typedef BsModInitFunc_T *BsModInitFunc_P;   ///< pointer to init function

/*!
 * \brief Exit module.
 *
 * Called once prior to module being unloaded.
 *
 * Any module owned resources should be freed here. Any active handles 
 * associated opened devices should be closed.
 */
typedef void (BsModExitFunc_T)();
typedef BsModExitFunc_T *BsModExitFunc_P;   ///< pointer to exit function

/*!
 * \brief Open device controlled by module and associate with handle.
 *
 * Subsequent calls to the module use the given handle to associate the 
 * specific module-device instance.
 *
 * The argument buffer contains packed message arguements specific to the 
 * device and module.
 *
 * \param hndVConn      Virtual connection handle.
 * \param bTrace        Do [not] enable message tracing on this handle.
 * \param sDevUri       Expanded, canonical device path name.
 * \param argbuf        Packed specific open configuration arguments submessage.
 * \param uArgLen       Length of packed argumets in buffer (number of bytes).
 *
 * \return
 * On success, returns a unique resource descriptor \h_ge 0 which is typically
 * an opened file descriptor or socket descriptor, but can be module defined.
 * \copydoc doc_return_ecode
 */
typedef int (BsModOpenFunc_T)(BsVConnHnd_T hndVConn,
                              bool_t       bTrace,
                              const char  *sDevUri,
                              byte_t       argbuf[],
                              size_t       uArgLen);
typedef BsModOpenFunc_T *BsModOpenFunc_P;   ///< pointer to open function

/*!
 * \brief Close device controlled by module and disassociate handle.
 *
 * The handle instance specific resources should be freed.
 *
 * \param hndVConn  Virtual connection handle.
 *
 * \return
 * \copydoc doc_return_std
 */
typedef int (BsModCloseFunc_T)(BsVConnHnd_T hndVConn);
typedef BsModCloseFunc_T *BsModCloseFunc_P;   ///< pointer to close function

/*!
 * \brief Service client-specific request directed to this interface module.
 *
 * \note A module-specific request service must send a response.
 * 
 * \param hndVConn  Virtual connection handle.
 * \param uTid      Request-Response transaction id.
 * \param uMsgIdReq Request message id.
 * \param bufReq    Packed request message buffer.
 * \param uReqLen   Length of request in buffer (number of bytes).
 *
 * \copydoc doc_return_std
 */
typedef int (BsModRequestFunc_T)(BsVConnHnd_T hndVConn,
                                 BsTid_T      uTid,
                                 BsMsgId_T    uMsgIdReq,
                                 byte_t       bufReq[],
                                 size_t       uReqLen);
typedef BsModRequestFunc_T *BsModRequestFunc_P;   ///< pointer to request func.

/*!
 * \brief Enable/disable message tracing on handle.
 *
 * \param hndVConn  Virtual connection handle.
 * \param bTrace    Do [not] enable message tracing on this handle.
 *
 * \copydoc doc_return_std
 */
typedef int (BsModTraceFunc_T)(BsVConnHnd_T hndVConn, bool_t bTrace);
typedef BsModTraceFunc_T *BsModTraceFunc_P;   ///< pointer to msg trace func.

/*!
 * \brief Query for the static module information.
 *
 * \return
 * Pointer to module static information.
 */
typedef const BsModInfo_T *(BsModInfoFunc_T)();
typedef BsModInfoFunc_T *BsModInfoFunc_P;   ///< pointer to mod info function


//..............................................................................
// Module Prototypes
//..............................................................................

extern BsModInitFunc_T    BSMOD_SYM_INIT;     ///< init prototype
extern BsModExitFunc_T    BSMOD_SYM_EXIT;     ///< exit prototype
extern BsModOpenFunc_T    BSMOD_SYM_OPEN;     ///< open prototype
extern BsModCloseFunc_T   BSMOD_SYM_CLOSE;    ///< close prototype
extern BsModRequestFunc_T BSMOD_SYM_REQUEST;  ///< request prototype
extern BsModTraceFunc_T   BSMOD_SYM_TRACE;    ///< trace prototype
extern BsModInfoFunc_T    BSMOD_SYM_INFO;     ///< mod info prototype

C_DECLS_END


#endif // _BSPROXYMODIF_H
