////////////////////////////////////////////////////////////////////////////////
//
// Package:   BotSense
//
// Program:   bsProxy
//
// File:      bsProxyMod.c
//
/*! \file
 *
 * $LastChangedDate: 2010-08-20 11:36:38 -0600 (Fri, 20 Aug 2010) $
 * $Rev: 568 $
 *
 * \brief Proxied interface module management operations.
 *
 * \sa http://www.yolinux.com/TUTORIALS/LibraryArchives-StaticAndDynamic.html
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

#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
#include <dlfcn.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/new.h"
#include "rnr/hash.h"
#include "rnr/dliststr.h"
#include "rnr/path.h"

#include "botsense/BotSense.h"
#include "botsense/bsProxyModIF.h"
#include "botsense/bsProxyMsgs.h"

#include "bsProxy.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

#ifndef __windows__
# define DLL_EXT    ".so"       ///< standard dll library file name extension
#else
# define DLL_EXT    ".DLL"      ///< windows dll library file name extension
#endif

#ifndef BS_AUX_LIBDIR
#define BS_AUX_LIBDIR "/prj/lib" ///< additional bsProxy library directory
#endif

#ifndef BS_PLUGIN_DIR
#define BS_PLUGIN_DIR "."       ///< bsProxy module plugin subdirectory
#endif

#define BSPROXY_MOD_HASH_MIN    32  ///< minimum hash table size
#define BSPROXY_MOD_HASH_MAX   256  ///< maximum hash table size

static pthread_mutex_t  BsModMutex;       ///< bsProxy module operations mutex
static hash_t          *BsModHashTbl;     ///< i/f module hash table
static DListStr_T      *BsModDllPaths;    ///< module search paths

/*!
 * \brief Log Proxy Server Module Error.
 *
 * \param moduri    Module URI string.
 * \param ecode     \h_botsense error code.
 * \param efmt      Error output format string literal.
 * \param ...       Error variable arguments.    
 */
#define BSPROXY_MOD_LOG_ERROR(moduri, ecode, efmt, ...) \
  LOGERROR("%s: \"%s\": %s(ecode=%d): " efmt, \
      ServerHasName(), moduri, \
      bsStrError(ecode), (ecode>=0? ecode: -ecode), \
      ##__VA_ARGS__)


//.............................................................................
// Mutual Exclusion Functions
//.............................................................................

/*!
 * \brief Lock module's global mutual exclusion.
 */
static inline void ModLock()
{
  int rc;

  if( (rc = pthread_mutex_lock(&BsModMutex)) != 0 ) 
  { 
    errno = rc;
    LOGSYSERROR("pthread_mutex_lock()");
  }
}

/*!
 * \brief Unlock module's global mutual exclusion.
 */
static inline void ModUnlock()
{
  int rc;

  if( (rc = pthread_mutex_unlock(&BsModMutex)) != 0 ) 
  { 
    errno = rc;
    LOGSYSERROR("pthread_mutex_unlock()");
  }
}

/*!
 * \brief Try to lock module's global mutual exclusion.
 *
 * \return
 * Returns true if lock is acquired. Otherwise returns false if mutex already
 * locked.
 */
static inline bool_t ModTryLock()
{
  return pthread_mutex_trylock(&BsModMutex) == 0? true: false;
}


//.............................................................................
// Dynamic Link Loader Functions
//.............................................................................

/*!
 * Initialize Dynamic Link Loader operating parameters.
 *
 * \param pDListLibPath   List of optional library search paths.
 *
 * For i/f module search path order:
 * \li command-line -L options in order listed
 * \li XML configuration paths
 * \li user's home lib directory if it exists
 * \li \h_botsense library install library path
 * \li default dlopen() locations (system dependent)
 */
static void ModDllInit(DListStr_T *pDListLibPath)
{
  DListStrIter_T  iter;
  char           *sLibPath;
  char           *sPath;

  BsModDllPaths = DListStrNewDft();

  // Add command-line -L paths here
  for(sLibPath=DListStrIterDataFirst(pDListLibPath, &iter);
      sLibPath!=NULL;
      sLibPath=DListStrIterDataNext(&iter))
  {
    // canonicalize path name
    if( (sPath = NewSearchPathCanonicalized(sLibPath)) == NULL )
    {
      LOGDIAG2("Warning: \"%s\": Ignoring bad library path.", sLibPath);
    }
    else
    {
      DListStrAppend(BsModDllPaths, sPath);
    }
  }

  // RDK add xml config paths here

  // RDK add user's home/botsense/libs path here
 
  // Add installed prefix/lib/botsense/ path.
  if( (sPath = NewJoinedPath(BS_AUX_LIBDIR, BS_PLUGIN_DIR)) != NULL )
  {
    DListStrAppend(BsModDllPaths, sPath);
  }
}

/*!
 * \brief Make DLL canonical file name.
 *
 * The library path is expanded, canonicalized and any necessary file extension is 
 * appended.
 *
 * \param  sModName   DLL library path name.
 *
 * \return
 * On success, an allocated, canonical library path name is returned.\n
 * On failure, NULL is returned.
 */
static char *ModDllNewCanonicalName(const char *sModName)
{
  char   *s = NULL;
  char   *t = NULL;
  size_t  m, n;

  // canonicalize module path name
  if( (s = NewSearchPathCanonicalized(sModName)) == NULL )
  {
    LOGERROR("NewSearchPathCanonicalized(\"%s\") failed.", sModName);
    return NULL;
  }

  m = strlen(s);
  n = strlen(DLL_EXT);

  // append library extension
  if( (DLL_EXT != NULL) && ((m <= n) || strncasecmp(s+m-n, DLL_EXT, n)) )
  {
    t = NEWSTR(m+n);
    sprintf(t, "%s%s", s, DLL_EXT);
    delete(s);
  }
  else
  {
    t = s;
  }

  return t;
}

/*!
 * \brief Dynamically load bsProxy server compatible interface module library.
 *
 * If the module path name is absolute, the only library at that location is
 * attempted to be loaded. Otherwise, a series of path locations are searched for
 * the library.
 *
 * \note The dlopen() and dlclose() functions only load/unload when a library 
 * iff the internal reference count is zero. However, bsProxy also needs to track
 * the reference count, which the dl interface does not provide access to.
 *
 * \param sModUri   Expanded, canonical name of the interface module.
 *
 * \return
 * On success, returns DLL handle.\n
 * On failure, returns NULL.
 */
static void *ModDllOpen(const char *sModUri)
{
  DListStrIter_T  iter;             ///< paths dlist iterator
  char           *sPath;            ///< a dll path
  char           *sFqPath;          ///< fully qualified path name
  void           *dllHandle = NULL; ///< loaded dll handle

  if( sModUri == NULL )
  {
    return NULL;
  }

  // clear any existing error
  dlerror();

  // 
  // Module path name is an absolute path. No search is made, simply let
  // dlopen() do the work.
  //
  if( PathIsAbsolute(sModUri) )
  {
    if( (sFqPath = NewRealPath(sModUri)) != NULL )
    {
      LOGDIAG3("%s: trying dlopen(\"%s\").", ServerHasName(), sFqPath);
      dllHandle = dlopen(sFqPath, RTLD_LAZY | RTLD_GLOBAL);
      delete(sFqPath);
    }
  }

  //
  // Search list of paths for the module's location and load.
  //
  else
  {
    for(sPath=DListStrIterDataFirst(BsModDllPaths, &iter), dllHandle=NULL;
        sPath!=NULL && dllHandle==NULL;
        sPath=DListStrIterDataNext(&iter))
    {
      if( (sFqPath = NewJoinedPath(sPath, sModUri)) != NULL )
      {
        LOGDIAG3("%s: trying dlopen(\"%s\").", ServerHasName(), sFqPath);
        dllHandle = dlopen(sFqPath, RTLD_LAZY | RTLD_GLOBAL);
        delete(sFqPath);
      }
    }

    //
    // Use dlopen() default search.
    //
    if( dllHandle == NULL )
    {
      LOGDIAG3("%s: trying dlopen(\"%s\").", ServerHasName(), sModUri);
      dllHandle = dlopen(sModUri, RTLD_LAZY | RTLD_GLOBAL);
    }
  }

  if( dllHandle == NULL )
  {
    BSPROXY_MOD_LOG_ERROR(sModUri, BS_ECODE_NO_MOD, "%s.", dlerror());
  }

  return dllHandle;
}

/*!
 * \brief Unload the dynamic library from the application. 
 *
 * \note The dlopen() and dlclose() functions only load/unload when a library 
 * iff the internal reference count is zero. However, bsProxy also needs to track
 * the reference count, which the dl interface does not provide access to.
 *
 * \param sModUri     Expanded, canonical name of the interface module.
 * \param dllHandle   DLL handle.
 */
static void ModDllClose(const char *sModUri, void *dllHandle)
{
  // clear any existing error
  dlerror();

  if( dlclose(dllHandle) != 0 )
  {
    BSPROXY_MOD_LOG_ERROR(sModUri, BS_ECODE_NO_EXEC, "%s.", dlerror());
  }
}

/*!
 * \brief Get the address of where the module symbol is loaded into memory.
 *
 * \param pModIF  Exported module interface.
 * \param sSym    Symbol name.
 *
 * \return
 * On success, returns the symbol address.\n
 * On failure, NULL is returned.
 */ 
static void *ModDllSym(BsProxyModIF_T *pModIF, const char *sSym)
{
  void  *pSym;
  char  *sEMsg;

  // clear any existing error
  dlerror();

  // get symbol address
  pSym = dlsym(pModIF->m_dllHandle, sSym);

  // error
  if( pSym == NULL )
  {
    sEMsg = dlerror();
    if( sEMsg == NULL )
    {
      sEMsg = "Not defined";
    }
    BSPROXY_MOD_LOG_ERROR(pModIF->m_sModUri, BS_ECODE_BAD_MOD,
                          "%s: %s.", sSym, sEMsg);
  }

  return pSym;
}

/*!
 * \brief Delete module interface block.
 *
 * \param pModIF   Exported module interface.
 */
static void ModDllDeleteIF(BsProxyModIF_T *pModIF)
{
  if( pModIF != NULL )
  {
    delete((char *)pModIF->m_sModUri);
    delete(pModIF);
  }
}

/*!
 * \brief Allocate an attached exported module interface block.
 *
 * \note dlsym() can return NULL with no errors if the symbol's value is NULL.
 * Checking the dlerror() return is the safe way to determine if an error has
 * occurred. However, a bsProxy compatible module's exported interface is
 * required to be defined.
 *
 * \param sModUri     Expanded, canonical name of the interface module.
 * \param dllHandle   DLL handle.
 *
 * \return
 * On success, returns a pointer to the module's exported interface.\n
 * On failure, returns NULL.
 */
static BsProxyModIF_T *ModDllNewIF(const char *sModUri, void *dllHandle)
{
  BsProxyModIF_T *pModIF = NEW(BsProxyModIF_T);
  int             rc = BS_OK;

  pModIF->m_sModUri   = new_strdup(sModUri);  // self reference
  pModIF->m_dllHandle = dllHandle;            // opaque dll handle
  pModIF->m_uRefCnt   = 1;                    // module reference count

  //
  // Init
  //
  pModIF->m_fnModInit = (BsModInitFunc_P)ModDllSym(pModIF, BSMOD_SYM_INIT_S);

  if( pModIF->m_fnModInit == NULL )
  {
    rc = -BS_ECODE_BAD_MOD;
  }

  //
  // Exit
  //
  pModIF->m_fnModExit = (BsModExitFunc_P)ModDllSym(pModIF, BSMOD_SYM_EXIT_S);

  if( pModIF->m_fnModExit == NULL )
  {
    rc = -BS_ECODE_BAD_MOD;
  }

  //
  // Open
  //
  pModIF->m_fnModOpen = (BsModOpenFunc_P)ModDllSym(pModIF, BSMOD_SYM_OPEN_S);

  if( pModIF->m_fnModOpen == NULL )
  {
    rc = -BS_ECODE_BAD_MOD;
  }

  //
  // Close
  //
  pModIF->m_fnModClose = (BsModCloseFunc_P)ModDllSym(pModIF, BSMOD_SYM_CLOSE_S);

  if( pModIF->m_fnModClose == NULL )
  {
    rc = -BS_ECODE_BAD_MOD;
  }


  //
  // Request
  //
  pModIF->m_fnModRequest = (BsModRequestFunc_P)ModDllSym(pModIF,
                                                         BSMOD_SYM_REQUEST_S);

  if( pModIF->m_fnModRequest == NULL )
  {
    rc = -BS_ECODE_BAD_MOD;
  }

  //
  // Trace
  //
  pModIF->m_fnModTrace = (BsModTraceFunc_P)ModDllSym(pModIF, BSMOD_SYM_TRACE_S);

  if( pModIF->m_fnModTrace == NULL )
  {
    rc = -BS_ECODE_BAD_MOD;
  }

  //
  // Info
  //
  pModIF->m_fnModInfo = (BsModInfoFunc_P)ModDllSym(pModIF, BSMOD_SYM_INFO_S);

  if( pModIF->m_fnModInfo == NULL )
  {
    rc = -BS_ECODE_BAD_MOD;
  }

  // Error clean up.
  if( rc < 0 )
  {
    ModDllDeleteIF(pModIF);
    pModIF = NULL;
  }

  return pModIF;
}


//.............................................................................
// I/F Module Hashing Functions
//.............................................................................

/*!
 * \brief Delete hash node data callback.
 *
 * Both the key and value are dynamically allocated.
 *
 * \param sModUri   Interface module hash table key.
 * \param pModIF    Interface module hash table value.
 */
static void ModHashDeleteData(void *sModUri, void *pModIF)
{
  delete(sModUri);
  ModDllDeleteIF((BsProxyModIF_T *)pModIF);
}

/*!
 * \brief Add the module data to the i/f module hash table.
 *
 * \param sModUri   Allocated interface module hash table key.
 * \param pModIF    Allocated interface module hash table value.
 *
 * \copydoc doc_return_std
 */
static int ModHashAdd(const char *sModUri, BsProxyModIF_T *pModIF)
{
  char *sKey = new_strdup(sModUri);

  // insert into hash table, automatically allocating hnode_t
  if( !hash_insert(BsModHashTbl, sKey, pModIF) )
  {
    LOGERROR("hash_insert(%s,...) failed", sKey);
    ModHashDeleteData(sKey, pModIF);
    return -BS_ECODE_NO_RSRC;
  }

  return BS_OK;
}

/*!
 * \brief Remove and delete the module data from the i/f module hash table.
 *
 * \param pModIF    Allocated interface module hash table value.
 *
 * \copydoc doc_return_std
 */
static void ModHashDelete(BsProxyModIF_T *pModIF)
{
  hnode_t *pNode;

  if( pModIF == NULL )
  {
    return;
  }
  else if( (pNode = hash_lookup(BsModHashTbl, pModIF->m_sModUri)) != NULL )
  {
    hash_node_delete(BsModHashTbl, pNode);
  }
}

/*!
 * \brief Get the interface module's i/f from the module hash table.
 *
 * \param sModUri   Interface module hash table key.
 *
 * \return
 * On success, returns pointer to the module's exported interface.\n
 * On failure, returns NULL.
 */
static BsProxyModIF_T *ModHashGetIF(const char *sModUri)
{
  hnode_t         *pNode;

  if( (pNode = hash_lookup(BsModHashTbl, sModUri)) == NULL )
  {
     return NULL;
  }
  else
  {
    return (BsProxyModIF_T *)hnode_get(pNode);
  }
}


// ---------------------------------------------------------------------------
// Interface Module to bsProxy Server Callback Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Send module-specific repsonse callback function.
 *
 * \note Service thread request handler has already locked the virtual
 * connection.
 *
 * \param hndVConn  Virtual connection handle.
 * \param uTid      Request-Response transaction id.
 * \param uMsgId    Response message id.
 * \param bufRsp    Packed repsonse message body.
 * \param uRspLen   Length of response in buffer (number of bytes).
 */
void ModCbSendRsp(BsVConnHnd_T hndVConn,
                  BsTid_T      uTid,
                  BsMsgId_T    uMsgId,
                  byte_t       bufRsp[],
                  size_t       uRspLen)
{
  BsProxyVConn_T  *pVConn;

  if( (pVConn = VConnGet(hndVConn)) != NULL )
  {
    ClientSendRsp(pVConn->m_hndClient, hndVConn, uTid, uMsgId, bufRsp, uRspLen);
  }
  else
  {
    LOGERROR("%s: %s(ecode=%d): VConn=%d.",
      ServerHasName(), bsStrError(BS_ECODE_NO_VCONN), BS_ECODE_NO_VCONN,
      hndVConn);
  }
}

/*!
 * \brief Send generic ok repsonse callback function.
 *
 * \note Service thread request handler has already locked the virtual
 * connection.
 *
 * \param hndVConn  Virtual connection handle.
 * \param uTid      Request-Response transaction id.
 */
void ModCbSendOkRsp(BsVConnHnd_T hndVConn, BsTid_T uTid)
{
  BsProxyVConn_T  *pVConn;

  if( (pVConn = VConnGet(hndVConn)) != NULL )
  {
    ClientSendOkRsp(pVConn->m_hndClient, uTid);
  }
  else
  {
    LOGERROR("%s: %s(ecode=%d): VConn=%d.",
      ServerHasName(), bsStrError(BS_ECODE_NO_VCONN), BS_ECODE_NO_VCONN,
      hndVConn);
  }
}

/*!
 * \brief Send generic error repsonse callback function.
 *
 * \note Service thread request handler has already locked the virtual
 * connection.
 *
 * \param hndVConn  Virtual connection handle.
 * \param uTid      Request-Response transaction id.
 * \param nECode    \copydoc doc_param_ecode
 * \param sErrFmt   Error format string.
 * \param ...       Format string variable arguments.
 */
void ModCbSendErrorRsp(BsVConnHnd_T hndVConn,
                       BsTid_T      uTid,
                       int          nECode,
                       const char  *sErrFmt,
                       ...)
{
  BsProxyVConn_T *pVConn;
  va_list         ap;

  if( (pVConn = VConnGet(hndVConn)) != NULL )
  {
    va_start(ap, sErrFmt);
    ClientSendVErrorRsp(pVConn->m_hndClient, uTid, nECode, sErrFmt, ap);
    va_end(ap);
  }
  else
  {
    LOGERROR("%s: %s(ecode=%d): VConn=%d.",
      ServerHasName(), bsStrError(BS_ECODE_NO_VCONN), BS_ECODE_NO_VCONN,
      hndVConn);
  }
}

/*!
 * \brief Allocatae a new module resource table of fixed size.
 *
 * The resource table is indexed by the virtual connection handle.
 *
 * \param nMaxResources   Maximum number of simultaneous resources supported.
 *
 * \return Pointer to allocated resource block.
 */
BsModRsrcTbl_T *ModCbRsrcTblNew(int nMaxResources)
{
  BsModRsrcTbl_T *pRsrcTbl = NEW(BsModRsrcTbl_T);

  pRsrcTbl->m_vecRsrc = new(sizeof(void *) * (size_t)nMaxResources);

  memset(pRsrcTbl->m_vecIndex, BSPROXY_VCONN_UNDEF,
      sizeof(pRsrcTbl->m_vecIndex));

  pRsrcTbl->m_uMaxResources = (uint_t)nMaxResources;
  pRsrcTbl->m_uInUseCount   = 0;

  return pRsrcTbl;
}

/*!
 * \brief Delete an allocated resource table.
 *
 * \warning Module-specific resources should be freed up prior to calling this
 * function.
 *
 * \param pRsrcTbl  Pointer to resource table.
 */
void ModCbRsrcTblDelete(BsModRsrcTbl_T *pRsrcTbl)
{
  if( pRsrcTbl != NULL )
  {
    delete(pRsrcTbl->m_vecRsrc);
    delete(pRsrcTbl);
  }
}

/*!
 * \brief Add a new resource to the resource table.
 *
 * \param pRsrcTbl  Pointer to resource table.
 * \param hndVConn  Virtual connection handle.
 * \param pRsrc     Pointer to allocated resource associated with handle.
 *
 * \copydoc doc_return_std
 */
int ModCbRsrcAdd(BsModRsrcTbl_T *pRsrcTbl, BsVConnHnd_T hndVConn, void *pRsrc)
{
  int  index;

  // bad handle
  if( !BSMOD_IS_VCONN_HANDLE(hndVConn) )
  {
    return -BS_ECODE_BAD_VCONN_HND;
  }

  // no more resources available  
  else if( pRsrcTbl->m_uInUseCount >= pRsrcTbl->m_uMaxResources )
  {
    return -BS_ECODE_NO_RSRC;
  }

  // find empty slot in resource table
  for(index=0; index<pRsrcTbl->m_uMaxResources; ++index)
  {
    if( pRsrcTbl->m_vecRsrc[index] == NULL )
    {
      break;
    }
  }

  // internal error - log these
  if( (uint_t)index >= pRsrcTbl->m_uMaxResources )
  {
    LOGERROR("Internal module data corruption.");
    return -BS_ECODE_INTERNAL;
  }

  // add resource to resource table
  pRsrcTbl->m_vecRsrc[index]     = pRsrc;
  pRsrcTbl->m_vecIndex[hndVConn] = (byte_t)index;
  pRsrcTbl->m_uInUseCount++;

  return index;
}

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
void *ModCbRsrcRemove(BsModRsrcTbl_T *pRsrcTbl, BsVConnHnd_T hndVConn)
{
  int   index;
  void *pRsrc;

  // bad handle
  if( !BSMOD_IS_VCONN_HANDLE(hndVConn) )
  {
    return NULL;
  }

  // get resource index
  else if( (index = BSMOD_RSRC_INDEX(pRsrcTbl, hndVConn)) < 0 )
  {
    return NULL;
  }

  // removed resource
  pRsrc = pRsrcTbl->m_vecRsrc[index];

  // remove resource to resource table
  pRsrcTbl->m_vecRsrc[index]     = NULL;
  pRsrcTbl->m_vecIndex[hndVConn] = (byte_t)BSPROXY_VCONN_UNDEF;
  pRsrcTbl->m_uInUseCount--;

  return pRsrc;
}

/*!
 * \brief Copy the device URI associated with the virtual connection.
 *
 * \param hndVConn    Virtual connection handle.
 * \param [out] dest  Destination buffer.
 * \param n           Size of buffer.
 *
 * \return Pointer to destination buffer dest.
 */
const char *ModCbGetDevUri(BsVConnHnd_T hndVConn, char dest[], size_t n)
{
  BsProxyVConn_T  *pVConn;

  if( (pVConn = VConnAcquire(hndVConn)) != NULL )
  {
    strncpy(dest, pVConn->m_pThCtl->m_sDevUri, n);
    dest[n-1] = 0;
    VConnRelease(hndVConn);
  }
  else
  {
    dest[0] = 0;
  }

  return dest;
}

/*!
 * \brief Delete the module iterator.
 *
 * \param pIter   Module iterator.
 */
void ModCbIterDelete(BsModIter_T *pIter)
{
  if( pIter != NULL )
  {
    delete(pIter->m_sDevUri);
    delete(pIter->m_sModUri);
    delete(pIter);
  }
}

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
BsModIter_T *ModCbIterNext(BsModIter_T *pIter)
{
  BsVConnHnd_T    hndVConn;
  BsProxyVConn_T *pVConn;
  bool_t          bHasMatch = false;

  if( pIter == NULL )
  {
    return NULL;
  }

  for(hndVConn=pIter->m_hndVConn+1;
      (pIter!=NULL) && (hndVConn<BSPROXY_VCONN_NUMOF);
      ++hndVConn)
  {
    if( (pVConn = VConnAcquire(hndVConn)) != NULL )
    {
      switch( pIter->m_eOver )
      {
        case BsModIterOverDevUri:
          if( !strcmp(pVConn->m_pThCtl->m_sDevUri, pIter->m_sDevUri) )
          {
            pIter->m_hndVConn = hndVConn;
            delete(pIter->m_sModUri);
            pIter->m_sModUri = new_strdup(pVConn->m_pModIF->m_sModUri);
            pIter->m_rd = pVConn->m_rd;
            bHasMatch = true;
          }
          break;
        case BsModIterOverModUri:
          if( !strcmp(pVConn->m_pModIF->m_sModUri, pIter->m_sModUri) )
          {
            pIter->m_hndVConn = hndVConn;
            delete(pIter->m_sDevUri);
            pIter->m_sDevUri = new_strdup(pVConn->m_pThCtl->m_sDevUri);
            pIter->m_rd = pVConn->m_rd;
            bHasMatch = true;
          }
          break;
        case BsModIterOverVConn:
            pIter->m_hndVConn = hndVConn;
            delete(pIter->m_sDevUri);
            pIter->m_sDevUri = new_strdup(pVConn->m_pThCtl->m_sDevUri);
            delete(pIter->m_sModUri);
            pIter->m_sModUri = new_strdup(pVConn->m_pModIF->m_sModUri);
            pIter->m_rd = pVConn->m_rd;
            bHasMatch = true;
          break;
        default:
          ModCbIterDelete(pIter);
          pIter = NULL;
          break;
      }
      VConnRelease(hndVConn);
    }
  }

  if( !bHasMatch )
  {
    ModCbIterDelete(pIter);
    pIter = NULL;
  }

  return pIter;
}

/*!
 * \brief Start an iterator over the virtual connections matching the given 
 * pattern.
 *
 * If a virtual connection match is found, an iterator is allocated and its data
 * is filled with the first set of information.
 *
 * The caller must delete the iterator by calling \ref ModCbIterNext() until
 * there are no more matches (auto-delete) or by calling \ref ModCbIterDelete().
 *
 * \param eOver       What pattern to iterator over (see \ref BsModIterOver_T).
 * \param sPattern    Device or module URI pattern string.
 *
 * \return On success, returns the pointer to the allocacted module iterator.\n
 * On failure, NULL is returned.
 */
BsModIter_T *ModCbIterFirst(BsModIterOver_T eOver, const char *sPattern)
{
  BsModIter_T  *pIter = NEW(BsModIter_T);

  pIter->m_eOver    = eOver;
  pIter->m_hndVConn = -1;
  pIter->m_sDevUri  = NULL;
  pIter->m_sModUri  = NULL;
  pIter->m_rd       = -1;

  switch( eOver )
  {
    case BsModIterOverDevUri:
      pIter->m_sDevUri = new_strdup(sPattern);
      return ModCbIterNext(pIter);
    case BsModIterOverModUri:
      pIter->m_sModUri = new_strdup(sPattern);
      return ModCbIterNext(pIter);
    case BsModIterOverVConn:
      return ModCbIterNext(pIter);
    default:
      ModCbIterDelete(pIter);
      return NULL;
  }
}

/*!
 * \brief Interface module callbacks to bsProxy server services.
 */
static BsModProxyCb_T BsModProxyCallbacks =
{
  ModCbSendRsp,         ///< send module-specific response
  ModCbSendOkRsp,       ///< send standard ok response
  ModCbSendErrorRsp,    ///< send standard error response
  ModCbRsrcTblNew,      ///< new resource table
  ModCbRsrcTblDelete,   ///< delete resource table
  ModCbRsrcAdd,         ///< add a new resource
  ModCbRsrcRemove,      ///< remove a resource
  ModCbGetDevUri,       ///< get virtual connection's device URI
  ModCbIterFirst,       ///< start module iterator over virtual connections 
  ModCbIterNext,        ///< get next module iteration instance
  ModCbIterDelete       ///< delete module iterator
};


// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief The bsProxy interface module handling one-time initialization.
 *
 * \param pDListLibPath   List of optional library search paths.
 */
void ModOneTimeInit(DListStr_T *pDListLibPath)
{
  // create mutex
  pthread_mutex_init(&BsModMutex, NULL);
  
  BsModHashTbl = hash_table_create(
        true,                               // dynamic table sizing
        (hashcount_t)BSPROXY_MOD_HASH_MIN,  // minimum size
        (hashcount_t)BSPROXY_MOD_HASH_MAX,  // maximum size
        NULL,                               // use default comparator function
        NULL,                               // use default hashing function
        ModHashDeleteData);                 // hash node data deletion function

  // turn off most asserts()
  hash_set_self_verify(false);

  // initialize DLL environment
  ModDllInit(pDListLibPath);
}

/*!
 * \brief Convert the module name to a quasi Uniform Resource Id.
 *
 * \param sModName  Module path name.
 *
 * \return
 * On success, an allocated, canonical library path name is returned.\n
 * On failure, NULL is returned.
 */
char *ModNewModUri(const char *sModName)
{
  return ModDllNewCanonicalName(sModName);
}

/*!
 * \brief Get the defined interface from a previously loaded i/f module.
 *
 * Multi-Thread safe.
 *
 * \param sModUri   Interface module expanded, canonical name.
 *
 * \return
 * On success, returns a pointer to the module's exported interface.\n
 * Returns NULL if no interface is found.
 */
BsProxyModIF_T *ModGetLoadedIF(const char *sModUri)
{
  BsProxyModIF_T  *pModIF;

  ModLock();

  pModIF = ModHashGetIF(sModUri);

  ModUnlock();

  return pModIF;
}

/*!
 * \brief Load the interface module.
 *
 * The module's expanded, canonical name is determined, the dynamic library is
 * loaded, the module's exported interface is attached, and the module's 
 * initialization routine is called.
 *
 * The the same module is loaded again, the reference count is bumped and the
 * already attached interface is returned.
 *
 * Multi-Thread safe.
 *
 * \param sModUri   Interface module expanded, canonical name.
 *
 * \return
 * On success, returns a pointer to the module's exported interface.\n
 * On failure, returns NULL.
 */
BsProxyModIF_T *ModLoad(const char *sModUri)
{
  BsProxyModIF_T *pModIF      = NULL;
  void           *dllHandle   = NULL;
  int             rc;

  // lock global mutex
  ModLock();

  //
  // Check if module is already loaded.
  //
  if( (pModIF = ModHashGetIF(sModUri)) != NULL )
  {
    rc = BS_OK;
    pModIF->m_uRefCnt++;
    LOGDIAG1("Interface Module \"%s\" attached (refcnt=%u).",
          pModIF->m_sModUri, pModIF->m_uRefCnt);
  }

  //
  // Load the dynamic library.
  //
  else if( (dllHandle = ModDllOpen(sModUri)) == NULL )
  {
    rc = -BS_ECODE_NO_MOD;
    BSPROXY_MOD_LOG_ERROR(sModUri, rc, "Failed to open.");
  }

  //
  // Allocate new interface control blcok and attach module's exported
  // interface.
  //
  else if( (pModIF = ModDllNewIF(sModUri, dllHandle)) == NULL )
  {
    rc = -BS_ECODE_BAD_MOD;
    BSPROXY_MOD_LOG_ERROR(sModUri, rc, "Failed to get interface.");
  }

  //
  // Call the newly loaded module's exported initialization function to
  // initialize any module specific data and resources.
  //
  else if( (rc = pModIF->m_fnModInit(sModUri, &BsModProxyCallbacks)) < 0 )
  {
    rc = -BS_ECODE_BAD_MOD;
    BSPROXY_MOD_LOG_ERROR(sModUri, rc, "Failed to initialize.");
  }

  //
  // Add the module to the hash table (failure auto-deletes node data).
  //
  else if( (rc = ModHashAdd(sModUri, pModIF)) < 0 )
  {
    rc = -BS_ECODE_NO_RSRC;
    BSPROXY_MOD_LOG_ERROR(sModUri, rc, "Failed to add module to hash table.");
  }

  //
  // Successfully loaded new module.
  //
  else
  {
    rc = BS_OK;
    LOGDIAG1("Interface Module \"%s\" loaded (refcnt=%u).",
          pModIF->m_sModUri, pModIF->m_uRefCnt);
  }

  ModUnlock();

  //
  // Error clean up.
  //
  if( rc < 0 )
  {
    if( dllHandle != NULL )
    {
      dlclose(dllHandle);
      dllHandle = NULL;
    }

    if( pModIF != NULL )
    {
      ModDllDeleteIF(pModIF);
      pModIF = NULL;
    }
  }

  return pModIF;
}

/*!
 * \brief Unload the interface module.
 *
 * The module's reference count is decremented. If the count is zero, then the
 * module's exit routine is called and the module dynamic library is unloaded
 * from bsProxy server application.
 *
 * Multi-Thread safe.
 *
 * \param pModIF   Exported module interface.
 */
void ModUnload(BsProxyModIF_T *pModIF)
{

  ModLock();

  if( (pModIF = ModHashGetIF(pModIF->m_sModUri)) != NULL )
  {
    if( pModIF->m_uRefCnt > 0 )
    {
      pModIF->m_uRefCnt--;
    }

    if( pModIF->m_uRefCnt == 0 )
    {
      // call module specific exported exit function
      pModIF->m_fnModExit();

      // unload the dll
      ModDllClose(pModIF->m_sModUri, pModIF->m_dllHandle);

      LOGDIAG1("Interface Module \"%s\" unloaded (refcnt=%u).",
          pModIF->m_sModUri, pModIF->m_uRefCnt);

      // delete data
      ModHashDelete(pModIF);
    }
    else
    {
      LOGDIAG1("Interface Module \"%s\" detached (refcnt=%u).",
          pModIF->m_sModUri, pModIF->m_uRefCnt);
    }
  }

  ModUnlock();
}
