////////////////////////////////////////////////////////////////////////////////
//
// Package:   BotSense
//
// Program:   bsProxy
//
// File:      bsProxyThread.c
//
/*! \file
 *
 * $LastChangedDate: 2010-08-20 11:36:38 -0600 (Fri, 20 Aug 2010) $
 * $Rev: 568 $
 *
 * \brief \h_botsense bsProxy service threads.
 *
 * A Sevice thread provides a separate execution context to service client
 * request-response exchanges. There are two types of service threads:
 * \termblock
 * \term Server Thread
 * \termdata 1 thread to service server-terminated requests.
 * \endterm
 * \term Device Thread
 * \termdata N device threads, one attached to each URI unique, opened device.
 * \endterm
 * \endtermblock
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 * (C) 2007-2010.  RoadNarrows LLC.
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/new.h"
#include "rnr/hash.h"
#include "rnr/dlistvoid.h"
#include "rnr/path.h"

#include "botsense/BotSense.h"
#include "botsense/libBotSense.h"
#include "botsense/bsProxyMsgs.h"

#include "bsProxy.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

#define BSPROXY_TH_HASH_MIN       8           ///< minimum hash table size
#define BSPROXY_TH_HASH_MAX       256         ///< maximum hash table size
#define BSPROXY_TH_MAX_QUEUE_SIZE 8           ///< request queue maximum depth
#define BSPROXY_TH_SERVER_URI     "#SERVER"   ///< server "URI"


static pthread_mutex_t  BsThGlobalMutex; ///< bsProxy module operations mutex
static hash_t          *BsThHashTbl;     ///< proxied device hash table

/*!
 * \brief Log Proxy Server Thread Error.
 *
 * \param devuri    Device URI string.
 * \param ecode     \h_botsense error code.
 * \param efmt      Error output format string literal.
 * \param ...       Error variable arguments.    
 */
#define BSPROXY_TH_LOG_ERROR(devuri, ecode, efmt, ...) \
  LOGERROR("%s: Service Thread \"%s\": %s(ecode=%d): " efmt, \
      ServerHasName(), devuri, \
      bsStrError(ecode), (ecode>=0? ecode: -ecode), \
      ##__VA_ARGS__)


//.............................................................................
// Service Thread Control Block Functions
//.............................................................................

/*!
 * \brief Queue callback to compare two queue entries.
 *
 * \param pData1  Pointer to queue entry 1.
 * \param pData2  Pointer to queue entry 2.
 *
 * \return Returns \h_lt 0, 0, or \h_gt 0 if data 1 is less than, equal to, or
 * greater than data2, respectively.
 */
static int ThQReqCmp(const void *pData1, const void *pData2)
{
  BsProxyThReq_T *pThReq1 = (BsProxyThReq_T *)pData1;
  BsProxyThReq_T *pThReq2 = (BsProxyThReq_T *)pData2;
  BsMsgUid_T      uid1 = BSPROXY_MAKE_MSGUID(pThReq1->m_hndVConn,
                                             pThReq1->m_uMsgId);
  BsMsgUid_T      uid2 = BSPROXY_MAKE_MSGUID(pThReq2->m_hndVConn,
                                             pThReq2->m_uMsgId);

  return (int)uid1 - (int)uid2;
}

/*!
 * \brief Queue callback to delete queue entry.
 *
 * \param pData   Pointer to queue entry.
 */
static void ThQReqDelete(void *pData)
{
  ThReqDelete((BsProxyThReq_T *)pData);
}

/*!
 * \brief Allocate new service thread control block.
 *
 * \param sDevUri   Device URI.
 *
 * \return Pointer to allocated thread control block.
 */
static BsProxyThCtl_T *ThCtlBlkNew(const char *sDevUri)
{
  BsProxyThCtl_T *pThCtl = NEW(BsProxyThCtl_T);

  pThCtl->m_sDevUri = new_strdup(sDevUri);
  pThCtl->m_uRefCnt = 0;
  pThCtl->m_eState  = BsProxyThStateUninit;
  pThCtl->m_queue   = DListVoidNew(ThQReqCmp, ThQReqDelete);

  // create mutex
  pthread_mutex_init(&pThCtl->m_mutexSync, NULL);

  // create condition
  pthread_cond_init(&pThCtl->m_condSync, NULL);

  return pThCtl;
}

/*!
 * \brief Delete service thread control block.
 *
 * \param pThCtl  Pointer to thread control block.
 */
static void ThCtlBlkDelete(BsProxyThCtl_T *pThCtl)
{
  if( pThCtl != NULL )
  {
    // delete uri
    delete((char *)pThCtl->m_sDevUri);

    // delete request queue
    DListVoidDelete(pThCtl->m_queue);

    // destroy condition
    pthread_cond_destroy(&pThCtl->m_condSync);

    // destroy mutex
    pthread_mutex_destroy(&pThCtl->m_mutexSync);

    // delete self
    delete(pThCtl);
  }
}


//.............................................................................
// Mutual Exclusion Functions
//.............................................................................

/*!
 * \brief Lock global mutual exclusion.
 */
static inline void ThGlobalLock()
{
  int rc;

  if( (rc = pthread_mutex_lock(&BsThGlobalMutex)) != 0 )
  { 
    errno = rc;
    LOGSYSERROR("Thread global mutex: pthread_mutex_lock()");
  }
}

/*!
 * \brief Unlock global mutual exclusion.
 */
static inline void ThGlobalUnlock()
{
  int rc;

  if( (rc = pthread_mutex_unlock(&BsThGlobalMutex)) != 0 )
  { 
    errno = rc;
    LOGSYSERROR("Thread global mutex: pthread_mutex_unlock()");
  }
}

/*!
 * \brief Try to lock global mutual exclusion.
 *
 * \return
 * Returns true if lock is acquired. Otherwise returns false if mutex already
 * locked.
 */
static inline bool_t ThGlobalTryLock()
{
  return pthread_mutex_trylock(&BsThGlobalMutex) == 0? true: false;
}

/*!
 * \brief Lock service thread's mutual exclusion.
 *
 * \param pThCtl    Service thread control.
 */
static inline void ThSyncLock(BsProxyThCtl_T *pThCtl)
{
  int rc;

  if( (rc = pthread_mutex_lock(&pThCtl->m_mutexSync)) != 0 )
  { 
    errno = rc;
    LOGSYSERROR("Service Thread \"%s\": pthread_mutex_lock()",
        pThCtl->m_sDevUri);
  }
}

/*!
 * \brief Unlock service thread's mutual exclusion.
 *
 * \param pThCtl    Service thread control.
 */
static inline void ThSyncUnlock(BsProxyThCtl_T *pThCtl)
{
  int rc;

  if( (rc = pthread_mutex_unlock(&pThCtl->m_mutexSync)) != 0 )
  { 
    errno = rc;
    LOGSYSERROR("Service Thread \"%s\": pthread_mutex_unlock()",
        pThCtl->m_sDevUri);
  }
}

/*!
 * \brief Signal service thread's condition.
 *
 * \param pThCtl    Service thread control.
 */
static inline void ThSyncSignal(BsProxyThCtl_T *pThCtl)
{
  int rc;

  if( (rc = pthread_cond_signal(&pThCtl->m_condSync)) != 0 )
  { 
    errno = rc;
    LOGSYSERROR("Service Thread \"%s\": pthread_cond_signal()",
        pThCtl->m_sDevUri);
  }
}

/*!
 * \brief Wait on service thread's condition.
 *
 * \param pThCtl    Service thread control.
 */
static inline void ThSyncWait(BsProxyThCtl_T *pThCtl)
{
  int rc;

  if( (rc = pthread_cond_wait(&pThCtl->m_condSync, &pThCtl->m_mutexSync)) != 0 )
  { 
    errno = rc;
    LOGSYSERROR("Service Thread \"%s\": pthread_cond_wait()",
        pThCtl->m_sDevUri);
  }
}


//.............................................................................
// Proxied Device Hashing Functions
//.............................................................................

/*!
 * \brief Delete hash node data callback.
 *
 * Both the key and value are dynamically allocated.
 *
 * \param sDevUri   Proxied device hash table key.
 * \param pThCtl    Service thread control.
 */
static void ThHashDeleteData(void *sDevUri, void *pThCtl)
{
  delete(sDevUri);
  ThCtlBlkDelete(pThCtl);
}

/*!
 * \brief Add the module data to the i/f module hash table.
 *
 * \param sDevUri   Allocated Proxied device hash table key.
 * \param pThCtl    Allocated service thread control.
 *
 * \copydoc doc_return_std
 */
static int ThHashAdd(const char *sDevUri, BsProxyThCtl_T *pThCtl)
{
  char *sKey = new_strdup(sDevUri);
  int   rc;

  // insert into hash table, automatically allocating hnode_t
  if( !hash_insert(BsThHashTbl, sKey, pThCtl) )
  {
    rc = -BS_ECODE_NO_RSRC;
    BSPROXY_TH_LOG_ERROR(sDevUri, rc, "hash_insert(%s,...) failed", sKey);
    ThHashDeleteData(sKey, pThCtl);
    return rc;
  }

  return BS_OK;
}

/*!
 * \brief Remove and delete the module data from the i/f module hash table.
 *
 * \param pThCtl    Allocated service thread control.
 *
 * \copydoc doc_return_std
 */
static void ThHashDelete(BsProxyThCtl_T *pThCtl)
{
  hnode_t *pNode;

  if( (pThCtl != NULL) && 
      ((pNode = hash_lookup(BsThHashTbl, pThCtl->m_sDevUri)) != NULL) )
  {
    hash_node_delete(BsThHashTbl, pNode);
  }
}

/*!
 * \brief Get the interface module's i/f from the module hash table.
 *
 * \param sDevUri   Proxied device hash table key.
 *
 * \return
 * On success, returns pointer to the associated thread control block.
 * On failure, returns NULL.
 */
static BsProxyThCtl_T *ThHashGetCtl(const char *sDevUri)
{
  hnode_t       *pNode;

  if( (pNode = hash_lookup(BsThHashTbl, sDevUri)) == NULL )
  {
     return NULL;
  }
  else
  {
    return (BsProxyThCtl_T *)hnode_get(pNode);
  }
}


//.............................................................................
// Service Threads
//.............................................................................

/*!
 * \brief Command service thread to run.
 *
 * \param pThCtl    Service thread control block.
 *
 * \par Execution Context:
 * Calling thread.
 */
static void ThSyncRun(BsProxyThCtl_T *pThCtl)
{
  ThSyncLock(pThCtl);
  pThCtl->m_eState = BsProxyThStateRunning;
  ThSyncSignal(pThCtl);
  ThSyncUnlock(pThCtl);
}

/*!
 * \brief Command service thread to exit.
 *
 * \par Execution Context:
 * Calling thread.
 *
 * \param pThCtl    Service thread control block.
 */
static void ThSyncExit(BsProxyThCtl_T *pThCtl)
{
  ThSyncLock(pThCtl);
  pThCtl->m_eState = BsProxyThStateExit;
  ThSyncSignal(pThCtl);
  ThSyncUnlock(pThCtl);
}

/*!
 * \brief Wait indefinitely until state change.
 *
 * \par Execution Context:
 * Service thread.
 *
 * \param pThCtl    Service thread control block.
 */
static void ThSyncWaitForRun(BsProxyThCtl_T *pThCtl)
{
  ThSyncLock(pThCtl);
  while( pThCtl->m_eState == BsProxyThStateInit )
  {
    ThSyncWait(pThCtl);
  }
  ThSyncUnlock(pThCtl);
}

/*!
 * \brief Device service thread request handler.
 *
 * \par Execution Context:
 * Device service thread.
 *
 * \param hndClient   \h_botsense client handle.
 * \param hndVConn    Virtual connection handle.
 * \param uTid        Request-response transaction id.
 * \param uMsgId      Request message id.
 * \param [in] bufReq Packed request message body buffer.   
 * \param uReqLen     Length of packed request (number of bytes).
 *
 * \copydoc doc_return_std
 */
static int ThDevRequest(BsProxyClientHnd_T hndClient,
                        BsVConnHnd_T       hndVConn,
                        BsTid_T            uTid,
                        BsMsgId_T          uMsgId,
                        byte_t             bufReq[],
                        size_t             uReqLen)
{
  BsProxyVConn_T *pVConn;
  int             rc;

  //
  // Acquire virtual connection.
  //
  if( (pVConn = VConnAcquire(hndVConn)) != NULL )
  {
    //
    // Call the interface module' specific request handler.
    // The module is responsible for sending a response back regardless if the
    // request is a success or a failure.
    //
    rc = pVConn->m_pModIF->m_fnModRequest(hndVConn, uTid, uMsgId, bufReq,
                                          uReqLen);

    // release virtual connection
    VConnRelease(hndVConn); 

    // error
    if( rc < 0 )
    {
      BSPROXY_LOG_ERROR(hndClient, rc, "Request failed.");
    }
  }

  else
  {
    BSPROXY_SEND_ERROR_RSP(hndClient, hndVConn, uTid, BS_ECODE_NO_VCONN,
        "VConn=%d", hndVConn);
    rc = -BS_ECODE_NO_VCONN;
  }

  return rc;
}

/*!
 * \brief The service thread.
 *
 * \param pThArg    Thread argument.
 * 
 * \return Returns NULL on thread exit.
 */
static void *ThServiceThread(void *pThArg)
{
  BsProxyThCtl_T *pThCtl = (BsProxyThCtl_T *)pThArg;
  BsProxyThReq_T *pThReq;
  int             rc;

  LOGDIAG1("Service Thread \"%s\" created.", pThCtl->m_sDevUri);

  ThSyncWaitForRun(pThCtl);

  //
  // Wait for any queue message or thread state change.
  //
  while( (pThReq = ThDequeue(pThCtl)) )
  {
    // exit thread
    if( pThCtl->m_eState == BsProxyThStateExit )
    {
      break;
    }
    // nothing on queue
    else if( pThReq == NULL )
    {
      BSPROXY_TH_LOG_ERROR(pThCtl->m_sDevUri, BS_ECODE_NO_EXEC,
          "No request on queue.");
    }
    // service request then destroy
    else
    {
      rc = pThCtl->m_fnRequest(pThReq->m_hndClient,
                               pThReq->m_hndVConn,
                               pThReq->m_uTid,
                               pThReq->m_uMsgId,
                               pThReq->m_bufReq,
                               pThReq->m_uReqLen);
      ThReqDelete(pThReq);
    }
  }

  LOGDIAG1("Service Thread \"%s\" destroyed.", pThCtl->m_sDevUri);

  return NULL;
}


// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

//.............................................................................
// Service Thread Request Functions
//.............................................................................

/*!
 * \brief Allocate and initalized a new service thread request.
 *
 * \param hndClient Client handle.
 * \param hndVConn  Virtual connection handle.
 * \param uTid      Request-Response transaction id.
 * \param uMsgId    Request message id.
 * \param bufReq    Allocated packed request message body.
 * \param uReqLen   Length of request in buffer (number of bytes).
 *
 * \return Pointer to allocated request.
 */
BsProxyThReq_T *ThReqNew(BsProxyClientHnd_T hndClient,
                         BsVConnHnd_T       hndVConn,
                         BsTid_T            uTid,
                         BsMsgId_T          uMsgId,
                         byte_t             bufReq[],
                         size_t             uReqLen)
{
  BsProxyThReq_T *pThReq = NEW(BsProxyThReq_T);

  pThReq->m_hndClient = hndClient;
  pThReq->m_hndVConn  = hndVConn;
  pThReq->m_uTid      = uTid;
  pThReq->m_uMsgId    = uMsgId;
  pThReq->m_bufReq    = bufReq;
  pThReq->m_uReqLen   = uReqLen;

  return pThReq;
}

/*!
 * \brief Delete service thread request.
 *
 * \param pThReq    Service thread request.
 */
void ThReqDelete(BsProxyThReq_T *pThReq)
{
  if( pThReq != NULL )
  {
    delete(pThReq->m_bufReq);
    delete(pThReq);
  }
}

/*!
 * \brief Queue a request for the given service thread.
 *
 * \par Execution Context:
 * Calling thread.
 *
 * \param pThCtl    Service thread control block.
 * \param hndClient Client handle.
 * \param hndVConn  Virtual connection handle.
 * \param uTid      Request-Response transaction id.
 * \param uMsgId    Request message id.
 * \param bufReq    Allocated packed request message body.
 * \param uReqLen   Length of request in buffer (number of bytes).
 *
 * \copydoc doc_return_std
 */
int ThQueue(BsProxyThCtl_T     *pThCtl,
            BsProxyClientHnd_T  hndClient,
            BsVConnHnd_T        hndVConn,
            BsTid_T             uTid,
            BsMsgId_T           uMsgId,
            byte_t              bufReq[],
            size_t              uReqLen)
{
  BsProxyThReq_T *pThReq;
  int             rc;

  // lock thread and queue
  ThSyncLock(pThCtl);

  //
  // Queue request on thread's message queue.
  //
  if( DListVoidCount(pThCtl->m_queue) < BSPROXY_TH_MAX_QUEUE_SIZE )
  {
    pThReq = ThReqNew(hndClient, hndVConn, uTid, uMsgId, bufReq, uReqLen);
    DListVoidQueue(pThCtl->m_queue, pThReq);
    ThSyncSignal(pThCtl);
    LOGDIAG3("%s: Service Thread \"%s\": queue depth=%u.",
      ServerHasName(), pThCtl->m_sDevUri, DListVoidCount(pThCtl->m_queue)); 
    rc = BS_OK;
  }
  else
  {
    rc = -BS_ECODE_NO_RSRC;
    BSPROXY_TH_LOG_ERROR(pThCtl->m_sDevUri, rc, "Queue overflow.");
  }

  // unlock thread and queue
  ThSyncUnlock(pThCtl);

  return rc;
}

/*!
 * \brief Dequeue a request for the given service thread.
 *
 * The service thread will block if no queued request is present.
 *
 * \par Execution Context:
 * Service thread.
 *
 * \return On success, returns removed request at the front of the queue.\n
 * On failure, NULL is returned.
 */
BsProxyThReq_T *ThDequeue(BsProxyThCtl_T *pThCtl)
{
  BsProxyThReq_T  *pThReq;

  // lock thread and queue
  ThSyncLock(pThCtl);

  //
  // Block waiting for queued request or state change event.
  //
  while( (pThCtl->m_eState == BsProxyThStateRunning) &&
         (DListVoidCount(pThCtl->m_queue) == 0) )
  {
    // Release mutex, block on conditional variable, auto-lock mutex on return.
    ThSyncWait(pThCtl);
  }

  if( pThCtl->m_eState == BsProxyThStateRunning )
  {
    pThReq = (BsProxyThReq_T *)DListVoidDequeue(pThCtl->m_queue);
  }
  else
  {
    pThReq = NULL;
  }

  // unlock thread and queue
  ThSyncUnlock(pThCtl);

  return pThReq;
}

/*!
 * \brief The service thread one-time global initialization.
 */
void ThOneTimeInit()
{
  // create global mutex
  pthread_mutex_init(&BsThGlobalMutex, NULL);
  
  // create thread hash table keyed by device uri
  BsThHashTbl = hash_table_create(
        true,                               // dynamic table sizing
        (hashcount_t)BSPROXY_TH_HASH_MIN,   // minimum size
        (hashcount_t)BSPROXY_TH_HASH_MAX,   // maximum size
        NULL,                               // use default comparator function
        NULL,                               // use default hashing function
        ThHashDeleteData);                  // hash node data deletion function

  // turn off most asserts()
  hash_set_self_verify(false);
}

/*!
 * \brief Convert the device name to a quasi Uniform Resource Id.
 *
 * \param sDevName  Device path name.
 *
 * \return
 * On success, an allocated, canonical device path name is returned.\n
 * On failure, NULL is returned.
 */
char *ThNewDevUri(const char *sDevName)
{
  return NewSearchPathCanonicalized(sDevName);
}

/*!
 * \brief Create a device service thread.
 *
 * The actual thread is only created if it does not already exists. The internal
 * reference count is incremented to keep track of the users of this thread.
 *
 * \par Execution Context:
 * Calling thread.
 *
 * \param sDevUri   Device URI.
 *
 * \return On success, returns the service thread control block.
 * On failure, NULL is returned.
 */
BsProxyThCtl_T *ThCreateDevThread(const char *sDevUri)
{
  BsProxyThCtl_T *pThCtl;
  int             rc;

  // lock global mutex
  ThGlobalLock();

  //
  // Device thread already exists. 
  //
  if( (pThCtl = ThHashGetCtl(sDevUri)) != NULL )
  {
    pThCtl->m_uRefCnt++;
    LOGDIAG1("Service Thread \"%s\" attached (refcnt=%u).",
        pThCtl->m_sDevUri, pThCtl->m_uRefCnt);
  }

  //
  // Create new device thread.
  //
  else
  {
    //
    // Create a new thread control block.
    //
    pThCtl = ThCtlBlkNew(sDevUri);

    pThCtl->m_fnRequest = ThDevRequest;       // device request handler
    pThCtl->m_uRefCnt   = 1;                  // thread reference count
    pThCtl->m_eState    = BsProxyThStateInit; // control is (mostly) initialized

    //
    // Add thread control to thread hash table
    //
    if( (rc = ThHashAdd(sDevUri, pThCtl)) < 0 )
    {
      LOGERROR("\"%s\": Failed to add thread control to hash table.", sDevUri);
      ThCtlBlkDelete(pThCtl);
      pThCtl = NULL;
    }

    //
    // Start the device service thread. The thread will block until the run
    // state is set this context.
    //
    else if( pthread_create(&pThCtl->m_thread, NULL, ThServiceThread,
                            (void *)pThCtl) )
    {
      LOGSYSERROR("pthread_create(\"%s\")", sDevUri);
      ThHashDelete(pThCtl);
      pThCtl = NULL;
    }

    //
    // Signal the thread to run.
    //
    else
    {
      ThSyncRun(pThCtl);
    }
  }

  // unlock thread mutex
  ThGlobalUnlock();

  return pThCtl;
}

/*!
 * \brief Create the special server service thread.
 *
 * There is only one server service thread per server. The server thread processes
 * all server terminated requests.
 *
 * \par Execution Context:
 * Calling thread.
 *
 * \return On success, returns the service thread control block.
 * On failure, NULL is returned.
 */
BsProxyThCtl_T *ThCreateServerThread()
{
  char           *sDevUri;
  BsProxyThCtl_T *pThCtl;
  int             rc;

  // Server URI
  sDevUri = new_strdup(BSPROXY_TH_SERVER_URI);

  // lock global mutex
  ThGlobalLock();

  //
  // There should only be one server thread.
  //
  if( (pThCtl = ThHashGetCtl(sDevUri)) != NULL )
  {
    BSPROXY_TH_LOG_ERROR(sDevUri, BS_ECODE_INTERNAL,
        "Server service thread already exists.");
  }

  //
  // Create new device thread.
  //
  else
  {
    //
    // Create a new thread control block.
    //
    pThCtl = ThCtlBlkNew(sDevUri);

    pThCtl->m_fnRequest = ServerRequest;      // server request handler
    pThCtl->m_uRefCnt   = 1;                  // thread reference count
    pThCtl->m_eState    = BsProxyThStateInit; // control is (mostly) initialized

    //
    // Add thread control to thread hash table
    //
    if( (rc = ThHashAdd(sDevUri, pThCtl)) < 0 )
    {
      LOGERROR("\"%s\": Failed to add thread control to hash table.", sDevUri);
      ThCtlBlkDelete(pThCtl);
      pThCtl = NULL;
    }

    //
    // Start the device service thread. The thread will block until the run
    // state is set this context.
    //
    else if( pthread_create(&pThCtl->m_thread, NULL, ThServiceThread,
                            (void *)pThCtl) )
    {
      LOGSYSERROR("pthread_create(\"%s\")", sDevUri);
      ThHashDelete(pThCtl);
      pThCtl = NULL;
    }

    //
    // Signal the thread to run.
    //
    else
    {
      ThSyncRun(pThCtl);
    }
  }

  // unlock global mutex
  ThGlobalUnlock();

  return pThCtl;
}

/*!
 * \brief Destroy service thread.
 *
 * The thread's reference count is decremented. If the count is zero, then the
 * thread is actually destroyed with all queued requests deleted. The calling
 * thread is suspended until the target service thread terminates.
 *
 * \par Execution Context:
 * Calling thread.
 *
 * \param pThCtl    Service thread control block.
 */
void ThDestroyThread(BsProxyThCtl_T *pThCtl)
{
  if( pThCtl == NULL )
  {
    return;
  }

  // lock global mutex
  ThGlobalLock();

  // do not destroy the thread since there exists users of this thread
  if( pThCtl->m_uRefCnt > 0 )
  {
    pThCtl->m_uRefCnt--;
  }

  // thread is not being used anymore - destroy
  if( pThCtl->m_uRefCnt == 0 )
  {
    // signal thread of exit state change
    ThSyncExit(pThCtl);

    // wait for thread to terminate
    pthread_join(pThCtl->m_thread, NULL);

    // delete the thread hash entry's control block (including its queue)
    ThHashDelete(pThCtl);
  }

  else
  {
    LOGDIAG1("Service Thread \"%s\" detached (refcnt=%u).",
        pThCtl->m_sDevUri, pThCtl->m_uRefCnt);
  }

  // unlock global mutex
  ThGlobalUnlock();
}
