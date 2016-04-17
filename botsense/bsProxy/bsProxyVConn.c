////////////////////////////////////////////////////////////////////////////////
//
// Package:   BotSense
//
// Program:   bsProxy
//
// File:      bsProxyVConn.c
//
/*! \file
 *
 * $LastChangedDate: 2010-08-20 11:36:38 -0600 (Fri, 20 Aug 2010) $
 * $Rev: 568 $
 *
 * \brief Virtual connections management operations.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright
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

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/new.h"

#include "botsense/BotSense.h"
#include "botsense/bsProxyModIF.h"

#include "bsProxy.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

//
// Local data
//
static pthread_mutex_t  BsVConnBusyMutex; ///< operations mutex
static pthread_cond_t   BsVConnBusyCond;  ///< block condition
static BsProxyVConn_T  *BsVConnTbl[BSPROXY_VCONN_NUMOF] = {NULL, };
                                          ///< table of virtual connections
static uint_t           BsVConnActiveCnt; ///< Number of active v. connections


//.............................................................................
// Mutual Exclusion Functions
//.............................................................................

/*!
 * \brief Lock virtual connection's global busy mutual exclusion.
 */
static inline void VConnLockBusy()
{
  int rc;

  if( (rc = pthread_mutex_lock(&BsVConnBusyMutex)) != 0 ) 
  { 
    errno = rc;
    LOGSYSERROR("pthread_mutex_lock()");
  }
}

/*!
 * \brief Unlock virtual connection's global busy mutual exclusion.
 */
static inline void VConnUnlockBusy()
{
  int rc;

  if( (rc = pthread_mutex_unlock(&BsVConnBusyMutex)) != 0 ) 
  { 
    errno = rc;
    LOGSYSERROR("pthread_mutex_unlock()");
  }
}

/*!
 * \brief Try to lock virtual connection's global busy mutual exclusion.
 *
 * \return
 * Returns true if lock is acquired. Otherwise returns false if mutex already
 * locked.
 */
static inline bool_t VConnBusyTryLock()
{
  return pthread_mutex_trylock(&BsVConnBusyMutex) == 0? true: false;
}

/*!
 * \brief Broadcast that a virtual connection has been freed or deleted.
 * 
 * A broadcast will unblock all threads currently blocked on the busy condition
 * variable. Only those thread waiting on virtual connections whose condition
 * has changed will run. If multiple threads are waiting on the same
 * virtual connection, then only one is schedule to run.
 */
static inline void VConnBroadcastNotBusy()
{
  int rc;

  if( (rc = pthread_cond_broadcast(&BsVConnBusyCond)) != 0 )
  { 
    errno = rc;
    LOGSYSERROR("pthread_cond_broadcast()");
  }
}

/*!
 * \brief Wait on a virtual connection to become's free.
 */
static inline void VConnWaitNotBusy()
{
  int rc;

  if( (rc = pthread_cond_wait(&BsVConnBusyCond, &BsVConnBusyMutex)) != 0 )
  { 
    errno = rc;
    LOGSYSERROR("pthread_cond_wait()");
  }
}


// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief The bsProxy virtual connections one-time initialization.
 */
void VConnOneTimeInit()
{
  // create mutex
  pthread_mutex_init(&BsVConnBusyMutex, NULL);

  // create condition
  pthread_cond_init(&BsVConnBusyCond, NULL);
}

/*!
 * \brief Create a new, allocated virtual connection entry.
 *
 * The new virtual connection is added to the virtual connection table. It is
 * marked busy (i.e. locked). The caller must call VConnRelease() or
 * VConnDelete() to free the entry.
 *
 * \return Allocated virtual connection on success, NULL on failure.
 */
BsProxyVConn_T *VConnNew()
{
  BsProxyVConn_T *pVConn = NULL;
  BsVConnHnd_T    hndVConn;

  // lock virtual connection table
  VConnLockBusy();

  // find an empty virtual connection table slot
  if( BsVConnActiveCnt < BSPROXY_VCONN_NUMOF )
  {
    for(hndVConn=0; hndVConn<BSPROXY_VCONN_MOD_NUMOF; ++hndVConn)
    {
      if( BsVConnTbl[hndVConn] == NULL )
      {
        pVConn               = NEW(BsProxyVConn_T);
        pVConn->m_bBusy      = true;
        pVConn->m_hndVConn   = hndVConn;
        BsVConnTbl[hndVConn] = pVConn;
        BsVConnActiveCnt++;
        break;
      }
    }
  }

  // unlock virtual connection table
  VConnUnlockBusy();

  return pVConn;
}

/*!
 * \brief Delete allocated virtual connection entry.
 *
 * The deletion event is broadcasted to all blocked threads waiting on this
 * connection.
 */
void VConnDelete(BsProxyVConn_T *pVConn)
{
  VConnLockBusy();

  if( pVConn != NULL )
  {
    if( BSPROXY_CHK_VCONN_HND(pVConn->m_hndVConn) )
    {
      BsVConnTbl[pVConn->m_hndVConn] = NULL;
      BsVConnActiveCnt--;
    }
    delete(pVConn);
  }

  VConnBroadcastNotBusy();

  VConnUnlockBusy();
}

/*!
 * \brief Get the virtual connection entry associated with the handle.
 *
 * \warning The virtual connection is not locked.
 *
 * \param hndVConn  \h_botsense virtual connection handle.
 *
 * \return Virtual connection entry on success, NULL on failure.
 */
BsProxyVConn_T *VConnGet(BsVConnHnd_T hndVConn)
{
  if( !BSPROXY_CHK_VCONN_HND(hndVConn) )
  {
    return NULL;
  }
  else
  {
    return BsVConnTbl[hndVConn];
  }
}

/*!
 * \brief Open special server virtual connection.
 *
 * \copydoc doc_return_std
 */
int VConnOpenServer()
{
  BsProxyThCtl_T *pThCtl;       // server service thread control
  BsProxyVConn_T *pVConn;       // new virtual connection entry
  BsVConnHnd_T    hndVConn = BSPROXY_VCONN_SERVER;  // reserved server handle
  int             rc;           // return code

  //
  // Create the one server thread.
  //
  if( (pThCtl = ThCreateServerThread()) == NULL )
  {
    rc = -BS_ECODE_NO_EXEC;
    LOGERROR("%s: Could not create server service thread.", ServerHasName());
  }

  //
  // Add this special virtual connection to the table. No mutual exclusion is
  // done since the server should be started prior to servicing an client
  // requests.
  //
  else
  {
    pVConn              = NEW(BsProxyVConn_T);
    pVConn->m_bBusy     = false;
    pVConn->m_hndClient = -1;
    pVConn->m_hndVConn  = hndVConn;
    pVConn->m_pThCtl    = pThCtl;
    pVConn->m_pModIF    = NULL;
    pVConn->m_rd        = -1;

    BsVConnTbl[hndVConn] = pVConn;
    BsVConnActiveCnt++;

    rc = BS_OK;
  }

  return rc;
}

/*!
 * \brief Open a virtual device connection.
 *
 * To open a virtual connection:
 * \li A new entry is created.
 * \li The associated device thread is located or created.
 * \li The associated interface module is located or loaded.
 * \li The module's exported ModOpen() routine is called. 
 *
 * \param hndClient   \h_botsense client handle.
 * \param sDevName    Device path name.
 * \param sModName    Interface module path name.
 * \param argbuf      Packed argument buffer specific to interface module's 
 *                    ModOpen().
 * \param uArgLen     Number of packed bytes in argument buffer.
 * \param bTrace      Initial module message tracing state.
 *
 * \return On success, returns virtual connection handle.\n
 * \copydoc doc_return_ecode
 */
int VConnOpenDev(BsProxyClientHnd_T  hndClient,
                 const char         *sDevName,
                 const char         *sModName,
                 byte_t              argbuf[],
                 size_t              uArgLen,
                 bool_t              bTrace)
{
  BsProxyVConn_T *pVConn  = NULL; // new virtual connection entry
  char           *sModUri = NULL; // module URI
  char           *sDevUri = NULL; // device URI
  BsProxyModIF_T *pModIF  = NULL; // interface module interface
  BsProxyThCtl_T *pThCtl  = NULL; // device thread control block
  int             rc;             // return code
  int             rd;             // resource descriptor

  //
  // Allocate a new virtual connection. If no more virtual connections are
  // available, NULL is returned. The return virtual connection is left in
  // the acquired state which requires releasing prior to becoming available
  // for normal operations.
  //
  if( (pVConn = VConnNew()) == NULL )
  {
    rc = -BS_ECODE_NO_RSRC;
    BSPROXY_LOG_ERROR(hndClient, rc,
        "No more virtual connections are available.");
  }

  //
  // Convert module path name to canonical uniform resource identifier.
  //
  else if( (sModUri = ModNewModUri(sModName)) == NULL )
  {
    rc = -BS_ECODE_BAD_VAL;
    BSPROXY_LOG_ERROR(hndClient, rc, "\"%s\": Bad module name.", sModName);
  }

  //
  // Convert device path name to canonical uniform resource identifier.
  //
  else if( (sDevUri = ThNewDevUri(sDevName)) == NULL )
  {
    rc = -BS_ECODE_BAD_VAL;
    BSPROXY_LOG_ERROR(hndClient, rc, "\"%s\": Bad device name.", sDevName);
  }

  //
  // Dynamically load and initialize library interface module and return the
  // module's interface. If the module already exists, then the same interface
  // is returned. The module interface maintains a reference count which is 
  // incremented.
  //
  else if( (pModIF = ModLoad(sModUri)) == NULL )
  {
    rc = -BS_ECODE_NO_MOD;
    BSPROXY_LOG_ERROR(hndClient, rc,
        "Could not load or attach to \"%s\" interface module.", sModUri);
  }

  //
  // Call the interface module's open() function to open the device and allocate
  // any internal resources. The return resource descriptor is typically a
  // file or socket descript, but may be any unique non-negative integer
  // defined by the module.
  //
  else if( (rd = pModIF->m_fnModOpen(pVConn->m_hndVConn, bTrace,
                                     sDevUri, argbuf, uArgLen)) < 0 )
  {
    rc = -BS_ECODE_NO_EXEC;
    BSPROXY_LOG_ERROR(hndClient, rd,
        "Could not open \"%s\" interface module.", pModIF->m_sModUri);
  }

  //
  // Create a device thread returning the thread's control. If the thread
  // already exists, then the same thread control is returned. The thread 
  // control maintains a reference count which is incremented.
  //
  else if( (pThCtl = ThCreateDevThread(sDevUri)) == NULL )
  {
    rc = -BS_ECODE_NO_EXEC;
    BSPROXY_LOG_ERROR(hndClient, rc,
        "Could not create or attach to \"%s\" device service thread.",
        sDevUri);
  }

  //
  // Success.
  //
  // Populate the virtual connection's data. All allocated data are not owned by
  // the virtual connection. That is, any allocated data will not be deleted
  // when the virtual connection is deleted.
  //
  else
  {
    pVConn->m_hndClient = hndClient;
    pVConn->m_pThCtl    = pThCtl;
    pVConn->m_pModIF    = pModIF;
    pVConn->m_rd        = rd;

    VConnRelease(pVConn->m_hndVConn);

    rc = (int)pVConn->m_hndVConn;

    LOGDIAG1("%s: %s: Virtual connection %d opened: %s('%s',rd=%d).",
        ServerHasName(), ClientHasName(hndClient), pVConn->m_hndVConn,
        sModUri, sDevUri, rd);

    LOGDIAG2("%s: %u active module virtual connections.",
        ServerHasName(), BsVConnActiveCnt-1);
  }

  //
  // Error occurred, so clean up before returning.
  //
  if( rc < 0 )
  {
    if( pThCtl != NULL )
    {
      ThDestroyThread(pThCtl);
    }
    if( pModIF != NULL )
    {
      ModUnload(pModIF);
    }
    if( pVConn != NULL )
    {
      VConnDelete(pVConn);
    }
  }

  //
  // Fixed clean up.
  //
  delete(sModUri);
  delete(sDevUri);

  return rc;
}

/*!
 * \brief Close the virtual connection.
 *
 * Closing a virtual connections may close the resource (file) descriptor
 * associated with the device, unload the interface module associated with
 * the device, and terminate the device thread. These actions occur only when
 * the reference counts are zero.
 *
 * \param hndClient   \h_botsense client handle.
 * \param hndVConn    \h_botsense virtual connection handle.
 *
 * \copydoc doc_return_std
 */
int VConnClose(BsProxyClientHnd_T  hndClient, BsVConnHnd_T hndVConn)
{
  BsProxyVConn_T *pVConn;
  int             rc;

  // Only virtual connections associated with devices can be closed.
  if( !BSPROXY_CHK_MOD_VCONN_HND(hndVConn) )
  {
    rc = -BS_ECODE_BAD_VCONN_HND;
  }

  //
  // Acquire virtual connection associated with handle. Aquiring a virtual
  // connections locks out other thread access to the vConnection until it 
  // is released.
  //
  else if( (pVConn = VConnAcquire(hndVConn)) == NULL )
  {
    rc = -BS_ECODE_NO_VCONN;
    BSPROXY_LOG_ERROR(hndClient, rc, "VConn=%d", hndVConn);
  }

  else
  {
    //
    // Destroy the device thread. The thread reference count is decremented and
    // only when it drops to zero is the thread actually terminated.
    //
    ThDestroyThread(pVConn->m_pThCtl);

    //
    // Close the device associated with virtual connection. The actual device
    // may or may not be closed, depending on the device and module
    // implementation.  For example, for an \h_i2c Bus device, the device is
    // only closed when the last reference to that device is closed.
    //
    pVConn->m_pModIF->m_fnModClose(pVConn->m_hndVConn);

    //
    // Dynamically unload the interface module from the bsProxy application. The
    // module's reference count is decremented and only when it drops to zero is
    // the module actually unloaded.
    //
    ModUnload(pVConn->m_pModIF);

    LOGDIAG1("%s: %s: Virtual connection %d closed.",
        ServerHasName(), ClientHasName(hndClient), pVConn->m_hndVConn);

    //
    // Delete the virtual connection. The slot is now available for a new
    // virtual connection.
    //
    VConnDelete(pVConn);

    LOGDIAG2("%s: %u active module virtual connections.",
        ServerHasName(), BsVConnActiveCnt-1);

    rc = BS_OK;
  }

  return rc;
}

/*!
 * \brief Acquire virtual connection, locking it from other threads.
 *
 * The calling thread is blocked until the virtual connection is availble or
 * has been deleted.
 *
 * \param hndVConn  \h_botsense virtual connection handle.
 *
 * \return On success, returns pointer to locked virtual connection.
 * On failure, NULL is returned.
 */
BsProxyVConn_T *VConnAcquire(BsVConnHnd_T hndVConn)
{
  BsProxyVConn_T  *pVConn;

  if( !BSPROXY_CHK_VCONN_HND(hndVConn) )
  {
    return NULL;
  }

  VConnLockBusy();

  while( (BsVConnTbl[hndVConn] != NULL) && BsVConnTbl[hndVConn]->m_bBusy )
  {
    VConnWaitNotBusy();
  }

  if( (pVConn = BsVConnTbl[hndVConn]) != NULL )
  {
    pVConn->m_bBusy = true;
  }

  VConnUnlockBusy();

  return pVConn;
}

/*!
 * \brief Release the locked virtual client.
 *
 * A broadcast is sent to all blocking threads on the freed event.
 *
 * \param hndVConn  \h_botsense virtual connection handle.
 */
void VConnRelease(BsVConnHnd_T hndVConn)
{
  if( !BSPROXY_CHK_VCONN_HND(hndVConn) )
  {
    return;
  }

  VConnLockBusy();

  if( BsVConnTbl[hndVConn] != NULL )
  {
    BsVConnTbl[hndVConn]->m_bBusy = false;
  }

  VConnBroadcastNotBusy();

  VConnUnlockBusy();
}
