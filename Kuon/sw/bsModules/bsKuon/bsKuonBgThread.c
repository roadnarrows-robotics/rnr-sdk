////////////////////////////////////////////////////////////////////////////////
//
// Package:   Kuon
//
// Module:    bsKuon
//
// Library:   libbsserver_kuon
//
// File:      bsKuonBgThread.c
//
//
/*! \file
 *
 * $LastChangedDate: 2012-06-12 14:57:23 -0600 (Tue, 12 Jun 2012) $
 * $Rev: 2041 $
 *
 * \par Command:
 *   Update shell state from background processing including processing
 *   asynchronous notifications from the RCB-3.
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

#include <stdio.h>
#include <pthread.h>
#include <sys/time.h>
#include <time.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/i2c.h"
#include "rnr/MPU6050.h"

#include "Kuon/kuon.h"
#include "Kuon/RS160DControl.h"

#include "bsKuonInternal.h"


// ---------------------------------------------------------------------------
// Private Interface
// ---------------------------------------------------------------------------

#define T_IMU_POLL    10000L    ///< 100 Hz
#define DT_IMU_POLL   ((float)T_IMU_POLL / (float)1000000.0)

/*!
 * \brief Background shell states
 */
typedef enum
{
  BgStateUninit,    ///< background control block not initialized 
  BgStateInit,      ///< background thread created, but not running
  BgStateRunning,   ///< background thread running
  BgStateExit       ///< background thread exiting
} BgState_T;

/*!
 * \brief Background thread control block type.
 */
typedef struct
{
  BgState_T       m_eState;     ///< thread state
  pthread_mutex_t m_mutexSync;  ///< synchonization mutex used by condition
  pthread_cond_t  m_condSync;   ///< synchonization condition
  pthread_t       m_thread;     ///< pthread identifier 
} BgCtlBlk_T;

/*!
 * \brief Background thread control block.
 */
static BgCtlBlk_T BgCtlBlk =
{
  BgStateUninit,
  PTHREAD_MUTEX_INITIALIZER,
  PTHREAD_COND_INITIALIZER,
};

/*!
 * \brief Command the background thread to run.
 *
 * \par Context:
 * calling thread
 */
static void BgThreadSyncRun()
{
  pthread_mutex_lock(&BgCtlBlk.m_mutexSync);
  BgCtlBlk.m_eState = BgStateRunning;
  pthread_cond_signal(&BgCtlBlk.m_condSync);
  pthread_mutex_unlock(&BgCtlBlk.m_mutexSync);
}

/*!
 * \brief Command the background thread to exit.
 *
 * \par Context:
 * calling thread
 */
static void BgThreadSyncExit()
{
  pthread_mutex_lock(&BgCtlBlk.m_mutexSync);
  BgCtlBlk.m_eState = BgStateExit;
  pthread_cond_signal(&BgCtlBlk.m_condSync);
  pthread_mutex_unlock(&BgCtlBlk.m_mutexSync);
}

/*!
 * \brief Wait indefinitely until state change.
 *
 * \par Context:
 * background thread
 */
static void BgThreadSyncWait()
{
  pthread_mutex_lock(&BgCtlBlk.m_mutexSync);
  while( BgCtlBlk.m_eState == BgStateInit )
  {
    pthread_cond_wait(&BgCtlBlk.m_condSync, &BgCtlBlk.m_mutexSync);
  }
  pthread_mutex_unlock(&BgCtlBlk.m_mutexSync);
}

/*!
 * \brief Wait until state change or timed out.
 *
 * \par Context:
 * background thread
 */
static void BgThreadSyncTimedWait(long lMicroSecs)
{
  struct timeval  tvNow;
  struct timespec tsTimeout;
  long int        lSecs;

  // now
  // note: cannot find this function: clock_gettime(CLOCK_REALTIME, &tsTimeout)
  //       so use this one.
  gettimeofday(&tvNow, NULL);
  
  // future
  lMicroSecs  += tvNow.tv_usec;
  lSecs        = lMicroSecs / 1000000;
  lMicroSecs  -= (lSecs * 1000000);

  tsTimeout.tv_sec  = tvNow.tv_sec + lSecs;
  tsTimeout.tv_nsec = lMicroSecs * 1000;

  // wait with timeout
  pthread_mutex_lock(&BgCtlBlk.m_mutexSync);
  pthread_cond_timedwait(&BgCtlBlk.m_condSync,
                         &BgCtlBlk.m_mutexSync,
                         &tsTimeout);
  pthread_mutex_unlock(&BgCtlBlk.m_mutexSync);
}

/*! 
 * \brief Update state.
 *
 * Updates:\n
 * + current servo positions
 *
 * \par Context:
 * background thread
 *
 * \param pImu  IMU data.
 *
 * \return Returns \>0 on success, \<0 on error.
 */
static int BgUpdate(i2c_t *pI2cImu, KuonImu_T *pImu)
{
  float ax, ay, az;
  float gx, gy, gz;

  BulkUpdate(pI2cImu, &ax, &ay, &az, &gx, &gy, &gz);

  pImu->m_fAccX = ax;
  pImu->m_fAccY = ay;
  pImu->m_fAccZ = az;

  pImu->m_fGyrX += gx * DT_IMU_POLL;
  pImu->m_fGyrY += gy * DT_IMU_POLL;
  pImu->m_fGyrZ += gz * DT_IMU_POLL;

  return KUON_OK;
}

/*!
 * \brief The background thread.
 *
 * \param pImulArg    Thread argument.
 * 
 * \return Returns NULL on thread exit.
 */
static void *BgThread(void *pImulArg)
{
  KuonImu_T  *pImu = (KuonImu_T *)pImulArg;
  i2c_t       i2cImu;

  LOGDIAG1("Kuon background thread created");

  // wait until run is issued
  BgThreadSyncWait();

  initMPU6050(&i2cImu);

  // Loop forever until exit
  while( BgCtlBlk.m_eState != BgStateExit )
  {
    // every 0.1 seconds
    BgThreadSyncTimedWait(T_IMU_POLL);

    // process background tasks
    switch( BgCtlBlk.m_eState )
    {
      case BgStateRunning:
        BgUpdate(&i2cImu, pImu);
        break;
      case BgStateExit:
        break;
      case BgStateInit:
      default:
        LOGERROR("%d: unexpected background thread state", BgCtlBlk.m_eState);
        BgCtlBlk.m_eState = BgStateExit;
    }
  }

  i2c_close(&i2cImu);

  BgCtlBlk.m_eState = BgStateUninit;

  LOGDIAG1("Kuon background thread destroyed");

  return NULL;
}

/*!
 * \brief Create a new background thread.
 *
 * \par Context:
 * calling thread
 *
 * \param pImu  IMU data.
 *
 * \return Returns \>0 on success, \<0 on failure.
 */
static int BgThreadCreate(KuonImu_T *pImu)
{
  // thread is initialized but not started
  BgCtlBlk.m_eState = BgStateInit;

  if( pthread_create(&BgCtlBlk.m_thread, NULL, BgThread, (void *)pImu) != 0 )
  {
    LOGSYSERROR("pthread_create()");
    return -KUON_ECODE_SYS;
  }

  return KUON_OK;
}


// ---------------------------------------------------------------------------
// Public Interface
// ---------------------------------------------------------------------------

/*!
 * \brief Start the background thread.
 *
 * The background thread updates shell state by: \n
 * &nbsp; + processing asynchronous notifications. \n
 * &nbsp; + reading current servo positions.
 *
 * \par Context:
 * calling thread
 *
 * \param pImu  IMU data.
 *
 * \return Returns \>0 on success, \<0 on failure.
 */
int bsKuonBgThreadStart(KuonImu_T *pImu)
{
  if( BgCtlBlk.m_eState != BgStateUninit )
  {
    LOGERROR("Kuon background thread already running, state=%d",
        BgCtlBlk.m_eState);
    return -KUON_ECODE_RSRC;
  }

  BgThreadCreate(pImu);

  BgThreadSyncRun();

  return KUON_OK;
}

/*!
 * \brief Stop the background thread.
 *
 * The background thread is deleted.
 *
 * \par Context:
 * calling thread
 */
void bsKuonBgThreadStop()
{
  BgThreadSyncExit();
}

/*!
 * \brief Wait for the background thread to end.
 *
 * \par Context:
 * calling thread
 */
void bsKuonBgThreadJoin()
{
  pthread_join(BgCtlBlk.m_thread, NULL);
}
