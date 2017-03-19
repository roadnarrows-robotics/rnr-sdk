////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Program:   StaleMate
//
// File:      SMLiveFeed.cxx
//
/*! \file
 *
 * $LastChangedDate: 2011-08-06 12:28:16 -0600 (Sat, 06 Aug 2011) $
 * $Rev: 1247 $
 *
 * \brief StaleMate annotated live feed thread.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2011-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
// Unless otherwise noted, all materials contained are copyrighted and may not
// be used except as provided in these terms and conditions or in the copyright
// notice (documents and software ) or other proprietary notice provided with
// the relevant materials.
//
//
// IN NO EVENT SHALL THE AUTHOR, ROADNARROWS, OR ANY MEMBERS/EMPLOYEES/
// CONTRACTORS OF ROADNARROWS OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
// PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
// EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.
//
// THE AUTHORS AND  ROADNARROWS SPECIFICALLY DISCLAIM ANY WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
// "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
// PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
//
////////////////////////////////////////////////////////////////////////////////

#include <sys/types.h>
#include <sys/time.h>
#include <time.h>
#include <stdarg.h>
#include <libgen.h>
#include <pthread.h>

#include <cstring>
#include <iostream>
#include <fstream>
#include <cmath>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/rnrWin.h"

#include "StaleMate.h"
#include "StaleMateTune.h"

using namespace std;
using namespace rnrWin;


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Live video feed states
 */
typedef enum
{
  LvfStateUninit,     ///< live feed control block not initialized 
  LvfStateInit,       ///< live feed thread created, but not running
  LvfStateRunning,    ///< live feed thread running
  LvfStateExit        ///< live feed thread exiting
} LvfState_T;

/*!
 * \brief Live video feed thread control block type.
 */
typedef struct
{
  LvfState_T      m_eState;     ///< thread state
  pthread_mutex_t m_mutexSync;  ///< synchonization mutex used by condition
  pthread_cond_t  m_condSync;   ///< synchonization condition
  pthread_t       m_thread;     ///< pthread identifier 
} LvfCtlBlk_T;

/*!
 * \brief Live video feed thread control block.
 */
static LvfCtlBlk_T LvfCtlBlk =
{
  LvfStateUninit,
  PTHREAD_MUTEX_INITIALIZER,
  PTHREAD_COND_INITIALIZER,
};

/*!
 * \brief Command the  video live feed thread to run.
 *
 * \par Context:
 * calling thread
 */
static void LvfThreadSyncRun()
{
  pthread_mutex_lock(&LvfCtlBlk.m_mutexSync);
  LvfCtlBlk.m_eState = LvfStateRunning;
  pthread_cond_signal(&LvfCtlBlk.m_condSync);
  pthread_mutex_unlock(&LvfCtlBlk.m_mutexSync);
}

/*!
 * \brief Command the  video live feed thread to exit.
 *
 * \par Context:
 * calling thread
 */
static void LvfThreadSyncExit()
{
  pthread_mutex_lock(&LvfCtlBlk.m_mutexSync);
  LvfCtlBlk.m_eState = LvfStateExit;
  pthread_cond_signal(&LvfCtlBlk.m_condSync);
  pthread_mutex_unlock(&LvfCtlBlk.m_mutexSync);
}

/*!
 * \brief Wait indefinitely until state change.
 *
 * \par Context:
 *  video live feed thread
 */
static void LvfThreadSyncWait()
{
  pthread_mutex_lock(&LvfCtlBlk.m_mutexSync);
  while( LvfCtlBlk.m_eState == LvfStateInit )
  {
    pthread_cond_wait(&LvfCtlBlk.m_condSync, &LvfCtlBlk.m_mutexSync);
  }
  pthread_mutex_unlock(&LvfCtlBlk.m_mutexSync);
}

/*!
 * \brief Wait until state change or timed out.
 *
 * \par Context:
 *  video live feed thread
 */
static void LvfThreadSyncTimedWait(long lMicroSecs)
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
  pthread_mutex_lock(&LvfCtlBlk.m_mutexSync);
  pthread_cond_timedwait(&LvfCtlBlk.m_condSync,
                         &LvfCtlBlk.m_mutexSync,
                         &tsTimeout);
  pthread_mutex_unlock(&LvfCtlBlk.m_mutexSync);
}

static int LvfCaptureFrame(StaleMateSession *pSession)
{
  int   cvrc;   // opencv return code
  int   rc;     // hekateros return code

  pthread_mutex_lock(&LvfCtlBlk.m_mutexSync);

  // video capturing is not running
  if( pSession->m_vid.pVidCapture == NULL )
  {
    LOGERROR("video capture not running.");
    rc = -HEK_ECODE_NO_RSRC;
  }

  // grab a frame into internal buffer
  else if( (cvrc = cvGrabFrame(pSession->m_vid.pVidCapture)) <= 0 )
  {
    LOGERROR("cvGrabFrame() cvrc=%d", cvrc);
    rc = -HEK_ECODE_VIDEO;
  }

  // convert grab frame to RGB image (not saved)
  else if( (pSession->m_vid.pImgFrame =
                cvRetrieveFrame(pSession->m_vid.pVidCapture)) == NULL )
  {
    LOGERROR("cvRetrieveFrame() pImgFrame=NULL");
    rc = -HEK_ECODE_VIDEO;
  }

  else
  {
    rc = HEK_OK;
  }

  pthread_mutex_unlock(&LvfCtlBlk.m_mutexSync);

  return rc;
}


static IplImage *LvfCreateSnapShot(StaleMateSession *pSession)
{
  IplImage *pImgFrame;
  IplImage *pImg;

  pthread_mutex_lock(&LvfCtlBlk.m_mutexSync);

  // video capturing is not running
  if( (pSession->m_vid.pVidCapture == NULL) ||
      (pSession->m_vid.pImgFrame == NULL) )
  {
    LOGERROR("Video capture not running or no frame is available.");
    pImg = NULL;
  }
  else
  {
    pImg = cvCloneImage(pSession->m_vid.pImgFrame);
  }

  pthread_mutex_unlock(&LvfCtlBlk.m_mutexSync);

  return pImg;
}

/*!
 * \brief The live video feed thread.
 *
 * \warn Cannot have gui calls in this thread.
 *
 * \param pSessionArg   Thread argument.
 * 
 * \return Returns NULL on thread exit.
 */
static void *SMLiveFeedThread(void *pSessionArg)
{
  StaleMateSession *pSession = (StaleMateSession *)pSessionArg;
  int               rc;

  LOGDIAG1("Live feed thread created");

  // wait until run is issued
  LvfThreadSyncWait();

  // Loop forever until exit
  while( (pSession->GetCurState() != StaleMateStateEnd)  &&
         (LvfCtlBlk.m_eState != LvfStateExit) )
  {
    // process  video live feed tasks
    switch( LvfCtlBlk.m_eState )
    {
      case LvfStateRunning:
        rc = LvfCaptureFrame(pSession);
        LvfThreadSyncTimedWait(30000L); // wait every 30ms between frames
        break;
      case LvfStateExit:
        break;
      case LvfStateInit:
      default:
        LOGERROR("%d: unexpected  video live feed thread state",
            LvfCtlBlk.m_eState);
        LvfCtlBlk.m_eState = LvfStateExit;
    }
  }

  LvfCtlBlk.m_eState = LvfStateUninit;

  LOGDIAG1("Video live feed thread destroyed");

  return NULL;
}

/*!
 * \brief Create a new video live feed thread.
 *
 * \par Context:
 * calling thread
 *
 * \param pSsssion  Pointer to session data.
 *
 * \return Returns HEK_OK on success, \<0 on failure.
 */
static int SMLiveFeedThreadCreate(StaleMateSession *pSession)
{
  int   rc;

  // thread is initialized but not started
  LvfCtlBlk.m_eState = LvfStateInit;

  rc = pthread_create(&LvfCtlBlk.m_thread, NULL,
                        SMLiveFeedThread, (void *)pSession);
 
  if( rc != 0 )
  {
    LOGSYSERROR("pthread_create()");
    return -HEK_ECODE_SYS;
  }

  return HEK_OK;
}


// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Start the video live feed thread.
 *
 * The live feed thread updates captures the Hekateros video and displays an
 * annotated version to display 0.
 *
 * \par Context:
 * calling thread
 *
 * \param pSsssion  Pointer to session data.
 *
 * \return Returns \>0 on success, \<0 on failure.
 */
int StaleMateLiveFeedThreadStart(StaleMateSession *pSession)
{
  if( LvfCtlBlk.m_eState != LvfStateUninit )
  {
    LOGERROR("Live video feed thread already running, state=%d",
            LvfCtlBlk.m_eState);
    return -HEK_ECODE_INTERNAL;
  }

  SMLiveFeedThreadCreate(pSession);
  
  LvfThreadSyncRun();

  return HEK_OK;
}

/*!
 * \brief Stop the live video feed thread.
 *
 * The live video feed thread is deleted.
 *
 * \par Context:
 * calling thread
 */
void StaleMateLiveFeedThreadStop()
{
  LvfThreadSyncExit();
}

/*!
 * \brief Wait for the live video feed thread to end.
 *
 * \par Context:
 * calling thread
 */
void StaleMateLiveFeedThreadJoin()
{
  pthread_join(LvfCtlBlk.m_thread, NULL);
}

/*!
 * \brief Stop thread and wait for the live video feed thread to end.
 *
 * \par Context:
 * calling thread
 */
void StaleMateLiveFeedThreadStopJoin()
{
  LvfThreadSyncExit();
  pthread_join(LvfCtlBlk.m_thread, NULL);
}

// (re)start
int StaleMateLiveFeedCaptureStart(StaleMateSession &session)
{
  int         nVidIndex;      // video index
  CvCapture  *pCapture;       // video capture pointer
  int         rc;             // return code

  // "/dev/video{n}" --> n
  nVidIndex = (int)session.m_vid.uVidDevMinor;

  pthread_mutex_lock(&LvfCtlBlk.m_mutexSync);

  // video capture already started
  if( session.m_vid.pVidCapture != NULL )
  {
    rc = HEK_OK;
  }

  // start video capture
  else if( (session.m_vid.pVidCapture =
                                  cvCreateCameraCapture(nVidIndex)) != NULL )
  {
    rc = HEK_OK;
  } 

  // failure to start
  else
  {
    LOGERROR("cvCreateCameraCapture(%d)", nVidIndex);
    rc = -HEK_ECODE_VIDEO;
  }

  pthread_mutex_unlock(&LvfCtlBlk.m_mutexSync);

  return rc;
}

void StaleMateLiveFeedCaptureStop(StaleMateSession &session)
{
  pthread_mutex_lock(&LvfCtlBlk.m_mutexSync);

  if( session.m_vid.pVidCapture != NULL )
  {
    // capture frame will be released here also
    cvReleaseCapture(&session.m_vid.pVidCapture);

    session.m_vid.pImgFrame   = NULL;
    session.m_vid.pVidCapture = NULL;
  }

  pthread_mutex_unlock(&LvfCtlBlk.m_mutexSync);
}

void StaleMateLiveFeedShow(StaleMateSession &session)
{
  static int  n = 0;

  IplImage  *pImg;

  if( (pImg = StaleMateVideoCreateSnapShot(session)) == NULL )
  {
    return;
  }

  StaleMateIoIShow(session, pImg, session.m_vid.uImgIndex0);

  cvReleaseImage(&pImg);

  //cerr << "DBG: frame " << n++ << endl;
}

int StaleMateVideoCaptureFrame(StaleMateSession &session, IplImage **ppImgFrame)
{
  int   cvrc;   // opencv return code
  int   rc;     // hekateros return code

  pthread_mutex_lock(&LvfCtlBlk.m_mutexSync);

  // grab a frame into internal buffer
  if( (cvrc = cvGrabFrame(session.m_vid.pVidCapture)) <= 0 )
  {
    LOGERROR("cvGrabFrame() cvrc=%d", cvrc);
    *ppImgFrame = NULL;
    rc = -HEK_ECODE_VIDEO;
  }

  // convert grab frame to RGB image (not saved)
  else if((*ppImgFrame = cvRetrieveFrame(session.m_vid.pVidCapture)) == NULL)
  {
    LOGERROR("cvRetrieveFrame() pImgFrame=NULL");
    rc = -HEK_ECODE_VIDEO;
  }

  else
  {
    rc = HEK_OK;
  }

  pthread_mutex_unlock(&LvfCtlBlk.m_mutexSync);

  return rc;
}

IplImage *StaleMateVideoCreateSnapShot(StaleMateSession &session)
{
  IplImage *pImg;

  pthread_mutex_lock(&LvfCtlBlk.m_mutexSync);

  // video capturing is not running
  if( (session.m_vid.pVidCapture == NULL) ||
      (session.m_vid.pImgFrame == NULL) )
  {
    LOGERROR("Video capture not running or no frame is available.");
    pImg = NULL;
  }
  else
  {
    pImg = cvCloneImage(session.m_vid.pImgFrame);
  }

  pthread_mutex_unlock(&LvfCtlBlk.m_mutexSync);

  return pImg;
}
