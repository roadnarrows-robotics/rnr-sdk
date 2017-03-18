////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Library:   librnr_cam
//
// File:      CameraCv.cxx
//
/*! \file
 *
 * $LastChangedDate: 2014-06-18 14:51:56 -0600 (Wed, 18 Jun 2014) $
 * $Rev: 3667 $
 *
 * \brief OpenCv video and still image camera class implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2012-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
/*
 * @EulaBegin@
 * 
 * Permission is hereby granted, without written agreement and without
 * license or royalty fees, to use, copy, modify, and distribute this
 * software and its documentation for any purpose, provided that
 * (1) The above copyright notice and the following two paragraphs
 * appear in all copies of the source code and (2) redistributions
 * including binaries reproduces these notices in the supporting
 * documentation.   Substantial modifications to this software may be
 * copyrighted by their authors and need not follow the licensing terms
 * described here, provided that the new terms are clearly indicated in
 * all files where they apply.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
 * OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#include <unistd.h>
#include <stdio.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/appkit/Camera.h"
#include "rnr/appkit/CameraCv.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"


using namespace std;
using namespace rnr;
using namespace cv;


//-----------------------------------------------------------------------------
// OpenCv Derived Camera Class
//-----------------------------------------------------------------------------

CameraCv::CameraCv(const std::string &strVideoDevName,
                   const CamRes      &resVideo,
                   const CamRes      &resImage) :
    Camera(strVideoDevName, resVideo, resImage)
{
}

CameraCv::~CameraCv()
{
  stopVideo();
}

int CameraCv::startVideo(const CamRes &resVideo)
{
  CamRes  res = resVideo;
  Mat     img;

  // default is the current video resolution
  if( isEqResolution(res, CamResDft) )
  {
    res = m_resVideo;
  }

  errno = 0;

  // start video
  if( !isCameraRunning() )
  {
    // start OpenCV video capture
    if( !m_capture.open(m_nVideoIndex) )
    {
      LOGSYSERROR("%s: Could not open video device.",
          m_strVideoDevName.c_str());
      m_bFatal = true;
      return RC_ERROR;
    }
  }

  m_bCameraRunning = true;

  // always set video resolution else camera may default to an alternate size
  m_resVideo = setCameraResolution(res);

  //
  // Grab a frame into internal buffer to get the actual resolution.
  //
  if( grabFrame(img) < 0 )
  {
    LOGERROR("Cannot determine captured image size, using defaults"); 
    m_resVideo = CamResQVGA;
  }

  // set actual resolution
  else 
  {
    m_resVideo.width  = img.cols;
    m_resVideo.height = img.rows;
  }

  // update current to match video
  m_resCurrent = m_resVideo;

  LOGDIAG3("Capturing %dx%d video.", m_resVideo.width, m_resVideo.height);

  return OK;
}

int CameraCv::stopVideo()
{
  if( !isCameraRunning() )
  {
    LOGDIAG3("Video capture already stoped.");
    return OK;
  }

  // capture frame will be released here also
  m_capture.release();

  m_bCameraRunning = false;

  LOGDIAG3("Video capturing stopped.");
}

int CameraCv::grabFrame(Mat &frame)
{
  if( !isCameraRunning() )
  {
    LOGERROR("No video capture object.");
    return RC_ERROR;
  }

  // grab a frame into internal buffer
  else if( !m_capture.grab() )
  {
    LOGERROR("Frame grab() failed.");
    return RC_ERROR;
  }

  // convert grabbed frame to RGB image (not saved)
  else if( !m_capture.retrieve(frame) )
  {
    LOGERROR("Frame retrieve() failed.");
    return RC_ERROR;
  }

  else
  {
    return OK;
  }
}

int CameraCv::clickImage(Mat &img, const CamRes &resImage)
{
  CamRes  resVideoPrev    = m_resVideo;
  bool    bCamWasRunning  = isCameraRunning();
  CamRes  res             = resImage;
  int     rc;

  m_bTakingImage = true;

  // default is the current image resolution
  if( isEqResolution(res, CamResDft) )
  {
    res = m_resImage;
  }

  //
  // Camera not running - start camera at image resolution.
  //
  if( !isCameraRunning() )
  {
    if( (rc = startVideo(res)) < 0 )
    {
      m_bTakingImage = false;
      return RC_ERROR;
    }
  }

  //
  // Camera already running, but at a different resolution. Set camera at image
  // resolution.
  //
  else if( !isEqResolution(res, m_resCurrent) )
  {
    setCameraResolution(res);
  }

  m_resImage = m_resCurrent;

  //
  // Grab a video frame.
  //
  if( grabFrame(img) < 0 )
  {
    LOGERROR("Cannot grab a snap shot.");
    m_bTakingImage = false;
    return RC_ERROR;
  }

  //
  // Success
  //
  m_resImage.width  = img.cols;
  m_resImage.height = img.rows;

  LOGDIAG3("Took a %dx%d still image.", m_resImage.width, m_resImage.height);

  m_bTakingImage = false;

  m_resVideo = resVideoPrev;

  if( !isEqResolution(m_resVideo, m_resCurrent) )
  {
    startVideo(m_resVideo);
  }

  if( !bCamWasRunning )
  {
    stopVideo();
  }

  return OK;
}

void CameraCv::autoFocus()
{
}

CamRes CameraCv::setCameraResolution(const CamRes &res)
{
  m_capture.set(CV_CAP_PROP_FRAME_WIDTH, res.width);
  m_capture.set(CV_CAP_PROP_FRAME_HEIGHT, res.height);

  m_resCurrent = res;

  return m_resCurrent;
}
