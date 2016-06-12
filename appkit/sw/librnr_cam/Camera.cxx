////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Library:   librnr_cam
//
// File:      Camera.cxx
//
/*! \file
 *
 * $LastChangedDate: 2013-07-13 14:12:15 -0600 (Sat, 13 Jul 2013) $
 * $Rev: 3126 $
 *
 * \brief Video and still image camera base class implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright
 * (C) 2012-2016.  RoadNarrows LLC.
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
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

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/appkit/Camera.h"

#include "opencv/cv.h"

using namespace std;
using namespace rnr;
using namespace cv;


//-----------------------------------------------------------------------------
// Camera Base Class
//-----------------------------------------------------------------------------

Camera::Camera(const std::string &strVideoDevName,
               const CamRes      &resVideo,
               const CamRes      &resImage)
{
  m_strVideoDevName = strVideoDevName;
  m_nVideoIndex     = getVideoIndex(m_strVideoDevName);
  m_resCurrent      = CamResUndef;
  m_resVideo        = resVideo;
  m_resImage        = resImage;
  m_bCameraRunning  = false;
  m_bTakingImage    = false;
  m_bFatal          = false;

  if( (m_resVideo.width <= 0) || (m_resVideo.height <= 0) )
  {
    m_resVideo = CamResQVGA;
  } 

  if( (m_resImage.width <= 0) || (m_resImage.height <= 0) )
  {
    m_resImage = CamResVGA;
  } 

  if( m_nVideoIndex < 0 )
  {
    m_bFatal = true;
  }
}

/*!
 * \brief Default destructor.
 */
Camera::~Camera()
{
  // reelase video resources
  if( isCameraRunning() )
  {
    stopVideo();
  }
}

int Camera::getVideoIndex(const string &strVideoDevName)
{
  struct stat statVid;
  uint_t      uMajor;
  uint_t      uMinor;

  if( strVideoDevName.empty() )
  {
    LOGERROR("No video device name.");
    return RC_ERROR;
  }

  else if( access(strVideoDevName.c_str(), F_OK|R_OK|W_OK) != 0 )
  {
    LOGSYSERROR("%s.", strVideoDevName.c_str());
    return RC_ERROR;
  }

  else if( stat(strVideoDevName.c_str(), &statVid) != 0 )
  {
    LOGSYSERROR("%s.", strVideoDevName.c_str());
    return RC_ERROR;
  }
  
  uMajor = major(statVid.st_rdev);
  uMinor = minor(statVid.st_rdev);

  if( uMajor != VideoDevMajor )
  {
    LOGERROR("%s: Not a video device",
                "Device major number %u != expected number %d.",
        strVideoDevName.c_str(), uMajor, VideoDevMajor);
    return RC_ERROR;
  }

  return (int)uMinor;
}
