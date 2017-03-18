////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Library:   librnr_cam
//
// File:      CameraEcon32.cxx
//
/*! \file
 *
 * $LastChangedDate: 2013-07-15 11:45:50 -0600 (Mon, 15 Jul 2013) $
 * $Rev: 3131 $
 *
 * \brief Econ 3.2 megapixel video and still image camera class implementation.
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

#if defined(ARCH_overo)

#include <stdio.h>
#include <stdlib.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/appkit/Camera.h"
#include "rnr/appkit/CameraCv.h"
#include "rnr/appkit/CameraEcon32.h"

#include "opencv/cv.h"

#include "econ/econ32.h"


using namespace std;
using namespace rnr;
using namespace cv;


//-----------------------------------------------------------------------------
// Econ 3.2MP Derived Camera Class (TBD)
//-----------------------------------------------------------------------------

CameraEcon32::CameraEcon32(const std::string &strVideoDevName,
                           const CamRes      &resVideo,
                           const CamRes      &resImage) :
    CameraCv(strVideoDevName, resVideo, resImage)
{
  makeTmpFile();
}

CameraEcon32::~CameraEcon32()
{
  stopVideo();

  if( m_bufTmpName[0] != 0 )
  {
    remove(m_bufTmpName);
  }
}

int CameraEcon32::clickImage(Mat &img, const CamRes &resImage)
{
  CamRes  resVideoPrev    = m_resVideo;
  bool    bCamWasRunning  = isCameraRunning();
  CamRes  res             = resImage;
  
  if( m_eCam == NULL )
  {
    LOGERROR("No ecam object.");
    return RC_ERROR;
  }

  m_bTakingImage = true;

  // default is the current image resolution
  if( isEqResolution(res, CamResDft) )
  {
    res = m_resImage;
  }

  stopVideo();

  m_resImage = setCameraResolution(res);

#if 0 // TODO RDK
  take_snap(m_eCam);

  save_snap(m_eCam, m_bufTmpName);
#endif // 0 TODO RDK

  img = imread(m_bufTmpName, CV_LOAD_IMAGE_COLOR);

  flip(img, img, 0);

  LOGDIAG3("Took a %dx%d still image.", img.cols, img.rows);

  // video was running
  if( bCamWasRunning )
  {
    startVideo(m_resVideo);
  }

  m_bTakingImage = false;

  return OK;
}

void CameraEcon32::autoFocus()
{
}

CamRes CameraEcon32::setCameraResolution(const CamRes &res)
{
  int oldWidth  = m_eCam->fmt.fmt.pix.width;
  int oldHeight = m_eCam->fmt.fmt.pix.height;

  m_eCam->fmt.fmt.pix.width  = res.width;
  m_eCam->fmt.fmt.pix.height = res.height;

  m_eCam->fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  m_eCam->fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
  m_eCam->fmt.fmt.pix.bytesperline = m_eCam->fmt.fmt.pix.width * 2; // BHW
  m_eCam->fmt.fmt.pix.priv = 0;
  m_eCam->fmt.fmt.pix.sizeimage = m_eCam->fmt.fmt.pix.width * \
          m_eCam->fmt.fmt.pix.height * 2;

  if (ioctl(m_eCam->fd_v4l2, VIDIOC_S_FMT, &m_eCam->fmt) < 0)
  {
    LOGERROR("Unable to set camera resolution %dx%d", res.width, res.height);
    m_eCam->fmt.fmt.pix.width  = oldWidth;
    m_eCam->fmt.fmt.pix.height = oldHeight;
  }

  m_resCurrent.width  = m_eCam->fmt.fmt.pix.width;
  m_resCurrent.height = m_eCam->fmt.fmt.pix.height;

  return m_resCurrent;
}

void CameraEcon32::makeTmpFile()
{
  string  strX("XXXXXX");     // pattern to be replaced
  string  strSuffix(".bmp");  // suffix
  int     fd;                 // file descriptor

  sprintf(m_bufTmpName, "/tmp/camecon32-%s%s", strX.c_str(), strSuffix.c_str());

#ifdef HAVE_MKSTEMPS
  if( (fd = mkstemps(m_bufTmpName, strSuffix.length())) < 0 )
  {
    LOGERROR("mkstemps(%s, %zu) failed.\n", m_bufTmpName, strSuffix.length());
    m_bufTmpName[0] = 0;
    m_bFatal = true;
  }

#else
  char tmp[PATH_MAX];

  sprintf(tmp, "/tmp/camecon32-%s", strX.c_str());

  if( (fd = mkstemp(tmp)) < 0 )
  {
    LOGERROR("mkstemp(%s) failed.\n", tmp);
    m_bufTmpName[0] = 0;
    m_bFatal = true;
  }

  else
  {
    rename(tmp, m_bufTmpName);
  }
#endif

  if( fd >= 0 )
  {
    close(fd);
  }
}

#endif // defined(ARCH_overo)
