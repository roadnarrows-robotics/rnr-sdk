////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Library:   librnr_cam
//
// File:      CameraCv.h
//
/*! \file
 *
 * $LastChangedDate: 2013-07-13 13:54:59 -0600 (Sat, 13 Jul 2013) $
 * $Rev: 3122 $
 *
 * \brief OpenCv video and still image camera class.
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

#ifndef _RNR_CAMERA_CV_H
#define _RNR_CAMERA_CV_H

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/appkit/Camera.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"

namespace rnr
{
  //---------------------------------------------------------------------------
  // OpenCv Camera Derived Class
  //---------------------------------------------------------------------------

  /*!
   * \brief OpenCv implementation of the camera class. The video is streamed
   * via OpenCv calls.
   */
  class CameraCv : public Camera
  {
  public:
    /*
     * \brief Default initialization constructor.
     *
     * \param strVideoDevName Video camera device name.
     * \param resVideo        Video resolution.
     * \param resImage        Image resolution.
     */
    CameraCv(const std::string &strVideoDevName="/dev/video0",
             const CamRes      &resVideo=CamResQVGA,
             const CamRes      &resImage=CamResVGA);

    /*!
     * \brief Destructor.
     */
    virtual ~CameraCv();

    /*!
     * \brief Start the camera streaming video.
     *
     * \param resVideo  Video resolution. If equal to CamResDft, then
     *                  the initial/last video resolution setting is used.
     *
     * \copydoc doc_return_std
     */
    virtual int startVideo(const CamRes &resVideo=CamResDft);

    /*!
     * \brief Stop the camera from streaming video.
     *
     * \copydoc doc_return_std
     */
    virtual int stopVideo();

    /*!
     * \brief Grab a image frame from the video stream.
     *
     * \param [in,out]  The image frame matrix. May be resized.
     *
     * \copydoc doc_return_std
     */
    virtual int grabFrame(cv::Mat &frame);

    /*!
     * \brief Take a still image.
     *
     * \param [in,out] img  Snap shot image taken.
     * \param resImage      Still image resolution. If equal to CamResDft, then
     *                      the initial/last image resolution setting is used.
     *
     * \copydoc doc_return_std
     */
    virtual int clickImage(cv::Mat &img, const CamRes &resImage=CamResDft);

    /*!
     * \brief Auto-focus camera.
     */
    virtual void autoFocus();

    /*!
     * \brief Get OpenCV captured object.
     *
     * \return Returns CvCapture*.
     */
    cv::VideoCapture &getCaptureObj()
    {
      return m_capture;
    }

  protected:
    cv::VideoCapture  m_capture;        ///< video capture object

    /*!
     * \brief Set the camera resolution in either video or still image mode.
     *
     * \param res   Target camera resolution.
     *
     * \return Actual resolution set.
     */
    virtual CamRes setCameraResolution(const CamRes &res);
  };

} // namespace rnr


#endif // _RNR_CAMERA_CV_H
