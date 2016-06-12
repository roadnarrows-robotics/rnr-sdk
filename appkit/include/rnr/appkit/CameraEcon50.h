////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Library:   librnr_cam
//
// File:      CameraEcon50.h
//
/*! \file
 *
 * $LastChangedDate: 2013-07-15 11:45:50 -0600 (Mon, 15 Jul 2013) $
 * $Rev: 3131 $
 *
 * \brief Econ 5.0 megapixel video and still image camera class.
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

#ifndef _RNR_CAMERA_ECON_50_H
#define _RNR_CAMERA_ECON_50_H

#if defined(ARCH_overo)

#include <limits.h>
#include <math.h>

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/appkit/Camera.h"
#include "rnr/appkit/CameraCv.h"

#include "econ/econ50.h"

#include "opencv/cv.h"

namespace rnr
{
  /*!
   * \brief Special Econ supported camera resolutions.
   */
  const CamRes CamResEcon5_0MP = CamRes2592x1944;  ///< max camera resolution


  //---------------------------------------------------------------------------
  // Econ 5.0MP Camera Derived Class (work in progress)
  //---------------------------------------------------------------------------

  /*!
   * \brief Econ 5.0MP implementation of the camera class.
   *
   * The video is streamed via OpenCv calls. The still images are taken via
   * econ methods to gain higher resolution (theoretically).
   *
   * \todo The CameraEcon50 class is not fully implemented.
   */
  class CameraEcon50 : public CameraCv
  {
  public:
    /*
     * \brief Default initialization constructor.
     *
     * \param strVideoDevName Video camera device name.
     * \param resVideo        Video resolution.
     * \param resImage        Image resolution.
     */
    CameraEcon50(const std::string &strVideoDevName="/dev/video0",
                 const CamRes      &resVideo=CamResQVGA,
                 const CamRes      &resImage=CamResVGA);

    /*!
     * \brief Destructor.
     */
    virtual ~CameraEcon50();

    /*!
     * \brief Take a still image.
     *
     * \note Any OpenCV video capture must be stopped before executing econ
     * image capture.
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

  protected:
    ecam50     *m_eCam;                 ///< econ 5.0 MP camera
    char        m_bufTmpName[PATH_MAX]; ///< temporary file name buffer

    /*!
     * \brief Set the camera resolution in either video or still image mode.
     *
     * \param res   Target camera resolution.
     *
     * \return Actual resolution set.
     */
    virtual CamRes setCameraResolution(const CamRes &res);

    /*!
     * \brief Make unique temporary file.
     */
    void makeTmpFile();
  };

} // namespace rnr

#endif // defined(ARCH_overo)

#endif // _RNR_CAMERA_ECON_50_H
