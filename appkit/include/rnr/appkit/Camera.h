////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_cam
//
// File:      Camera.h
//
/*! \file
 *
 * $LastChangedDate: 2013-07-13 13:54:59 -0600 (Sat, 13 Jul 2013) $
 * $Rev: 3122 $
 *
 * \brief Video and still image camera base class.
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

#ifndef _RNR_CAMERA_H
#define _RNR_CAMERA_H

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "opencv/cv.h"

#ifdef CAM_USE_ECAM_IMPL
#include "econ/econ32.h"
#endif // CAM_USE_ECAM_IMPL

namespace rnr
{
  /*!
   * \brief Camera resolution structure.
   */
  struct CamRes
  {
    int   width;    ///< width in pixels
    int   height;   ///< height in pixels
  };

  /*!
   * \brief Common 4:3 aspect ratio camera resolutions.
   */
  const CamRes CamResUndef      = {  -1,   -1}; ///< undefined resolution
  const CamRes CamResDft        = {   0,    0}; ///< default resolution
  const CamRes CamResQVGA       = { 320,  240}; ///< Quarter VGA 320 x 240 res
  const CamRes CamResVGA        = { 640,  480}; ///< VGA 640 x 480 resolution
  const CamRes CamRes1024x768   = {1024,  768}; ///< 1024 x 768 resolution
  const CamRes CamRes1440x1080  = {1440, 1080}; ///< 1440 x 1080 resolution
  const CamRes CamRes1600x1200  = {1600, 1200}; ///< 1600 x 1200 resolution
  const CamRes CamRes2048x1536  = {2048, 1536}; ///< 2048 x 1536 resolution
  const CamRes CamRes2592x1944  = {2592, 1944}; ///< 2592 x 1944 resolution

  /*!
   * \brief Common 16:10 aspect ratio camera resolutions.
   */
  const CamRes CamRes1280x800   = {1280,  800}; ///< 1280 x 800 resolution
  const CamRes CamRes1440x900   = {1440,  900}; ///< 1440 x 900 resolution

  /*!
   * \brief Common ~16:9 aspect ratio camera resolutions.
   */
  const CamRes CamRes1280x720   = {1280,	720}; ///< 1280 x 720 resolution
  const CamRes CamRes1920x1080  = {1920, 1080}; ///< 1920 x 1080 resolution

  /*!
   * \brief Default video device.
   */
  const char* const VideoDevDft   = "/dev/video0";  ///< default video device
  const int         VideoIndexDft = 0;              ///< default video index
  const int         VideoDevMajor = 81;             ///< major device number


  //---------------------------------------------------------------------------
  // Camera Base Class
  //---------------------------------------------------------------------------

  /*!
   * \brief Camera base class.
   */
  class Camera
  {
  public:
    /*
     * \brief Default initialization constructor.
     *
     * \param strVideoDevName Video camera device name.
     * \param resVideo        Video resolution.
     * \param resImage        Image resolution.
     */
    Camera(const std::string &strVideoDevName=VideoDevDft,
           const CamRes      &resVideo=CamResQVGA,
           const CamRes      &resImage=CamResVGA);

    /*!
     * \brief Destructor.
     */
    virtual ~Camera();

    /*!
     * \brief Start the camera streaming video.
     *
     * \param resVideo  Video resolution. If equal to CamResDft, then
     *                  the initial/last video resolution setting is used.
     *
     * \copydoc doc_return_std
     */
    virtual int startVideo(const CamRes &resVideo=CamResDft)
    {
      CamRes res = resVideo;

      // default is the current video resolution
      if( isEqResolution(res, CamResDft) )
      {
        res = m_resVideo;
      }

      // video resolution differs from camera's current resolution
      if( !isEqResolution(res, m_resCurrent) )
      {
        setCameraResolution(res);
      }

      m_resVideo        = m_resCurrent;
      m_bCameraRunning  = true;

      return OK;
    }

    /*!
     * \brief Stop the camera from streaming video.
     *
     * \copydoc doc_return_std
     */
    virtual int stopVideo()
    {
      m_bCameraRunning = false;

      return OK;
    }

    /*!
     * \brief Grab a image frame from the video stream.
     *
     * \param [in,out]  The image frame matrix. May be resized.
     *
     * \copydoc doc_return_std
     */
    virtual int grabFrame(cv::Mat &frame)
    {
      return RC_ERROR;
    }

    /*!
     * \brief Take a still image.
     *
     * \param [in,out] img  Snap shot image taken.
     * \param resImage      Still image resolution. If equal to CamResDft, then
     *                      the initial/last image resolution setting is used.
     *
     * \copydoc doc_return_std
     */
    virtual int clickImage(cv::Mat &img, const CamRes &resImage=CamResDft)
    {
      return RC_ERROR;
    }

    /*!
     * \brief Auto-focus camera.
     */
    virtual void autoFocus()
    {
    }

    /*!
     * \brief Test if the camera is on and running.
     *
     * \return Returns true if camera is on, else false.
     */
    bool isCameraRunning() const
    {
      return m_bCameraRunning;
    }

    /*!
     * \brief Test if a still image is currently being taken.
     *
     * \return Returns true if an image is currently being taken, else false.
     */
    bool isTakingAnImage() const
    {
      return m_bTakingImage;
    }

    /*!
     * \brief Test if camera object is in a fatal condition.
     *
     * \return Returns true or false.
     */
    bool isFatal() const
    {
      return m_bFatal;
    }

    /*!
     * \brief Check is two camera resolutions are equal.
     *
     * \param res1    Resolution 1.
     * \param res2    Resolution 2.
     *
     * \return Returns true or false.
     */
    static bool isEqResolution(const CamRes &res1, const CamRes &res2)
    {
      return (res1.width == res2.width) &&
             (res1.height == res2.height)? true: false;
    }

    /*!
     * \brief Get the video index associated with the device.
     *
     * \param strVideoDevName Video camera device name.
     *
     * \return Returns video index on success, \ref RC_ERROR(-1) on failure.
     */
    static int getVideoIndex(const std::string &strVideoDevName);

    /*!
     * \brief Get the current video resolution.
     *
     * \return Returns video resolution size.
     */
    CamRes getVideoResolution() const
    {
      return m_resVideo;
    }

    /*!
     * \brief Get the current still image resolution.
     *
     * \return Returns still image resolution size.
     */
    CamRes getImageResolution() const
    {
      return m_resImage;
    }

  protected:
    std::string   m_strVideoDevName;  ///< video device name
    int           m_nVideoIndex;      ///< video index
    CamRes        m_resCurrent;       ///< current camera resolution
    CamRes        m_resVideo;         ///< current video resolution
    CamRes        m_resImage;         ///< current still image resolution
    bool          m_bCameraRunning;   ///< camera is [not] on and running video
    bool          m_bTakingImage;     ///< taking an image is [not] finished
    bool          m_bFatal;           ///< camera instance is in a fatal state

    /*!
     * \brief Set the camera resolution in either video or still image mode.
     *
     * \param res   Target camera resolution.
     *
     * \return Actual resolution set.
     */
    virtual CamRes setCameraResolution(const CamRes &res)
    {
      m_resCurrent = res;

      return m_resCurrent;
    }
  };

} // namespace rnr


#endif // _RNR_CAMERA_H
