////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_cam
//
// File:      CameraGst.h
//
/*! \file
 *
 * $LastChangedDate: 2013-07-13 13:54:59 -0600 (Sat, 13 Jul 2013) $
 * $Rev: 3122 $
 *
 * \brief Gstreamer video and still image camera class.
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

#ifndef _RNR_CAMERA_GST_H
#define _RNR_CAMERA_GST_H

#include <limits.h>

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "opencv/cv.h"

#include <gst/gst.h>

namespace rnr
{
  //---------------------------------------------------------------------------
  // GStreamer Camera Derived Class
  //---------------------------------------------------------------------------

  /*!
   * \brief GStreamer implementation of the camera class. The video is streamed
   * via a Gstreamer/GTK callback mechanism.
   */
  class CameraGst : public Camera
  {
  public:
    /*
     * \brief Default initialization constructor.
     *
     * \param strVideoDevName Video camera device name.
     * \param resVideo        Video resolution.
     * \param resImage        Image resolution.
     */
    CameraGst(const std::string &strVideoDevName="/dev/video0",
              const CamRes      &resVideo=CamResQVGA,
              const CamRes      &resImage=CamResVGA);

    /*!
     * \brief Destructor.
     */
    virtual ~CameraGst();

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
     * \brief Set X identifier associated with widget receiving streaming video.
     *
     * \param uXid  X identifier.
     */
    void setXid(gulong uXid);

    /*!
     * \brief Overlay text on video stream.
     *
     * \param sText   Null-terminated string.
     */
    void setTextOverlay(const char *sText)
    {
      g_object_set(m_pElemVidText, "text", sText, NULL);
    }

    /*!
     * \brief Clear text overlay on video stream.
     */
    void clearTextOverlay()
    {
      g_object_set(m_pElemVidText, "text", "", NULL);
    }

  protected:
    GstElement *m_pPipeline;            ///< camera pipeline
    GstElement *m_pBinCameraSrc;        ///< camera source bin
    GstElement *m_pElemCamFilter;       ///< camera properties element
    GstElement *m_pBinVideoSink;        ///< video sink bin
    GstElement *m_pElemVidText;         ///< video text overlay
    GstElement *m_pElemVidSink;         ///< video sink 
    GstElement *m_pBinImageSink;        ///< still image sink bin
    GstElement *m_pElemImgSink;         ///< still image sink
    int         m_nSignalId;            ///< camera still image callback signal
    gulong      m_uVidWinXid;           ///< overlay window X-windows id
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
     * \brief Start the camera and the pipeline.
     */
    void startPipeline();

    /*!
     * \brief Stop the camera and the pipeline.
     */
    void stopPipeline();

    /*!
     * \brief Make gstreamer video pipeline.
     */
    int makePipeline();

    /*!
     * \brief Video source bin.
     *
     * \verbatim
     *           |---------------------------------|
     * camera - [] v4l2src - capsfilter - identity [] -
     *           |---------------------------------|
     * \endverbatim
     *
     * \copydoc doc_return_std
     */
    int makeCameraSrcBin();

    /*!
     * \brief Video sink bin.
     *
     * \verbatim
     *    |------------------------------------------------------------------|
     *    |       - queue - scale - capsfilter - colorspace - gconfvideosink |
     *    |      /                                                           |
     * - [] tee -                                                            |
     *    |      \                                                           |
     *    |       - queue (to save image)                                    []
     *    |------------------------------------------------------------------|
     * \endverbatim
     *
     * \copydoc doc_return_std
     */
    int makeVideoSinkBin();

    /*!
     * \brief Image sink bin.
     *
     * \verbatim
     *    |-----------------------|  ~~  |-----
     * - [] colorspace - fakesink |      | callback
     *    |-----------------------|  ~~  |-----
     * \endverbatim
     *
     * \copydoc doc_return_std
     */
    int makeImageSinkBin();

    //void changeSink(GstElement *pBinNewSink);

    static GstBusSyncReply gstBusSyncHandler(GstBus     *bus,
                                             GstMessage *message,
                                             gpointer    user_data);

    /*!
     * \brief Gstream pipeline synchronous bus handler.
     *
     * \param bus         GStream bus.
     * \param message     Bus message.
     * \param user_data   User data.
     *
     * \return Returns GST_BUS_PASS if message not relevant for the stream.\n
     * Returns GST_BUS_DROP otherwise.
     */
    static void gstBusMsgCb(GstBus     *bus,
                            GstMessage *message,
                            gpointer    user_data);

    /*!
     * \brief Still image capture asynchronouse callback.
     *
     * This function is called when an image buffer from the camera has been
     * retrieved.
     *
     * \param element   GStreamer element.
     * \param buffer    Image buffer.
     * \param pad       GStreamer pad.
     * \param user_data User data (this)
     */
    static void clickCb(GstElement *element,
                        GstBuffer  *buffer,
                        GstPad     *pad,
                        void       *user_data);

#if 0
    static gint64 onBlockVideoSelector(GstElement *element,
                                       gpointer    user_data);

    static gint64 onSwitchVideoSelector(GstElement *element,
                                        GstPad      *pad,
                                        gint64      stop_time,
                                        gint64      start_time,
                                        gpointer    user_data);
#endif

    /*!
     * \brief Make unique temporary file.
     */
    void makeTmpFile();

  };

} // namespace rnr


#endif // _RNR_CAMERA_GST_H
