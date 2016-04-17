////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Library:   librnr_cam
//
// File:      CameraGst.cxx
//
/*! \file
 *
 * $LastChangedDate: 2013-07-15 11:45:50 -0600 (Mon, 15 Jul 2013) $
 * $Rev: 3131 $
 *
 * \brief GStreamer video and still image camera class implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright
 * (C) 2012-2013.  RoadNarrows LLC.
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

#include <stdio.h>
#include <stdlib.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/Camera.h"
#include "rnr/CameraGst.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include <gst/interfaces/xoverlay.h>
#include <gst/gst.h>

#include <gtk/gtk.h>
#include <gdk/gdkx.h>

using namespace std;
using namespace rnr;
using namespace cv;


//-----------------------------------------------------------------------------
// GStreamer Derived Camera Class
//-----------------------------------------------------------------------------

CameraGst::CameraGst(const std::string &strVideoDevName,
                     const CamRes      &resVideo,
                     const CamRes      &resImage) :
    Camera(strVideoDevName, resVideo, resImage)
{
  makePipeline();
  makeTmpFile();
}

CameraGst::~CameraGst()
{
  stopVideo();

  g_object_unref(G_OBJECT(m_pPipeline));
  
  if( m_bufTmpName[0] != 0 )
  {
    remove(m_bufTmpName);
  }
}

int CameraGst::startVideo(const CamRes &resVideo)
{
  CamRes  res = resVideo;

  // default is the current video resolution
  if( isEqResolution(res, CamResDft) )
  {
    res = m_resVideo;
  }
  
  // start video with the given resolution
  if( !isCameraRunning() )
  {
    setCameraResolution(res);
    startPipeline();
  }

  // resolution is different from current setting
  else if( !isEqResolution(res, m_resCurrent) )
  {
    stopPipeline();
    setCameraResolution(res);
    startPipeline();
  }

  // camera already running at target resolution
  else
  {
    LOGDIAG3("Camera already started with target resolution.");
  }

  m_resVideo = m_resCurrent;

  LOGDIAG3("Capturing %dx%d video.", m_resVideo.width, m_resVideo.height);

  return OK;
}

int CameraGst::stopVideo()
{
  if( !isCameraRunning() )
  {
    LOGDIAG3("Video capture already stoped.");
  }

  else
  {
    stopPipeline();
    LOGDIAG3("Video capturing stopped.");
  }

  return OK;
}

int CameraGst::grabFrame(Mat &frame)
{
  if( !isCameraRunning() )
  {
    LOGERROR("No video capture pipeline started.");
    return RC_ERROR;
  }

  // grab a frame 
  else 
  {
    return clickImage(frame, m_resVideo);
  }
}

int CameraGst::clickImage(Mat &img, const CamRes &resImage)
{  
  CamRes  resVideoPrev    = m_resVideo;
  bool    bCamWasRunning  = isCameraRunning();
  CamRes  res             = resImage;
  uint_t  usecSleep       = 10000;      // 10 milliseconds
  uint_t  usecMax         = 10000000;   // 10 seconds
  uint_t  usec;
  int     rc;

  if( m_bTakingImage || (m_nSignalId != 0) )
  {
    LOGDIAG3("Image capture already in progress.");
    return RC_ERROR;
  }

  m_bTakingImage = true;

  // default is the current image resolution
  if( isEqResolution(res, CamResDft) )
  {
    res = m_resImage;
  }

  // new resolution
  if( !isEqResolution(res, m_resCurrent) )
  {
    stopPipeline();
    setCameraResolution(res);
  }

  // start camera
  if( !isCameraRunning() )
  {
    startPipeline();
  }

  m_resImage = res;

  // take the photo by connecting the handoff signal
  m_nSignalId = g_signal_connect(G_OBJECT(m_pElemImgSink),
                                  "handoff",
                                  G_CALLBACK(clickCb),
                                  this);

  //
  // Block wait with timeout
  //
  for(usec = 0; (usec < usecMax) && (m_nSignalId != 0); usec += usecSleep)
  {
    gtk_main_iteration_do(FALSE);
    
    if( m_nSignalId != 0 )
    {
      usleep(usecSleep);
    }
  }
  
  // timed out
  if( m_nSignalId != 0 )
  {
    // g_signal_handler_disconnect(G_OBJECT(m_pElemImgSink), m_nSignalId);
    m_nSignalId = 0;
    LOGERROR("Timedout taking still image.");
    rc = RC_ERROR;
  }

  // got an image
  else
  {
    img = imread(m_bufTmpName, CV_LOAD_IMAGE_COLOR);
    LOGDIAG3("Took a %dx%d still image.", img.cols, img.rows);
    rc = OK;
  }

  // video was running
  if( bCamWasRunning )
  {
    // restart at different resolution
    if( !isEqResolution(m_resVideo, m_resCurrent) )
    {
      startVideo(m_resVideo);
    }
    // else live running at current resolution
  }

  // video only started to take image - stop
  else
  {
    stopVideo();
  }

  m_bTakingImage = false;

  return rc;
}

void CameraGst::clickCb(GstElement *element,
                        GstBuffer  *buffer,
                        GstPad     *pad,
                        void       *user_data)
{
  CameraGst *pCam = (CameraGst *)user_data;

  GstCaps            *caps;
  const GstStructure *structure;
  int                 width, height, stride;
  GdkPixbuf          *pixbuf;
  const int           bits_per_pixel = 8;
  guchar             *data;

  caps      = gst_buffer_get_caps(buffer);
  structure = gst_caps_get_structure(caps, 0);

  gst_structure_get_int(structure, "width", &width);
  gst_structure_get_int(structure, "height", &height);

  stride = buffer->size / height;

  //
  // Only copy the data if we're giving away a pixbuf, not if we're throwing
  // everything away straight away
  // 
  if (pCam->m_bufTmpName[0] != 0 )
  {
    data = NULL;
  }
  else
  {
    data = (guchar *)g_memdup(GST_BUFFER_DATA(buffer), buffer->size);
  }

  pixbuf = gdk_pixbuf_new_from_data(data? data: GST_BUFFER_DATA(buffer),
                          GDK_COLORSPACE_RGB,
                          FALSE, bits_per_pixel, width, height, stride,
                          data? (GdkPixbufDestroyNotify)g_free: NULL, NULL);

  g_signal_handler_disconnect(G_OBJECT(pCam->m_pElemImgSink),
                              pCam->m_nSignalId);

  if (pCam->m_bufTmpName[0] != 0 )
  {
    gdk_pixbuf_save(pixbuf, pCam->m_bufTmpName, "jpeg", NULL, NULL);
    g_object_unref(G_OBJECT(pixbuf));
  }

  pCam->m_resImage.width  = width;
  pCam->m_resImage.height = height;

  pCam->m_nSignalId = 0;
}

void CameraGst::autoFocus()
{
}

CamRes CameraGst::setCameraResolution(const CamRes &res)
{
  bool      bWasRunning;
  GstCaps  *caps;

  // camera pipeline must be stopped before applying capabilities
  if( (bWasRunning = isCameraRunning()) )
  {
    stopPipeline();
  }

  caps = gst_caps_new_simple("video/x-raw-yuv",
                              "width",  G_TYPE_INT, res.width,
                              "height", G_TYPE_INT, res.height,
                              NULL);

  g_object_set(G_OBJECT(m_pElemCamFilter), "caps", caps, NULL);
  gst_caps_unref(caps);

  m_resCurrent = res;

  if( bWasRunning )
  {
    startPipeline();
  }

  return m_resCurrent;
}

void CameraGst::setXid(gulong uXid)
{
  m_uVidWinXid = uXid;

  //g_object_set(m_pElemVidGenSrc, "xid", m_uVidWinXid, NULL);
}

void CameraGst::startPipeline()
{
  if( !m_bCameraRunning )
  {
    gst_element_set_state(m_pPipeline, GST_STATE_PLAYING);
    m_bCameraRunning = true;
  }
}

void CameraGst::stopPipeline()
{
  if( m_bCameraRunning )
  {
    gst_element_set_state(m_pPipeline, GST_STATE_NULL);
    m_bCameraRunning = false;
  }
}

int CameraGst::makePipeline()
{
  const char *sFactoryName;
  GstElement *pElemVidSrc;
  GstBus     *bus;
  gboolean    ok;
  int         rc;

  m_pPipeline       = NULL;
  m_pBinCameraSrc   = NULL;
  m_pElemCamFilter  = NULL;
  m_pBinVideoSink   = NULL;
  m_pElemVidText    = NULL;
  m_pBinImageSink   = NULL;
  m_pElemImgSink    = NULL;
  m_nSignalId       = 0;
  m_uVidWinXid      = 0;

  gst_init(NULL, NULL);

  m_pPipeline = gst_pipeline_new("pipeline");

  if( m_pPipeline == NULL )
  {
    LOGERROR("Failed to create new gstreamer pipeline.");
    m_bFatal = true;
    return RC_ERROR;
  }

  if( (rc = makeCameraSrcBin()) < 0 )
  {
    LOGERROR("Failed to create gstreamer camera source bin.");
    m_bFatal = true;
    return rc;
  }

  if( (rc = makeVideoSinkBin()) < 0 )
  {
    LOGERROR("Failed to create gstreamer video display sink bin.");
    m_bFatal = true;
    return rc;
  }

  if( (rc = makeImageSinkBin()) < 0 )
  {
    LOGERROR("Failed to create gstreamer still image sink bin.");
    m_bFatal = true;
    return rc;
  }

  gst_bin_add_many(GST_BIN(m_pPipeline),
                    m_pBinCameraSrc,
                    m_pBinVideoSink,
                    m_pBinImageSink,
                    NULL);

  ok = gst_element_link_many(
                    m_pBinCameraSrc,
                    m_pBinVideoSink,
                    m_pBinImageSink,
                    NULL);

  if( !ok )
  {
    LOGERROR("Failed to link gstreamer pipeline.");
    m_bFatal = true;
    return RC_ERROR;
  }

  //
  // Set up pipeline bus async message callback and synchronous handler
  //
  bus = gst_pipeline_get_bus(GST_PIPELINE(m_pPipeline));
  gst_bus_add_signal_watch(bus);
  g_signal_connect(G_OBJECT(bus), "message", G_CALLBACK(gstBusMsgCb), this);
  gst_bus_set_sync_handler(bus, (GstBusSyncHandler)gstBusSyncHandler, this);
  gst_object_unref(bus);

  return OK;
}

int CameraGst::makeCameraSrcBin()
{
  const char *sFactoryName;
  GstElement *pElemCamSrc;
  GstElement *pElemCamIdentity;
  GstCaps    *caps;
  GstPad     *pad;
  gboolean    ok;

  m_pBinCameraSrc = gst_bin_new("camera_bin");

  if( m_pBinCameraSrc == NULL )
  {
    LOGERROR("Failed to create new gstreamer camera bin.");
    m_bFatal = true;
    return RC_ERROR;
  }

  //
  // Source element
  //
  sFactoryName = "v4l2src";
  pElemCamSrc = gst_element_factory_make(sFactoryName, "cam_src"); 

  if( pElemCamSrc == NULL )
  {
    LOGERROR("Failed to create gstreamer element: %s.", sFactoryName);
    m_bFatal = true;
    return RC_ERROR;
  }

  g_object_set(G_OBJECT(pElemCamSrc),
      "device", m_strVideoDevName.c_str(),
      NULL);

  //
  // Filter Element
  //
  sFactoryName = "capsfilter";
  m_pElemCamFilter = gst_element_factory_make(sFactoryName, "cam_caps");

  if( m_pElemCamFilter == NULL )
  {
    LOGERROR("Failed to create gstreamer element: %s.", sFactoryName);
    m_bFatal = true;
    return RC_ERROR;
  }

  caps = gst_caps_new_simple("video/x-raw-yuv",
                              "width",  G_TYPE_INT, m_resVideo.width,
                              "height", G_TYPE_INT, m_resVideo.height,
                              NULL);

  g_object_set(G_OBJECT(m_pElemCamFilter), "caps", caps, NULL);
  gst_caps_unref(caps);

  m_resVideo = m_resVideo;

  //
  // Identity element
  //
  sFactoryName = "identity";
  pElemCamIdentity = gst_element_factory_make(sFactoryName, "cam_identity"); 

  if( pElemCamIdentity == NULL )
  {
    LOGERROR("Failed to create gstreamer element: %s.", sFactoryName);
    m_bFatal = true;
    return RC_ERROR;
  }

  gst_bin_add_many(GST_BIN(m_pBinCameraSrc),
                    pElemCamSrc,
                    m_pElemCamFilter,
                    pElemCamIdentity,
                    NULL);

  ok = gst_element_link_many(
                    pElemCamSrc,
                    m_pElemCamFilter,
                    pElemCamIdentity,
                    NULL);

  if( !ok )
  {
    LOGERROR("Failed to link camera gstreamer elements.");
    m_bFatal = true;
    return RC_ERROR;
  }

  // 
  // Add souce ghostpad since bins normally do not have pads.
  //
  pad = gst_element_get_pad(pElemCamIdentity, "src");
  gst_element_add_pad(m_pBinCameraSrc, gst_ghost_pad_new("src", pad));
  gst_object_unref(GST_OBJECT(pad));

  return OK;
}

int CameraGst::makeVideoSinkBin()
{
  const char *sFactoryName;
  GstElement *pElemVidIdentity;
  GstElement *pElemVidTee;
  GstElement *pElemVidSaveQueue;
  GstElement *pElemVidDispQueue;
  GstElement *pElemVidScale;
  GstElement *pElemVidFilter;
  GstElement *pElemVidFfmpeg;
  GstCaps    *caps;
  GstPad     *pad;
  gboolean    ok;

  m_pBinVideoSink = gst_bin_new("video_bin");

  if( m_pBinVideoSink == NULL )
  {
    LOGERROR("Failed to create new gstreamer video bin.");
    m_bFatal = true;
    return RC_ERROR;
  }

  //
  // Tee element
  //
  sFactoryName = "tee";
  pElemVidTee = gst_element_factory_make(sFactoryName, "vid_tee"); 
 
  if( pElemVidTee == NULL )
  {
    LOGERROR("Failed to create gstreamer element: %s.", sFactoryName);
    m_bFatal = true;
    return RC_ERROR;
  }

  //
  // Save queue element
  //
  sFactoryName = "queue";
  pElemVidSaveQueue = gst_element_factory_make(sFactoryName, "vid_save_queue"); 

  if( pElemVidSaveQueue == NULL )
  {
    LOGERROR("Failed to create gstreamer element: %s.", sFactoryName);
    m_bFatal = true;
    return RC_ERROR;
  }

  g_object_set(G_OBJECT(pElemVidSaveQueue),
      "max-size-buffers", 2,
      NULL);

  //
  // Video display queue element
  //
  sFactoryName = "queue";
  pElemVidDispQueue = gst_element_factory_make(sFactoryName, "vid_disp_queue"); 

  if( pElemVidDispQueue == NULL )
  {
    LOGERROR("Failed to create gstreamer element: %s.", sFactoryName);
    m_bFatal = true;
    return RC_ERROR;
  }

  //
  // Scale element
  //
  sFactoryName = "videoscale";
  pElemVidScale = gst_element_factory_make(sFactoryName, "vid_scale"); 

  if( pElemVidScale == NULL )
  {
    LOGERROR("Failed to create gstreamer element: %s.", sFactoryName);
    m_bFatal = true;
    return RC_ERROR;
  }

  // use bilinear scaling
  g_object_set(pElemVidScale, "method", 1, NULL);

  //
  // Filter Element
  //
  sFactoryName = "capsfilter";
  pElemVidFilter = gst_element_factory_make(sFactoryName, "vid_caps");

  if( pElemVidFilter == NULL )
  {
    LOGERROR("Failed to create gstreamer element: %s.", sFactoryName);
    m_bFatal = true;
    return RC_ERROR;
  }

  caps = gst_caps_new_simple("video/x-raw-yuv",
                              "width",  G_TYPE_INT, 320,
                              "height", G_TYPE_INT, 240,
                              NULL);

  g_object_set(G_OBJECT(pElemVidFilter), "caps", caps, NULL);
  gst_caps_unref(caps);

  //
  // Text overlay element
  //
  sFactoryName = "textoverlay";
  m_pElemVidText = gst_element_factory_make(sFactoryName, "vid_text"); 

  if( m_pElemVidText == NULL )
  {
    LOGERROR("Failed to create gstreamer element: %s.", sFactoryName);
    m_bFatal = true;
    return RC_ERROR;
  }

  // no text
  g_object_set(m_pElemVidText,
      "text", "",
      "font-desc", "Aria 32",
      NULL);

  //
  // Color space element
  //
  sFactoryName = "ffmpegcolorspace";
  pElemVidFfmpeg = gst_element_factory_make(sFactoryName, "vid_color"); 
 
  if( pElemVidFfmpeg == NULL )
  {
    LOGERROR("Failed to create gstreamer element: %s.", sFactoryName);
    m_bFatal = true;
    return RC_ERROR;
  }

  //
  // Display sink element
  //
  sFactoryName = "gconfvideosink";
  m_pElemVidSink = gst_element_factory_make(sFactoryName, "vid_sink"); 

  if( m_pElemVidSink == NULL )
  {
    LOGERROR("Failed to create gstreamer element: %s.", sFactoryName);
    m_bFatal = true;
    return RC_ERROR;
  }

  gst_bin_add_many(GST_BIN(m_pBinVideoSink),
                    pElemVidTee,
                    pElemVidSaveQueue,
                    pElemVidDispQueue,
                    pElemVidScale,
                    pElemVidFilter,
                    m_pElemVidText,
                    pElemVidFfmpeg,
                    m_pElemVidSink,
                    NULL);

  ok = gst_element_link_many(
                    pElemVidTee,
                    pElemVidSaveQueue,
                    NULL);

  if( !ok )
  {
    LOGERROR("Failed to link video tee save gstreamer elements.");
    m_bFatal = true;
    return RC_ERROR;
  }

  ok = gst_element_link_many(
                    pElemVidTee,
                    pElemVidDispQueue,
                    pElemVidScale,
                    pElemVidFilter,
                    m_pElemVidText,
                    pElemVidFfmpeg,
                    m_pElemVidSink,
                    NULL);

  if( !ok )
  {
    LOGERROR("Failed to link video tee display gstreamer elements.");
    m_bFatal = true;
    return RC_ERROR;
  }

  // 
  // Add sink ghostpad since bins normally do not have pads.
  //
  pad = gst_element_get_pad(pElemVidTee, "sink");
  gst_element_add_pad(m_pBinVideoSink, gst_ghost_pad_new("sink", pad));
  gst_object_unref(GST_OBJECT(pad));

  // 
  // Add source ghostpad to save stream
  //
  pad = gst_element_get_pad(pElemVidSaveQueue, "src");
  gst_element_add_pad(m_pBinVideoSink, gst_ghost_pad_new("src", pad));
  gst_object_unref(GST_OBJECT(pad));

  return OK;
}

int CameraGst::makeImageSinkBin()
{
  const char *sFactoryName;
  GstElement *pElemImgFfmpeg;
  GstCaps    *caps;
  GstPad     *pad;
  gboolean    ok;

  m_pBinImageSink = gst_bin_new("image_bin");

  if( m_pBinImageSink == NULL )
  {
    LOGERROR("Failed to create new gstreamer image bin.");
    m_bFatal = true;
    return RC_ERROR;
  }

  //
  // Colorspace element
  //
  sFactoryName = "ffmpegcolorspace";
  pElemImgFfmpeg = gst_element_factory_make(sFactoryName, "img_color"); 
 
  if( pElemImgFfmpeg == NULL )
  {
    LOGERROR("Failed to create gstreamer element: %s.", sFactoryName);
    m_bFatal = true;
    return RC_ERROR;
  }

  //
  // Sink element
  //
  sFactoryName = "fakesink";
  m_pElemImgSink = gst_element_factory_make(sFactoryName, "img_sink");
  
  if( m_pElemImgSink == NULL )
  {
    LOGERROR("Failed to create gstreamer element: %s.", sFactoryName);
    m_bFatal = true;
    return RC_ERROR;
  }

  g_object_set(G_OBJECT(m_pElemImgSink),
      "signal-handoffs", TRUE,
      NULL);

  //
  // Add elements to the pipeline
  //
  gst_bin_add_many(GST_BIN(m_pBinImageSink),
                    pElemImgFfmpeg,
                    m_pElemImgSink,
                    NULL);

  caps = gst_caps_new_simple("video/x-raw-rgb",
                              "bpp",    G_TYPE_INT, 24,
                              "depth",  G_TYPE_INT, 24,
                              NULL);
  ok = gst_element_link_filtered(pElemImgFfmpeg, m_pElemImgSink, caps);

  if( !ok )
  {
    LOGERROR("Failed to set filtered link.");
    m_bFatal = true;
    return RC_ERROR;
  }

  gst_caps_unref(caps);


  // 
  // Add sink ghostpad since bins normally do not have pads.
  //
  pad = gst_element_get_pad(pElemImgFfmpeg, "sink");
  gst_element_add_pad(m_pBinImageSink, gst_ghost_pad_new("sink", pad));
  gst_object_unref(GST_OBJECT(pad));

  return OK;
}

#if 0 // EXAMPLE
void CameraGst::changeSink(GstElement *pBinNewSink)
{
  GstElement *pBinOldSink;
  GstElement *pBinNewSink;

  if( eNewMode == m_eCameraMode )
  {
    return;
  }

  gst_element_set_state(m_pPipeline, GST_STATE_NULL);

  if( eNewMode == CameraModePhoto )
  {
    pBinOldSink = m_pBinVideoSink;
    pBinNewSink = m_pBinImageSink;
  }
  else
  {
    pBinOldSink = m_pBinImageSink;
    pBinNewSink = m_pBinVideoSink;
  }

  gst_element_unlink(m_pBinCameraSrc, pBinOldSink);
  gst_object_ref(pBinOldSink);
  gst_bin_remove(GST_BIN(m_pPipeline), pBinOldSink);

  gst_bin_add(GST_BIN(m_pPipeline), pBinNewSink);
  gst_element_link(m_pBinCameraSrc, pBinNewSink);

  m_eCameraMode = eNewMode;

  LOGDIAG3("%s Mode.", (m_eCameraMode == CameraModePhoto? "Photo": "Video"));
}
#endif

void CameraGst::gstBusMsgCb(GstBus     *bus,
                              GstMessage *message,
                              void       *user_data)
{
  CameraGst *pCam = (CameraGst *)user_data;

  switch( GST_MESSAGE_TYPE(message) )
  {
    case GST_MESSAGE_EOS:
      LOGDIAG3("GST: End of stream.");
      break;

    case GST_MESSAGE_ERROR:
      {
        gchar  *debug;
        GError *error;

        gst_message_parse_error(message, &error, &debug);
        g_free(debug);
        LOGERROR("GST: %s.", error->message);
        g_error_free(error);
      }
      break;

    default:
      //fprintf(stderr, "DBG: GST: busmsg=%u\n", GST_MESSAGE_TYPE(message));
      break;
  }
}

GstBusSyncReply CameraGst::gstBusSyncHandler(GstBus     *bus,
                                               GstMessage *message,
                                               gpointer    user_data)
{
  CameraGst *pCam = (CameraGst *)user_data;

  if( GST_MESSAGE_TYPE(message) == GST_MESSAGE_EOS )
  {
    LOGDIAG3("GST: Sync: End of stream.");
    return GST_BUS_PASS;
  }

  // ignore anything but 'prepare-xwindow-id' element messages
  if( GST_MESSAGE_TYPE(message) != GST_MESSAGE_ELEMENT )
  {
    return GST_BUS_PASS;
  }

  if( !gst_structure_has_name(message->structure, "prepare-xwindow-id") )
  {
    return GST_BUS_PASS;
  }

  if( pCam->m_uVidWinXid != 0 )
  {
    GstXOverlay *xoverlay;

    xoverlay = GST_X_OVERLAY(GST_MESSAGE_SRC(message));

    if( g_object_class_find_property(G_OBJECT_GET_CLASS(xoverlay),
                                    "force-aspect-ratio") )
    {
      g_object_set(G_OBJECT(xoverlay), "force-aspect-ratio", TRUE, NULL);
    }

    // GST_MESSAGE_SRC (message) will be the video sink element
    gst_x_overlay_set_xwindow_id(xoverlay, pCam->m_uVidWinXid);
    // new method in 10.31
    // gst_x_overlay_set_window_handle(xoverlay, pCam->m_uVidWinXid);
  }
  else
  {
   LOGERROR("Should have obtained video_window_xid by now!");
  }

  gst_message_unref(message);

  return GST_BUS_DROP;
}

#if 0
gint64 CameraGst::onBlockVideoSelector(GstElement *element,
                                         gpointer    user_data)
{
  //fprintf(stderr, "DBG: onBlock\n");
  return 0;
}

gint64 CameraGst::onSwitchVideoSelector(GstElement *element,
                                          GstPad      *pad,
                                          gint64      stop_time,
                                          gint64      start_time,
                                          gpointer    user_data)
{
  //fprintf(stderr, "DBG: onSwitch\n");
  return 0;
}
#endif

void CameraGst::makeTmpFile()
{
  string  strX("XXXXXX");     // pattern to be replaced
  string  strSuffix(".jpg");  // suffix
  int     fd;                 // file descriptor

  sprintf(m_bufTmpName, "/tmp/camegst-%s%s", strX.c_str(), strSuffix.c_str());

#ifdef HAVE_MKSTEMPS
  if( (fd = mkstemps(m_bufTmpName, strSuffix.length())) < 0 )
  {
    LOGERROR("mkstemps(%s, %zu) failed.\n", m_bufTmpName, strSuffix.length());
    m_bufTmpName[0] = 0;
    m_bFatal = true;
  }

#else
  char tmp[PATH_MAX];

  sprintf(tmp, "/tmp/camgst-%s", strX.c_str());

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
