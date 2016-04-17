////////////////////////////////////////////////////////////////////////////////
//
// Package:     Eudoxus
//
// SubPackage:  GStreamer
//
// Plug-In:     libgstpcsfilesrc
//
// File:        gstpcsfilesrc.cxx
//
/*! \file
 *
 * $LastChangedDate: 2016-01-19 17:29:01 -0700 (Tue, 19 Jan 2016) $
 * $Rev: 4267 $
 *
 * \brief Eudoxus GStreamer Point Cloud Stream source element plug-in
 * definitions.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * 
 * \par Copyright:
 * (C) 2012-2015.  RoadNarrows
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
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
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

/**
 * SECTION:element-pcsfilesrc
 *
 * OpenNI source element.
 *
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch -v pcsfilesrc location=output.pcs ! pcsviewersink
 * ]| Streams a previously saved Point Cloud Stream file to the PCS viewer. 
 * </refsect2>
 */

#include "Eudoxus/euConf.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <gst/gst.h>
#include <gst/base/gstpushsrc.h>

#include "rnr/rnrconfig.h"

#include "Eudoxus/Eudoxus.h"
#include "Eudoxus/euUtils.h"
#include "Eudoxus/euPcs.h"

#include "gsteudoxus.h"

#include "gstpcsfilesrc.h"

GstDebugCategory *GST_CAT_DEFAULT;

using namespace eu;

/*!
 * \brief Element Filter signals and args
 */
enum
{
  /* FILL ME */
  LAST_SIGNAL       ///< last signal
};

/*!
 * \brief Element Properties
 */
enum
{
  PROP_0,           ///< start
  PROP_LOCATION,    ///< PCS file location
  PROP_LOOP,        ///< enable/disable looping
  PROP_LAST         ///< last
};

/*!
 * \brief Element source pad capabilities.
 */
static GstStaticPadTemplate pcs_file_src_src_pad_template =
  GST_STATIC_PAD_TEMPLATE("src",
  GST_PAD_SRC,
  GST_PAD_ALWAYS,
  GST_STATIC_CAPS(
    "oni/pcs-ascii, "
      "width = (int) { 320, 640, 1280 }, "
      "height = (int) { 240, 480, 1024 }, "
      "framerate = (fraction) [ 1, 60 ], "
      "mirror = (boolean) false; "
    "oni/pcs-binary, "
      "width = (int) { 320, 640, 1280 }, "
      "height = (int) { 240, 480, 1024 }, "
      "framerate = (fraction) [ 1, 60 ], "
      "mirror = (boolean) false; "
    "oni/pcs-oni, "
      "width = (int) { 320, 640, 1280 }, "
      "height = (int) { 240, 480, 1024 }, "
      "framerate = (fraction) [ 1, 60 ], "
      "mirror = (boolean) false; "
    "oni/oni, "
      "width = (int) { 320, 640, 1280 }, "
      "height = (int) { 240, 480, 1024 }, "
      "framerate = (fraction) [ 1, 60 ], "
      "mirror = (boolean) false"
  )
);

#ifdef EU_HAS_GST_1_0
G_DEFINE_TYPE(GstPcsFileSrc, gst_pcs_file_src, GST_TYPE_ELEMENT);
#else // 0.10
GST_BOILERPLATE(GstPcsFileSrc,
                gst_pcs_file_src,
                GstPushSrc,
                GST_TYPE_PUSH_SRC);
#endif // EU_HAS_GST_1_0

//
// Prototypes
//
#ifdef EU_HAS_GST_1_0
static void gst_pcs_file_src_init(GstPcsFileSrc *pcs_file_src);
#else // 0.10
static void gst_pcs_file_src_init(GstPcsFileSrc      *pcs_file_src,
                                  GstPcsFileSrcClass *gclass)
#endif // EU_HAS_GST_1_0

static void gst_pcs_file_src_set_property(GObject      *object,
                                          guint         prop_id,
                                          const GValue *value,
                                          GParamSpec   *pspec);

static void gst_pcs_file_src_get_property(GObject   *object,
                                          guint       prop_id,
                                          GValue     *value,
                                          GParamSpec *pspec);

static gboolean gst_pcs_file_src_parse_caps(const GstCaps  *caps,
                                            EuMimeType     *pMimeType,
                                            EuResolution   *pResolution,
                                            gint           *pFpsN,
                                            gint           *pFpsD,
                                            gboolean       *pMirror);

static gboolean gst_pcs_file_src_set_caps(GstBaseSrc *base_src, GstCaps *caps);

static gboolean gst_pcs_file_src_start(GstBaseSrc *base_src);

static gboolean gst_pcs_file_src_stop(GstBaseSrc *base_src);

static GstFlowReturn gst_pcs_file_src_create(GstPushSrc  *push_src,
                                             GstBuffer  **buffer);

static void gst_pcs_file_src_get_times(GstBaseSrc   *base_src,
                                       GstBuffer    *buffer,
                                       GstClockTime *start,
                                       GstClockTime *end);

static void gst_pcs_file_src_test_plugin_grab(GstPcsFileSrc *pcs_file_src,
                                              guchar        *dest);


/* GObject vmethod implementations */
static void gst_pcs_file_src_base_init(gpointer gclass)
{
  GstElementClass *gstelement_class = GST_ELEMENT_CLASS(gclass);

  gst_element_class_add_pad_template(gstelement_class,
                  gst_static_pad_template_get(&pcs_file_src_src_pad_template));

  gst_element_class_set_details_simple(gstelement_class,
    "OpenNI source",
    "Source/PCSFile",
    "Generate PCS stream from file.",
    "RoadNarrows <oneway@roadnarrows.com>");
}

/* initialize the pcsfilesrc's class */
static void gst_pcs_file_src_class_init(GstPcsFileSrcClass *klass)
{
  GObjectClass    *gobject_class;
  GstElementClass *gstelement_class;
  GstBaseSrcClass *gstbasesrc_class;
  GstPushSrcClass *gstpushsrc_class;

  gobject_class     = (GObjectClass *)klass;
  gstelement_class  = (GstElementClass *)klass;
  gstbasesrc_class  = (GstBaseSrcClass *)klass;
  gstpushsrc_class  = (GstPushSrcClass *)klass;

  gobject_class->set_property = gst_pcs_file_src_set_property;
  gobject_class->get_property = gst_pcs_file_src_get_property;

  g_object_class_install_property(gobject_class, PROP_LOCATION,
      g_param_spec_string("location", "file location",
          "Loacation of PCS file to read",
          NULL, (GParamFlags)G_PARAM_READWRITE));

  g_object_class_install_property(gobject_class, PROP_LOOP,
      g_param_spec_boolean("loop", "loop through file",
          "Continuously loop through PCS file",
          FALSE, (GParamFlags)G_PARAM_READWRITE));

  gstbasesrc_class->set_caps  = gst_pcs_file_src_set_caps;
  gstbasesrc_class->get_times = gst_pcs_file_src_get_times;
  gstbasesrc_class->start     = gst_pcs_file_src_start;
  gstbasesrc_class->stop      = gst_pcs_file_src_stop;
  gstpushsrc_class->create    = gst_pcs_file_src_create;
}

/* initialize the new element
 * instantiate pads and add them to element
 * set pad calback functions
 * initialize instance structure
 */
#ifdef EU_HAS_GST_1_0
static void gst_pcs_file_src_init(GstPcsFileSrc *pcs_file_src)
#else // 0.10
static void gst_pcs_file_src_init(GstPcsFileSrc      *pcs_file_src,
                                  GstPcsFileSrcClass *gclass)
#endif // EU_HAS_GST_1_0
{
  // properties
  pcs_file_src->m_sLocation     = NULL;
  pcs_file_src->m_bLoop         = FALSE;

  // capabilities
  pcs_file_src->m_nFrameRateN   = 30;
  pcs_file_src->m_nFrameRateD   = 1;

  // state
  pcs_file_src->m_fp            = NULL;
  pcs_file_src->m_pPcsFrame     = NULL;
  pcs_file_src->m_clockRunning  = 0;

  //gst_element_add_pad(GST_ELEMENT (pcs_file_src), pcs_file_src->srcpad);

  // we operate in time
  gst_base_src_set_format(GST_BASE_SRC(pcs_file_src), GST_FORMAT_TIME);
  gst_base_src_set_live(GST_BASE_SRC(pcs_file_src), TRUE);
  //pcs_file_src->peer_alloc = DEFAULT_PEER_ALLOC;
}

static void gst_pcs_file_src_set_property(GObject      *object,
                                          guint         prop_id,
                                          const GValue *value,
                                          GParamSpec   *pspec)
{
  GstPcsFileSrc  *pcs_file_src = GST_PCS_FILE_SRC(object);
  const gchar    *sLocation;

  switch( prop_id )
  {
    case PROP_LOCATION:
      sLocation = g_value_get_string(value);
      if( pcs_file_src->m_sLocation != NULL )
      {
        delete pcs_file_src->m_sLocation;
        pcs_file_src->m_sLocation = NULL;
      }
      if( sLocation != NULL )
      {
        pcs_file_src->m_sLocation = new char[strlen(sLocation)+1];
        strcpy(pcs_file_src->m_sLocation, sLocation);
        GST_INFO("Location=%s.", sLocation);
      }
      break;

    case PROP_LOOP:
      pcs_file_src->m_bLoop = g_value_get_boolean(value);
      GST_INFO("Loop=%s.", (pcs_file_src->m_bLoop? "true": "false"));
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
      return;
  }

}

static void gst_pcs_file_src_get_property(GObject    *object,
                                          guint       prop_id,
                                          GValue     *value,
                                          GParamSpec *pspec)
{
  GstPcsFileSrc *pcs_file_src = GST_PCS_FILE_SRC(object);

  switch( prop_id )
  {
    case PROP_LOCATION:
      g_value_set_string(value, pcs_file_src->m_sLocation);
      break;
    case PROP_LOOP:
      g_value_set_boolean(value, pcs_file_src->m_bLoop);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
      break;
  }
}

static gboolean gst_pcs_file_src_parse_caps(const GstCaps  *caps,
                                            gint           *pFpsN,
                                            gint           *pFpsD)
{
  const GstStructure *structure;
  const gchar        *sMimeType;
  gboolean            rc;

  GST_DEBUG("parsing caps");

  structure = gst_caps_get_structure(caps, 0);

  sMimeType = gst_structure_get_name(structure);

  rc = gst_structure_get_fraction(structure, "framerate", pFpsD, pFpsN);

  return rc;
}

/* this function handles the link with other elements */
static gboolean gst_pcs_file_src_set_caps(GstBaseSrc *base_src, GstCaps *caps)
{
  GstPcsFileSrc      *pcs_file_src;
  EuMimeType      eMimeType;
  EuResolution    resolution;
  gint            nFpsN;
  gint            nFpsD;
  gboolean        bMirror;
  gboolean        rc;

  pcs_file_src = GST_PCS_FILE_SRC(base_src);

  rc = gst_pcs_file_src_parse_caps(caps, &nFpsN, &nFpsD);

  if( rc == TRUE )
  {
    pcs_file_src->m_nFrameRateN  = nFpsN;
    pcs_file_src->m_nFrameRateD  = nFpsD;

    GST_INFO("framerate=%d/%d",
        pcs_file_src->m_nFrameRateN, 
        pcs_file_src->m_nFrameRateD);
  }

  return rc;
}

static gboolean gst_pcs_file_src_start(GstBaseSrc *base_src)
{
  GstPcsFileSrc *pcs_file_src = GST_PCS_FILE_SRC(base_src);

  if( (pcs_file_src->m_sLocation == NULL) || (*pcs_file_src->m_sLocation == 0) )
  {
    GST_ERROR("No PCS file location specified.");
    return FALSE;
  }

  if( pcs_file_src->m_fp != NULL )
  {
    fclose(pcs_file_src->m_fp);
    pcs_file_src->m_fp = NULL;
  }

  if( (pcs_file_src->m_fp = fopen(pcs_file_src->m_sLocation, "r")) == NULL )
  {
    GST_ERROR("%s: %s(errno=%d).",
                          pcs_file_src->m_sLocation, strerror(errno), errno);
    return FALSE;
  }

  // old pcs frame instance
  if( pcs_file_src->m_pPcsFrame != NULL )
  {
    delete pcs_file_src->m_pPcsFrame;
    pcs_file_src->m_pPcsFrame = NULL;
  }

  pcs_file_src->m_pPcsFrame = new EuPcsFrame;

  pcs_file_src->m_clockRunning = 0;
}

static gboolean gst_pcs_file_src_stop(GstBaseSrc *base_src)
{
  GstPcsFileSrc *pcs_file_src = GST_PCS_FILE_SRC(base_src);

  if( pcs_file_src->m_fp != NULL )
  {
    fclose(pcs_file_src->m_fp);
    pcs_file_src->m_fp = NULL;
  }

  return TRUE;
}

static GstFlowReturn gst_pcs_file_src_create(GstPushSrc *push_src,
                                             GstBuffer **buffer)
{
  GstPcsFileSrc  *pcs_file_src;
  guchar         *pData;
  gulong          uSize;
  ssize_t         nBytes;
  guint           sizeOut;
  gulong          uFrameId;
  GstBuffer      *bufOut;
  GstClockTime    clockNext;

#ifdef EU_HAS_GST_1_0
  GstMapInfo      info;
#endif // EU_HAS_GST_1_0

  pcs_file_src = GST_PCS_FILE_SRC(push_src);

  // extract pcs header
  nBytes = pcs_file_src->m_pPcsFrame->extractPcsHeader(pcs_file_src->m_fp);

  //
  // If at the end of file and looping is enabled, rewind and try extracting
  // the header again.
  //
  if( (nBytes <= 0) && feof(pcs_file_src->m_fp) && pcs_file_src->m_bLoop )
  {
    clearerr(pcs_file_src->m_fp);
    rewind(pcs_file_src->m_fp);
    nBytes = pcs_file_src->m_pPcsFrame->extractPcsHeader(pcs_file_src->m_fp);
  }

  // error extracting header
  if( nBytes < 0 )
  {
    GST_ERROR("%s.", pcs_file_src->m_pPcsFrame->getErrorMsg());
    return GST_FLOW_ERROR;
  }

  // eof
  else if( nBytes == 0 )
  {
    // TODO need a better way to end 
    return GST_FLOW_ERROR;
  }

  // get maximum size of frame
  sizeOut = (guint)pcs_file_src->m_pPcsFrame->getMaxTotalSize();

  g_return_val_if_fail(sizeOut>0, GST_FLOW_ERROR);

  // allocate new gst buffer
  bufOut = gst_buffer_new_and_alloc(sizeOut);

  GST_LOG("Creating frame buffer of %u bytes", sizeOut);

  // set caps on buffer
#ifndef EU_HAS_GST_1_0
  gst_buffer_set_caps(bufOut, GST_PAD_CAPS(GST_BASE_SRC_PAD(push_src)));
#endif // !EU_HAS_GST_1_0

#ifdef EU_HAS_GST_1_0
  gst_buffer_map(bufOut, &info, GST_MAP_READ);
  pData = info.data;
  uSize = info.size;
  gst_buffer_unmap(bufOut, &info);
#else // 0.10
  pData = GST_BUFFER_DATA(bufOut),
  uSize = GST_BUFFER_SIZE(bufOut),
#endif // EU_HAS_GST_1_0

  // extract frame to buffer
  nBytes = pcs_file_src->m_pPcsFrame->extractFrameToBuf(
                                      pcs_file_src->m_fp, 
                                      (byte_t *)pData,
                                      (size_t)uSize,
                                      false);

  // error
  if( nBytes < 0 )
  {
    GST_ERROR("%s", pcs_file_src->m_pPcsFrame->getErrorMsg());
    return GST_FLOW_ERROR;
  }

  // resize to exact frame size
#ifdef EU_HAS_GST_1_0
  gst_buffer_map(bufOut, &info, GST_MAP_WRITE);
  info.size = (guint)nBytes;
  gst_buffer_unmap(bufOut, &info);
#else // 0.10
  GST_BUFFER_SIZE(bufOut) = (guint)nBytes;
#endif // EU_HAS_GST_1_0
  
  // frame id 
  uFrameId = (guint)pcs_file_src->m_pPcsFrame->getFrameId();

  // timestamp
  GST_BUFFER_TIMESTAMP(bufOut) = pcs_file_src->m_clockRunning;

  // set this stream offset (frame id)
  GST_BUFFER_OFFSET(bufOut) = uFrameId;

  uFrameId++;

  // set next stream offset (frame id)
  GST_BUFFER_OFFSET_END(bufOut) = uFrameId;

  // set next time to produce frame given frame rate
  clockNext = gst_util_uint64_scale_int(uFrameId * GST_SECOND,
                                        pcs_file_src->m_nFrameRateN,
                                        pcs_file_src->m_nFrameRateD);

  GST_BUFFER_DURATION(bufOut) = clockNext - pcs_file_src->m_clockRunning;

  pcs_file_src->m_clockRunning = clockNext;

  *buffer = bufOut;

#ifdef EU_HAS_GST_1_0
  gst_buffer_map(bufOut, &info, GST_MAP_READ);
  uSize = info.size;
  gst_buffer_unmap(bufOut, &info);
#else // 0.10
  uSize = GST_BUFFER_SIZE(bufOut);
#endif // EU_HAS_GST_1_0
  
  GST_LOG("Sent %s frame buffer of %lu bytes for frame %lu",
      getMimeTypeStr(pcs_file_src->m_pPcsFrame->getMimeType()),
      uSize,
      uFrameId);

  return GST_FLOW_OK;
}

static void gst_pcs_file_src_get_times(GstBaseSrc   *base_src,
                                       GstBuffer    *buffer,
                                       GstClockTime *start,
                                       GstClockTime *end)
{
  // for live sources, sync on the timestamp of the buffer
  if( gst_base_src_is_live(base_src) )
  {
    GstClockTime timestamp = GST_BUFFER_TIMESTAMP(buffer);

    if( GST_CLOCK_TIME_IS_VALID(timestamp) )
    {
      // get duration to calculate end time 
      GstClockTime duration = GST_BUFFER_DURATION(buffer);

      if( GST_CLOCK_TIME_IS_VALID(duration) )
      {
        *end = timestamp + duration;
      }
      *start = timestamp;
    }
  }
  else
  {
    *start = -1;
    *end   = -1;
  }
}


/*!
 * \brief Entry point to initialize the plug-in
 *
 * Initialize the plug-in itself.
 * Register the element factories and other features.
 */
static gboolean plugin_init(GstPlugin *plugin)
{
  /*
   * Debug category for fltering log messages.
   */
  GST_DEBUG_CATEGORY_INIT(pcs_file_src_debug, "pcsfilesrc", 0,
                                        "PCS File Source");

  return gst_element_register(plugin,
                              "pcsfilesrc",
                              GST_RANK_NONE,
                              GST_TYPE_PCS_FILE_SRC);
}

/*!
 * GStreamer looks for this structure to register this plug-in.
 */
GST_PLUGIN_DEFINE (
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    pcsfilesrc,
    "PCS file source element",
    plugin_init,
    VERSION,
    EU_GST_PACKAGE_LICENSE,
    EU_GST_PACKAGE_NAME,
    EU_GST_PACKAGE_URL
)
