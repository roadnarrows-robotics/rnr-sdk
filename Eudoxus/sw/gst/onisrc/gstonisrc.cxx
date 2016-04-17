////////////////////////////////////////////////////////////////////////////////
//
// Package:     Eudoxus
//
// SubPackage:  GStreamer
//
// Plug-In:     libgstonisrc
//
// File:        gstonisrc.cxx
//
/*! \file
 *
 * $LastChangedDate: 2016-01-19 17:29:01 -0700 (Tue, 19 Jan 2016) $
 * $Rev: 4267 $
 *
 * \brief Eudoxus GStreamer OpenNI source element plug-in definitions.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * 
 * \par Copyright:
 * (C) 2012-2016.  RoadNarrows
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
 * @EulaEnd@
 */
//
////////////////////////////////////////////////////////////////////////////////

/*
 * GStreamer
 * Copyright (C) 2005 Thomas Vander Stichele <thomas@apestaart.org>
 * Copyright (C) 2005 Ronald S. Bultje <rbultje@ronald.bitfreak.net>
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Alternatively, the contents of this file may be used under the
 * GNU Lesser General Public License Version 2.1 (the "LGPL"), in
 * which case the following provisions apply instead of the ones
 * mentioned above:
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * SECTION:element-onisrc
 *
 * OpenNI source element.
 *
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch -v onisrc ! filesink locataion=out.oni
 * ]| This generates an oni file out.oni containing data from production nodes
 * specified in openni-config.xml.
 * |[
 * Target: gst-launch onisrc nodes=depth,image ! udpsink host=myhost port=4000
 * Host:   gst-launch udpsrc port=4000 ! filesink location=out.oni
 * ]| This generates an oni file out.oni on the host myhost, which receives
 * UDP data form the target. The data contain both depth maps with color images.
 * </refsect2>
 */

#include "Eudoxus/euConf.h"

#include <string.h>

#include <gst/gst.h>
#include <gst/base/gstpushsrc.h>

#include "Eudoxus/Eudoxus.h"
#include "Eudoxus/euUtils.h"
#include "Eudoxus/euPcs.h"

#include "gsteudoxus.h"

#include "gstonisrc.h"

GstDebugCategory *GST_CAT_DEFAULT;

using namespace eu;

/*!
 * \brief Handy macro.
 */
#define NODE_ENDIS_STR(bits, mask) (((bits)&(mask))? "enabled": "disabled")

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
  PROP_NODE_DEPTH,  ///< enable/disable depth production node
  PROP_NODE_IMAGE,  ///< enable/disable image production node
  PROP_NODE_AUDIO,  ///< enable/disable audio production node
  PROP_NODE_IMU,    ///< enable/disable imu production node
  PROP_LAST         ///< last
};

/*!
 * \brief Element source pad capabilities.
 */
static GstStaticPadTemplate oni_src_src_pad_template =
  GST_STATIC_PAD_TEMPLATE("src",
  GST_PAD_SRC,
  GST_PAD_ALWAYS,
  GST_STATIC_CAPS(
    "oni/pcs-ascii, "
      "width = (int) { 320, 640, 1280 }, "
      "height = (int) { 240, 480, 1024 }, "
      "framerate = (fraction) [ 1, 60 ], "
      "mirror = (boolean) true; "
    "oni/pcs-binary, "
      "width = (int) { 320, 640, 1280 }, "
      "height = (int) { 240, 480, 1024 }, "
      "framerate = (fraction) [ 1, 60 ], "
      "mirror = (boolean) true; "
    "oni/pcs-oni, "
      "width = (int) { 320, 640, 1280 }, "
      "height = (int) { 240, 480, 1024 }, "
      "framerate = (fraction) [ 1, 60 ], "
      "mirror = (boolean) true; "
    "oni/oni, "
      "width = (int) { 320, 640, 1280 }, "
      "height = (int) { 240, 480, 1024 }, "
      "framerate = (fraction) [ 1, 60 ], "
      "mirror = (boolean) true"
  )
);


#ifdef EU_HAS_GST_1_0
G_DEFINE_TYPE(GstOniSrc, gst_oni_src, GST_TYPE_PUSH_SRC);
#else
GST_BOILERPLATE(GstOniSrc, gst_oni_src, GstPushSrc, GST_TYPE_PUSH_SRC);
#endif // EU_HAS_GST_1_0

//
// Prototypes
//
static void gst_oni_src_set_property(GObject      *object,
                                     guint         prop_id,
                                     const GValue *value,
                                     GParamSpec   *pspec);

static void gst_oni_src_get_property(GObject   *object,
                                    guint       prop_id,
                                    GValue     *value,
                                    GParamSpec *pspec);

static gboolean gst_oni_src_parse_caps(const GstCaps  *caps,
                                       EuMimeType     *pMimeType,
                                       EuResolution   *pResolution,
                                       gint           *pFpsN,
                                       gint           *pFpsD,
                                       gboolean       *pMirror);

static gboolean gst_oni_src_set_caps(GstBaseSrc *base_src, GstCaps *caps);

static gboolean gst_oni_src_start(GstBaseSrc *base_src);

static gboolean gst_oni_src_stop(GstBaseSrc *base_src);

static GstFlowReturn gst_oni_src_create(GstPushSrc  *push_src,
                                        GstBuffer  **buffer);

static void gst_oni_src_get_times(GstBaseSrc   *base_src,
                                  GstBuffer    *buffer,
                                  GstClockTime *start,
                                  GstClockTime *end);

static void gst_oni_src_test_plugin_grab(GstOniSrc *oni_src, guchar *dest);


/* GObject vmethod implementations */
static void gst_oni_src_base_init(gpointer gclass)
{
  GstElementClass *gstelement_class = GST_ELEMENT_CLASS(gclass);

  gst_element_class_add_pad_template(gstelement_class,
                    gst_static_pad_template_get(&oni_src_src_pad_template));

  gst_element_class_set_details_simple(gstelement_class,
    "OpenNI source",
    "Oni/Oni",
    "Generate OpenNI data in an oni file stream format.",
    "RoadNarrows <oneway@roadnarrows.com>");
}

/* initialize the onisrc's class */
static void gst_oni_src_class_init(GstOniSrcClass *klass)
{
  GObjectClass    *gobject_class;
  GstElementClass *gstelement_class;
  GstBaseSrcClass *gstbasesrc_class;
  GstPushSrcClass *gstpushsrc_class;

  gobject_class     = (GObjectClass *)klass;
  gstelement_class  = (GstElementClass *)klass;
  gstbasesrc_class  = (GstBaseSrcClass *)klass;
  gstpushsrc_class  = (GstPushSrcClass *)klass;

  gobject_class->set_property = gst_oni_src_set_property;
  gobject_class->get_property = gst_oni_src_get_property;

  g_object_class_install_property(gobject_class, PROP_NODE_DEPTH,
      g_param_spec_boolean("depth-node", "Depth node",
          "Enable/disable OpenNI depth production node",
          FALSE, (GParamFlags)G_PARAM_READWRITE));

  g_object_class_install_property(gobject_class, PROP_NODE_IMAGE,
      g_param_spec_boolean("image-node", "Image node",
          "Enable/disable OpenNI image production node",
          FALSE, (GParamFlags)G_PARAM_READWRITE));

  g_object_class_install_property(gobject_class, PROP_NODE_AUDIO,
      g_param_spec_boolean("audio-node", "Audio node",
          "Enable/disable OpenNI audio production node",
          FALSE, (GParamFlags)G_PARAM_READWRITE));

  g_object_class_install_property(gobject_class, PROP_NODE_IMU,
      g_param_spec_boolean("imu-node", "IMU node",
          "Enable/disable Inertial Measurement Unit production node",
          FALSE, (GParamFlags)G_PARAM_READWRITE));

  gstbasesrc_class->set_caps  = gst_oni_src_set_caps;
  gstbasesrc_class->get_times = gst_oni_src_get_times;
  gstbasesrc_class->start     = gst_oni_src_start;
  gstbasesrc_class->stop      = gst_oni_src_stop;
  gstpushsrc_class->create    = gst_oni_src_create;
}

/* initialize the new element
 * instantiate pads and add them to element
 * set pad calback functions
 * initialize instance structure
 */
#ifdef EU_HAS_GST_1_0
static void gst_oni_src_init(GstOniSrc *oni_src)
#else // 0.10
static void gst_oni_src_init(GstOniSrc *oni_src, GstOniSrcClass *gclass)
#endif // EU_HAS_GST_1_0
{
  // properties
  oni_src->m_uProdNodes   = (guint)EuProdNodeTypeNone;

  // capabilities
  oni_src->m_eMimeType            = EuMimeTypePcsBinary;
  oni_src->m_resolution.m_uWidth  = 320;
  oni_src->m_resolution.m_uHeight = 240;
  oni_src->m_bMirror              = TRUE;
  oni_src->m_nFrameRateN          = 10;
  oni_src->m_nFrameRateD          = 1;

  // state
  oni_src->m_pFramePcs    = NULL;
  oni_src->m_clockRunning = 0;

  gst_oni_init(oni_src);

  //gst_element_add_pad(GST_ELEMENT (oni_src), oni_src->srcpad);

  // we operate in time
  gst_base_src_set_format(GST_BASE_SRC(oni_src), GST_FORMAT_TIME);
  gst_base_src_set_live(GST_BASE_SRC(oni_src), TRUE);
  //oni_src->peer_alloc = DEFAULT_PEER_ALLOC;
}

static void gst_oni_src_set_property(GObject      *object,
                                     guint         prop_id,
                                     const GValue *value,
                                     GParamSpec   *pspec)
{
  GstOniSrc  *oni_src = GST_ONI_SRC(object);
  gboolean  bVal;
  guint     uMask;

  switch( prop_id )
  {
    case PROP_NODE_DEPTH:
      uMask = (guint)EuProdNodeTypeDepth;
      if( (bVal = g_value_get_boolean(value)) == TRUE )
      {
        oni_src->m_uProdNodes |= uMask;
      }
      else
      {
        oni_src->m_uProdNodes &= ~uMask;
      } 
      GST_INFO("Depth node %s.", NODE_ENDIS_STR(oni_src->m_uProdNodes, uMask));
      break;

    case PROP_NODE_IMAGE:
      uMask = (guint)EuProdNodeTypeImage;
      if( (bVal = g_value_get_boolean(value)) == TRUE )
      {
        oni_src->m_uProdNodes |= uMask;
      }
      else
      {
        oni_src->m_uProdNodes &= ~uMask;
      } 
      GST_INFO("Image node %s.", NODE_ENDIS_STR(oni_src->m_uProdNodes, uMask));
      break;

    case PROP_NODE_AUDIO:
      uMask = (guint)EuProdNodeTypeAudio;
      if( (bVal = g_value_get_boolean(value)) == TRUE )
      {
        oni_src->m_uProdNodes |= uMask;
      }
      else
      {
        oni_src->m_uProdNodes &= ~uMask;
      } 
      GST_INFO("Audio node %s.", NODE_ENDIS_STR(oni_src->m_uProdNodes, uMask));
      break;

    case PROP_NODE_IMU:
      uMask = (guint)EuProdNodeTypeImu;
      if( (bVal = g_value_get_boolean(value)) == TRUE )
      {
        oni_src->m_uProdNodes |= uMask;
      }
      else
      {
        oni_src->m_uProdNodes &= ~uMask;
      } 
      GST_INFO("IMU node %s.", NODE_ENDIS_STR(oni_src->m_uProdNodes, uMask));
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
      return;
  }

}

static void gst_oni_src_get_property(GObject    *object,
                                     guint       prop_id,
                                     GValue     *value,
                                     GParamSpec *pspec)
{
  GstOniSrc *oni_src = GST_ONI_SRC(object);
  gboolean  bVal;

  switch( prop_id )
  {
    case PROP_NODE_DEPTH:
      bVal = (oni_src->m_uProdNodes & (guint)EuProdNodeTypeDepth)? TRUE: FALSE;
      g_value_set_boolean(value, bVal);
      break;
    case PROP_NODE_IMAGE:
      bVal = (oni_src->m_uProdNodes & (guint)EuProdNodeTypeImage)? TRUE: FALSE;
      g_value_set_boolean(value, bVal);
      break;
    case PROP_NODE_AUDIO:
      bVal = (oni_src->m_uProdNodes & (guint)EuProdNodeTypeAudio)? TRUE: FALSE;
      g_value_set_boolean(value, bVal);
      break;
    case PROP_NODE_IMU:
      bVal = (oni_src->m_uProdNodes & (guint)EuProdNodeTypeImu)? TRUE: FALSE;
      g_value_set_boolean(value, bVal);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
      break;
  }
}

static gboolean gst_oni_src_parse_caps(const GstCaps  *caps,
                                       EuMimeType     *pMimeType,
                                       EuResolution   *pResolution,
                                       gint           *pFpsN,
                                       gint           *pFpsD,
                                       gboolean       *pMirror)
{
  const GstStructure *structure;
  const gchar        *sMimeType;
  int                 nWidth;
  int                 nHeight;
  gboolean            bGoodRes;
  gboolean            rc;

  GST_DEBUG("parsing caps");

  structure = gst_caps_get_structure(caps, 0);

  sMimeType = gst_structure_get_name(structure);

  *pMimeType = mapMimeTypeToEnum(sMimeType);

  switch( *pMimeType )
  {
    case EuMimeTypeNone:
      GST_ERROR("'%s': Unknown MimeType", sMimeType);
      return FALSE;
    case EuMimeTypeOni:
      GST_ERROR("'%s': MimeType not supported yet", sMimeType);
      return FALSE;
  }

  rc  = gst_structure_get_int(structure, "width", &nWidth);
  rc &= gst_structure_get_int(structure, "height", &nHeight);
  rc &= gst_structure_get_fraction(structure, "framerate", pFpsD, pFpsN);
  rc &= gst_structure_get_boolean(structure, "mirror", pMirror);

  bGoodRes = TRUE;

  switch( nWidth )
  {
    case 320:
      if( nHeight != 240 )
      {
        rc = bGoodRes = FALSE;
      }
      break;
    case 640:
      if(nHeight != 480 )
      {
        rc = bGoodRes = FALSE;
      }
      break;
    case 1280:
      if( nHeight != 1024 )
      {
        rc = bGoodRes = FALSE;
      }
      break;
    default:
      rc = bGoodRes = FALSE;
      break;
  }

  if( bGoodRes )
  {
    pResolution->m_uWidth  = (int)nWidth;
    pResolution->m_uHeight = (int)nHeight;
  }
  else
  {
    GST_ERROR("Unsupported resolution %dx%d.", nWidth, nHeight);
  }

  return rc;
}

/* GstElement vmethod implementations */

/* this function handles the link with other elements */
static gboolean gst_oni_src_set_caps(GstBaseSrc *base_src, GstCaps *caps)
{
  GstOniSrc      *oni_src;
  EuMimeType      eMimeType;
  EuResolution    resolution;
  gint            nFpsN;
  gint            nFpsD;
  gboolean        bMirror;
  gboolean        rc;

  oni_src = GST_ONI_SRC(base_src);

  rc = gst_oni_src_parse_caps(caps, &eMimeType, &resolution,
                                    &nFpsN, &nFpsD, &bMirror);
  if( rc == TRUE )
  {
    oni_src->m_eMimeType    = eMimeType;
    oni_src->m_resolution   = resolution;
    oni_src->m_nFrameRateN  = nFpsN;
    oni_src->m_nFrameRateD  = nFpsD;
    oni_src->m_bMirror      = bMirror;

    rc = gst_oni_set_caps(oni_src);

    GST_INFO("%s,width=%d,height=%d,framerate=%d/%d,mirror=%s",
        getMimeTypeStr(oni_src->m_eMimeType),
        oni_src->m_resolution.m_uWidth, 
        oni_src->m_resolution.m_uHeight, 
        oni_src->m_nFrameRateN, 
        oni_src->m_nFrameRateD, 
        (oni_src->m_bMirror? "true": "false"));
  }

  return rc;
}

static gboolean gst_oni_src_start(GstBaseSrc *base_src)
{
  GstOniSrc *oni_src = GST_ONI_SRC(base_src);

  switch( oni_src->m_eMimeType )
  {
    case EuMimeTypeOni:
    case EuMimeTypePcsOni:
      if( oni_src->m_uProdNodes == (guint)EuProdNodeTypeNone )
      {
        GST_ERROR("No OpenNI production nodes enabled.");
        return FALSE;
      }
      break;
    case EuMimeTypePcsAscii:
    case EuMimeTypePcsBinary:
    case EuMimeTypePcsBinaryLzf:
      if( !(oni_src->m_uProdNodes & (guint)EuProdNodeTypeDepth) )
      {
        GST_ERROR("Depth production node not enabled but required.");
        return FALSE;
      }
      break;
    default:
      GST_ERROR("Unknown format %d.", oni_src->m_eMimeType);
      return FALSE;
  }

  // old oni instance
  if( oni_src->m_pFramePcs != NULL )
  {
    delete oni_src->m_pFramePcs;
    oni_src->m_pFramePcs = NULL;
  }

  oni_src->m_pFramePcs = new EuPcsFrame;

  oni_src->m_clockRunning = 0;

  return gst_oni_start(oni_src);
}

static gboolean gst_oni_src_stop(GstBaseSrc *base_src)
{
  GstOniSrc *oni_src = GST_ONI_SRC(base_src);

  return gst_oni_stop(oni_src);
}

static GstFlowReturn gst_oni_src_create(GstPushSrc *push_src,
                                        GstBuffer **buffer)
{
  GstOniSrc      *oni_src;
  guint           sizeOut;
  gulong          uFrameId;
  GstBuffer      *bufOut;
  GstClockTime    clockNext;
  GstFlowReturn   rc;

  oni_src = GST_ONI_SRC(push_src);

  sizeOut = gst_oni_get_size(oni_src);

  g_return_val_if_fail(sizeOut>0, GST_FLOW_ERROR);

  uFrameId = (guint)oni_src->m_pFramePcs->getFrameId();

  GST_LOG("Creating buffer of %u bytes with %dx%d frame for frame %lu",
      sizeOut, oni_src->m_resolution.m_uWidth, oni_src->m_resolution.m_uHeight,
      uFrameId);

  bufOut = gst_buffer_new_and_alloc(sizeOut);

#ifndef EU_HAS_GST_1_0
  gst_buffer_set_caps(bufOut, GST_PAD_CAPS(GST_BASE_SRC_PAD(push_src)));
#endif // EU_HAS_GST_1_0

  rc = gst_oni_grab_frame(oni_src, bufOut);

  GST_BUFFER_TIMESTAMP(bufOut) = oni_src->m_clockRunning;

  GST_BUFFER_OFFSET(bufOut) = uFrameId;

  uFrameId++;

  GST_BUFFER_OFFSET_END(bufOut) = uFrameId;

  clockNext = gst_util_uint64_scale_int(uFrameId * GST_SECOND,
                                        oni_src->m_nFrameRateN,
                                        oni_src->m_nFrameRateD);

  GST_BUFFER_DURATION(bufOut) = clockNext - oni_src->m_clockRunning;

  oni_src->m_clockRunning = clockNext;

  *buffer = bufOut;

  if( rc == GST_FLOW_OK )
  {
#ifdef EU_HAS_GST_1_0
    GstMapInfo      info;
    gst_buffer_map(bufOut, &info, GST_MAP_READ);
    GST_INFO("Sent %lu byte size frame.", info.size);
    gst_buffer_unmap(bufOut, &info);
#else // 0.10
    GST_INFO("Sent %u byte size frame.", GST_BUFFER_SIZE(bufOut));
#endif // EU_HAS_GST_1_0
  }

  return rc;
}

static void gst_oni_src_get_times(GstBaseSrc   *base_src,
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
  GST_DEBUG_CATEGORY_INIT(oni_src_debug, "onisrc", 0, "OpenNI Source");

  return gst_element_register(plugin,
                              "onisrc",
                              GST_RANK_NONE,
                              GST_TYPE_ONI_SRC);
}

/*!
 * GStreamer looks for this structure to register this plug-in.
 */
GST_PLUGIN_DEFINE (
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    onisrc,
    "OpenNI source element",
    plugin_init,
    VERSION,
    EU_GST_PACKAGE_LICENSE,
    EU_GST_PACKAGE_NAME,
    EU_GST_PACKAGE_URL
)
