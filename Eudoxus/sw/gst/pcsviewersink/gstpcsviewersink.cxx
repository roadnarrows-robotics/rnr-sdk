////////////////////////////////////////////////////////////////////////////////
//
// Package:     Eudoxus
//
// SubPackage:  GStreamer
//
// Plug-In:     libgstpcsviewersink
//
// File:        gstpcsviewersink.cxx
//
/*! \file
 *
 * $LastChangedDate: 2016-01-19 17:29:01 -0700 (Tue, 19 Jan 2016) $
 * $Rev: 4267 $
 *
 * \brief Eudoxus GStreamer Point Cloud Stream viewer sink element plug-in
 * declarations.
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

/* GStreamer
 * Copyright (C) <1999> Erik Walthinsen <omega@cse.ogi.edu>
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
 * OpenNI PDU encoder element.
 *
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch onisrc node=depth ! oni/pcs-ascii ! pcsviewersink
 * ]| This pipeline visualized a point cloud stream.
 * </refsect2>
 */

#include "Eudoxus/euConf.h"

#include <stdio.h>
#include <string.h>

#include <gst/gst.h>
#include <gst/base/gstbasesink.h>

#include "Eudoxus/Eudoxus.h"
#include "Eudoxus/euPcs.h"

#include "gsteudoxus.h"

#include "gstpcsviewersink.h"

GstDebugCategory *GST_CAT_DEFAULT;

using namespace eu;


/* PcsViewerSink signals and args */
enum
{
  LAST_SIGNAL
};

enum
{
  PROP_0,
  PROP_BG,
  PROP_PT_COLOR,
  PROP_PT_SIZE,
  PROP_LAST
};

static GstStaticPadTemplate gst_pcs_viewer_sink_sink_pad_template =
  GST_STATIC_PAD_TEMPLATE("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS("ANY")
  );

#ifdef EU_HAS_GST_1_0
G_DEFINE_TYPE(GstPcsViewerSink, gst_pcs_viewer_sink, GST_TYPE_ELEMENT);
#else // 0.10
GST_BOILERPLATE(GstPcsViewerSink,
                gst_pcs_viewer_sink,
                GstBaseSink,
                GST_TYPE_BASE_SINK);
#endif // EU_HAS_GST_1_0


//static void gst_pcs_viewer_sink_base_init(gpointer g_class);
//static void gst_pcs_viewer_sink_class_init(GstPcsViewerSinkClass *klass);

#ifdef EU_HAS_GST_1_0
static void gst_pcs_viewer_sink_init(GstPcsViewerSink *pcs_viewer_sink);
#else // 0.10
static void gst_pcs_viewer_sink_init(GstPcsViewerSink      *pcs_viewer_sink,
                                     GstPcsViewerSinkClass *gclass);
#endif // EU_HAS_GST_1_0

#ifndef EU_HAS_GST_1_0
static void gst_pcs_viewer_sink_finalize(GObject *object);
#endif // !EU_HAS_GST_1_0


//static GstCaps *gst_pcs_viewer_sink_get_caps(GstPad *pad);
static gboolean gst_pcs_viewer_sink_set_caps(GstPad *pad, GstCaps *caps);

static void gst_pcs_viewer_sink_set_property(GObject      *object,
                                             guint         prop_id,
                                             const GValue *value,
                                             GParamSpec   *pspec);
static void gst_pcs_viewer_sink_get_property(GObject    *object,
                                             guint       prop_id,
                                             GValue     *value,
                                             GParamSpec *pspec);

static gboolean gst_pcs_viewer_sink_start(GstBaseSink *sink);
static gboolean gst_pcs_viewer_sink_stop(GstBaseSink *sink);
static GstFlowReturn gst_pcs_viewer_sink_render(GstBaseSink *sink,
                                                GstBuffer   *buf);

static void gst_pcs_viewer_sink_base_init(gpointer g_class)
{
  GstElementClass *gstelement_class = GST_ELEMENT_CLASS(g_class);

  gst_element_class_add_pad_template(gstelement_class,
      gst_static_pad_template_get(&gst_pcs_viewer_sink_sink_pad_template));

  gst_element_class_set_details_simple(gstelement_class,
    "Point Cloud Stream viewer",
    "PCS/Viewer",
    "Point Cloud Stream viewer sink element",
    "RoadNarrows <support@roadnarrows.com>");
}

static void gst_pcs_viewer_sink_class_init(GstPcsViewerSinkClass *klass)
{
  GObjectClass     *gobject_class;
  GstElementClass  *gstelement_class;
  GstBaseSinkClass *gstbasesink_class;

  gobject_class     = (GObjectClass *)klass;
  gstelement_class  = (GstElementClass *)klass;
  gstbasesink_class = (GstBaseSinkClass *)klass;

  gobject_class->set_property = gst_pcs_viewer_sink_set_property;
  gobject_class->get_property = gst_pcs_viewer_sink_get_property;

#ifndef EU_HAS_GST_1_0
  gobject_class->finalize     = gst_pcs_viewer_sink_finalize;
#endif // !EU_HAS_GST_1_0

  g_object_class_install_property(gobject_class, PROP_BG,
      g_param_spec_int(
          "bg",
          "Background color",
          "Set the PCS viewer's background color in RGB format",
          PCS_VIEWER_SINK_RGB_MIN,
          PCS_VIEWER_SINK_RGB_MAX,
          PCS_VIEWER_SINK_BG_DFT,
          (GParamFlags)G_PARAM_READWRITE));

  g_object_class_install_property(gobject_class, PROP_PT_COLOR,
      g_param_spec_int(
          "point-color",
          "Point color",
          "Set the cloud point color in RGB format. (-1 = auto-select). "
          "If auto-select, then if the point cloud has RGB info, that color "
          "is used. Otherwise a fixed color is used.",
          -1,
          PCS_VIEWER_SINK_RGB_MAX,
          PCS_VIEWER_SINK_PT_COLOR_DFT,
          (GParamFlags)G_PARAM_READWRITE));

  g_object_class_install_property(gobject_class, PROP_PT_SIZE,
      g_param_spec_int(
          "point-size",
          "Point size",
          "Set the point pixel size",
          PCS_VIEWER_SINK_PT_SIZE_MIN,
          PCS_VIEWER_SINK_PT_SIZE_MAX,
          PCS_VIEWER_SINK_PT_SIZE_DFT,
          (GParamFlags)G_PARAM_READWRITE));

  //gstbasesink_class->get_caps  = gst_pcs_viewer_sink_get_caps;
  //gstbasesink_class->set_caps  = gst_pcs_viewer_sink_set_caps;

  //gstbasesink_class->get_times  = NULL;
  gstbasesink_class->start      = GST_DEBUG_FUNCPTR(gst_pcs_viewer_sink_start);
  gstbasesink_class->stop       = GST_DEBUG_FUNCPTR(gst_pcs_viewer_sink_stop);
  gstbasesink_class->render     = GST_DEBUG_FUNCPTR(gst_pcs_viewer_sink_render);
}

#ifdef EU_HAS_GST_1_0
static void gst_pcs_viewer_sink_init(GstPcsViewerSink *pcs_viewer_sink)
#else // 0.10
static void gst_pcs_viewer_sink_init(GstPcsViewerSink      *pcs_viewer_sink,
                                     GstPcsViewerSinkClass *gclass)
#endif // EU_HAS_GST_1_0
{
  // properties
  pcs_viewer_sink->m_nBgRgb       = PCS_VIEWER_SINK_BG_DFT;
  pcs_viewer_sink->m_nPointRgb    = PCS_VIEWER_SINK_PT_COLOR_DFT;
  pcs_viewer_sink->m_nPointSize   = PCS_VIEWER_SINK_PT_SIZE_DFT;
  
  // state
  pcs_viewer_sink->m_bFirstCloud  = true;
  pcs_viewer_sink->m_pPcdViewer   = NULL;

  gst_pcd_viewer_init(pcs_viewer_sink);
}

#ifndef EU_HAS_GST_1_0
static void gst_pcs_viewer_sink_finalize(GObject *object)
{
  GstPcsViewerSink *pcs_viewer_sink = GST_PCS_VIEWER_SINK(object);

  // do any final initializations

  G_OBJECT_CLASS(parent_class)->finalize(object);
}
#endif // !EU_HAS_GST_1_0

#ifdef EU_GST_FUTURE
static GstCaps *gst_pcs_viewer_sink_get_caps(GstPad *pad)
{
  GstPcsViewerSink *pcs_viewer_sink = 
                                GST_PCS_VIEWER_SINK(gst_pad_get_parent(pad));

  GstPad        *otherpad;
  const GstCaps *tcaps;
  GstCaps       *caps;
  GstCaps       *capsResult;
  const char    *name;
  int            i;
  GstStructure  *structure = NULL;

  // We want to proxy properties like width, height and framerate from the
  // other end of the element.
  //otherpad = (pad == oni_pdu_enc->srcpad)? oni_pdu_enc->sinkpad:
  //                                         oni_pdu_enc->srcpad;

  // get template caps, we always need this to fiter the peer caps
  //tcaps = gst_pad_get_pad_template_caps(otherpad);

  // get any constraints on the peer pad
  caps = gst_pad_peer_get_caps(otherpad);

  if( caps == NULL )
  {
    caps = gst_caps_copy(tcaps);
  }
  else
  {
    caps = gst_caps_make_writable(caps);
  }

  // intersect with the template
  capsResult = gst_caps_intersect(caps, tcaps);

  gst_caps_unref(caps);

  if( pad == oni_pdu_enc->srcpad )
  {
    name = "oni/pdu";
  }
  else
  {
    //name = "oni/pcs-binary";   // RDK TODO need to read from src pad
    name = "ANY";
  }

  //
  // we can only copy width, height, framerate from one side to the other
  //
  for(i=0; i<gst_caps_get_size(capsResult); i++)
  {
    structure = gst_caps_get_structure(capsResult, i);

    gst_structure_set_name(structure, name);

    //gst_structure_remove_field (structure, "format");
    /* ... but for the sink pad, we only do I420 anyway, so add that */
    //if (pad == oni_pdu_enc->sinkpad)
    //{
    //  gst_structure_set (structure, "format", GST_TYPE_FOURCC,
    //      GST_STR_FOURCC ("I420"), NULL);
    //}
  }

#if 0 // RDK investigate
#endif // RDK investigate

  return capsResult;

}
#endif // EU_GST_FUTURE

static gboolean gst_pcs_viewer_sink_set_caps(GstPad *pad, GstCaps *caps)
{
  GstPcsViewerSink *pcs_viewer_sink;
  GstStructure     *structure;
  gboolean          rc;

  pcs_viewer_sink = GST_PCS_VIEWER_SINK(gst_pad_get_parent(pad));

  structure = gst_caps_get_structure(caps, 0);

  gst_pcd_viewer_set_caps(pcs_viewer_sink);

  return TRUE;
}

static void gst_pcs_viewer_sink_set_property(GObject      *object,
                                             guint         prop_id,
                                             const GValue *value,
                                             GParamSpec   *pspec)
{
  GstPcsViewerSink *pcs_viewer_sink;

  g_return_if_fail(GST_IS_PCS_VIEWER_SINK(object));

  pcs_viewer_sink = GST_PCS_VIEWER_SINK(object);

  switch( prop_id )
  {
    case PROP_BG:
      pcs_viewer_sink->m_nBgRgb = g_value_get_int(value);
      GST_INFO("Background color 0x%06x.", pcs_viewer_sink->m_nBgRgb);
      break;
 
    case PROP_PT_COLOR:
      pcs_viewer_sink->m_nPointRgb = g_value_get_int(value);
      if( pcs_viewer_sink->m_nPointRgb == PCS_VIEWER_SINK_PT_COLOR_DFT )
      {
        GST_INFO("Point cloud auto color.");
      }
      else
      {
        GST_INFO("Cloud point color 0x%06x.", pcs_viewer_sink->m_nPointRgb);
      }
      break;
 
    case PROP_PT_SIZE:
      pcs_viewer_sink->m_nPointSize = g_value_get_int(value);
      GST_INFO("Cloud point size %d.", pcs_viewer_sink->m_nPointSize);
      break;
 
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
      break;
  }
}

static void gst_pcs_viewer_sink_get_property(GObject    *object,
                                             guint       prop_id,
                                             GValue     *value,
                                             GParamSpec *pspec)
{
  GstPcsViewerSink *pcs_viewer_sink;

  g_return_if_fail(GST_IS_PCS_VIEWER_SINK(object));

  pcs_viewer_sink = GST_PCS_VIEWER_SINK(object);

  switch( prop_id )
  {
    case PROP_BG:
      g_value_set_int(value, pcs_viewer_sink->m_nBgRgb);
      break;
    case PROP_PT_COLOR:
      g_value_set_int(value, pcs_viewer_sink->m_nPointRgb);
      break;
    case PROP_PT_SIZE:
      g_value_set_int(value, pcs_viewer_sink->m_nPointSize);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
      break;
  }
}

static gboolean gst_pcs_viewer_sink_start(GstBaseSink *sink)
{
  GstPcsViewerSink *pcs_viewer_sink;

  pcs_viewer_sink = GST_PCS_VIEWER_SINK(sink);

  pcs_viewer_sink->m_bFirstCloud  = true;

  return gst_pcd_viewer_start(pcs_viewer_sink);
}

static gboolean gst_pcs_viewer_sink_stop(GstBaseSink *sink)
{
  GstPcsViewerSink *pcs_viewer_sink;

  pcs_viewer_sink = GST_PCS_VIEWER_SINK(sink);

  return gst_pcd_viewer_stop(pcs_viewer_sink);
}

static GstFlowReturn gst_pcs_viewer_sink_render(GstBaseSink *sink,
                                                GstBuffer   *buf)
{
  GstPcsViewerSink *pcs_viewer_sink;
  guchar           *bufData;
  size_t            bufSize;

#ifdef EU_HAS_GST_1_0
  GstMapInfo      info;
  gst_buffer_map(buf, &info, GST_MAP_READ);
  bufData = info.data;
  bufSize = info.size;
  gst_buffer_unmap(buf, &info);
#else // 0.10
  bufData = GST_BUFFER_DATA(buf);
  bufSize = (size_t)GST_BUFFER_SIZE(buf);
#endif // EU_HAS_GST_1_0

  pcs_viewer_sink = GST_PCS_VIEWER_SINK(sink);

  return gst_pcd_viewer_render(pcs_viewer_sink, bufData, bufSize);
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
  GST_DEBUG_CATEGORY_INIT(pcs_viewer_sink_debug, "pcsviewersink", 0,
      "Point Cloud Stream Viewer");

  return gst_element_register(plugin,
                              "pcsviewersink",
                              GST_RANK_NONE,
                              GST_TYPE_PCS_VIEWER_SINK);
}

/*!
 * GStreamer looks for this structure to register this plug-in.
 */
GST_PLUGIN_DEFINE (
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    pcsviewersink,
    "Point Cloud Stream viewer sink element",
    plugin_init,
    VERSION,
    EU_GST_PACKAGE_LICENSE,
    EU_GST_PACKAGE_NAME,
    EU_GST_PACKAGE_URL
)
