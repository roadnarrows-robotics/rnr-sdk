////////////////////////////////////////////////////////////////////////////////
//
// Package:     Eudoxus
//
// SubPackage:  GStreamer
//
// Plug-In:     libgstonipduenc
//
// File:        gstonipduenc.cxx
//
/*! \file
 *
 * $LastChangedDate: 2016-01-19 17:29:01 -0700 (Tue, 19 Jan 2016) $
 * $Rev: 4267 $
 *
 * \brief Eudoxus GStreamer OpenNI Protocol Data Unit encoder element plug-in
 * definitions.
 *
 * The onipduenc plug-in (optionally) compresses OpenNI frames into network
 * capable PDU's.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * 
 * \copyright
 *   \h_copy 2012-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
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
 * gst-launch onisrc node=depth ! oni/pcs-ascii ! onipduenc compress=zlib ! udpsink
 * ]| This generates a point cloud stream from and OpenNi device, compresses
 * the stream and chunks the compressed frame in network capable PDUs.
 * </refsect2>
 */

#include "Eudoxus/euConf.h"

#include <string.h>
#include <zlib.h>

#include <gst/gst.h>

#include "gsteudoxus.h"

#include "gstonipduenc.h"
#include "gstonipdu.h"

GstDebugCategory *GST_CAT_DEFAULT;


/* OniPduEnc signals and args */
enum
{
  LAST_SIGNAL
};

enum
{
  PROP_0,
  PROP_COMPRESSION,
  PROP_LAST
};

static GstStaticPadTemplate gst_oni_pdu_enc_sink_pad_template =
  GST_STATIC_PAD_TEMPLATE("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS("ANY")
  );

#if 0
    "oni/pcs-ascii, "
      "width = (int) { 320, 640 }, "
      "height = (int) { 240, 480 }, "
      "mirror = (boolean) true, "
      "endianness = (int) BYTE_ORDER"
  )
#endif

static GstStaticPadTemplate gst_oni_pdu_enc_src_pad_template =
  GST_STATIC_PAD_TEMPLATE("src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS("oni/pdu")
  );

#ifdef EU_HAS_GST_1_0
G_DEFINE_TYPE(GstOniPduEnc, gst_oni_pdu_enc, GST_TYPE_ELEMENT);
#else // 0.10
GST_BOILERPLATE(GstOniPduEnc, gst_oni_pdu_enc, GstElement, GST_TYPE_ELEMENT);
#endif // EU_HAS_GST_1_0


static void gst_oni_pdu_enc_base_init(gpointer g_class);
static void gst_oni_pdu_enc_class_init(GstOniPduEncClass *klass);

#ifdef EU_HAS_GST_1_0
static void gst_oni_pdu_enc_init(GstOniPduEnc *oni_pdu_enc);
#else // 0.10
static void gst_oni_pdu_enc_init(GstOniPduEnc      *oni_pdu_enc,
                                 GstOniPduEncClass *gclass);
#endif // EU_HAS_GST_1_0

#ifndef EU_HAS_GST_1_0
static void gst_oni_pdu_enc_finalize(GObject *object);
#endif // !EU_HAS_GST_1_0

#ifndef EU_HAS_GST_1_0
static GstStateChangeReturn gst_oni_pdu_enc_change_state(GstElement *element,
                                                    GstStateChange transition);
#endif // !EU_HAS_GST_1_0

static GstFlowReturn gst_oni_pdu_enc_chain(GstPad *pad,
                                          GstObject *parent,
                                          GstBuffer *buf);

#ifdef EU_HAS_GST_1_0
static gboolean gst_oni_pdu_enc_query_caps(GstPad *pad,
                                  GstObject *parent, GstQuery *query);
#else // 0.10
static GstCaps *gst_oni_pdu_enc_get_caps(GstPad *pad);
#endif // EU_HAS_GST_1_0

static gboolean gst_oni_pdu_enc_set_caps(GstPad *pad, GstCaps *caps);

static void gst_oni_pdu_enc_set_property(GObject      *object,
                                         guint         prop_id,
                                         const GValue *value,
                                         GParamSpec   *pspec);
static void gst_oni_pdu_enc_get_property(GObject    *object,
                                         guint       prop_id,
                                         GValue     *value,
                                         GParamSpec *pspec);

static GType gst_oni_pdu_compression_get_type()
{
  static GType oni_pdu_compression_type = 0;

  static const GEnumValue compression_types[] =
  {
    { GST_ONI_PDU_COMPRESS_NONE,
      "No compression",
      "none"
    },
    { GST_ONI_PDU_COMPRESS_ZLIB,
      "ZLib compression",
      "zlib"
    },
    {0, NULL, NULL}
  };

  if( !oni_pdu_compression_type )
  {
    oni_pdu_compression_type = 
          g_enum_register_static("GstOniPduCompression", compression_types);
  }

  return oni_pdu_compression_type;
}


static void gst_oni_pdu_enc_base_init(gpointer g_class)
{
  GstElementClass *gstelement_class = GST_ELEMENT_CLASS(g_class);

  gst_element_class_add_pad_template(gstelement_class,
      gst_static_pad_template_get(&gst_oni_pdu_enc_sink_pad_template));
  gst_element_class_add_pad_template(gstelement_class,
      gst_static_pad_template_get(&gst_oni_pdu_enc_src_pad_template));

  gst_element_class_set_details_simple(gstelement_class,
    "OpenNI PDU encoder",
    "Oni/PDU",
    "OpenNI encoder to compress and fragment OpenNI frames into PDUs",
    "RoadNarrows <oneway@roadnarrows.com>");
}

static void gst_oni_pdu_enc_class_init(GstOniPduEncClass *klass)
{
  GObjectClass    *gobject_class;
  GstElementClass *gstelement_class;

  gobject_class     = (GObjectClass *)klass;
  gstelement_class  = (GstElementClass *)klass;

#ifndef EU_HAS_GST_1_0
  gobject_class->finalize     = gst_oni_pdu_enc_finalize;
#endif // !EU_HAS_GST_1_0

  gobject_class->set_property = gst_oni_pdu_enc_set_property;
  gobject_class->get_property = gst_oni_pdu_enc_get_property;

  g_object_class_install_property(gobject_class, PROP_COMPRESSION,
      g_param_spec_enum ("compression", "Compression",
          "Data compression algorithm",
          gst_oni_pdu_compression_get_type(),
          GST_ONI_PDU_COMPRESS_NONE,
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

#ifndef EU_HAS_GST_1_0
  gstelement_class->change_state =
      GST_DEBUG_FUNCPTR(gst_oni_pdu_enc_change_state);
#endif // !EU_HAS_GST_1_0
}

#ifdef EU_HAS_GST_1_0
static void gst_oni_pdu_enc_init(GstOniPduEnc *oni_pdu_enc)
#else // 0.10
static void gst_oni_pdu_enc_init(GstOniPduEnc      *oni_pdu_enc,
                                 GstOniPduEncClass *gclass)
#endif // EU_HAS_GST_1_0
{
  // create the sink pad
  oni_pdu_enc->sinkpad = gst_pad_new_from_static_template(
                                          &gst_oni_pdu_enc_sink_pad_template,
                                          "sink");
  gst_pad_set_chain_function(oni_pdu_enc->sinkpad, gst_oni_pdu_enc_chain);

  //gst_pad_set_setcaps_function(oni_pdu_enc->sinkpad, gst_oni_pdu_enc_set_caps);
  gst_element_add_pad(GST_ELEMENT(oni_pdu_enc), oni_pdu_enc->sinkpad);

  // create the source pad
  oni_pdu_enc->srcpad = gst_pad_new_from_static_template(
                                          &gst_oni_pdu_enc_src_pad_template,
                                          "src");
#ifdef EU_HAS_GST_1_0
  gst_pad_set_query_function(oni_pdu_enc->srcpad, gst_oni_pdu_enc_query_caps);
#else // 0.10
  gst_pad_set_getcaps_function(oni_pdu_enc->srcpad, gst_oni_pdu_enc_get_caps);
#endif // EU_HAS_GST_1_0

  gst_pad_use_fixed_caps(oni_pdu_enc->srcpad);
  gst_element_add_pad(GST_ELEMENT(oni_pdu_enc), oni_pdu_enc->srcpad);

  gst_oni_pdu_state_defaults(oni_pdu_enc->m_state);
}

#ifndef EU_HAS_GST_1_0
static void gst_oni_pdu_enc_finalize(GObject *object)
{
  GstOniPduEnc *oni_pdu_enc = GST_ONI_PDU_ENC(object);

  // do any final initializations

  G_OBJECT_CLASS(parent_class)->finalize(object);
}
#endif // !EU_HAS_GST_1_0

#ifdef EU_HAS_GST_1_0
static gboolean gst_oni_pdu_enc_query_caps(GstPad *pad,
                                          GstObject *parent,
                                          GstQuery *query)
{
  GstOniPduEnc  *oni_pdu_enc = GST_ONI_PDU_ENC(parent);

  return TRUE;
}

#else // 0.10
static GstCaps *gst_oni_pdu_enc_get_caps(GstPad *pad)
{
  GstOniPduEnc  *oni_pdu_enc = GST_ONI_PDU_ENC(gst_pad_get_parent(pad));
  GstPad        *otherpad;
  const GstCaps *tcaps;
  GstCaps       *caps;
  GstCaps       *capsResult;
  const char    *name;
  int            i;
  GstStructure  *structure = NULL;

  // We want to proxy properties like width, height and framerate from the
  // other end of the element.
  otherpad = (pad == oni_pdu_enc->srcpad)? oni_pdu_enc->sinkpad:
                                           oni_pdu_enc->srcpad;

  // get template caps, we always need this to fiter the peer caps
  tcaps = gst_pad_get_pad_template_caps(otherpad);

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
#endif // EU_HAS_GST_1_0

static gboolean gst_oni_pdu_enc_set_caps(GstPad *pad, GstCaps *caps)
{
  GstOniPduEnc *oni_pdu_enc;
  GstStructure *structure;
  const GValue *framerate;
  GstCaps      *srccaps;
  gboolean      rc;

  oni_pdu_enc = GST_ONI_PDU_ENC(gst_pad_get_parent(pad));

  structure = gst_caps_get_structure(caps, 0);

  if( pad == oni_pdu_enc->srcpad )
  {
    GST_DEBUG("set_caps: srcpad");
  }
  else
  {
    GST_DEBUG("set_caps: sinkpad");
  }

#if 0 // RDK TODO investigate
  framerate = gst_structure_get_value (structure, "framerate");

  if( framerate )
  {
    oni_pdu_enc->fps_num = gst_value_get_fraction_numerator(framerate);
    oni_pdu_enc->fps_denom = gst_value_get_fraction_denominator(framerate);
  }
  else
  {
    oni_pdu_enc->fps_num = 0;
    oni_pdu_enc->fps_denom = 1;
  }

  gst_structure_get_int(structure, "width", &oni_pdu_enc->width);
  gst_structure_get_int(structure, "height", &oni_pdu_enc->height);

  srccaps = gst_caps_new_simple ("video/pdu",
      "width",  G_TYPE_INT, oni_pdu_enc->width,
      "height", G_TYPE_INT, oni_pdu_enc->height,
      "framerate", GST_TYPE_FRACTION, oni_pdu_enc->fps_num,
                                      oni_pdu_enc->fps_denom,
      NULL);

  rc = gst_pad_set_caps(oni_pdu_enc->srcpad, srccaps);

  gst_caps_unref(srccaps);

  return rc;
#endif // RDK TODO investigate

  return TRUE;
}

static GstBuffer *oni_pdu_enc_compress_zlib(GstOniPduEnc *oni_pdu_enc,
                                            guchar       *pInData,
                                            guint         uInSize)
{
  GstBuffer  *bufComp;
  guchar     *pCompData;
  gulong      uCompSize;
  int         rc;

#ifdef EU_HAS_GST_1_0
  GstMapInfo      info;
#endif // EU_HAS_GST_1_0

  // compressed buffer
  bufComp = gst_buffer_new_and_alloc(uInSize);

#ifdef EU_HAS_GST_1_0
  gst_buffer_map(bufComp, &info, GST_MAP_READ);
  pCompData = info.data;
  gst_buffer_unmap(bufComp, &info);
#else // 0.10
  pCompData = GST_BUFFER_DATA(bufComp);
#endif // EU_HAS_GST_1_0

  uCompSize = (gulong)uInSize;

  rc = compress(pCompData, &uCompSize, pInData, (gulong)uInSize);

  if( rc != Z_OK)
  {
    GST_ERROR("Failed to compress frame.");
    gst_buffer_unref(bufComp);
    return NULL;
  }

#ifdef EU_HAS_GST_1_0
  gst_buffer_map(bufComp, &info, GST_MAP_WRITE);
  info.size = uCompSize;
  gst_buffer_unmap(bufComp, &info);
#else // 0.10
  GST_BUFFER_SIZE(bufComp) = (guint)uCompSize;
#endif // EU_HAS_GST_1_0

  return bufComp;
}

static GstFlowReturn gst_oni_pdu_enc_chain(GstPad *pad,
                                            GstObject *parent,
                                            GstBuffer *buf)
{
  GstOniPduEnc   *oni_pdu_enc;
  GstOniPduState *pState;
  guchar         *pInData;
  guint           uInSize;
  GstBuffer      *bufCompressed;
  guint           uFrameSize;
  guchar         *pOutData;
  guint           uPayloadSize;
  GstFlowReturn   rc;

#ifdef EU_HAS_GST_1_0
  GstMapInfo      info;
#endif // EU_HAS_GST_1_0

  oni_pdu_enc = GST_ONI_PDU_ENC(GST_OBJECT_PARENT(pad));

  pState = &oni_pdu_enc->m_state;

#ifdef EU_HAS_GST_1_0
  gst_buffer_map(buf, &info, GST_MAP_READ);
  pInData = info.data;
  uInSize = info.size;
  gst_buffer_unmap(buf, &info);
#else // 0.10
  pInData = GST_BUFFER_DATA(buf);
  uInSize = GST_BUFFER_SIZE(buf);
#endif // EU_HAS_GST_1_0

  uFrameSize = uInSize;

  GST_LOG("Received frame buffer of %u bytes", uInSize);

  switch( oni_pdu_enc->m_eCompression )
  {
    case GST_ONI_PDU_COMPRESS_ZLIB:
      bufCompressed = oni_pdu_enc_compress_zlib(oni_pdu_enc, pInData, uInSize);
      if( bufCompressed != NULL )
      {
        gst_buffer_unref(buf);
        buf = bufCompressed;

#ifdef EU_HAS_GST_1_0
        gst_buffer_map(buf, &info, GST_MAP_READ);
        pInData = info.data;
        uInSize = info.size;
        gst_buffer_unmap(buf, &info);
#else // 0.10
        pInData = GST_BUFFER_DATA(buf);
        uInSize = GST_BUFFER_SIZE(buf);
#endif // EU_HAS_GST_1_0
      }
      else
      {
        oni_pdu_enc->m_eCompression = GST_ONI_PDU_COMPRESS_NONE;
      }
      break;

    case GST_ONI_PDU_COMPRESS_NONE:
    default:
      break;
  }

  pState->m_eStateId  = GstOniPduStateIdStart;
  pState->m_uOffset   = 0;
  pState->m_pOutBuf   = NULL;

  GST_INFO("Start: %sdata_size=%u.",
    (oni_pdu_enc->m_eCompression==GST_ONI_PDU_COMPRESS_NONE? "": "compressed "),
    uInSize);

  gst_oni_pdu_hdr_init(pState->m_hdr,
                       oni_pdu_enc->m_eCompression,
                       uFrameSize,
                       uInSize);

  oni_pdu_enc->m_state.m_eStateId = GstOniPduStateIdFrag;

  for(pState->m_uOffset = 0;
      pState->m_uOffset < uInSize;
      pState->m_uOffset += uPayloadSize)
  {
    uPayloadSize = gst_oni_pdu_payload_size(pState->m_uOffset, uInSize);

    pState->m_pOutBuf = 
                  gst_buffer_new_and_alloc(GST_ONI_PDU_HDR_SIZE+uPayloadSize);
#ifdef EU_HAS_GST_1_0
    gst_buffer_map(pState->m_pOutBuf, &info, GST_MAP_READ);
    pOutData = info.data;
    gst_buffer_unmap(pState->m_pOutBuf, &info);
#else // 0.10
    pOutData = GST_BUFFER_DATA(pState->m_pOutBuf);
#endif // EU_HAS_GST_1_0

    GST_BUFFER_TIMESTAMP(pState->m_pOutBuf)  = GST_BUFFER_TIMESTAMP(buf);
    GST_BUFFER_DURATION(pState->m_pOutBuf)   = GST_BUFFER_DURATION(buf);

#ifndef EU_HAS_GST_1_0
    gst_buffer_set_caps(pState->m_pOutBuf, GST_PAD_CAPS(oni_pdu_enc->srcpad));
#endif // !EU_HAS_GST_1_0

    gst_oni_pdu_pack_hdr(pState->m_hdr, pOutData);

    gst_oni_pdu_pack_payload(pInData+pState->m_uOffset,
                             pOutData+GST_ONI_PDU_HDR_SIZE,
                             uPayloadSize);

    rc = gst_pad_push(oni_pdu_enc->srcpad, pState->m_pOutBuf);

    if( rc != GST_FLOW_OK )
    {
      gst_oni_pdu_state_reset(*pState);
      return rc;
    }

    GST_LOG("Frag: Sent pdu_num=%u, payload_size=%u, %u/%u.",
        pState->m_hdr.m_uPduNum, uPayloadSize,
        pState->m_uOffset+uPayloadSize, uInSize);
          
    pState->m_hdr.m_uPduNum++;
    pState->m_pOutBuf = NULL;
  }

  pState->m_eStateId = GstOniPduStateIdEnd;

  gst_buffer_unref(buf);

  GST_INFO("End: Sent %sdata_size=%u in %u PDUs.",
    (oni_pdu_enc->m_eCompression==GST_ONI_PDU_COMPRESS_NONE? "": "compressed "),
    uInSize, pState->m_hdr.m_uPduNum);

  return GST_FLOW_OK;
}

static void gst_oni_pdu_enc_set_property(GObject      *object,
                                         guint         prop_id,
                                         const GValue *value,
                                         GParamSpec   *pspec)
{
  GstOniPduEnc *oni_pdu_enc;

  g_return_if_fail(GST_IS_ONI_PDU_ENC(object));

  oni_pdu_enc = GST_ONI_PDU_ENC(object);

  switch( prop_id )
  {
    case PROP_COMPRESSION:
      oni_pdu_enc->m_eCompression = g_value_get_enum(value);
      GST_INFO("Compression=%d.", oni_pdu_enc->m_eCompression); 
      break;
    default:
      break;
  }
}

static void gst_oni_pdu_enc_get_property(GObject    *object,
                                         guint       prop_id,
                                         GValue     *value,
                                         GParamSpec *pspec)
{
  GstOniPduEnc *oni_pdu_enc;

  g_return_if_fail(GST_IS_ONI_PDU_ENC(object));

  oni_pdu_enc = GST_ONI_PDU_ENC(object);

  switch( prop_id )
  {
    case PROP_COMPRESSION:
      g_value_set_enum(value, oni_pdu_enc->m_eCompression);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
      break;
  }
}

#ifndef EU_HAS_GST_1_0
static GstStateChangeReturn gst_oni_pdu_enc_change_state(GstElement *element,
                                                      GstStateChange transition)
{
  GstStateChangeReturn ret;
  GstOniPduEnc *oni_pdu_enc;

  oni_pdu_enc = GST_ONI_PDU_ENC(element);

  // do any pre-transition work
  switch( transition )
  {
    case GST_STATE_CHANGE_READY_TO_PAUSED:
    default:
      break;
  }

  ret = GST_ELEMENT_CLASS(parent_class)->change_state(element, transition);

  if( ret != GST_STATE_CHANGE_SUCCESS )
  {
    return ret;
  }

  // do any post-transition work
  switch( transition )
  {
    case GST_STATE_CHANGE_PAUSED_TO_READY:
      break;
    default:
      break;
  }

  return ret;
}
#endif // !EU_HAS_GST_1_0

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
  GST_DEBUG_CATEGORY_INIT(oni_pdu_enc_debug, "onipduenc", 0,
      "OpenNI PDU Encoder");

  return gst_element_register(plugin,
                              "onipduenc",
                              GST_RANK_NONE,
                              GST_TYPE_ONI_PDU_ENC);
}

/*!
 * GStreamer looks for this structure to register this plug-in.
 */
GST_PLUGIN_DEFINE (
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    onipduenc,
    "OpenNI PDU encoder element",
    plugin_init,
    VERSION,
    EU_GST_PACKAGE_LICENSE,
    EU_GST_PACKAGE_NAME,
    EU_GST_PACKAGE_URL
)
