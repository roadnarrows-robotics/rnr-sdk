////////////////////////////////////////////////////////////////////////////////
//
// Package:     Eudoxus
//
// SubPackage:  GStreamer
//
// Plug-In:     libgstonipdudec
//
// File:        gstonipdudec.cxx
//
/*! \file
 *
 * $LastChangedDate: 2016-01-19 17:29:01 -0700 (Tue, 19 Jan 2016) $
 * $Rev: 4267 $
 *
 * \brief Eudoxus GStreamer OpenNI Protocol Data Unit decoder element plug-in
 * definitions.
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
 * OpenNI PDU decoder element.
 *
 * <refsect2>
 * <title>Example launch line</title>
 * |[
 * gst-launch onisrc node=depth ! oni/pcs-ascii ! onipdudec compress=zlib ! udpsink
 * ]| This generates a point cloud stream from and OpenNi device, compresses
 * the stream and chunks the compressed frame in network capable PDUs.
 * </refsect2>
 */

#include "Eudoxus/euConf.h"

#include <string.h>
#include <zlib.h>

#include <gst/gst.h>

#include "gsteudoxus.h"

#include "gstonipdudec.h"
#include "gstonipdu.h"

GstDebugCategory *GST_CAT_DEFAULT;


/* OniPduDec signals and args */
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

static GstStaticPadTemplate gst_oni_pdu_dec_sink_pad_template =
  GST_STATIC_PAD_TEMPLATE("sink",
    GST_PAD_SINK,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS("oni/pdu")
  );

static GstStaticPadTemplate gst_oni_pdu_dec_src_pad_template =
  GST_STATIC_PAD_TEMPLATE("src",
    GST_PAD_SRC,
    GST_PAD_ALWAYS,
    GST_STATIC_CAPS("ANY")
  );

#ifdef EU_HAS_GST_1_0
G_DEFINE_TYPE(GstOniPduDec, gst_oni_pdu_dec, GST_TYPE_ELEMENT);
#else // 0.10
GST_BOILERPLATE(GstOniPduDec, gst_oni_pdu_dec, GstElement, GST_TYPE_ELEMENT);
#endif // EU_HAS_GST_1_0


static void gst_oni_pdu_dec_base_init(gpointer g_class);
static void gst_oni_pdu_dec_class_init(GstOniPduDecClass *klass);

#ifdef EU_HAS_GST_1_0
static void gst_oni_pdu_dec_init(GstOniPduDec *oni_pdu_dec);
#else // 0.10
static void gst_oni_pdu_dec_init(GstOniPduDec      *oni_pdu_dec,
                                 GstOniPduDecClass *gclass);
#endif // EU_HAS_GST_1_0

#ifndef EU_HAS_GST_1_0
static gboolean gst_oni_pdu_dec_finalize(GObject *object);
#endif // !EU_HAS_GST_1_0

#ifndef EU_HAS_GST_1_0
static GstStateChangeReturn gst_oni_pdu_dec_change_state(GstElement *element,
                                                    GstStateChange transition);
#endif // !EU_HAS_GST_1_0

#ifdef EU_HAS_GST_1_0
static GstFlowReturn gst_oni_pdu_dec_chain(GstPad *pad,
                                            GstObject *parent,
                                            GstBuffer *buf);
#else // 0.10
static GstFlowReturn gst_oni_pdu_dec_chain(GstPad *pad, GstBuffer *buf);
#endif // EU_HAS_GST_1_0

#ifdef EU_HAS_GST_1_0
static gboolean gst_oni_pdu_dec_query_caps(GstPad *pad,
                                  GstObject *parent, GstQuery *query);
#else // 0.10
static GstCaps *gst_oni_pdu_dec_get_caps(GstPad *pad);
#endif // EU_HAS_GST_1_0

static gboolean gst_oni_pdu_dec_set_caps(GstPad *pad, GstCaps *caps);

static void gst_oni_pdu_dec_set_property(GObject      *object,
                                         guint         prop_id,
                                         const GValue *value,
                                         GParamSpec   *pspec);
static void gst_oni_pdu_dec_get_property(GObject    *object,
                                         guint       prop_id,
                                         GValue     *value,
                                         GParamSpec *pspec);

static void gst_oni_pdu_dec_state_flushing(GstOniPduState &state)
{
  gst_oni_pdu_state_reset(state);
  state.m_eStateId = GstOniPduStateIdFlushing;
  GST_INFO("Flushing.");
}

static void gst_oni_pdu_dec_base_init(gpointer g_class)
{
  GstElementClass *gstelement_class = GST_ELEMENT_CLASS(g_class);

  gst_element_class_add_pad_template(gstelement_class,
      gst_static_pad_template_get(&gst_oni_pdu_dec_sink_pad_template));
  gst_element_class_add_pad_template (gstelement_class,
      gst_static_pad_template_get(&gst_oni_pdu_dec_src_pad_template));

  gst_element_class_set_details_simple(gstelement_class,
    "OpenNI PDU decoder",
    "Oni/PDU",
    "OpenNI decoder to defragment OpenNI PDUs to (compressed) frames "
    "and decompress",
    "RoadNarrows <oneway@roadnarrows.com>");
}

static void gst_oni_pdu_dec_class_init(GstOniPduDecClass *klass)
{
  GObjectClass    *gobject_class;
  GstElementClass *gstelement_class;

  gobject_class     = (GObjectClass *)klass;
  gstelement_class  = (GstElementClass *)klass;

#ifndef EU_HAS_GST_1_0
  gobject_class->finalize     = gst_oni_pdu_dec_finalize;
#endif // !EU_HAS_GST_1_0

  gobject_class->set_property = gst_oni_pdu_dec_set_property;
  gobject_class->get_property = gst_oni_pdu_dec_get_property;

#ifndef EU_HAS_GST_1_0
  gstelement_class->change_state =
      GST_DEBUG_FUNCPTR(gst_oni_pdu_dec_change_state);
#endif // !EU_HAS_GST_1_0
}

#ifdef EU_HAS_GST_1_0
static void gst_oni_pdu_dec_init(GstOniPduDec *oni_pdu_dec)
#else // 0.10
static void gst_oni_pdu_dec_init(GstOniPduDec      *oni_pdu_dec,
                                 GstOniPduDecClass *gclass)
#endif // EU_HAS_GST_1_0
{
  // create the sink pad
  oni_pdu_dec->sinkpad = gst_pad_new_from_static_template(
                                          &gst_oni_pdu_dec_sink_pad_template,
                                          "sink");

#ifdef EU_HAS_GST_1_0
  gst_pad_set_chain_function(oni_pdu_dec->sinkpad, gst_oni_pdu_dec_chain);
#else // 0.10
  gst_pad_set_chain_function(oni_pdu_dec->sinkpad, gst_oni_pdu_dec_chain);
#endif // EU_HAS_GST_1_0

#ifdef EU_HAS_GST_1_0
  gst_pad_set_query_function(oni_pdu_dec->sinkpad, gst_oni_pdu_dec_query_caps);
#else // 0.10
  gst_pad_set_getcaps_function(oni_pdu_dec->sinkpad, gst_oni_pdu_dec_get_caps);
#endif // EU_HAS_GST_1_0

  //gst_pad_set_setcaps_function(oni_pdu_dec->sinkpad, gst_oni_pdu_dec_set_caps);

  gst_element_add_pad(GST_ELEMENT(oni_pdu_dec), oni_pdu_dec->sinkpad);

  // create the source pad
  oni_pdu_dec->srcpad = gst_pad_new_from_static_template(
                                          &gst_oni_pdu_dec_src_pad_template,
                                          "src");

#ifdef EU_HAS_GST_1_0
  gst_pad_set_query_function(oni_pdu_dec->srcpad, gst_oni_pdu_dec_query_caps);
#else // 0.10
  gst_pad_set_getcaps_function(oni_pdu_dec->srcpad, gst_oni_pdu_dec_get_caps);
#endif // EU_HAS_GST_1_0

  gst_pad_use_fixed_caps(oni_pdu_dec->srcpad);
  gst_element_add_pad(GST_ELEMENT(oni_pdu_dec), oni_pdu_dec->srcpad);

  gst_oni_pdu_state_defaults(oni_pdu_dec->m_state);
}

#ifndef EU_HAS_GST_1_0
static void gst_oni_pdu_dec_finalize(GObject *object)
{
  GstOniPduDec *oni_pdu_dec = GST_ONI_PDU_DEC(object);

  // do any final initializations

  G_OBJECT_CLASS(parent_class)->finalize(object);
}
#endif // !EU_HAS_GST_1_0

#ifdef EU_HAS_GST_1_0
static gboolean gst_oni_pdu_dec_query_caps(GstPad *pad,
                                          GstObject *parent,
                                          GstQuery *query)
{
  GstOniPduDec  *oni_pdu_dec = GST_ONI_PDU_DEC(parent);

  return TRUE;
}

#else // 0.10
static GstCaps *gst_oni_pdu_dec_get_caps(GstPad *pad)
{
  GstOniPduDec  *oni_pdu_dec = GST_ONI_PDU_DEC(gst_pad_get_parent(pad));
  GstPad        *otherpad;
  const GstCaps *tcaps;
  GstCaps       *caps;
  GstCaps       *capsResult;
  const char    *name;
  int            i;
  GstStructure  *structure = NULL;

  // We want to proxy properties like width, height and framerate from the
  // other end of the element.
  otherpad = (pad == oni_pdu_dec->srcpad)? oni_pdu_dec->sinkpad:
                                           oni_pdu_dec->srcpad;

  // get template caps, we always need this to filter the peer caps
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

  gst_caps_unref (caps);

  if( pad == oni_pdu_dec->srcpad )
  {
    name = "oni/pcs-ascii";   // RDK TODO need to read from src pad
  }
  else
  {
    name = "oni/pdu";
  }

  //
  // we can only copy width, height, framerate from one side to the other
  //
#if 0 // RDK investigate
  for(i=0; i<gst_caps_get_size(capsResult); i++)
  {
    structure = gst_caps_get_structure (capsResult, i);

    gst_structure_set_name (structure, name);
    gst_structure_remove_field (structure, "format");
    /* ... but for the sink pad, we only do I420 anyway, so add that */
    if (pad == oni_pdu_dec->sinkpad) {
      gst_structure_set (structure, "format", GST_TYPE_FOURCC,
          GST_STR_FOURCC ("I420"), NULL);
    }
  }
#endif // RDK investigate

  return capsResult;
}
#endif // EU_HAS_GST_1_0

static gboolean gst_oni_pdu_dec_set_caps(GstPad *pad, GstCaps *caps)
{
  GstOniPduDec *oni_pdu_dec;
  GstStructure *structure;
  const GValue *framerate;
  GstCaps      *srccaps;
  gboolean      rc;

  oni_pdu_dec = GST_ONI_PDU_DEC(gst_pad_get_parent(pad));

  structure = gst_caps_get_structure(caps, 0);

#if 0 // RDK TODO investigate
  framerate = gst_structure_get_value (structure, "framerate");

  if( framerate )
  {
    oni_pdu_dec->fps_num = gst_value_get_fraction_numerator(framerate);
    oni_pdu_dec->fps_denom = gst_value_get_fraction_denominator(framerate);
  }
  else
  {
    oni_pdu_dec->fps_num = 0;
    oni_pdu_dec->fps_denom = 1;
  }

  gst_structure_get_int(structure, "width", &oni_pdu_dec->width);
  gst_structure_get_int(structure, "height", &oni_pdu_dec->height);

  srccaps = gst_caps_new_simple ("video/pdu",
      "width",  G_TYPE_INT, oni_pdu_dec->width,
      "height", G_TYPE_INT, oni_pdu_dec->height,
      "framerate", GST_TYPE_FRACTION, oni_pdu_dec->fps_num,
                                      oni_pdu_dec->fps_denom,
      NULL);

  rc = gst_pad_set_caps(oni_pdu_dec->srcpad, srccaps);

  gst_caps_unref(srccaps);

  return rc;
#endif // RDK TODO investigate

  return TRUE;
}

static gboolean oni_pdu_dec_uncompress_zlib(GstOniPduDec *oni_pdu_dec)
{
  GstBuffer  *bufUncomp;
  guchar     *pCompData;
  gulong      uCompSize;
  guchar     *pUncompData;
  gulong      uUncompSize;
  int         rc;

#ifdef EU_HAS_GST_1_0
  GstMapInfo      info;
#endif // EU_HAS_GST_1_0

  // new uncompressed buffer
  bufUncomp = gst_buffer_new_and_alloc(oni_pdu_dec->m_state.m_hdr.m_uFrameSize);

#ifdef EU_HAS_GST_1_0
  gst_buffer_map(bufUncomp, &info, GST_MAP_READ);
  pUncompData = info.data;
  gst_buffer_unmap(bufUncomp, &info);
  uUncompSize = (gulong)oni_pdu_dec->m_state.m_hdr.m_uFrameSize;

  gst_buffer_map(oni_pdu_dec->m_state.m_pOutBuf, &info, GST_MAP_READ);
  pCompData = info.data;
  uCompSize = info.size;
  gst_buffer_unmap(oni_pdu_dec->m_state.m_pOutBuf, &info);
#else // 0.10
  pUncompData = GST_BUFFER_DATA(bufUncomp);
  uUncompSize = (gulong)oni_pdu_dec->m_state.m_hdr.m_uFrameSize;

  pCompData = GST_BUFFER_DATA(oni_pdu_dec->m_state.m_pOutBuf);
  uCompSize = GST_BUFFER_SIZE(oni_pdu_dec->m_state.m_pOutBuf);
#endif // EU_HAS_GST_1_0

  rc = uncompress(pUncompData, &uUncompSize, pCompData, uCompSize);

  if( rc != Z_OK)
  {
    GST_ERROR("Failed to uncompress frame.");
    gst_buffer_unref(bufUncomp);
    return FALSE;
  }

  else if( uUncompSize != (gulong)oni_pdu_dec->m_state.m_hdr.m_uFrameSize )
  {
    GST_ERROR("Uncompressed size %lu != expected frame size %u.",
        uUncompSize, oni_pdu_dec->m_state.m_hdr.m_uFrameSize);
    gst_buffer_unref(bufUncomp);
    return FALSE;
  }

  gst_buffer_unref(oni_pdu_dec->m_state.m_pOutBuf);

  oni_pdu_dec->m_state.m_pOutBuf = bufUncomp;

  return TRUE;
}

#ifdef EU_HAS_GST_1_0
static GstFlowReturn gst_oni_pdu_dec_chain(GstPad *pad,
                                           GstObject *parent,
                                           GstBuffer *buf)
{
  GstOniPduDec   *oni_pdu_dec;
  GstOniPduState *pState;
  guchar         *pInData;
  guint           uInSize;
  GstOniPduHdr    hdr;
  guchar         *pOutData;
  guint           uPayloadSize;
  GstFlowReturn   rc;
  GstMapInfo      info;

  oni_pdu_dec = GST_ONI_PDU_DEC(parent);

  pState = &oni_pdu_dec->m_state;

  gst_buffer_map(buf, &info, GST_MAP_READ);
  pInData = info.data;
  uInSize = info.size;
  gst_buffer_unmap(buf, &info);

  GST_LOG("Received PDU buffer of %u bytes", uInSize);

  if( uInSize < GST_ONI_PDU_HDR_SIZE )
  {
    GST_ERROR("PDU buffer size %u < minimum size %u",
        uInSize, GST_ONI_PDU_HDR_SIZE);
    gst_oni_pdu_state_reset(*pState);
    return GST_FLOW_ERROR;
  }
  else if( gst_oni_pdu_unpack_hdr(pInData, hdr) != GST_ONI_PDU_OK )
  {
    GST_ERROR("Bad PDU header");
    gst_oni_pdu_state_reset(*pState);
    return GST_FLOW_ERROR;
  }

  // ---
  // Start of a frame.
  // ---
  if( hdr.m_uPduNum == 0 )
  {
    // reset to start state, freeing any allocated resources
    gst_oni_pdu_state_reset(*pState);

    // copy header
    pState->m_hdr       = hdr;

    // new output buffer
    pState->m_pOutBuf  = gst_buffer_new_and_alloc(pState->m_hdr.m_uDataSize);
    pState->m_uOffset  = 0;

    GST_INFO("Start: data_size=%u.", pState->m_hdr.m_uDataSize);

    pState->m_eStateId = GstOniPduStateIdDefrag;
  }

  // ---
  // Start or Flushing State
  // ---
  if( (pState->m_eStateId == GstOniPduStateIdStart) ||
      (pState->m_eStateId == GstOniPduStateIdFlushing) )
  {
    return GST_FLOW_OK;
  }

  // ---
  // Defragmentation State
  // ---
  if( pState->m_eStateId == GstOniPduStateIdDefrag )
  {
    uPayloadSize = uInSize - GST_ONI_PDU_HDR_SIZE;

    // lost or out-of-sequence PDU
    if( hdr.m_uPduNum != pState->m_hdr.m_uPduNum )
    {
      GST_WARNING("Expected PDU number %u, received PDU number %u.",
          pState->m_hdr.m_uPduNum, hdr.m_uPduNum);
      gst_oni_pdu_dec_state_flushing(*pState);
      return GST_FLOW_OK;
    }

    // too many PDUs
    else if( hdr.m_uPduNum >= pState->m_hdr.m_uPduTot )
    {
      GST_WARNING("Received PDU number %u >= expected total of %u.",
          hdr.m_uPduNum, pState->m_hdr.m_uPduTot);
      gst_oni_pdu_dec_state_flushing(*pState);
      return GST_FLOW_OK;
    }

    // received frame too big
    else if( pState->m_uOffset + uPayloadSize > pState->m_hdr.m_uDataSize )
    {
      GST_WARNING("Received PDU will exceed expected data size of %u.",
          pState->m_hdr.m_uDataSize);
      gst_oni_pdu_dec_state_flushing(*pState);
      return GST_FLOW_OK;
    }

    gst_buffer_map(pState->m_pOutBuf, &info, GST_MAP_READ);
    pOutData = info.data;
    gst_buffer_unmap(pState->m_pOutBuf, &info);

    gst_oni_pdu_unpack_payload(pInData+GST_ONI_PDU_HDR_SIZE,
                               pOutData+pState->m_uOffset,
                               uPayloadSize);

    pState->m_hdr.m_uPduNum++;
    pState->m_uOffset += uPayloadSize;

    GST_INFO("Defrag: pdu_num=%u, payload_size=%u, %u/%u.",
        pState->m_hdr.m_uPduNum-1, uPayloadSize,
        pState->m_uOffset, pState->m_hdr.m_uDataSize);

    if( pState->m_hdr.m_uPduNum == hdr.m_uPduTot )
    {
      pState->m_eStateId = GstOniPduStateIdEnd;
    }
  }

  // ---
  // End State
  // ---
  if( pState->m_eStateId == GstOniPduStateIdEnd )
  {
    switch( pState->m_hdr.m_eCompression )
    {
      case GST_ONI_PDU_COMPRESS_ZLIB:
        GST_DEBUG("Decompressing.");
        if( !oni_pdu_dec_uncompress_zlib(oni_pdu_dec) )
        {
          gst_oni_pdu_dec_state_flushing(*pState);
          return GST_FLOW_OK;
        }
        break;

      case GST_ONI_PDU_COMPRESS_NONE:
      default:
        break;
    }

    GST_BUFFER_TIMESTAMP(pState->m_pOutBuf)  = GST_BUFFER_TIMESTAMP(buf);
    GST_BUFFER_DURATION(pState->m_pOutBuf)   = GST_BUFFER_DURATION(buf);

    //gst_buffer_set_caps(pState->m_pOutBuf, GST_PAD_CAPS(oni_pdu_dec->srcpad));
    
    rc = gst_pad_push(oni_pdu_dec->srcpad, pState->m_pOutBuf);

    if( rc != GST_FLOW_OK )
    {
      gst_oni_pdu_state_reset(*pState);
      return rc;
    }

    gst_buffer_map(pState->m_pOutBuf, &info, GST_MAP_READ);
    uPayloadSize = info.size;
    gst_buffer_unmap(pState->m_pOutBuf, &info);

    GST_INFO("End: Sent frame buffer of %u bytes", uPayloadSize);

    pState->m_eStateId = GstOniPduStateIdStart;
    pState->m_uOffset  = 0;;
    pState->m_pOutBuf  = NULL;;
  }

  return GST_FLOW_OK;
}

#else // 0.10
static GstFlowReturn gst_oni_pdu_dec_chain(GstPad *pad, GstBuffer *buf)
{
  GstOniPduDec   *oni_pdu_dec;
  GstOniPduState *pState;
  guchar         *pInData;
  guint           uInSize;
  GstOniPduHdr    hdr;
  guchar         *pOutData;
  guint           uPayloadSize;
  GstFlowReturn   rc;

  oni_pdu_dec = GST_ONI_PDU_DEC(GST_OBJECT_PARENT(pad));

  pState = &oni_pdu_dec->m_state;

  pInData = GST_BUFFER_DATA(buf);
  uInSize = GST_BUFFER_SIZE(buf);

  GST_LOG("Received PDU buffer of %u bytes", uInSize);

  if( uInSize < GST_ONI_PDU_HDR_SIZE )
  {
    GST_ERROR("PDU buffer size %u < minimum size %u",
        uInSize, GST_ONI_PDU_HDR_SIZE);
    gst_oni_pdu_state_reset(*pState);
    return GST_FLOW_ERROR;
  }
  else if( gst_oni_pdu_unpack_hdr(pInData, hdr) != GST_ONI_PDU_OK )
  {
    GST_ERROR("Bad PDU header");
    gst_oni_pdu_state_reset(*pState);
    return GST_FLOW_ERROR;
  }

  // ---
  // Start of a frame.
  // ---
  if( hdr.m_uPduNum == 0 )
  {
    // reset to start state, freeing any allocated resources
    gst_oni_pdu_state_reset(*pState);

    // copy header
    pState->m_hdr       = hdr;

    // new output buffer
    pState->m_pOutBuf  = gst_buffer_new_and_alloc(pState->m_hdr.m_uDataSize);
    pState->m_uOffset  = 0;

    GST_INFO("Start: data_size=%u.", pState->m_hdr.m_uDataSize);

    pState->m_eStateId = GstOniPduStateIdDefrag;
  }

  // ---
  // Start or Flushing State
  // ---
  if( (pState->m_eStateId == GstOniPduStateIdStart) ||
      (pState->m_eStateId == GstOniPduStateIdFlushing) )
  {
    return GST_FLOW_OK;
  }

  // ---
  // Defragmentation State
  // ---
  if( pState->m_eStateId == GstOniPduStateIdDefrag )
  {
    uPayloadSize = uInSize - GST_ONI_PDU_HDR_SIZE;

    // lost or out-of-sequence PDU
    if( hdr.m_uPduNum != pState->m_hdr.m_uPduNum )
    {
      GST_WARNING("Expected PDU number %u, received PDU number %u.",
          pState->m_hdr.m_uPduNum, hdr.m_uPduNum);
      gst_oni_pdu_dec_state_flushing(*pState);
      return GST_FLOW_OK;
    }

    // too many PDUs
    else if( hdr.m_uPduNum >= pState->m_hdr.m_uPduTot )
    {
      GST_WARNING("Received PDU number %u >= expected total of %u.",
          hdr.m_uPduNum, pState->m_hdr.m_uPduTot);
      gst_oni_pdu_dec_state_flushing(*pState);
      return GST_FLOW_OK;
    }

    // received frame too big
    else if( pState->m_uOffset + uPayloadSize > pState->m_hdr.m_uDataSize )
    {
      GST_WARNING("Received PDU will exceed expected data size of %u.",
          pState->m_hdr.m_uDataSize);
      gst_oni_pdu_dec_state_flushing(*pState);
      return GST_FLOW_OK;
    }

    pOutData = GST_BUFFER_DATA(pState->m_pOutBuf);

    gst_oni_pdu_unpack_payload(pInData+GST_ONI_PDU_HDR_SIZE,
                               pOutData+pState->m_uOffset,
                               uPayloadSize);

    pState->m_hdr.m_uPduNum++;
    pState->m_uOffset += uPayloadSize;

    GST_INFO("Defrag: pdu_num=%u, payload_size=%u, %u/%u.",
        pState->m_hdr.m_uPduNum-1, uPayloadSize,
        pState->m_uOffset, pState->m_hdr.m_uDataSize);

    if( pState->m_hdr.m_uPduNum == hdr.m_uPduTot )
    {
      pState->m_eStateId = GstOniPduStateIdEnd;
    }
  }

  // ---
  // End State
  // ---
  if( pState->m_eStateId == GstOniPduStateIdEnd )
  {
    switch( pState->m_hdr.m_eCompression )
    {
      case GST_ONI_PDU_COMPRESS_ZLIB:
        GST_DEBUG("Decompressing.");
        if( !oni_pdu_dec_uncompress_zlib(oni_pdu_dec) )
        {
          gst_oni_pdu_dec_state_flushing(*pState);
          return GST_FLOW_OK;
        }
        break;

      case GST_ONI_PDU_COMPRESS_NONE:
      default:
        break;
    }

    GST_BUFFER_TIMESTAMP(pState->m_pOutBuf)  = GST_BUFFER_TIMESTAMP(buf);
    GST_BUFFER_DURATION(pState->m_pOutBuf)   = GST_BUFFER_DURATION(buf);

    gst_buffer_set_caps(pState->m_pOutBuf, GST_PAD_CAPS(oni_pdu_dec->srcpad));
    
    rc = gst_pad_push(oni_pdu_dec->srcpad, pState->m_pOutBuf);

    if( rc != GST_FLOW_OK )
    {
      gst_oni_pdu_state_reset(*pState);
      return rc;
    }

    GST_INFO("End: Sent frame buffer of %u bytes",
        GST_BUFFER_SIZE(pState->m_pOutBuf));

    pState->m_eStateId = GstOniPduStateIdStart;
    pState->m_uOffset  = 0;;
    pState->m_pOutBuf  = NULL;;
  }

  return GST_FLOW_OK;
}
#endif // EU_HAS_GST_1_0

static void gst_oni_pdu_dec_set_property(GObject      *object,
                                         guint         prop_id,
                                         const GValue *value,
                                         GParamSpec   *pspec)
{
  GstOniPduDec *oni_pdu_dec;

  g_return_if_fail(GST_IS_ONI_PDU_DEC(object));

  oni_pdu_dec = GST_ONI_PDU_DEC(object);
}

static void gst_oni_pdu_dec_get_property(GObject    *object,
                                         guint       prop_id,
                                         GValue     *value,
                                         GParamSpec *pspec)
{
  GstOniPduDec *oni_pdu_dec;

  g_return_if_fail(GST_IS_ONI_PDU_DEC(object));

  oni_pdu_dec = GST_ONI_PDU_DEC(object);

  return;
}

#ifndef EU_HAS_GST_1_0
static GstStateChangeReturn gst_oni_pdu_dec_change_state(GstElement *element,
                                                      GstStateChange transition)
{
  GstStateChangeReturn ret;
  GstOniPduDec *oni_pdu_dec;

  oni_pdu_dec = GST_ONI_PDU_DEC(element);

  // do any pre-transition work
  switch( transition )
  {
    case GST_STATE_CHANGE_READY_TO_PAUSED:
      gst_oni_pdu_state_reset(oni_pdu_dec->m_state);
      break;
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
      gst_oni_pdu_state_reset(oni_pdu_dec->m_state);
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
  GST_DEBUG_CATEGORY_INIT(oni_pdu_dec_debug, "onipdudec", 0,
      "OpenNI PDU Decoder");

  return gst_element_register(plugin,
                              "onipdudec",
                              GST_RANK_NONE,
                              GST_TYPE_ONI_PDU_DEC);
}

/*!
 * GStreamer looks for this structure to register this plug-in.
 */
GST_PLUGIN_DEFINE (
    GST_VERSION_MAJOR,
    GST_VERSION_MINOR,
    onipdudec,
    "OpenNI PDU decoder element",
    plugin_init,
    VERSION,
    EU_GST_PACKAGE_LICENSE,
    EU_GST_PACKAGE_NAME,
    EU_GST_PACKAGE_URL
)
