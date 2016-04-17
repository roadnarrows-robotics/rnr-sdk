////////////////////////////////////////////////////////////////////////////////
//
// Package:     Eudoxus
//
// SubPackage:  GStreamer
//
// Plug-Ins:    libgstonipduenc, libgstonipdudec
//
// File:        gstonipdu.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-11-20 15:52:58 -0700 (Fri, 20 Nov 2015) $
 * $Rev: 4213 $
 *
 * \brief Eudoxus GStreamer common OpenNI PDU file definitions.
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

#include <string.h>
#include <gst/gst.h>

#include "Eudoxus/Eudoxus.h"

#include "gsteudoxus.h"

#include "gstonipdu.h"

static inline void pack8(guint val, guchar *buf)
{
  buf[0] = (guchar)(val & 0xff);
}

static inline guint unpack8(guchar *buf)
{
  return (guint)buf[0];
}

static inline void pack16(guint val, guchar *buf)
{
  buf[0] = (guchar)((val >> 8) & 0xff);
  buf[1] = (guchar)(val & 0xff);
}

static inline guint unpack16(guchar *buf)
{
  return (guint)( ((guint)buf[0] << 8) | (guint)buf[1] );
}

static inline void pack32(guint val, guchar *buf)
{
  buf[0] = (guchar)((val >> 24) & 0xff);
  buf[1] = (guchar)((val >> 16) & 0xff);
  buf[2] = (guchar)((val >> 8) & 0xff);
  buf[3] = (guchar)(val & 0xff);
}

static inline guint unpack32(guchar *buf)
{
  return (guint)( ((guint)buf[0] << 24) |
                  ((guint)buf[1] << 16) |
                  ((guint)buf[2] << 8) |
                  (guint)buf[3] );
}

void gst_oni_pdu_state_defaults(GstOniPduState &state)
{
  state.m_eStateId      = GstOniPduStateIdStart;
  state.m_uOffset       = 0;
  state.m_pOutBuf       = NULL;

  gst_oni_pdu_hdr_init(state.m_hdr,
                      GST_ONI_PDU_COMPRESS_NONE,
                      0,
                      0);
}

void gst_oni_pdu_state_reset(GstOniPduState &state)
{
  if( state.m_pOutBuf != NULL )
  {
    gst_buffer_unref(state.m_pOutBuf);
    state.m_pOutBuf = NULL;
  }

  state.m_uOffset       = 0;
  state.m_eStateId      = GstOniPduStateIdStart;

  gst_oni_pdu_hdr_init(state.m_hdr,
                      GST_ONI_PDU_COMPRESS_NONE,
                      0,
                      0);
}

void gst_oni_pdu_hdr_init(GstOniPduHdr &hdr,
                          guint         eCompression,
                          guint         uFrameSize,
                          guint         uDataSize)
{
  hdr.m_eCompression  = eCompression;
  hdr.m_uPduNum       = 0;
  hdr.m_uPduTot       = gst_oni_pdu_total(uDataSize);
  hdr.m_uFrameSize    = uFrameSize;
  hdr.m_uDataSize     = uDataSize;
}

guint gst_oni_pdu_total(guint uDataSize)
{
  guint uPduTot;

  uPduTot = uDataSize / GST_ONI_PDU_MAX_PAYLOAD_SIZE;

  if( (uDataSize % GST_ONI_PDU_MAX_PAYLOAD_SIZE) != 0 )
  {
    ++uPduTot;
  }

  GST_DEBUG("Total PDUs==%u.\n", uPduTot);

  return uPduTot;
}

guint gst_oni_pdu_payload_size(guint uOffset, guint uDataSize)
{
  guint uBytes = uDataSize - uOffset;

  if( uBytes >= GST_ONI_PDU_MAX_PAYLOAD_SIZE )
  {
    return GST_ONI_PDU_MAX_PAYLOAD_SIZE;
  }
  else
  {
    GST_DEBUG("Final frame payload size=%u.\n", uBytes);
    return uBytes;
  }
}

void gst_oni_pdu_pack_hdr(GstOniPduHdr &hdr, guchar *pOutData)
{
  pack16(GST_ONI_PDU_START_CODE, pOutData+GST_ONI_PDU_IDX_START_CODE);
  pack8(0, pOutData+GST_ONI_PDU_IDX_RESERVED);
  pack8(hdr.m_eCompression, pOutData+GST_ONI_PDU_IDX_COMPRESS);
  pack32(hdr.m_uPduNum, pOutData+GST_ONI_PDU_IDX_PDU_NUM);
  pack32(hdr.m_uPduTot, pOutData+GST_ONI_PDU_IDX_PDU_TOT);
  pack32(hdr.m_uFrameSize, pOutData+GST_ONI_PDU_IDX_FRAME_SIZE);
  pack32(hdr.m_uDataSize, pOutData+GST_ONI_PDU_IDX_DATA_SIZE);
}

gint gst_oni_pdu_unpack_hdr(guchar *pInData, GstOniPduHdr &hdr)
{
  guint uStartCode;

  uStartCode = unpack16(pInData+GST_ONI_PDU_IDX_START_CODE);

  if( uStartCode != GST_ONI_PDU_START_CODE )
  {
    GST_ERROR("Received PDU start code 0x%04x, expected 0x%04x. Resync'ing",
        uStartCode, GST_ONI_PDU_START_CODE);
    return -GST_ONI_PDU_ECODE_START_CODE;
  }

  hdr.m_eCompression  = unpack8(pInData+GST_ONI_PDU_IDX_COMPRESS);
  hdr.m_uPduNum       = unpack32(pInData+GST_ONI_PDU_IDX_PDU_NUM);
  hdr.m_uPduTot       = unpack32(pInData+GST_ONI_PDU_IDX_PDU_TOT);
  hdr.m_uFrameSize    = unpack32(pInData+GST_ONI_PDU_IDX_FRAME_SIZE);
  hdr.m_uDataSize     = unpack32(pInData+GST_ONI_PDU_IDX_DATA_SIZE);

  return GST_ONI_PDU_OK;
}

char *gst_oni_pdu_sprintf_hdr(char buf[], size_t sizeBuf, GstOniPduHdr &hdr)
{
  snprintf(buf, sizeBuf,
      "PDU header: start_code=0x%04x, "
      "frame_compression=0x%02x, pdu_num=%u, pdu_tot=%u, frame_size=%u, "
      "data_size=%u.",
      GST_ONI_PDU_START_CODE, hdr.m_eCompression,
      hdr.m_uPduNum, hdr.m_uPduTot, hdr.m_uFrameSize, hdr.m_uDataSize);

  buf[sizeBuf-1] = 0;

  return buf;
}

void gst_oni_pdu_pack_payload(guchar *pInData,
                              guchar *pOutData,
                              guint   uPayloadSize)
{
  memcpy(pOutData, pInData, (size_t)uPayloadSize);
}

void gst_oni_pdu_unpack_payload(guchar *pInData,
                                guchar *pOutData,
                                guint   uPayloadSize)
{
  memcpy(pOutData, pInData, (size_t)uPayloadSize);
}
