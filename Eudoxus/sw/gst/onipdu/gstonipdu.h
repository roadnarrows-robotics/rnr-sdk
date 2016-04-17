////////////////////////////////////////////////////////////////////////////////
//
// Package:     Eudoxus
//
// SubPackage:  GStreamer
//
// Plug-Ins:    libgstonipduenc, libgstonipdudec
//
// File:        gstonipdu.h
//
/*! \file
 *
 * $LastChangedDate: 2015-11-20 15:52:58 -0700 (Fri, 20 Nov 2015) $
 * $Rev: 4213 $
 *
 * \brief Eudoxus GStreamer common OpenNI PDU file.
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

#ifndef _GST_ONI_PDU_H
#define _GST_ONI_PDU_H

#include <gst/gst.h>

#include "Eudoxus/Eudoxus.h"

#include "gsteudoxus.h"


/*!
 * \brief OpenNi Stream PDU start code.
 */
#define GST_ONI_PDU_IDX_START_CODE  0         ///< buffer index
#define GST_ONI_PDU_LEN_START_CODE  2         ///< field length
#define GST_ONI_PDU_START_CODE      0xa1b2    ///< start code

/*!
 * \brief Reserved byte.
 */
#define GST_ONI_PDU_IDX_RESERVED    2         ///< buffer index
#define GST_ONI_PDU_LEN_RESERVED    1         ///< field length

/*!
 * \brief Data payload compression algorithm.
 */
#define GST_ONI_PDU_IDX_COMPRESS    3         ///< buffer index
#define GST_ONI_PDU_LEN_COMPRESS    1         ///< field length
#define GST_ONI_PDU_COMPRESS_NONE   0         ///< no compression
#define GST_ONI_PDU_COMPRESS_ZLIB   1         ///< zlib compression

/*!
 * \brief PDU number.
 */
#define GST_ONI_PDU_IDX_PDU_NUM     4         ///< buffer index
#define GST_ONI_PDU_LEN_PDU_NUM     4         ///< field length

/*!
 * \brief PDU total number for application frame.
 */
#define GST_ONI_PDU_IDX_PDU_TOT     8         ///< buffer index
#define GST_ONI_PDU_LEN_PDU_TOT     4         ///< field length

/*!
 * \brief Total size of application frame.
 */
#define GST_ONI_PDU_IDX_FRAME_SIZE  12        ///< buffer index
#define GST_ONI_PDU_LEN_FRAME_SIZE  4         ///< field length

/*!
 * \brief Total size of (compressed) application frame.
 */
#define GST_ONI_PDU_IDX_DATA_SIZE   16        ///< buffer index
#define GST_ONI_PDU_LEN_DATA_SIZE   4         ///< field length

/*!
 * \brief Start of payload data.
 */
#define GST_ONI_PDU_IDX_DATA        20        ///< buffer index

/*!
 * \brief PDU header size.
 */
#define GST_ONI_PDU_HDR_SIZE \
  (GST_ONI_PDU_LEN_START_CODE + \
   GST_ONI_PDU_LEN_RESERVED + \
   GST_ONI_PDU_LEN_COMPRESS + \
   GST_ONI_PDU_LEN_PDU_NUM + \
   GST_ONI_PDU_LEN_PDU_TOT + \
   GST_ONI_PDU_LEN_FRAME_SIZE + \
   GST_ONI_PDU_LEN_DATA_SIZE)

/*!
 * \brief Payload size.
 *
 * Must be smaller than the UDP max size of:
 *  65,507 = 2^16 - 1 - 8 byte UDP header - 20 byte IP header.
 */
#define GST_ONI_PDU_MAX_PAYLOAD_SIZE  65000

/*!
 * \brief Return codes.
 */
#define GST_ONI_PDU_OK                0   ///< no error
#define GST_ONI_PDU_ECODE_START_CODE  1   ///< bad PDU start code
#define GST_ONI_PDU_ECODE_PDU_SEQ     2   ///< PDU out of sequence or missing
#define GST_ONI_PDU_ECODE_FRAME_SIZE  3   ///< bad frame size

typedef struct
{
  guint           m_eCompression;   ///< compression algorithm
  guint32         m_uPduNum;        ///< PDU number
  guint32         m_uPduTot;        ///< total number of PDUs for this frame
  guint           m_uFrameSize;     ///< frame size in bytes
  guint           m_uDataSize;      ///< (compressed) frame size in bytes
} GstOniPduHdr;

typedef enum
{
  GstOniPduStateIdStart,
  GstOniPduStateIdFrag,
  GstOniPduStateIdDefrag,
  GstOniPduStateIdFlushing,
  GstOniPduStateIdEnd,
} GstOniPduStateId;

typedef struct
{
  GstOniPduStateId  m_eStateId;       ///< state id
  guint             m_uOffset;        ///< current frame data offset
  GstBuffer        *m_pOutBuf;        ///< ouput PDU or frame buffer
  GstOniPduHdr      m_hdr;            ///< PDU header
} GstOniPduState;


extern void gst_oni_pdu_state_defaults(GstOniPduState &state);

extern void gst_oni_pdu_state_reset(GstOniPduState &state);

extern void gst_oni_pdu_hdr_init(GstOniPduHdr  &hdr,
                                 guint         eCompression,
                                 guint         uFrameSize,
                                 guint         uDataSize);

extern guint gst_oni_pdu_total(guint uDataSize);

extern guint gst_oni_pdu_payload_size(guint32 uOffset, guint uDataSize);

extern void gst_oni_pdu_pack_hdr(GstOniPduHdr &hdr, guchar *pOutData);

extern gint gst_oni_pdu_unpack_hdr(guchar *pInData, GstOniPduHdr &hdr);

extern char *gst_oni_pdu_sprintf_hdr(char buf[],
                                     size_t sizeBuf,
                                     GstOniPduHdr &hdr);

extern void gst_oni_pdu_pack_payload(guchar *pInData,
                                     guchar *pOutData,
                                     guint   uPayloadSize);

extern void gst_oni_pdu_unpack_payload(guchar *pInData,
                                       guchar *pOutData,
                                       guint   uPayloadSize);

#endif // _GST_ONI_PDU_H
