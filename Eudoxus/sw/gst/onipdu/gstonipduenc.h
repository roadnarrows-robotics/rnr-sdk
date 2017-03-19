////////////////////////////////////////////////////////////////////////////////
//
// Package:     Eudoxus
//
// SubPackage:  GStreamer
//
// Plug-In:     libgstonipduenc
//
// File:        gstonipduenc.h
//
/*! \file
 *
 * $LastChangedDate: 2015-11-20 15:52:58 -0700 (Fri, 20 Nov 2015) $
 * $Rev: 4213 $
 *
 * \brief Eudoxus GStreamer OpenNI Protocol Data Unit element plug-in
 * declarations.
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

#ifndef __GST_ONI_PDU_ENC_H__
#define __GST_ONI_PDU_ENC_H__


#include <gst/gst.h>

#include "gstonipdu.h"

G_BEGIN_DECLS

#define GST_TYPE_ONI_PDU_ENC \
  (gst_oni_pdu_enc_get_type())

#define GST_ONI_PDU_ENC(obj) \
  (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_ONI_PDU_ENC,GstOniPduEnc))

#define GST_ONI_PDU_ENC_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_ONI_PDU_ENC,GstOniPduEncClass))

#define GST_IS_ONI_PDU_ENC(obj) \
  (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_ONI_PDU_ENC))

#define GST_IS_ONI_PDU_ENC_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_ONI_PDU_ENC))


typedef struct _GstOniPduEnc GstOniPduEnc;
typedef struct _GstOniPduEncClass GstOniPduEncClass;

GST_DEBUG_CATEGORY_EXTERN(oni_pdu_enc_debug);
#define GST_CAT_DEFAULT oni_pdu_enc_debug

struct _GstOniPduEnc
{
  GstElement element;

  // pads
  GstPad   *sinkpad;
  GstPad   *srcpad;

  // properties
  guint           m_eCompression;   ///< compression algorithm property
  
  // state
  GstOniPduState  m_state;
};

struct _GstOniPduEncClass
{
  GstElementClass parent_class;
};

GType gst_oni_pdu_enc_get_type(void);

G_END_DECLS

#endif /* __GST_ONI_PDU_ENC_H__ */
