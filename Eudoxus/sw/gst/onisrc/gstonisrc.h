////////////////////////////////////////////////////////////////////////////////
//
// Package:     Eudoxus
//
// SubPackage:  GStreamer
//
// Plug-In:     libgstonisrc
//
// File:        gstonisrc.h
//
/*! \file
 *
 * $LastChangedDate: 2016-01-18 14:13:40 -0700 (Mon, 18 Jan 2016) $
 * $Rev: 4263 $
 *
 * \brief Eudoxus GStreamer OpenNI source element plug-in declarations.
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

#ifndef __GST_ONI_SRC_H__
#define __GST_ONI_SRC_H__

#include <gst/gst.h>
#include <gst/base/gstpushsrc.h>

#include "Eudoxus/Eudoxus.h"
#include "Eudoxus/euPcs.h"

G_BEGIN_DECLS

#define GST_TYPE_ONI_SRC  (gst_oni_src_get_type())

#define GST_ONI_SRC(obj) \
  (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_ONI_SRC,GstOniSrc))

#define GST_ONI_SRC_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_ONI_SRC,GstOniSrcClass))

#define GST_IS_ONI_SRC(obj) \
  (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_ONI_SRC))

#define GST_IS_ONI_SRC_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_ONI_SRC))

typedef struct _GstOniSrc      GstOniSrc;
typedef struct _GstOniSrcClass GstOniSrcClass;

GST_DEBUG_CATEGORY_EXTERN(oni_src_debug);
#define GST_CAT_DEFAULT oni_src_debug

//
// Forward declarations
//
class Oni;


/*!
 * \brief GStreamer OpenNI source element structure.
 */
struct _GstOniSrc
{
  GstPushSrc        element;        ///< parent structure

  // Properties
  guint             m_uProdNodes;   ///< bit list of production nodes

  // Capabilities
  eu::EuMimeType    m_eMimeType;    ///< source mimetype (format)
  eu::EuResolution  m_resolution;   ///< depth/image capture point/pixel res.
  gboolean          m_bMirror;      ///< [don't] flip horizontally
  gint              m_nFrameRateN;  ///< target frame rate numerator (frames)
  gint              m_nFrameRateD;  ///< target frame rate denomerator (seconds)

  // State
  eu::EuPcsFrame   *m_pFramePcs;    ///< PCS frame
  GstClockTime      m_clockRunning; ///< total running time 
  Oni              *m_pOni;         ///< oni class object
};

struct _GstOniSrcClass 
{
  GstPushSrcClass parent_class;
};

GType gst_oni_src_get_type(void);

G_END_DECLS

//
// Prototypes
//
extern void gst_oni_init(GstOniSrc *oni_src);

extern void gst_oni_deinit(GstOniSrc *oni_src);

extern gboolean gst_oni_set_caps(GstOniSrc *oni_src);

extern gboolean gst_oni_start(GstOniSrc *oni_src);

extern gboolean gst_oni_stop(GstOniSrc *oni_src);

extern guint gst_oni_get_size(GstOniSrc *oni_src);

extern GstFlowReturn gst_oni_grab_frame(GstOniSrc *oni_src,
                                        GstBuffer *bufFrame);


#endif /* __GST_ONI_SRC_H__ */
