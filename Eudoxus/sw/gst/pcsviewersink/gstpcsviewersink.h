////////////////////////////////////////////////////////////////////////////////
//
// Package:     Eudoxus
//
// SubPackage:  GStreamer
//
// Plug-In:     libgstpcsviewersink
//
// File:        gstpcsviewersink.h
//
/*! \file
 *
 * $LastChangedDate: 2015-12-03 17:11:37 -0700 (Thu, 03 Dec 2015) $
 * $Rev: 4233 $
 *
 * \brief Eudoxus GStreamer Point Cloud Stream viewer sink element plug-in
 * declarations.
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

#ifndef __GST_PCS_VIEWER_SINK_H__
#define __GST_PCS_VIEWER_SINK_H__

#include "Eudoxus/euConf.h"

#include <gst/gst.h>
#include <gst/base/gstbasesink.h>

#include "Eudoxus/Eudoxus.h"
#include "Eudoxus/euPcs.h"


G_BEGIN_DECLS

#define GST_TYPE_PCS_VIEWER_SINK (gst_pcs_viewer_sink_get_type())

#define GST_PCS_VIEWER_SINK(obj) \
  (G_TYPE_CHECK_INSTANCE_CAST((obj),GST_TYPE_PCS_VIEWER_SINK,GstPcsViewerSink))

#define GST_PCS_VIEWER_SINK_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_CAST((klass),GST_TYPE_PCS_VIEWER_SINK,GstPcsViewerSinkClass))

#define GST_IS_PCS_VIEWER_SINK(obj) \
  (G_TYPE_CHECK_INSTANCE_TYPE((obj),GST_TYPE_PCS_VIEWER_SINK))

#define GST_IS_PCS_VIEWER_SINK_CLASS(klass) \
  (G_TYPE_CHECK_CLASS_TYPE((klass),GST_TYPE_PCS_VIEWER_SINK))


typedef struct _GstPcsViewerSink GstPcsViewerSink;
typedef struct _GstPcsViewerSinkClass GstPcsViewerSinkClass;

GST_DEBUG_CATEGORY_EXTERN(pcs_viewer_sink_debug);
#define GST_CAT_DEFAULT pcs_viewer_sink_debug

//
// Limits and Defaults
//
#define PCS_VIEWER_SINK_RGB_MIN         0         ///< minimum RGB color
#define PCS_VIEWER_SINK_RGB_MAX         0xffffff  ///< maximum RGB color

#define PCS_VIEWER_SINK_BG_DFT          0         ///< background color default

#define PCS_VIEWER_SINK_PT_COLOR_AUTO   -1        ///< auto point color
#define PCS_VIEWER_SINK_PT_COLOR_FIXED  0x00ff00  ///< fixed point color default
#define PCS_VIEWER_SINK_PT_COLOR_DFT    PCS_VIEWER_SINK_PT_COLOR_AUTO
                                                  ///< point color default

#define PCS_VIEWER_SINK_PT_SIZE_MIN     1         ///< maximum point size 
#define PCS_VIEWER_SINK_PT_SIZE_MAX     25        ///< maximum point size 
#define PCS_VIEWER_SINK_PT_SIZE_DFT     1         ///< point size default


struct _GstPcsViewerSink
{
  GstBaseSinkClass parent;          ///< parent structure

  // properties
  gint              m_nBgRgb;       ///< background viewer RGB color
  gint              m_nPointRgb;    ///< point RGB color
  gint              m_nPointSize;   ///< point size

   // state
  bool              m_bFirstCloud;  ///< [not] the first cloud
  void             *m_pPcdViewer;   ///< Point Cloud Data viewer class object
};

struct _GstPcsViewerSinkClass
{
  GstBaseSinkClass parent_class;
};

GType gst_pcs_viewer_sink_get_type(void);

G_END_DECLS


//
// Prototypes
//
extern void gst_pcd_viewer_init(GstPcsViewerSink *pcs_viewer_sink);

extern void gst_pcd_viewer_deinit(GstPcsViewerSink *pcs_viewer_sink);

extern gboolean gst_pcd_viewer_set_caps(GstPcsViewerSink *pcs_viewer_sink);

extern gboolean gst_pcd_viewer_start(GstPcsViewerSink *pcs_viewer_sink);

extern gboolean gst_pcd_viewer_stop(GstPcsViewerSink *pcs_viewer_sink);

extern GstFlowReturn gst_pcd_viewer_render(GstPcsViewerSink *pcs_viewer_sink,
                                           guchar            frame[],
                                           size_t            sizeFrame);


#endif /* __GST_PCS_VIEWER_SINK_H__ */
