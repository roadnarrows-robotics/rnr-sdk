////////////////////////////////////////////////////////////////////////////////
//
// Package:     Eudoxus
//
// SubPackage:  GStreamer
//
// Plug-In:     libgstonisrc
//
// File:        gstoni.cxx
//
/*! \file
 *
 * $LastChangedDate: 2016-01-18 14:13:40 -0700 (Mon, 18 Jan 2016) $
 * $Rev: 4263 $
 *
 * \brief Eudoxus GStreamer OpenNI source element oni interface definitions.
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
 * @EulaEnd@
 */
//
////////////////////////////////////////////////////////////////////////////////

/****************************************************************************
*                                                                           *
*  OpenNI 1.x Alpha                                                         *
*  Copyright (C) 2011 PrimeSense Ltd.                                       *
*                                                                           *
*  This file is part of OpenNI.                                             *
*                                                                           *
*  OpenNI is free software: you can redistribute it and/or modify           *
*  it under the terms of the GNU Lesser General Public License as published *
*  by the Free Software Foundation, either version 3 of the License, or     *
*  (at your option) any later version.                                      *
*                                                                           *
*  OpenNI is distributed in the hope that it will be useful,                *
*  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
*  GNU Lesser General Public License for more details.                      *
*                                                                           *
*  You should have received a copy of the GNU Lesser General Public License *
*  along with OpenNI. If not, see <http://www.gnu.org/licenses/>.           *
*                                                                           *
****************************************************************************/

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <string>

#include <gst/gst.h>
#include <gst/base/gstpushsrc.h>

#include "XnTypes.h"
#include "XnOpenNI.h"
#include "XnLog.h"
#include "XnCodecIDs.h"
#include "XnCppWrapper.h"

#include "Eudoxus/Eudoxus.h"
#include "Eudoxus/euUtils.h"
#include "Eudoxus/euOni.h"
#include "Eudoxus/euPcd.h"
#include "Eudoxus/euPcs.h"

#include "gsteudoxus.h"

#include "gstonisrc.h"

using namespace std;
using namespace eu;


#define ONI_TRY_OK(rc, efmt, ...) \
  do \
  { \
    if( (rc) != XN_STATUS_OK ) \
    { \
      GST_ERROR("ONI: error=%d: " efmt, (rc), ##__VA_ARGS__); \
      return FALSE; \
    } \
  } \
  while(0)

#define ONI_TRY_NODE_PRESENT(rc, errors, efmt, ...) \
  do \
  { \
    if( rc == XN_STATUS_NO_NODE_PRESENT ) \
    { \
      XnChar strError[1024]; \
      errors.ToString(strError, 1024); \
      GST_ERROR("ONI: %s", strError); \
    } \
    ONI_TRY_OK(rc, efmt, ##__VA_ARGS__); \
  } \
  while(0)

static XnMapOutputMode  QVGAMode  = { 320,  240, 30};
static XnMapOutputMode  VGAMode   = { 640,  480, 30};
static XnMapOutputMode  SXGAMode  = {1280, 1024, 30};

static XnFieldOfView    FoVZero   = {0.0, 0.0};


// ----------------------------------------------------------------------------
// Oni Class
// ----------------------------------------------------------------------------

class Oni
{
public:
  // configuration and fixed properties
  XnMapOutputMode     m_xnDepthMode;        ///< depth resolution and fps
  XnMapOutputMode     m_xnImageMode;        ///< image resolution and fps
  XnBool              m_xnMirror;           ///< do [not] flip horizontally
  XnFieldOfView       m_xnDepthFoV;         ///< depth sensor field of view
  XnFieldOfView       m_xnImageFoV;         ///< image sensor field of view

  // OpenNI context and state
  xn::Context         m_xnContext;          ///< oni context
  XnBool              m_xnIsGenerating;     ///< [not] generating node data

  // OpenNi depth node objects
  xn::DepthGenerator  m_xnDepthGenerator;   ///< depth node generator
  guint               m_uDepthSize;         ///< grabbed sensor depth frame size
  const XnDepthPixel *m_pDepthMap;          ///< grabbed sensor depth map
  xn::DepthMetaData   m_depthMD;            ///< grabbed sensor depth metadata

  // OpenNi image node objects
  xn::ImageGenerator  m_xnImageGenerator;   ///< image node generator
  guint               m_uImageSize;         ///< grabbed sensor iamge frame size
  const XnRGB24Pixel *m_pImageMap;          ///< grabbed sensor image map
  xn::ImageMetaData   m_imageMD;            ///< grabbed sensor image metadata

  Oni()
  {
    int rc;

    m_xnDepthMode = QVGAMode;
    m_xnImageMode = QVGAMode;
    m_xnMirror    = FALSE;

    m_xnDepthFoV  = FoVZero;
    m_xnImageFoV  = FoVZero;

    if( (rc = m_xnContext.Init()) != XN_STATUS_OK )
    {
      GST_ERROR("ONI: error=%d: context.Init().", rc);
    }

    m_xnIsGenerating = FALSE;
  }

  ~Oni()
  {
    m_xnContext.StopGeneratingAll();
    m_xnImageGenerator.Release();
    m_xnDepthGenerator.Release();
    m_xnContext.Release();
  }

  gboolean CreateGenerators(guint uProdNodes)
  {
    xn::EnumerationErrors errors;
    XnStatus              rc = XN_STATUS_OK;

    //
    // Configure depth production node
    //
    if( uProdNodes & EuProdNodeTypeDepth )
    {
      rc = m_xnContext.CreateAnyProductionTree(XN_NODE_TYPE_DEPTH,
                                           NULL,
                                           m_xnDepthGenerator,
                                           &errors);

      ONI_TRY_NODE_PRESENT(rc, errors, "Failed to create depth node.");
    
      // Set Hole Filter
      m_xnDepthGenerator.SetIntProperty("HoleFilter", TRUE);
      
      // field of view parameters
      m_xnDepthGenerator.GetFieldOfView(m_xnDepthFoV);

      GST_INFO("Depth production node created.");
    }

    //
    // Configure image production node
    //
    if( uProdNodes & EuProdNodeTypeImage )
    {
      rc = m_xnContext.CreateAnyProductionTree(XN_NODE_TYPE_IMAGE,
                                                NULL,
                                                m_xnImageGenerator,
                                                &errors);
      ONI_TRY_NODE_PRESENT(rc, errors, "Failed to create image node.");

      // same field of view as depth sensor
      m_xnImageFoV = m_xnDepthFoV;

      GST_INFO("Image production node created.");
    }

    //
    // Synchronization configuration
    //
    if( (uProdNodes & EuProdNodeTypeDepth) &&
        (uProdNodes & EuProdNodeTypeImage) )
    {
      // registration
      if( m_xnDepthGenerator.IsCapabilitySupported(
                                        XN_CAPABILITY_ALTERNATIVE_VIEW_POINT) )
      {
        rc = m_xnDepthGenerator.GetAlternativeViewPointCap().SetViewPoint(
            m_xnImageGenerator);
        ONI_TRY_OK(rc, "Failed to set depth node registration.");
      }

      // frame Sync
      if( m_xnDepthGenerator.IsCapabilitySupported(XN_CAPABILITY_FRAME_SYNC))
      {
        if( m_xnDepthGenerator.GetFrameSyncCap().CanFrameSyncWith(
                                                        m_xnImageGenerator) )
        {
          rc = m_xnDepthGenerator.GetFrameSyncCap().FrameSyncWith(
                                                          m_xnImageGenerator);
          ONI_TRY_OK(rc,
              "Failed to set frame sync'ing depth and image generators.");
        }
      }
    }

    return TRUE;
  }

  gboolean ConfigureGenerators(guint uProdNodes)
  {
    xn::EnumerationErrors errors;
    XnStatus              rc = XN_STATUS_OK;

    //
    // Configure depth production node
    //
    if( uProdNodes & EuProdNodeTypeDepth )
    {
      rc = m_xnDepthGenerator.SetMapOutputMode(m_xnDepthMode);

      ONI_TRY_OK(rc, "Failed to set depth node configuration.");

      // do [not] mirror output
      if( m_xnDepthGenerator.IsCapabilitySupported(XN_CAPABILITY_MIRROR))
      {
        m_xnDepthGenerator.GetMirrorCap().SetMirror(m_xnMirror);
      }
    }

    //
    // Configure image production node
    //
    if( uProdNodes & EuProdNodeTypeImage )
    {
      rc = m_xnImageGenerator.SetMapOutputMode(m_xnImageMode);

      ONI_TRY_OK(rc, "Failed to set image node configuration.");

      if( m_xnImageGenerator.IsCapabilitySupported(XN_CAPABILITY_MIRROR))
      {
        m_xnImageGenerator.GetMirrorCap().SetMirror(m_xnMirror);
      }
    }

    return TRUE;
  }

  gboolean StartGeneratingAll()
  {
    XnStatus rc = XN_STATUS_OK;

    rc = m_xnContext.StartGeneratingAll();

    ONI_TRY_OK(rc, "Failed to start generating data from node(s).");

    m_xnIsGenerating = TRUE;

    return TRUE;
  }

  gboolean StopGeneratingAll()
  {
    XnStatus rc = XN_STATUS_OK;

    rc = m_xnContext.StopGeneratingAll();

    ONI_TRY_OK(rc, "Failed to stop generating data from node(s).");

    m_xnIsGenerating = FALSE;

    return TRUE;
  }

  void WaitAndUpdateAll()
  {
    m_xnContext.WaitAndUpdateAll();
  }
};


// ----------------------------------------------------------------------------
// Public Interface
// ----------------------------------------------------------------------------

void gst_oni_init(GstOniSrc *oni_src)
{
}

void gst_oni_deinit(GstOniSrc *oni_src)
{
}

gboolean gst_oni_set_caps(GstOniSrc *oni_src)
{
  Oni          *pOni = oni_src->m_pOni;
  EuResolution  resolution = oni_src->m_resolution;

  //
  // Only make changes if different from current parameters
  //
  if((pOni->m_xnDepthMode.nXRes != (XnUInt32)resolution.m_uWidth) ||
     (pOni->m_xnDepthMode.nYRes != (XnUInt32)resolution.m_uHeight) ||
      (pOni->m_xnMirror != (XnBool)oni_src->m_bMirror) )
  {
    pOni->m_xnDepthMode.nXRes = (XnUInt32)resolution.m_uWidth;
    pOni->m_xnDepthMode.nYRes = (XnUInt32)resolution.m_uHeight;
    pOni->m_xnImageMode.nXRes = (XnUInt32)resolution.m_uWidth;
    pOni->m_xnImageMode.nYRes = (XnUInt32)resolution.m_uHeight;
    pOni->m_xnMirror          = (XnBool)oni_src->m_bMirror;

    if( pOni->m_xnIsGenerating )
    {
      pOni->StopGeneratingAll();

      if( !pOni->ConfigureGenerators(oni_src->m_uProdNodes) )
      {
        return FALSE;
      }


      pOni->StartGeneratingAll();
    }
  }

  oni_src->m_pFramePcs->configure(oni_src->m_eMimeType,
                                  oni_src->m_uProdNodes,
                                  resolution,
                                  pOni->m_xnDepthFoV,
                                  pOni->m_xnImageFoV);

  return TRUE;
}

gboolean gst_oni_start(GstOniSrc *oni_src)
{
  Oni      *pOni = NULL;
  gboolean  rc   = TRUE;

  // old oni instance
  if( oni_src->m_pOni != NULL )
  {
    delete (Oni *)oni_src->m_pOni;
    oni_src->m_pOni = NULL;
  }

  pOni = new Oni();

  // create production node generators
  if( !pOni->CreateGenerators(oni_src->m_uProdNodes) )
  {
    rc = FALSE;
  }

  // configure generators
  else if( !pOni->ConfigureGenerators(oni_src->m_uProdNodes) )
  {
    rc = FALSE;
  }

  // start generators
  else if( !pOni->StartGeneratingAll() )
  {
    rc = FALSE;
  }

  // success
  if( rc == TRUE )
  {
    oni_src->m_pFramePcs->configure(oni_src->m_eMimeType,
                                    oni_src->m_uProdNodes,
                                    oni_src->m_resolution,
                                    pOni->m_xnDepthFoV,
                                    pOni->m_xnImageFoV);
    oni_src->m_pOni = pOni;
  }

  // error condition
  else
  {
    if( pOni != NULL )
    {
      delete pOni;
    }
  }

  return rc;
}

gboolean gst_oni_stop(GstOniSrc *oni_src)
{
  if( oni_src->m_pOni != NULL )
  {
    delete (Oni *)oni_src->m_pOni;
    oni_src->m_pOni = NULL;
  }
  return TRUE;
}

guint gst_oni_get_size(GstOniSrc *oni_src)
{
  return (guint)oni_src->m_pFramePcs->getMaxFrameSize();
}

GstFlowReturn gst_oni_grab_frame(GstOniSrc *oni_src, GstBuffer *bufGst)
{
  Oni            *pOni = oni_src->m_pOni;
  byte_t         *bufFrame;
  size_t          sizeFrame;
  ssize_t         n;
  GstFlowReturn   rc;

#ifdef EU_HAS_GST_1_0
  GstMapInfo      info;
#endif // EU_HAS_GST_1_0

  pOni->WaitAndUpdateAll();

  GST_LOG("Grabbed frame from sensor.");

#ifdef EU_HAS_GST_1_0
  gst_buffer_map(bufGst, &info, GST_MAP_READ);
  bufFrame = info.data;
  sizeFrame = info.size;
  gst_buffer_unmap(bufGst, &info);
#else // 0.10
  bufFrame  = (byte_t *)GST_BUFFER_DATA(bufGst);
  sizeFrame = (size_t)GST_BUFFER_SIZE(bufGst);
#endif // EU_HAS_GST_1_0

  n = oni_src->m_pFramePcs->insertFrame(pOni->m_xnDepthGenerator,
                                        pOni->m_xnImageGenerator,
                                        bufFrame,
                                        sizeFrame);

  if( n < 0 )
  {
    GST_ERROR("%s", oni_src->m_pFramePcs->getErrorMsg());
    return GST_FLOW_ERROR;
  }

  else
  {
#ifdef EU_HAS_GST_1_0
    gst_buffer_map(bufGst, &info, GST_MAP_WRITE);
    info.size = n;
    gst_buffer_unmap(bufGst, &info);
#else // 0.10
    GST_BUFFER_SIZE(bufGst) = (guint)n;
#endif // EU_HAS_GST_1_0
    return GST_FLOW_OK;
  }

  return rc;
}
