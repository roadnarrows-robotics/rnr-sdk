/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: pcd_viewer.cpp 5094 2012-03-15 01:03:51Z rusu $
 *
 */

#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>

#include <cfloat>
#include <iostream>

#include <gst/gst.h>

#include <Eigen/Geometry>

// PCL
#include <boost/shared_ptr.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

// PCL
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/visualization/point_picking_event.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

// VTK
#include <vtkPolyDataReader.h>

// Eudoxus
#include "Eudoxus/Eudoxus.h"
#include "Eudoxus/euOni.h"
#include "Eudoxus/euPcd.h"
#include "Eudoxus/euPcs.h"

#include "gsteudoxus.h"

#include "gstpcsviewersink.h"


using namespace eu;
using namespace pcl::console;

typedef pcl::visualization::PointCloudColorHandler<pcl::PCLPointCloud2> ColorHandler;
typedef ColorHandler::Ptr ColorHandlerPtr;
typedef ColorHandler::ConstPtr ColorHandlerConstPtr;

typedef pcl::visualization::PointCloudGeometryHandler<pcl::PCLPointCloud2> GeometryHandler;
typedef GeometryHandler::Ptr GeometryHandlerPtr;
typedef GeometryHandler::ConstPtr GeometryHandlerConstPtr;

#define NORMALS_SCALE 0.01
#define PC_SCALE 0.001

// --------------------------------------------------------------------------
// Good Stuff
// --------------------------------------------------------------------------

static void unpackRGB(int nRgb, double rgb[])
{
  rgb[0] = (double)((nRgb & 0x00ff0000) >> 16) / 255.0;
  rgb[1] = (double)((nRgb & 0x0000ff00) >> 8) / 255.0;
  rgb[2] = (double)(nRgb & 0x000000ff) / 255.0;
}

static void unpackRGB(int nRgb, int rgb[])
{
  rgb[0] = (double)((nRgb & 0x00ff0000) >> 16);
  rgb[1] = (double)((nRgb & 0x0000ff00) >> 8);
  rgb[2] = (double)(nRgb & 0x000000ff);
}

// --------------------------------------------
// -----Open 3D viewer and add point cloud-----
// --------------------------------------------
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

// ----------------------------------------------------------------------------
// PCD Viewer Class
// ----------------------------------------------------------------------------
class PcdViewer
{
public:
  static const long T_SPIN    = 10;     ///< visualizer spin time (msec)
  static const long T_SLEEP   = 10000;  ///< visualizer thread wait time (usec)

  /*! background thread states */
  typedef enum {
    ViewerStateZombie,    ///< zombie instance - no thread exists
    ViewerStateReady,     ///< thread created and ready to run
    ViewerStateRunning,   ///< thread running
    ViewerStatePaused,    ///< thread paused
    ViewerStateExit       ///< thread exiting
  } ViewerState;

  //pcl::visualization::PCLHistogramVisualizer m_phGlobal;
  //boost::shared_ptr<pcl::visualization::PCLHistogramVisualizer> m_ph;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> m_pViz;
  //pcl::console::TicToc m_tt;
  ColorHandlerPtr m_color_handler;
  GeometryHandlerPtr m_geometry_handler;
  float m_min_p;
  float m_max_p;
  int   m_viewport;

  int   m_nBgRgb;
  int   m_nPointRgb;
  int   m_nPointSize;
  bool  m_bFirstCloud;

  PcdViewer()
  {
    m_min_p       = FLT_MAX;
    m_max_p       = -FLT_MAX;
    m_viewport    = 0;
    m_nBgRgb      = PCS_VIEWER_SINK_BG_DFT;
    m_nPointRgb   = PCS_VIEWER_SINK_PT_COLOR_DFT;
    m_nPointSize  = PCS_VIEWER_SINK_PT_SIZE_DFT;
    m_bFirstCloud = true;

    pthread_mutex_init(&m_mutexSync, NULL);
    pthread_cond_init(&m_condSync, NULL);

    m_eState = ViewerStateZombie;

    BgThreadCreate();
  }

  ~PcdViewer()
  {
    BgThreadExit();

    pthread_cond_destroy(&m_condSync);
    pthread_mutex_destroy(&m_mutexSync);
  }

  void start(int nBgRgb, int nPointRgb, int nPointSize)
  {
    double rgb[3];
    double normals_scale = NORMALS_SCALE;
    double pc_scale = PC_SCALE;

    m_nBgRgb      = nBgRgb;
    m_nPointRgb   = nPointRgb;
    m_nPointSize  = nPointSize;

    m_pViz.reset(new pcl::visualization::PCLVisualizer("Eudoxus PCS Viewer"));

    unpackRGB(m_nBgRgb, rgb);

    m_pViz->setBackgroundColor(rgb[0], rgb[1], rgb[2]);

    m_pViz->addCoordinateSystem(0.1);
    m_pViz->initCameraParameters();
  
    Eigen::Vector4f    origin      = Eigen::Vector4f::Zero();
    Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();
    Eigen::Matrix3f    rotation;

    rotation = orientation;

    // RDK Was m_pViz->setCameraPose(origin[0],
    m_pViz->setCameraPosition(origin[0],
                          origin[1],
                          origin[2],
                          origin[0] + rotation(0, 2),
                          origin[1] + rotation(1, 2),
                          origin[2] + rotation(2, 2),
                          rotation(0, 1),
                          rotation(1, 1),
                          rotation(2, 1));

    //pcl::getMinMax (*cloud, 0, cloud->fields[0].name, min_p, max_p);
    //ph->addFeatureHistogram (*cloud, cloud->fields[0].name, cloud_name.str ());
    //  pViewer->m_color_handler.reset(new pcl::visualization::PointCloudColorHandlerRandom<pcl::PCLPointCloud2>(cloud));

    // Add the dataset with a XYZ and a random handler
    //  pViewer->m_geometry_handler.reset(
    //      new pcl::visualization::PointCloudGeometryHandlerXYZ<pcl::PCLPointCloud2>(cloud));

    // run background thread
    run();
  }

  int run()
  {
    int   rc;   // return code

    switch( m_eState )
    {
      case ViewerStateReady:
        m_eState = ViewerStateRunning;
        pthread_cond_signal(&m_condSync);
        GST_LOG("PcdViewer background thread started.");
        rc = 0;
        break;
      case ViewerStateRunning:
        break;
      case ViewerStateZombie:
        rc = -1;
        GST_ERROR("No PcdViewer background thread.");
        break;
      case ViewerStatePaused:
      case ViewerStateExit:
      default:
        rc = -1;
        GST_ERROR("PcdViewer background thread in invalid state %d.", m_eState);
        break;
    }

    return rc;
  }

  int pause()
  {
    int   rc;

    lock();

    switch( m_eState )
    {
      case ViewerStateRunning:
        m_eState = ViewerStatePaused;
        pthread_cond_signal(&m_condSync);
        GST_LOG("PcdViewer background thread paused.");
        rc = 0;
        break;
      case ViewerStateZombie:
        rc = -1;
        GST_ERROR("No PcdViewer background thread.");
        break;
      case ViewerStateReady:
      case ViewerStatePaused:
      case ViewerStateExit:
      default:
        rc = -1;
        GST_ERROR("PcdViewer background thread in invalid state %d.", m_eState);
        break;
    } 

    unlock();

    return rc;
  }

  int resume()
  {
    int   rc;             // return code

    lock();

    switch( m_eState )
    {
      case ViewerStatePaused:
        m_eState = ViewerStateRunning;
        pthread_cond_signal(&m_condSync);
        GST_LOG("PcdViewer background thread resumed.");
        rc = 0;
        break;
      case ViewerStateZombie:
        rc = -1;
        GST_ERROR("No PcdViewer background thread.");
        break;
      case ViewerStateReady:
      case ViewerStateRunning:
      case ViewerStateExit:
      default:
        rc = -1;
        GST_ERROR("PcdViewer background thread in invalid state %d.", m_eState);
        break;
    }  

    unlock();

    return rc;
  }

  ViewerState getCurrentState()
  {
    return m_eState;
  }

  void setCloud(pcl::PCLPointCloud2::Ptr cloud,
                Eigen::Vector4f          &origin,
                Eigen::Quaternionf       &orientation)
  {
    bool    bUseRgbFldHandler;
    int     rgb[3];

    lock();

    if( m_bFirstCloud )
    {
      //m_pViz->registerPointPickingCallback(&pp_callback, (void*)&cloud);
      
      m_bFirstCloud = false;
    }

    else
    {
      m_pViz->removePointCloud("eudoxus_cloud");
    }

    m_geometry_handler.reset(
        new pcl::visualization::PointCloudGeometryHandlerXYZ
                                            <pcl::PCLPointCloud2>(cloud));

    //
    // point cloud color
    //
    bUseRgbFldHandler = false;

    if( m_nPointRgb == PCS_VIEWER_SINK_PT_COLOR_AUTO )
    {
      for(size_t i = 0; i < cloud->fields.size(); ++i)
      {
        if( cloud->fields[i].name == "rgb" || cloud->fields[i].name == "rgba")
        {
          m_color_handler.reset(
              new pcl::visualization::PointCloudColorHandlerRGBField
                                      <pcl::PCLPointCloud2>(cloud));
          bUseRgbFldHandler = true;
          break;
        }
      }
    }

    if( !bUseRgbFldHandler )
    {
      if( m_nPointRgb == PCS_VIEWER_SINK_PT_COLOR_AUTO )
      {
        unpackRGB(PCS_VIEWER_SINK_PT_COLOR_FIXED, rgb);
      }
      else
      {
        unpackRGB(m_nPointRgb, rgb);
      }

      m_color_handler.reset(
          new pcl::visualization::PointCloudColorHandlerCustom
                  <pcl::PCLPointCloud2>(cloud, rgb[0], rgb[1], rgb[2]));
    }

    // Add the cloud to the renderer
    m_pViz->addPointCloud(cloud,
                m_geometry_handler,
                m_color_handler,
                origin,
                orientation,
                "eudoxus_cloud", // cloud name
                m_viewport);

    m_pViz->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        m_nPointSize,
        "eudoxus_cloud");

    //m_phGlobal->setGlobalYRange(pViewer->m_min_p, pViewer->m_max_p);
    //m_phGlobal->updateWindowPositions();

    GST_INFO("Set new cloud.");

    signalViz();

    unlock();
  }

protected:
  // thread, state, and synchronization
  ViewerState     m_eState;       ///< thread state
  pthread_mutex_t m_mutexSync;    ///< synchonization mutex
  pthread_cond_t  m_condSync;     ///< synchonization condition
  pthread_t       m_thread;       ///< pthread identifier 

  void lock()
  {
    pthread_mutex_lock(&m_mutexSync);
  }

  void unlock()
  {
    pthread_mutex_unlock(&m_mutexSync);
  }

  void changeState(PcdViewer::ViewerState eNewState)
  {
    lock();
    m_eState = eNewState;
    pthread_cond_signal(&m_condSync);
    unlock();
    GST_LOG("PcdViewer background thread: New state: %d", m_eState);
  }

  void signalViz()
  {
    pthread_cond_signal(&m_condSync);
  }

  void readywait()
  {
    lock();

    while( (m_eState == ViewerStateReady) || (m_eState == ViewerStatePaused) )
    {
      pthread_cond_wait(&m_condSync, &m_mutexSync);
    }

    unlock();
  }

  void timewait(long lMicroSecs)
  {
    struct timeval  tvNow;
    struct timespec tsTimeout;
    long int        lSecs;

    // now
    gettimeofday(&tvNow, NULL);
  
    // future
    lMicroSecs  += tvNow.tv_usec;
    lSecs        = lMicroSecs / 1000000;
    lMicroSecs  -= (lSecs * 1000000);

    tsTimeout.tv_sec  = tvNow.tv_sec + lSecs;
    tsTimeout.tv_nsec = lMicroSecs * 1000;

    lock();

    // wait with timeout
    pthread_cond_timedwait(&m_condSync, &m_mutexSync, &tsTimeout);

    unlock();
  }

  void  BgThreadCreate()
  {
    int   rc;
    
    m_eState = ViewerStateReady;

    rc = pthread_create(&m_thread, NULL, PcdViewer::BgThread, (void *)this);
 
    if( rc != 0 )
    {
      GST_ERROR("pthread_create()");
      m_eState = ViewerStateZombie;
    }

    usleep(1000);
  }

  void  BgThreadExit()
  {
    changeState(ViewerStateExit);

    pthread_join(m_thread, NULL);
  }

  static void *BgThread(void *pArg)
  {
    PcdViewer *pThis = (PcdViewer *)pArg;
    int       rc;

    GST_INFO("PcdViewer background thread created.");

    pThis->readywait();

    //
    // Loop forever until exit.
    //
    while( pThis->m_eState != ViewerStateExit )
    {
      switch( pThis->m_eState )
      {
        case ViewerStateRunning:
          pThis->lock();
          pThis->m_pViz->spinOnce(T_SPIN);
          if( pThis->m_pViz->wasStopped() )
          {
            pThis->m_eState = ViewerStateExit;
          }
          pThis->unlock();
          pThis->timewait(T_SLEEP);
          break;
        case ViewerStateReady:
        case ViewerStatePaused:
          pThis->readywait();
          break;
        case ViewerStateExit:
          break;
        default:
          GST_ERROR("%d: Unexpected PcdViewer background thread state.",
              pThis->m_eState);
          pThis->m_eState = ViewerStateExit;
          break;
      }
    }

    pThis->m_eState = ViewerStateZombie;

    GST_INFO("PcdViewer background thread exited.");

    return NULL;
  }
};

// ----------------------------------------------------------------------------
// Public Interface
// ----------------------------------------------------------------------------

void gst_pcd_viewer_init(GstPcsViewerSink *pcs_viewer_sink)
{
}

void gst_pcd_viewer_deinit(GstPcsViewerSink *pcs_viewer_sink)
{
}

gboolean gst_pcd_viewer_set_caps(GstPcsViewerSink *pcs_viewer_sink)
{
  return TRUE;
}

gboolean gst_pcd_viewer_start(GstPcsViewerSink *pcs_viewer_sink)
{
  PcdViewer *pViewer;

  if( pcs_viewer_sink->m_pPcdViewer == NULL )
  {
    pViewer = new PcdViewer;
  }

  pcs_viewer_sink->m_pPcdViewer = pViewer;

  return TRUE;
}

gboolean gst_pcd_viewer_stop(GstPcsViewerSink *pcs_viewer_sink)
{
  if( pcs_viewer_sink->m_pPcdViewer != NULL )
  {
    delete (PcdViewer *)pcs_viewer_sink->m_pPcdViewer;
    pcs_viewer_sink->m_pPcdViewer = NULL;
  }

  return TRUE;
}

GstFlowReturn gst_pcd_viewer_render(GstPcsViewerSink *pcs_viewer_sink,
                                    guchar            bufFrame[],
                                    size_t            sizeFrame)
{
  EuPcsFrame                framePcs;
  pcl::PCLPointCloud2::Ptr  cloud;
  Eigen::Vector4f           origin;
  Eigen::Quaternionf        orientation;
  PcdViewer                *pViewer;
  ssize_t                   n;

  pViewer = (PcdViewer *)pcs_viewer_sink->m_pPcdViewer;

  switch( pViewer->getCurrentState() )
  {
    case PcdViewer::ViewerStateRunning:
      break;

    case PcdViewer::ViewerStateReady:
      pViewer->start(pcs_viewer_sink->m_nBgRgb,
                     pcs_viewer_sink->m_nPointRgb,
                     pcs_viewer_sink->m_nPointSize);
      break;

    case PcdViewer::ViewerStateZombie:
    case PcdViewer::ViewerStateExit:
      GST_ERROR("PcdViewer thread died.");
      return GST_FLOW_ERROR;

    case PcdViewer::ViewerStatePaused:
    default:
      break;
  }

  cloud.reset(new pcl::PCLPointCloud2);

  n = framePcs.extractFrameToPointCloud(bufFrame, sizeFrame,
                                        *cloud, origin, orientation);

  if( n < 0 )
  {
    GST_ERROR("%s", framePcs.getErrorMsg());
    return GST_FLOW_OK;
  }

  else if( (n == 0) || (framePcs.getDataSize() == 0) )
  {
    GST_WARNING("No data in frame.");
    return GST_FLOW_OK;
  }

  pViewer->setCloud(cloud, origin, orientation);

  return GST_FLOW_OK;
}
