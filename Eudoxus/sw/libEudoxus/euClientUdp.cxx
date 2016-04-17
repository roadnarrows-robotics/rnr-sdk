////////////////////////////////////////////////////////////////////////////////
//
// Package:   Eudoxus
//
// Library:   libEudoxus
//
// File:      euClientUdp.cxx
//
/*! \file
 *
 * $LastChangedDate: 2016-01-18 14:13:40 -0700 (Mon, 18 Jan 2016) $
 * $Rev: 4263 $
 *
 * \brief Eudoxus Gstreamer over UDP client class definities.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2012-2016.  RoadNarrows LLC
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
// IN NO EVENT SHALL THE AUTHOR, COMEDTEC, OR ANY MEMBERS/EMPLOYEES/CONTRACTORS
// OF COMEDTEC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
// PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
// EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.
//
// THE AUTHORS AND  COMEDTEC SPECIFICALLY DISCLAIM ANY WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
// "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
// PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
//
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#include "Eudoxus/euConf.h"

#ifdef EU_HAS_PCL
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>

#include <cfloat>
#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include <gst/gst.h>

#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>

#include "Eudoxus/Eudoxus.h"
#include "Eudoxus/euPcs.h"
#include "Eudoxus/euClientUdp.h"

using namespace std;
using namespace pcl;
using namespace eu;

namespace eu
{
  //----------------------------------------------------------------------------
  // GstBuffer Ring Buffer Class
  //----------------------------------------------------------------------------

  /*!
   * \brief GStreamer ring buffer class. 
   */
  class GstRingBuf
  {
  public:
    /*! Ring buffer structure */
    typedef struct
    {
      byte_t  *m_buf;     ///< buffer data
      size_t   m_size;    ///< buffer size
    } Ring_T;

    /*!
     * \brief Default initialization constructor.
     *
     * \param nNumBuffers   Number of ring buffers
     */
    GstRingBuf(const int nNumBuffers=OniGstClientUdp::UdpPortDft)
    {
      m_nNumBuffers = nNumBuffers >= 2? nNumBuffers: 2;
      m_nWriteIdx   = 0;
      m_nReadIdx    = 0;
  
      pthread_mutex_init(&m_mutexSync, NULL);
      pthread_cond_init(&m_condSync, NULL);
  
      m_pRing = new Ring_T[m_nNumBuffers];

      for(int i=0; i<m_nNumBuffers; ++i)
      {
        m_pRing[i].m_buf  = NULL;
        m_pRing[i].m_size = 0;
      }
    }
  
    /*!
     * \brief Destructor.
     */
    ~GstRingBuf()
    {
      for(int i=0; i<m_nNumBuffers; ++i)
      {
        if( m_pRing[i].m_buf != NULL )
        {
          delete[] m_pRing[i].m_buf;
        }
      }
  
      delete[] m_pRing;
  
      pthread_cond_destroy(&m_condSync);
      pthread_mutex_destroy(&m_mutexSync);
    }
  
    /*!
     * \brief Write new GstBuffer to ring buffer.
     *
     * Any existing buffer at the write position will be overwritten.
     *
     * \param buffer  GStreamer allocated GstBuffer.
     */
    void write(GstBuffer *buffer)
    {
      GstMapInfo  info;

      if( buffer == NULL )
      {
        return;
      }

      gst_buffer_map(buffer, &info, GST_MAP_READ);

      if( (info.data == NULL) || (info.size == 0) )
      {
        gst_buffer_unmap(buffer, &info);
        return;
      }

      lock(); // mutex loc
  
      // delete old buffer in ring
      if( m_pRing[m_nWriteIdx].m_buf != NULL )
      {
        delete[] m_pRing[m_nWriteIdx].m_buf;
        m_pRing[m_nWriteIdx].m_buf  = NULL;
        m_pRing[m_nWriteIdx].m_size = 0;
      }
  
      // new buffer in ring
      m_pRing[m_nWriteIdx].m_size = (size_t)info.size;
      m_pRing[m_nWriteIdx].m_buf  = new byte_t[m_pRing[m_nWriteIdx].m_size];

      // save
      memcpy(m_pRing[m_nWriteIdx].m_buf,
              info.data,
              m_pRing[m_nWriteIdx].m_size);
  
      gst_buffer_unmap(buffer, &info);

      //fprintf(stderr, "DBG: write buffer: ring[%d]=%p(%zu)\n",
      //  m_nWriteIdx, m_pRing[m_nWriteIdx].m_buf, m_pRing[m_nWriteIdx].m_size);
             
      // advance write index
      m_nWriteIdx = (m_nWriteIdx + 1) % m_nNumBuffers;
  
      // push read index
      if( m_nWriteIdx == m_nReadIdx )
      {
        m_nReadIdx = (m_nReadIdx + 1) % m_nNumBuffers;
      }
  
      // signal any blocked read thread
      pthread_cond_signal(&m_condSync);
  
      // unlock
      unlock();
    }
  
    /*!
     * \brief Read GstBuffer from ring buffer.
     *
     * \param fnCpBuf   User-supplied copy buffer function.
     * \param lMilliSec Block wait time in milliseconds. If 0, then block
     *                  forever until buffer available.
     *
     * \copydoc doc_return_std
     */
    int read(int (*fnCpBuf)(byte_t *, size_t), long lMilliSec=0)
    {
      int   rc;
  
      if( (rc = readBegin(lMilliSec)) == EU_OK )
      {
        rc = fnCpBuf(m_pRing[m_nReadIdx].m_buf, m_pRing[m_nReadIdx].m_size);
        readEnd();
      }
  
      return rc;
    }
  
    /*!
     * \brief Read GstBuffer from ring buffer and convert to PointCloud.
     *
     * \param [out] cloud       Output point cloud.
     * \param [out] origin      Sensor origin.
     * \param [out] orientation Sensor orientation.
     * \param lMilliSec         Block wait time in milliseconds. If 0, then
     *                          block forever until buffer available.
     *
     * \copydoc doc_return_std
     */
    int read(PCLPointCloud2     &cloud,
             Eigen::Vector4f    &origin,
             Eigen::Quaternionf &orientation,
             long               lMilliSec=0)
    {
      EuPcsFrame  pcsFrame;
      int         rc;
      ssize_t     n;
  
      if( (rc = readBegin(lMilliSec)) == EU_OK )
      {
        n = pcsFrame.extractFrameToPointCloud(m_pRing[m_nReadIdx].m_buf,
                                              m_pRing[m_nReadIdx].m_size,
                                              cloud,
                                              origin,
                                              orientation);
        readEnd();

        if( n < 0 )
        {
          rc = (int)n;
        }
      }
  
      return rc;
    }
  
  protected:
    pthread_mutex_t m_mutexSync;    ///< synchronization mutex 
    pthread_cond_t  m_condSync;     ///< synchronization condition variable
  
    int m_nNumBuffers;              ///< number of buffers in ring
    int m_nWriteIdx;                ///< write index
    int m_nReadIdx;                 ///< read index
  
    Ring_T *m_pRing;                ///< the ring of buffer
  
    /*!
     * \brief Lock.
     */
    void lock()
    {
      pthread_mutex_lock(&m_mutexSync);
    }
  
    /*!
     * \brief Unlock.
     */
    void unlock()
    {
      pthread_mutex_unlock(&m_mutexSync);
    }
  
    /*!
     * \brief Begin read.
     *
     * \param lMilliSec     Block wait time in milliseconds. If 0, then block
     *                      forever until buffer available.
     *
     * \copydoc doc_return_std
     */
    int readBegin(long lMilliSec)
    {
      struct timespec tsTimeout;
      int             rc;
      
      // lock
      lock();
  
      //
      // Empty, so block.
      //
      if( m_nReadIdx == m_nWriteIdx )
      {
        // Block with timeout until buffer is available to read.
        if( lMilliSec > 0 )
        {
          tsTimeout.tv_sec = lMilliSec / 1000;
          tsTimeout.tv_nsec = (lMilliSec - tsTimeout.tv_sec * 1000) * 1000000;
          pthread_cond_timedwait(&m_condSync, &m_mutexSync, &tsTimeout);
  
        }
        // Block indefinitely until buffer is available to read.
        else
        {
          pthread_cond_wait(&m_condSync, &m_mutexSync);
        }
      }
  
      // still empty
      if( m_nReadIdx == m_nWriteIdx )
      {
        rc = lMilliSec > 0? -EU_ECODE_TIMEDOUT: -EU_ECODE_NO_RSRC;
        unlock();
      }

      // got a frame
      else
      {
        rc = EU_OK;
      }
  
      return rc;
    }
  
    /*!
     * \brief End read.
     */
    void readEnd()
    {
      m_nReadIdx = (m_nReadIdx + 1) % m_nNumBuffers;
      unlock();
    }
  };
}


//------------------------------------------------------------------------------
// Eudoxus OpenNI GStreamer UDP client termination class.
//------------------------------------------------------------------------------

OniGstClientUdp::OniGstClientUdp(const int nPort, const int nNumBuffers)
{
  m_nPort           = nPort;

  m_bIsStreaming    = false;
  m_bFatal          = false;
  m_nSignalId       = 0;

  m_pRingBuf = new GstRingBuf(nNumBuffers);

  makePipeline();
}

OniGstClientUdp::~OniGstClientUdp()
{
  stop();
  destroyPipeline();
  delete m_pRingBuf;
}

int OniGstClientUdp::start()
{
  // already streaming
  if( isStreaming() )
  {
    LOGDIAG3("Streaming already started.");
  }

  // start streaming pipeline
  else
  {
    startPipeline();
  }

  LOGDIAG3("Streaming OpenNI frames.");

  return EU_OK;
}


int OniGstClientUdp::stop()
{
  if( !isStreaming() )
  {
    LOGDIAG3("Streaming already stoped.");
    return EU_OK;
  }

  stopPipeline();

  LOGDIAG3("OpenNi streaming stopped.");

  return EU_OK;
}

int OniGstClientUdp::grabPointCloudFrame(int (*fnCpBuf)(byte_t *, size_t),
                                         long lMilliSec)
{
  if( !isStreaming() )
  {
    LOGERROR("No frame streaming running.");
    return -EU_ECODE_NO_EXEC;
  }

  // grab a frame 
  else 
  {
    return m_pRingBuf->read(fnCpBuf, lMilliSec);
  }
}

int OniGstClientUdp::grabPointCloudFrame(PCLPointCloud2     &cloud,
                                         Eigen::Vector4f    &origin,
                                         Eigen::Quaternionf &orientation,
                                         long                lMilliSec)
{
  if( !isStreaming() )
  {
    LOGERROR("No frame streaming running.");
    return -EU_ECODE_NO_EXEC;
  }

  // grab a frame 
  else 
  {
    return m_pRingBuf->read(cloud, origin, orientation, lMilliSec);
  }
}

void OniGstClientUdp::frameCb(GstElement *element,
                              GstBuffer  *buffer,
                              GstPad     *pad,
                              void       *user_data)
{
  OniGstClientUdp *pClient = (OniGstClientUdp *)user_data;

  pClient->m_pRingBuf->write(buffer);
}

void OniGstClientUdp::startPipeline()
{
  if( !m_bIsStreaming )
  {
    gst_element_set_state(m_pPipeline, GST_STATE_PLAYING);

    // install frame grab callback
    m_nSignalId = g_signal_connect(G_OBJECT(m_pElemFakeSink),
                                   "handoff",
                                   G_CALLBACK(frameCb),
                                   this);

    LOGDIAG3("Started pipeline.");

    m_bIsStreaming = true;
  }
}

void OniGstClientUdp::stopPipeline()
{
  if( m_bIsStreaming )
  {
    gst_element_set_state(m_pPipeline, GST_STATE_NULL);

    if( m_nSignalId != 0 )
    {
      g_signal_handler_disconnect(G_OBJECT(m_pElemFakeSink), m_nSignalId);
      m_nSignalId = 0;
    }

    LOGDIAG3("Stopped pipeline.");

    m_bIsStreaming = false;
  }
}

int OniGstClientUdp::makePipeline()
{
  const char *sFactoryName;
  GstBus     *bus;
  gboolean    ok;
  int         rc;

  m_pPipeline       = NULL;
  m_pElemUdpSrc     = NULL;
  m_pElemOniPduDec  = NULL;
  m_pElemFakeSink   = NULL;

  // initialize gstreamer (required)
  gst_init(NULL, NULL);

  //
  // GStreamer pipeline
  //
  m_pPipeline = gst_pipeline_new("pipeline");

  if( m_pPipeline == NULL )
  {
    LOGERROR("Failed to create new gstreamer pipeline.");
    m_bFatal = true;
    return -EU_ECODE_GST;
  }

  //
  // Source element
  //
  sFactoryName  = "udpsrc";
  m_pElemUdpSrc = gst_element_factory_make(sFactoryName, "udp_src"); 

  if( m_pElemUdpSrc == NULL )
  {
    LOGERROR("Failed to create gstreamer element: %s.", sFactoryName);
    m_bFatal = true;
    return -EU_ECODE_GST;
  }

  // set properties
  g_object_set(G_OBJECT(m_pElemUdpSrc),
      "port", m_nPort,
      NULL);

  //
  // OpenNI PDU decoder element
  //
  sFactoryName      = "onipdudec";
  m_pElemOniPduDec  = gst_element_factory_make(sFactoryName, "oni_pdu_dec"); 

  if( m_pElemOniPduDec == NULL )
  {
    LOGERROR("Failed to create gstreamer element: %s.", sFactoryName);
    m_bFatal = true;
    return -EU_ECODE_GST;
  }

  //
  // Fack sink element which supports a callback mechanism to provide a
  // 'non' gstreamer application interface.
  //
  sFactoryName    = "fakesink";
  m_pElemFakeSink = gst_element_factory_make(sFactoryName, "fake_sink");
  
  if( m_pElemFakeSink == NULL )
  {
    LOGERROR("Failed to create gstreamer element: %s.", sFactoryName);
    m_bFatal = true;
    return -EU_ECODE_GST;
  }

  // set properties
  g_object_set(G_OBJECT(m_pElemFakeSink),
      "signal-handoffs", TRUE,
      NULL);

  //
  // Add elements to pipeline and link elements
  //
  gst_bin_add_many(GST_BIN(m_pPipeline),
                    m_pElemUdpSrc,
                    m_pElemOniPduDec,
                    m_pElemFakeSink,
                    NULL);

  ok = gst_element_link_many(
                    m_pElemUdpSrc,
                    m_pElemOniPduDec,
                    m_pElemFakeSink,
                    NULL);

  if( !ok )
  {
    LOGERROR("Failed to link camera gstreamer elements.");
    m_bFatal = true;
    return -EU_ECODE_GST;
  }

  //
  // Set up pipeline bus async message callback and synchronous handler
  //
  bus = gst_pipeline_get_bus(GST_PIPELINE(m_pPipeline));
  gst_bus_add_signal_watch(bus);
  g_signal_connect(G_OBJECT(bus), "message", G_CALLBACK(gstBusMsgCb), this);
  gst_bus_set_sync_handler(bus, (GstBusSyncHandler)gstBusSyncHandler, this,
      (GDestroyNotify)gstBusDestroyHandler);
  gst_object_unref(bus);
  
  return EU_OK;
}

int OniGstClientUdp::destroyPipeline()
{
  if( m_pPipeline != NULL )
  {
    g_object_unref(G_OBJECT(m_pPipeline));
  }
}

void OniGstClientUdp::gstBusMsgCb(GstBus     *bus,
                                  GstMessage *message,
                                  void       *user_data)
{
  OniGstClientUdp *pClient = (OniGstClientUdp *)user_data;

  switch( GST_MESSAGE_TYPE(message) )
  {
    case GST_MESSAGE_EOS:
      LOGDIAG3("GST: End of stream.");
      break;

    case GST_MESSAGE_ERROR:
      {
        gchar  *debug;
        GError *error;

        gst_message_parse_error(message, &error, &debug);
        g_free(debug);
        LOGERROR("GST: %s.", error->message);
        g_error_free(error);
      }
      break;

    default:
      //fprintf(stderr, "DBG: GST: busmsg=%u\n", GST_MESSAGE_TYPE(message));
      break;
  }
}

GstBusSyncReply OniGstClientUdp::gstBusSyncHandler(GstBus     *bus,
                                               GstMessage *message,
                                               gpointer    user_data)
{
  OniGstClientUdp *pClient = (OniGstClientUdp *)user_data;

  if( GST_MESSAGE_TYPE(message) == GST_MESSAGE_EOS )
  {
    LOGDIAG3("GST: Sync: End of stream.");
    return GST_BUS_PASS;
  }

  // ignore anything but ? element messages
  if( GST_MESSAGE_TYPE(message) != GST_MESSAGE_ELEMENT )
  {
    return GST_BUS_PASS;
  }

  gst_message_unref(message);

  return GST_BUS_DROP;
}

void OniGstClientUdp::gstBusDestroyHandler(gpointer data)
{
}

#endif // EU_HAS_PCL
