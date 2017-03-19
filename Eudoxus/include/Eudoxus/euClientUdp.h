////////////////////////////////////////////////////////////////////////////////
//
// Package:   Eudoxus
//
// Library:   libEudoxus
//
// File:      euClientUdp.h
//
/*! \file
 *
 * $LastChangedDate: 2016-01-18 14:13:40 -0700 (Mon, 18 Jan 2016) $
 * $Rev: 4263 $
 *
 * \brief Eudoxus Gstreamer over UDP client class declarations.
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
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _EU_CLIENT_UDP_H
#define _EU_CLIENT_UDP_H

#include "Eudoxus/euConf.h"

#ifdef EU_HAS_PCL

#include <string.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include <gst/gst.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLPointCloud2.h>

#include "Eudoxus/Eudoxus.h"

using namespace std;

namespace eu
{
  //
  // Forward declarations
  //
  class GstRingBuf;

  //---------------------------------------------------------------------------
  // Eudoxus OpenNI GStreamer UDP client termination class.
  //---------------------------------------------------------------------------
  class OniGstClientUdp
  {
  public:
    static const int UdpPortDft    = 4000; ///< UDP port default
    static const int NumBuffersDft = 2;
                                ///< number of buffers (double buffered) default

    /*!
     * \brief Default initialization constructor.
     *
     * \param nPort         UDP port number.
     * \param nNumBuffers   Number of ring buffers.
     */
    OniGstClientUdp(const int nPort = UdpPortDft,
                    const int nNumBuffers = NumBuffersDft);

    /*!
     * \brief Destructor.
     */
    ~OniGstClientUdp();

    /*!
     * \brief Start receiving point cloud stream over UDP.
     *
     * \copydoc doc_return_std
     */
    virtual int start();

    /*!
     * \brief Stop receiving point cloud stream of UDP and release resources.
     * 
     * \copydoc doc_return_std
    */
    virtual int stop();

    /*!
     * \brief Grab a point cloud frame from the UDP stream.
     *
     * \param fnCpBuf   User-supplied copy buffer function.
     * \param lMilliSec Block wait time in milliseconds. If 0, then block
     *                  forever until buffer available.
     *
     * \copydoc doc_return_std
     */
    virtual int grabPointCloudFrame(int  (*fnCpBuf)(byte_t *, size_t),
                                    long lMilliSec=0);

    /*!
     * \brief Grab a point cloud frame from the UDP stream.
     *
     * \param [out] cloud       Output point cloud.
     * \param [out] origin      Sensor origin.
     * \param [out] orientation Sensor orientation.
     * \param lMilliSec         Block wait time in milliseconds. If 0, then
     *                          block forever until buffer available.
     *
     * \copydoc doc_return_std
     */
    virtual int grabPointCloudFrame(pcl::PCLPointCloud2 &cloud,
                                    Eigen::Vector4f     &origin,
                                    Eigen::Quaternionf  &orientation,
                                    long                lMilliSec=0);

    /*!
     * \brief Test if pipeline stream is running.
     *
     * \return Returns true if camera is on, else false.
     */
    bool isStreaming()
    {
      return m_bIsStreaming;
    }

    /*!
     * \brief Test if object is in a fatal condition.
     *
     * \return Returns true or false.
     */
    bool isFatal()
    {
      return m_bFatal;
    }

  protected:
    int         m_nPort;            ///< UDP port
    bool        m_bIsStreaming;     ///< is [not] streaming
    bool        m_bFatal;           ///< instance is in a fatal state
    GstElement *m_pPipeline;        ///< pipeline
    GstElement *m_pElemUdpSrc;      ///< UDP source element
    GstElement *m_pElemOniPduDec;   ///< OpenNI PDU decoder element
    GstElement *m_pElemFakeSink;    ///< fake sink 
    int         m_nSignalId;        ///< frame callback signal
    GstRingBuf *m_pRingBuf;         ///< frame ring buffer

    /*!
     * \brief Start the camera and the pipeline.
     */
    void startPipeline();

    /*!
     * \brief Stop the camera and the pipeline.
     */
    void stopPipeline();

    /*!
     * \brief Make the Eudoxus client OpenNI gstreamer UDP pipeline.
     *
     * \par Required Eudoxus server pipeline:
     * \verbatim
     *   |-----------------------------------|
     *   |onisrc - ... - onipduenc - udpsink []
     *   |-----------------------------------|
     * \endverbatim
     *
     * \par Eudoxus client pipeline:
     * \verbatim
     *            |-------------------------------|      |-----
     * udpsink - [] udpsrc - oniudpdec - fakesink [] ~~ [] frame callback
     *            |-------------------------------|      |-----
     *     \endverbatim
     *
     * \copydoc doc_return_std
     */
    int makePipeline();

    /*!
     * \brief Destroy GStreamer pipeline releasing all related resources.
     *
     * \copydoc doc_return_std
     */
    int destroyPipeline();

    static GstBusSyncReply gstBusSyncHandler(GstBus     *bus,
                                             GstMessage *message,
                                             gpointer    user_data);

    static void gstBusDestroyHandler(gpointer    user_data);

    /*!
     * \brief GStreamer bus message callback.
     *
     * \param bus       GStreamer bus.
     * \param message   Bus message.
     * \param user_data User data (this)
     */
    static void gstBusMsgCb(GstBus     *bus,
                            GstMessage *message,
                            gpointer    user_data);

    /*!
     * \brief GStreamer receive point cloud frame callback.
     *
     * This function is called when a Gst buffer from the sensor has been
     * retrieved.
     *
     * \param element   GStreamer element.
     * \param buffer    Frame buffer.
     * \param pad       GStreamer pad.
     * \param user_data User data (this)
     */
    static void frameCb(GstElement *element,
                        GstBuffer  *buffer,
                        GstPad     *pad,
                        void       *user_data);
  };

} // namespace

#endif // EU_HAS_PCL

#endif // _EU_CLIENT_UDP_H
