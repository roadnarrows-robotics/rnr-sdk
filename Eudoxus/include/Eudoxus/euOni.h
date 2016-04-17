////////////////////////////////////////////////////////////////////////////////
//
// Package:   Eudoxus
//
// Library:   libEudoxus
//
// File:      euOni.h
//
/*! \file
 *
 * $LastChangedDate: 2016-01-18 14:13:40 -0700 (Mon, 18 Jan 2016) $
 * $Rev: 4263 $
 *
 * \brief Eudoxus OpenNI include file.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2012-2015  RoadNarrows
 * (http://www.RoadNarrows.com)
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

#ifndef _EU_ONI_H
#define _EU_ONI_H

#include "Eudoxus/euConf.h"

#include <sys/types.h>
#include <sys/time.h>
#include <string>

#include "rnr/rnrconfig.h"

#ifdef EU_HAS_PCL
#include <Eigen/Geometry>
#endif // EU_HAS_PCL
  
#include "Eudoxus/Eudoxus.h"
#include "Eudoxus/euHeader.h"


// ............................................................................
// Eudoxus OpenNI Common Node Header Fields
// ............................................................................

#define EU_ONI_NODE_HDR_VERSION   "0.7" ///< Eudoxus OpenNI header version

/*!
 * \brief Header start code.
 */
#define EU_ONI_NODE_HDR_IDX_START_CODE    0       ///< buffer index
#define EU_ONI_NODE_HDR_LEN_START_CODE    2       ///< field length
#define EU_ONI_NODE_HDR_START_CODE        0x455a  ///< start code (EZ)

/*!
 * \brief Production Node Type (\ref EuHdrType).
 */
#define EU_ONI_NODE_HDR_IDX_HDR_TYPE      2       ///< buffer index
#define EU_ONI_NODE_HDR_LEN_HDR_TYPE      1       ///< field length

/*!
 * \brief Production Node Data Format (\ref OniDataFormat).
 */
#define EU_ONI_NODE_HDR_IDX_DATA_FORMAT   3       ///< buffer index
#define EU_ONI_NODE_HDR_LEN_DATA_FORMAT   1       ///< field length

/*!
 * \brief Data Size (bytes)
 */
#define EU_ONI_NODE_HDR_IDX_DATA_SIZE     4       ///< buffer index
#define EU_ONI_NODE_HDR_LEN_DATA_SIZE     4       ///< field length

/*!
 * \brief Shared Component Size of Eudoxus OpenNI node headers.
 */
#define EU_ONI_SHARED_NODE_HDR_SIZE \
      (EU_ONI_NODE_HDR_LEN_START_CODE + \
       EU_ONI_NODE_HDR_LEN_HDR_TYPE + \
       EU_ONI_NODE_HDR_LEN_DATA_FORMAT + \
       EU_ONI_NODE_HDR_LEN_DATA_SIZE)


// ............................................................................
// Eudoxus OpenNI Depth Node Header Fields
// ............................................................................

/*!
 * \brief Depth Sensor Horizontal Field of View
 */
#define EU_ONI_DEPTH_NODE_HDR_IDX_FOV_H   8       ///< buffer index
#define EU_ONI_DEPTH_NODE_HDR_LEN_FOV_H   8       ///< field length

/*!
 * \brief Depth Sensor Vertical Field of View
 */
#define EU_ONI_DEPTH_NODE_HDR_IDX_FOV_V   16      ///< buffer index
#define EU_ONI_DEPTH_NODE_HDR_LEN_FOV_V   8       ///< field length

/*!
 * \brief Depth Sensor Horizontal Resolution
 */
#define EU_ONI_DEPTH_NODE_HDR_IDX_WIDTH   24      ///< buffer index
#define EU_ONI_DEPTH_NODE_HDR_LEN_WIDTH   2       ///< field length

/*!
 * \brief Depth Sensor Vertical Resolution
 */
#define EU_ONI_DEPTH_NODE_HDR_IDX_HEIGHT  26      ///< buffer index
#define EU_ONI_DEPTH_NODE_HDR_LEN_HEIGHT  2       ///< field length

/*!
 * \brief Depth Node Header Size.
 */
#define EU_ONI_DEPTH_NODE_HDR_SIZE \
      (EU_ONI_SHARED_NODE_HDR_SIZE + \
       EU_ONI_DEPTH_NODE_HDR_LEN_FOV_H + \
       EU_ONI_DEPTH_NODE_HDR_LEN_FOV_V + \
       EU_ONI_DEPTH_NODE_HDR_LEN_WIDTH + \
       EU_ONI_DEPTH_NODE_HDR_LEN_HEIGHT)


// ............................................................................
// Eudoxus OpenNI Image Node Header Fields
// ............................................................................

/*!
 * \brief Image Sensor Horizontal Field of View
 */
#define EU_ONI_IMAGE_NODE_HDR_IDX_FOV_H   8       ///< buffer index
#define EU_ONI_IMAGE_NODE_HDR_LEN_FOV_H   8       ///< field length

/*!
 * \brief Image Sensor Vertical Field of View
 */
#define EU_ONI_IMAGE_NODE_HDR_IDX_FOV_V   16      ///< buffer index
#define EU_ONI_IMAGE_NODE_HDR_LEN_FOV_V   8       ///< field length

/*!
 * \brief Image Sensor Horizontal Resolution
 */
#define EU_ONI_IMAGE_NODE_HDR_IDX_WIDTH   24      ///< buffer index
#define EU_ONI_IMAGE_NODE_HDR_LEN_WIDTH   2       ///< field length

/*!
 * \brief Image Sensor Vertical Resolution
 */
#define EU_ONI_IMAGE_NODE_HDR_IDX_HEIGHT  26      ///< buffer index
#define EU_ONI_IMAGE_NODE_HDR_LEN_HEIGHT  2       ///< field length

/*!
 * \brief Image Node Header Size.
 */
#define EU_ONI_IMAGE_NODE_HDR_SIZE \
      (EU_ONI_SHARED_NODE_HDR_SIZE + \
       EU_ONI_IMAGE_NODE_HDR_LEN_FOV_H + \
       EU_ONI_IMAGE_NODE_HDR_LEN_FOV_V + \
       EU_ONI_IMAGE_NODE_HDR_LEN_WIDTH + \
       EU_ONI_IMAGE_NODE_HDR_LEN_HEIGHT)


// ............................................................................
// Eudoxus OpenNI Audio Node Header Fields
// ............................................................................

/*!
 * \brief Audio Node Header Size.
 */
#define EU_ONI_AUDIO_NODE_HDR_SIZE \
      (EU_ONI_SHARED_NODE_HDR_SIZE)   // TODO


// ............................................................................
// Eudoxus OpenNI IMU Node Header Fields
// ............................................................................

/*!
 * \brief IMU Node Header Size.
 */
#define EU_ONI_IMU_NODE_HDR_SIZE \
      (EU_ONI_SHARED_NODE_HDR_SIZE)   // TODO


namespace eu
{
  /*!
   * \brief OpenNI Production Node Data Formats.
   */
  typedef enum
  {
    EuOniDataFormatNone = 0,        ///< no or unknown data format
    EuOniDataFormatDepthProjective, ///< projective coordinates depth data
    EuOniDataFormatDepthRealWorld,  ///< real world coordinates depth data
    EuOniDataFormatImageRgb,        ///< image red, green, blue 
    EuOniDataFormatImageJpeg,       ///< image jpeg
    EuOniDataFormatAudioAu,         ///< audio au format
    EuOniDataFormatImuOrigOrient,   ///< imu origin + orientation

    EuOniDataFormatNumOf            ///< number of node types (keep last)
  } EuOniDataFormat;


  // ---------------------------------------------------------------------------
  // EuOniNodeHeader Base Class
  // ---------------------------------------------------------------------------

  /*!
   * \brief Eudoxus OpenNI production node base header class.
   */
  class EuOniNodeHeader : public EuHeader
  {
  public:
    /*!
     * \brief Default initialization constructor.
     *
     * \param eHdrType      Header type.
     * \param strHdrName    Header name.
     * \param eDataFormat   Data format.
     */
    EuOniNodeHeader(EuHdrType         eHdrType,
                    string            strHdrName,
                    EuOniDataFormat   eDataFormat) :
        EuHeader(eHdrType, strHdrName),
        m_eDataFormat(eDataFormat)
    {
    }

    /*!
     * \brief Destructor.
     */
    virtual ~EuOniNodeHeader() { }

    /*!
     * \brief Pack common oni header values into the given buffer.
     *
     * \param [out] buf Output buffer.
     * \param sizeBuf   Size of buffer.
     *
     * \return
     * On success, returns number of packed bytes.\n
     * On failure, return error code\h_lt 0.
     */
    ssize_t pack(byte_t buf[], size_t sizeBuf);

    /*!
     * \brief Unpack common oni header values from the given buffer.
     *
     * \param [in] buf  Input buffer.
     * \param uLen      Length of data in buffer.
     *
     * \return
     * On success, returns number of unpacked bytes.\n
     * On failure, return error code\h_lt 0.
     */
    ssize_t unpack(const byte_t buf[], size_t uLen);

    /*!
     * \brief Peek at header type in common part of oni header
     *
     * \param [in] buf  Input packed buffer.
     * \param uLen      Length of data in buffer.
     *
     * \return Returns the header type.
     */
    static EuHdrType peekHdrType(byte_t buf[], size_t uLen);

    EuOniDataFormat   getDataFormat() { return m_eDataFormat; }

  protected:
    EuOniDataFormat   m_eDataFormat;
  };


  // ---------------------------------------------------------------------------
  // EuOniDepthNodeHeader Class
  // ---------------------------------------------------------------------------

  /*!
   * \brief Eudoxus OpenNI depth production node header class.
   */
  class EuOniDepthNodeHeader : public EuOniNodeHeader
  {
  public:
    /*!
     * \brief Default constructor.
     */
    EuOniDepthNodeHeader();

    /*!
     * \brief Initialization constructor.
     *
     * \param resolution  Sensor resolution.
     * \param fov         Sensor field of view.
     */
    EuOniDepthNodeHeader(EuResolution &resolution, EuFoV &fov);

    /*!
     * \brief destructor.
     */
    virtual ~EuOniDepthNodeHeader() { }

    /*!
     * \brief Pack header values into the internal header buffer.
     *
     * \return
     * On success, returns number of packed bytes.\n
     * On failure, return error code\h_lt 0.
     */
    ssize_t pack()
    {
      return pack(m_bufHdr, sizeof(m_bufHdr));
    }

    /*!
     * \brief Pack header values into the given buffer.
     *
     * \param [out] buf Output buffer.
     * \param sizeBuf   Size of buffer.
     *
     * \return
     * On success, returns number of packed bytes.\n
     * On failure, return error code\h_lt 0.
     */
    ssize_t pack(byte_t buf[], size_t sizeBuf);

    /*!
     * \brief Unpack header values from the internal header buffer.
     *
     * \return
     * On success, returns number of unpacked bytes.\n
     * On failure, return error code\h_lt 0.
     */
    ssize_t unpack()
    {
      return unpack(m_bufHdr, sizeof(m_bufHdr));
    }

    /*!
     * \brief Unpack header values from the given buffer.
     *
     * \param [in] buf  Input buffer.
     * \param uLen      Length of data in buffer.
     *
     * \return
     * On success, returns number of unpacked bytes.\n
     * On failure, return error code\h_lt 0.
     */
    ssize_t unpack(const byte_t buf[], size_t uLen);

    /*!
     * \brief Get the maximum [sub]frame size of packed header plus data.
     *
     * \return Total size in bytes.
     */
    virtual size_t getMaxTotalSize()
    {
      return getTotalSize();
    }

    /*!
     * \brief Get the point cloud data resolution.
     *
     * \return Returns resolution.
     */
    EuResolution getResolution() { return m_resolution; }

    /*!
     * \brief Get the point cloud viewpoint origin.
     *
     * \return Returns origin.
     */
    EuFoV getFieldOfView() { return m_fov; }

    /*!
     * \brief Get the packed header buffer.
     *
     * \return Byte buffer.
     */
    byte_t *getPackedHdr() { return m_bufHdr; }

    /*!
     * \brief Set all relevant parameters.
     *
     * \param resolution  Sensor resolution.
     * \param fov         Sensor field of view.
     */
    void setParams(EuResolution &resolution,
                   EuFoV        &fov)
    {
      setFieldOfView(fov);
      setResolution(resolution);
    }

    /*!
     * \brief Set sensor field of view.
     *
     * \param fov  Sensor field of view.
     */
    void setFieldOfView(EuFoV &fov)
    {
      m_fov = fov;
    }

    /*!
     * \brief Set sensor resolution.
     *
     * \param resolution  Sensor resolution.
     */
    void setResolution(EuResolution &resolution)
    {
      setResolution(resolution.m_uWidth, resolution.m_uHeight);
    }

    /*!
     * \brief Set sensor resolution.
     *
     * \param uWidth  Sensor pixel width.
     * \param uHeight Sensor pixel height.
     */
    void setResolution(uint_t uWidth, uint_t uHeight)
    {
      m_resolution.m_uWidth   = uWidth;
      m_resolution.m_uHeight  = uHeight;
      m_uRecordCnt            = uWidth * uHeight;
      m_uDataSize             = m_uRecordCnt * m_uRecordSize;
      m_uTotalSize            = EU_ONI_DEPTH_NODE_HDR_SIZE + m_uDataSize;
    }

  protected:
    EuFoV         m_fov;
    EuResolution  m_resolution;
    byte_t        m_bufHdr[EU_ONI_DEPTH_NODE_HDR_SIZE];
  };


  // ---------------------------------------------------------------------------
  // EuOniImageNodeHeader Class
  // ---------------------------------------------------------------------------

  /*!
   * \brief Eudoxus OpenNI image production node header class.
   */
  class EuOniImageNodeHeader : public EuOniNodeHeader
  {
  public:
    /*!
     * \brief Default constructor.
     */
    EuOniImageNodeHeader();

    /*!
     * \brief Initialization constructor.
     *
     * \param resolution  Sensor resolution.
     * \param fov         Sensor field of view.
     */
    EuOniImageNodeHeader(EuResolution &resolution, EuFoV &fov);

    /*!
     * \brief destructor.
     */
    virtual ~EuOniImageNodeHeader() { }

    /*!
     * \brief Pack header values into the internal header buffer.
     *
     * \return
     * On success, returns number of packed bytes.\n
     * On failure, return error code\h_lt 0.
     */
    ssize_t pack()
    {
      return pack(m_bufHdr, sizeof(m_bufHdr));
    }

    /*!
     * \brief Pack header values into the given buffer.
     *
     * \param [out] buf Output buffer.
     * \param sizeBuf   Size of buffer.
     *
     * \return
     * On success, returns number of packed bytes.\n
     * On failure, return error code\h_lt 0.
     */
    ssize_t pack(byte_t buf[], size_t sizeBuf);

    /*!
     * \brief Unpack header values from the internal header buffer.
     *
     * \return
     * On success, returns number of unpacked bytes.\n
     * On failure, return error code\h_lt 0.
     */
    ssize_t unpack()
    {
      return unpack(m_bufHdr, sizeof(m_bufHdr));
    }

    /*!
     * \brief Unpack header values from the given buffer.
     *
     * \param [in] buf  Input buffer.
     * \param uLen      Length of data in buffer.
     *
     * \return
     * On success, returns number of unpacked bytes.\n
     * On failure, return error code\h_lt 0.
     */
    ssize_t unpack(const byte_t buf[], size_t uLen);

    /*!
     * \brief Get the maximum [sub]frame size of packed header plus data.
     *
     * \return Total size in bytes.
     */
    virtual size_t getMaxTotalSize()
    {
      return getTotalSize();
    }

    /*!
     * \brief Get the point cloud data resolution.
     *
     * \return Returns resolution.
     */
    EuResolution getResolution() { return m_resolution; }

    /*!
     * \brief Get the point cloud viewpoint origin.
     *
     * \return Returns origin.
     */
    EuFoV getFieldOfView() { return m_fov; }

    /*!
     * \brief Get the packed header buffer.
     *
     * \return Byte buffer.
     */
    byte_t *getPackedHdr() { return m_bufHdr; }

    /*!
     * \brief Set all relevant parameters.
     *
     * \param resolution  Sensor resolution.
     * \param fov         Sensor field of view.
     */
    void setParams(EuResolution &resolution,
                   EuFoV        &fov)
    {
      setFieldOfView(fov);
      setResolution(resolution);
    }

    /*!
     * \brief Set sensor field of view.
     *
     * \param fov  Sensor field of view.
     */
    void setFieldOfView(EuFoV &fov)
    {
      m_fov = fov;
    }

    /*!
     * \brief Set sensor resolution.
     *
     * \param resolution  Sensor resolution.
     */
    void setResolution(EuResolution &resolution)
    {
      setResolution(resolution.m_uWidth, resolution.m_uHeight);
    }

    /*!
     * \brief Set sensor resolution.
     *
     * \param uWidth  Sensor pixel width.
     * \param uHeight Sensor pixel height.
     */
    void setResolution(uint_t uWidth, uint_t uHeight)
    {
      m_resolution.m_uWidth   = uWidth;
      m_resolution.m_uHeight  = uHeight;
      m_uRecordCnt            = uWidth * uHeight;
      m_uDataSize             = m_uRecordCnt * m_uRecordSize;
      m_uTotalSize            = EU_ONI_DEPTH_NODE_HDR_SIZE + m_uDataSize;
    }

  protected:
    EuFoV         m_fov;
    EuResolution  m_resolution;
    byte_t        m_bufHdr[EU_ONI_DEPTH_NODE_HDR_SIZE];
  };


  // ---------------------------------------------------------------------------
  // EuOniAudioNodeHeader Class
  // ---------------------------------------------------------------------------

  /*!
   * \brief Eudoxus OpenNI audio production node header class.
   */
  class EuOniAudioNodeHeader : public EuOniNodeHeader
  {
  public:
    /*!
     * \brief Default constructor.
     * TODO
     */
    EuOniAudioNodeHeader() :
          EuOniNodeHeader(EuHdrTypeOniAudio,
                    "oni_audio",
                    EuOniDataFormatAudioAu)
    {
    }

    /*!
     * \brief destructor.
     */
    virtual ~EuOniAudioNodeHeader() { }

    /*!
     * \brief Pack header values into the internal header buffer.
     *
     * \return
     * On success, returns number of packed bytes.\n
     * On failure, return error code\h_lt 0.
     */
    ssize_t pack()
    {
      return pack(m_bufHdr, sizeof(m_bufHdr));
    }

    /*!
     * \brief Pack header values into the given buffer.
     *
     * \param [out] buf Output buffer.
     * \param sizeBuf   Size of buffer.
     *
     * \return
     * On success, returns number of packed bytes.\n
     * On failure, return error code\h_lt 0.
     */
    ssize_t pack(byte_t buf[], size_t sizeBuf)
    {
      return -EU_ECODE_INTERNAL;    // TODO
    }

    /*!
     * \brief Unpack header values from the internal header buffer.
     *
     * \return
     * On success, returns number of unpacked bytes.\n
     * On failure, return error code\h_lt 0.
     */
    ssize_t unpack()
    {
      return unpack(m_bufHdr, sizeof(m_bufHdr));
    }

    /*!
     * \brief Unpack header values from the given buffer.
     *
     * \param [in] buf  Input buffer.
     * \param uLen      Length of data in buffer.
     *
     * \return
     * On success, returns number of unpacked bytes.\n
     * On failure, return error code\h_lt 0.
     */
    ssize_t unpack(const byte_t buf[], size_t uLen)
    {
      return -EU_ECODE_INTERNAL;    // TODO
    }

    /*!
     * \brief Get the maximum [sub]frame size of packed header plus data.
     *
     * \return Total size in bytes.
     */
    virtual size_t getMaxTotalSize()
    {
      return getTotalSize();
    }

  protected:
    // TODO
    byte_t        m_bufHdr[EU_ONI_AUDIO_NODE_HDR_SIZE];
  };


  // ---------------------------------------------------------------------------
  // EuOniImuNodeHeader Class
  // ---------------------------------------------------------------------------

  /*!
   * \brief Eudoxus OpenNI Inertial Measurement Unit production node header
   * class.
   */
  class EuOniImuNodeHeader : public EuOniNodeHeader
  {
  public:
    /*!
     * \brief Default constructor.
     * TODO
     */
    EuOniImuNodeHeader() :
          EuOniNodeHeader(EuHdrTypeOniImu,
                    "oni_imu",
                    EuOniDataFormatImuOrigOrient)
    {
    }

    virtual ~EuOniImuNodeHeader() { }

    /*!
     * \brief Pack header values into the internal header buffer.
     *
     * \return
     * On success, returns number of packed bytes.\n
     * On failure, return error code\h_lt 0.
     */
    ssize_t pack()
    {
      return pack(m_bufHdr, sizeof(m_bufHdr));
    }

    /*!
     * \brief Pack header values into the given buffer.
     *
     * \param [out] buf Output buffer.
     * \param sizeBuf   Size of buffer.
     *
     * \return
     * On success, returns number of packed bytes.\n
     * On failure, return error code\h_lt 0.
     */
    ssize_t pack(byte_t buf[], size_t sizeBuf)
    {
      return -EU_ECODE_INTERNAL;    // TODO
    }

    /*!
     * \brief Unpack header values from the internal header buffer.
     *
     * \return
     * On success, returns number of unpacked bytes.\n
     * On failure, return error code\h_lt 0.
     */
    ssize_t unpack()
    {
      return unpack(m_bufHdr, sizeof(m_bufHdr));
    }

    /*!
     * \brief Unpack header values from the given buffer.
     *
     * \param [in] buf  Input buffer.
     * \param uLen      Length of data in buffer.
     *
     * \return
     * On success, returns number of unpacked bytes.\n
     * On failure, return error code\h_lt 0.
     */
    ssize_t unpack(const byte_t buf[], size_t uLen)
    {
      return -EU_ECODE_INTERNAL;    // TODO
    }

    /*!
     * \brief Get the maximum [sub]frame size of packed header plus data.
     *
     * \return Total size in bytes.
     */
    virtual size_t getMaxTotalSize()
    {
      return getTotalSize();
    }

  protected:
    // TODO
    byte_t        m_bufHdr[EU_ONI_IMU_NODE_HDR_SIZE];
  };

} // namespace eu


#endif // _EU_ONI_H
