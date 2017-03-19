////////////////////////////////////////////////////////////////////////////////
//
// Package:   Eudoxus
//
// Library:   libEudoxus
//
// File:      euPcd.h
//
/*! \file
 *
 * $LastChangedDate: 2016-01-18 14:13:40 -0700 (Mon, 18 Jan 2016) $
 * $Rev: 4263 $
 *
 * \brief Eudoxus Point Cloud Data include file
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
////////////////////////////////////////////////////////////////////////////////

#ifndef _EU_PCD_H
#define _EU_PCD_H

#include "Eudoxus/euConf.h"

#include <sys/types.h>

#include "rnr/rnrconfig.h"

#ifdef EU_HAS_PCL
#include <Eigen/Geometry>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#endif // EU_HAS_PCL

#include "Eudoxus/Eudoxus.h"
#include "Eudoxus/euHeader.h"

using namespace std;

#define EU_PCD_HDR_BUF_SIZE           514   ///< max packed PCD header buf size

#define EU_PCD_ASC_FIELD_FLOAT_WIDTH  12    ///< ascii float field width
#define EU_PCD_ASC_FIELD_INT_WIDTH    8     ///< ascii integer field width
#define EU_PCD_BIN_FIELD_FLOAT_WIDTH  4     ///< binary float field width
#define EU_PCD_BIN_FIELD_INT_WIDTH    4     ///< binary integer field width

namespace eu
{
  /*! \brief ASCII XYZ data record format string */
  static const char * const PCDAsciiDataRecordFmtXYZ = "%*.6f %*.6f %*.6f";

  /*! \brief ASCII XYZ Not a Number data record format string */
  static const char * const PCDAsciiDataRecordFmtNaN = "%*s %*s %*s";

  /*! \brief ASCII RGB appended data field format string */
  static const char * const PCDAsciiDataRecordFmtRGB = " %*u";

  /*! \brief ASCII RGB appended End of Record format string */
  static const char * const PCDAsciiDataRecordFmtEoR = "\n";
  
  /*!
   * \brief PCD x y z header format string.
   */
  static const char * const PCDHeaderFmtXYZ_0_7 =
    "VERSION .7\n"                          // pcd version
    "FIELDS x y z\n"                        // pcd data fields
    "SIZE 4 4 4\n"                          // pcd data field sizes
    "TYPE F F F\n"                          // pcd data field types
    "COUNT 1 1 1\n"                         // pcd data counts per field
    "WIDTH %u\n"                            // point cloud width
    "HEIGHT %u\n"                           // point cloud height
#ifdef EU_HAS_PCL
    "VIEWPOINT %f %f %f %f %f %f %f\n"      // viewpoint tx ty tz qw qx qy qz
#else
    "VIEWPOINT 0 0 0 1 0 0 0\n"      // viewpoint tx ty tz qw qx qy qz
#endif // EU_HAS_PCL
    "POINTS %lu\n"                          // total number of points
    "DATA %s\n";                            // data format

  /*!
   * \brief PCD x y z rgb header format string.
   */
  static const char * const PCDHeaderFmtXYZRGB_0_7 =
    "VERSION .7\n"                          // pcd version
    "FIELDS x y z rgb\n"                    // pcd data fields
    "SIZE 4 4 4 4\n"                        // pcd data field sizes
    "TYPE F F F I\n"                        // pcd data field types
    "COUNT 1 1 1 1\n"                       // pcd data counts per field
    "WIDTH %u\n"                            // point cloud width
    "HEIGHT %u\n"                           // point cloud height
#ifdef EU_HAS_PCL
    "VIEWPOINT %f %f %f %f %f %f %f\n"      // viewpoint tx ty tz qw qx qy qz
#else
    "VIEWPOINT 0 0 0 1 0 0 0\n"      // viewpoint tx ty tz qw qx qy qz
#endif // EU_HAS_PCL
    "POINTS %lu\n"                          // total number of points
    "DATA %s\n";                            // data format


  // ---------------------------------------------------------------------------
  // EuPcdHeader Base Class
  // ---------------------------------------------------------------------------

  /*!
   * \brief Point Cloud Data header base class.
   */
  class EuPcdHeader : public EuHeader
  {
  public:
    /*!
     * \brief Initialization constructor.
     *
     * \param eHdrType    Eudoxus header type.
     * \param strHdrName  Header name.
     * \param sHdrFmt     PCD header format string.
     * \param strDataFmt  PCD data format string (ascii binary).
     */
    EuPcdHeader(EuHdrType   eHdrType,
                string      strHdrName,
                const char *sHdrFmt,
                string      strDataFmt);

    /*!
     * \brief Destructor.
     */
    virtual ~EuPcdHeader();

    /*!
     * \brief Pack header values into the internal header buffer.
     *
     * \return
     * On success, returns number of packed bytes.\n
     * On failure, return error code\h_lt 0.
     */
    virtual ssize_t pack()
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
    virtual ssize_t pack(byte_t buf[], size_t sizeBuf);

    /*!
     * \brief Unpack header values from the internal header buffer.
     *
     * \return
     * On success, returns number of unpacked bytes.\n
     * On failure, return error code\h_lt 0.
     */
    virtual ssize_t unpack()
    {
      return pack(m_bufHdr, sizeof(m_bufHdr));
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
    virtual ssize_t unpack(const byte_t buf[], size_t uLen);

    /*!
     * \brief Get the maximum [sub]frame size of packed header plus data.
     *
     * \return Total size in bytes.
     */
    virtual size_t getMaxTotalSize()
    {
      return EU_PCD_HDR_BUF_SIZE - 1 + m_uDataSize;
    }

    /*!
     * \brief Get the point cloud data resolution.
     *
     * \return Returns resolution.
     */
    EuResolution getResolution() { return m_resolution; }

#ifdef EU_HAS_PCL
    /*!
     * \brief Get the point cloud viewpoint origin.
     *
     * \return Returns origin.
     */
    Eigen::Vector4f getOrigin() { return m_origin; }

    /*!
     * \brief Get the point cloud viewpoint orientation.
     *
     * \return Returns orientation.
     */
    Eigen::Quaternionf getOrientation() { return m_orientation; }
#endif // EU_HAS_PCL

    /*!
     * \brief Get the packed header buffer.
     *
     * \return Byte buffer.
     */
    byte_t *getPackedHdr() { return m_bufHdr; }

    /*!
     * \brief Set all relevant parameters.
     *
     * \param resolution  Point cloud resolution.
     * \param origin      Viewpoint origin.
     * \param orientation Viewpoint orientation.
     */
#ifdef EU_HAS_PCL
    void setParams(EuResolution       &resolution,
                   Eigen::Vector4f    &origin,
                   Eigen::Quaternionf &orientation)
    {
      setResolution(resolution);
      setOrigin(origin);
      setOrientation(orientation);
    }
#else
    void setParams(EuResolution       &resolution)
    {
      setResolution(resolution);
    }
#endif // EU_HAS_PCL

    /*!
     * \brief Set point cloud resolution.
     *
     * \param resolution  Point cloud resolution.
     */
    void setResolution(EuResolution &resolution)
    {
      setResolution(resolution.m_uWidth, resolution.m_uHeight);
    }

    /*!
     * \brief Set point cloud resolution.
     *
     * \param uWidth  Point cloud width.
     * \param uHeight Point cloud height.
     */
    void setResolution(uint_t uWidth, uint_t uHeight)
    {
      m_resolution.m_uWidth   = uWidth;
      m_resolution.m_uHeight  = uHeight;
      m_uRecordCnt            = uWidth * uHeight;
      m_uDataSize             = m_uRecordCnt * m_uRecordSize;
    }

#ifdef EU_HAS_PCL
    /*!
     * \brief Set viewpoint origin.
     *
     * \param origin      Viewpoint origin.
     */
    void setOrigin(Eigen::Vector4f &origin)
    {
      m_origin = origin;
    }

    /*!
     * \brief Set viewpoint orientation.
     *
     * \param orientation Viewpoint orientation.
     */
    void setOrientation(Eigen::Quaternionf &orientation)
    {
      m_orientation = orientation;
    }
#endif // EU_HAS_PCL

  protected:
    const char         *m_sHdrFmt;      ///< header format 
    string              m_strDataFmt;   ///< data format: one of: ascii binary
    EuResolution        m_resolution;   ///< point cloud resolution
#ifdef EU_HAS_PCL
    Eigen::Vector4f     m_origin;       ///< point cloud viewpoint origin
    Eigen::Quaternionf  m_orientation;  ///< point cloud viewpoint orientation
#endif // EU_HAS_PCL
    byte_t              m_bufHdr[EU_PCD_HDR_BUF_SIZE];  ///< packed header buf
  };


  // ---------------------------------------------------------------------------
  // EuPcdAsciiXyzHeader Class
  // ---------------------------------------------------------------------------

  /*!
   * \brief Point Cloud Data ASCII x y z header class.
   */
  class EuPcdAsciiXyzHeader : public EuPcdHeader
  {
  public:
    /*!
     * \brief Default constructor.
     */
    EuPcdAsciiXyzHeader() :
        EuPcdHeader(EuHdrTypePcdAsciiXyz, "pcd_ascii_xyz",
                    PCDHeaderFmtXYZ_0_7, "ascii")
    {
      m_uRecordSize = 3 * (EU_PCD_ASC_FIELD_FLOAT_WIDTH + 1);
    }

    /*!
     * \brief Initialization constructor.
     *
     * \param resolution  Point cloud resolution.
     */
    EuPcdAsciiXyzHeader(EuResolution &resolution) :
        EuPcdHeader(EuHdrTypePcdAsciiXyz, "pcd_ascii_xyz",
                    PCDHeaderFmtXYZ_0_7, "ascii")
    {
      m_uRecordSize = 3 * (EU_PCD_ASC_FIELD_FLOAT_WIDTH + 1);
      setResolution(resolution);
    }

    /*!
     * \brief Destructor.
     */
    virtual ~EuPcdAsciiXyzHeader() { }
  };


  // ---------------------------------------------------------------------------
  // EuPcdBinaryXyzHeader Class
  // ---------------------------------------------------------------------------

  /*!
   * \brief Point Cloud Data binary x y z header class.
   */
  class EuPcdBinaryXyzHeader : public EuPcdHeader
  {
  public:
    /*!
     * \brief Default constructor.
     */
    EuPcdBinaryXyzHeader() :
        EuPcdHeader(EuHdrTypePcdBinaryXyz, "pcd_binary_xyz",
                    PCDHeaderFmtXYZ_0_7, "binary")
    {
      m_uRecordSize = 3 * EU_PCD_BIN_FIELD_FLOAT_WIDTH;
    }

    /*!
     * \brief Initialization constructor.
     *
     * \param resolution  Point cloud resolution.
     */
    EuPcdBinaryXyzHeader(EuResolution &resolution) :
        EuPcdHeader(EuHdrTypePcdBinaryXyz, "pcd_binary_xyz",
                    PCDHeaderFmtXYZ_0_7, "binary")
    {
      m_uRecordSize = 3 * EU_PCD_BIN_FIELD_FLOAT_WIDTH;
      setResolution(resolution);
    }

    /*!
     * \brief Destructor.
     */
    virtual ~EuPcdBinaryXyzHeader() { }
  };


  // ---------------------------------------------------------------------------
  // EuPcdAsciiXyzRgbHeader Class
  // ---------------------------------------------------------------------------

  /*!
   * \brief Point Cloud Data ASCII x y z rgb header class.
   */
  class EuPcdAsciiXyzRgbHeader : public EuPcdHeader
  {
  public:
    /*!
     * \brief Default constructor.
     */
    EuPcdAsciiXyzRgbHeader() :
        EuPcdHeader(EuHdrTypePcdAsciiXyzRgb, "pcd_ascii_xyz_rgb",
                    PCDHeaderFmtXYZRGB_0_7, "ascii")
    {
      m_uRecordSize = 3 * (EU_PCD_ASC_FIELD_FLOAT_WIDTH + 1) +
                           EU_PCD_ASC_FIELD_INT_WIDTH + 1;
    }

    /*!
     * \brief Initialization constructor.
     *
     * \param resolution  Point cloud resolution.
     */
    EuPcdAsciiXyzRgbHeader(EuResolution &resolution) :
        EuPcdHeader(EuHdrTypePcdAsciiXyzRgb, "pcd_ascii_xyz_rgb",
                    PCDHeaderFmtXYZRGB_0_7, "ascii")
    {
      m_uRecordSize = 3 * (EU_PCD_ASC_FIELD_FLOAT_WIDTH + 1) +
                           EU_PCD_ASC_FIELD_INT_WIDTH + 1;
      setResolution(resolution);
    }

    /*!
     * \brief Destructor.
     */
    virtual ~EuPcdAsciiXyzRgbHeader() { }
  };


  // ---------------------------------------------------------------------------
  // EuPcdBinaryXyzRgbHeader Class
  // ---------------------------------------------------------------------------

  /*!
   * \brief Point Cloud Data Binary x y z rgb header class.
   */
  class EuPcdBinaryXyzRgbHeader : public EuPcdHeader
  {
  public:
    /*!
     * \brief Default constructor.
     */
    EuPcdBinaryXyzRgbHeader() :
        EuPcdHeader(EuHdrTypePcdBinaryXyzRgb, "pcd_binary_xyz_rgb",
                    PCDHeaderFmtXYZRGB_0_7, "binary")
    {
      m_uRecordSize = 3 * EU_PCD_BIN_FIELD_FLOAT_WIDTH +
                          EU_PCD_BIN_FIELD_INT_WIDTH;
    }

    /*!
     * \brief Initialization constructor.
     *
     * \param resolution  Point cloud resolution.
     */
    EuPcdBinaryXyzRgbHeader(EuResolution &resolution) :
        EuPcdHeader(EuHdrTypePcdBinaryXyzRgb, "pcd_binary_xyz_rgb",
                    PCDHeaderFmtXYZRGB_0_7, "binary")
    {
      m_uRecordSize = 3 * EU_PCD_BIN_FIELD_FLOAT_WIDTH +
                          EU_PCD_BIN_FIELD_INT_WIDTH;
      setResolution(resolution);
    }

    /*!
     * \brief Destructor.
     */
    virtual ~EuPcdBinaryXyzRgbHeader() { }
  };


#ifdef EU_OLD_1_6
#ifdef EU_HAS_PCL

  // ---------------------------------------------------------------------------
  // EuPcdIo Class
  // ---------------------------------------------------------------------------
  
  class EuPcdIo
  {
  public:
    EuPcdIo() { }

    virtual ~EuPcdIo() { }

    static ssize_t readbuf(byte_t buf[],
                           size_t sizeBuf,
                           //pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
                           pcl::PCLPointCloud2 &cloud);

    static ssize_t readbuf(byte_t buf[],
                           size_t sizeBuf,
                           //pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                           pcl::PCLPointCloud2 &cloud,
                           Eigen::Vector4f &origin,
                           Eigen::Quaternionf &orientation, 
                           int &pcd_version);

    static ssize_t readbufHeader(byte_t buf[],
                                 size_t sizeBuf,
                                 //pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                           pcl::PCLPointCloud2 &cloud,
                                 Eigen::Vector4f &origin,
                                 Eigen::Quaternionf &orientation, 
                                 int &pcd_version,
                                 int &data_type);

    static ssize_t readbufDataAscii(byte_t data[],
                                    size_t sizeData,
                           //pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
                           pcl::PCLPointCloud2 &cloud);

    static ssize_t readbufDataBinary(byte_t data[],
                                     size_t sizeData,
                           //pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
                           pcl::PCLPointCloud2 &cloud);

  protected:
    static size_t getline(byte_t buf[],
                          size_t sizeBuf,
                          std::string &line);
    
  };
#endif // EU_HAS_PCL
#endif // EU_OLD_1_6

} // namespace eu


#endif // _EU_PCD_H
