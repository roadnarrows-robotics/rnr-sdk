////////////////////////////////////////////////////////////////////////////////
//
// Package:   Eudoxus
//
// Library:   libEudoxus
//
// File:      euPcs.h
//
/*! \file
 *
 * $LastChangedDate: 2016-01-18 14:13:40 -0700 (Mon, 18 Jan 2016) $
 * $Rev: 4263 $
 *
 * \brief Eudoxus Point Cloud Stream include file
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

#ifndef _EU_PCS_H
#define _EU_PCS_H

#include "Eudoxus/euConf.h"

#include <sys/types.h>
#include <sys/time.h>
#include <stdio.h>

#include <string>

#include "rnr/rnrconfig.h"

#ifdef EU_HAS_PCL
#include <Eigen/Geometry>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>

#endif // EU_HAS_PCL
  
#include "ni/XnOpenNI.h"
#include "ni/XnTypes.h"
#include "ni/XnCppWrapper.h"

#include "Eudoxus/Eudoxus.h"
#include "Eudoxus/euHeader.h"
#include "Eudoxus/euPcd.h"
#include "Eudoxus/euOni.h"

using namespace std;

#define EU_PCS_VERSION            "0.8"   ///< PCS frame version
#define EU_PCS_HDR_BUF_SIZE       514     ///< maximum length of PCS header

#define EU_PCS_HDR_LEN_DATASIZE   10      ///< DataSize value length
#define EU_PCS_HDR_LEN_EOR        1       ///< End of Record length

namespace eu
{
  /*!
   * \brief PCS Header 0.8 Format.
   */
  static const char * const PCSHeaderFmt_0_8 =
    "# @PCS PointCloudStream: %s\n"       // pcs frame start pattern w/ version
    "# @PCS MimeType: %s\n"               // pcs frame mime type
    "# @PCS Endian: %s\n"                 // pcs frame byte order
    "# @PCS FrameId: %zu\n"               // pcs frame id (number)
    "# @PCS TimeStamp: %lu.%lu\n"         // pcs frame time stamp (seconds)
    "# @PCS DataSize: %*zu\n";            // pcs data size (bytes)


  // ---------------------------------------------------------------------------
  // EuPcsHeader Class
  // ---------------------------------------------------------------------------
  
  class EuPcsHeader : public EuHeader
  {
  public:
    EuPcsHeader();

    virtual ~EuPcsHeader();

    /*!
     * \brief Pack Point Cloud Stream header values into internal buffer.
     *
     * \return Returns number of packed bytes.
     */
    ssize_t pack()
    {
      return pack(m_bufHdr, sizeof(m_bufHdr));
    }

    /*!
     * \brief Pack Point Cloud Stream header values into provided buffer.
     *
     * \param [out] buf       Output packed buffer.
     * \param sizeBuf         Size of output buffer.
     *
     * \return Returns number of packed bytes.
     */
    ssize_t pack(byte_t buf[], size_t sizeBuf);

    /*!
     * \brief Unpack Point Cloud Stream header from internal buffer.
     *
     * \return Returns number of unpacked bytes.
     */
    ssize_t unpack()
    {
      return unpack(m_bufHdr, sizeof(m_bufHdr));
    }

    /*!
     * \brief Unpack Point Cloud Stream header from buffer.
     *
     * \param [int] buf         Input packed buffer.
     * \param uLen              Length of data in buffer.
     *
     * \return Returns number of unpacked bytes.
     */
    ssize_t unpack(const byte_t buf[], size_t uLen);

    ssize_t unpack(FILE *fp);

    /*!
     * \brief Patch-upt the DataSize field value.
     *
     * The header is already packed, so patching is a post-packed operation.
     *
     * \param sizeHdr   Size of packed header
     * \param sizeData  DataSize field value to insert.
     *
     * \return Bytes replaced.
     */
    ssize_t patchDataSize(byte_t buf[], size_t sizeHdr, size_t sizeData);

    /*!
     * \brief Get the maximum packed header size.
     *
     * \return Total size in bytes.
     */
    virtual size_t getMaxHdrSize()
    {
      return EU_PCS_HDR_BUF_SIZE;
    }

    /*!
     * \brief Get the maximum frame size of packed header plus data.
     *
     * \return Total size in bytes.
     */
    virtual size_t getMaxTotalSize()
    {
      return EU_PCS_HDR_BUF_SIZE - 1 + m_uDataSize;
    }

    string getVersion() { return m_strVersion; }

    EuMimeType getMimeType()  { return m_eMimeType; }

    EuEndian getEndian()  { return m_eEndian; }

    size_t getFrameId() { return m_uFrameId; }
  
    struct timeval getTimeStamp() { return m_tvTimeStamp; }

    /*!
     * \brief Get the packed header buffer.
     *
     * \return Byte buffer.
     */
    byte_t *getPackedHdr() { return m_bufHdr; }

    /*!
     * \brief Set all relevant parameters.
     *
     * \param eMimeType   Eudoxus MimeType enumerate.
     * \param uFrameId    Frame id.
     * \param tvTimeStamp Frame time stamp.
     */
    void setParams(const EuMimeType       eMimeType,
                   const size_t           uFrameId,
                   const struct timeval  &tvTimeStamp)
    {
      setMimeType(eMimeType);
      setFrameId(uFrameId);
      setTimeStamp(tvTimeStamp);
    }

    /*!
     * \brief Set frame MimeType
     *
     * \param eMimeType   Eudoxus MimeType enumerate.
     */
    void setMimeType(const EuMimeType eMimeType)
    {
      m_eMimeType = eMimeType;
    }

    /*!
     * \brief Set frame id.
     *
     * \param uFrameId    Frame id.
     */
    void setFrameId(const size_t uFrameId)
    {
      m_uFrameId = uFrameId;
    }

    /*!
     * \brief Increment frame id.
     *
     * \return Incremented frame id.
     */
    size_t bumpFrameId()
    {
      return ++m_uFrameId;
    }

    /*!
     * \brief Set frame time stamp.
     *
     * \param tvTimeStamp Frame time stamp.
     */
    void setTimeStamp(const struct timeval &tvTimeStamp)
    {
      m_tvTimeStamp = tvTimeStamp;
    }

  protected:
    string          m_strVersion;     ///< PCS version
    EuMimeType      m_eMimeType;      ///< frame MimeType
    EuEndian        m_eEndian;        ///< frame byte order
    size_t          m_uFrameId;       ///< frame id
    struct timeval  m_tvTimeStamp;    ///< frame time stamp
    byte_t          m_bufHdr[EU_PCS_HDR_BUF_SIZE]; ///< packed header buffer

    /*!
     * \brief Find next line in buffer.
     *
     * \param [in] buf  Input buffer.
     * \param uLen      Length of data in buffer.
     *
     * \return Index of next '\n' or the '\0' character.
     */
    static size_t findLine(const byte_t buf[], size_t uDataSize);

    static size_t findLine(FILE *fp, char buf[], size_t sizeBuf);
  };


  // ---------------------------------------------------------------------------
  // EuPcsFrame Class
  // ---------------------------------------------------------------------------

  class EuPcsFrame
  {
  public:
    static const int MaxSections = (int)4;  ///< max sections/frame

    /*!
     * \brief Default constructor.
     */
    EuPcsFrame();

    /*!
     * \brief Destructor.
     */
    virtual ~EuPcsFrame();

    /*!
     * \brief Configure PCS frame.
     *
     * This call is important when producing (inserting) frames. On extraction,
     * these values are imbedded in the encoded frame and will be automatically
     * discovered.
     *
     * This function needs to be called only once prior to frame production
     * unless the values change.
     *
     * \param eMimeType   Eudoxus MimeType.
     * \param uProdNodes  Bit list of enabled productions nodes
     *                    (see \refEuProdNodeType).
     * \param resolution  Depth and/or image sensor resolution. Note that if
     *                    both production nodes are specified, the resolution
     *                    is always kept equal.
     * \param fovDepth    Depth sensor field of view.
     * \param fovImage    Image sensor field of view.
     *
     * \return
     * On success, returns EU_OK.\n
     * On failure, return error code \h_lt 0.
     */
    int configure(EuMimeType    eMimeType,
                  uint_t        uProdNodes,
                  EuResolution &resolution,
                  EuFoV        &fovDepth,
                  EuFoV        &fovImage);

    /*!
     * \brief Get the maximum frame size.
     *
     * \note The appropiate data must be initialized prior to calling this
     * function (see \ref configure).
     *
     * \return Number of bytes.
     */
    size_t getMaxFrameSize();

    size_t getMaxTotalSize() { return m_hdrPcs.getMaxTotalSize(); }

    size_t getDataSize() { return m_hdrPcs.getDataSize(); }

    EuMimeType getMimeType()  { return m_hdrPcs.getMimeType(); }

    size_t getFrameId() { return m_hdrPcs.getFrameId(); }

    /*!
     * \brief Get the error message of the most recent frame error.
     *
     * \return Error message.
     */
    const char *getErrorMsg()
    {
      return m_bufErrorMsg;
    }

    ssize_t insertFrame(xn::DepthGenerator &xnGenDepth,
                        xn::ImageGenerator &xnGenImage,
                        byte_t              buf[],
                        size_t              sizeBuf);

#ifdef EU_HAS_PCL
    ssize_t extractFrameToPointCloud(byte_t buf[],
                                     size_t sizeBuf,
                                     pcl::PCLPointCloud2 &cloud,
                                     Eigen::Vector4f &origin,
                                     Eigen::Quaternionf &orientation);
#endif // EU_HAS_PCL

    ssize_t extractPcsHeader(FILE *fp)
    {
      return m_hdrPcs.unpack(fp);
    }

    ssize_t extractFrameToBuf(FILE   *fp,
                              byte_t  buf[],
                              size_t  sizeBuf,
                              bool    bExtractHdr=true);

  protected:
    uint_t        m_uProdNodes;               ///< enabled production nodes
    EuPcsHeader   m_hdrPcs;                   ///< PCS header
    int           m_nNumSections;             ///< number of sections in frame
    EuHeader     *m_pHdrSection[MaxSections]; ///< section headers
    char          m_bufErrorMsg[256];         ///< error message buffer

    /*!
     * \brief Clear allocated section data and state.
     */
    void clearSections();

    /*!
     * \brief Set error message
     */
    void setErrorMsg(const char *sFmt, ...);

    ssize_t insertSectionOniDepth(EuHeader           *pHdr,
                                  xn::DepthGenerator &xnGenDepth,
                                  byte_t              buf[],
                                  size_t              sizeBuf);

    ssize_t insertSectionOniImage(EuHeader           *pHdr,
                                  xn::ImageGenerator &xnGenImage,
                                  byte_t              buf[],
                                  size_t              sizeBuf);

    ssize_t insertSectionPcdBinary(EuHeader           *pHdr,
                                   xn::DepthGenerator &xnGenDepth,
                                   xn::ImageGenerator &xnGenImage,
                                   byte_t              buf[],
                                   size_t              sizeBuf);

    ssize_t insertSectionPcdAscii(EuHeader           *pHdr,
                                  xn::DepthGenerator &xnGenDepth,
                                  xn::ImageGenerator &xnGenImage,
                                  byte_t              buf[],
                                  size_t              sizeBuf);

    ssize_t extractOniFrameToPcdBinary(byte_t   buf[],
                                       size_t   sizeBuf,
                                       byte_t *&pBufOut);

#if defined(EU_HAS_PCL) && defined(EU_FUTURE)
    ssize_t extractOniFrameToPointCloud(
                                  byte_t buf[],
                                  size_t sizeBuf,
                                  pcl::PCLPointCloud2 &cloud,
                                  Eigen::Vector4f &origin,
                                  Eigen::Quaternionf &orientation);
#endif // defined(EU_HAS_PCL) && defined(EU_FUTURE)
  };

} // namespace eu


#endif // _EU_PCS_H
