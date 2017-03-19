////////////////////////////////////////////////////////////////////////////////
//
// Package:   Eudoxus
//
// Library:   libEudoxus
//
// File:      euPcs.cxx
//
/*! \file
 *
 * $LastChangedDate: 2016-01-18 14:13:40 -0700 (Mon, 18 Jan 2016) $
 * $Rev: 4263 $
 *
 * \brief Eudoxus Point Cloud Stream definitions and helper functions.
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

#include "Eudoxus/euConf.h"

#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <endian.h>
#include <string.h>
#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#ifdef EU_HAS_PCL
#include <Eigen/Geometry>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#endif // EU_HAS_PCL

#include "Eudoxus/Eudoxus.h"
#include "Eudoxus/euUtils.h"
#include "Eudoxus/euOni.h"
#include "Eudoxus/euIo.h"
#include "Eudoxus/euPcd.h"
#include "Eudoxus/euPcs.h"

using namespace std;
using namespace xn;
#ifdef EU_HAS_PCL
using namespace pcl;
#endif // EU_HAS_PCL
using namespace eu;

#define RETURN_FRAME_ERROR(ecode, hdr, fmt, ...) \
  do \
  { \
    setErrorMsg("%s frame %zu: " fmt, \
      getMimeTypeStr((hdr).getMimeType()), (hdr).getFrameId(), ##__VA_ARGS__); \
    LOGERROR("%s", m_bufErrorMsg); \
    return (ecode) > 0? -(ecode): (ecode); \
  } while(0)

namespace eu
{
  typedef union
  {
    struct /*anonymous*/
    {
      unsigned char Blue;
      unsigned char Green;
      unsigned char Red;
      unsigned char Alpha;
    };
    float float_value;
    long long_value;
  } RGBValue;

#ifdef EU_HAS_PCL
  static float bad_point = std::numeric_limits<float>::quiet_NaN();
#endif // EU_HAS_PCL
} // namespace eu

// -----------------------------------------------------------------------------
// EuPcsHeader Class
// -----------------------------------------------------------------------------

/*!
 * \brief Default constructor.
 */
EuPcsHeader::EuPcsHeader() :
    EuHeader(EuHdrTypePcs, "pcs"),
    m_strVersion(EU_PCS_VERSION)
{
  m_eMimeType           = EuMimeTypeNone;
  m_eEndian             = BYTE_ORDER==BIG_ENDIAN? EuEndianBig: EuEndianLittle;
  m_uFrameId            = 0;
  m_tvTimeStamp.tv_sec  = 0;
  m_tvTimeStamp.tv_usec = 0;
}

/*!
 * \brief Destructor.
 */
EuPcsHeader::~EuPcsHeader()
{
}

ssize_t EuPcsHeader::pack(byte_t buf[], size_t sizeBuf)
{
  int   n;

  n = snprintf((char *)buf, sizeBuf, PCSHeaderFmt_0_8,
              // PointCloudStream
              m_strVersion.c_str(),
              // MimeType
              getMimeTypeStr(m_eMimeType),
              // Endian
              getEndianStr(m_eEndian),
              // FrameId
              m_uFrameId,
              // TimeStamp
              m_tvTimeStamp.tv_sec, m_tvTimeStamp.tv_usec,
              // DataSize
              EU_PCS_HDR_LEN_DATASIZE, m_uDataSize);

  // printf error
  if( n < 0 )
  {
    LOGERROR("Invalid PCS header value(s).");
    return -EU_ECODE_BAD_VAL;
  }

  // buffer too small so truncate
  else if( n >= sizeBuf )
  {
    LOGERROR("PCD header output buffer too small.");
    return -EU_ECODE_TOO_SMALL;
  }

  m_uHdrSize = n;

  return (ssize_t)n;
}

ssize_t EuPcsHeader::unpack(const byte_t buf[], size_t uLen)
 
{
  unsigned char  *p;
  size_t          n;
  size_t          uTot;
  char            bufTag[256];
  char            bufVal[256];
  int             nFields;

  m_strVersion.clear();
  m_eMimeType           = EuMimeTypeNone;
  m_eEndian             = EuEndianNone;
  m_uFrameId            = 0;
  m_tvTimeStamp.tv_sec  = 0;
  m_tvTimeStamp.tv_usec = 0;
  m_uDataSize           = 0;

  for(p=(unsigned char *)buf, n=0, uTot=0; 
      (n = EuPcsHeader::findLine(p, uLen-uTot)) > 0;
      p += n, uTot += n)
  {
    p[n-1] = 0;

    bufTag[0] = 0;
    bufVal[0] = 0;

    nFields = sscanf((const char *)p, "# @PCS %s %s", bufTag, bufVal);

    //fprintf(stderr, "DBG: Read %d fields: [%s], [%s]\n",
    //    nFields, bufTag, bufVal);

    if( nFields != 2 )
    {
      continue;
    }

    if( !strcmp(bufTag, "PointCloudStream:") )
    {
      m_strVersion = bufVal;
    } 
    else if( !strcmp(bufTag, "MimeType:") )
    {
      m_eMimeType = mapMimeTypeToEnum(bufVal);
    }
    else if( !strcmp(bufTag, "Endian:") )
    {
      m_eEndian = mapEndianToEnum(bufVal);
    }
    else if( !strcmp(bufTag, "FrameId:") )
    {
      sscanf(bufVal, "%zu", &m_uFrameId);
    }
    else if( !strcmp(bufTag, "TimeStamp:") )
    {
      sscanf(bufVal, "%zu.%zu", &m_tvTimeStamp.tv_sec, &m_tvTimeStamp.tv_usec);
    }
    else if( !strcmp(bufTag, "DataSize:") )
    {
      sscanf(bufVal, "%zu", &m_uDataSize);
    }
  }

  LOGDIAG3("%s %s %s %zu %zu.%zu %zu.",
        m_strVersion.c_str(), getMimeTypeStr(m_eMimeType),
        getEndianStr(m_eEndian), m_uFrameId,
        m_tvTimeStamp.tv_sec, m_tvTimeStamp.tv_usec, m_uDataSize);

  m_uHdrSize    = uTot;
  m_uTotalSize  = m_uHdrSize + m_uDataSize;

  return (ssize_t)uTot;
}

ssize_t EuPcsHeader::unpack(FILE *fp)
 
{
  size_t          n;
  size_t          uTot;
  char            bufLine[512];
  char            bufTag[256];
  char            bufVal[256];
  int             nFields;

  m_strVersion.clear();
  m_eMimeType           = EuMimeTypeNone;
  m_eEndian             = EuEndianNone;
  m_uFrameId            = 0;
  m_tvTimeStamp.tv_sec  = 0;
  m_tvTimeStamp.tv_usec = 0;
  m_uDataSize           = 0;

  for(uTot  = 0; 
      (n    = EuPcsHeader::findLine(fp, bufLine, sizeof(bufLine))) > 0;
      uTot += n)
  {
    bufLine[n-1] = 0;

    bufTag[0] = 0;
    bufVal[0] = 0;

    nFields = sscanf(bufLine, "# @PCS %s %s", bufTag, bufVal);

    //fprintf(stderr, "DBG: Read %d fields: [%s], [%s]\n",
    //    nFields, bufTag, bufVal);

    if( nFields != 2 )
    {
      continue;
    }

    if( !strcmp(bufTag, "PointCloudStream:") )
    {
      m_strVersion = bufVal;
    } 
    else if( !strcmp(bufTag, "MimeType:") )
    {
      m_eMimeType = mapMimeTypeToEnum(bufVal);
    }
    else if( !strcmp(bufTag, "Endian:") )
    {
      m_eEndian = mapEndianToEnum(bufVal);
    }
    else if( !strcmp(bufTag, "FrameId:") )
    {
      sscanf(bufVal, "%zu", &m_uFrameId);
    }
    else if( !strcmp(bufTag, "TimeStamp:") )
    {
      sscanf(bufVal, "%zu.%zu", &m_tvTimeStamp.tv_sec, &m_tvTimeStamp.tv_usec);
    }
    else if( !strcmp(bufTag, "DataSize:") )
    {
      sscanf(bufVal, "%zu", &m_uDataSize);
    }
  }

  LOGDIAG3("%s %s %s %zu %zu.%zu %zu.",
        m_strVersion.c_str(), getMimeTypeStr(m_eMimeType),
        getEndianStr(m_eEndian), m_uFrameId,
        m_tvTimeStamp.tv_sec, m_tvTimeStamp.tv_usec, m_uDataSize);

  m_uHdrSize    = uTot;
  m_uTotalSize  = m_uHdrSize + m_uDataSize;

  return (ssize_t)uTot;
}

ssize_t EuPcsHeader::patchDataSize(byte_t buf[],
                                   size_t sizeHdr,
                                   size_t sizeData)
{
  size_t    offset;
  ssize_t   n;

  // backup from end of header null, end-of-record, start of data size value
  offset = sizeHdr - EU_PCS_HDR_LEN_EOR - EU_PCS_HDR_LEN_DATASIZE;
  n = (ssize_t)sprintf((char *)buf+offset, "%*zu",
                      EU_PCS_HDR_LEN_DATASIZE, sizeData);
  buf[sizeHdr-EU_PCS_HDR_LEN_EOR] = '\n';
  return n;
}

size_t EuPcsHeader::findLine(const byte_t buf[], size_t uLen)
{
  size_t         n;

  if( (uLen == 0) || (buf[0] != '#') )
  {
    return 0;
  }

  for(n=0; n<uLen; ++n)
  {
    if( buf[n] == '\n' )
    {
      return n + 1;
    }
  }

  return n;
}

size_t EuPcsHeader::findLine(FILE *fp, char buf[], size_t sizeBuf)
{
  int     c;
  size_t  n;

  // peek
  c = fgetc(fp);
  ungetc(c, fp);

  if( c != '#' )
  {
    return 0;
  }

  // read line
  for(n=0; n<sizeBuf;)
  {
    switch( (c = fgetc(fp)) )
    {
      case EOF:
        return n;
      case '\n':
        buf[n++];
        return n;
      default:
        buf[n++] = c;
        break;
    }
  }

  return n;
}


// -----------------------------------------------------------------------------
// EuPcsFrame Class
// -----------------------------------------------------------------------------

EuPcsFrame::EuPcsFrame()
{
  int   i;

  m_nNumSections   = 0;

  for(i=0; i<MaxSections; ++i)
  {
    m_pHdrSection[i] = NULL;
  }

  m_bufErrorMsg[0] = 0;
}

EuPcsFrame::~EuPcsFrame()
{
  clearSections();
}

void EuPcsFrame::clearSections()
{
  int   i;

  for(i=0; i<m_nNumSections; ++i)
  {
    if( m_pHdrSection[i] != NULL )
    {
      delete m_pHdrSection[i];
      m_pHdrSection[i] = NULL;
    }
  }

  m_nNumSections = 0;
  m_uProdNodes   = (uint_t)EuProdNodeTypeNone;
}

/*!
 * \brief Set error message
 */
void EuPcsFrame::setErrorMsg(const char *sFmt, ...)
{
  va_list         ap;

  // format error message
  va_start(ap, sFmt);
  vsnprintf(m_bufErrorMsg, sizeof(m_bufErrorMsg), sFmt, ap);
  m_bufErrorMsg[sizeof(m_bufErrorMsg)-1] = 0;
  va_end(ap);
}

int EuPcsFrame::configure(EuMimeType    eMimeType,
                          uint_t        uProdNodes,
                          EuResolution &resolution,
                          EuFoV        &fovDepth,
                          EuFoV        &fovImage)
{
  struct timeval  tvZero = {0, 0};

  clearSections();

  m_uProdNodes     = uProdNodes;
  m_bufErrorMsg[0] = 0;

  m_hdrPcs.setParams(eMimeType, 0, tvZero);

  switch( eMimeType )
  {
    case EuMimeTypePcsOni:
      if( uProdNodes & (uint_t)EuProdNodeTypeDepth )
      {
        m_pHdrSection[m_nNumSections++] = new EuOniDepthNodeHeader(resolution,
                                                                  fovDepth);
      }
      if( uProdNodes & (uint_t)EuProdNodeTypeImage )
      {
        m_pHdrSection[m_nNumSections++] = new EuOniImageNodeHeader(resolution,
                                                                  fovImage);
      }
      if( uProdNodes & (uint_t)EuProdNodeTypeAudio )
      {
        m_pHdrSection[m_nNumSections++] = new EuOniAudioNodeHeader();
      }
      if( uProdNodes & (uint_t)EuProdNodeTypeImu )
      {
        m_pHdrSection[m_nNumSections++] = new EuOniImuNodeHeader();
      }
      break;

    case EuMimeTypePcsBinary:
      if( !(uProdNodes & (uint_t)EuProdNodeTypeDepth) )
      {
        LOGERROR("MimeType %s requires depth production node.",
            getMimeTypeStr(eMimeType));
        return -EU_ECODE_BAD_VAL;
      }
      if( uProdNodes & (uint_t)EuProdNodeTypeImage )
      {
        m_pHdrSection[m_nNumSections++] = 
                                        new EuPcdBinaryXyzRgbHeader(resolution);
      }
      else
      {
        m_pHdrSection[m_nNumSections++] = new EuPcdBinaryXyzHeader(resolution);
      }
      break;

    case EuMimeTypePcsAscii:
      if( !(uProdNodes & (uint_t)EuProdNodeTypeDepth) )
      {
        LOGERROR("MimeType %s requires depth production node.",
            getMimeTypeStr(eMimeType));
        return -EU_ECODE_BAD_VAL;
      }
      if( uProdNodes & (uint_t)EuProdNodeTypeImage )
      {
        m_pHdrSection[m_nNumSections++] =
                                        new EuPcdAsciiXyzRgbHeader(resolution);
      }
      else
      {
        m_pHdrSection[m_nNumSections++] = new EuPcdAsciiXyzHeader(resolution);
      }
      break;

    case EuMimeTypeOni:
    default:
      LOGERROR("MimeType %s not supported or unknown.",
            getMimeTypeStr(eMimeType));
      return -EU_ECODE_INTERNAL;
  }

  return EU_OK;
}

size_t EuPcsFrame::getMaxFrameSize()
{
  size_t  n;
  int     i;

  n = m_hdrPcs.getMaxHdrSize();

  for(i=0; i<m_nNumSections; ++i)
  {
    n += m_pHdrSection[i]->getMaxTotalSize();
  }

  return n;
}

ssize_t EuPcsFrame::insertFrame(DepthGenerator &xnGenDepth,
                                ImageGenerator &xnGenImage,
                                byte_t          buf[],
                                size_t          sizeBuf)
{
  struct timeval  tvNow;
  ssize_t         m, n, k;
  int             i;

  //
  // Set PCS header frame by frame variable parameters
  //
  gettimeofday(&tvNow, NULL);
  m_hdrPcs.setTimeStamp(tvNow);
  
  //
  // Pack PCS header.
  //
  if( (m = m_hdrPcs.pack(buf, sizeBuf)) < 0 )
  {
    RETURN_FRAME_ERROR(m, m_hdrPcs, "Failed to pack %s header: %s(%d).",
        m_hdrPcs.getHdrName().c_str(), getStrError((int)m), (int)m);
  }

  //
  // Insert frame section(s).
  //
  switch( m_hdrPcs.getMimeType() )
  {
    case EuMimeTypePcsOni:
      for(i=0, n=0; i<m_nNumSections; ++i)
      {
        switch( m_pHdrSection[i]->getHdrType() )
        {
          case EuHdrTypeOniDepth:
            k = insertSectionOniDepth(m_pHdrSection[i], xnGenDepth,
                                      buf+m+n, sizeBuf-(size_t)(m+n));
            break;
          case EuHdrTypeOniImage:
            k = insertSectionOniImage(m_pHdrSection[i], xnGenImage,
                                      buf+m+n, sizeBuf-(size_t)(m+n));
            break;
          case EuHdrTypeOniImu:
          case EuHdrTypeOniAudio:
          default:
            k = 0; // ignore others for now
            break;
        }
        if( k < 0 )
        {
          return k;
        }
        n += k;
      }
      break;

    case EuMimeTypePcsBinary:
      n = insertSectionPcdBinary(m_pHdrSection[0], xnGenDepth, xnGenImage,
                                 buf+m, sizeBuf-(size_t)m);
      break;

    case EuMimeTypePcsAscii:
      n = insertSectionPcdAscii(m_pHdrSection[0], xnGenDepth, xnGenImage,
                                buf+m, sizeBuf-(size_t)m);
      break;

    case EuMimeTypeOni:  // TODO
    default:
      RETURN_FRAME_ERROR(EU_ECODE_INTERNAL, m_hdrPcs,
          "Unsupported/unknown MimeType %d.", m_hdrPcs.getMimeType());
      break;
  }

  if( n < 0 )
  {
    return n;
  }

  m_hdrPcs.patchDataSize(buf, (size_t)m, (size_t)n);

  m_hdrPcs.bumpFrameId();

  return m+n;
}

ssize_t EuPcsFrame::insertSectionOniDepth(EuHeader       *pHdr,
                                          DepthGenerator &xnGenDepth,
                                          byte_t          buf[],
                                          size_t          sizeBuf)
{
  size_t              uDepthSize;
  const XnDepthPixel *pDepthMap;
  ssize_t             m;

  if( (m = pHdr->pack(buf, sizeBuf)) < 0 )
  {
    RETURN_FRAME_ERROR(m, m_hdrPcs, "Failed to pack %s section header: %s(%d).",
        pHdr->getHdrName().c_str(), getStrError((int)m), (int)m);
  }

  else if( (uDepthSize = (size_t)xnGenDepth.GetDataSize()) == 0 )
  {
    RETURN_FRAME_ERROR(0, m_hdrPcs, "Depth data length is zero.");
  }

  else if( ((size_t)m + uDepthSize) > sizeBuf )
  {
    RETURN_FRAME_ERROR(EU_ECODE_TOO_SMALL, m_hdrPcs,
        "Buffer size=%zu too small to hold depth section size=%zu.",
        sizeBuf, (size_t)m+uDepthSize);
  }

  else if( (pDepthMap = xnGenDepth.GetDepthMap()) == NULL )
  {
    RETURN_FRAME_ERROR(0, m_hdrPcs, "Depth data is null.");
  }

  // depth node data
  memcpy(buf+m, pDepthMap, uDepthSize);

  return m + (ssize_t)uDepthSize;
}

ssize_t EuPcsFrame::insertSectionOniImage(EuHeader       *pHdr,
                                          ImageGenerator &xnGenImage,
                                          byte_t          buf[],
                                          size_t          sizeBuf)
{
  uint_t              uImageSize;
  const XnRGB24Pixel *pImageMap;
  ssize_t             m;

  if( (m = pHdr->pack(buf, sizeBuf)) < 0 )
  {
    RETURN_FRAME_ERROR(m, m_hdrPcs, "Failed to pack %s section header: %s(%d).",
        pHdr->getHdrName().c_str(), getStrError((int)m), (int)m);
  }

  else if( (uImageSize = (size_t)xnGenImage.GetDataSize()) == 0 )
  {
    RETURN_FRAME_ERROR(0, m_hdrPcs, "Image data length is zero.");
  }

  else if( ((size_t)m + uImageSize) > sizeBuf )
  {
    RETURN_FRAME_ERROR(EU_ECODE_TOO_SMALL, m_hdrPcs,
        "Buffer size=%zu too small to hold image section size=%zu.",
        sizeBuf, (size_t)m+uImageSize);
  }

  else if( (pImageMap = xnGenImage.GetRGB24ImageMap()) == NULL )
  {
    RETURN_FRAME_ERROR(0, m_hdrPcs, "Image data is null.");
  }

  // image node data
  memcpy(buf+m, pImageMap, uImageSize);

  return m + (ssize_t)uImageSize;
}

ssize_t EuPcsFrame::insertSectionPcdBinary(EuHeader       *pHdr,
                                           DepthGenerator &xnGenDepth,
                                           ImageGenerator &xnGenImage,
                                           byte_t          buf[],
                                           size_t          sizeBuf)
{
  DepthMetaData       xnMetaDepth;
  uint_t              uDepthSize;
  const XnDepthPixel *pDepthMap;
  ImageMetaData       xnMetaImage;
  uint_t              uImageSize;
  const XnRGB24Pixel *pImageMap;
  uint_t              x, y, z, index;
  uint_t              x_max, y_max;
  EuPoint3D           ptProjective;
  EuPoint3D           ptRealWorld;
  XnRGB24Pixel        rgb;
  ssize_t             n;

  xnGenDepth.GetMetaData(xnMetaDepth);

  if( (uDepthSize = (size_t)xnGenDepth.GetDataSize()) == 0 )
  {
    RETURN_FRAME_ERROR(0, m_hdrPcs, "Depth data length is zero.");
  }

  else if( (pDepthMap = xnGenDepth.GetDepthMap()) == NULL )
  {
    RETURN_FRAME_ERROR(0, m_hdrPcs, "Depth data is null.");
  }

  if( m_uProdNodes & EuProdNodeTypeImage )
  {
    xnGenImage.GetMetaData(xnMetaImage);

    if( (uImageSize = (size_t)xnGenImage.GetDataSize()) == 0 )
    {
      RETURN_FRAME_ERROR(0, m_hdrPcs, "Image data length is zero.");
    }

    else if( (pImageMap = xnGenImage.GetRGB24ImageMap()) == NULL )
    {
      RETURN_FRAME_ERROR(0, m_hdrPcs, "Image data is null.");
    }
  }

  // both depth and image devices are kept in sync at the same resolution
  x_max = xnMetaDepth.XRes();
  y_max = xnMetaDepth.YRes();

  if( (n = pHdr->pack(buf, sizeBuf)) < 0 )
  {
    RETURN_FRAME_ERROR(n, m_hdrPcs,
        "Failed to pack %s section header: %s(%d).",
        pHdr->getHdrName().c_str(), getStrError((int)n), (int)n);
  }

  // column major for some reason
  for(y=0, index=0; y<y_max; ++y)
  {
    for(x=0; x<x_max; ++x, ++index)
    {
      if( (n + pHdr->getRecordSize()) <= sizeBuf )
      {
        z = (uint_t)pDepthMap[index];

        if( z > 0 )
        {
          ptProjective.X  = x;
          ptProjective.Y  = y;
          ptProjective.Z  = z;

          xnGenDepth.ConvertProjectiveToRealWorld(1, &ptProjective,
                                                     &ptRealWorld);

          n += pack32f(ptRealWorld.X, buf+n);
          n += pack32f(ptRealWorld.Y, buf+n);
          n += pack32f(ptRealWorld.Z, buf+n);
        }
        else
        {
          n += pack32f(NAN, buf+n);
          n += pack32f(NAN, buf+n);
          n += pack32f(NAN, buf+n);
        }

        if( m_uProdNodes & EuProdNodeTypeImage )
        {
          n += packRGB(pImageMap[index], buf+n);
        }
      }
      else
      {
        RETURN_FRAME_ERROR(EU_ECODE_TOO_SMALL, m_hdrPcs,
            "Buffer size=%zu too small.", sizeBuf);
      }
    }
  }

  return n;
}

ssize_t EuPcsFrame::insertSectionPcdAscii(EuHeader       *pHdr,
                                          DepthGenerator &xnGenDepth,
                                          ImageGenerator &xnGenImage,
                                          byte_t          buf[],
                                          size_t          sizeBuf)
{
  DepthMetaData       xnMetaDepth;
  uint_t              uDepthSize;
  const XnDepthPixel *pDepthMap;
  ImageMetaData       xnMetaImage;
  uint_t              uImageSize;
  const XnRGB24Pixel *pImageMap;
  uint_t              x, y, z, index;
  uint_t              x_max, y_max;
  EuPoint3D           ptProjective;
  EuPoint3D           ptRealWorld;
  XnRGB24Pixel        rgb;
  uint_t              uRGB;
  ssize_t             n;

  xnGenDepth.GetMetaData(xnMetaDepth);

  if( (uDepthSize = (size_t)xnGenDepth.GetDataSize()) == 0 )
  {
    RETURN_FRAME_ERROR(0, m_hdrPcs, "Depth data length is zero.");
  }

  else if( (pDepthMap = xnGenDepth.GetDepthMap()) == NULL )
  {
    RETURN_FRAME_ERROR(0, m_hdrPcs, "Depth data is null.");
  }

  if( m_uProdNodes & EuProdNodeTypeImage )
  {
    xnGenImage.GetMetaData(xnMetaImage);

    if( (uImageSize = (size_t)xnGenImage.GetDataSize()) == 0 )
    {
      RETURN_FRAME_ERROR(0, m_hdrPcs, "Image data length is zero.");
    }

    else if( (pImageMap = xnGenImage.GetRGB24ImageMap()) == NULL )
    {
      RETURN_FRAME_ERROR(0, m_hdrPcs, "Image data is null.");
    }
  }

  // both depth and image devices are kept in sync at the same resolution
  x_max = xnMetaDepth.XRes();
  y_max = xnMetaDepth.YRes();

  if( (n = pHdr->pack(buf, sizeBuf)) < 0 )
  {
    RETURN_FRAME_ERROR(n, m_hdrPcs, "Failed to pack %s section header: %s(%d).",
        pHdr->getHdrName().c_str(), getStrError((int)n), (int)n);
  }

  // column major for some reason
  for(y=0, index=0; y<y_max; ++y)
  {
    for(x=0; x<x_max; ++x, ++index)
    {
      if( (n + pHdr->getRecordSize()) <= sizeBuf )
      {
        z = (uint_t)pDepthMap[index];

        if( z > 0 )
        {
          ptProjective.X  = x;
          ptProjective.Y  = y;
          ptProjective.Z  = z;

          xnGenDepth.ConvertProjectiveToRealWorld(1, &ptProjective,
                                                     &ptRealWorld);

          n += sprintf((char *)buf+n, PCDAsciiDataRecordFmtXYZ,
                      EU_PCD_ASC_FIELD_FLOAT_WIDTH, ptRealWorld.X,
                      EU_PCD_ASC_FIELD_FLOAT_WIDTH, ptRealWorld.Y,
                      EU_PCD_ASC_FIELD_FLOAT_WIDTH, ptRealWorld.Z);
        }
        else
        {
          n += sprintf((char *)buf+n, PCDAsciiDataRecordFmtNaN,
                      EU_PCD_ASC_FIELD_FLOAT_WIDTH, "nan",
                      EU_PCD_ASC_FIELD_FLOAT_WIDTH, "nan",
                      EU_PCD_ASC_FIELD_FLOAT_WIDTH, "nan");
        }

        if( m_uProdNodes & EuProdNodeTypeImage )
        {
          rgb = pImageMap[index];

          uRGB = (uint_t)( ((uint_t)rgb.nRed << 16) |
                           ((uint_t)rgb.nGreen << 8) |
                           ((uint_t)rgb.nBlue) );

          n += sprintf((char *)buf+n, PCDAsciiDataRecordFmtRGB,
                      EU_PCD_ASC_FIELD_INT_WIDTH, uRGB);
          
        }

        n += sprintf((char *)buf+n, PCDAsciiDataRecordFmtEoR);
      }
      else
      {
        RETURN_FRAME_ERROR(EU_ECODE_TOO_SMALL, m_hdrPcs,
            "Buffer size=%zu too small.", sizeBuf);
      }
    }
  }

  return n;
}

#ifdef EU_HAS_PCL
ssize_t EuPcsFrame::extractFrameToPointCloud(byte_t buf[],
                                             size_t sizeBuf,
                                             PCLPointCloud2 &cloud,
                                             Eigen::Vector4f &origin,
                                             Eigen::Quaternionf &orientation)
{
  //int        nVersion;
  byte_t    *pBufTmp;
  ssize_t    m, n;

  //
  // Old frame state
  //
  clearSections();

  //
  // Unpack PCS header.
  //
  if( (m = m_hdrPcs.unpack(buf, sizeBuf)) < 0 )
  {
    RETURN_FRAME_ERROR(m, m_hdrPcs, "Failed to unpack %s header: %s(%d).",
        m_hdrPcs.getHdrName().c_str(), getStrError((int)m), (int)m);
  }

  if( m_hdrPcs.getDataSize() == 0 )
  {
    RETURN_FRAME_ERROR(m, m_hdrPcs, "No data in frame.");
  }

  switch( m_hdrPcs.getMimeType() )
  {
    case EuMimeTypePcsAscii:
    case EuMimeTypePcsBinary:
      //n = EuPcdIo::readbuf(buf+m, sizeBuf-(size_t)m,
      //                     cloud, origin, orientation, nVersion);
      n = io::loadPCDBuf(buf+m, sizeBuf-(size_t)m, cloud, origin, orientation);
      if( n < 0 )
      {
        RETURN_FRAME_ERROR(EU_ECODE_PCL, m_hdrPcs,
            "Failed to extract PCD section to point cloud.");
      }
      break;
    case EuMimeTypePcsOni:
      n = extractOniFrameToPcdBinary(buf+m, sizeBuf-(size_t)m, pBufTmp);
      if( n < 0 )
      {
        return n;
      }
      //n = EuPcdIo::readbuf(pBufTmp, n, cloud, origin, orientation, nVersion);
      n = io::loadPCDBuf(pBufTmp, n, cloud, origin, orientation);
      delete[] pBufTmp;
      if( n < 0 )
      {
        RETURN_FRAME_ERROR(EU_ECODE_PCL, m_hdrPcs,
            "Failed to extract PCD section to point cloud.");
      }
      break;
    default:
      RETURN_FRAME_ERROR(EU_ECODE_INTERNAL, m_hdrPcs,
          "Unsupported/unknown MimeType %d.", m_hdrPcs.getMimeType());
  }

  return m+n;
}

#endif // EU_HAS_PCL

ssize_t EuPcsFrame::extractOniFrameToPcdBinary(byte_t buf[],
                                               size_t sizeBuf,
                                               byte_t *&pBufOut)
{
  EuHdrType             eHdrType;
  EuOniDepthNodeHeader  hdrDepth;
  EuOniImageNodeHeader  hdrImage;
  EuOniAudioNodeHeader  hdrAudio;
  EuOniImuNodeHeader    hdrImu;
  EuResolution          resolution;
  EuFoV                 fov;
  byte_t               *pDepthMap;
  byte_t               *pImageMap;
  EuPcdHeader          *pHdrPcd;
  size_t                sizeBufOut;
  double                fRealWorldXtoZ, fRealWorldYtoZ;
  uint_t                x, y, z, index, index2;
  uint_t                x_max, y_max;
  EuPoint3D             ptProjective;
  EuPoint3D             ptRealWorld;
  XnRGB24Pixel          rgb;
  ssize_t               m, n, k;
 
  m_uProdNodes = (uint_t)EuProdNodeTypeNone;

  //
  // Jump through frame unpacking section headers and saving header parameters
  // and data locations.
  //
  for(n=0; n<sizeBuf; )
  {
    eHdrType = EuOniNodeHeader::peekHdrType(buf+n, sizeBuf-(size_t)n);

    switch( eHdrType )
    {
      case EuHdrTypeOniDepth:    // depth section
        if( (m = hdrDepth.unpack(buf+n, sizeBuf-(size_t)n)) < 0 )
        {
          RETURN_FRAME_ERROR(m, m_hdrPcs,
              "Failed to unpack %s section header: %s(%d).",
              hdrDepth.getHdrName().c_str(), getStrError((int)m), (int)m);
        }

        if( (k = (ssize_t)hdrDepth.getDataSize()) == 0 )
        {
          setErrorMsg("Frame %s depth section has no data.",
            getMimeTypeStr(m_hdrPcs.getMimeType()));
          LOGERROR("%s", m_bufErrorMsg);
          return -EU_ECODE_BAD_FMT;
        }

        resolution    = hdrDepth.getResolution();
        fov           = hdrDepth.getFieldOfView();
        m_uProdNodes |= (uint_t)EuProdNodeTypeDepth;

        //pDepthMap = (XnDepthPixel *)buf+n+m;
        pDepthMap = buf+n+m;

        n += m + k;
        break;

      case EuHdrTypeOniImage:    // image section
        if( (m = hdrImage.unpack(buf+n, sizeBuf-(size_t)n)) < 0 )
        {
          RETURN_FRAME_ERROR(m, m_hdrPcs,
              "Failed to unpack %s section header: %s(%d).",
              hdrImage.getHdrName().c_str(), getStrError((int)m), (int)m);
        }

        if( (k = (ssize_t)hdrImage.getDataSize()) > 0 )
        {
          m_uProdNodes |= (uint_t)EuProdNodeTypeImage;
          //pImageMap = (XnRGB24Pixel *)buf+n+m;
          pImageMap = buf+n+m;
        }

        n += m + k;
        break;

      case EuHdrTypeOniAudio:    // audio section
        if( (m = hdrAudio.unpack(buf+n, sizeBuf-(size_t)n)) < 0 )
        {
          RETURN_FRAME_ERROR(m, m_hdrPcs,
              "Failed to unpack %s section header: %s(%d).",
              hdrAudio.getHdrName().c_str(), getStrError((int)m), (int)m);
        }

        if( (k = (ssize_t)hdrAudio.getDataSize()) > 0 )
        {
          // TODO
        }

        n += m + k;
        break;

      case EuHdrTypeOniImu:    // imu section
        if( (m = hdrImu.unpack(buf+n, sizeBuf-(size_t)n)) < 0 )
        {
          RETURN_FRAME_ERROR(m, m_hdrPcs,
              "Failed to unpack %s section header: %s(%d).",
              hdrImu.getHdrName().c_str(), getStrError((int)m), (int)m);
        }

        if( (k = (ssize_t)hdrImu.getDataSize()) > 0 )
        {
          // TODO
        }

        n += m + k;
        break;

      default:                // unknown
        RETURN_FRAME_ERROR(EU_ECODE_BAD_FMT, m_hdrPcs,
              "Unexpected section header type: %d.", eHdrType);
    }
  }

  // Check frame sizes.
  if( n != sizeBuf )
  {
    RETURN_FRAME_ERROR(EU_ECODE_BAD_FMT, m_hdrPcs,
        "Calculated size=%zu != to received size=%zu.", n, sizeBuf);
  }

  // Check for required depth section.
  if( !(m_uProdNodes & (uint_t)EuProdNodeTypeDepth) )
  {
    RETURN_FRAME_ERROR(EU_ECODE_BAD_FMT, m_hdrPcs,
        "Required depth data missing.");
  }

  //
  // Allocate PCD header of the required flavor.
  //
  if( m_uProdNodes & EuProdNodeTypeImage )
  {
    pHdrPcd = new EuPcdBinaryXyzRgbHeader(resolution);
  }
  else
  {
    pHdrPcd = new EuPcdBinaryXyzHeader(resolution);
  }

  // Allocate output buffer.
  sizeBufOut = pHdrPcd->getMaxTotalSize();
  pBufOut    = new byte_t[sizeBufOut];

  //
  // Pack PCD header.
  //
  if( (n = pHdrPcd->pack(pBufOut, sizeBufOut)) < 0 )
  {
    string strHdrName = pHdrPcd->getHdrName();
    delete[] pBufOut;
    delete   pHdrPcd;
    RETURN_FRAME_ERROR(n, m_hdrPcs,
        "Failed to pack %s section header: %s(%d).",
        strHdrName.c_str(), getStrError((int)n), (int)n);
  }

  // Calculate sensor real world parameters
  setSensorFieldOfViewParams(fov, fRealWorldXtoZ, fRealWorldYtoZ);

  //
  // Pack PCD data secion. Note column major arrangement.
  //
  for(y=0, index=0; y<resolution.m_uHeight; ++y)
  {
    for(x=0; x<resolution.m_uWidth; ++x, ++index)
    {
      if( (n + pHdrPcd->getRecordSize()) > sizeBufOut )
      {
        delete[] pBufOut;
        delete   pHdrPcd;
        RETURN_FRAME_ERROR(EU_ECODE_TOO_SMALL, m_hdrPcs,
            "Buffer size=%zu too small.", sizeBufOut);
      }

      //z = (uint_t)pDepthMap[index];
      pDepthMap += unpack16Little(pDepthMap, z);

      if( z > 0 )
      {
        ptProjective.X  = x;
        ptProjective.Y  = y;
        ptProjective.Z  = z;

        convertSensorProjectiveToRealWorld(ptProjective,
                                           fRealWorldXtoZ,
                                           fRealWorldYtoZ,
                                           resolution,
                                           ptRealWorld);

        n += pack32f(ptRealWorld.X, pBufOut+n);
        n += pack32f(ptRealWorld.Y, pBufOut+n);
        n += pack32f(ptRealWorld.Z, pBufOut+n);
      }
      else
      {
        n += pack32f(NAN, pBufOut+n);
        n += pack32f(NAN, pBufOut+n);
        n += pack32f(NAN, pBufOut+n);
      }

      if( m_uProdNodes & EuProdNodeTypeImage )
      {
        pBufOut[n+3] = 0;
        pBufOut[n+2] = *pImageMap++;
        pBufOut[n+1] = *pImageMap++;
        pBufOut[n]   = *pImageMap++;
        n += 4;
      }
    }
  }

  delete pHdrPcd;

  return n;
}

#if defined(EU_HAS_PCL) && defined(EU_FUTURE)
ssize_t EuPcsFrame::extractOniFrameToPointCloud(
                                            byte_t buf[],
                                            size_t sizeBuf,
                                            PCLPointCloud2 &cloud,
                                            Eigen::Vector4f &origin,
                                            Eigen::Quaternionf &orientation)
{
  EuHdrType             eHdrType;
  EuOniDepthNodeHeader  hdrDepth;
  EuOniImageNodeHeader  hdrImage;
  EuOniAudioNodeHeader  hdrAudio;
  EuOniImuNodeHeader    hdrImu;
  EuResolution          resolution;
  EuFoV                 fov;
  byte_t               *pDepthMap;
  byte_t               *pImageMap;
  double                fRealWorldXtoZ, fRealWorldYtoZ;
  EuPoint3D             ptProjective;
  EuPoint3D             ptRealWorld;
  int                   x, y;
  uint_t                z;
  uint_t                index;
  RGBValue              color;
  ssize_t               m, n, k;
 
  m_uProdNodes = (uint_t)EuProdNodeTypeNone;

  //
  // Jump through frame unpacking section headers and saving header parameters
  // and data locations.
  //
  for(n=0; n<sizeBuf; )
  {
    eHdrType = EuOniNodeHeader::peekHdrType(buf+n, sizeBuf-(size_t)n);

    switch( eHdrType )
    {
      case EuHdrTypeOniDepth:    // depth section
        if( (m = hdrDepth.unpack(buf+n, sizeBuf-(size_t)n)) < 0 )
        {
          RETURN_FRAME_ERROR(m, m_hdrPcs,
              "Failed to unpack %s section header: %s(%d).",
              hdrDepth.getHdrName().c_str(), getStrError((int)m), (int)m);
        }

        if( (k = (ssize_t)hdrDepth.getDataSize()) == 0 )
        {
          setErrorMsg("Frame %s depth section has no data.",
            getMimeTypeStr(m_hdrPcs.getMimeType()));
          LOGERROR("%s", m_bufErrorMsg);
          return -EU_ECODE_BAD_FMT;
        }

        resolution    = hdrDepth.getResolution();
        fov           = hdrDepth.getFieldOfView();
        m_uProdNodes |= (uint_t)EuProdNodeTypeDepth;

        //pDepthMap = (XnDepthPixel *)buf+n+m;
        pDepthMap = buf+n+m;

        n += m + k;
        break;

      case EuHdrTypeOniImage:    // image section
        if( (m = hdrImage.unpack(buf+n, sizeBuf-(size_t)n)) < 0 )
        {
          RETURN_FRAME_ERROR(m, m_hdrPcs,
              "Failed to unpack %s section header: %s(%d).",
              hdrImage.getHdrName().c_str(), getStrError((int)m), (int)m);
        }

        if( (k = (ssize_t)hdrImage.getDataSize()) > 0 )
        {
          m_uProdNodes |= (uint_t)EuProdNodeTypeImage;
          //pImageMap = (XnRGB24Pixel *)buf+n+m;
          pImageMap = buf+n+m;
        }

        n += m + k;
        break;

      case EuHdrTypeOniAudio:    // audio section
        if( (m = hdrAudio.unpack(buf+n, sizeBuf-(size_t)n)) < 0 )
        {
          RETURN_FRAME_ERROR(m, m_hdrPcs,
              "Failed to unpack %s section header: %s(%d).",
              hdrAudio.getHdrName().c_str(), getStrError((int)m), (int)m);
        }

        if( (k = (ssize_t)hdrAudio.getDataSize()) > 0 )
        {
          // TODO
        }

        n += m + k;
        break;

      case EuHdrTypeOniImu:    // imu section
        if( (m = hdrImu.unpack(buf+n, sizeBuf-(size_t)n)) < 0 )
        {
          RETURN_FRAME_ERROR(m, m_hdrPcs,
              "Failed to unpack %s section header: %s(%d).",
              hdrImu.getHdrName().c_str(), getStrError((int)m), (int)m);
        }

        if( (k = (ssize_t)hdrImu.getDataSize()) > 0 )
        {
          // TODO
        }

        n += m + k;
        break;

      default:                // unknown
        RETURN_FRAME_ERROR(EU_ECODE_BAD_FMT, m_hdrPcs,
              "Unexpected section header type: %d.", eHdrType);
    }
  }

  // Check frame sizes.
  if( n != sizeBuf )
  {
    RETURN_FRAME_ERROR(EU_ECODE_BAD_FMT, m_hdrPcs,
        "Calculated size=%zu != to received size=%zu.", n, sizeBuf);
  }

  // Check for required depth section.
  else if( !(m_uProdNodes & (uint_t)EuProdNodeTypeDepth) )
  {
    RETURN_FRAME_ERROR(EU_ECODE_BAD_FMT, m_hdrPcs,
        "Required depth data missing.");
  }

  // Check for required image section.
  else if( !(m_uProdNodes & (uint_t)EuProdNodeTypeImage) )
  {
    RETURN_FRAME_ERROR(EU_ECODE_BAD_FMT, m_hdrPcs,
        "Required iamge data missing.");
  }

  // Calculate sensor real world parameters
  setSensorFieldOfViewParams(fov, fRealWorldXtoZ, fRealWorldYtoZ);

  cloud.header.frame_id  = m_hdrPcs.getFrameId();
  cloud.height           = resolution.m_uHeight;
  cloud.width            = resolution.m_uWidth;
  cloud.is_dense         = false;

  cloud.data.resize(cloud.height * cloud.width);
  
  color.Alpha = 0;

float zmin=100000000;
float zmax=-100000000;
  for(y=cloud.height-1, index=0; y>=0; --y)
  {
    for(x=cloud.width-1; x>=0; --x, ++index)
    {
      pcl::PointXYZRGB &pt = cloud.data[index];

      pDepthMap += unpack16Little(pDepthMap, z);

      // Check for invalid measurements
      if( z == 0 )
      {
        pt.x = pt.y = pt.z = bad_point;
      }
      else
      {
        ptProjective.X  = x;
        ptProjective.Y  = y;
        ptProjective.Z  = z;

        convertSensorProjectiveToRealWorld(ptProjective,
                                           fRealWorldXtoZ,
                                           fRealWorldYtoZ,
                                           resolution,
                                           ptRealWorld);

        pt.x = ptRealWorld.X;
        pt.y = ptRealWorld.Y;
        pt.z = ptRealWorld.Z;
if( pt.z < zmin ) zmin = pt.z;
if( pt.z > zmax ) zmax = pt.z;
      }

      // Fill in color
      color.Red   = *pImageMap++;
      color.Green = *pImageMap++;
      color.Blue  = *pImageMap++;

      pt.rgb = color.float_value;
    }
  }

  return n;
}
#endif // defined(EU_HAS_PCL) && defined(EU_FUTURE)

//
// EuPcsFrame::extractFrameToBuf
//
ssize_t EuPcsFrame::extractFrameToBuf(FILE   *fp,
                                      byte_t  buf[],
                                      size_t  sizeBuf,
                                      bool    bExtractHdr)
{
  int        nVersion;
  byte_t    *pBufTmp;
  ssize_t    m;
  size_t     n, k;

  //
  // Old frame state
  //
  clearSections();

  if( bExtractHdr )
  {
    if( (m = extractPcsHeader(fp)) < 0 )
    {
      RETURN_FRAME_ERROR(m, m_hdrPcs, "Failed to unpack %s header: %s(%d).",
        m_hdrPcs.getHdrName().c_str(), getStrError((int)m), (int)m);
    }
  }

  if( (k = m_hdrPcs.getDataSize()) == 0 )
  {
    return 0;
  }

  n = getMaxTotalSize();

  if( n > sizeBuf )
  {
    RETURN_FRAME_ERROR(-EU_ECODE_TOO_SMALL, m_hdrPcs,
        "Buffer size=%zu too small to hold frame size=%zu.",
        sizeBuf, n);
  }

  m = m_hdrPcs.pack(buf, sizeBuf);

  if( fread(buf+m, sizeof(byte_t), k, fp) != k )
  {
    RETURN_FRAME_ERROR(-EU_ECODE_TOO_SMALL, m_hdrPcs,
        "Failed to read %zu byte frame data.", k);
  }

  return m+k;
}
