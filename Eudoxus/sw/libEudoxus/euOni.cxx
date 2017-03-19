////////////////////////////////////////////////////////////////////////////////
//
// Package:   Eudoxus
//
// Library:   libEudoxus
//
// File:      euOni.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-11-30 15:41:20 -0700 (Mon, 30 Nov 2015) $
 * $Rev: 4226 $
 *
 * \brief Eudoxus OpenNI definitions and helper functions.
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
#include <string.h>
#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "XnTypes.h"

#include "Eudoxus/Eudoxus.h"
#include "Eudoxus/euHeader.h"
#include "Eudoxus/euOni.h"
#include "Eudoxus/euUtils.h"

using namespace std;
using namespace eu;



// -----------------------------------------------------------------------------
// PcsOniNodeHeader Class
// -----------------------------------------------------------------------------

ssize_t EuOniNodeHeader::pack(byte_t buf[], size_t sizeBuf)
{
  ssize_t n;

  if( sizeBuf < EU_ONI_SHARED_NODE_HDR_SIZE )
  {
    LOGERROR("Buffer size=%zu too small to hold %zu byte packed header.",
        sizeBuf, EU_ONI_SHARED_NODE_HDR_SIZE);
    return -EU_ECODE_TOO_SMALL;
  }

  n  = pack16(EU_ONI_NODE_HDR_START_CODE, buf+EU_ONI_NODE_HDR_IDX_START_CODE);
  n += pack8(m_eHdrType, buf+EU_ONI_NODE_HDR_IDX_HDR_TYPE);
  n += pack8(m_eDataFormat, buf+EU_ONI_NODE_HDR_IDX_DATA_FORMAT);
  n += pack32(m_uDataSize, buf+EU_ONI_NODE_HDR_IDX_DATA_SIZE);

  return n;
}

ssize_t EuOniNodeHeader::unpack(const byte_t buf[], size_t uLen)
{
  uint_t    uStartCode;
  ssize_t   n;
  uint_t    val;

  if( uLen < EU_ONI_SHARED_NODE_HDR_SIZE )
  {
    LOGERROR("Buffer data length=%zu too small to hold %zu byte packed header.",
        uLen,EU_ONI_SHARED_NODE_HDR_SIZE); 
    return -EU_ECODE_TOO_SMALL;
  }

  n = unpack16(buf+EU_ONI_NODE_HDR_IDX_START_CODE, uStartCode);

  if( uStartCode != EU_ONI_NODE_HDR_START_CODE )
  {
    LOGERROR("Bad header start code=0x%04x, expected 0x%04x.",
        uStartCode, EU_ONI_NODE_HDR_START_CODE);
    return -EU_ECODE_BAD_VAL;
  }

  n += unpack8(buf+EU_ONI_NODE_HDR_IDX_HDR_TYPE, val);
  m_eHdrType = (EuHdrType)val;
  n += unpack8(buf+EU_ONI_NODE_HDR_IDX_DATA_FORMAT, val);
  m_eDataFormat = (EuOniDataFormat)val;
  n += unpack32(buf+EU_ONI_NODE_HDR_IDX_DATA_SIZE, m_uDataSize);

  return n;
}

EuHdrType EuOniNodeHeader::peekHdrType(byte_t buf[], size_t uLen)
{
  uint_t val;

  if( uLen < EU_ONI_SHARED_NODE_HDR_SIZE )
  {
    return EuHdrTypeNone;
  }

  unpack8(buf+EU_ONI_NODE_HDR_IDX_HDR_TYPE, val);

  return (EuHdrType)val;
}


// -----------------------------------------------------------------------------
// PcsOniDepthNodeHeader Class
// -----------------------------------------------------------------------------

EuOniDepthNodeHeader::EuOniDepthNodeHeader() :
    EuOniNodeHeader(EuHdrTypeOniDepth,
                    "oni_depth",
                    EuOniDataFormatDepthProjective)
{
  m_uHdrSize              = EU_ONI_DEPTH_NODE_HDR_SIZE;
  m_uRecordSize           = sizeof(EuPointDepth);
  m_fov.fHFOV             = 0.0;
  m_fov.fVFOV             = 0.0;
  m_resolution.m_uWidth   = 0;
  m_resolution.m_uHeight  = 0;
}

EuOniDepthNodeHeader::EuOniDepthNodeHeader(EuResolution &resolution, EuFoV &fov)
  : EuOniNodeHeader(EuHdrTypeOniDepth,
                    "oni_depth",
                    EuOniDataFormatDepthProjective)
{
  m_uHdrSize      = EU_ONI_DEPTH_NODE_HDR_SIZE;
  m_uRecordSize   = sizeof(EuPointDepth);

  setFieldOfView(fov);
  setResolution(resolution);
}

ssize_t EuOniDepthNodeHeader::pack(byte_t buf[], size_t sizeBuf)
{
  ssize_t m, n;

  if( sizeBuf < m_uHdrSize )
  {
    LOGERROR("Output buffer size=%zu < header size=%zu.\n",
        sizeBuf, m_uHdrSize);
    return -EU_ECODE_TOO_SMALL;
  }

  n  = EuOniNodeHeader::pack(buf, sizeBuf);
  n += pack64f(m_fov.fHFOV, buf+EU_ONI_DEPTH_NODE_HDR_IDX_FOV_H);
  n += pack64f(m_fov.fVFOV, buf+EU_ONI_DEPTH_NODE_HDR_IDX_FOV_V);
  n += pack16(m_resolution.m_uWidth, buf+EU_ONI_DEPTH_NODE_HDR_IDX_WIDTH);
  n += pack16(m_resolution.m_uHeight, buf+EU_ONI_DEPTH_NODE_HDR_IDX_HEIGHT);

  return n;
}

ssize_t EuOniDepthNodeHeader::unpack(const byte_t buf[], size_t uLen)
{
  ssize_t   n;
  size_t    uDataSize;

  if( uLen < m_uHdrSize )
  {
    LOGERROR("Buffer data length=%zu too small to hold %zu byte packed header.",
        uLen, m_uHdrSize); 
    return -EU_ECODE_TOO_SMALL;
  }

  n = EuOniNodeHeader::unpack(buf, uLen);

  if( n <= 0 )
  {
    return n;
  }

  n += unpack64f(buf+EU_ONI_DEPTH_NODE_HDR_IDX_FOV_H, m_fov.fHFOV);
  n += unpack64f(buf+EU_ONI_DEPTH_NODE_HDR_IDX_FOV_V, m_fov.fVFOV);
  n += unpack16(buf+EU_ONI_DEPTH_NODE_HDR_IDX_WIDTH, m_resolution.m_uWidth);
  n += unpack16(buf+EU_ONI_DEPTH_NODE_HDR_IDX_HEIGHT, m_resolution.m_uHeight);

  m_uRecordCnt = m_resolution.m_uWidth * m_resolution.m_uHeight;

  // calculated data size 
  uDataSize  = m_uRecordCnt * m_uRecordSize;

  if( uDataSize != m_uDataSize )
  {
    LOGERROR("Header specified data size=%zu differs from calculated size=%zu.",
        m_uDataSize, uDataSize); 
    return -EU_ECODE_BAD_FMT;
  }

  m_uTotalSize = n + m_uDataSize;

  return n;
}


// -----------------------------------------------------------------------------
// PcsOniImageNodeHeader Class
// -----------------------------------------------------------------------------

EuOniImageNodeHeader::EuOniImageNodeHeader() :
    EuOniNodeHeader(EuHdrTypeOniImage,
                    "oni_image",
                    EuOniDataFormatImageRgb)
{
  m_uHdrSize              = EU_ONI_IMAGE_NODE_HDR_SIZE;
  m_uRecordSize           = sizeof(XnRGB24Pixel);
  m_fov.fHFOV             = 0.0;
  m_fov.fVFOV             = 0.0;
  m_resolution.m_uWidth   = 0;
  m_resolution.m_uHeight  = 0;
}

EuOniImageNodeHeader::EuOniImageNodeHeader(EuResolution &resolution, EuFoV &fov)
  : EuOniNodeHeader(EuHdrTypeOniImage,
                    "oni_image",
                    EuOniDataFormatImageRgb)
{
  m_uHdrSize      = EU_ONI_IMAGE_NODE_HDR_SIZE;
  m_uRecordSize   = sizeof(XnRGB24Pixel);

  setFieldOfView(fov);
  setResolution(resolution);
}

ssize_t EuOniImageNodeHeader::pack(byte_t buf[], size_t sizeBuf)
{
  ssize_t m, n;

  if( sizeBuf < m_uHdrSize )
  {
    LOGERROR("Output buffer size=%zu < header size=%zu.", sizeBuf, m_uHdrSize);
    return -EU_ECODE_TOO_SMALL;
  }

  n  = EuOniNodeHeader::pack(buf, sizeBuf);
  n += pack64f(m_fov.fHFOV, buf+EU_ONI_IMAGE_NODE_HDR_IDX_FOV_H);
  n += pack64f(m_fov.fVFOV, buf+EU_ONI_IMAGE_NODE_HDR_IDX_FOV_V);
  n += pack16(m_resolution.m_uWidth, buf+EU_ONI_IMAGE_NODE_HDR_IDX_WIDTH);
  n += pack16(m_resolution.m_uHeight, buf+EU_ONI_IMAGE_NODE_HDR_IDX_HEIGHT);

  return n;
}

ssize_t EuOniImageNodeHeader::unpack(const byte_t buf[], size_t uLen)
{
  ssize_t   n;
  size_t    uDataSize;

  if( uLen < m_uHdrSize )
  {
    LOGERROR("Buffer data length=%zu too small to hold %zu byte packed header.",
        uLen, m_uHdrSize); 
    return -EU_ECODE_TOO_SMALL;
  }

  n = EuOniNodeHeader::unpack(buf, uLen);

  if( n <= 0 )
  {
    return n;
  }

  n += unpack64f(buf+EU_ONI_IMAGE_NODE_HDR_IDX_FOV_H, m_fov.fHFOV);
  n += unpack64f(buf+EU_ONI_IMAGE_NODE_HDR_IDX_FOV_V, m_fov.fVFOV);
  n += unpack16(buf+EU_ONI_IMAGE_NODE_HDR_IDX_WIDTH, m_resolution.m_uWidth);
  n += unpack16(buf+EU_ONI_IMAGE_NODE_HDR_IDX_HEIGHT, m_resolution.m_uHeight);

  m_uRecordCnt = m_resolution.m_uWidth * m_resolution.m_uHeight;

  uDataSize  = m_uRecordCnt * m_uRecordSize;

  if( uDataSize != m_uDataSize )
  {
    LOGERROR("Header specified data size=%zu differs from calculated size=%zu.",
        m_uDataSize, uDataSize); 
    return -EU_ECODE_BAD_FMT;
  }

  m_uTotalSize = n + m_uDataSize;

  return n;
}
