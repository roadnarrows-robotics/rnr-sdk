////////////////////////////////////////////////////////////////////////////////
//
// Package:   Eudoxus
//
// File:      Eudoxus.h
//
/*! \file
 *
 * $LastChangedDate: 2016-01-07 08:07:26 -0700 (Thu, 07 Jan 2016) $
 * $Rev: 4253 $
 *
 * \brief Top-level package include file
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

#ifndef _EUDOXUS_H
#define _EUDOXUS_H

#include <sys/types.h>
#include <sys/time.h>
#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "ni/XnTypes.h"

/*!
 * \ingroup eu_h
 * \defgroup eu_ecodes  RoadNarrows Eudoxus Error Codes
 *
 * Eudoxus package-wide error codes.
 *
 * \{
 */
#define EU_OK                       0 ///< not an error, success

#define EU_ECODE_GEN                1 ///< general, unspecified error
#define EU_ECODE_SYS                2 ///< system (errno) error
#define EU_ECODE_INTERNAL           3 ///< internal error (bug)
#define EU_ECODE_BAD_VAL            4 ///< bad value general error
#define EU_ECODE_TOO_BIG            5 ///< value/list/size too big
#define EU_ECODE_TOO_SMALL          6 ///< value/list/size too small
#define EU_ECODE_RANGE              7 ///< value out-of-range
#define EU_ECODE_BAD_OP             8 ///< invalid operation error
#define EU_ECODE_TIMEDOUT           9 ///< operation timed out error
#define EU_ECODE_NO_DEV            10 ///< device not found error
#define EU_ECODE_NO_RSRC           11 ///< no device/resource available error
#define EU_ECODE_BUSY              12 ///< resource busy error
#define EU_ECODE_NO_EXEC           13 ///< cannot execute error
#define EU_ECODE_PERM              14 ///< cannot execute error
#define EU_ECODE_BAD_FMT           15 ///< bad data format
#define EU_ECODE_NI                16 ///< open natural interface specific error
#define EU_ECODE_PCL               17 ///< point cloud library specific error
#define EU_ECODE_GST               18 ///< gstreamer specific error

#define EU_ECODE_BADEC             19 ///< bad error code

#define EU_ECODE_NUMOF             20 ///< number of error codes
/*! \} */

//
// Product names and marks
//
#define EU_PROD_NAME  "Eudoxus"

namespace eu
{
  /*!
   * \brief Eudoxus OpenNI media MimeTypes.
   */
  typedef enum
  {
    EuMimeTypeNone = 0,       ///< no or unknown format specified
    EuMimeTypeOni,            ///< oni/oni            OpenNI native
    EuMimeTypePcsAscii,       ///< oni/pcs-ascii      ASCII PCD Stream 
    EuMimeTypePcsBinary,      ///< oni/pcs-binary     Binary PCD Stream 
    EuMimeTypePcsBinaryLzf,   ///< oni/pcs-binary-lzf LZF compressed binary PCD
    EuMimeTypePcsOni,         ///< oni/pcs-oni        Raw OpenNI PC Stream

    EuMimeTypeNumOf           ///< number of mimetypes (keep last)
  } EuMimeType;

  /*!
   * \brief Binary Data Endianess.
   */
  typedef enum
  {
    EuEndianNone = 0,     ///< unknown
    EuEndianBig,          ///< big endian
    EuEndianLittle        ///< little endian
  } EuEndian;

  /*!
   * \brief Production Node Types.
   */
  typedef enum
  {
    EuProdNodeTypeNone  = 0x00,     ///< no or unknown production node type
    EuProdNodeTypeDepth = 0x01,     ///< depth production node
    EuProdNodeTypeImage = 0x02,     ///< image production node
    EuProdNodeTypeAudio = 0x04,     ///< audio production node
    EuProdNodeTypeImu   = 0x08,     ///< IMU production node

    EuProdNodeTypeNumOf = 4         ///< number of production node types
  } EuProdNodeType;

  typedef enum
  {
    EuHdrTypeNone = 0,              ///< undefined header type
    EuHdrTypePcs,                   ///< top-level point cloud stream header 
    EuHdrTypePcdAsciiXyz,           ///< PCD ASCII XYZ section header
    EuHdrTypePcdAsciiXyzRgb,        ///< PCD ASCII XYZ RGB section header
    EuHdrTypePcdBinaryXyz,          ///< PCD binary XYZ section header
    EuHdrTypePcdBinaryXyzRgb,       ///< PCD binary XYZ RGB section header
    EuHdrTypeOniDepth,              ///< OpenNI depth section header
    EuHdrTypeOniImage,              ///< OpenNI image section header
    EuHdrTypeOniAudio,              ///< OpenNI audio section header
    EuHdrTypeOniImu,                ///< OpenNI IMU section header

    EuHdrTypeNumOf                  ///< number of header types (keep last)
  } EuHdrType;

  typedef XnFieldOfView EuFoV;
  typedef XnDepthPixel  EuPointDepth;
  typedef XnPoint3D     EuPoint3D;
  typedef XnRGB24Pixel  EuRGB24Pixel;

  typedef struct
  {
    uint_t  m_uWidth;
    uint_t  m_uHeight;
  } EuResolution;

} // namespace eu


#endif // _EUDOXUS_H
