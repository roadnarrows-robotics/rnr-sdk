////////////////////////////////////////////////////////////////////////////////
//
// Package:   Eudoxus
//
// Library:   libEudoxus
//
// File:      euHeader.h
//
/*! \file
 *
 * $LastChangedDate: 2015-11-30 15:41:20 -0700 (Mon, 30 Nov 2015) $
 * $Rev: 4226 $
 *
 * \brief Eudoxus data header base declarations.
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

#ifndef _EU_HEADER_H
#define _EU_HEADER_H

#include "Eudoxus/euConf.h"

#include <string>

#include "rnr/rnrconfig.h"

#include "Eudoxus/Eudoxus.h"

using namespace std;

namespace eu
{
  // --------------------------------------------------------------------------
  // Eudoxus Data Header Base Absract Class
  // --------------------------------------------------------------------------

  /*!
   * \brief Eudoxus data header base abstract class.
   *
   * All libEudoxus headers are derived from this class.
   */
  class EuHeader
  {
  public:
    EuHeader(EuHdrType eHdrType=EuHdrTypeNone, string strHdrName="undef")
    {
      m_eHdrType    = eHdrType;
      m_strHdrName  = strHdrName;
      m_uTotalSize  = 0;
      m_uHdrSize    = 0;
      m_uRecordSize = 0;
      m_uDataSize   = 0;
    }

    virtual ~EuHeader() { }

    virtual ssize_t pack(byte_t buf[], size_t sizeBuf) = 0;
    virtual ssize_t unpack(const byte_t buf[], size_t uLen) = 0;
    virtual size_t  getMaxTotalSize() = 0;

    bool isHdrType(EuHdrType eHdrType)
    {
      return eHdrType == m_eHdrType;
    }

    bool isHdrType(string &strHdrType)
    {
      return strHdrType == m_strHdrName;
    }

    EuHdrType getHdrType()    { return m_eHdrType; }
    string    getHdrName()    { return m_strHdrName; }
    size_t    getTotalSize()  { return m_uTotalSize; }
    size_t    getHdrSize()    { return m_uHdrSize; }
    size_t    getRecordSize() { return m_uRecordSize; }
    size_t    getRecordCnt()  { return m_uRecordCnt; }
    size_t    getDataSize()   { return m_uDataSize; }

  protected:
    EuHdrType       m_eHdrType;         ///< header type
    string          m_strHdrName;       ///< header type name
    size_t          m_uTotalSize;       ///< actual packed header + data size
    size_t          m_uHdrSize;         ///< actual packed header size
    size_t          m_uRecordSize;      ///< data record size
    size_t          m_uRecordCnt;       ///< data record count
    size_t          m_uDataSize;        ///< total packed data size
  };

} // namespace

#endif // _EU_HEADER_H
