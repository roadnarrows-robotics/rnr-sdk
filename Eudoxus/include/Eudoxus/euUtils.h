////////////////////////////////////////////////////////////////////////////////
//
// Package:   Eudoxus
//
// Library:   libEudoxus
//
// File:      euUtils.h
//
/*! \file
 *
 * $LastChangedDate: 2016-01-19 17:29:01 -0700 (Tue, 19 Jan 2016) $
 * $Rev: 4267 $
 *
 * \brief Eudoxus library utility declarations and defines.
 *
 * \todo TODO use netmsgs routines.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2012-2016.  RoadNarrows
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 */
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
////////////////////////////////////////////////////////////////////////////////

#ifndef _EU_UTILS_H
#define _EU_UTILS_H

#include "Eudoxus/euConf.h"

#include <sys/types.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Eudoxus/Eudoxus.h"


namespace eu
{
  inline ssize_t pack8(uint_t val, byte_t *buf)
  {
    buf[0] = (byte_t)(val & 0xff);
    return 1;
  }
  
  inline ssize_t unpack8(const byte_t *buf, uint_t &val)
  {
    val = (uint_t)buf[0];
    return 1;
  }
  
  inline ssize_t pack16(uint_t val, byte_t *buf)
  {
    buf[0] = (byte_t)((val >> 8) & 0xff);
    buf[1] = (byte_t)(val & 0xff);

    return 2;
  }
  
  inline ssize_t unpack16(const byte_t *buf, uint_t &val)
  {
    val = (uint_t)( ((uint_t)buf[0] << 8) | (uint_t)buf[1] );
    return 2;
  }
  
  inline ssize_t unpack16Little(const byte_t *buf, uint_t &val)
  {
    val = (uint_t)( ((uint_t)buf[1] << 8) | (uint_t)buf[0] );
    return 2;
  }
  
  inline ssize_t pack32(uint_t val, byte_t *buf)
  {
    buf[0] = (byte_t)((val >> 24) & 0xff);
    buf[1] = (byte_t)((val >> 16) & 0xff);
    buf[2] = (byte_t)((val >> 8) & 0xff);
    buf[3] = (byte_t)(val & 0xff);

    return 4;
  }
  
  inline ssize_t unpack32(const byte_t *buf, uint_t &val)
  {
    val = (uint_t)( ((uint_t)buf[0] << 24) |
                    ((uint_t)buf[1] << 16) |
                    ((uint_t)buf[2] << 8) |
                    (uint_t)buf[3] );
    return 4;
  }
  
// size_t == uint_t on overo
#if !defined(ARCH_odroid) && !defined(ARCH_overo) && !defined(ARCH_linaro) && !defined(ARCH_i386)
  inline ssize_t pack32(size_t val, byte_t *buf)
  {
    buf[0] = (byte_t)((val >> 24) & 0xff);
    buf[1] = (byte_t)((val >> 16) & 0xff);
    buf[2] = (byte_t)((val >> 8) & 0xff);
    buf[3] = (byte_t)(val & 0xff);

    return 4;
  }
  
  inline ssize_t unpack32(const byte_t *buf, size_t &val)
  {
    val = (uint_t)( ((size_t)buf[0] << 24) |
                    ((size_t)buf[1] << 16) |
                    ((size_t)buf[2] << 8) |
                     (size_t)buf[3] );
    return 4;
  }
#endif // 32-bit archs
  
  inline ssize_t pack32f(float val, byte_t *buf)
  {
    byte_t  *p = (byte_t *)&val;
  
    buf[0] = p[0];
    buf[1] = p[1];
    buf[2] = p[2];
    buf[3] = p[3];

    return 4;
  }
  
  inline ssize_t unpack32f(const byte_t *buf, float &val)
  {
    byte_t *p = (byte_t *)&val;
  
    p[0] = buf[0];
    p[1] = buf[1];
    p[2] = buf[2];
    p[3] = buf[3];
  
    return 4;
  }
  
  inline ssize_t pack64f(double val, byte_t *buf)
  {
    byte_t  *p = (byte_t *)&val;
  
    buf[0] = p[0];
    buf[1] = p[1];
    buf[2] = p[2];
    buf[3] = p[3];
    buf[4] = p[4];
    buf[5] = p[5];
    buf[6] = p[6];
    buf[7] = p[7];

    return 8;
  }
  
  inline ssize_t unpack64f(const byte_t *buf, double &val)
  {
    byte_t *p = (byte_t *)&val;
  
    p[0] = buf[0];
    p[1] = buf[1];
    p[2] = buf[2];
    p[3] = buf[3];
    p[4] = buf[4];
    p[5] = buf[5];
    p[6] = buf[6];
    p[7] = buf[7];
  
    return 8;
  }

  inline ssize_t packRGB(const EuRGB24Pixel &val, byte_t *buf)
  {
    buf[3] = 0;
    buf[2] = (byte_t)(val.nRed   & 0xff);
    buf[1] = (byte_t)(val.nGreen & 0xff);
    buf[0] = (byte_t)(val.nBlue  & 0xff);
  
    return 4;
  }

  inline ssize_t unpackRGB(const byte_t *buf, EuRGB24Pixel &val)
  {
    val.nRed    = buf[2];
    val.nGreen  = buf[1];
    val.nBlue   = buf[0];
  
    return 4;
  }

  /*!
   * \brief Integer absolute value.
   *
   * \param a   Integer value.
   *
   * \return |a|
   */
  inline int iabs(int a)
  {
    return a>=0? a: -a;
  }
  
  /*!
   * \brief Minimum of two integer values
   *
   * \param a   Integer a.
   * \param b   Integer b.
   *
   * \return min(a,b)
   */
  inline int imin(int a, int b)
  {
    return a<=b? a: b;
  }
  
  /*!
   * \brief Maximum of two  integer values
   *
   * \param a   Integer a.
   * \param b   Integer b.
   *
   * \return max(a,b)
   */
  inline int imax(int a, int b)
  {
    return a>=b? a: b;
  }

  extern const char *getStrError(int ecode);

  extern std::string getConfigFileName(const char *sCmdOption=NULL);

  extern const char *getMimeTypeStr(const EuMimeType eMimeType);

  extern EuMimeType mapMimeTypeToEnum(const char *sMimeType);

  extern const char *getEndianStr(const EuEndian eEndian);

  extern EuEndian mapEndianToEnum(const char *sEndian);

  extern void setSensorFieldOfViewParams(EuFoV  &fov,
                                         double &fRealWorldXtoZ,
                                         double &fRealWorldYtoZ);

  extern void convertSensorProjectiveToRealWorld(const EuPoint3D &ptProjective,
                                                 double          fRealWorldXtoZ,
                                                 double          fRealWorldYtoZ,
                                                 EuResolution   &resolution,
                                                 EuPoint3D      &ptRealWorld);
} // namespace eu


#endif // _EU_UTILS_H
