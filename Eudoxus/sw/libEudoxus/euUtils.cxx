////////////////////////////////////////////////////////////////////////////////
//
// Package:   Eudoxus
//
// Library:   libEudoxus
//
// File:      euUtils.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-11-30 15:41:20 -0700 (Mon, 30 Nov 2015) $
 * $Rev: 4226 $
 *
 * \brief Eudoxus utilities.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2012-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
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

#include "Eudoxus/euConf.h"

#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "ni/XnTypes.h"

#include "Eudoxus/Eudoxus.h"

using namespace std;


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

namespace eu
{
  /*!
   * \brief Eudoxus MimeType table.
   *
   * \note Keep in EuMimeType enumeration order.
   */
  static const char *MimeTypeTbl[] =
  {
    "undef",
    "oni/oni",
    "oni/pcs-ascii",
    "oni/pcs-binary",
    "oni/pcs-binary-compressed",
    "oni/pcs-oni"
  };

  /*!
   * \brief Eudoxus byte order table.
   *
   * \note Keep in EuEndian enumeration order.
   */
  static const char *EndianTbl[] =
  {
    "undef",
    "big",
    "little"
  };

} // namepsace eu


// ---------------------------------------------------------------------------
// Public Interface
// ---------------------------------------------------------------------------

namespace eu
{
  /*!
   * \brief Get the associated MimeType string.
   *  
   * \param eMimeType   Eudoxus MimeType enumerate.
   *
   * \return const char *.
   */
  const char *getMimeTypeStr(const EuMimeType eMimeType)
  {
    if( eMimeType < arraysize(MimeTypeTbl) )
    {
      return MimeTypeTbl[eMimeType];
    }
    else
    {
      return MimeTypeTbl[EuMimeTypeNone];
    }
  }
  
  /*!
   * \brief Map MimeType string to the associated Eudoxus enumerate.
   *  
   * \param sMimeType   Eudoxus MimeType string.
   *
   * \return Eudoxus MimeType enumerate.
   */
  EuMimeType mapMimeTypeToEnum(const char *sMimeType)
  {
    for(int i=0; i<arraysize(MimeTypeTbl); ++i)
    {
      if( !strcasecmp(sMimeType, MimeTypeTbl[i]) )
      {
        return (EuMimeType)i;
      } 
    }
  
    return EuMimeTypeNone;
  }
  
  /*!
   * \brief Get the associated byte order string.
   *  
   * \param eEndian   Eudoxus byte order enumerate.
   *
   * \return const char *.
   */
  const char *getEndianStr(const EuEndian eEndian)
  {
    if( eEndian < arraysize(EndianTbl) )
    {
      return EndianTbl[eEndian];
    }
    else
    {
      return EndianTbl[EuEndianNone];
    }
  }
  
  /*!
   * \brief Map byte order string to associated enumerate.
   *  
   * \param sEndian   Eudoxus endian string.
   *
   * \return Eudoxus byte order enumerate.
   */
  EuEndian mapEndianToEnum(const char *sEndian)
  {
    for(int i=0; i<arraysize(EndianTbl); ++i)
    {
      if( !strcasecmp(sEndian, EndianTbl[i]) )
      {
        return (EuEndian)i;
      } 
    }
  
    return EuEndianNone;
  }
  
  void setSensorFieldOfViewParams(EuFoV  &fov,
                                  double &fRealWorldXtoZ,
                                  double &fRealWorldYtoZ)
  {
    fRealWorldXtoZ = tan(fov.fHFOV/2.0)*2.0;
    fRealWorldYtoZ = tan(fov.fVFOV/2.0)*2.0;
  }
  
  void convertSensorProjectiveToRealWorld(const EuPoint3D &ptProjective,
                                          double           fRealWorldXtoZ,
                                          double           fRealWorldYtoZ,
                                          EuResolution    &resolution,
                                          EuPoint3D       &ptRealWorld)
  {
    double fNormalizedX = ptProjective.X / (double)resolution.m_uWidth - 0.5;
    double fNormalizedY = 0.5 - ptProjective.Y / (double)resolution.m_uHeight;
  
    ptRealWorld.X = (float)(fNormalizedX * ptProjective.Z * fRealWorldXtoZ);
    ptRealWorld.Y = (float)(fNormalizedY * ptProjective.Z * fRealWorldYtoZ);
    ptRealWorld.Z = ptProjective.Z;
  }

} // namepsace eu
