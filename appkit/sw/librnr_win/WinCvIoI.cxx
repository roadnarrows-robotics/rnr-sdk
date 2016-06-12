////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Library:   librnr_win
//
// File:      WinCvIoI.cxx
//
/*! \file
 *
 * $LastChangedDate: 2013-05-03 07:45:13 -0600 (Fri, 03 May 2013) $
 * $Rev: 2904 $
 *
 * \brief RoadNarrows Robotics OpenCV Image of Interest class implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011-2016  RoadNarrows
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 */
/* 
 * @EulaBegin@
 * 
 * Permission is hereby granted, without written agreement and without
 * license or royalty fees, to use, copy, modify, and distribute this
 * software and its documentation for any purpose, provided that
 * (1) The above copyright notice and the following two paragraphs
 * appear in all copies of the source code and (2) redistributions
 * including binaries reproduces these notices in the supporting
 * documentation.   Substantial modifications to this software may be
 * copyrighted by their authors and need not follow the licensing terms
 * described here, provided that the new terms are clearly indicated in
 * all files where they apply.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
 * OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#include <sys/types.h>
#include <stdarg.h>
#include <libgen.h>

#include <cstring>
#include <iostream>
#include <fstream>
#include <cmath>

#include "rnr/rnrconfig.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "rnr/appkit/Win.h"
#include "rnr/appkit/WinOpenCv.h"
#include "rnr/appkit/WinCvIoI.h"

using namespace std;
using namespace cv;
using namespace rnr;


//.............................................................................
// Class WinCvIoI
//.............................................................................

WinCvIoI::WinCvIoI(Size    &sizeIoI,
                   Size    &sizeTgt,
                   RotOp    eOpRot,
                   AlignOp  eOpAlign,
                   bool     bCropToFit)
{
  Rect rectRoIIoI(0, 0, sizeIoI.width, sizeIoI.height);

  setTransformParams(sizeIoI, rectRoIIoI, sizeTgt,
                     eOpRot, eOpAlign, bCropToFit);
}

void WinCvIoI::setTransformParams(Size     &sizeIoI,
                                  Rect     &rectRoIIoI,
                                  Size     &sizeTgt,
                                  RotOp     eOpRot,
                                  AlignOp   eOpAlign,
                                  bool      bCropToFit)
{
  Point2f   center;           // rotation axis
  Size      sizeScaled;       // size of rotated and resized image
  int       x, y;             // working coordinates
  int       width, height;    // working dimensions

  m_sizeIoI     = sizeIoI;
  m_rectRoIIoI  = rectRoIIoI;
  m_sizeTgt     = sizeTgt;
  m_eOpRot      = eOpRot;
  m_bOpRotate   = false;
  m_eOpAlign    = eOpAlign;
  m_bOpScale    = false;
  m_bOpCrop     = false;

  // rotate about upper left corner (makes translation calculations easier)
  center.x = 0;
  center.y = 0;

  //
  // Rotation and resized images
  //
  switch( eOpRot )
  {
    // 270 degree rotation = -90 degrees
    case RotOp270:
      // set 2x3 rotation matrix
      m_matRot = getRotationMatrix2D(center, -90.0, 1.0);

      // translate
      cvSetReal2D(&m_matRot, 0, 2, sizeIoI.height);
      cvSetReal2D(&m_matRot, 1, 2, 0);

      // rotated IoI
      m_sizeRotated = Size(rectRoIIoI.height, rectRoIIoI.width);
      m_bOpRotate   = true;

      break;

    // 180 degree rotation
    case RotOp180:
      // set 2x3 rotation matrix
      m_matRot = getRotationMatrix2D(center, 180.0, 1.0);

      // translate
      cvSetReal2D(&m_matRot, 0, 2, sizeIoI.width);
      cvSetReal2D(&m_matRot, 1, 2, sizeIoI.height);

      // rotated IoI
      m_sizeRotated = cvSize(rectRoIIoI.width, rectRoIIoI.height);
      m_bOpRotate   = true;

      break;

    // 90 degree rotation
    case RotOp90:
      // set 2x3 rotation matrix
      m_matRot = getRotationMatrix2D(center, 90.0, 1.0);

      // translate
      cvSetReal2D(&m_matRot, 0, 2, 0);
      cvSetReal2D(&m_matRot, 1, 2, sizeIoI.width);

      // rotated IoI
      m_sizeRotated = cvSize(rectRoIIoI.height, rectRoIIoI.width);
      m_bOpRotate   = true;

      break;

    // 0 degree rotation = no rotations
    case RotOp0:
    default:
      // set 2x3 rotation matrix
      m_matRot = getRotationMatrix2D(center, 0.0, 1.0);

      // rotated IoI
      m_sizeRotated = cvSize(rectRoIIoI.width, rectRoIIoI.height);
      m_bOpRotate   = false;

      break;
  }

  //
  // Crop image to fit
  //
  if( bCropToFit )
  {
    // initialize
    x         = 0;
    y         = 0;
    width     = m_sizeRotated.width;
    height    = m_sizeRotated.height;
    m_bOpCrop = false;

    if( m_sizeRotated.width > sizeTgt.width )
    {
      x         = (m_sizeRotated.width - sizeTgt.width) / 2;
      width     = sizeTgt.width;
      m_bOpCrop = true;
    }
    if( m_sizeRotated.height > sizeTgt.height )
    {
      y         = (m_sizeRotated.height - sizeTgt.height) / 2;
      height    = sizeTgt.height;
      m_bOpCrop = true;
    }

    if( m_bOpCrop )
    {
      m_rectRoICropped = Rect(x, y, width, height);
    }
  }

  //
  // Scale image to fit
  // 
  else
  {
    // initialize
    m_bOpScale = false;

    // best fit in a 4:3 aspect ratio 
    // RDK TODO allow for other aspect ratios
    sizeScaled = calcMaxFit43(m_sizeRotated, sizeTgt);

    if( (sizeScaled.width  < m_sizeRotated.width) || 
        (sizeScaled.height < m_sizeRotated.height) )
    {
      m_sizeScaled  = sizeScaled;
      m_bOpScale    = true;
    }

    width   = sizeScaled.width;
    height  = sizeScaled.height;
  }

  //
  // Alignment ROI
  //
  switch( eOpAlign )
  {
    // align center
    case AlignOpCenter:
      x = (sizeTgt.width - width) / 2;
      y = (sizeTgt.height - height) / 2;
      m_rectRoITgt = cvRect(x, y, width, height);
      break;

    // align right
    case AlignOpRight:
      x = (sizeTgt.width - width) / 2;
      y = sizeTgt.height - 1;
      m_rectRoITgt = cvRect(x, y, width, height);
      break;

    // align left
    case AlignOpLeft:
    case AlignOpDefault:
    default:
      x = 0;
      y = (sizeTgt.height - height) / 2;
      m_rectRoITgt = cvRect(x, y, width, height);
      break;
  }
}

void WinCvIoI::transform(Mat &imgIoI, Mat &imgTgt)
{
  Mat     img;
  Size    dsize;

  img = Mat(imgIoI, m_rectRoIIoI);

  //
  // Rotate IoI
  //
  if( m_bOpRotate )
  {
    warpAffine(img, imgTgt, m_matRot, m_sizeRotated, INTER_LINEAR);
  }
  else
  {
    imgTgt = img;
  }

  //
  // Scale IoI
  //
  if( m_bOpScale )
  {
    resize(imgTgt, imgTgt, m_sizeScaled, 0.0, 0.0, INTER_LINEAR);
  }

  //
  // Crop IoI
  //
  if( m_bOpCrop )
  {
    imgTgt = Mat(imgTgt, m_rectRoICropped);
  }
}

Point WinCvIoI::mapPoint(Point &ptDisplay)
{
  Point   pt;
  int     t;
  double  fScaleX;
  double  fScaleY;

  // adjust of image offset
  pt.x = ptDisplay.x - m_rectRoITgt.x; 
  pt.y = ptDisplay.y - m_rectRoITgt.y; 

  // point not within target image of interest
  if( (pt.x < 0) || (pt.x >= m_rectRoITgt.width) ||
      (pt.y < 0) || (pt.y >= m_rectRoITgt.height) )
  {
    pt.x = pt.y = -1;
    return pt;
  }

  // remap x,y relative to image of interest origin (ulc)
  switch( m_eOpRot )
  {
    case RotOp270:
      t = pt.x;
      pt.x = pt.y + m_rectRoICropped.y;
      pt.y = m_rectRoITgt.width - (t - m_rectRoICropped.x) - 1;
      fScaleX = (double)(m_rectRoIIoI.width) / (double)(m_rectRoITgt.height);
      fScaleY = (double)(m_rectRoIIoI.height) / (double)(m_rectRoITgt.width);
      break;

    case RotOp180:
      pt.x = m_rectRoITgt.width - pt.x - 1;
      pt.y = m_rectRoITgt.height - pt.y - 1;
      fScaleX = (double)(m_rectRoIIoI.width) / (double)(m_rectRoITgt.width);
      fScaleY = (double)(m_rectRoIIoI.height) / (double)(m_rectRoITgt.height);
      break;

    case RotOp90:
      t = pt.x;
      pt.x = m_rectRoITgt.height - pt.y - m_rectRoICropped.y - 1;
      pt.y = t + m_rectRoICropped.x;
      fScaleX = (double)(m_rectRoIIoI.width) / (double)(m_rectRoITgt.height);
      fScaleY = (double)(m_rectRoIIoI.height) / (double)(m_rectRoITgt.width);
      break;

    case RotOp0:
    default:
      fScaleX = (double)(m_rectRoIIoI.width) / (double)(m_rectRoITgt.width);
      fScaleY = (double)(m_rectRoIIoI.height) / (double)(m_rectRoITgt.height);
      break;
  }

  // scale coordinate to match image of interest real size
  pt.x = (int)((double)(pt.x) * fScaleX);
  pt.y = (int)((double)(pt.y) * fScaleY);

  // adjust of target image of interest ROI
  pt.x += m_rectRoIIoI.x;
  pt.y += m_rectRoIIoI.y;

  return pt;
}

Size WinCvIoI::calcMaxFit43(Size &sizeSrc, Size &sizeTgt)
{
  Size  sizeScaled = sizeSrc;

  // fit width into target size
  if( sizeSrc.width > sizeTgt.width )
  {
    sizeScaled = ar43width(sizeTgt);
  }

  // fit scaled-by-width height into target size
  if( sizeScaled.height > sizeTgt.height )
  {
    sizeScaled    = ar43height(sizeTgt);
  }

  return sizeScaled;
}
