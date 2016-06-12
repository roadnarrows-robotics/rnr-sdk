////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Roboitics Windowing Package
//
// Library:   librnrwin-gtk
//
// File:      rnrGtkWinUtil.cxx
//
/*! \file
 *
 * $LastChangedDate: 2013-05-03 07:45:13 -0600 (Fri, 03 May 2013) $
 * $Rev: 2904 $
 *
 * \brief Windowing utilities.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011-2016.  RoadNarrows
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

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "rnr/rnrWin.h"

using namespace std;
using namespace rnrWin;


/*!
 * \brief Apply color gradient to image.
 *
 * The gradient ranges from black to the given color, given the intensity
 * of each pixel in the unaltered image.
 *
 * \param [in,out] pImg   Pointer to image to change.
 * \param red             8-bit red component of RGB.
 * \param green           8-bit green component of RGB.
 * \param blue            8-bit blue component of RGB.
 */
void ApplyColorGradient(IplImage *pImg, int red, int green, int blue)
{
  CvScalar  s;
  CvScalar  t;
  double    intensity;

  for(int i=0; i<pImg->width; ++i)
  {
    for(int j=0; j<pImg->height; ++j)
    {
      s =cvGet2D(pImg, i, j);
      intensity =  ((((byte_t)s.val[0]) & 0xff) << 16) |
                ((((byte_t)s.val[1]) & 0xff) << 8) |
                (((byte_t)s.val[2]) & 0xff);
      intensity /= (double)(0x00ffffff);
      t = cvScalar(blue*intensity, green*intensity, red*intensity);
      cvSet2D(pImg, i, j, t);
    }
  }
}
