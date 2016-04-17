////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Library:   librnr_win
//
// File:      WinOpenCv.h
//
/*! \file
 *
 * $LastChangedDate: 2013-05-03 07:45:13 -0600 (Fri, 03 May 2013) $
 * $Rev: 2904 $
 *
 * \brief RoadNarrows Robotics OpenCV Utilities.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011-2013.  RoadNarrows
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

#ifndef _RNR_WIN_OPEN_CV_H
#define _RNR_WIN_OPEN_CV_H

#include <stdio.h>
#include <math.h>

#include "rnr/rnrconfig.h"

#include "opencv2/core/core.hpp"

//
// Windowing Declarations
//
namespace rnr
{
  /*
   * Image channel indices.
   * Note:  For some odd reason, OpenCV does not define these. And RGB is in
   *        reverse order than what would be expected.
   */
  #define CHANNEL_GRAY       -1     ///< all channels summed to gray

  // RGB
  #define CHANNEL_BLUE        0     ///< blue channel
  #define CHANNEL_GREEN       1     ///< green channel
  #define CHANNEL_RED         2     ///< red channel

  // HSV
  #define CHANNEL_HUE         0     ///< hue (color) channel
  #define CHANNEL_SATURATION  1     ///< saturation (intensity) channel
  #define CHANNEL_VALUE       2     ///< value (brightness) channel

  const cv::Point   nopoint(-1, -1);              ///< integer 2D "No Point"
  const cv::Point2f nopoint2D(-1.0, -1.0);        ///< fpn 2D "No Point"
  const cv::Point3f nopoint3D(-1.0, -1.0, -1.0);  ///< fpn 3D "No Point"

  /*!
   * \brief Check if point is not set ("no point").
   *
   * \param pt  2D integer point.
   *
   * \return true or false.
   */
  inline bool isnopoint(const cv::Point &pt)
  {
    return (pt.x < 0) || (pt.y < 0)? true: false;
  }

  /*!
   * \brief Check if point is not set ("no point").
   *
   * \param pt  2D floating-point number point.
   *
   * \return true or false.
   */
  inline bool isnopoint(const cv::Point2f &pt)
  {
    return (pt.x < 0.0) || (pt.y < 0.0)? true: false;
  }

  /*!
   * \brief Check if point is not set ("no point").
   *
   * \param pt  3D floating-point number point
   *
   * \return true or false.
   */
  inline bool isnopoint(const cv::Point3f &pt)
  {
    return (pt.x < 0.0) || (pt.y < 0.0) || (pt.z < 0.0)? true: false;
  }

  /*!
   *  \brief Calculate the L1 (taxi cab) distance between two 2D points.
   *
   * \param pt1 2D integer point 1.
   * \param pt2 2D integer point 2.
   *
   * \return ||pt2-pt1||<sub>1</sub>
   */
  inline int distL1(const cv::Point &p1, const cv::Point &p2)
  {
    return (int)(fabs((double)(p1.x-p2.x)) + fabs((double)(p1.y-p2.y)));
  }

  /*!
   *  \brief Calculate the L1 (taxi cab) distance between two 2D points.
   *
   * \param pt1 2D floating-point number point 1.
   * \param pt2 2D floating-point number point 2.
   *
   * \return ||pt2-pt1||<sub>1</sub>
   */
  inline int distL1(const cv::Point2f &p1, const cv::Point2f &p2)
  {
    return fabs(p1.x-p2.x) + fabs(p1.y-p2.y);
  }

  /*!
   *  \brief Calculate the L1 (taxi cab) distance between two 3D points.
   *
   * \param pt1 3D floating-point number point 1.
   * \param pt2 3D floating-point number point 2.
   *
   * \return ||pt2-pt1||<sub>1</sub>
   */
  inline int distL1(const cv::Point3f &p1, const cv::Point3f &p2)
  {
    return fabs(p1.x-p2.x) + fabs(p1.y-p2.y) + fabs(p1.z-p2.z);
  }

  /*!
   *  \brief Calculate the L2 (euclidean) distance between two 2D points.
   *
   * \param pt1 2D integer point 1.
   * \param pt2 2D integer point 2.
   *
   * \return Distance.
   */
  inline int distL2(const cv::Point &p1, const cv::Point &p2)
  {
    return (int)sqrt( (double)((p1.x-p2.x)*(p1.x-p2.x)) + 
                      (double)((p1.y-p2.y)*(p1.y-p2.y)) );
  }

  /*!
   *  \brief Calculate the L2 (euclidean) distance between two 2D points.
   *
   * \param pt1 2D floating-point number point 1.
   * \param pt2 2D floating-point number point 2.
   *
   * \return Distance.
   */
  inline double distL2(const cv::Point2f &p1, const cv::Point2f &p2)
  {
    return sqrt( (p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) );
  }

  /*!
   *  \brief Calculate the L2 (euclidean) distance between two 3D points.
   *
   * \param pt1 3D floating-point number point 1.
   * \param pt2 3D floating-point number point 2.
   *
   * \return Distance.
   */
  inline double distL2(const cv::Point3f &p1, const cv::Point3f &p2)
  {
    return sqrt( (p1.x-p2.x)*(p1.x-p2.x) +
                 (p1.y-p2.y)*(p1.y-p2.y) +
                 (p1.z-p2.z)*(p1.z-p2.z) );
  }

  /*!
   *  \brief Calculate the Linf distance between two 2D points.
   *
   * \param pt1 2D integer point 1.
   * \param pt2 2D integer point 2.
   *
   * \return Distance.
   */
  inline int distLinf(const cv::Point &p1, const cv::Point &p2)
  {
    return (int)fmax(fabs((double)(p1.x-p2.x)), fabs((double)(p1.y-p2.y)));
  }

  /*!
   *  \brief Calculate the Linf distance between two 2D points.
   *
   * \param pt1 2D floating-point number point 1.
   * \param pt2 2D floating-point number point 2.
   *
   * \return Distance.
   */
  inline int distLinf(const cv::Point2f &p1, const cv::Point2f &p2)
  {
    return fmax(fabs(p1.x-p2.x), fabs(p1.y-p2.y));
  }

  /*!
   *  \brief Calculate the Linf distance between two 3D points.
   *
   * \param pt1 3D floating-point number point 1.
   * \param pt2 3D floating-point number point 2.
   *
   * \return Distance.
   */
  inline int distLinf(const cv::Point3f &p1, const cv::Point3f &p2)
  {
    return fmax(fmax(fabs(p1.x-p2.x), fabs(p1.y-p2.y)), fabs(p1.z-p2.z));
  }

  /*!
   * \brief Check if point x,y is with rectangle r.
   *
   * \param x   X integer coordinate.
   * \param y   Y integer coordinate.
   * \param r   Integer ectangle.
   *
   * \return true or flase
   */
  inline bool isinrect(int x, int y, cv::Rect &r)
  {
    return (x >= r.x) && (x < r.x+r.width) && (y >= r.y) && (y < r.y+r.height)?
            true: false;
  }

  /*!
   * \brief Calculate the nearest 4:3 aspect ratio from the width component of
   * the target size.
   *
   * \param siz   Target object dimensions.
   *
   * \return The 4:3 dimensions \h_le target siz.
   */
  inline cv::Size ar43width(cv::Size &siz)
  {
    cv::Size siz43;
    int w = siz.width;
    int r = w % 4;
    siz43.height = (3 * (w-r)) / 4;
    siz43.width  = (4 * siz43.height) / 3;
    return siz43;
  }

  /*!
   * \brief Calculate the nearest 4:3 aspect ratio from the target width.
   *
   * \param width   Target width
   *
   * \return The 4:3 dimension with width \h_le target width.
   */
  inline cv::Size ar43width(int width)
  {
    cv::Size siz43;
    int w = width;
    int r = w % 4;
    siz43.height = (3 * (w-r)) / 4;
    siz43.width  = (4 * siz43.height) / 3;
    return siz43;
  }

  /*!
   * \brief Calculate the nearest 4:3 aspect ratio from the height component of
   * the target size.
   *
   * \param siz   Target object dimensions.
   *
   * \return 4:3 dimensions \h_le siz
   */
  inline cv::Size ar43height(cv::Size &siz)
  {
    cv::Size siz43;
    int h = siz.height;
    int r = h % 3;
    siz43.width  = (4 * (h-r)) / 3;
    siz43.height = (3 * siz43.width) / 4;
    return siz43;
  }

  /*!
   * \brief Calculate the nearest 4:3 aspect ratio from the target height.
   *
   * \param height   Target height.
   *
   * \return 4:3 dimensions with height \h_le target height.
   */
  inline cv::Size ar43height(int height)
  {
    cv::Size siz43;
    int h = height;
    int r = h % 3;
    siz43.width  = (4 * (h-r)) / 3;
    siz43.height = (3 * siz43.width) / 4;
    return siz43;
  }

  /*!
   * \brief Debug print rectangle.
   *
   * \param s   Preface string.
   * \param r   Rectangle.
   */
  inline void dbgrect(const std::string &str, const cv::Rect &r)
  {
    fprintf(stderr, "%s=(%d,%d,%d,%d)\n",
        str.c_str(), r.x, r.y, r.width, r.height);
  }

  /*!
   * \brief Debug print point.
   *
   * \param s   Preface string.
   * \param pt  Point.
   */
  inline void dbgpoint(const std::string &str, const cv::Point &pt)
  {
    fprintf(stderr, "%s=(%d,%d)\n", str.c_str(), pt.x, pt.y);
  }

  /*!
   * \brief Debug print size.
   *
   * \param s   Preface string.
   * \param siz Size.
   */
  inline void dbgsize(const std::string &str, const cv::Size &siz)
  {
    fprintf(stderr, "%s=(%d,%d)\n", str.c_str(), siz.width, siz.height);
  }
} // namespace


#endif // _RNR_WIN_OPEN_CV_H
