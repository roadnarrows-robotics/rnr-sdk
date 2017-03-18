////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_win
//
// File:      WinCvIoI.h
//
/*! \file
 *
 * $LastChangedDate: 2013-05-03 07:45:13 -0600 (Fri, 03 May 2013) $
 * $Rev: 2904 $
 *
 * \brief RoadNarrows Robotics OpenCV Image of Interest class interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2011-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
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

#ifndef _RNR_WIN_CV_IOI_H
#define _RNR_WIN_CV_IOI_H

#include "rnr/rnrconfig.h"

#include "opencv2/core/core.hpp"

#include "rnr/appkit/Win.h"

//
// RoaNarrows Robotics Windowing Declarations
//
namespace rnr
{
  //...........................................................................
  // Class WinCvIoI
  //...........................................................................

  /*!
   * Window OpenCV Image of Interest (IoI) Class.
   *
   * An IoI is typically at a higher resolution the what can be showed on a
   * display. Moreover, the display of IoI may require rotation and alignment.
   */
  class WinCvIoI
  {
  public:
    /*!
     * \brief Image of Interest class initialization constructor.
     *
     * Initialize the transformations needed to place a IoI in the window.
     *
     * \param sizeIoI     Size of hi-res Image of Interest.
     * \param sizeTgt     Size of target workspace, oriented at 0 degrees.
     * \param eOpRot      Target image rotation operation.
     *                    See \ref rnmpwin_rot.
     * \param eOpAlign    Target image alignment operation.
     *                    See \ref rnmpwin_align.
     * \param bCropToFit  If true, the transformed image is cropped to fit
     *                    into target size. Otherwise the transformed image is
     *                    scaled to fit.
     */
    WinCvIoI(cv::Size  &sizeIoI,
             cv::Size  &sizeTgt,
             RotOp      eOpRot,
             AlignOp    eOpAlign,
             bool       bCropToFit);

    /*!
     * \brief Destructor.
     */
    virtual ~WinCvIoI() { }

    /*!
     * \brief Set Image of Interest transfromation parameters.
     *
     * Initialize the transformations needed to place a IoI in the window.
     *
     * \param sizeIoI     Size of hi-res Image of Interest.
     * \param rectRoIIoI  Region of Interest within Image of Interest.
     * \param sizeTgt     Size of target workspace, oriented at 0 degrees.
     * \param eOpRot      Target image rotation operation.
     *                    See \ref rnmpwin_rot.
     * \param eOpAlign    Target image alignment operation.
     *                    See \ref rnmpwin_align.
     * \param bCropToFit  If true, the transformed image is cropped to fit
     *                    into target size. Otherwise the transformed image is
     *                    scaled to fit.
 
     */
    void setTransformParams(cv::Size &sizeIoI,
                            cv::Rect &rectRoIIoI,
                            cv::Size &sizeTgt,
                            RotOp     eOpRot,
                            AlignOp   eOpAlign,
                            bool      bCropToFit);

    /*!
     * \brief Transform Image of Interest into the target image.
     *
     * \param imgIoI            Image of Interest.
     * \param [in, out]imgTgt   Target image. Must be of size of RDK?
     */
    virtual void transform(cv::Mat &imgIoI, cv::Mat &imgTgt);

    /*!
     * \brief Map display pixel coordinates to Image of Interest pixel
     * coordinates.
     *
     * \param ptDisplay   Point in display coordinates.
     *
     * \return Cooresponding point in target IoI.
     */
    cv::Point mapPoint(cv::Point &ptDisplay);

    /*!
     * \breif Get the IoI original size.
     *
     * \return Size.
     */
    cv::Size getOriginalSize()
    {
      return cv::Size(m_sizeIoI.width, m_sizeIoI.height);
    }

    /*!
     * \brief Get the transformed target size.
     *
     * \return Size.
     */
    cv::Size getTransformedSize()
    {
      return cv::Size(m_rectRoITgt.width, m_rectRoITgt.height);
    }

    /*!
     * \brief Calculate the maximal size the source can fit into the target
     * size while keeping a 4:3 aspect ratio.
     *
     * \param sizeSrc   Source size.
     * \param sizeTgt   Target size.
     *
     * \return Maximum size.
     */
    cv::Size calcMaxFit43(cv::Size &sizeSrc, cv::Size &sizeTgt);

  protected:
    cv::Size      m_sizeIoI;        ///< image native size
    cv::Rect      m_rectRoIIoI;     ///< image of interest region of interest
    cv::Size      m_sizeTgt;        ///< target workspace size
    cv::Rect      m_rectRoITgt;     ///< target workspace region of interest
    RotOp         m_eOpRot;         ///< image rotation operator
    cv::Mat       m_matRot;         ///< 2x3 transformation matrix
    cv::Size      m_sizeRotated;    ///< rotated image size
    bool          m_bOpRotate;      ///< do [not] apply rotation operation
    bool          m_bOpScale;       ///< do [not] apply resize operation
    cv::Size      m_sizeScaled;     ///< scaled image size
    bool          m_bOpCrop;        ///< do [not] apply crop operation
    cv::Rect      m_rectRoICropped; ///< cropped region of interest
    AlignOp       m_eOpAlign;       ///< image alignment operator
  };
    
} // namespace


#endif // _RNR_WIN_CV_IOI_H
