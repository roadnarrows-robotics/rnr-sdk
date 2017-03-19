////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekProdEEGraboid.h
//
/*! \file
 *
 * $LastChangedDate: 2015-02-10 13:39:03 -0700 (Tue, 10 Feb 2015) $
 * $Rev: 3866 $
 *
 * \brief Hekateros 1 DoF Graboid end effector family.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2013-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
/*
 * @EulaBegin@
 * 
 * Unless otherwise stated explicitly, all materials contained are copyrighted
 * and may not be used without RoadNarrows LLC's written consent,
 * except as provided in these terms and conditions or in the copyright
 * notice (documents and software) or other proprietary notice provided with
 * the relevant materials.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY 
 * MEMBERS/EMPLOYEES/CONTRACTORS OF ROADNARROWS OR DISTRIBUTORS OF THIS SOFTWARE
 * BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
 * DOCUMENTATION, EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHORS AND  ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _HEK_PROD_EE_GRABOID_H
#define _HEK_PROD_EE_GRABOID_H

#include "Dynamixel/Dynamixel.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekOptical.h"
#include "Hekateros/hekSpec.h"


/*!
 * \ingroup hek_spec
 * \defgroup hek_prod_4l  Hekateros 4 DoF Long Specification
 *
 * \{
 */

/*! end effector product family */
#define HEK_EE_GRABOID_FAMILY       0x02

/*! product id */
#define HEK_EE_GRABOID_PRODUCT_ID \
  HEK_EE_PRODUCT_ID(HEK_EE_GRABOID_FAMILY, HekProdSizeStd, 1, 0)

namespace hekateros
{
  const int HekProdEEGraboidFamily  = HEK_EE_GRABOID_FAMILY;     ///< family
  const int HekProdEEGraboidId      = HEK_EE_GRABOID_PRODUCT_ID; ///< prod id

  const int HekProdEEGraboidNumLinks      = 3;  ///< number of fixed links
  const int HekProdEEGraboidDoF           = 1;  ///< degrees of freedom 
  const int HekProdEEGraboidNumOptLimits  = 0;  ///< number of optical limits
  const int HekProdEEGraboidNumServos     = 1;  ///< number of servos

  /*!
   * \brief Specification of links.
   */
  extern const HekSpecLink_T HekProdEEGraboidSpecLinks[];

  /*! \brief Specification of joints, v1.0 */
  extern const HekSpecJoint_T HekProdEEGraboidSpecJoints_1_0[];

  /*! \brief Specification of servos, v1.0 */
  extern const HekSpecServo_T HekProdEEGraboidSpecServos_1_0[];

  /*! \brief Specification of joints, v1.1 */
  extern const HekSpecJoint_T HekProdEEGraboidSpecJoints_1_1[];

  /*! \brief Specification of servos, v1.1 */
  extern const HekSpecServo_T HekProdEEGraboidSpecServos_1_1[];

  /*! \brief Specification of joints, v1.2 */
  extern const HekSpecJoint_T HekProdEEGraboidSpecJoints_1_2[];

  /*! \brief Specification of servos, v1.2 */
  extern const HekSpecServo_T HekProdEEGraboidSpecServos_1_2[];

} // namespace hekateros

/*! \} */


#endif // _HEK_PROD_EE_GRABOID_H
