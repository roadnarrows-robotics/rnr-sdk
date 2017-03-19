////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekProd5L.h
//
/*! \file
 *
 * $LastChangedDate: 2015-12-10 10:43:51 -0700 (Thu, 10 Dec 2015) $
 * $Rev: 4239 $
 *
 * \brief Hekateros 5 DoF long robotic arm static specification.
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

#ifndef _HEK_PROD_ARM_5L_H
#define _HEK_PROD_ARM_5L_H

#include "Dynamixel/Dynamixel.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekOptical.h"


/*!
 * \ingroup hek_spec
 * \defgroup hek_prod_5la Hekateros 5 DoF Long Specification
 *
 * \{
 */

/*! product id */
#define HEK_5L_PRODUCT_ID   HEK_PRODUCT_ID(HekProdSizeLong, 5, 0)

namespace hekateros
{
  const int HekProdArm5LId            = HEK_5L_PRODUCT_ID; ///< product id

  const int HekProdArm5LNumLinks      = 5;  ///< number of fixed links
  const int HekProdArm5LDoF           = 5;  ///< degrees of freedom 
  const int HekProdArm5LNumOptLimits  = 6;  ///< number of optical limits
  const int HekProdArm5LNumServos     = 6;  ///< number of servos

  /*!
   * \brief Specification of links.
   */
  extern const HekSpecLink_T HekProdArm5LSpecLinks[];

  /*! \brief Specification of joints, v1.1 */
  extern const HekSpecJoint_T  HekProdArm5LSpecJoints_1_1[];

  /*! \brief Specification of servos, v1.1 */
  extern const HekSpecServo_T HekProdArm5LSpecServos_1_1[];

  /*! \brief Specification of joints, v1.2 */
  extern const HekSpecJoint_T  HekProdArm5LSpecJoints_1_2[];

  /*! \brief Specification of servos, v1.2 */
  extern const HekSpecServo_T HekProdArm5LSpecServos_1_2[];

  /*! \brief Specification of joints, v1.3 */
  extern const HekSpecJoint_T  HekProdArm5LSpecJoints_1_3[];

  /*! \brief Specification of servos, v1.3 */
  extern const HekSpecServo_T HekProdArm5LSpecServos_1_3[];

  /*! \brief Specification of joints, v1.3.5 */
  extern const HekSpecJoint_T  HekProdArm5LSpecJoints_1_3_5[];

  /*! \brief Specification of servos, v1.4 */
  extern const HekSpecServo_T HekProdArm5LSpecServos_1_3_5[];

  /*! \brief Specification of joints, v1.4 */
  extern const HekSpecJoint_T  HekProdArm5LSpecJoints_1_4[];

  /*! \brief Specification of servos, v1.4 */
  extern const HekSpecServo_T HekProdArm5LSpecServos_1_4[];

  /*! \brief Specification of joints, v2.0 */
  extern const HekSpecJoint_T  HekProdArm5LSpecJoints_2_0[];

  /*! \brief Specification of servos, v2.0 */
  extern const HekSpecServo_T HekProdArm5LSpecServos_2_0[];


} // namespace hekateros

/*! \} */


#endif // _HEK_PROD_ARM_5L_H
