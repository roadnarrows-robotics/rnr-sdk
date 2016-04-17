////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekProd4L.h
//
/*! \file
 *
 * $LastChangedDate: 2014-10-06 15:13:20 -0600 (Mon, 06 Oct 2014) $
 * $Rev: 3773 $
 *
 * \brief Hekateros 4 DoF long robotic arm static specification.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2013.  RoadNarrows
 * (http://www.RoadNarrows.com)
 * \n All Rights Reserved
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

#ifndef _HEK_PROD_ARM_4L_H
#define _HEK_PROD_ARM_4L_H

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

/*! product id */
#define HEK_4L_PRODUCT_ID   HEK_PRODUCT_ID(HekProdSizeLong, 4, 0)

namespace hekateros
{
  const int HekProdArm4LId            = HEK_4L_PRODUCT_ID; ///< product id

  const int HekProdArm4LNumLinks      = 4;  ///< number of fixed links
  const int HekProdArm4LDoF           = 4;  ///< degrees of freedom 
  const int HekProdArm4LNumOptLimits  = 4;  ///< number of optical limits
  const int HekProdArm4LNumServos     = 5;  ///< number of servos

  /*!
   * \brief Specification of links.
   */
  extern const HekSpecLink_T HekProdArm4LSpecLinks[];

  /*!
   * \brief Specification of joints, v1.1
   */
  extern const HekSpecJoint_T  HekProdArm4LSpecJoints_1_1[];

  /*!
   * \brief Specification of servos, v1.1
   */
  extern const HekSpecServo_T HekProdArm4LSpecServos_1_1[];

} // namespace hekateros

/*! \} */


#endif // _HEK_PROD_ARM_4L_H
