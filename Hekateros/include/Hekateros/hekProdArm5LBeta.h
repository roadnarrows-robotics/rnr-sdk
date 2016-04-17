////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekProd5LBeta.h
//
/*! \file
 *
 * $LastChangedDate: 2014-09-18 16:53:49 -0600 (Thu, 18 Sep 2014) $
 * $Rev: 3748 $
 *
 * \brief Hekateros 5 DoF long beta robotic arm static specification.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 * \author Rob Shiely     (rob@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2012-2013.  RoadNarrows
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

#ifndef _HEK_PROD_ARM_5L_BETA_H
#define _HEK_PROD_ARM_5L_BETA_H

#include "Dynamixel/Dynamixel.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekOptical.h"
#include "Hekateros/hekSpec.h"


/*!
 * \ingroup hek_spec
 * \defgroup hek_prod_5l_beta  Hekateros 5 DoF Long Beta Specification
 *
 * \{
 */

/*! product id */
#define HEK_5L_BETA_PRODUCT_ID HEK_PRODUCT_ID(HekProdSizeLong, 5, HEK_PROD_BETA)

/*! hardware version */
#define HEK_5L_BETA_VERSION     HEK_VERSION(0, 9, 2)

namespace hekateros
{
  const int HekProdArm5LBetaId        = HEK_5L_BETA_PRODUCT_ID; ///< product id
  const int HekProdArm5LBetaVersion   = HEK_5L_BETA_VERSION;    ///< hw version

  const int HekProdArm5LBetaNumLinks      = 5;  ///< number of fixed links
  const int HekProdArm5LBetaDoF           = 5;  ///< degrees of freedom 
  const int HekProdArm5LBetaNumOptLimits  = 0;  ///< number of optical limits
  const int HekProdArm5LBetaNumServos     = 6;  ///< number of servos

  /*!
   * \brief Specification of links.
   */
  extern const HekSpecLink_T HekProdArm5LBetaSpecLinks[];

  /*!
   * \brief Specification of joints.
   */
  extern const HekSpecJoint_T  HekProdArm5LBetaSpecJoints[];

  /*!
   * \brief Specification of servos.
   */
  extern const HekSpecServo_T HekProdArm5LBetaSpecServos[];

} // namespace hekateros

/*! \} */


#endif // _HEK_PROD_ARM_5L_BETA_H
