////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekProd4S.h
//
/*! \file
 *
 * $LastChangedDate: 2014-09-18 16:53:49 -0600 (Thu, 18 Sep 2014) $
 * $Rev: 3748 $
 *
 * \brief Hekateros 4 DoF short robotic arm static specification.
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

#ifndef _HEK_PROD_ARM_4S_H
#define _HEK_PROD_ARM_4S_H

#include "Dynamixel/Dynamixel.h"

#include "Hekateros/hekateros.h"


/*!
 * \ingroup hek_spec
 * \defgroup hek_prod_4s  Hekateros 4 DoF Short Specification
 *
 * \{
 */

/*! product id */
#define HEK_4S_PRODUCT_ID   HEK_PRODUCT_ID(HekProdSizeShort, 4, 0)

/*! hardware version */
#define HEK_4S_VERSION      HEK_VERSION(1, 0, 0)

namespace hekateros
{
  const int HekProdArm4SId        = HEK_4S_PRODUCT_ID; ///< product id
  const int HekProdArm4SVersion   = HEK_4S_VERSION;    ///< hw version

  const int HekProdArm4SNumLinks  = 4;  ///< number of fixed links
  const int HekProdArm4SDoF       = 4;  ///< degrees of freedom 
  const int HekProdArm4SNumServos = 5;  ///< number of servos


} // namespace hekateros

/*! \} */


#endif // _HEK_PROD_ARM_4S_H
