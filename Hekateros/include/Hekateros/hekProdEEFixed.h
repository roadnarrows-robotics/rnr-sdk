////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekProdEEFixed.h
//
/*! \file
 *
 * $LastChangedDate: 2014-09-18 16:53:49 -0600 (Thu, 18 Sep 2014) $
 * $Rev: 3748 $
 *
 * \brief Hekateros 0 DoF family of fixed end effectors.
 *
 * Fixed end effectors have no moving parts but may contain lights, cameras,
 * etc.
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

#ifndef _HEK_PROD_EE_FIXED_H
#define _HEK_PROD_EE_FIXED_H

#include "Dynamixel/Dynamixel.h"

#include "Hekateros/hekateros.h"


/*!
 * \ingroup hek_spec
 * \defgroup hek_prod_4l  Hekateros 4 DoF Long Specification
 *
 * \{
 */

/*! end effector product family */
#define HEK_EE_FIXED_FAMILY       0x01

/*! product id */
#define HEK_EE_FIXED_PRODUCT_ID \
  HEK_EE_PRODUCT_ID(HEK_EE_FIXED_FAMILY, HekProdSizeStd, 1, 0)

/*! hardware version */
#define HEK_EE_FIXED_VERSION      HEK_VERSION(1, 0, 0)

namespace hekateros
{
  const int HekProdEEFixedFamily    = HEK_EE_FIXED_FAMILY;     ///< family
  const int HekProdEEFixedId        = HEK_EE_FIXED_PRODUCT_ID; ///< prod id
  const int HekProdEEFixedVersion   = HEK_EE_FIXED_VERSION;    ///< hw ver

  const int HekProdEEFixedNumLinks  = 1;  ///< number of fixed links
  const int HekProdEEFixedDoF       = 0;  ///< degrees of freedom 
  const int HekProdEEFixedNumServos = 0;  ///< number of servos

  /*!
   * \brief Link lengths (mm).
   */
  const double HekProdEEFixedLinkLengths[HekProdEEFixedNumLinks] =
  {
    0.0       // length from end effector zero point to tip
  };

} // namespace hekateros

/*! \} */


#endif // _HEK_PROD_EE_FIXED_H
