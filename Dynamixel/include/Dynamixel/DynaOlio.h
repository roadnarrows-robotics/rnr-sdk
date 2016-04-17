////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   libDynamixel
//
// File:      DynaOlio.h
//
/*! \file
 *
 * $LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
 * $Rev: 3845 $
 *
 * \ingroup dyna_lib_hdrs
 *
 * \brief Miscellaneous collection of useful utilities.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011-2015.  RoadNarrows LLC.
 * (http://www.roadnarrows.com)
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

#ifndef _DYNA_OLIO_H
#define _DYNA_OLIO_H

#include "rnr/rnrconfig.h"
#include "rnr/units.h"


/*!
 * \brief Return maximum integer of a and b.
 *
 * \param a Integer value a.
 * \param b Integer value b.
 *
 * \return Maximum value.
 */
INLINE_IN_H int imax(int a, int b)
{
  return a>=b? a: b;
}

/*!
 * \brief Return minimum integer of a and b.
 *
 * \param a Integer value a.
 * \param b Integer value b.
 *
 * \return Minimum value.
 */
INLINE_IN_H int imin(int a, int b)
{
  return a<=b? a: b;
}

/*!
 * \brief Return absolute value of a.
 *
 * \param a Integer value a.
 *
 * \return Absolute value.
 */
INLINE_IN_H int iabs(int a)
{
  return a>=0? a: -a;
}

/*!
 * \brief Return value of a within minimum,maximum range.
 *
 * \param a Integer value a.
 * \param m Minimum value of a.
 * \param M Maximum value of a.
 *
 * \return Adjusted value of a in range [m,M].
 */
INLINE_IN_H int irange(int a, int m, int M)
{
  return a<m? m: a>M? M: a;
}


/*!
 * \brief a mod b, \h_ge 0.
 *
 * param a  The dividend.
 * param b  The divisor.
 *
 * \return The remainder in [0, b-1].
 */
INLINE_IN_H int imod(int a, int b)
{
  a = a % b;
  return a >= 0? a: a+b;
}

/*!
 * \brief Convert units to short symbol name.
 *
 * \param u Units to look up.
 *
 * \return Symbol name string.
 */
INLINE_IN_H const char *dynaUnitsSym(units_t u)
{
  return u==units_raw? "": units_shortname(u);
}


#endif // _DYNA_OLIO_H
