////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// Library:   librnr
//
// File:      units.c
//
/*! \file
 *
 * $LastChangedDate: 2013-03-18 11:39:24 -0600 (Mon, 18 Mar 2013) $
 * $Rev: 2765 $
 *
 * \brief Units names, conversions, etc.
 *
 * \todo Add conversion functions to library.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2010-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
// Permission is hereby granted, without written agreement and without
// license or royalty fees, to use, copy, modify, and distribute this
// software and its documentation for any purpose, provided that
// (1) The above copyright notice and the following two paragraphs
// appear in all copies of the source code and (2) redistributions
// including binaries reproduces these notices in the supporting
// documentation.   Substantial modifications to this software may be
// copyrighted by their authors and need not follow the licensing terms
// described here, provided that the new terms are clearly indicated in
// all files where they apply.
//
// IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
// OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
// PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
// EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.
//
// THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
// "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
// PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
//
////////////////////////////////////////////////////////////////////////////////

#include <sys/types.h>
#include <stdlib.h>
#include <string.h>

#include "rnr/rnrconfig.h"
#include "rnr/assoc.h"
#include "rnr/units.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

/*!
 * Unit short names table. Singular form. 
 */
static Nvp_T UnitNamesTbl[] =
{
  {"?",           units_undef},
  {"raw",         units_raw},
  {"%",           units_percent},
  {"%o",          units_permil},
  {"norm",        units_norm},
  {"um",          units_um},
  {"mm",          units_mm},
  {"cm",          units_cm},
  {"m",           units_m},
  {"rad",         units_radians},
  {"deg",         units_degrees},
  {"um/s",        units_um_per_s},
  {"mm/s",        units_mm_per_s},
  {"cm/s",        units_cm_per_s},
  {"m/s",         units_m_per_s},
  {"rad/s",       units_rad_per_s},
  {"deg/s",       units_deg_per_s},
  {"rpm",         units_rpm},
  {"g",           units_g},
  {"m/s^2",       units_m_per_s2},
  {"kgf",         units_kgf},
  {"gf",          units_gf},
  {"newton",      units_newton},
  {"dyne",        units_dyne},
  {"V",           units_volt},
  {"A",           units_amp},
  {"ohm",         units_ohm},
  {"S",           units_conductance},
  {"F",           units_compacitance},
  {"T_F",         units_t_f},
  {"T_C",         units_t_c},
  {"T_K",         units_t_k}
};

// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Get the units name.
 *
 * \param u         Name-Value Pair table
 *
 * \return 
 * The associated units short name.
 */
const char *units_shortname(units_t u)
{
  return NvpVal2Name(UnitNamesTbl, arraysize(UnitNamesTbl), u);
}
