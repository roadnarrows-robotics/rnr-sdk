////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// Library:   librnr
//
// File:      color.c
//
/*! \file
 *
 * $LastChangedDate: 2013-10-03 10:33:30 -0600 (Thu, 03 Oct 2013) $
 * $Rev: 3344 $
 *
 * \brief Non-GUI color systems.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \pkgcopyright{2013-2018,RoadNarrows LLC.,http://www.roadnarrows.com}
 *
 * \EulaBegin
 * See the README and EULA files for any copyright and licensing information.
 * \EulaEnd
 */
////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdarg.h>

#include "rnr/rnrconfig.h"
#include "rnr/color.h"

int colorprintf(const char *sFgColor, const char *sFmt, ...)
{
  va_list ap;
  int     n;

  set_color(sFgColor);
  va_start(ap, sFmt);
  n = vfprintf(stdout, sFmt, ap);
  va_end(ap);
  reset_colors();
  return n;
}
