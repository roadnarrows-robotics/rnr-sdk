////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// Library:   librnr
//
// File:      color.h
//
/*! \file
 *
 * $LastChangedDate: 2013-10-03 10:33:30 -0600 (Thu, 03 Oct 2013) $
 * $Rev: 3344 $
 *
 * \brief Non-GUI color systems.
 *
 * Supported Color Systems:
 * \li ANSI escape sequence colors.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2013-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
//
// @EulaBegin@
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
// @EulaEnd@
//
////////////////////////////////////////////////////////////////////////////////

#ifndef _COLOR_H
#define _COLOR_H

#include <stdio.h>

#include "rnr/rnrconfig.h"

C_DECLS_BEGIN

/*!
 * \defgroup rnr_color_ansi
 *
 * ANSI escape code colors.
 *
 * \{
 */

#define ANSI_CSI  "\033["     ///< Control Sequence Introducer

//
// Start and end color sequences.
//
#define ANSI_COLOR_PRE    ANSI_CSI        ///< color escape sequence prefix
#define ANSI_COLOR_RESET  ANSI_CSI "0m"   ///< color reset to default

//
// Select Graphic Rendition foreground color parameters.
//
#define ANSI_SGR_FG_COLOR_BLACK         "0;30m"     ///< normal black
#define ANSI_SGR_FG_COLOR_RED           "0;31m"     ///< normal red
#define ANSI_SGR_FG_COLOR_GREEN         "0;32m"     ///< normal green
#define ANSI_SGR_FG_COLOR_YELLOW        "0;33m"     ///< normal yellow (brown)
#define ANSI_SGR_FG_COLOR_BLUE          "0;34m"     ///< normal blue
#define ANSI_SGR_FG_COLOR_MAGENTA       "0;35m"     ///< normal magenta
#define ANSI_SGR_FG_COLOR_CYAN          "0;36m"     ///< normal cyan
#define ANSI_SGR_FG_COLOR_GRAY          "0;37m"     ///< normal gray
#define ANSI_SGR_FG_COLOR_DARK_GRAY     "1;30m"     ///< light black
#define ANSI_SGR_FG_COLOR_LIGHT_RED     "1;31m"     ///< light red
#define ANSI_SGR_FG_COLOR_LIGHT_GREEN   "1;32m"     ///< light green
#define ANSI_SGR_FG_COLOR_LIGHT_YELLOW  "1;33m"     ///< light yellow
#define ANSI_SGR_FG_COLOR_LIGHT_BLUE    "1;34m"     ///< light blue
#define ANSI_SGR_FG_COLOR_LIGHT_MAGENTA "1;35m"     ///< light magenta
#define ANSI_SGR_FG_COLOR_LIGHT_CYAN    "1;36m"     ///< light cyan
#define ANSI_SGR_FG_COLOR_WHITE         "1;37m"     ///< white

//
// Select Graphic Rendition background color parameters.
//
#define ANSI_SGR_BG_COLOR_BLACK         "0;40m"     ///< normal black
#define ANSI_SGR_BG_COLOR_RED           "0;41m"     ///< normal red
#define ANSI_SGR_BG_COLOR_GREEN         "0;42m"     ///< normal green
#define ANSI_SGR_BG_COLOR_YELLOW        "0;43m"     ///< normal yellow (brown)
#define ANSI_SGR_BG_COLOR_BLUE          "0;44m"     ///< normal blue
#define ANSI_SGR_BG_COLOR_MAGENTA       "0;45m"     ///< normal magenta
#define ANSI_SGR_BG_COLOR_CYAN          "0;46m"     ///< normal cyan
#define ANSI_SGR_BG_COLOR_GRAY          "0;47m"     ///< normal gray
#define ANSI_SGR_BG_COLOR_DARK_GRAY     "1;40m"     ///< light black
#define ANSI_SGR_BG_COLOR_LIGHT_RED     "1;41m"     ///< light red
#define ANSI_SGR_BG_COLOR_LIGHT_GREEN   "1;42m"     ///< light green
#define ANSI_SGR_BG_COLOR_LIGHT_YELLOW  "1;43m"     ///< light yellow
#define ANSI_SGR_BG_COLOR_LIGHT_BLUE    "1;44m"     ///< light blue
#define ANSI_SGR_BG_COLOR_LIGHT_MAGENTA "1;45m"     ///< light magenta
#define ANSI_SGR_BG_COLOR_LIGHT_CYAN    "1;46m"     ///< light cyan
#define ANSI_SGR_BG_COLOR_WHITE         "1;47m"     ///< white

//-----------------------------------------------------------------------------
// Color Functions
//-----------------------------------------------------------------------------

/*!
 * \brief Set foreground or background color.
 *
 * The color is in effect until a new foreground/background color is specified
 * or the colors are reset to default values.
 *
 * \param sColor    ANSI SGR color string.
 */
INLINE_IN_H void set_color(const char *sColor)
{
  printf("%s%s", ANSI_COLOR_PRE, sColor);
}

/*!
 * \brief Reset foreground and background colors to defaults.
 */
INLINE_IN_H void reset_colors()
{
  printf("%s", ANSI_COLOR_RESET);
}

/*!
 * \brief Print output in the specified foreground color.
 *
 * After printing, the colors are reset to defaults. Subsequent output will be
 * in the default color.
 *
 * \param sFgColor    ANSI SGR foreground color string.
 * \param sFmt        Format string following printf(3) syntax.
 * \param ...         Variable argument list to print.
 *
 * \return
 * On success, returns the number of characters printed sans any color escape
 * sequents.\n
 * On output error, a negative value is returned.
 */
extern int colorprintf(const char *sFgColor, const char *sFmt, ...);

/*
 * \}
 */

C_DECLS_END


#endif // _COLOR_H
