////////////////////////////////////////////////////////////////////////////////
/*! \file
 *
 * \brief Non-GUI color systems.
 *
 * Supported Color Systems:
 *  - ANSI escape sequence colors.
 *
 * \pkgsynopsis
 * RoadNarrows Robotics Common Library 1
 *
 * \pkgcomponent{Library}
 * librnr
 *
 * \pkgfile{rnr/color.h}
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2013-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 *
 * \license{MIT}
 *
 * \EulaBegin
 * Permission is hereby granted, without written agreement and without
 * license or royalty fees, to use, copy, modify, and distribute this
 * software and its documentation for any purpose, provided that
 * (1) The above copyright notice and the following two paragraphs
 * appear in all copies of the source code and (2) redistributions
 * including binaries reproduces these notices in the supporting
 * documentation.   Substantial modifications to this software may be
 * copyrighted by their authors and need not follow the licensing terms
 * described here, provided that the new terms are clearly indicated in
 * all files where they apply.
 * \n\n
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
 * OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * \n\n
 * THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * \EulaEnd
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _RNR_COLOR_H
#define _RNR_COLOR_H

#include <stdio.h>

#include "rnr/rnrconfig.h"

C_DECLS_BEGIN

/*!
 * \defgroup rnr_color_ansi
 *
 * ANSI escape color codes.
 *
 * \{
 */

#define ANSI_CSI  "\033["     ///< Control Sequence Introducer

//
// Start, separator, and end color escape sequences.
//
#define ANSI_COLOR_PRE    ANSI_CSI        ///< color escape sequence prefix
#define ANSI_COLOR_SEP    ";"             ///< color values separator
#define ANSI_COLOR_MODE   "m"             ///< color mode

//
// Select Graphic Rendition text attribute codes.
//
#define ANSI_SGR_TEXT_NORMAL        "0"   ///< color normal and reset to default
#define ANSI_SGR_TEXT_BOLD          "1"   ///< bold or increase intensity
#define ANSI_SGR_TEXT_UNDERSCORE    "4"   ///< underscore
#define ANSI_SGR_TEXT_BLINK         "5"   ///< slow blink
#define ANSI_SGR_TEXT_REVERSE       "7"   ///< swap fg and bg colors

//
// Select Graphic Rendition foreground color codes.
//
#define ANSI_SGR_FG_COLOR_BLACK         "30"     ///< normal black
#define ANSI_SGR_FG_COLOR_RED           "31"     ///< normal red
#define ANSI_SGR_FG_COLOR_GREEN         "32"     ///< normal green
#define ANSI_SGR_FG_COLOR_YELLOW        "33"     ///< normal yellow (brown)
#define ANSI_SGR_FG_COLOR_BLUE          "34"     ///< normal blue
#define ANSI_SGR_FG_COLOR_MAGENTA       "35"     ///< normal magenta
#define ANSI_SGR_FG_COLOR_CYAN          "36"     ///< normal cyan
#define ANSI_SGR_FG_COLOR_WHITE         "37"     ///< normal white (gray)

//
// Select Graphic Rendition background color codes.
//
#define ANSI_SGR_BG_COLOR_BLACK         "40"     ///< normal black
#define ANSI_SGR_BG_COLOR_RED           "41"     ///< normal red
#define ANSI_SGR_BG_COLOR_GREEN         "42"     ///< normal green
#define ANSI_SGR_BG_COLOR_YELLOW        "43"     ///< normal yellow (brown)
#define ANSI_SGR_BG_COLOR_BLUE          "44"     ///< normal blue
#define ANSI_SGR_BG_COLOR_MAGENTA       "45"     ///< normal magenta
#define ANSI_SGR_BG_COLOR_CYAN          "46"     ///< normal cyan
#define ANSI_SGR_BG_COLOR_WHITE         "47"     ///< normal white (gray)

/*!
 * \brief Macro to build color escape sequence string.
 *
 * \param _attr Text attribute.
 * \param _fg   Foreground color.
 * \param _bg   Background color.
 */
#define ANSI_COLOR(_attr, _fg, _bg) \
  ANSI_COLOR_PRE _attr ANSI_COLOR_SEP _fg ANSI_COLOR_SEP _bg ANSI_COLOR_MODE

/*!
 * \brief Macro to build normal foreground color escape sequence string.
 *
 * Background color is left at current setting.
 *
 * \param _fg   Foreground color.
 */
#define ANSI_FG_COLOR(_fg) \
  ANSI_COLOR_PRE ANSI_SGR_TEXT_NORMAL ANSI_COLOR_SEP _fg ANSI_COLOR_MODE

/*!
 * \brief Macro to build bright foreground color escape sequence string.
 *
 * Background color is left at current setting.
 *
 * \param _fg   Foreground color.
 */
#define ANSI_FG_BRIGHT_COLOR(_fg) \
  ANSI_COLOR_PRE ANSI_SGR_TEXT_BOLD ANSI_COLOR_SEP _fg ANSI_COLOR_MODE

/*!
 * \brief Macro to build foreground color with the given text attribute
 * escape sequence string.
 *
 * Background color is left at current setting.
 *
 * \param _attr Text attribute.
 * \param _fg   Foreground color.
 */
#define ANSI_FG_ATTR_COLOR(_attr, _fg) \
  ANSI_COLOR_PRE _attr ANSI_COLOR_SEP _fg ANSI_COLOR_MODE

/*!
 * \brief Macro to build background color escape sequence string.
 *
 * Foreground color is left at current setting.
 *
 * \param _bg   Foreground color.
 */
#define ANSI_BG_COLOR(_bg)  ANSI_COLOR_PRE _bg ANSI_COLOR_MODE

/*!
 * \brief Macro to build normal color escape sequence string.
 * 
 * Actual colors are dependent on the terminal emulator.
 *
 * \param _fg   Foreground color.
 * \param _bg   Background color.
 */
#define ANSI_NORMAL_COLOR(_fg, _bg) ANSI_COLOR(ANSI_SGR_TEXT_NORMAL, _fg, _bg)

/*!
 * \brief Macro to build bright color escape sequence string.
 * 
 * Actual colors are dependent on the terminal emulator.
 *
 * \param _fg   Foreground color.
 * \param _bg   Background color.
 */
#define ANSI_BRIGHT_COLOR(_fg, _bg) ANSI_COLOR(ANSI_SGR_TEXT_BOLD, _fg, _bg)

//
// Handy, common color escape sequences.
//

/*!
 * \brief color reset to default
 */
#define ANSI_COLOR_RESET  \
  ANSI_CSI ANSI_SGR_TEXT_NORMAL ANSI_COLOR_MODE

/*!
 * \brief Foreground colors escape sequences.
 * \{
 */
#define ANSI_FG_BLACK   ANSI_FG_COLOR(ANSI_SGR_FG_COLOR_BLACK)
#define ANSI_FG_RED     ANSI_FG_COLOR(ANSI_SGR_FG_COLOR_RED)
#define ANSI_FG_GREEN   ANSI_FG_COLOR(ANSI_SGR_FG_COLOR_GREEN)
#define ANSI_FG_YELLOW  ANSI_FG_COLOR(ANSI_SGR_FG_COLOR_YELLOW)
#define ANSI_FG_BLUE    ANSI_FG_COLOR(ANSI_SGR_FG_COLOR_BLUE)
#define ANSI_FG_MAGENTA ANSI_FG_COLOR(ANSI_SGR_FG_COLOR_MAGENTA)
#define ANSI_FG_CYAN    ANSI_FG_COLOR(ANSI_SGR_FG_COLOR_CYAN)
#define ANSI_FG_WHITE   ANSI_FG_COLOR(ANSI_SGR_FG_COLOR_WHITE)

#define ANSI_FG_BRIGHT_BLACK    ANSI_FG_BRIGHT_COLOR(ANSI_SGR_FG_COLOR_BLACK)
#define ANSI_FG_BRIGHT_RED      ANSI_FG_BRIGHT_COLOR(ANSI_SGR_FG_COLOR_RED)
#define ANSI_FG_BRIGHT_GREEN    ANSI_FG_BRIGHT_COLOR(ANSI_SGR_FG_COLOR_GREEN)
#define ANSI_FG_BRIGHT_YELLOW   ANSI_FG_BRIGHT_COLOR(ANSI_SGR_FG_COLOR_YELLOW)
#define ANSI_FG_BRIGHT_BLUE     ANSI_FG_BRIGHT_COLOR(ANSI_SGR_FG_COLOR_BLUE)
#define ANSI_FG_BRIGHT_MAGENTA  ANSI_FG_BRIGHT_COLOR(ANSI_SGR_FG_COLOR_MAGENTA)
#define ANSI_FG_BRIGHT_CYAN     ANSI_FG_BRIGHT_COLOR(ANSI_SGR_FG_COLOR_CYAN)
#define ANSI_FG_BRIGHT_WHITE    ANSI_FG_BRIGHT_COLOR(ANSI_SGR_FG_COLOR_WHITE)
/*! \} */

/*!
 * \brief Background colors escape sequences.
 * \{
 */
#define ANSI_BG_BLACK   ANSI_FG_COLOR(ANSI_SGR_BG_COLOR_BLACK)
#define ANSI_BG_RED     ANSI_FG_COLOR(ANSI_SGR_BG_COLOR_RED)
#define ANSI_BG_GREEN   ANSI_FG_COLOR(ANSI_SGR_BG_COLOR_GREEN)
#define ANSI_BG_YELLOW  ANSI_FG_COLOR(ANSI_SGR_BG_COLOR_YELLOW)
#define ANSI_BG_BLUE    ANSI_FG_COLOR(ANSI_SGR_BG_COLOR_BLUE)
#define ANSI_BG_MAGENTA ANSI_FG_COLOR(ANSI_SGR_BG_COLOR_MAGENTA)
#define ANSI_BG_CYAN    ANSI_FG_COLOR(ANSI_SGR_BG_COLOR_CYAN)
#define ANSI_BG_WHITE   ANSI_FG_COLOR(ANSI_SGR_BG_COLOR_WHITE)
/*! \} */


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
  printf("%s%s%s", ANSI_COLOR_PRE, sColor, ANSI_COLOR_MODE);
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


#endif // _RNR_COLOR_H
