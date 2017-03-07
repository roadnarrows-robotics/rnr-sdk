////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library    librnr_win
//
// File:      LookFeel.h
//
/*! \file
 *
 * $LastChangedDate: 2013-05-03 07:45:13 -0600 (Fri, 03 May 2013) $
 * $Rev: 2904 $
 *
 * \brief RoadNarrows top-level look and feel user interface declarations.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2012-2017.  RoadNarrows LLC
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
 * 
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
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
 * OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _RNR_WIN_LOOK_FEEL_H
#define _RNR_WIN_LOOK_FEEL_H

#include "opencv/cv.h"
#include "opencv/highgui.h"

namespace rnr
{
  // ...........................................................................
  // GUI COLORS AND STYLES
  // ...........................................................................

  //
  // Colors and Fonts
  //
  const char* const GuiStrColorWinBg        = "#000000";  ///< window bg color
  const char* const GuiStrColorStatusFg     = "#fed700";  ///< status fg color
  const char* const GuiStrColorStatusBg     = "#000000";  ///< status bg color
  const char* const GuiStrColorStatusBorder = "#aa0000";  ///< status border
  const char* const GuiStrColorBttnBg       = "#e6e6e6";  ///< button bg color
  const char* const GuiStrColorTextFg       = "#ffffff";  ///< text fg color
  const char* const GuiStrColorTextBg       = "#000000";  ///< text bg color
  const char* const GuiStrColorImageBg      = "#ffffff";  ///< image bg color

  const char* const GuiStrColorRNRed        = "#aa0000";  ///< RN red
  const char* const GuiStrColorRNBlack      = "#000000";  ///< RN black
  const char* const GuiStrColorRNWhite      = "#ffffff";  ///< RN white
  const char* const GuiStrColorRNYellow     = "#fed700";  ///< RN yellow
  
  const char* const GuiStrFontLarge         = "Arial 20"; ///< large font
  const char* const GuiStrFontMedium        = "Arial 15"; ///< medium font
  const char* const GuiStrFontSmall         = "Arial 12"; ///< small font
  const char* const GuiStrFontTiny          = "Arial 8";  ///< tiny font
} // namespace rnr


#endif // _RNR_WIN_LOOK_FEEL_H
