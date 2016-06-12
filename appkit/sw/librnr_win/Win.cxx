////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Library:   librnr_win
//
// File:      Win.cxx
//
/*! \file
 *
 * $LastChangedDate: 2013-05-03 07:45:13 -0600 (Fri, 03 May 2013) $
 * $Rev: 2904 $
 *
 * \brief RoadNarrows Robotics Win abstract base class implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011-2016.  RoadNarrows
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

#include <string>
#include <map>

#include "rnr/rnrconfig.h"

#include "opencv2/core/core.hpp"

#include "rnr/appkit/WinLookFeel.h"
#include "rnr/appkit/Win.h"

using namespace std;
using namespace cv;
using namespace rnr;

Win::Win(const string &strWinName,
         int           nWidth,
         int           nHeight,
         bool          bDecorate) :
      m_strWinName(strWinName),
      m_nWinWidth(nWidth),
      m_nWinHeight(nHeight),
      m_bDecorate(bDecorate)
{
  if( m_strWinName.empty() )
  {
    m_strWinName = "Main";
  }

  for(int i=0; i<MaxCvImages; ++i)
  {
    m_funcMouseCb[i] = NULL;
    m_dataMouseCb[i] = NULL;
  }

  m_funcKeyCb   = NULL;
  m_dataKeyCb   = NULL;
  m_funcBttnCb  = NULL;
  m_dataBttnCb  = NULL;
  m_uGstWinXid  = 0;
  m_uLastKey    = 0;
  m_bMouseEvent = false;

  setLookAndFeelDefaults();
}

void Win::setLookAndFeelDefaults()
{
  m_mapLookFeel["color_win_bg"]         = GuiStrColorWinBg;
  m_mapLookFeel["color_status_fg"]      = GuiStrColorStatusFg;
  m_mapLookFeel["color_status_bg"]      = GuiStrColorStatusBg;
  m_mapLookFeel["color_status_border"]  = GuiStrColorStatusBorder;
  m_mapLookFeel["color_button_bg"]      = GuiStrColorBttnBg;
  m_mapLookFeel["color_text_fg"]        = GuiStrColorTextFg;
  m_mapLookFeel["color_text_bg"]        = GuiStrColorTextBg;
  m_mapLookFeel["color_image_bg"]       = GuiStrColorImageBg;
  m_mapLookFeel["color_rn_black"]       = GuiStrColorRNBlack;
  m_mapLookFeel["color_rn_white"]       = GuiStrColorRNWhite;
  m_mapLookFeel["color_rn_red"]         = GuiStrColorRNRed;
  m_mapLookFeel["color_rn_yellow"]      = GuiStrColorRNYellow;
  m_mapLookFeel["font_text"]            = GuiStrFontSmall;
  m_mapLookFeel["font_status"]          = GuiStrFontSmall;
  m_mapLookFeel["font_large"]           = GuiStrFontLarge;
  m_mapLookFeel["font_medium"]          = GuiStrFontMedium;
  m_mapLookFeel["font_small"]           = GuiStrFontSmall;
  m_mapLookFeel["font_tiny"]            = GuiStrFontTiny;
}
