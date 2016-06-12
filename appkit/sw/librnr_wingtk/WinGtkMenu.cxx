////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Library:   librnr_wingtk
//
// File:      WinGtkMenu.cxx
//
/*! \file
 *
 * $LastChangedDate: 2013-05-03 07:45:13 -0600 (Fri, 03 May 2013) $
 * $Rev: 2904 $
 *
 * \brief RoadNarrows Robotics GTK derived window button menu implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011-2016  RoadNarrows
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

#include <sys/types.h>
#include <limits.h>
#include <libgen.h>
#include <unistd.h>
#include <stdarg.h>

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/appkit/Win.h"
#include "rnr/appkit/WinMenu.h"
#include "rnr/appkit/WinGtk.h"
#include "rnr/appkit/WinGtkMenu.h"


using namespace std;
using namespace rnr;


//.............................................................................
// Class WinGtkButton
//.............................................................................

WinGtkButton::WinGtkButton(WinButtonType eType,
                           int           nEvent,
                           WidgetState   eInitState,
                           AlignOp       eAlign,
                           const string &strAltText,
                           const string &strTagNormal,
                           const string &strToolTipNormal,
                           const string &strTagActive,
                           const string &strToolTipActive) :
        WinButton(eType, nEvent, eInitState, eAlign, strAltText,
                  strTagNormal, strToolTipNormal,
                  strTagActive, strToolTipActive)
{
  if( eType == WinButtonTypeImage )
  {
    m_wImgNormal = loadIcon(strTagNormal);
    m_wImgActive = loadIcon(strTagActive);
  }
  else
  {
    m_wImgNormal = NULL;
    m_wImgActive = NULL;
  }
}

WinGtkButton::WinGtkButton(const WinGtkButton &src) : WinButton(src)
{
  if( m_eType == WinButtonTypeImage )
  {
    m_wImgNormal = loadIcon(m_strTagNormal);
    m_wImgActive = loadIcon(m_strTagActive);
  }
  else
  {
    m_wImgNormal = NULL;
    m_wImgActive = NULL;
  }
}

WinGtkButton::~WinGtkButton()
{
  if( m_wImgNormal != NULL )
  {
    gtk_widget_destroy(m_wImgNormal);
  }

  if( m_wImgActive != NULL )
  {
    gtk_widget_destroy(m_wImgActive);
  }
}

GtkWidget *WinGtkButton::loadIcon(const string &strIconPath)
{
  GtkWidget  *wBttnImg;

  if( access(strIconPath.c_str(), F_OK|R_OK) != 0 )
  {
    LOGSYSERROR("%s: %s(errno=%d)", strIconPath.c_str());
    return NULL;
  }

  // never returns null - will return "broken" icon instead
  wBttnImg = gtk_image_new_from_file(strIconPath.c_str());

  // menu item owns this widget
  gtk_widget_ref(wBttnImg);

  return wBttnImg;
}


//.............................................................................
// Class WinGtkButtonMenu
//.............................................................................

bool WinGtkButtonMenu::addImageButton(int           nEvent,
                                      WidgetState   eInitState,
                                      AlignOp       eAlign,
                                      const string &strAltText,
                                      const string &strIconNormal,
                                      const string &strToolTipNormal,
                                      const string &strIconActive,
                                      const string &strToolTipActive)
{
  // button associated with the given event already exists
  if( m_mapButtons.find(nEvent) != m_mapButtons.end() )
  {
    return false;
  }
  
  string strIconPathNormal = makeIconPath(strIconNormal);
  string strIconPathActive = makeIconPath(strIconActive);

  // add image button
  if( !strIconPathNormal.empty() )
  {
    m_mapButtons[nEvent] = new WinGtkButton(WinButtonTypeImage,
                                            nEvent,
                                            eInitState,
                                            eAlign,
                                            strAltText,
                                            strIconPathNormal,
                                            strToolTipNormal,
                                            strIconPathActive,
                                            strToolTipActive);

  }

  // no image, make an label button instead
  else
  {
    m_mapButtons[nEvent] = new WinGtkButton(WinButtonTypeLabel,
                                            nEvent,
                                            eInitState,
                                            eAlign,
                                            "",
                                            strAltText,
                                            strToolTipNormal);
  }

  return true;
}

bool WinGtkButtonMenu::addLabelButton(int           nEvent,
                                      WidgetState   eInitState,
                                      AlignOp       eAlign,
                                      const string &strLabelNormal,
                                      const string &strToolTipNormal,
                                      const string &strLabelActive,
                                      const string &strToolTipActive)
{
  // button associated with the given event already exists
  if( m_mapButtons.find(nEvent) != m_mapButtons.end() )
  {
    return false;
  }
  
  // add label button
  m_mapButtons[nEvent] = new WinGtkButton(WinButtonTypeLabel,
                                          nEvent,
                                          eInitState,
                                          eAlign,
                                          "",
                                          strLabelNormal,
                                          strToolTipNormal,
                                          strLabelActive,
                                          strToolTipNormal);

  return true;
}
