////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Library:   librnr_win
//
// File:      WinMenu.cxx
//
/*! \file
 *
 * $LastChangedDate: 2013-05-03 07:45:13 -0600 (Fri, 03 May 2013) $
 * $Rev: 2904 $
 *
 * \brief RoadNarrows Robotics window base window button menu implementation.
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
#include <stdio.h>
#include <libgen.h>
#include <unistd.h>
#include <stdarg.h>
#include <errno.h>

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/appkit/Win.h"
#include "rnr/appkit/WinMenu.h"


using namespace std;
using namespace rnr;


//.............................................................................
// Class WinButton
//.............................................................................

void WinButton::copy(const WinButton *pSrc)
{
  m_eType             = pSrc->m_eType;
  m_nEvent            = pSrc->m_nEvent;
  m_eInitState        = pSrc->m_eInitState;
  m_eState            = pSrc->m_eInitState;
  m_eAlign            = pSrc->m_eAlign;
  m_strAltText        = pSrc->m_strAltText;
  m_strTagNormal      = pSrc->m_strTagNormal;
  m_strToolTipNormal  = pSrc->m_strToolTipNormal;
  m_strTagActive      = pSrc->m_strTagActive;
  m_strToolTipActive  = pSrc->m_strToolTipActive;
}

void *WinButton::getCurrentImageWidget()
{
  if( m_eType != WinButtonTypeImage )
  {
    return NULL;
  }
  else if( (m_eState == WidgetStateActive) && (getActiveImageWidget() != NULL) )
  {
    return getActiveImageWidget();
  }
  else
  {
    return getNormalImageWidget();
  }
} 

string WinButton::getCurrentLabel()
{
  if( m_eType != WinButtonTypeLabel )
  {
    return "";
  }
  else if( (m_eState == WidgetStateActive) && !m_strTagActive.empty() )
  {
    return m_strTagActive;
  }
  else
  {
    return m_strTagNormal;
  }
}

string WinButton::getCurrentToolTip()
{
  if( m_eState == WidgetStateActive )
  {
    switch( m_eType )
    {
      case WinButtonTypeImage:
        if( getActiveImageWidget() != NULL )
        {
          return m_strToolTipActive;
        }
        break;
      case WinButtonTypeLabel:
        if( !m_strTagActive.empty() )
        {
          return m_strToolTipActive;
        }
        break;
      default:
        break;
    }
  }
  return m_strToolTipNormal;
}


//.............................................................................
// Class WinButtonMenu
//.............................................................................

WinButtonMenu::~WinButtonMenu()
{
  MapBttns_T::iterator  iter;

  for(iter=m_mapButtons.begin(); iter!=m_mapButtons.end(); ++iter)
  {
    delete iter->second;
  }
  m_mapButtons.clear();
}

void WinButtonMenu::bind(Win *pWin)
{
  MapBttns_T::iterator  iter;
  WinButton            *pButton;

  if( pWin == NULL )
  {
    LOGERROR("Binding window is NULL.");
    return;
  }

  // unbind any previous menu
  if( m_pWin != NULL )
  {
    unbind();
  }

  m_pWin = pWin;

  // add menu items to bound window
  for(iter=m_mapButtons.begin(); iter!=m_mapButtons.end(); ++iter)
  {
    pButton = iter->second;

    switch( pButton->m_eType )
    {
      case WinButtonTypeImage:
        m_pWin->addImageButton(pButton->m_nEvent,
                               pButton->m_eAlign,
                               pButton->getCurrentImageWidget(),
                               pButton->m_strAltText,
                               pButton->getCurrentToolTip());
        break;
      case WinButtonTypeLabel:
        m_pWin->addLabelButton(pButton->m_nEvent,
                               pButton->m_eAlign,
                               pButton->getCurrentLabel(),
                               pButton->getCurrentToolTip());
        break;
      default:
        break;
    }
  }

  // reset menu item states
  resetAllButtonStates();

  // register menu button press callback
  m_pWin->registerButtonClickCallback(WinButtonMenu::onMenuButtonClick, this);
}

void WinButtonMenu::unbind()
{
  if( m_pWin != NULL )
  {
    m_pWin->unregisterButtonClickCallback();
    m_pWin->removeAllButtons();
    m_pWin = NULL;
    resetAllButtonStates();
  }
}

void WinButtonMenu::setButtonStateList(const string &strEvent, ...)
{
  va_list               ap;
  size_t                i;
  int                   nEvent;
  WidgetState           eNewState;
  MapBttns_T::iterator  pos;

  LOGDIAG3("'%s' bulk menu state changes.", strEvent.c_str());

  va_start(ap, strEvent);

  //
  // Loop through button map, but limit to number of buttons.
  //
  for(i=0; i<m_mapButtons.size(); ++i)
  {
    nEvent   = va_arg(ap, int);

    // stop criteria
    if( nEvent == UIEventNone )
    {
      break;
    }

    eNewState = (WidgetState)va_arg(ap, int);

    if( (pos = m_mapButtons.find(nEvent)) != m_mapButtons.end() )
    {
      changeButtonState(pos->second, eNewState);
    }
  }

  va_end(ap);
}

void WinButtonMenu::resetAllButtonStates()
{
  MapBttns_T::iterator  iter;

  for(iter=m_mapButtons.begin(); iter!=m_mapButtons.end(); ++iter)
  {
    changeButtonState(iter->second, iter->second->m_eInitState);
  }
}

string WinButtonMenu::makeIconPath(const string &strIconFile)
{
  string  strPath;                      // working path of search directories
  string  strIconFqName;                // icon fully-qualified file name
  size_t  pos1;                         // substring starting position 1
  size_t  pos2;                         // substring ending position 2

  //
  // No file name. Note this is not an error.
  //
  if( strIconFile.empty() )
  {
    return strIconFqName;
  }

  //
  // Icon file name is an absolute path.
  //
  if( strIconFile[0] == DIR_SEP_CHAR )
  {
    strIconFqName = strIconFile;
    if( access(strIconFqName.c_str(), F_OK|R_OK) != 0 )
    {
      LOGWARN("%s: %s(errno=%d)",
          strIconFqName.c_str(), strerror(errno), errno);
      strIconFqName.clear();
    }
    return strIconFqName;
  }

  //
  // Search for file.
  //
  if( m_strIconPath.empty() )
  {
    strPath = ".";
  }
  else
  {
    strPath = m_strIconPath + PATH_SEP_STR + ".";
  }

  for(pos1=0, pos2=strPath.find(PATH_SEP_STR, pos1);
      pos1 != strPath.npos;
      pos1=pos2+1, pos2=strPath.find(PATH_SEP_STR, pos1))
  {
    strIconFqName = strPath.substr(pos1, pos2) + DIR_SEP_STR + strIconFile; 
    if( access(strIconFqName.c_str(), F_OK|R_OK) == 0 )
    {
      return strIconFqName;
    }
  }

  //
  // Failed to find icon file.
  //
  LOGWARN("%s: Icon not found in %s.", strIconFile.c_str(), strPath.c_str());

  strIconFqName.clear();

  return strIconFqName;
}

void WinButtonMenu::changeButtonState(WinButton *pButton, WidgetState eNewState)
{
  WidgetState   eCurState;

  eCurState         = pButton->m_eState;
  pButton->m_eState = eNewState;

  if( m_pWin != NULL )
  {
    switch( pButton->m_eType )
    {
      case WinButtonTypeImage:
        if( pButton->getActiveImageWidget() != NULL )
        {
          // normal,disabled --> active
          if( (eCurState != WidgetStateActive) &&
              (eNewState == WidgetStateActive) )
          {
            m_pWin->replaceButtonImage(pButton->m_nEvent,
                                       pButton->getActiveImageWidget(),
                                       pButton->m_strToolTipActive);
          }
          // active --> normal,disabled
          else if( (eCurState == WidgetStateActive) &&
                   (eNewState != WidgetStateActive) )
          {
            m_pWin->replaceButtonImage(pButton->m_nEvent,
                                       pButton->getNormalImageWidget(),
                                       pButton->m_strToolTipNormal);
          }
        }
        break;
      case WinButtonTypeLabel:
        if( !pButton->m_strTagActive.empty() )
        {
          // normal,disabled --> active
          if( (eCurState != WidgetStateActive) &&
              (eNewState == WidgetStateActive) )
          {
            m_pWin->replaceButtonLabel(pButton->m_nEvent,
                                       pButton->m_strTagActive,
                                       pButton->m_strToolTipActive);
          }
          // active --> normal,disabled
          else if( (eCurState == WidgetStateActive) &&
                   (eNewState != WidgetStateActive) )
          {
            m_pWin->replaceButtonImage(pButton->m_nEvent,
                                       pButton->m_strTagNormal,
                                       pButton->m_strToolTipNormal);
          }
        }
        break;
    }

    // show gui widget state
    m_pWin->showButtonState(pButton->m_nEvent, eNewState);
  }
}

void WinButtonMenu::onMenuButtonClick(int nEvent, void *user_data)
{
  WinButtonMenu        *pMenu = (WinButtonMenu *)user_data;
  MapBttns_T::iterator  pos;
  WidgetState           eState;

  pMenu->setCurrentEvent(nEvent);
  
  if( (pos = pMenu->m_mapButtons.find(nEvent)) != pMenu->m_mapButtons.end() )
  {
    eState = pos->second->m_eState;

    // toggle state
    if( eState == WidgetStateNormal )
    {
      pMenu->changeButtonState(pos->second, WidgetStateActive);
    }
    else if( eState == WidgetStateActive )
    {
      pMenu->changeButtonState(pos->second, WidgetStateNormal);
    }
  }
}
