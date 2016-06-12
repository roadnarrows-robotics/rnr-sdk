////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Library:   librnr_win
//
// File:      StateWin.cxx
//
/*! \file
 *
 * $LastChangedDate: 2013-07-13 14:12:15 -0600 (Sat, 13 Jul 2013) $
 * $Rev: 3126 $
 *
 * \brief GUI window StateWin derived state class implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2012-2016.  RoadNarrows LLC
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

#include <stdio.h>

#include <string>
#include <map>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/appkit/Win.h"
#include "rnr/appkit/WinMenu.h"
#include "rnr/appkit/WinCvMouse.h"
#include "rnr/appkit/SessionWin.h"
#include "rnr/appkit/State.h"
#include "rnr/appkit/StateWin.h"

using namespace std;
using namespace rnr;


//------------------------------------------------------------------------------
// StateKb Class
//------------------------------------------------------------------------------

int StateWin::receiveEvent()
{
  int   nEventId;
  
  // wait
  nEventId = m_session.win().waitKey(m_usecTimeOut / 1000);
  
  // keyboard event
  if( m_bKbEvents && (nEventId >= 0) )
  {
    return nEventId;
  }

  // button menu event
  else if( (m_pButtons != NULL) &&
           ((nEventId = m_pButtons->getCurrentEvent()) != UIEventNone) )
  {
    m_pButtons->setCurrentEvent(UIEventNone);
    return nEventId;
  }

  // mouse event
  else if( m_bMouseEvents )
  {
    nEventId = m_mouse.getCurrentEvent();
    m_mouse.setCurrentEvent(UIEventNone);
  }

  // no event
  else
  {
    nEventId = UIEventNone;
  }

  return nEventId;
}

void StateWin::actionEnterState(int nPrevStateId, int nEventId)
{
  if( !m_bOneTimeInit )
  {
    initOnceGuiInterface();
  }

  // build (create) interface
  buildGuiInterface();

  // bind menu to window
  m_pButtons->bind(m_session.getWin());

  // page identifier
  m_session.win().showPageRef(m_strRefTag);

  // enable/disable/toggle menu button states
  setButtonStates();

  // set local state context in session
  m_session.setContext(this);

  // show interface
  showGuiInterface();
}

void StateWin::actionExitState(int nNextStateId, int nEventId)
{
  m_pButtons->unbind();

  m_session.win().showPageRef("");
  
  destroyGuiInterface();
}
