////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Library:   librnr_win
//
// File:      WinCvMouse.cxx
//
// Package:   RoadNarrows Robotics Windowing Package
//
/*! \file
 *
 * $LastChangedDate: 2013-05-08 08:22:06 -0600 (Wed, 08 May 2013) $
 * $Rev: 2920 $
 *
 * \brief RoadNarrows Robotics base OpenCV Mouse class implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2011-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
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

#include <cstring>
#include <iostream>
#include <fstream>
#include <cmath>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "rnr/appkit/Win.h"
#include "rnr/appkit/WinCvMouse.h"
#include "rnr/appkit/WinOpenCv.h"

using namespace std;
using namespace cv;
using namespace rnr;

//.............................................................................
// Class WinCvMouse
//.............................................................................

WinCvMouse::WinCvMouse()
{
  m_pWin          = NULL;
  m_nCurrentEvent = UIEventNone;
  m_ptMouse       = nopoint;
  m_bDrag         = false;
  m_bDragState    = false;
}

WinCvMouse::~WinCvMouse()
{
  if( m_pWin != NULL )
  {
    unbind();
  }
}

void WinCvMouse::bind(Win *pWin)
{
  if( m_pWin != NULL )
  {
    unbind();
  }

  m_pWin          = pWin;
  m_nCurrentEvent = UIEventNone;
  m_ptMouse       = nopoint;
  m_bDrag         = false;
  m_bDragState    = false;

  m_pWin->registerCvImageMouseCallback(WinCvMouse::onMouse, this);
}

void WinCvMouse::unbind()
{
  if( m_pWin != NULL )
  {
    m_pWin->unregisterCvImageMouseCallback();
    m_pWin = NULL;
  }
}


void WinCvMouse::onMouse(int event, int x, int y, int flags, void* param)
{
  WinCvMouse *pMouse = (WinCvMouse *)param;

  pMouse->m_nCurrentEvent = UIEventNone;

  // drag the mouse events enabled
  switch( event )
  {
    case CV_EVENT_MOUSEMOVE:
      if( pMouse->m_bDrag && pMouse->m_bDragState )
      {
        pMouse->m_ptMouse.x = x;
        pMouse->m_ptMouse.y = y;
        pMouse->m_nCurrentEvent = UIEventDragging;
      }
      break;

    case CV_EVENT_LBUTTONUP:
      if( pMouse->m_bDrag )
      {
        pMouse->m_ptMouse.x = x;
        pMouse->m_ptMouse.y = y;
        pMouse->m_bDragState = false;
        pMouse->m_nCurrentEvent = UIEventDragEnd;
      }
      break;

    case CV_EVENT_LBUTTONDOWN:
      pMouse->m_ptMouse.x = x;
      pMouse->m_ptMouse.y = y;

      if( pMouse->m_bDrag )
      {
        pMouse->m_bDragState = true;
        pMouse->m_nCurrentEvent = UIEventDragStart;
      }
      else
      {
        pMouse->m_nCurrentEvent = UIEventClick;
      }
      break;

    default:
      break;
  }
}
