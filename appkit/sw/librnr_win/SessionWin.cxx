////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Library:   librnr_win
//
// File:      SessionWin.cxx
//
/*! \file
 *
 * $LastChangedDate: 2013-07-13 14:12:15 -0600 (Sat, 13 Jul 2013) $
 * $Rev: 3126 $
 *
 * \brief SessionWin class implementation.
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

#include <stdarg.h>
#include <string.h>
#include <errno.h>

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/appkit/Camera.h"
#include "rnr/appkit/SessionWin.h"

using namespace std;
using namespace rnr;


//------------------------------------------------------------------------------
// SessionWin Class
//------------------------------------------------------------------------------

const char * const SessionWin::VideoDevNameDft = "/dev/video0";

int SessionWin::setVideoDevice(const std::string &strVideoDevName)
{
  m_strVideoDevName = strVideoDevName;
  m_nVideoIndex     = getVideoIndex(m_strVideoDevName);

  if( m_nVideoIndex < 0 )
  {
    setError(RC_ERROR, "'%s': Failed to set video index (device minor).",
        m_strVideoDevName.c_str());
    return RC_ERROR;
  }
  else
  {
    return OK;
  }
}

int SessionWin::getVideoIndex(const string &strVideoDevName)
{
  return Camera::getVideoIndex(strVideoDevName);
}
