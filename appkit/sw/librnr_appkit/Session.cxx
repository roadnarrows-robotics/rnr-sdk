////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Library:   librnr_appkit
//
// File:      Session.cxx
//
/*! \file
 *
 * $LastChangedDate: 2013-05-06 10:03:14 -0600 (Mon, 06 May 2013) $
 * $Rev: 2907 $
 *
 * \brief Session base class implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2012-2013.  RoadNarrows LLC
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

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/Session.h"

using namespace std;
using namespace rnr;


//------------------------------------------------------------------------------
// Session Class
//------------------------------------------------------------------------------

void Session::setError(int ecode, const char *sFmt, ...)
{
  va_list         ap;

  m_ecode = ecode;

  // format error message
  va_start(ap, sFmt);
  vsnprintf(m_bufErrorMsg, sizeof(m_bufErrorMsg), sFmt, ap);
  m_bufErrorMsg[sizeof(m_bufErrorMsg)-1] = 0;
  va_end(ap);
}

void Session::setFatal(int ecode, const char *sFmt, ...)
{
  va_list         ap;

  m_ecode     = ecode;
  m_bHasFatal = true;

  // format error message
  va_start(ap, sFmt);
  vsnprintf(m_bufErrorMsg, sizeof(m_bufErrorMsg), sFmt, ap);
  m_bufErrorMsg[sizeof(m_bufErrorMsg)-1] = 0;
  va_end(ap);
}
