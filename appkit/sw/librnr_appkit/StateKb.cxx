////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Library:   librnr_appkit
//
// File:      StateKb.cxx
//
/*! \file
 *
 * $LastChangedDate: 2013-05-06 10:03:14 -0600 (Mon, 06 May 2013) $
 * $Rev: 2907 $
 *
 * \brief Keyboard StateKb derived state class implementation.
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

#include <sys/select.h>
#include <stdio.h>
#include <unistd.h>
#include <stdarg.h>
#include <libgen.h>
#include <fcntl.h>
#include <termios.h>

#include <sstream>
#include <string>
#include <map>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/Random.h"
#include "rnr/State.h"
#include "rnr/StateKb.h"

using namespace std;
using namespace rnr;


//------------------------------------------------------------------------------
// Private
//------------------------------------------------------------------------------


PRAGMA_IGNORED(sign-conversion)
/*!
 * \brief FD_SET() wrapper with no annoying warnings.
 * \param fd    File descriptor to add to set.
 * \param pset  Pointer to fd set.
 */
static inline void fdset_nowarn(int fd, fd_set *pset)
{
  FD_SET(fd, pset);
}
PRAGMA_WARNING(sign-conversion)


//------------------------------------------------------------------------------
// StateKb Class
//------------------------------------------------------------------------------

int             StateKb::ClassObjRefCnt       = 0;
int             StateKb::OrigInputStatusFlags = 0;
struct termios  StateKb::OrigInputTio         = {0, };

int StateKb::receiveEvent()
{
  byte_t          byte;
  fd_set          rset;
  struct timeval  timeout;
  int             fd = fileno(stdin);
  int             nFd;
  ssize_t         n;

  FD_ZERO(&rset);
  fdset_nowarn(fd, &rset);

  // wait for character with timeout
  if( m_usecTimeOut > 0 )
  {
    // load timeout
    timeout.tv_sec  = (time_t)(m_usecTimeOut / 1000000);
    timeout.tv_usec = (time_t)(m_usecTimeOut % 1000000);

    nFd = select(fd+1, &rset, NULL, NULL, &timeout);
  }

  // block indefinitely for character
  else 
  {
    nFd = select(fd+1, &rset, NULL, NULL, NULL);
  }

  // system error occurred
  if( nFd < 0 )
  {
    LOGSYSERROR("select(%d,...)", fd);
    return KbEventError;
  }

  // select() timeout occurred
  else if( nFd == 0 )
  {
    LOGDIAG4("select() on read timed out");
    errno = ETIMEDOUT;
    return KbEventTimeOut;
  }

  // data available from serial device, but error on read
  else if( (n = read(fd, &byte, (size_t)1)) < 0 )
  {
    LOGSYSERROR("read(%d,...)", fd);
    return KbEventError;
  }

  // nothing read
  else if( n == 0 )
  {
    LOGERROR("0=read(%d,...)", fd);
    errno = EIO;
    return KbEventError;
  }

  // read character
  else
  {
    LOGDIAG4("%s() byte=0x%02x read", LOGFUNCNAME, byte);
    return (int)(((uint_t)(byte)) & 0x00ff);
  }
}

void StateKb::configInput()
{
	struct termios  tio;
  int             fd = fileno(stdin);

	// get the terminal settings for stdin
	tcgetattr(fd, &OrigInputTio);

	// copy settings
	tio = OrigInputTio;

	// disable canonical mode (buffered i/o) and local echo 
	tio.c_lflag &= (uint_t)(~ICANON & ~ECHO);

	// set the new settings immediately
	tcsetattr(fd, TCSANOW, &tio);

  // get file status flags
  OrigInputStatusFlags = fcntl(fd, F_GETFL, 0);
    
  // set non-blocking mode
  fcntl(fd, F_SETFL, OrigInputStatusFlags | O_NONBLOCK);
}

void StateKb::restoreInput()
{
  int fd = fileno(stdin);

	tcsetattr(fd, TCSANOW, &OrigInputTio);

  fcntl(fd, F_SETFL, OrigInputStatusFlags);
}
