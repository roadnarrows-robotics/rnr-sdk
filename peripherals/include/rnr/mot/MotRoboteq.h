////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Peripherals
//
// Library:   libmot
//
// File:      roboteq.h
//
/*! \file
 *
 * $LastChangedDate: 2013-09-24 09:34:42 -0600 (Tue, 24 Sep 2013) $
 * $Rev: 3323 $
 *
 * \brief Common Roboteq Motor Controller Interface.
 *
 * \author: Robin Knight      (robin.knight@roadnarrows.com)
 * \author: Daniel Packard    (daniel@roadnarrows.com)
 * \author: Jessica Trujillo  (jessica@roadnarrows.com)
 * \author: Maurice Woods III (maurice@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2012-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
// Permission is hereby granted, without written agreement and without
// license or royalty fees, to use, copy, modify, and distribute this
// software and its documentation for any purpose, provided that
// (1) The above copyright notice and the following two paragraphs
// appear in all copies of the source code and (2) redistributions
// including binaries reproduces these notices in the supporting
// documentation.   Substantial modifications to this software may be
// copyrighted by their authors and need not follow the licensing terms
// described here, provided that the new terms are clearly indicated in
// all files where they apply.
//
// IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
// OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
// PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
// EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.
//
// THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
// "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
// PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
//
////////////////////////////////////////////////////////////////////////////////


#ifndef _MOT_ROBOTEQ_H
#define _MOT_ROBOTEQ_H

#include <vector>
#include "rnr/rnrconfig.h"
#include "rnr/units.h"
#include "rnr/mot/Mot.h"

namespace rnr
{
  class MotRoboteq : public Mot
  {
    public:
      MotRoboteq() : Mot()
      {
        m_fd=0;
        m_ntimeout = 100000;
        m_baudRate = 115200;
      }
    
      MotRoboteq() : Mot()
      {
      }


      virtual ~MotRoboteq()
      {
        close();
      }

      int open(const std::string &devName, int baudRate);
      int close();
      int sendCommand(int fd, byte_t *buf, int nBytes, int timeout);
      int recvResponse(int fd, byte_t *buf, int timeout);

    protected:
      int m_fd;
      int m_ntimeout;
      int m_nbaudRate;
  }
}



#endif // _MOT_ROBOTEQ_H
