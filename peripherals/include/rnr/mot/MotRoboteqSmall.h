////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Peripherals
//
// Library:   libhid
//
// File:      hid.h
//
/*! \file
 *
 * $LastChangedDate: 2013-04-02 15:23:46 -0600 (Tue, 02 Apr 2013) $
 * $Rev: 2806 $
 *
 * \brief Common Human Interface Device Interface.
 *
 * \author: Robin Knight      (robin.knight@roadnarrows.com)
 * \author: Daniel Packard    (daniel@roadnarrows.com)
 * \author: Jessica Trujillo  (jessica@roadnarrows.com)
 * \author: Maurice Woods III (maurice@roadnarrows.com)
 *
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


#ifndef _MOT_ROBOTEQ_SMALL_H
#define _MOT_ROBOTEQ_SMALL_H

#include <vector>
#include "rnr/rnrconfig.h"
#include "rnr/units.h"
#include "rnr/mot/Mot.h"

namespace rnr
{
  class MotRoboteqSmall : public Mot
  {
    public:
      MotRoboteqSmall() : Mot()
      {
        m_fd = 0;
        m_ntimeout = 1000000;
        m_nbaudRate = 115200;
      }

      MotRoboteqSmall(float maxSpeed, float minSpeed, float speedStepSize, float maxBrake, float minBrake, float brakeStepSize) : Mot(maxSpeed, minSpeed, speedStepSize, maxBrake, minBrake, brakeStepSize)
      {
      }

      virtual ~MotRoboteqSmall()
      {
        close();
      }

      int open(const std::string &devName, int baudRate);
      int close();
      int sendCommand(int fd, byte_t *buf, int nBytes, int timeout);
      int recvResponse(int fd, byte_t *buf,int timeout);
      bool motIDIsValid(int motID);
      int setSpeed(int motID, float speed, units_t units=units_norm);
      int setSpeed(VecSpeedTupples vecSpeedTupple, units_t units=units_norm);
      int setSpeedRaw(int motID, int speed);
      int setSpeedRaw(VecSpeedRawTupples vecSpeedRawTupple);
      int stop(int motID);
      int stop();
      int eStop();
      int eStopRelease();
      int getCurrent(int motID, units_t units=units_amp);
      int getCurrentLimits();
      int setCurrentLimits(int motID, int current, units_t units=units_amp);
      int getVoltageLimits();
      int setVoltageLimits(int lowVoltage, int overVoltage);
      int getVoltage();
      int setSpeedProfile(int motID, int accel, int decel=DEF_DEC);

    protected:
      int m_fd;
      int m_ntimeout;
      int m_nbaudRate;
  };
}




#endif // _MOT_ROBOTEQ_SMALL_H
