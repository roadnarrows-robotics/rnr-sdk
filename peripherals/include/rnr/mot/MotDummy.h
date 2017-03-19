////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Peripherals
//
// Library:   libmot
//
// File:      MotDummy.h
//
/*! \file
 *
 * $LastChangedDate: 2013-02-13 16:41:34 -0700 (Wed, 13 Feb 2013) $
 * $Rev: 2683 $
 *
 * \brief Dummy Motor Controller Interface.
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



#ifndef _MOTDUMMY_H
#define _MOTDUMMY_H
#include "rnr/mot/Mot.h"
#define MOTDUMMY_NUM_OUTPUTS 2
namespace rnr
{
  class MotDummy : Mot
  {
    public:
    MotDummy()
    {
       m_nMinSpeed = -255;
       m_nMaxSpeed = 255;
       m_nSpeedStepSize = 1;
       m_nMinBrake = 0;
       m_nMaxBrake = 31;
       m_nBrakeStepSize = 1;
    }

    int setSpeed( int id, int speed);
    int getAttrSpeed(int *minSpeed , int *maxSpeed,int *stepSize);
    int setBrake(int id, int brake);
    int getAttrBrake(int *minBrake, int *maxBrake, int *stepSizeBrake);
    private:
    int m_speed[MOTDUMMY_NUM_OUTPUTS];
    int m_brake[MOTDUMMY_NUM_OUTPUTS];

  };
}

#endif

