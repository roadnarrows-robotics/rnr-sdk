////////////////////////////////////////////////////////////////////////////////
//
//  Package:  Peripherals
//  
/*! \file
 *  
 * $LastChangedDate: 2013-02-13 16:41:34 -0700 (Wed, 13 Feb 2013) $
 * $Rev: 2683 $
 *
 *  \ingroup periphs_ut
 *  
 *  \brief Unit test for libmot roboteq-small motor controller
 *
 *  \author Daniel Packard (daniel@roadnarrows.com)
 *  
 *  \par Copyright:
 *  (C) 2013, RoadNarrows LLC
 *  (http://roadnarrows.com)
 *  All rights reserved.
 */
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met: 
//
// 1. Redistributions of source code must retain the above copyright notice, 
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution. 
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE.
//
// The views and conclusions contained in the software and documentation are 
// those of the authors and should not be interpreted as representing official 
// policies, either expressed or implied, of the FreeBSD Project.

#include <iostream>
#include <string>
#include <stdio.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/mot/MotDummy.h"

#include <gtest/gtest.h>

using namespace rnr;
/*!
 *  \ingroup periphs_ut
 *  \defgroup periphs_ut_libmot_roboteq_s Roboteq Small Unit Tests
 *  \brief Fine-grained testing of Roboteq Small motor controller
 *  \{
 */

/*!
 *  \brief Sanity test Roboteq-small interface.
 *
 *  \return Returns 0 if test succeeds, else returns \h_lt 0.
 */
static int testMotDummySetSpeed()
{
  MotDummy motor;
  return motor.setSpeed(0, 0);
}

TEST(MotDummy, setSpeed)
{
  EXPECT_TRUE( testMotDummySetSpeed() == 0 );
}

/*!
 *  \brief Sanity test Roboteq-small interface.
 *
 *  \return Returns 0 if test succeeds, else returns \h_lt 0.
 */
static int testMotDummyGetAttrSpeed()
{
  MotDummy motor;
  int min,max,step;
  int rc = motor.getAttrSpeed(&min, &max, &step);
  printf("Speed Attributes = min:%d max:%d step:%d \n",min, max, step);
}

TEST(MotDummy, getAttrSpeed)
{
  EXPECT_TRUE( testMotDummyGetAttrSpeed() == 0 );
}

#ifndef JENKINS

// put tests that require human input here.

#endif // JENKINS


/*!
 *  \}
 */
