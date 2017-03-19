////////////////////////////////////////////////////////////////////////////////
//
//  Package:  Peripherals
//  
/*! \file
 *  
 * $LastChangedDate: 2013-09-24 09:34:42 -0600 (Tue, 24 Sep 2013) $
 * $Rev: 3323 $
 *
 *  \ingroup periphs_ut
 *  
 *  \brief Unit test for libmot roboteq-small motor controller
 *
 *  \author Daniel Packard (daniel@roadnarrows.com)
 *  
 * \copyright
 *   \h_copy 2013-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
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
#include <vector>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/mot/MotRoboteqSmall.h"

#include <gtest/gtest.h>

using namespace rnr;
using namespace std;

/*!
 *  \ingroup periphs_ut
 *  \defgroup periphs_ut_libmot_roboteq_s Roboteq Small Unit Tests
 *  \brief Fine-grained testing of Roboteq Small motor controller
 *  \{ST
 */

/*!
 *  \brief Sanity test Roboteq-small interface.
 *
 *  \return Returns 0 if test succeeds, else returns  0.
 */
static int testRoboteqSmallSanity()
{
  return 0;
}



TEST(RoboteqSmall, sanity)
{
  EXPECT_TRUE( testRoboteqSmallSanity() == 0 );
}

#ifndef JENKINS

static int testRoboteqSmallOpenClose()
{
  MotRoboteqSmall mot;
  string devName="/dev/ttyACM0";
  int baudRate=115200;
  if(mot.open(devName, baudRate)!=0)
  {
    fprintf(stderr, "jnt failed to open\n");
    return -1;
  }

  if(mot.close()!=0)
  {
    fprintf(stderr, "jnt failed to close\n");
    return -1;
  }
  return 0;
}

TEST(RoboteqSmall, openClose)
{
  EXPECT_TRUE(testRoboteqSmallOpenClose() == 0);
}

static int testRoboteqSmallSetSpeed()
{
  MotRoboteqSmall mot;
  string devName="/dev/ttyACM0";
  int baudRate=115200;
  if(mot.open(devName, baudRate)!=0)
  {
    fprintf(stderr, "jnt failed to open\n");
    return -1;
  }
int i=0;
int j=100;
int h=j*2;
/*  while (i <j)
  {
    mot.setSpeed(2,1);
    mot.getCurrent(2);
    usleep(10000);
    i++;
  }
  mot.setSpeed(2,0);
  while (i <h)
  {
    mot.setSpeed(1,1);
    mot.getCurrent(1);
    usleep(10000);
    i++;
  }
  mot.setSpeed(1,0);

  if(mot.close()!=0)
  {
    fprintf(stderr, "jnt failed to close\n");
    return -1;
  }
*/  return 0;
}

TEST(RoboteqSmall, setSpeed)
{
  EXPECT_TRUE(testRoboteqSmallSetSpeed() == 0);
}

static int testRoboteqSmallSetSpeedVec()
{
  MotRoboteqSmall mot;
  string devName="/dev/ttyACM0";
  int baudRate=115200;
  if(mot.open(devName, baudRate)!=0)
  {
    fprintf(stderr, "jnt failed to open\n");
    return -1;
  }

  Mot::VecSpeedTupples motSpeeds;
  Mot::IDFloatTupple tmp;

  mot.setVoltageLimits(6, 29);
  mot.getVoltageLimits();
  mot.getVoltage();

  mot.setCurrentLimits(1, 19);
  mot.setCurrentLimits(2, 20);
  mot.getCurrentLimits();

//
  tmp.m_nMotID=1;
  tmp.m_fVal=0.5;
  motSpeeds.push_back(tmp);

  tmp.m_nMotID=2;
  tmp.m_fVal=0.2;
  motSpeeds.push_back(tmp);

  mot.setSpeed(motSpeeds);
  usleep(500000);

//
  mot.setSpeedProfile(1,2000);
  mot.setSpeedProfile(2,2000);

  tmp.m_nMotID=1;
  tmp.m_fVal=1;
  motSpeeds.push_back(tmp);

  tmp.m_nMotID=2;
  tmp.m_fVal=1;
  motSpeeds.push_back(tmp);

  mot.setSpeed(motSpeeds);brakeStepSize
  usleep(500000);

//
  tmp.m_nMotID=1;
  tmp.m_fVal=1;
  motSpeeds.push_back(tmp);

  tmp.m_nMotID=2;
  tmp.m_fVal=1;
  motSpeeds.push_back(tmp);

  mot.setSpeed(motSpeeds);
  usleep(500000);

//
  tmp.m_nMotID=1;
  tmp.m_fVal=1;
  motSpeeds.push_back(tmp);

  tmp.m_nMotID=2;
  tmp.m_fVal=1;
  motSpeeds.push_back(tmp);

  mot.setSpeed(motSpeeds);
  usleep(500000);

//
  tmp.m_nMotID=1;
  tmp.m_fVal=1;
  motSpeeds.push_back(tmp);

  tmp.m_nMotID=2;
  tmp.m_fVal=1;
  motSpeeds.push_back(tmp);

  mot.setSpeed(motSpeeds);
  usleep(500000);

//
  tmp.m_nMotID=1;
  tmp.m_fVal=1;
  motSpeeds.push_back(tmp);

  tmp.m_nMotID=2;
  tmp.m_fVal=1;
  motSpeeds.push_back(tmp);

  mot.setSpeed(motSpeeds);
  usleep(500000);

//
  mot.setSpeedProfile(1,20000);
  mot.setSpeedProfile(2,20000);

  mot.stop(2);
  mot.stop(1); 	

  if(mot.close()!=0)
  {
    //fprintf(stderr, "jnt failed to close\n");
    return -1;
  }

  return 0;
}


TEST(RoboteqSmall, setSpeedVec)
{
  EXPECT_TRUE(testRoboteqSmallSetSpeedVec() == 0);
}



// put tests that require human input here.

#endif // JENKINS


/*!
 *  \}
 */
