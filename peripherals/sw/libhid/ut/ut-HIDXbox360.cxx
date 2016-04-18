////////////////////////////////////////////////////////////////////////////////
//
//  Package:  Peripherals
//  
/*! \file
 *  
 * $LastChangedDate: 2014-03-03 09:01:51 -0700 (Mon, 03 Mar 2014) $
 * $Rev: 3586 $
 *
 *  \ingroup periphs_ut
 *  
 *  \brief Unit test for libhid xbox360 controller
 *
 *  \author Daniel Packard (daniel@roadnarrows.com)
 *  
 *  \par Copyright:
 *  (C) 2013-2014, RoadNarrows LLC
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

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/hid/HID.h"
#include "rnr/hid/HIDXbox360.h"

#include <gtest/gtest.h>

using namespace rnr;

/*!
 *  \ingroup periphs_ut
 *  \defgroup periphs_ut_libhid_xbox360 Xbox360 Unit Tests
 *  \brief Fine-grained testing of xbox360 library.
 *  \{
 */

/*!
 *  \brief Sanity test Xbox interface.
 *
 *  \return Returns 0 if test succeeds, else returns \h_lt 0.
 */
static int testXboxSanity()
{
  HIDXbox360 *pXbox;
  int         rc;

  // manually set rnr log level
  LOG_SET_THRESHOLD(LOG_LEVEL_DIAG3);

  pXbox = new HIDXbox360(XBOX_LIBUSB_DEBUG_WARN);

  if( (rc = pXbox->open()) < 0 )
  {
    LOGERROR("Failed to open Xbox360 controller.");
  }

  else if( !pXbox->ping() )
  {
    LOGERROR("Failed to ping Xbox360 controller.");
    rc = -1;
  }

  else
  {
    rc = 0;
  }

  if( pXbox->isConnected() )
  {
    pXbox->close();
  }

  delete pXbox;

  return rc;
}

/*!
 *  \brief Test Xbox input interface.
 *
 *  \return Returns 0 if test succeeds, else returns \h_lt 0.
 */
static int testXboxInput()
{
  int   bttnSuccess = HIDInput::MNEM_START;
  int   bttnFail    = bttnSuccess + 1;
  int   rc;

  // manually set rnr log level
  LOG_SET_THRESHOLD(LOG_LEVEL_DIAG3);

  HIDXbox360  xbox(XBOX_LIBUSB_DEBUG_WARN);

  if( (rc = xbox.open()) < 0 )
  {
    LOGERROR("Failed to open Xbox360 controller.");
    return -1;
  }

  xbox.assocFeature(Xbox360FeatIdXButton, bttnSuccess);
  xbox.assocFeature(Xbox360FeatIdYButton, bttnFail);

  fprintf(stderr,
      "\nTesting Xbox360 Input Interface.\n"
      "  Press the Xbox 'X' button to terminate test with success.\n"
      "  Press the Xbox 'Y' button to terminate test with failure.\n\n");

  xbox.debugPrintHdr();

  while( true )
  {
    if( (rc = xbox.update()) < 0 )
    {
      LOGERROR("Failed to update Xbox360 controller state.");
      break;
    }

    xbox.debugPrintState();
    
    if( xbox.getFeatureVal(bttnSuccess) == HID_BTTN_DOWN )
    {
      rc = 0;
      break;
    }
    else if( xbox.getFeatureVal(bttnFail) == HID_BTTN_DOWN )
    {
      rc = -1;
      break;
    }
  }

  fprintf(stderr, "\n\n");

  return rc;
}

/*!
 *  \brief Tactically test Xbox rumble interface.
 *
 *  \return Returns 0 if test succeeds, else returns \h_lt 0.
 */
static int testXboxRumble()
{
  int   bttnSuccess   = HIDInput::MNEM_START;
  int   bttnFail      = bttnSuccess + 1;
  int   nLeftTrigger  = 0;
  int   nRightTrigger = 0;
  int   nLeftRumble   = 0;
  int   nRightRumble  = 0;
  int   rc;

  // manually set rnr log level
  LOG_SET_THRESHOLD(LOG_LEVEL_DIAG3);

  HIDXbox360  xbox(XBOX_LIBUSB_DEBUG_WARN);

  if( (rc = xbox.open()) < 0 )
  {
    LOGERROR("Failed to open Xbox360 controller.");
    return -1;
  }

  xbox.assocFeature(Xbox360FeatIdXButton, bttnSuccess);
  xbox.assocFeature(Xbox360FeatIdYButton, bttnFail);

  fprintf(stderr,
      "\nTesting Xbox360 Rumble Interface.\n"
      "  Squeeze left (right) trigger for left (right) rumble motor.\n"
      "  Press the Xbox 'X' button to terminate test with success.\n"
      "  Press the Xbox 'Y' button to terminate test with failure.\n\n");

  xbox.debugPrintHdr();

  while( true )
  {
    if( (rc = xbox.update()) < 0 )
    {
      LOGERROR("Failed to update Xbox360 controller state.");
      break;
    }

    xbox.debugPrintState();
    
    if( xbox.getFeatureVal(bttnSuccess) == HID_BTTN_DOWN )
    {
      rc = 0;
      break;
    }
    else if( xbox.getFeatureVal(bttnFail) == HID_BTTN_DOWN )
    {
      rc = -1;
      break;
    }
    else
    {
      nLeftTrigger  = xbox.getFeatureVal(Xbox360FeatIdLeftTrigger);
      nRightTrigger = xbox.getFeatureVal(Xbox360FeatIdRightTrigger);
      nLeftRumble   = xbox.getFeatureVal(Xbox360FeatIdLeftRumble);
      nRightRumble  = xbox.getFeatureVal(Xbox360FeatIdRightRumble);

      if( (nLeftTrigger != nLeftRumble) || (nRightTrigger != nRightRumble) )
      {
        xbox.setRumble(nLeftTrigger, nRightTrigger);
      }
    }
  }

  fprintf(stderr, "\n\n");

  return rc;
}

/*!
 *  \brief Visually test Xbox LED interface.
 *
 *  \return Returns 0 if test succeeds, else returns \h_lt 0.
 */
static int testXboxLED()
{
  int   bttnSuccess   = HIDInput::MNEM_START;
  int   bttnFail      = bttnSuccess + 1;
  int   nLeftBump  = 0;
  int   nRightBump = 0;
  int   nPattern   = 0;
  int   rc;

  // manually set rnr log level
  LOG_SET_THRESHOLD(LOG_LEVEL_DIAG3);

  HIDXbox360  xbox(XBOX_LIBUSB_DEBUG_WARN);

  if( (rc = xbox.open()) < 0 )
  {
    LOGERROR("Failed to open Xbox360 controller.");
    return -1;
  }

  xbox.assocFeature(Xbox360FeatIdXButton, bttnSuccess);
  xbox.assocFeature(Xbox360FeatIdYButton, bttnFail);

  fprintf(stderr,
      "\nTesting Xbox360 LED Interface.\n"
      "  Press left (right) bumper to increment (decrement) the LED pattern.\n"
      "  Press the Xbox 'X' button to terminate test with success.\n"
      "  Press the Xbox 'Y' button to terminate test with failure.\n\n");

  xbox.debugPrintHdr();

  while( true )
  {
    if( (rc = xbox.update()) < 0 )
    {
      LOGERROR("Failed to update Xbox360 controller state.");
      break;
    }

    xbox.debugPrintState();
    
    if( xbox.getFeatureVal(bttnSuccess) == HID_BTTN_DOWN )
    {
      rc = 0;
      break;
    }
    else if( xbox.getFeatureVal(bttnFail) == HID_BTTN_DOWN )
    {
      rc = -1;
      break;
    }
    else if( nLeftBump != xbox.getFeatureVal(Xbox360FeatIdLeftBump) )
    {
      nLeftBump = xbox.getFeatureVal(Xbox360FeatIdLeftBump);

      if( nLeftBump )
      {
        nPattern = (nPattern + 1) % XBOX360_LED_PAT_NUMOF;
        xbox.setLED(nPattern);
      }
    }
    else if( nRightBump != xbox.getFeatureVal(Xbox360FeatIdRightBump) )
    {
      nRightBump = xbox.getFeatureVal(Xbox360FeatIdRightBump);

      if( nRightBump )
      {
        nPattern = (nPattern-1 + XBOX360_LED_PAT_NUMOF)%XBOX360_LED_PAT_NUMOF;
        xbox.setLED(nPattern);
      }
    }
  }

  fprintf(stderr, "\n\n");

  return rc;
}

#ifndef JENKINS

/*!
 *  \brief Sanity test Xbox360 controller interface.
 *
 * \par The Test:
 * Construct HIDXbox360 object.\n
 * Open.\n
 * Ping.\n
 * Close.\n
 * Destroy HIDXbox360 object.
 */
TEST(Xbox360, XboxSanity)
{
  EXPECT_TRUE( testXboxSanity() == 0 );
}


/*!
 *  \brief Visually test Xbox360 controller input interface.
 *
 * \par The Test:
 * Construct HIDXbox360 object.\n
 * Open.\n
 * User interaction.\n
 * Close.\n
 * Destroy HIDXbox360 object.
 */
TEST(Xbox360, XboxInput)
{
  EXPECT_TRUE( testXboxInput() == 0 );
}

/*!
 *  \brief Tactually test Xbox360 controller rumble interface.
 *
 * \par The Test:
 * Construct HIDXbox360 object.\n
 * Open.\n
 * User interaction.\n
 * Close.\n
 * Destroy HIDXbox360 object.
 */
TEST(Xbox360, XboxRumble)
{
  EXPECT_TRUE( testXboxRumble() == 0 );
}

/*!
 *  \brief Visually test Xbox360 controller LED interface.
 *
 * \par The Test:
 * Construct HIDXbox360 object.\n
 * Open.\n
 * User interaction.\n
 * Close.\n
 * Destroy HIDXbox360 object.
 */
TEST(Xbox360, XboxLED)
{
  EXPECT_TRUE( testXboxLED() == 0 );
}

#endif // JENKINS


/*!
 *  \}
 */
