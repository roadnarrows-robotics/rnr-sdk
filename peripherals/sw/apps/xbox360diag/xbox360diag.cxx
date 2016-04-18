////////////////////////////////////////////////////////////////////////////////
//
// Package:   peripherals
//
// Program:   xbox360diag
//
// File:      xbox360diag.cxx
//
/*! \file
 *
 * $LastChangedDate: 2014-03-07 11:57:26 -0700 (Fri, 07 Mar 2014) $
 * $Rev: 3594 $
 *
 * \brief Xbox360 diagnostic tool.
 *
 * \author Robin Knight     (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 * (C) 2014.  RoadNarrows LLC.
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
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
 * @EulaEnd@
 */
//
///////////////////////////////////////////////////////////////////////////////

#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <errno.h>

#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/opts.h"

#include "rnr/hid/HID.h"
#include "rnr/hid/HIDXbox360.h"

#include "version.h"

using namespace std;
using namespace rnr;


//------------------------------------------------------------------------------
// Constants and types 
//------------------------------------------------------------------------------
#define APP_EC_OK     0   ///< success exit code
#define APP_EC_GEN    1   ///< command-line options/arguments error exit code
#define APP_EC_ARGS   2   ///< command-line options/arguments error exit code
#define APP_EC_EXEC   4   ///< execution exit code


//------------------------------------------------------------------------------
// Command-Line Options
//------------------------------------------------------------------------------
static char  *Argv0;                      ///< the command
static bool   OptsDftThresholds = false;  ///< do [not] use error thresholding

//------------------------------------------------------------------------------
// Command-Line Arguments
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// Global State Data
//------------------------------------------------------------------------------

//
// Forward Declarations
//

/*!
 * \brief Program information.
 */
static OptsPgmInfo_T AppPgmInfo =
{
  // usage_args
  NULL,

  // synopsis
  "Xbox360 diagnostic tool.",
  
  // long_desc 
  "The %P tool debugs the libhid interface to Xbox360 controllers.\n"
  "  Button states are 0 or 1.\n"
  "  Triggers are 0 - 255.\n"
  "  Joysticks are -32767 - 32767.\n"
  "  LEDs and rumble motors are outputs to the Xbox360.\n"
  "  Link is determined by the libhid and is true or false.\n"
  "  Status is sent by the dongle when wireless link wth Xbox360 has changed.\n"
  "  Battery status is sent by Xbox360 occasionally.\n"
  "  Error counts:\n"
  "    ERTO   consecutive Receive Error TimeOut count.\n" 
  "    ER     consecutive Receive Error count.\n" 
  "    ES     consecutive Send Error count.\n" 
  "    ETOT   Total Error count.", 
 
  // diagnostics
  NULL
};

/*!
 * \brief Command line options information.
 */
static OptsInfo_T AppOptsInfo[] =
{
  // --thresholds, -t
  {
    "thresholds",         // long_opt
    't',                  // short_opt
    no_argument,          // has_arg
    true,                 // has_default
    &OptsDftThresholds,   // opt_addr
    OptsCvtArgBool,       // fn_cvt
    OptsFmtBool,          // fn_fmt
    NULL,                 // arg_name
    "Use error threshold defaults."
                          // opt desc
  },
  

  {NULL, }
};

/*!
 * \brief Main.
 *
 * \param argc    Command-line argument count.
 * \param argv    Command-line argument list.
 *
 * \return Returns exit code.
 */
int main(int argc, char* argv[])
{
  int   leftTrigger;    // left trigger old value
  int   rightTrigger;   // right trigger old value
  int   BButton;        // B button value
  int   state;          // current state of one button
  int   led;            // led state
  int   rc;             // return code

  // name of this process
  Argv0 = basename(argv[0]);

  // parse input options
  argv = OptsGet(Argv0, &PkgInfo, &AppPgmInfo, AppOptsInfo, true, &argc, argv);

  HIDXbox360 xbox;

  // turn off receive error thresholding for better diagnostics
  if( !OptsDftThresholds  )
  {
    xbox.setErrorThresholds(0, 0, 0);
  }

  // connect to controller
  if( (rc = xbox.open()) < 0 )
  {
    LOGERROR("Failed to open Xbox360 controller. Is it connected?");
    return APP_EC_EXEC;
  }

  leftTrigger   = 0;
  rightTrigger  = 0;
  BButton       = 0;

  // start updated in thread
  xbox.run();

  fprintf(stderr, "  left trigger -> left rumble;"
                  "  right trigger -> right rumble;"
                  "  B button -> next LED pattern;"
                  "  right bumper -> refresh;"
                  "  X button -> exit\n");

  xbox.debugPrintHdr();

  //
  // 
  while( xbox.isConnected() )
  {
    state = xbox.getFeatureVal(Xbox360FeatIdLeftTrigger);

    if( state != leftTrigger )
    {
      leftTrigger = state;
      xbox.setRumble(leftTrigger, rightTrigger);
    }

    state = xbox.getFeatureVal(Xbox360FeatIdRightTrigger);

    if( state != rightTrigger )
    {
      rightTrigger = state;
      xbox.setRumble(leftTrigger, rightTrigger);
    }

    state = xbox.getFeatureVal(Xbox360FeatIdBButton);

    if( state && !BButton )
    {
      led = xbox.getFeatureVal(Xbox360FeatIdLEDPat);
      led = (led +1) % XBOX360_LED_PAT_NUMOF;
      xbox.setLED(led);
    }

    BButton = state;

    state = xbox.getFeatureVal(Xbox360FeatIdXButton);

    if( state )
    {
      break;
    }

    state = xbox.getFeatureVal(Xbox360FeatIdRightBump);

    if( state )
    {
      fprintf(stderr, "\n\n");
      xbox.debugPrintHdr();
    }

    xbox.debugPrintState();

    usleep(20000);
  }

  xbox.close();

  fprintf(stderr, "\n\nEnd Diagnostics\n");

  // exit
  return APP_EC_OK;
}
