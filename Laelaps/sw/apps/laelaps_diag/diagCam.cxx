////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Program:   laelaps_diag   
//
// File:      diagCam.cxx
//
/*! \file
 *
 * $LastChangedDate: 2016-02-01 15:14:45 -0700 (Mon, 01 Feb 2016) $
 * $Rev: 4289 $
 *
 * \brief Perform Laelaps cameras diagnostics.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2015-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
/*
 * @EulaBegin@
 * 
 * Unless otherwise stated explicitly, all materials contained are copyrighted
 * and may not be used without RoadNarrows LLC's written consent,
 * except as provided in these terms and conditions or in the copyright
 * notice (documents and software) or other proprietary notice provided with
 * the relevant materials.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY 
 * MEMBERS/EMPLOYEES/CONTRACTORS OF ROADNARROWS OR DISTRIBUTORS OF THIS SOFTWARE
 * BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
 * DOCUMENTATION, EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHORS AND  ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/opts.h"
#include "rnr/pkg.h"

// common
#include "Laelaps/laelaps.h"
#include "Laelaps/laeUtils.h"

// hardware
#include "Laelaps/laeCams.h"

#include "laelaps_diag.h"

using namespace std;
using namespace laelaps;

static const char *SubSysName = "Camera";

static DiagStats initCams()
{
  DiagStats   stats;

  printSubHdr("Initialize Cameras");

  printTestResult(FatalTag, "%s diagnostics not supported yet.", SubSysName);

  ++stats.testCnt;
  stats.fatal = true;

  return stats;
}

DiagStats runCamDiagnostics()
{
  DiagStats   statsTest;
  DiagStats   statsTotal;

  printHdr("Camera Diagnostics");

  //
  // Init Tests
  //
  statsTest = initCams();

  printSubTotals(statsTest);

  statsTotal += statsTest;

#if 0
  //
  // X Tests
  //
  if( !statsTotal.fatal )
  {
    statsTest = readInfo();

    printSubTotals(statsTest);

    statsTotal += statsTest;
  }
#endif
 
  //
  // Summary
  //
  printTotals(statsTotal);

  return statsTotal;
}
