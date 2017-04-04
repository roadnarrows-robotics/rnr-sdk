////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Program:   laelaps_diag   
//
// File:      diagToF.cxx
//
/*! \file
 *
 * \brief Perform Laelaps Time-of-Flight sensors diagnostics.
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
#include "Laelaps/laeDesc.h"

// hardware
#include "Laelaps/laeSysDev.h"
#include "Laelaps/laeI2C.h"
#include "Laelaps/laeVL6180.h"

#include "laelaps_diag.h"

using namespace std;
using namespace laelaps;
using namespace sensor::vl6180;

static const char *SubSysName = "ToF";
static const char *ProdName   = "VL6180";

static const char *SensorKeys[ToFSensorMaxNumOf] = 
{
  "front", "left_front", "left", "left_rear",
  "rear", "right_rear", "right", "right_front"
};

static DiagStats initSensors(LaeRangeSensorGroup &rnggrp)
{
  uint_t    uVerMajor, uVerMinor, uFwVer;

  DiagStats   stats;

  printSubHdr("Initialize Sensors");

  stats.testCnt += 1;

  if( rnggrp.setInterface(RobotDesc.getProdHwVer()) == LAE_OK )
  {
    printTestResult(PassTag, "Range Sensor Group interface set.");
    stats.passCnt += 1;
  }
  else
  {
    printTestResult(FatalTag,
        "Failed to set interface with product hw version 0x%x.", 
        RobotDesc.getProdHwVer());
    stats.fatal = true;
  }

  if( !stats.fatal )
  {
    stats.testCnt += 1;

    if( rnggrp.getInterfaceVersion(uVerMajor, uVerMinor, uFwVer) == LAE_OK )
    {
      printTestResult(PassTag, "Range Sensor Group interface %u.%u, fwver=%u.",
        uVerMajor, uVerMinor, uFwVer);
      stats.passCnt += 1;

      rnggrp.clearSensedData();
    }
    else
    {
      printTestResult(FailTag,
        "Failed to read range sensor group interface version.");
    }
  }

  if( !stats.fatal )
  {
    stats.testCnt += 1;

    if( rnggrp.configure(RobotDesc) == LAE_OK )
    {
      printTestResult(PassTag,
          "Configured range sensors from robot description.");
      stats.passCnt += 1;
    }
    else
    {
      printTestResult(FailTag, "Failed to configure range sensors.");
    }
  }

  return stats;
}

static DiagStats getSensorInfo(LaeRangeSensorGroup &rnggrp)
{
  const char *sKey;
  string      strRadiationType;
  double      fFoV;
  double      fBeamDir;
  double      fMin;
  double      fMax;
  bool        bInstalled;
  int         rc;

  DiagStats     stats;

  printSubHdr("Read Sensors Properties and Tuning");

  //
  // Read Info and Config
  //
  for(size_t i = 0; i < ToFSensorMaxNumOf; ++i)
  {
    sKey = SensorKeys[i];

    stats.testCnt += 1;

    rc = rnggrp.getSensorProps(sKey, strRadiationType,
                               fFoV, fBeamDir, fMin, fMax);

    if( rc == LAE_OK )
    {
      printTestResult(PassTag, "%s %s sensor: Got sensor properties.",
          ProdName, sKey);
      stats.passCnt += 1;
      bInstalled = true;
    }
    else if( rc == -LAE_ECODE_BAD_VAL )
    {
      printTestResult(PassTag, "%s %s sensor: NOT INSTALLED.", ProdName, sKey);
      stats.passCnt += 1;
      bInstalled = false;
    }
    else 
    {
      printTestResult(FailTag, "%s %s sensor: Failed to get sensor properties.",
          ProdName, sKey);
      bInstalled = false;
    }

    // GET IDENT
    // GET TUNING
 
    printf("\n");
    printf("%s %s Sensor Info:\n", SubSysName, ProdName);
    printf("  Location:       %s\n", sKey);
    printf("  Installed:      %s\n", (bInstalled? "yes": "no"));
    printf("  Beam Direction: %.1lf degrees\n", radToDeg(fBeamDir));
    printf("  Radiation:      %s\n", strRadiationType.c_str());
    printf("  FoV:            %.1lf degrees\n", radToDeg(fFoV));
    printf("  Min Distance:   %.3lf meters\n", fMin);
    printf("  MAx Distance:   %.3lf meters", fMax);
#if 0
    printf("  Name:             %s\n", ToFSensors[i]->getNameId().c_str());
    printf("  Desc:             %s\n", ToFSensors[i]->getDesc().c_str());
    printf("  Sensor:           %s\n", ProdName);
    printf("  Model:            0x%02x v%u.%u\n",
          id.idModel, id.idModelRevMajor, id.idModelRevMinor);
    printf("  Module:           v%u.%u\n",
        id.idModuleRevMajor, id.idModuleRevMinor);
    printf("  Date/Time:        %u/%u\n", id.idDate, id.idTime);
    printf("  Range Offset:     %u\n", valRangeOffset);
    printf("  Range Cross-Talk: %u\n", valRangeCrossTalk);
    printf("  ALS Gain:         %u\n", valAlsGain & 0x07);
    printf("  ALS Int. Period:  %u\n", valAlsIntPeriod);
#endif // 0
    printf("\n\n");
  }

  return stats;
}

static DiagStats measureDistance(LaeRangeSensorGroup &rnggrp, int cnt)
{
  vector<string>  keys;
  vector<double>  distance;
  bool            showLabel;
  int             rc;

  DiagStats stats;

  showLabel = cnt == 0;

  ++stats.testCnt;

  rc = rnggrp.getRange(keys, distance);

  if( rc == OK )
  {
    ++stats.passCnt;
  }
  else
  {
    printTestResult(FailTag, "%s: Failed to get distance measurements.",
      SubSysName);
    showLabel = true;
  }

  if( showLabel )
  {
    printf("%8s", "");
    for(size_t i = 0; i < keys.size(); ++i)
    {
      printf("%10s ", keys[i].c_str());
    }
    printf("\n");
  }

  printf("%6d. ", cnt);
  for(size_t i = 0; i < distance.size(); ++i)
  {
    if( distance[i] > VL6180X_RANGE_MAX )
    {
      printf("%10s ", "noobj");
    }
    else if( distance[i] < 0 )
    {
      printf("%10s ", "error");
    }
    else
    {
      printf("%10.3lf ", distance[i]);
    }
  }
  printf("\r");
  fflush(stdout);

  return stats;
}

static DiagStats measureAmbient(LaeRangeSensorGroup &rnggrp, int cnt)
{
  vector<string>  keys;
  vector<double>  ambient;
  bool            showLabel;
  int             rc;

  DiagStats stats;

  showLabel = cnt == 0;

  ++stats.testCnt;

  rc = rnggrp.getAmbientLight(keys, ambient);

  if( rc == OK )
  {
    ++stats.passCnt;
  }
  else
  {
    printTestResult(FailTag, "%s: Failed to get ambient light measurements.",
      SubSysName);
    showLabel = true;
  }

  if( showLabel )
  {
    printf("%8s", "");
    for(size_t i = 0; i < keys.size(); ++i)
    {
      printf("%10s ", keys[i].c_str());
    }
    printf("\n");
  }

  printf("%6d. ", cnt);
  for(size_t i = 0; i < ambient.size(); ++i)
  {
    printf("%10.2lf ", ambient[i]);
  }
  printf("\r");
  fflush(stdout);

  return stats;
}

DiagStats runToFDiagnostics(bool bAnyKey)
{
  DiagStats   statsTest;
  DiagStats   statsTotal;
  int         cnt;
  bool        bQuit;

  printHdr("Time-of-Flight Sensors Diagnostics");

  LaeRangeSensorGroup  rangegroup(I2CBus);

  //
  // Init Tests
  //
  statsTest = initSensors(rangegroup);

  printSubTotals(statsTest);

  statsTotal += statsTest;

  //
  // Info and Config Tests
  //
  if( !statsTotal.fatal )
  {
    statsTest = getSensorInfo(rangegroup);

    printSubTotals(statsTest);

    statsTotal += statsTest;
  }

  //
  // Take distance measurements.
  //
 
  statsTest.zero();
  bQuit = statsTotal.fatal;
  cnt = 0;

  printSubHdr("Distance Measurements");

  while( !bQuit )
  {
    rangegroup.exec();

    statsTest += measureDistance(rangegroup, cnt++);

    if( !bAnyKey || kbhit() || statsTest.fatal )
    {
      printf("\n");
      printSubTotals(statsTest);
      bQuit = true;
    }
    else
    {
      usleep(100000);
    }
  }

  statsTotal += statsTest;

  //
  // Take ambient light measurements
  //

  statsTest.zero();
  bQuit = statsTotal.fatal;
  cnt = 0;

  printSubHdr("Ambient Light Measurements");

  while( !bQuit )
  {
    rangegroup.exec();

    statsTest += measureAmbient(rangegroup, cnt++);

    if( !bAnyKey || kbhit() || statsTest.fatal )
    {
      printf("\n");
      printSubTotals(statsTest);
      bQuit = true;
    }
    else
    {
      usleep(500000);
    }
  }

  statsTotal += statsTest;

  //
  // Summary
  //
  printTotals(statsTotal);

  return statsTotal;
}
