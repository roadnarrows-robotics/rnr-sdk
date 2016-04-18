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
 * $LastChangedDate: 2016-02-01 15:14:45 -0700 (Mon, 01 Feb 2016) $
 * $Rev: 4289 $
 *
 * \brief Perform Laelaps Time-of-Flight sensors diagnostics.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2015  RoadNarrows
 * (http://www.RoadNarrows.com)
 * \n All Rights Reserved
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
#include "Laelaps/laeSysDev.h"
#include "Laelaps/laeI2C.h"
#include "Laelaps/laeI2CMux.h"
#include "Laelaps/laeVL6180.h"

#include "laelaps_diag.h"

using namespace std;
using namespace laelaps;
using namespace sensor::vl6180;

static const char *SubSysName = "ToF";
static const char *ProdName   = "VL6180";

/*!
 * \brief Laelaps Time-of-Flight subsystem information structure.
 */
struct LaelapsToFInfo
{
  const char *m_sNameKey;     ///< sensor name key
  int         m_nChan;        ///< sensor mux channel
  double      m_fDir;         ///< sensor beam center direction
  bool        m_bIsStd;       ///< sensor is [not] standard option
  bool        m_bBlackListed; ///< sensor is black listed (bad)
};

/*!
 * \brief Laelaps Time-of-Flight subsystem information.
 */
static LaelapsToFInfo ToFInfo[ToFSensorMaxNumOf] =
{
  {"front",       ToFSensor0Chan, ToFSensor0Dir, true,    false},
  {"left_front",  ToFSensor1Chan, ToFSensor1Dir, true,    false},
  {"left",        ToFSensor2Chan, ToFSensor2Dir, false,   false},
  {"left_rear",   ToFSensor3Chan, ToFSensor3Dir, false,   false},
  {"rear",        ToFSensor4Chan, ToFSensor4Dir, false,   false},
  {"right_rear",  ToFSensor5Chan, ToFSensor5Dir, false,   false},
  {"right",       ToFSensor6Chan, ToFSensor6Dir, false,   false},
  {"right_front", ToFSensor7Chan, ToFSensor7Dir, true,    false}
};

static vector<LaeVL6180Mux*>  ToFSensors;

static DiagStats initSensors(LaeI2CMux &i2cMux)
{
  size_t      i;
  DiagStats   stats;

  printSubHdr("Initialize Sensors");

  //
  // Requirements
  //
  stats.testCnt += 1;
  if( i2cMux.isOpen() )
  {
    printTestResult(PassTag, "Communication interface %s is open.",
        i2cMux.getDevName().c_str());
    stats.passCnt += 1;
  }
  else
  {
    printTestResult(FatalTag,
        "Communication interface not open - cannot continue.");
    stats.fatal = true;
    return stats;
  }

  //
  // Create
  //
  for(i=0; i<arraysize(ToFInfo); ++i)
  {
    ToFSensors.push_back(new LaeVL6180Mux(i2cMux,
                                  ToFInfo[i].m_nChan,
                                  ToFInfo[i].m_fDir,
                                  0.0, 
                                  ToFInfo[i].m_sNameKey));
  }

  //
  // Initialize
  //
  for(i=0; i<ToFSensors.size(); ++i)
  {
    if( !ToFInfo[i].m_bIsStd )
    {
      continue;
    }

    ++stats.testCnt;

    if( ToFSensors[i]->initSensor() == LAE_OK )
    {
      printTestResult(PassTag, "%s Sensor %s(%d): Initialized.",
          SubSysName, ToFInfo[i].m_sNameKey, ToFInfo[i].m_nChan);
      ++stats.passCnt;
    }
    else
    {
      printTestResult(FailTag, "%s Sensor %s(%d): Failed to initialize.",
          SubSysName, ToFInfo[i].m_sNameKey, ToFInfo[i].m_nChan);
      ToFInfo[i].m_bBlackListed = true;
    }
  }

  return stats;
}

static DiagStats readInfo()
{
  struct VL6180xIdentification id;
  byte_t        valRangeOffset;
  byte_t        valRangeCrossTalk;
  byte_t        valAlsGain;
  u16_t         valAlsIntPeriod;
  const char   *sTag;
  DiagStats     stats;

  printSubHdr("Read Sensors Information and Configuration");

  //
  // Read Info and Config
  //
  for(size_t i = 0; i < ToFSensors.size(); ++i)
  {
    if( !ToFInfo[i].m_bIsStd )
    {
      continue;
    }

    if( ToFInfo[i].m_bBlackListed )
    {
      ++stats.testCnt;
      printTestResult(FailTag, "%s %s(%d) sensor: Black listed.",
          SubSysName, ToFInfo[i].m_sNameKey, ToFInfo[i].m_nChan);
      continue;
    }

    ++stats.testCnt;

    memset(&id, 0, sizeof(struct VL6180xIdentification));
    sTag = FailTag;
    if( ToFSensors[i]->readId(id) == LAE_OK )
    {
      ++stats.passCnt;
      sTag = PassTag;
    }
    printTestResult(sTag,
          "%s %s(%d) sensor: Read sensor identification.",
          SubSysName, ToFInfo[i].m_sNameKey, ToFInfo[i].m_nChan);

    ++stats.testCnt;

    valRangeOffset = 0;
    sTag = FailTag;
    if( ToFSensors[i]->readReg8(VL6180X_SYSRANGE_PART_TO_PART_RANGE_OFFSET,
                                valRangeOffset) == LAE_OK )
    {
      ++stats.passCnt;
      sTag = PassTag;
    }
    printTestResult(sTag,
          "%s %s(%d) sensor: Read range sensor part-to-part offset.",
          SubSysName, ToFInfo[i].m_sNameKey, ToFInfo[i].m_nChan);

    ++stats.testCnt;

    valRangeCrossTalk = 0;
    sTag = FailTag;
    if( ToFSensors[i]->readReg8(VL6180X_SYSRANGE_CROSSTALK_COMPENSATION_RATE,
                                valRangeCrossTalk) == LAE_OK )
    {
      ++stats.passCnt;
      sTag = PassTag;
    }
    printTestResult(sTag,
          "%s %s(%d) sensor: Read range sensor cross-talk compensation.",
          SubSysName, ToFInfo[i].m_sNameKey, ToFInfo[i].m_nChan);

    ++stats.testCnt;

    valAlsGain = 0;
    sTag = FailTag;
    if( ToFSensors[i]->readReg8(VL6180X_SYSALS_ANALOGUE_GAIN,
                                valAlsGain) == LAE_OK )
    {
      ++stats.passCnt;
      sTag = PassTag;
    }
    printTestResult(sTag,
          "%s %s(%d) sensor: Read ambient light sensor gain.",
          SubSysName, ToFInfo[i].m_sNameKey, ToFInfo[i].m_nChan);

    ++stats.testCnt;

    valAlsIntPeriod = 0;
    sTag = FailTag;
    if( ToFSensors[i]->readReg16(VL6180X_SYSALS_INTEGRATION_PERIOD,
                                valAlsIntPeriod) == LAE_OK )
    {
      ++stats.passCnt;
      sTag = PassTag;
    }
    printTestResult(sTag,
          "%s %s(%d) sensor: Read ambient light sensor integration period.",
          SubSysName, ToFInfo[i].m_sNameKey, ToFInfo[i].m_nChan);

    printf("\n");
    printf("%s %s(%d) Sensor Info:\n",
          SubSysName, ToFInfo[i].m_sNameKey, ToFInfo[i].m_nChan);
    printf("  Name:             %s\n", ToFSensors[i]->getNameId().c_str());
    printf("  Desc:             %s\n", ToFSensors[i]->getDesc().c_str());
    printf("  Beam Direction:   %.1lf degress\n",
        radToDeg(ToFSensors[i]->getBeamDir()));
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
    printf("\n");
  }

  return stats;
}

static DiagStats measure()
{
  double    fMeters;
  double    fLux;
  int       cnt;
  int       measCnt;
  DiagStats stats;

  printSubHdr("Make Measurements");

  //
  // Measure
  //
  for(size_t i = 0; i < ToFSensors.size(); ++i)
  {
    if( !ToFInfo[i].m_bIsStd )
    {
      continue;
    }

    if( ToFInfo[i].m_bBlackListed )
    {
      ++stats.testCnt;

      printTestResult(FailTag, "%s %s(%d) sensor: Black listed.",
          SubSysName, ToFInfo[i].m_sNameKey, ToFInfo[i].m_nChan);
      continue;
    }

    printf("%s %s %s(%d) sensor: Range(meters): ",
          WaitTag, SubSysName, ToFInfo[i].m_sNameKey, ToFInfo[i].m_nChan);
    fflush(stdout);

    //
    // Meassure object range N times.
    //
    ++stats.testCnt;

    for(cnt = 0, measCnt = 0; cnt < 4; ++cnt)
    {
      fMeters = ToFSensors[i]->measureRange();

      if( fMeters == VL6180X_ERR_MEAS )
      {
        printf("error ");
      }
      else if( fMeters == VL6180X_RANGE_NO_OBJ )
      {
        ++measCnt;
        printf("no_obj ");
      }
      else
      {
        ++measCnt;
        printf("%.3lf ", fMeters);
      }
      fflush(stdout);
    }

    if( measCnt >= (cnt - 1) )
    {
      printf("\r%s\n", PassTag);
      ++stats.passCnt;
    }
    else
    {
      printf("\r%s\n", FailTag);
    }

    printf("%s %s %s(%d) sensor: Ambient(lux):  ",
          WaitTag, SubSysName, ToFInfo[i].m_sNameKey, ToFInfo[i].m_nChan);
    fflush(stdout);

    //
    // Meassure ambient light N times.
    //
    ++stats.testCnt;

    for(cnt = 0, measCnt = 0; cnt < 4; ++cnt)
    {
      fLux = ToFSensors[i]->measureAmbientLight();

      if( fLux == VL6180X_ERR_MEAS )
      {
        printf("error ");
      }
      else
      {
        ++measCnt;
        printf("%.1lf ", fLux);
      }
      fflush(stdout);
    }

    if( measCnt >= (cnt - 1) )
    {
      printf("\r%s\n", PassTag);
      ++stats.passCnt;
    }
    else
    {
      printf("\r%s\n", FailTag);
    }
  }

  return stats;
}

DiagStats runToFDiagnostics(LaeI2CMux &i2cMux)
{
  DiagStats   statsTest;
  DiagStats   statsTotal;

  printHdr("Time-of-Flight Sensors Diagnostics");

  //
  // Init Tests
  //
  statsTest = initSensors(i2cMux);

  printSubTotals(statsTest);

  statsTotal += statsTest;

  //
  // Info and Config Tests
  //
  if( !statsTotal.fatal )
  {
    statsTest = readInfo();

    printSubTotals(statsTest);

    statsTotal += statsTest;
  }
 
  //
  // Take measurements
  //
  if( !statsTotal.fatal )
  {
    statsTest = measure();

    printSubTotals(statsTest);

    statsTotal += statsTest;
  }
 
  //
  // Summary
  //
  printTotals(statsTotal);

  return statsTotal;
}
