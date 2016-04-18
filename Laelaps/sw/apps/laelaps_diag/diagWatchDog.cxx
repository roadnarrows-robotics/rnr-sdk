////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Program:   laelaps_diag   
//
// File:      diagImu.cxx
//
/*! \file
 *
 * $LastChangedDate: 2016-02-01 15:14:45 -0700 (Mon, 01 Feb 2016) $
 * $Rev: 4289 $
 *
 * \brief Perform Laelaps IMU diagnostics.
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
#include "Laelaps/laeWatchDog.h"
#include "Laelaps/laeWd.h"

#include "laelaps_diag.h"

using namespace std;
using namespace laelaps;

static const char *SubSysName = "WatchDog";

static DiagStats initWatchDog(LaeI2C &i2cBus)
{
  DiagStats   stats;

  printSubHdr("Initialize WatchDog");

  //
  // Requirements
  //
  stats.testCnt += 1;
  if( i2cBus.isOpen() )
  {
    printTestResult(PassTag, "Communication interface %s is open.",
        i2cBus.getDevName().c_str());
    stats.passCnt += 1;
  }
  else
  {
    printTestResult(FatalTag,
        "Communication interface not open - cannot continue.");
    stats.fatal = true;
    return stats;
  }

  return stats;
}

static DiagStats readInfo(LaeWd &watchdog)
{
  uint_t    uVerNum;
  DiagStats stats;

  printSubHdr("Read WatchDog Info");

  ++stats.testCnt;
  if( watchdog.cmdGetFwVersion(uVerNum) < 0 )
  {
    printTestResult(FailTag, "%s: Failed to read firmware version number.",
          SubSysName);
    uVerNum = 0;
  }
  else
  {
    printTestResult(PassTag, "%s: Read firmware version number.", SubSysName);
    ++stats.passCnt;
  }

  printf("Info:\n");
  printf("  Name:       %s\n", SubSysName);
  printf("  Desc:       Arduino compatible Atmega328 sub-processor\n");
  printf("  FW Version: %u\n", uVerNum);

  return stats;
}

static DiagStats testAlarms(LaeWd &watchdog)
{
  static uint_t alarms[] =
  {
    LaeWdArgAlarmNone,
    LaeWdArgAlarmGen,
    LaeWdArgAlarmBatt,
    LaeWdArgAlarmTemp,
    LaeWdArgAlarmEStop,
    LaeWdArgAlarmBatt | LaeWdArgAlarmBattCrit,
    LaeWdArgAlarmTemp | LaeWdArgAlarmCrit,
    LaeWdArgAlarmNone
  };

  static const char *names[] =
  {
    "no", "general", "battery", "temperature", "estop",
    "battery critical", "temperature critical", 
    "no"
  };

  DiagStats stats;

  printSubHdr("Test WatchDog Alarm Control");

  for(size_t i = 0; i < arraysize(names); ++i)
  {
    ++stats.testCnt;
    printf("%s Set %s alarms = (0x%04x)\r", WaitTag, names[i], alarms[i]);
    fflush(stdout);
    if( watchdog.cmdSetAlarms(alarms[i]) < 0 )
    {
      printf("%s\n", FailTag);
    }
    else
    {
      usleep(4000000);
      printf("%s\n", PassTag);
      ++stats.passCnt;
    }
  }

  return stats;
}

static DiagStats testBatteryCharge(LaeWd &watchdog)
{
  static uint_t charge[] =
  {
    100, 90, 75, 60, 50, 40, 25, 10, 0, 100
  };

  DiagStats stats;

  printSubHdr("Test Battery Charge LED Control");

  for(size_t i = 0; i < arraysize(charge); ++i)
  {
    ++stats.testCnt;
    printf("%s Set battery charge to %u%%\r", WaitTag, charge[i]);
    fflush(stdout);
    if( watchdog.cmdSetBatterySoC(charge[i]) < 0 )
    {
      printf("%s\n", FailTag);
    }
    else
    {
      usleep(2000000);
      printf("%s\n", PassTag);
      ++stats.passCnt;
    }
  }

  return stats;
}

static DiagStats testLeds(LaeWd &watchdog)
{
  static byte_t    rgb[][3] =
  {
    {255, 0, 0}, {255, 255, 0}, {0, 255, 0},
    {0, 255, 255}, {0, 0, 255}, {255, 0, 255}
  };

  static const char *colors[] =
  {
    "red", "yellow", "green", "cyan", "blue", "magenta"
  };

  DiagStats stats;

  printSubHdr("Test USER LED Control");

  for(size_t i = 0; i < arraysize(colors); ++i)
  {
    ++stats.testCnt;
    printf("%s Set RGB LED to %s = (%u, %u, %u)\r",
          WaitTag, colors[i], rgb[i][0], rgb[i][1], rgb[i][2]);
    fflush(stdout);
    if( watchdog.cmdSetRgbLed(rgb[i][0], rgb[i][1], rgb[i][2]) < 0 )
    {
      printf("%s\n", FailTag);
    }
    else
    {
      usleep(1000000);
      printf("%s\n", PassTag);
      ++stats.passCnt;
    }
  }

  ++stats.testCnt;
  if( watchdog.cmdResetRgbLed() < 0 )
  {
    printTestResult(FailTag, "Failed to reset RGB LED to state defaults.");
  }
  else
  {
    printTestResult(PassTag, "Reset RGB LED to state defaults.");
    ++stats.passCnt;
  }

  return stats;
}

static DiagStats testDigitalIO(LaeWd &watchdog)
{
  uint_t      pin;
  uint_t      value;
  const char *sTag;
  DiagStats   stats;

  printSubHdr("Test Digital I/O Control");

  for(pin = LaeWdArgDPinNumMin; pin <= LaeWdArgDPinNumMax; ++pin)
  {
    printf("%s * Pin %u\n", WaitTag, pin);

    //
    // Configure pin as output.
    //
    ++stats.testCnt;
    if( watchdog.cmdConfigDPin(pin, LaeWdArgDPinDirOut) == LAE_OK )
    {
      ++stats.passCnt;
      sTag = PassTag;
    }
    else
    {
      sTag = FailTag;
    }
    printTestResult(sTag, "Configure digital pin %u to output.", pin);

    //
    // Write high value to pin.
    //
    ++stats.testCnt;
    value = LaeWdArgDPinValHigh;
    if( watchdog.cmdWriteDPin(pin, value) == LAE_OK )
    {
      ++stats.passCnt;
      sTag = PassTag;
    }
    else
    {
      sTag = FailTag;
    }
    printTestResult(sTag, "Write digital pin %u value to high=%u.", pin, value);

    //
    // Read pin's value.
    //
    ++stats.testCnt;
    if( watchdog.cmdReadDPin(pin, value) == LAE_OK )
    {
      ++stats.passCnt;
      sTag = PassTag;
      printTestResult(sTag, "Read digital pin %u value=%u.", pin, value);
    }
    else
    {
      sTag = FailTag;
      printTestResult(sTag, "Read digital pin %u.", pin);
    }

    //
    // Write low value to pin.
    //
    ++stats.testCnt;
    value = LaeWdArgDPinValLow;
    if( watchdog.cmdWriteDPin(pin, value) == LAE_OK )
    {
      ++stats.passCnt;
      sTag = PassTag;
    }
    else
    {
      sTag = FailTag;
    }
    printTestResult(sTag, "Write digital pin %u value to low=%u.", pin, value);
  }

  return stats;
}

static DiagStats testAnalogOutput(LaeWd &watchdog)
{
  uint_t      pin;
  uint_t      value;
  const char *sTag;
  DiagStats   stats;

  printSubHdr("Test Analog Output");

  for(pin = LaeWdArgAOutPinNumMin; pin <= LaeWdArgAOutPinNumMax; ++pin)
  {
    printf("%s * Pin %u\n", WaitTag, pin);

    //
    // Write analog high value to pin.
    //
    ++stats.testCnt;
    value = LaeWdArgAOutPinValMax;
    if( watchdog.cmdWriteAPin(pin, value) == LAE_OK )
    {
      ++stats.passCnt;
      sTag = PassTag;
    }
    else
    {
      sTag = FailTag;
    }
    printTestResult(sTag, "Write pin %u analog value=%u.", pin, value);

    //
    // Write digital high value to pin.
    //
    ++stats.testCnt;
    value = LaeWdArgDPinValLow;
    if( watchdog.cmdWriteDPin(pin, value) == LAE_OK )
    {
      ++stats.passCnt;
      sTag = PassTag;
    }
    else
    {
      sTag = FailTag;
    }
    printTestResult(sTag, "Write pin %u digital value to low=%u.", pin, value);
  }

  return stats;
}


static DiagStats testAnalogInput(LaeWd &watchdog)
{
  uint_t      pin;
  uint_t      value;
  const char *sTag;
  DiagStats   stats;

  printSubHdr("Test Analog Input");

  for(pin = LaeWdArgAInPinNumMin; pin <= LaeWdArgAInPinNumMax; ++pin)
  {
    printf("%s * Pin %u\n", WaitTag, pin);

    //
    // Read pin's value.
    //
    ++stats.testCnt;
    if( watchdog.cmdReadAPin(pin, value) == LAE_OK )
    {
      ++stats.passCnt;
      sTag = PassTag;
      printTestResult(sTag, "Read analog pin %u value=%u.", pin, value);
    }
    else
    {
      sTag = FailTag;
      printTestResult(sTag, "Read analog pin %u.", pin);
    }
  }

  return stats;
}

DiagStats runWatchDogDiagnostics(LaeI2C &i2cBus)
{
  LaeWd       watchdog(i2cBus);
  DiagStats   statsTest;
  DiagStats   statsTotal;

  printHdr("WatchDog Diagnostics");

  watchdog.sync();

  //
  // Init Tests
  //
  statsTest = initWatchDog(i2cBus);
  printSubTotals(statsTest);
  statsTotal += statsTest;

  //
  // Read and Print Information Tests
  //
  if( !statsTotal.fatal )
  {
    statsTest = readInfo(watchdog);
    printSubTotals(statsTest);
    statsTotal += statsTest;
  }
 
  //
  // Alarm Tests
  //
  if( !statsTotal.fatal )
  {
    statsTest = testAlarms(watchdog);
    printSubTotals(statsTest);
    statsTotal += statsTest;
  }
 
  //
  // Battery Charge Tests
  //
  if( !statsTotal.fatal )
  {
    statsTest = testBatteryCharge(watchdog);
    printSubTotals(statsTest);
    statsTotal += statsTest;
  }
 
  //
  // RGB LED Tests
  //
  if( !statsTotal.fatal )
  {
    statsTest = testLeds(watchdog);
    printSubTotals(statsTest);
    statsTotal += statsTest;
  }
 
  //
  // Digital I/O Tests
  //
  if( !statsTotal.fatal )
  {
    statsTest = testDigitalIO(watchdog);
    printSubTotals(statsTest);
    statsTotal += statsTest;
  }
 
  //
  // Analog Output Tests
  //
  if( !statsTotal.fatal )
  {
    statsTest = testAnalogOutput(watchdog);
    printSubTotals(statsTest);
    statsTotal += statsTest;
  }
 
  //
  // Analog Input Tests
  //
  if( !statsTotal.fatal )
  {
    statsTest = testAnalogInput(watchdog);
    printSubTotals(statsTest);
    statsTotal += statsTest;
  }
 
  //
  // Summary
  //
  printTotals(statsTotal);

  return statsTotal;
}