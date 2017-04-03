////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Program:   laelaps_diag   
//
// File:      diagBatt.cxx
//
/*! \file
 *
 * \brief Perform Laelaps battery diagnostics.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2017. RoadNarrows LLC.\n
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
#include <errno.h>

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
#include "Laelaps/RoboClaw.h"
#include "Laelaps/laeGpio.h"
#include "Laelaps/laeWd.h"
#include "Laelaps/laeMotor.h"

#include "laelaps_diag.h"

using namespace std;
using namespace laelaps;
using namespace motor::roboclaw;

static const char *SubSysName = "Batteries";

/*!
 * \brief Laelaps motors subsystem information structure.
 */
struct LaelapsMotorsInfo
{
  const char *m_sCtlrKey;     ///< motor controller key
  uint_t      m_addr;         ///< motor controller address.
  int         m_idx;          ///< motor controller index 
  bool        m_bIsStd;       ///< motor controller is [not] standard option
  bool        m_bBlackListed; ///< motor controller is black listed (bad)
  struct                      ///< motors info sub-structure
  {
    const char *m_sMotorKey;        ///< motor key
    int         m_idx;              ///< motor index in controller
  } m_motors[LaeNumMotorsPerCtlr];  ///< motors info
};

/*!
 * \brief Laelaps motors subsystem information.
 */
static LaelapsMotorsInfo MotorsInfo[LaeNumMotorCtlrs] =
{
  {
    LaeKeyFront, LaeMotorCtlrAddrFront, LaeMotorCtlrIdFront, true,    false,
    { {LaeKeyLeftFront, LaeMotorLeft}, {LaeKeyRightFront, LaeMotorRight} }
  }, 

  {
    LaeKeyRear, LaeMotorCtlrAddrRear, LaeMotorCtlrIdRear, true,    false,
    { {LaeKeyLeftRear, LaeMotorLeft}, {LaeKeyRightRear, LaeMotorRight} }
  }, 
};

//
// The interface.
//
#if 0 // Deprecated
static LaeMotorCtlrEnable MotorCtlrEnable;  ///< motor controllers power enable
static LaeMotorCtlrChipSelect MotorCtlrCs;  ///< motor controllers chip select 
#endif // Deprecated

static RoboClawComm   MotorCtlrComm;    ///< motor ctrl serial communication bus
static RoboClaw      *MotorCtlr[LaeNumMotorCtlrs];
                                            ///< RoboClaw motor controllers

static DiagStats initMotorInterfaces()
{
  string      strDevSymName(LaeDevMotorCtlrs);    // symbolic name
  string      strDevName;                         // real device names
  int         nBaudRate = LaeBaudRateMotorCtlrs;  // serial baudrate
  int         nCtlr;                              // motor controller id
  DiagStats   stats;                              // test stats

  printSubHdr("Initialize Motor Controller Interfaces");

  //
  // Enable power to the motor controllers via watchdog subprocessor gpio.
  //
  ++stats.testCnt;

  if( WatchDog.cmdEnableMotorCtlrs(true) < 0 )
  {
    printTestResult(FatalTag, "%s: Motor controllers power enable.",
        SubSysName);
    stats.fatal = true;
    return stats;
  }
  else
  {
    printTestResult(PassTag, "%s: Enabled power to motor controllers.",
        SubSysName);
    ++stats.passCnt;
  }

#if 0 // Deprecated
  //
  // Enable power to the motor controllers via odroid gpio.
  //
  MotorCtlrEnable.sync();
  if( MotorCtlrEnable.isConfigured() )
  {
    ++stats.testCnt;
    if( MotorCtlrEnable.isEnabled() )
    {
      printTestResult(WarnTag,
        "Motor controllers are already enabled - may be in usse by another "
        "application.");
      ++stats.passCnt;
    }
    else if( MotorCtlrEnable.enable() )
    {
      printTestResult(PassTag, "Motor controllers enabled.");
      ++stats.passCnt;
    }
    else
    {
      printTestResult(FatalTag, "Motor controllers failed to be enabled.");
      stats.fatal = true;
      return stats;
    }
  }
#endif // Deprecated

#if 0 // Deprecated
  //
  // Open the motor controller chip select via gpio
  //
  ++stats.testCnt;

  if( MotorCtlrCs.open(LaeGpioMotorCtlrCs) < 0 )
  {
    printTestResult(FatalTag,
        "Failed to open motor controllers chip select on GPIO pin %d: %s(%d).",
        LaeGpioMotorCtlrCs, strerror(errno), errno);
    stats.fatal = true;
  }
  else
  {
    printTestResult(PassTag,
        "Open motor controllers chip select on GPIO pin %d.",
        LaeGpioMotorCtlrCs);
    ++stats.passCnt;
  }
#endif // Deprecated

  //
  // Open the motor controller.
  //
  ++stats.testCnt;
  
  // get real device name, not any symbolic links
  strDevName = getRealDeviceName(strDevSymName);

  if( MotorCtlrComm.open(strDevName, nBaudRate) < 0 )
  {
    printTestResult(FatalTag,
        "%s: %s: Failed to open motor controllers comm at %d baud: %s(%d)",
        SubSysName, strDevName.c_str(), nBaudRate, strerror(errno), errno);
    stats.fatal = true;
  }
  else
  {
    printTestResult(PassTag,
        "%s: Open motor controllers serial communication on %s@%d.",
        SubSysName, strDevName.c_str(), nBaudRate);
    ++stats.passCnt;
  }

  if( stats.fatal )
  {
    return stats;
  }

  // 
  // Create front motor controller interface.
  //
  ++stats.testCnt;

  nCtlr = LaeMotorCtlrIdFront;
  MotorCtlr[nCtlr] = 
            new RoboClaw(MotorCtlrComm, LaeMotorCtlrAddrFront, LaeKeyFront);
  MotorCtlr[nCtlr]->setMotorDir(LaeMotorLeft, LaeMotorDirNormal);
  MotorCtlr[nCtlr]->setMotorDir(LaeMotorRight, LaeMotorDirNormal);
  printTestResult(PassTag,
      "%s: Created %s motor controller interface.\n  address=0x%02x.",
      SubSysName, LaeKeyFront, LaeMotorCtlrAddrFront);
  ++stats.passCnt;

  // 
  // Create rear motor controller interface.
  //
  ++stats.testCnt;

  nCtlr = LaeMotorCtlrIdRear;
  MotorCtlr[nCtlr]  =
            new RoboClaw(MotorCtlrComm, LaeMotorCtlrAddrRear, LaeKeyRear);
  MotorCtlr[nCtlr]->setMotorDir(LaeMotorLeft, LaeMotorDirNormal);
  MotorCtlr[nCtlr]->setMotorDir(LaeMotorRight, LaeMotorDirNormal);
  printTestResult(PassTag,
      "%s: Created %s motor controller interface, address=0x%02x.",
      SubSysName, LaeKeyRear, LaeMotorCtlrAddrRear);
  ++stats.passCnt;

  return stats;
}

static DiagStats testBatteryState(int cnt)
{
  int       nCtlr;
  double    voltMotBatt[LaeNumMotorCtlrs];
  double    voltWdJack;
  double    voltWdBatt;
  bool      isCharging;
  bool      showLabel;
  int       rc;

  DiagStats stats;

  showLabel = cnt == 0;

  for(nCtlr = 0; nCtlr < LaeNumMotorCtlrs; ++nCtlr)
  {
    ++stats.testCnt;

    rc = MotorCtlr[nCtlr]->cmdReadMainBattVoltage(voltMotBatt[nCtlr]);

    if( rc == OK )
    {
      ++stats.passCnt;
    }
    else
    {
      printTestResult(FailTag,
        "%s: Motor Controller %s(0x%02x): Read main battery voltage.",
      SubSysName, MotorsInfo[nCtlr].m_sCtlrKey, MotorsInfo[nCtlr].m_addr);

      voltMotBatt[nCtlr] = 0.0;
      showLabel = true;
    }
  }

  ++stats.testCnt;

  // petting the dog checks the charging state
  if( (rc = WatchDog.cmdPetTheDog()) == LAE_OK )
  {
    ++stats.passCnt;
    isCharging = WatchDog.isCharging();
  }
  else
  {
    printTestResult(FailTag, "%s: Failed to pet the watchdog.", SubSysName);
    isCharging = false;
    showLabel = true;
  }

  ++stats.testCnt;

  if( (rc = WatchDog.cmdReadVoltages(voltWdJack, voltWdBatt)) == LAE_OK )
  {
    ++stats.passCnt;
  }
  else
  {
    printTestResult(FailTag, "%s: Failed to read the watchdog voltages.",
        SubSysName);
    voltWdJack = 0.0;
    voltWdBatt = 0.0;
    showLabel = true;
  }

  if( showLabel )
  {
    printf("%8s%13s%17s\n", "", "motor-ctlrs", "watchdog   ");
    printf("%8s%6s %6s %6s chg %6s\n", "",
        LaeKeyFront, LaeKeyRear, "batt", "jack");
  }

  printf("%6d. %5.1lfV %5.1lfV %5.1lfV %s %5.1lfV\r",
      cnt,
      voltMotBatt[0], voltMotBatt[1], voltWdBatt,
      (isCharging? "yes": " no"), voltWdJack);
  fflush(stdout);

  return stats;
}

static DiagStats deinitMotorInterfaces()
{
  DiagStats   stats;                              // test stats

  printSubHdr("De-Initialize Motor Controller Interfaces");

  //
  // Enable power to the motor controllers via watchdog subprocessor gpio.
  //
  ++stats.testCnt;

  if( WatchDog.cmdEnableMotorCtlrs(false) < 0 )
  {
    printTestResult(FailTag, "%s: Motor controllers power disable.",
        SubSysName);
  }
  else
  {
    printTestResult(PassTag, "%s: Disabled power to motor controllers.",
        SubSysName);
    ++stats.passCnt;
  }

  //
  // Close serial connection with motor controllers.
  //
  ++stats.testCnt;

  if( MotorCtlrComm.close() < 0 )
  {
    printTestResult(FailTag, "%s: Close motor controllers serial connection.",
        SubSysName);
  }
  else
  {
    printTestResult(PassTag,
        "%s: Close serial connection with motor controllers.",
        SubSysName);
    ++stats.passCnt;
  }

  return stats;
}

DiagStats runBatteryDiagnostics(bool bAnyKey)
{
  DiagStats   statsTest;
  DiagStats   statsTotal;
  int         cnt;
  bool        bQuit;

  printHdr("Battery Diagnostics");

  //
  // Init Tests
  //
  statsTest = initMotorInterfaces();

  printSubTotals(statsTest);

  statsTotal += statsTest;

  statsTest.zero();
  bQuit = statsTotal.fatal;
  cnt = 0;

  printSubHdr("Battery Voltages");

  //
  // Read battery voltages and charging state.
  //
  while( !bQuit )
  {
    statsTest += testBatteryState(cnt++);

    if( !bAnyKey || kbhit() || statsTest.fatal )
    {
      printf("\n");
      printSubTotals(statsTest);
      bQuit = true;
    }

    if( !bQuit )
    {
      usleep(500000);
    }
  }

  statsTotal += statsTest;

  //
  // Deinit Tests
  //
  statsTest = deinitMotorInterfaces();

  printSubTotals(statsTest);

  statsTotal += statsTest;

  printSubTotals(statsTest);

  statsTotal += statsTest;

  //
  // Summary
  //
  printTotals(statsTotal);

  return statsTotal;
}
