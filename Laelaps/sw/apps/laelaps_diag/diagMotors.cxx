////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Program:   laelaps_diag   
//
// File:      diagMotors.cxx
//
/*! \file
 *
 * $LastChangedDate: 2016-02-01 15:14:45 -0700 (Mon, 01 Feb 2016) $
 * $Rev: 4289 $
 *
 * \brief Perform Laelaps motors diagnostics.
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

static const char *SubSysName = "Motors";
static const char *ProdName   = "RoboClaw";

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

static DiagStats initMotorInterfaces(LaeWd &watchdog)
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

  if( watchdog.cmdEnableMotorCtlrs(true) < 0 )
  {
    printTestResult(FatalTag, "Motor controllers power enable.");
    stats.fatal = true;
    return stats;
  }
  else
  {
    printTestResult(PassTag, "Enabled power to motor controllers.");
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
        "%s: Failed to open motor controllers comm at %d baud: %s(%d)",
        strDevName.c_str(), nBaudRate, strerror(errno), errno);
    stats.fatal = true;
  }
  else
  {
    printTestResult(PassTag,
        "Open motor controllers serial communication on %s@%d.",
        strDevName.c_str(), nBaudRate);
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
      "Created %s motor controller interface, address=0x%02x.",
      LaeKeyFront, LaeMotorCtlrAddrFront);
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
      "Created %s motor controller interface, address=0x%02x.",
      LaeKeyRear, LaeMotorCtlrAddrRear);
  ++stats.passCnt;

  return stats;
}

static DiagStats readCtlrState(LaelapsMotorsInfo &info)
{
  RoboClaw   *pCtlr;                        // motor controller
  string      strFwVer;                     // firmware version 
  double      fMainBattMinV, fMainBattMaxV; // main battery voltage cutoffs
  double      fLogicMinV, fLogicMaxV;       // logic voltage cutoffs
  uint_t      uErrors;                      // error bits
  double      fTemp;                        // board temperature
  double      fMainBattV;                   // main battery voltage
  double      fLogicV;                      // logic voltage

  const char *sTag;                         // pass/fail/fatal tag
  DiagStats   stats;                        // test stats

  printSubHdr("Read Motor Controller Information and State");

  pCtlr = MotorCtlr[info.m_idx];

  ++stats.testCnt;
  sTag = FailTag;
  if( pCtlr->cmdReadFwVersion(strFwVer) == OK )
  {
    sTag = PassTag;
    ++stats.passCnt;
  }
  printTestResult(sTag, "Motor Controller %s(0x%02x): Read firmware version.",
      info.m_sCtlrKey, info.m_addr);

  ++stats.testCnt;
  fMainBattMinV = 0.0;
  fMainBattMaxV = 0.0;
  sTag = FailTag;
  if( pCtlr->cmdReadMainBattCutoffs(fMainBattMinV, fMainBattMaxV) == OK )
  {
    sTag = PassTag;
    ++stats.passCnt;
  }
  printTestResult(sTag,
      "Motor Controller %s(0x%02x): Read main battery voltage cutoffs.",
      info.m_sCtlrKey, info.m_addr);

  ++stats.testCnt;
  fLogicMinV = 0.0;
  fLogicMaxV = 0.0;
  sTag = FailTag;
  if( pCtlr->cmdReadLogicCutoffs(fLogicMinV, fLogicMaxV) == OK )
  {
    sTag = PassTag;
    ++stats.passCnt;
  }
  printTestResult(sTag,
      "Motor Controller %s(0x%02x): Read logic voltage cutoffs.",
      info.m_sCtlrKey, info.m_addr);

  ++stats.testCnt;
  uErrors = 0;
  sTag = FailTag;
  if( pCtlr->cmdReadStatus(uErrors) == OK )
  {
    sTag = PassTag;
    ++stats.passCnt;
  }
  printTestResult(sTag, "Motor Controller %s(0x%02x): Read error status.",
      info.m_sCtlrKey, info.m_addr);

  ++stats.testCnt;
  fTemp = 0.0;
  sTag = FailTag;
  if( pCtlr->cmdReadBoardTemperature(fTemp) == OK )
  {
    sTag = PassTag;
    ++stats.passCnt;
  }
  printTestResult(sTag, "Motor Controller %s(0x%02x): Read board temperature.",
      info.m_sCtlrKey, info.m_addr);


  ++stats.testCnt;
  fMainBattV = 0.0;
  sTag = FailTag;
  if( pCtlr->cmdReadMainBattVoltage(fMainBattV) == OK )
  {
    sTag = PassTag;
    ++stats.passCnt;
  }
  printTestResult(sTag,
      "Motor Controller %s(0x%02x): Read main battery voltage.",
      info.m_sCtlrKey, info.m_addr);

  ++stats.testCnt;
  fLogicV = 0.0;
  sTag = FailTag;
  if( pCtlr->cmdReadLogicVoltage(fLogicV) == OK )
  {
    sTag = PassTag;
    ++stats.passCnt;
  }
  printTestResult(sTag,
      "Motor Controller %s(0x%02x): Read logic voltage.",
      info.m_sCtlrKey, info.m_addr);

  printf("\n");
  printf("Motor controller %s Info and State Summary:\n", info.m_sCtlrKey);
  printf("  Position:                %s\n", info.m_sCtlrKey);
  printf("  Address:                 0x%02x\n", pCtlr->getAddress());
  printf("  Motor Controller:        %s\n", ProdName);
  printf("  FW Version:              %s\n", strFwVer.c_str());
  printf("  Main Battery Cutoffs(V): [%.1lf, %.1lf]\n",
      fMainBattMinV, fMainBattMaxV);
  printf("  Logic Cutoffs(V):        [%.1lf, %.1lf]\n", fLogicMinV, fLogicMaxV);
  printf("  Status:                  0x%04x\n", uErrors);
  printf("  Temperature(C):          %.1lf\n", fTemp);
  printf("  Battery(V):              %.1lf\n", fMainBattV);
  printf("  Logic(v):                %.1lf\n", fLogicV);
  printf("\n");

  return stats;
}

static DiagStats testMotorCtlrsState()
{
  int       nCtlr;

  DiagStats stats;
  DiagStats statsSubTotal;

  for(nCtlr = 0; nCtlr < LaeNumMotorCtlrs; ++nCtlr)
  {
    stats = readCtlrState(MotorsInfo[nCtlr]);

    printSubTotals(stats);

    statsSubTotal += stats;
  }
    
  return statsSubTotal;
}

static DiagStats readMotorState(LaelapsMotorsInfo &info, int nMotor)
{
  RoboClaw   *pCtlr;          // motor controller
  const char *sCtlrKey;       // motor controller key
  const char *sMotorKey;      // motor key
  byte_t      mode, dummy;    // motor encoder modes
  string      strEncMode;     // encoder mode, translated
  u32_t       uKp, uKi, uKd;  // PID constants
  u32_t       qpps;           // quadrature pulses/second
  double      Kp, Ki, Kd;     // PID constants (translated)
  s64_t       encoder;        // motor encoder position
  s32_t       speed;          // motor speed (qpps)
  double      fMaxAmps;       // motor maximum current limit
  double      amps, dummy2;   // motor current draw (amperes)
  int         rc;             // return code

  const char *sTag;           // pass/fail/fatal tag
  DiagStats   stats;          // test stats

  printSubHdr("Read Motor Information and State");

  pCtlr     = MotorCtlr[info.m_idx];
  sCtlrKey  = info.m_sCtlrKey;
  sMotorKey = info.m_motors[nMotor].m_sMotorKey;

  ++stats.testCnt;
  mode = 0xff;
  sTag = FailTag;
  if( nMotor == Motor1 )
  {
    rc = pCtlr->cmdReadEncoderMode(mode, dummy);
  }
  else
  {
    rc = pCtlr->cmdReadEncoderMode(dummy, mode);
  }
  if( rc == OK )
  {
    sTag = PassTag;
    ++stats.passCnt;
  }
  printTestResult(sTag, "Motor %s: Read encoder mode.", sMotorKey);

  ++stats.testCnt;
  sTag = FailTag;
  if( mode == ParamEncModeQuad )
  {
    strEncMode = "Quadrature";
    sTag = PassTag;
    ++stats.passCnt;
  }
  else if( mode & ParamEncModeAbs )
  {
    strEncMode = "Absolute";
  }
  else if( mode & ParamEncModeRCAnalogEn )
  {
    strEncMode = "RC Analog";
  }
  else
  {
    strEncMode = "Unknown";
  }
  printTestResult(sTag, "Motor %s: Encoder mode 0x%02x = %s.",
      sMotorKey, mode, strEncMode.c_str());

  ++stats.testCnt;
  Kp = Ki = Kd = 0.0;
  qpps = 0;
  sTag = FailTag;
  if( pCtlr->cmdReadVelocityPidConst(nMotor, uKp, uKi, uKd, qpps) == OK )
  {
    sTag = PassTag;
    ++stats.passCnt;

    Kp = (double)uKp / (double)ParamVelPidCvt;
    Ki = (double)uKi / (double)ParamVelPidCvt;
    Kd = (double)uKd / (double)ParamVelPidCvt;
  }
  printTestResult(sTag, "Motor %s: Read velocity PID constants.", sMotorKey);

  ++stats.testCnt;
  fMaxAmps = 0.0;
  sTag = FailTag;
  rc = pCtlr->cmdReadMotorMaxCurrentLimit(mode, fMaxAmps);
  if( rc == OK )
  {
    sTag = PassTag;
    ++stats.passCnt;
  }
  printTestResult(sTag, "Motor %s: Read maximum current limit.", sMotorKey);

  ++stats.testCnt;
  sTag = FailTag;
  if( fMaxAmps == LaeMotorMaxAmps )
  {
    sTag = PassTag;
    ++stats.passCnt;
  }
  printTestResult(sTag, "Motor %s: Maximum current limit %.2lf = req'd %.2lf.",
      sMotorKey, fMaxAmps, LaeMotorMaxAmps);

  ++stats.testCnt;
  encoder = 0;
  sTag = FailTag;
  if( pCtlr->cmdReadQEncoder(nMotor, encoder) == OK )
  {
    sTag = PassTag;
    ++stats.passCnt;
  }
  printTestResult(sTag, "Motor %s: Read motor encoder position.", sMotorKey);

  ++stats.testCnt;
  speed = 0;
  sTag = FailTag;
  if( pCtlr->cmdReadQSpeed(nMotor, speed) == OK )
  {
    sTag = PassTag;
    ++stats.passCnt;
  }
  printTestResult(sTag, "Motor %s: Read motor speed.", sMotorKey);

  ++stats.testCnt;
  amps = 0.0;
  sTag = FailTag;
  if( nMotor == Motor1 )
  {
    rc = pCtlr->cmdReadMotorCurrentDraw(amps, dummy2);
  }
  else
  {
    rc = pCtlr->cmdReadMotorCurrentDraw(dummy2, amps);
  }
  if( rc == OK )
  {
    sTag = PassTag;
    ++stats.passCnt;
  }
  printTestResult(sTag, "Motor %s: Read motor current draw.", sMotorKey);

  printf("\n");
  printf("Motor %s Info and State Summary:\n", sMotorKey);
  printf("  Encoder Mode: 0x%02x (%s)\n", mode, strEncMode.c_str());
  printf("  Velocity PID:\n");
  printf("    Kp:       %3lf (raw = 0x%08x)\n", Kp, uKp);
  printf("    Ki:       %3lf (raw = 0x%08x)\n", Ki, uKi);
  printf("    Kd:       %3lf (raw = 0x%08x)\n", Kd, uKd);
  printf("    Max QPPS: %u\n", qpps);
  printf("  Max. Current: %.2lf (amperes)\n", fMaxAmps);
#ifdef ARCH_x86_64
  printf("  Encoder:      %ld (pulses)\n", encoder);
#else
  printf("  Encoder:      %lld (pulses)\n", encoder);
#endif
  printf("  Speed:        %d (qpps)\n", speed);
  printf("  Draw:         %.2lf (amperes)\n", amps);

  printf("\n");

  return stats;
}

static DiagStats testMotorsState()
{
  int       i, j;           // working indices

  DiagStats stats;          // single motor test stats
  DiagStats statsSubTotal;  // subtotal stats

  for(i = 0; i < LaeNumMotorCtlrs; ++i)
  {
    for(j = 0; j < LaeNumMotorCtlrs; ++j)
    {
      stats = readMotorState(MotorsInfo[i], MotorsInfo[i].m_motors[j].m_idx);

      printSubTotals(stats);

      statsSubTotal += stats;
    }
  }
    
  return statsSubTotal;
}

static DiagStats testMotors(LaeWd &watchdog)
{
  // test speed profile
  static s32_t speedProf[] =
  {
    1000, 5000, 8000, 5000, 1000, -1000, -5000, -8000, -5000, -1000
  };

  uint_t  red, green, blue; 
  float   wt;

  int         i, j;         // working indices
  size_t      k;            // working speed profile index
  double      t;            // working time
  RoboClaw   *pCtlr;        // motor controller
  const char *sCtlrKey;     // motor controller key
  int         nMotor;       // motor index
  const char *sMotorKey;    // motor key
  s64_t       encoder;      // motor encoder position
  s32_t       speed;        // motor speed (qpps)
  s32_t       safeSpeed;    // motor target safe speed for mathy things
  bool        bAtSpeed;     // did [not] reach target speed

  const char *sTag;         // wait/pass/fail/fatal tag
  DiagStats   stats;        // test stats

  //
  // Initialize
  //
  for(i = 0; i < LaeNumMotorCtlrs; ++i)
  {
    pCtlr     = MotorCtlr[MotorsInfo[i].m_idx];
    sCtlrKey  = MotorsInfo[i].m_sCtlrKey;

    ++stats.testCnt;
    sTag = FailTag;
    if( pCtlr->cmdQStop() == OK )
    {
      sTag = PassTag;
      ++stats.passCnt;
      watchdog.cmdResetRgbLed();
    }
    printTestResult(sTag, "%s motors stopped.", sCtlrKey);

    ++stats.testCnt;
    sTag = FailTag;
    if( pCtlr->cmdResetQEncoders() == OK )
    {
      sTag = PassTag;
      ++stats.passCnt;
    }
    printTestResult(sTag, "%s motor encoders reset.", sCtlrKey);
  }
    
  //
  // Drive motors
  //
  for(i = 0; i < LaeNumMotorCtlrs; ++i)
  {
    pCtlr     = MotorCtlr[MotorsInfo[i].m_idx];
    sCtlrKey  = MotorsInfo[i].m_sCtlrKey;

    for(j = 0; j < LaeNumMotorCtlrs; ++j)
    {
      nMotor    = MotorsInfo[i].m_motors[j].m_idx;
      sMotorKey = MotorsInfo[i].m_motors[j].m_sMotorKey;

      for(k = 0; k < arraysize(speedProf); ++k)
      {
        ++stats.testCnt;
        if( pCtlr->cmdQDrive(nMotor, speedProf[k]) == OK )
        {
          ++stats.passCnt;
          printTestResult(PassTag, "Motor %s: Set target speed=%d.",
              sMotorKey, speedProf[k]);

          wt = fabs((float)speedProf[k] / 8000.0);
          if( speedProf[k] < 0 )
          {
            red = (uint_t)(255.0 * wt);
            green = 0;
            blue = 0;
          }
          else
          {
            red = 0;
            green = 0;
            blue = (uint_t)(255.0 * wt);
          }
          watchdog.cmdSetRgbLed(red, green, blue);

          usleep(2000);
        }
        else
        {
          printTestResult(FailTag, "Motor %s: Set target speed=%d.",
              sMotorKey, speedProf[k]);
          continue;
        }

        ++stats.testCnt;
        sTag = FailTag;
        safeSpeed   = speedProf[k] != 0? speedProf[k]: 1;
        bAtSpeed = false;

        for(t = 0.0; !bAtSpeed && t <= 4.0; t += 0.1)
        {
          speed   = 0;
          encoder = 0;
          pCtlr->cmdReadQSpeed(nMotor, speed);
          pCtlr->cmdReadQEncoder(nMotor, encoder);
          if( fabs(speed - speedProf[k])/ fabs(safeSpeed) <= 0.05 )
          {
            bAtSpeed = true;
          }
          else
          {
            usleep(100000); // 1/10th of a second
          }
          if( isatty(fileno(stdout)) )
          {    
#ifdef ARCH_x86_64
            printf("%s Motor %s: Accelerating: speed=%5d: encoder=%8ld\r",
              WaitTag, sMotorKey, speed, encoder);
#else
            printf("%s Motor %s: Accelerating: speed=%5d: encoder=%8lld\r",
              WaitTag, sMotorKey, speed, encoder);
#endif // ARCH_x86_64
          fflush(stdout);
          }
        }
        
        if( bAtSpeed )
        {
          sTag = PassTag;
          ++stats.passCnt;
        }
#ifdef ARCH_x86_64
        printTestResult(sTag,
            "Motor %s: Accelerating: speed=%5d: encoder %8ld.",
            sMotorKey, speed, encoder);
#else
        printTestResult(sTag,
            "Motor %s: Accelerating: speed=%5d: encoder %8lld.",
            sMotorKey, speed, encoder);
#endif // ARCH_x86_64
      }

      ++stats.testCnt;
      sTag = FailTag;
      if( pCtlr->cmdStop(nMotor) == OK )
      {
        sTag = PassTag;
        ++stats.passCnt;
        watchdog.cmdResetRgbLed();
      }
      printTestResult(sTag, "Motor %s stopped.", sMotorKey);
    }
  }

  return stats;
}

static DiagStats deinitMotorInterfaces(LaeWd &watchdog)
{
  DiagStats   stats;                              // test stats

  printSubHdr("De-Initialize Motor Controller Interfaces");

  //
  // Enable power to the motor controllers via watchdog subprocessor gpio.
  //
  ++stats.testCnt;

  if( watchdog.cmdEnableMotorCtlrs(false) < 0 )
  {
    printTestResult(FailTag, "Motor controllers power disable.");
  }
  else
  {
    printTestResult(PassTag, "Disabled power to motor controllers.");
    ++stats.passCnt;
  }

  //
  // Close serial connection with motor controllers.
  //
  ++stats.testCnt;

  if( MotorCtlrComm.close() < 0 )
  {
    printTestResult(FailTag, "Close motor controllers serial connection.");
  }
  else
  {
    printTestResult(PassTag, "Close serial connection with motor controllers.");
    ++stats.passCnt;
  }

  return stats;
}

DiagStats runMotorsDiagnostics(LaeI2C &i2cBus, bool bTestMotion)
{
  LaeWd       watchdog(i2cBus);
  DiagStats   statsTest;
  DiagStats   statsTotal;

  printHdr("Motors Diagnostics");

  if( bTestMotion )
  {
    printf(" (Warning: motors and wheels will rotate)");
  }

  printf("\n\n");

  watchdog.sync();

  //
  // Init Tests
  //
  statsTest = initMotorInterfaces(watchdog);

  printSubTotals(statsTest);

  statsTotal += statsTest;

  //
  // Read and Verify Motor Controller State Tests
  //
  if( !statsTotal.fatal )
  {
    statsTest = testMotorCtlrsState();

    printSubTotals(statsTest);

    statsTotal += statsTest;
  }
 
  //
  // Read and Verify Motor State Tests
  //
  if( !statsTotal.fatal )
  {
    statsTest = testMotorsState();

    printSubTotals(statsTest);

    statsTotal += statsTest;
  }
 
  //
  // Motor Operation Tests
  //
  if( bTestMotion )
  {
    if( !statsTotal.fatal )
    {
      statsTest = testMotors(watchdog);

      printSubTotals(statsTest);

      statsTotal += statsTest;
    }
  }
 
  //
  // Deinit Tests
  //
  statsTest = deinitMotorInterfaces(watchdog);

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
