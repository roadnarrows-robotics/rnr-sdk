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
#include "Laelaps/laeImu.h"

#include "laelaps_diag.h"

using namespace std;
using namespace laelaps;
using namespace sensor::imu;
using namespace sensor::imu::msp;

static const char *SubSysName = "IMU";
static const char *ProdName   = "CleanFlight Naze32";

static DiagStats initImu(LaeImuCleanFlight &imu)
{
  const char *sTag;
  DiagStats   stats;

  printSubHdr("Initialize IMU");

  ++stats.testCnt;

  if( imu.open(LaeDevIMU, LaeBaudRateIMU) == LAE_OK )
  {
    sTag = PassTag;
    ++stats.passCnt;
  }
  else
  {
    sTag = FailTag;
    stats.fatal = true;
  }

  printTestResult(sTag, "Open IMU serial device %s.", LaeDevIMU);

  return stats;
}

static DiagStats readInfo(LaeImuCleanFlight &imu)
{
  string      strIdent;

  const char *sTag;
  DiagStats   stats;

  printSubHdr("Read IMU Information and Configuration");

  ++stats.testCnt;

  if( imu.readIdentity(strIdent) == LAE_OK )
  {
    sTag = PassTag;
    ++stats.passCnt;
  }
  else
  {
    sTag = FailTag;
    strIdent = imu.getIdentity();  // default
  }

  printTestResult(sTag, "Read IMU identity.", LaeDevIMU);

  printf("\n");
  printf("  %s\n", strIdent.c_str());
  printf("\n");

  return stats;
}

static DiagStats readImu(LaeImuCleanFlight &imu)
{
  double      accel[NumOfAxes];
  double      gyro[NumOfAxes];
  double      rpy[NumOfAxes];
  int         cnt;

  DiagStats   stats;

  printSubHdr("Read IMU Sensor Data");

  for(cnt = 0; cnt < 10; ++cnt)
  {
    ++stats.testCnt;
    if( imu.readRawImu() == LAE_OK )
    {
      ++stats.passCnt;
      printTestResult(PassTag, "Read raw IMU data");

      imu.convertRawToSI();

      imu.getInertiaData(accel, gyro);
      imu.getAttitude(rpy[ROLL], rpy[PITCH], rpy[YAW]);

      printf("Data:\n");
      printf("  accel[x,y,z] (m/s^2): %.4lf, %.4lf, %.4lf\n",
          accel[X], accel[Y], accel[Z]);
      printf("  gyro[x,y,z] (rad/s):  %.4lf, %.4lf, %.4lf\n",
          gyro[X], gyro[Y], gyro[Z]);
      printf("  roll,pitch,yaw (rad): %.4lf, %.4lf, %.4lf\n",
          rpy[ROLL], rpy[PITCH], rpy[YAW]);
    }
    else
    {
      printTestResult(FailTag, "Read raw IMU data.");
    }
  }

  return stats;
}

DiagStats runImuDiagnostics()
{
  LaeImuCleanFlight imu;
  DiagStats         statsTest;
  DiagStats         statsTotal;

  printHdr("Inertia Measurement Unit Diagnostics");

  //
  // Init Tests
  //
  statsTest = initImu(imu);

  printSubTotals(statsTest);

  statsTotal += statsTest;

  //
  // Read Information Tests
  //
  if( !statsTotal.fatal )
  {
    statsTest = readInfo(imu);

    printSubTotals(statsTest);

    statsTotal += statsTest;
  }
 
  //
  // Read Information Tests
  //
  if( !statsTotal.fatal )
  {
    statsTest = readImu(imu);

    printSubTotals(statsTest);

    statsTotal += statsTest;
  }
 
  //
  // Summary
  //
  printTotals(statsTotal);

  return statsTotal;
}
