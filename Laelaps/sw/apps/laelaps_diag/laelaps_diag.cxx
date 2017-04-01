////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Program:   laelaps_diag   
//
// File:      laelaps_diag.cxx
//
/*! \file
 *
 * \brief Perform Laelaps hardware and software interface diagnostics.
 *
 * This C++ version uses the liblaelaps library, so both it and the hardware
 * are tested.
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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

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
#include "Laelaps/laeXmlCfg.h"

// hardware
#include "Laelaps/laeSysDev.h"
#include "Laelaps/RoboClaw.h"
#include "Laelaps/laeMotor.h"
#include "Laelaps/laeI2C.h"
#include "Laelaps/laeI2CMux.h"

#include "version.h"

#include "laelaps_diag.h"

using namespace std;
using namespace laelaps;


//------------------------------------------------------------------------------
// Private Interface
//------------------------------------------------------------------------------

/*!
 * \ingroup apps
 * \defgroup laelaps_diag laelaps_diag
 * \{
 */

#define APP_EC_OK       0   ///< success exit code
#define APP_EC_ARGS     2   ///< command-line options/arguments error exit code
#define APP_EC_EXEC     4   ///< execution exit code

static char    *Argv0;                ///< the command
static bool_t   OptsMotion  = false;  ///< motion option
static bool_t   OptsAnyKey  = false;  ///< user presses anykey to stop option

/*! \brief Available diagnostics. */
static const char *DiagnosticsAvail[] =
{
  "product", "cpu", "motors", "watchdog", "tof", "imu", "cam", "batt"
};

static vector<string> DiagnosticsToRun;    ///< diagnostics to run

/*!
 * \brief Program information.
 */
static OptsPgmInfo_T PgmInfo =
{
  // usage_args
  "DIAG [DIAG ...]",

  // synopsis
  "Run Laelaps diagnostics.",

  // long_desc = 
  "The %P command executes Laelaps subsystem diagnostics. "
  "The Laelaps embedded software interfaces are used to test the subsystems."
  "\n\n"
  "DIAG: Diagnostic to run. One of:\n"
  "  all       - Run all dignostics.\n"
  "  batt      - Run battery dignostics.\n"
  "  cam       - Run camera diagnotics.\n"
  "  cpu       - Run main CPU diagnotics.\n"
  "  imu       - Run Inertia Measurement Unit diagnostics.\n"
  "  motors    - Run motors diagnotics.\n"
  "  product   - Run Laelaps product diagnotics.\n"
  "  tof       - Run time-of-flight sensors diagnostics.\n"
  "  watchdog  - Run Arduino watchdog sub-processor diagnostics.\n\n"
  "The ROS node laelaps_control cannot be running while using this tool.",

  // diagnostics
  NULL
};

/*!
 * \brief Command line options information.
 */
static OptsInfo_T OptsInfo[] =
{
  // --motion
  {
    "motion",             // long_opt
    OPTS_NO_SHORT,        // short_opt
    no_argument,          // has_arg
    true,                 // has_default
    &OptsMotion,          // opt_addr
    OptsCvtArgBool,       // fn_cvt
    OptsFmtBool,          // fn_fmt
    NULL,                 // arg_name
                          // opt desc
    "Run diagnositcs that causes the Laelaps to move."
  },

  // --anykey
  {
    "prompt",             // long_opt
    OPTS_NO_SHORT,        // short_opt
    no_argument,          // has_arg
    true,                 // has_default
    &OptsAnyKey,          // opt_addr
    OptsCvtArgBool,       // fn_cvt
    OptsFmtBool,          // fn_fmt
    NULL,                 // arg_name
                          // opt desc
    "For each diagnositc, the user presses any keyboard key to progress to"
    " the next diagnostic."
  },


  {NULL, }
};


static DiagStats prelims()
{
  bool        bUsesI2C = false;
  bool        bUsesWd  = false;
  DiagStats   statsTotal;
  LaeXmlCfg   xml;

  printHdr("Diagnostic Prelims");

  ++statsTotal.testCnt;

  //
  // Get robot description (and version)
  //
  if( xml.load(RobotDesc, LaeSysCfgPath, LaeEtcCfg) < 0 )
  {
    printTestResult(FatalTag, "Loading XML file '%s' failed.", LaeEtcCfg);
    statsTotal.fatal = true;
    return statsTotal;
  }
  else
  {
    RobotDesc.markAsDescribed();
    printTestResult(PassTag, "Robot v%s description from '%s' loaded.\n",
        RobotDesc.getProdHwVerString().c_str(), LaeEtcCfg);
    ++statsTotal.passCnt;
  }

  //
  // Laelaps diagnostics that use the shared I2C bus.
  //
  for(size_t i = 0; i < DiagnosticsToRun.size(); ++i)
  {
    if( (DiagnosticsToRun[i] == "watchdog") ||
        (DiagnosticsToRun[i] == "motors") ||
        (DiagnosticsToRun[i] == "batt") )
    {
      bUsesI2C  = true;
      bUsesWd   = true;
    }
    else if( (DiagnosticsToRun[i] == "tof") )
    {
      bUsesI2C  = true;
    }
  }

  if( bUsesI2C )
  {
    ++statsTotal.testCnt;
    if( I2CBus.open(LaeDevI2C) < 0 )
    {
      printTestResult(FailTag, "Open I2C bus %s failed: %s(%d).",
          LaeDevI2C, strerror(errno), errno);
    }
    else
    {
      printTestResult(PassTag, "I2C bus %s opened.", LaeDevI2C);
      ++statsTotal.passCnt;
    }
  }

  if( bUsesWd )
  {
    uint_t  uFwVer;

    ++statsTotal.testCnt;
    if( WatchDog.cmdGetFwVersion(uFwVer) < 0 )
    {
      printTestResult(FailTag, "WatchDog: Failed to get firmware version.");
    }
    else
    {
      WatchDog.sync();
      printTestResult(PassTag, "Connected to WatchDog sub-processor, fwver=%u.",
          uFwVer);
      ++statsTotal.passCnt;
    }
  }

  ++statsTotal.testCnt;
  printTestResult(PassTag, "Preliminaries completed.");
  ++statsTotal.passCnt;

  printf("\n");
  printTotals(statsTotal);

  return statsTotal;
}

/*!
 * \brief Main initialization.
 *
 * \param argc    Command-line argument count.
 * \param argv    Command-line argument list.
 *
 * \par Exits:
 * Program terminates on conversion error.
 */
static void mainInit(int argc, char *argv[])
{
  string  strArg;
  bool    bFound;

  // name of this process
  Argv0 = basename(argv[0]);

  // parse input options
  argv = OptsGet(Argv0, &PkgInfo, &PgmInfo, OptsInfo, true, &argc, argv);

  if( argc == 0 )
  {
    fprintf(stderr, "%s: Error: No diagnostics specified.\n", Argv0);
    fprintf(stderr, "Try '%s --help' for more information.\n", Argv0);
    exit(APP_EC_ARGS);
  }

  for(int i = 0;  i < argc; ++i)
  {

    strArg = argv[i];

    if( strArg == "all" )
    {
      DiagnosticsToRun.clear();
      for(size_t j = 0; j < arraysize(DiagnosticsAvail); ++j)
      {
        DiagnosticsToRun.push_back(DiagnosticsAvail[j]);
      }
      break;
    }

    bFound = false;

    for(size_t j = 0; j < arraysize(DiagnosticsAvail); ++j)
    {
      if( strArg == DiagnosticsAvail[j] )
      {
        DiagnosticsToRun.push_back(DiagnosticsAvail[j]);
        bFound = true;
        break;
      }
    }

    if( !bFound )
    {
      fprintf(stderr, "%s: Warning: Unknown diganostic %s - ignoring.\n",
            Argv0, strArg.c_str());
    }
  }

  if( DiagnosticsToRun.empty() )
  {
    fprintf(stderr, "%s: Error: No valid diagnostics to run.\n", Argv0);
    fprintf(stderr, "Try '%s --help' for more information.\n", Argv0);
    exit(APP_EC_ARGS);
  }
}


//------------------------------------------------------------------------------
// Public Interface
//------------------------------------------------------------------------------

//
// Shared interfaces used by diagnostics 
//
LaeDesc   RobotDesc;
LaeI2C    I2CBus;
LaeI2CMux I2CMux(I2CBus);
LaeWd     WatchDog(I2CBus);

/*!
 * \brief Main.
 *
 * \param argc    Command-line argument count.
 * \param argv    Command-line argument list.
 *
 * \return Returns 0 on succes, non-zero on failure.
 */
int main(int argc, char* argv[])
{
  string    strDiag;
  DiagStats statsGrandTotal;

  mainInit(argc, argv);

  setTags(LOG_IN_COLOR());

  statsGrandTotal = prelims();

  for(size_t i = 0; i < DiagnosticsToRun.size(); ++i)
  {
    if( statsGrandTotal.fatal )
    {
      break;
    }

    strDiag = DiagnosticsToRun[i];

    if( strDiag == "product" )
    {
      statsGrandTotal += runProductDiagnostics();
    }
    else if( strDiag == "cpu" )
    {
      statsGrandTotal += runCpuDiagnostics();
    }
    else if( strDiag == "motors" )
    {
      statsGrandTotal += runMotorsDiagnostics(OptsMotion);
    }
    else if( strDiag == "cam" )
    {
      statsGrandTotal += runCamDiagnostics();
    }
    else if( strDiag == "tof" )
    {
      statsGrandTotal += runToFDiagnostics();
    }
    else if( strDiag == "imu" )
    {
      statsGrandTotal += runImuDiagnostics();
    }
    else if( strDiag == "watchdog" )
    {
      statsGrandTotal += runWatchDogDiagnostics();
    }
    else if( strDiag == "batt" )
    {
      statsGrandTotal += runBatteryDiagnostics(OptsAnyKey);
    }
  }

  printGrandTotals(statsGrandTotal);

  return APP_EC_OK;
}

/*!
 * \}
 */
