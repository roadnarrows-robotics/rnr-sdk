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
 * $LastChangedDate: 2016-02-01 15:14:45 -0700 (Mon, 01 Feb 2016) $
 * $Rev: 4289 $
 *
 * \brief Perform Laelaps hardware and software interface diagnostics.
 *
 * This C++ version uses the liblaelaps library, so both it and the hardware
 * are tested.
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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <errno.h>

#include <string>
#include <vector>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/opts.h"
#include "rnr/color.h"
#include "rnr/pkg.h"

// common
#include "Laelaps/laelaps.h"
#include "Laelaps/laeUtils.h"

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

static char    *Argv0;                    ///< the command
static bool_t   OptsNoMotion = false;     ///< no motion option

/*! \brief Available diagnostics. */
static const char *DiagnosticsAvail[] =
{
  "product", "cpu", "motors", "watchdog", "tof", "imu", "cam"
};

static vector<string> DiagnosticsToRun;    ///< diagnostics to run

static LaeI2C     I2CBus;
static LaeI2CMux  I2CMux(I2CBus);

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
  // --no-motion
  {
    "no-motion",          // long_opt
    OPTS_NO_SHORT,        // short_opt
    no_argument,          // has_arg
    true,                 // has_default
    &OptsNoMotion,        // opt_addr
    OptsCvtArgBool,       // fn_cvt
    OptsFmtBool,          // fn_fmt
    NULL,                 // arg_name
                          // opt desc
    "Do not run diagnositcs that causes the Laelaps to move."
  },


  {NULL, }
};

#define COLOR_PASS  ANSI_CSI ANSI_SGR_FG_COLOR_GREEN 
#define COLOR_WARN  ANSI_CSI ANSI_SGR_FG_COLOR_YELLOW 
#define COLOR_FAIL  ANSI_CSI ANSI_SGR_FG_COLOR_RED 
#define COLOR_YN    ANSI_CSI ANSI_SGR_FG_COLOR_BLUE 
#define COLOR_FATAL ANSI_CSI ANSI_SGR_FG_COLOR_LIGHT_RED 
#define COLOR_OFF   ANSI_COLOR_RESET

static void setTags(bool bColor)
{
  if( bColor )
  {
    sprintf(PassTag,  "[" COLOR_PASS "PASS" COLOR_OFF "] ");
    sprintf(WarnTag,  "[" COLOR_WARN "WARN" COLOR_OFF "] ");
    sprintf(FailTag,  "[" COLOR_FAIL "FAIL" COLOR_OFF "] ");
    sprintf(WaitTag,  "       ");
    sprintf(YNTag,    "[" COLOR_YN "y/n" COLOR_OFF "]  ");
    sprintf(FatalTag, "[" COLOR_FATAL "FATAL" COLOR_OFF"] ");
  }
  else
  {
    sprintf(PassTag,  "[PASS] ");
    sprintf(WarnTag,  "[WARN] ");
    sprintf(FailTag,  "[FAIL] ");
    sprintf(WaitTag,  "       ");
    sprintf(YNTag,    "[y/n]  ");
    sprintf(FatalTag, "[FATAL]");
  }
}

static void prelims()
{
  bool        bUsesI2C = false;
  DiagStats   statsTotal;

  printHdr("Diagnostic Prelims");

  //
  // Laelaps diagnostics that use the shared I2C bus.
  //
  for(size_t i = 0; i < DiagnosticsToRun.size(); ++i)
  {
    if( (DiagnosticsToRun[i] == "tof") ||
        (DiagnosticsToRun[i] == "watchdog") ||
        (DiagnosticsToRun[i] == "motors") )
    {
      bUsesI2C = true;
      break;
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

  ++statsTotal.testCnt;
  printTestResult(PassTag, "Preliminaries completed.");
  ++statsTotal.passCnt;

  printf("\n");
  printTotals(statsTotal);
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

const int MaxTagSize = 32;

char PassTag[MaxTagSize];
char WarnTag[MaxTagSize];
char FailTag[MaxTagSize];
char WaitTag[MaxTagSize];
char YNTag[MaxTagSize];
char FatalTag[MaxTagSize];

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

  prelims();

  for(size_t i = 0; i < DiagnosticsToRun.size(); ++i)
  {
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
      statsGrandTotal += runMotorsDiagnostics(I2CBus, OptsNoMotion==false);
    }
    else if( strDiag == "cam" )
    {
      statsGrandTotal += runCamDiagnostics();
    }
    else if( strDiag == "tof" )
    {
      statsGrandTotal += runToFDiagnostics(I2CMux);
    }
    else if( strDiag == "imu" )
    {
      statsGrandTotal += runImuDiagnostics();
    }
    else if( strDiag == "watchdog" )
    {
      statsGrandTotal += runWatchDogDiagnostics(I2CBus);
    }
  }

  printGrandTotals(statsGrandTotal);

  return APP_EC_OK;
}

void printHdr(string strDiag)
{
  printf("%s\n", DiagSep);
  printf("%s\n", strDiag.c_str());
  printf("%s\n\n", DiagSep);
}

void printSubHdr(string strName)
{
  //printf("%s\n", SubHdrSep);
  printf("+ + %s + +\n", strName.c_str());
}

void printTestResult(const char *sTag, const char *sFmt, ...)
{
  va_list ap;

  va_start(ap, sFmt);
  printf("%s ", sTag);
  vprintf(sFmt, ap);
  printf("\n");
  va_end(ap);
  fflush(stdout);
}

void printSubTotals(DiagStats &stats)
{
  printf("%s %d/%d passed.\n\n", SubSumTag, stats.passCnt, stats.testCnt);
}

void printTotals(DiagStats &stats)
{
  printf("%s %d/%d diagnostics passed.\n\n",
      TotSumTag, stats.passCnt, stats.testCnt);
}

void printGrandTotals(DiagStats &stats)
{
  printf("%s\n", DiagSep);
  printf("%s\n", DiagSep);
  printf("Gran Total: %d/%d diagnostics passed.\n",
      stats.passCnt, stats.testCnt);
  printf("%s\n", DiagSep);
  printf("%s\n", DiagSep);
}

/*!
 * \}
 */
