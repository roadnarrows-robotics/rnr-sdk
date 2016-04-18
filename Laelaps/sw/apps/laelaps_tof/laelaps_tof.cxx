////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Program:   laelaps_tof   
//
// File:      laelaps_tof.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-01-15 12:45:22 -0700 (Thu, 15 Jan 2015) $
 * $Rev: 3857 $
 *
 * \brief Test and calibrate Laelaps Time-of-Flight sensors.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2015-2016  RoadNarrows
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

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/opts.h"
#include "rnr/pkg.h"

#include "Laelaps/laelaps.h"
#include "Laelaps/laeUtils.h"
#include "Laelaps/laeSysDev.h"
#include "Laelaps/laeI2C.h"
#include "Laelaps/laeI2CMux.h"
#include "Laelaps/laeVL6180.h"

#include "version.h"

using namespace std;
using namespace laelaps;
using namespace sensor::vl6180;

/*!
 * \ingroup apps
 * \defgroup laelaps_tof laelaps_tof
 * \{
 */

#define APP_EC_OK       0   ///< success exit code
#define APP_EC_ARGS     2   ///< command-line options/arguments error exit code
#define APP_EC_EXEC     4   ///< execution exit code

static char  *Argv0;                    ///< the command
static bool_t OptsNoAuto      = false;  ///< don't auto-run
static int    OptsTimePeriod  = 500;    ///< sense time period (msec)
static char  *OptsMeasure     = (char *)"range"; ///<  measure range
static bool_t OptsCalibOffset = false;  ///< do [not] calibrate range offset
static bool_t OptsCalibXTalk  = false;  ///< do [not] calibrate range crosstalk

static const char *ToFName[] =
{
  "front", "left_front", "left", "left_rear",
  "rear", "right_rear", "right", "right_front"
};

static const double ToFDir[] =
{
  degToRad(0.0), degToRad(15.0), degToRad(90.0), degToRad(165.0),
  degToRad(180.0), degToRad(195.0), degToRad(270.0), degToRad(345.0)
};

enum MeasType
{
  MeasTypeDistance,
  MeasTypeAmbient,
  MeasTypeBoth
};

static int            SensorId;
static int            Measure;
static LaeI2C         I2CBus;
static LaeI2CMux      I2CMux(I2CBus);
static LaeVL6180Mux  *ToFSensor;

static const char *Sep = \
"----------------------------------------------------------------------------";

//
// Forward declarations.
//
static int OptsCvtArgMeas(const char *argv0, const char *sOptName,
                              char *optarg, void *pOptVal);

/*!
 * \brief Program information.
 */
static OptsPgmInfo_T PgmInfo =
{
  // usage_args
  "{<Id> | <Name>}",

  // synopsis
  "Test and calibrate Laelaps Time-of-Flight sensors.",

  // long_desc = 
  "The %P command tests and calibrates the Laelaps Time-of-Flight sensors. "
  "The standard Laelaps comes with 3 forward facing ToFs, with an additional "
  "5 sensors available as an upgrade option. Sensors are identified by "
  "the number Id number of symbolic Name.\n"
  "  Id  Name         Direction    Package\n"
  "  --  -----------  ---------    -------\n"
  "  0   front          0 degrees  std\n"
  "  1   left_front    15 degrees  std\n"
  "  2   left          90 degrees  opt\n"
  "  3   left_rear    165 degrees  opt\n"
  "  4   rear         180 degrees  opt\n"
  "  5   right_rear   195 degrees  opt\n"
  "  6   right        270 degrees  opt\n"
  "  7   right_front  345 degrees  std\n\n"
  "The ROS node laelaps_control cannot be running while using this tool.",

  // diagnostics
  NULL
};

/*!
 * \brief Command line options information.
 */
static OptsInfo_T OptsInfo[] =
{
  // --calib-crosstalk
  {
    "calib-crosstalk",    // long_opt
    OPTS_NO_SHORT,        // short_opt
    no_argument,          // has_arg
    true,                 // has_default
    &OptsCalibXTalk,      // opt_addr
    OptsCvtArgBool,       // fn_cvt
    OptsFmtBool,          // fn_fmt
    NULL,                 // arg_name
                          // opt desc
    "Perform range sensor cross-talk compensation calibration "
    "procedures. After calibration, %P measurements automatically begin."
  },

  // --calib-offset
  {
    "calib-offset",       // long_opt
    OPTS_NO_SHORT,        // short_opt
    no_argument,          // has_arg
    true,                 // has_default
    &OptsCalibOffset,     // opt_addr
    OptsCvtArgBool,       // fn_cvt
    OptsFmtBool,          // fn_fmt
    NULL,                 // arg_name
                          // opt desc
    "Perform range sensor part-to-part offset calibration "
    "procedures. After calibration, %P measurements automatically begin."
  },

  // -m, --measure
  {
    "measure",            // long_opt
    'm',                  // short_opt
    required_argument,    // has_arg
    true,                 // has_default
    &OptsMeasure,         // opt_addr
    OptsCvtArgMeas,       // fn_cvt
    OptsFmtStr,           // fn_fmt
    NULL,                 // arg_name
                          // opt desc
    "Sensor measurement(s). One of:\n  range ambient both."
  },

  // --no-auto
  {
    "no-auto",            // long_opt
    OPTS_NO_SHORT,        // short_opt
    no_argument,          // has_arg
    true,                 // has_default
    &OptsNoAuto,          // opt_addr
    OptsCvtArgBool,       // fn_cvt
    OptsFmtBool,          // fn_fmt
    NULL,                 // arg_name
                          // opt desc
    "Don't auto-sense at the given millisecond time period. In this mode the "
    "user must provide keyboard input to control when to sense."
  },

  // -t, --time-period
  {
    "time-period",        // long_opt
    't',                  // short_opt
    required_argument,    // has_arg
    true,                 // has_default
    &OptsTimePeriod,      // opt_addr
    OptsCvtArgInt,        // fn_cvt
    OptsFmtInt,           // fn_fmt
    NULL,                 // arg_name
                          // opt desc
    "Auto-sense at the given millisecond time period rate."
  },

  {NULL, }
};

static int OptsCvtArgMeas(const char *argv0, const char *sOptName,
                              char *optarg, void *pOptVal)
{
  // pOptVal is OptsMeasure
  OptsCvtArgStr(argv0, sOptName, optarg, pOptVal);

  if( !strcmp(OptsMeasure, "range") )
  {
    Measure = MeasTypeDistance;
  }
  else if( !strcmp(OptsMeasure, "ambient") )
  {
    Measure = MeasTypeAmbient;
  }
  else if( !strcmp(OptsMeasure, "both") )
  {
    Measure = MeasTypeBoth;
  }
  else
  {
    OptsInvalid(Argv0, "'%s': Invalid '%s' measurement value.",
        optarg, sOptName);
  }

  return OK;
}

static int tofNameToInt(const string &str, int &val)
{
  size_t  i;

  for(i = 0; i < arraysize(ToFName); ++i)
  {
    if( !strcmp(ToFName[i], str.c_str()) )
    {
      val = (int)i;
      return OK;
    }
  }

  return RC_ERROR;
}

static int strToInt(const string &str, int &val)
{
  long long int val1; // must use 64-bit for arm 32-bit compilers

  if( sscanf(str.c_str(), "%lli", &val1) != 1 )
  {
    return RC_ERROR;
  }

  val = (int)val1;

  return OK;
}

int getch()
{
  struct termios old = {0};
  char           c = 0;

  if( tcgetattr(0, &old) < 0 )
  {
    return EOF;
  }

  old.c_lflag &= ~ICANON;
  old.c_lflag &= ~ECHO;
  old.c_cc[VMIN] = 1;
  old.c_cc[VTIME] = 0;

  if( tcsetattr(0, TCSANOW, &old) < 0 )
  {
    return EOF;
  }

  if( read(0, &c, 1) < 0 )
  {
    return EOF;
  }

  old.c_lflag |= ICANON;
  old.c_lflag |= ECHO;

  if( tcsetattr(0, TCSADRAIN, &old) < 0 )
  {
  }

  return c;
}

int kbhit()
{
  int   c;

  while( (c = getch()) == 0 );

  return c;
}

static int connectSensor(int n)
{
  //
  // Laelaps peripherals I2C bus.
  //
  if( I2CBus.open(LaeDevI2C) < 0 )
  {
    LOGSYSERROR("%s.", LaeDevI2C);
    return -LAE_ECODE_NO_DEV;
  }

  //
  // Time of Flight proximity sensors.
  //
  ToFSensor = new LaeVL6180Mux(I2CMux, SensorId, ToFDir[SensorId], 0.0, 
                                  ToFName[SensorId]);

  if( ToFSensor->initSensor() < 0 )
  {
    LOGERROR("Sensor %d %s: Failed to initialize.",
        SensorId, ToFName[SensorId]);
    return -LAE_ECODE_NO_EXEC;
  }

  return LAE_OK;
}

static int readSensorInfo()
{
  struct VL6180xIdentification id;

  if( ToFSensor->readId(id) < 0 )
  {
    LOGERROR("Failed to read sensor identification.");
    return -LAE_ECODE_IO;
  }

  printf("%s\n", Sep);
  printf("VL6180 Sensor Info:\n");
  printf("  Name:      %s\n", ToFSensor->getNameId().c_str());
  printf("  Desc:      %s\n", ToFSensor->getDesc().c_str());
  printf("  Model:     0x%02x v%u.%u\n",
      id.idModel, id.idModelRevMajor, id.idModelRevMinor);
  printf("  Module:    v%u.%u\n", id.idModuleRevMajor, id.idModuleRevMinor);
  printf("  Date/Time: %u/%u\n", id.idDate, id.idTime);

  return LAE_OK;
}

static int calibrateSensorCrossTalk()
{
  int     nOffsetPre;
  double  fAvgPre;
  int     nOffsetPost;
  double  fAvgPost;
  int     nCrossTalk;
  int     c;
  int     rc;

  printf(
"\n"
"Range sensor cross-talk compensation calibration.\n" 
"  Note: Perform range part-to-part offset calibration, "
"if required, first.\n\n"); 

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Cross-talk compensation calibration
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  printf("%s\n", Sep);
  printf("Calibrate Cross-Talk Compensation\n\n");

  printf("Setup: Place a black target (<= 3%% reflectance) at a distance\n");
  printf("       of 100mm from sensor (or front of glass cover if present).\n");

  printf("\nPress any key to when ready ('q' to quit): ");
  fflush(stdout);
  c = kbhit();
  if( (c == 'q') || (c == EOF) )
  {
    return -LAE_ECODE_NO_EXEC;
  }

  printf("\n  Calibrating...\r");
  fflush(stdout);

  rc = ToFSensor->calibCrossTalk(nCrossTalk);

  if( rc < 0 )
  {
    printf("Error: Failed to calibrate cross-talk compensation.\n");
    return -LAE_ECODE_NO_EXEC;
  }

  printf("Cross-Talk Calibration Summary:\n");
  printf("               Cross-Talk\n");
  printf("  Post-Calib:   %3d\n", nCrossTalk);
  printf("  Note: Cross-talk compensation  is stored in volatile register\n"
         "        SYSRANGE_CROSSTALK_COMPENSATION_RATE (0x%04x)\n"
         "        Tune parameter <tof_crosstalk>\n",
          VL6180X_SYSRANGE_CROSSTALK_COMPENSATION_RATE);

  printf("\n");

  return LAE_OK;
}

static int calibrateSensorOffset()
{
  int     nOffsetPre;
  double  fAvgPre;
  int     nOffsetPost;
  double  fAvgPost;
  int     nCrossTalk;
  int     c;
  int     rc;

  printf(
"\n"
"Range sensor part-to-part offset calibration.\n\n" );

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Offset calibration
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  printf("%s\n", Sep);
  printf("Calibrate Part-To-Part Offset\n\n");

  printf("Setup: Place a white target (>= 88%% reflectance) at a distance\n");
  printf("       of 50mm from sensor (or front of glass cover if present).\n");

  printf("\nPress any key to when ready ('q' to quit): ");
  fflush(stdout);
  c = kbhit();
  if( (c == 'q') || (c == EOF) )
  {
    return -LAE_ECODE_NO_EXEC;
  }

  printf("\n  Calibrating...\r");
  fflush(stdout);

  rc = ToFSensor->calibOffset(nOffsetPre, fAvgPre, nOffsetPost, fAvgPost);

  if( rc < 0 )
  {
    printf("Error: Failed to calibrate offset.\n");
    return -LAE_ECODE_NO_EXEC;
  }

  printf("Offset Calibration Summary:\n");
  printf("               Offset  Average\n");
  printf("  Pre-Calib:   %3d     %.3lf\n", nOffsetPre, fAvgPre);
  printf("  Post-Calib:  %3d     %.3lf\n", nOffsetPost, fAvgPost);
  printf("  Note: Offset is stored in volatile register\n"
         "        SYSRANGE_PART_TO_PART_RANGE_OFFSET (0x%04x)\n"
         "        Tune parameter <tof_offset>\n",
          VL6180X_SYSRANGE_PART_TO_PART_RANGE_OFFSET);
  printf("\n");

  return LAE_OK;
}

static void readSensorCalibTuneRegs()
{
  byte_t regRangeOffset;
  byte_t regRangeCrossTalk;
  byte_t regAlsGain;
  u16_t  regAlsIntPeriod;

  ToFSensor->readShadowRegs(regRangeOffset, regRangeCrossTalk,
                            regAlsGain, regAlsIntPeriod);
  printf("%s\n", Sep);
  printf("VL6180 Sensor Calibration/Tuning Register Values:\n");
  printf("  Range Offset:           %u\n", regRangeOffset);
  printf("  Range Cross Talk:       %u\n", regRangeCrossTalk);
  printf("  ALS Analog Gain:        %u\n", regAlsGain);
  printf("  ALS Integration Period: %u\n", regAlsIntPeriod);
}

static void printHdr(int what)
{
  switch( what )
  {
    case MeasTypeAmbient:
      printf("             lux\n");
      break;
    case MeasTypeBoth:
      printf("          meters     lux\n");
      break;
    case MeasTypeDistance:
    default:
      printf("          meters\n");
      break;
  }
}

static void printMeasurement(int seqnum, int what, double fMeters, double fFlux)
{
  printf("%7d  ", seqnum);

  // distance
  if( (what == MeasTypeDistance) || (what == MeasTypeBoth) )
  {
    if( fMeters > VL6180X_RANGE_MAX )
    {
      printf(" no_obj ");
    }
    else if( fMeters == VL6180X_ERR_MEAS )
    {
      printf(" error  ");
    }
    else
    {
      printf(" %6.3lf  ", fMeters);
    }
  }

  if( (what == MeasTypeAmbient) || (what == MeasTypeBoth) )
  {
    if( fFlux == VL6180X_ERR_MEAS )
    {
      printf(" error  ");
    }
    else
    {
      printf(" %6.1lf  ", fMeters);
    }
  }

  printf("\r");
  fflush(stdout);
}

static int measure()
{
  int     seqnum;
  double  fMeters = 0.0, fLux = 0.0;
  int     c;

  printf("%s\n", Sep);

  if( OptsNoAuto )
  {
    printf("Press any key to start measurements ('q' to quit): ");
    fflush(stdout);
    if( kbhit() == EOF )
    {
      return APP_EC_OK;
    }
    printf("\n\n");
  }
  else
  {
    //printf("Press 'q' to quit.\n\n");
  }

  printHdr(Measure);

  for(seqnum=0; ; ++seqnum)
  {
    switch( Measure )
    {
      case MeasTypeAmbient:
        fLux = ToFSensor->measureAmbientLight();
        printMeasurement(seqnum, Measure, 0.0, fLux);
        break;
      case MeasTypeBoth:
        fMeters = ToFSensor->measureRange();
        fLux    = ToFSensor->measureAmbientLight();
        printMeasurement(seqnum, Measure, fMeters, fLux);
        break;
      case MeasTypeDistance:
      default:
        fMeters = ToFSensor->measureRange();
        printMeasurement(seqnum, Measure, fMeters, 0.0);
        break;
    }

    if( OptsNoAuto )
    {
      c = kbhit();
      if( (c == 'q') || (c == EOF) )
      {
        break;
      }
    }
    //else if( getch() == 'q' )
    //{
    //  break;
    //}
    else
    {
      usleep(1000*OptsTimePeriod);
    }
  }

  return LAE_OK;
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
  // name of this process
  Argv0 = basename(argv[0]);

  // parse input options
  argv = OptsGet(Argv0, &PkgInfo, &PgmInfo, OptsInfo, true, &argc, argv);

  if( argc == 0 )
  {
    fprintf(stderr, "%s: No sensor number <N> specified.\n", Argv0);
    fprintf(stderr, "Try '%s --help' for more information.\n", Argv0);
    exit(APP_EC_ARGS);
  }
  else if( strToInt(argv[0], SensorId) < 0 )
  {
    if( tofNameToInt(argv[0], SensorId) < 0 )
    {
      fprintf(stderr, "%s: '%s': Bad sensor Id/Name.\n", Argv0, argv[0]);
      fprintf(stderr, "Try '%s --help' for more information.\n", Argv0);
      exit(APP_EC_ARGS);
    }
  }
  else if( (SensorId < ToFSensor0Chan) || (SensorId > ToFSensor7Chan) )
  {
    fprintf(stderr, "%s: %d: Sensor number out of range.\n", Argv0, SensorId);
    fprintf(stderr, "Try '%s --help' for more information.\n", Argv0);
    exit(APP_EC_ARGS);
  }
}

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
  int   rc;   // return code

  mainInit(argc, argv);

  if( connectSensor(SensorId) < 0 )
  {
    return APP_EC_EXEC;
  }

  readSensorInfo();

  if( OptsCalibOffset )
  {
    calibrateSensorOffset();
  }

  if( OptsCalibXTalk )
  {
    calibrateSensorCrossTalk();
  }

  readSensorCalibTuneRegs();

  rc = measure();

  I2CBus.close();

  return rc == LAE_OK? APP_EC_OK: APP_EC_EXEC;
}

/*!
 * \}
 */
