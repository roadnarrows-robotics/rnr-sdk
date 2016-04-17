/****************************************************************************
*                                                                           *
*  OpenNI 1.x Alpha                                                         *
*  Copyright (C) 2011 PrimeSense Ltd.                                       *
*                                                                           *
*  This file is part of OpenNI.                                             *
*                                                                           *
*  OpenNI is free software: you can redistribute it and/or modify           *
*  it under the terms of the GNU Lesser General Public License as published *
*  by the Free Software Foundation, either version 3 of the License, or     *
*  (at your option) any later version.                                      *
*                                                                           *
*  OpenNI is distributed in the hope that it will be useful,                *
*  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
*  GNU Lesser General Public License for more details.                      *
*                                                                           *
*  You should have received a copy of the GNU Lesser General Public License *
*  along with OpenNI. If not, see <http://www.gnu.org/licenses/>.           *
*                                                                           *
****************************************************************************/
//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------

#include <sys/time.h>
#include <string>

#include "XnOpenNI.h"
#include "XnLog.h"
#include "XnCppWrapper.h"
#include "XnFPSCalculator.h"

#include "ofsensor.h"


using namespace std;

// ---------------------------------------------------------------------------
// ROADNARROWS Standard Interface Begin
// ---------------------------------------------------------------------------

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/opts.h"
#include "rnr/install.h"

#include "Eudoxus/Eudoxus.h"

#include "version.h"

//
// Exit Codes
//
#define APP_EC_OK       0                ///< success exit code
#define APP_EC_USAGE    2                ///< usage error exit code
#define APP_EC_EXEC     4                ///< execution error exit code

// 
// The command with option and argument values.
//
static char *Argv0;                   ///< the command
static char *OptsCfgFile  = NULL;     ///< openni configuration xml file name
static char *OptsModFile  = NULL;     ///< modules device


/*!
 * \brief Program information.
 */
static OptsPgmInfo_T AppPgmInfo =
{
  // usage_args
  NULL,

  // synopsis
  "Simple center distance measurement reader",

  // long_desc = 
  "The %P command ...",

  // diagnostics
  NULL
};

/*!
 * \brief Command line options information.
 */
static OptsInfo_T AppOptsInfo[] =
{
  // -c, --ni-config <file>
  {
    "ni-config",          // long_opt
    'c',                  // short_opt
    required_argument,    // has_arg
    false,                 // has_default
    &OptsCfgFile,         // opt_addr
    OptsCvtArgStr,        // fn_cvt
    OptsFmtStr,           // fn_fmt
    "<file>",             // arg_name
    "OpenNI configuration file.",      // opt desc
  },

  {NULL, }
};

/*!
 * \brief Main command-line argument initialization.
 *
 * \param argc      Command-line argument count.
 * \param argv      Command-line argument list.
 *
 * \par Exits:
 * Program terminates on conversion error.
 */
static void MainInitArgs(int argc, char *argv[])
{
  // name of this process
  Argv0 = basename(argv[0]);

  // parse input options
  argv = OptsGet(Argv0, &PkgInfo, &AppPgmInfo, AppOptsInfo, true, &argc, argv);

  // other option checks here
}


/*!
 * \brief Main initialization.
 *
 * \param argc     Command-line argument count.
 * \param argv     Command-line argument list.
 *
 * \par Exits:
 * Program terminates on conversion error.
 */
static XnStatus MainInit(int argc, char *argv[])
{
  // parse, validate, and save command-line arguments
  MainInitArgs(argc, argv);

  // other initialization here
  
  return XN_STATUS_OK;
}

/*! 
 * \brief Mark the current time. Resolution is microseconds.
 *
 * \param pTvMark   Pointer to timeval structure to be populated with
 *                  the current system seconds and useconds.
 */
static inline void timer_mark(struct timeval *pTvMark)
{
  if( gettimeofday(pTvMark, NULL) != OK )
  {
    LOGSYSERROR("gettimeofday()");
    timerclear(pTvMark);
  }
}

/*! 
 * \brief Calculate the elapsed time between the given time mark and this call.
 *
 * \param pTvMark   Pointer to timeval holding time mark.
 *
 * \return 
 * Time elapsed.
 */
static double timer_elapsed(struct timeval *pTvMark)
{
  struct timeval  tvEnd, tvDelta;

  timer_mark(&tvEnd);

  if( !timerisset(pTvMark) || !timerisset(&tvEnd) )
  {
    return UINT_MAX;
  }

  tvDelta.tv_sec = tvEnd.tv_sec - pTvMark->tv_sec;
  if( tvEnd.tv_usec < pTvMark->tv_usec )
  {
    tvDelta.tv_sec--;
    tvEnd.tv_usec += 1000000;
  }
  tvDelta.tv_usec = tvEnd.tv_usec - pTvMark->tv_usec;

  return (double)tvDelta.tv_sec + (double)tvDelta.tv_usec/1000000.0;
}

// ---------------------------------------------------------------------------
// ROADNARROWS Standard Interface End
// ---------------------------------------------------------------------------



//---------------------------------------------------------------------------
// Defines
//---------------------------------------------------------------------------
#define SAMPLE_XML_PATH "../../../../Data/SamplesConfig.xml"
#define SAMPLE_XML_PATH_LOCAL "SamplesConfig.xml"

//---------------------------------------------------------------------------
// Macros
//---------------------------------------------------------------------------
#define CHECK_RC(rc, what)                      \
  if (rc != XN_STATUS_OK)                      \
  {                                \
    printf("%s failed: %s\n", what, xnGetStatusString(rc));    \
    return rc;                          \
  }

//---------------------------------------------------------------------------
// Code
//---------------------------------------------------------------------------

using namespace xn;

XnBool fileExists(const char *fn)
{
  XnBool exists;
  xnOSDoesFileExist(fn, &exists);
  return exists;
}

int main(int argc, char *argv[])
{
  XnStatus  nRetVal = XN_STATUS_OK;

  Context context;
  ScriptNode scriptNode;
  EnumerationErrors errors;
  struct timeval    tvMark;
  OFlow             flow;
  double            fSum_of;
  double            fSum_ps;
  int               x_0_ps;
  int               x_1_ps;
  double            dx_ps;
  double            dxdt_ps;
  int               dxdt_of, dydt_of;
  double            dx_of;
  double            dt;
  double            t;
  int               count;
  double            fAvg_of;
  double            fAvg_ps;
  int               rc;

  const char *fn = NULL;

  // .........................................................................
  // ROADNARROWS
  
  nRetVal = MainInit(argc, argv);

  CHECK_RC(nRetVal, "MainInit() failed.");

  string    strCfgFile;

  strCfgFile = eu::getConfigFileName(OptsCfgFile);

  if( strCfgFile.empty() )
  {
    return XN_STATUS_NO_NODE_PRESENT;
  }

  // ROADNARROWS
  // .........................................................................
  
  fn = strCfgFile.c_str();

  printf("Reading config from: '%s'\n", fn);
  nRetVal = context.InitFromXmlFile(fn, scriptNode, &errors);

  if (nRetVal == XN_STATUS_NO_NODE_PRESENT)
  {
    XnChar strError[1024];
    errors.ToString(strError, 1024);
    printf("%s\n", strError);
    return (nRetVal);
  }
  else if (nRetVal != XN_STATUS_OK)
  {
    printf("Open failed: %s\n", xnGetStatusString(nRetVal));
    return (nRetVal);
  }

  DepthGenerator depth;
  nRetVal = context.FindExistingNode(XN_NODE_TYPE_DEPTH, depth);
  CHECK_RC(nRetVal, "Find depth generator");

  XnFPSData xnFPS;
  nRetVal = xnFPSInit(&xnFPS, 180);
  CHECK_RC(nRetVal, "FPS Init");

  DepthMetaData depthMD;

  if( (rc = flow.OpenOF()) < 0 )
  {
    LOGERROR("Failed to open Optical Flow device.");
    return APP_EC_EXEC;
  }

  fSum_of   = 0.0;
  fSum_ps   = 0.0;
  dxdt_of   = 0;
  dydt_of   = 0;
  dx_of     = 0.0;
  x_0_ps    = 0;    
  x_1_ps    = 0;    
  dx_ps     = 0.0;
  dxdt_ps   = 0.0;
  dt        = 0.0;
  t         = 0.0;
  count     = 0;
  fAvg_of   = 0.0;
  fAvg_ps   = 0.0;

  fprintf(stderr,
    "\n\n-------------------------------------------------------------\n");
  fprintf(stderr, "Start moving to initiate test...\n");
  
  while( (dxdt_of == 0) &&  !xnOSWasKeyboardHit() )
  {
    flow.ReadOF(dxdt_of, dydt_of);
  }

  fprintf(stderr, "\n\n     t     dt   x_ps  dx_of   v_ps   v_of\n"
                      "------ ------ ------ ------ ------ ------\n");

  while (!xnOSWasKeyboardHit())
  {
    timer_mark(&tvMark);

    nRetVal = context.WaitOneUpdateAll(depth);
    if (nRetVal != XN_STATUS_OK)
    {
      printf("UpdateData failed: %s\n", xnGetStatusString(nRetVal));
      continue;
    }

    xnFPSMarkFrame(&xnFPS);

    depth.GetMetaData(depthMD);
    const XnDepthPixel* pDepthMap = depthMD.Data();

    x_0_ps = (int)(depthMD(depthMD.XRes() / 2, depthMD.YRes() / 2));

    flow.ReadOF(dxdt_of, dydt_of);

    dt = timer_elapsed(&tvMark);

    t += dt;

    fSum_of += (double)dxdt_of;
    dx_of += ((double)dxdt_of * dt);

    if( dt > 0.0 )
    {
      if( x_1_ps != 0 )
      {
        dx_ps += (double)(x_0_ps - x_1_ps);
        dxdt_ps = (double)(x_0_ps - x_1_ps) / dt;
      }
      else
      {
        dxdt_ps = 0.0;
      }
    }
    else
    {
      dxdt_ps = 0.0;
    }
    x_1_ps = x_0_ps;

    fSum_ps += dxdt_ps;

    printf("%6.4lf %6.4lf %6d %6.4lf %6.1lf %6d\n",
             t,     dt,   x_0_ps, dx_of, dxdt_ps, dxdt_of);

    ++count;
  }

  if( t > 0.0 )
  {
    fAvg_of = dx_of / t;
    fAvg_ps = dx_ps / t;
  }

  printf("\nt=%lf, avg_of=%lf, avg_ps=%lf\n\n", t, fAvg_of, fAvg_ps);

  depth.Release();
  scriptNode.Release();
  context.Release();

  flow.CloseOF();

  return 0;
}
