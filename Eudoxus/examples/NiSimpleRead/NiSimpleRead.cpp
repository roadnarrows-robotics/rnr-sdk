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


using namespace std;

// ---------------------------------------------------------------------------
// ROADNARROWS Standard Interface Begin
// ---------------------------------------------------------------------------

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/opts.h"

#include "Eudoxus/Eudoxus.h"
#include "Eudoxus/euUtils.h"

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
using namespace eu;

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

  const char *fn = NULL;

  // .........................................................................
  // ROADNARROWS
  
  nRetVal = MainInit(argc, argv);

  CHECK_RC(nRetVal, "MainInit() failed.");

  string    strCfgFile;

  strCfgFile = getConfigFileName(OptsCfgFile);

  if( strCfgFile.empty() )
  {
    return XN_STATUS_NO_NODE_PRESENT;
  }

  // ROADNARROWS
  // .........................................................................
  
#ifdef CODE_WAS_THIS
  if  (fileExists(SAMPLE_XML_PATH)) fn = SAMPLE_XML_PATH;
  else if (fileExists(SAMPLE_XML_PATH_LOCAL)) fn = SAMPLE_XML_PATH_LOCAL;
  else {
    printf("Could not find '%s' nor '%s'. Aborting.\n" , SAMPLE_XML_PATH, SAMPLE_XML_PATH_LOCAL);
    return XN_STATUS_ERROR;
  }
#endif // CODE_WAS_THIS

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

#ifdef CODE_WAS_THIS
    printf("Frame %d Middle point is: %u. FPS: %f\n", depthMD.FrameID(), depthMD(depthMD.XRes() / 2, depthMD.YRes() / 2), xnFPSCalc(&xnFPS));
#else
    printf("Frame: %d %lf %u\n",
      depthMD.FrameID(),
      timer_elapsed(&tvMark),
      depthMD(depthMD.XRes() / 2, depthMD.YRes() / 2));
#endif // CODE_WAS_THIS
  }

  depth.Release();
  scriptNode.Release();
  context.Release();

  return 0;
}
