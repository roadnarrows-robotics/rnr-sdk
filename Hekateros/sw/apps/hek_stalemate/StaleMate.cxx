////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Program:   StaleMate
//
// File:      StaleMate.cxx
//
/*! \file
 *
 * $LastChangedDate: 2012-06-13 10:47:00 -0600 (Wed, 13 Jun 2012) $
 * $Rev: 2043 $
 *
 * \brief  Hekateros StaleMate chess playing application.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011.  RoadNarrows
 * (http://www.RoadNarrows.com)
 * \n All Rights Reserved
 */
// Unless otherwise noted, all materials contained are copyrighted and may not
// be used except as provided in these terms and conditions or in the copyright
// notice (documents and software ) or other proprietary notice provided with
// the relevant materials.
//
//
// IN NO EVENT SHALL THE AUTHOR, ROADNARROWS, OR ANY MEMBERS/EMPLOYEES/
// CONTRACTORS OF ROADNARROWS OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
// PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
// EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.
//
// THE AUTHORS AND  ROADNARROWS SPECIFICALLY DISCLAIM ANY WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
// "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
// PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
//
////////////////////////////////////////////////////////////////////////////////

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <libgen.h>
#include <stdarg.h>
#include <errno.h>

#include <cstring>
#include <iostream>
#include <fstream>
#include <cmath>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/opts.h"
#include "rnr/install.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "rnr/rnrWin.h"

#include "Hekateros/Hekateros.h"

#include "StaleMateTune.h"
#include "StaleMate.h"

#include "SMBotPlanner.h"

using namespace std;
using namespace rnrWin;

#include "version.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

//
// Exit Codes
//
#define APP_EC_OK       0                ///< success exit code
#define APP_EC_USAGE    2                ///< usage error exit code
#define APP_EC_EXEC     4                ///< execution error exit code

// 
// The command with option and argument values.
//
static char *Argv0;                               ///< the command
static char *OptsVidDev = (char *)"/dev/video0";  ///< video device
static char *OptsSerDev = (char *)"/dev/ttyUSB0"; ///< serial device
static int   OptsNoArm  = false;                  ///< not no arm
static int   OptsNoChessEngine = false;           ///< not no chess engine
static int   OptsNoOpenRave = false;              ///< not no open rave
static int   OptsChessBoardDim = 8;               ///< chess board dimension


/*!
 * \brief Program information.
 */
static OptsPgmInfo_T AppPgmInfo =
{
  // usage_args
  NULL,

  // synopsis
  "Hekateros StaleMate chess player",

  // long_desc = 
  "The %P command is a visual chess playing application that uses vision to "
  "determine the current game state, calculate the next chess move, and "
  "directs the Hekatero robotic manipulater to make the move.",

  // diagnostics
  NULL
};

/*!
 * \brief Command line options information.
 */
static OptsInfo_T AppOptsInfo[] =
{
  // -v, --video <device>
  {
    "video",              // long_opt
    'v',                  // short_opt
    required_argument,    // has_arg
    true,                 // has_default
    &OptsVidDev,          // opt_addr
    OptsCvtArgStr,        // fn_cvt
    OptsFmtStr,           // fn_fmt
    "<device>",           // arg_name
    "Video device.",      // opt desc
  },

  // -s, --serial <device>
  {
    "serial",             // long_opt
    's',                  // short_opt
    required_argument,    // has_arg
    true,                 // has_default
    &OptsSerDev,          // opt_addr
    OptsCvtArgStr,        // fn_cvt
    OptsFmtStr,           // fn_fmt
    "<device>",           // arg_name
    "Serial device.",     // opt desc
  },

  // --no-arm
  {
    "no-arm",             // long_opt
    OPTS_NO_SHORT,        // short_opt
    no_argument,          // has_arg
    true,                 // has_default
    &OptsNoArm,           // opt_addr
    OptsCvtArgBool,       // fn_cvt
    OptsFmtBool,          // fn_fmt
    NULL,                 // arg_name
    "Don't control the arm.", // opt desc
  },

  // --no-chess-engine
  {
    "no-chess-engine",    // long_opt
    OPTS_NO_SHORT,        // short_opt
    no_argument,          // has_arg
    true,                 // has_default
    &OptsNoChessEngine,   // opt_addr
    OptsCvtArgBool,       // fn_cvt
    OptsFmtBool,          // fn_fmt
    NULL,                 // arg_name
    "Don't interface to chess engine backend.", // opt desc
  },

  // --no-openrave
  {
    "no-openrave",        // long_opt
    OPTS_NO_SHORT,        // short_opt
    no_argument,          // has_arg
    true,                 // has_default
    &OptsNoOpenRave,      // opt_addr
    OptsCvtArgBool,       // fn_cvt
    OptsFmtBool,          // fn_fmt
    NULL,                 // arg_name
    "Don't interface to OpenRave simulator.", // opt desc
  },

  // --chess-board-dim
  {
    "chess-board-dim",    // long_opt
    OPTS_NO_SHORT,        // short_opt
    required_argument,    // has_arg
    true,                 // has_default
    &OptsChessBoardDim,   // opt_addr
    OptsCvtArgInt,        // fn_cvt
    OptsFmtInt,           // fn_fmt
    "<N>",                // arg_name
    "Chess board NxN dimension.", // opt desc
  },

  {NULL, }
};


/*!
 * \brief Show StaleMate main splash screen.
 */
static void SMMainSplash(StaleMateSession &session)
{
  char        imagePath[MAX_PATH];
  IplImage   *pImgSplash;

  snprintf(imagePath, sizeof(imagePath), "%s/%s",
      HekImgDir, "HekaterosSplashScreen.jpg");
  imagePath[sizeof(imagePath)-1] = 0;

  if( access(imagePath, F_OK|R_OK) != 0 )
  {
    LOGERROR("%s: %s(errno=%d)", imagePath, strerror(errno), errno);
    return;
  }

  pImgSplash = cvLoadImage(imagePath, CV_LOAD_IMAGE_COLOR);

  // set the window workspace as an opencv image
  session.m_gui.pWin->WorkspaceSetAsCvImage();

  // show
  session.m_gui.pWin->ShowCvImage(pImgSplash);
  session.m_gui.pWin->WaitKey(20);
}

static void SMMainUnSplash(StaleMateSession &session)
{
  session.m_gui.pWin->WorkspaceRemoveAll();
}

/*!
 * \brief Validate video device command line option.
 *
 * Validation error results in termination of program.
 *
 * \return Video device minor number.
 */
static uint_t SMMainVideoDevMinor(const char *sVidDev)
{
  struct stat statVid;
  uint_t      uMajor;
  uint_t      uMinor;

  if( access(sVidDev, F_OK|R_OK|W_OK) != 0 )
  {
    OptsInvalid(Argv0, "video device '%s': %s(errno=%d}.",
        sVidDev, strerror(errno), errno);
  }

  if( stat(sVidDev, &statVid) != 0 )
  {
    OptsInvalid(Argv0, "video device '%s': cannot stat: %s(errno=%d}.",
        sVidDev, strerror(errno), errno);
  }
  
  uMajor = major(statVid.st_rdev);
  uMinor = minor(statVid.st_rdev);

  if( uMajor != StaleMateVidDevMajor )
  {
    OptsInvalid(Argv0, "video device '%s': not a video device: major %d != %d",
        sVidDev, uMajor, StaleMateVidDevMajor);
  }

  return uMinor;
}

/*!
 * \brief Main command-line argument initialization.
 *
 * \param argc      Command-line argument count.
 * \param argv      Command-line argument list.
 * \param session  media pack demo session data.
 *
 * \par Exits:
 * Program terminates on conversion error.
 */
static void SMMainInitArgs(int argc, char *argv[], StaleMateSession &session)
{
  uint_t  uVidDevMinor;

  // name of this process
  Argv0 = basename(argv[0]);

  // parse input options
  argv = OptsGet(Argv0, &PkgInfo, &AppPgmInfo, AppOptsInfo, true,
                 &argc, argv);

  // validate existence and permissions of video device
  uVidDevMinor = SMMainVideoDevMinor(OptsVidDev);

  session.SetVideoDevice(OptsVidDev, uVidDevMinor);

  session.SetHekDevice(OptsSerDev, 1000000);

  if( (OptsChessBoardDim < 4) || (OptsChessBoardDim > 8) )
  {
    OptsInvalid(Argv0, "%d: Out-of-range 'chess-board-dim' argument [5,8].",
        OptsChessBoardDim);
  }
}

/*!
 * \brief Main initialization.
 *
 * \param argc     Command-line argument count.
 * \param argv     Command-line argument list.
 * \param session RNMP session data.
 *
 * \par Exits:
 * Program terminates on conversion error.
 */
static int SMMainInit(int argc, char *argv[], StaleMateSession &session)
{
  int   nBaudRate = 1000000;
  int   rc;

  // parse, validate, and save command-line arguments
  SMMainInitArgs(argc, argv, session);

  // create interface to chess engine backend
  if( !OptsNoChessEngine )
  {
    if( (rc = StaleMateUciOneTimeInit(session, "/opt/bin/gnuchess")) < 0 )
    {
      LOGERROR("Failed to create chess engine backend.");
      return rc;
    }
    session.m_game.bUseChessEngine = true;
  }
  else
  {
    session.m_game.bUseChessEngine = false;
  }

  // create interface to chess engine backend
  if( !OptsNoOpenRave )
  {
    if( (rc = StaleMateOrpOneTimeInit(session)) < 0 )
    {
      LOGERROR("Failed to create OpenRave python interface.");
      return rc;
    }
    session.m_hek.bUseOpenRave = true;
  }
  else
  {
    session.m_hek.bUseOpenRave = false;
  }

  // create the main window
  session.m_gui.pWin = new rnrWindow(StaleMateWinName,
                                  StaleMateWinWidth,
                                  StaleMateWinHeight,
                                  StaleMateMaxButtonsPerMenu,
                                  true);

  // show splash screen
  SMMainSplash(session);

  // do [not] use hekateros robotic arm interface
  if( !OptsNoArm )
  {
    session.m_hek.bUseArm = true;
  }
  else
  {
    session.m_hek.bUseArm = false;
  }

  session.m_game.nChessBoardDim = OptsChessBoardDim;

  session.m_gui.pWin->Wait(1000); 

  SMMainUnSplash(session);

  return HEK_OK;
}

static void SMMainCleanup(StaleMateSession &session)
{
  StaleMateUciKill(session);
  StaleMateOrpKill(session);
}


// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief StaleMate main.
 *
 * \param argc  Command-line argument count.
 * \param argv  Command-line argument list.
 *
 * \par Exit Status:
 * Program exits with 0 success, \h_gt 0 on failure.
 */
int main(int argc, char *argv[])
{
  StaleMateSession  session;    // the workflow session data
  int               rc;         // return code

  rc = SMMainInit(argc, argv, session);

  if( rc == HEK_OK )
  {
    rc = StaleMateMasterControl(session);
  }

  SMMainCleanup(session);

  return rc < 0? APP_EC_EXEC: APP_EC_OK;
}
