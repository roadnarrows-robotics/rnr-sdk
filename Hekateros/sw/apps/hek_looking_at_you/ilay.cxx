////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Program:   ImLookingAtYou
//
// File:      Ilay.cxx
//
/*! \file
 *
 * $LastChangedDate: 2011-07-20 11:17:03 -0600 (Wed, 20 Jul 2011) $
 * $Rev: 1145 $
 *
 * \brief  Hekateros fing and follow the face application.
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

#include "ilay.h"
//#include "ilayTune.h"

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
static char *OptsDevice = (char *)"/dev/video0";  ///< video device


/*!
 * \brief Program information.
 */
static OptsPgmInfo_T AppPgmInfo =
{
  // usage_args
  NULL,

  // synopsis
  "Hekateros I'm Looking At You",

  // long_desc = 
  "The %P command performs face recognition and directs the Hekateros "
  "robotic manipulator to locate and track the face.",

  // diagnostics
  NULL
};

/*!
 * \brief Command line options information.
 */
static OptsInfo_T AppOptsInfo[] =
{
  // -d, --device <device>
  {
    "device",             // long_opt
    'd',                  // short_opt
    required_argument,    // has_arg
    true,                 // has_default
    &OptsDevice,          // opt_addr
    OptsCvtArgStr,        // fn_cvt
    OptsFmtStr,           // fn_fmt
    "<device>",           // arg_name
    "Video device.",      // opt desc
  },

  {NULL, }
};


/*!
 * \brief Show ILAY main splash screen.
 */
static void IlayMainSplash(IlaySession &session)
{
  char        iconPath[MAX_PATH];
  IplImage   *pImgSplash;

  snprintf(iconPath, sizeof(iconPath), "%s/%s",
      HekIconDir, "HekateroSplashScreen.jpg");
  iconPath[sizeof(iconPath)-1] = 0;

  if( access(iconPath, F_OK|R_OK) != 0 )
  {
    LOGERROR("%s: %s(errno=%d)", iconPath, strerror(errno), errno);
    return;
  }

  pImgSplash = cvLoadImage(iconPath, CV_LOAD_IMAGE_COLOR);

  // set the window workspace as an opencv image
  session.m_pWin->WorkspaceSetAsCvImage();

  // image data can be dirty, so clear
  session.m_pWin->ShowCvImage(pImgSplash);
}

static void IlayMainUnSplash(IlaySession &session)
{
  session.m_pWin->WorkspaceRemoveAll();
}

/*!
 * \brief Validate video device command line option.
 *
 * Validation error results in termination of program.
 *
 * \return Video device minor number.
 */
static uint_t IlayMainVideoDevMinor(const char *sVidDev)
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

  if( uMajor != IlayVidDevMajor )
  {
    OptsInvalid(Argv0, "video device '%s': not a video device: major %d != %d",
        sVidDev, uMajor, IlayVidDevMajor);
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
static void IlayMainInitArgs(int argc, char *argv[], IlaySession &session)
{
  uint_t  uVidDevMinor;

  // name of this process
  Argv0 = basename(argv[0]);

  // parse input options
  argv = OptsGet(Argv0, &PkgInfo, &AppPgmInfo, AppOptsInfo, true,
                 &argc, argv);

  // validate existence and permissions of video device
  uVidDevMinor = IlayMainVideoDevMinor(OptsDevice);

  // record options in session datga
  session.SetVideoDevice(OptsDevice, uVidDevMinor);
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
static void IlayMainInit(int argc, char *argv[], IlaySession &session)
{
  // parse, validate, and save command-line arguments
  IlayMainInitArgs(argc, argv, session);

  // create the main window
  session.m_pWin = new rnrWindow(IlayWinName,
                                  IlayWinWidth,
                                  IlayWinHeight,
                                  IlayMaxButtonsPerMenu);

  // show splash screen
  IlayMainSplash(session);

  session.m_pWin->WaitKey(2000);

  IlayMainUnSplash(session);

}

static void IlayMainCleanup(IlaySession &session)
{
  delete session.m_pWin;
}


// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief ILAY main.
 *
 * \param argc  Command-line argument count.
 * \param argv  Command-line argument list.
 *
 * \par Exit Status:
 * Program exits with 0 success, \h_gt 0 on failure.
 */
int main(int argc, char *argv[])
{
#if 0
  IlaySession  session;    // the workflow session data
  int               rc;         // return code

  IlayMainInit(argc, argv, session);

  //rc = IlayMasterControl(session);

  IlayMainCleanup(session);

  return rc < 0? APP_EC_EXEC: APP_EC_OK;
#endif
  return 0;
}
