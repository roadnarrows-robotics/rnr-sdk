////////////////////////////////////////////////////////////////////////////////
//
// Package:   Kuon
//
// Program:   kuon_teleop
//
// File:      kuon_teleop.cxx
//
/*! \file
 *
 * $LastChangedDate: 2013-07-18 14:40:51 -0600 (Thu, 18 Jul 2013) $
 * $Rev: 3150 $
 *
 * \brief Teleoperated Kuon application.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2013-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
// Permission is hereby granted, without written agreement and without
// license or royalty fees, to use, copy, modify, and distribute this
// software and its documentation for any purpose, provided that
// (1) The above copyright notice and the following two paragraphs
// appear in all copies of the source code and (2) redistributions
// including binaries reproduces these notices in the supporting
// documentation.   Substantial modifications to this software may be
// copyrighted by their authors and need not follow the licensing terms
// described here, provided that the new terms are clearly indicated in
// all files where they apply.
//
// IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
// OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
// PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
// EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.
//
// THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
// "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
// PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
//
////////////////////////////////////////////////////////////////////////////////

#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <unistd.h>

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/opts.h"
#include "rnr/uri.h"

#include "rnr/hid/HIDXbox360.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "botsense/BotSense.h"
#include "botsense/libBotSense.h"

#include "botsense/bsKuon.h"

#include "version.h"

using namespace std;
using namespace rnr;


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

//
// Exit Codes
//
#define APP_EC_OK       0     ///< success exit code
#define APP_EC_USAGE    2     ///< usage error exit code
#define APP_EC_EXEC     4     ///< execution error exit code

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

// 
// The command with option and argument values.
//
static char  *Argv0;                        ///< the command
static char  *OptsKuonUri   = (char *)"botsense://localhost:"
                                      TOSTRING(BSPROXY_LISTEN_PORT_DFT);
                                            ///< Kuon URI
static bool   OptsIsServer  = false;        ///< [not] executing on Kuon
static bool   OptsEcho      = false;        ///< do [not] echo controller output
static bool   OptsShowGui   = false;        ///< do [not] display gui

//
// State
//
// TODO Put into Session derived class when appkit package is available.
//
static string       KuonHostName;     ///< Kuon host name / address
static int          KuonPortNum;      ///< Kuon connection port number
static BsClient_P   KuonBsClient;     ///< Kuon botsense client
static int          KuonVConn;        ///< botsense virtual connection
static HIDXbox360  *KuonXbox;         ///< Xbox360 HID
static bool         TeleopPaused;     ///< teloperations [not] paused
static int          KuonSpeedLeft;    ///< Kuon left side motor speeds
static int          KuonSpeedRight;   ///< Kuon right side motor speeds
static float        KuonGovernor;     ///< Kuon speed limie governor
static int          KuonBrake;        ///< Kuon brake level
static int          KuonSlew;         ///< Kuon slew level
static int          KuonSpeedProf;    ///< Kuon speed profile

/*!
 * \brief HID input user action mnemonics.
 *
 * Abstracts out the actual physical buttons, etc.
 */
enum UserAction
{
  UserActionExit  = HIDInput::MNEM_START,   ///< exit teleoperation
  UserActionPause,                          ///< pause teleoperation
  UserActionStart,                          ///< start teleoperation
  UserActionEStop,                          ///< emergency stop
  UserActionSpeedLimitInc,                  ///< increment speed limit
  UserActionSpeedLimitDec,                  ///< decrement speed limit
  UserActionThrottleX,                      ///< forward/backward speed throttle
  UserActionThrottleY,                      ///< left/right speed throttle
  UserActionSpeedProf                       ///< adjust speed profile
};


/*!
 * \brief Program information.
 */
static OptsPgmInfo_T PgmInfo =
{
  // usage_args
  NULL,

  // synopsis
  "Teleoperate the Kuon.",

  // long_desc = 
  "The %P application provides teleoperated control of a Kuon through a "
  "input human interface device such as a game controller or keyboard/mouse. "
  "Teleoperation may be performed connecting either directly to the Kuon "
  "or by proxy over an IP network.",

  // diagnostics
  NULL
};

/*!
 * \brief Command line options information.
 */
static OptsInfo_T OptsInfo[] =
{
  // -k, --kuon <uri>
  {
    "kuon",               // long_opt
    'k',                  // short_opt
    required_argument,    // has_arg
    true,                 // has_default
    &OptsKuonUri,         // opt_addr
    OptsCvtArgStr,        // fn_cvt
    OptsFmtStr,           // fn_fmt
    "<uri>",              // arg_name
    "(Proxied) Kuon Uniform Resource Identifier. Syntax:\n"  // opt desc
    "  [botsense://][hostname][:port]",
  },

  // -s, --server
  {
    "server",             // long_opt
    's',                  // short_opt
    no_argument,          // has_arg
    true,                 // has_default
    &OptsIsServer,        // opt_addr
    OptsCvtArgBool,       // fn_cvt
    OptsFmtBool,          // fn_fmt
    NULL,                 // arg_name
    "This instance is executing as the daemon server on the main "
    "Kuon processor. The server instance has higher priority "
    "over any host side client instances."
  },

  // -e, --echo
  {
    "echo",               // long_opt
    'e',                  // short_opt
    no_argument,          // has_arg
    true,                 // has_default
    &OptsEcho,            // opt_addr
    OptsCvtArgBool,       // fn_cvt
    OptsFmtBool,          // fn_fmt
    NULL,                 // arg_name
    "Do [not] echo HID input actions to the terminal."
  },

  // -e, --echo
  {
    "gui",                // long_opt
    OPTS_NO_SHORT,        // short_opt
    no_argument,          // has_arg
    true,                 // has_default
    &OptsShowGui,             // opt_addr
    OptsCvtArgBool,       // fn_cvt
    OptsFmtBool,          // fn_fmt
    NULL,                 // arg_name
    "Do [not] display GUI. (not implemented yet)"
  },

  {NULL, }
};

/*!
 * \brief Emergency stop Kuon.
 *
 * The motors are stopped and the brakes are hit hard.
 */
static void estopKuon()
{
  bsKuonReqStop(KuonBsClient, KuonVConn);
  bsKuonReqAlterBrake(KuonBsClient, KuonVConn, 31, 31, 31, 31);

  KuonSpeedLeft  = 0;
  KuonSpeedRight = 0;
  KuonBrake      = 31;
}

/*!
 * \brief Stop Kuon.
 *
 * The motors are stopped using the current speed profile.
 */
static void stopKuon()
{
  bsKuonReqStop(KuonBsClient, KuonVConn);

  KuonSpeedLeft  = 0;
  KuonSpeedRight = 0;
}

/*!
 * \brief Set the Kuon speed.
 *
 * \param nThrottleX    Forward/reverse throttle raw value.
 * \param nThrottleY    Left/right throttle raw value.
 * \param fGovernor     Speed limit governor.
 */
static void setKuonSpeed(int nThrottleX, int nThrottleY, float fGovernor)
{
  // mix throttle x and y values
  KuonSpeedLeft  = nThrottleX + nThrottleY;
  KuonSpeedRight = nThrottleY - nThrottleX;

  // scale 16-bit throttle values to 9-bit speed values
  KuonSpeedLeft  >>= 7;
  KuonSpeedRight >>= 7;

  // cap speeds
  if( KuonSpeedLeft >= 250 )
  {
    KuonSpeedLeft = 249;
  }
  else if( KuonSpeedLeft <= -250 )
  {
    KuonSpeedLeft = -249;
  }
  if( KuonSpeedRight >= 250 )
  {
    KuonSpeedRight = 249;
  }
  else if( KuonSpeedRight <= -250 )
  {
    KuonSpeedRight = -249;
  }

  // govern motor speeds (left side is reversed)
  KuonSpeedLeft  = (int)(-KuonSpeedLeft * fGovernor);
  KuonSpeedRight = (int)(KuonSpeedRight * fGovernor);

  // set all four motor speeds
  bsKuonReqSetMotorSpeeds(KuonBsClient, KuonVConn,
                          KuonSpeedLeft, KuonSpeedRight);
}

/*!
 * \brief Set Kuon speed profile.
 *
 * \param nSlew   Acceleration slew. Higher values equal slow accelerations.
 *                [0,255].
 * \param nBrake  Deacceleration brake. Higher value equal hard braking.
 *                [0,255].
 */
static void setKuonSpeedProf(int nSlew, int nBrake)
{
  bsKuonReqAlterSlew(KuonBsClient,  KuonVConn, nSlew, nSlew, nSlew, nSlew);
  bsKuonReqAlterBrake(KuonBsClient, KuonVConn, nBrake, nBrake, nBrake, nBrake);

  KuonSlew  = nSlew;
  KuonBrake = nBrake;
}

/*!
 * \brief Connect to the Kuon via bsProxy.
 *
 * \return Returns OK(0) on success, \h_lt 0 on failure.
 */
static int connectToKuon()
{
  int   rc;

  //
  // Connect to BotSense proxy server.
  //
  rc = bsServerConnect(KuonBsClient, KuonHostName.c_str(), KuonPortNum);

  if( rc < 0 )
  {
    LOGERROR("Can not connect to Kuon at %s:%d: %s.",
        KuonHostName.c_str(), KuonPortNum, bsStrError(rc));
    return rc;
  }

  LOGDIAG1("%s %s connected to Kuon bsProxy at %s:%d.",
      Argv0, (OptsIsServer? "server": "client"),
      KuonHostName.c_str(), KuonPortNum);

  //
  // Open proxied Kuon.
  //
  KuonVConn = bsKuonReqOpen(KuonBsClient, "0", "1", OptsIsServer, false);

  if( KuonVConn < 0 )
  {
    LOGERROR("Failed to open proxied Kuon: %s", bsStrError(KuonVConn));
    return KuonVConn;
  }

  // make sure Kuon is stopped
  stopKuon();
 
  return OK;
}

/*!
 * \brief Connect to the Xbox360 HID.
 *
 * \return Returns OK(0) on success, \h_lt 0 on failure.
 */
static int connectToXbox()
{
  int   nMaxTries = 3;
  int   nTries;
  int   rc;

  //
  // Find Xbox360 controller and open.
  //
  for(nTries=0; nTries<nMaxTries; ++nTries)
  {
    if( (rc = KuonXbox->open()) < 0 )
    {
      usleep(250000);   // 1/4/ second wait
    }
    else
    {
      break;
    }
  }

  if( rc < 0 )
  {
    LOGERROR("Cannot open to Xbox360 controller. Is it conneceted?");
    return rc;
  }

  usleep(250000);

  //
  // Ping controller.
  // RDK Does not work for wireless
  //
  //if( !KuonXbox->ping() )
  //{
  //  LOGERROR("Failed to ping Xbox360 controller.");
  //  return RC_ERROR;
  //}

  //
  // Associate mnemonics to buttons
  //
  KuonXbox->assocFeature(Xbox360FeatIdCenterX,      UserActionExit);
  KuonXbox->assocFeature(Xbox360FeatIdBack,         UserActionPause);
  KuonXbox->assocFeature(Xbox360FeatIdStart,        UserActionStart);
  KuonXbox->assocFeature(Xbox360FeatIdBButton,      UserActionEStop);
  KuonXbox->assocFeature(Xbox360FeatIdPadUp,        UserActionSpeedLimitInc);
  KuonXbox->assocFeature(Xbox360FeatIdPadDown,      UserActionSpeedLimitDec);
  KuonXbox->assocFeature(Xbox360FeatIdLeftJoyX,     UserActionThrottleX);
  KuonXbox->assocFeature(Xbox360FeatIdLeftJoyY,     UserActionThrottleY);
  KuonXbox->assocFeature(Xbox360FeatIdRightTrigger, UserActionSpeedProf);

  return OK;
}

/*!
 * \brief Parse Kuon URI string setting global state values.
 *
 * \param sUri    Uniform Resource Identifier.
 *
 * \return Returns OK(0) on success, \h_lt 0 on failure.
 */
static int parseKuonUri(const char *sUri)
{
  Uri_T  *pUri;
  int     rc;

  // parse
  if( (pUri = UriParseNew(sUri)) == NULL )
  {
    LOGERROR("Failed to parse URI \"%s\".", sUri);
    rc = -1;
  }

  // botsense scheme only support scheme for now
  else if( (pUri->m_sScheme != NULL) &&
            strcasecmp(pUri->m_sScheme, BSPROXY_URI_SCHEME) )
  {
    LOGERROR("For now only \"%s\" scheme is supported.", BSPROXY_URI_SCHEME);
    rc = -1;
  }

  else
  {
    KuonHostName = pUri->m_sHostName==NULL? "localhost": pUri->m_sHostName;
    KuonPortNum  = pUri->m_nPortNum==0? BSPROXY_LISTEN_PORT_DFT:
                                        pUri->m_nPortNum;
    rc = OK;
  }
  
  UriDelete(pUri);

  return rc;
}

/*!
 * \brief Main command-line options initialization.
 *
 * \param argc    Command-line argument count.
 * \param argv    Command-line argument list.
 *
 * \par Exits:
 * Program terminates on conversion error.
 */
static void mainInitArgs(int argc, char *argv[])
{
    // name of this process
  Argv0 = basename(argv[0]);

  // parse input options
  argv = OptsGet(Argv0, &PkgInfo, &PgmInfo, OptsInfo, true, &argc, argv);

  if( parseKuonUri(OptsKuonUri) < 0 )
  {
    OptsInvalid(Argv0, "Invalid '%s' argument to '%s' option.",
        OptsKuonUri, "--kuon");
  }
}

/*!
 * \brief Main initialization.
 *
 * \return Returns OK(0) on success, \h_lt 0 on failure.
 */
static int mainInit()
{
  int   rc;

  //
  // Teloperated Kuon initial state.
  //
  TeleopPaused    = true;
  KuonSpeedLeft   = 0;
  KuonSpeedRight  = 0;
  KuonGovernor    = 0.25;
  KuonBrake       = 0;
  KuonSlew        = 0;
  KuonSpeedProf   = -1;

  // create new botsense client
  KuonBsClient = bsClientNew("kuon");

  // create new Xbox360 controller
  KuonXbox = new HIDXbox360(); //XBOX_LIBUSB_DEBUG_ERROR);

  // show optional gui
  if( OptsShowGui )
  {
    //guiInit();
  }

  //
  // Connect to Kuon.
  //
  if( (rc = connectToKuon()) < 0 )
  {
    LOGERROR("Failed to connect to Kuon.");
  }

  //
  // Connect to Xbox360 controller.
  //
  else if( (rc = connectToXbox()) < 0 )
  {
    LOGERROR("Failed to connect to Xbox360.");
  }

  return rc;
}

/*!
 * \brief Main cleanup.
 */
static void mainCleanup()
{
  BsClientConnState_T connState;

  bsClientAttrGetConnState(KuonBsClient, &connState);

  if( connState.m_bIsConnected )
  {
    stopKuon();
    bsKuonReqClose(KuonBsClient, KuonVConn);
    bsServerDisconnect(KuonBsClient);
  }

  bsClientDelete(KuonBsClient);

  if( KuonXbox->isConnected() )
  {
    KuonXbox->close();
  }
  
  delete KuonXbox;

  if( OptsEcho )
  {
    printf("\nTeleoperation terminated\n");
  }
}

/*!
 * \brief Print Kuon teleoperation status header.
 */
static void echoStatusHdr()
{
  printf("\n\n");
  printf("              Xbox360 Teleoperation Status\n");
  printf("  State   LeftMotors   RightMotors   SpeedLimit   Slew   Brake\n");
}

/*!
 * \brief Print Kuon teleoperation status.
 */
static void echoStatus()
{
  if( KuonXbox->isLinked() )
  {
    printf("  %s    %4d          %4d          %4.2f       %3d    %3d\r",
      (TeleopPaused? "paused": "teleop"),
      KuonSpeedLeft,
      KuonSpeedRight,
      KuonGovernor,
      KuonSlew,
      KuonBrake);
  }
  else
  {
    printf("not linked%*s\r", 70, "");
  }
}

/*!
 * \brief Print Xbox360 teleop control mapping.
 */
static void menuXbox()
{
  printf("\n\n");
  printf("Xbox360 Teleoperation Controls\n");
  printf("  Button          Operation\n");
  printf("  ------          ---------\n");
  printf("  Center-X        Exit (ignored if server).\n");
  printf("  B               Emergence stop, pausing teleoperation.\n");
  printf("  Back            Print this menu, pausing teleoperation.\n");
  printf("  Start           Start/resume teleoperation.\n");
  printf("  Left Joy Stick  Speed throttle.\n");
  printf("  D Pad Up        Increment speed limit.\n");
  printf("  D Pad Down      Decrement speed limit.\n");
  printf("  Right Trigger   Adjust speed profile. The harder pressed,\n");
  printf("                  the softer the reaction.\n");
  printf("\n");
}

/*!
 * \brief Run teleooperation session.
 *
 * \return Returns OK(0) on success, \h_lt 0 on failure.
 */
static int teleop()
{
  int   nBttnPause  = HID_BTTN_UP;  // pause button state
  int   nBttnStart  = HID_BTTN_UP;  // restart button state
  int   nBttnEStop  = HID_BTTN_UP;  // estop button state
  int   nBttnIncGov = HID_BTTN_UP;  // increment governor button state
  int   nBttnDecGov = HID_BTTN_UP;  // decrement governor button state
  int   nVal;                       // button working value 1
  bool  bIsLinked;                  // is linked to xbox
  int   rc;                         // return code

  if( OptsEcho )
  {
    printf("Press 'start' to start, 'back' for help, 'center_x' to exit.\n\n");

    //KuonXbox->debugPrintHdr();
    echoStatusHdr();
  }

  bIsLinked = KuonXbox->isLinked();

  while( KuonXbox->isConnected() )
  {
    rc = KuonXbox->update();

    if( rc < 0 )
    {
      fprintf(stderr, "%d = update()\n", rc);
      break;
    }

    if( OptsEcho )
    {
      //KuonXbox->debugPrintState();
      echoStatus();
    }

    //
    // Handle wireless linked, not linked transitions.
    //
    if( KuonXbox->isLinked() != bIsLinked )
    {
      bIsLinked = KuonXbox->isLinked();
      if( !bIsLinked )
      {
        stopKuon();
      }
    }

    if( !bIsLinked )
    {
      continue;
    }

    //
    // Exit teleoperation.
    //
    if( (KuonXbox->getFeatureVal(UserActionExit) == HID_BTTN_DOWN) &&
        !OptsIsServer )
    {
      break;
    }

    //
    // Start teleoperation.
    //
    nVal = KuonXbox->getFeatureVal(UserActionStart);
    if( (nVal == HID_BTTN_DOWN) && (nBttnStart == HID_BTTN_UP) )
    {
      TeleopPaused = false;
    }
    nBttnStart = nVal;

    //
    // Pause teleoperation (has priority over start).
    //
    nVal = KuonXbox->getFeatureVal(UserActionPause);
    if( (nVal == HID_BTTN_DOWN) && (nBttnPause == HID_BTTN_UP) )
    {
      if( !TeleopPaused )
      {
        stopKuon();
      }
      menuXbox();
      TeleopPaused = true;
      if( OptsEcho )
      {
        echoStatusHdr();
      }
    }
    nBttnPause = nVal;

    //
    // Emergency stop.
    //
    nVal = KuonXbox->getFeatureVal(UserActionEStop);
    if( (nVal == HID_BTTN_DOWN) && (nBttnEStop == HID_BTTN_UP) )
    {
      estopKuon();
      TeleopPaused = true;
    }
    nBttnEStop = nVal;

    //
    // Paused state - do nothing.
    //
    if( TeleopPaused )
    {
      continue;
    }

    //
    // Boost speed limit governor.
    //
    nVal = KuonXbox->getFeatureVal(UserActionSpeedLimitInc);
    if( (nVal == HID_BTTN_DOWN) && (nBttnIncGov == HID_BTTN_UP) )
    {
      if( KuonGovernor <= 0.91 )
      {
        KuonGovernor += 0.05;
      }
    }
    nBttnIncGov = nVal;

    //
    // Clamp speed limit governor.
    //
    nVal = KuonXbox->getFeatureVal(UserActionSpeedLimitDec);
    if( (nVal == HID_BTTN_DOWN) && (nBttnDecGov == HID_BTTN_UP) )
    {
      if( KuonGovernor >= 0.09 )
      {
        KuonGovernor -= 0.05;
      }
    }
    nBttnDecGov = nVal;

    //
    // Set new speed profile
    //
    nVal = KuonXbox->getFeatureVal(UserActionSpeedProf);

    // profile 0
    if( nVal < 80 )
    {
      if( KuonSpeedProf != 0 )
      {
        setKuonSpeedProf(0, 31);
        KuonSpeedProf = 0;
      }
    }
    // profile 1
    else if( nVal < 160 )
    {
      if( KuonSpeedProf != 1 )
      {
        setKuonSpeedProf(40, 15);
        KuonSpeedProf = 1;
      }
    }
    // profile 2
    else
    {
      if( KuonSpeedProf != 2 )
      {
        setKuonSpeedProf(80, 0);
        KuonSpeedProf = 2;
      }
    }

    //
    // Set speed
    //
    setKuonSpeed(KuonXbox->getFeatureVal(UserActionThrottleX),
                 KuonXbox->getFeatureVal(UserActionThrottleY),
                 KuonGovernor);
  }

  return OK;
}


// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Application main.
 *
 * \param argc  Command-line argument count.
 * \param argv  Command-line argument list.
 *
 * \par Exit Status:
 * Program exits with 0 success, \h_gt 0 on failure.
 */
int main(int argc, char *argv[])
{
  int   ec = APP_EC_OK;   // exit code

  // initialize command-line options and arguments
  mainInitArgs(argc, argv);

  // teleoperation initialization
  if( mainInit() < 0 )
  {
    ec = APP_EC_EXEC;
  }

  // teleoperate
  else if( teleop() < 0 )
  {
    ec = APP_EC_EXEC;
  }
  
  // teleoperation cleanup
  mainCleanup();

  // wait for udev to settle prior to exiting
  if( OptsIsServer )
  {
    usleep(5000000);
  }

  return ec;
}
