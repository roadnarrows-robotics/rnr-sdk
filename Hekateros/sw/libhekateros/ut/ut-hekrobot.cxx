////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Unit Test: Test libhekateros library.
//  
/*! \file
 *  
 * $LastChangedDate: 2014-10-10 12:43:53 -0600 (Fri, 10 Oct 2014) $
 * $Rev: 3781 $
 *
 *  \ingroup hek_ut
 *  
 *  \brief Unit test for libhekateros HekRobot top-level class.
 *
 *  \author Robin Knight (robin.knight@roadnarrows.com)
 *  
 * \copyright
 *   \h_copy 2013-2017. RoadNarrows LLC.\n
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

#include <stdio.h>

#include <iostream>
#include <string>
#include <vector>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekTune.h"
#include "Hekateros/hekXmlCfg.h"
#include "Hekateros/hekXmlTune.h"
#include "Hekateros/hekTraj.h"
#include "Hekateros/hekUtils.h"
#include "Hekateros/hekRobot.h"

#include "gtest/gtest.h"

using namespace ::std;
using namespace ::hekateros;

/*!
 *  \ingroup hek_ut
 *  \defgroup hek_ut_robot Library libhekateros Unit Tests
 *  \brief Testing of basic functionality of \ref hekateros::HekRobot
 *  class instance.
 *  \{
 */

/*!
 * \brief Test menu.
 */
static const char *TestMenu = 
  "\nTest Menu\n"
  "   1             Create robot.\n"
  "   2             Delete robot.\n"
  "   3 [<file>]    Read xml cfg <file>. DEFAULT: /etc/hekateros.conf\n"
  "   4 [<file>]    Read xml tune <file>. DEFAULT: /etc/hek_tune.conf\n"
  "   5 [<dev>]     Connect to dynamixel bus <dev>. DEFAULT: /dev/ttyUSB0\n" 
  "   6             Disconnect from dynamixel bus.\n"
  "   7 <how>       [A]synchronously calibrate arm. <how> is 'a' or 's'. "
  "DEFAULT: a\n"
  "   8 <move>      Execute pre-defined move: calibrated balanced parked "
  "   9 <deg>       Rotate wrist by the given degrees."
  "   a             Cancel asynchronous task."
  "gopen gclose.\n"
  "\n"
  "   d             Enable library diagnostics logging.\n"
  "   ?             Show this menu.\n"
  "   q             Quit unit test.\n\n";


void dbgPrintTunes(HekTunes &tunes)
{
  HekTunes::MapJointTunes::iterator iter;

  fprintf(stderr, "HekTunes\n");
  fprintf(stderr, "{\n");
  fprintf(stderr, "  m_fKinematicsHz      = %lf;\n", tunes.m_fKinematicsHz);
  fprintf(stderr, "  m_fClearTorqueOffset = %lf;\n", tunes.m_fClearTorqueOffset);
  fprintf(stderr, "  m_fVelDerate         = %lf;\n", tunes.m_fVelDerate);
  fprintf(stderr, "  m_eTrajNorm          = %d;\n", tunes.m_eTrajNorm);
  fprintf(stderr, "  m_fTrajEpsilon       = %lf;\n", tunes.m_fTrajEpsilon);
  for(iter = tunes.m_mapJointTunes.begin();
      iter != tunes.m_mapJointTunes.end();
      ++iter)
  {
    fprintf(stderr, "  joint %s\n", iter->first.c_str());
    fprintf(stderr, "  {\n");
    fprintf(stderr, "    m_fTolPos       = %lf;\n", iter->second.m_fTolPos);
    fprintf(stderr, "    m_fTolVel       = %lf;\n", iter->second.m_fTolVel);
    fprintf(stderr, "    m_fPidKp        = %lf;\n", iter->second.m_fPidKp);
    fprintf(stderr, "    m_fPidKi        = %lf;\n", iter->second.m_fPidKi);
    fprintf(stderr, "    m_fPidKd        = %lf;\n", iter->second.m_fPidKd);
    fprintf(stderr, "    m_fOverTorqueTh = %lf;\n", iter->second.m_fOverTorqueTh);
    fprintf(stderr, "  };\n");
  }
  fprintf(stderr, "};\n");
}


// .............................................................................
// HekRobot Unit Tests
// .............................................................................

/*!
 *  \brief Test HekRobot connected to hekateros.
 *
 *  \return Returns 0 if test succeeds, else returns \h_lt 0.
 */
static int testHekRobot()
{
  /*!
   * \brief Handy dandy checker.
   */
  #define BOT_CHK(conn) \
    if( pRobot == NULL ) \
    { \
      printf("Error: Robot object does not exist - create first.\n"); \
      continue; \
    } \
    else if( conn && !pRobot->isConnected() ) \
    { \
      printf("Error: Robot not connected.\n"); \
      continue; \
    }

  HekRobot   *pRobot = NULL;
  char        c;
  char        line[256];
  char        arg1[64];
  int         nArgs;
  double      fVal;
  bool        bQuit = false;

  printf("%s", TestMenu);

  while( !bQuit )
  {
    printf("> ");

    if( gets(line) == NULL )
    {
      c = EOF;
    }

    if( (nArgs = sscanf(line, " %c %s", &c, arg1)) == EOF )
    {
      continue;
    }

#if DEBGUG
    fprintf(stderr, "#%d ", nArgs);
    if( nArgs >= 1 )
    {
      fprintf(stderr, "c=%c ", c);
    }
    if( nArgs >= 2 )
    {
      fprintf(stderr, "arg1=%s ", arg1);
    }
    fprintf(stderr, "\n");
#endif

    switch(c)
    {
      // create robot object
      case '1':
        if( pRobot == NULL )
        {
          pRobot = new HekRobot;
          printf("Robot created.\n");
        }
        else
        {
          printf("Robot already created.\n");
        }
        break;

      // delete robot object
      case '2':
        if( pRobot != NULL )
        {
          delete pRobot;
          pRobot = NULL;
          printf("Robot deleted.\n");
        }
        else
        {
          printf("Robot not created.\n");
        }
        break;

      // parse xml configuration and set robot descriptions
      case '3':
        BOT_CHK(false);
        {
          string    str = nArgs<2? "/etc/hekateros.conf": arg1;
          HekXmlCfg xml;
          if( xml.loadFile(str) < 0 )
          {
            printf("Error: Loading XML file '%s' failed.\n", str.c_str());
          }
          else if( xml.setHekDescFromDOM(*pRobot->getHekDesc()) < 0 )
          {
            printf("Error: Setting robot description failed.\n");
          }
          else if( pRobot->getHekDesc()->markAsDescribed() < 0 )
          {
            printf("Error: Failed to finalize descriptions.\n");
          }
          else
          {
            printf("Robot description complete for %s.\n",
               pRobot->getHekDesc()->getFullProdBrief().c_str());
          }
        }
        break;

      // parse xml tune file
      case '4':
        BOT_CHK(false);
        {
          string    str = nArgs<2? "/etc/hek_tune.conf": arg1;
          fprintf(stderr, "rdk here 1\n");
          HekXmlTune xml;
          fprintf(stderr, "rdk here 2\n");
          HekTunes tunes;
          fprintf(stderr, "rdk here 3\n");
          if( xml.loadFile(str) < 0 )
          {
            printf("Error: Loading XML file '%s' failed.\n", str.c_str());
          }
          else if( xml.setTunesFromDOM(tunes) < 0 )
          {
            printf("Error: Setting robot tune parameters failed.\n");
          }
          else
          {
            printf("Robot tuning complete.\n");
            dbgPrintTunes(tunes);
          }
        }
        break;

      // connect
      case '5':
        BOT_CHK(false);
        {
          string    str = nArgs<2? "/dev/ttyUSB0": arg1;
          if( pRobot->connect(str) < 0 )
          {
            printf("Error: Failed to connnect to '%s'.\n", str.c_str());
          }
          else
          {
            printf("Robot connected.\n");
          }
        }
        break;

      // disconnect
      case '6':
        BOT_CHK(false);
        if( pRobot->isConnected() )
        {
          pRobot->disconnect();
          printf("Robot disconnected.\n");
        }
        else
        {
          printf("Robot not connected.\n");
        }
        break;

      // calibrate
      case '7':
        BOT_CHK(true);
        {
          string    str = nArgs<2? "a": arg1;
          if( (str == "s") || (str == "sync") )
          {
            if( pRobot->calibrate() < 0 )
            {
              printf("Error: Calibration failed.\n");
            }
            else
            {
              printf("Robot calibrated.\n");
            }
          }
          else
          {
            if( pRobot->calibrateAsync() < 0 )
            {
              printf("Error: Async calibration failed.\n");
            }
            else
            {
              printf("Calibrating robot - enter '9' to cancel.\n");
            }
          }
        }
        break;

      // move
      case '8':
        BOT_CHK(true);
        if( !pRobot->isCalibrated() )
        {
          printf("Error: Robot not calibrated.\n");
        }
        else if( nArgs < 2 )
        {
          printf("Error: Missing <move> argument.\n");
        }
        else if( strlen(arg1) < 2 )
        {
          printf("Error: \"%s\": Ambiguous <move>.\n", arg1);
        }
        else if( !strncmp(arg1, "calibrated", strlen(arg1)) )
        {
          pRobot->gotoZeroPtPos();
        }
        else if( !strncmp(arg1, "balanced", strlen(arg1)) )
        {
          pRobot->gotoBalancedPos();
        }
        else if( !strncmp(arg1, "park", strlen(arg1)) )
        {
          pRobot->gotoParkedPos();
        }
        else if( !strncmp(arg1, "gopen", strlen(arg1)) )
        {
          pRobot->openGripper();
        }
        else if( !strncmp(arg1, "gclose", strlen(arg1)) )
        {
          pRobot->closeGripper();
        }
        else
        {
          printf("Error: \"%s\": Unknown <move>.\n", arg1);
        }
        break;

      // move
      case '9':
        BOT_CHK(true);
        if( !pRobot->isCalibrated() )
        {
          printf("Error: Robot not calibrated.\n");
        }
        else if( nArgs < 2 )
        {
          printf("Error: Missing <deg> argument.\n");
        }
        else if( sscanf(arg1, "%lf", &fVal) < 1 )
        {
          printf("Error: \"%s\": NaN.\n", arg1);
        }
        else
        {
          HekJointTrajectoryPoint   trajPoint;
          trajPoint.append(string("wrist_rot"), degToRad(fVal), 5.0);
          pRobot->moveArm(trajPoint);
        }
        break;

      // cancel async task
      case 'a':
        BOT_CHK(true);
        pRobot->cancelAsyncTask();
        break;
      
      case 'd':
        LOG_SET_THRESHOLD(LOG_LEVEL_DIAG3);
        break;

      // show menu
      case '?':
        printf("%s", TestMenu);
        break;

      // quit
      case 'q':
      case EOF:
        bQuit = true;
        break;

      default:
        printf("'%c': huh?\n", c);
        break;
    }
  }

  if( pRobot != NULL )
  {
    delete pRobot;
  }

  return 0;
}


#ifndef JENKINS

/*!
 * \brief Test State Machine class.
 *
 * \par The Test:
 * Construct a (2,3) Turing State Machin and run.
 */
TEST(HEKROBOT, HEKROBOT)
{
  EXPECT_TRUE( testHekRobot() == 0 );
}

#endif // JENKINS


/*!
 *  \}
 */
