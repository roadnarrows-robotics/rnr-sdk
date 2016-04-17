////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Program:   StaleMate
//
// File:      SMOrp.cxx
//
/*! \file
 *
 * $LastChangedDate: 2012-05-08 16:00:59 -0600 (Tue, 08 May 2012) $
 * $Rev: 1943 $
 *
 * \brief StaleMate OpenRave Python Interface.
 *
 * \author Rob Shiely (rob@roadnarrows.com)
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011.  RoadNarrows
 * (http://www.roadnarrows.com)
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
#include <sys/wait.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <signal.h>
#include <libgen.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include <cstring>
#include <iostream>
#include <fstream>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/rnrWin.h"

#include "StaleMate.h"
#include "StaleMateTune.h"

using namespace std;
using namespace rnrWin;


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

#define ORP_MAX_BUF_SIZE      256

static int    OrpPipeToSim[2];
static int    OrpPipeFromSim[2];
static pid_t  OrpPidChild;

// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

// RDK TODO make variable arguments
int StaleMateOrpOneTimeInit(StaleMateSession &session, const char *sOpenRaveApp)
{
  char   *argv[8];

  // make stalemate to chess app pipe
  if( pipe(OrpPipeToSim) == -1 )
  {
    LOGSYSERROR("Failed to create pipe to OpenRave python application.");
    return -HEK_ECODE_SYS;
   }

  // make chess app to stalemate pipe
  if ( pipe(OrpPipeFromSim) == -1 )
  {
    LOGSYSERROR("Failed to create pipe from OpenRave python application.");
    return -HEK_ECODE_SYS;
  }

  // for stalemate
  OrpPidChild = fork();

  // fork fail error
  if( OrpPidChild == -1 )
  {
    LOGSYSERROR("Could not fork %s.", sOpenRaveApp);
    return -HEK_ECODE_SYS;
  }

  //
  // Child Process
  //
  if( OrpPidChild == 0 )
  {
    // close unused pipe ends 
    close(OrpPipeToSim[1]);
    close(OrpPipeFromSim[0]);

    // duplciate pipe ends to standard input and output
    dup2(OrpPipeToSim[0], 0);
    dup2(OrpPipeFromSim[1], 1);

    // build exec arguments
    argv[0] = (char *)sOpenRaveApp;
    argv[1] = (char *)"-i";
    argv[2] = (char *)"/prj/pkg/Hekateros/sw/openrave/env/hek0.91.env.xml";
    argv[3] = NULL;

    // exec open rave python application
    execvp(argv[0], argv);

    // if here then exec failed
    exit(1);
  }

  //
  // Parent Process
  //
  else
  {
    LOGDIAG1("Forked-exec'ed %s, child pid=%u.", sOpenRaveApp, OrpPidChild);

    // close unused pipe ends 
    close(OrpPipeToSim[0]);
    close(OrpPipeFromSim[1]);

    sleep(1);

    StaleMateOrpCfg(session);

    return HEK_OK;
  }
}

void StaleMateOrpCfg(StaleMateSession &session)
{
  if( !session.m_hek.bUseOpenRave )
  {
    return;
  }

  // do any configuration here
}

void StaleMateOrpKill(StaleMateSession &session)
{
  if( !session.m_hek.bUseOpenRave )
  {
    return;
  }

  kill(OrpPidChild, SIGTERM);
  waitpid(OrpPidChild, NULL, WEXITED);
}

int StaleMateOrpDegToRadAndSend(StaleMateSession &session,
                                DynaRealTuple_T   posDeg[],
                                size_t            nPos)
{
  double  rad[SM_HEK_DOF_TOTAL];
  char    buf[ORP_MAX_BUF_SIZE];
  int     i, j;
  ssize_t n;

  if( !session.m_hek.bUseOpenRave )
  {
    return HEK_OK;
  }

  LOGDIAG2("Updating openRAVE with %u link positions.", nPos);

  memset(rad, 0, sizeof(rad));

  // map servo id to fixed joint locations
  for(i=0; (i<SM_HEK_DOF_TOTAL) && (i < (int)nPos); ++i)
  {
    switch( posDeg[i].nServoId )
    {
      case HEK_SERVO_ID_BASE:
        rad[0] = posDeg[i].fVal;
        break;
      case HEK_SERVO_ID_SHOULDER_L:
      case HEK_SERVO_ID_SHOULDER_R:
        rad[1] = posDeg[i].fVal - 90.0;
        break;
      case HEK_SERVO_ID_ELBOW:
        rad[2] = posDeg[i].fVal;
        break;
      case HEK_SERVO_ID_WRIST_ROT:
        rad[3] = -posDeg[i].fVal;
        break;
      case HEK_SERVO_ID_WRIST_PITCH:
        rad[4] = posDeg[i].fVal;
        break;
      case HEK_SERVO_ID_GRABOID:
        rad[5] = posDeg[i].fVal;
        break;
    }
  }

  // convert degrees to radians
  for(i=0; i<SM_HEK_DOF_TOTAL; ++i)
  {
    rad[i] = (rad[i] * M_PI) / 180.0;
  }

  //
  // build message
  //
  j = sprintf(buf, "robot.SetActiveDOFValues([");

  for(i=0; i<SM_HEK_DOF_TOTAL; ++i)
  {
    j += sprintf(buf+j, "%lf%c", rad[i], (i<SM_HEK_DOF_TOTAL-1? ',': ']'));
  }

  j = sprintf(buf+j, ")\n");

  //
  // send message
  //
  n = write(OrpPipeToSim[1], buf, strlen(buf));

  if( n < (ssize_t)j )
  {
    return -HEK_ECODE_SYS;
  }

  else
  {
    return HEK_OK;
  }
}
