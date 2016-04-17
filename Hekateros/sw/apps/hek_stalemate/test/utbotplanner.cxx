////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Program:   StaleMate bot planner unit test
//
// File:      StaleMate.cxx
//
/*! \file
 *
 * $LastChangedDate: 2011-07-28 13:18:38 -0600 (Thu, 28 Jul 2011) $
 * $Rev: 1171 $
 *
 * \brief  Hekateros StaleMate botplanner test application.
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

#include "../StaleMate.h"
#include "../StaleMateTune.h"

#include "../SMBotPlanner.h"

using namespace std;
using namespace rnrWin;

#include "version.h"

int main(int argc, char *argv[])
{
  cerr << "~~~~~~~~~~~~~~~ SMBotPlanner unit test ~~~~~~~~~~~~~~~~~\n";
  SMBotPlanner *pBot = new SMBotPlanner();
  cerr << "Created botplanner pBot\n";

  // pBot->initRobotHek91();
  pBot->initRobotHek90();
  cerr << "initialized botplanner pBot\n\n";

  double j[] = {0,0,0,0,0};
  CvPoint3D32f p = pBot->Angles2RealWorld(j);

  cerr << " home: " << p.x << " " << p.y << " " << p.z << endl;

  CvPoint3D32f goal = cvPoint3D32f(596.5,85.5,100);
  CvPoint3D32f plan; 

  DynaSyncWriteTuple_T servos[1];
  uint_t          nServos = 1;

  cerr << "GOAL: " << goal.x << " " << goal.y << " " << goal.z << endl<<endl;
#if 0
  cerr << "Pyramid planner...\n";
  cerr << "distance from goal: " << 
    pBot->PyramidPlanner(goal, servos, nServos, 13) << endl; 

  plan = pBot->Angles2RealWorld(pBot->m_bestJoints);
  cerr << "planned position: " << plan.x << " " << plan.y <<" "<< plan.z <<"\n";

  cerr << "best joints using pyramid planner: " << endl; 
    cerr << pBot->m_bestJoints[0] << " " ; 
    cerr << pBot->m_bestJoints[1] << " " ; 
    cerr << pBot->m_bestJoints[2] << " " ; 
    cerr << pBot->m_bestJoints[3] << " " ; 
    cerr << pBot->m_bestJoints[4] << " " << endl; 

  cerr << "best servo vals using pyramid planner: " << endl; 
    cerr << servos[0].uVal << " " ; 
    cerr << servos[1].uVal << " " ; 
    cerr << servos[2].uVal << " " ; 
    cerr << servos[3].uVal << " " ; 
    cerr << servos[4].uVal << " " ; 
    cerr << servos[5].uVal << endl; 
#endif
  cerr << "\nConstrained planner...\n";
  cerr << "distance from goal: " << 
    pBot->ConstrainedPlanner(goal, servos, nServos) << endl; 

  plan = pBot->Angles2RealWorld(pBot->m_bestJoints);
  cerr << "planned position: " << plan.x << " " << plan.y <<" "<< plan.z <<"\n";

  cerr << "best joints using constrained planner: " << endl; 
    cerr << pBot->m_bestJoints[0] << " " ; 
    cerr << pBot->m_bestJoints[1] << " " ; 
    cerr << pBot->m_bestJoints[2] << " " ; 
    cerr << pBot->m_bestJoints[3] << " " ; 
    cerr << pBot->m_bestJoints[4] << " " << endl; 

  cerr << "best servo vals using constrained planner: " << endl; 
    cerr << servos[0].uVal << " " ; 
    cerr << servos[1].uVal << " " ; 
    cerr << servos[2].uVal << " " ; 
    cerr << servos[3].uVal << " " ; 
    cerr << servos[4].uVal << " " ; 
    cerr << servos[5].uVal << endl; 

  return 0;

}
