////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Program:   StaleMate
//
// File:      SMBotPlanner.cxx
//
/*! \file
 *
 * $LastChangedDate: 2012-06-13 10:47:00 -0600 (Wed, 13 Jun 2012) $
 * $Rev: 2043 $
 *
 * \brief StaleMate kinematic planning utils implementation.
 *
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
#include <stdarg.h>
#include <libgen.h>

#include <cstring>
#include <iostream>
#include <fstream>
#include <cmath>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/rnrWin.h"

#include "StaleMate.h"
#include "StaleMateTune.h"

#include "SMBotPlanner.h"

using namespace std;
using namespace rnrWin;

//
//  Planning utility helper classes
//

// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

/*!
 *  \brief initialize robot to hardcoded Hek-9.0 values
 */
void SMBotPlanner::initRobotHek90()
{
  //shoulder offset from base [mm]
  m_offsetX   = 22.65;
  m_offsetY   = -5.76;
  m_offsetZ   = 174.29;

  // link dimensions [mm]
  m_links[0]  = 240;  // upper arm
  m_links[1]  = 250;  // fore arm
  m_links[2]  = 156.51; // end effector

  // initialize planning joints to zero
  m_bestJoints[0] = 0; 
  m_bestJoints[1] = 0; 
  m_bestJoints[2] = 0; 
  m_bestJoints[3] = 0; 
  m_bestJoints[4] = 0; 
  
  // servo offsets
  m_servoOffsets[0] = 652;    // raw in servo land = 0[deg] in joint land
  m_servoOffsets[1] = 718;    // raw in servo land = 0[deg] in joint land
  //RDK m_servoOffsets[2] = 324;    // raw in servo land = 0[deg] in joint land
  m_servoOffsets[2] = 648;    // raw in servo land = 0[deg] in joint land
  m_servoOffsets[3] = 716;    // raw in servo land = 0[deg] in joint land
  m_servoOffsets[4] = 680;    // raw in servo land = 0[deg] in joint land

  // servos min
  m_servosMin[0] = 10;    // [raw] in servo land = 0[deg] in joint land
  m_servosMin[1] = 10;    // [raw] in servo land = 0[deg] in joint land
  //RDK m_servosMin[2] = 10;    // [raw] in servo land = 0[deg] in joint land
  m_servosMin[2] = 10;    // [raw] in servo land = 0[deg] in joint land
  m_servosMin[3] = 10;    // [raw] in servo land = 0[deg] in joint land
  m_servosMin[4] = 10;    // [raw] in servo land = 0[deg] in joint land

  // servos max
  m_servosMax[0] = 1015;    // [raw] in servo land = 0[deg] in joint land
  m_servosMax[1] = 1015;    // [raw] in servo land = 0[deg] in joint land
  //RDK m_servosMax[2] = 1015;    // [raw] in servo land = 0[deg] in joint land
  m_servosMax[2] = 1015;    // [raw] in servo land = 0[deg] in joint land
  m_servosMax[3] = 1015;    // [raw] in servo land = 0[deg] in joint land
  m_servosMax[4] = 1015;    // [raw] in servo land = 0[deg] in joint land

  // gear ratios 
  m_servoDirections[0] = 1; 
  m_servoDirections[1] = -1;
  //RDK m_servoDirections[2] = 1;
  m_servoDirections[2] = -1;
  m_servoDirections[3] = 1;
  m_servoDirections[4] = -1;

  // gear ratios 
  m_servoGearRatios[0] = 3.0/1.0;
  m_servoGearRatios[1] = 2.0/1.0;
  //RDK m_servoGearRatios[2] = 2.0/1.0;
  m_servoGearRatios[2] = 1.75;
  m_servoGearRatios[3] = 1.477;
  m_servoGearRatios[4] = 1.477;

  // servo directions 
  m_servoDirections[0] = 1;
  m_servoDirections[1] = -1;
  //RDK m_servoDirections[2] = 1;
  m_servoDirections[2] = -1;
  m_servoDirections[3] = 1;
  m_servoDirections[4] = -1;

  return;
}

/*!
 *  \brief initialize robot to hardcoded Hek-9.1 values
 */
void SMBotPlanner::initRobotHek91()
{
  //shoulder offset from base [mm]
  m_offsetX   = 22.65;
  m_offsetY   = -5.76;
  m_offsetZ   = 174.29;

  // link dimensions [mm]
  m_links[0]  = 372.1;  // upper arm
  m_links[1]  = 367.6;  // fore arm
  m_links[2]  = 156.51; // end effector

  // initialize planning joints to zero
  m_bestJoints[0] = 0;          // base rotation
  m_bestJoints[1] = 0;          // shoulder
  m_bestJoints[2] = 0;          // elbow
  m_bestJoints[3] = 0;          // wrist tilt
  m_bestJoints[4] = 0;          // wrist rotation
  
  // angle offsets
  m_servoOffsets[0] = 544;    // [raw] in servo land = 0[deg] in joint land
  m_servoOffsets[1] = 2629;   // [raw] in servo land = 0[deg] in joint land
  m_servoOffsets[2] = -180;   // [raw] in servo land = 0[deg] in joint land
  m_servoOffsets[3] = 520;    // [raw] in servo land = 0[deg] in joint land
  m_servoOffsets[4] = 959;    // [raw] in servo land = 0[deg] in joint land
  
  // servos min
  m_servosMin[0] = 10;      // min servo raw value 
  m_servosMin[1] = 50;      // min servo raw value
  //RDK m_servosMin[2] = 50;      // min servo raw value
  m_servosMin[2] = 10;      // min servo raw value
  m_servosMin[3] = 10;      // min servo raw value
  m_servosMin[4] = 10;      // min servo raw value
 
  // servos max
  m_servosMax[0] = 1015;    // max servo raw value
  m_servosMax[1] = 4050;    // max servo raw value
  //RDK m_servosMax[2] = 4050;    // max servo raw value
  m_servosMax[2] = 1015;    // max servo raw value
  m_servosMax[3] = 1015;    // max servo raw value
  m_servosMax[4] = 1015;    // max servo raw value
 
  // gear ratios 
  m_servoGearRatios[0] = 3.0/1.0;
  m_servoGearRatios[1] = 2.0/1.0;
  //RDK m_servoGearRatios[2] = 2.0/1.0;
  m_servoGearRatios[2] = 8.0/3.0;
  m_servoGearRatios[3] = 30.0/18.0;
  m_servoGearRatios[4] = 30.0/20.0;

  // servo directions 
  m_servoDirections[0] = 1;
  m_servoDirections[1] = -1;
  //RDK m_servoDirections[2] = 1;
  m_servoDirections[2] = 1;
  m_servoDirections[3] = 1;
  m_servoDirections[4] = 1;

  return;
}

/*!
 *  \brief initialize robot description from XML
 */
void SMBotPlanner::initRobotXml(const char* filename)
{
  // TODO: load robot params from XML
}

/*!
 * \brief set a goal (x,y,z)
 */
void SMBotPlanner::setGoal(double x, double y, double z)
{
  m_goal.x = x;
  m_goal.y = y;
  m_goal.z = z;
}

/*!
 * \brief set a goal (CvPoint3D32f)
 */
void SMBotPlanner::setGoal(const CvPoint3D32f& goal)
{
  m_goal.x = goal.x;
  m_goal.y = goal.y;
  m_goal.z = goal.z;
}

/*!
 * \brief Determines joint values to reach goal. 
 */
double SMBotPlanner::ConstrainedPlanner( CvPoint3D32f goal,
                                         DynaPosTuple_T servos[],
                                         uint_t nServos )
{
  m_goal = goal;
  m_goal.z = m_goal.z + ((m_goal.x-TuneChessDEBaseX)/57.0)*7.5;
  m_goal.y = m_goal.y * 0.90;
  double joints[5] = {0,0,0,0,0};

  static double deg2rad = PI/180.0;
  static double rad2deg = 180.0/PI;

  m_distance2goal = distance2goal(joints);

  // if both x AND y nonzero
  if( m_goal.x != 0 && m_goal.y != -5.76 )
  {
    // DHP: kludgy because of y-offset... this is really an implicit function
    // 5.76 is a first order correction...  gets us pretty close.
    // worse for large y
    joints[0]  = rad2deg * atan2(m_goal.y + 5.76, m_goal.x);
    joints[4]  = -1.0 * joints[0];
  }
  // if x OR y == 0
  else
  {
    joints[0] = joints[4] = 0;
  }

  for(double shoulder = -60; shoulder < 90; shoulder += .1)
  {
    joints[1] = shoulder;
    for (double elbow = 22.5; elbow < 135; elbow += .1)
    {
      joints[2] = elbow;
      joints[3] = 180 - (shoulder + elbow);
      if( distance2goal(joints) < m_distance2goal )
      {
        for(int i = 0; i < 5; i++ )
        {
          m_bestJoints[i] = joints[i];
          m_distance2goal = distance2goal(m_bestJoints);
        }
      }
    }
  }

  if( !(setServos(servos, nServos)) )
  {
    return -1*HEK_ECODE_DYNA;
  }

  return m_distance2goal;
}

/*!
 * \brief Determines joint values to reach goal. 
 */
double SMBotPlanner::PyramidPlanner( CvPoint3D32f goal,
                                     DynaPosTuple_T servos[],
                                     uint_t nServos,
                                     int maxdepth )
{
  m_maxdepth   = maxdepth;
  m_goal       = goal;
  
  double         tmpdist;

  // initialize joints to 0 
  for (int i = 0; i < 5; i++)
  {
    m_bestJoints[i] = 0;
  }

  m_distance2goal = distance2goal(m_bestJoints);
  
  // setup search space around 'center'
  Q ss(0, m_bestJoints, ranges);

  // recurse to find best joints
  m_distance2goal = r(1, ss);

  if( !(setServos(servos, nServos)) )
  {
    return -1*HEK_ECODE_DYNA;
  }
  
  return m_distance2goal;
}


double SMBotPlanner::r(int depth, Q ss)
{
  if( depth > m_maxdepth )
  {
    return m_distance2goal;
  }

  for(int i = 0; i < 32; i++)
  {
    if(distance2goal(ss.pts[i]) < m_distance2goal ) 
    {
      for(int j = 0; j < 5; j++ )
      {  
        m_bestJoints[j] = ss.pts[i][j];
      }
      m_distance2goal = distance2goal(ss.pts[i]);
    }
  }

  // found solution within tolerance
  if(m_distance2goal < m_epsilon)
  {
    return m_distance2goal;
  }

  // search finer-grained subspace
  else
  {
    Q ss2(depth++, m_bestJoints, ranges);
    return r(depth, ss2);
  }

}

/*!
 * \brief Calculate the real world coordinates associated with joint angles
 *
 * \return loc  Real world coordinates of end effector.
 */
CvPoint3D32f SMBotPlanner::Angles2RealWorld(double joints[])
{
    static double a = PI/180.0;  // degrees->radians

    CvPoint3D32f planned;
    double rho;
    rho = m_offsetX + m_links[0]*sin(joints[1]*a) + 
                      m_links[1]*sin((joints[1]+joints[2])*a) + 
                      m_links[2]*sin((joints[1]+joints[2]+joints[3])*a);
    
    planned.x = rho * cos(joints[0]*a) - m_offsetY * sin(joints[0]*a);
    planned.y = rho * sin(joints[0]*a) + m_offsetY * cos(joints[0]*a);
    planned.z = m_offsetZ + m_links[0]*cos(joints[1]*a) + 
                      m_links[1]*cos((joints[1]+joints[2])*a) + 
                      m_links[2]*cos((joints[1]+joints[2]+joints[3])*a);

  return planned;
}

/*!
 *  \brief returns L1 distance to goal..
 */
double SMBotPlanner::distance2goal(double joints[])
{
  CvPoint3D32f plan = Angles2RealWorld(joints);

  double distance2goal = fabs(plan.x-m_goal.x) + 
                         fabs(plan.y-m_goal.y) +
                         fabs(plan.z-m_goal.z); 

  return distance2goal;
}


/*!
 *  \brief set servo values based on current m_bestJoints
 */
bool SMBotPlanner::setServos(DynaPosTuple_T servos[], uint_t nServos)
{

  static double deg2servo1 = 1.0/0.29;
  static double deg2servo2 = 1.0/0.06;
  double ser[6];

  bool ret = true;

  // BASE 
  ser[0] = ( m_servoOffsets[0] + 
   m_servoDirections[0] * m_bestJoints[0] * deg2servo1 * m_servoGearRatios[0]);

  //shoulderL
  ser[1] = ( m_servoOffsets[1] + 
   m_servoDirections[1] * m_bestJoints[1] * deg2servo2 * m_servoGearRatios[1]);

  //shoulderR
  //RDK ser[2] = ( m_servoOffsets[2] + 
  //RDK m_servoDirections[2] * m_bestJoints[1] * deg2servo2 * m_servoGearRatios[2]);

  //Elbow
  ser[2] = ( m_servoOffsets[2] + 
   m_servoDirections[2] * m_bestJoints[2] * deg2servo1 * m_servoGearRatios[2]);
          
  // wrist pitch
  ser[3] = ( m_servoOffsets[3] + 
   m_servoDirections[3] * m_bestJoints[3] * deg2servo1 * m_servoGearRatios[3]);

  // wrist rotate
  ser[4] = (m_servoOffsets[4] + 
   m_servoDirections[4] * m_bestJoints[4] * deg2servo1 * m_servoGearRatios[4]
   - (ser[3]-m_servoOffsets[3]));
            


  for(int i=0; i < SM_HEK_NSERVOS_BASE_M; i++)
  {
    if( ser[i] < m_servosMin[i] )
    {
      LOGERROR("Servo %d out of range: %f. Setting to minimum value: %f", 
                i+1, ser[i], m_servosMin[i]);
      ser[i] = m_servosMin[i];
      ret = false;
    }
    else if ( ser[i] > m_servosMax[i] ) 
    {
      LOGERROR("Servo %d out of range: %f. Setting to maximum value: %f", 
                i+1, ser[i], m_servosMax[i]);
      ser[i] = m_servosMax[i];
      ret = false;
    }
  }

  // BASE 
  servos[0].m_nServoId = HEK_SERVO_ID_BASE;
  servos[0].m_nPos = (int)ser[0];

  //shoulderL
  servos[1].m_nServoId = HEK_SERVO_ID_SHOULDER_L;
  servos[1].m_nPos = (int)ser[1];

  //shoulderR
  //RDK servos[2].uVal = (uint_t)ser[2];

  //Elbow
  servos[2].m_nServoId = HEK_SERVO_ID_ELBOW;
  servos[2].m_nPos = (int)ser[2];

  // wrist pitch
  servos[3].m_nServoId = HEK_SERVO_ID_WRIST_PITCH;
  servos[3].m_nPos = (int)ser[3];

  // wrist rotate
  servos[4].m_nServoId = HEK_SERVO_ID_WRIST_ROT;
  servos[4].m_nPos = (int)ser[4];


  LOGDIAG2("Planned servo values: %d %d %d %d %d", servos[0].m_nPos,
                                                      servos[1].m_nPos,
                                                      servos[2].m_nPos,
                                                      servos[3].m_nPos,
                                                      servos[4].m_nPos );

  return ret;
}


/*!
 *  \brief set servo values based on current m_bestJoints
 */
bool SMBotPlanner::setOffsets(DynaPosTuple_T servos[], uint_t nServos)
{
  for ( int i = 0; i < nServos; i++ ) 
  {
    m_servoOffsets[i] = servos[i].m_nPos;
  }

  return true;
}


void SMBotPlanner::rawToDeg(DynaPosTuple_T servoRaw[], 
                            DynaRealTuple_T      servoDeg[], 
                            int                  nNumServos)
{
  static double servo2deg1 = 0.29;
  static double servo2deg2 = 0.06;
  int   i;

  for(i=0; i<nNumServos; ++i)
  {
    switch( servoRaw[i].m_nServoId )
    {
      case HEK_SERVO_ID_BASE:
        servoDeg[0].nServoId = servoRaw[i].m_nServoId;
        servoDeg[0].fVal = (servoRaw[i].m_nPos - m_servoOffsets[0]) * servo2deg1
                            / m_servoGearRatios[0] * m_servoDirections[0];
        break;
      case HEK_SERVO_ID_SHOULDER_L:
        servoDeg[1].nServoId = servoRaw[i].m_nServoId;
        servoDeg[1].fVal = (servoRaw[i].m_nPos - m_servoOffsets[1]) * servo2deg2
                            / m_servoGearRatios[1] * m_servoDirections[1];
        break;
      case HEK_SERVO_ID_ELBOW:
        servoDeg[2].nServoId = servoRaw[i].m_nServoId;
        servoDeg[2].fVal = (servoRaw[i].m_nPos - m_servoOffsets[2]) * servo2deg1
                            / m_servoGearRatios[2] * m_servoDirections[2];
        break;
      case HEK_SERVO_ID_WRIST_ROT:
        servoDeg[3].nServoId = servoRaw[i].m_nServoId;
        servoDeg[3].fVal = (servoRaw[i].m_nPos - m_servoOffsets[3]) * servo2deg1
                            / m_servoGearRatios[3] * m_servoDirections[3];
        break;
      case HEK_SERVO_ID_WRIST_PITCH:
        servoDeg[4].nServoId = servoRaw[i].m_nServoId;
        servoDeg[4].fVal = (servoRaw[i].m_nPos - m_servoOffsets[4]) * servo2deg1
                            / m_servoGearRatios[4] * m_servoDirections[4];
        break;
      case HEK_SERVO_ID_GRABOID:
        // RDK fix me
        servoDeg[5].nServoId = servoRaw[i].m_nServoId;
        servoDeg[5].fVal = servoRaw[i].m_nPos * servo2deg1;
        break;
      case HEK_SERVO_ID_SHOULDER_R:
      default:
        break;
    }
  }
}
