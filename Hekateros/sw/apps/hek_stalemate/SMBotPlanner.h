////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Program:   StaleMate
//
// File:      SMBotPlanner.h
//
/*! \file
 *
 * $LastChangedDate: 2012-06-05 15:17:26 -0600 (Tue, 05 Jun 2012) $
 * $Rev: 2028 $
 *
 * \brief  StaleMate kinematic planning utils declarations. 
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

#ifndef _SMBOTPLANNER_H
#define _SMBOTPLANNER_H

#include <time.h>
#include <string.h>

#include <fstream>

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "rnr/rnrWin.h"
#include "rnr/rnrWinOpenCv.h"
#include "rnr/rnrWinMenu.h"
#include "rnr/rnrWinIoI.h"

#include "Dynamixel/Dynamixel.h"

#include "Hekateros/Hekateros.h"


//------------------------------------------------------------------------//
//                          Planning Constants 
//------------------------------------------------------------------------//

/*!
 *  \brief Center of joints
 */ 
static double center [] = {0, 0, 0, 0, 0};

/*!
 *  \brief half-ranges of motion for each joint
 */ 
static double ranges[] = {90, 90, 90, 90, 90};

//------------------------------------------------------------------------//
//                          Planning Helper Classes 
//------------------------------------------------------------------------//

/*!
 * \brief Q helps manages the search space for the pyramid planner
 */
class Q
{
public:
  Q(int depth, double cent[], double delta[])
  {
    for(int i = 0; i < 5; i++)
    {
      d[i] = delta[i]/pow(2, depth+1);
    }

    for(int i = 0; i < 32; i++)
    {
      for(int j = 0; j < 3; j++)
      {
        pts[i][j]  = cent[j];
        pts[i][j] += d[j] * pow(-1, i/(int)(pow(2,j)));
      }
      pts[i][3] = 180 - pts[i][2] - pts[i][1];
      pts[i][4] = -1 * pts[i][0];
    }
  }

  ~Q()
  {
  }

  void printQ()
  {
  }

  double d[5];          ///< delta from the center point
  double pts[32][5];    ///< pts to be tested
}; 


//--------------------------------------------------------------------------//
//                            SMBotPlanner Class                            //
//--------------------------------------------------------------------------//

/*!
 * SMBotPlanner and Data Class
 */
class SMBotPlanner
{
public:
  SMBotPlanner(double epsilon = 15)
  {
    m_epsilon = epsilon;
    initRobotHek91();
    setGoal(0,0,0);
  }

  ~SMBotPlanner()
  {
    
  }

  //----------------------------------------------------------------------//
  //                  Physical Characteristics of Robot                   //
  //----------------------------------------------------------------------//
  double               m_offsetX; ///< X offset of shoulder joint wrto base axis
  double               m_offsetY; ///< Y offset of shoulder joint wrto base axis
  double               m_offsetZ; ///< Z offset of shoulder joint wrto table

  double               m_links[3];           // arm link lengths [mm]

  double               m_servoOffsets[6];    // joint offsets for servo land
  double               m_servosMin[6];       // min servo vals
  double               m_servosMax[6];       // max servo vals
  double               m_servoGearRatios[6]; // gear ratios for servo land
  int                  m_servoDirections[6]; // joint direction for servo land

  void initRobotHek90();
  void initRobotHek91();
  void initRobotXml(const char* filename);


  //----------------------------------------------------------------------//
  //                           Planning Data                              //
  //----------------------------------------------------------------------//
  int               m_maxdepth;     ///< maximum pyramid planning depth
  CvPoint3D32f      m_goal;         ///< planning goal, xyz
  double m_epsilon;             

  double            m_bestJoints[5];  ///< current best joint angles
  CvPoint3D32f      m_bestPlan;       ///< curent best planned position 

  double            m_distance2goal;  ///< current shortest distance to goal

  void setGoal(double x, double y, double z); 
  void setGoal(const CvPoint3D32f& goal); 

  //----------------------------------------------------------------------//
  //                           Planning Algorithms                        // 
  //----------------------------------------------------------------------//
  double PyramidPlanner( CvPoint3D32f goal, 
                         DynaPosTuple_T servos[], 
                         uint_t nServos,
                         int maxdepth = 10);

  double ConstrainedPlanner( CvPoint3D32f goal, 
                             DynaPosTuple_T servos[], 
                             uint_t nServos );

  bool   setServos(DynaPosTuple_T servos[], uint_t nServos);
  bool   setOffsets(DynaPosTuple_T servos[], uint_t nServos);

  double r(int depth, Q ss);
  
  double distance2goal(double joints[]);

  //----------------------------------------------------------------------//
  //                       Coordinate transformations                     //
  //----------------------------------------------------------------------//

  // internal planning joint angles to Real World Coordinates 
  CvPoint3D32f          Angles2RealWorld(double joints[]);

  void rawToDeg(DynaPosTuple_T  servoRaw[], 
                DynaRealTuple_T servoDeg[], 
                int             nNumServos);
};

#endif // _SMBOTPLANNER_H
