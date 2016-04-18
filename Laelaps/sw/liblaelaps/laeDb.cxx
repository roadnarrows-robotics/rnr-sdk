////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Library:   liblaelaps
//
// File:      laeDb.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-12-28 10:06:00 -0700 (Mon, 28 Dec 2015) $
 * $Rev: 4247 $
 *
 * \brief Laelaps real-time "database" implementation. 
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2015-2016  RoadNarrows
 * (http://www.RoadNarrows.com)
 * \n All Rights Reserved
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

#include "rnr/rnrconfig.h"

#include  "Laelaps/laelaps.h"
#include  "Laelaps/laeSysDev.h"
#include  "Laelaps/RoboClaw.h"
#include  "Laelaps/laeMotor.h"
#include  "Laelaps/laeVL6180.h"
#include  "Laelaps/laeImu.h"
#include  "Laelaps/laeAlarms.h"
#include  "Laelaps/laeTune.h"
#include  "Laelaps/laeDb.h"

using namespace std;
using namespace laelaps;
using namespace motor::roboclaw;
using namespace sensor::vl6180;

//
// The real-time database.
//
LaeDb  laelaps::RtDb = 
{
  LAE_VERSION(1, 0, 0),   // db version keep first

  // product
  {
    LaeProdIdUnknown,       // RoadNarrows product id
    LAE_VERSION(0, 0, 0),   // hardware version
    {0, 0},                 // motor controllers firmware version
    0,                      // IMU firmware version
    0,                      // watchdog sub-processor firmware version
    0,                      // time-of-flight mux sub-processor firmware version
  },

  // robot top-level status
  {
    false,                // critical hardware [not] connected
    LaeRobotModeUnknown,  // robot mode
    false,                // robot is [not] emergency stopped
    false,                // robot is [not] alarmed
    false,                // robot motors are [not] driven
    false,                // 1+ motors are [not] moving
    0.0                   // average interior temperature (C)
  },

  // gpio
  {
    false,        // motor controllers enable
    false,        // battery aux. port enable
    false         // 5V aux. port enable
  },

  // motor controllers 
  {
    // front
    {
      ParamStatusNormal,    // status
      0.0,                  // temperature
      0.0,                  // battery voltage
      {0.0, }               // motor currents
    },
    // rear
    {
      ParamStatusNormal,    // status
      0.0,                  // temperature
      0.0,                  // battery voltage
      {0.0, }               // motor currents
    }
  },

  // range sensors
  {
    {VL6180X_RANGE_NO_OBJ, VL6180X_AMBIENT_MIN}, 
  },

  // imu
  {
    {0.0, },          // acceleration
    {0.0, },          // gyroscope
    {0.0, },          // roll, pitch, yaw
  },

  // kinodynamics 
  {
    { {0, 0, 0.0, 0.0, 0.0, 0.0}, },    // powertrain kinodynamics
    {0.0, 0.0, 0.0, 0.0, 0.0}           // robot kinodynamics
  },

  // energy monitoring
  {
    false,                  // battery is not being charged
    0.0,                    // power supply jack voltage
    LaeTuneBattNominalV,    // battery voltage
    100.0,                  // battery state of charge
    0.1,                    // total robot current draw
    0.5,                    // total robot power
  },

  // alarms
  {
    // system alarms
    {false, LAE_ALARM_NONE, LAE_WARN_NONE},

    // battery alarms
    {false, LAE_ALARM_NONE, LAE_WARN_NONE},

    // motor controller alarms
    { {false, LAE_ALARM_NONE, LAE_WARN_NONE}, },

    // motor alarms
    { {false, LAE_ALARM_NONE, LAE_WARN_NONE}, }
  }
};
