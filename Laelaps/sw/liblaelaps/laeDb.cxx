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
 * \par Copyright
 *   \h_copy 2015-2017. RoadNarrows LLC.\n
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

#include "rnr/rnrconfig.h"

#include  "Laelaps/laelaps.h"
#include  "Laelaps/laeSysDev.h"
#include  "Laelaps/RoboClaw.h"
#include  "Laelaps/laeMotor.h"
#include  "Laelaps/laeVL6180.h"
#include  "Laelaps/laeImu.h"
#include  "Laelaps/laeAlarms.h"
#include  "Laelaps/laeTune.h"
#include  "Laelaps/laeThread.h"
#include  "Laelaps/laeThreadWd.h"
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
  // header
  {0, },                  // signature (keep first)
  LAE_VERSION(1, 2, 0),   // db version

  // product
  {
    LaeProdIdUnknown,     // RoadNarrows product id
    LAE_VERSION(0, 0, 0), // hardware version
    {0, 0},               // motor controllers firmware version
    0,                    // IMU firmware version
    0,                    // watchdog sub-processor firmware version
    0,                    // time-of-flight mux sub-processor firmware version
    0                     // accessory sub-processor firmware version
  },

  // configuration
  {
    LaeTuneThreadKinHzDft,      ///< kinodynamics thread exec rate (Hertz)
    LaeTuneThreadImuHzDft,      ///< imu thread exec rate (Hertz)
    LaeTuneThreadRangeHzDft,    ///< range thread exec rate (Hertz)
    LaeThreadWd::ThreadWdHzDft, ///< watchdog thread exec rate (Hertz)
    LaeTuneWdTimeoutDft,        ///< watchdog timeout (seconds)
    LaeTuneVelDerateDft         ///< velocity derate (percent)
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

  // enables
  {
    false,        // motor controllers enable
    false,        // battery aux. port enable
    false,        // 5V aux. port enable
    false,        // Range sensors enable
    false,        // IMU sensor enable
    true          // front camera enable
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
    {false, LAE_ALARM_NONE, LAE_WARN_NONE},

    // motor alarms
    {false, LAE_ALARM_NONE, LAE_WARN_NONE},

    // sensor alarms
    {false, LAE_ALARM_NONE, LAE_WARN_NONE}
  }
};
