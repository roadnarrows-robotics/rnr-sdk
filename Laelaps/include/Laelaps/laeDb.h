////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// File:      laeDb.h
//
/*! \file
 *
 * $LastChangedDate: 2015-12-28 10:06:00 -0700 (Mon, 28 Dec 2015) $
 * $Rev: 4247 $
 *
 * \brief Laelaps real-time "database". 
 *
 * The Laelaps library use a single instance of this DB to export library global
 * data. No concurrency control is exercised. For each DB datam, there should
 * only be one writer (producer) and that the writes are atomic enough. Multiple
 * readers can access the data.
 *
 * A more robust, fullier featured database may be developed at some future
 * date, if needed.
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

#ifndef _LAE_DB_H
#define _LAE_DB_H

#include "rnr/rnrconfig.h"

#include  "Laelaps/laelaps.h"
#include  "Laelaps/laeSysDev.h"
#include  "Laelaps/laeMotor.h"
#include  "Laelaps/laeImu.h"
#include  "Laelaps/laeAlarms.h"

/*!
 *  \brief The \h_laelaps namespace encapsulates all \h_laelaps related
 *  constructs.
 */
namespace laelaps
{
  /*!
   * \brief Product data.
   */
  struct LaeDbProduct
  {
    int     m_eProdId;          ///< base product id
    uint_t  m_uProdHwVer;       ///< product hardware version number
    uint_t  m_uMotorCtlrFwVer[LaeNumMotorCtlrs];
                                ///< motor controllers firmware versions
    uint_t  m_uImuFwVer;        ///< IMU sub-processor firmware version
    uint_t  m_uWatchDogFwVer;   ///< watchdog sub-processor firmware version
    uint_t  m_uToFMuxFwVer;     ///< tof mux sub-processor firmware version
  }; // LaeDbProduct

  /*!
   * \brief Robot top-level status.
   */
  struct LaeDbRobotStatus
  {
    bool            m_bIsConnected;     ///< critical hardware [not] connected
    LaeRobotMode    m_eRobotMode;       ///< robot operating mode
    bool            m_bIsEStopped;      ///< robot is [not] emergency stopped
    bool            m_bAlarmState;      ///< robot is [not] alarmed
    bool            m_bAreMotorsPowered;///< robot motors are [not] driven
    bool            m_bInMotion;        ///< 1+ motors are [not] moving
    double          m_fTempAvg;         ///< average interior temperature
  };

  struct LaeDbGpio
  {
    bool    m_bMotorCtlrEn;     ///< motor controller enable
    bool    m_bAuxPortBattEn;   ///< battery auxilliary port enable
    bool    m_bAuxPort5vEn;     ///< 5 volt auxilliary port enable
  };

  /*!
   * \brief Motor controller data.
   */
  struct LaeDbMotorCtlr
  {
    u32_t   m_uStatus;            ///< status bits
    double  m_fTemperature;       ///< sensed temperature (C)
    double  m_fBatteryVoltage;    ///< sensed battery voltage (V)
    double  m_fMotorCurrent[LaeNumMotorsPerCtlr]; ///< motor currents (A)
  }; // LaeDbMotorCtlr

  /*!
   * \brief Range sensor data.
   */
  struct LaeDbRange
  {
    double  m_fRange;             ///< range (m)
    double  m_fAmbientLight;      ///< ambient light (lux)
  };

  /*!
   * \brief IMU data.
   */
  struct LaeDbImu
  {
    double  m_accel[sensor::imu::NumOfAxes];  ///< accelerometer data (m/s^2)
    double  m_gyro[sensor::imu::NumOfAxes];   ///< gyroscope data (radians/s)
    double  m_rpy[sensor::imu::NumOfAxes];    ///< vehicle roll,pitch,yaw (rads)
  };

  /*!
   * \brief Powertrain kinodynamics data.
   */
  struct LaeDbKinPowertrain
  {
    s64_t   m_nEncoder;       ///< motor encoder position (quad pulses)
    int     m_nSpeed;         ///< raw speed (qpps)
    double  m_fPosition;      ///< wheel angular position (radians)
    double  m_fVelocity;      ///< wheel angular velocity (radians/second)
    double  m_fPe;            ///< motor input electrical power (W)
    double  m_fTorque;        ///< wheel torque (N-m)
  };

  /*!
   * \brief Robot platform kinodynamics data.
   */
  struct LaeDbKinRobot
  {
    double  m_x;            ///< robot absolute x position (meters)
    double  m_y;            ///< robot absolute y position (meters)
    double  m_theta;        ///< robot orientation (radians)
    double  m_fOdometer;    ///< robot odometer (meters)
    double  m_fVelocity;    ///< robot velocity (meters/second)
  };

  /*!
   * \brief Kinodynamics data.
   */
  struct LaeDbKin
  {
    LaeDbKinPowertrain  m_powertrain[LaeMotorsNumOf];
                                    ///< powertrain kinodynamics
    LaeDbKinRobot       m_robot;    ///< robot kinodynmics
  };

  /*!
   * \brief Energy monitoring data.
   */
  struct LaeDbEnergy
  {
    bool    m_bBatteryIsCharging; ///< batteries are [not] being charged
    double  m_fJackVoltage;       ///< sensed power supply jack voltage
    double  m_fBatteryVoltage;    ///< sensed battery subsystem voltage (V)
    double  m_fBatterySoC;        ///< estimated battery state of charge (%)
    double  m_fTotalCurrent;      ///< estimated total system current draw (A)
    double  m_fTotalPower;        ///< estimated total system power (watts)
  }; // LaeDbEnergy

  /*!
   * \brief System and subsystem alarms and warnings data tree.
   */
  struct LaeDbAlarms
  {
    LaeAlarmInfo  m_system;                 ///< system alarm summary state
    LaeAlarmInfo  m_battery;                ///< battery subsystem alarm state
    LaeAlarmInfo  m_motorctlr[LaeNumMotorCtlrs];
                                            ///< motor contollers subsytem state
    LaeAlarmInfo  m_motor[LaeMotorsNumOf];  ///< motors subsytem state
  }; // LaeDbAlarms

  /*!
   * \brief Simple real-time database structure.
   */
  struct LaeDb
  {
    u32_t             m_uDbVer;           ///< database version
    LaeDbProduct      m_product;          ///< product data
    LaeDbRobotStatus  m_robotstatus;      ///< robot status data
    LaeDbGpio         m_gpio;             ///< key gpio state data
    LaeDbMotorCtlr    m_motorctlr[LaeNumMotorCtlrs];
                                          ///< motor controller and motor data
    LaeDbRange        m_range[ToFSensorMaxNumOf];
                                          ///< range sensors
    LaeDbImu          m_imu;              ///< imu
    LaeDbKin          m_kin;              ///< kinodynamics
    LaeDbEnergy       m_energy;           ///< battery and energy data
    LaeDbAlarms       m_alarms;           ///< alarm state data
  }; // LaeDb

  /*!
   * \brief The real-time database.
   */
  extern LaeDb  RtDb;

} // namespace laelaps


#endif // _LAE_DB_H
