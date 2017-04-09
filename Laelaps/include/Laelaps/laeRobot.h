////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelasp
//
// Library:   liblaelaps
//
// File:      laeRobot.h
//
/*! \file
 *
 * $LastChangedDate: 2016-04-11 13:03:57 -0600 (Mon, 11 Apr 2016) $
 * $Rev: 4381 $
 *
 * \brief Laelasp Robot Class interface.
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

#ifndef _LAE_ROBOT_H
#define _LAE_ROBOT_H

#include <pthread.h>

#include <string>
#include <map>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/units.h"
#include "rnr/i2c.h"

// common
#include "Laelaps/laelaps.h"
#include "Laelaps/laeUtils.h"

// hardware
#include "Laelaps/laeSysDev.h"
#include "Laelaps/RoboClaw.h"
#include "Laelaps/laeMotor.h"
#include "Laelaps/laeGpio.h"
#include "Laelaps/laeI2C.h"
#include "Laelaps/laeI2CMux.h"
#include "Laelaps/laeWatchDog.h"
#include "Laelaps/laeWd.h"
#include "Laelaps/laeVL6180.h"
#include "Laelaps/laeImu.h"
#include "Laelaps/laeCams.h"

// descriptions and tuning
#include "Laelaps/laeTune.h"
#include "Laelaps/laeDesc.h"

// control and application interface
#include "Laelaps/laeTraj.h"
#include "Laelaps/laePowertrain.h"
#include "Laelaps/laePlatform.h"
#include "Laelaps/laeKin.h"
#include "Laelaps/laeReports.h"
#include "Laelaps/laeThread.h"
#include "Laelaps/laeThreadAsync.h"
#include "Laelaps/laeThreadImu.h"
#include "Laelaps/laeThreadKin.h"
#include "Laelaps/laeThreadRange.h"
#include "Laelaps/laeThreadWd.h"

namespace laelaps
{
  typedef std::vector<sensor::vl6180::LaeVL6180Mux*> VecToFSensors;

  /*!
   * \brief Laelaps robotic manipulator plus accesories class.
   */
  class LaeRobot
  { 
  public:
    static const double GovernorDft;  ///< speed limit governor start-up default

    /*!
     * \brief Default initialization constructor.
     *
     * \param bNoExec Do [not] execute on physical hardware. All commands
     *                and responses are supported but the lower level hardware
     *                commands will not be issued.
     */
    LaeRobot(bool bNoExec=false);

    /*!
     * \brief Destructor.
     */
    virtual ~LaeRobot();


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Communication and Robot Initialization Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Connect to \h_laelaps.
     *
     * \copydoc doc_return_std
     */
    int connect();

    /*!
     * \brief Disconnect from \h_laelaps.
     *
     * \copydoc doc_return_std
     */
    int disconnect();

    /*!
     * \brief Reload \h_laelaps's reloadable configuration and reset operational
     * parameters.
     *
     * The robot connection and calibration states are uneffected.
     */
    int reload();


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Action Methods.
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Emergency stop.
     *
     * All motor will stop driving, so arm or accessories may fall.
     *
     * \copydoc doc_return_std
     */
    int estop();

    /*!
     * \brief Reset (clears) emergency stop condition.
     *
     * \note Motors are not re-powered until an move or freeze action is called.
     */
    int resetEStop();

    /*!
     * \brief Freeze robot and accessories at current position.
     *
     * Motors are still being driven. The motors are either dynamically or 
     * regeneratively braked.
     *
     * \copydoc doc_return_std
     */
    int freeze();

    /*!
     * \brief Release robot and accessories.
     *
     * Motors will stop being driven. The robot will coast to a stop (on a flat
     * surface) and is capabable of being externally pushed.
     *
     * \copydoc doc_return_std
     */
    int release();

    /*!
     * \brief Stop robot with full dynamic braking.
     *
     * \copydoc doc_return_std
     */
    int stop();

    /*!
     * \brief Move by setting powertrain angular wheel velocities.
     *
     * The robot is controlled by setting the goal velocities of a [sub]set of 
     * powertrain motors.
     *
     * \param velocity  Map of powertrain velocities.\n
     *                  Keys are left_front, right_front, left_rear, and
     *                  right_rear.\n
     *                  Values are drive shaft/wheel angular velocities
     *                  (radians/second).
     *
     * \copydoc doc_return_std
     */
    int move(const LaeMapVelocity &velocity);

    /*!
     * \brief Move at the given linear and angular velocities.
     *
     * This move is typically used with the ROS Twist message where:
     *  - linear velocity is the x component Twist.linear.x
     *  - angular velocity is the yaw component Twist.angular.z
     *
     * \param fVelLinear  Linear velocity (meters/second).
     * \param fVelAngular Angular velocity (radians/second).
     *
     * \copydoc doc_return_std
     */
    int move(double fVelLinear, double fVelAngular);

    /*!
     * \brief Set powertrain motor duty cycles.
     *
     * The robot is controlled by setting the goal duties of a [sub]set of 
     * powertrain motors.
     *
     * \param duty  Map of powertrain duty cycles.\n
     *              Keys are left_front, right_front, left_rear, and
     *                  right_rear.\n
     *              Values are normalized duty cycles [-1.0, 1.0].
     *
     * \copydoc doc_return_std
     */
    int setDutyCycles(const LaeMapDutyCycle &duty);

    /*!
     * \brief Set speed limit governor value.
     *
     * \note Software governor is not implemented as yet.
     *
     * Governor is defined as:\n
     * speed = cap(set_speed, min_speed * governor, max_speed * governor)
     *
     * \param fGovernor   Governor value between [0.0, 1.0].
     *
     * \return Returns new governor value.
     */
    double setGovernor(double fGovernor);

    /*!
     * \brief Increment/decrement speed limit governor value.
     *
     * \note Software governor is not implemented as yet.
     *
     * Governor is defined as:\n
     * speed = set_speed * governor
     *
     * \param fDelta  Governor \h_plusmn delta.
     *
     * \return Returns new governor value.
     */
    double incrementGovernor(double fDelta);

    /*!
     * \brief Get current speed limit governor setting.
     *
     * \note Software governor is not implemented as yet.
     *
     * \return Return value.
     */
    double getGovernor();

    /*!
     * \brief Set robot's operational mode.
     *
     * \param eRobotMode Robot operation mode. See \ref LaeRobotMode.
     */
    void setRobotMode(LaeRobotMode eRobotMode);

    /*!
     * \brief Set top deck auxilliary power out enable state.
     *
     * \param strName   Aux. power port name. One of: aux_batt_en aux_5v_en
     * \param eState    Aux. power state. One of: DISABLED(0) ENABLED(1)
     *
     * \copydoc doc_return_std
     */
    int setAuxPower(const std::string &strName, LaeTriState eState);

    /*!
     * \brief Attempt to clear all alarms.
     *
     * \note For Laelaps, most alarms are not clearable (e.g. temperature,
     * low battery). Only external intervention is effective.
     *
     * \copydoc doc_return_std
     */
    int clearAlarms();

    /*!
     * \brief Configure a digital pin on the the watchdog subprocessor.
     *
     * \param pin   Digital pin number.
     * \param dir   Pin direction. 0 == input, 1 == output.
     *
     * \copydoc doc_return_std
     */
    virtual int configDigitalPin(uint_t pin, uint_t dir);

    /*!
     * \brief Read the value of a digital pin on the watchdog subprocessor.
     *
     * \param pin         Digital pin number.
     * \param [out] val   Digital pin low (0) or high (1) value.
     *
     * \copydoc doc_return_std
     */
    virtual int readDigitalPin(uint_t pin, uint_t &val);

    /*!
     * \brief Write a value to a digital pin on the watchdog subprocessor.
     *
     * \param pin     Digital pin number.
     * \param val     Digital pin low (0) or high (1) value.
     *
     * \copydoc doc_return_std
     */
    virtual int writeDigitalPin(uint_t pin, uint_t val);

    /*!
     * \brief Read the value of an analog pin on the watchdog subprocessor.
     *
     * \param pin         Analog pin number.
     * \param [out] val   Analog 10-bit value [0-1023].
     *
     * \copydoc doc_return_std
     */
    virtual int readAnalogPin(uint_t pin, uint_t &val);

    /*!
     * \brief Get the last read and converted inertia data.
     *
     * Not applicable data are set to zero.
     *
     * \param [out] accel   Accelerometer data (m/s^2).
     *                      The array size must be \h_ge \ref NumOfAxes.
     * \param [out] gyro    Gyroscope data (radians/s).
     *                      The array size must be \h_ge \ref NumOfAxes.
     * \param [out] rpy     Roll,pitch,yaw data (radians).
     *                      The array size must be \h_ge \ref NumOfAxes.
       * \param [out] q     Vehicle quaternion.
     *
     * \copydoc doc_return_std
     */
     virtual int getImu(double accel[],
                        double gyro[],
                        double rpy[],
                        sensor::imu::Quaternion &q);

    /*!
     * \brief Get range sensor properties.
     *
     * \param strKey                 Sensor's unique name id (key).
     * \param [out] strRadiationType Radiation type. 
     * \param [out] fFoV             Field of View (radians).
     * \param [out] fBeamdir         Center of beam direction (radians).
     * \param [out] fMin             Minimum range (meters).
     * \param [out] fMax             Maximum range (meters).
     *
     * \copydoc doc_return_std
     */
    virtual int getRangeSensorProps(const std::string &strKey,
                                    std::string       &strRadiationType,
                                    double            &fFoV,
                                    double            &fBeamDir,
                                    double            &fMin,
                                    double            &fMax);

    /*!
     * \brief Get a range measurement.
     *
     * \param strKey        Sensor's unique name (key).
     * \param [out] fRange  Sensed object range (meters).
     *
     * \copydoc doc_return_std
     */
    virtual int getRange(const std::string &strKey, double &fRange);

    /*!
     * \brief Get all sensor range measurements.
     *
     * \param [out] vecNames    Vector of sensor unique names.
     * \param [out] vecRanges   Vector of associated sensor measured ranges
     *                          (meters).
     *
     * \copydoc doc_return_std
     */
    virtual int getRange(std::vector<std::string> &vecNames,
                         std::vector<double>      &vecRanges);

    /*!
     * \brief Get an ambient light illuminance measurement.
     *
     * \param strKey          Sensor's unique name (key).
     * \param [out] fAmbient  Sensed ambient light (lux).
     *
     * \copydoc doc_return_std
     */
    virtual int getAmbientLight(const std::string &strKey, double &fAmbient);

    /*!
     * \brief Get all sensor ambient light illuminance measurements.
     *
     * \param [out] vecNames    Vector of sensor unique names.
     * \param [out] vecAmbient  Vector of associated sensor measured ambients
     *                          (lux).
     *
     * \copydoc doc_return_std
     */
    virtual int getAmbientLight(std::vector<std::string> &vecNames,
                                std::vector<double> &vecAmbient);

    /*!
     * \brief Run asynchronous job.
     *
     * The new job object must be already created.
     *
     * \par Context:
     * Calling thread.
     *
     * \copydoc doc_return_std
     */
    int runAsyncJob();

    /*!
     * \brief Cancel any asynchronous job.
     *
     * \note There may be a little delay between canceling an async job
     * and the job actually stopping.
     */
    void cancelAsyncJob();

    /*!
     * \brief Get the current asynchronous job state.
     *
     * \return Job state enum value.
     */
    LaeAsyncJob::JobState getAsyncJobState();
    
    /*!
     * \brief Get the last asynchronous job exit return code.
     *
     * \return
     * Returns LAE_OK when job terminated successfully.
     * Otherwise, returns \h_lt 0 \ref laelaps_ecodes.
     */
    int getAsyncJobRc();


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Reports
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Get the robot current status.
     *
     * \param [in,out] rptBotStatus   Robot status report.
     *
     * \copydoc doc_return_std
     */
    int getRobotStatus(LaeRptRobotStatus &rptBotStatus);

    /*!
     * \brief Get the robot full dynamic state.
     *
     * \param [in,out] rptDynamics  Robot dynamics state report.
     *
     * \copydoc doc_return_std
     */
    int getDynamics(LaeRptDynamics &rptDynamics);

    /*!
     * \brief Get simple navigation feedback.
     *
     * \param [in,out] pathFeedback  Navigation feedback state.
     *
     * \copydoc doc_return_std
     */
    int getNavigationState(LaeSimplePathFeedback &pathFeedback);

    /*!
     * \brief Get navigation feedback.
     *
     * \param [in,out] pathFeedback  Navigation feedback state.
     *
     * \copydoc doc_return_std
     */
    int getNavigationState(LaePathFeedback &pathFeedback);


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Attribute Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Test if connected to \h_laelaps hardware.
     *
     * \return Returns true or false.
     */
    bool isConnected();

    /*!
     * \brief Get the \h_laelaps hardware version number.
     *
     * \param [out] nVerMajor     Major version number.
     * \param [out] nVerMinor     Minor version number.
     * \param [out] nVerRevision  Revision version number.
     */
    void getVersion(int &nVerMajor, int &nVerMinor, int &nRevision);

    /*!
     * \brief Get the \h_laelaps robotic arm hardware version string.
     *
     * Version number strings are of the dotted form maj.min.rev.
     *
     * \return Version string.
     */
    std::string getVersionString();

    /*!
     * \brief Get the \h_laelaps product description.
     *
     * \return Returns reference to description.
     */
    LaeDesc &getLaelapsDesc();

    /*!
     * \brief Convenience function to get this \h_laelaps description's base
     * product id.
     *
     * \return Returns product id. See \ref LaeProdId.
     */
    int getProdId();
  
    /*!
     * \brief Convenience function to get this \h_laelaps description's base
     * product name.
     *
     * \return Returns product name. See \ref LaeProdName.
     */
    std::string getProdName();
  
    /*!
     * \brief Get the \h_laelaps full brief descirption.
     *
     * \return Returns product brief description.
     */
    std::string getFullProdBrief();
  
    /*!
     * \brief Test if robot is fully described via configuration XML.
     *
     * \return Returns true or false.
     */
    bool isDescribed();

    /*!
     * \brief Get robot's operational mode.
     *
     * \return Robot operation mode. See \ref LaeRobotMode.
     */
    LaeRobotMode getRobotMode();

    /*!
     * \brief Test if robot is current emergency stopped.
     *
     * \return Returns true or false.
     */
    bool isEStopped();

    /*!
     * \brief Test if robot motor are currently being driven (powered).
     *
     * \return Returns true or false.
     */
    bool areMotorsPowered();

    /*!
     * \brief Test if any joint in any of the kinematic chains is moving.
     *
     * \note RS160D motor controller provide no feedback.
     *
     * \return Returns true or false.
     */
    bool isInMotion();

    /*!
     * \brief Test if robot is alarmed.
     *
     * \return Returns true or false.
     */
    bool isAlarmed();

    /*!
     * \brief Test if robot is safe to operate, given the current robot and
     * alarm state.
     *
     * \return Returns true or false.
     */
    bool canMove();


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Kinematic Access and Mapping Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Get the robotic joint in kinematic chain.
     *
     * \param strName   Powertrain unique name (primary key).
     *
     * \return If found, returns pointer to joint. Otherwise returns NULL.
     */
    LaePowertrain *getPowertrain(const std::string &strName);

    /*!
     * \brief Get robotic joint in kinematic chain.
     *
     * \param nMotorId  Motor id (secondary key).
     *
     * \return If found, returns pointer to joint. Otherwise returns NULL.
     */
    LaePowertrain *getPowertrain(int nMotorId);

  protected:
    // state
    bool            m_bNoExec;          ///< do [not] execute physical movements
    LaeDesc         m_descLaelaps;      ///< \h_laelaps description
    bool            m_bIsConnected;     ///< critical hardware [not] connected
    LaeRobotMode    m_eRobotMode;       ///< robot operating mode
    bool            m_bIsEStopped;      ///< robot is [not] emergency stopped
    bool            m_bAlarmState;      ///< robot is [not] alarmed
    double          m_fGovernor;        ///< speed limit governor setting

    // tuning
    LaeTunes        m_tunes;            ///< tuning parameters

    // auxilliary power enables (moved to watchdog)
    //LaeAuxBattOutEnable  m_powerBatt; ///< top deck battery power enable
    //LaeAux5VOutEnable    m_power5V;   ///< top deck regulated 5V power enable

    // I2C sensors, IMU, and watchdog
    LaeI2C                          m_i2cBus;   ///< \h_i2c sensor bus
    sensor::vl6180::LaeRangeSensorGroup m_range; ///< range sensor group.
    sensor::imu::LaeImuCleanFlight  m_imu;      ///< inertia measurement unit
    LaeWd                           m_watchdog; ///< watchdog sub-processor

    // kinodynamics
    LaeKinematics     m_kin;            ///< robot base dynamics and kinematics

    // threads
    LaeThreadAsync    m_threadAsync;    ///< asynchronous action thread
    LaeThreadImu      m_threadImu;      ///< IMU thread
    LaeThreadKin      m_threadKin;      ///< kinodynamics thread
    LaeThreadRange    m_threadRange;    ///< ToF range sensors thread
    LaeThreadWd       m_threadWatchDog; ///< watchdog thread

    // asynchronous job control
    LaeAsyncJob      *m_pAsyncJob;      ///< asynchronous job

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Database Methods.
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Synchronize real-time database with current robot state.
     */
    void syncDb();


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Hardare Connection and Configuration Methods.
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Connect to the \h_laelaps built-in sensors.
     *
     * Sensors are not critical to core operation, so failed sensors are 
     * simply blacklisted, rather than failing the robot.
     *
     * \par Sensors:
     * * IMU
     * * ToF infrared distance sensors
     *
     * \copydoc doc_return_std
     */
    int connSensors();

    /*!
     * \brief Connect to the watchdog subprocessor.
     *
     * \copydoc doc_return_std
     */
    int connWatchDog();

    /*!
     * \brief Connect to the \h_laelaps motor controllers
     *
     * Motors controller serial interface support multi-drop, so one serial
     * device can support up to 8 motor controllers.
     *
     * \param strDevMotorCtlrs    Motor controllers serial device name.
     * \param nBaudRate           Motor controllers serial baudrate.
     *
     * \copydoc doc_return_std
     */
    int connMotorControllers(const std::string &strDevMotorCtlrs,
                             const int         nBaudRate);

    /*!
     * \brief Configure \h_laelaps for normal operation.
     *
     * The arm, end effector, and accessory effectors are configured for
     * normal operation.
     *
     * \copydoc doc_return_std
     */
    int configForOperation();


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Threads 
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Create and start all real-time persistent core threads.
     *
     * \copydoc doc_return_std
     */
    int startCoreThreads();

    /*!
     * \brief Create and start a thread at the given priority and hertz.
     *
     * \param pThread   Pointer the thread object.
     * \param nPriority Thread priority.
     * \param fHz       Thread execution hertz.
     *
     * \copydoc doc_return_std
     */
    int startThread(LaeThread *pThread, int nPriority, double fHz);


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Friends 
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    friend class LaeRptDynamics;
    friend class LaeRptRobotStatus;

  }; // class LaeRobot

} // namespace laelaps


#endif // _LAE_ROBOT_H
