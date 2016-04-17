////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   libDynamixel
//
// File:      DynaServoGeneric.h
//
/*! \file
 *
 * $LastChangedDate: 2015-01-12 11:54:07 -0700 (Mon, 12 Jan 2015) $
 * $Rev: 3846 $
 *
 * \ingroup dyna_lib_hdrs
 *
 * \brief Generic Dynamixel Servo Base Class Interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011-2015.  RoadNarrows LLC.
 * (http://www.roadnarrows.com)
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

#ifndef _DYNA_SERVO_GENERIC_H
#define _DYNA_SERVO_GENERIC_H

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaError.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaServo.h"


// ---------------------------------------------------------------------------
// Generic Dynamixel Servo Base Class
// ---------------------------------------------------------------------------

/*!
 * \ingroup dyna_lib_classes
 *
 * \brief Generic Dynamixel Servo Base Class
 *
 * The DynaServo base class provides the most commonly used interface functions
 * for the various Dynamixel series servos.
 */
class DynaServoGeneric : public DynaServo
{
public:
  /*! Model number */
  static const int DYNA_MODEL_NUM = DYNA_MODEL_NUM_GENERIC;

  /*!
   * \brief Bare-bones initialization constructor.
   *
   * May be used be derived classes to avoid undue communication and
   * initializaton overhead.
   *
   * \param comm      Dynamixel bus communication instance.
   * 
   */
  DynaServoGeneric(DynaComm &comm) : 
                  DynaServo(comm, DYNA_ID_NONE, DYNA_MODEL_NUM, DYNA_FWVER_NA)
  {
  }

  /*!
   * \brief Initialization constructor.
   *
   * \param comm      Dynamixel bus communication instance.
   * \param nServoId  Servo Id.
   * \param uModelNum Servo model number.
   * \param uFwVer    Servo firmware version.
   */
  DynaServoGeneric(DynaComm &comm,
                   int       nServoId,
                   uint_t    uModelNum = DYNA_MODEL_NUM,
                   uint_t    uFwVer    = DYNA_FWVER_NA);

  /*!
   * \brief Destructor.
   */
  virtual ~DynaServoGeneric();


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Attribute Functions
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  
  // Add any specific derived attribute functions here

  /*!
   * \brief Convert raw temperature coding to degrees Celsius.
   *
   * \param uTemp   Raw temperature value.
   *
   * \return Celsius.
   */
  virtual float CvtRawTempToC(uint uTemp)
  {
    return (float)uTemp;    // 1-1 mapping
  }

  /*!
   * \brief Convert raw volts coding to volts.
   *
   * \param uTemp   Raw volts value.
   *
   * \return Volatage.
   */
  virtual float CvtRawVoltToVolts(uint uVolts)
  {
    return (float)uVolts / 10.0;
  }


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Servo Proxy Agent Functions
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \brief Agent write to the servo state memory to set the new goal position.
   *
   * If the operation requires a servo proxy agent, then a call is made to the
   * registered agent, if any. Otherwise a direct write to the servo is made.
   *
   * \par Control Table:
   * \ref dyna_memmap_gen_goal_pos.\n
   * \ref dyna_servo_pos.
   *
   * \param nGoalPos    Goal position (odometer ticks).
   *
   * \copydoc doc_return_std
   */
  virtual int AgentWriteGoalPos(int nGoalOdPos);

  /*!
   * \brief Agent write to the servo state memory to set the new goal speed.
   *
   * If the operation requires a servo proxy agent, then a call is made to the
   * registered agent, if any. Otherwise a direct write to the servo is made.
   *
   * \par Control Table:
   * \ref dyna_memmap_gen_goal_speed.\n
   * \ref dyna_servo_speed.
   *
   * \param nGoalSpeed    Goal speed (raw).
   *
   * \copydoc doc_return_std
   */
  virtual int AgentWriteGoalSpeed(int nGoalSpeed);

  /*!
   * \brief Agent write to the servo state memory to set the new goal position
   * and speed.
   *
   * If the operation requires a servo proxy agent, then a call is made to the
   * registered agent, if any. Otherwise a direct write to the servo is made.
   *
   * \par Control Table:
   * \ref dyna_memmap_gen_goal_pos.\n
   * \ref dyna_servo_pos.
   * \ref dyna_memmap_gen_goal_speed.\n
   * \ref dyna_servo_speed.
   *
   * \param nGoalOdPos  Goal position (odometer ticks).
   * \param nGoalSpeed  Goal speed (raw).
   *
   * \copydoc doc_return_std
   */
  virtual int AgentWriteGoalSpeedPos(int nGoalSpeed, int nGoalOdPos);
  
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Servo Move Functions
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \brief Move to the goal postition.
   *
   * The move will proceed at the current goal speed. If the servo is in 
   * continuous mode, but supports 360\h_deg positioning data, then a registered
   * external user control function is required.
   *
   * If this servo is the master in a linked pair of servos, both servos will
   * move synchronously.
   *
   * \par Operational Modes:
   * Servo mode (\ref DYNA_MODE_SERVO).\n
   * Continuous mode (\ref DYNA_MODE_CONTINUOUS) with 360\h_deg position data.\n
   *
   * \param nGoalOdPos    Goal position (odometer ticks).
   *
   * \copydoc doc_return_std
   */
  virtual int MoveTo(int nGoalOdPos);

  /*!
   * \brief Move at speed to the goal postition.
   *
   * For servos in continuous mode, speeds are in the range [-max,max] where the
   * values \h_lt 0, 0, and \h_gt 0 specify goal rotation in the clockwise
   * direction, stop, or goal rotation in the counterclockwise direction,
   * respectively.
   *
   * For servos in servo mode, the goal direction is not applicable, and
   * therefore, the speeds are in the range [0,max]. The special value 0 is
   * equivalent to the maximum rpm speed without servo speed control. The
   * absolute value of the goal speed is used.
   *
   * If the servo is in continuous mode, but supports 360\h_deg positioning
   * data, a registered external user control function is required.
   *
   * If this servo is the master in a linked pair of servos, both servos will
   * move synchronously.
   *
   * \par Operational Modes:
   * Servo mode (\ref DYNA_MODE_SERVO). The direction will be ignored.\n
   * Continuous mode (\ref DYNA_MODE_CONTINUOUS) with 360\h_deg position data.\n
   *
   * \param nGoalSpeed  Goal speed (raw).
   * \param nGoalOdPos  Goal position (odometer ticks).
   *
   * \copydoc doc_return_std
   */
  virtual int MoveAtSpeedTo(int nGoalSpeed, int nGoalOdPos);

  /*!
   * \brief Move at speed.
   *
   * Speeds are in the range [-max,max] where the values \h_lt 0, 0, and \h_gt 0
   * specify goal rotation in the clockwise direction, stop, or goal rotation in
   * the counterclockwise direction, respectively.
   *
   * If this servo is the master in a linked pair of servos, both servos will
   * move synchronously.
   *
   * \par Operational Modes:
   * Continuous mode (\ref DYNA_MODE_CONTINUOUS).
   *
   * \param nGoalSpeed    Goal speed (raw).
   *
   * \copydoc doc_return_std
   */
  virtual int MoveAtSpeed(int nGoalSpeed);

  /*!
   * \brief Emergency stop servo.
   *
   * All torque is removed.
   *
   * If this servo is the master in a linked pair of servos, both servos will
   * be emergency stopped.
   *
   * \copydoc doc_return_std
   */
  virtual int EStop();

  /*!
   * \brief Stop servo.
   * 
   * Current torque levels are kept.
   *
   * If this servo is the master in a linked pair of servos, both servos will
   * be stopped.
   *
   * \copydoc doc_return_std
   */
  virtual int Stop();

  /*!
   * \brief Freeze servo at current position.
   *
   * Torque is applied.
   *
   * If this servo is the master in a linked pair of servos, both servos will
   * be frozen.
   *
   * \copydoc doc_return_std
   */
  virtual int Freeze();

  /*!
   * \brief Release servo from any applied torque. 
   *
   * Servo is free to rotate.
   *
   * If this servo is the master in a linked pair of servos, both servos will
   * be released.
   *
   * \copydoc doc_return_std
   */
  virtual int Release();

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Servo Read/Write Functions
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \brief Read from the servo configuration EEPROM the current rotation
   * limits.
   *
   * If this servo is the master in a linked pair of servos, both servos' 
   * configuration data are read.
   *
   * \par Control Table:
   * \ref dyna_memmap_gen_lim_cw.\n
   * \ref dyna_memmap_gen_lim_ccw.\n
   * \ref dyna_servo_pos.
   *
   * \param [out] pCwLim    Clockwise limit.
   * \param [out] pCcwLim   Counterclockwise limit.
   *
   * \copydoc doc_return_std
   */
  virtual int CfgReadRotationLimits(uint_t *pCwLim, uint_t *pCcwLim);

  /*!
   * \brief Write to the servo configuration EEPROM the new rotation limits.
   *
   * If this servo is the master in a linked pair of servos, both servos' 
   * configuration data are written.
   *
   * \par Control Table:
   * \ref dyna_memmap_gen_lim_cw.
   * \ref dyna_memmap_gen_lim_ccw.\n
   * \ref dyna_servo_pos.
   *
   * \param uCwLim    Clockwise limit.
   * \param uCcwLim   Counterclockwise limit.
   *
   * \copydoc doc_return_std
   */
  virtual int CfgWriteRotationLimits(uint_t uCwLim, uint_t uCcwLim);

  /*!
   * \brief Read from the servo configuration EEPROM the current maximum
   * temperature limit.
   *
   * If this servo is the master in a linked pair of servos, both servos' 
   * configuration data are read.
   *
   * \par Control Table:
   * \ref dyna_memmap_gen_lim_temp_max.
   *
   * \param [out] pTempLim  Temperature limit.
   *
   * \copydoc doc_return_std
   */
  virtual int CfgReadTemperatureLimit(uint_t *pTempLim);

  /*!
   * \brief Write to the servo configuration EEPROM the new maximum temperature
   * limit.
   *
   * \par Control Table:
   * \ref dyna_memmap_gen_lim_temp_max.
   *
   * \param uTempLim  Temperature limit.
   *
   * \copydoc doc_return_std
   */
  virtual int CfgWriteTemperatureLimit(uint_t uTempLim);

  /*!
   * \brief Read from the servo configuration EEPROM the current voltage limits.
   *
   * If this servo is the master in a linked pair of servos, both servos' 
   * configuration data are read.
   *
   * \par Control Table:
   * \ref dyna_memmap_gen_lim_volt_min.\n
   * \ref dyna_memmap_gen_lim_volt_max.
   *
   * \param [out] pMinVoltLim   Minimum voltage limit.
   * \param [out] pMaxVoltLim   Maximum voltage limit.
   *
   * \copydoc doc_return_std
   */
  virtual int CfgReadVoltageLimits(uint_t *pMinVoltLim, uint_t *pMaxVoltLim);

  /*!
   * \brief Write to the servo configuration EEPROM the new voltage limits.
   *
   * If this servo is the master in a linked pair of servos, both servos' 
   * configuration data are written.
   *
   * \par Control Table:
   * \ref dyna_memmap_gen_lim_volt_min.\n
   * \ref dyna_memmap_gen_lim_volt_max.
   *
   * \param uMinVoltLim   Minimum voltage limit.
   * \param uMaxVoltLim   Maximum voltage limit.
   *
   * \copydoc doc_return_std
   */
  virtual int CfgWriteVoltageLimits(uint_t uMinVoltLim, uint_t uMaxVoltLim);

  /*!
   * \brief Read from the servo configuration EEPROM the current on power-up
   * maximum torque limit.
   *
   * If this servo is the master in a linked pair of servos, both servos' 
   * configuration data are read.
   *
   * \par Control Table:
   * \ref dyna_memmap_gen_lim_torque_max_on.\n
   * \ref dyna_servo_torque.
   *
   * \param [out] pMaxTorqueLim   Maximum torque limit.
   *
   * \copydoc doc_return_std
   */
  virtual int CfgReadMaxTorqueLimit(uint_t *pMaxTorqueLim);

  /*!
   * \brief Write to the servo configuration EEPROM the new on power-up maximum
   * torque limit.
   *
   * If this servo is the master in a linked pair of servos, both servos' 
   * configuration data are written.
   *
   * \par Control Table:
   * \ref dyna_memmap_gen_lim_torque_max_on.\n
   * \ref dyna_servo_torque.
   *
   * \param uMaxTorqueLim   Maximum torque limit.
   *
   * \copydoc doc_return_std
   */
  virtual int CfgWriteMaxTorqueLimit(uint_t uMaxTorqueLim);

  /*!
   * \brief Read from the servo configuration EEPROM the current servo shutdown
   * on alarms mask.
   *
   * If this servo is the master in a linked pair of servos, both servos' 
   * configuration data are read.
   *
   * \par Control Table:
   * \ref dyna_memmap_gen_alarm_shutdown.
   *
   * \param [out] pAlarmMask   Shutdown alarm mask.
   *
   * \copydoc doc_return_std
   */
  virtual int CfgReadAlarmShutdownMask(uint_t *pAlarmMask);

  /*!
   * \brief Write to the servo configuration EEPROM the new servo shutdown on
   * alarms mask.
   *
   * \par Control Table:
   * \ref dyna_memmap_gen_alarm_shutdown.
   *
   * \param uAlarmMask   Shutdown alarm mask.
   *
   * \copydoc doc_return_std
   */
  virtual int CfgWriteAlarmShutdownMask(uint_t uAlarmMask);

  /*!
   * \brief Read from the servo configuration EEPROM to determine the servo
   * operational mode.
   *
   * If this servo is the master in a linked pair of servos, both servos' 
   * configuration data are read.
   *
   * \par Control Table:
   * \ref dyna_servo_mode.\n
   * \ref dyna_memmap_gen_lim_cw.\n
   * \ref dyna_memmap_gen_lim_ccw.
   *
   * \param [out] pServoMode  Servo mode.
   *
   * \copydoc doc_return_std
   */
  virtual int CfgReadServoMode(uint_t *pServoMode);

  /*!
   * \brief Write to the servo configuration EEPROM to set the servo operational
   * mode.
   *
   * If this servo is the master in a linked pair of servos, both servos' 
   * configuration data are written.
   *
   * \par Control Table:
   * \ref dyna_servo_mode.\n
   * \ref dyna_memmap_gen_lim_cw.\n
   * \ref dyna_memmap_gen_lim_ccw.
   *
   * \param uCwLim    Clockwise limit.
   * \param uCcwLim   Counterclockwise limit.
   *
   * \copydoc doc_return_std
   */
  virtual int CfgWriteServoMode(uint_t uCwLim, uint_t uCcwLim);

  /*!
   * \brief Write to the servo configuration EEPROM to set the servo operational
   * mode to the full/continuous mode.
   *
   * If this servo is the master in a linked pair of servos, both servos' 
   * configuration data are written.
   *
   * \par Control Table:
   * \ref dyna_servo_mode.\n
   * \ref dyna_memmap_gen_lim_cw.\n
   * \ref dyna_memmap_gen_lim_ccw.
   *
   * \copydoc doc_return_std
   */
  virtual int CfgWriteServoModeContinuous();

  /*!
   * \brief Read from the servo state memory the current torque enable value.
   *
   * If this servo is the master in a linked pair of servos, both servos' state
   * data are read.
   *
   * \par Control Table:
   * \ref dyna_memmap_gen_torque_en.
   *
   * \param [out] pState    Torque enabled (true) or disabled (off) state.
   *
   * \copydoc doc_return_std
   */
  virtual int ReadTorqueEnable(bool *pState);

  /*!
   * \brief Write to the servo state memory to set the new torque enable value.
   *
   * If the torque state is false (off), no power is applied to the servo,
   * allowing it to be free rotated by any external force.
   *
   * If this servo is the master in a linked pair of servos, both servos' state
   * written.
   *
   * \par Control Table:
   * \ref dyna_memmap_gen_torque_en.
   *
   * \param bState    Torque enabled (true) or disabled (off) state.
   *
   * \copydoc doc_return_std
   */
  virtual int WriteTorqueEnable(bool bState);

  /*!
   * \brief Read from the servo state memory the current LED on/off value.
   *
   * If this servo is the master in a linked pair of servos, both servos' state
   * data are read.
   *
   * \par Control Table:
   * \ref dyna_memmap_gen_led.
   *
   * \param [out] pState    On (true) or off (false) state.
   *
   * \copydoc doc_return_std
   */
  virtual int ReadLed(bool *pState);

  /*!
   * \brief Write to the servo state memory to turn on or off the servo LED.
   *
   * If this servo is the master in a linked pair of servos, both servos' state
   * written.
   *
   * \par Control Table:
   * \ref dyna_memmap_gen_led.
   *
   * \param bState    On (true) or off (false) state.
   *
   * \copydoc doc_return_std
   */
  virtual int WriteLed(bool bState);

  /*!
   * \brief Read from the servo state memory the current control method
   * parameters.
   *
   * The control method used is determined by the servo's specification.
   *
   * If this servo is the master in a linked pair of servos, both servos' state
   * data are read.
   *
   * \par Control Table:
   * \ref dyna_ctl_method.\n
   * \ref dyna_memmap_gen_cw_comp_margin.\n
   * \ref dyna_memmap_gen_ccw_comp_margin.\n
   * \ref dyna_memmap_gen_cw_comp_slope.\n
   * \ref dyna_memmap_gen_ccw_comp_slope.\n
   * \ref dyna_memmap_gen_p_gain.\n
   * \ref dyna_memmap_gen_i_gain.\n
   * \ref dyna_memmap_gen_d_gain.
   *
   * \param [out] pCtlMethod    Control method parameters.
   *
   * \copydoc doc_return_std
   */
  virtual int ReadControlMethod(DynaServoCtlMethod_T *pCtlMethod);

  /*!
   * \brief Write to the servo state memory the new control method parameters.
   *
   * The control method used is determined by the servo's specification.
   *
   * If this servo is the master in a linked pair of servos, both servos' state
   * data are written.
   *
   * \par Control Table:
   * \ref dyna_ctl_method.\n
   * \ref dyna_memmap_gen_cw_comp_margin.\n
   * \ref dyna_memmap_gen_ccw_comp_margin.\n
   * \ref dyna_memmap_gen_cw_comp_slope.\n
   * \ref dyna_memmap_gen_ccw_comp_slope.\n
   * \ref dyna_memmap_gen_p_gain.\n
   * \ref dyna_memmap_gen_i_gain.\n
   * \ref dyna_memmap_gen_d_gain.
   *
   * \param ctlMethod     Control method parameters.
   *
   * \copydoc doc_return_std
   */
  virtual int WriteControlMethod(DynaServoCtlMethod_T &ctlMethod);

  /*!
   * \brief Read from the servo state memory the current goal position.
   *
   * If this servo is the master in a linked pair of servos, both servos' state
   * data are read.
   *
   * \par Control Table:
   * \ref dyna_memmap_gen_goal_pos.\n
   * \ref dyna_servo_pos.
   *
   * \param [out] pGoalPos    Servo Mode:      Encoder goal position.\n
   *                          Continuous Mode: Odometer goal position.
   *
   * \copydoc doc_return_std
   */
  virtual int ReadGoalPos(int *pGoalPos);

  /*!
   * \brief Write to the servo state memory to set the new goal position.
   *
   * If the servo [pair] are in servo mode, rotation will occur until the
   * position(s) are achieved.
   *
   * If this servo is the master in a linked pair of servos, both servos' state
   * written.
   *
   * \par Control Table:
   * \ref dyna_memmap_gen_goal_pos.\n
   * \ref dyna_servo_pos.
   *
   * \param nGoalOdPos  Odometer goal position (odometer ticks).
   *                    In continuous mode, writing the goal position has no
   *                    effect on movement.
   *
   * \copydoc doc_return_std
   */
  virtual int WriteGoalPos(int nGoalOdPos);

  /*!
   * \brief Read from the servo state memory the current goal speed and
   * direction.
   *
   * For servos in continuous mode, speeds are in the range [-max,max] where the
   * values \h_lt 0, 0, and \h_gt 0 specify goal rotation in the clockwise
   * direction, stop, or goal rotation in the counterclockwise direction,
   * respectively.
   *
   * For servos in servo mode, the goal direction is not applicable, and
   * therefore, the speeds are in the range [0,max]. The special value 0 is
   * equivalent to the maximum rpm speed without servo speed control.
   *
   * If this servo is the master in a linked pair of servos, both servos' state
   * data are read.
   *
   * \par Control Table:
   * \ref dyna_memmap_gen_goal_speed.\n
   * \ref dyna_servo_speed.\n
   * \ref dyna_servo_dir.
   *
   * \param [out] pGoalSpeed    Goal speed.
   *
   * \copydoc doc_return_std
   */

  virtual int ReadGoalSpeed(int *pGoalSpeed);

  /*!
   * \brief Write to the servo state memory the new goal speed and direction.
   *
   * For servos in continuous mode, speeds are in the range [-max,max] where the
   * values \h_lt 0, 0, and \h_gt 0 specify goal rotation in the clockwise
   * direction, stop, or goal rotation in the counterclockwise direction,
   * respectively.
   *
   * For servos in servo mode, the goal direction is not applicable, and
   * therefore, the speeds are in the range [0,max]. The special value 0 is
   * equivalent to the maximum rpm speed without servo speed control.
   *
   * If the servo is in continuous mode, rotation will occur indefinitely
   * in the direction specified and at the given speed.
   *
   * \par Control Table:
   * \ref dyna_memmap_gen_goal_speed.\n
   * \ref dyna_servo_speed.\n
   * \ref dyna_servo_dir.
   *
   * \param nGoalSpeed      Goal speed.
   *
   * \copydoc doc_return_std
   */
  virtual int WriteGoalSpeed(int nGoalSpeed);

  /*!
   * \brief Read from the servo state memory the current maximum torque limit.
   *
   * The servo will initially set this value to the value in the
   * \ref dyna_memmap_gen_lim_torque_max configuration.
   * The servo will set this value to 0 on an alarm shutdown event.
   *
   * If this servo is the master in a linked pair of servos, both servos' state
   * data are read.
   *
   * \par Control Table:
   * \ref dyna_memmap_gen_lim_torque_max.\n
   * \ref dyna_servo_torque.
   *
   * \param [out] pMaxTorqueLim   Maximum torque limit.
   *
   * \copydoc doc_return_std
   */
  virtual int ReadMaxTorqueLimit(uint_t *pMaxTorqueLim);

  /*!
   * \brief Write to the servo state memory to set the new maximum torque limit.
   *
   * If this servo is the master in a linked pair of servos, both servos' state
   * written.
   *
   * \par Control Table:
   * \ref dyna_memmap_gen_lim_torque_max.\n
   * \ref dyna_servo_torque.
   *
   * \param uMaxTorqueLim   Maximum torque limit.
   *
   * \copydoc doc_return_std
   */
  virtual int WriteMaxTorqueLimit(uint_t uMaxTorqueLim);

  /*!
   * \brief Reload the maximum torque limit from the configuration.
   *
   * If this servo is the master in a linked pair of servos, both servos' state
   * data are reloaded.
   *
   * \par Control Table:
   * \ref dyna_memmap_gen_lim_torque_max_on.\n
   * \ref dyna_memmap_gen_lim_torque_max.
   *
   * \copydoc doc_return_std
   */
  virtual int ReloadMaxTorqueLimit();

  /*!
   * \brief Read from the servo state memory the current servo position.
   *
   * If this servo is the master in a linked pair of servos, both servos' state
   * data are read.
   *
   * \par Control Table:
   * \ref dyna_memmap_gen_cur_pos.\n
   * \ref dyna_servo_pos.
   *
   * \param [out] pCurOdPos   Odometer current position (ticks).
   *
   * \copydoc doc_return_std
   */
  virtual int ReadCurPos(int *pCurOdPos);

  /*!
   * \brief Read from the servo state memory the current speed and direction.
   *
   * Speeds are in the range [-max,max] where the values \h_lt 0, 0, and \h_gt 0
   * specify the servo is rotating in the clockwise direction, the servo is
   * stopped, or the servo is rotating in the counterclockwise direction,
   * respectively.
   *
   * If this servo is the master in a linked pair of servos, both servos' state
   * data are read.
   *
   * \par Control Table:
   * \ref dyna_memmap_gen_cur_speed.\n
   * \ref dyna_servo_speed.\n
   * \ref dyna_servo_dir.
   *
   * \param [out] pCurSpeed   Current speed.
   *
   * \copydoc doc_return_std
   */
  virtual int ReadCurSpeed(int *pCurSpeed);

  /*!
   * \brief Read from the servo state memory the current load.
   *
   * Loads are in the range [-max,max] where the values \h_lt 0, 0, and \h_gt 0
   * specify torques loads in clockwise direction, no torque load, or torque 
   * loads in the counterclockwise direction, respectively.
   * Load is approximately 0.1% of maximum.
   *
   * \par Control Table:
   * \ref dyna_memmap_gen_cur_load.
   *
   * \param [out] pCurLoad    Current load.
   *
   * \copydoc doc_return_std
   */
  virtual int ReadCurLoad(int *pCurLoad);

  /*!
   * \brief Read from the servo state memory the current servo dynamics.
   *
   * If this servo is the master in a linked pair of servos, both servos' state
   * data are read.
   *
   * \par Control Table:
   * \ref dyna_memmap_gen_cur_pos.\n
   * \ref dyna_memmap_gen_cur_speed.\n
   * \ref dyna_memmap_gen_cur_load.\n
   *
   * \param [out] pCurPos     Current position.
   * \param [out] pCurSpeed   Current speed.
   * \param [out] pCurLoad    Current load.
   *
   * \copydoc doc_return_std
   */
  virtual int ReadDynamics(int *pCurPos,
                           int *pCurSpeed,
                           int *pCurLoad);

  /*!
   * \brief Read from the servo state memory the current servo health.
   *
   * If this servo is the master in a linked pair of servos, both servos' state
   * data are read.
   *
   * \par Control Table:
   * \ref dyna_memmap_gen_cur_load.\n
   * \ref dyna_memmap_gen_cur_volt.\n
   * \ref dyna_memmap_gen_cur_temp.
   *
   * \param [out] pAlarms     Servo alarms.
   * \param [out] pCurLoad    Current load.
   * \param [out] pCurVolt    Current voltage.
   * \param [out] pCurTemp    Current temperature.
   *
   * \copydoc doc_return_std
   */
  virtual int ReadHealth(uint_t *pAlarms,
                         int    *pCurLoad,
                         uint_t *pCurVolt,
                         uint_t *pCurTemp);

  /*!
   * \brief Read from the servo state memory to test if the servo is currently
   * moving.
   *
   * If this servo is the master in a linked pair of servos, both servos' state
   * data are read.
   *
   * \par Control Table:
   * \ref dyna_memmap_gen_is_moving.
   *
   * \param [out] pState    Servo is [not] moving (true/false) state.
   *
   * \copydoc doc_return_std
   */
  virtual int ReadIsMoving(bool *pState);

  /*!
   * Read a raw value from the servo EEPROM/RAM control table.
   *
   * \warning
   * The shadowed configuration and state data are not update.\n
   * Linked servos are not kept in sync.
   *
   * \param uAddr       Control table address.
   * \param [out] pVal  Read raw value.
   *
   * \copy doc_return_std
   */
  virtual int  Read(uint_t uAddr, uint_t *pVal);

  /*!
   * Write a raw value to the servo EEPROM/RAM control table.
   *
   * \warning The shadowed configuration and state data are not updated and will
   * hence be out of sync.
   *
   * \warning Any linked master-slave servos may get out of sync and may result
   * in physical damage.
   *
   * \param uAddr   Control table address.
   * \param uVal    Raw value to write.
   *
   * \copy doc_return_std
   */
  virtual int  Write(uint_t uAddr, uint_t uVal);

  /*!
   * \brief Ping this servo.
   *
   * \return Returns true if the servo responded, else false.
   */
  virtual bool Ping();

  /*!
   * \brief Reset this servo back to default values.
   *
   * \warning All configuration is lost.
   *
   * \copydoc doc_std_return
   */
  virtual int  Reset();

  /*!
   * \brief Synchronize the shadowed configuration and state data to the servo
   * control table.
   *
   * \copydoc doc_return_std
   */
  virtual int SyncData();

  /*!
   * \brief Synchronize the shadowed configuration to the servo control table
   * EEPROM configuration.
   *
   * \copydoc doc_return_std
   */
  virtual int SyncCfg();

  /*!
   * \brief Synchronize the shadowed state data to the servo control table RAM
   * state.
   *
   * \copydoc doc_return_std
   */
  virtual int SyncState();

  /*!
   * \brief Dump contents of the servo EEPROM and RAM control tables.
   */
  virtual void Dump();

protected:

  /*!
   * \brief Initialize servo class instance.
   */
  void Init();

  /*!
   * \brief Initialize servo fixed specification data.
   */
  void InitSpec();

  /*!
   * \brief Initialize servo configuration data.
   */
  void InitCfg();

  /*!
   * \brief Initialize servo state data.
   */
  void InitState();

  /*!
   * \brief Check data for consitencies.
   */
  virtual void CheckData();

  /*! 
   * \brief Set the servo mode given the servo capabilites and the current
   * rotation limits.
   * 
   * See \ref dyna_servo_mode.
   */
  void SetServoMode();

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Linking Fuctions
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \brief Calculate the linked mate's goal position given this servo's goal
   * position.
   *
   * \param nGoalOdPos            Goal position (odometer ticks).
   * \param [out] pGoalOdPosMate  Mate's calculated goal
   *                              position (odometer ticks).
   *
   * \copydoc doc_return_std
   */
  virtual int CalcMatesGoalPos(int nGoalOdPos, int *pGoalOdPosMate);

  /*!
   * \brief Calculate the linked mate's speed speed given this servo's goal
   * speed.
   *
   * \return Mate's goal speed.
   */
  virtual int CalcMatesGoalSpeed(int nGoalSpeed)
  {
    return m_link.m_bRotReversed? -nGoalSpeed: nGoalSpeed;
  }

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Protect Read/Write Functions
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
 * \brief Read from the servo state memory the current compliance control method
 * parameters.
 *
 * If this servo is the master in a linked pair of servos, both servos' state
 * data are read.
 *
 * \par Control Table:
 * \ref dyna_ctl_method.\n
 * \ref dyna_memmap_gen_cw_comp_margin.\n
 * \ref dyna_memmap_gen_ccw_comp_margin.\n
 * \ref dyna_memmap_gen_cw_comp_slope.\n
 * \ref dyna_memmap_gen_ccw_comp_slope.
 *
 * \param [out] pCtlMethod    Control method parameters.
 *
 * \copydoc doc_return_std
 */
  virtual int ReadCtlMethodCompliance(DynaServoCtlMethod_T *pCtlMethod);

  /*!
 * \brief Write to the servo state memory the new compliance control method
 * parameters.
 *
 * If this servo is the master in a linked pair of servos, both servos' state
 * data are written.
 *
 * \par Control Table:
 * \ref dyna_ctl_method.\n
 * \ref dyna_memmap_gen_cw_comp_margin.\n
 * \ref dyna_memmap_gen_ccw_comp_margin.\n
 * \ref dyna_memmap_gen_cw_comp_slope.\n
 * \ref dyna_memmap_gen_ccw_comp_slope.\n
 *
 * \param ctlMethod     Control method parameters.
 *
 * \copydoc doc_return_std
 */
  virtual int WriteCtlMethodCompliance(DynaServoCtlMethod_T &ctlMethod);

  /*!
 * \brief Read from the servo state memory the current PID control method
 * parameters.
 *
 * If this servo is the master in a linked pair of servos, both servos' state
 * data are read.
 *
 * \par Control Table:
 * \ref dyna_ctl_method.\n
 * \ref dyna_memmap_gen_p_gain.\n
 * \ref dyna_memmap_gen_i_gain.\n
 * \ref dyna_memmap_gen_d_gain.
 *
 * \param [out] pCtlMethod    Control method parameters.
 *
 * \copydoc doc_return_std
 */
  virtual int ReadCtlMethodPid(DynaServoCtlMethod_T *pCtlMethod);

  /*!
 * \brief Write to the servo state memory the new PID control method
 * parameters.
 *
 * If this servo is the master in a linked pair of servos, both servos' state
 * data are written.
 *
 * \par Control Table:
 * \ref dyna_ctl_method.\n
 * \ref dyna_memmap_gen_p_gain.\n
 * \ref dyna_memmap_gen_i_gain.\n
 * \ref dyna_memmap_gen_d_gain.
 *
 * \param ctlMethod     Control method parameters.
 *
 * \copydoc doc_return_std
 */
  virtual int WriteCtlMethodPid(DynaServoCtlMethod_T &ctlMethod);

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Field Checking and Packing Functions
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
 
  /*!
 * \brief Check validity of compliance slope discrete values.
 *
 * \param uVal    Slope value.
 *
 * \return Returns true if value is valid, false otherwise.
 */
  virtual bool ChkComplianceSlope(uint_t uVal);

  /*!
 * \brief Pack the goal speed into the control table value.
 *
 * For servos in continuous mode, speeds are in the range [-max,max] where the
 * values \h_lt 0, 0, and \h_gt 0 specify rotation in the clockwise direction,
 * stop, or rotation in the counterclockwise direction, respectively.
 *
 * For servos in servo mode, the goal direction is not applicable, and
 * therefore, the speeds are in the range [0,max]. The special value 0 is
 * equivalent to the maximum rpm speed without servo speed control.
 *
 * \param nGoalSpeed    Goal speed.
 *
 * \return Returns packed value.
 */
  virtual uint_t PackGoalSpeed(int nGoalSpeed);

  /*!
 * \brief Unpack goal speed from the control table value.
 *
 * For servos in continuous mode, speeds are in the range [-max,max] where the
 * values \h_lt 0, 0, and \h_gt 0 specify the servo is rotating in the clockwise
 * direction, the servo is stopped, or the servo is rotating in the
 * counterclockwise direction, respectively.
 *
 * For servos in servo mode, the goal direction is not applicable, and
 * therefore, the speeds are in the range [0,max]. The special value 0 is
 * equivalent to the maximum rpm speed without servo speed control.
 *
 * \param uVal  Packed field.
 *
 * \return Goal speed.
 */
  virtual int UnpackGoalSpeed(uint_t uVal);

  /*!
 * \brief Unpack current speed from the control table value.
 *
 * Speeds are in the range [-max,max] where the values \h_lt 0, 0, and \h_gt 0
 * specify the servo is rotating in the clockwise direction, the servo is
 * stopped, or the servo is rotating in the counterclockwise direction,
 * respectively.
 *
 * \param uVal  Packed field.
 *
 * \return Current speed.
 */
  virtual int UnpackCurSpeed(uint_t uVal);

  /*!
 * \brief Unpack current load estimate from the control table value.
 *
 * Loads are in the range [-max,max] where the values \h_lt 0, 0, and \h_gt 0
 * specify torques loads in clockwise direction, no torque load, or torque 
 * loads in the counterclockwise direction, respectively.
 *
 * \param uVal  Packed field.
 *
 * \return  Current load.
 */
  virtual int UnpackCurLoad(uint_t uVal);
};


#endif // _DYNA_SERVO_GENERIC_H
