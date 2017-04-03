////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// File:      laeAlarms.h
//
/*! \file
 *
 * $LastChangedDate: 2016-01-18 10:01:31 -0700 (Mon, 18 Jan 2016) $
 * $Rev: 4262 $
 *
 * \brief Laelaps alarm monitoring class interface.
 *
 * A single instance of this class executing in a thread will monitor, raise,
 * and clear system and subsystem alarms. It will publish to the Laelaps 
 * real-time database.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2016-2017. RoadNarrows LLC.\n
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

#ifndef _LAE_ALARMS_H
#define _LAE_ALARMS_H

#include "rnr/rnrconfig.h"

#include  "Laelaps/laelaps.h"
#include  "Laelaps/laeSysDev.h"
#include  "Laelaps/laeMotor.h"

/*!
 *  \brief The \h_laelaps namespace encapsulates all \h_laelaps related
 *  constructs.
 */
namespace laelaps
{
  /*!
   * \ingroup laelaps_h
   * \defgroup lae_alarms  Laelaps Alarms and Warnings
   *
   * Laelaps package-wide alarm and warning codes.
   *
   * \{
   */
  static const u32_t LAE_ALARM_NONE       = 0x00000000; ///< no alarms

  //
  // Common alarms.
  //
  static const u32_t LAE_ALARM_GEN        = 0x00000001; ///< general alarm
  static const u32_t LAE_ALARM_ESTOP      = 0x00000002; ///< emergency stop
  static const u32_t LAE_ALARM_BATT       = 0x00000004; ///< battery low alarm
  static const u32_t LAE_ALARM_TEMP       = 0x00000008; ///< temperature alarm
  static const u32_t LAE_ALARM_NO_RSRC    = 0x00000010; ///< resource not found
  static const u32_t LAE_ALARM_SUBSYS     = 0x00000020; ///< 1+ subsystem alarms

  //
  // Motor controller specific alarms.
  //
  static const u32_t LAE_ALARM_MOTCTLR_BATT_V_HIGH  = 0x00000100;
                        ///< motor controller battery input over voltage alarm
  static const u32_t LAE_ALARM_MOTCTLR_LOGIC_V_HIGH = 0x00000400;
                          ///< motor controller logic over voltage alarm
  static const u32_t LAE_ALARM_MOTCTLR_LOGIC_V_LOW  = 0x00000800;
                          ///< motor contorller logic under voltage alarm

  //
  // Motor specific alarms.
  //
  static const u32_t LAE_ALARM_MOT_OVER_CUR = 0x00001000;
                                ///< motor over current alarm
  static const u32_t LAE_ALARM_MOT_FAULT    = 0x00002000;
                                ///< motor drive fault alarm

  //
  // Sensor specific alarms.
  //
  static const u32_t LAE_ALARM_IMU    = 0x00010000; ///< imu alarm
  static const u32_t LAE_ALARM_RANGE  = 0x00020000; ///< range alarm
  static const u32_t LAE_ALARM_FCAM   = 0x00040000; ///< front camera alarm

  /*!
   * \brief The [sub]sytem warnings.
   */
  static const u32_t LAE_WARN_NONE  = 0x00000000; ///< no warnings

  //
  // Common warnings.
  //
  static const u32_t LAE_WARN_BATT    = 0x00000004; ///< battery low warning
  static const u32_t LAE_WARN_TEMP    = 0x00000008; ///< temperature warning
  static const u32_t LAE_WARN_SUBSYS  = 0x00000020; ///< 1+ subsystem warnings

  //
  // Motor controller specific warnings.
  //
  static const u32_t LAE_WARN_MOTCTLR_BATT_V_HIGH = 0x00000100;
                        ///< motor controller battery input over voltage warning
  static const u32_t LAE_WARN_MOTCTLR_BATT_V_LOW  = 0x00000200;
                        ///< motor controller battery input under voltage warn

  //
  // Motor specific warnings.
  //
  static const u32_t LAE_WARN_MOT_OVER_CUR  = 0x00001000;
                        ///< motor over current warning


  /*!
   * \brief Alarm thresholds, criticalities, etc.
   */
  static const double LAE_WARN_BATT_SOC = 20.0;
                        ///< battery warning state of charge threshold
  static const double LAE_CRIT_BATT_SOC = 15.0;
                        ///< battery critical state of charge threshold

  static const u32_t  LAE_CRIT_MOTCTLR  = LAE_ALARM_MOTCTLR_LOGIC_V_LOW;
                        ///< critical alarms for motor controller subsystem

  static const u32_t  LAE_CRIT_MOT = LAE_ALARM_MOT_FAULT;
                        ///< critacal alarms for motor subsystem 

  /*! \} */

  /*!
   * \brief Alarm information state structure.
   */
  struct LaeAlarmInfo
  {
    bool  m_bIsCritical;    ///< is [not] critical alarm
    u32_t m_uAlarms;        ///< alarm or'ed bits
    u32_t m_uWarnings;      ///< warning or'ed bits
  }; // struct LaeAlarmInfo

  /*!
   * \brief Alarm class.
   */
  class LaeAlarms
  {
  public:
    /*!
     * \brief Default contructor.
     */
    LaeAlarms();

    /*!
     * \brief Destructor.
     */
    virtual ~LaeAlarms();

    /*!
     * \brief Test if critcal alarms exist.
     *
     * The determination is made solely on the real-time database current state.
     * No subsystems are queried.
     *
     * \return Returns true or false.
     */
    static bool isCritical();

    /*!
     * \brief Test if battery critcal alarms exist.
     *
     * The determination is made solely on the real-time database current state.
     * No subsystems are queried.
     *
     * \return Returns true or false.
     */
    static bool isBatteryCritical();

    /*!
     * \brief Test if robot is safe to operate given the current alarm state.
     *
     * The determination is made solely on the real-time database current state.
     * No subsystems are queried.
     *
     * \return Returns true or false.
     */
    static bool isSafeToOperate();

    /*!
     * \brief Clear alarm information state.
     *
     * \param [in,out] info   Alarm state.
     */
    static void clearAlarms(LaeAlarmInfo &info);

    /*!
     * \brief Copy alarm state.
     *
     * \param [in] src    Source alarm state.
     * \param [out] dst   Destination alarm state.
     */
    static void copyAlarms(const LaeAlarmInfo &src, LaeAlarmInfo &dst);

    /*!
     * \brief Update alarm and warning state.
     *
     * Status, alarms, and erros from all monitored subsystems are processed to
     * provide a unified alarm framework. The alarm state is written to the
     * Laelaps real-time database.
     */
    virtual void update();

  protected:
    /*!
     * \brief Update system alarm state from subsystem alarm state.
     *
     * \param [in] subsys   Subsystem alarm state.
     * \param [out] sys     System alarm state.
     */
    virtual void updateSysAlarms(const LaeAlarmInfo &subsys, LaeAlarmInfo &sys);

  }; // LaeAlarms

} // namespace laelaps


#endif // _LAE_ALARMS_H
