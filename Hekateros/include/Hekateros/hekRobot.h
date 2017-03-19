////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekRobot.h
//
/*! \file
 *
 * $LastChangedDate: 2014-11-18 14:31:49 -0700 (Tue, 18 Nov 2014) $
 * $Rev: 3810 $
 *
 * \brief HekRobot - Hekateros Robot Class interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2011-2017. RoadNarrows LLC.\n
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

#ifndef _HEK_ROBOT_H
#define _HEK_ROBOT_H

#include <pthread.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/units.h"
#include "rnr/i2c.h"

#include <string>
#include <map>

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaComm.h"
#include "Dynamixel/DynaServo.h"
#include "Dynamixel/DynaChain.h"
#include "Dynamixel/DynaBgThread.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekTune.h"
#include "Hekateros/hekUtils.h"
#include "Hekateros/hekDesc.h"
#include "Hekateros/hekJoint.h"
#include "Hekateros/hekCalib.h"
#include "Hekateros/hekCalibStretch.h"
#include "Hekateros/hekTraj.h"
#include "Hekateros/hekKin.h"
#include "Hekateros/hekState.h"
#include "Hekateros/hekMonitor.h"

namespace hekateros
{
  /*!
   * \brief Hekateros robotic manipulator plus accesories class.
   *
   * Includes up to 3 kinematic chains.
   */
  class HekRobot
  { 
  public:
    /*!
     * \brief Indirect map of robot joints.
     *
     * \termblock
     * \term key: \termdata joint name \endterm
     * \term mapped type: \termdata master servo id \endterm
     * \endtermblock
     */
    typedef map<std::string, int> IMapRobotJoints;

    /*!
     * \brief Asynchronous task id.
     */
    enum AsyncTaskId
    {
      AsyncTaskIdNone,        ///< no task
      AsyncTaskIdCalibrate    ///< calibrate hekateros arm task id

      // add others here, as needed
    };

    /*!
     * \brief Map of doubles.
     */
    typedef std::map<std::string, double> MapDouble;

    /*!
     * \brief Default initialization constructor.
     *
     * \param bNoExec Do [not] execute arm physical movements. All commands
     *                and responses are supported but the lower level arm
     *                movement commands will not be issued.
     */
    HekRobot(bool bNoExec=false);

    /*!
     * \brief Destructor.
     */
    virtual ~HekRobot();


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Communication and Robot Initialization Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Connect to \h_hek.
     *
     * \param strDevDynabus     Dynabus serial device name.
     * \param nBaudRateDynabus  Dynabus baud rate.
     * \param strDevArduino     Arduino serial device name.
     * \param nBaudRateArduino  Arduino baud rate.
     *
     * \copydoc doc_return_std
     */
    int connect(const std::string &strDevDynabus    = HekDevDynabus,
                int                nBaudRateDynabus = HekBaudRateDynabus,
                const std::string &strDevArduino    = HekDevArduino,
                int                nBaudRateArduino = HekBaudRateArduino);

    //int connectByProxy(...);

    /*!
     * \brief Disconnect from \h_hek.
     *
     * \copydoc doc_return_std
     */
    int disconnect();

    /*!
     * \brief Calibrate \h_hek's odometers and limit switch positions.
     *
     * \param bForceRecalib   If true, recalibrate all joints.
     *                        Otherwise calibrate only the uncalibrated.
     *
     * \copydoc doc_return_std
     */
    int calibrate(bool bForceRecalib=true);

    /*!
     * \brief Asynchronously calibrate \h_hek's odometers and limit switch
     * positions.
     *
     * Call \ref cancelAsync() to cancel operation.
     *
     * \param bForceRecalib   If true, recalibrate all joints.
     *                        Otherwise calibrate only the uncalibrated.
     *
     * \copydoc doc_return_std
     */
    int calibrateAsync(bool bForceRecalib=true);

    /*!
     * \brief Reload \h_hek's reloadable configuration and reset operational
     * parameters.
     *
     * The robot connection and calibration states are uneffected.
     */
    void reload();


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Canned Movements
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Move robotic arm to its pre-defined balanced position.
     *
     * \par Some Considerations:
     * This move assumes an unrestricted workspace save for an horizontal plane
     * defined by the platform on which the \h_hek is mounted to. Strategy:
     * \li The shoulder movement is the most critical as it will lift away 
     * the horizontal plane, and hopefully away from any obstructions.
     * \li The elbow joint may swing out, possibly causing a collision before
     * the shoulder has sufficiently moved to a safer position. Therefore the
     * elbow joint's velocity is slower than the shoulder's velocity.
     * \li The rotating base, if present, swings left or right, possibly
     * dragging the arm across the horizontal plane. Therefore, this joint
     * also moves slower than the shoulder.
     * \li The other joints are of no consequence.
     *
     * \copydoc doc_return_std
     */
    int gotoBalancedPos();

    /*!
     * \brief Move robotic arm to its pre-defined parked position.
     *
     * See discussion at \ref gotoBalancedPos about pre-defined movements.
     *
     * \copydoc doc_return_std
     */
    int gotoParkedPos();

    /*!
     * \brief Move robotic arm to its pre-defined calibrated zero point
     * position.
     *
     * See discussion at \ref gotoBalancedPos about pre-defined movements.
     *
     * \copydoc doc_return_std
     */
    int gotoZeroPtPos();

    /*!
     * \brief Open gripper to its maximum opened position.
     *
     * \copydoc doc_return_std
     */
    int openGripper();

    /*!
     * \brief Close gripper to its minimum closed position.
     *
     * \copydoc doc_return_std
     */
    int closeGripper();


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Fundamental Arm Operations
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Emergency stop.
     *
     * All servos will stop driving, so arm or accessories may fall.
     * Emergency stop requires recalibration.
     *
     * \copydoc doc_return_std
     */
    int estop();

    /*!
     * \brief Freeze arm and accessories at current position.
     *
     * Servos are still being driven.
     *
     * \copydoc doc_return_std
     */
    int freeze();

    /*!
     * \brief Release arm and accessories.
     *
     * Servos will stop driving, so the arm may fall. This call is assumed to 
     * be under control, so recalibration is not required. Typically, the arm
     * is released during manual repositioning or teaching.
     *
     * \copydoc doc_return_std
     */
    int release();

    /*!
     * \brief Stops specified joints at current position.
     *
     * Servos are still being driven.
     *
     * \param vecNames  Vector of joint names.
     *
     * \copydoc Returns the number of joints stopped.
     */
    int stop(const std::vector<std::string> &vecNames);

    /*!
     * \brief Attempt to clear all servo alarms in all kinematic chains.
     *
     * \note Not all alarms are clearable (e.g. temperature).
     *
     * \copydoc doc_return_std
     */
    int clearAlarms();

    /*!
     * \brief Reset (clears) emergency stop condition.
     *
     * \note Servos are not re-powered until an move or freeze action is called.
     */
    void resetEStop()
    {
      m_bIsEStopped = false;
      m_monitor.markEStopCond(m_bIsEStopped);
    }

    /*!
     * \brief Move arm through trajectory point.
     *
     * \param trajectoryPoint   Trajectory end point.
     *
     * \copydoc doc_return_std
     */
    int moveArm(HekJointTrajectoryPoint &trajectoryPoint);


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Arm State and Status
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Get the robot current state.
     *
     * \param [in,out] robotState   Robot state.
     *
     * \copydoc doc_return_std
     */
    int getRobotState(HekRobotState &robotState);

    /*!
     * \brief Get the joint states of a kinematic chain.
     *
     * \param [in,out] jointStatePoint  Joint state point.
     *
     * \copydoc doc_return_std
     */
    int getJointState(HekJointStatePoint &jointStatePoint);

    /*!
     * \brief Get trajectory feedback.
     *
     * \param [in,out] trajectoryFeedback  Joint state point.
     *
     * \copydoc doc_return_std
     */
    int getTrajectoryState(HekJointTrajectoryFeedback &trajectoryFeedback);

    /*!
     * \brief Set robot's operational mode.
     *
     * \return eRobotMode Robot operation mode. See \ref HekRobotMode.
     */
    void setRobotMode(HekRobotMode eRobotMode)
    {
      m_eRobotMode = eRobotMode;
    }

    /*!
     * \brief Test if any joint in any of the kinematic chains is moving.
     *
     * \return Returns true or false.
     */
    bool isInMotion();

    /*!
     * \brief Cancel any asynchronous task.
     *
     * \note There may be a little delay between canceling an async task
     * and the task actually stopping.
     */
    void cancelAsyncTask();

    /*!
     * \brief Get the current asynchronous task state.
     *
     * \return \ref HekAsyncTaskState enum value.
     */
    HekAsyncTaskState getAsyncState();

    /*!
     * \brief Get the last asynchronous task return code.
     *
     * \return
     * Returns HEK_OK when task terminated successfully.
     * Otherwise, returns \h_lt 0 \ref hek_ecodes.
     */
    int getAsyncRc();


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Attribute Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Test if connected to \h_hek hardware.
     *
     * \return Returns true or false.
     */
    bool isConnected()
    {
      return (m_pDynaComm != NULL) && m_pDynaComm->IsOpen()? true: false;
    }

    /*!
     * \brief Get the \h_hek robotic arm hardware version numbers.
     *
     * \param [out] nVerMajor     Major version number.
     * \param [out] nVerMinor     Minor version number.
     * \param [out] nVerRevision  Revision version number.
     */
    void getVersion(int &nVerMajor, int &nVerMinor, int &nRevision)
    {
      uint_t  uHwVer;

      uHwVer    = strToVersion(m_descHek.getArmDesc()->getProdHwVer());
      nVerMajor = HEK_VER_MAJOR(uHwVer);
      nVerMinor = HEK_VER_MINOR(uHwVer);
      nRevision = HEK_VER_REV(uHwVer);
    }

    /*!
     * \brief Get the \h_hek robotic arm hardware version compact number.
     *
     * \return Return byte-packed comparable verion number.
     */
    uint_t getVersionNum()
    {
      return strToVersion(m_descHek.getArmDesc()->getProdHwVer());
    }

    /*!
     * \brief Get the \h_hek robotic arm hardware version string.
     *
     * Version number strings are of the dotted form maj.min.rev.
     *
     * \return Version string.
     */
    std::string getVersion()
    {
      return m_descHek.getArmDesc()->getProdHwVer();
    }

    /*!
     * \brief Get the \h_hek product description.
     *
     * \return Returns pointer to description.
     */
    HekDesc *getHekDesc()
    {
      return &m_descHek;
    }

    /*!
     * \brief Convenience function to get this \h_hek description's base
     * product id.
     *
     * \return Returns product id. See \ref HekProdId.
     */
    int getProdId()
    {
      return m_descHek.getArmDesc()->getProdId();
    }
  
    /*!
     * \brief Convenience function to get this \h_hek description's base
     * product name.
     *
     * \return Returns product name. See \ref HekProdName.
     */
    std::string getProdName()
    {
      return m_descHek.getArmDesc()->getProdName();
    }
  
    /*!
     * \brief Get the \h_hek full brief descirption.
     *
     * \return Returns product brief description.
     */
    std::string getFullProdBrief()
    {
      return m_descHek.getFullProdBrief();
    }
  
    /*!
     * \brief Test if robot is fully described via configuration XML.
     *
     * \return Returns true or false.
     */
    int isDescribed()
    {
      return m_descHek.isDescribed();
    }

    /*!
     * \brief Test if robot is calibrated.
     *
     * \return Returns true or false.
     */
    int isCalibrated()
    {
      return m_eOpState == HekOpStateCalibrated;
    }

    /*!
     * \brief Test if robot is current emergency stopped.
     *
     * \return Returns true or false.
     */
    int isEStopped()
    {
      return m_bIsEStopped;
    }

    /*!
     * \brief Test if robot servos are currently being driven (powered).
     *
     * \return Returns true or false.
     */
    int areServosPowered()
    {
      return m_bAreServosPowered;
    }

    /*!
     * \brief Test if robot is alarmed.
     *
     * \return Returns true or false.
     */
    int isAlarmed()
    {
      // monitor always has the latest alarm conditions. Robot simple shadows.
      m_bAlarmState = m_monitor.getAlarmCond();
      return m_bAlarmState;
    }

    /*!
     * \brief Test if robot is as pre-defined balanced position.
     *
     * \return Returns true or false.
     */
    int isAtBalancedPos()
    {
      return m_bAtBalancedPos;
    }
  
    /*!
     * \brief Test if robot is as pre-defined parked position.
     *
     * \return Returns true or false.
     */
    int isAtParkedPos()
    {
      return m_bAtParkedPos;
    }
  
    /*!
     * \brief Get the dynamixel chain object.
     *
     * \return Returns point to DynaChain object.
     */
    DynaChain *getDynaChain()
    {
      return m_pDynaChain;
    }

    /*!
     * \brief Get trajectory parameters.
     *
     * \param [out] eNorm     Distance norm.
     * \param [out] fEpsilon  Waypoint precision (radians)
     */
    void getTrajectoryParams(HekNorm &eNorm, double &fEpsilon)
    {
      m_tunes.getTrajectoryParams(eNorm, fEpsilon);
    }


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Kinematic Access and Mapping Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Get robotic joint in arm+ee kinematic chain at the given index.
     *
     * \param index   Joint index.
     *
     * \return If found, returns pointer to joint. Otherwise returns NULL.
     */
    HekRobotJoint *getArmJointAt(int index)
    {
      return index < m_jointsArm.size()? &m_jointsArm.at(index): NULL;
    }

    /*!
     * \brief Get robotic joint in arm+ee kinematic chain.
     *
     * \param strName   Joint name (key).
     *
     * \return If found, returns pointer to joint. Otherwise returns NULL.
     */
    HekRobotJoint *getArmJoint(const std::string &strName)
    {
      IMapRobotJoints::iterator  iter;

      if( (iter = m_imapJoints.find(strName)) != m_imapJoints.end() )
      {
        return &(m_jointsArm[iter->second]);
      }
      else
      {
        return NULL;
      }
    }

    /*!
     * \brief Get robotic joint in arm+ee kinematic chain.
     *
     * \param nServoId  Master servo id (primary key).
     *
     * \return If found, returns pointer to joint. Otherwise returns NULL.
     */
    HekRobotJoint *getArmJoint(int nServoId)
    {
      MapRobotJoints::iterator  iter;

      if( (iter = m_jointsArm.find(nServoId)) != m_jointsArm.end() )
      {
        return &(iter->second);
      }
      else
      {
        return NULL;
      }
    }

    /*!
     * \brief Get robotic joint's master servo.
     *
     * \param joint     Robotic joint.
     *
     * \return If found, returns pointer to servo. Otherwise returns NULL.
     */
    DynaServo *getMasterServo(HekRobotJoint &joint)
    {
      return m_pDynaChain!=NULL? m_pDynaChain->GetServo(joint.m_nMasterServoId):
                                 NULL;
    }

    /*!
     * \brief Convert percent of maximum to joint velocity.
     *
     * The maximum joint velocity is the theoretical maximum joint velocity
     * based on joint servo specs of unloaded maximum RPMs and the Hekateros
     * external gear ratios.
     *
     * \param joint   Joint description.
     * \param fValPct Joint velocity (%).
     *
     * \return Joint velocity (radians/second).
     */
    double pctOfMaxJointVelocity(HekRobotJoint &joint, double fVelPct)
    {
      return fVelPct < 100.0? fVelPct/100.0 * joint.m_fMaxJointRadsPerSec:
                              joint.m_fMaxJointRadsPerSec;
    }

  protected:
    // state
    bool              m_bNoExec;        ///< do [not] execute physical movements
    HekDesc           m_descHek;        ///< \h_hek description
    HekRobotMode      m_eRobotMode;     ///< robot operating mode
    HekOpState        m_eOpState;       ///< arm operational state
    bool              m_bIsEStopped;    ///< arm is [not] emergency stopped
    bool              m_bAlarmState;    ///< robot is [not] alarmed
    bool              m_bAreServosPowered;///< arm servos are [not] driven
    bool              m_bAtBalancedPos; ///< arm is [not] at balanced position
    bool              m_bAtParkedPos;   ///< arm is [not] at parked position

    // tuning
    HekTunes          m_tunes;          ///< tune parameters

    // dynamixel i/f
    DynaComm         *m_pDynaComm;      ///< dynamixel communication
    DynaChain        *m_pDynaChain;     ///< dynamixel chain
    DynaBgThread     *m_pDynaBgThread;  ///< dynamixel background thread

    // joint descriptions
    MapRobotJoints    m_jointsArm;      ///< robot arm + end effector joints
    MapRobotJoints    m_jointsEquipDeck;///< robot equipement deck kin. joints
    MapRobotJoints    m_jointsAux;      ///< robot auxiliary kinematic joints
    IMapRobotJoints   m_imapJoints;     ///< joints indirect map

    // dynamics and kinematics 
    HekKinematics    *m_pKin;           ///< dynamics and kinematics

    // motion state
    HekJointTrajectoryPoint m_lastTrajArm; ///< last trajectory point for arm 

    // monitor
    HekMonitor        m_monitor;          ///< power, health, and safety monitor

    // asynchronous task control and synchronization
    HekAsyncTaskState m_eAsyncTaskState;  ///< asynchronous task state
    int               m_rcAsyncTask;      ///< last async task return code
    AsyncTaskId       m_eAsyncTaskId;     ///< asynchronous task id
    void             *m_pAsyncTaskArg;    ///< asynchronous argument
    pthread_t         m_threadAsync;      ///< async pthread identifier 
    pthread_mutex_t   m_mutex;            ///< synchronization mutex

    friend class HekCalib;
    friend class HekCalibStretch;

    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Scan Hardware Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Scan \h_hek hardware and match against description.
     *
     * \copydoc doc_return_std
     */
    int scanHw();

    /*!
     * \brief Scan \h_hek dynamixel bus hardware and match against description.
     *
     * \param nMaxTries   Maximums number of scan attempts before failing.
     *
     * \copydoc doc_return_std
     */
    int scanDynaBus(int nMaxTries);


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Convert Static Specifications Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Convert specification(s) to operational parameters.
     *
     * The specifications must be defined and the connecton to the \h_hek
     * hardware must be open.
     *
     * There are up to 3 kinematic chains, given the specifications and the
     * hardware:
     * \li arm + end effector
     * \li equipment deck effector
     * \li auxiliary effector
     *
     * \note A kinematic chain differs from the dynamixel chain. \h_hek
     * supports one dynamixel bus in which all servos hang. A dynamixel chain
     * defines this communication bus. Servos on the bus can be associated with
     * any of the above three kinematic chains.
     *
     * \copydoc doc_return_std
     */
    int convertSpecs();

    /*!
     * \brief Add a joint to robot's kinematic chain.
     *
     * \param [in] pSpecJoint   Pointer to joint spcecification.
     * \param [in] pSpecServo   Pointer to master servo spcecification.
     * \param [out] kin         Modified kinematics chain of joints.
     * \param [out] imap        Indirect map of kinematic chain.
     *
     * \copydoc doc_return_std
     */
    int addRobotJoint(HekSpecJoint_T  *pSpecJoint,
                      HekSpecServo_T  *pSpecServo,
                      MapRobotJoints  &kin,
                      IMapRobotJoints &imap);

    /*!
     * \brief Adjust tuning parameters for values in compiled product
     * specifications.
     */
    void adjustTuningFromSpecs();


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Hardware and Software Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Faux calibrate \h_hek.
     *
     * \par DANGER:
     * This function should in place of calibrate() for debugging.
     * Make suere the arm is placed at the zero point prior to starting.
     */
    void fauxcalibrate();

    /*!
     * \brief Configure \h_hek servos.
     *
     * Each servo's EEPROM and RAM control tables must match the joint
     * descriptions.
     *
     * \copydoc doc_return_std
     */
    int configServos();

    /*!
     * \brief Configure the EEPROM for all \h_hek servos in kinematic chain.
     *
     * For each servo, ensure that the proper servo EEPROM values are set.
     *
     * \param kin   Kinematic chain of joints.
     *
     * \copydoc doc_return_std
     */
    int configEEPROMForAllServos(MapRobotJoints &kin);

    /*!
     * \brief Configure one \h_hek servo EEPROM.
     *
     * A read, then write if necessary is performed to limit EEPROM wear.
     *
     * \note System reads, writes, and sleeps are pthread cancelation points.
     *
     * \param nServoId    Servo id.
     * \param joint       Associated robotic joint.
     *
     * \copydoc doc_return_std
     */
    int configEEPROMForServo(int nServoId, HekRobotJoint &joint);

    /*!
     * \brief Configure the RAM for all \h_hek servos in kinematic chain.
     *
     * For each servo, ensure that the proper servo RAM values are set.
     *
     * \param kin   Kinematic chain of joints.
     *
     * \copydoc doc_return_std
     */
    int configRAMForAllServos(MapRobotJoints &kin);

    /*!
     * \brief Configure one \h_hek servo RAM.
     *
     * \note System reads, writes, and sleeps are pthread cancelation points.
     *
     * \param nServoId    Servo id.
     * \param joint       Associated robotic joint.
     *
     * \copydoc doc_return_std
     */
    int configRAMForServo(int nServoId, HekRobotJoint &joint);

    /*!
     * \brief Mark all relevant joints for recalibration.
     *
     * \param bForceRecalib   If true, force recalibration all joints.
     *                        Otherwise calibrate only the uncalibrated.
     */
    void resetCalibStateForAllJoints(bool bForceRecalib);

    /*!
     * \brief Determine robot operational state from collective joint
     * operational states.
     *
     * \return Determined robot operational state synchronized with joint
     * states.
     */
    HekOpState determineRobotOpState();

    /*!
     * \brief Move servo to position.
     *
     * This call blocks until move is complete.
     *
     * \param pServo      Pointer to servo.
     * \param nOdGoalPos  Goal position in raw odometer units.
     * \param nSpeed      Speed in raw units.
     *
     * \return New odometer position.
     */
    int moveWait(DynaServo *pServo, int nOdGoalPos, int nSpeed);

    /*!
     * \brief Wait for the specified joints to stop moving.
     *
     * A maximum number of wait-test tries is performed until either all of the
     * joints haved stopped or the maximum number of tries has been reached. The
     * between-test wait is 10 milliseconds.
     *
     * \param vecNames  Vector of joint names.
     * \param nMaxTries Maximum number of test-wait tries.
     */
    void stopWait(const std::vector<std::string> &vecNames, int nMaxTries=10);

    /*!
     * \brief Callback to control wrist rotation.
     *
     * The wrist rotation is coupled to wrist pitch.
     * 
     * \param pUserArg  Point to this.
     */
    static void cbWristRot(void *pUserArg);

    /*!
     * \brief Move wrist rotation for any necessary final position adjustments.
     *
     * The wrist coupled pitch and rotation joints can have their respective
     * goal trajectories change by various actions.
     */
    void moveWristRot();

    /*!
     * \brief Set all joint goals to a null trajectory.
     *
     * A null trajectory for a joint is the joint's current position and zero
     * velocity.
     */
    void setAllJointGoalsToNull();

    /*!
     * \brief Set a set of joint goals to a null trajectory.
     *
     * A null trajectory for a joint is the joint's current position and zero
     * velocity.
     *
     * \param vecNames  Vector of joint names.
     */
    void setJointGoalsToNull(const std::vector<std::string> &vecNames);

    /*!
     * \brief Set joint goal to null a trajectory.
     *
     * A null trajectory for a joint is the joint's current position and zero
     * velocity.
     *
     * \param strName   Joint name.
     */
    void setJointGoalToNull(const std::string &strName);


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Asynchronous Operation Methods and Synchronization
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Create the asynchronous thread.
     *
     * \par Context:
     * Calling thread.
     *
     * \copydoc doc_return_std
     */
    int createAsyncThread();

    /*!
     * \brief Asynchronous operation thread.
     *
     * \param pArg   Thread argument (point to HekRobot object).
     * 
     * \return Returns NULL on thread exit.
     */
     static void *asyncThread(void *pArg);

    /*!
     * \brief Lock the robot mutex.
     *
     * The lock()/unlock() primitives provide thread-safe access.
     *
     * \par Context:
     * Any.
     */
    void lock()
    {
      pthread_mutex_lock(&m_mutex);
    }

    /*!
     * \brief Unlock the robot mutex.
     *
     * \par Context:
     * Any.
     */
    void unlock()
    {
      pthread_mutex_unlock(&m_mutex);
    }

    /*!
     * \brief Try to lock the robot mutex.
     *
     * \par Context:
     * Any.
     *
     * \return Returns true if lock acquired, else returns false.
     */
    bool trylock()
    {
      return pthread_mutex_trylock(&m_mutex) == 0? true: false;
    }
  };

} // namespace hekateros


#endif // _HEK_ROBOT_H
