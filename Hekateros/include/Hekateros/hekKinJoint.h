////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekKinJoint.h
//
/*! \file
 *
 * $LastChangedDate: 2015-02-10 13:39:03 -0700 (Tue, 10 Feb 2015) $
 * $Rev: 3866 $
 *
 * \brief Hekateros powered joint kinematics and dynamics control class
 * interface.
 *
 * \copyright
 *   \h_copy 2014-2017. RoadNarrows LLC.\n
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

#ifndef _HEK_KIN_JOINT_H
#define _HEK_KIN_JOINT_H

#include <sys/time.h>
#include <time.h>
#include <pthread.h>

#include <vector>
#include <deque>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaError.h"
#include "Dynamixel/DynaTypes.h"
#include "Dynamixel/DynaPid.h"
#include "Dynamixel/DynaServo.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekJoint.h"
#include "Hekateros/hekPid.h"
#include "Hekateros/hekUtils.h"

//
// Define or undef for
//
#define HEK_KIN_EXEC_ALG_SYNC    ///< synchronous move execution algorithm

#ifdef HEK_KIN_EXEC_ALG_SYNC
#undef  HEK_KIN_EXEC_ALG_INDIV   ///< individual move execution algorithm
#else
#define HEK_KIN_EXEC_ALG_INDIV   ///< individual move execution algorithm
#endif // HEK_KIN_EXEC_ALG_SYNC


namespace hekateros
{
  // ---------------------------------------------------------------------------
  // SyncMoveMsgs
  // ---------------------------------------------------------------------------

  /*!
   * \brief Container class to hold Dynamixel synchronous messages data.
   */
  class SyncMoveMsgs
  {
  public:
    /*!
     * \brief Default constructor.
     */
    SyncMoveMsgs();

    /*!
     * \brief Destructor.
     */
    virtual ~SyncMoveMsgs()
    {
    }

    /*!
     * \brief Clear sync messages.
     */
    void clear();

    /*!
     * \brief Add servo speed to speed tuple array.
     *
     * \param nServoId    Servo id.
     * \param nServoSpeed Servo speed (raw unitless).
     *
     * \return
     * On success, returns index of added tuple of speed tuple array.
     * On failures, -1.
     */
    int addSpeed(int nServoId, int nServoSpeed);

    /*!
     * \brief Add servo position to position tuple array.
     *
     * \param nServoId  Servo id.
     * \param nServoPos Servo position (odometer ticks).
     *
     * \return
     * On success, returns index of added tuple of position tuple array.
     * On failures, -1.
     */
    int addOdPos(int nServoId, int nServoPos);

    /*!
     * \brief Get the number of added speed tuples to speed tuple array.
     *
     * \return Count.
     */
    uint_t getSpeedTupleCount()
    {
      return m_uSpeedTupCount;
    }

    /*!
     * \brief Get the speed tuple array.
     *
     * \return Pointer.
     */
    DynaSpeedTuple_T *getSpeedTuples()
    {
      return m_tupSpeed;
    }

    /*!
     * \brief Get the number of added position tuples to position tuple array.
     *
     * \return Count.
     */
    uint_t getOdPosTupleCount()
    {
      return m_uOdPosTupCount;
    }

    /*!
     * \brief Get the position tuple array.
     *
     * \return Pointer.
     */
    DynaPosTuple_T *getOdPosTuples()
    {
      return m_tupOdPos;
    }

  protected:
    DynaSpeedTuple_T  m_tupSpeed[DYNA_ID_NUMOF];    ///< speed tuple array
    uint_t            m_uSpeedTupCount;             ///< speed tuple count
    DynaPosTuple_T    m_tupOdPos[DYNA_ID_NUMOF];    ///< position tuple array
    uint_t            m_uOdPosTupCount;             ///< positon tuple count
  };


  // ---------------------------------------------------------------------------
  // VelSpeedTuple
  // ---------------------------------------------------------------------------

  /*!
   * \brief Joint velocity to servo speed tuple class
   *
   * For each measured joint velocity (radians/second) the associated servo
   * speed (raw unitless) has been read.
   */
  class VelSpeedTuple
  {
  public:
    double  m_fJointVel;      ///< joint velocity (radians/second)
    int     m_nServoSpeed;    ///< associated servo speed (raw unitless)

    /*!
     * \brief Default constructor.
     */
    VelSpeedTuple();

    /*!
     * \brief Initialization constructor.
     */
    VelSpeedTuple(const double fJointVel, const int nServoSpeed);

    /*!
     * \brief Copy constructor.
     *
     * \param src   Source object to copy.
     */
    VelSpeedTuple(const VelSpeedTuple &src);

    /*!
     * \brief Destructor.
     */
    ~VelSpeedTuple()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs Right hand side object.
     *
     * \return Returns copy of this.
     */
    VelSpeedTuple operator=(const VelSpeedTuple &rhs);
  };
 

  // ---------------------------------------------------------------------------
  // VelSpeedLookupTbl Class
  // ---------------------------------------------------------------------------

  /*!
   * \brief Joint velocity/ servo speed lookup table class.
   *
   * A table of associated joint velocities to servo speed is dynamically built
   * and updated to provide:
   * \li Good estimates of servo speed given the goal velocity through linear
   * interpolation.
   * \li Adaptive kinodynamics.
   */
  class VelSpeedLookupTbl
  {
  public:
    /*! \brief Velocity-speed table type. */
    typedef std::vector<VelSpeedTuple>  VelSpeedTbl;

    /*! \brief No table entry value. */
    static const int NO_ENTRY = -1;

    /*! \brief Minimum servo speed for non-zero velocities. */
    static const int MIN_NON_ZERO_SPEED = 10;

    /*!
     * \brief Default constructor.
     */
    VelSpeedLookupTbl();

    /*!
     * \brief Destructor.
     */
    ~VelSpeedLookupTbl()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs Right hand side object.
     *
     * \return Returns copy of this.
     */
    VelSpeedLookupTbl operator=(const VelSpeedLookupTbl &rhs);

    /*!
     * \brief Create lookup table.
     *
     * \param fJointMaxVel  Joint maximum velocity (radians/second).
     * \param fBucketSize   Table bucket size (radians/second).
     */
    void create(double fJointMaxVel, double fBucketSize);
 
    /*!
     * \brief Hash joint velocity to table index.
     *
     * \param fJointVel   Joint velocity (radians/second).
     *
     * \return Table index.
     */
    int hash(double fJointVel);

    /*!
     * \brief Update lookup table entry.
     *
     * \param fJointVel   Joint velocity (radians/second).
     * \param nServoSpeed Servo speed (raw unitless).
     */
    void update(double fJointVel, int nServoSpeed);

    /*!
     * \brief Estimate servo goal speed.
     *
     * Linear interpolation is performed between the two nearest neighbors in
     * the lookup table.
     *
     * \param fJointGoalVel   Joint goal velocity (radians/second).
     *
     * \return Servo speed estimate.
     */
    int estimate(double fJointGoalVel);

    /*!
     * \brief Get the lookup table fixed size.
     *
     * \return int
     */
    int getLookupTableSize()
    {
      return (int)m_tbl.size();
    }

    /*!
     * \brief Get the number of populated entries in lookup table.
     *
     * \return int
     */
    int getNumLookupTableEntries()
    {
      return m_nNumEntries;
    }

    /*!
     * \brief Enable table debugging.
     * 
     * Debugging is printed to stderr. 
     * 
     * \param strId   Debug identification string.
     */
    void enableDebugging(const std::string &strId);

    /*!
     * \brief Disable table debugging.
     */
    void disableDebugging();

    /*!
     * \brief Dump lookup table entries.
     * 
     * Debugging is printed to stderr. 
     * 
     * \param strId   Debug identification string.
     */
    void dumpTable(const std::string &strId);

  protected:
    double        m_fBucketSize;    ///< table entry size (degrees/second)
    int           m_nMaxIndex;      ///< maximum index into table
    int           m_nNumEntries;    ///< number of populated table entries.
    VelSpeedTbl   m_tbl;            ///< dynamic interpolation table
    std::string   m_strDbgId;       ///< debugging id
    bool          m_bDbg;           ///< enable [disable] debugging
  };
 

  // ---------------------------------------------------------------------------
  // HekKinJoint Class
  // ---------------------------------------------------------------------------
 
  /*!
   * \brief Hekateros powered joint dynamics and kinematics control class.
   *
   * One or more motors can power a joint. Two or more joints may be coupled
   * (e.g. wrist pitch and rotation).
   * Motors can be actuators with built in controllers, DC motors, synthetic
   * muscle, or any other physical entity that can provide usefull force and be
   * controlled.
   *
   * Currently, only Dynamixels actuators (servos) are supported.
   */
  class HekKinJoint
  {
  public:
    /*!
     * \brief Position sliding window size for low-pass band filtering of input
     * positions.
     */
    static const size_t POS_WIN_SIZE = 6;

    /*!
     * \brief Delta t sliding window size for low-pass band filtering of input
     * delta times.
     */
    static const size_t DT_WIN_SIZE = 10;

    /*!
     * \brief Velocity sliding window size for low-pass band filtering of input
     * Velocities.
     */
    static const size_t VEL_WIN_SIZE = 6;

    // stop delta v as a fraction of max_delta_v tune parameter
    static const double SLOW_DERATE_DELTA_V = 1.0;

    /*!
     * \brief Torque sliding window size for low-pass band filtering of input
     * torques.
     */
    static const size_t TORQUE_WIN_SIZE = 4;

    /*!
     * \brief Torque control backoff speed (raw unitless)
     */
    static const int TORQUE_CTL_BACKOFF_SPEED = 100;

    /*!
     * \brief Torque control backoff positon (odometer ticks)
     */
    static const int TORQUE_CTL_BACKOFF_POS = 5;

    /*!
     * \brief Kinedynamics joint position and velocity control states.
     *
     * \note Actively apply torque control in an over torque condition is 
     * somewhat independent of the control states.
     */
    enum MoveState
    {
      MoveStateIdle,      ///< no active goal or stopping control 
      MoveStateNewMove,   ///< start move to new goal
      MoveStateMoving,    ///< actively moving to goal
      MoveStateStopping,  ///< actively stopping joint
    };
  
    /*!
     * \brief Default constructor.
     */
    HekKinJoint();

    /*!
     * \brief Initialization constructor.
     *
     * \param pJoint    Pointer to joint description.
     * \param pServo    Pointer to master servo controller.
     * \param tunes     Hekateros tuning parameters.
     */
    HekKinJoint(HekRobotJoint   *pJoint,
                DynaServo       *pServo,
                const HekTunes  &tunes);
  
    /*!
     * \brief Copy constructor.
     *
     * \param src   Source object.
     */
    //HekKinJoint(const HekKinJoint &src);

    /*!
     * \brief Destructor.
     */
    virtual ~HekKinJoint()
    {
      nlstop();
      pthread_mutex_destroy(&m_mutexSync);
    }
  
    /*!
     * \brief Couple joint.
     *
     * \param pKinJointCoupled  Base class joint has no coupled joint.
     */
    virtual void coupleJoint(HekKinJoint *)
    {
      // N/A
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return *this
     */
    HekKinJoint &operator=(const HekKinJoint &rhs);

    /*!
     * \brief Reload configuration tuning parameters.
     *
     * \param tunes     Hekateros tuning parameters.
     */
    virtual void reload(const HekTunes &tunes);


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Attribute and State Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Get move control state for this joint.
     *
     * \return MoveState
     */
    MoveState getMoveState()
    {
      return m_eState;
    }

    /*!
     * \brief Test if actively moving joint to goal.
     *
     * \return Returns true or false.
     */
    bool isMovingToGoal()
    {
      return m_eState == MoveStateNewMove || m_eState == MoveStateMoving;
    }

    /*!
     * \brief Test if actively stopping joint.
     *
     * \return Returns true or false.
     */
    bool isStopping()
    {
      return m_eState == MoveStateStopping;
    }

    /*!
     * \brief Test if actively moving joint.
     *
     * \return Returns true or false.
     */
    bool isMoving()
    {
      return isMovingToGoal() || isStopping();
    }

    /*!
     * \brief Test if joint is stopped (i.e. control is not active).
     *
     * \return Returns true if stopped, false otherwise.
     */
    virtual bool isStopped()
    {
      return m_eState == MoveStateIdle;
    }

    /*!
     * \brief Test if can stop now without slowing down.
     *
     * \return Returns true or false.
     */
    bool canStopNow()
    {
      return (m_pJoint->m_eOpState != HekOpStateCalibrated) ||
             (m_fStopVelTh == HekTunePidDeltaVNoMax) || 
             (fabs(m_fJointCurVel) <= m_fStopVelTh);
    }

    /*!
     * \brief Get the joint's position and velocity PID K parameters.
     *
     * \param [out] fKp   Proportional gain parameter.
     * \param [out] fKi   Integral gain parameter.
     * \param [out] fKd   Derivative gain parameter.
     */
    void getPidKParams(double &fKp, double &fKi, double &fKd) const
    {
      m_pid.getKParams(fKp, fKi, fKd);
    }
  
    /*!
     * \brief Set the joint's position and velocity PID K parameters.
     *
     * \param [in] fKp   Proportional gain parameter.
     * \param [in] fKi   Integral gain parameter.
     * \param [in] fKd   Derivative gain parameter.
     *
     * \return
     */
    void setPidKParams(const double fKp, const double fKi, const double fKd)
    {
      m_pid.setKParams(fKp, fKi, fKd);
    }
  
    /*!
     * \brief Get joint PID maximum delta v (radians/second) parameter.
     *
     * \return Maximum output delta velocity (radians/second).
     */
    double getPidMaxDeltaVParam() const
    {
      return m_pid.getMaxDeltaVParam();
    }

    /*!
     * \brief Set joint PID maximum delta v (radians/second) parameter.
     *
     * \param fMaxDeltaV  Maximum output delta velocity (radians/second).
     */
    void setPidMaxDeltaVParam(const double fMaxDeltaV)
    {
      m_pid.setMaxDeltaVParam(fMaxDeltaV);
    }

    /*!
     * \brief Get the position and velocity tolerance parameters.
     *
     * The joint PID will try to control to the goal position \h_plusmn position
     * tolerance and to the goal velocity \h_plusmn velocity tolerance.
     *
     * \param [out] fTolPos   Joint position tolerance (radians).
     * \param [out] fTolVel   Joint velocity tolerance (radians/second).
     */
    void getToleranceParams(double &fTolPos, double &fTolVel) const
    {
      fTolPos = m_fTolPos;
      fTolVel = m_fTolVel;
    }
  
    /*!
     * \brief Set the position and velocity tolerance parameters.
     *
     * The joint PID will try to control to the goal position \h_plusmn position
     * tolerance and to the goal velocity \h_plusmn velocity tolerance.
     *
     * \param [in] fTolPos   Joint position tolerance (radians).
     * \param [in] fTolVel   Joint velocity tolerance (radians/second).
     */
    void setToleranceParams(const double fTolPos, const double fTolVel)
    {
      m_fTolPos = fTolPos;
      m_fTolVel = fTolVel;
    }
  
    /*!
     * \brief Get the instantaneous current joint position and velocity.
     *
     * \param [out] fCurPos   Current joint position (radians).
     * \param [out] fCurVel   Current joint velocity (radians/second).
     */
    void getJointCurPosVel(double &fCurPos, double &fCurVel) const
    {
      fCurPos = m_fJointCurPos;
      fCurVel = m_fJointCurVel;
    }

    /*!
     * \brief Get the smoothed (filtered) current joint position and velocity.
     *
     * \param [out] fCurPos   Current joint position (radians).
     * \param [out] fCurVel   Current joint velocity (radians/second).
     */
    void getFilteredJointCurPosVel(double &fCurPos, double &fCurVel) const
    {
      fCurPos = m_fJointPosOut;
      fCurVel = m_fJointVelOut;
    }

    /*!
     * \brief Get the instantaneous current servo position and speed.
     *
     * \param [out] nServoCurPos    Current servo position (odometer ticks).
     * \param [out] nServoCurSpeed  Current servo velocity (raw unitless).
     */
    void getServoCurPosSpeed(int &nServoCurPos, int &nServoCurSpeed) const
    {
      nServoCurPos   = m_nServoCurPos;
      nServoCurSpeed = m_nServoCurSpeed;
    }

    /*!
     * \brief Get the PID output target joint velocity and the corresponding
     * servo speed estemate.
     *
     * \param [out] fTgtVel     Target joint velocity (radians/second).
     * \param [out] nTgtSpeed   Target servo speed (raw unitless).
     */
    void getTgtVelSpeed(double &fTgtVel, int &nTgtSpeed) const
    {
      fTgtVel   = m_fJointTgtVel;
      nTgtSpeed = m_nServoTgtSpeed;
    }

    /*!
     * \brief Test if joint is in an over torque condition.
     *
     * \return Returns true if in condition, false otherwise.
     */
    virtual bool hasOverTorqueCondition()
    {
      return m_bOverTorqueCond;
    }

    /*!
     * \brief Convert joint position to the equivalent servo position.
     *
     * \param fPos  Joint position (radians).
     *
     * \return Equivalent servo position (odometer ticks).
     */
    virtual int jointPosToServoPos(const double fPos);

    /*!
     * \brief Convert servo position to the equivalent joint position.
     *
     * \param nOdPos  Servo position (odometer ticks).
     *
     * \return Equivalent joint position (radian).
     */
    virtual double servoPosToJointPos(const int nOdPos);


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Kinodynamics Methods
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Reset servo odometer zero point to the current encoder position.
     *
     * \copydoc doc_return_std
     */
    virtual int resetServoOdometer();

    /*!
     * \brief Specify a new joint goal position and velocity move.
     *
     * The joint PID will begin to control to the new goals, smoothly
     * transitioning (I hope) from the current goals.
     *
     * \param fGoalPos    Joint goal position (radians).
     * \param fGoalVel    Joint goal velocity (radians/second).
     *
     * \copydoc doc_return_std
     */
    int specifyMove(const double fGoalPos, const double fGoalVel);

    /*!
     * \brief Sense (read) the current servo dynamics and transform to useful
     * state.
     *
     * \param bIsControlling    Joint is [not] being actively controlled.
     *
     * \copydoc doc_return_std
     */
    virtual int senseDynamics(bool bIsControlling);

    /*!
     * \brief React to recently read sensor data.
     *
     * \param bIsControlling  Joint is [not] being actively controlled.
     *                        However, reaction may still be desired.
     * \param msgs            Synchronous move messages container object.
     *
     * \copydoc doc_return_std
     */
    virtual int react(bool bIsControlling, SyncMoveMsgs &msgs);

    /*!
     * \brief Plan motion.
     *
     * \param bIsControlling  Joint is [not] being actively controlled.
     *                        However, planning may still be desired.
     * \param msgs            Synchronous move messages container object.
     *
     * \return Move control state after call.
     */
    virtual MoveState planMotion(bool bIsControlling, SyncMoveMsgs &msgs);

    /*!
     * \brief Act (write) to effect the servo dynamics for torque and motion
     * control.
     *
     * \copydoc doc_return_std
     */
    virtual int act();

    /*!
     * \brief Apply torque limit control.
     *
     * \copydoc doc_return_std
     */
    virtual int applyTorqueControl();
  
    /*!
     * \brief Move joint using joint PID controller.
     *
     * Once the move has reached the goal joint position, within tolerence,
     * the move will automatically stop.
     *
     * \return Move control state after call.
     */
    virtual MoveState move();
  
    /*!
     * \brief Stop movement and move control of the joint.
     *
     * \copydoc doc_return_std
     */
    virtual int stop();

  protected:
    // base
    MoveState       m_eState;     ///< joint control state
    pthread_mutex_t m_mutexSync;  ///< synchonization mutex
    HekRobotJoint*  m_pJoint;     ///< joint description
    DynaServo*      m_pServo;     ///< joint master servo control

    // joint tolerances, thresholds, and pid
    double      m_fTolPos;    ///< joint position \h_plusmn tolerance (rads)
    double      m_fTolVel;    ///< joint velocity \h_plusmn tolerance (rads/s)
    double      m_fStopVelTh; ///< joint velocity threshold to safely stop move
    HekPid      m_pid;        ///< joint position and velocity PID

    // position state
    double      m_fJointRadPerTick; ///< joint radians / odometer tick
    double      m_fJointGoalPos;    ///< joint goal position (radians)
    int         m_nServoGoalPos;    ///< servo goal position (odometer ticks)
    double      m_fJointCurPos;     ///< joint current position (radians)
    int         m_nServoCurPos;     ///< servo current position (odometer ticks)
    double      m_fJointPrevPos;    ///< joint current position (radians)
    int         m_nServoPrevPos;    ///< servo previous pos (odometer ticks)
    double      m_fJointPosOut;     ///< low-pass filtered joint cur position
    std::deque<double> m_histPosIn; ///< recent history of joint positions

    // time
    double          m_fDt;          ///< delta time between positions reads
    struct timespec m_tsPrev;       ///< previous time mark
    double          m_fDtAvg;       ///< average delta t
    std::deque<double> m_histDtIn;  ///< sliding window of delta t's

    // velocity state
    double      m_fJointGoalVel;    ///< joint goal velocity (radians/second)
    double      m_fJointCurVel;     ///< joint current velocity (radians/second)
    int         m_nServoCurSpeed;   ///< current motor speed (raw unitless)
    double      m_fVelDerate;       ///< joint velocity derate (normalized)
    VelSpeedLookupTbl m_tblVelSpeed; ///< dynamic velocity/speed lookup table
    double      m_fJointVelOut;     ///< low-pass filtered joint cur velocity
    std::deque<double> m_histVelIn; ///< sliding window of current velocities

    // pid output state
    double      m_fJointTgtVel;     ///< joint target velocity (radians/second)
    int         m_nServoTgtSpeed;   ///< target motor speed (raw unitless)

    // torque state 
    bool        m_bOverTorqueCond;  ///< is [not] in torque overload condition
    double      m_fOverTorqueTh;    ///< set over torque condition threshold
    double      m_fClearTorqueTh;   ///< clear over torque condition threshold
    int         m_nServoCurLoad;    ///< servo current load (raw unitless)
    double      m_fTorqueOut;       ///< low-pass filtered torque
    std::deque<double> m_histTorqueIn; ///< recent history of joint torques

    /*!
     * \brief Lock the joint kinematics.
     *
     * The calling thread will block while waiting for the mutex to become 
     * available. Once locked, the Hekateros kinematics thread will block when
     * accessing this joint.
     *
     * The lock()/unlock() primitives provide a safe mechanism to modify the 
     * joint data, such as the next goals.
     *
     * \par Context:
     * Any.
     */
    void lock()
    {
      pthread_mutex_lock(&m_mutexSync);
    }
  
    /*!
     * \brief Unlock the joint kinematics.
     *
     * The Hekateros kinematics thread will be available to run when accessing
     * this joint.
     *
     * \par Context:
     * Any.
     */
    void unlock()
    {
      pthread_mutex_unlock(&m_mutexSync);
    }

    /*!
     * \brief Calculate the delta time between calls to this.
     *
     * \return Delta time secs.frac (seconds).
     */
    double delta_t();

    /*!
     * \brief Apply a running average for the delta times (seconds).
     *
     * A simple moving average (SMa) of delta times with a window size of
     * \ref DT_WIN_SIZE is used.
     *
     * \param fDtAvg  Current delta time average (seconds).
     * \param fDt     New delta time (seconds).
     *
     * \return New average delta time (seconds).
     */
    double averageDt(double fDtAvg, double fDt);

    /*!
     * \brief De-jitter servo odometer position.
     *
     * The servo can report encoder position of \plusmn 1.
     *
     * \param nServoCurPos  Servo current odometer position.
     * \param nServoPrevPos Servo previous odometer position.
     *
     * \return Return de-jittered current servo position.
     */
    int dejitterServoPos(int nServoCurPos, int nServoPrevPos);

    /*!
     * \brief Apply a low-pass band filter on the joint positions (radians).
     *
     * A simple moving average (SMA) of positions with a window size of
     * \ref POS_WIN_SIZE is used as the filter.
     *
     * \param fPosAvg     Current joint position average (radians).
     * \param fJointPos   New joint position (radians).
     *
     * \return New filtered joint position (radians).
     */
    double filterPositions(double fPosAvg, double fJointPos);

    /*!
     * \brief Apply a low-pass band filter on the joint velocities
     * (radians/second).
     *
     * A simple moving average (SMA) of velocities with a window size of
     * \ref VEL_WIN_SIZE is used as the filter.
     *
     * \param fVelAvg     Current joint velocitie average (radians/second).
     * \param fJointVel   New joint velocity (radians/second).
     *
     * \return New filtered joint velocity (radians/second).
     */
    double filterVelocities(double fVelAvg, double fJointVel);

    /*!
     * \brief Apply a low-pass band filter on the sensed torques (loads).
     *
     * The torques are interpreted from the read servo loads which are unitless
     * values (and probably non-linear).
     *
     * A simple moving average (SMA) of load values with a window size of
     * \ref TORQUE_WIN_SIZE is used as the filter.
     *
     * \param fTorqueAvg  Current torque (load) average value.
     * \param nServoLoad  New servo load value (raw unitless).
     *
     * \return New torque average (raw unitless).
     */
    double filterTorques(double fTorqueAvg, int nServoLoad);

    /*!
     * \brief Get the current goal rotation direction.
     *
     * The direction is deterimed by the difference of current odometer value
     * and the goal position.
     *
     * Normally counter-clockwise is positive, clockwise is negative, but the
     * sence can be reversed in the odometer configuration.
     *
     * \return 1 or -1.
     */
    int getGoalDir();
  
    /*!
     * \brief Get the planned target rotation direction.
     *
     * The direction is deterimed sign of the servo speed.
     *
     * Normally counter-clockwise is positive, clockwise is negative, but the
     * sence can be reversed in the odometer configuration.
     *
     * \param nServoTgtSpeed    Planned servo target speed (raw unitless).
     *                          Odometer configuration is already added!
     *
     * \return 1 or -1.
     */
    int getPlannedDir(int nServoTgtSpeed);
  
    /*!
     * \brief Test if joint can move in direction of goal position.
     *
     * The joint must not be in an over torque condition or the move will ease
     * the torque, not exacerbate the condition.
     *
     * \return Returns true or false.
     */
    bool canMoveInGoalDir();

    /*!
     * \brief Test if joint can move in planned direction.
     *
     * The joint must not be in an over torque condition or the move will ease
     * the torque, not worsen the condition.
     *
     * \param nServoTgtSpeed    Planned servo target speed (raw unitless).
     *
     * \return Returns true or false.
     */
    bool canMoveInPlannedDir(int nServoTgtSpeed);

    /*!
     * \brief Calculate the joint velocity from delta servo positions.
     *
     * \param fPos1   Later joint position (radians/second).
     * \param fPos0   Earlier joint position (radians/second).
     * \param fDt     Delta time in seconds.
     *
     * \return Calculated joint velocity (radians).
     */
    virtual double jointVelocity(double fPos1, double fPos0, double fDt);
  
    /*!
     * \brief Update joint velocity - servo speed association.
     *
     * \param fJointVel   Joint velocity (radians/second).
     * \param nServoSpeed Servo speed (raw unitless).
     */
    virtual void updateAssoc(double fJointVel, int nServoSpeed);
    
    /*!
     * \brief Estimate of target servo speed from target joint velocity.
     *
     * The Dynamixel speed value is a raw, unitless (and probably non-linear)
     * value. The estimate is made using linear interpolation from a dynamically
     * maintained velocity,speed lookup table.
     *
     * \param fJointTgtVel  Target joint velocity (radians/seconds).
     *
     * \return Servo speed estimate matching target velocity.
     */
    virtual int estimateServoTgtSpeed(double fJointTgtVel);

    /*!
     * \brief Simple estimate of target servo speed from target joint velocity.
     *
     * The Dynamixel speed value is a raw, unitless (and probably non-linear)
     * value. The estimate is made by interpolating the current
     * speed between 0 and the maximum unloaded joint velocity.
     *
     * \param fJointTgtVel  Target joint velocity (radians/seconds).
     *
     * \return Servo speed estimate matching target velocity.
     */
    virtual int estimateServoTgtSpeedSimple(double fJointTgtVel);

    /*!
     * \brief Move servo.
     *
     * \param nServoGoalSpeed Servo goal speed (raw unitless).
     * \param nServoGoalPos   Servo goal position (odometer ticks).
     * \param bNewMove        This is [not] a new move.
     */
    void moveServo(int nServoGoalSpeed, int nServoGoalPos);


    /*!
     * \brief Add servo to synchronous move message(s).
     *
     * \param msgs            Synchronous move messages container object.
     * \param nServoGoalSpeed Servo goal speed (raw unitless).
     * \param nServoGoalPos   Servo goal position (odometer ticks).
     */
    void addServoMsgEntries(SyncMoveMsgs &msg,
                            int           nServoGoalSpeed,
                            int           nServoGoalPos);

    /*!
     * \brief Specify a new joint goal position and velocity move
     * (no lock version).
     *
     * The joint PID will begin to control to the new goals, smoothly
     * transitioning (I hope) from the current goals.
     *
     * \param fGoalPos    Joint goal position (radians).
     * \param fGoalVel    Joint goal velocity (radians/second).
     *
     * \copydoc doc_return_std
     */
    virtual int nlspecifyMove(const double fGoalPos, const double fGoalVel);

    /*!
     * \brief Apply torque limit control (no lock version).
     *
     * \copydoc doc_return_std
     */
    virtual int nlapplyTorqueControl();
  
    /*!
     * \brief Move joint using joint PID controller (no lock version).
     *
     * Once the move has reached the goal joint position, within tolerence,
     * the move will automatically stop.
     *
     * \return Move control state after move call.
     */
    virtual MoveState nlmove();
  
    /*!
     * \brief Stop movement and move control of the joint (no lock version).
     *
     * \copydoc doc_return_std
     */
    virtual int nlstop();

    /*!
     * \brief Slow to stop (no lock version).
     *
     * \param msgs  Synchronous move messages container object.
     *
     * \copydoc doc_return_std
     */
    virtual int nlslowToStop(SyncMoveMsgs &msg);
  };


  // ---------------------------------------------------------------------------
  // HekKinJointWristRot Class
  // ---------------------------------------------------------------------------
 
  /*!
   * \brief Special wrist rotation joint kinematics and dynamics class.
   *
   * The Hekateros wrist joints are coupled in that wrist pitch effect wrist
   * rotation. This class "decouples" these joints in software.
   */
  class HekKinJointWristRot : public HekKinJoint
  {
  public:
    /*!
     * \brief Default constructor.
     */
    HekKinJointWristRot() : HekKinJoint()
    {
      m_pKinJointWristPitch = NULL;
      m_eStateWristPitch    = MoveStateIdle;
    }

    /*!
     * \brief Initialization constructor.
     *
     * \param pJoint    Pointer to joint description.
     * \param pServo    Pointer to master servo controller.
     * \param tunes     Hekateros tuning parameters.
     */
    HekKinJointWristRot(HekRobotJoint   *pJoint,
                        DynaServo       *pServo,
                        const HekTunes  &tunes) :
      HekKinJoint(pJoint, pServo, tunes)
    {
      m_pKinJointWristPitch = NULL;
      m_eStateWristPitch    = MoveStateIdle;
    }
  
    /*!
     * \brief Destructor.
     */
    virtual ~HekKinJointWristRot()
    {
    }
  
    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return *this
     */
    HekKinJointWristRot &operator=(const HekKinJointWristRot &rhs)
    {
      m_pKinJointWristPitch = rhs.m_pKinJointWristPitch;
      m_eStateWristPitch    = rhs.m_eStateWristPitch;

      return *this;
    }

    /*!
     * \brief Couple joint.
     *
     * \param pKinJointWristPitch   Wrist pitch joint kinematics.
     */
    virtual void coupleJoint(HekKinJoint *pKinJointWristPitch)
    {
      m_pKinJointWristPitch = pKinJointWristPitch;
      if( m_pKinJointWristPitch != NULL )
      {
        m_eStateWristPitch = m_pKinJointWristPitch->getMoveState();
      }
    }

    /*!
     * \brief React to recently read sensor data.
     *
     * \param bIsControlling  Joint is [not] being actively controlled.
     *                        However, reaction may still be desired.
     * \param msgs            Synchronous move messages container object.
     *
     * \copydoc doc_return_std
     */
    virtual int react(bool bIsControlling, SyncMoveMsgs &msgs);

    /*!
     * \brief Plan motion.
     *
     * \param bIsControlling  Joint is [not] being actively controlled.
     *                        However, planning may still be desired.
     * \param msgs            Synchronous move messages container object.
     *
     * \return Move control state after call.
     */
    virtual MoveState planMotion(bool bIsControlling, SyncMoveMsgs &msgs);

    /*!
     * \brief Act (write) to effect the servo dynamics for torque and motion
     * control.
     *
     * Since the wrist rotation is coupled to wrist pitch, if the pitch is
     * moving, so must the rotation.
     *
     * \copydoc doc_return_std
     */
    virtual int act();
  
    /*!
     * \brief Convert joint position to the equivalent servo position.
     *
     * Adjust for wrist pitch.
     *
     * \param fPos  Joint position (radians).
     *
     * \return Equivalent servo position (odometer ticks).
     */
    virtual int jointPosToServoPos(double fPos);

    /*!
     * \brief Convert servo position to the equivalent joint position.
     *
     * Adjust for wrist pitch.
     *
     * \param nOdPos  Servo position (odometer ticks).
     *
     * \return Equivalent joint position (radian).
     */
    virtual double servoPosToJointPos(int nOdPos);

  protected:
    HekKinJoint  *m_pKinJointWristPitch;  ///< wrist pitch joint description
    MoveState     m_eStateWristPitch;     ///< joint control state

    /*!
     * \brief Update joint velocity - servo speed association.
     *
     * Adjustment is made for wrist pitch current velocity.
     *
     * \param fJointVel   Joint velocity (radians/second).
     * \param nServoSpeed Servo speed (raw unitless).
     */
    virtual void updateAssoc(double fJointVel, int nServoSpeed);

    /*!
     * \brief Get coupled joint's move state.
     *
     * \return State.
     */
    MoveState getCoupledJointState();

    /*!
     * \brief Test if coupled joint is stopped.
     *
     * \return Returns true if stopped, false otherwise.
     */
    virtual bool isCoupledJointStopped()
    {
      return getCoupledJointState() == MoveStateIdle;
    }

    /*!
     * \brief Test if coupled joint is moving.
     *
     * \return Returns true if moving, false otherwise.
     */
    virtual bool isCoupledJointMoving()
    {
      return !isCoupledJointStopped();
    }

    /*!
     * \brief Move joint using joint PID controller (no lock version).
     *
     * Once the move has reached the goal joint position, within tolerence,
     * the move will automatically stop.
     *
     * Since the wrist rotation is coupled to wrist pitch, if the pitch is
     * moving, so must the rotation.
     *
     * \return Move control state after call.
     */
    virtual MoveState nlmove();

    /*!
     * \brief Slow to stop (no lock version).
     *
     * Since the wrist rotation is coupled to wrist pitch, if the pitch is
     * moving, so must the rotation. Slowing down must take this coupled action
     * into account.

     * \param msgs  Synchronous move messages container object.
     *
     * \copydoc doc_return_std
     */
    virtual int nlslowToStop(SyncMoveMsgs &msg);
  };

} // namespace hekateros

#endif // _HEK_KIN_JOINT_H
