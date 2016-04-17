////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekTune.h
//
/*! \file
 *
 * $LastChangedDate: 2015-06-03 15:37:18 -0600 (Wed, 03 Jun 2015) $
 * $Rev: 4012 $
 *
 * \brief Hekateros tuning.
 *
 * All tuning defaults defined below can be overridden in the
 * /etc/hek_tune.conf XML file.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2014.  RoadNarrows LLC
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

#ifndef _HEK_TUNE_H
#define _HEK_TUNE_H

#include <iostream>
#include <string>
#include <map>

#include "rnr/rnrconfig.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekUtils.h"

namespace hekateros
{
  //----------------------------------------------------------------------------
  // Hekateros Tuning Defaults
  //----------------------------------------------------------------------------
 
  /*!
   * defgroup hek_tunes
   * \{
   */

  // . .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .
  // Kinematics Thread Run Rate Tuning

  /*!
   * \brief Default kinematics thread cycle rate (Hertz).
   * 
   * All dynamics and kinematics tasks exectute per each cycle. 
   *
   * \verbatim
   * kin_hz = tune_kin_hz
   * T_task = 1/kin_hz * 1/(numof_joints + 1)
   *   Where each joint dynamics is monitored and controlled each cycle and the
   *   extra task is for health monitoring.
   * \endverbatim
   *
   * Range: \h_ge \ref HekTuneKinHzMin
   * Scope: global
   */
  static const double HekTuneKinHzDft = 30.0;

  /*!
   * \brief Minimum kinematics thread cycle rate (Hertz).
   *
   * Gotta give me something boys!
   */
  static const double HekTuneKinHzMin = 0.1;


  // . .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .
  // Over Torque Threshold Tuning

  /*!
   * \brief Default joint over torque threshold (% of maximum)
   *
   * Default default for all arm joints excluding end effector. The actual
   * defaults are specified in the compiled product spec on a per joint basis.
   *
   * The working value is converted to raw motor load values internally.
   *
   * \verbatim
   * over_torque_th = tune_over_torque_th / 100.0 * max_motor_load(joint)
   * \endverbatim
   *
   * Range: [\ref HekTuneOverTorqueThMin, \ref HekTuneOverTorqueThMax]
   * Scope: per joint
   */
  static const double HekTuneOverTorqueThDft = 80.0;

  /*!
   * \brief Default end effector over torque threshold (% of maximum)
   *
   * Default default for all end effector joints. The actual defaults are
   * specified in the compiled product spec on a per joint basis.
   *
   * The working value is converted to raw motor load values internally.
   *
   * \verbatim
   * over_torque_th = tune_over_torque_th / 100.0 * max_motor_load(joint)
   * \endverbatim
   *
   * Range: [\ref HekTuneOverTorqueThMin, \ref HekTuneOverTorqueThMax]
   * Scope: global
   */
  static const double HekTuneEEOverTorqueThDft = 40.0;

  /*!
   * \brief Minimum joint over torque threshold (% of maximum)
   */
  static const double HekTuneOverTorqueThMin = 10.0;

  /*!
   * \brief Maximum joint over torque threshold (% of maximum)
   */
  static const double HekTuneOverTorqueThMax = 100.0;


  // . .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .
  // Clear Over Torque Condition Threshold Offset Tuning

  /*!
   * \brief Default clear torque condition hysteresis threshhold offset
   * (% of over torque threshold).
   *
   * The fraction is of the joint's over torque threshold. Once the servos
   * associtated with a joint enter the over torque condition, the condition
   * does not clear until torque is below the clear torque threshold. 
   *
   * The working value is converted to raw motor load values internally.
   *
   * \verbatim
   * clear_torque_offset = tune_clear_torque_offset / 100.0
   * clear_over_torque_cond_th = over_torque_th * clear_torque_offset
   * \endverbatim
   *
   * Range: [\ref HekTuneClearTorqueOffsetMin, \ref HekTuneClearTorqueOffsetMax]
   * Scope: global
   */
  static const double HekTuneClearTorqueOffsetDft = 90.0;

  /*!
   * \brief Minimum clear torque condition hysteresis threshhold offset
   * (% of over torque threshold).
   */
  static const double HekTuneClearTorqueOffsetMin = 10.0;

  /*!
   * \brief Maximum clear torque condition hysteresis threshhold offset
   * (% of over torque threshold).
   */
  static const double HekTuneClearTorqueOffsetMax = 99.5;


  // . .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .
  // Derated Velocity Tuning

  /*!
   * \brief Default Hekateros robot velocity derate (% of goal velocities).
   *
   * When a new move command is received, all of the goal velocities are
   * multiplied by this derated value.
   *
   * The working value is converted to a normalized value internally.
   *
   * \verbatim
   * velocity_derate = tune_velocity_derate / 100.0
   * joint_goal_vel = joint_goal_vel * velocity_derate
   * \endverbatim
   *
   * Range: [\ref HekTuneVelDerateMin, \ref HekTuneVelDerateMax]
   * Scope: global
   */
  static const double HekTuneVelDerateDft = 100.0;

  /*!
   * \brief Minimum robot velocity derate (% of goal velocities).
   */
  static const double HekTuneVelDerateMin = 10.0;

  /*!
   * \brief Maximum robot velocity derate (% of goal velocities).
   */
  static const double HekTuneVelDerateMax = 100.0;


  // . .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .
  // Position Tolerance Tuning

  /*!
   * \brief Default joint position control tolerance (degrees).
   *
   * A joint goal position that is within the position tolerance of the current
   * joint position is considered identical. If the joint was moving to the 
   * goal position, the joint will be automatically stopped.
   *
   * The working value is converted to radians internally.
   *
   * \verbatim
   * tol_pos = deg_to_rad(tune_tol_pos)
   * |joint_goal_pos - joint_cur_pos| < tol_pos <==> same position.
   * \endverbatim
   *
   * Range: \h_ge \ref HekTuneTolPosMin
   * Scope: per joint
   */
  static const double HekTuneTolPosDft = 0.2;
  
  /*!
   * \brief Minimum joint position control tolerance (degrees).
   */
  static const double HekTuneTolPosMin = 0.0;


  // . .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .
  // Velocity Tolerance Tuning

  /*!
   * \brief Default joint velocity control tolerance (degrees/second).
   *
   * A joint goal velocity that is within the velocity tolerance of the current
   * joint velocity is considered identical. No speed updates will be written
   * to the servo contoller.
   *
   * The working value is converted to radians/second internally.
   *
   *
   * \verbatim
   * tol_vel = deg_to_rad(tune_tol_vel)
   * |joint_goal_vel - joint_cur_vel| < tol_vel <==> same velocity.
   * \endverbatim
   *
   * Range: \h_ge \ref HekTuneTolVelMin
   * Scope: per joint
   */
  static const double HekTuneTolVelDft = 1.0;
  
  /*!
   * \brief Minimum joint velocition control tolerance (degrees/second).
   */
  static const double HekTuneTolVelMin = 0.0;


  // . .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .
  // Position and Velocity PID Tuning

  /*!
   * \brief Default joint position and velocity PID proportional constant.
   *
   * Range: \h_ge \ref HekTunePidKMin
   * Scope: per joint
   */
  static const double HekTunePidKpDft = 3.0;

  /*!
   * \brief Default joint position and velocity PID integral constant.
   *
   * Range: \h_ge \ref HekTunePidKMin
   * Scope: per joint
   */
  static const double HekTunePidKiDft = 0.05;

  /*!
   * \brief Default joint position and velocity PID derivative constant.
   *
   * Range: \h_ge \ref HekTunePidKMin
   * Scope: per joint
   */
  static const double HekTunePidKdDft = 0.1;

  /*!
   * \brief Minimum PID K constant value.
   */
  static const double HekTunePidKMin = 0.0;

  /*!
   * \brief Maximum PID delta V output (degrees/second)
   *
   * Range: \ref HekTunePidDeltaVNoMax or \h_gt 0.0
   * Scope: per joint
   */
  static const double HekTunePidMaxDeltaVDft = 30.0;

  /*!
   * \brief No maximum PID delta V output special value.
   */
  static const double HekTunePidDeltaVNoMax = 0;


  // . .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .
  // Trajectory Tuning

  /*!
   * \brief Default trajectory norm.
   *
   * Range: \ref HekNormL1, \ref HekNormL2, \ref HekNormLinf
   * Scope: global
   */
  static const HekNorm HekTuneTrajNormDft = HekNormLinf;

  /*!
   * \brief Default trajectory distance epsilon (degrees).
   *
   * Trajectory waypoints distances within epsilon are considered reached.
   *
   * The working value is converted to radians internally.
   *
   * \verbatim
   * ||WP-P|| < epsilon <==> waypoint reached
   *   Where WP is the vector of joint waypoint positions and P is the vector
   *   of current joint positions.
   * \endverbatim
   *
   * Range: \h_ge \ref HekTuneTrajEpsilonMin
   */
  static const double HekTuneTrajEpsilonDft = 20.0;

  /*!
   * \brief Minimum epsilon value.
   */
  static const double HekTuneTrajEpsilonMin = 0.0;

  /*!
   * \}
   */


  //----------------------------------------------------------------------------
  // HekTunesJoint Class
  //----------------------------------------------------------------------------
  
  /*!
   * \brief Hekateros tuning per joint data class.
   */
  class HekTunesJoint
  {
  public:
    double  m_fTolPos;        ///< position tolerance (radians)
    double  m_fTolVel;        ///< velocitiy tolerance (radians/second)
    double  m_fPidKp;         ///< position and velocity PID proportional const
    double  m_fPidKi;         ///< position and velocity PID integral constant
    double  m_fPidKd;         ///< position and velocity PID derivative constant
    double  m_fPidMaxDeltaV;  ///< max output delta v (radians/second)
    double  m_fOverTorqueTh;  ///< over torque conditon threshold (fraction)

    /*!
     * \brief Default constructor.
     */
    HekTunesJoint();

    /*!
     * \brief Destructor.
     */
    ~HekTunesJoint()
    {
    }

    /*!
     * \brief Assignment operator
     *
     * \param rhs   Right hand side object.
     *
     * \return *this
     */
    HekTunesJoint &operator=(const HekTunesJoint &rhs);
  };


  //----------------------------------------------------------------------------
  // HekTunes Class
  //----------------------------------------------------------------------------
  
  /*!
   * \brief Hekateros tuning data class.
   */
  class HekTunes
  {
  public:
    /*! Map of joint tuning parameters. */
    typedef std::map<std::string, HekTunesJoint>  MapJointTunes;

    // global tuning
    double  m_fKinematicsHz;      ///< kinematic thread rate (hertz)
    double  m_fClearTorqueOffset; ///< clear over torque condition offset (frac)
    double  m_fVelDerate;         ///< velocity derate (fraction)
    HekNorm m_eTrajNorm;          ///< trajectory distanct norm
    double  m_fTrajEpsilon;       ///< trajectory epsilon distance (radians)

    // per joint tuning
    MapJointTunes m_mapJointTunes;    ///< per joint tuning

    /*!
     * \brief Default constructor.
     */
    HekTunes();

    /*!
     * \brief Destructor.
     */
    ~HekTunes()
    {
    }

    /*!
     * \brief Get kinematics thread cycle rate tune parameter (hertz).
     *
     * \return Hertz.
     */
    double getKinematicsHz() const
    {
      return m_fKinematicsHz;
    }

    /*!
     * \brief Get derated velocity tune parameter (normalized).
     *
     * \return Derated value [0, 1].
     */
    double getVelocityDerate() const
    {
      return m_fVelDerate;
    }

    /*!
     * \brief Get trajectory tune parameters.
     *
     * \param [out] eNorm     Distance norm.
     * \param [out] fEpsilon  Waypoint precision (radians)
     */
    void getTrajectoryParams(HekNorm &eNorm, double &fEpsilon) const
    {
      eNorm     = m_eTrajNorm;
      fEpsilon  = m_fTrajEpsilon;
    }

    /*!
     * \brief Get joint tolerance tune parameters.
     *
     * \param strJointName    Name of joint.
     * \param [out] fTolPos   Position tolerance (radians)
     * \param [out] fTolVel   Velocity tolerance (radians/second)
     */
    void getToleranceParams(const std::string &strJointName,
                            double &fTolPos, double &fTolVel) const;

    /*!
     * \brief Get joint PID K tune parameters.
     *
     * \param strJointName  Name of joint.
     * \param [out] fKp     Proportional constant.
     * \param [out] fKi     Integral constant.
     * \param [out] fKd     Derivative constant.
     */
    void getPidKParams(const std::string &strJointName,
                       double &fKp, double &fKi, double &fKd) const;

    /*!
     * \brief Get joint PID maximum delta v (radians/second) tune parameter.
     *
     * \param strJointName  Name of joint.
     *
     * \return Maximum output delta velocity (radians/second).
     */
    double getPidMaxDeltaV(const std::string &strJointName) const;

    /*!
     * \brief Get joint torque parameters
     *
     * \param strJointName          Name of joint.
     * \param [out] fOverTorqueTh   Over torque threshold (normalized).
     * \param [out] fClearTorqueTh  Clear over torque condition threshold
     *                              (normalized).
     */
    void getTorqueParams(const std::string &strJointName,
                         double &fOverTorqueTh,
                         double &fClearTorqueTh) const;
  };

} // hekateros namespace

#endif // _HEK_TUNE_H
