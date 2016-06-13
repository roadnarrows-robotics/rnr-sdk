////////////////////////////////////////////////////////////////////////////////
//
// Package:   Kuon
//
// Library:   libkuon
//
// File:      hekJoint.h
//
/*! \file
 *
 * $LastChangedDate: 2014-04-07 08:16:52 -0600 (Mon, 07 Apr 2014) $
 * $Rev: 3631 $
 *
 * \brief Kuon Robot Joint class interfaces.
 *
 * The Joints are the four wheels driven by the geared motors.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2014-2016.  RoadNarrows
 * (http://www.RoadNarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
 * 
 * Permission is hereby granted, without written agreement and without
 * license or royalty fees, to use, copy, modify, and distribute this
 * software and its documentation for any purpose, provided that
 * (1) The above copyright notice and the following two paragraphs
 * appear in all copies of the source code and (2) redistributions
 * including binaries reproduces these notices in the supporting
 * documentation.   Substantial modifications to this software may be
 * copyrighted by their authors and need not follow the licensing terms
 * described here, provided that the new terms are clearly indicated in
 * all files where they apply.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
 * OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _KUON_JOINT_H
#define _KUON_JOINT_H

#include <string>
#include <vector>

#include "rnr/rnrconfig.h"

#include "Kuon/kuon.h"

namespace kuon
{
  //
  // Forward declarations
  //
  class KuonRobot;

  // ---------------------------------------------------------------------------
  // Class KuonRobotJoint
  // ---------------------------------------------------------------------------
 
  /*!
   * \brief Operational robotic joint class.
   *
   * This class contains all of the data necessary to control the operation
   * of an \h_hek robotic joint.
   */
  class KuonRobotJoint
  {
  public:
    /*!
     * \brief Default constructor.
     */
    KuonRobotJoint();

    /*!
     * \brief Copy constructor.
     */
    KuonRobotJoint(const KuonRobotJoint &src);

    /*!
     * \brief Destructor.
     */
    ~KuonRobotJoint();

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns copy of this.
     */
    KuonRobotJoint operator=(const KuonRobotJoint &rhs);

    /*!
     * \brief Get joint name.
     *
     * \return String.
     */
    std::string getJointName() const
    {
      return m_strName;
    }

    /*!
     * \brief Get motor id.
     *
     * \return Returns motor id.
     */
    int getMotorId() const
    {
      return m_nMotorId;
    }

  protected:
    // (derived) specification
    std::string m_strName;            ///< joint name
    int         m_nMotorId;           ///< motor id
    int         m_nMotorCtlrId;       ///< motor controller id
    int         m_nMotorIndex;        ///< motor controller unique motor index
    int         m_nMotorDir;          ///< motor normalized direction
    int         m_eJointType;         ///< joint type
    double      m_fGearRatio;         ///< joint gear ratio
    double      m_fTireRadius;        ///< tire radius
    double      m_fTicksPerMotorRad;  ///< encoder/odom. ticks per motor radian
    double      m_fTicksPerWheelRad;  ///< encoder/odom. ticks per wheel radian

    // discovered limits and positions

    friend class KuonRobot;
  };



  // ---------------------------------------------------------------------------
  // Class KuonJointState
  // ---------------------------------------------------------------------------

  /*!
   * \brief  Joint state class.
   *
   * This class encapsulates the current state of one joint.
   */
  class KuonJointState
  {
  public:
    /*!
     * \brief Default constructor.
     */
    KuonJointState();

    /*!
     * \brief Copy constructor.
     */
    KuonJointState(const KuonJointState &src);

    /*!
     * \brief Destructor.
     */
    ~KuonJointState()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns copy of this.
     */
    KuonJointState operator=(const KuonJointState &rhs);

    std::string m_strName;        ///< motor joint name
    int         m_nMotorId;       ///< motor id
    double      m_fPosition;      ///< current joint position (radians)
    double      m_fVelocity;      ///< current joint velocity (radians/second)
    double      m_fEffort;        ///< current joint torque (N-m)
    double      m_fOdometer;      ///< current wheel odometer reading (meters)
    int         m_nEncoder;       ///< current motor encoder position (ticks)
    double      m_fVelocityMps;   ///< current wheel velocity (meters/second)
    int         m_nSpeed;         ///< current raw speed
    double      m_fPe;            ///< current motor input electrical power (W)
    double      m_fPm;            ///< current motor output mechanical power (W)
    float       m_fBrake;         ///< current motor braking (normalized)
    float       m_fSlew;          ///< current motor power slewing (normalized)
  };


  // ---------------------------------------------------------------------------
  // Class KuonJointStatePoint
  // ---------------------------------------------------------------------------
  
  /*!
   * \brief  Joint state point class.
   *
   * This class encapsulates the current state of all joints in a kinematic
   * chain.
   *
   * A joint state point (J0, J1, ..., Jn-1) specifies the current dynamic
   * state of a set of joints (kinematic chain) Ji, i=0,n-1.
   */
  class KuonJointStatePoint
  {
  public:
    /*!
     * \brief Default constructor.
     */
    KuonJointStatePoint()
    {
    }

    /*!
     * \brief Copy constructor.
     */
    KuonJointStatePoint(const KuonJointStatePoint &src)
    {
      m_strKinName = src.m_strKinName;
      m_jointState = src.m_jointState;
    }

    /*!
     * \brief Destructor.
     */
    ~KuonJointStatePoint()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns copy of this.
     */
    KuonJointStatePoint operator=(const KuonJointStatePoint &rhs)
    {
      m_strKinName = rhs.m_strKinName;
      m_jointState = rhs.m_jointState;

      return *this;
    }

    /*!
     * \brief Get the kinematic chain name.
     *
     * \return String.
     */
    std::string getKinematicChainName()
    {
      return m_strKinName;
    }

    /*!
     * \brief Set the kinematic chain name.
     *
     * \param String.
     */
    void setKinematicChainName(std::string strKinName)
    {
      m_strKinName = strKinName;
    }

    /*!
     * \brief Get the number of joint states in joint state point.
     *
     * \return Number of points.
     */
    size_t getNumPoints()
    {
      return m_jointState.size();
    }

    /*!
     * \brief Test if joint is a member of the joint state.
     *
     * \param strJointName  Joint name.
     *
     * \return Returns true or false.
     */
    bool hasJoint(const std::string &strJointName);

    /*!
     * \brief Append joint state to end of joint state point.
     *
     * \param jointState    Joint state to append.
     */
    void append(const KuonJointState &jointState)
    {
      m_jointState.push_back(jointState);
    }

    /*!
     * \brief Subscript operator to get reference to joint point at the
     * given index.
     *
     * Big boy warranty. No out of bound checks are made.
     *
     * \param i   Index.
     *
     * \return Joint state.
     */
    KuonJointState &operator[](const size_t i)
    {
      return m_jointState[i];
    }

    /*!
     * \brief Key subscript operator to get reference to joint point at the
     * given index.
     *
     * Big boy warranty. No out of bound checks are made.
     *
     * \param strJointName  Joint name.
     *
     * \return Joint state.
     */
    KuonJointState &operator[](const std::string &strJointName);

    /*!
     * \brief Clear all state data.
     */
    void clear()
    {
      m_strKinName.clear();
      m_jointState.clear();
    }

  protected:
    std::string                 m_strKinName;   ///< name of kinematic chain 
    std::vector<KuonJointState> m_jointState;   ///< vector of joint states
  };

} // namespace kuon


#endif // _KUON_JOINT_H
