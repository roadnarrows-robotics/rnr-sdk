////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekJoint.h
//
/*! \file
 *
 * $LastChangedDate: 2014-11-18 14:31:49 -0700 (Tue, 18 Nov 2014) $
 * $Rev: 3810 $
 *
 * \brief Hekateros joint classes interfaces.
 *
 * Data classes to hold description and state information.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2013-2014  RoadNarrows
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

#ifndef _HEK_JOINT_H
#define _HEK_JOINT_H

#include <string>
#include <vector>
#include <map>

#include "rnr/rnrconfig.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekOptical.h"

namespace hekateros
{
  //
  // Forward declarations
  //
  class HekRobot;

  // ---------------------------------------------------------------------------
  // Class HekRobotJoint
  // ---------------------------------------------------------------------------
 
  /*!
   * \brief Operational robotic joint description class.
   *
   * This class contains all of the data necessary to control the operation
   * of an \h_hek robotic joint.
   */
  class HekRobotJoint
  {
  public:
    /*!
     * \brief Default constructor.
     */
    HekRobotJoint();

    /*!
     * \brief Copy constructor.
     */
    HekRobotJoint(const HekRobotJoint &src);

    /*!
     * \brief Destructor.
     */
    ~HekRobotJoint();

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns copy of this.
     */
    HekRobotJoint operator=(const HekRobotJoint &rhs);

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
     * \brief Get master servo id.
     *
     * \return Returns servo id.
     */
    int getMasterServoId() const
    {
      return m_nMasterServoId;
    }

    /*!
     * \brief Set auto-stop at joint electronic limits state.
     *
     * \param bStopAtLimits   Do [not] auto-stop at limits.
     */
    void setStopAtLimitsState(bool bStopAtLimits)
    {
      m_bStopAtOptLimits = bStopAtLimits;
    }

    /*!
     * \brief Get auto-stop at joint electronic limits state.
     *
     * \return Returns true or false.
     */
    bool doStopAtLimits() const
    {
      return m_bStopAtOptLimits;
    }

  protected:
    // (derived) specification
    std::string m_strName;            ///< joint name
    int         m_nMasterServoId;     ///< master servo id
    int         m_nSlaveServoId;      ///< linked slave servo id (if any)
    bool        m_bIsServoContinuous; ///< servo should [not] be continuous mode
    int         m_nMasterServoDir;    ///< master servo normalized direction
    int         m_eJointType;         ///< joint type
    double      m_fGearRatio;         ///< joint gear ratio
    double      m_fTicksPerServoRad;  ///< encoder/odom. ticks per servo radian
    double      m_fTicksPerJointRad;  ///< encoder/odom. ticks per joint radian
    double      m_fMaxServoRadsPerSec;///< maximum servo radians per second
    double      m_fMaxJointRadsPerSec;///< maximum joint radians per second

    // discovered limits and positions
    double      m_fMinPhyLimitRads;   ///< joint min physical limit (radians)
    double      m_fMaxPhyLimitRads;   ///< joint max physical limit (radians)
    int         m_nMinPhyLimitOd;     ///< joint min phys limit (odometer ticks)
    int         m_nMaxPhyLimitOd;     ///< joint max phys limit (odometer ticks)
    double      m_fMinSoftLimitRads;  ///< joint min soft limit (radians)
    double      m_fMaxSoftLimitRads;  ///< joint max soft limit (radians)
    int         m_nMinSoftLimitOd;    ///< joint min soft limit (odometer ticks)
    int         m_nMaxSoftLimitOd;    ///< joint max soft limit (odometer ticks)
    double      m_fCalibPosRads;      ///< joint calibrated position (radians)
    double      m_fBalPosRads;        ///< joint balanced position (radians)
    double      m_fParkPosRads;       ///< joint parked position (radians)
    int         m_eLimitTypes;        ///< joint limit types
    double      m_fOverTorqueThDft;   ///< joint over torque th default (norm).
    byte_t      m_byOptLimitMask[HekOptLimitMaxPerJoint];
                                      ///< optical limit mask array

    // state
    HekOpState  m_eOpState;           ///< current operational state
    bool        m_bStopAtOptLimits;   ///< do [not] stop at optical limits

    // who isn't a friend of Mr. Loosey Joint?
    friend class HekRobot;
    friend class HekKinJoint;
    friend class HekKinJointWristRot;
    friend class HekMonitor;
    friend class HekCalibStretch;
    friend class HekXmlTune;
  };

  /*!
   * \brief Map of robot joints.
   *
   * \termblock
   * \term key: \termdata master servo id \endterm
   * \term mapped type: \termdata joint data \endterm
   * \endtermblock
   *
   * \note Joint order is critical. Ascending servo ids keeps map in order,
   * but if this cannot be guaranteed, then change strategy.
   */
  typedef std::map<int, HekRobotJoint> MapRobotJoints;


  // ---------------------------------------------------------------------------
  // Class HekJointOptical
  // ---------------------------------------------------------------------------
 
  /*!
   * \brief Operational robotic joint optical limits class.
   *
   * This class contains all of the data necessary to control the operation
   * of the robotic joint optical limits triggers.
   */
  class HekJointOptical
  {
  public:
    /*!
     * \brief Default constructor.
     */
    HekJointOptical()
    {
      m_uMask   = 0x00;
      m_uCurVal = 0x00;
      m_pJoint  = NULL;

      m_limit.clear();
    }

    /*!
     * \brief Copy constructor.
     */
    HekJointOptical(const HekJointOptical &src)
    {
      m_uMask   = src.m_uMask;
      m_uCurVal = src.m_uCurVal;
      m_pJoint  = src.m_pJoint;
      m_limit   = src.m_limit;
    }

    /*!
     * \brief Destructor.
     */
    ~HekJointOptical()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns copy of this.
     */
    HekJointOptical operator=(const HekJointOptical &rhs)
    {
      m_uMask   = rhs.m_uMask;
      m_uCurVal = rhs.m_uCurVal;
      m_pJoint  = rhs.m_pJoint;
      m_limit   = rhs.m_limit;

      return *this;
    }

  protected:
    byte_t              m_uMask;    ///< i/o expander bit mask
    byte_t              m_uCurVal;  ///< i/o expander current bit value
    HekRobotJoint      *m_pJoint;   ///< limit switch associated robotic joint
    HekOpticalLimit_T   m_limit;    ///< optical limit info

    friend class HekRobot;
    friend class HekMonitor;
    friend class HekCalib;
    friend class HekCalibStretch;
  };


  // ---------------------------------------------------------------------------
  // Class HekJointState
  // ---------------------------------------------------------------------------

  /*!
   * \brief  Joint state class.
   *
   * This class encapsulates the current state of one joint.
   */
  class HekJointState
  {
  public:
    /*!
     * \brief Default constructor.
     */
    HekJointState();

    /*!
     * \brief Copy constructor.
     */
    HekJointState(const HekJointState &src);

    /*!
     * \brief Destructor.
     */
    ~HekJointState()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns copy of this.
     */
    HekJointState operator=(const HekJointState &rhs);

    std::string m_strName;        ///< joint name
    HekOpState  m_eOpState;       ///< joint operational state
    int         m_nMasterServoId; ///< master servo id
    int         m_nSlaveServoId;  ///< linked slave servo id (if any)
    double      m_fPosition;      ///< current joint position (radians)
    double      m_fVelocity;      ///< current joint velocity (% of max)
    double      m_fEffort;        ///< current joint effort (servo load)
    int         m_nOdPos;         ///< current master servo odometer pos (ticks)
    int         m_nEncPos;        ///< current master servo encoder pos (ticks)
    int         m_nSpeed;         ///< current master servo raw speed (unitless)
    HekTriState m_eOptSwitch[HekOptLimitMaxPerJoint];
                                  ///< current state of optical switch(es)
  };


  // ---------------------------------------------------------------------------
  // Class HekJointStatePoint
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
  class HekJointStatePoint
  {
  public:
    /*!
     * \brief Default constructor.
     */
    HekJointStatePoint()
    {
    }

    /*!
     * \brief Copy constructor.
     */
    HekJointStatePoint(const HekJointStatePoint &src)
    {
      m_strKinName = src.m_strKinName;
      m_jointState = src.m_jointState;
    }

    /*!
     * \brief Destructor.
     */
    ~HekJointStatePoint()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns copy of this.
     */
    HekJointStatePoint operator=(const HekJointStatePoint &rhs)
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
    void append(const HekJointState &jointState)
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
    HekJointState &operator[](const size_t i)
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
    HekJointState &operator[](const std::string &strJointName);

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
    std::vector<HekJointState>  m_jointState;   ///< vector of joint states
  };

} // namespace hekateros


#endif // _HEK_JOINT_H
