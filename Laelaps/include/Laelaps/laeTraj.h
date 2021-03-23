////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Library:   liblaelaps
//
// File:      laeTraj.h
//
/*! \file
 *
 * $LastChangedDate: 2016-02-18 13:42:54 -0700 (Thu, 18 Feb 2016) $
 * $Rev: 4323 $
 *
 * \brief Trajectory classes interfaces.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
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

#ifndef _LAE_TRAJ_H
#define _LAE_TRAJ_H

#include <sys/types.h>
#include <time.h>

#include <string>
#include <vector>
#include <map>

#include "rnr/rnrconfig.h"

#include "Laelaps/laelaps.h"
#include "Laelaps/laeUtils.h"

namespace laelaps
{
  // ---------------------------------------------------------------------------
  // Constants, Types, and Defines
  // ---------------------------------------------------------------------------
 
  /*!
   * \brief Start navigation immediately.
   */
  static const timespec_t StartImmediately = {0, 0};

  /*!
   * \brief Trajectory feedback identifiers (and indices).
   */
  static const int  TrajDesired = 0;  ///< desired trajectory
  static const int  TrajActual  = 1;  ///< actual trajectory
  static const int  TrajError   = 2;  ///< errors in trajectory
  static const int  TrajNumOf   = 3;  ///< number of trajectories ids


  // ---------------------------------------------------------------------------
  // Class LaePoint
  // ---------------------------------------------------------------------------

  /*!
   * \brief Laelaps 2D point class.
   *
   * Point = (x, y)
   */
  class LaePoint
  {
  public:
    double  m_x;      ///< x (meters)
    double  m_y;      ///< y (meters)
    
    /*!
     * \brief Default constructor.
     */
    LaePoint()
    {
      clear();
    }

    /*!
     * \brief Initialization constructor.
     *
     * \param x       x (meters).
     * \param y       y (meters).
     */
    LaePoint(double x, double y, double theta=0.0)
    {
      m_x     = x;
      m_y     = y;
    }

    /*!
     * \brief Copy constructor.
     *
     * \param src   Source object.
     */
    LaePoint(const LaePoint &src)
    {
      m_x     = src.m_x;
      m_y     = src.m_y;
    }

    /*!
     * \brief Destructor.
     */
    virtual ~LaePoint()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns reference to this.
     */
    LaePoint &operator=(const LaePoint &rhs)
    {
      m_x     = rhs.m_x;
      m_y     = rhs.m_y;

      return *this;
    }

    void clear()
    {
      m_x     = 0.0;
      m_y     = 0.0;
    }

  }; // class LaePoint


  // ---------------------------------------------------------------------------
  // Class LaePose
  // ---------------------------------------------------------------------------

  /*!
   * \brief Laelaps 2D pose class.
   *
   * Pose = (x, y, theta)
   */
  class LaePose
  {
  public:
    double  m_x;      ///< robot absolute x position (meters)
    double  m_y;      ///< robot absolute y position (meters)
    double  m_theta;  ///< robot orientation (radians)
    
    /*!
     * \brief Default constructor.
     */
    LaePose()
    {
      clear();
    }

    /*!
     * \brief Initialization constructor.
     *
     * \param x       Robot absolute x position (meters).
     * \param y       Robot absolute y position (meters).
     * \param theta   Robot orientation (radians).
     */
    LaePose(double x, double y, double theta=0.0)
    {
      m_x     = x;
      m_y     = y;
      m_theta = theta;
    }

    /*!
     * \brief Copy constructor.
     *
     * \param src   Source object.
     */
    LaePose(const LaePose &src)
    {
      m_x     = src.m_x;
      m_y     = src.m_y;
      m_theta = src.m_theta;
    }

    /*!
     * \brief Destructor.
     */
    virtual ~LaePose()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns reference to this.
     */
    LaePose &operator=(const LaePose &rhs)
    {
      m_x     = rhs.m_x;
      m_y     = rhs.m_y;
      m_theta = rhs.m_theta;

      return *this;
    }

    void clear()
    {
      m_x     = 0.0;
      m_y     = 0.0;
      m_theta = 0.0;
    }

  }; // class LaePose


  // ---------------------------------------------------------------------------
  // Velocity trajectory types.
  // ---------------------------------------------------------------------------
  
  /*!
   * \brief Velocity trajectory type.
   *
   * The robot is controlled by seting the goal velocities of a [sub]set of 
   * powertrain motors.
   *
   * \li key    Name of powertrain. One of: left_front, right_front,
   *                    left_rear, right_rear.
   * \li value  Wheel angular velocity (radians/second).
   */
  typedef std::map<std::string, double>  LaeMapVelocity;

  /*!
   * \brief Duty cycle trajectory type.
   *
   * The robot is controlled by seting the goal motor duty cycles.
   *
   * \li key    Name of powertrain. One of: left_front, right_front,
   *                    left_rear, right_rear.
   * \li value  Normalized duty cycle [-1.0, 1.0].
   */
  typedef std::map<std::string, double>  LaeMapDutyCycle;


  // ---------------------------------------------------------------------------
  // Class LaeSimpleWaypoint
  // ---------------------------------------------------------------------------
  
  /*!
   * \brief Robot simple path waypoint.
   *
   * A simple waypoint specifies the goal x,y distance from the robot's current
   * position, along with a target velocity with acceleration at the
   * destination. The waypoint is used in path navigation.
   */
  class LaeSimpleWaypoint
  {
  public:
    /*!
     * \brief Default constructor.
     */
    LaeSimpleWaypoint()
    {
      m_fVelocity     = 0.0;
      m_fAcceleration = 0.0;
    }

    /*!
     * \brief Initialization constructor.
     *
     * \param ptDist        Robot goal distance (meters, meters).
     * \param fVelocity     Robot target velocity at waypoint (meters/second).
     * \param fAcceleration Robot target acceleration (meters/second^2).
     */
    LaeSimpleWaypoint(const LaePoint &ptDist,
                      const double    fVelocity,
                      const double    fAcceleration)
    {
      m_ptDist        = ptDist;
      m_fVelocity     = fVelocity;
      m_fAcceleration = fAcceleration;
    }

    /*!
     * \brief Copy constructor.
     *
     * \param src   Source object.
     */
    LaeSimpleWaypoint(const LaeSimpleWaypoint &src)
    {
      m_ptDist        = src.m_ptDist;
      m_fVelocity     = src.m_fVelocity;
      m_fAcceleration = src.m_fAcceleration;
    }

    /*!
     * \brief Destructor.
     */
    ~LaeSimpleWaypoint()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns reference to this.
     */
    LaeSimpleWaypoint &operator=(const LaeSimpleWaypoint &rhs)
    {
      m_ptDist        = rhs.m_ptDist;
      m_fVelocity     = rhs.m_fVelocity;
      m_fAcceleration = rhs.m_fAcceleration;

      return *this;
    }

    /*!
     * \brief Get the waypoint data.
     *
     * \param [out] ptDist        Robot goal distance (meters, meters).
     * \param [out] fVelocity     Robot target velocity (meters/second).
     * \param [out] fAcceleration Robot target acceleration (meters/second^2).
     */
    void get(LaePoint &ptDist,
             double   &fVelocity,
             double   &fAcceleration)
    {
      ptDist        = m_ptDist;
      fVelocity     = m_fVelocity;
      fAcceleration = m_fAcceleration;
    }

  protected:
    LaePoint    m_ptDist;         ///< robot goal x,y distance (meters, meters)
    double      m_fVelocity;      ///< robot target velocity (meters/second)
    double      m_fAcceleration;  ///< robot target acceleration (meters/s^2)
  }; // class LaeSimpleWaypoint


  // ---------------------------------------------------------------------------
  // Class LaeSimplePath
  // ---------------------------------------------------------------------------
  
  /*!
   * \brief  Robot simple path class.
   *
   * A simple path defines a set of waypoints from the robot's current position
   * to the endpoint. Each waypoint defines a x,y distance, plus target velocity
   * and acceleration.
   *
   * A sequence of waypoints points Wp_i, i=0,n define a path.
   */
  class LaeSimplePath
  {
  public:
    /*!
     * \brief Default constructor.
     */
    LaeSimplePath()
    {
      m_timeStart = StartImmediately;
    }

    /*!
     * \brief Copy constructor.
     */
    LaeSimplePath(const LaeSimplePath &src)
    {
      m_timeStart = src.m_timeStart;
      m_path      = src.m_path;
    }

    /*!
     * \brief Destructor.
     */
    ~LaeSimplePath()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns reference to this.
     */
    LaeSimplePath &operator=(const LaeSimplePath &rhs)
    {
      m_timeStart = rhs.m_timeStart;
      m_path      = rhs.m_path;

      return *this;
    }

    /*!
     * \brief Get navigation start time.
     *
     * \return Start time. Time is in seconds,nanoseconds from last Epoch.
     */
    timespec_t getStartTime()
    {
      return m_timeStart;
    }

    /*!
     * \brief Set navigation start time.
     *
     * \param timeStart Time to start path navigation.
     *                  Time is in seconds,nanoseconds from last Epoch.
     */
    void setStartTime(timespec_t &timeStart)
    {
      m_timeStart = timeStart;
    }

    /*!
     * \brief Get the number of wheel points in trajectory.
     *
     * \return Number of points.
     */
    size_t getNumPoints()
    {
      return m_path.size();
    }

    /*!
     * \brief Append wheel point to end of trajectory.
     *
     * \param ptDist        Robot goal distance (meters, meters).
     * \param fVelocity     Robot goal velocity (meters/second).
     * \param fAcceleration Robot goal acceleration (meters/second^2).
     */
    void append(LaePoint &ptDist,
                double    fVelocity,
                double    fAcceleration)
    {
      LaeSimpleWaypoint pt(ptDist, fVelocity, fAcceleration);
      m_path.push_back(pt);
    }

    /*!
     * \brief Subscript operator to get reference to wheel point at the
     * given index.
     *
     * Big boy warranty. No out of bound checks are made.
     *
     * \param i   Index.
     *
     * \return Wheel point.
     */
    LaeSimpleWaypoint &operator[](const size_t i)
    {
      return m_path[i];
    }

    /*!
     * \brief Clear data.
     */
    void clear()
    {
      m_path.clear();
      m_timeStart = StartImmediately;
    }

  protected:
    timespec_t                      m_timeStart;  ///< start time
    std::vector<LaeSimpleWaypoint>  m_path;       ///< path
  }; // class LaeSimplePath


  // ---------------------------------------------------------------------------
  // Class LaeSimplePathFeedback
  // ---------------------------------------------------------------------------
  
  /*!
   * \brief  Simple path feedback class.
   *
   * The feedback provides informaation about the last issued desired path
   * and the current trajectory.
   */
  class LaeSimplePathFeedback
  {
  public:

    /*!
     * \brief Default constructor.
     */
    LaeSimplePathFeedback()
    {
    }

    /*!
     * \brief Copy constructor.
     */
    LaeSimplePathFeedback(const LaeSimplePathFeedback &src);

    /*!
     * \brief Destructor.
     */
    ~LaeSimplePathFeedback()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns reference to this.
     */
    LaeSimplePathFeedback &operator=(const LaeSimplePathFeedback &rhs);

    /*!
     * \brief Get the number of wheel points in trajectory.
     *
     * \return Number of points.
     */
    size_t getNumPoints()
    {
      return m_path[TrajDesired].getNumPoints();
    }

    /*!
     * \brief Subscript operator to get reference to given trajectory.
     *
     * \param i   Trajectory point Index.
     *
     * \return Trajectory.
     */
    LaeSimplePath &operator[](const size_t i);

    /*!
     * \brief Clear data.
     */
    void clear();

  protected:
    LaeSimplePath   m_path[TrajNumOf];
  };


  // ---------------------------------------------------------------------------
  // Class LaeWaypoint
  // ---------------------------------------------------------------------------
  
  /*!
   * \brief Robot fully-defined waypoint.
   *
   * A waypoint specifies the goal pose (absolute x,y,theta position), plus
   * the target velocity with the given acceleration at the destination.
   * The waypoint is used in path navigation.
   *
   * Each waypoint has an associtated name identifier and start time.
   */
  class LaeWaypoint
  {
  public:
    /*!
     * \brief Default constructor.
     */
    LaeWaypoint();

    /*!
     * \brief Initialization constructor.
     *
     * \param Pose          Robot 2D pose (x,y,theta).
     * \param fVelocity     Robot target velocity at waypoint (meters/second).
     * \param fAcceleration Robot target acceleration (meters/second^2).
     * \param strName       Name of waypoint.
     * \param timeStart     Start time to navigate to this waypoint.
     */
    LaeWaypoint(const LaePose     &pose,
                const double      fVelocity,
                const double      fAcceleration,
                const std::string &strName="",
                const timespec_t  &timeStart=StartImmediately);

    /*!
     * \brief Copy constructor.
     *
     * \param src   Source object.
     */
    LaeWaypoint(const LaeWaypoint &src);

    /*!
     * \brief Destructor.
     */
    ~LaeWaypoint()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns reference to this.
     */
    LaeWaypoint &operator=(const LaeWaypoint &rhs);

    /*!
     * \brief Get waypoint data.
     *
     * \param [out] strName       Waypoint name.
     * \param [out] timeStart     Waypoint navigation start time.
     * \param [out] pose          Robot 2D pose (x,y,theta).
     * \param [out] fVelocity     Target waypoint velocity (meters/second).
     * \param [out] fAcceleration Target acceleration (meters/second^2).
     */
    void get(std::string &strName,
             timespec_t  &timeStart,
             LaePose     &pose,
             double      &fVelocity,
             double      &fAcceleration);

    /*!
     * \brief Get waypoint navigation start time.
     *
     * \return Start time. Time is in seconds,nanoseconds from last Epoch.
     */
    timespec_t getStartTime()
    {
      return m_timeStart;
    }

    /*!
     * \brief Set waypoint navigation start time.
     *
     * \param timeStart Time to start execution.
     *                  Time is in seconds,nanoseconds from last Epoch.
     */
    void setStartTime(timespec_t timeStart)
    {
      m_timeStart = timeStart;
    }

    /*!
     * \brief Clear (reset) waypoint data.
     */
    void clear();

  protected:
    std::string m_strName;        ///< waypoint name
    timespec_t  m_timeStart;      ///< time to start navigation
    LaePose     m_pose;           ///< robot pose at waypoint
    double      m_fVelocity;      ///< target velocity at waypoint (meters/s)
    double      m_fAcceleration;  ///< target acceleration (meters/s^2)
  };


  // ---------------------------------------------------------------------------
  // Class LaePath
  // ---------------------------------------------------------------------------
  
  /*!
   * \brief  Robot path class.
   *
   * A waypoint specifies the goal pose (absolute x,y,theta position), plus
   * the target velocity with the given acceleration at the destination.
   * The waypoint is used in path navigation.
   *
   * A sequence of waypoints points Wp_i, i=0,n define a path.
   */
  class LaePath
  {
  public:
    /*!
     * \brief Default constructor.
     */
    LaePath()
    {
      m_timeStart = StartImmediately;
    }

    /*!
     * \brief Copy constructor.
     */
    LaePath(const LaePath &src)
    {
      m_timeStart = src.m_timeStart;
      m_path      = src.m_path;
    }

    /*!
     * \brief Destructor.
     */
    ~LaePath()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns reference to this.
     */
    LaePath &operator=(const LaePath &rhs)
    {
      m_timeStart = rhs.m_timeStart;
      m_path      = rhs.m_path;

      return *this;
    }

    /*!
     * \brief Get navigation start time.
     *
     * \return Start time. Time is in seconds,nanoseconds from last Epoch.
     */
    timespec_t getStartTime()
    {
      return m_timeStart;
    }

    /*!
     * \brief Set navigation start time.
     *
     * \param timeStart   Start time.
     *                    Time is in seconds,nanoseconds from last Epoch.
     */
    void setStartTime(timespec_t timeStart)
    {
      m_timeStart = timeStart;
    }

    /*!
     * \brief Get the number of wheel points in trajectory.
     *
     * \return Number of points.
     */
    size_t getNumPoints()
    {
      return m_path.size();
    }

    /*!
     * \brief Append waypoint to end of path.
     *
     * \param Pose          Robot 2D pose (x,y,theta).
     * \param fVelocity     Robot target velocity at waypoint (meters/second).
     * \param fAcceleration Robot target acceleration (meters/second^2).
     * \param strName       Name of waypoint
     * \param timeStart    Start time to navigate to this waypoint.
     */
    void append(const LaePose     &pose,
                const double       fVelocity,
                const double       fAcceleration,
                const std::string &strName="",
                const timespec_t  &timeStart=StartImmediately);

    /*!
     * \brief Subscript operator to get reference to wheel point at the
     * given index.
     *
     * Big boy warranty. No out of bound checks are made.
     *
     * \param i   Index.
     *
     * \return Wheel point.
     */
    LaeWaypoint &operator[](const size_t i)
    {
      return m_path[i];
    }

    /*!
     * \brief Clear data.
     */
    void clear()
    {
      m_path.clear();
      m_timeStart = StartImmediately;
    }

  protected:
    timespec_t                m_timeStart;  ///< start time
    std::vector<LaeWaypoint>  m_path;       ///< path
  }; // class LaePath


  // ---------------------------------------------------------------------------
  // Class LaePathFeedback
  // ---------------------------------------------------------------------------
  
  /*!
   * \brief  Simple path feedback class.
   *
   * The feedback provides informaation about the last issued desired path
   * and the current trajectory.
   */
  class LaePathFeedback
  {
  public:

    /*!
     * \brief Default constructor.
     */
    LaePathFeedback()
    {
    }

    /*!
     * \brief Copy constructor.
     */
    LaePathFeedback(const LaePathFeedback &src);

    /*!
     * \brief Destructor.
     */
    ~LaePathFeedback()
    {
    }

    /*!
     * \brief Assignment operator.
     *
     * \param rhs   Right hand side object.
     *
     * \return Returns reference to this.
     */
    LaePathFeedback &operator=(const LaePathFeedback &rhs);

    /*!
     * \brief Get the number of wheel points in trajectory.
     *
     * \return Number of points.
     */
    size_t getNumPoints()
    {
      return m_path[TrajDesired].getNumPoints();
    }

    /*!
     * \brief Subscript operator to get reference to given trajectory.
     *
     * \param i   Trajectory point Index.
     *
     * \return Trajectory.
     */
    LaePath &operator[](const size_t i);

    /*!
     * \brief Clear data.
     */
    void clear();

  protected:
    LaePath   m_path[TrajNumOf];
  };  // class LaePathFeedback

} // namespace laelaps


#endif // _LAE_TRAJ_H
