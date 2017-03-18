////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Library:   liblaelaps
//
// File:      laeTune.h
//
/*! \file
 *
 * $LastChangedDate: 2016-01-21 16:50:25 -0700 (Thu, 21 Jan 2016) $
 * $Rev: 4268 $
 *
 * \brief Laelaps tuning.
 *
 * All tuning defaults defined below can be overridden in the
 * /etc/laelaps_tune.conf XML file.
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

#ifndef _LAE_TUNE_H
#define _LAE_TUNE_H

#include <iostream>
#include <string>
#include <map>

#include "rnr/rnrconfig.h"

#include "Laelaps/laelaps.h"
#include "Laelaps/laeUtils.h"

namespace laelaps
{
  //----------------------------------------------------------------------------
  // Laelaps Tuning Defaults, Limits, and Fixed Values
  //----------------------------------------------------------------------------
 
  /*!
   * defgroup lae_tunes
   * \{
   */

  // . .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .
  // Thread Tuning

  /*!
   * \brief Default IMU thread cycle rate (Hertz).
   * 
   * All IMU tasks exectute per each cycle. 
   *
   * Range: \h_ge \ref LaeTuneThreadHzMin
   * Scope: global
   */
  extern const double LaeTuneThreadImuHzDft;

  /*!
   * \brief Default kinematics thread cycle rate (Hertz).
   * 
   * All dynamics and kinematics tasks exectute per each cycle. 
   *
   * Range: \h_ge \ref LaeTuneThreadHzMin
   * Scope: global
   */
  extern const double LaeTuneThreadKinHzDft;

  /*!
   * \brief Default range sensing thread cycle rate (Hertz).
   * 
   * All sensing tasks exectute per each cycle. 
   *
   * Range: \h_ge \ref LaeTuneThreadHzMin
   * Scope: global
   */
  extern const double LaeTuneThreadRangeHzDft;

  /*!
   * \brief Minimum thread cycle rate (Hertz).
   *
   * Gotta give me something boys!
   */
  extern const double LaeTuneThreadHzMin;


  // . .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .
  // Watchdog Tuning

  /*!
   * \brief Default Laelaps watchdog timeout
   *
   * The watchdog sub-processor must be petted at a faster rate than its 
   * timeout. Any received valid I2C or serial command by the watchdog
   * results in a reset of the watchdog timer. If the watchdog timer times out,
   * the no-service state is entered. Power to the motor controllers and motors
   * is disabled and the deck status LED flashes green.
   *
   * Units: seconds
   * Range: [\ref LaeTuneWdTimeoutMin, \ref LaeTuneWdTimeoutMax]
   * Scope: global
   */
  extern const double LaeTuneWdTimeoutDft;

  /*!
   * \brief Minimum watchdog timeout (msec).
   */
  extern const double LaeTuneWdTimeoutMin;

  /*!
   * \brief Maximum watchdog timeout (msec).
   */
  extern const double LaeTuneWdTimeoutMax;


  // . .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .
  // Derated Velocity Tuning

  /*!
   * \brief Default Laelaps robot velocity derate (% of goal velocities).
   *
   * When a new move command is received, all of the goal velocities are
   * multiplied by this derated value.
   *
   * The working value is converted to a normalized value internally.
   *
   * \verbatim
   * velocity_derate = tune_velocity_derate / 100.0
   * goal_vel = goal_vel * velocity_derate
   * \endverbatim
   *
   * Range: [\ref LaeTuneVelDerateMin, \ref LaeTuneVelDerateMax]
   * Scope: global
   */
  extern const double LaeTuneVelDerateDft;

  /*!
   * \brief Minimum robot velocity derate (% of goal velocities).
   */
  extern const double LaeTuneVelDerateMin;

  /*!
   * \brief Maximum robot velocity derate (% of goal velocities).
   */
  extern const double LaeTuneVelDerateMax;


  // . .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .
  // Navigation Trajectory Tuning

  /*!
   * \brief Default trajectory norm.
   *
   * Range: \ref LaeNormL1, \ref LaeNormL2, \ref LaeNormLinf
   * Scope: global
   */
  extern const LaeNorm LaeTuneTrajNormDft;

  /*!
   * \brief Default trajectory distance epsilon (meters).
   *
   * Trajectory waypoints distances within epsilon are considered reached.
   *
   * \verbatim
   * ||WP-P|| < epsilon <==> waypoint reached
   *   Where WP is the x,y goal vector of positions and P is the vector
   *   of current x,y position.
   * \endverbatim
   *
   * Range: \h_ge \ref LaeTuneTrajEpsilonMin
   */
  extern const double LaeTuneTrajEpsilonDft;

  /*!
   * \brief Minimum epsilon value (meters).
   */
  extern const double LaeTuneTrajEpsilonMin;

 
  // . .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .
  // Battery Tuning (mostly fixed for v2.0)

  /*!
   * \brief Fixed battery type.
   *
   * Fixed: lithium-ion polymer
   * Scope: battery
   */
  extern const char* const LaeTuneBattType;

  /*!
   * \brief Fixed battery chemistry.
   *
   * The chemistry islithium nickel manganese cobalt oxide (NMC).
   *
   * Fixed: NMC
   * Scope: battery
   */
  extern const char* const LaeTuneBattChem;

  /*!
   * \brief Fixed battery capacity (Amp-hours).
   *
   * Fixed: 10
   * Scope: battery
   */
  extern const double      LaeTuneBattCapAh;

  /*!
   * \brief Fixed battery cell count.
   *
   * Fixed: 3
   * Scope: battery
   */
  extern const int         LaeTuneBattCells;

  /*!
   * \brief Maximum maximum battery voltage.
   */
  extern const double      LaeTuneBattMaxVMax;

  /*!
   * brief Default maximum operating voltage.
   *
   * Range: [\ref LaeTuneBattNominalV, \ref LaeTuneBattMaxVMax]
   * Scope: battery
   */
  extern const double      LaeTuneBattMaxVDft;
 
  /*!
   * \brief Fixed nominal operating voltage.
   */
  extern const double      LaeTuneBattNominalV;

  /*!
   * brief Default minimum operating voltage.
   *
   * Range: [\ref LaeTuneBattMinVMin, \ref LaeTuneBattNominalV]
   * Scope: battery
   */
  extern const double      LaeTuneBattMinVDft;
 
  /*!
   * \brief Minimum minimum battery voltage.
   */
  extern const double      LaeTuneBattMinVMin;


  // . .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .
  // Velocity PID Tuning

  /*!
   * \brief Default motor velocity PID proportional constant.
   *
   * Range: \h_ge \ref LaeTuneVelPidKMin
   * Scope: powertrains
   */
  extern const double LaeTuneVelPidKpDft;

  /*!
   * \brief Default motor velocity PID integral constant.
   *
   * Range: \h_ge \ref LaeTunePidKMin
   * Scope: powertrains
   */
  extern const double LaeTuneVelPidKiDft;

  /*!
   * \brief Default motor velocity PID derivative constant.
   *
   * Range: \h_ge \ref LaeTunePidKMin
   * Scope: powertrains
   */
  extern const double LaeTuneVelPidKdDft;

  /*!
   * \brief Minimum PID K constant value.
   */
  extern const double LaeTuneVelPidKMin;

  // . .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .
  // Tires Tuning

  /*!
   * \brief Default tire radius (meters).
   *
   * Range: \h_ge \ref LaeTuneTireMin
   * Scope: powertrains
   */
  extern const double LaeTuneTireRadiusDft;

  /*!
   * \brief Default tire width (meters).
   *
   * Range: \h_ge \ref LaeTuneTireMin
   * Scope: powertrains
   */
  extern const double LaeTuneTireWidthDft;

  /*!
   * \brief Minimum tire dimension.
   */
  extern const double LaeTuneTireDimMin;

  // . .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .. .
  // VL6180 Range Sensor Tuning

  /*!
   * \brief Default Time-of-Flight part-to-part offset.
   *
   * Range: [\ref LaeTuneVL6180TofOffsetMin, \ref LaeTuneVL6180TofOffsetMax]
   * Scope: range_sensor
   */
  extern const int    LaeTuneVL6180TofOffsetDft;

  /*!
   * \brief Minimum Time-of-Flight part-to-part offset value.
   */
  extern const int    LaeTuneVL6180TofOffsetMin;

  /*!
   * \brief Maximum Time-of-Flight part-to-part offset value.
   */
  extern const int    LaeTuneVL6180TofOffsetMax;

  /*!
   * \brief Default Time-of-Flight cross-talk compensation.
   *
   * Range: [\ref LaeTuneVL6180TofXTalkMin, \ref LaeTuneVL6180TofXTalkMax]
   * Scope: range_sensor
   */
  extern const int    LaeTuneVL6180TofXTalkDft;

  /*!
   * \brief Minimum Time-of-Flight cross-talk compensation value.
   */
  extern const int    LaeTuneVL6180TofXTalkMin;

  /*!
   * \brief Maximum Time-of-Flight cross-talk compensation value.
   */
  extern const int    LaeTuneVL6180TofXTalkMax;

  /*!
   * \brief Default Ambient Light Sensor analog gain.
   *
   * Range: [\ref LaeTuneVL6180AlsGainMin, \ref LaeTuneVL6180AlsGainMax]
   * Scope: range_sensor
   */
  extern const double LaeTuneVL6180AlsGainDft;

  /*!
   * \brief Minimum Ambient Light Sensor analog gain.
   */
  extern const double LaeTuneVL6180AlsGainMin;

  /*!
   * \brief Maximum Ambient Light Sensor analog gain.
   */
  extern const double LaeTuneVL6180AlsGainMax;

  /*!
   * \brief Default Ambient Light Sensor integration period (msec).
   *
   * Range: [\ref LaeTuneVL6180AlsIntPeriodMin,
   *         \ref LaeTuneVL6180AlsIntPeriodMax]
   * Scope: range_sensor
   */
  extern const int    LaeTuneVL6180AlsIntPeriodDft;

  /*!
   * \brief Minimum Ambient Light Sensor integration period (msec).
   */
  extern const int    LaeTuneVL6180AlsIntPeriodMin;

  /*!
   * \brief Maximum Ambient Light Sensor integration period (msec).
   */
  extern const int    LaeTuneVL6180AlsIntPeriodMax;

  /*!
   * \}
   */


  //----------------------------------------------------------------------------
  // LaeTunesBattery Class
  //----------------------------------------------------------------------------
  
  /*!
   * \brief Laelaps battery tuning data class.
   */
  class LaeTunesBattery
  {
  public:
    std::string m_strType;    ///< battery type
    std::string m_strChem;    ///< battery chemistry
    double      m_fCapAh;     ///< battery capacity in Amp-hours
    int         m_nCells;     ///< number of battery cells
    double      m_fMaxV;      ///< maximum operating voltage
    double      m_fNominalV;  ///< nominal operation voltage
    double      m_fMinV;      ///< minimum operation voltage

    /*!
     * \brief Default constructor.
     */
    LaeTunesBattery();

    /*!
     * \brief Destructor.
     */
    ~LaeTunesBattery()
    {
    }

    /*!
     * \brief Assignment operator
     *
     * \param rhs   Right hand side object.
     *
     * \return *this
     */
    LaeTunesBattery &operator=(const LaeTunesBattery &rhs);

    /*!
     * \brief Print out tuning parameters to stdout.
     *
     * \param indent  Left indentation.
     */ 
    void print(int indent=0);

  }; // class LaeTunesBattery


  //----------------------------------------------------------------------------
  // LaeTunesPowertrain Class
  //----------------------------------------------------------------------------
  
  /*!
   * \brief Laelaps powertrain tuning data class.
   */
  class LaeTunesPowertrain
  {
  public:
    double  m_fVelPidKp;    ///< motor velocity PID proportional const
    double  m_fVelPidKi;    ///< motor velocity PID integral constant
    double  m_fVelPidKd;    ///< motor velocity PID derivative constant
    double  m_fTireRadius;  ///< tire radius (meters)
    double  m_fTireWidth;   ///< tire width (meters)

    /*!
     * \brief Default constructor.
     */
    LaeTunesPowertrain();

    /*!
     * \brief Destructor.
     */
    ~LaeTunesPowertrain()
    {
    }

    /*!
     * \brief Assignment operator
     *
     * \param rhs   Right hand side object.
     *
     * \return *this
     */
    LaeTunesPowertrain &operator=(const LaeTunesPowertrain &rhs);

    /*!
     * \brief Print out tuning parameters to stdout.
     *
     * \param strKey  Powertrain key.
     * \param indent  Left indentation.
     */ 
    void print(const std::string &strKey, int indent=0);

  }; // class LaeTunesPowertrain


  //----------------------------------------------------------------------------
  // LaeTunesPowertrain Class
  //----------------------------------------------------------------------------
  
  /*!
   * \brief Laelaps powertrain tuning data class.
   */
  class LaeTunesVL6180
  {
  public:
    int     m_nTofOffset;       ///< ToF part-to-part offset
    int     m_nTofCrossTalk;    ///< ToF cross-talk compensation
    double  m_fAlsGain;         ///< ALS analog gain
    int     m_nAlsIntPeriod;    ///< ALS integration period

    /*!
     * \brief Default constructor.
     */
    LaeTunesVL6180();

    /*!
     * \brief Destructor.
     */
    ~LaeTunesVL6180()
    {
    }

    /*!
     * \brief Assignment operator
     *
     * \param rhs   Right hand side object.
     *
     * \return *this
     */
    LaeTunesVL6180 &operator=(const LaeTunesVL6180 &rhs);

    /*!
     * \brief Print out tuning parameters to stdout.
     *
     * \param strKey  Sensor key.
     * \param indent  Left indentation.
     */ 
    void print(const std::string &strKey, int indent=0);

  }; // class LaeTunesVL6180


  //----------------------------------------------------------------------------
  // LaeTunes Class
  //----------------------------------------------------------------------------
  
  /*!
   * \brief Laelaps tuning data class.
   */
  class LaeTunes
  {
  public:
    /*! Map of powertrain pair tuning parameters. */
    typedef std::map<std::string, LaeTunesPowertrain>  LaeTunesMapPtp;

    /*! Map of VL6180 range sensor tuning parameters. */
    typedef std::map<std::string, LaeTunesVL6180>  LaeTunesMapVL6180;

    // global tuning
    double    m_fImuHz;           ///< kinematic thread rate (hertz)
    double    m_fKinematicsHz;    ///< kinematic thread rate (hertz)
    double    m_fRangeHz;         ///< kinematic thread rate (hertz)
    double    m_fWatchDogTimeout; ///< watchdog timeout (seconds)
    double    m_fVelDerate;       ///< velocity derate (fraction)
    LaeNorm   m_eTrajNorm;        ///< trajectory distanct norm
    double    m_fTrajEpsilon;     ///< trajectory epsilon distance (radians)

    // subsystem tuning
    LaeTunesBattery   m_battery;        ///< battery tuning
    LaeTunesMapPtp    m_mapPtp;         ///< powertrain pair tuning
    LaeTunesMapVL6180 m_mapVL6180;      ///< range sensors tuning
    /*!
     * \brief Default constructor.
     */
    LaeTunes();

    /*!
     * \brief Destructor.
     */
    ~LaeTunes()
    {
    }

    /*!
     * \brief Get IMU tasks thread cycle rate tune parameter (hertz).
     *
     * \return Hertz.
     */
    double getImuHz() const;

    /*!
     * \brief Get kinematics thread cycle rate tune parameter (hertz).
     *
     * \return Hertz.
     */
    double getKinematicsHz() const;

    /*!
     * \brief Get range sensing thread cycle rate tune parameter (hertz).
     *
     * \return Hertz.
     */
    double getRangeHz() const;

    /*!
     * \brief Get watchdog timeout (seconds).
     *
     * \return Seconds.
     */
    double getWatchDogTimeout() const;

    /*!
     * \brief Get derated velocity tune parameter (normalized).
     *
     * \return Derated value [0.0, 1.0].
     */
    double getVelocityDerate() const;

    /*!
     * \brief Get trajectory tune parameters.
     *
     * \param [out] eNorm     Distance norm.
     * \param [out] fEpsilon  Waypoint precision (radians)
     */
    void getTrajectoryParams(LaeNorm &eNorm, double &fEpsilon) const;

    /*!
     * \brief Get battery minimum and maximum allowed operating range voltages
     * tune parameters.
     *
     * \param [out] fBattMinV   Minimum battery voltage.
     * \param [out] fBattMaxV   Maximum battery voltage.
     */
    void getBattOpRangeParams(double &fBattMinV, double &fBattMaxV) const;

    /*!
     * \brief Get motor velocity PID K tune parameters.
     *
     * \param strName     Name of powertrain or powertrain pair.
     *                    One of: front left_front right_front
     *                            rear left_rear right_rear.
     * \param [out] fKp   Proportional constant.
     * \param [out] fKi   Integral constant.
     * \param [out] fKd   Derivative constant.
     */
    void getVelPidKParams(const std::string &strName,
                          double &fKp, double &fKi, double &fKd) const;

    /*!
     * \brief Get tire dimensions tune parameters.
     *
     * \param strName           Name of powertrain or powertrain pair.
     *                          One of: front left_front right_front
     *                                  rear left_rear right_rear.
     * \param [out] fTireRadius Tire radius (meters).
     * \param [out] fTireWidth  Tire width (meters).
     */
    void getTireDimParams(const std::string &strName,
                          double &fTireRadius, double &fTireWidth) const;

    /*!
     * \break Map powertrain (pair) key to powertrain pair key.
     *
     * \param strKey  Powertrain (pair) key.
     *
     * \return "front" or "rear"
     */
    std::string mapToPtp(const std::string &strKey) const;

    /*!
     * \brief Get VL6180 range sensor tune parameters.
     *
     * \param strName             Name of range sensor (key)
     * \param [out] nTofOffset    ToF part-to-part offset
     * \param [out] nTofCrossTalk ToF cross-talk compensation
     * \param [out] fAlsGain      ALS analog gain
     * \param [out] nAlsIntPeriod ALS integration period
     */
    void getVL6180Params(const std::string &strName,
                         int     &nTofOffset,
                         int     &nTofCrossTalk,
                         double  &fAlsGain,
                         int     &nAlsIntPeriod) const;


    /*!
     * \brief Print out tuning parameters to stdout.
     *
     * \param indent  Left indentation.
     */ 
    void print(int indent=0);

  }; // class LaeTunes

} // laelaps namespace

#endif // _LAE_TUNE_H
