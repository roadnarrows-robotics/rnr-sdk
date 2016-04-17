////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekPid.h
//
/*! \file
 *
 * $LastChangedDate: 2014-11-21 11:07:53 -0700 (Fri, 21 Nov 2014) $
 * $Rev: 3814 $
 *
 * \brief The Hekateros joint position and velocity PID class interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2014  RoadNarrows LLC.
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

#ifndef _HEK_PID_H
#define _HEK_PID_H

#include <math.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaPid.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekTune.h"

namespace hekateros
{
  // ---------------------------------------------------------------------------
  // HekPid Class
  // ---------------------------------------------------------------------------
  
  class HekPid : public DynaPid
  {
  public:
    //
    // Good default PID values for all Hekateros joints.
    //
    static const double WI_DFT = 0.25;  ///< integral sum of errors weight const
  
    /*!
     * \brief Default constructor.
     *
     * \param fKp         PID proportional constant.
     * \param fKi         PID integral constant.
     * \param fKd         PID derivative constant.
     * \param fWi         PID integral sum of errors weight constant.
     */
    HekPid(double fKp=HekTunePidKpDft,
           double fKi=HekTunePidKiDft,
           double fKd=HekTunePidKdDft,
           double fWi=WI_DFT) :
      DynaPid(fKp, fKi, fKd, fWi)
    {
      m_fMaxDeltaV    = HekTunePidDeltaVNoMax;
      m_fJointGoalPos = 0.0;
      m_fJointGoalVel = 0.0;
      m_fJointCurVel  = 0.0;
    }
  
    /*!
     * \brief Copy constructor.
     */
    HekPid(const HekPid &src) : DynaPid(src)
    {
      m_fMaxDeltaV    = src.m_fMaxDeltaV;
      m_fJointGoalPos = src.m_fJointGoalPos;
      m_fJointGoalVel = src.m_fJointGoalVel;
      m_fJointCurVel  = src.m_fJointCurVel;
    }
  
    /*!
     * \brief Destructor.
     */
    virtual ~HekPid() { }
  
    /*!
     * \brief Assignment operator.
     */
    HekPid &operator=(const HekPid &rhs)
    {
      m_fWi = rhs.m_fWi;
      SetConstants(rhs.m_fKp, rhs.m_fKi, rhs.m_fKd);
      InitControl();

      m_fMaxDeltaV    = rhs.m_fMaxDeltaV;
      m_fJointGoalPos = rhs.m_fJointGoalPos;
      m_fJointGoalVel = rhs.m_fJointGoalVel;
      m_fJointCurVel  = rhs.m_fJointCurVel;

      return *this;
    }
  
    /*!
     * \brief Get the position and velocity PID K parameters.
     *
     * \param [out] fKp   Proportional gain parameter.
     * \param [out] fKi   Integral gain parameter.
     * \param [out] fKd   Derivative gain parameter.
     */
    void getKParams(double &fKp, double &fKi, double &fKd) const
    {
      fKp = m_fKp;
      fKi = m_fKi;
      fKd = m_fKd;
    }
  
    /*!
     * \brief Set the position and velocity PID K parameters.
     *
     * \param [in] fKp   Proportional gain parameter.
     * \param [in] fKi   Integral gain parameter.
     * \param [in] fKd   Derivative gain parameter.
     *
     * \return
     */
    void setKParams(const double fKp, const double fKi, const double fKd)
    {
      m_fKp = fKp;
      m_fKi = fKi;
      m_fKd = fKd;
    }

    /*!
     * \brief Get the maximum delta V (ramp) parameter (radians/second).
     * 
     * \return Maximum delta velocity.
     */
    double getMaxDeltaVParam() const
    {
      return m_fMaxDeltaV;
    }
  
    /*!
     * \brief Set the maximum delta V (ramp) parameter (radians/second).
     * 
     * \param [in] fMaxDeltaV   Maximum delta velocity.
     */
    void setMaxDeltaVParam(const double fMaxDeltaV)
    {
      m_fMaxDeltaV = fMaxDeltaV;
    }

    /*!
     * \brief Specify the PID set point.
     *
     * The PID will control the primary postion Set Point SP through the
     * velocity Control Variable CV. The PID will strive to reach the goal
     * velocity (secondary SP) while achieving the primary SP.
     *
     * \param fJointGoalPos New goal position set point (radians).
     * \param fJointGoalVel New goal velocity set point (radians/second).
     */
    virtual void specifySetPoint(double fJointGoalPos, double fJointGoalVel);
  
    /*!
     * \brief Apply PID control.
     *
     * \param fJointCurPos  Current joint position (radians).
     *                      The primary Process Variable PV.
     * \param fJointCurVel  Current joint velocity (radians/second).
     *                      The secondary PV.
     * \param dt            Delta time step (seconds).
     *
     * \return Output.
     */
    virtual double control(double fJointCurPos,
                           double fJointCurVel,
                           double fDt)
    {
      m_fJointCurVel = fJointCurVel;
      return Control(fJointCurPos, fDt);
    }
  
  protected:
    double    m_fMaxDeltaV;     ///< maximum allowd delta v output
    double    m_fJointGoalPos;  ///< goal position (primary SP)
    double    m_fJointGoalVel;  ///< goal velocity (secondary SP)
    double    m_fJointCurVel;   ///< current velocity (secondary PV)
  
    /*!
     * \brief Convert Control Variable to application-specific output value.
     *
     * \param fJointTgtVel  Raw joint target velocity (radians/second).
     *                      The Control Variable CV from PID.
     *
     * \return Target velocity.
     */
    virtual double toOutput(double fJointTgtVel);
  };

} // namespace hekateros

#endif // _HEK_PID_H
