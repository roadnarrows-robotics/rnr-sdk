////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   librnr_dynamixel
//
// File:      DynaPidSpeed.h
//
/*! \file
 *
 * $LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
 * $Rev: 3845 $
 *
 * \brief The Dynamixel Speed PID Class.
 *
 * The speed PID controls a servo to a speed setpoint.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011-2016  RoadNarrows LLC.
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

#ifndef _DYNA_PID_SPEED_H
#define _DYNA_PID_SPEED_H

#include <cstring>
#include <iostream>
#include <fstream>
#include <vector>
#include <map>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaPid.h"

using namespace std;


// -----------------------------------------------------------------------------
// DynaPidSpeed Class
// -----------------------------------------------------------------------------

/*!
 * \brief Speed PID class.
 */
class DynaPidSpeed : public DynaPid
{
public:
  static const double SpeedPidKpDft     = 0.5;  ///< default Kp constant
  static const double SpeedPidKiDft     = 1.0;  ///< default Ki constant
  static const double SpeedPidKdDft     = 0.1;  ///< default Kd constant

  /*!
   * \brief Default constructor.
   *
   * \param fKp   Speed PID proportional constant.
   * \param fKi   Speed PID integral constant.
   * \param fKd   Speed PID derivative constant.
   * \param fWi   Speed PID integral sum of errors weight constant.
   */
  DynaPidSpeed(double fKp=SpeedPidKpDft,
               double fKi=SpeedPidKiDft,
               double fKd=SpeedPidKdDft,
               double fWi=DynaPid::WiSumErrDft) :
      DynaPid(fKp, fKi, fKd, fWi)
  {
    SetServoParams(DYNA_MODE_SERVO, DYNA_SPEED_MIN_CTL, DYNA_SPEED_MAX_CTL,
                    DYNA_POS_MODULO);
    SetStartingPos(0);
  }

  /*!
   * \brief Copy constructor.
   */
  DynaPidSpeed(const DynaPidSpeed &src) :
      DynaPid(src.m_fKp, src.m_fKi, src.m_fKd, src.m_fWi)
  {
    SetServoParams(src.m_nMode, src.m_nSpeedMin, src.m_nSpeedMax,
        src.m_nOdModulo);
    SetStartingPos(src.m_nPos[0]);
  }

  /*!
   * \brief Default destructor.
   */
  virtual ~DynaPidSpeed() { }

  /*!
   * \brief Set servo paramaters required for speed PID.
   *
   * \param nMode       Servo mode. (See \ref dyna_servo_mode).
   * \param nSpeedMin   Servo minimum raw speed.
   * \param nSpeedMax   Servo maximum raw speed.
   * \param nOdModulo   Odometer roll-over modulo.
   */
  virtual void SetServoParams(int nMode,
                              int nSpeedMin,
                              int nSpeedMax,
                              int nOdModulo)
  {
    m_nMode     = nMode;
    m_nSpeedMin = nSpeedMin;
    m_nSpeedMax = nSpeedMax;
    m_nOdModulo = nOdModulo;
  }

  /*!
   * \brief Set speed PID starting positions.
   *
   * \param uStartPos   Starting servo position.
   */ 
  virtual void SetStartingPos(uint_t uStartPos)
  {
    m_nPos[0] = (int)uStartPos;
    m_nPos[1] = (int)uStartPos;
  }

  /*! \brief Convert the current and previous positions measured over delta time
   * to dp/dt.
   *
   * Since there are two solutions to delta position for servos in continuous
   * mode, the smallest delta is taken. This assumes that the position sampling
   * rate is fast enough not to cause ambiguity.
   *
   * \param nCurPos   Current servo position.
   * \param nPrevPos  Current servo position.
   * \param dt        Delta time between previous and current positions
   *                  (seconds).
   *
   * \return dp/dt.
   */
  virtual double CalcDpDt(uint_t uCurPos, uint_t uPrevPos, double dt)
  {
    return CalcDpDt((int)uCurPos, (int)uPrevPos, dt);
  }

  /*!
   * \brief Convert the current and previous positions measured over delta time
   * to dp/dt.
   *
   * Since there are two solutions to delta position for servos in continuous
   * mode, the smallest delta is taken. This assumes that the position sampling
   * rate is fast enough not to cause ambiguity.
   *
   * \param nCurPos   Current servo position.
   * \param nPrevPos  Current servo position.
   * \param dt        Delta time between previous and current positions
   *                  (seconds).
   *
   * \return dp/dt.
   */
  virtual double CalcDpDt(int nCurPos, int nPrevPos, double dt);

  /*!
   * \brief Specify the setpoint.
   *
   * The PID can also be reset. This PID technique is used when a new setpoint
   * causes the PID to behave badly, normally through integral windup. The
   * reset:
   *  o Zeros accumulated sum of errors.
   *
   * \param fDpDt     Speed dp/dt where dp = delta position, dt = delta time.
   * \param bUnwind   Do [not] reset controller integral sum of errors term
   */
  virtual void SpecifySetPoint(double  fDpDt, bool bUnwind = false);

  /*!
   * \brief Apply PID control.
   *
   * \param uCurPos   Current servo position.
   * \param dt        Delta time (seconds).
   *
   * \return Output servo raw speed.
   */
   virtual double Control(uint_t uCurPos, double dt)
   {
     return DynaPid::Control(toPV(uCurPos, dt), dt);
   }

  /*!
   * \brief Assignment operator.
   *
   * \param rhs   Right hand side object.
   *
   * \return Returns *this.
   */
  DynaPidSpeed &operator=(const DynaPidSpeed &rhs)
  {
    m_fWi = rhs.m_fWi;
    SetConstants(rhs.m_fKp, rhs.m_fKi, rhs.m_fKd);
    SetServoParams(rhs.m_nMode, rhs.m_nSpeedMin, rhs.m_nSpeedMax,
                      rhs.m_nOdModulo);
    SetStartingPos(rhs.m_nPos[0]);
    InitControl();
    return *this;
  }

protected:
  int   m_nPos[2];    ///< servo current (0) and previous (1) positions
  int   m_nMode;      ///< servo mode
  int   m_nSpeedMin;  ///< servo minimum raw value
  int   m_nSpeedMax;  ///< servo maximum raw value
  int   m_nOdModulo;  ///< servo odometer rollover modulo

  /*! \brief Save current position and convert to dp/dt process variable.
   *
   * \param uCurPos Current servo position.
   * \param dt      Delta time between previous and current positions (seconds).
   *
   * \return Process variable (PV).
   */
  virtual double toPV(uint_t uCurPos, double dt)
  {
    // age
    m_nPos[1] = m_nPos[0];
    m_nPos[0] = (int)uCurPos;

    return CalcDpDt(m_nPos[0], m_nPos[1], dt);
  }

  /*!
   * \brief Convert control variable to raw servo speed.
   *
   * \return Output.
   */
  virtual double toOutput();
};


#endif // _DYNA_PID_SPEED_H
