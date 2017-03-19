////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   librnr_dynamixel
//
// File:      DynaPid.h
//
/*! \file
 *
 * $LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
 * $Rev: 3845 $
 *
 * \brief The Dynamixel PID Base Class.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
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

#ifndef _DYNA_PID_H
#define _DYNA_PID_H

#include <cstring>
#include <iostream>
#include <fstream>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"

using namespace std;


// -----------------------------------------------------------------------------
// DynaPid Class
// -----------------------------------------------------------------------------

/*!
 * \brief Proportional–Integral–Derivative Controller for Dynamixel servos.
 *
 * \par The Servo PID:
 * \termblock
 * \term setpoint (SP) \termdata = \termdata application specific \endterm
 * \term process variable (PV) \termdata =
 *  \termdata application specific \endterm
 * \term control variable (CV) \termdata = 
 *  \termdata application specific \endterm
 * \endtermblock
 *
 * \par The PID Control Loop:
 * \termblock
 * \term CV \termdata = \termdata
 *  K<sub>p</sub> * err<sub>j</sub> +
 *  K<sub>i</sub> * \h_sigma<sup>j</sup>err<sub>k</sub> * dt<sub>k</sub> + 
 *  K<sub>d</sub> * (err<sub>j</sub> - err<sub>j-1</sub>) / dt<sub>j</sub>
 * \endterm
 * \term output \termdata = \termdata converted control variable \endterm
 * \endtermblock
 * \n
 * <b>where:</b>
 * \termblock
 * \term j \termdata this time step \endterm
 * \term j-1 \termdata previous time step \endterm
 * \term dt<sub>k</sub> \termdata
 * delta time between time steps k-1 and k \endterm
 * \term err<sub>k</sub> \termdata error = SP - PV<sub>k</sub> \endterm
 * \term PV<sub>k</sub>
 *  \termdata process variable at time step k \endterm
 * \term K<sub>p</sub> \termdata proportional constant \endterm
 * \term K<sub>i</sub> \termdata integral constant \endterm
 * \term K<sub>d</sub> \termdata derivative constant \endterm
 * \endtermblock
 */
class DynaPid
{
public:
  static const double PidKpDft    = 0.5;  ///< default Kp constant
  static const double PidKiDft    = 1.0;  ///< default Ki constant
  static const double PidKdDft    = 0.1;  ///< default Kd constant
  static const double WiSumErrDft = 0.1;  ///< default sum error weight

  /*!
   * \brief Default constructor.
   *
   * \param fKp   PID proportional constant.
   * \param fKi   PID integral constant.
   * \param fKd   PID derivative constant.
   * \param fWi   PID integral sum of errors weight constant.
   */
  DynaPid(double fKp=PidKpDft,
          double fKi=PidKiDft,
          double fKd=PidKdDft,
          double fWi=WiSumErrDft)
  {
    m_fWi = fWi;
    SetConstants(fKp, fKi, fKd);
    InitControl();
  }

  /*!
   * \brief Copy constructor.
   */
  DynaPid(const DynaPid &src)
  {
    m_fWi = src.m_fWi;
    SetConstants(src.m_fKp, src.m_fKi, src.m_fKd);
    InitControl();
  }

  /*!
   * \brief Default destructor.
   */
  virtual ~DynaPid() { }

  /*!
   * \brief Set PID constants.
   *
   * \param fKp   PID proportional constant.
   * \param fKi   PID integral constant.
   * \param fKd   PID derivative constant.
   */
  virtual void SetConstants(double fKp, double fKi, double fKd)
  {
    m_fKp = fKp;
    m_fKi = fKi;
    m_fKd = fKd;
  }

  /*!
   * \brief Get PID proportional constant.
   * \return K<sub>p</sub>
   */
  double GetKp() const { return m_fKp; }

  /*!
   * \brief Get PID integral constant.
   * \return K<sub>i</sub>
   */
  double GetKi() const { return m_fKi; }

  /*!
   * \brief Get PID derivative constant.
   * \return K<sub>d</sub>
   */
  double GetKd() const { return m_fKd; }

  /*!
   * \brief Get PID setpoint.
   * \return SP.
   */
  double GetSP() const { return m_fSP; }

  /*!
   * \brief Get PID control variable.
   * \return CV.
   */
  double GetCV() const { return m_fCV; }

  /*!
   * \brief Get PID output.
   * \return Output.
   */
  double GetOutput() const { return m_fOutput; }
  
  /*!
   * \brief Get current error
   * \return Error.
   */
  double GetError() const { return m_fErr; }
  
  /*!
   * \brief Initialize the PID control variables.
   */
  virtual void InitControl()
  {
    m_fSP         = 0.0;
    m_fPV         = 0.0;
    m_fErrPrev    = 0.0;
    m_bHasPrev    = false;
    m_fErr        = 0.0;
    m_fErrSum     = 0.0;
    m_fCV         = 0.0;
    m_fOutput     = 0.0;
  }

  /*!
   * \brief Specify the setpoint.
   *
   * The PID can also be unwound. This PID technique is used when a new setpoint
   * causes the PID to behave badly, normally through integral windup. Unwinding
   * resets the accumulated sum of errors integral term to zero.
   *
   * \param fSP       New setpoint.
   * \param bUnwind   Do [not] reset controller integral sum of errors term
   */
  virtual void SpecifySetPoint(double fSP, bool bUnwind = false);

  /*!
   * \brief Apply PID control.
   *
   * \param fPV Process variable.
   * \param dt  Delta time step (seconds).
   *
   * \return Output.
   */
  virtual double Control(double fPV, double dt);

  /*!
   * \brief Assignment operator.
   *
   * \param rhs   Right hand side object.
   *
   * \return Returns *this.
   */
  DynaPid &operator=(const DynaPid &rhs)
  {
    m_fWi = rhs.m_fWi;
    SetConstants(rhs.m_fKp, rhs.m_fKi, rhs.m_fKd);
    InitControl();
    return *this;
  }

protected:
  double      m_fSP;        ///< setpoint
  double      m_fPV;        ///< process variable
  double      m_fKp;        ///< proportional constant
  double      m_fKi;        ///< integral constant
  double      m_fKd;        ///< derivative constant
  double      m_fWi;        ///< weighted sum of errors moving average constant
  bool        m_bHasPrev;   ///< has previous error
  double      m_fErrPrev;   ///< previous error
  double      m_fErr;       ///< error
  double      m_fErrSum;    ///< sum of errors
  double      m_fCV;        ///< control variable value
  double      m_fOutput;    ///< control variable output (could be same as CV)

  /*!
   * \brief Calculate error from setpoint and current process variable value.
   *
   * \param fPV     Process variable value.
   *
   * \return Error.
   */
  virtual double error(double fPV)
  {
    return m_fSP - fPV;
  }

  /*!
   * \brief Convert control variable to application-specific output value.
   *
   * \param fCV       Control variable value.
   *
   * \return Output.
   */
  virtual double toOutput(double fCV)
  {
    return fCV;
  }
};


#endif // _DYNA_PID_H
