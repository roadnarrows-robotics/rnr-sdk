////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   libDynamixel
//
// File:      DynaPidPos.h
//
/*! \file
 *
 * $LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
 * $Rev: 3845 $
 *
 * \brief The Dynamixel Position PID Class.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011-2015  RoadNarrows LLC.
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

#ifndef _DYNA_PID_POS_H
#define _DYNA_PID_POS_H

#include <math.h>

#include <cstring>
#include <iostream>
#include <fstream>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaPid.h"

using namespace std;


// -----------------------------------------------------------------------------
// DynaPidPos Class
// -----------------------------------------------------------------------------

/*!
 * \brief Position PID class.
 */
class DynaPidPos : public DynaPid
{
public:
  static const double TuneMaxSpeedDelta = 1000.0; ///< max speed change deltas
  static const double TuneAbsMinSpeed   =  0.0;   ///< absolute minimum speed

  /*!
   * \brief Default constructor.
   *
   * \param fKp   PID proportional constant.
   * \param fKi   PID integral constant.
   * \param fKd   PID derivative constant.
   * \param fWi   PID integral sum of errors weight constant.
   */
  DynaPidPos(double fKp=0.5,        // good default value
             double fKi=0.0,        // good default value
             double fKd=0.005,      // good default value
             double fWi=0.25) :
    DynaPid(fKp, fKi, fKd, fWi)
  {
    initParams();
  }

  /*!
   * \brief Copy constructor.
   */
  DynaPidPos(const DynaPid &src) : DynaPid(src)
  {
    initParams();
  }

  /*!
   * \brief Default destructor.
   */
  virtual ~DynaPidPos() { }

  /*!
   * \brief Assignment operator.
   */
  DynaPidPos &operator=(const DynaPidPos &rhs)
  {
    m_fWi = rhs.m_fWi;
    SetConstants(rhs.m_fKp, rhs.m_fKi, rhs.m_fKd);
    InitControl();
    initParams();
    return *this;
  }

  /*!
   * \brief Specify position setpoint.
   *
   * The PID can also be unwound. This PID technique is used when a new setpoint
   * causes the PID to behave badly, normally through integral windup. Unwinding
   * resets the accumulated sum of errors integral term to zero.
   *
   * \param nOdPosStart Starting position (odometer ticks).
   * \param nOdPosGoal  New goal position setpoint (odometer ticks).
   * \param nSpeedLim   Speed limit (raw).
   * \param nCurSpeed   Current servo speed (raw).
   * \param bIsReversed Odometer is [not] reversed from encoder.
   * \param bUnwind     Do [not] reset controller integral sum of errors term
   */
  virtual void SpecifySetPoint(int  nOdPosStart,
                               int  nOdPosGoal,
                               int  nSpeedLim,
                               int  nCurSpeed,
                               bool bIsReversed,
                               bool bUnwind = false);

protected:
  int       m_nOdPosStart;    ///< starting odometer position
  int       m_nOdPosGoal;     ///< goal ing odometer position (set point)
  int       m_nSpeedLim;      ///< speed limit (max neg/pos speed)
  int       m_nOdDir;         ///< odometer direction
  int       m_nOdPosCur;      ///< current odometer position
  double    m_fSpeedAbsMax;   ///< absolute speed limit >= 0
  double    m_fPrevSpeed;     ///< previous speed

  /*!
   * \brief Initialize parameters.
   */
  void initParams()
  {
    m_nOdPosStart   = 0;
    m_nOdPosGoal    = 0;
    m_nSpeedLim     = 0;
    m_nOdDir        = 1;
    m_nOdPosCur     = 0;
    m_fSpeedAbsMax  = 0.0;
    m_fPrevSpeed    = 0.0;
  }

  /*!
   * \brief Calculate error from goal postion setpoint and current position
   * \process variable value.
   *
   * \param fPVOdPosCur   Current odometer position process variable value.
   *
   * \return Error.
   */
  virtual double error(double fPVodPosCur);

  /*!
   * \brief Convert control variable to application-specific output value.
   *
   * \param fCVSpeed    Speed control variable value.
   *
   * \return Output.
   */
  virtual double toOutput(double fCVSpeed);
};

#endif // _DYNA_PID_POS_H
