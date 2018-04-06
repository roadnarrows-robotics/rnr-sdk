////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   libDynamixel
//
// File:      DynaPid.cxx
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
 *   \h_copy 2011-2018. RoadNarrows LLC.\n
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

#include <cstring>
#include <iostream>
#include <fstream>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaPid.h"

using namespace std;


// -----------------------------------------------------------------------------
// DynaPid Class
// -----------------------------------------------------------------------------

const double DynaPid::PidKpDft    = 0.5;  ///< default Kp constant
const double DynaPid::PidKiDft    = 1.0;  ///< default Ki constant
const double DynaPid::PidKdDft    = 0.1;  ///< default Kd constant
const double DynaPid::WiSumErrDft = 0.1;  ///< default sum error weight

void DynaPid::SpecifySetPoint(double fSP, bool bUnwind)
{
  m_fSP = fSP;

  m_bHasPrev = false;

  if( bUnwind )
  {
    m_fErrSum = 0.0;
  }
}

double DynaPid::Control(double fPV, double dt)
{
  double  fErrDt;       // change in error w.r.t delta time

  // new process variable value
  m_fPV = fPV;
  
  // error
  m_fErr = error(m_fPV);
    
  if( !m_bHasPrev )
  {
    m_fErrPrev = m_fErr;
    m_bHasPrev = true;
  }

  // weigthed sum of errors moving average
  m_fErrSum = (1.0 - m_fWi) * m_fErrSum + m_fWi * (m_fErr * dt);

  // delta error
  if( dt > 0.0 )
  {
    fErrDt = (m_fErr - m_fErrPrev)/dt;
  }
  else
  {
    fErrDt = 0.0;
  }
  
  m_fErrPrev = m_fErr;

  // new control variable value
  m_fCV = (m_fKp * m_fErr) + (m_fKi * m_fErrSum) + (m_fKd * fErrDt);
  
  m_fOutput = toOutput(m_fCV);

  return m_fOutput;
}
