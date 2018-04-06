////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   libDynamixel
//
// File:      DynaPidSpeed.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
 * $Rev: 3845 $
 *
 * \brief The Dynamixel Speed PID Class.
 *
 * The speed PID controls to a servo's speed setpoint.
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
#include <vector>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaPid.h"
#include "Dynamixel/DynaPidSpeed.h"
#include "Dynamixel/DynaOlio.h"


using namespace std;


// -----------------------------------------------------------------------------
// DynaPidSpeed Class
// -----------------------------------------------------------------------------

#if 0 // RDK move to DynaServo
  {
    m_pServo = pServo;

    m_nSpeedMax = (int)m_pServo->GetSpecification().m_uRawSpeedMax;

    if( m_pServo->GetServoMode() == DYNA_MODE_SERVO )
    {
      // cannot set to 0 which is, unfortunately, the default full speed value
      m_nSpeedMin = DYNA_SPEED_MIN_CTL;
    }
    else
    {
      m_nSpeedMin = -m_nSpeedMax;
    }

    m_nOdModulo = m_pServo->GetOdometerModulo();
  }
#endif // 0
 
const double DynaPidSpeed::SpeedPidKpDft  = 0.5;  ///< default Kp constant
const double DynaPidSpeed::SpeedPidKiDft  = 1.0;  ///< default Ki constant
const double DynaPidSpeed::SpeedPidKdDft  = 0.1;  ///< default Kd constant

void DynaPidSpeed::SpecifySetPoint(double fDpDt, bool bUnwind)
{
  DynaPid::SpecifySetPoint(fDpDt, bUnwind);
}

double DynaPidSpeed::CalcDpDt(int nCurPos, int nPrevPos, double dt)
{
  int     nDp;      // delta position
  double  fDpDt;    // dp/dt where dp = delta raw position, dt = delta time

  nDp = nCurPos - nPrevPos;

  //
  // The two positions straddle the roll-over point.
  //
  if( (m_nMode == DYNA_MODE_CONTINUOUS) && (iabs(nDp) > m_nOdModulo/2) )
  {
    if( nDp < 0 )
    {
      nDp = m_nOdModulo + nDp;
    }
    else
    {
      nDp = nDp - m_nOdModulo;
    }
  }

  if( dt > 0.0 )
  {
    fDpDt = ((double)(nDp))/dt;
  }
  else
  {
    fDpDt = 0.0;
  }

  return fDpDt;
}

double DynaPidSpeed::toOutput()
{
  if( m_fCV < (double)m_nSpeedMin )
  {
    return (double)m_nSpeedMin;
  }
  else if( m_fCV > (double)m_nSpeedMax )
  {
    return (double)m_nSpeedMax;
  }
  else
  {
    return m_fCV;
  }
}
