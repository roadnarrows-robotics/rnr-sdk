////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Library:   libDynamixel
//
// File:      DynaPidPos.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
 * $Rev: 3845 $
 *
 * The position PID controls to a servo's odometer position setpoint.
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

#include <cstring>
#include <iostream>
#include <fstream>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaOlio.h"
#include "Dynamixel/DynaPid.h"
#include "Dynamixel/DynaPidPos.h"

using namespace std;


// -----------------------------------------------------------------------------
// DynaPidPos Class
// -----------------------------------------------------------------------------

void DynaPidPos::SpecifySetPoint(int    nOdPosStart,
                                 int    nOdPosGoal,
                                 int    nSpeedLim,
                                 int    nCurSpeed,
                                 bool   bIsReversed,
                                 bool   bUnwind)
{ 
  DynaPid::SpecifySetPoint((double)nOdPosGoal, bUnwind);

  m_nOdPosStart   = nOdPosStart;
  m_nOdPosGoal    = nOdPosGoal;
  m_nSpeedLim     = nSpeedLim;
  m_nOdDir        = bIsReversed? -1: 1;

  m_nOdPosCur     = nOdPosStart;

  m_fSpeedAbsMax  = m_nSpeedLim >= 0? (double)m_nSpeedLim: -(double)m_nSpeedLim;
  m_fPrevSpeed    = (double)nCurSpeed;
}

double DynaPidPos::error(double fPVOdPosCur)
{
  m_nOdPosCur = (int)fPVOdPosCur;

  return (double)(m_nOdPosGoal - m_nOdPosCur);
}

double DynaPidPos::toOutput(double fCVSpeed)
{
  double  fSpeed;

  fSpeed = fCVSpeed * (double)m_nOdDir;

  //
  // Cap speed to the absolute maximum which is the target goal speed.
  //
  if( fSpeed > m_fSpeedAbsMax )
  {
    fSpeed = m_fSpeedAbsMax;
  }
  else if( fSpeed < -m_fSpeedAbsMax )
  {
    fSpeed = -m_fSpeedAbsMax;
  }

  //
  // If there is an absolute lower speed, then cap as necessary.
  //
  if( TuneAbsMinSpeed > 0.0 )
  {
    if( (fSpeed > 0.0) && (fSpeed < TuneAbsMinSpeed) )
    {
      fSpeed = TuneAbsMinSpeed;
    }
    else if( (fSpeed < 0.0) && (fSpeed > -TuneAbsMinSpeed) )
    {
      fSpeed = -TuneAbsMinSpeed;
    }
  }

  //
  // Cap the acceleration so that jerkiness and oscillations are minimized.
  //
  if( fabs(fSpeed-m_fPrevSpeed) > TuneMaxSpeedDelta )
  {
    if( m_fPrevSpeed > fSpeed )
    {
      fSpeed = m_fPrevSpeed - TuneMaxSpeedDelta;
    }
    else 
    {
      fSpeed = m_fPrevSpeed + TuneMaxSpeedDelta;
    }
  }

  m_fPrevSpeed = fSpeed;

  return fSpeed;
}
