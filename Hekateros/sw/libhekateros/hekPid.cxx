////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekPid.cxx
//
/*! \file
 *
 * $LastChangedDate: 2014-11-30 19:56:45 -0700 (Sun, 30 Nov 2014) $
 * $Rev: 3817 $
 *
 * The Hekateros joint position and velocity PID class implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2014-2018. RoadNarrows LLC.\n
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

#include <math.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Dynamixel/Dynamixel.h"
#include "Dynamixel/DynaPid.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekPid.h"
#include "Hekateros/hekUtils.h"

using namespace std;
using namespace hekateros;


// -----------------------------------------------------------------------------
// HekPid Class
// -----------------------------------------------------------------------------

const double HekPid::WI_DFT = 0.25;   ///< integral sum of errors weight const

void HekPid::specifySetPoint(double fJointGoalPos, double fJointGoalVel)
{ 
  DynaPid::SpecifySetPoint(fJointGoalPos, true);

  m_fJointGoalPos = fJointGoalPos;
  m_fJointGoalVel = fJointGoalVel;
}

double HekPid::toOutput(double fCVJointTgtVel)
{
  double fMax = fabs(m_fJointGoalVel);

  // limit delta velocity
  if( m_fMaxDeltaV != HekTunePidDeltaVNoMax )
  {
    fCVJointTgtVel = fcap(fCVJointTgtVel, m_fJointCurVel-m_fMaxDeltaV,
                                          m_fJointCurVel+m_fMaxDeltaV);
  }

  return fcap(fCVJointTgtVel, -fMax, fMax);
}
