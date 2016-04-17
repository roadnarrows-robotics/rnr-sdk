////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekDesc.cxx
//
/*! \file
 *
 * $LastChangedDate: 2014-09-18 16:53:49 -0600 (Thu, 18 Sep 2014) $
 * $Rev: 3748 $
 *
 * \brief Hekateros full description.
 *
 * \author Robin Knight     (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2013-2014.  RoadNarrows
 * (http://www.roadnarrows.com)
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

#include <stdio.h>

#include <string>
#include <vector>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekDescArm.h"
#include "Hekateros/hekDescEE.h"
#include "Hekateros/hekDesc.h"

using namespace std;
using namespace hekateros;


void HekDesc::resetDesc()
{
  m_descArm.resetDesc();
  m_descEE.resetDesc();
  m_strFullBrief.clear();
  m_bIsDescribed = false;
}

int HekDesc::markAsDescribed()
{
  if( !m_descArm.isDescribed() )
  {
    LOGERROR("Hekateros base description is undefined.");
    return -HEK_ECODE_BAD_OP;
  }

  if( !m_descEE.isDescribed() )
  {
    LOGERROR("Hekateros end effector description is undefined.");
    return -HEK_ECODE_BAD_OP;
  }

  m_strFullBrief =  m_descArm.getProdBrief() +
                    " with " +
                    m_descEE.getProdBrief();

  m_bIsDescribed = true;

  return HEK_OK;
}

bool HekDesc::hasServo(int nServoId)
{
  if( m_descArm.hasServo(nServoId) )
  {
    return true;
  }
  else if( m_descEE.hasServo(nServoId) )
  {
  }
  else
  {
    return false;
  }
}



#ifdef RDK_REDO
void hekDesc::setServoOffset(int servoId, int servoOffset) {
  m_servoOffsets[servoId] = servoOffset;
}

void hekDesc::setServoLimCW(int servoId, int servoMin) {
  m_servosLimCW[servoId] = servoMin;
}

void hekDesc::setServoLimCCW(int servoId, int servoMax) {
  m_servosLimCCW[servoId] = servoMax;
}
#endif // RDK_REDO
