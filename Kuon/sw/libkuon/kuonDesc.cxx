////////////////////////////////////////////////////////////////////////////////
//
// Package:   Kuon
//
// Library:   libkuon
//
// File:      kuonDesc.cxx
//
/*! \file
 *
 * $LastChangedDate: 2014-04-04 17:05:03 -0600 (Fri, 04 Apr 2014) $
 * $Rev: 3629 $
 *
 * \brief Kuon full robotic mobile platform descripition class implementation.
 *
 * The description includes the base platform plus any supported sensor and
 * robotic payloads.
 *
 * Descriptions are a mash-up of factory fixed specificiations plus run-time
 * XML configuration.
 *
 * \author Robin Knight     (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2014-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
/*
 * @EulaBegin@
// Unless otherwise noted, all materials contained are copyrighted and may not
// be used except as provided in these terms and conditions or in the copyright
// notice (documents and software ) or other proprietary notice provided with
// the relevant materials.
//
//
// IN NO EVENT SHALL THE AUTHOR, ROADNARROWS, OR ANY MEMBERS/EMPLOYEES/
// CONTRACTORS OF ROADNARROWS OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
// PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
// EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.
//
// THE AUTHORS AND  ROADNARROWS SPECIFICALLY DISCLAIM ANY WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
// "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
// PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>

#include <string>
#include <vector>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Kuon/kuon.h"
#include "Kuon/kuonDescBase.h"
#include "Kuon/kuonDesc.h"

using namespace std;
using namespace kuon;


void KuonDesc::resetDesc()
{
  m_descBase.resetDesc();
  m_strFullBrief.clear();
  m_bIsDescribed = false;
}

int KuonDesc::markAsDescribed()
{
  if( !m_descBase.isDescribed() )
  {
    LOGERROR("Kuon base platform description is undefined.");
    return -KUON_ECODE_BAD_OP;
  }

  // add payload descriptions when available.
  m_strFullBrief =  m_descBase.getProdBrief();

  m_bIsDescribed = true;

  return KUON_OK;
}
