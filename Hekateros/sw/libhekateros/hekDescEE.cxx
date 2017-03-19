////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekDescEE.cxx
//
//
/*! \file
 *
 * $LastChangedDate: 2014-09-18 16:53:49 -0600 (Thu, 18 Sep 2014) $
 * $Rev: 3748 $
 *
 * \brief HekDescEE - Hekateros end effector tool description class interface.
 *
 * \author Daniel Packard (daniel@roadnarrows.com)
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2012-2017. RoadNarrows LLC.\n
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

#include <stdio.h>

#include <string>
#include <vector>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekUtils.h"
#include "Hekateros/hekSpec.h"
#include "Hekateros/hekProdEE.h"
#include "Hekateros/hekDescEE.h"

using namespace std;
using namespace hekateros;


HekDescEE::HekDescEE()
{
  m_eProdId     = HekProdIdUnknown;
  m_eProdSize   = HekProdSizeUnknown; 
  m_uProdHwVer  = 0;
  m_nDoF        = 0;
}

void HekDescEE::setDesc(int           eProdId,
                        const string &strProdName,
                        const string &strProdBrief,
                        const string &strHwVer,
                        int           nDoF,
                        int           eProdSize)
{
  m_eProdId       = eProdId;
  m_strProdName   = strProdName.empty()? getProdName(m_eProdId): strProdName;
  m_strProdBrief  = strProdBrief.empty()? getProdBrief(m_eProdId): strProdBrief;
  m_strProdHwVer  = strHwVer;
  m_uProdHwVer    = strToVersion(strHwVer);
  m_nDoF          = nDoF == 0? getDoF(m_eProdId): nDoF;
  m_eProdSize     = eProdSize == HekProdSizeUnknown? getProdSize(m_eProdId):
                                                     eProdSize;

  // set end effector specification
  m_spec.set(m_eProdId, m_uProdHwVer);
}

void HekDescEE::resetDesc()
{
  m_eProdId       = HekProdIdUnknown;
  m_strProdName.clear();
  m_strProdBrief.clear();
  m_strProdHwVer.clear();
  m_nDoF          = 0;
  m_eProdSize     = HekProdSizeUnknown;

  // clear arm specification
  m_spec.clear();
}

const char *HekDescEE::getProdName(int eProdId)
{
  switch( eProdId )
  {
    case HekProdEEFixedId:
      return "Fixed";
    case HekProdEEGraboidId:
      return "Graboid";
    default:
      return "";
  }
}

const char *HekDescEE::getProdBrief(int eProdId)
{
  switch( eProdId )
  {
    case HekProdEEFixedId:
      return "Fixed end effector";
    case HekProdEEGraboidId:
      return "RoadNarrows Graboid 1DOF simple end effector";
    default:
      return "";
  }
}

int HekDescEE::getProdSize(int eProdId)
{
  switch( eProdId )
  {
    case HekProdEEFixedId:
    case HekProdEEGraboidId:
      return HekProdSizeStd;
    default:
      return HekProdSizeUnknown;
  }
}

int HekDescEE::getDoF(int eProdId)
{
  switch( eProdId )
  {
    case HekProdEEFixedId:
      return 0;
    case HekProdEEGraboidId:
      return 1;
    default:
      return 0;
  }
}
