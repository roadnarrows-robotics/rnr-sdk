////////////////////////////////////////////////////////////////////////////////
//
// Package:   Kuon
//
// Library:   libkuon
//
// File:      kuonDescBase.cxx
//
/*! \file
 *
 * $LastChangedDate: 2014-04-07 08:16:52 -0600 (Mon, 07 Apr 2014) $
 * $Rev: 3631 $
 *
 * \brief Kuon robotic base mobile platform description class implementation.
 *
 * The base description does not include any payload descriptions.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 * (C) 2014.  RoadNarrows LLC.
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
 * 
 * Permission is hereby granted, without written agreement and without
 * license or royalty fees, to use, copy, modify, and distribute this
 * software and its documentation for any purpose, provided that
 * (1) The above copyright notice and the following two paragraphs
 * appear in all copies of the source code and (2) redistributions
 * including binaries reproduces these notices in the supporting
 * documentation.   Substantial modifications to this software may be
 * copyrighted by their authors and need not follow the licensing terms
 * described here, provided that the new terms are clearly indicated in
 * all files where they apply.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
 * OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <math.h>

#include <string>
#include <vector>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Kuon/kuon.h"
#include "Kuon/kuonUtils.h"
#include "Kuon/kuonSpec.h"
#include "Kuon/kuonProdBase.h"
#include "Kuon/kuonDescBase.h"

using namespace std;
using namespace kuon;


KuonDescBase::KuonDescBase() :
  m_strProdFamily(KuonProdFamily)
{
  m_eProdId     = KuonProdIdUnknown;
  m_uProdHwVer  = 0;
}

void KuonDescBase::setDesc(int          eProdId,
                           const string &strProdName,
                           const string &strProdBrief,
                           const string &strHwVer,
                           double        fFrontTireRadius,
                           double        fRearTireRadius)
{
  m_eProdId       = eProdId;
  m_strProdName   = strProdName.empty()? getProdName(m_eProdId): strProdName;
  m_strProdBrief  = strProdBrief.empty()? getProdBrief(m_eProdId): strProdBrief;
  m_strProdHwVer  = strHwVer;
  m_uProdHwVer    = strToVersion(strHwVer);

  // set arm specification
  m_spec.set(m_eProdId, m_uProdHwVer, fFrontTireRadius, fRearTireRadius);
}

void KuonDescBase::resetDesc()
{
  m_eProdId       = KuonProdIdUnknown;
  m_strProdName.clear();
  m_strProdBrief.clear();
  m_strProdHwVer.clear();

  // clear arm specification
  m_spec.clear();
}

const char *KuonDescBase::getProdName(int eProdId)
{
  switch( eProdId )
  {
    case KuonProdIdStd:
      return "Kuon-Standard";
    case KuonProdIdNarrow:
      return "Kuon-Narrow";
    default:
      return "";
  }
}

const char *KuonDescBase::getProdBrief(int eProdId)
{
  switch( eProdId )
  {
    case KuonProdIdStd:
      return "RoadNarrows Kuon Standard robotic mobile platform";
    case KuonProdIdNarrow:
      return "RoadNarrows Kuon Narrow robotic mobile platform";
    default:
      return "";
  }
}
