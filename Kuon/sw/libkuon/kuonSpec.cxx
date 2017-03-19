////////////////////////////////////////////////////////////////////////////////
//
// Package:   Kuon
//
// Library:   libkuon
//
// File:      kuonSpec.cxx
//
/*! \file
 *
 * $LastChangedDate: 2014-04-04 17:05:03 -0600 (Fri, 04 Apr 2014) $
 * $Rev: 3629 $
 *
 * \brief \h_kuon product factory specification base implimentations.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2013-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
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

#include <string>
#include <vector>

#include "rnr/rnrconfig.h"

#include "Kuon/kuon.h"
#include "Kuon/kuonSpec.h"
#include "Kuon/kuonProdBase.h"

using namespace std;
using namespace kuon;


KuonSpec::KuonSpec()
{
  m_eProdId           = KuonProdIdUnknown;
  m_uHwVer            = 0;
  m_nNumMotors        = 0;
  m_fFrontTireRadius  = 0.0;
  m_fRearTireRadius   = 0.0;
}

KuonSpec::~KuonSpec()
{
}

int KuonSpec::set(int    eProdId,
                  uint_t uHwVer,
                  double fFrontTireRadius,
                  double fRearTireRadius)
{
  const KuonSpecMotor_T  *pSpecMotors = NULL;
  int                     i;

  clear();

  switch( eProdId )
  {
    //
    // Standard size Kuon
    //
    case KuonProdIdStd:
      m_nNumMotors  = KuonProdBaseStdNumMotors;
      pSpecMotors   = KuonProdBaseStdSpecMotors;
      break;

    //
    // Narrow size Kuon
    //
    case KuonProdIdNarrow:
      break;
  }

  if( pSpecMotors != NULL )
  {
    for(i=0; i<m_nNumMotors; ++i)
    {
      m_vecSpecMotors.push_back(pSpecMotors[i]);

      if( (pSpecMotors[i].m_nMotorId == KuonMotorIdLF) ||
          (pSpecMotors[i].m_nMotorId == KuonMotorIdRF) )
      {
        m_fFrontTireRadius = pSpecMotors[i].m_fTireRadius;
      }
      else if( (pSpecMotors[i].m_nMotorId == KuonMotorIdLR) ||
               (pSpecMotors[i].m_nMotorId == KuonMotorIdRR) )
      {
        m_fRearTireRadius = pSpecMotors[i].m_fTireRadius;
      }
    }
  }

  m_eProdId     = eProdId;
  m_uHwVer      = uHwVer;

  //
  // Override factory defaults.
  //
  if( fFrontTireRadius > 0.0 )
  {
    m_fFrontTireRadius = fFrontTireRadius;
  }
  if( fRearTireRadius > 0.0 )
  {
    m_fRearTireRadius = fRearTireRadius;
  }

  return KUON_OK;
}

void KuonSpec::clear()
{
  m_eProdId = KuonProdIdUnknown;
  m_uHwVer  = 0;
  m_vecSpecMotors.clear();
}

KuonSpecMotor_T *KuonSpec::getMotorSpec(const string &strName)
{
  vector<KuonSpecMotor_T>::iterator    iter;

  for(iter = m_vecSpecMotors.begin(); iter != m_vecSpecMotors.end(); ++iter)
  {
    if( iter->m_strName == strName )
    {
      return &(*iter);
    }
  }

  return NULL;
}

KuonSpecMotor_T *KuonSpec::getMotorSpec(int nMotorId)
{
  vector<KuonSpecMotor_T>::iterator    iter;

  for(iter = m_vecSpecMotors.begin(); iter != m_vecSpecMotors.end(); ++iter)
  {
    if( iter->m_nMotorId == nMotorId )
    {
      return &(*iter);
    }
  }

  return NULL;
}
