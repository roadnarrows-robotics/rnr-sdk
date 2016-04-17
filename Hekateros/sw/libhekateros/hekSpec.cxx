////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekSpec.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-12-10 10:43:51 -0700 (Thu, 10 Dec 2015) $
 * $Rev: 4239 $
 *
 * \brief \h_hek product specification base implimentations.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2013.  RoadNarrows
 * (http://www.RoadNarrows.com)
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

#include <string>
#include <vector>

#include "rnr/rnrconfig.h"

#include "Dynamixel/Dynamixel.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekProdArm.h"
#include "Hekateros/hekProdEE.h"
#include "Hekateros/hekProdEquip.h"
#include "Hekateros/hekProdAux.h"

using namespace std;
using namespace hekateros;


HekSpec::HekSpec()
{
  m_eProdFamily   = HekProdFamilyUnknown;
  m_eProdId       = HekProdIdUnknown;
  m_uHwVer        = 0;
  m_nNumLinks     = 0;
  m_nDoF          = 0;
  m_nNumOptLimits = 0;
  m_nNumServos    = 0;
}

HekSpec::~HekSpec()
{
}

int HekSpec::set(int eProdId, uint_t uHwVer)
{
  const HekSpecLink_T  *pSpecLinks;
  const HekSpecJoint_T *pSpecJoints;
  const HekSpecServo_T *pSpecServos;
  int                   i;

  clear();

  switch( eProdId )
  {
    //
    // Arms
    //
    case HekProdArm5LBetaId: // defunct
      break;
    case HekProdArm4SId:
      break;
    case HekProdArm4LId:
      m_eProdFamily   = HEK_ARM_FAMILY;
      m_nNumLinks     = HekProdArm4LNumLinks;
      m_nDoF          = HekProdArm4LDoF;
      m_nNumOptLimits = HekProdArm4LNumOptLimits;
      m_nNumServos    = HekProdArm4LNumServos;
      if( uHwVer < HEK_VERSION(1, 2, 0) )
      {
        pSpecLinks      = HekProdArm4LSpecLinks;
        pSpecJoints     = HekProdArm4LSpecJoints_1_1;
        pSpecServos     = HekProdArm4LSpecServos_1_1;
      }
      else
      {
        LOGWARN("Unsupported 4L v%u.%u.%u, defaulting to v1.1 specs.",
          HEK_VER_MAJOR(uHwVer), HEK_VER_MINOR(uHwVer), HEK_VER_REV(uHwVer)); 
        pSpecLinks      = HekProdArm4LSpecLinks;
        pSpecJoints     = HekProdArm4LSpecJoints_1_1;
        pSpecServos     = HekProdArm4LSpecServos_1_1;
      }
      LOGDIAG3("Hekateros 4L v%u.%u.%u specification loaded.",
          HEK_VER_MAJOR(uHwVer), HEK_VER_MINOR(uHwVer), HEK_VER_REV(uHwVer)); 
      break;
    case HekProdArm5SId:
      break;
    case HekProdArm5LId:
      m_eProdFamily   = HEK_ARM_FAMILY;
      m_nNumLinks     = HekProdArm5LNumLinks;
      m_nDoF          = HekProdArm5LDoF;
      m_nNumOptLimits = HekProdArm5LNumOptLimits;
      m_nNumServos    = HekProdArm5LNumServos;
      if( uHwVer < HEK_VERSION(1, 2, 0) )
      {
        pSpecLinks      = HekProdArm5LSpecLinks;
        pSpecJoints     = HekProdArm5LSpecJoints_1_1;
        pSpecServos     = HekProdArm5LSpecServos_1_1;
      }
      else if( uHwVer < HEK_VERSION(1, 3, 0) )
      {
        pSpecLinks      = HekProdArm5LSpecLinks;
        pSpecJoints     = HekProdArm5LSpecJoints_1_2;
        pSpecServos     = HekProdArm5LSpecServos_1_2;
      }
      else if( uHwVer < HEK_VERSION(1, 4, 0) )
      {
        if( uHwVer == HEK_VERSION(1, 3, 5) )   // special SIUE arm hybrid
        {
          pSpecLinks      = HekProdArm5LSpecLinks;
          pSpecJoints     = HekProdArm5LSpecJoints_1_3_5;
          pSpecServos     = HekProdArm5LSpecServos_1_3_5;
        }
        else
        {
          pSpecLinks      = HekProdArm5LSpecLinks;
          pSpecJoints     = HekProdArm5LSpecJoints_1_3;
          pSpecServos     = HekProdArm5LSpecServos_1_3;
        }
      }
      else if( uHwVer < HEK_VERSION(1, 5, 0) )
      {
        pSpecLinks      = HekProdArm5LSpecLinks;
        pSpecJoints     = HekProdArm5LSpecJoints_1_4;
        pSpecServos     = HekProdArm5LSpecServos_1_4;
      }
      else if( uHwVer < HEK_VERSION(2, 2, 0) )
      {
        pSpecLinks      = HekProdArm5LSpecLinks;
        pSpecJoints     = HekProdArm5LSpecJoints_2_0;
        pSpecServos     = HekProdArm5LSpecServos_2_0;
      }
      else
      {
        LOGWARN("Unsupported 5L v%u.%u.%u, defaulting to v1.3 specs.",
          HEK_VER_MAJOR(uHwVer), HEK_VER_MINOR(uHwVer), HEK_VER_REV(uHwVer)); 
        pSpecLinks      = HekProdArm5LSpecLinks;
        pSpecJoints     = HekProdArm5LSpecJoints_1_3;
        pSpecServos     = HekProdArm5LSpecServos_1_3;
      }
      LOGDIAG3("Hekateros 5L v%u.%u.%u specification loaded.",
          HEK_VER_MAJOR(uHwVer), HEK_VER_MINOR(uHwVer), HEK_VER_REV(uHwVer)); 
      break;

    //
    // End Effectors
    //
    case HekProdEEGraboidId:
      m_eProdFamily   = HekProdEEGraboidFamily;
      m_nNumLinks     = HekProdEEGraboidNumLinks;
      m_nDoF          = HekProdEEGraboidDoF;
      m_nNumOptLimits = HekProdEEGraboidNumOptLimits;
      m_nNumServos    = HekProdEEGraboidNumServos;
      if( uHwVer < HEK_VERSION(1, 1, 0) )
      {
        pSpecLinks      = HekProdEEGraboidSpecLinks;
        pSpecJoints     = HekProdEEGraboidSpecJoints_1_0;
        pSpecServos     = HekProdEEGraboidSpecServos_1_0;
      }
      else if( uHwVer < HEK_VERSION(1, 2, 0) )
      {
        pSpecLinks      = HekProdEEGraboidSpecLinks;
        pSpecJoints     = HekProdEEGraboidSpecJoints_1_1;
        pSpecServos     = HekProdEEGraboidSpecServos_1_1;
      }
      else
      {
        LOGWARN("Unsupported Graboid EE v%u.%u.%u, "
                "defaulting to v1.1 specs.",
          HEK_VER_MAJOR(uHwVer), HEK_VER_MINOR(uHwVer), HEK_VER_REV(uHwVer)); 
        pSpecLinks      = HekProdEEGraboidSpecLinks;
        pSpecJoints     = HekProdEEGraboidSpecJoints_1_1;
        pSpecServos     = HekProdEEGraboidSpecServos_1_1;
      }
      LOGDIAG3("Graboid EE v%u.%u.%u specification loaded.",
          HEK_VER_MAJOR(uHwVer), HEK_VER_MINOR(uHwVer), HEK_VER_REV(uHwVer)); 
      break;

    // error
    default:
      LOGERROR("0x%08x: Unknown or unsupported product id.", eProdId);
      return -HEK_ECODE_BAD_VAL;
  }

  if( pSpecLinks != NULL )
  {
    for(i=0; i<m_nNumLinks; ++i)
    {
      m_vecSpecLinks.push_back(pSpecLinks[i]);
    }
  }

  if( pSpecJoints != NULL )
  {
    for(i=0; i<m_nDoF; ++i)
    {
      m_vecSpecJoints.push_back(pSpecJoints[i]);
    }
  }

  if( pSpecServos != NULL )
  {
    for(i=0; i<m_nNumServos; ++i)
    {
      m_vecSpecServos.push_back(pSpecServos[i]);
    }
  }

  m_eProdId     = eProdId;
  m_uHwVer      = uHwVer;

  return HEK_OK;
}

void HekSpec::clear()
{
  m_eProdId = HekProdIdUnknown;
  m_uHwVer  = 0;
  m_vecSpecLinks.clear();
  m_vecSpecJoints.clear();
  m_vecSpecServos.clear();
}

HekSpecJoint_T *HekSpec::getJointSpec(string &strName)
{
  vector<HekSpecJoint_T>::iterator    iter;

  for(iter = m_vecSpecJoints.begin(); iter != m_vecSpecJoints.end(); ++iter)
  {
    if( iter->m_strName == strName )
    {
      return &(*iter);
    }
  }

  return NULL;
}

HekSpecJoint_T *HekSpec::getJointSpec(int nServoId)
{
  vector<HekSpecJoint_T>::iterator    iter;

  for(iter = m_vecSpecJoints.begin(); iter != m_vecSpecJoints.end(); ++iter)
  {
    if( iter->m_nMasterServoId == nServoId )
    {
      return &(*iter);
    }
  }

  return NULL;
}

HekSpecServo_T *HekSpec::getServoSpec(int nServoId)
{
  vector<HekSpecServo_T>::iterator    iter;

  for(iter = m_vecSpecServos.begin(); iter != m_vecSpecServos.end(); ++iter)
  {
    if( iter->m_nServoId == nServoId )
    {
      return &(*iter);
    }
  }

  return NULL;
}

bool HekSpec::hasServo(int nServoId)
{
  vector<HekSpecServo_T>::iterator iter;

  for(iter = m_vecSpecServos.begin(); iter != m_vecSpecServos.end(); ++iter)
  {
    if( iter->m_nServoId == nServoId )
    {
      return true;
    }
  }
  return false;
}
