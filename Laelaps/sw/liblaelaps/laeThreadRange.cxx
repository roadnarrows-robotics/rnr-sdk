////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// File:      laeThreadRange.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-08-07 14:25:35 -0600 (Fri, 07 Aug 2015) $
 * $Rev: 4051 $
 *
 * \brief Laelaps watchdog thread class implementation.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2015  RoadNarrows
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

#include <sys/types.h>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Laelaps/laelaps.h"
#include "Laelaps/laeUtils.h"
#include "Laelaps/laeTune.h"
#include "Laelaps/laeDb.h"

#include "Laelaps/laeVL6180.h"

#include "Laelaps/laeThread.h"
#include "Laelaps/laeThreadRange.h"

using namespace std;
using namespace laelaps;
using namespace sensor::vl6180;

//------------------------------------------------------------------------------
// LaeThreadRange Class
//------------------------------------------------------------------------------
  
LaeThreadRange::LaeThreadRange(LaeRangeSensorGroup &hwif) :
   LaeThread("Range"), m_hwif(hwif)
{
  m_iSensor = 0;
}

LaeThreadRange::~LaeThreadRange()
{
}

int LaeThreadRange::reload(const LaeTunes &tunes)
{
  double  fHz;

  lock();

  fHz = tunes.getRangeHz();

  if( fHz != m_fHz )
  {
    setHz(fHz);
  }

  unlock();

  return LAE_OK;
}

void LaeThreadRange::exec()
{
  if( RtDb.m_product.m_uProdHwVer >= LAE_VERSION(2, 1, 0) )
  {
    m_hwif.m_interface_2_1.exec();
  }
  else
  {
    m_hwif.m_interface_2_0.exec();
  }
}
