////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// File:      laeThreadWd.cxx
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
 * \par Copyright
 *   \h_copy 2015-2018. RoadNarrows LLC.\n
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

#include "Laelaps/laeBatt.h"
#include "Laelaps/laeAlarms.h"
#include "Laelaps/laeWatchDog.h"
#include "Laelaps/laeWd.h"

#include "Laelaps/laeThread.h"
#include "Laelaps/laeThreadWd.h"

using namespace std;
using namespace laelaps;

//------------------------------------------------------------------------------
// LaeThreadWd Class
//------------------------------------------------------------------------------
  
const double LaeThreadWd::ThreadWdPrioDft = 50;   ///< default priority
const double LaeThreadWd::ThreadWdHzDft   = 1.0;  ///< default run rate

double LaeThreadWd::optimizeHz(const double fWatchDogTimeout)
{
  double  t;

  t = fWatchDogTimeout / 2.0 - 0.001;
  if( t > LaeThreadWd::ThreadWdHzDft )
  {
    t = LaeThreadWd::ThreadWdHzDft;
  }
  return t;
}

LaeThreadWd::LaeThreadWd(LaeWd &hwif) : LaeThread("WatchDog"), m_hwif(hwif)
{
}

LaeThreadWd::~LaeThreadWd()
{
}

int LaeThreadWd::reload(const LaeTunes &tunes)
{
  int     rc;

  lock();

  rc = m_hwif.reload(tunes);

  unlock();

  return rc;
}

void LaeThreadWd::exec()
{
  m_hwif.exec();
  m_battery.update();
  m_alarms.update();
}
