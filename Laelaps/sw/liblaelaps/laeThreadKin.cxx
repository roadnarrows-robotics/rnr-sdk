////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// File:      laeThreadKin.cxx
//
/*! \file
 *
 * $LastChangedDate: 2016-02-08 16:43:26 -0700 (Mon, 08 Feb 2016) $
 * $Rev: 4307 $
 *
 * \brief Laelaps kinodynamics thread class implementation.
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

#include "Laelaps/laeKin.h"

#include "Laelaps/laeThread.h"
#include "Laelaps/laeThreadKin.h"

using namespace std;
using namespace laelaps;

//------------------------------------------------------------------------------
// LaeThreadKin Class
//------------------------------------------------------------------------------
  
const double LaeThreadKin::ThreadKinPrioDft    = 99;
const double LaeThreadKin::ThreadKinHzDft      = 30.0;
const long   LaeThreadKin::ThreadKinTHealthDft = 5;

LaeThreadKin::LaeThreadKin(LaeKinematics &kin) :
    LaeThread("Kinodynamics"), m_kin(kin)
{
  m_bWaitOneCycle = false;
}

LaeThreadKin::~LaeThreadKin()
{
}

int LaeThreadKin::reload(const LaeTunes &tunes)
{
  double  fHz;
  int     rc;

  lock();

  fHz = tunes.getKinematicsHz();

  if( fHz != m_fHz )
  {
    setHz(fHz);
  }

  rc = m_kin.reload(tunes);

  unlock();

  return rc;
}

void LaeThreadKin::waitOneCycle()
{
  lock();

  m_bWaitOneCycle = true;

  pthread_cond_wait(&m_condSync, &m_mutexSync);

  unlock();
}

void LaeThreadKin::transToRunning()
{
  m_tsSchedHealth = m_tsExecThis;
}

void LaeThreadKin::exec()
{
  m_kin.exec();

  // time to monitor health
  if( m_tsSchedHealth < m_tsExecThis )
  { 
    m_kin.monitorHealth();
    m_tsSchedHealth = m_tsExecThis;
    m_tsSchedHealth.tv_sec += ThreadKinTHealthDft;
  }
  
  // caller blocked waiting one execution cycle - signal to unblock
  if( m_bWaitOneCycle )
  {
    m_bWaitOneCycle = false;
    pthread_cond_signal(&m_condSync);
  }
}
