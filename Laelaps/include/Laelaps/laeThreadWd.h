////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// File:      laeThreadWd.h
//
/*! \file
 *
 * $LastChangedDate: 2015-08-07 14:25:35 -0600 (Fri, 07 Aug 2015) $
 * $Rev: 4051 $
 *
 * \brief Laelaps watchdog thread class interface.
 *
 * The watchdog thread monitors system health and energy, and provides an
 * interface to the WatchDog subprocessor.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2015-2017. RoadNarrows LLC.\n
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

#ifndef _LAE_THREAD_WD_H
#define _LAE_THREAD_WD_H

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

#include "Laelaps/laeGpio.h"
#include "Laelaps/laeBatt.h"
#include "Laelaps/laeAlarms.h"
#include "Laelaps/laeWatchDog.h"
#include "Laelaps/laeWd.h"

#include "Laelaps/laeThread.h"

/*!
 *  \brief The \h_laelaps namespace encapsulates all \h_laelaps related
 *  constructs.
 */
namespace laelaps
{

  //----------------------------------------------------------------------------
  // LaeThreadWd Class
  //----------------------------------------------------------------------------
  
  /*!
   * Watchdog thread class.
   */
  class LaeThreadWd : public LaeThread
  {
  public:
    static const double ThreadWdPrioDft = 50;   ///< default priority
    static const double ThreadWdHzDft   = 1.0;  ///< default run rate

    /*!
     * \brief Optimize thread hertz rate given the watchdog timeout value.
     *
     * \param fWatchDogTimeout    Seconds.
     *
     * \return Returns hertz.
     */
    static double optimizeHz(const double fWatchDogTimeout);

    /*!
     * \brief Default constructor.
     */
    LaeThreadWd(LaeWd &hwif);

    /*!
     * \brief Destructor.
     */
    virtual ~LaeThreadWd();

    /*!
     * \brief Reload tunable paramaters. 
     *
     * This function executes under the lock/unlock mutex.
     */
    int reload(const LaeTunes &tunes);

  protected:
    LaeWd            &m_hwif;     ///< hardware interface
    LaeWatchDogReset  m_reset;    ///< sub-processor reset
    LaeBattery        m_battery;  ///< battery and energy monitor
    LaeAlarms         m_alarms;   ///< alarm monitor

    /*!
     * \brief Execute watchdog task within scheduled cycle.
     *
     * This function executes under the lock/unlock mutex.
     *
     * \par Context:
     * This thread.
     */
    virtual void exec();
  };
  
} // namespace laelaps


#endif // _LAE_THREAD_WD_H
