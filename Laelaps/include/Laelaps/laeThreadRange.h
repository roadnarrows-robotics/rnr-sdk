////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// File:      laeThreadRange.h
//
/*! \file
 *
 * $LastChangedDate: 2015-08-07 14:25:35 -0600 (Fri, 07 Aug 2015) $
 * $Rev: 4051 $
 *
 * \brief Laelaps range sensors thread class interface.
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

#ifndef _LAE_THREAD_RANGE_H
#define _LAE_THREAD_RANGE_H

#include <sys/types.h>
#include <sys/time.h>
#include <time.h>
#include <pthread.h>

#include <string>
#include <vector>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Laelaps/laelaps.h"
#include "Laelaps/laeUtils.h"
#include "Laelaps/laeTune.h"

#include "Laelaps/laeGpio.h"
#include "Laelaps/laeVL6180.h"

#include "Laelaps/laeThread.h"

/*!
 *  \brief The \h_laelaps namespace encapsulates all \h_laelaps related
 *  constructs.
 */
namespace laelaps
{
  //----------------------------------------------------------------------------
  // LaeThreadRange Class
  //----------------------------------------------------------------------------
  
  /*!
   * Range sensors thread class.
   */
  class LaeThreadRange : public LaeThread
  {
  public:
    static const double ThreadRangePrioDft; ///< default priority
    static const double ThreadRangeHzDft;   ///< default run rate

    /*!
     * \brief Default constructor.
     */
    LaeThreadRange(sensor::vl6180::LaeRangeSensorGroup &hwif);

    /*!
     * \brief Destructor.
     */
    virtual ~LaeThreadRange();

    /*!
     * \brief Reload tunable paramaters. 
     *
     * This function executes under the lock/unlock mutex.
     */
    int reload(const LaeTunes &tunes);

  protected:
    sensor::vl6180::LaeRangeSensorGroup &m_hwif;  ///< hardware interface
    LaeI2CMuxReset       m_reset;      ///< multiplexer reset
    double               m_fTSubTask;  ///< sub-task time (seconds)
    size_t               m_iSensor;    ///< next scheduled sensor index

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


#endif // _LAE_THREAD_RANGE_H
