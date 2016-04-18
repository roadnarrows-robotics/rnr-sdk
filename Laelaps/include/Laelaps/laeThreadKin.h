////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// File:      laeThreadKin.h
//
/*! \file
 *
 * $LastChangedDate: 2016-01-21 16:50:25 -0700 (Thu, 21 Jan 2016) $
 * $Rev: 4268 $
 *
 * \brief Laelaps kinodynamics thread class interface.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2015-2016  RoadNarrows
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

#ifndef _LAE_THREAD_KIN_H
#define _LAE_THREAD_KIN_H

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

/*!
 *  \brief The \h_laelaps namespace encapsulates all \h_laelaps related
 *  constructs.
 */
namespace laelaps
{

  //----------------------------------------------------------------------------
  // LaeThreadKin Class
  //----------------------------------------------------------------------------
  
  /*!
   * Watchdog thread class.
   */
  class LaeThreadKin : public LaeThread
  {
  public:
    static const double ThreadKinPrioDft    = 99;   ///< default priority
    static const double ThreadKinHzDft      = 30.0; ///< default run rate
    static const long   ThreadKinTHealthDft = 5;    ///< monitor health period

    /*!
     * \brief Default constructor.
     */
    LaeThreadKin(LaeKinematics &kin);

    /*!
     * \brief Destructor.
     */
    virtual ~LaeThreadKin();

    /*!
     * \brief Reload tunable paramaters. 
     *
     * This function executes under the lock/unlock mutex.
     */
    int reload(const LaeTunes &tunes);

    /*!
     * \brief Force calling thread to wait one full execution cycle.
     *
     * \par Context:
     * Calling thread.
     */
    virtual void waitOneCycle();

  protected:
    LaeKinematics   &m_kin;           ///< kinematics interface
    bool            m_bWaitOneCycle;  ///< [not] waiting for one cycle
    struct timespec m_tsSchedHealth;  ///< working health scheduler time stamp

    /*!
     * \brief Ready to Running state transition function.
     *
     * This function is called after entering the Running state but prior to
     * any run execution.
     *
     * This function does not execute under the lock/unlock mutex.
     *
     * \par Context:
     * This thread.
     */
    virtual void transToRunning();

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


#endif // _LAE_THREAD_KIN_H
