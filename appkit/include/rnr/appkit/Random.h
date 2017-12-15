////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_appkit
//
// File:      Random.h
//
/*! \file
 *
 * $LastChangedDate: 2015-11-09 17:38:34 -0700 (Mon, 09 Nov 2015) $
 * $Rev: 4195 $
 *
 * \brief Random variable generator class interface.
 *
 * \author Daniel Packard (daniel@roadnarrows.com)
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2012-2017. RoadNarrows LLC.\n
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

#ifndef _RNR_RANDOM_H
#define _RNR_RANDOM_H

#include <stdlib.h>
#include <limits.h>

#include "rnr/rnrconfig.h"

/*!
 * \brief RoadNarrows Robotics
 */
namespace rnr
{
  //----------------------------------------------------------------------------
  // Random Class
  //----------------------------------------------------------------------------
 
  /*!
   *  \ingroup rnr_appkit
   *
   *  \brief Random variable generators class.
   */
  class Random
  {
  public:
    static const ulong_t AUTO_SEED = 0;

    /*!
     * \brief Default initialization constructor.
     *
     * \param luSeed    Random generator seed.
     */
    Random(ulong_t luSeed = AUTO_SEED);
    
    /*!
     * \brief Destructor.
     */
    virtual ~Random() { }

    /*!
     * \brief Generates a random integer uniformally distrubuted
     *        between [-2^31, 2^32-1].
     *
     * \return Random value.
     */
    int randint()
    {
      long lRandVal;

      mrand48_r(&m_randState, &lRandVal);

      return (int)lRandVal;
    }

    /*!
     * \brief Generates a random integer uniformally distrubuted
     *        between [nMin, nMax].
     *
     * \param nMin    Minimum value.
     * \param nMax    Maximum value.
     *
     * \return Random value.
     */
    int   randrange(int nMin=0, int nMax=INT_MAX);

    /*!
     * \brief Generates a random 64-bit integer uniformally distrubuted
     *        between [nMin, nMax].
     *
     * \param nMin    Minimum value.
     * \param nMax    Maximum value.
     *
     * \return Random value.
     */
    s64_t randrange(s64_t nMin=0, s64_t nMax=LLONG_MAX);

    /*!
     * \brief Generates a random float uniformally distrubuted
     *        between [fMin, fMax].
     *
     * \param fMin    Minimum value.
     * \param fMax    Maximum value.
     *
     * \return Random value.
     */
    float uniform(float fMin=0.0, float fMax=1.0);

  protected:
    struct drand48_data m_randState;    ///< random generator opaque state.
  };
}

#endif // _RNR_RANDOM_H
