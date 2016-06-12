////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Library:   librnr_appkit
//
// File:      Random.cxx
//
//
/*! \file
 *
 * $LastChangedDate: 2015-11-09 11:56:44 -0700 (Mon, 09 Nov 2015) $
 * $Rev: 4194 $
 *
 * \brief Random variable generator class implementation.
 *
 * \author Daniel Packard (daniel@roadnarrows.com)
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 * (C) 2012-2016.  RoadNarrows LLC.
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

#include <stdlib.h>
#include <time.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/appkit/Random.h"


using namespace rnr;


// -----------------------------------------------------------------------------
// Random Class
// -----------------------------------------------------------------------------

Random::Random(ulong_t luSeed)
{
  ushort_t  seed[3];  // 48-bits

  if( luSeed == AUTO_SEED )
  {
    luSeed = (ulong_t)time(NULL);
  }

  seed[0] = (ushort_t)(luSeed & 0xffff);
  seed[1] = (ushort_t)((luSeed >> 16) & 0xffff);
  seed[2] = (ushort_t)(seed[0] << 8) | (ushort_t)(seed[1] >> 8);

  seed48_r(seed, &m_randState);
}

int Random::randrange(int nMin, int nMax)
{
  double  lfRandVal;

  drand48_r(&m_randState, &lfRandVal);

  return (int)((double)(nMax - nMin) * lfRandVal + nMin);
}

s64_t Random::randrange(s64_t nMin, s64_t nMax)
{
  double  lfRandVal;

  drand48_r(&m_randState, &lfRandVal);

  return (s64_t)((double)(nMax - nMin) * lfRandVal + nMin);
}

float Random::uniform(float fMin, float fMax)
{
  double  lfRandVal;

  drand48_r(&m_randState, &lfRandVal);

  return (float)((double)(fMax - fMin) * lfRandVal + fMin);
}
