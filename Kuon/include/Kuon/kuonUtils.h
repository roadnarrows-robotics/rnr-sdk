////////////////////////////////////////////////////////////////////////////////
//
// Package:   Kuon
//
// Library:   libkuon
//
// File:      kuonUtils.h
//
//
/*! \file
 *
 * $LastChangedDate: 2014-04-04 17:05:03 -0600 (Fri, 04 Apr 2014) $
 * $Rev: 3629 $
 *
 * \brief Kuon common utilities.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 * (C) 2014.  RoadNarrows LLC.
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

#ifndef _KUON_UTILS_H
#define _KUON_UTILS_H

#include <sys/types.h>
#include <sys/time.h>
#include <math.h>

#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Kuon/kuon.h"

#define M_TAU (2.0 * M_PI)    ///< tau = 2 * pi

namespace kuon
{
  /*!
   * \brief Convert version dotted string to integer equivalent.
   *
   * \param str   Dotted version string "M[.m[.R]]".
   *
   * \return Version number.
   */
  extern uint_t strToVersion(const std::string &str);

  /*!
   * \brief Get the error string describing the \h_hek error code.
   *
   * The absolute value of the error code is taken prior retrieving the string.
   * An unknown or out-of-range error code will be mapped to
   * \ref HEK_ECODE_BADEC.
   *
   * \param  ecode  Instance of \ref hek_ecodes.
   *
   * \return Returns the appropriate error code string.
   */
  extern const char *getStrError(const int ecode);

  /*!
   * \brief Convert degrees to radians
   *
   * \param d   Degrees.
   *
   * \return Radians.
   */
  inline double degToRad(double d)
  {
    return d / 360.0 * M_TAU;
  }

  /*!
   * \brief Convert radians to degrees
   *
   * \param r   Radians.
   *
   * \return Degrees.
   */
  inline double radToDeg(double r)
  {
    return r / M_TAU * 360.0;
  }

  /*!
   * \brief Integer absolute value.
   *
   * \param a   Integer value.
   *
   * \return |a|
   */
  inline int iabs(int a)
  {
    return a>=0? a: -a;
  }

  /*!
   * \brief Cap value within limits [min, max].
   *
   * \param a     Value.
   * \param min   Minimum.
   * \param max   Maximum.
   *
   * \return a: min \h_le a \h_le max
   */
  inline double fcap(double a, double min, double max)
  {
    return a<min? min: a>max? max: a;
  }

  /*!
   * \brief Cap value within limits [min, max].
   *
   * \param a     Value.
   * \param min   Minimum.
   * \param max   Maximum.
   *
   * \return a: min \h_le a \h_le max
   */
  inline int icap(int a, int min, int max)
  {
    return a<min? min: a>max? max: a;
  }

  /*!
   * \brief Boolean to string.
   *
   * \param b   Boolean value.
   *
   * \return Pointer to null-terminated constant character string.
   */
  inline const char *boolstr(bool b)
  {
    return b? "true": "false";
  }

  /*!
   * \brief Compare operator to test if left hand side time is earlier than
   * the right hand side time.
   *
   * \term lhs \h_lt rhs \<==\> lhs is an earlier time than rhs.
   *
   * \param lhs   Left hand side time.
   * \param rhs   Right hand side time.
   *
   * \return Returns true or false.
   */
  inline bool operator<(const struct timeval& lhs, const struct timeval& rhs)
  {
    if( lhs.tv_sec < rhs.tv_sec )
    {
      return true;
    }
    else if( (lhs.tv_sec == rhs.tv_sec) && (lhs.tv_usec < rhs.tv_usec) )
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  /*!
   * \brief Compare operator to test if left hand side time equals
   * the right hand side time.
   *
   * \term lhs == rhs \<==\> lhs time is the same time as the rhs.
   *
   * \param lhs   Left hand side time.
   * \param rhs   Right hand side time.
   *
   * \return Returns true or false.
   */
  inline bool operator==(const struct timeval& lhs, const struct timeval& rhs)
  {
    return (lhs.tv_sec == rhs.tv_sec) && (lhs.tv_usec == rhs.tv_usec)?
              true: false;
  }

  /*!
   * \brief Get real device name.
   *
   * If the given device name is a symbolic link, then the real device the link
   * references is returned. Otherwise the given device name is returned.
   *
   * \param strDevName  Given device name.
   *
   * \return String.
   */
  std::string getRealDeviceName(const std::string &strDevName);

} // namespace kuon


#endif // _KUON_UTILS_H
