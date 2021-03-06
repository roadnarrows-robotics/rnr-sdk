////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekUtils.h
//
//
/*! \file
 *
 * $LastChangedDate: 2015-04-17 15:31:34 -0600 (Fri, 17 Apr 2015) $
 * $Rev: 3942 $
 *
 * \brief Hekateros common utilities.
 *
 * \author Daniel Packard (daniel@roadnarrows.com)
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2012-2017. RoadNarrows LLC.\n
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

#ifndef _HEK_UTILS_H
#define _HEK_UTILS_H

#include <sys/types.h>
#include <sys/time.h>
#include <time.h>
#include <math.h>

#include <string>
#include <vector>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Hekateros/hekateros.h"

#define M_TAU (2.0 * M_PI)    ///< tau = 2 * pi

namespace hekateros
{
  const long      MILLION = 1000000;
  const long long BILLION = 1000000000;


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // String functions.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

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


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Math functions.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

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
  inline int cap(int a, int min, int max)
  {
    return a<min? min: a>max? max: a;
  }

  /*!
   * \brief Sign of a.
   *
   * \return Return 1.0 or -1.0.
   */
  inline double signof(double a)
  {
    return a<0? -1.0: 1.0;
  }

  /*!
   * \brief Sign of a.
   *
   * \return Return 1 or -1.
   */
  inline int signof(int a)
  {
    return a<0? -1: 1;
  }


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // timeval functions.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \brief Compare operator to test if left hand side time is earlier than
   * the right hand side time.
   *
   * lhs \h_lt rhs
   *
   * \param lhs   Left hand side time.
   * \param rhs   Right hand side time.
   *
   * \return Returns true or false.
   */
  bool operator<(const struct timeval& lhs, const struct timeval& rhs);

  /*!
   * \brief Compare operator to test if left hand side time equals
   * the right hand side time.
   *
   * lhs == rhs
   *
   * \param lhs   Left hand side time.
   * \param rhs   Right hand side time.
   *
   * \return Returns true or false.
   */
  bool operator==(const struct timeval& lhs, const struct timeval& rhs);

  /*!
   * \brief Compare operator to test if left hand side time is later than
   * the right hand side time.
   *
   * lhs \h_gt rhs
   *
   * \param lhs   Left hand side time.
   * \param rhs   Right hand side time.
   *
   * \return Returns true or false.
   */
  bool operator>(const struct timeval& lhs, const struct timeval& rhs);

  /*!
   * \brief Addition operator.
   *
   * op1 + op2
   *
   * \param op1   Left hand side time.
   * \param op2   Right hand side time.
   *
   * \return Sum of times.
   */
  struct timeval operator+(const struct timeval& op1,
                           const struct timeval& op2);

  /*!
   * \brief Subtraction operator.
   *
   * op1 - op2, op1 \h_ge op2.
   *
   * \param op1   Left hand side time.
   * \param op2   Right hand side time.
   *
   * \return Difference of times.
   */
  struct timeval operator-(const struct timeval& op1,
                           const struct timeval& op2);

  /*!
   * \brief Calculate delta time in microseconds.
   *
   * t1 - t0, t1 \h_ge t0
   *
   * \param t1  Later time.
   * \param t0  Earlier time.
   *
   * \return Returns dt in microseconds.
   */
  long dt_usec(struct timeval& t1, struct timeval& t0);

  /*!
   * \brief Calculate delta time.
   *
   * t1 - t0
   *
   * \param t1  Time 1.
   * \param t0  Time 0.
   *
   * \return Returns dt as a double. May be negative.
   */
  double dt(struct timeval& t1, struct timeval& t0);


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // timespec functions.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \brief Compare operator to test if left hand side time is earlier than
   * the right hand side time.
   *
   * lhs \h_lt rhs
   *
   * \param lhs   Left hand side time.
   * \param rhs   Right hand side time.
   *
   * \return Returns true or false.
   */
  bool operator<(const struct timespec& lhs, const struct timespec& rhs);

  /*!
   * \brief Compare operator to test if left hand side time equals
   * the right hand side time.
   *
   * lhs == rhs
   *
   * \param lhs   Left hand side time.
   * \param rhs   Right hand side time.
   *
   * \return Returns true or false.
   */
  bool operator==(const struct timespec& lhs, const struct timespec& rhs);

  /*!
   * \brief Compare operator to test if left hand side time is later than
   * the right hand side time.
   *
   * lhs \h_gt rhs
   *
   * \param lhs   Left hand side time.
   * \param rhs   Right hand side time.
   *
   * \return Returns true or false.
   */
  bool operator>(const struct timespec& lhs, const struct timespec& rhs);

  /*!
   * \brief Addition operator.
   *
   * op1 + op2
   *
   * \param op1   Left hand side time.
   * \param op2   Right hand side time.
   *
   * \return Sum of times.
   */
  struct timespec operator+(const struct timespec& op1,
                            const struct timespec& op2);

  /*!
   * \brief Subtraction operator.
   *
   * op1 - op2, op1 \h_ge op2.
   *
   * \param op1   Left hand side time.
   * \param op2   Right hand side time.
   *
   * \return Difference of times.
   */
  struct timespec operator-(const struct timespec& op1,
                            const struct timespec& op2);

  /*!
   * \brief Calculate delta time in nanoseconds.
   *
   * t1 - t0, t1 \h_ge t0
   *
   * \param t1  Later time.
   * \param t0  Earlier time.
   *
   * \return Returns dt in nanoseconds.
   */
  long long dt_nsec(struct timespec& t1, struct timespec& t0);
  
  /*!
   * \brief Calculate delta time.
   *
   * t1 - t0
   *
   * \param t1  Time 1.
   * \param t0  Time 0.
   *
   * \return Returns dt as a double. May be negative.
   */
  double dt(struct timespec& t1, struct timespec& t0);
  

  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Miscellanea.
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  
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

  /*!
   * \brief Split string at the delimiter character.
   *
   * \param s     String to split.
   * \param delem Delimiter character.
   * \param [out] elems   Vector of split strings.
   *
   * \return Reference to vector of split strings.
   */
  std::vector<std::string> &split(const std::string &s,
                                  char delim,
                                  std::vector<std::string> &elems);

  /*!
   * \brief Split string at the delimiter character.
   *
   * \param s     String to split.
   * \param delem Delimiter character.
   *
   * \return Vector of split strings.
   */
  std::vector<std::string> split(const std::string &s, char delim);

} // namespace hekateros


#endif // _HEK_UTILS_H
