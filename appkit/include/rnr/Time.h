////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Library:   librnr_appkit
//
// File:      Time.h
//
//
/*! \file
 *
 * $LastChangedDate: 2015-11-09 17:38:34 -0700 (Mon, 09 Nov 2015) $
 * $Rev: 4195 $
 *
 * \brief Time class interfaces.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 * (C) 2015.  RoadNarrows LLC.
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _RNR_TIME_H
#define _RNR_TIME_H

#include <sys/types.h>
#include <sys/time.h>
#include <time.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

namespace rnr
{
  const long      MILLION = 1000000;
  const long long BILLION = 1000000000;


  //----------------------------------------------------------------------------
  // Class Time
  //----------------------------------------------------------------------------

  /*!
   * \brief Time class.
   *
   * The class uses timespec as unlining interface to system calls. Some
   * calls require the lower resolution timeval interface. This class
   * autobmaitcally down/up samples between the two interfaces
   */
  class Time
  {
  public:
    typedef struct timespec timespec_t;   ///< typedef'ed timespec structure
    typedef struct timeval timeval_t;     ///< typedef'ed timeval structure

    /*!
     * \brief Default constructor.
     */
    Time();

    /*!
     * \brief Initialization constructor.
     *
     * \param ts  Time in timespec format.
     */
    Time(const timespec_t &ts);

    /*!
     * \brief Initialization constructor.
     *
     * \param t   Seconds.
     */
    Time(const double &t);

    /*!
     * \brief Destructor.
     */
    ~Time() 
    {
    }

    /*!
     * \brief Get this object's value as a floating point number of seconds
     * and fractions of a second.
     *
     * \return Double.
     */
    double t()
    {
      return m_fpTime;
    }

    /*!
     * \brief Get this object's value in timespec format.
     *
     * \return timespec_t.
     */
    timespec_t ts()
    {
      return m_tsTime;
    }

    /*!
     * \brief Check if this object's time is set.
     *
     * \return Returns true or false.
     */
    bool isSet()
    {
      return Time::isSet(m_tsTime);
    }

    /*!
     * \brief Clear this object's time.
     */
    void clear();


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Clock and Time Functions
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    
    /*!
     * \brief Get the CLOCK_REALTIME resolution.
     *
     * \return Returns a floating point number of the resolution in seconds
     * and fractions of a second.
     */
    double getResolution();

    /*!
     * \brief Get the current time, indentified by CLOCK_REALTIME, since the
     * last Epoch.
     *
     * \return Returns a floating point number of the current time in seconds
     * and fractions of a second.
     */
    double now();

    /*!
     * \brief Mark the current time, indentified by CLOCK_REALTIME, since the
     * last Epoch.
     *
     * The internal time data are automatically updated.
     *
     * \return Returns a floating point number of the current time in seconds
     * and fractions of a second.
     */
    double markNow();


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // (Compound) Assignment Operators
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief this = b
     *
     * \param b   Rvalue object.
     *
     * \return Reference to this.
     */
    Time &operator=(const Time &b);

    /*!
     * \brief this = b
     *
     * \param b   Rvalue object.
     *
     * \return Reference to this.
     */
    Time &operator=(const timespec_t &b);

    /*!
     * \brief this = b
     *
     * \param b   Rvalue object.
     *
     * \return Reference to this.
     */
    Time &operator=(const double &b);

    /*!
     * \brief this += b
     *
     * \param b   Rvalue object.
     *
     * \return Reference to this.
     */
    Time &operator+=(const Time &b);

    /*!
     * \brief this += b
     *
     * \param b   Rvalue object.
     *
     * \return Reference to this.
     */
    Time &operator+=(const timespec_t &b);

    /*!
     * \brief this += b
     *
     * \param b   Rvalue object.
     *
     * \return Reference to this.
     */
    Time &operator+=(const double &b);

    /*!
     * \brief this -= b
     *
     * \param b   Rvalue object.
     *
     * \return Reference to this.
     */
    Time &operator-=(const Time &b);

    /*!
     * \brief this -= b
     *
     * \param b   Rvalue object.
     *
     * \return Reference to this.
     */
    Time &operator-=(const timespec_t &b);

    /*!
     * \brief this -= b
     *
     * \param b   Rvalue object.
     *
     * \return Reference to this.
     */
    Time &operator-=(const double &b);

    /*!
     * \brief this *= b
     *
     * \param b   Rvalue object.
     *
     * \return Reference to this.
     */
    Time &operator*=(const double &b);


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Arithmetic Operators
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief c = a + b
     *
     * \param a   Time object.
     * \param b   Rvalue object.
     *
     * \return Time
     */
    friend Time operator+(Time a, const Time &b)
    {
      return a += b;
    }

    /*!
     * \brief c = a + b
     *
     * \param a   Time object.
     * \param b   Rvalue object.
     *
     * \return Time
     */
    friend Time operator+(Time a, const timespec_t &b)
    {
      return a += b;
    }

    /*!
     * \brief c = a + b
     *
     * \param a   Time object.
     * \param b   Rvalue object.
     *
     * \return Time
     */
    friend Time operator+(Time a, const double &b)
    {
      return a += b;
    }

    /*!
     * \brief c = a - b
     *
     * \param a   Time object.
     * \param b   Rvalue object.
     *
     * \return Time
     */
    friend Time operator-(Time a, const Time &b)
    {
      return a -= b;
    }

    /*!
     * \brief c = a - b
     *
     * \param a   Time object.
     * \param b   Rvalue object.
     *
     * \return Time
     */
    friend Time operator-(Time a, const timespec_t &b)
    {
      return a -= b;
    }

    /*!
     * \brief c = a - b
     *
     * \param a   Time object.
     * \param b   Rvalue object.
     *
     * \return Time
     */
    friend Time operator-(Time a, const double &b)
    {
      return a -= b;
    }

    /*!
     * \brief c = a * b
     *
     * \param a   Time object.
     * \param b   Rvalue object.
     *
     * \return Time
     */
    friend Time operator*(Time a, const double &b)
    {
      return a *= b;
    }


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Comparision Operators
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief this == b
     *
     * \param b   Rvalue object.
     *
     * \return
     * Returns true if this object is the same time as b.
     * Otherwise returns false.
     */
    bool operator==(const Time &b)
    {
      return operator==(b.m_tsTime);
    }

    /*!
     * \brief this == b
     *
     * \param b   Rvalue object.
     *
     * \return
     * Returns true if this object is the same time as b.
     * Otherwise returns false.
     */
    bool operator==(const timespec_t &b)
    {
      return  (m_tsTime.tv_sec == b.tv_sec) && (m_tsTime.tv_nsec == b.tv_nsec);
    }

    /*!
     * \brief this == b
     *
     * \param b   Rvalue object.
     *
     * \return
     * Returns true if this object is the same time as b
     * Otherwise returns false.
     */
    bool operator==(const double &b)
    {
      return m_fpTime == b;
    }

    /*!
     * \brief this < b
     *
     * \param b   Rvalue object.
     *
     * \return
     * Returns true if this object is an earlier time than b.
     * Otherwise returns false.
     */
    bool operator<(const Time &b)
    {
      return operator<(b.m_tsTime);
    }

    /*!
     * \brief this < b
     *
     * \param b   Rvalue object.
     *
     * \return
     * Returns true if this object is an earlier time than b.
     * Otherwise returns false.
     */
    bool operator<(const timespec_t &b);

    /*!
     * \brief this < b
     *
     * \param b   Rvalue object.
     *
     * \return
     * Returns true if this object is an earlier time than b.
     * Otherwise returns false.
     */
    bool operator<(const double &b)
    {
      return m_fpTime < b;
    }

    /*!
     * \brief this > b
     *
     * \param b   Rvalue object.
     *
     * \return
     * Returns true if this object is an later time than b.
     * Otherwise returns false.
     */
    bool operator>(const Time &b)
    {
      return operator>(b.m_tsTime);
    }

    /*!
     * \brief this > b
     *
     * \param b   Rvalue object.
     *
     * \return
     * Returns true if this object is an later time than b.
     * Otherwise returns false.
     */
    bool operator>(const timespec_t &b);

    /*!
     * \brief this > b
     *
     * \param b   Rvalue object.
     *
     * \return
     * Returns true if this object is an later time than b.
     * Otherwise returns false.
     */
    bool operator>(const double &b)
    {
      return m_fpTime > b;
    }

    /*!
     * \brief this <= b
     *
     * \param b   Rvalue object.
     *
     * \return
     * Returns true if this object is an earlier or equal time to b.
     * Otherwise returns false.
     */
    bool operator<=(const Time &b)
    {
      return operator<(b.m_tsTime) || operator==(b.m_tsTime);
    }

    /*!
     * \brief this <= b
     *
     * \param b   Rvalue object.
     *
     * \return
     * Returns true if this object is an earlier or equal time to b.
     * Otherwise returns false.
     */
    bool operator<=(const timespec_t &b)
    {
      return operator<(b) || operator==(b);
    }

    /*!
     * \brief this <= b
     *
     * \param b   Rvalue object.
     *
     * \return
     * Returns true if this object is an earlier or equal time to b.
     * Otherwise returns false.
     */
    bool operator<=(const double &b)
    {
      return m_fpTime <= b;
    }

    /*!
     * \brief this >= b
     *
     * \param b   Rvalue object.
     *
     * \return
     * Returns true if this object is an later or equal time to b.
     * Otherwise returns false.
     */
    bool operator>=(const Time &b)
    {
      return operator>(b.m_tsTime) || operator==(b.m_tsTime);
    }

    /*!
     * \brief this >= b
     *
     * \param b   Rvalue object.
     *
     * \return
     * Returns true if this object is an later or equal time to b.
     * Otherwise returns false.
     */
    bool operator>=(const timespec_t &b)
    {
      return operator>(b) || operator==(b);
    }

    /*!
     * \brief this >= b
     *
     * \param b   Rvalue object.
     *
     * \return
     * Returns true if this object is an later or equal time to b.
     * Otherwise returns false.
     */
    bool operator>=(const double &b)
    {
      return m_fpTime >= b;
    }


    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
    // Statics
    // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

    /*!
     * \brief Get the current time, indentified by CLOCK_REALTIME, since the
     * last Epoch.
     *
     * \param [out] ts    Timespec.
     *
     * \copydoc doc_return_std
     */
    static int now(timespec_t &ts);

    /*!
     * \brief Convert timespec to floating point number equivalent.
     *
     * \param ts    Timespec.
     *
     * \return  Seconds and fractions of a second.
     */
    static double toFp(const timespec_t &ts);

    /*!
     * \brief Convert floating point seconds to timespec equivalent.
     *
     * \param  Seconds and fractions of a second.
     *
     * \return ts    Timespec.
     */
    static timespec_t toTs(const double &t);

    /*!
     * \brief Check if timespec is set.
     *
     * \param ts    Timespec.
     *
     * \return Returns true or false.
     */
    static bool isSet(const timespec_t &a);

    /*!
     * \brief Clear this object's time.
     *
     * \param ts    Timespec.
     */
    static void clear(timespec_t &ts);

    /*!
     * \brief Add two timespecs.
     *
     * a + b.
     *
     * \param a   Timespec a.
     * \param b   Timespec b.
     *
     * \return Normalized timespec.
     */
    static timespec_t add(const timespec_t &a, const timespec_t &b);

    /*!
     * \brief Subtract two timespecs.
     *
     * a - b
     *
     * \param a   Timespec a.
     * \param b   Timespec b.
     *
     * \return Normalized timespec.
     */
    static timespec_t sub(const timespec_t &a, const timespec_t &b);

    /*!
     * \brief Normalize the timespec.
     *
     * \param a   Timespec.
     *
     * \return Normalized timespec where tv_nsec has a value in the range
     * 0 to 999,999,999.
     */
    static timespec_t norm(const timespec_t &a);

  protected:
    timespec_t  m_tsTime;   // time in timespec_t format
    double      m_fpTime;   // time in floating point format
  }; // Class Time

} // namespace rnr


#endif // _RNR_TIME_H
