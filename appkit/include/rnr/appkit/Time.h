////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_appkit
//
// File:      Time.h
//
//
/*! \file
 *
 * \brief Time functions and class interfaces.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2015-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
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

#include <iostream>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Global Namespace
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

/*!
 * \brief A timespec insertion operator.
 *
 * \param os  Output stream.
 * \param obj Object to insert.
 *
 * \return Reference to output stream.
 */
extern std::ostream &operator<<(std::ostream &os, const timespec &obj);

namespace rnr
{
  namespace time
  {
    const long      MILLION = 1000000;      ///< 1,000,000
    const long long BILLION = 1000000000;   ///< 1,000,000,000

    /*!
     * \brief Get the current time, indentified by CLOCK_REALTIME, since the
     * last Epoch.
     *
     * \return Timespec.
     */
    extern timespec now();

    /*!
     * \brief Get the current time, indentified by CLOCK_REALTIME, since the
     * last Epoch.
     *
     * \param [out] ts    Timespec.
     *
     * \copydoc doc_return_std
     */
    extern int now(timespec &ts);

    /*!
     * \brief Convert timespec to floating point number equivalent.
     *
     * \param ts    Timespec.
     *
     * \return  Seconds and fractions of a second.
     */
    extern double toFp(const timespec &ts);

    /*!
     * \brief Convert floating point seconds to timespec equivalent.
     *
     * \param  Seconds and fractions of a second.
     *
     * \return ts    Timespec.
     */
    extern timespec toTs(const double &t);

    /*!
     * \brief Check if timespec is set.
     *
     * \param ts    Timespec.
     *
     * \return Returns true or false.
     */
    extern bool isSet(const timespec &a);

    /*!
     * \brief Clear this object's time.
     *
     * \param ts    Timespec.
     */
    extern void clear(timespec &ts);

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
    extern timespec add(const timespec &a, const timespec &b);

    /*!
     * \brief a + b
     *
     * \param a   Timespec a.
     * \param b   Timespec b.
     *
     * \return Normalized timespec.
     */
    inline timespec operator+(const timespec &a, const timespec &b)
    {
      return add(a, b);
    }
  
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
    extern timespec sub(const timespec &a, const timespec &b);

    /*!
     * \brief a - b
     *
     * \param a   Timespec a.
     * \param b   Timespec b.
     *
     * \return Normalized timespec.
     */
    inline timespec operator-(const timespec &a, const timespec &b)
    {
      return sub(a, b);
    }
  
    /*!
     * \brief Normalize the timespec.
     *
     * \param a   Timespec.
     *
     * \return Normalized timespec where tv_nsec has a value in the range
     * 0 to 999,999,999.
     */
    extern timespec normalize(const timespec &a);


    //--------------------------------------------------------------------------
    // Class Time
    //--------------------------------------------------------------------------
  
    /*!
     * \brief Time class.
     *
     * The class uses timespec as the underlining interface to system calls.
     * Some calls require the lower resolution timeval interface. This class
     * will autobmaitcally down/up samples between the two interfaces
     */
    class Time
    {
    public:
      /*!
       * \brief Default constructor.
       */
      Time();
  
      /*!
       * \brief Initialization constructor.
       *
       * \param ts  Time in timespec format.
       */
      Time(const timespec &ts);
  
      /*!
       * \brief Initialization constructor.
       *
       * \param t   Seconds.
       */
      Time(const double &t);
  
      /*!
       * \brief Copy constructor.
       *
       * \param src Source object.
       */
      Time(const Time &src);
  
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
       * \return timespec.
       */
      timespec ts()
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
        return time::isSet(m_tsTime);
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
       * No internal time data are touched.
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
  
  
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // (Compound) Assignment Operators
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  
      /*!
       * \defgroup doc_t_op_assign
       * \{
       * \brief this = b
       *
       * \param b   Rvalue object.
       *
       * \return Reference to this.
       * \}
       * \copydoc doc_t_op_assign
       */
      Time &operator=(const Time &b);
  
      /*! \copydoc doc_t_op_assign */
      Time &operator=(const timespec &b);
  
      /*! \copydoc doc_t_op_assign */
      Time &operator=(const double &b);
  
      /*!
       * \defgroup doc_t_op_add_assign
       * \{
       * \brief this += b
       *
       * \param b   Rvalue object.
       *
       * \return Reference to this.
       * \}
       * \copydoc doc_t_op_add_assign
       */
      Time &operator+=(const Time &b);
  
      /*! \copydoc doc_t_op_add_assign */
      Time &operator+=(const timespec &b);
  
      /*! \copydoc doc_t_op_add_assign */
      Time &operator+=(const double &b);
  
      /*!
       * \defgroup doc_t_op_sub_assign
       * \{
       * \brief this -= b
       *
       * \param b   Rvalue object.
       *
       * \return Reference to this.
       * \}
       * \copydoc doc_t_op_sub_assign
       */
      Time &operator-=(const Time &b);
  
      /*! \copydoc doc_t_op_sub_assign */
      Time &operator-=(const timespec &b);
  
      /*! \copydoc doc_t_op_sub_assign */
      Time &operator-=(const double &b);
  
      /*!
       * \brief this *= b
       *
       * \param b   Rvalue object.
       *
       * \return Reference to this.
       */
      Time &operator*=(const double &b);
  
  
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Arithmetic Operators
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  
      /*!
       * \defgroup doc_t_op_add
       * \{
       * \brief c = a + b
       *
       * \param a   Time object.
       * \param b   Rvalue object.
       *
       * \return Time
       * \}
       * \copydoc doc_t_op_add
       */
      friend Time operator+(Time a, const Time &b)
      {
        return a += b;
      }
  
      /*! \copydoc doc_t_op_add */
      friend Time operator+(Time a, const timespec &b)
      {
        return a += b;
      }
  
      /*! \copydoc doc_t_op_add */
      friend Time operator+(Time a, const double &b)
      {
        return a += b;
      }
  
      /*!
       * \defgroup doc_t_op_sub
       * \{
       * \brief c = a - b
       *
       * \param a   Time object.
       * \param b   Rvalue object.
       *
       * \return Time
       * \}
       * \copydoc doc_t_op_sub
       */
      friend Time operator-(Time a, const Time &b)
      {
        return a -= b;
      }
  
      /*! \copydoc doc_t_op_sub */
      friend Time operator-(Time a, const timespec &b)
      {
        return a -= b;
      }
  
      /*! \copydoc doc_t_op_sub */
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
  
  
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Comparision Operators
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  
      /*!
       * \defgroup doc_t_op_eq
       * \{
       * \brief this == b
       *
       * \param b   Rvalue object.
       *
       * \return
       * Returns true if this object is the same time as b.
       * Otherwise returns false.
       * \}
       * \copydoc doc_t_op_eq
       */
      bool operator==(const Time &b)
      {
        return operator==(b.m_tsTime);
      }
  
      /*! \copydoc doc_t_op_eq */
      bool operator==(const timespec &b)
      {
        return  (m_tsTime.tv_sec == b.tv_sec) &&
                (m_tsTime.tv_nsec == b.tv_nsec);
      }
  
      /*! \copydoc doc_t_op_eq */
      bool operator==(const double &b)
      {
        return m_fpTime == b;
      }
  
      /*!
       * \defgroup doc_t_op_lt
       * \{
       * \brief this < b
       *
       * \param b   Rvalue object.
       *
       * \return
       * Returns true if this object is an earlier time than b.
       * Otherwise returns false.
       * \}
       * \copydoc doc_t_op_lt
       */
      bool operator<(const Time &b)
      {
        return operator<(b.m_tsTime);
      }
  
      /*! \copydoc doc_t_op_lt */
      bool operator<(const timespec &b);
  
      /*! \copydoc doc_t_op_lt */
      bool operator<(const double &b)
      {
        return m_fpTime < b;
      }
  
      /*!
       * \defgroup doc_t_op_gt
       * \{
       * \brief this > b
       *
       * \param b   Rvalue object.
       *
       * \return
       * Returns true if this object is an later time than b.
       * Otherwise returns false.
       * \}
       * \copydoc doc_t_op_gt
       */
      bool operator>(const Time &b)
      {
        return operator>(b.m_tsTime);
      }
  
      /*! \copydoc doc_t_op_gt */
      bool operator>(const timespec &b);
  
      /*! \copydoc doc_t_op_gt */
      bool operator>(const double &b)
      {
        return m_fpTime > b;
      }
  
      /*!
       * \defgroup doc_t_op_le
       * \{
       * \brief this <= b
       *
       * \param b   Rvalue object.
       *
       * \return
       * Returns true if this object is an earlier or equal time to b.
       * Otherwise returns false.
       * \}
       * \copydoc doc_t_op_le
       */
      bool operator<=(const Time &b)
      {
        return operator<(b.m_tsTime) || operator==(b.m_tsTime);
      }
  
      /*! \copydoc doc_t_op_le */
      bool operator<=(const timespec &b)
      {
        return operator<(b) || operator==(b);
      }
  
      /*! \copydoc doc_t_op_le */
      bool operator<=(const double &b)
      {
        return m_fpTime <= b;
      }
  
      /*!
       * \defgroup doc_t_op_ge
       * \{
       * \brief this >= b
       *
       * \param b   Rvalue object.
       *
       * \return
       * Returns true if this object is an later or equal time to b.
       * Otherwise returns false.
       * \}
       * \copydoc doc_t_op_ge
       */
      bool operator>=(const Time &b)
      {
        return operator>(b.m_tsTime) || operator==(b.m_tsTime);
      }
  
      /*! \copydoc doc_t_op_ge */
      bool operator>=(const timespec &b)
      {
        return operator>(b) || operator==(b);
      }
  
      /*! \copydoc doc_t_op_ge */
      bool operator>=(const double &b)
      {
        return m_fpTime >= b;
      }
  
  
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // String Dates and Times
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  
      /*
       * \brief Convert this time to local calender time.
       *
       * Examples:
       * Resolution = 0: "Wed Jun 30 21:49:08 2017" \n
       * Resolution = 2: "Wed Jun 30 21:49:08.38 2017"
       * 
       * \param resSec  Resolution of the second field [0,9].
       *
       * \return String.
       */
      std::string calendarTime(const int resSec = 0) const;
  
      friend std::ostream &operator<<(std::ostream &os, const Time &obj);
  
    protected:
      timespec  m_tsTime;   ///< time in timespec format
      double    m_fpTime;   ///< time in floating point format
    }; // class Time

    /*!
     * \brief Time insertion operator.
     *
     * \param os  Output stream.
     * \param obj Object to insert.
     *
     * \return Reference to output stream.
     */
    extern std::ostream &operator<<(std::ostream &os, const Time &obj);
  
  } // namespace time
} // namespace rnr


#endif // _RNR_TIME_H
