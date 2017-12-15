////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Library:   librnr_appkit
//
// File:      Time.cxx
//
//
/*! \file
 *
 * $LastChangedDate: 2015-11-09 17:38:34 -0700 (Mon, 09 Nov 2015) $
 * $Rev: 4195 $
 *
 * \brief Time functions and class implementations.
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


#include <sys/types.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <math.h>
#include <stdio.h>
#include <errno.h>

#include <iostream>
#include <iomanip>
#include <sstream>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/appkit/StringTheory.h"
#include "rnr/appkit/Time.h"

using namespace std;
using namespace rnr;
using namespace rnr::chronos;


ostream &operator<<(ostream &os, const timespec &obj)
{
  char    c = os.fill();
  size_t  w = os.width();

  os << obj.tv_sec << "."
    << setw(9) << setfill('0') << obj.tv_nsec
    << setfill(c) << setw(w);

  return os;
}

namespace rnr
{
  namespace chronos
  {
    timespec now()
    {
      timespec  ts;
    
      if( now(ts) != OK )
      {
        clear(ts);
      }
      return ts;
    }
    
    int now(timespec &ts)
    {
      return clock_gettime(CLOCK_REALTIME, &ts) == 0? OK: RC_ERROR;
    }
    
    double toFp(const timespec &ts)
    {
      return (double)ts.tv_sec + (double)ts.tv_nsec/ (double)BILLION;
    }
    
    timespec toTs(const double &t)
    {
      timespec  ts;
      double    sec, nsec;
    
      sec  = floor(t); 
      nsec = (t - sec) * (double)BILLION;
    
      ts.tv_sec  = (long)sec;
      ts.tv_nsec = (long)nsec;
    
      return ts;
    }
    
    bool isSet(const timespec &ts)
    {
      return (ts.tv_sec != 0) || (ts.tv_nsec != 0);
    }
    
    void clear(timespec &ts)
    {
      ts.tv_sec   = 0;
      ts.tv_nsec  = 0;
    }
    
    timespec add(const timespec &a, const timespec &b)
    {
      timespec  ts;
    
      ts.tv_sec  = a.tv_sec  + b.tv_sec;
      ts.tv_nsec = a.tv_nsec + b.tv_nsec;
    
      return rnr::chronos::normalize(ts);
    }
    
    timespec sub(const timespec &a, const timespec &b)
    {
      timespec  ts;
    
      ts.tv_sec  = a.tv_sec  - b.tv_sec;
      ts.tv_nsec = a.tv_nsec - b.tv_nsec;
    
      return rnr::chronos::normalize(ts);
    }
    
    timespec normalize(const timespec &a)
    {
      timespec    ts = a;
    
      while( ts.tv_nsec >= BILLION )
      {
        ts.tv_sec += 1;
        ts.tv_nsec -= BILLION;
      }
    
      while( ts.tv_nsec < 0 )
      {
        ts.tv_sec -= 1;
        ts.tv_nsec += BILLION;
      }
    
      return ts;
    }
  } // namespace chronos
} // namesapce rnr


//------------------------------------------------------------------------------
// Time Class 
//------------------------------------------------------------------------------

Time::Time()
{
  clear();
}

Time::Time(const timespec &ts)
{
  m_tsTime  = ts;
  m_fpTime  = chronos::toFp(m_tsTime);
}

Time::Time(const double &t)
{
  m_fpTime  = t;
  m_tsTime  = chronos::toTs(m_fpTime);
}

Time::Time(const Time &src)
{
  m_tsTime = src.m_tsTime;
  m_fpTime = src.m_fpTime;
}

void Time::clear()
{
  chronos::clear(m_tsTime);
  m_fpTime = 0.0;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Clock and Time Functions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

double Time::getResolution()
{
  timespec res;

  if( clock_getres(CLOCK_REALTIME, &res) < 0 )
  {
    chronos::clear(res);
  }

  return chronos::toFp(res);
}

double Time::now()
{
  timespec  ts = chronos::now();

  return chronos::toFp(ts);
}

double Time::markNow()
{
  chronos::now(m_tsTime);
  m_fpTime = chronos::toFp(m_tsTime);
  return m_fpTime;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// (Compound) Assignment Operators
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

Time &Time::operator=(const Time &b)
{
  m_tsTime = b.m_tsTime;
  m_fpTime = b.m_fpTime;
  return *this;
}

Time &Time::operator=(const timespec &b)
{
  m_tsTime = b;
  m_fpTime = chronos::toFp(m_tsTime);
  return *this;
}

Time &Time::operator=(const double &b)
{
  m_fpTime = b;
  m_tsTime = chronos::toTs(m_fpTime);
  return *this;
}

Time &Time::operator+=(const Time &b)
{
  m_tsTime = chronos::add(m_tsTime, b.m_tsTime);
  m_fpTime = chronos::toFp(m_tsTime);
  return *this;
}

Time &Time::operator+=(const timespec &b)
{
  m_tsTime = chronos::add(m_tsTime, b);
  m_fpTime = chronos::toFp(m_tsTime);
  return *this;
}

Time &Time::operator+=(const double &b)
{
  m_fpTime += b;
  m_tsTime  = toTs(m_fpTime);
  return *this;
}

Time &Time::operator-=(const Time &b)
{
  m_tsTime = chronos::sub(m_tsTime, b.m_tsTime);
  m_fpTime = chronos::toFp(m_tsTime);
  return *this;
}

Time &Time::operator-=(const timespec &b)
{
  m_tsTime = chronos::sub(m_tsTime, b);
  m_fpTime = chronos::toFp(m_tsTime);
  return *this;
}

Time &Time::operator-=(const double &b)
{
  m_fpTime -= b;
  m_tsTime  = chronos::toTs(m_fpTime);
  return *this;
}

Time &Time::operator*=(const double &b)
{
  m_fpTime *= b;
  m_tsTime  = chronos::toTs(m_fpTime);
  return *this;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Arithmetic Operators
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Comparison Operators
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

bool Time::operator<(const timespec &b)
{
  if( m_tsTime.tv_sec < b.tv_sec )
  {
    return true;
  }
  else if( (m_tsTime.tv_sec == b.tv_sec) && (m_tsTime.tv_nsec < b.tv_nsec) )
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool Time::operator>(const timespec &b)
{
  if( m_tsTime.tv_sec > b.tv_sec )
  {
    return true;
  }
  else if( (m_tsTime.tv_sec == b.tv_sec) && (m_tsTime.tv_nsec > b.tv_nsec) )
  {
    return true;
  }
  else
  {
    return false;
  }
}

std::string Time::calendarTime(const int resSec) const
{
  char  buf[32]; // must be >= 26 bytes

  // "Wed Jun 30 21:49:08 2017\n"
  if( ctime_r(&(m_tsTime.tv_sec), buf) == NULL )
  {
    return "??? ??? ?? ??:??:?? ????";
  }

  string str(buf);

  str::rtrim(str);

  if( resSec > 0 )
  {
    stringstream ss;

    //
    // if t = 153.096 seconds
    //    res = 2                         res = 3
    //  t - floor(t)  ==> 0.096         t - floor(t)  ==> 0.096
    //  0.096 * 10^2  ==> 9.6           0.096 * 10^3  ==> 96.0
    //  trunc(9.6)    ==> 9.0           trunc(96.0)   ==> 96.0
    //  out           ==> 09            out           ==> 096
    //
    int     w = resSec <= 9? resSec: 9;
    double  y = (double)w;
    long    nsec = (long)trunc(((m_fpTime - floor(m_fpTime)) * pow(10.0, y)));

    ss << str.substr(0, 19);
    ss << "." << std::setfill('0') << std::setw(w) << nsec;
    ss << str.substr(20);

    return ss.str();
  }
  else
  {
    return str;
  }
}

ostream &rnr::chronos::operator<<(ostream &os, const Time &obj)
{
  os << obj.m_tsTime;
  return os;
}

