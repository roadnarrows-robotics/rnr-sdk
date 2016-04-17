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
 * \brief Time class implementations.
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


#include <sys/types.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <math.h>
#include <stdio.h>
#include <errno.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/Time.h"

using namespace std;
using namespace rnr;


//------------------------------------------------------------------------------
// Time Class 
//------------------------------------------------------------------------------

Time::Time()
{
  clear();
}

Time::Time(const timespec_t &ts)
{
  m_tsTime  = ts;
  m_fpTime  = toFp(m_tsTime);
}

Time::Time(const double &t)
{
  m_fpTime  = t;
  m_tsTime  = toTs(m_fpTime);
}

void Time::clear()
{
  Time::clear(m_tsTime);
  m_fpTime = 0.0;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Clock and Time Functions
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

double Time::getResolution()
{
  timespec_t res;

  if( clock_getres(CLOCK_REALTIME, &res) < 0 )
  {
    clear(res);
  }

  return toFp(res);
}

double Time::now()
{
  timespec_t  ts;

  if( Time::now(ts) != OK )
  {
    clear(ts);
  }

  return toFp(ts);
}

double Time::markNow()
{
  Time::now(m_tsTime);
  m_fpTime = toFp(m_tsTime);
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

Time &Time::operator=(const timespec_t &b)
{
  m_tsTime = b;
  m_fpTime = toFp(m_tsTime);
  return *this;
}

Time &Time::operator=(const double &b)
{
  m_fpTime = b;
  m_tsTime = toTs(m_fpTime);
  return *this;
}

Time &Time::operator+=(const Time &b)
{
  m_tsTime = add(m_tsTime, b.m_tsTime);
  m_fpTime = toFp(m_tsTime);
  return *this;
}

Time &Time::operator+=(const timespec_t &b)
{
  m_tsTime = Time::add(m_tsTime, b);
  m_fpTime = toFp(m_tsTime);
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
  m_tsTime = sub(m_tsTime, b.m_tsTime);
  m_fpTime = toFp(m_tsTime);
  return *this;
}

Time &Time::operator-=(const timespec_t &b)
{
  m_tsTime = sub(m_tsTime, b);
  m_fpTime = toFp(m_tsTime);
  return *this;
}

Time &Time::operator-=(const double &b)
{
  m_fpTime -= b;
  m_tsTime  = toTs(m_fpTime);
  return *this;
}

Time &Time::operator*=(const double &b)
{
  m_fpTime *= b;
  m_tsTime  = toTs(m_fpTime);
  return *this;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Arithmetic Operators
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Comparison Operators
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

bool Time::operator<(const timespec_t &b)
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

bool Time::operator>(const timespec_t &b)
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


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Statics
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

int Time::now(timespec_t &ts)
{
  return clock_gettime(CLOCK_REALTIME, &ts) == 0? OK: RC_ERROR;
}

double Time::toFp(const timespec_t &ts)
{
  return (double)ts.tv_sec + (double)ts.tv_nsec/ (double)BILLION;
}

Time::timespec_t Time::toTs(const double &t)
{
  timespec_t  ts;
  double      sec, nsec;

  sec  = floor(t); 
  nsec = (t - sec) * (double)BILLION;

  ts.tv_sec  = (long)sec;
  ts.tv_nsec = (long)nsec;

  return ts;
}

bool Time::isSet(const timespec_t &ts)
{
  return (ts.tv_sec != 0) || (ts.tv_nsec != 0);
}

void Time::clear(timespec_t &ts)
{
  ts.tv_sec   = 0;
  ts.tv_nsec  = 0;
}

Time::timespec_t Time::add(const timespec_t &a, const timespec_t &b)
{
  timespec_t  ts;

  ts.tv_sec  = a.tv_sec  + b.tv_sec;
  ts.tv_nsec = a.tv_nsec + b.tv_nsec;

  return Time::norm(ts);
}

Time::timespec_t Time::sub(const timespec_t &a, const timespec_t &b)
{
  timespec_t  ts;

  ts.tv_sec  = a.tv_sec  - b.tv_sec;
  ts.tv_nsec = a.tv_nsec - b.tv_nsec;

  return Time::norm(ts);
}

Time::timespec_t Time::norm(const timespec_t &a)
{
  timespec_t    ts = a;

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
