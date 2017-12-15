////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_appkit
//
// File:      LogBook.cxx
//
/*! \file
 *
 * \brief
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2017-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 *
 * \par License:
 * MIT
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

#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <deque>
#include <map>
#include <vector>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/appkit/Time.h"
#include "rnr/appkit/LogBook.h"

using namespace std;
using namespace rnr;
using namespace rnr::chronos;
//using rnr::chronos::operator<<;

static timespec       notime = {0, 0};
static LogBook::Entry noentry;


// -----------------------------------------------------------------------------
// Class LogBook::Entry
// -----------------------------------------------------------------------------

LogBook::Entry::Entry() : m_timestamp(notime)
{
}

LogBook::Entry::Entry(const string &strMark, const string &strText) :
    m_strMark(strMark), m_strText(strText)
{
  now(m_timestamp);
}

LogBook::Entry::Entry(const LogBook::Entry &src)
{
  m_timestamp = src.m_timestamp;
  m_strMark   = src.m_strMark;
  m_strText   = src.m_strText;
}

LogBook::Entry &LogBook::Entry::operator=(const LogBook::Entry &rhs)
{
  m_timestamp = rhs.m_timestamp;
  m_strMark   = rhs.m_strMark;
  m_strText   = rhs.m_strText;
}

LogBook::Entry::~Entry()
{
}

bool LogBook::Entry::empty()
{
  return m_strMark.empty() && m_strText.empty() && !isSet(m_timestamp);
}


// -----------------------------------------------------------------------------
// Class LogBook::BookMark
// -----------------------------------------------------------------------------

LogBook::BookMark::BookMark() : m_index(0)
{
}

LogBook::BookMark::BookMark(const LogBook::BookMark &src)
{
  m_strMark = src.m_strMark;
  m_index   = src.m_index;
}

LogBook::BookMark & LogBook::BookMark::operator=(const LogBook::BookMark &rhs)
{
  m_strMark = rhs.m_strMark;
  m_index   = rhs.m_index;
}


// -----------------------------------------------------------------------------
// Class LogBook
// -----------------------------------------------------------------------------

LogBook::LogBook(const std::string strName,
                 size_t            uMaxEntries,
                 size_t            uMaxEntryLen) :
    m_strName(strName),
    m_uMaxEntries(uMaxEntries),
    m_uMaxEntryLen(uMaxEntryLen)

{
  m_uTotalEver    = 0;
  m_uFlags        = FlagONum;
  m_bWarnThrottle = false;
}

LogBook::LogBook(const LogBook &src)
{
  copy(src);
}

LogBook::~LogBook()
{
}

LogBook &LogBook::operator=(const LogBook &rhs)
{
  copy(rhs);
  return *this;
}

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// LogBook Insertion Operators
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

LogBook &LogBook::operator<<(bool val)
{
  if( checkLen() )
  {
    m_ssText << val;
  }

  return *this;
}

LogBook &LogBook::operator<<(char val)
{
  if( checkLen() )
  {
    m_ssText << val;
  }

  return *this;
}

LogBook &LogBook::operator<<(int val)
{
  if( checkLen() )
  {
    m_ssText << val;
  }

  return *this;
}

LogBook &LogBook::operator<<(unsigned int val)
{
  if( checkLen() )
  {
    m_ssText << val;
  }

  return *this;
}

LogBook &LogBook::operator<<(long val)
{
  if( checkLen() )
  {
    m_ssText << val;
  }

  return *this;
}

LogBook &LogBook::operator<<(unsigned long val)
{
  if( checkLen() )
  {
    m_ssText << val;
  }

  return *this;
}

LogBook &LogBook::operator<<(long long val)
{
  if( checkLen() )
  {
    m_ssText << val;
  }

  return *this;
}

LogBook &LogBook::operator<<(unsigned long long val)
{
  if( checkLen() )
  {
    m_ssText << val;
  }

  return *this;
}

LogBook &LogBook::operator<<(float val)
{
  if( checkLen() )
  {
    m_ssText << val;
  }

  return *this;
}

LogBook &LogBook::operator<<(double val)
{
  if( checkLen() )
  {
    m_ssText << val;
  }

  return *this;
}

LogBook &LogBook::operator<<(long double val)
{
  if( checkLen() )
  {
    m_ssText << val;
  }

  return *this;
}

LogBook &LogBook::operator<<(const char *val)
{
  if( checkLen(strlen(val)) )
  {
    m_ssText << val;
  }

  return *this;
}

LogBook &LogBook::operator<<(void *val)
{
  if( checkLen() )
  {
    m_ssText << val;
  }

  return *this;
}

LogBook &LogBook::operator<<(const string &val)
{
  if( checkLen(val.size()) )
  {
    m_ssText << val;
  }

  return *this;
}

LogBook &LogBook::operator<<(LogBook& (*pf)(LogBook&))
{
  pf(*this);
  return *this;
}

LogBook &LogBook::operator<<(ostream& (*pf)(ostream&))
{
  pf(m_ssText);
  return *this;
}

LogBook &LogBook::operator<<(ios& (*pf)(ios&))
{
  pf(m_ssText);
  return *this;
}

LogBook &LogBook::operator<<(ios_base& (*pf)(ios_base&))
{
  pf(m_ssText);
  return *this;
}

LogBook &LogBook::operator<<(const LogBook &log)
{
  size_t      numThis, numSrc;
  BookCIter   iter;

  numThis = size();
  numSrc  = log.size();

  // append source entries
  for(iter = log.m_book.begin(); iter != log.m_book.end(); ++iter)
  {
    m_book.push_back(*iter);
  }

  m_uTotalEver += numSrc;

  // limit log's size
  limitSize();

  return *this;
}

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// LogBook Editing, Access, and Utilities
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

size_t LogBook::logEntry(const string strText, const string strMark)
{
  if( m_uFlags & FlagDebug )
  {
    cerr << "DBG: " << __func__ << "("
      << "mark=\"" << strMark << "\", "
      << "text=\"" << strText << "\")" <<endl;
  }

  Entry  entry(strMark, strText);

  // add log entry to log book
  m_book.push_back(entry);

  ++m_uTotalEver;

  // limit log's size
  limitSize();

  return size();
}

size_t LogBook::logPending()
{
  // no pending entry
  if( m_ssText.str().empty() )
  {
    return size();
  }

  if( m_uFlags & FlagDebug )
  {
    cerr << "DBG: " << __func__ << "() "
      << "mark=\"" << m_strMark << "\", "
      << "text=\"" << m_ssText.str() << "\"" <<endl;
  }

  Entry  entry(m_strMark, m_ssText.str());

  // add pending log entry to log book
  m_book.push_back(entry);

  // erase pending
  erasePending();

  ++m_uTotalEver;

  // limit log's size
  limitSize();

  return size();
}

void LogBook::setPendingText(const string strText)
{
  ssflush();

  m_ssText << strText;
}

void LogBook::setPendingBookMark(const string strMark)
{
  m_strMark = strMark;
}

void LogBook::setPending(const string strText, const string strMark)
{
  setPendingText(strText);
  setPendingBookMark(strMark);
}

void LogBook::erasePendingText()
{
  ssflush();
}

void LogBook::erasePendingBookMark()
{
  m_strMark.clear();
}

void LogBook::erasePending()
{
  ssflush();
  m_strMark.clear();
}

size_t LogBook::eraseToMark(const string strMark, int whence)
{
  BookIter    pos;

  // no bookmark found
  if( (pos = findMark(strMark)) == m_book.end() )
  {
    return size();
  }

  switch( whence )
  {
    case NEWEST:
      m_book.erase(pos, m_book.end());
      break;
    case OLDEST:
      m_book.erase(m_book.begin(), pos);
      break;
    default:
      LOGERROR("Unknown whence = %d", whence);
      break;
  }

  return size();
}

size_t LogBook::eraseEntries(size_t uNumEntries, int whence)
{
  BookIter  first, last;

  if( uNumEntries > size() )
  {
    uNumEntries = size();
  }

  switch( whence )
  {
    case OLDEST:
      first = m_book.begin();
      last  = first + uNumEntries;
      m_book.erase(first, last);
      break;
    case NEWEST:
      last  = m_book.end();
      first = last - uNumEntries;
      m_book.erase(first, last);
      break;
    default:
      LOGERROR("Unknown whence = %d", whence);
      break;
  }

  return size();
}

void LogBook::resize(size_t uMaxEntries)
{
  if( uMaxEntries > 0 )
  {
    m_uMaxEntries = uMaxEntries;
    limitSize();
  }
}

void LogBook::clear()
{
  // logged
  m_book.clear();

  // pending
  erasePending();
}

const LogBook::Entry &LogBook::at(const string &strMark) const
{
  BookCIter pos;

  if( (pos = findMark(strMark)) != m_book.end() )
  {
    return *pos;
  }
  else
  {
    return noentry;
  }
}

const LogBook::Entry &LogBook::at(const size_t index) const
{
  if( index < size() )
  {
    return m_book[index];
  }
  else
  {
    return noentry;
  }
}

const LogBook::Entry &LogBook::at(const size_t index, int whence) const
{
  size_t  i;

  if( index < size() )
  {
    i = whence == NEWEST? size() - index - 1: index;
    return m_book[i];
  }
  else
  {
    return noentry;
  }
}

const LogBook::Entry &LogBook::operator[](const size_t index) const
{
  return at(index);
}

const LogBook::Entry &LogBook::operator[](const string &strMark) const
{
  return at(strMark);
}

const LogBook::Entry &LogBook::lastEntry() const
{
  if( size() > 0 )
  {
    return m_book[size()-1];
  }
  else
  {
    return noentry;
  }
}


size_t LogBook::getBookMarks(BookMarkList &list, int whence) const
{
  switch( whence )
  {
    case OLDEST:
      sortMarks(list);
      break;
    case NEWEST:
      rsortMarks(list);
      break;
    default:
      LOGERROR("Unknown whence = %d", whence);
      list.clear();
  }

  return list.size();
}

std::string LogBook::makeBookMarkLabel()
{
  stringstream ss;

  ss << "_bookmark." << m_uTotalEver << "_";

  return ss.str();
}

// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// LogBook Output Stream Manipulators
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

ostream &rnr::operator<<(ostream &os, const LogBook &log)
{
  return log.printLog(os);
}

ostream &LogBook::printLog(ostream &os) const
{
  // reverse order: newest to oldest
  if( m_uFlags & FlagORev )
  {
    rprt(os, 0, size() - 1);
  }
  // forward order: oldest to newest
  else
  {
    prt(os, 0, size() - 1);
  }

  return os;
}

ostream &LogBook::printToMark(ostream &os, string strMark, int endpt) const
{
  BookCIter pos;
  size_t    indexMark;

  // no mark
  if( (pos = findMark(strMark)) == m_book.end() )
  {
    return os;
  }

  indexMark = distance(m_book.begin(), pos);

  // reverse order: newest to oldest
  if( m_uFlags & FlagORev )
  {
    if( endpt == OLDEST )
    {
      rprt(os, 0, indexMark-1);
    }
    else // NEWEST
    {
      rprt(os, indexMark, size()-1);
    }
  }

  // forward order: oldest to newest
  else
  {
    if( endpt == OLDEST )
    {
      prt(os, 0, indexMark-1);
    }
    else // NEWEST
    {
      prt(os, indexMark, size()-1);
    }
  }

  return os;
}

void LogBook::prt(ostream &os, size_t index0, size_t index1) const
{
  BookCIter iter, pos0, pos1;
  size_t    index;

  if( m_uFlags & FlagDebug )
  {
    os << "DBG: " << __func__ << "(" << index0 << ", " << index1 << ")" << endl;
  }

  if( size() == 0 )
  {
    return;
  }
  else if( (index0 > index1) || (index1 >= size()) )
  {
    return;
  }

  pos0  = m_book.begin() + index0;
  pos1  = m_book.begin() + index1;
  index = index0;

  for(iter = pos0; iter != m_book.end() && iter <= pos1; ++iter)
  {
    if( (m_uFlags & FlagOMark) && !iter->m_strMark.empty() )
    {
      os << "        ~~ " << iter->m_strMark << endl;
    }

    if( m_uFlags & FlagONum )
    {
      os << setw(3) << index << ". ";
    }

    if( m_uFlags & FlagOTime )
    {
      os << "[" << iter->m_timestamp << "] ";
    }

    os << iter->m_strText << endl;

    ++index;
  }
}

void LogBook::rprt(ostream &os, size_t index0, size_t index1) const
{
  BookCRIter  iter, pos0, pos1;
  size_t      last;
  size_t      index;

  if( m_uFlags & FlagDebug )
  {
    os << "DBG: " << __func__ << "(" << index0 << ", " << index1 << ")" << endl;
  }

  if( size() == 0 )
  {
    return;
  }
  else if( (index0 > index1) || (index1 >= size()) )
  {
    return;
  }

  last = size() - 1;

  index  = index1;

  pos0  = m_book.rbegin() + last  - index1;
  pos1  = m_book.rbegin() + last  - index0;

  for(iter = pos0; iter != m_book.rend() && iter <= pos1; ++iter)
  {
    if( m_uFlags & FlagONum )
    {
      os << setw(3) << index << ". ";
    }

    if( m_uFlags & FlagOTime )
    {
      os << "[" << iter->m_timestamp << "] ";
    }

    os << iter->m_strText << endl;

    if( (m_uFlags & FlagOMark) && !iter->m_strMark.empty() )
    {
      os << "        ~~ " << iter->m_strMark << endl;
    }

    --index;
  }
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// Support
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

void LogBook::limitSize()
{
  if( m_book.size() > m_uMaxEntries )
  {
    eraseEntries(m_book.size() - m_uMaxEntries, OLDEST);
  }
}

void LogBook::ssflush()
{
  m_ssText.str(string());
  m_ssText.clear();
  m_bWarnThrottle = false;
}

bool LogBook::checkLen(size_t uEstLen)
{
  if( m_ssText.str().size()+uEstLen < m_uMaxEntryLen )
  {
    return true;
  }
  else
  {
    if( !m_bWarnThrottle )
    {
      LOGWARN("Pending entry exceeds max entry length %zu.", m_uMaxEntryLen);
      m_bWarnThrottle = true;
    }
    return false;
  }
}

LogBook::BookIter LogBook::findMark(const string &strMark)
{
  BookIter  iter;

  for(iter = m_book.begin(); iter != m_book.end(); ++iter)
  {
    if( iter->m_strMark == strMark )
    {
      return iter;
    }
  }

  return m_book.end();
}

LogBook::BookCIter LogBook::findMark(const string &strMark) const
{
  BookCIter iter;

  for(iter = m_book.begin(); iter != m_book.end(); ++iter)
  {
    if( iter->m_strMark == strMark )
    {
      return iter;
    }
  }

  return m_book.end();
}

void LogBook::sortMarks(BookMarkList &sorted) const
{
  BookCIter iter;
  size_t    index;
  BookMark  bookmark;

  sorted.clear();

  index = 0;

  for(iter = m_book.begin(); iter != m_book.end(); ++iter)
  {
    if( !iter->m_strMark.empty() )
    {
      bookmark.m_strMark  = iter->m_strMark;
      bookmark.m_index    = index;

      if( m_uFlags & FlagDebug )
      {
        cerr << "DBG: mark=\"" << bookmark.m_strMark
          << "\", index=" << bookmark.m_index << endl;
      }

      sorted.push_back(bookmark);
    }

    ++index;
  }
}

void LogBook::rsortMarks(BookMarkList &sorted) const
{
  BookCRIter  iter;
  size_t      index;
  BookMark    bookmark;

  sorted.clear();

  index = size() - 1;

  for(iter = m_book.rbegin(); iter != m_book.rend(); ++iter)
  {
    if( !iter->m_strMark.empty() )
    {
      bookmark.m_strMark  = iter->m_strMark;
      bookmark.m_index    = index;

      if( m_uFlags & FlagDebug )
      {
        cerr << "DBG: mark=\"" << bookmark.m_strMark
          << "\", index=" << bookmark.m_index << endl;
      }

      sorted.push_back(bookmark);
    }

    --index;
  }
}

void LogBook::copy(const LogBook &src)
{
  m_strName       = src.m_strName;
  m_uMaxEntries   = src.m_uMaxEntries;
  m_uMaxEntryLen  = src.m_uMaxEntryLen;
  m_uTotalEver    = src.m_uTotalEver;
  m_uFlags        = src.m_uFlags;
  m_book          = src.m_book;
}


// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
// LogBook Stream Manipulators
// . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

LogBook &rnr::eoe(LogBook &log)
{
  log.logPending();
  return log;
}

LogBook &rnr::bookmark(LogBook &log)
{
  // generate unique name
  log.setPendingBookMark(log.makeBookMarkLabel());
  return log;
}

rnr::lbmanip_bm_ rnr::bookmark(const std::string str)
{
  lbmanip_bm_ o;
  o.m_strMark = str;
  return o;
}

LogBook &rnr::operator<<(LogBook &log, rnr::lbmanip_bm_ f)
{
  log.setPendingBookMark(f.m_strMark);
  return log;
}

rnr::lbmanip_fg_ rnr::setflags(const unsigned flags)
{
  lbmanip_fg_ o;
  o.m_uFlags = flags;
  return o;
}

LogBook &rnr::operator<<(LogBook &log, rnr::lbmanip_fg_ f)
{
  log.setFlags(f.m_uFlags);
  return log;
}
