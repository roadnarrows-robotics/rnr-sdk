////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_appkit
//
// File:      Token.cxx
//
/*! \file
 *
 * \brief Simple, token container class implementation.
 *
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2016-2017. RoadNarrows LLC.\n
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

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include <iostream>
#include <sstream>
#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/color.h"

#include "rnr/appkit/StringTheory.h"
#include "rnr/appkit/LogBook.h"
#include "rnr/appkit/Token.h"

using namespace std;
using namespace rnr;
using namespace rnr::str;
using namespace rnr::cmd;

// -----------------------------------------------------------------------------
// Token Class
// -----------------------------------------------------------------------------

Token::Token()
{
  m_lineNum  = 0;
  m_posStart = 0;
  m_posEnd   = 0;
}

Token::Token(const string &strValue)
  : m_strValue(strValue), m_lineNum(0), m_posStart(0), m_posEnd(0)
{
}

Token::Token(const string &strValue,
             const size_t lineNum,
             const size_t posStart,
             const size_t posEnd)
  : m_strValue(strValue),
    m_lineNum(lineNum),
    m_posStart(posStart),
    m_posEnd(posEnd)
{
}

Token::Token(const Token &src)
{
  m_strValue  = src.m_strValue;
  m_lineNum   = src.m_lineNum;
  m_posStart  = src.m_posStart;
  m_posEnd    = src.m_posEnd;
}

Token::~Token()
{
}

Token &Token::operator=(const Token &rhs)
{
  m_strValue  = rhs.m_strValue;
  m_lineNum   = rhs.m_lineNum;
  m_posStart  = rhs.m_posStart;
  m_posEnd    = rhs.m_posEnd;

  return *this;
}

ostream &Token::printAnnotated(ostream      &os,
                               const string &strLine,
                               const bool   bLoc)
{
  string color(ANSI_FG_BRIGHT_RED);
  string reset(ANSI_COLOR_RESET);

  size_t  lw = strLine.length();    // line width
  size_t  pos;                      // working substring starting position
  size_t  len;                      // working substring length

  //
  // Print line number
  //
  if( bLoc )
  {
    os << "[";
    if( m_lineNum > 0 )
    {
      os << m_lineNum << ":";
    }
    os << m_posStart << "," << m_posEnd;
    os << "] ";
  }

  //
  // Print token embedded in line.
  //
  if( m_posStart < lw )
  {
    //
    // Print substring before token in plaintext
    //
    if( m_posStart > 0 )
    {
      os << strLine.substr(0, m_posStart);
    }

    //
    // Print token substring in color.
    //
    len = m_posEnd - m_posStart + 1;
    if( len > lw )
    {
      len = lw - m_posStart;
    }
    os << color << strLine.substr(m_posStart, len) << reset;

    //
    // Print substring after token in plaintext
    //
    pos = m_posEnd + 1;
    if( pos < lw )
    {
      os << strLine.substr(pos);
    }
  }

  //
  // Print line.
  //
  else
  {
    os << strLine << endl;
  }

  return os;
}

ostream &rnr::cmd::operator<<(ostream &os, const Token &tok)
{
  os << "'" << prettify(tok.value()) << "'";
  //os << " @" << tok.m_posStart << ":" << tok.m_posEnd;

  return os;
}

LogBook &rnr::cmd::operator<<(LogBook &log, const Token &tok)
{
  stringstream ss;

  ss  << tok;
  log << ss.str();

  return log;
}
