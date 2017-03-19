////////////////////////////////////////////////////////////////////////////////
//
// Package:   Dynamixel
//
// Program:   dynashell
//
// File:      dynashell_regex.h
//
/*! \file
 *
 * $LastChangedDate: 2015-01-12 10:56:06 -0700 (Mon, 12 Jan 2015) $
 * $Rev: 3845 $
 *
 * \brief The Dynamixel Shell Regular Expression Class.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
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

#ifndef _DYNASHELL_REGEX_H
#define _DYNASHELL_REGEX_H

#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <regex.h>
#include <string>

#include <rnr/rnrconfig.h>

using namespace std;

// ----------------------------------------------------------------------------
// Class RegEx
// ----------------------------------------------------------------------------

/*!
 * \brief Regular Express Class.
 */
class RegEx
{
public:
  /*!
   * \brief Default constructor.
   */
  RegEx()
  {
    m_bIsValid = false;
    memset(&m_regex, 0, sizeof(m_regex));
  }

  /*!
   * \brief String initialization constructor.
   */
  RegEx(const string &strRegEx)
  {
    m_strRegEx = strRegEx;
    Compile();
  }

  /*!
   * \brief Null-terminated string initialization constructor.
   */
  RegEx(const char *sRegEx)
  {
    if( sRegEx != NULL )
    {
      m_strRegEx = sRegEx;
      Compile();
    }
    else
    {
      m_bIsValid = false;
    }
  }

  /*!
   * \brief Copy constructor.
   */
  RegEx(const RegEx &src)
  {
    m_strRegEx = src.m_strRegEx;
    Compile();
  }

  /*!
   * \brief Default destructor.
   */
  virtual ~RegEx()
  {
    regfree(&m_regex);
  }

  /*!
   * \brief Match pattern against regular expression.
   *
   * \param strPat    String pattern.
   *
   * \return Returns true if the pattern matches the expression,
   * false otherwise.
   */
  bool Match(const string &strPat)
  {
    return Match(strPat.c_str());
  }

  /*!
   * \brief Match pattern against regular expression.
   *
   * \param sPat    Null-terminated string pattern.
   *
   * \return Returns true if the pattern matches the expression,
   * false otherwise.
   */
  bool Match(const char *sPat)
  {
    if( m_bIsValid && (regexec(&m_regex, sPat, 0, NULL, 0) == 0) )
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  /*!
   * \brief Get this object's regular expression.
   *
   * \return If a regular expressing exist, then the expression string is 
   * returned. Else NULL is returned.
   */
  const char *GetRegEx() const
  {
    return m_strRegEx.length() > 0? m_strRegEx.c_str(): NULL;
  }

  /*!
   * \brief Test if in valid state.
   *
   * \return Returns true or false;
   */
  bool IsValid()
  {
    return m_bIsValid;
  }

  /*!
   * \brief Assignment copy operator.
   *
   * \param rhs   Regular expression object. 
   *
   * \return This regular expression object.
   */
  RegEx &operator=(const RegEx &rhs)
  {
    regfree(&m_regex);
    m_strRegEx = rhs.m_strRegEx;
    Compile();
    return *this;
  }

  /*!
   * \brief Assignment operator.
   *
   * \param rhs   String regular expression. 
   *
   * \return This regular expression object.
   */
  RegEx &operator=(const string &rhs)
  {
    regfree(&m_regex);
    m_strRegEx = rhs;
    Compile();
    return *this;
  }

  /*!
   * \brief Assignment operator.
   *
   * \param rhs   Null-terminated string expression object. 
   *
   * \return This regular expression object.
   */
  RegEx &operator=(const char *rhs)
  {
    regfree(&m_regex);
    if( rhs != NULL )
    {
      m_strRegEx = rhs;
      Compile();
    }
    else
    {
      m_strRegEx.clear();
      m_bIsValid = false;
    }
    return *this;
  }

protected:
  bool    m_bIsValid;   ///< expression is [not] valid
  string  m_strRegEx;   ///< pre-compiled regular expression string
  regex_t m_regex;      ///< compiled reqular expression 

  void Compile();
};

#endif // _DYNASHELL_REGEX_H
