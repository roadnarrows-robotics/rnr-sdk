////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_appkit
//
// File:      StringTheory.h
//
/*! \file
 *
 * \brief Of string spaces and their strangian operators.
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

#ifndef _RNR_STRING_THEORY_H
#define _RNR_STRING_THEORY_H

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <ctype.h>

#include <sstream>
#include <string>
#include <vector>
#include <algorithm> 
#include <functional> 

/*!
 * \brief RoadNarrows Robotics
 */
namespace rnr
{
  /*!
   * \brief String
   */
  namespace str
  {
    /*!
     * \brief Useful types.
     */
    typedef std::vector<std::string>  StringVec;  ///< vector of strings type

    /*!
     * \brief Falsehood and truthhood strings. Each list termintate with a NULL.
     */
    extern const char *FalseHood[]; ///< strings that equate to false
    extern const char *TruthHood[]; ///< strings that equate to true

    /*!
     * \brief Convert boolean to "ok" or "not-ok".
     *
     * \param b   Boolean value.
     *
     * \return String.
     */
    inline std::string okstr(bool b)
    {
      return b? "ok": "not-ok";
    }

    /*!
     * \brief Create space string.
     *
     * \param n Number of spaces.
     *
     * \return String.
     */
    inline std::string space(unsigned n)
    {
      return std::string(n, ' ');
    }

    /*!
     * \brief Split string.
     *
     * \param [in]     str    String to split.
     * \param [in]     delim  Character delimiter.
     * \param [in,out] elems  Vector of split strings.
     *
     * \return Returns number of element strings appended to elems.
     */
    size_t split(const std::string &str, const char delim, StringVec &elems);

    /*!
     * \brief Split string.
     *
     * \param [in] str        String to split.
     * \param [in] delim      Character delimiter.
     *
     * \return Vector of split strings.
     */
    StringVec split(const std::string &str, const char delim);

    /*!
     * \brief Trim string in-place of leading whitespace.
     *
     * \param [in,out] str  String to trim.
     *
     * \return Left trimmed string.
     */
    inline std::string &ltrim(std::string &str)
    {
      str.erase( str.begin(), std::find_if(str.begin(), str.end(),
                std::not1(std::ptr_fun<int, int>(std::isspace))) );
      return str;
    }
    
    /*!
     * \brief Trim copy of string of leading whitespace.
     *
     * \param [in] s  Null-terminated character string. Cannot be NULL.
     *
     * \return Left trimmed string.
     */
    inline std::string ltrim(const char *s)
    {
      std::string str(s);
      return ltrim(str);
    }

    /*!
     * \brief Trim string in-place of trailing whitespace.
     *
     * \param [in,out] str  String to trim.
     *
     * \return Right trimmed string.
     */
    inline std::string &rtrim(std::string &str)
    {
      str.erase( std::find_if(str.rbegin(), str.rend(),
          std::not1(std::ptr_fun<int, int>(std::isspace))).base(), str.end() );
      return str;
    }
    
    /*!
     * \brief Trim copy of string of trailing whitespace.
     *
     * \param [in] s  Null-terminated character string. Cannot be NULL.
     *
     * \return Right trimmed string.
     */
    inline std::string rtrim(const char *s)
    {
      std::string str(s);
      return rtrim(str);
    }

    /*!
     * \brief Trim string in-place of leading and trailing whitespace.
     *
     * \param [in,out] str  String to trim.
     *
     * \return Trimmed string.
     */
    inline std::string &trim(std::string &str)
    {
      return ltrim(rtrim(str));
    }

    /*!
     * \brief Trim copy of string of leading and trailing whitespace.
     *
     * \param [in] s  Null-terminated character string. Cannot be NULL.
     *
     * \return Trimmed string.
     */
    inline std::string trim(const char *s)
    {
      std::string str(s);
      return trim(str);
    }

    /*!
     * \brief In-place replace all whats in string with with.
     *
     * \todo Investigate using c++ function approach.
     *
     * \param           what  What substrings to replace.
     * \param           with  With this substring.
     * \param [in,out]  str   String before and after.
     *
     * \return String with replacements.
     */
    extern std::string &replace(const std::string &what,
                                const std::string &with,
                                std::string       &str);

    /*!
     * \brief Copy replace all whats in string with with.
     *
     * \todo Investigate using c++ function approach.
     *
     * \param       what  What substrings to replace.
     * \param       with  With this substring.
     * \param [in]  s     Null-terminated character string. Cannot be NULL.
     *
     * \return String with replacements.
     */
    inline std::string replace(const std::string &what,
                               const std::string &with,
                               const char        *s)
    {
      std::string str(s);
      return replace(what, with, str);
    }

    /*!
     * \brief Convert in-place string to lower case.
     *
     * \param [in,out] str  String to convert.
     *
     * \return Lower case string.
     */
    std::string lowercase(const std::string &str);

    /*!
     * \brief Convert copy of string to lower case.
     *
     * \param [in] s  Null-terminated character string. Cannot be NULL.
     *
     * \return Lower case string.
     */
    inline std::string lowercase(const char *s)
    {
      std::string str(s);
      return lowercase(str);
    }

    /*!
     * \brief Convert string to upper case.
     *
     * \param [in,out] str  String to convert.
     *
     * \return Upper case string.
     */
    std::string uppercase(const std::string &str);

    /*!
     * \brief Convert copy of string to upper case.
     *
     * \param [in] s  Null-terminated character string. Cannot be NULL.
     *
     * \return Lower case string.
     */
    inline std::string uppercase(const char *s)
    {
      std::string str(s);
      return uppercase(str);
    }

    /*!
     * \brief Find the length of the Greatest Common SubString.
     *
     * Examples:
     * string 1   | string 2    | length
     * :--------  | :--------   | ------:
     * "abcd"     | "x"         | 0
     * "abcd"     | "bc"        | 2
     * "abcd"     | "bcx"       | 2
     * "abcd"     | "bcdef"     | 3
     *
     * \param str1  String 1.
     * \param str2  String 2.
     * \param pos   Starting position in str1.
     *
     * \return Returns length of common substring.
     */
    size_t gcss(const std::string &str1,
                const std::string &str2,
                const size_t      pos = 0);

    /*!
     * \brief Convert string to boolean.
     *
     *  false: "0" "false" "f" "off" "low"   "disable"  "open"
     *  true:  "1" "true"  "t" "on"  "high"  "enable"   "close"
     *
     * \param [in] str    String in hex, decimal, or octal format.
     * \param [out] val   Converted boolean value.
     *
     * \copydoc doc_return_std
     */
     int tobool(const std::string &str, bool &val);

    /*!
     * \brief Convert string to a long integer.
     *
     * \param [in] str    String in hex, decimal, or octal format.
     * \param [out] val   Converted long value.
     *
     * \copydoc doc_return_std
     */
     int tolong(const std::string &str, long &val);

    /*!
     * \brief Convert string to a double-precision floating-point number.
     *
     * \param [in] str    String in hex, decimal, or octal format.
     * \param [out] val   Converted double value.
     *
     * \copydoc doc_return_std
     */
     int todouble(const std::string &str, double &val);

    /*!
     * \brief Prettify string.
     *
     * Binary characters are converted to ascii escape hex sequences. Control
     * characters are converted to standard ascii escape sequences. If required,
     * the string is double quoted ('"') to conform to sh(1) strings.
     *
     * \param str String to make pretty.
     *
     * \return Prettified version of string.
     */
    extern std::string prettify(const std::string &str);

    /*!
     * \brief Simple canonicalization of a string.
     *
     * The string is canonical when:
     *   - leading and trailing whitespace is stripped
     *   - only single space between words
     *
     * \note c14n is an cute abbreviation where 14 represents the number of
     * letters between the 'c' and 'n' in the word "canonicalization".
     *
     * \param str String to canonicalize.
     *
     * \return Return copy of string holding canonical form.
     */
    extern std::string c14n(const std::string &str);
    
    /*!
     * \brief Canonicalization of a list of tokens into a string.
     *
     * \note The name c14n is an cute abbreviation where 14 represents the
     * number of letters between the 'c' and 'n' in the word
     * "canonicalization".
     *
     * \param tokens  String tokens to canonicalize.
     *
     * \return Return string holding canonical form.
     */
    extern std::string c14n(const str::StringVec &tokens);

  } // namespace str
} // namespace rnr

#endif // _RNR_STRING_THEORY_H
