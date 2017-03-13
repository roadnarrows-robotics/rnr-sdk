////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_appkit
//
// File:      RegEx.h
//
/*! \file
 *
 * \brief The Regular Expression Class interface.
 *
 * RegEx provides a wrapper around the regex C library calls. See REGEX(3) man
 * page.
 *
 * \note Generalized from dynashell_regex.h source found in RoadNarrows Robotics
 * Dynamixel SDK package.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2017  RoadNarrows LLC.
 * \n All Rights Reserved
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

#ifndef _RNR_REGEX_H
#define _RNR_REGEX_H

#include <sys/types.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <regex.h>

#include <iostream>
#include <string>
#include <vector>

#include "rnr/rnrconfig.h"

/*!
 * \brief RoadNarrows Robotics Namespace
 */
namespace rnr
{
  // --------------------------------------------------------------------------
  // Class RegEx
  // --------------------------------------------------------------------------

  /*!
   * \brief Regular Express Class.
   *
   * Regular expression are evaluated using the POSIX Extended Regular
   * Expression syntax.
   */
  class RegEx
  {
  public:
    /*!
     * \brief Special return and error codes.
     *
     * \note The error codes must to be 'out-of-band' from the the standard
     * regex REG_* codes. Large negative values should do the trick.
     */
    static const int ReOk       = REG_NOERROR;  ///< regex operation success
    static const int ReENoExpr  = -1000;        ///< no pre-compiled express
    static const int ReENotComp = -1001;        ///< not compiled

    /*!
     * \brief Regular expression compile and matching behavior flags.
     *
     * See regex(3) for further description of flags.
     *
     * Flags are a bitwise-or of zero or more of these values.
     */ 
    static const int ReFlagDefaults = 0;          ///< default flags

    // supported compile flags
    static const int ReFlagICase   = REG_ICASE;   ///< ignore case when matching
    static const int ReFlagNewLine = REG_NEWLINE; ///< \n force bol/eol matching

    // supported matching flags
    static const int ReFlagNotBoL  = REG_NOTBOL;  ///< input not begin of line
    static const int ReFlagNotEoL  = REG_NOTEOL;  ///< input not end of line

    /*! 
     * \brief Concerning, in reality, in regards to invalid regular expressions.
     */ 
    static const char *ReInvalid;

    /*!
     * \brief Regular expression match structure.
     */
    struct ReMatch
    {
      size_t      m_uStart;   ///< starting index in input where match is found
      size_t      m_uEnd;     ///< ending index in input where match is found
      std::string m_strMatch; ///< matching (sub)string
    };

    /*! default maximum submatches per match */
    static const size_t ReMaxSubMatchesDft = 32;

    typedef std::vector<ReMatch> ReMatchVec; ///< vector of matches

    /*!
     * \brief Default constructor.
     *
     * No regular expression compile will be attempted.
     *
     * \param nFlags  Bitwise-or of compile behavior flags.
     */
    RegEx(int nFlags = ReFlagDefaults);
  
    /*!
     * \brief String initialization constructor.
     *
     * Regular expression compile will be attempted.
     *
     * \param strRegEx  Pre-compile regular expression string.
     * \param nFlags    Bitwise-or of compile behavior flags.
     */
    RegEx(const std::string &strRegEx, int nFlags = ReFlagDefaults);
  
    /*!
     * \brief Null-terminated char* initialization constructor.
     *
     * If not NULL or empty, regular expression compile will be attempted.
     *
     * \param sRegEx  Null-terminated pre-compile regular expression string.
     * \param nFlags  Bitwise-or of compile behavior flags.
     */
    RegEx(const char *sRegEx, int nFlags = ReFlagDefaults);
  
    /*!
     * \brief Copy constructor.
     *
     * Regular expression compile will be attempted.
     *
     * \param src     Source object.
     */
    RegEx(const RegEx &src);
  
    /*!
     * \brief Default destructor.
     */
    virtual ~RegEx();
  
    /*!
     * \brief Assignment copy operator.
     *
     * Regular expression compile will be attempted.
     *
     * \param rhs   Regular expression class object. 
     *
     * \return This regular expression object.
     */
    RegEx &operator=(const RegEx &rhs);
  
    /*!
     * \brief Assignment operator.
     *
     * Regular expression compile will be attempted.
     *
     * \param rhs   String regular expression. 
     *
     * \return This regular expression object.
     */
    RegEx &operator=(const std::string &rhs);
  
    /*!
     * \brief Assignment operator.
     *
     * If not NULL or empty, regular expression compile will be attempted.
     *
     * \param rhs   Null-terminated string expression object. 
     *
     * \return This regular expression object.
     */
    RegEx &operator=(const char *rhs);
  
    /*!
     * \{
     *
     * \brief Match the input string against the regular expression.
     *
     * The entire input must match the regular expression.
     *
     * \param [in] strInput Input string to match.
     *
     * \return
     * Returns true if a match, false otherwise.
     * For the non-constant version, an error is set if no match.
     */
    bool match(const std::string &strInput, const int nFlags = ReFlagDefaults);

    bool match(const std::string &strInput,
               const int nFlags = ReFlagDefaults) const;
    /*!
     * \}
     */
  
    /*!
     * \brief Match the input char* against the regular expression.
     *
     * The entire input must match the regular expression.
     *
     * \param [in] sInput   Null-terminated input char* to match.
     *
     * \return
     * Returns true if a match, false otherwise.
     * For the non-constant version, an error is set if no match.
     */
    bool match(const char *sInput, const int nFlags = ReFlagDefaults);

    bool match(const char *sInput, const int nFlags = ReFlagDefaults) const;
    /*!
     * \}
     */
  
    /*!
     * \brief Find all substrings in the input that match the regular
     * expression.
     *
     * As an example, the following C++ code snippet:
     * ~~~~~~~~~~{.cxx}
     * rnr::RegEx re("[A-Z][a-z ]*[ ]+(cat)[ ]+[a-z ]+[ ]+(dog)[a-z ]*\.");
     *
     * std::string input("My cat is not a dog. But my cat has dog breath.");
     *
     * rnr::RegEx::ReMatchVec matches;
     *
     * re.match(input, matches, 4);
     *
     * for(size_t i = 0; i < matches.size(); ++i)
     * {
     *   std::cout << i << ". (" << matches[i].m_uStart << ","
     *    << matches[i].m_uEnd << ") '"
     *    << matches[i].m_strMatch << "'" << std::endl;
     * }
     * ~~~~~~~~~~
     *
     * Produces the output:
     * ~~~~~~~~~~
     * 0. (0,19) 'My cat is not a dog.'
     * 1. (3,5) 'cat'
     * 2. (16,18) 'dog'
     * 3. (21,46) 'But my cat has dog breath.'
     * 4. (28,30) 'cat'
     * 5. (36,38) 'dog'
     * ~~~~~~~~~~
     *
     * Notes:
     *  - The match start and end values are indices into the original input
     *    string, marking the location of the matched string.
     *  - In the call match(), the maximum number of specified submatches is 4,
     *    but the number of matches found is 6. This is because 2 entire
     *    matches occurred, listed on output 0 and 3. Each occurance matched
     *    3 (sub)strings, which is less than the 4 maximum.
     *
     * \param [in] strInput   Input string.
     * \param [in,out]        Vector of matches.
     * \param uMaxSubMatches  Maximum number of submatches per match.
     * \param nFlags          Bitwise-or of matching behavior flags.
     *
     * \return Returns number of matches.
     * For the non-constant version, an error is set if no match.
     */
    size_t match(const std::string &strInput,
                 ReMatchVec        &matches,
                 const size_t      uMaxSubMatches = ReMaxSubMatchesDft,
                 const int         nFlags = ReFlagDefaults);

    size_t match(const std::string &strInput,
                 ReMatchVec        &matches,
                 const size_t      uMaxSubMatches = ReMaxSubMatchesDft,
                 const int         nFlags = ReFlagDefaults) const;
    /*!
     * \}
     */

    /*!
     * \brief Find all substrings in input that match the regular expression.
     *
     * \sa See above for description.
     *
     * \param [in] sInput     Null-terminated input char*.
     * \param [in,out]        Vector of matches.
     * \param uMaxSubMatches  Maximum number of submatches per match.
     * \param nFlags          Bitwise-or of matching behavior flags.
     *
     * \return Returns number of matches.
     * For the non-constant version, an error is set if no match.
     */
    size_t match(const char   *sInput,
                 ReMatchVec   &matches,
                 const size_t uMaxSubMatches = ReMaxSubMatchesDft,
                 const int    nFlags = ReFlagDefaults);

    size_t match(const char   *sInput,
                 ReMatchVec   &matches,
                 const size_t uMaxSubMatches = ReMaxSubMatchesDft,
                 const int    nFlags = ReFlagDefaults) const;
    /*!
     * \}
     */

    /*!
     * \brief Get the pre-compiled regular expression.
     *
     * \return If a regular expressing exist, then the expression string is 
     * returned. Else NULL is returned.
     */
    const std::string &getRegEx() const
    {
      return m_strRegEx;
    }
  
    /*!
     * \brief Test if in a valid state (i.e. compiled).
     *
     * If valid, match operation may be applied.
     *
     * \return Returns true or false;
     */
    bool isValid() const
    {
      return m_bIsValid;
    }
  
    /*!
     * \brief Get compile behavior flags.
     *
     * \return Bitwise-or of compile behavior flags.
     */
    int getFlags() const
    {
      return m_nFlags;
    }
  
    /*!
     * \brief Set new compile behavior flags.
     *
     * \note May result in a new regular expression re-compile.
     *
     * \param nFlags  Bitwise-or of compile behavior flags.
     */
    void setFlags(int nFlags);

    /*!
     * \brief Get the extened return code from the last RegEx operation.
     *
     * Return Code        | Description
     * ------------------ | -----------
     * ReOk (REG_NOERROR) | No error
     * REG_NOMATCH        | Input does no match regular expression
     * REG_BAD*, REG_E*   | The regex C interface standard compile errors
     * ReENoExpr          | No regular expression extended error
     * ReENotComp         | Not compiled extended error
     *
     * \return nCode Extended return code. 
     */
    int getReturnCode() const
    {
      return m_nReCode;
    }

    /*!
     * \brief Get the last RegExs operation error string.
     *
     * \return Error string.
     */
    const std::string &getErrorStr() const
    {
      return m_strReError;
    }

    //
    // Friends
    //
    friend std::ostream &operator<<(std::ostream &os, const RegEx &re);
    friend std::istream &operator>>(std::istream &is, RegEx &re);

  protected:
    std::string m_strRegEx;   ///< pre-compiled regular expression string
    int         m_nFlags;     ///< compile and matching flags
    bool        m_bIsValid;   ///< expression [not] successfully compiled
    regex_t     m_regex;      ///< compiled reqular expression 
    int         m_nReCode;    ///< compiled regular expression return code
    std::string m_strReError; ///< compiled regualar expresson error string
  
    /*!
     * \brief Compile the regular expression string.
     *
     * \return Returns true success, false otherwise. On failure, check the
     * reason with getReturnCode() and/or getErrorStr().
     */
    bool compile();

    /*!
     * \brief Groom compile behavior flags, disabling any unsupported flags.
     *
     * \param nFlags  Bitwise-or of compile behavior flags.
     */
    void groomFlags(const int nFlags);

    /*!
     * \brief Set the error code and associated error string.
     *
     * \param nCode Extended return code. 
     */
    void setError(const int nCode);

    /*!
     * \brief Free compiled regular expression memory.
     */
    void freeReMem();
  };


  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  // Stream Manipulators
  // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  /*!
   * \brief Insert object into output stream.
   *
   * Output: re"REGEX" string where REGEX is the regular expression pattern.
   *
   * \param os  Output stream.
   * \param re  Object to insert.
   *
   * \return Reference to output stream.
   */
  extern std::ostream &operator<<(std::ostream &os, const RegEx &re);

  /*!
   * \brief Extract from input stream to object.
   *
   * Input: re"REGEX" string where REGEX is the regular expression pattern.
   *
   * \param is  Input stream.
   * \param re  Object to extract into.
   *
   * \return Reference to input stream.
   */
  extern std::istream &operator>>(std::istream &is, RegEx &re);

} // namespace rnr

#endif // _RNR_REGEX_H
