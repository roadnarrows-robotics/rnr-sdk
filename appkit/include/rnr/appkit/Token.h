////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_appkit
//
// File:      Token.h
//
/*! \file
 *
 * \brief Simple, token container class interface.
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

#ifndef _RNR_TOKEN_H
#define _RNR_TOKEN_H

#include <stdlib.h>

#include <iostream>
#include <string>
#include <vector>

#include "rnr/appkit/LogBook.h"

/*!
 * \brief RoadNarrows Robotics
 */
namespace rnr
{
  namespace cmd
  {
    //--------------------------------------------------------------------------
    // Token Class
    //--------------------------------------------------------------------------
  
    /*!
     * \brief Parsed token container class.
     */
    class Token
    {
    public:
      std::string m_strValue;   ///< token string value
      size_t      m_lineNum;    ///< line number
      size_t      m_posStart;   ///< line start position of token
      size_t      m_posEnd;     ///< line end position of token

      /*!
       * \brief Default constructor.
       */
      Token();
      
      /*!
       * \brief Initialization constructor.
       *
       * \param strValue  Token value.
       */
      Token(const std::string &strValue);

      /*!
       * \brief Initialization constructor.
       *
       * \param strValue  Token value.
       * \param lineNum   Line number. Set to 0 if unknown or not associated
       *                  with I/O input.
       * \param posStart  Token start position in line.
       * \param posEnd    Token end position in line.
       */
      Token(const std::string &strValue,
            const size_t      lineNum,
            const size_t      posStart,
            const size_t      posEnd);

      /*!
       * \brief Copy constructor.
       *
       * \param src Source object.
       */
      Token(const Token &src);

      /*!
       * \brief Destructor.
       */
      virtual ~Token();

      /*!
       * \brief Assignment operator.
       *
       * \param rhs   Right-hand side object.
       *
       * \return *this
       */
      Token &operator=(const Token &rhs);
  

      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Attribute Methods
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  
      /*!
       * \brief Return token string.
       *
       * \return String.
       */
      const std::string &value() const
      {
        return m_strValue;
      }

      /*!
       * \brief Return input line number where token was located.
       *
       * \return Line number.
       */
      const size_t linenum() const
      {
        return m_lineNum;
      }

      /*!
       * \brief Return input line position where token was located.
       *
       * \param [out] posStart  Starting character position of token.
       * \param [out] posEnd    Ending character position of token.
       */
      void position(size_t &posStart, size_t &posEnd) const
      {
        posStart = m_posStart;
        posEnd   = m_posEnd;
      }


      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Output Methods and Operators
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

      /*!
       * \brief Output annotated token embedded location in the input line.
       *
       * \par Output:
       * [linenum] line of text where <b>token</b> was generated
       *
       * \param os        Output stream.
       * \param strLine   Line whence token was generated.
       * \param bLoc      If true, then include line number and token characater
       *                  positions.
       *
       * \return Reference to output stream.
       */
      std::ostream &printAnnotated(std::ostream      &os,
                                   const std::string &strLine,
                                   const bool        bLoc = false);

      /*!
       * \brief Insert object into output stream.
       *
       * \param os  Output stream.
       * \param tok Object to insert.
       *
       * \return Reference to output stream.
       */
      friend std::ostream &operator<<(std::ostream &os, const Token &tok);

      /*!
       * \brief Insert object into LogBook pending entry.
       *
       * \param log LogBook stream.
       * \param tok Object to insert.
       *
       * \return Reference to LogBook.
       */
      friend LogBook &operator<<(LogBook &log, const Token &tok);
    }; // class Token

    //
    // Types
    //
    typedef std::vector<Token> TokenVec;  ///< vector of tokens type

  } // namespace cmd
} // namespace rnr

#endif // _RNR_TOKEN_H
