////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_appkit
//
// File:      CmdArgDef.h
//
/*! \file
 *
 * \brief Command line argument definition class interface.
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

#ifndef _RNR_CMD_ARG_DEF_H
#define _RNR_CMD_ARG_DEF_H

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

#include <iostream>
#include <string>
#include <vector>

#include "rnr/appkit/StringTheory.h"
#include "rnr/appkit/RegEx.h"
#include "rnr/appkit/CmdCore.h"
#include "rnr/appkit/CmdExtArg.h"

/*!
 * \brief RoadNarrows Robotics
 */
namespace rnr
{
  namespace cmd
  {
    //--------------------------------------------------------------------------
    // CmdArgDef Class
    //--------------------------------------------------------------------------
  
    /*!
     * \brief Command argument compiled definition class.
     */
    class CmdArgDef
    {
    public:
      /*!
       * \brief Argument type enumeration.
       */
      enum ArgType
      {
        ArgTypeUndef   = 0, ///< undefined argument type
        ArgTypeLiteral,     ///< literal constant
        ArgTypeWord,        ///< any non-whitespace contiguous char sequence
        ArgTypeMultiWord,   ///< any (quoted) character sequence
        ArgTypeIdentifier,  ///< identifier (C conforming)
        ArgTypeBoolean,     ///< boolean (bool)
        ArgTypeInteger,     ///< integer (long)
        ArgTypeFpn,         ///< floating point number (double)
        ArgTypeFile,        ///< file path
        ArgTypeRegEx        ///< regular expression
      };
  
      /*!
       * \brief Argument flag modifiers.
       */
      enum ArgFlags
      {
        FlagCommand   = 0x0001, ///< argument is the command
        FlagOptional  = 0x0002, ///< argument is optional
        FlagXorList   = 0x0004, ///< argument has a mutually exclusive list
        FlagRepeat    = 0x0008  ///< argument supports repetition
      };

      /*!
       * \brief Number minimum and maximum (sub)range.
       */
      struct range
      {
        double min;   ///< minimum value
        double max;   ///< maximum value
      };

      typedef std::vector<range>  RangeVec; ///< vector of subranges

      /*!
       * \brief Default constructor.
       */
      CmdArgDef();
  
      /*!
       * \brief Copy constructor.
       *
       * \param src Source object.
       */
      CmdArgDef(const CmdArgDef &src);

      /*!
       * \brief Destructor.
       */
      virtual ~CmdArgDef();
  
      /*!
       * \brief Assignment operator.
       *
       * \param rhs   Right-hand side object.
       *
       * \return *this
       */
      CmdArgDef &operator=(const CmdArgDef &rhs);
  

      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Public Attribute Methods
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

      /*!
       * \brief Test if argument definition is sufficiently defined.
       *
       * \return Returns true or false.
       */
      bool isDefined() const;

      /*!
       * \brief Get parent command's unique id.
       *
       * \return Unique id.
       */
      int getParentCmdUid() const
      {
        return m_nCmdUid;
      }

      /*!
       * \brief Get parent command form's index.
       *
       * \return Zero based index.
       */
      int getParentFormIndex() const
      {
        return m_nFormIndex;
      }

      /*!
       * \brief Get argument's command line index.
       *
       * \return Zero based index..
       */
      int getIndex() const
      {
        return m_nIndex;
      }

      /*!
       * \brief Get argument's name.
       *
       * \return Name string.
       */
      const std::string &getName() const
      {
        return m_strName;
      }

      /*!
       * \brief Get argument's type.
       *
       * \return Type enum.
       */
      ArgType getType() const
      {
        return m_eType;
      }

      /*!
       * \brief Get the number of mutually exclusive literals.
       *
       * \return Number.
       */
      int numOfLiterals() const
      {
        return (int)m_literals.size();
      }

      /*!
       * \brief Get literal value at index.
       *
       * \param nIndex  Index of literal in list.
       *
       * \return String value.
       */
      const std::string &literalAt(const int nIndex) const;

      /*!
       * \brief Get numeric range values.
       *
       * \return Vector of ranges.
       */
      const RangeVec &getRanges() const
      {
        return m_ranges;
      }

      /*!
       * \brief Get regular expression value.
       *
       * \return String value.
       */
      const std::string &getRegEx() const
      {
        return m_re.getRegEx();
      }

      /*!
       * \{
       *
       * \brief Check if value is in the specified range.
       *
       * \param value   Value to check.
       *
       * \return Returns true or false.
       */
      bool inRange(const long value) const;

      bool inRange(const double value) const;
      /*!
       * \}
       */

      /*!
       * \brief Get argument's modifier flags.
       *
       * \return Bit-or'ed flags.
       */
      unsigned getFlags() const
      {
        return m_uFlags;
      }

      /*!
       * \brief Test if argument is the command argument (argv0).
       *
       * \return Returns true or false.
       */
      bool isCommand() const
      {
        return (m_uFlags & FlagCommand) != 0;
      }

      /*!
       * \brief Test if argument is an optional argument.
       *
       * \return Returns true or false.
       */
      bool isOptional() const
      {
        return (m_uFlags & FlagOptional) != 0;
      }

      /*!
       * \brief Construct literal list string.
       *
       * \param sep Seperator string between literals.
       *
       * \return Literal list string.
       */
      std::string constructLiteralList(const std::string sep = " ") const;

      /*!
       * \brief Construct ranges string.
       *
       * \param sep Seperator string between ranges.
       *
       * \return Literal list string.
       */
      std::string constructRangeList(const std::string sep = ",") const;

      /*!
       * \brief Construct syntax equivalent string from argument data.
       *
       * \return Syntax string.
       */
      std::string constructSyntax() const;


      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Pattern Matching Methods
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

      /*!
       * \brief Match argument string against argument definition pattern.
       *
       * The match weight returned is a heuristic measure of match strength.
       * A value of 0.0 is no match, while anything \> 0.0 is a match, with 1.0
       * being the best possible fit. With a matched weight assigned to
       * each command argument, the best matched command applied to the input
       * can be heuristically determined.
       *
       * \par For Example:
       * Given two arguments whose syntax are specified as:
       * \verbatim
       * girl
       * <teen:word>
       * \endverbatim
       *
       * Then the weights returned from the following match calls are:
       * \verbatim
       * arg girl: match("girl") --> 1.00
       * arg teen: match("girl") --> 0.91
       * arg girl: match("boy")  --> 0.00
       * arg teen: match("boy")  --> 0.91
       * \endverbatim
       *
       * \param strArg      Argument value.
       * \param bIgnoreCase Do [not] ignore case when applying pattern matching.
       *
       * \return Returns match weight [0.0 - 1.0].
       */
      double match(const std::string &strArg,
                   const bool        bIgnoreCase = false) const;

      /*!
       * \brief Match argument agains literal enumeration list.
       *
       * \param strArg      Argument value.
       * \param bIgnoreCase Do [not] ignore case when applying pattern matching.
       *
       * \return
       * On success, returns literal list index \>= 0. Otherwise -1 is returned.
       */
      int matchLiteral(const std::string &strArg,
                       const bool        bIgnoreCase = false) const;

      /*!
       * \brief Convert argument string to type.
       *
       * \param strArg      Argument source string.
       * \param bIgnoreCase Do [not] ignore case when applying pattern matching.
       *
       * \return Converted argument object.
       */
      CmdExtArg convert(const std::string &strArg,
                        const bool        bIgnoreCase = false) const;

      /*!
       * \brief Look up argument type, given argument type symbol.
       *
       * \param strSymbol Type Symbol.
       *
       * \return Type enum.
       */
      static CmdArgDef::ArgType lookupArgType(const std::string strSymbol);

      /*!
       * \brief Look up argument symbol, given argument type.
       *
       * \param eType Type enum.
       *
       * \return Type symbol string.
       */
      static const std::string lookupArgSymbol(const CmdArgDef::ArgType eType);

      /*!
       * \brief Look up argument flags, given flags value.
       *
       * \param uFlags  Bit or'ed list of flags.
       *
       * \return String.
       */
      static const std::string lookupFlagNames(const unsigned uFlags);


      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Output Methods and Operators
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

      /*!
       * \brief Insert object into output stream.
       *
       * \param os      Output stream.
       * \param argdef  Object to insert.
       *
       * \return Reference to output stream.
       */
      friend std::ostream &operator<<(std::ostream    &os,
                                      const CmdArgDef &argdef);
  
      //
      // Friends
      //
      friend class CmdFormDef;
      friend class CmdDef;
      friend class CommandLine;

    protected:
      int             m_nCmdUid;    ///< parent command's unique id
      int             m_nFormIndex; ///< parent command's form index
      int             m_nIndex;     ///< argument index
      std::string     m_strName;    ///< argument name
      ArgType         m_eType;      ///< argument type
      str::StringVec  m_literals;   ///< literal argument valid values
      RangeVec        m_ranges;     ///< numeric argument valid ranges
      RegEx           m_re;         ///< reg expression argument valid pattern
      unsigned        m_uFlags;     ///< argument modifiers

      /*!
       * \brief Set parent's command and form id's.
       *
       * \param nCmdUid     Command unique id.
       * \param nFormIndex  Zero based form index within command.
       */
      void setParent(const int nCmdUid, const int nFormIndex);

      /*!
       * \brief Set arguments's command line index.
       *
       * \param nIndex  Zero based index.
       */
      void setIndex(const int nIndex);

      /*!
       * \brief Set argument's name.
       *
       * \param strName Name string.
       */
      void setName(const std::string &strName);

      /*!
       * \brief Set argument's type.
       *
       * \param eType Type enum.
       */
      void setType(const ArgType eType);

      /*!
       * \brief Add literal value to list of argument values.
       *
       * \param strValue    Value to add.
       */
      void addLiteralValue(const std::string &strValue);

      /*!
       * \brief Set numeric range values.
       *
       * \param ranges  Vector of ranges.
       */
      void setRanges(const RangeVec &ranges);

      /*!
       * \brief Set regular expression value.
       *
       * \param re  Regular expression.
       */
      void setRegEx(const RegEx &re);

      /*!
       * \brief Or flags into argment modifier flags.
       *
       * \param uFlags  Bit list of flags.
       */
      void orFlags(const unsigned uFlags);
    }; // class CmdArgDef
  
    //
    // Types and Data
    //
    typedef std::vector<CmdArgDef> ArgDefVec;  ///< vector of argument defs
    
  } // namespace cmd
} // namespace rnr

#endif // _RNR_CMD_ARG_DEF_H
