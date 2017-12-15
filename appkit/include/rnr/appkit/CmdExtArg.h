////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_appkit
//
// File:      ExtArg.h
//
/*! \file
 *
 * \brief Command line extended argument interface.
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

#ifndef _RNR_CMD_EXT_ARG_H
#define _RNR_CMD_EXT_ARG_H

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

#include <iostream>
#include <string>
#include <vector>

#include "rnr/appkit/LogBook.h"

/*!
 * \brief RoadNarrows Robotics
 */
namespace rnr
{ 
  /*!
   * \brief Commands
   */
  namespace cmd
  {

    //--------------------------------------------------------------------------
    // CmdExtArg Class
    //--------------------------------------------------------------------------

    /*!
     * \brief Command EXTended ARGument class holding parsed command context and
     * the raw and converted argmument values.
     *
     * Extended arguments are an alternative command interface to the standard
     * simple command string arguments.
     */
    class CmdExtArg
    {
    public:
      /*!
       * \brief Converted types.
       */
      enum CvtType
      {
        CvtTypeUndef,       ///< unknown or uninitialized type
        CvtTypeString,      ///< string type
        CvtTypeEnum,        ///< index into literal enumeration list
        CvtTypeBoolean,     ///< boolean type
        CvtTypeInteger,     ///< integer type
        CvtTypeFpn          ///< floating-point number type
      };

      /*!
       * \brief Default constructor.
       */
      CmdExtArg();

      /*!
       * \brief Initialization constructor.
       *
       * \param nCmdUid       Command definition unique id.
       * \param nFormIndex    Form definition index.
       * \param nArgIndex     Argument definition index.
       * \param nArgInstance  Argument instance (FUTURE).
       * \param strArg        Source argument string.
       */
      CmdExtArg(const int         &nCmdUid,
                const int         &nFormIndex,
                const int         &nArgIndex,
                const int         &nArgInstance,
                const std::string &strArg);

      /*!
       * \brief Copy constructor.
       *
       * \param src Source object.
       */
      CmdExtArg(const CmdExtArg &src);

      /*!
       * \brief Destructor.
       */
      virtual ~CmdExtArg();

      /*!
       * \brief Assignment operator.
       *
       * \param rhs   Right-hand side object.
       *
       * \return *this
       */
      CmdExtArg &operator=(const CmdExtArg &rhs);


      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Parsed Command Context Methods
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

      /*!
       * \brief Get the argument's associated parsed command unique id.
       *
       * \return Unique id.
       */
      int uid() const { return m_nCmdUid; }

      /*!
       * \brief Get the argument's associated parsed form definition index.
       *
       * \return Form index.
       */
      int formIndex() const { return m_nFormIndex; }

      /*!
       * \brief Get the argument's associated parsed argument definition index.
       *
       * \return Argument index.
       */
      int argIndex() const { return m_nArgIndex; }

      /*!
       * \brief Get the argument's instance number of (uid, formindex, argindex)
       * parsed argument definition index.
       *
       * FUTURE
       *
       * \return Argument index.
       */
      int argInstance() const { return m_nArgInstance; }


      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Argument Access Methods
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

      /*!
       * \brief Get raw source argument string (or a pirate grunting).
       *
       * \return String.
       */
      const std::string &arg() const { return m_strArg; }

      /*!
       * \brief Check if converted value is valid.
       *
       * \return Returns true or false.
       */
      bool isValid() const { return m_eCvtType != CvtTypeUndef; }

      /*!
       * \brief Get the converted argument type.
       *
       * \return CmdExtArg::CvtType value.
       */
      CvtType type() const { return m_eCvtType; }

      /*!
       * \brief Get the converted string value.
       *
       * \return Returns string.
       */
      const std::string &s() const { return m_strCvtVal; }

      /*!
       * \brief Get the converted enumeration index value.
       *
       * \return Returns index.
       */
      long e() const { return m_lCvtVal; }

      /*!
       * \brief Get the converted boolean value.
       *
       * \return Returns bool.
       */
      bool b() const { return m_bCvtVal; }

      /*!
       * \brief Get the converted integer value.
       *
       * \return Returns long.
       */
      long i() const { return m_lCvtVal; }

      /*!
       * \brief Get the converted floating-point number value.
       *
       * \return Returns double.
       */
      double f() const { return m_fCvtVal; }


      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Comparison Operators
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

      /*!
       * \{
       *
       * \brief Comparison operator this == rval.
       *
       * \param rval   Rvalue object.
       *
       * \return Returns true if equal, false otherwise.
       */
      bool operator==(const CmdExtArg &rval) const;

      bool operator==(const std::string &rval) const;

      bool operator==(const char* const &rval) const;

      bool operator==(const bool &rval) const;

      bool operator==(const long &rval) const;

      bool operator==(const double &rval) const;
      /*!
       * \}
       */

      /*!
       * \{
       *
       * \brief Comparison operator this != rval.
       *
       * \param rval   Rvalue object.
       *
       * \return Returns true if not equal, false otherwise.
       */
      bool operator!=(const CmdExtArg &rval) const { return !(*this == rval); }

      bool operator!=(const std::string &rval) const {return !(*this == rval);}

      bool operator!=(const char* const &rval) const {return !(*this == rval);}

      bool operator!=(const bool &rval) const { return !(*this == rval); }

      bool operator!=(const long &rval) const { return !(*this == rval); }

      bool operator!=(const double &rval) const { return !(*this == rval); }
      /*!
       * \}
       */


      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Output Methods and Operators
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

      /*!
       * \brief Insert object into output stream.
       *
       * \param os  Output stream.
       * \param arg Object to insert.
       *
       * \return Reference to output stream.
       */
      friend std::ostream &operator<<(std::ostream &os, const CmdExtArg &arg);

      /*!
       * \brief Insert object into LogBook pending entry.
       *
       * \param log LogBook stream.
       * \param arg Object to insert.
       *
       * \return Reference to LogBook.
       */
      friend LogBook &operator<<(LogBook &log, const CmdExtArg &arg);

      //
      // Friends
      //
      friend class CmdArgDef;

    protected:
      // command context 
      int         m_nCmdUid;      ///< command definition unique id
      int         m_nFormIndex;   ///< form definition index
      int         m_nArgIndex;    ///< argument definition index
      int         m_nArgInstance; ///< argument instance (FUTURE)

      // raw value
      std::string m_strArg;       ///< argument unconverted raw value string

      // converted type and value
      CvtType     m_eCvtType;     ///< converted type
      std::string m_strCvtVal;    ///< converted string value
      bool        m_bCvtVal;      ///< converted boolean value
      long        m_lCvtVal;      ///< converted integer/index value
      double      m_fCvtVal;      ///< converted float-point number value

      /*!
       * \brief Set the argument string value.
       *
       * \param strArg  Source argument string.
       */
      void arg(const std::string &strArg)
      {
        m_strArg = strArg;
      }

      /*!
       * \brief Set the converted string value.
       *
       * \param strVal    Value to set.
       */
      void s(const std::string &strVal);

      /*!
       * \brief Set the converted index into literal enum list value.
       *
       * \note A literal has both a string and integer enumeration. So any
       * relevant operators and methods supports both interface.
       *
       * \param eVal    Value to set.
       */
      void e(const long eVal);

      /*!
       * \brief Set the converted boolean value.
       *
       * \param bVal    Value to set.
       */
      void b(const bool bVal);

      /*!
       * \brief Set the converted integer value.
       *
       * \param lVal    Value to set.
       */
      void i(const long lVal);

      /*!
       * \brief Set the converted floating-point number value.
       *
       * \param fVal    Value to set.
       */
      void f(const double fVal);
    }; // class CmdExtArg

    //
    // Types
    //
    typedef std::vector<CmdExtArg>  CmdExtArgVec; ///< vector of ext args type

  } // namespace cmd
} // namespace rnr

#endif // _RNR_CMD_EXT_ARG_H
