////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_appkit
//
// File:      CmdFormDef.h
//
/*! \file
 *
 * \brief Command line command form definition class interface.
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

#ifndef _RNR_CMD_FORM_DEF_H
#define _RNR_CMD_FORM_DEF_H

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include <iostream>
#include <string>
#include <vector>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/appkit/IOManip.h"
#include "rnr/appkit/CmdCore.h"
#include "rnr/appkit/CmdArgDef.h"

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
    // forward declarations in namespace rnr::cmd
    class CmdFormDef;
    std::ostream &operator<<(std::ostream &os, const CmdFormDef &formdef);

    //--------------------------------------------------------------------------
    // CmdFormDef Class
    //--------------------------------------------------------------------------

    /*!
     * \brief Compiled command form defintion class.
     */
    class CmdFormDef
    {
    public:
      /*!
       * \brief Default constructor.
       */
      CmdFormDef();
  
      /*!
       * \brief Initialization constructor.
       */
      CmdFormDef(const std::string &strSyntax);
  
      /*!
       * \brief Copy constructor.
       *
       * \param src Source object.
       */
      CmdFormDef(const CmdFormDef &src);

      /*!
       * \brief Destructor.
       */
      virtual ~CmdFormDef();
  
      /*!
       * \brief Assignment operator.
       *
       * \param rhs   Right-hand side object.
       *
       * \return *this
       */
      CmdFormDef &operator=(const CmdFormDef &rhs);
  

      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Public Attribute Methods
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

      /*
       * \brief Test if form definition is sufficiently defined.
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
       * \brief Get forms's index.
       *
       * \return Zero based index.
       */
      int getIndex() const
      {
        return m_nIndex;
      }

      /*!
       * \brief Get form's extended usage syntax.
       */
      std::string getSyntax() const
      {
        return m_strSyntax;
      }

      /*!
       * \brief Get form's argument at index.
       *
       * \param nIndex    Argument index.
       *
       * \return Command argument definition reference.
       */
      const CmdArgDef &at(const int nIndex) const;

      /*!
       * \brief Index operator.
       *
       * Get form's argument at index.
       *
       * \param nIndex    Argument index.
       *
       * \return Command argument definition reference.
       */
      const CmdArgDef &operator[](const int nIndex) const
      {
        return at(nIndex);
      }

      /*!
       * \brief Get the total number of arguments.
       *
       * The first argument (argv0) is the command.
       *
       * \return Number of arguments.
       */
      int numOfArgs() const
      {
        return (int)m_argDefs.size();
      }

      /*!
       * \brief Get the total number of required arguments.
       *
       * The required number includes the command argv0.
       *
       * Required arguments start at argument index 0.
       *
       * \return Number of arguments.
       */
      int numOfRequiredArgs() const
      {
        return m_nArgcReq;
      }

      /*!
       * \brief Get the total number of optional arguments.
       *
       * Optional arguments start at argument index numOfRequiredArgs().
       *
       * \return Number of arguments.
       */
      int numOfOptionalArgs() const
      {
        return m_nArgcOpt;
      }
  

      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Output Methods and Operators
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

      /*!
       * \brief Insert object into output stream.
       *
       * \param os      Output stream.
       * \param formdef Object to insert.
       *
       * \return Reference to output stream.
       */
      friend std::ostream &rnr::cmd::operator<<(std::ostream     &os,
                                                const CmdFormDef &formdef);

      //
      // Friends
      //
      friend class CmdDef;
      friend class CommandLine;

    protected:
      int         m_nCmdUid;    ///< parent command's unique id
      int         m_nIndex;     ///< forms index
      std::string m_strSyntax;  ///< command extened usage syntax
      ArgDefVec   m_argDefs;    ///< command argument definitions
      int         m_nArgcReq;   ///< number of required arguments
      int         m_nArgcOpt;   ///< number of optional arguments

      /*!
       * \brief Reset form definition to pre-compiled state.
       */
      void reset();

      /*!
       * \brief Set parent's command id.
       *
       * \param nCmdUid     Command unique id.
       */
      void setParent(const int nCmdUid);

      /*!
       * \brief Set forms's index.
       *
       * \param nIndex  Zero based index.
       */
      void setIndex(const int nIndex);

      /*!
       * \brief Set command's extended usage syntax.
       *
       * \param strSyntax   Syntax string.
       */
      void setSyntax(const std::string &strSyntax);

      /*!
       * \brief Push argument to end of argument list.
       *
       * \param argdef Command argument definition.
       */
      void pushArg(CmdArgDef &argdef);

      /*!
       * \brief Get form's modifiable argument at index.
       *
       * Protected version of at().
       *
       * \param nIndex    Argument index.
       *
       * \return Command argument definition reference.
       */
      CmdArgDef &argAt(const int nIndex);

      /*!
       * \brief Get the last pushed argument.
       *
       * \return Argument definition reference.
       */
      CmdArgDef &lastArg();
    }; // class CmdFormDef

    //
    // Types and Data
    //
    typedef std::vector<CmdFormDef> CmdFormDefVec;  ///< vector of command forms
    
  } // namespace cmd
} // namespace rnr

#endif // _RNR_CMD_FORM_DEF_H
