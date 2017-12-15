////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_appkit
//
// File:      CmdDef.h
//
/*! \file
 *
 * \brief Command line command definition class interface.
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

#ifndef _RNR_CMD_DEF_H
#define _RNR_CMD_DEF_H

#include <stdlib.h>

#include <iostream>
#include <string>
#include <vector>

#include "rnr/appkit/StringTheory.h"
#include "rnr/appkit/CmdCore.h"
#include "rnr/appkit/CmdExtArg.h"
#include "rnr/appkit/CmdArgDef.h"
#include "rnr/appkit/CmdFormDef.h"

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
    // CmdDef Class
    //--------------------------------------------------------------------------
  
    /*!
     * \brief Compiled command definition class.
     */
    class CmdDef
    {
    public:
      /*!
       * \brief Default constructor.
       */
      CmdDef();
  
      /*!
       * \brief Copy constructor.
       *
       * \param src Source object.
       */
      CmdDef(const CmdDef &src);

      /*!
       * \brief Destructor.
       */
      virtual ~CmdDef();
  
      /*!
       * \brief Assignment operator.
       *
       * \param rhs   Right-hand side object.
       *
       * \return *this
       */
      CmdDef &operator=(const CmdDef &rhs);
  

      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
      // Public Attribute Methods
      // . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

      /*
       * \brief Test if command definition is sufficiently defined.
       *
       * \return Returns true or false.
       */
      bool isDefined() const;

      /*!
       * \brief Get command's unique id.
       *
       * \return Unique id.
       */
      int getUid() const
      {
        return m_nUid;
      }

      /*!
       * \brief Return command's name.
       *
       * \return Name string.
       */
      const std::string &getName() const
      {
        return m_strName;
      }

      /*!
       * \brief Return command's syntax usage.
       *
       * \return Syntax string.
       */
      const std::string &getSyntax() const
      {
        return m_strSyntax;
      }

      /*!
       * \brief Return command's synopsis.
       *
       * \return Synopsis string.
       */
      const std::string &getSynopsis() const
      {
        return m_strSynopsis;
      }

      /*!
       * \brief Return command's long description.
       *
       * \return Long description string.
       */
      const std::string &getLongDesc() const
      {
        return m_strLongDesc;
      }

      /*!
       * \brief Get the total number command forms.
       *
       * \return Number of forms.
       */
      int numOfForms() const
      {
        return (int)m_formDefs.size();
      }

      /*!
       * \brief Get command form at index.
       *
       * \param nIndex  Form's index.
       *
       * \return Command form definition reference.
       */
      const CmdFormDef &at(const int nIndex) const;

      /*!
       * \brief Index operator.
       *
       * Get command form at index.
       *
       * \param nIndex    Argument index.
       *
       * \return Command form definition reference.
       */
      const CmdFormDef &operator[](const int nIndex) const
      {
        return at(nIndex);
      }

      /*!
       * \brief Insert object into output stream.
       *
       * \param os      Output stream.
       * \param cmddef  Object to insert.
       *
       * \return Reference to output stream.
       */
      friend std::ostream &operator<<(std::ostream &os, const CmdDef &cmddef);

      //
      // Friends
      //
      friend class CommandLine;

    protected:
      int           m_nUid;         ///< command unique id
      std::string   m_strName;      ///< command name
      std::string   m_strSyntax;    ///< parsable command extended usage syntax
      std::string   m_strSynopsis;  ///< short command synopsis
      std::string   m_strLongDesc;  ///< long command description
      CmdFormDefVec m_formDefs;     ///< vector of command forms

      /*!
       * \brief Reset command definition to pre-compiled state.
       */
      void reset();

      /*!
       * \brief Set command's unique id.
       *
       * \param uid     Unique id.
       */
      void setUid(const int uid);

      /*!
       * \brief Set command's name.
       *
       * \param strName Name string.
       */
      void setName(const std::string &strName);

      ///@{
      /*!
       * \brief Add help to command.
       *
       * \param sSynopsis     Short command synopsis.
       * \param sLongDesc     Long command description.
       * \param strSynopsis   Short command synopsis.
       * \param strLongDesc   Long command description.
       */
      void addHelp(const char *sSynopsis, const char *sLongDesc);

      void addHelp(const std::string &strSynopsis,
                   const std::string &strLongDesc);
      ///@}

      /*!
       * \brief Push new command form to end of form list.
       *
       * \param formdef   Command form definition.
       */
      void pushForm(CmdFormDef &formdef);

      /*!
       * \brief Get command modifiable form at index.
       *
       * Protected version of at().
       *
       * \param nIndex  Form's index.
       *
       * \return Command form definition reference.
       */
      CmdFormDef &formAt(const int nIndex);

    }; // class CmdDef
  
    //
    // Types and Data
    //
    
  } // namespace cmd
} // namespace rnr

#endif // _RNR_CMD_DEF_H
