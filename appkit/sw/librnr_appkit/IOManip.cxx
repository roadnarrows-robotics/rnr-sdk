////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Application Tool Kit
//
// Link:      https://github.com/roadnarrows-robotics/rnr-sdk
//
// Library:   librnr_appkit
//
// File:      IOManip.cxx
//
/*! \file
 *
 * \brief Stream I/O manipulators and helpers.
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
#include <ctype.h>

#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>

#include "rnr/appkit/IOManip.h"

using namespace std;

namespace rnr
{
  namespace io
  {
    /*!
     * \brief Set absolute indentation level.
     *
     * \param nIndent   Number of spaces to left indent.
     *
     * \return Indent manipulation object.
     */
    osManipIndent setindent(const long nIndent)
    {
      osManipIndent obj;

      obj.m_nIndent = nIndent;
      obj.m_eChange = osManipIndent::Absolute;
      obj.m_bOut    = false;

      return obj;
    }

    /*!
     * \brief Set relative delta indentation level.
     *
     * \param nDelta    Plus or minus delta from current indentation level.
     *
     * \return Indent manipulation object.
     */
    osManipIndent deltaindent(const long nDelta)
    {
      osManipIndent obj;

      obj.m_nIndent = nDelta;
      obj.m_eChange = osManipIndent::Relative;
      obj.m_bOut    = false;

      return obj;
    }

    /*!
     * \brief Left indent at current stream indentation level.
     *
     * \return Indent manipulation object.
     */
    osManipIndent indent()
    {
      osManipIndent obj;

      obj.m_nIndent = 0;
      obj.m_eChange = osManipIndent::NoChange;
      obj.m_bOut    = true;

      return obj;
    }

    /*!
     * \brief Insert indentation object into output stream.
     *
     * \param os  Output stream.
     * \param f   Object to insert.
     *
     * \return Reference to output stream.
     */
    ostream &operator<<(ostream &os, const osManipIndent &f)
    {
      const static int index = os.xalloc();

      long  level;

      switch( f.m_eChange )
      {
        case osManipIndent::Relative:
          level = os.iword(index) + f.m_nIndent;
          if( level < 0 )
          {
            level = 0;
          }
          os.iword(index) = level;
          break;
        case osManipIndent::Absolute:
          level = f.m_nIndent >= 0? f.m_nIndent: 0;
          os.iword(index) = level;
          break;
        case osManipIndent::NoChange:
        default:
          break;
      }

      if( f.m_bOut )
      {
        for(level = 0; level < os.iword(index); ++level)
        {
          os << ' ';
        }
      }

      return os;
    }

  } // namespace io
} // namespace rnr