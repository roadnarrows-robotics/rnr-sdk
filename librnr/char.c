////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// Library:   librnr
//
// File:      char.h
//
/*! \file
 *
 * $LastChangedDate: 2010-03-24 10:19:36 -0600 (Wed, 24 Mar 2010) $
 * $Rev: 307 $
 *
 * \brief Simple character manipulations.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2005-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
// Permission is hereby granted, without written agreement and without
// license or royalty fees, to use, copy, modify, and distribute this
// software and its documentation for any purpose, provided that
// (1) The above copyright notice and the following two paragraphs
// appear in all copies of the source code and (2) redistributions
// including binaries reproduces these notices in the supporting
// documentation.   Substantial modifications to this software may be
// copyrighted by their authors and need not follow the licensing terms
// described here, provided that the new terms are clearly indicated in
// all files where they apply.
//
// IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
// OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
// PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
// EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.
//
// THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
// "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
// PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
//
////////////////////////////////////////////////////////////////////////////////

#include <ctype.h>

#include "rnr/rnrconfig.h"
#include "rnr/new.h"
#include "rnr/char.h"

/*!
 * 
 * \brief Allocates a string buffer and copies a 'prettified' readable 
 * version of the contents of the given (binary) buffer.
 *
 * \param buf Buffer to prettify
 * \param len Number of bytes in buf.
 *
 * \return
 *  Allocated, null-terminated, prettified string. Use delete() to deallocate.
 */
char *NewPrettyBuf(byte_t buf[], size_t len)
{
  char  *sPretty = NEWSTR(len*4);
  char  *s;
  int   i, c;

  for(i=0, s=sPretty; i<len; ++i)
  {
    c = (int)buf[i];
    if( c == ' ' )
    {
      *s++ = ' ';
    }
    else if( isspace(c) )
    {
      *s++ = '\\';
      switch(c)
      {
        case '\f':
          *s++ = 'f';
          break;
        case '\n':
          *s++ = 'n';
          break;
        case '\r':
          *s++ = 'r';
          break;
        case '\t':
          *s++ = 't';
          break;
        case '\v':
          *s++ = 'v';
          break;
        default:
          *s++ = hexnibbletoa( (c>>4) & 0x0f );
          *s++ = hexnibbletoa( c & 0x0f );
          break;
      }
    }
    else if( isprint(c) )
    {
      *s++ = (char)c;
    }
    else
    {
      *s++ = '\\';
      *s++ = 'x';
      *s++ = hexnibbletoa( (c>>4) & 0x0f );
      *s++ = hexnibbletoa( c & 0x0f );
    }
  }
  *s = 0;
  return sPretty;
}

/*!
 * 
 * \brief Print out 'prettified' readable version of the contents of the 
 * given (binary) buffer.
 *
 * \param fp  Output file pointer.
 * \param buf Buffer to prettify.
 * \param len Number of bytes in buf.
 *
 * \return
 *  Number of characters printed.
 */
int PrettyPrintBuf(FILE *fp, byte_t buf[], size_t len)
{
  char   *sPretty;
  int     n;

  sPretty = NewPrettyBuf(buf, len);

  n = fprintf(fp, "%s", sPretty);

  delete(sPretty);

  return n;
}
