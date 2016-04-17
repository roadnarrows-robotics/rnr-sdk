////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// Library:   librnr
//
// File:      win.c
//
/*! \file
 *
 * $LastChangedDate: 2010-03-24 10:19:36 -0600 (Wed, 24 Mar 2010) $
 * $Rev: 307 $
 *
 * \brief Windows functions for linux.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2005-2010.  RoadNarrows LLC.
 * \n All Rights Reserved
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

#include "rnr/rnrconfig.h"
#include "rnr/new.h"

#if !defined(__windows__)

#include <sys/types.h>
#include <stdio.h>
#include <string.h>

/*! 
 * \brief Copy source string to destinations string. At most n bytes will be
 * copied. The destination string is guaranteed to be null terminated.
 *
 * \param dest  Pointer to destination string.
 * \param n     Maximum number of bytes to copy.
 * \param src   Pointer to source string.
 *
 * \return Return pointer to destination string.
 */
char *strcpy_s(char *dest, size_t n, const char *src)
{
  strncpy(dest, src, (size_t)n);
  dest[n-1] = 0;
  return dest;
}

/*! 
 * \brief Format print to string up to n-1 characters. String is 
 * guaranteed to be null terminated.
 *
 * \param str     Destination string buffer.
 * \param n       Size of string buffer.
 * \param format  Standard printf formatting string.
 * \param ...     Variable argument list.
 *
 * \return Number of characters printed.
 */
int sprintf_s(char *str, size_t n, const char *format, ...)
{
  va_list ap;
  int     nPrinted;

  va_start(ap, format);
  nPrinted = vsnprintf(str, (size_t)n, format, ap);
  va_end(ap);
  str[n-1] = 0;
  return nPrinted;
}

#endif // __windows__
