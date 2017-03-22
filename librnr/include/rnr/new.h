////////////////////////////////////////////////////////////////////////////////
/*! \file
 *
 * \brief Memory allocation and deallocation declarations.
 *
 * General purpose dynamic memory allocate and deallocate declarations and
 * defines. None of the calls will fail, unless fatally for the entire 
 * program. Modeled on C++ new() and delete() operators.
 *
 * \pkgsynopsis
 * RoadNarrows Robotics Common Library 1
 *
 * \pkgcomponent{Library}
 * librnr
 *
 * \pkgfile{rnr/new.h}
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2005-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 *
 * \license{MIT}
 *
 * \EulaBegin
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
 * \n\n
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
 * OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * \n\n
 * THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * \EulaEnd
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _RNR_NEW_H
#define _RNR_NEW_H

#include "rnr/rnrconfig.h"

#include <sys/types.h>
#include <stdlib.h>
#include <stdarg.h>

/*!
 * \brief Allocate new type.
 *
 * \param T Data type.
 *
 * \return Pointer T* to allocated memory
 *
 * \exception "Bad Allocation" 
 *    Program is terminated with exit code \ref EC_ERROR.
 */
#define NEW(T) (T *)new(sizeof(T))

//
// Allocate new string
//
/*!
 * \brief Allocate new string buffer of length len+1.
 *
 * \param len Length of string buffer less null-termination character.
 *
 * \return Returns char * to allocated, zero'ed out memory.
 *
 * \exception "Bad Allocation" 
 *    Program is terminated with exit code \ref EC_ERROR.
 */
#define NEWSTR(len) (char *)new( sizeof(char)*((size_t)((len)+1)) )

C_DECLS_BEGIN

#ifndef __cplusplus

/*!
 * \brief delete() "operator" 
 *
 * Frees previously allocated memory. NULL pointer produce no action.
 *
 * \param p Pointer to memory.
 */
INLINE_IN_H void delete(void *p)
{
  if( p != NULL )
  {
    free(p);
  }
}

extern void *new(size_t size);

#endif // __cplusplus 

extern void *new_overs(void *pSrc, size_t sizeDup, size_t sizeNew);

extern void *new_memdup(size_t size, void *data);

extern char *new_strdup(const char *s);

extern char *new_strndup(const char *s, size_t n);

C_DECLS_END

//
// Common platform utilities
//
#if !defined(__windows__)

C_DECLS_BEGIN

extern char *strcpy_s(char *dest, size_t n, const char *src);

C_DECLS_END

#ifdef __cplusplus
/*! 
 * \brief Format print to string up to sizeof(str)-1 characters. String is 
 * guaranteed to be null terminated.
 *
 * \param str     Destination string buffer.
 * \param format  Standard printf formatting string.
 * \param ...     Variable argument list.
 *
 * \return Number of characters printed.
 */
template <size_t size>
inline int sprintf_s(char (&str)[size], const char *format, ...)
{
  va_list ap;
  size_t  n = sizeof(str);
  int     nPrinted;

  va_start(ap, format);
  nPrinted = vsnprintf(str, n, format, ap);
  va_end(ap);
  str[n-1] = 0;
  return nPrinted;
}

#else

C_DECLS_BEGIN

extern int sprintf_s(char *str, size_t n, const char *format, ...);

C_DECLS_END

#endif // __cplusplus

#endif // ! __windows__ 


#endif // _RNR_NEW_H
