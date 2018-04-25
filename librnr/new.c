////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// Library:   librnr
//
// File:      new.c
//
/*! \file
 *
 * $LastChangedDate: 2010-03-24 10:19:36 -0600 (Wed, 24 Mar 2010) $
 * $Rev: 307 $
 *
 * \brief Memory allocation and deallocation definitions.
 *
 * General purpose dynamic memory allocate and deallocate definitions.
 * None of the calls will fail, unless fatally for the entire 
 * program. Modeled on C++ new() and delete() operators.
 *
 * The current implementation uses the malloc() and free() standard library
 * calls.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \pkgcopyright{2005-2018,RoadNarrows LLC.,http://www.roadnarrows.com}
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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "rnr/rnrconfig.h"
#include "rnr/new.h"

/*!
 * \brief new() "operator" 
 *
 * Dynamic memory allocator. All allocated data are zero'ed (i.e. set to 0).
 *
 * \param size Number of bytes.
 *
 * \return
 *    Return void * to newly allocated memory. Memory is zero'ed.
 *
 * \exception "Bad Allocation" 
 *    Program is terminated with exit code \ref EC_ERROR.
 */
void *new(size_t size)
{
  void  *p;

  if( size == 0 )
  {
    size = 1;
  }

  if( (p = malloc(size)) == NULL )
  {
    fprintf(stderr, "new(): malloc() failed\n");
    exit(EC_ERROR);
  }
  memset(p, 0, size);
  return p;
}

/*!
 * \brief Allocate and duplicate.
 *
 * A new block a memory of sizeNew bytes is allocated. sizeDup bytes from
 * the source data are copied to the new block. All allocated, but not
 * duplicated data are zero'ed (i.e. set to 0).
 *
 * \note The actual size of source block is assumed to be >= sizeDup.
 *
 * \param pSrc    Source memory block to duplicate.
 * \param sizeDup Number of bytes to duplicate, with sizeDup <= sizeNew.
 * \param sizeNew Number of bytes for new block.
 *
 * \return
 *    Return void * to newly allocated and duplicated memory.
 *
 * \exception "Bad Allocation" 
 *    Program is terminated with exit code \ref EC_ERROR.
 * \exception "Bad Sizes" 
 *    Program is terminated with exit code \ref EC_ERROR.
 */
void *new_overs(void *pSrc, size_t sizeDup, size_t sizeNew)
{
  void  *pNew;
  
  if( sizeDup > sizeNew )
  {
    fprintf(stderr, "new_overs(): duplicate size %lu > allocated size %lu\n",
        (unsigned long)sizeDup, (unsigned long)sizeNew);
    exit(EC_ERROR);
  }

  pNew = new(sizeNew);

  if( sizeDup > 0 )
  {
    memcpy(pNew, pSrc, sizeDup);
  }

  return pNew;
}

/*!
 * \brief Duplicate data.
 *
 * \param size Number of bytes.
 * \param data Data to duplicate
 *
 * \return Return pointer to duplicated memory.
 *
 * \exception "Bad Allocation" 
 *    Program is terminated with exit code \ref EC_ERROR.
 *
 * \note If size is zero or data is NULL, one byte is allocated.
 */
void *new_memdup(size_t size, void *data)
{
  void *p;

  if( (size == 0) || (data == NULL) )
  {
    p = (void *)new((size_t)1);
    memset(p, 0, (size_t)1);
  }
  else
  {
    p = new(size);
    memcpy(p, data, size);
  }
  return p;
}

/*!
 * \brief Duplicate a string.
 *
 * \param s Null terminated string to duplicate.
 *
 * \return Return pointer to duplicated string.
 *
 * \exception "Bad Allocation" 
 *    Program is terminated with exit code \ref EC_ERROR.
 *
 * \note Guaranteed to return a null terminated string.
 * If string is NULL, one byte is allocated (== 0).
 */
char *new_strdup(const char *s)
{
  char *t;

  if( (s != NULL) && (*s != 0) )
  {
    t = NEWSTR(strlen(s));
    strcpy(t, s);
  }
  else
  {
    t = NEWSTR(1);
    t[0] = 0;
  }
  return t;
}

/*!
 * \brief Duplicate not more than n characters of string.
 *
 * Duplicate the first n characters of string or until a null character
 * is encounter, whichever comes first. Guaranteed to return a null 
 * terminated string.
 *
 * \param s String to duplicate.
 * \param n Maximum number of characters to duplicate.
 *
 * \exception "Bad Allocation" 
 *    Program is terminated with exit code \ref EC_ERROR.
 *
 *  \return Return pointer to duplicated string.
 */
char *new_strndup(const char *s, size_t n)
{
  char *t;

  if( (s != NULL) && (*s != 0) && (n > 0) )
  {
    t = NEWSTR(n+1);
    strncpy(t, s, n);
    t[n] = 0;
  }
  else
  {
    t = NEWSTR(1);
    t[0] = 0;
  }
  return t;
}
