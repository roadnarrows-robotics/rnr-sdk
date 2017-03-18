////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// Library:   librnr
//
// File:      dliststr.h
//
/*! \file
 *
 * $LastChangedDate: 2010-03-24 10:19:36 -0600 (Wed, 24 Mar 2010) $
 * $Rev: 307 $
 *
 * \brief Doubly linked list of character strings "inherited" from dlistvoid.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright
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

#ifndef _DLISTSTR_H
#define _DLISTSTR_H

#include <stdio.h>
#include <string.h>

#include "rnr/new.h"

#define DLIST_DNAME Str     ///< dlist namespace name
#define DLIST_DTYPE char    ///< dlist data type 

#include "rnr/dlistvoid.h"

C_DECLS_BEGIN


/*!
 * \brief Node data comparator callback.
 *
 * \param sData1  Node 1 string data.
 * \param sData2  Node 2 string data.
 *
 * \return Returns <0, 0, or >0 if pData1 is less than, equal, or greater than
 * pData2, respectively.
 */
static inline int DListStrDataCmp(const char *sData1, const char *sData2)
{
  return strcmp(sData1, sData2);
}

/*!
 * \brief Node data delete callback.
 *
 * \param sData Node string data.
 */
static inline void DListStrDataDelete(char *sData)
{
  delete(sData);
}

/*!
 * \brief Print node data callback.
 *
 * \param fp    File stream pointer.
 * \param sData Node string data.
 *
 * \sa DListVoidPrint()
 */
static inline void DListStrDataPrint(FILE *fp, char *sData)
{
  if( sData != NULL )
  {
    fprintf(fp, "%s", sData);
  }
}

/*!
 * \brief Allocator and initializer new empty string dlist with default 
 * callbacks.
 *
 * Default user string comparator function is strcmp().
 * Default string data deallocator function is delete().
 *
 * \return Returns pointer to new dlist (head) on success.
 */
static inline DListStr_T *DListStrNewDft()
{
  return DListStrNew(DListStrDataCmp, DListStrDataDelete);
}

C_DECLS_END


#endif // _DLISTSTR_H
