////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// File:      example_dlist.h
//
/*! \file
 *
 * $LastChangedDate: 2010-03-24 10:19:36 -0600 (Wed, 24 Mar 2010) $
 * $Rev: 307 $
 *
 * \brief Example of a "derived" DListVoid doubly-linked lists.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2007-2010  RoadNarrows LLC.
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

#include <stdio.h>

#include "rnr/rnrconfig.h"
#include "rnr/new.h"

#define DLIST_DNAME  Zoo    ///< derived dlist name space (i.e. DListZoo<x>)
#define DLIST_DTYPE  Zoo_T  ///< derived dlist user data type

/*!
 * \brief Zoo Type.
 */
typedef struct
{
  int         m_nZid;       ///< zoo animal unique id
  const char *m_sSciName;   ///< zoo animal scientific name
  const char *m_sComName;   ///< zoo animal common name
  char       *m_sPetName;   ///< zoo animal pet (pr) name
} Zoo_T;

#include "rnr/dlistvoid.h"  ///< define "derived" data and functions

/*!
 * \brief Node zoo data comparator callback.
 *
 * \param pData1  Zoo node data 1.
 * \param pData2  Zoo node data 2.
 *
 * \returns \h_lt 0, 0, or \h_gt 0 if pData1 is less than, equal to, or greater
 * than pData2, respectively.
 */
static inline int DListZooDataCmp(const Zoo_T *pData1, const Zoo_T *pData2)
{
  return strcmp(pData1->m_sPetName, pData2->m_sPetName);
}

/*!
 * \brief Node zoo data delete callback.
 *
 * \param pData  Zoo node data.
 */
static inline void DListZooDataDelete(Zoo_T *pData)
{
  delete(pData->m_sPetName);
  delete(pData);
}

/*!
 * \brief Node zoo data print callback.
 *
 * \param fp     File pointer.
 * \param pData  Zoo node data.
 */
static inline void DListZooDataPrint(FILE *fp, Zoo_T *pData)
{
  if( pData != NULL )
  {
    fprintf(fp, "%03d \"%s\": %s (%s)",
        pData->m_nZid, pData->m_sPetName, pData->m_sSciName, pData->m_sComName);
  }
}
