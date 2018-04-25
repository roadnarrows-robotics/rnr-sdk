////////////////////////////////////////////////////////////////////////////////
/*! \file
 *
 * \brief Doubly linked list of character strings "inherited" from dlistvoid.
 *
 * \pkgsynopsis
 * RoadNarrows Robotics Common Library 1
 *
 * \pkgcomponent{Library}
 * librnr
 *
 * \pkgfile{rnr/dliststr.h}
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \pkgcopyright{2005-2018,RoadNarrows LLC.,http://www.roadnarrows.com}
 *
 * \license{MIT}
 *
 * \EulaBegin
 * See the README and EULA files for any copyright and licensing information.
 * \EulaEnd
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _RNR_DLISTSTR_H
#define _RNR_DLISTSTR_H

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


#endif // _RNR_DLISTSTR_H
