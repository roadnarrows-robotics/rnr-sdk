////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// Library:   librnr
//
// File:      dlistvoid.c
//
/*! \file
 *
 * $LastChangedDate: 2010-03-24 10:19:36 -0600 (Wed, 24 Mar 2010) $
 * $Rev: 307 $
 *
 * \brief Doubly linked list (dlist) of void pointers definitions.
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "rnr/rnrconfig.h"
#include "rnr/dlist.h"
#include "rnr/dlistvoid.h"
#include "rnr/log.h"
#include "rnr/new.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Unit Tests (define to build test main)
 */
#if defined(unittestDListVoid) && !defined(debugDListVoid)
#define  debugDListVoid     ///< UT needs debugging code
#endif

/*!
 * \brief Doubly linked list head structure.
 */
struct dlistvoid_head_t
{
  dnode_t                   m_dnodeHead;    ///< head of dlist
  int                       m_nNumNodes;    ///< number of nodes in dlist
  DListVoidFuncDataCmp_T    m_fnDataCmp;    ///< data compare function
  DListVoidFuncDataDelete_T m_fnDataDelete; ///< data delete function
};

/*!
 * \brief Doubly linked list node structure.
 */
struct dlistvoid_node_t
{
  dnode_t   m_dnode;  ///< dlist pointers
  void     *m_pData;  ///< user data
};


//
// Helper Macros
//

/*!
 * \brief Get embedding node DListVoidNode_T  
 * \param dnode dnode_t *
 * \return DListVoidNode_T* 'pointed to' by dnode_t*
 */
#define DLISTVOID_ENTRY(dnode)  dlist_entry(dnode, DListVoidNode_T, m_dnode)

/*!
 * \brief Test if DListVoid_T* holds the empty list.
 * \param dvoidhead dnode_t *
 * \return True (non-zero) if list is empty, 0 otherwise.
 */
#define DLISTVOID_IS_EMPTY(dvoidhead) \
            dlist_is_empty(&((dvoidhead)->m_dnodeHead))

/*!
 * \brief Test if node is at end-of-list.
 * \param dvoidhead dnode_t *
 * \param pdnode    dnode_t *
 * \return True (non-zero) if at end-of-list, 0 otherwise.
 */
#define DLISTVOID_IS_EOL(dvoidhead, pdnode) \
            ((pdnode) == &((dvoidhead)->m_dnodeHead))



// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Allocate and initialize new empty dlist.
 *
 * \param fnDataCmp     User-supplied data comparator function.
 *                      NULL will disable some functions such as searches.
 * \param fnDataDelete  User-supplied data deallocator.
 *                      NULL will cause user data not to be deleted. 
 *
 * \return Returns pointer to new dlist (head) on success.
 */
DListVoid_T *DListVoidNew(DListVoidFuncDataCmp_T fnDataCmp,
                          DListVoidFuncDataDelete_T fnDataDelete)
{
  DListVoid_T  *pHead;

  LOGDIAG4CALL(_TPTR(fnDataCmp), _TPTR(fnDataDelete));

  // allocate head
  pHead = NEW(DListVoid_T);

  // initialize data
  DLIST_NODE_INIT(pHead->m_dnodeHead);
  pHead->m_nNumNodes    = 0;
  pHead->m_fnDataCmp    = fnDataCmp;
  pHead->m_fnDataDelete = fnDataDelete;

  LOGDIAG4(_TPTR(pHead));

  return pHead;
}

/*!
 * \brief Delete entire dlist.
 *
 * All user data are deleted with a per node callback to the user-supplied
 * data delete function, if any. All nodes in the dlist and the dlist itself
 * will be deleted. The dlist pointer will no longer be valid after this call.
 *
 * \param pHead   Pointer to dlist (head). 
 */
void DListVoidDelete(DListVoid_T *pHead)
{
  LOGDIAG4CALL(_TPTR(pHead));

  if( pHead == NULL )
  {
    return;
  }

  DListVoidDeleteAllNodes(pHead);
  delete(pHead);
}

/*!
 * \brief Unlink and delete all nodes and data from dlist, leaving dlist empty.
 *
 * User data are deleted with a per node callback to the user-supplied
 * data delete function, if any.
 *
 * \param pHead   Pointer to dlist (head). 
 */
void DListVoidDeleteAllNodes(DListVoid_T *pHead)
{
  dnode_t *p, *q, *head;

  LOGDIAG4CALL(_TPTR(pHead));

  if( pHead == NULL )
  {
    return;
  }

  head = &(pHead->m_dnodeHead);

  dlist_for_each_safe(p, q, head)
  {
    DListVoidDeleteNode(pHead, DLISTVOID_ENTRY(p));
  }
}

/*!
 * \brief Unlink node from the dlist and delete the node and it's attached
 * user data.
 *
 * User data are deleted with a callback to the user-supplied data delete
 * function, if any.
 *
 * \param pHead  Pointer to dlist (head).
 * \param pNode  Pointer to node to be deleted.
 */
void DListVoidDeleteNode(DListVoid_T *pHead, DListVoidNode_T *pNode)
{
  LOGDIAG4CALL(_TPTR(pHead), _TPTR(pNode));

  CHKPTR(pHead);
  CHKPTR(pNode);

  if( DLISTVOID_IS_EMPTY(pHead) )
  {
    LOGERROR("dlist is empty");
    return;
  }

  // remove from list
  dlist_delete(&pNode->m_dnode);

  // free any data by value
  if( pHead->m_fnDataDelete != NULL )
  {
    pHead->m_fnDataDelete(pNode->m_pData);
  }

  // free up node
  delete(pNode);

  pHead->m_nNumNodes--;
}

/*!
 * \brief Unlink the node from the dlist at the given index and delete it and
 * it's attached user data.
 *
 * User data are deleted with a callback to the user-supplied data delete
 * function, if any.
 *
 * \param pHead  Pointer to dlist (head).
 * \param iIndex Index to dlist entry to delete.
 */
void DListVoidDeleteNodeAt(DListVoid_T *pHead, int iIndex)
{
  DListVoidNode_T  *pNode;

  LOGDIAG4CALL(_TPTR(pHead), _TINT(iIndex));

  CHKPTR(pHead);

  // get node to delete
  if( (pNode = DListVoidGetNodeAt(pHead, iIndex)) != NULL )
  {
    DListVoidDeleteNode(pHead, pNode);
  }
}

/*!
 * \brief Unlink the node from the dlist at the given index.
 *
 * The node and it's attached data are not deleted.
 *
 * \param pHead  Pointer to dlist (head).
 * \param iIndex Index to dlist entry to unlink.
 *
 * \return Returns pointer to unlinked node on success, NULL on failure.
 */
DListVoidNode_T *DListVoidUnlinkNodeAt(DListVoid_T *pHead, int iIndex)
{
  DListVoidNode_T  *pNode;

  LOGDIAG4CALL(_TPTR(pHead), _TINT(iIndex));

  CHKPTR(pHead, NULL);

  // get node to delete
  if( (pNode = DListVoidGetNodeAt(pHead, iIndex)) != NULL )
  {
    // remove from list
    dlist_delete(&pNode->m_dnode);

    pHead->m_nNumNodes--;

    return pNode;
  }
  else
  {
    return NULL;
  }
}

/*!
 * \brief Unlink the user data from the dlist at the given index.
 *
 * The node is deleted, but not the attached data.
 *
 * \param pHead  Pointer to dlist (head).
 * \param iIndex Index to dlist entry to unlink.
 *
 * \return Returns void* to user data on success, NULL on failure.
 */
void *DListVoidUnlinkDataAt(DListVoid_T *pHead, int iIndex)
{
  DListVoidNode_T  *pNode;
  void             *pData;

  LOGDIAG4CALL(_TPTR(pHead), _TINT(iIndex));

  CHKPTR(pHead, NULL);

  // get node to delete
  if( (pNode = DListVoidGetNodeAt(pHead, iIndex)) != NULL )
  {
    pData = pNode->m_pData;

    // remove from list
    dlist_delete(&pNode->m_dnode);

    // free up node, but not the data
    delete(pNode);

    pHead->m_nNumNodes--;

    return pData;
  }
  else
  {
    return NULL;
  }
}

/*!
 * \brief Attach user data to a new node and link to the end of dlist.
 *
 * \param pHead Pointer to dlist (head).
 * \param pData Pointer to user (allocated) data.
 *
 * \return Returns pointer to new node on success, NULL on failure.
 */
DListVoidNode_T *DListVoidAppend(DListVoid_T *pHead, void *pData)
{
  DListVoidNode_T  *pNode;

  LOGDIAG4CALL(_TPTR(pHead), _TPTR(pData));

  CHKPTR(pHead, NULL);

  // allocate node
  pNode = NEW(DListVoidNode_T);

  // attach data
  pNode->m_pData = pData;

  // insert into list
  dlist_append(&pNode->m_dnode, &pHead->m_dnodeHead);

  pHead->m_nNumNodes++;

  LOGDIAG4(_TPTR(pNode));

  return pNode;
}

/*!
 * \brief Attach the user data to a new node and link to the start of dlist.
 *
 * \param pHead Pointer to dlist (head).
 * \param pData Pointer to user (allocated) data.
 *
 * \return Returns pointer to new node on success, NULL on failure.
 */
DListVoidNode_T *DListVoidPrepend(DListVoid_T *pHead, void *pData)
{
  DListVoidNode_T  *pNode;

  LOGDIAG4CALL(_TPTR(pHead), _TPTR(pData));

  CHKPTR(pHead, NULL);

  // allocate node
  pNode = NEW(DListVoidNode_T);

  // attach data
  pNode->m_pData = pData;

  // insert into list
  dlist_prepend(&pNode->m_dnode, &pHead->m_dnodeHead);

  pHead->m_nNumNodes++;

  LOGDIAG4(_TPTR(pNode));

  return pNode;
}

/*!
 * \brief Attach the user data to a new node and insert before the dlist index.
 *
 * The special index value DLIST_INSERT_AT_END will insert at end-of-list
 * (append).
 *
 * \param pHead   Pointer to dlist (head).
 * \param iIndex  Index to dlist entry where insertion occurs.
 * \param pData   Pointer to user (allocated) data.
 *
 * \return Returns pointer to new node on success, NULL on failure.
 */
DListVoidNode_T *DListVoidInsert(DListVoid_T *pHead, int iIndex, void *pData)
{
  DListVoidNode_T  *pNext;
  DListVoidNode_T  *pNode;

  LOGDIAG4CALL(_TPTR(pHead), _TINT(iIndex), _TPTR(pData));

  CHKPTR(pHead, NULL);

  if( iIndex == 0 )
  {
    return DListVoidPrepend(pHead, pData);
  }
  else if( (iIndex == DLIST_INSERT_AT_END)  || 
           (iIndex == DListVoidCount(pHead)) )
  {
    return DListVoidAppend(pHead, pData);
  }

  // get insertion point
  if( (pNext = DListVoidGetNodeAt(pHead, iIndex)) == NULL )
  {
    return NULL;
  }

  // allocate node
  pNode = NEW(DListVoidNode_T);

  // attach data
  pNode->m_pData = pData;

  // insert into list
  _dlist_add(&pNode->m_dnode, pNext->m_dnode.prev, &pNext->m_dnode);

  pHead->m_nNumNodes++;

  LOGDIAG4(_TPTR(pNode));

  return pNode;
}

/*!
 * \brief Get the node at the given dlist index.
 *
 * \param pHead   Pointer to dlist (head).
 * \param iIndex  Index to dlist entry to find.
 *
 * \return Returns node at index on success, NULL on failure.
 */
void *DListVoidGetNodeAt(DListVoid_T *pHead, int iIndex)
{
  int       nCnt;
  dnode_t  *p, *q, *head;

  LOGDIAG4CALL(_TPTR(pHead), _TINT(iIndex));

  CHKPTR(pHead, NULL);

  nCnt = pHead->m_nNumNodes;

  if( iIndex == DLIST_INSERT_AT_END )
  {
    iIndex = nCnt - 1;
  }

  if( (iIndex < 0) || (iIndex >= nCnt) )
  {
    LOGERROR("%d index is out of range", iIndex);
  }

  head = &(pHead->m_dnodeHead);

  dlist_for_each_safe(p, q, head)
  {
    if( iIndex-- == 0 )
    {
      return DLISTVOID_ENTRY(p);
    }
  }

  return NULL;
}

/*!
 * \brief Get the user data from the given node.
 *
 * \param pNode   Pointer to node with data.
 *
 * \return Returns void* to user data on success, NULL on failure.
 */
void *DListVoidGetData(DListVoidNode_T *pNode)
{
  return pNode == NULL? NULL: pNode->m_pData;
}

/*!
 * \brief Get the user data from the dlist at the given index.
 *
 * \param pHead   Pointer to dlist (head).
 * \param iIndex  Index to dlist entry.
 *
 * \return Returns void* to user data on success, NULL on failure.
 */
void *DListVoidGetDataAt(DListVoid_T *pHead, int iIndex)
{
  DListVoidNode_T  *pNode;

  if( (pNode = DListVoidGetNodeAt(pHead, iIndex)) != NULL )
  {
    return DListVoidGetData(pNode);
  }
  else
  {
    return NULL;
  }
}

/*!
 * \brief Get the number of nodes in dlist.
 *
 * \param pHead Pointer to dlist (head).
 *
 * \return Number of nodes in dlist.
 */
int DListVoidCount(DListVoid_T *pHead)
{
  return pHead == NULL? 0: pHead->m_nNumNodes;
}

/*!
 * \brief Initialize dlist iterator and return the first node in dlist.
 *
 * \param pHead   Pointer to dlist (head).
 * \param pIter   Pointer to iterator.
 *
 * \return Returns first node in dlist on success, NULL if dlist empty or 
 * on error.
 */
DListVoidNode_T *DListVoidIterNodeFirst(DListVoid_T *pHead, 
                                        DListVoidIter_T *pIter)
{
  CHKPTR(pHead, NULL);
  CHKPTR(pIter, NULL);

  pIter->m_pHead = pHead;
  pIter->m_pThis = pHead->m_dnodeHead.next;
  pIter->m_pNext = ((dnode_t *)(pIter->m_pThis))->next;

  if( DLISTVOID_IS_EMPTY(pHead) )
  {
    return NULL;
  }
  else
  {
    return DLISTVOID_ENTRY(((dnode_t *)(pIter->m_pThis)));
  }
}

/*!
 * \brief Get the next iterated node in dlist.
 *
 * \param pIter   Pointer to initialized iterator.
 *
 * \return Returns next node in dlist on success, NULL if if at end of list or 
 * on error.
 */
DListVoidNode_T *DListVoidIterNodeNext(DListVoidIter_T *pIter)
{
  CHKPTR(pIter, NULL);
  CHKPTR(pIter->m_pHead, NULL);
  CHKPTR(pIter->m_pNext, NULL);

  // end of list
  if( DLISTVOID_IS_EOL(pIter->m_pHead, pIter->m_pNext) )
  {
    return NULL;
  }
  else
  {
    pIter->m_pThis = pIter->m_pNext;
    pIter->m_pNext = ((dnode_t *)(pIter->m_pThis))->next;
    return DLISTVOID_ENTRY(((dnode_t *)(pIter->m_pThis)));
  }
}

/*!
 * \brief Initialize dlist iterator and return the last node in dlist.
 *
 * \param pHead   Pointer to dlist (head).
 * \param pIter   Pointer to iterator.
 *
 * \return Returns last node in dlist on success, NULL if dlist empty or 
 * on error.
 */
DListVoidNode_T *DListVoidIterNodeLast(DListVoid_T *pHead, 
                                        DListVoidIter_T *pIter)
{
  CHKPTR(pHead, NULL);
  CHKPTR(pIter, NULL);

  pIter->m_pHead = pHead;
  pIter->m_pThis = pHead->m_dnodeHead.prev;
  pIter->m_pNext = ((dnode_t *)(pIter->m_pThis))->prev;

  if( DLISTVOID_IS_EMPTY(pHead) )
  {
    return NULL;
  }
  else
  {
    return DLISTVOID_ENTRY(((dnode_t *)(pIter->m_pThis)));
  }
}

/*!
 * \brief Get the previous iterated node in dlist.
 *
 * \param pIter   Pointer to initialized iterator.
 *
 * \return Returns previous node in dlist on success,
 * NULL if if at end of list or on error.
 */
DListVoidNode_T *DListVoidIterNodePrev(DListVoidIter_T *pIter)
{
  CHKPTR(pIter, NULL);
  CHKPTR(pIter->m_pHead, NULL);
  CHKPTR(pIter->m_pNext, NULL);

  // end of list
  if( DLISTVOID_IS_EOL(pIter->m_pHead, pIter->m_pNext) )
  {
    return NULL;
  }
  else
  {
    pIter->m_pThis = pIter->m_pNext;
    pIter->m_pNext = ((dnode_t *)(pIter->m_pThis))->prev;
    return DLISTVOID_ENTRY(((dnode_t *)(pIter->m_pThis)));
  }
}

/*!
 * \brief Find the first node with the matching data.
 *
 * The user provided comparator function is used, if any.
 *
 * \param pHead   Pointer to dlist (head).
 * \param pData   Pointer to data to find.
 *
 * \return Returns first node with mathcing data on success, NULL if
 * no matches or on error.
 */
DListVoidNode_T *DListVoidFindNode(DListVoid_T *pHead, const void *pData)
{
  dnode_t         *p, *q, *head;
  DListVoidNode_T *pNode;

  LOGDIAG4CALL(_TPTR(pHead), _TPTR(pData));

  CHKPTR(pHead, NULL);

  if( pHead->m_fnDataCmp == NULL )
  {
    return NULL;
  }

  head = &(pHead->m_dnodeHead);

  dlist_for_each_safe(p, q, head)
  {
    pNode = DLISTVOID_ENTRY(p);
    if( !pHead->m_fnDataCmp(pData, pNode->m_pData) )
    {
      return pNode;
    }
  }
  return NULL;
}

/*!
 * \brief Traverse dlist and print.
 *
 * \param pHead     Pointer to dlist (head).
 * \param fnDataPr  User data printing fucntion.
 * \param fp        Output file pointer
 */
void DListVoidPrint(DListVoid_T *pHead, DListVoidFuncDataPrint_T fnDataPr,
                    FILE *fp)
{
  dnode_t         *p, *q, *head;
  DListVoidNode_T *pNode;
  int              n;

  head = &(pHead->m_dnodeHead);
  n = 0;

  dlist_for_each_safe(p, q, head)
  {
    pNode = DLISTVOID_ENTRY(p);
    fprintf(fp, "Node %2d: ", n);
    fnDataPr(fp, pNode->m_pData);
    fprintf(fp, "\n");
    ++n;
  }
}


//-----------------------------------------------------------------------------
// Unit Test
//-----------------------------------------------------------------------------

#ifdef unittestDListVoid

#define UT_APP(p, s) DListVoidAppend(p, (void *)s, (size_t)(strlen(s)+1))
#define UT_PRT(p)    DListVoidPrint(p, UTDListVoidTestPrintCallback, fp)

static void UTDListVoidTestPrintCallback(FILE *fp, void *pData)
{
  fprintf(fp, "'%s'", (char *)pData);
}

void UTDListVoidTest()
{
  const char      *sTest = "DListVoidTest";
  FILE            *fp = stdout;
  DListVoid_T *pHead;
  DListVoidNode_T *pSav0, *pSav1, *pSav2;
  DListVoidIter_T  iter;

  pHead = DListVoidNew(DListDataBaseByValue, NULL);

  fprintf(fp, "%s: Appending apples, bananas, cherries, and dates\n", sTest); 
  pSav0 = UT_APP(pHead, "apples");
  pSav1 = UT_APP(pHead, "bananas");
  UT_APP(pHead, "cherries");
  pSav2 = UT_APP(pHead, "dates");

  UT_PRT(pHead);

  fprintf(fp, "%s: Deleting bananas\n", sTest); 
  DListVoidDeleteNode(pHead, pSav1);
  UT_PRT(pHead);

  fprintf(fp, "%s: Deleting apples and dates\n", sTest); 
  DListVoidDeleteNode(pHead, pSav0);
  DListVoidDeleteNode(pHead, pSav2);
  UT_PRT(pHead);

  fprintf(fp, "%s: Appending eggplant, fennel, and grapes\n", sTest); 
  UT_APP(pHead, "eggplant");
  UT_APP(pHead, "fennel");
  UT_APP(pHead, "grapes");
  UT_PRT(pHead);

  fprintf(fp, "%s: Testing iterator\n", sTest); 
  for(pSav0=DListVoidIterFirst(pHead, &iter); pSav0 != NULL; 
      pSav0=DListVoidIterNext(&iter))
  {
    fprintf(fp, "%s: node data = '%s'\n", 
        sTest, (char *)DListVoidGetData(pSav0));
  }

  fprintf(fp, "%s: Deleting all\n", sTest);
  DListVoidDeleteAllNodes(pHead);
}

#endif // unittestDListVoid
