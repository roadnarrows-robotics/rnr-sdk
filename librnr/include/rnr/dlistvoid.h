////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// Library:   librnr
//
// File:      dlistvoid.h
//
/*! \file
 *
 * $LastChangedDate: 2010-03-24 10:19:36 -0600 (Wed, 24 Mar 2010) $
 * $Rev: 307 $
 *
 * \brief Doubly linked list (dlist) of data pointers \#defines, types,
 * and declarations.
 *
 * This file is configured to support a "poor man's" C version of class
 * inheritance and templating.
 *
 * The "base" set of dlist data types and functions are DListVoid\<<em>x</em>\>
 * named definitions with data types returning void*. These definitions are
 * defined in librnr.
 *
 * A "derived" dlist can be defined using two \#define's 
 * (usually in a header file). Follow these \#defines with the inclusion of
 * the this header file:
 * <dl>
 * <dt> \ref DLIST_DNAME </dt>
 *  <dd>
 *    The dlist derived namespace DList\<\ref DLIST_DNAME\>\<<em>x</em>\>.
 *  </dd>
 * <dt> \ref DLIST_DTYPE </dt>
 *  <dd> The dlist derived data type. </dd>
 * </dl>
 *
 * Multiple dlist derived types and the base type may be used in the same C
 * source.
 *
 * \sa 
 * dliststr.h for librnr string example. \n
 * \ref example_dlist under "Related Pages" for another example.
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

#ifndef _DLISTVOID_H
#include <sys/types.h>
#include <stdio.h>
#endif // _DLISTVOID_H

C_DECLS_BEGIN


//-----------------------------------------------------------------------------
// Base and Derived DList Controls
//-----------------------------------------------------------------------------

/*!
 * Define the dlist name (must be literal).
 *
 * If \ref DLIST_DNAME is defined then a new set of dlist data types and
 * functions will be defined, prefaced by DList\<\ref DLIST_DNAME\>.
 *
 * If \ref DLIST_DNAME is not defined, then the base set 
 * DListVoid\<<em>x</em>\> data types and functions will be used.
 */
#ifndef DLIST_DNAME
#define DLIST_DNAME Void          ///< base dlist name
#undef  _DLIST_DERIVED            ///< not a "derived" dlist
#else
#define _DLIST_DERIVED            ///< derived dlist
#endif
#define _DLIST_NAME DLIST_DNAME   ///< dlist name

/*!
 * Define the derived dlist data type (must be literal).
 *
 * If DLIST_DTYPE is defined then the data stored in the dlist and returned
 * to the user or to callbacks will be \ref DLIST_DTYPE*.
 *
 * If \ref DLIST_DTYPE is not defined, then void is used.
 */
#ifndef DLIST_DTYPE
#define DLIST_DTYPE   void          ///< base dlist data type
#endif
#define _DLIST_TYPE   DLIST_DTYPE   ///< dlist data type

/*!
 * Useful string concatenation macros.
 */
#undef _CONCAT
#undef _CONCAT_
#define _CONCAT_(x, y)  x ## y            ///< build concatenation operator
#define _CONCAT(x, y)   _CONCAT_(x, y)    ///< now concatenate

/*!
 * \brief DList (base or derived) definition preface
 */
#undef _DLIST_DEFPREFACE
#define _DLIST_DEFPREFACE   _CONCAT(DList, _DLIST_NAME)

/*!
 * \brief DList (base or derived) definition name
 */
#undef _DLIST_DEF
#define _DLIST_DEF(name) _CONCAT(_DLIST_DEFPREFACE, name)

/*!
 * DList (base or derived) node types.
 */
#define _DLIST_HEAD       _DLIST_DEF(_T)            ///< dlist head
#define _DLIST_NODE       _DLIST_DEF(Node_T)        ///< dlist node
#define _DLIST_ITER       _DLIST_DEF(Iter_T)        ///< dlist iterator


//-----------------------------------------------------------------------------
// Base Data Types
//-----------------------------------------------------------------------------

#ifndef _DLISTVOID_H

/*!
 * \brief Special dlist index value to insert before end-of-list.
 */
#undef  DLIST_INSERT_AT_END
#define DLIST_INSERT_AT_END   (-1)

/*!
 * \brief Base: Node data comparator callback function type. 
 *
 * \return Returns <0, 0, or >0 if pData1 is less than, equal, or greater than
 * pData2, respectively.
 */
typedef int (*DListVoidFuncDataCmp_T)(const void *, const void *);

/*!
 * \brief Base: Node data delete callback function type. 
 */
typedef void (*DListVoidFuncDataDelete_T)(void *);

/*!
 * \brief Base: Print node data callback function type.
 * \sa DListVoidPrint()
 */
typedef void (*DListVoidFuncDataPrint_T)(FILE *, void *);

/*!
 * Head and node stucture types.
 */
typedef struct dlistvoid_head_t DListVoid_T;        ///< base dlist head
typedef struct dlistvoid_node_t DListVoidNode_T;    ///< base dlist node

/*!
 * \brief Base dlist iterator structure.
 *
 * \note void* is used instead of dnode_t* to hide implementation details but
 *       to also allow calling functions to define the iterator.
 */
typedef struct
{
  DListVoid_T   *m_pHead; ///< pointer to dlistvoid head
  void          *m_pThis; ///< pointer to current dnode
  void          *m_pNext; ///< pointer to next dnode
} DListVoidIter_T;

#endif // _DLISTVOID_H


//-----------------------------------------------------------------------------
// Derived Data Types
//-----------------------------------------------------------------------------

#ifdef _DLIST_DERIVED

/*!
 * \brief Node data comparator callback function type. 
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>FuncDataCmp_T
 */
typedef int (*_DLIST_DEF(FuncDataCmp_T))(const _DLIST_TYPE *,
                                         const _DLIST_TYPE *);

/*!
 * \brief Node data delete callback function type. 
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>FuncDataDelete_T
 */
typedef void (*_DLIST_DEF(FuncDataDelete_T))(_DLIST_TYPE *);

/*!
 * \brief Print node data callback function type.
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>FuncDataPrint_T
 */
typedef void (*_DLIST_DEF(FuncDataPrint_T))(FILE *, _DLIST_TYPE *);

/*!
 * Head, node, and iterator stucture types.
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>_T \n
 * DList\<\ref DLIST_DNAME\>Node_T \n
 * DList\<\ref DLIST_DNAME\>Iter_T
 */
typedef DListVoid_T       _DLIST_HEAD;    ///< dlist head
typedef DListVoidNode_T   _DLIST_NODE;    ///< dlist node
typedef DListVoidIter_T   _DLIST_ITER;    ///< dlist iterator

#endif // _DLIST_DERIVED


//-----------------------------------------------------------------------------
// Base Prototypes
//-----------------------------------------------------------------------------

#ifndef _DLISTVOID_H

// Base
extern DListVoid_T *DListVoidNew(DListVoidFuncDataCmp_T fnDataCmp,
                                DListVoidFuncDataDelete_T fnDataDelete);

// Base
extern void DListVoidDelete(DListVoid_T *pHead);

// Base
extern void DListVoidDeleteAllNodes(DListVoid_T *pHead);

// Base
extern void DListVoidDeleteNode(DListVoid_T *pHead, DListVoidNode_T *pNode);

// Base
extern void DListVoidDeleteNodeAt(DListVoid_T *pHead, int iIndex);

// Base
extern DListVoidNode_T *DListVoidUnlinkNodeAt(DListVoid_T *pHead, int iIndex);

// Base
extern void *DListVoidUnlinkDataAt(DListVoid_T *pHead, int iIndex);

// Base
extern DListVoidNode_T *DListVoidAppend(DListVoid_T *pHead, void *pData);

// Base
extern DListVoidNode_T *DListVoidPrepend(DListVoid_T *pHead, void *pData);

// Base
extern DListVoidNode_T *DListVoidInsert(DListVoid_T *pHead,
                                        int iIndex,
                                        void *pData);

// Base
extern void *DListVoidGetNodeAt(DListVoid_T *pHead, int iIndex);

// Base
extern void *DListVoidGetData(DListVoidNode_T *pNode);

// Base
extern void *DListVoidGetDataAt(DListVoid_T *pHead, int iIndex);

// Base
extern int DListVoidCount(DListVoid_T *pHead);

// Base
extern DListVoidNode_T *DListVoidIterNodeFirst(DListVoid_T *pHead, 
                                        DListVoidIter_T *pIter);

// Base
extern DListVoidNode_T *DListVoidIterNodeNext(DListVoidIter_T *pIter);

// Base
extern DListVoidNode_T *DListVoidIterNodeLast(DListVoid_T *pHead, 
                                        DListVoidIter_T *pIter);

// Base
extern DListVoidNode_T *DListVoidIterNodePrev(DListVoidIter_T *pIter);

// Base
extern DListVoidNode_T *DListVoidFindNode(DListVoid_T *pHead,
                                        const void *pData);

// Base
extern void DListVoidPrint(DListVoid_T *pHead,
                            DListVoidFuncDataPrint_T fnDataPr,
                            FILE *fp);

/*!
 *
 * \brief Attach user data to a new node and push on dlist stack.
 *
 * The dlist can serve as a LIFO stack (last in, first out) by using the 
 * push and pop operators.
 *
 * \param pHead   Pointer to dlist (head).
 * \param pData   Pointer to user (allocated) data.
 *
 * \return Returns pointer to new node on success, NULL on failure.
 */
static inline DListVoidNode_T *DListVoidPush(DListVoid_T *pHead,
                                            void *pData)
{
  return DListVoidPrepend(pHead, pData);
}

/*!
 * \brief Unlink node at top of dlist stack.
 *
 * The dlist can serve as a LIFO stack (last in, first out) by using the 
 * push and pop operators.
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>Pop()
 *
 * \param pHead   Pointer to dlist (head).
 *
 * \return
 * Returns void* to popped user data on success. \n
 * NULL on empty stack or on failure.
 */
static inline void *DListVoidPop(DListVoid_T *pHead)
{
  return DListVoidUnlinkDataAt(pHead, 0);
}

/*!
 * \brief Attach user data to a new node and queue on dlist queue.
 *
 * The dlist can serve as a FIFO queue (first in, first out) by using the 
 * queue and dequeue operators.
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>Queue()
 *
 * \param pHead   Pointer to dlist (head).
 * \param pData   Pointer to user (allocated) data.
 *
 * \return Returns pointer to new node on success, NULL on failure.
 */
static inline DListVoidNode_T *DListVoidQueue(DListVoid_T *pHead,
                                             void *pData)
{
  return DListVoidAppend(pHead, pData);
}

/*!
 * \brief Attach user data to a new node and queue on dlist queue.
 * \brief Unlink node at the front of dlist queue.
 *
 * The dlist can serve as a FIFO queue (first in, first out) by using the 
 * queue and dequeue operators.
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>Dequeue()
 *
 * \param pHead   Pointer to dlist (head).
 *
 * Returns pointer to dequeued node on success. \n
 * NULL on empty queue or on failure.
 */
static inline void *DListVoidDequeue(DListVoid_T *pHead)
{
  return DListVoidUnlinkDataAt(pHead, 0);
}

/*!
 * \brief Initialize dlist iterator and return the data in first node in dlist.
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>IterDataFirst()
 *
 * \param pHead   Pointer to dlist (head).
 * \param pIter   Pointer to iterator.
 *
 * \return
 * Returns data in first node in dlist on success. \n
 * NULL if dlist empty or on error.
 */
static inline void *DListVoidIterDataFirst(DListVoid_T *pHead,
                                                     _DLIST_ITER *pIter)
{
  return( DListVoidGetData(DListVoidIterNodeFirst(pHead, pIter)) );
}

/*!
 * \brief Get the data in the next iterated node in dlist.
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>IterDataNext()
 *
 * \param pIter   Pointer to initialized iterator.
 *
 * \return
 * Returns void* to popped user data on success. \n
 * NULL if if at end of list or on error.
 */
static inline void *DListVoidIterDataNext(_DLIST_ITER *pIter)
{
  return( DListVoidGetData(DListVoidIterNodeNext(pIter)) );
}

#endif // _DLISTVOID_H


//-----------------------------------------------------------------------------
// Derived Extensions
//-----------------------------------------------------------------------------

#ifdef _DLIST_DERIVED

/*!
 * \brief Derived: Allocate and initialize new empty derived dlist.
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>New()
 *
 * \param fnDataCmp     User-supplied data comparator function.
 *                      NULL will disable some functions such as searches.
 * \param fnDataDelete  User-supplied data deallocator.
 *                      NULL will cause user data not to be deleted. 
 *
 * \return Returns pointer to new dlist (head) on success.
 */
static inline _DLIST_HEAD *_DLIST_DEF(New)(_DLIST_DEF(FuncDataCmp_T) fnDataCmp,
                                     _DLIST_DEF(FuncDataDelete_T) fnDataDelete)
{
  return (_DLIST_HEAD *)DListVoidNew((DListVoidFuncDataCmp_T)fnDataCmp,
                                     (DListVoidFuncDataDelete_T)fnDataDelete);
}

/*!
 * \brief Derived: Delete entire dlist.
 *
 * All user data are deleted with a per node callback to the user-supplied
 * data delete function, if any. All nodes in the dlist and the dlist itself
 * will be deleted. The dlist pointer will no longer be valid after this call.
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>Delete()
 *
 * \param pHead   Pointer to dlist (head). 
 */
static inline void _DLIST_DEF(Delete)(_DLIST_HEAD *pHead)
{
  DListVoidDelete(pHead);
}

/*!
 * \brief Derived: Unlink and delete all nodes and data from dlist,
 * leaving dlist empty.
 *
 * User data are deleted with a per node callback to the user-supplied
 * data delete function, if any.
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>DeleteAllNodes()
 *
 * \param pHead   Pointer to dlist (head). 
 */
static inline void _DLIST_DEF(DeleteAllNodes)(_DLIST_HEAD *pHead)
{
  DListVoidDeleteAllNodes(pHead);
}

/*!
 * \brief Derived: Unlink node from the dlist and delete the node and it's
 * attached user data.
 *
 * User data are deleted with a callback to the user-supplied data delete
 * function, if any.
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>DeleteNode()
 *
 * \param pHead  Pointer to dlist (head).
 * \param pNode  Pointer to node to be deleted.
 */
static inline void _DLIST_DEF(DeleteNode)(_DLIST_HEAD *pHead,
                                          _DLIST_NODE *pNode)
{
  DListVoidDeleteNode(pHead, pNode);
}

/*!
 * \brief Derived: Unlink the node from the dlist at the given index and delete
 * it and it's attached user data.
 *
 * User data are deleted with a callback to the user-supplied data delete
 * function, if any.
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>DeleteNodeAt()
 *
 * \param pHead  Pointer to dlist (head).
 * \param iIndex Index to dlist entry to delete.
 */
static inline void _DLIST_DEF(DeleteNodeAt)(_DLIST_HEAD *pHead, int iIndex)
{
  DListVoidDeleteNodeAt(pHead, iIndex);
}

/*!
 * \brief nlink the node from the dlist at the given index.
 *
 * The node and it's attached data are are deleted.
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>UnlinkNodeAt()
 *
 * \param pHead  Pointer to dlist (head).
 * \param iIndex Index to dlist entry to unlink.
 *
 * \return Returns pointer to unlinked node on success, NULL on failure.
 */
static inline _DLIST_NODE *_DLIST_DEF(UnlinkNodeAt)(_DLIST_HEAD *pHead,
                                                    int iIndex)
{
  return (_DLIST_NODE *)DListVoidUnlinkNodeAt(pHead, iIndex);
}

/*!
 * \brief Unlink the user data from the dlist at the given index.
 *
 * The node is deleted, but not the attached data.
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>UnlinkDataAt()
 *
 * \param pHead  Pointer to dlist (head).
 * \param iIndex Index to dlist entry to unlink.
 *
 * \return Returns DLIST_DTYPE* to user data on success, NULL on failure.
 */
static inline _DLIST_TYPE *_DLIST_DEF(UnlinkDataAt)(_DLIST_HEAD *pHead,
                                                    int iIndex)
{
  return (_DLIST_TYPE *)DListVoidUnlinkDataAt(pHead, iIndex);
}

/*!
 * \brief Derived: Attach user data to a new node and link to the end of dlist.
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>Append()
 *
 * \param pHead Pointer to dlist (head).
 * \param pData Pointer to user (allocated) string.
 *
 * \return Returns pointer to new node on success, NULL on failure.
 */
static inline _DLIST_NODE *_DLIST_DEF(Append)(_DLIST_HEAD *pHead,
                                              _DLIST_TYPE *pData)
{
  return (_DLIST_NODE *)DListVoidAppend(pHead, pData);
}

/*!
 * \brief Attach the user data to a new node and link to the start of dlist.
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>Prepend()
 *
 * \param pHead Pointer to dlist (head).
 * \param pData Pointer to user (allocated) string.
 *
 * \return Returns pointer to new node on success, NULL on failure.
 */
static inline _DLIST_NODE *_DLIST_DEF(Prepend)(_DLIST_HEAD *pHead,
                                               _DLIST_TYPE *pData)
{
  return (_DLIST_NODE *)DListVoidPrepend(pHead, pData);
}

/*!
 * \brief Attach the user data to a new node and insert before the dlist index.
 *
 * The special index value DLIST_INSERT_AT_END will insert at end-of-list
 * (append).
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>Insert()
 *
 * \param pHead   Pointer to dlist (head).
 * \param iIndex  Index to dlist entry where insertion occurs.
 * \param pData   Pointer to user (allocated) string.
 *
 * \return Returns pointer to new node on success, NULL on failure.
 */
static inline _DLIST_NODE *_DLIST_DEF(Insert)(_DLIST_HEAD *pHead,
                                              int iIndex,
                                              _DLIST_TYPE *pData)
{
  return (_DLIST_NODE *)DListVoidInsert(pHead, iIndex, pData);
}

/*!
 * \brief Derived: Get the node at the given dlist index.
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>GetNodeAt()
 *
 * \param pHead   Pointer to dlist (head).
 * \param iIndex  Index to dlist entry to find.
 *
 * \return Returns node at index on success, NULL on failure.
 */
static inline _DLIST_NODE *_DLIST_DEF(GetNodeAt)(_DLIST_HEAD *pHead, int iIndex)
{
  return (_DLIST_NODE *)DListVoidGetNodeAt(pHead, iIndex);
}

/*!
 * \brief Derived: Get the user data from the given node.
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>GetData()
 *
 * \param pNode   Pointer to node with data.
 *
 * \return Returns DLIST_DTYPE* to user string on success, NULL on failure.
 */
static inline _DLIST_TYPE *_DLIST_DEF(GetData)(_DLIST_NODE *pNode)
{
  return (_DLIST_TYPE *)DListVoidGetData(pNode);
}

/*!
 * \brief Derived: Get the user string from the dlist at the given index.
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>GetDataAt()
 *
 * \param pHead   Pointer to dlist (head).
 * \param iIndex  Index to dlist entry.
 *
 * \return Returns char* to user string on success, NULL on failure.
 */
static inline _DLIST_TYPE *_DLIST_DEF(GetDataAt)(_DLIST_HEAD *pHead, int iIndex)
{
  return (_DLIST_TYPE *)DListVoidGetDataAt(pHead, iIndex);
}

/*!
 * \brief Derived: Get the number of nodes in dlist.
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>Count()
 *
 * \param pHead Pointer to dlist (head).
 *
 * \return Number of nodes in dlist.
 */
static inline int _DLIST_DEF(Count)(_DLIST_HEAD *pHead)
{
  return DListVoidCount(pHead);
}

/*!
 * \brief Derived: Initialize dlist iterator and return the first node in dlist.
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>IterNodeFirst()
 *
 * \param pHead   Pointer to dlist (head).
 * \param pIter   Pointer to iterator.
 *
 * \return
 * Returns first node in dlist on success.\n
 * NULL if dlist empty or on error.
 */
static inline _DLIST_NODE *_DLIST_DEF(IterNodeFirst)(_DLIST_HEAD *pHead,
                                                     _DLIST_ITER *pIter)
{
  return (_DLIST_NODE *)DListVoidIterNodeFirst(pHead, pIter);
}

/*!
 * \brief Derived: Get the next iterated node in dlist.
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>IterNodeNext()
 *
 * \param pIter   Pointer to initialized iterator.
 *
 * \return 
 * Returns next node in dlist on success.\n
 * NULL if if at end of list or on error.
 */
static inline _DLIST_NODE *_DLIST_DEF(IterNodeNext)(_DLIST_ITER *pIter)
{
  return (_DLIST_NODE *)DListVoidIterNodeNext(pIter);
}

/*!
 * \brief Derived: Initialize dlist iterator and return the last node in dlist.
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>IterNodeLast()
 *
 * \param pHead   Pointer to dlist (head).
 * \param pIter   Pointer to iterator.
 *
 * \return
 * Returns last node in dlist on success.\n
 * NULL if dlist empty or on error.
 */
static inline _DLIST_NODE *_DLIST_DEF(IterNodeLast)(_DLIST_HEAD *pHead,
                                                     _DLIST_ITER *pIter)
{
  return (_DLIST_NODE *)DListVoidIterNodeLast(pHead, pIter);
}

/*!
 * \brief Derived: Get the previous iterated node in dlist.
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>IterNodePrev()
 *
 * \param pIter   Pointer to initialized iterator.
 *
 * \return 
 * Returns previous node in dlist on success.\n
 * NULL if if at end of list or on error.
 */
static inline _DLIST_NODE *_DLIST_DEF(IterNodePrev)(_DLIST_ITER *pIter)
{
  return (_DLIST_NODE *)DListVoidIterNodePrev(pIter);
}

/*!
 * \brief Derived: Find the first node with the matching data.
 *
 * The user provided comparator function is used, if any.
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>FindNode()
 *
 * \param pHead   Pointer to dlist (head).
 * \param pData   User data to find.
 *
 * \return
 * Returns first node with mathcing data on success.\n
 * NULL if no matches or on error.
 */
static inline _DLIST_NODE *_DLIST_DEF(FindNode)(_DLIST_HEAD *pHead,
                                                const _DLIST_TYPE *pData)
{
  return (_DLIST_NODE *)DListVoidFindNode(pHead, pData);
}

/*!
 * \brief Derived: Traverse dlist and print.
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>Print()
 *
 * \param pHead     Pointer to dlist (head).
 * \param fnDataPr  User data printing fucntion.
 * \param fp        Output file pointer.
 */
static inline void _DLIST_DEF(Print)(_DLIST_HEAD *pHead,
                                    _DLIST_DEF(FuncDataPrint_T) fnDataPr,
                                    FILE *fp)
{
  DListVoidPrint(pHead, (DListVoidFuncDataPrint_T)fnDataPr, fp);
}

/*!
 *
 * \brief Attach user data to a new node and push on dlist stack.
 *
 * The dlist can serve as a LIFO stack (last in, first out) by using the 
 * push and pop operators.
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>Push()
 *
 * \param pHead   Pointer to dlist (head).
 * \param pData   Pointer to user (allocated) data.
 *
 * \return Returns pointer to new node on success, NULL on failure.
 */
static inline _DLIST_NODE *_DLIST_DEF(Push)(_DLIST_HEAD *pHead,
                                            _DLIST_TYPE *pData)
{
  return _DLIST_DEF(Prepend)(pHead, pData);
}

/*!
 * \brief Unlink node at top of dlist stack.
 *
 * The dlist can serve as a LIFO stack (last in, first out) by using the 
 * push and pop operators.
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>Pop()
 *
 * \param pHead   Pointer to dlist (head).
 *
 * \return
 * Returns void* to popped user data on success. \n
 * NULL on empty stack or on failure.
 */
static inline _DLIST_TYPE *_DLIST_DEF(Pop)(_DLIST_HEAD *pHead)
{
  return _DLIST_DEF(UnlinkDataAt)(pHead, 0);
}

/*!
 * \brief Attach user data to a new node and queue on dlist queue.
 *
 * The dlist can serve as a FIFO queue (first in, first out) by using the 
 * queue and dequeue operators.
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>Queue()
 *
 * \param pHead   Pointer to dlist (head).
 * \param pData   Pointer to user (allocated) data.
 *
 * \return Returns pointer to new node on success, NULL on failure.
 */
static inline _DLIST_NODE *_DLIST_DEF(Queue)(_DLIST_HEAD *pHead,
                                             _DLIST_TYPE *pData)
{
  return _DLIST_DEF(Append)(pHead, pData);
}

/*!
 * \brief Attach user data to a new node and queue on dlist queue.
 * \brief Unlink node at the front of dlist queue.
 *
 * The dlist can serve as a FIFO queue (first in, first out) by using the 
 * queue and dequeue operators.
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>Dequeue()
 *
 * \param pHead   Pointer to dlist (head).
 *
 * Returns pointer to dequeued node on success. \n
 * NULL on empty queue or on failure.
 */
static inline _DLIST_TYPE *_DLIST_DEF(Dequeue)(_DLIST_HEAD *pHead)
{
  return _DLIST_DEF(UnlinkDataAt)(pHead, 0);
}

/*!
 * \brief Initialize dlist iterator and return the data in first node in dlist.
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>IterDataFirst()
 *
 * \param pHead   Pointer to dlist (head).
 * \param pIter   Pointer to iterator.
 *
 * \return
 * Returns data in first node in dlist on success. \n
 * NULL if dlist empty or on error.
 */
static inline _DLIST_TYPE *_DLIST_DEF(IterDataFirst)(_DLIST_HEAD *pHead,
                                                     _DLIST_ITER *pIter)
{
  return( _DLIST_DEF(GetData)(_DLIST_DEF(IterNodeFirst)(pHead, pIter)) );
}

/*!
 * \brief Get the data in the next iterated node in dlist.
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>IterDataNext()
 *
 * \param pIter   Pointer to initialized iterator.
 *
 * \return
 * Returns data in the next node on success. \n
 * NULL if if at end of list or on error.
 */
static inline _DLIST_TYPE *_DLIST_DEF(IterDataNext)(_DLIST_ITER *pIter)
{
  return( _DLIST_DEF(GetData)(_DLIST_DEF(IterNodeNext)(pIter)) );
}

/*!
 * \brief Initialize dlist iterator and return the data in last node in dlist.
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>IterDataLast()
 *
 * \param pHead   Pointer to dlist (head).
 * \param pIter   Pointer to iterator.
 *
 * \return
 * Returns data in last node in dlist on success. \n
 * NULL if dlist empty or on error.
 */
static inline _DLIST_TYPE *_DLIST_DEF(IterDataLast)(_DLIST_HEAD *pHead,
                                                     _DLIST_ITER *pIter)
{
  return( _DLIST_DEF(GetData)(_DLIST_DEF(IterNodeLast)(pHead, pIter)) );
}

/*!
 * \brief Get the data in the previous iterated node in dlist.
 *
 * \par Name:
 * DList\<\ref DLIST_DNAME\>IterDataPrev()
 *
 * \param pIter   Pointer to initialized iterator.
 *
 * \return
 * Returns data in the previous node on success. \n
 * NULL if if at end of list or on error.
 */
static inline _DLIST_TYPE *_DLIST_DEF(IterDataPrev)(_DLIST_ITER *pIter)
{
  return( _DLIST_DEF(GetData)(_DLIST_DEF(IterNodePrev)(pIter)) );
}

#endif // _DLIST_DERIVED

C_DECLS_END


//-----------------------------------------------------------------------------
// Fix Up Defines
//-----------------------------------------------------------------------------

/*!
 * Undefine macros to allow redefinition.
 */
#undef DLIST_DNAME
#undef DLIST_DTYPE
#undef _DLIST_DERIVED
#undef _DLIST_NAME
#undef _DLIST_TYPE
#undef _DLIST_HEAD
#undef _DLIST_NODE
#undef _DLIST_ITER

#define _DLISTVOID_H    ///< include base declaration only once
