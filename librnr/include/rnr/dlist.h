////////////////////////////////////////////////////////////////////////////////
/*! \file
 *
 * \brief Doubly linked list base implementation. 
 *
 * The dlists are intended to be embedded within the parent structure of 
 * a specific type. Higher level constructs using the dlist base can build
 * implementations doubly-linked lists, queues, stacks, circular buffers,
 * and binary trees with data of any type.
 *
 * Some of the internal functions ("_dlist_xxx") are useful when manipulating 
 * whole lists rather than single entries, as sometimes we already know the 
 * next/prev entries and we can generate better code by using them directly
 * rather than using the generic single-entry routines.
 *
 * This file has been modified from the original source (see below).
 *
 * \pkgsynopsis
 * RoadNarrows Robotics Common Library 1
 *
 * \pkgcomponent{Library}
 * librnr
 *
 * \pkgfile{rnr/dlist.h}
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
 *
 * <hr>
 * \par Original Source and Copyright
 *
 * \par Original Author
 * Linux 2.6 
 *
 * \par Original Source Header
 * Linux header file <linux/list.h>
 *
 * <hr>
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _RNR_DLIST_H
#define _RNR_DLIST_H

C_DECLS_BEGIN

/*!
 * \brief The doubly linked node structure
 */
typedef struct dlist_node 
{
  struct dlist_node *next;  ///< next node in dlist
  struct dlist_node *prev;  ///< previous node in dlist
} dnode_t;


/*!
 * \brief Initialize node links (pointer version).
 * \param pnode pointer to dnode_t
 */
#define DLIST_PNODE_INIT_DATA(pnode)  (pnode)->next = (pnode)->prev = pnode

/*!
 * \brief Initialize node links.
 * \param node dnode_t
 */
#define DLIST_NODE_INIT(node)         DLIST_PNODE_INIT_DATA(&(node))

/*!
 * \brief Initialize node links (pointer version).
 * \param pnode pointer to dnode_t
 */
#define DLIST_PNODE_INIT(pnode)       DLIST_PNODE_INIT_DATA(pnode)

/*!
 * \brief insert a new entry between two known consecutive entries
 *
 * \param node  the new node to be inserted
 * \param prev  previous node to new node
 * \param next  next node to new node
 *
 * \note This is only for internal list manipulation where we know the prev/next
 *       entries already!
 */
static inline void _dlist_add(dnode_t *node, dnode_t *prev, dnode_t *next)
{
  next->prev = node;
  node->next = next;
  node->prev = prev;
  prev->next = node;
}

/*!
 * \brief prepend new node to head of list
 *
 * Insert a new node after the specified head. This is good for implementing 
 * stacks.
 *
 * \param node  the new node to be inserted at head of the list
 * \param head  list head
 */
static inline void dlist_prepend(dnode_t *node, dnode_t *head)
{
  _dlist_add(node, head, head->next);
}

/*!
 * \brief append a new node to the in of the list
 *
 * Insert a new node before the specified head. This is useful for implementing
 * queues.
 *
 * \param node  the new node to be inserted at end of the list
 * \param head  list head
 */
static inline void dlist_append(dnode_t *node, dnode_t *head)
{
  _dlist_add(node, head->prev, head);
}

/*!
 * \brief  delete the node(s) between the prev/next nodes.
 *
 * \param prev  previous node to link to next
 * \param next  next node to link with previous
 *
 * \note This is only for internal list manipulation where we know the prev/next
 *       entries already!
 */
static inline void _dlist_del(dnode_t *prev, dnode_t *next)
{
  next->prev = prev;
  prev->next = next;
}

/*!
 * \brief deletes node from list
 *
 * \param node  the node to delete from the list.
 *
 * Note: Deleted node becomes an orphan (i.e. empty) head node.
 */
static inline void dlist_delete(dnode_t *node)
{
  _dlist_del(node->prev, node->next);
  DLIST_PNODE_INIT(node);
}

/*!
 * \brief tests whether a list is empty
 *
 * \param head list head
 *
 * \return non-zero if empty, 0 otherwise
 */
static inline int dlist_is_empty(dnode_t *head)
{
  return head->next == head;
}

/*!
 * \brief prepend new list at head of given list
 *
 * \param list  the new list to add
 * \param head  list head holding newly spliced lists
 *
 * \note The list prev/next pointers are not altered so it is still a valid
 *       list.
 */
static inline void dlist_splice(dnode_t *list, dnode_t *head)
{
  if( !dlist_is_empty(list) )
  {
    dnode_t *first = list->next;
    dnode_t *last  = list->prev;
    dnode_t *at    = head->next;

    first->prev = head;
    head->next  = first;

    last->next = at;
    at->prev   = last;
  }
}

/*!
 * \brief get the pointer to the embedding struct for ptr
 *
 * \param ptr     the &dnode_t pointer.
 * \param type    the name of the struct the dnode_t is embedded in.
 * \param member  the name of the member in type that is the dnode_t.
 *
 * \return pointer to embedding structure
 */
#define dlist_entry(ptr, type, member) \
  ((type *)((char *)(ptr)-(unsigned long)(&((type *)0)->member)))

/*!
 * \brief iterate over a list
 *
 * \param pos   the dnode_t pointer to use as a loop counter
 * \param head  the head of list to iterate over
 */
#define dlist_for_each(pos, head) \
  for(pos = (head)->next; pos != (head); pos = pos->next)
          
/*!
 * \brief   safely iterate over a list 
 *
 * \param pos   the dnode_t pointer to be use as a loop counter
 * \param npos  another dnode_t pointer to use as temporary storage
 * \param head  the head of list to iterate over
 *
 * \note (Somewhat) safe against removal of list entry. That is, node pos can
 *       be deleted, etc and this interator still works. However, this (or the
 *       above interator) are not thread safe.
 */
#define dlist_for_each_safe(pos, npos, head) \
  for(pos = (head)->next, npos = pos->next; pos != (head); \
    pos = npos, npos = pos->next)

C_DECLS_END


#endif // _RNR_DLIST_H
