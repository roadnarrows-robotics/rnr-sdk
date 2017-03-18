////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// Library:   librnr
//
// File:      hash.h
//
/*! \file
 *
 * $LastChangedDate: 2010-03-24 10:19:36 -0600 (Wed, 24 Mar 2010) $
 * $Rev: 307 $
 *
 * \brief General purpose hash data and function declarations.
 *
 * This file has been modified from the original source (see below).
 *
 * \sa 
 * \ref example_hash under "Related Pages" for an example usage of hashing.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2005-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 *
 * <hr>
 * \par Original Source and Copyright:
 *
 * \par Original Author:
 * Kaz Kylheku (kaz@ashi.footprints.net)
 *
 * \par Original Copyright:
 * (C) 1997
 *
 * \par Original Header:
 * See "Original Source Header EULA" in source file.
 *
 * <hr>
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
// Original Source Header EULA:
//
// Hash Table Data Type
// Copyright (C) 1997 Kaz Kylheku <kaz@ashi.footprints.net>
//
// Free Software License:
//
// All rights are reserved by the author, with the following exceptions:
// Permission is granted to freely reproduce and distribute this software,
// possibly in exchange for a fee, provided that this copyright notice appears
// intact. Permission is also granted to adapt this software to produce
// derivative works, as long as the modified versions carry this copyright
// notice and additional notices stating that the work has been modified.
// The copyright extends to translations of this work into other languages,
// including machine languages. 
//
// $Id: hash.h,v 1.1 1999/11/05 00:22:34 jtravis Exp $
// $Name:  $
//
////////////////////////////////////////////////////////////////////////////////

#ifndef _HASH_H
#define _HASH_H

#include <limits.h>

#ifdef KAZLIB_SIDEEFFECT_DEBUG
#include "sfx.h"
#endif

#include "rnr/rnrconfig.h"

/*!
 * \brief Hash implementation (original source had different hash configs,
 * we have one)
 */
#define HASH_IMPLEMENTATION

/*!
 * \brief Maximum maximum hash table size
 */
#define HASHCOUNT_T_MAX ULONG_MAX

/*!
 * \brief Maximum hashed value (a Mersenne number 2^N-1)
 */
#define HASH_VAL_T_MAX ULONG_MAX


/*
 * Blurb for inclusion into C++ translation units
 */
C_DECLS_BEGIN

typedef unsigned long hashcount_t;    ///< maximum hash table size type

typedef unsigned long hash_val_t;     ///< hashed value return type

/*!
 * \brief Minimum and initial size of hash table.
 *
 * \warning Must be a power of two.
 */
#ifndef HASH_MIN_SIZE
#define HASH_MIN_SIZE  ((hashcount_t)4)
#endif

/*!
 * \brief Maximum size of hash table.
 */
#ifndef HASH_MAX_SIZE
#define HASH_MAX_SIZE  ((hashcount_t)HASHCOUNT_T_MAX)
#endif

/*!
 * \brief Hash chain node structure.
 *
 * \par Notes:
 * \b 1.
 *    This preprocessing directive is for debugging purposes.  The effect is
 *    that if the preprocessor symbol KAZLIB_OPAQUE_DEBUG is defined prior to
 *    the inclusion of this header,  then the structure shall be declared as
 *    having the single member int __OPAQUE__.   This way, any attempts by the
 *    client code to violate the principles of information hiding (by accessing
 *    the structure directly) can be diagnosed at translation time. However,
 *    note the resulting compiled unit is not suitable for linking.\n
 * \b 2.
 *    This is a pointer to the next node in the chain. In the last node of a
 *    chain, this pointer is null.\n
 * \b 3.
 *    This is a back-pointer to the primary pointer to this node.  The primary
 *    pointer is the previous node's next pointer to this node. If there is no
 *    previous node, the primary pointer is the pointer that resides in the
 *    hash table. This back-pointer lets us handle deletions easily without
 *    searching the chain.\n
 * \b 4.
 *    The key is a pointer to some user supplied data that contains a unique
 *    identifier for each hash node in a given table. The interpretation of
 *    the data is up to the user. When creating or initializing a hash table,
 *    the user must supply a pointer to a function for comparing two keys,
 *    and a pointer to a function for hashing a key into a numeric value.\n
 * \b 5.
 *    The value is a user-supplied pointer to void which may refer to
 *    any data object. It is not interpreted in any way by the hashing
 *    module.\n
 * \b 6.
 *    The hashed key is stored in each node so that we don't have to rehash
 *    each key when the table must grow or shrink.
 */
typedef struct hnode_t
{
#if defined(HASH_IMPLEMENTATION) || !defined(KAZLIB_OPAQUE_DEBUG)  ///<Note 1
  struct hnode_t  *next;    ///< Note 2
  struct hnode_t **pself;   ///< Note 3
  void            *key;     ///< Note 4
  void            *data;    ///< Note 5
  hash_val_t       hkey;    ///< Note 6
#else
  int OPAQUE;
#endif
} hnode_t;

/*!
 * \brief User comparison function override type.
 *
 * A comparison function takes two keys and produces a value of -1 if the left
 * key is less than the right key, a value of 0 if the keys are equal,
 * and a value of 1 if the left key is greater than the right key.
 *
 * \par Default:
 * Built-in wrapper to strcmp().
 *
 * \param key1  Key 1
 * \param key2  Key 2
 *
 * \sa hash_table_create()
 */
typedef int (*hash_comp_t)(const void *key1, const void *key2);

/*!
 * \brief User hashing function type.
 *
 * The hashing function performs some computation on a key and produces an
 * integral value of type hash_val_t based on that key. For best results, the
 * function should have a good randomness properties in *all* significant bits
 * over the set of keys that are being inserted into a given hash table. In
 * particular, the most significant bits of hash_val_t are most significant to
 * the hash module. Only as the hash table expands are less significant bits
 * examined. Thus a function that has good distribution in its upper bits but
 * not lower is preferrable to one that has poor distribution in the upper bits
 * but not the lower ones.
 *
 * \par Default:
 * Built-in hashing function that hashes over string keys.
 *
 * \param key Key to hash.
 *
 * \sa hash_table_create()
 */
typedef hash_val_t (*hash_fun_t)(const void *key);

/*!
 * \brief User node data deallocator function.
 *
 * \par Default:
 * NULL (no user key and data are deallocated).
 *
 * \param key Pointer to node key data.
 * \param key Pointer to node user data.
 *
 * \sa hash_table_create()
 */
typedef void (*hnode_data_free_t)(void *key, void *data);

/*!
 * \brief Hash table control structure
 *
 * The hash table keeps track of information about a hash table, as well as the
 * actual hash table itself.
 *
 * \par Notes:
 * \b 1.
 *    Pointer to the hash table proper. The table is an array of pointers to
 *    hash nodes (of type hnode_t). If the table is empty, every element of
 *    this table is a null pointer. A non-null entry points to the first
 *    element of a chain of nodes.\n
 * \b 2.
 *    Minimum and initial size of the hash table (must be a power of two).\n
 * \b 3.
 *    Maximum hash table size (must be >= minsize). The maximum size is the
 *    greatest number of nodes that can populate this table. If the table
 *    contains this many nodes, no more can be inserted, and the hash_isfull()
 *    function returns true.\n
 * \b 4.
 *    The low mark is a minimum population threshold, measured as a number of
 *    nodes. If the table population drops below this value, a table shrinkage
 *    will occur. Only dynamic tables are subject to this reduction.  No table
 *    will shrink beneath a certain absolute minimum number of nodes.\n
 * \b 5.
 *    The high mark is a population threshold, measured as a number of nodes,
 *    which, if exceeded, will trigger a table expansion. Only dynamic hash
 *    tables are subject to this expansion.\n
 * \b 6.
 *    This member keeps track of the size of the hash table---that is, the
 *    number of chain pointers.\n
 * \b 7.
 *    The current hash table mask. If the size of the hash table is 2^N,
 *    this value has its low N bits set to 1, and the others clear. It is used
 *    to select bits from the result of the hashing function to compute an
 *    index into the table.\n
 * \b 8.
 *    The count member maintains the number of elements that are presently
 *    in the hash table.\n
 * \b 9.
 *    This is the a pointer to the hash table's comparison function. The
 *    function is set once at initialization or creation time.\n
 * \b 10.
 *    Pointer to the table's hashing function, set once at creation or
 *    initialization time.\n
 * \b 12.
 *    User data deallocator. Default is NULL. (i.e. nothing is done to data 
 *    when hash node is deleted).\n
 * \b 13.
 *    A flag which indicates whether the table is to be dynamically resized. It
 *    is set to 1 in dynamically allocated tables, 0 in tables that are
 *    statically allocated.
 */
typedef struct hash_t
{
#if defined(HASH_IMPLEMENTATION) || !defined(KAZLIB_OPAQUE_DEBUG)
  hnode_t           **table;          ///< Note 1
  hashcount_t         minsize;        ///< Note 2
  hashcount_t         maxsize;        ///< Note 3
  hashcount_t         highmark;       ///< Note 4
  hashcount_t         lowmark;        ///< Note 5
  hashcount_t         nchains;        ///< Note 6
  hash_val_t          mask;           ///< Note 7
  hashcount_t         count;          ///< Note 8
  hash_comp_t         compare;        ///< Note 9
  hash_fun_t          hash;           ///< Note 10
  hnode_data_free_t   freenodedata;   ///< Note 11
  bool_t              dynamic;        ///< Note 12
#else
  int OPAQUE;
#endif
} hash_t;

/*!
 * \brief Hash scanner structure.
 *
 * The scanner is used for traversals of the data structure.
 *
 * \par Notes:
 * \b 1.
 *    Pointer to the hash table that is being traversed.\n
 * \b 2.
 *    Reference to the current chain in the table being traversed (the chain
 *    that contains the next node that shall be retrieved).\n
 * \b 3.
 *    Pointer to the node that will be retrieved by the subsequent call to
 *    hash_scan_next().
 */
typedef struct hscan_t
{
#if defined(HASH_IMPLEMENTATION) || !defined(KAZLIB_OPAQUE_DEBUG)
  hash_t       *hash;       ///< Note 1
  hash_val_t    chain;      ///< Note 2
  hnode_t      *next;       ///< Note 3
#else
  int OPAQUE;
#endif
} hscan_t;


//
// Prototypes
//

extern hash_t *hash_table_create(bool_t isdynamic,
                                hashcount_t minsize,
                                hashcount_t maxsize,
                                hash_comp_t compfun,
                                hash_fun_t hashfun,
                                hnode_data_free_t freedatafun);

extern void hash_table_destroy(hash_t *hash);

extern void hash_table_delete(hash_t *hash);

extern hash_t *hash_table_init(hash_t *hash,
                              hashcount_t minsize,
                              hashcount_t maxsize,
                              hash_comp_t compfun,
                              hash_fun_t hashfun,
                              hnode_data_free_t freedatafun,
                              hnode_t **table,
                              hashcount_t nchains);

extern int hash_table_verify(hash_t *hash);

extern void hash_scan_begin(hscan_t *scan, hash_t *hash);

extern hnode_t *hash_scan_next(hscan_t *scan);

extern void hash_node_insert(hash_t *hash, hnode_t *node, void *key);

extern hnode_t *hash_node_unlink(hash_t *hash, hnode_t *node);

extern hnode_t *hash_node_fast_unlink(hash_t *hash, hnode_t *node);

extern void hash_node_delete(hash_t *hash, hnode_t *node);

extern hnode_t *hash_lookup(hash_t *hash, const void *key);

extern bool_t hash_insert(hash_t *hash, void *key, void *data);

extern bool_t hash_delete(hash_t *hash, void *key);

extern void hash_set_self_verify(bool_t selfverify);

extern hnode_t *hnode_create(void *data);

extern hnode_t *hnode_init(hnode_t *node, void *data);

extern void hnode_destroy(hnode_t *node, hnode_data_free_t freedatafun);

extern void hnode_delete(hnode_t *node);


//
// Helper Macros
//

#if defined(HASH_IMPLEMENTATION) || !defined(KAZLIB_OPAQUE_DEBUG)

/*!
 * \brief Returns true if table is full, false otherwise.
 * \param H Hash table.
 */
#define hash_isfull(H) ((H)->count == (H)->maxcount)

/*!
 * \brief Returns true if table is empty, false otherwise.
 * \param H Hash table.
 */
#define hash_isempty(H) ((H)->count == 0)

/*!
 * \brief Returns number of entires in hash table.
 * \param H Hash table.
 */
#define hash_count(H) ((H)->count)

/*!
 * \brief Returns size of hash table.
 * \param H Hash table.
 */
#define hash_size(H) ((H)->nchains)

/*!
 * \brief Get hash node user data.
 * \param N Hash node.
 */
#define hnode_get(N) ((N)->data)

/*!
 * \brief Get hash node hash key.
 * \param N Hash node.
 */
#define hnode_getkey(N) ((N)->key)

/*!
 * \brief Pet hash node user data.
 * \param N Hash node.
 * \param V User data.
 */
#define hnode_put(N, V) ((N)->data = (V))

#endif // !HASH_IMPLEMENTATION

C_DECLS_END


#endif // _HASH_H
