////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// Library:   librnr
//
// File:      hash.c
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
 * Exceptons map to assert() calls that terminate the calling application.
 *
 * \todo
 * Convert the assert() error handling to appropriate return codes and librnr
 * error handling.
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

#include <stdlib.h>
#include <stddef.h>
#include <assert.h>
#include <string.h>

#include "rnr/rnrconfig.h"
#include "rnr/new.h"
#include "rnr/log.h"

#include "rnr/hash.h"

// ---------------------------------------------------------------------------
// Private Interface
// ---------------------------------------------------------------------------

//
// Original RCS Id and copyright strings. Note used, but keep.
//
#if 0
static const char rcsid[] = "$Id: hash.c,v 1.2 2002/03/11 00:51:44 jick Exp $";
static const char right[] = "Copyright (C) 1997 Kaz Kylheku";
#endif


/*!
 * \brief Number of bits in hashed value. Must be a power of 2.
 */
static int hash_val_t_bit = 0;

/*!
 * \brief Do [not] perform auto-verification after hash table operations.
 */
static bool_t hash_self_verify  = false;

/*!
 * \brief Compute the number of bits in the \ref hash_val_t type.
 *
 * We know that \ref hash_val_t is an unsigned integral type.
 * Thus the highest value it can hold is a
 * Mersenne number (power of two, less one). We initialize a \ref hash_val_t
 * object with this value and then shift bits out one by one while counting.
 *
 * \par Notes &amp; Steps:
 * \b 1.
 *    \ref HASH_VAL_T_MAX is a Mersenne number---one that is one less than a
 *    power of two. This means that its binary representation consists of all
 *    one bits, and hence ``val'' is initialized to all one bits.\n
 * \b 2.
 *    We reset the bit count to zero in case this function is invoked more than
 *    once.\n
 * \b 3.
 *    While bits remain in val, we increment the bit count and shift it to the
 *    right, replacing the topmost bit by zero.
 */
static void compute_bits()
{
  hash_val_t val  = HASH_VAL_T_MAX;  // Steps 1
  int bits        = 0;

  // Steps 3
  while(val)
  {
    bits++;
    val >>= 1;
  }

  hash_val_t_bit = bits;
}

/*!
 * \brief Verify whether the given argument is a power of two.
 *
 * \param arg  Argument to test.
 *
 * \return Returns true if arg is a power of 2, false otherwise.
 */
static bool_t is_power_of_two(hash_val_t arg)
{
  if( arg == 0 )
  {
    return false;
  }

  while( (arg & 0x01) == 0 )
  {
    arg >>= 1;
  }

 return arg == 1? true: false;
}

/*!
 * \brief Compute a mask for a given table size.
 *
 * \param size  Table size.
 *
 * \return Shift amount.
 */
static hash_val_t compute_mask(hashcount_t size)
{
  hash_val_t mask = size;

  assert(is_power_of_two(size));
  assert(size >= 2);

  mask /= 2;

  while( (mask & 0x01) == 0 )
  {
    mask |= (mask >> 1);
  }

  return mask;
}


/*!
 * \brief Initialize the table size values.
 *
 * The minimum size is adjusted to be a power of two and is guaranteed to be
 * at least \ref HASH_MIN_SIZE. 
 *
 * The maximum size adjusted to be >= minsize.
 *
 * \param hash      Hash table.
 * \param minsize   Desired minimum and initial size.
 * \param maxsize   Desired maximum size.
 */
static void init_hash_sizes(hash_t *hash,
                             hashcount_t minsize,
                             hashcount_t maxsize)
{
  size_t    numbits, rshift, lshift;

  // number of bits in hashcount_t
  numbits = sizeof(hashcount_t) * 8;

  // round down to the closest power of two value
  for(rshift=0, lshift=0; rshift<numbits; ++rshift)
  {
    if( minsize & 0x01 )
    {
      lshift = rshift;
    }
    minsize >>= 1;
  }
  minsize = lshift>0? (((hashcount_t)(0x01)) << lshift):
                      (hashcount_t)HASH_MIN_SIZE;

  // absolute minimum
  if( minsize < HASH_MIN_SIZE )
  {
    minsize = HASH_MIN_SIZE;
  }

  if( maxsize < minsize )
  {
    maxsize = minsize;
  }

  hash->minsize       = minsize;
  hash->maxsize       = maxsize;
  hash->lowmark       = minsize / 2;
  hash->highmark      = minsize * 2;

  LOGDIAG4("minsize=%lu, maxsize=%lu, lowmark=%lu, highmark=%lu",
      hash->minsize, hash->maxsize, hash->lowmark, hash->highmark);
}

/*!
 * \brief Default hashing function over null-terminated string keys.
 *
 * \param key Null-terminated key string.
 *
 * \return Hashed value.
 */
static hash_val_t hash_fun_default(const void *key)
{
  static unsigned long randbox[] =
  {
    0x49848f1bU, 0xe6255dbaU, 0x36da5bdcU, 0x47bf94e9U,
    0x8cbcce22U, 0x559fc06aU, 0xd268f536U, 0xe10af79aU,
    0xc1af4d69U, 0x1d2917b5U, 0xec4c304dU, 0x9ee5016cU,
    0x69232f74U, 0xfead7bb3U, 0xe9089ab6U, 0xf012f6aeU,
  };

  const unsigned char *str  = (const unsigned char *)key;
  hash_val_t           acc  = 0;

  while( *str )
  {
    acc ^= randbox[(*str + acc) & 0xf];
    acc = (acc << 1) | (acc >> 31);
    acc &= 0xffffffffU;
    acc ^= randbox[((*str++ >> 4) + acc) & 0xf];
    acc = (acc << 2) | (acc >> 30);
    acc &= 0xffffffffU;
  }
  return acc;
}

/*!
 * \brief Default hash key comparator function of null-terminated key strings.
 *
 * \param key1  Key string 1.
 * \param key2  Key string 2.
 *
 * \return
 * Returns <0, 0, >0 if key1 is lexigraphically less than key2, respectively.
 */
static int hash_comp_default(const void *key1, const void *key2)
{
  return strcmp((const char *)key1, (const char *)key2);
}

/*!
 * \brief Initialize the table of pointers to null.
 *
 * \param hash  Hash table.
 */
static void clear_table(hash_t *hash)
{
  hash_val_t i;

  for(i=0; i<hash->nchains; i++)
  {
    hash->table[i] = NULL;
  }
}

/*!
 * \brief Double the size of a dynamic table.
 *
 * This works as follows:
 *
 * Each chain splits into two adjacent chains. The shift amount increases by
 * one, exposing an additional bit of each hashed key. For each node in the
 * original chain, the value of this newly exposed bit will decide which of the
 * two new chains will receive the node: if the bit is 1, the chain with the
 * higher index will have the node, otherwise the lower chain will receive the
 * node. In this manner, the hash table will continue to function exactly as
 * before without having to rehash any of the keys.
 *
 * \par Notes &amp; Steps:
 * \b 1.
 *    Overflow check.\n
 * \b 2.
 *    The new number of chains is twice the old number of chains.\n
 * \b 3.
 *    The new mask is one bit wider than the previous, revealing a
 *    new bit in all hashed keys.\n
 * \b 4.
 *    Allocate a new table of chain pointers that is twice as large as the
 *    previous one.\n
 * \b 5.
 *    If the reallocation was successful, we perform the rest of the growth
 *    algorithm, otherwise we do nothing.\n
 * \b 6.
 *    The exposed_bit variable holds a mask with which each hashed key can be
 *    AND-ed to test the value of its newly exposed bit.\n
 * \b 7.
 *    Loop over the lower half of the table, which, at first, holds all of
 *    the chains.\n
 * \b 8.
 *    Each chain from the original table must be split into two chains.
 *    The nodes of each chain are examined in turn. The ones whose key value's
 *    newly exposed bit is 1 are removed from the chain and put into newchain
 *    (Steps 9 through 14). After this separation, the new chain is assigned
 *    into its appropriate place in the upper half of the table (Step 15).\n
 * \b 9.
 *    Since we have relocated the table of pointers, we have to fix the
 *    back-reference from the first node of each non-empty chain so it
 *    properly refers to the moved pointer.\n
 * \b 10.
 *    We loop over the even chain looking for any nodes whose exposed bit is
 *    set. Such nodes are removed from the lower-half chain and put into its
 *    upper-half sister.\n
 * \b 11.
 *    Before moving the node to the other chain, we remember what the next
 *    node is so we can coninue the loop. We have to do this because we will
 *    be overwriting the node's next pointer when we insert it to its new
 *    home.\n
 * \b 12.
 *    The next node's back pointer must be updated to skip to the previous
 *    node.\n
 * \b 13.
 *    The deleted node's back pointer must be updated to refer to the next
 *    node.\n
 * \b 14.
 *    We insert the node at the beginning of the new chain.\n
 * \b 15.
 *    Place the new chain into an upper-half slot.\n
 * \b 16.
 *    We have finished dealing with the chains and nodes. We now update
 *    the various bookeeping fields of the hash structure.
 *
 * \param hash  Hash table.
 *
 * \exception "Table Overflow" Reached system limits.
 * \exception "Verification Failed" Only if \ref hash_self_verify enabled.
 */
static void grow_table(hash_t *hash)
{
  hnode_t     **newtable;
  size_t        size;

  LOGDIAG4CALL(_TPTR(hash));

  // Step 1
  assert(2 * hash->nchains > hash->nchains);

  // Step 4
  size = sizeof(hnode_t *) * hash->nchains;
  newtable = new_overs(hash->table, size, size *2);

  // Step 5
  if( newtable )
  {
    // Step 3
    hash_val_t mask = (hash->mask << 1) | 1;

    // Step 6
    hash_val_t exposed_bit = mask ^ hash->mask;

    hash_val_t chain;

    assert(mask != hash->mask);

    // Step 7
    for(chain = 0; chain < hash->nchains; chain++)
    {
      // Step 8
      hnode_t *hptr = newtable[chain];
      hnode_t *newchain = NULL;

      // Step 9
      if(hptr)
      {
        hptr->pself = &newtable[chain];
      }

      // Step 10
      while(hptr)
      {
        if((hptr->hkey & exposed_bit))
        {
          // Step 11
          hnode_t *next = hptr->next;

          // Step 12
          if(next)
          {
            next->pself = hptr->pself;
          }

          // Step 13
          *hptr->pself = next;

          // Step 14
          hptr->next = newchain;
          if(newchain)
          {
            newchain->pself = &hptr->next;
          }
          newchain = hptr;
          hptr->pself = &newchain;
          hptr = next;
        }
        else
        {
          hptr = hptr->next;
        }
      }
      // Step 15
      newtable[chain + hash->nchains] = newchain;
      if(newchain)
      {
        newchain->pself = &newtable[chain + hash->nchains];
      }
    }

    // Step 16
    hash->table     = newtable;
    hash->mask      = mask;
    hash->nchains  *= 2;
    hash->lowmark  *= 2;
    hash->highmark *= 2;
  }

  LOGDIAG4(_VARFMT(hash->nchains, "%lu") ", " _VARFMT(hash->lowmark, "%lu") ", "
      _VARFMT(hash->highmark, "%lu") ", " _VARFMT(hash->mask, "0x%lx"),
      hash->nchains, hash->lowmark, hash->highmark, hash->mask);

  if( hash_self_verify )
  {
    assert(hash_table_verify(hash));
  }
}

/*!
 * \brief Cut a dynamic table size in half.
 *
 * This is done by folding together adjacent chains and populating the lower
 * half of the table with these chains. The chains are simply spliced together.
 * Once this is done, the whole table is reallocated to a smaller object.
 *
 * \par Notes &amp; Steps:
 * \b 1.
 *     It is illegal to have a hash table with one slot. This would mean that
 *     hash->shift is equal to hash_val_t_bit, an illegal shift value.
 *     Also, other things could go wrong, such as hash->lowmark becoming zero.\n
 * \b 2.
 *     Looping over each adjacent chain of pairs, the lo_chain is set to
 *     reference the lower-numbered member of the pair, whereas hi_chain
 *     is the index of the higher-numbered one.\n
 * \b 3.
 *     The intent here is to compute a pointer to the last node of the
 *     lower chain into the lo_tail variable. If this chain is empty,
 *     lo_tail ends up with a null value.\n
 * \b 4.
 *     If the lower chain is not empty, we have to merge chains, but only
 *     if the upper chain is also not empty. In either case, the lower chain
 *     will come first, with the upper one appended to it.\n
 * \b 5.
 *     The first part of the join is done by having the tail node of the lower
 *     chain point to the head node of the upper chain. If the upper chain
 *     is empty, this is remains a null pointer.\n
 * \b 6.
 *     If the upper chain is non-empty, we must do the additional house-keeping
 *     task of ensuring that the upper chain's first node's back-pointer
 *     references the tail node of the lower chain properly.\n
 * \b 7. (defunct)
 * \b 8.
 *     If the low chain is empty, but the high chain is not, then the
 *     high chain simply becomes the new chain.\n
 * \b 9.
 *     Otherwise if both chains are empty, then the merged chain is also
 *     empty.\n
 * \b 10.
 *     All the chain pointers are in the lower half of the table now, so
 *     we reallocate it to a smaller object. This, of course, invalidates
 *     all pointer-to-pointers which reference into the table from the
 *     first node of each chain.\n
 * \b 11.
 *     Though it's unlikely, the reallocation may fail. In this case we
 *     pretend that the table _was_ reallocated to a smaller object.\n
 * \b 12.
 *     This loop performs the housekeeping task of updating the back pointers
 *     from the first node of each chain so that they reference their 
 *     corresponding table entries.\n
 * \b 13.
 *     Finally, update the various table parameters to reflect the new size.
 *
 * \param hash  Hash table.
 *
 * \exception "Table Overflow" Reached system limits.
 * \exception "Verification Failed" Only if \ref hash_self_verify enabled.
 */
static void shrink_table(hash_t *hash)
{
  hashcount_t   chain, nchains;
  size_t        size;
  hnode_t     **newtable, *lo_tail, *lo_chain, *hi_chain;

  LOGDIAG4CALL(_TPTR(hash));

  // Step 1
  assert(hash->nchains >= 2);

  // downsize chains
  nchains = hash->nchains / 2;

  // Step 2
  for(chain = 0; chain < nchains; chain++)
  {
    lo_chain = hash->table[chain];
    hi_chain = hash->table[chain + nchains];

    // Step 3
    for(lo_tail=lo_chain; lo_tail && lo_tail->next; lo_tail=lo_tail->next)
    {
      ;
    }

    // Step 4
    if(lo_chain)
    {
      // Step 5
      lo_tail->next = hi_chain;

      // Step 6
      if(hi_chain)        
      {
        hi_chain->pself = &lo_tail->next;
      }
    }

    // Step 8
    else if(hi_chain)
    {
      hash->table[chain] = hi_chain;
    }

    // Step 9
    else
    {
      hash->table[chain] = NULL;
    }
  }

  // Step 10
  size = sizeof(hnode_t *) * nchains;             // downsized size
  newtable = new_overs(hash->table, size, size);

  // Step 11
  if(newtable)
  {
    hash->table = newtable;
  }

  // Step 12
  for(chain = 0; chain < nchains; chain++)
  {
    if(hash->table[chain])
    {
      hash->table[chain]->pself = &hash->table[chain];
    }
  }

  // Step 13
  hash->mask      >>= 1;
  hash->nchains     = nchains;
  hash->lowmark    /= 2;
  hash->highmark   /= 2;

  LOGDIAG4(_VARFMT(hash->nchains, "%lu") ", " _VARFMT(hash->lowmark, "%lu") ", "
      _VARFMT(hash->highmark, "%lu") ", " _VARFMT(hash->mask, "0x%lx"),
      hash->nchains, hash->lowmark, hash->highmark, hash->mask);

  if( hash_self_verify )
  {
    assert(hash_table_verify(hash));
  }
}


// ---------------------------------------------------------------------------
// Public Interface
// ---------------------------------------------------------------------------

/*!
 * \brief Create a dynamic hash table.
 *
 * Both the hash table structure and the table itself are dynamically allocated.
 * Furthermore, the table is extendible in that it will automatically grow as
 * its load factor increases beyond a certain threshold.
 *
 * \par Notes &amp; Steps:
 * \b 1.
 *    If the number of bits in the hash_val_t type has not been computed yet,
 *    we do so here, because this is likely to be the first function that the
 *    user calls.\n
 * \b 2.
 *    Allocate hash table.\n
 * \b 3.
 *    Initialize hash table sizes. The minsize should be a power of two.
 *    The maxsize should be >= minsize. Both values will be adjusted. The high
 *    and low marks are always set to be twice the table size and half the
 *    table size respectively. When the number of nodes in the table grows
 *    beyond the high size (beyond load factor 2), it will double in size to
 *    cut the load factor down to about about 1. If the table shrinks down to
 *    or beneath load factor 0.5, it will shrink, bringing the load up to
 *    about 1. However, the table will never shrink beneath minsize even if
 *    it's emptied.\n
 * \b 4.
 *    Allocate the table of hash chains.\n
 * \b 5.
 *    Finish initializing the hash structure and the table.
 * \b 6.
 *    This indicates that the table is dynamically allocated and dynamically
 *    resized on the fly. A table that has this value set to zero is
 *    assumed to be statically allocated and will not be resized.\n
 * \b 7.
 *    The table of chains must be properly reset to all null pointers.
 * \b 8.
 *    Verify table's correctness, if self-verification is true.
 *
 * \param isdynamic   The hash table is [not] automatically grow. If set to
 *                    false, the hash table is initialized t be fixed at 
 *                    minsize and maxsize is ignored.
 * \param minsize     Minimum and initial number of hash entries that will be
 *                    supported.
 * \param maxsize     Maximum number of hash entries that will be supported.
 * \param compfun     Hash key comparator function. NULL will use the default
 *                    strcmp() wrapper.
 * \param hashfun     Hashing function. NULL will used the default string
 *                    hashing function.
 * \param freedatafun User key and data deallocator function. NULL will cause
 *                    no user key or data to be deleted.
 *
 * \return Newly created, empty hash table.
 *
 * \exception "Verification Failed" Only if \ref hash_self_verify enabled.
 */
hash_t *hash_table_create(bool_t isdynamic,
                          hashcount_t minsize,
                          hashcount_t maxsize,
                          hash_comp_t compfun,
                          hash_fun_t hashfun,
                          hnode_data_free_t freedatafun)
{
  hash_t *hash;

  LOGDIAG4CALL(_TBOOL(isdynamic), _TULONG(minsize), _TULONG(maxsize),
      _TPTR(compfun), _TPTR(hashfun), _TPTR(freedatafun));

  // Step 1
  if( hash_val_t_bit == 0 )
  {
    compute_bits();
  }

  // Step 2
  hash = NEW(hash_t);

  // Step 3
  init_hash_sizes(hash, minsize, maxsize);

  // Step 4
  hash->table = (hnode_t **)new(sizeof(hnode_t *) * hash->minsize);

  // Step 5
  hash->nchains       = minsize;
  hash->mask          = compute_mask(hash->nchains);
  hash->count         = 0;
  hash->compare       = compfun ? compfun : hash_comp_default;
  hash->hash          = hashfun ? hashfun : hash_fun_default;
  hash->freenodedata  = freedatafun;

  // Step 6
  hash->dynamic       = isdynamic;

  // Step 7
  clear_table(hash);

  // Step 8
  if( hash_self_verify )
  {
    assert(hash_table_verify(hash));
  }

  LOGDIAG4("min=%lu, max=%lu, high=%lu, low=%lu, nchains=%lu, mask=0x%lx\n",
      hash->minsize, hash->maxsize, hash->highmark,
      hash->lowmark, hash->nchains, hash->mask);

  return hash;
}

/*!
 * \brief Delete the hash table, all of its entries, and all of the user data.
 *
 * \param hash  Hash table.
 */
void hash_table_destroy(hash_t *hash)
{
  hscan_t  hs;
  hnode_t *node;

  LOGDIAG4CALL(_TPTR(hash));

  hash_scan_begin(&hs, hash);
  while((node = hash_scan_next(&hs)))
  {
    hash_node_fast_unlink(hash, node);
    hnode_destroy(node, hash->freenodedata);
  }
  hash_table_delete(hash);
}

/*!
 * \brief Frees a dynamic hash table structure.
 *
 * \param hash  Hash table.
 *
 * \warning The hash table must be empty.
 *
 * \exception "Not Initialzed"  Hashing not initialized.
 * \exception "Not Empty" Hash table not empty.
 */
void hash_table_delete(hash_t *hash)
{
  assert(hash_val_t_bit != 0);
  assert(hash_isempty(hash));

  delete(hash->table);
  delete(hash);
}

/*!
 * \brief Initialize a user supplied hash table.
 * 
 * The user also supplies a table of chains which is assigned to the hash
 * table . The table is static---it will not grow or shrink.
 *
 * \par Notes &amp; Steps:
 * \b 1.
 *    Initialize hashing.\n
 * \b 2.
 *    Assign hash chains. The user supplied array of pointers hopefully
 *    contains nchains nodes.\n
 * \b 3.
 *    Fixed size.
 * \b 4.
 *    We must dynamically compute the mask from the given power of two table
 *    size. \n
 * \b 5.
 *    The user supplied table can't be assumed to contain null pointers,
 *    so we reset it here.
 *
 * \warning All data in destination hash table is overwritten.
 *
 * \sa hash_table_create()
 *
 * \param hash        The destination hash table.
 * \param minsize     Minimum and initial number of hash entries that will be
 *                    supported.
 * \param maxsize     Maximum number of hash entries that will be supported.
 * \param compfun     Hash key comparator function. NULL will use the default
 *                    strcmp() wrapper.
 * \param hashfun     Hashing function. NULL will used the default string
 *                    hashing function.
 * \param freedatafun User key and data deallocator function. NULL will cause
 *                    no user key or data to be deleted.
 * \param table       The actual hash chain table to attach.
 * \param nchains     Number of nodes in hash chain.
 *
 * \return Hash table.
 *
 * \exception "Bad Parameter" nchain not a power of 2.
 * \exception "Verification Failed" Only if \ref hash_self_verify enabled.
 */
hash_t *hash_table_init(hash_t *hash, hashcount_t minsize, hashcount_t maxsize,
                        hash_comp_t compfun, hash_fun_t hashfun,
                        hnode_data_free_t freedatafun,
                        hnode_t **table, hashcount_t nchains)
{
  LOGDIAG4CALL(_TPTR(hash), _TULONG(minsize), _TULONG(maxsize),
      _TPTR(compfun), _TPTR(hashfun), _TPTR(freedatafun),
      _TPTR(table), _TULONG(nchains));

  // Step 1
  if(hash_val_t_bit == 0)
  {
    compute_bits();
  }

  assert(is_power_of_two(nchains));

  init_hash_sizes(hash, minsize, maxsize);

  // Step 2
  hash->table         = table;
  hash->nchains       = nchains;
  hash->mask          = compute_mask(nchains);  // Step 4
  hash->count         = 0;
  hash->compare       = compfun ? compfun : hash_comp_default;
  hash->hash          = hashfun ? hashfun : hash_fun_default;
  hash->freenodedata  = freedatafun;
  hash->dynamic       = false;    // Step 3

  clear_table(hash);    // Step 5

  if( hash_self_verify )
  {
    assert (hash_table_verify(hash));
  }

  LOGDIAG4("min=%lu, max=%lu, high=%lu, low=%lu, nchains=%lu, mask=0x%lx\n",
      hash->minsize, hash->maxsize, hash->highmark,
      hash->lowmark, hash->nchains, hash->mask);

  return hash;
}

/*!
 * \brief Verify hash table consistency.
 * 
 * If \ref hash_self_verify enabled (see \ref hash_set_self_verify()), this
 * call is automatically called during hash table modification operations.
 *
 * \par Notes &amp; Steps:
 * \b 1.
 *    If the hash table is dynamic, verify whether the high and
 *    low expansion/shrinkage thresholds are powers of two.
 * \b 2.
 *    For each chain, verify whether the back pointers are correctly
 *    set. Count all nodes in the table, and test each hash value
 *    to see whether it is correct for the node's chain.
 *
 * \return Returns true is the table is okay, else returns false.
 */
bool_t hash_table_verify(hash_t *hash)
{
  hashcount_t count = 0;
  hash_val_t chain;
  hnode_t **npp;

  // Step 1
  if (hash->dynamic)
  {
    CHKEXPR_ULONG(hash->lowmark, (hash->lowmark < hash->highmark), false);
    CHKEXPR_ULONG(hash->lowmark, is_power_of_two(hash->lowmark), false);
    CHKEXPR_ULONG(hash->highmark, is_power_of_two(hash->highmark), false);
  }

  // Step 2
  for(chain = 0; chain < hash->nchains; chain++)
  {
    for(npp = hash->table + chain; *npp; npp = &(*npp)->next)
    {
      CHKEXPR_PTR(npp, ((*npp)->pself == npp), false);
      CHKEXPR_PTR(npp, (((*npp)->hkey & hash->mask) == chain), false);
      count++;
    }
  }

  CHKEXPR_ULONG(hash->count, (hash->count == count), false);

  return true;
}

/*!
 * \brief Reset the hash scanner (iterator).
 *
 * After this call the next element retrieved by \ref hash_scan_next() shall
 * be the first element on the first non-empty chain. 
 *
 * \par Notes &amp; Steps:
 * \b 1.
 *    Locate the first non empty chain.\n
 * \b 2.
 *    If an empty chain is found, remember which one it is and set the next
 *    pointer to refer to its first element.\n
 * \b 3.
 *    Otherwise if a chain is not found, set the next pointer to NULL
 *    so that hash_scan_next() shall indicate failure.
 *
 * \param scan  Hash iterator.
 * \param hash  Hash table.
 */
void hash_scan_begin(hscan_t *scan, hash_t *hash)
{
  hash_val_t nchains = hash->nchains;
  hash_val_t chain;

  scan->hash = hash;

  // Step 1
  for(chain = 0; chain < nchains && hash->table[chain] == 0; chain++);

  // Step 2
  if(chain < nchains)
  {
    scan->chain = chain;
    scan->next = hash->table[chain];
  }

  // Step 3
  else
  {
    scan->next = NULL;
  }
}

/*!
 * \brief Retrieve the next node from the hash table.
 *
 * The scanner (iterator) updates the pointer for the next invocation of
 * hash_scan_next(). 
 *
 * \par Notes &amp; Steps:
 * \b 1.
 *    Remember the next pointer in a temporary value so that it can be
 *    returned.\n
 * \b 2.
 *    This assertion essentially checks whether the module has been properly
 *    initialized. The first point of interaction with the module should be
 *    either hash_table_create() or hash_init(), both of which set
 *    hash_val_t_bit to a non zero value.\n
 * \b 3.
 *    If the next pointer we are returning is not NULL, then the user is
 *    allowed to call hash_scan_next() again. We prepare the new next pointer
 *    for that call right now. That way the user is allowed to delete the node
 *    we are about to return, since we will no longer be needing it to locate
 *    the next node.\n
 * \b 4.
 *    If there is a next node in the chain (next->next), then that becomes the
 *    new next node, otherwise ...\n
 * \b 5.
 *    We have exhausted the current chain, and must locate the next subsequent
 *    non-empty chain in the table.\n
 * \b 6.
 *    If a non-empty chain is found, the first element of that chain becomes
 *    the new next node. Otherwise there is no new next node and we set the
 *    pointer to NULL so that the next time hash_scan_next() is called, a null
 *    pointer shall be immediately returned.\n
 *
 * \return Returns next node on success, NULL if no more nodes.
 *
 * \exception "Not Initialzed"  Hashing not initialized.
 */
hnode_t *hash_scan_next(hscan_t *scan)
{
  hnode_t     *next     = scan->next;    // Step 1
  hash_t      *hash     = scan->hash;
  hash_val_t   chain    = scan->chain + 1;
  hash_val_t   nchains  = hash->nchains;

  assert(hash_val_t_bit != 0);  // Step 2

  // Step 3
  if(next)
  {
    // Step 4
    if(next->next)
    {
      scan->next = next->next;
    }
    else
    {
      // Step 5
      while(chain < nchains && hash->table[chain] == 0)
      {
        chain++;
      }
      // Step 6
      if (chain < nchains)
      {
        scan->chain = chain;
        scan->next = hash->table[chain];
      }
      else
      {
        scan->next = NULL;
      }
    }
  }
  return next;
}

/*!
 * \brief Insert a node into the hash table.
 *
 * \par Steps &amp; Notes:
 * \b 1.
 *    It's illegal to insert more than the maximum number of nodes. The client
 *    should verify that the hash table is not full before attempting an
 *    insertion.\n
 * \b 2.
 *    The same key may not be inserted into a table twice.\n
 * \b 3.
 *    If the table is dynamic and the load factor is already at >= 2,
 *    grow the table.\n
 * \b 4.
 *    We take the top N bits of the hash value to derive the chain index,
 *    where N is the base 2 logarithm of the size of the hash table. 
 *
 * \param hash  Hash table.
 * \param node  Node to insert.
 * \param key   Hash key.
 *
 * \exception "Not Initialzed"  Hashing not initialized.
 * \exception "Too Big" Reached maximum table size.
 * \exception "Duplicate Key" Node with key already exists.
 * \exception "Verification Failed" Only if \ref hash_self_verify enabled.
 */
void hash_node_insert(hash_t *hash, hnode_t *node, void *key)
{
  LOGDIAG4CALL(_TPTR(hash), _TPTR(node), _TPTR(key));

  hash_val_t hkey, chain;

  assert(hash_val_t_bit != 0);
  assert(hash->count < hash->maxsize);      // Step 1
  assert(hash_lookup(hash, key) == NULL);   // Step 2

  // Step 3
  if(hash->dynamic && (hash->count+1) >= hash->highmark)
  {
    grow_table(hash);
  }

  hkey  = hash->hash(key);
  chain = hkey & hash->mask;  // Step 4

  node->key   = key;
  node->hkey  = hkey;
  node->pself = hash->table + chain;
  node->next  = hash->table[chain];
  if (node->next)
  {
    node->next->pself = &node->next;
  }
  hash->table[chain] = node;
  hash->count++;

  if( hash_self_verify )
  {
    assert(hash_table_verify(hash));
  }
}

/*!
 * \brief Unlink the given node from the hash table.
 *
 * This is easy, because each node contains a back pointer to the previous
 * node's next pointer.
 *
 * No data are deleted.
 *
 * \par Notes &amp; Steps:
 * \b 1.
 *    The node must belong to this hash table, and its key must not have
 *    been tampered with.\n
 * \b 2.
 *    If there is a next node, then we must update its back pointer to
 *    skip this node.\n
 * \b 3.
 *    We must update the pointer that is pointed at by the back-pointer
 *    to skip over the node that is being deleted and instead point to
 *    the successor (or to NULL if the node being deleted is the last one).
 *
 * \param hash  Hash table.
 * \param node  Hash node to unlink.
 *
 * \return Unlinked node.
 *
 * \exception "Not Initialzed"  Hashing not initialized.
 * \exception "No Key" Hash key not found.
 * \exception "Verification Failed" Only if \ref hash_self_verify enabled.
 */
hnode_t *hash_node_unlink(hash_t *hash, hnode_t *node)
{
  LOGDIAG4CALL(_TPTR(hash), _TPTR(node));

  assert(hash_val_t_bit != 0);
  assert(hash_lookup(hash, node->key) == node);  // Step 1

  if(hash->dynamic && (hash->count-1) <= hash->lowmark
      && hash->count > hash->minsize)
  {
    shrink_table(hash);
  }

  // Step 2
  if (node->next)
  {
    node->next->pself = node->pself;
  }

  // Step 3
  *node->pself = node->next;

  hash->count--;

  if( hash_self_verify )
  {
    assert(hash_table_verify(hash));
  }

  return node;
}

/*!
 * \brief Fast version to unlink the given node from the hash table.
 *
 * Same as \ref hash_node_unlink() except does not trigger table shrinkage.
 * This call is optimized for hash table scan operations.
 * 
 * \sa hash_node_unlink().
 *
 * \param hash  Hash table.
 * \param node  Node to unlink, typically found by scanning.
 *
 * \return Unlinked node.
 *
 * \exception "Not Initialzed"  Hashing not initialized.
 * \exception "No Key" Hash key not found.
 * \exception "Verification Failed" Only if \ref hash_self_verify enabled.
 *
 */
hnode_t *hash_node_fast_unlink(hash_t *hash, hnode_t *node)
{
  LOGDIAG4CALL(_TPTR(hash), _TPTR(node));

  assert(hash_val_t_bit != 0);
  assert(hash_lookup(hash, node->key) == node);  /* 1 */

  // Step 2 as in hash_node_inlink()
  if (node->next)
  {
    node->next->pself = node->pself;
  }

  // Step 3 as in hash_node_inlink()
  *node->pself = node->next;

  hash->count--;

  if( hash_self_verify )
  {
    assert(hash_table_verify(hash));
  }

  return node;
}

/*!
 * \brief Unlink and delete a hash node from the hash table. 
 *
 * The user data and key are deallocated as per the any user supplied
 * data free function. The node is then deallocated.
 *
 * \param hash  Hash table.
 * \param node  Hash node to delete.
 *
 * \exception "Not Initialzed"  Hashing not initialized.
 * \exception "No Key" Hash key not found.
 * \exception "Verification Failed" Only if \ref hash_self_verify enabled.
 */
void hash_node_delete(hash_t *hash, hnode_t *node)
{
  hash_node_unlink(hash, node);
  hnode_destroy(node, hash->freenodedata);
}

/*!
 * \brief Find a node in the hash table and return a pointer to it.
 *
 * \par Notes &amp; Steps:
 * \b 1.
 *    We hash the key and keep the entire hash value. As an optimization, when
 *    we descend down the chain, we can compare hash values first and only if
 *    hash values match do we perform a full key comparison.\n
 * \b 2.
 *    To locate the chain from among 2^N chains, we look at the lower N bits of
 *    the hash value by anding them with the current mask.\n
 * \b 3.
 *    Looping through the chain, we compare the stored hash value inside each
 *    node against our computed hash. If they match, then we do a full
 *    comparison between the unhashed keys. If these match, we have located the
 *    entry.
 *
 *
 * \param hash  Hash table.
 * \param key   Hash key.
 *
 * \return Returns node with key on success, NULL if node not found.
 */
hnode_t *hash_lookup(hash_t *hash, const void *key)
{
  hash_val_t hkey, chain;
  hnode_t *nptr;

  hkey = hash->hash(key);       // Step 1
  chain = hkey & hash->mask;    // Step 2

  // Step 3
  for(nptr = hash->table[chain]; nptr; nptr = nptr->next)
  {
    if(nptr->hkey == hkey && hash->compare(nptr->key, key) == 0)
    {
      return nptr;
    }
  }

  return NULL;
}

/*!
 * \brief Insert user data with the given key into the hash table.
 *
 * The data and key are attached to new hash node which is automatically
 * allocated and the node is inserted into the hash table.
 *
 * \param hash  Hash table.
 * \param key   Hash key.
 * \param data  User data.
 *
 * \return Returns true if successful, else returns false.
 *
 * \exception "Not Initialzed"  Hashing not initialized.
 * \exception "Too Big" Reached maximum table size.
 * \exception "Duplicate Key" Node with key already exists.
 * \exception "Verification Failed" Only if \ref hash_self_verify enabled.
 */
bool_t hash_insert(hash_t *hash, void *key, void *data)
{
  hnode_t *node = hnode_create(data);

  if (node)
  {
    hash_node_insert(hash, node, key);
    return true;
  }

  return false;
}

/*!
 * \brief Unlink and delete a hash node with the given key from the hash table. 
 *
 * The user data and key are deallocated as per the any user supplied
 * data free function. The node is then deallocated.
 *
 * \param hash  Hash table.
 * \param key   Hash key.
 *
 * \return Returns true if successful, else returns false.
 *
 * \exception "Not Initialzed"  Hashing not initialized.
 * \exception "Verification Failed" Only if \ref hash_self_verify enabled.
 */
bool_t hash_delete(hash_t *hash, void *key)
{
  hnode_t *node;

  if( (node = hash_lookup(hash, key)) != NULL )
  {
    hash_node_unlink(hash, node);
    hnode_destroy(node, hash->freenodedata);
    return true;
  }
  return false;
}

/*!
 * \brief Enable (disable) automatic self-checks during hash table modification
 * operatations.
 *
 * \param selfverify  True to enable, false to disable.
 *
 * \sa hash_table_verify()
 */
void hash_set_self_verify(bool_t selfverify)
{
  hash_self_verify = selfverify;
}

/*!
 * \brief Create a hash table node dynamically and assign it the given data.
 *
 * \param data  User data.
 *
 * \return New hash node.
 */
hnode_t *hnode_create(void *data)
{
  hnode_t *node = NEW(hnode_t);

  node->data  = data;
  node->next  = NULL;
  node->pself = NULL;

  return node;
}

/*!
 * \brief Initialize a client-supplied hash node.
 *
 * \param node  Hash node.
 * \param data  User data.
 *
 * \return Same hash node.
 */
hnode_t *hnode_init(hnode_t *node, void *data)
{
  node->data  = data;
  node->next  = NULL;
  node->pself = NULL;
  return node;
}

/*!
 * \brief Hash node and user data deallocator.
 *
 * \param node        Node to deallocate.
 * \param freedatafun User key and data deallocator. May be NULL. 
 */
void hnode_destroy(hnode_t *node, hnode_data_free_t freedatafun)
{
  if( freedatafun != NULL )
  {
    freedatafun(node->key, node->data);
  }
  delete(node);
}

/*!
 * \brief Destroy a dynamically allocated node.
 *
 * User data and key are not touched.
 *
 * \param node  Hash node.
 */
void hnode_delete(hnode_t *node)
{
    delete(node);
}
