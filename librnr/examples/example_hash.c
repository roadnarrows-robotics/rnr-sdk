////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// File:      example_hash.c
//
/*! \file
 *
 * $LastChangedDate: 2011-11-18 13:30:34 -0700 (Fri, 18 Nov 2011) $
 * $Rev: 1577 $
 *
 * \brief Example of using a librnr hash table.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2007-2017. RoadNarrows LLC.\n
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
#include <string.h>
#include <ctype.h>
#include <stdarg.h>

#include "rnr/rnrconfig.h"
#include "rnr/new.h"
#include "rnr/log.h"
#include "rnr/hash.h"

/*!
 * \brief Working input buffer.
 */
typedef char input_t[256];

/*!
 * \brief Tokenize input
 *
 * \param string        Input string to parse.
 * \param [out] ...     Variable argument list of char** arguments terminated
 *                      by a last NULL argument.
 *
 * \return Number of tokens.
 */
static int tokenize(char *string, ...)
{
  char    **tokptr; 
  va_list   arglist;
  int       tokcount = 0;

  va_start(arglist, string);
  tokptr = va_arg(arglist, char **);
  while(tokptr)
  {
    while(*string && isspace((int)*string))
    {
      string++;
    }
    if(!*string)
    {
      break;
    }
    *tokptr = string;
    while(*string && !isspace((int)*string))
    {
      string++;
    }
    tokptr = va_arg(arglist, char **);
    tokcount++;
    if(!*string)
    {
      break;
    }
    *string++ = 0;
  }
  va_end(arglist);

  return tokcount;
}

/*!
 * \brief Delete node data - both key and value are dynamically allocated.
 *
 * \param key   Data key.
 * \param val   Data value.
 */
static void del_node_data(void *key, void *val)
{
  delete(key);
  delete(val);
}

/*!
 * \brief Colorado's 14,000+ foot mountains. 
 *
 * N.B. there are 53 of them, I'm stopping at the highest 16.
 */
static char *Colorado14ers[][2] =
{
  {"mt_elbert",         "14,433"},
  {"mt_massive",        "14,421"},
  {"mt_harvard",        "14,420"},
  {"blanca_peak",       "14,345"},
  {"la_plata_peak",     "14,336"},
  {"uncompahgre_peak",  "14,309"},
  {"crestone_peak",     "14,294"},
  {"mt_lincoln",        "14,286"},
  {"grays_peak",        "14,270"},
  {"mt_antero",         "14,269"},
  {"torreys_peak",      "14,267"},
  {"castle_peak",       "14,265"},
  {"quandary_peak",     "14,265"},
  {"mt_evans",          "14,264"},
  {"longs_peak",        "14,255"},
  {"mt_wilson",         "14,246"}
};

/*!
 * \brief Make random keys unique.
 */
static int MtnCounter = 0;

/*!
 * \brief Force table to grow by feeding the table random entries.
 *
 * \param h   Pointer to hash table.
 */
static void force_grow(hash_t *h)
{
  hashcount_t   count, size;
  size_t        num_mtns = arraysize(Colorado14ers);
  int           mtn;
  char          buf[64];
  char         *key, *val;

  count = hash_count(h);
  size  = hash_size(h);

  printf(" starting count=%lu, table size=%lu\n", count, size); 

  // tables grows at highwater mark
  size = h->highmark;

  while( count++ < size )
  {
    mtn = (int)(random() % (long int)num_mtns);
    sprintf(buf, "%s(%d)", Colorado14ers[mtn][0], MtnCounter++);
    key = new_strdup(buf);
    val = new_strdup(Colorado14ers[mtn][1]);

    if( !hash_insert(h, key, val) )
    {
      puts("hash_insert failed");
      del_node_data(key, val);
      return;
    }
    else
    {
      printf(" added %s -> %s\n", key, val);
    }
  }

  count = hash_count(h);
  size  = hash_size(h);

  printf(" ending count=%lu, table size=%lu\n", count, size); 
}

/*!
 * \brief Force table to shink by deleting table random entries.
 *
 * \param h   Pointer to hash table.
 */
static void force_shrink(hash_t *h)
{
  hashcount_t   count, size;
  hscan_t       hs;
  hnode_t      *hn;
  char         *key, *val;

  count = hash_count(h);
  size  = hash_size(h);

  printf(" starting count=%lu, table size=%lu\n", count, size); 

  // tables shrinks at lowwater mark
  size = h->lowmark;

  hash_scan_begin(&hs, h);

  // delete first count nodes
  while( ((hn = hash_scan_next(&hs)) != NULL) && (count-- > size) )
  {
    key = (char *)hnode_getkey(hn);
    val = (char *)hnode_get(hn);
    printf(" deleted %s -> %s\n", key, val); 
    hash_node_delete(h, hn);
  }

  count = hash_count(h);
  size  = hash_size(h);

  printf(" ending count=%lu, table size=%lu\n", count, size); 
}

/*!
 * \brief Example main.
 *
 * \param argc  Command-line argument count.
 * \param argv  Command-line argument list.
 *
 * \par Exit Status:
 * Program exits with 0 success, \h_gt 0 on failure.
 */
int main(int argc, char *argv[])
{
  input_t   in;
  hash_t   *h = NULL;
  hnode_t  *hn;
  hscan_t   hs;
  char     *tok1, *tok2;
  char     *key, *val;
  int       n;

  char *help =
  "n                      create new empty hash table\n"
  "x                      delete entire hash table\n"
  "a <key> <val>          add value to hash table\n"
  "d <key>                delete value from hash table\n"
  "l <key>                lookup value in hash table\n"
  "c                      show counts: number of entries and table size\n"
  "t                      dump whole hash table\n"
  "+                      force increase hash table (by auto-adding)\n"
  "-                      force decrease hash table (by auto-deleting)\n"
  "v <level>              set logging verbosity level 0-4\n"
  "q                      quit";


  puts("Hashing example (enter '?' for list of commands)");

  // process user commands
  for(;;)
  {
    printf("hash> ");

    // read user input
    if( !fgets(in, (int)sizeof(input_t), stdin) )
    {
      break;
    }

    // execute command
    switch( in[0] )
    {
      // help
      case '?':
        puts(help);
        break;

      // create new hash table
      case 'n':
        if( h != NULL )
        {
          printf("table exists, delete first\n");
        }
        else
        {
          h = hash_table_create(
                  true,             // dynamic table sizing
                  (hashcount_t)4,   // minimum size
                  HASHCOUNT_T_MAX,  // maximum size
                  NULL,             // use default comparator function
                  NULL,             // use default hashing function
                  del_node_data);   // hash node data deletion function

          hash_set_self_verify(true); // off by default
        }
        break;

      // delete hash table and all user data 
      case 'x':
        if( h == NULL )
        {
          printf("no table\n");
        }
        else
        {
          hash_table_destroy(h);
          h = NULL;
        }
        break;

      // add hash table entry
      case 'a':
        if( h == NULL )
        {
          printf("no hash table\n");
        }
        else if(tokenize(in+1, &tok1, &tok2, (char **) 0) != 2)
        {
          puts("what?");
        }
        else
        {
          // new key and value
          key = new_strdup(tok1);
          val = new_strdup(tok2);

          // insert into hash table, automatically allocating hnode_t
          if( !hash_insert(h, key, val) )
          {
            puts("hash_insert failed");
            del_node_data(key, val);
          }
        }
        break;

      // delete hash table entry
      case 'd':
        if( h == NULL )
        {
          printf("no hash table\n");
        }
        else if(tokenize(in+1, &tok1, (char **)0) != 1)
        {
          puts("what?");
        }
        else if( (hn = hash_lookup(h, tok1)) == NULL )
        {
          puts("hash_lookup failed");
        }
        else
        {
          key = (char *)hnode_getkey(hn);
          val = (char *)hnode_get(hn);
          printf("deleted %s -> %s\n", key, val); 
          hash_node_delete(h, hn);
        }
        break;

      // lookup hash table entry
      case 'l':
        if( h == NULL )
        {
          printf("no hash table\n");
        }
        else if(tokenize(in+1, &tok1, (char **) 0) != 1)
        {
          puts("what?");
        }
        else if( (hn = hash_lookup(h, tok1)) == NULL )
        {
          puts("hash_lookup failed");
        }
        else
        {
          val = hnode_get(hn);
          printf("%s -> %s\n", tok1, val); 
        }
        break;

      // get current number of hash table entries and current table size
      case 'c':
        if( h == NULL )
        {
          printf("no hash table\n");
        }
        else
        {
          printf("count=%lu, size=%lu\n",
              (unsigned long)hash_count(h), (unsigned long)hash_size(h));
        }
        break;

      // dump hash table entries
      case 't':
        if( h == NULL )
        {
          printf("no hash table\n");
        }
        else
        {
          hash_scan_begin(&hs, h);
          while( (hn = hash_scan_next(&hs)) != NULL )
          {
            printf("%s -> %s\n", (char *)hnode_getkey(hn),
                (char *)hnode_get(hn));
          }
        }
        break;

      // force hash table to grow
      case '+':
        if( h == NULL )
        {
          printf("no hash table\n");
        }
        else
        {
          force_grow(h);
        }
        break;

      // force hash table to shrink
      case '-':
        if( h == NULL )
        {
          printf("no hash table\n");
        }
        else
        {
          force_shrink(h);
        }
        break;

      // logging level
      case 'v':
        if( tokenize(in+1, &tok1, (char **)0) != 1 )
        {
          puts("what?");
        }
        else
        {
          n = atoi(tok1);
          LOG_SET_THRESHOLD(n);
        }
        break;

      // quit
      case 'q':
        exit(0);
        break;

      // null command
      case '\0':
      case '\n':
        break;

      // input error
      default:
        puts("?");
        break;
    }
  }

  return 0;
}
