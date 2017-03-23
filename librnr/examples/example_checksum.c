////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// File:      example_checksum.c
//
/*! \file
 *
 * $LastChangedDate: 2013-03-05 09:45:13 -0700 (Tue, 05 Mar 2013) $
 * $Rev: 2729 $
 *
 * \brief Example of librnr support for checksum algorithms.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2013-2017. RoadNarrows LLC.\n
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
#include <libgen.h>
#include <time.h>
#include <string.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/new.h"
#include "rnr/opts.h"
#include "rnr/checksum.h"

#include "version.h"

// 
// The command with option and argument values.
//
static char    *Argv0;                  ///< the command
static bool_t   OptsPrintPoem = false;  ///< do [not] print poem

/*!
 * \brief Program information.
 */
static OptsPgmInfo_T PgmInfo =
{
  .synopsis = "Example of librnr checksum algorithms.",

  .long_desc = 
    "The %P command runs serveral checksum algorithms over a fixed body of "
    "text and prints the results to stdout."
};

/*!
 * \brief Command line options information.
 */
static OptsInfo_T OptsInfo[] =
{
  // --poem
  {
    .long_opt     = "poem",
    .short_opt    = OPTS_NO_SHORT,
    .has_arg      = no_argument,
    .has_default  = true,
    .opt_addr     = &OptsPrintPoem,
    .fn_cvt       = OptsCvtArgBool,
    .fn_fmt       = OptsFmtBool,
    .arg_name     = NULL,
    .opt_desc     = "Do [not] print poem."
  },

  {NULL, }
};

/*!
 * \brief Main initialization.
 *
 * \param argc  Command-line argument count.
 * \param argv  Command-line argument list.
 *
 * \return Returns 1 on success, exits on failure.
 */
static int init(int argc, char *argv[])
{
  // name of this process
  Argv0 = basename(argv[0]);

  // parse input options
  argv = OptsGet(Argv0, &PkgInfo, &PgmInfo, OptsInfo, true, &argc, argv);

  return 1;
}

/*!
 * \brief Jabberwocky by Lewis Carroll
 */
static char *Jabberwocky =
  "Jabberwocky -- Lewis Carroll\n"
  "\n"
  "'Twas brillig, and the slithy toves\n"
  "Did gyre and gimble in the wabe;\n"
  "All mimsy were the borogoves,\n"
  "And the mome raths outgrabe.\n"
  "\n"
  "\"Beware the Jabberwock, my son!\n"
  "The jaws that bite, the claws that catch!\n"
  "Beware the Jubjub bird, and shun\n"
  "The frumious Bandersnatch!\"\n"
  "\n"
  "He took his vorpal sword in hand:\n"
  "Long time the manxome foe he sought—\n"
  "So rested he by the Tumtum tree,\n"
  "And stood awhile in thought.\n"
  "\n"
  "And as in uffish thought he stood,\n"
  "The Jabberwock, with eyes of flame,\n"
  "Came whiffling through the tulgey wood,\n"
  "And burbled as it came!\n"
  "\n"
  "One, two! One, two! and through and through\n"
  "The vorpal blade went snicker-snack!\n"
  "He left it dead, and with its head\n"
  "He went galumphing back.\n"
  "\n"
  "\"And hast thou slain the Jabberwock?\n"
  "Come to my arms, my beamish boy!\n"
  "O frabjous day! Callooh! Callay!\"\n"
  "He chortled in his joy.\n"
  "\n"
  "'Twas brillig, and the slithy toves\n"
  "Did gyre and gimble in the wabe;\n"
  "All mimsy were the borogoves,\n"
  "And the mome raths outgrabe.\n";

/*!
 * \brief Start nano second timer.
 *
 * \param [out] pstart  Pointer to start timespec.
 */
static void starttimer(struct timespec *pstart)
{
  clock_gettime(CLOCK_REALTIME, pstart);
}

/*!
 * \brief Stop nano second timer.
 *
 * \param [in] pstart   Pointer to start timespec.
 *
 * \return Elaspse time in nano seconds.
 */
static long stoptimer(struct timespec *pstart)
{
  struct timespec tstop;
  long            nsec;

  clock_gettime(CLOCK_REALTIME, &tstop);

  nsec = (tstop.tv_sec - pstart->tv_sec) * 1000000000;

  if( tstop.tv_nsec >= pstart->tv_nsec )
  {
    nsec += (tstop.tv_nsec - pstart->tv_nsec);
  }
  else
  {
    nsec += (1000000000 - pstart->tv_nsec + tstop.tv_nsec); 
  }

  return nsec;
}

/*!
 * \brief Profile the various checksum algorithms.
 *
 * \param sText   Null-terminated text string.
 */
static void profile(char *sText)
{
  struct timespec t;
  uint_t          chksum;
  long            nsec;

  printf("Algorithm               Result         nSecs\n");

  starttimer(&t);
  chksum = (uint_t)generate_checksum8((byte_t *)sText, strlen(sText));
  nsec = stoptimer(&t);
  printf(" 8-bit chksum  %10u 0x%08x  %6ld\n", chksum, chksum, nsec);

  starttimer(&t);
  chksum = (uint_t)generate_checksum16((byte_t *)sText, strlen(sText));
  nsec = stoptimer(&t);
  printf("16-bit chksum  %10u 0x%08x  %6ld\n", chksum, chksum, nsec);

  starttimer(&t);
  chksum = (uint_t)generate_checksum32((byte_t *)sText, strlen(sText));
  nsec = stoptimer(&t);
  printf("32-bit chksum  %10u 0x%08x  %6ld\n", chksum, chksum, nsec);

  starttimer(&t);
  chksum = (uint_t)generate_crc32((byte_t *)sText, strlen(sText));
  nsec = stoptimer(&t);
  printf("32-bit crc     %10u 0x%08x  %6ld\n", chksum, chksum, nsec);
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
  char   *sText;
  char    c;
  int     k;
  int     i;

  if( !init(argc, argv) )
  {
    return 2;
  }

  if( OptsPrintPoem )
  {
    printf("\n%s\n", Jabberwocky);
  }

  //
  // --- Profile 1
  //
  printf("Text:   Jabberwocky  -- Lewis Carroll\n");
  printf("Length: %zu bytes\n", strlen(Jabberwocky));

  profile(Jabberwocky);

  printf("\n");

  //
  // --- Profile 2
  //
  sText = new_strdup(Jabberwocky);

  c = sText[0];
  sText[0] = sText[1];
  sText[1] = c;

  printf("Text:   Jabberwocky, 1 byte swapped  -- Lewis Carroll\n");
  printf("Length: %zu bytes\n", strlen(sText));

  profile(sText);

  printf("\n");

  delete(sText);

  //
  // --- Profile 3
  //
  k = 25;

  sText = new(strlen(Jabberwocky) * (size_t)k + (size_t)1);

  for(i=0, *sText=0; i<k; ++i)
  {
    strcat(sText, Jabberwocky);
  }

  printf("Text:   Jabberwocky * %d  -- Lewis Carroll\n", k);
  printf("Length: %zu bytes\n", strlen(sText));

  profile(sText);

  printf("\n");

  delete(sText);

  return 0;
}
