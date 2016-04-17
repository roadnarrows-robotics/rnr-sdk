////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// File:      example_log.c
//
/*! \file
 *
 * $LastChangedDate: 2012-10-31 09:53:31 -0600 (Wed, 31 Oct 2012) $
 * $Rev: 2497 $
 *
 * \brief Example of librnr support for logging and command-line options
 *       processing.
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
#include <string.h>
#include <libgen.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/opts.h"
#include "rnr/install.h"

#include "version.h"

// 
// The command with option and argument values.
//
static char *Argv0;               ///< the command
static int   OptsTest = 1;        ///< test option value

/*!
 * \brief Program information.
 */
static OptsPgmInfo_T LogExamplePgmInfo =
{
  .synopsis = "Example of librnr logging and options processing facilities.",

  .long_desc = 
    "The %P command demonstrates various macros and functions of the "
    "librnr logging facilities, plus command-line options processing."
};

/*!
 * \brief Command line options information.
 */
static OptsInfo_T LogExampleOptsInfo[] =
{
  // -t, --test <test>
  {
    .long_opt   = "test",
    .short_opt  = 't',
    .has_arg    = required_argument,
    .has_default= true,
    .opt_addr   = &OptsTest,
    .fn_cvt     = OptsCvtArgInt,
    .fn_fmt     = OptsFmtInt,
    .arg_name   = "<test>",
    .opt_desc   =
      "Set the logging test(s) to conduct.\n"
      "%A is one of:\n"
      "  0   - No tests.\n"
      "  1   - Test #1.\n"
      "  2   - Test #2.\n"
      "  3   - All tests."
  },

  {NULL, }
};

/*!
 * \brief Log test number 1.
 *
 * \param p         Void pointer.
 * \param sFruit    Character pointer.
 * \param n         Integer.
 * \param half      Short.
 * \param f         Double.
 * \param hex       Integer in hex.
 * \param bAreRipe  Boolean.
 */
static void logtest_num1(void *p, const char *sFruit, int n, int half,
    double f, int hex, bool_t bAreRipe)
{
  // test log function call diagnostics 1
  LOGDIAG1CALL(_TPTR(p), _TSTR(sFruit), _TINT(n), _TSHORT(half), _TFLOAT(f),
      _THEX(hex), _TBOOL(bAreRipe));

  // test diagnostic 2 test
  if( bAreRipe )
  {
    LOGDIAG2("There are %d juicy %s in %s", n, sFruit, "Tuscany");
  }
  else
  {
    LOGDIAG2("There are %d green %s in %s", n, sFruit, "Utah");
  }
}

/*!
 * \brief Log test number 2.
 *
 * \param count   Count.
 */
static void logtest_num2(int count)
{
  FILE        *fp;
  const char  *sFileName = "PackingShedOnStrike.newsflash";

  // test log function call diagnostics 2
  LOGDIAG2CALL(_TINT(count));

  LOGWARN("Ebola is your friend, Mr. %s.", "Sicko");

  // test error logging
  while( count-- > 0 )
  {
    LOGERROR("Salmonella scare %d...", count);
  }

  // test system error logging
  if( (fp = fopen(sFileName, "r")) == NULL )
  {
    LOGSYSERROR("%s", sFileName);
  }
  else
  {
    LOGDIAG4("%s: what a pleasant surprise", sFileName);
    fclose(fp);
  }
}

/*!
 * \brief Run log test(s).
 *
 * \returns 1 on success, \h_gt 0 on failure.
 */
static int runlogtests()
{
  short   h = 10;
  int     i = 12;

  switch( OptsTest )
  {
    case 0:
      break;
    case 1:
      logtest_num1(&i, "apples", 53, h, 123.5, 0xdead, false);
      break;
    case 2:
      logtest_num2(3);
      break;
    case 3:
      logtest_num1(&h, "pears", 12, h, -1.5E-2, 0xf1, true);
      logtest_num2(3);
      break;
    default:
      LOGERROR("bug: %d: unexpected test", OptsTest);
      return 0;
  }
  return 1;
}

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
  FILE  *fp;
  int    i;

  // name of this process
  Argv0 = basename(argv[0]);

  // parse input options
  argv = OptsGet(Argv0, &PkgInfo, &LogExamplePgmInfo, LogExampleOptsInfo, true,
                 &argc, argv);

  // test log helper macro
  CHKEXPR_INT(OptsTest, ((OptsTest >= 0) || (OptsTest <= 3)), 0);

  // test complicated logging
  if( LOGABLE(LOG_LEVEL_DIAG1) )
  {
    LOGDIAG1("Post options processed non-option arguments:");
    fp = LOG_GET_LOGFP();
    for(i=0; i<argc; ++i)
    {
      fprintf(fp, "  argv[%d]=\"%s\"\n", i, argv[i]);
    }
  }
 
  return 1;
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
  printf("arch   : %s\n", ARCH);
  printf("install: \n");
  printf("  %s\n", PKG_INSTALL_BINDIR);
  printf("  %s\n", PKG_INSTALL_LIBDIR);
  printf("  %s\n", PKG_INSTALL_INCDIR);
  printf("  %s\n", PKG_INSTALL_SYSCONFDIR);

  if( !init(argc, argv) )
  {
    return 3;
  }

  return runlogtests()? 0: 4;
}
