////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Program:   arduinodiag   
//
// File:      arduinodiag.cxx
//
/*! \file
 *
 * $LastChangedDate: 2014-10-22 10:04:28 -0600 (Wed, 22 Oct 2014) $
 * $Rev: 3791 $
 *
 * \brief Print Hekateros serial numbers.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2014  RoadNarrows
 * (http://www.RoadNarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////


#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/opts.h"
#include "rnr/pkg.h"
#include "rnr/serdev.h"

#include "version.h"

using namespace std;

/*!
 * \ingroup apps
 * \defgroup arduinodiag arduinodiag
 * \{
 */

#define APP_EC_OK       0   ///< success exit code
#define APP_EC_ARGS     2   ///< command-line options/arguments error exit code
#define APP_EC_EXEC     4   ///< execution exit code

static char    *Argv0;                  ///< the command
static bool_t   OptsEqualize  = false;  ///< do [not] equalize color


/*!
 * \brief Program information.
 */
static OptsPgmInfo_T PgmInfo =
{
  // usage_args
  "",

  // synopsis
  "Print Hekateros hardware serial numbers to stdout.",

  // long_desc = 
  "The %P command prints the programmed serial numbers on a Hekateros to "
  "stdout.",

  // diagnostics
  NULL
};

/*!
 * \brief Command line options information.
 */
static OptsInfo_T OptsInfo[] =
{
  {NULL, }
};


/*!
 * \brief Main initialization.
 *
 * \param argc    Command-line argument count.
 * \param argv    Command-line argument list.
 *
 * \par Exits:
 * Program terminates on conversion error.
 */
static void mainInit(int argc, char *argv[])
{
  // name of this process
  Argv0 = basename(argv[0]);

  // parse input options
  argv = OptsGet(Argv0, &PkgInfo, &PgmInfo, OptsInfo, true, &argc, argv);
}

/*!
 * \brief Main.
 *
 * \param argc    Command-line argument count.
 * \param argv    Command-line argument list.
 *
 * \return Returns 0 on succes, non-zero on failure.
 */
int main(int argc, char* argv[])
{
  mainInit(argc, argv);

  return APP_EC_OK;
}

/*!
 * \}
 */
