////////////////////////////////////////////////////////////////////////////////
//
// Package:   Eudoxus
//
// Program:   utEudoxus   
//
// File:      utEudoxus.cxx
//
/*! \file
 *
 * $LastChangedDate: 2016-01-12 14:08:01 -0700 (Tue, 12 Jan 2016) $
 * $Rev: 4256 $
 *
 * \brief Unit test libEudoxus.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2016-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
/*
 * @EulaBegin@
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include <iostream>
#include <fstream>
#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/opts.h"
#include "rnr/pkg.h"

#include "Eudoxus/Eudoxus.h"
#include "Eudoxus/euPcs.h"

#include "version.h"

using namespace std;
using namespace eu;

/*!
 * \ingroup apps
 * \defgroup unittest utEudoxus
 * \{
 */

#define APP_EC_OK       0   ///< success exit code
#define APP_EC_ARGS     2   ///< command-line options/arguments error exit code
#define APP_EC_EXEC     4   ///< execution exit code

static char    *Argv0;              ///< the command

/*!
 * \brief Program information.
 */
static OptsPgmInfo_T PgmInfo =
{
  // usage_args
  "<FILE>",

  // synopsis
  "Unit test libEudoxus library.",

  // long_desc = 
  "The %P command unit tests the libEudoxus basic operation.",

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
  EuPcsFrame *pFrame;
  uint_t      sec;

  mainInit(argc, argv);

  printf("Creating EuPCSFrame.\n");
  pFrame = new EuPcsFrame;
  printf("Created.\n");

  sec = 1;
  printf("  sleeping %u seconds ...", sec); 
  sleep(sec);
  printf("\n");

  printf("Deleting EuPcsFrame.\n");
  delete pFrame;
  printf("Deleted.\n");

  sec = 1;
  printf("  sleeping %u seconds ...", sec); 
  sleep(sec);
  printf("\n");

  return APP_EC_OK;
}

/*!
 * \}
 */
