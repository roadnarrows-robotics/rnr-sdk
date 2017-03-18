////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Program:   utRobot   
//
// File:      utRobot.cxx
//
/*! \file
 *
 * $LastChangedDate: 2016-02-01 15:14:45 -0700 (Mon, 01 Feb 2016) $
 * $Rev: 4289 $
 *
 * \brief Unit test liblaelaps robot class.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright
 *   \h_copy 2015-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
/*
 * @EulaBegin@
 * 
 * Unless otherwise stated explicitly, all materials contained are copyrighted
 * and may not be used without RoadNarrows LLC's written consent,
 * except as provided in these terms and conditions or in the copyright
 * notice (documents and software) or other proprietary notice provided with
 * the relevant materials.
 * 
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY 
 * MEMBERS/EMPLOYEES/CONTRACTORS OF ROADNARROWS OR DISTRIBUTORS OF THIS SOFTWARE
 * BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
 * DOCUMENTATION, EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHORS AND  ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 * 
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

#include "Laelaps/laelaps.h"
#include "Laelaps/laeDesc.h"
#include "Laelaps/laeXmlCfg.h"
#include "Laelaps/laeRobot.h"

#include "version.h"

using namespace std;
using namespace laelaps;

/*!
 * \ingroup apps
 * \defgroup unittest utRobot
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
  "Unit test liblaelaps LaeRobot class.",

  // long_desc = 
  "The %P command unit tests the liblaelap LaeRobot class basic operation.",

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
  const char* const CfgFile = "/prj/pkg/Laelaps/share/etc/laelaps/laelaps.conf";

  LaeRobot     *pRobot;
  LaeXmlCfg     xml;
  uint_t        sec;
  int           rc;

  mainInit(argc, argv);

  printf("Creating robot.\n");
  pRobot = new LaeRobot;
  printf("Created.\n");

  sec = 1;
  printf("  sleeping %u seconds ...", sec); 
  sleep(sec);
  printf("\n");

  LaeDesc &desc = pRobot->getLaelapsDesc();

  printf("Parsing robot description.\n");
  if( (rc = xml.loadFile(desc, CfgFile)) == LAE_OK )
  {
    rc = desc.markAsDescribed();
  }

  if( rc != LAE_OK )
  {
    printf("Failed to load/parse description.\n");
    return APP_EC_EXEC;
  }

  printf("Connecting robot.\n");
  if( (rc = pRobot->connect()) == LAE_OK )
  {
    printf("Connected\n");
  }
  else
  {
    printf("Failed to connect.\n");
  }

  sec = 5;
  printf("  sleeping %u seconds ...", sec); 
  sleep(sec);
  printf("\n");

  if( pRobot->isConnected() )
  {
    printf("Disconnecting robot.\n");
    if( (rc = pRobot->disconnect()) == LAE_OK )
    {
      printf("Disconnected\n");
    }
    else
    {
      printf("Failed to disconnect.\n");
    }
  }

  printf("Deleting robot.\n");
  delete pRobot;
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
