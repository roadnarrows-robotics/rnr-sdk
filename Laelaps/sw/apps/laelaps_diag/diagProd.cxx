////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Program:   laelaps_diag   
//
// File:      diagProd.cxx
//
/*! \file
 *
 * $LastChangedDate: 2016-02-01 15:14:45 -0700 (Mon, 01 Feb 2016) $
 * $Rev: 4289 $
 *
 * \brief Perform Laelaps product diagnostics.
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
#include <termios.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/opts.h"
#include "rnr/pkg.h"

// common
#include "Laelaps/laelaps.h"
#include "Laelaps/laeUtils.h"

// target diagnostics
#include "Laelaps/laeDesc.h"
#include "Laelaps/laeTune.h"
#include "Laelaps/laeXmlCfg.h"
#include "Laelaps/laeXmlTune.h"

#include "laelaps_diag.h"

using namespace std;
using namespace laelaps;

static const char *SubSysName = "Product";

static string               ProdSN("??????");
static LaeDesc              ProdDesc;
static LaeTunes             ProdTunes;
static map<string, string>  ProdEnv;


static DiagStats readProdSN()
{
  FILE       *fp;
  char        buf[7];
  size_t      snlen = sizeof(buf) - 1;

  DiagStats   stats;

  printSubHdr("Reading Serial Number");

  ++stats.testCnt;
  if( (fp = fopen("/usr/local/share/misc/.sn", "r")) == NULL )
  {
    printTestResult(FailTag, "%s: Find serial number.", SubSysName);
  }
  else if( fread(buf, sizeof(char), snlen, fp) < snlen )
  {
    printTestResult(FailTag, "%s: Read serial number.", SubSysName);
  }
  else
  {
    buf[snlen] = 0;
    ProdSN = buf;
    printTestResult(PassTag, "%s: Read serial number %s.", SubSysName, buf);
    ++stats.passCnt;
  }

  if( fp != NULL )
  {
    fclose(fp);
  }

  return stats;
}

static DiagStats readProdDesc()
{
  LaeXmlCfg     xml;
  int           rc;

  DiagStats   stats;

  ++stats.testCnt;

  if( (rc = xml.load(ProdDesc, LaeSysCfgPath, LaeEtcCfg)) != LAE_OK )
  {
    printTestResult(FailTag, "%s: Load and parse file %s.",
        SubSysName, LaeEtcCfg);
  }
  else
  {
    printTestResult(PassTag, "%s: Load and parse file %s.",
        SubSysName, LaeEtcCfg);
    ++stats.passCnt;
  }

  ++stats.testCnt;

  if( (rc = ProdDesc.markAsDescribed()) != LAE_OK )
  {
    printTestResult(FailTag, "%s: Description complete.", SubSysName);
  }
  else
  {
    printTestResult(PassTag, "%s: Description complete.", SubSysName);
    ++stats.passCnt;
  }

  return stats;
}

static DiagStats readProdTunes()
{
  LaeXmlTune    xml;
  int           rc;
  DiagStats     stats;

  ++stats.testCnt;

  if( (rc = xml.load(ProdTunes, LaeSysCfgPath, LaeEtcTune)) != LAE_OK )
  {
    printTestResult(FailTag, "%s: Load and parse file %s.",
        SubSysName, LaeEtcCfg);
  }
  else
  {
    printTestResult(PassTag, "%s: Load and parse file %s.",
        SubSysName, LaeEtcCfg);
    ++stats.passCnt;
  }

  return stats;
}

static DiagStats readProdEnv()
{
  const char *EnvName[] =
  {
    "ROS_DISTRO", "ROS_ETC_DIR", "ROS_MASTER_URI", "ROS_PACKAGE_PATH", 
    "ROS_ROOT" 
  };

  char         *sVal;
  DiagStats     stats;
  const char   *sTag;
  size_t        i;

  for(i = 0; i < arraysize(EnvName); ++i)
  {
    ++stats.testCnt;
    sTag = FailTag;
    if( (sVal = getenv(EnvName[i])) != NULL )
    {
      ProdEnv[EnvName[i]] = sVal;
      ++stats.passCnt;
      sTag = PassTag;
    }
    printTestResult(sTag, "%s: getenv(%s).", SubSysName, EnvName[i]);
  }

  return stats;
}

static void printResults()
{
  map<string, string>::iterator iter;

  printf("\n");
  printf("Laelaps Product Summary:\n");

  printf("  Serial Number:  %s\n", ProdSN.c_str());

  printf("\n");
  ProdDesc.print(2);

  printf("\n");
  ProdTunes.print(2);

  printf("\n");
  for(iter = ProdEnv.begin(); iter != ProdEnv.end(); ++iter)
  {
    printf("%18s: %s\n", iter->first.c_str(), iter->second.c_str());
  }

  printf("\n\n");
}

DiagStats runProductDiagnostics()
{
  DiagStats   statsTest;
  DiagStats   statsTotal;

  printHdr("Product Diagnostics");

  //
  // Read Serial Number Tests
  //
  if( !statsTotal.fatal )
  {
    statsTest = readProdSN();

    printSubTotals(statsTest);

    statsTotal += statsTest;
  }

  //
  // Read and Parse Product Description Tests
  //
  if( !statsTotal.fatal )
  {
    statsTest = readProdDesc();

    printSubTotals(statsTest);

    statsTotal += statsTest;
  }

  //
  // Read and Parse Product Tuning Paramers Tests
  //
  if( !statsTotal.fatal )
  {
    statsTest = readProdTunes();

    printSubTotals(statsTest);

    statsTotal += statsTest;
  }

  //
  // Read Product Environment Tests
  //
  if( !statsTotal.fatal )
  {
    statsTest = readProdEnv();

    printSubTotals(statsTest);

    statsTotal += statsTest;
  }

  //
  // Summary
  //
  printTotals(statsTotal);

  printResults();
 
  return statsTotal;
}
