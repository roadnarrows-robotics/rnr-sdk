////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Program:   laelaps_diag   
//
// File:      diagCpu.cxx
//
/*! \file
 *
 * $LastChangedDate: 2016-02-01 15:14:45 -0700 (Mon, 01 Feb 2016) $
 * $Rev: 4289 $
 *
 * \brief Perform Laelaps main CPU diagnostics.
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

#include <sys/utsname.h>
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
#include <algorithm>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/opts.h"
#include "rnr/pkg.h"

// common
#include "Laelaps/laelaps.h"
#include "Laelaps/laeUtils.h"

// hardware
#include "Laelaps/laeSysDev.h"

#include "laelaps_diag.h"

using namespace std;
using namespace laelaps;

static const char *SubSysName = "CPU";
static const char *ProdName   = "Odroid";

static void trim(string &str)
{
  string    strWork(str);
  size_t    i, j, k;

  // leading white space
  for(i=0; i<strWork.size(); ++i)
  {
    if( (strWork[i] != ' ')  && (strWork[i] != '\t') &&
        (strWork[i] != '\n') && (strWork[i] != '\r') )
    {
      break;
    }
  }

  // trailing white space
  for(j=strWork.size()-1; j>=i; --j)
  {
    if( (strWork[j] != ' ')  && (strWork[j] != '\t') &&
        (strWork[j] != '\n') && (strWork[j] != '\r') )
    {
      break;
    }
  }

  // copy back
  if( j >= i )
  {
    str.resize(j-i+1);
    for(k=0; k<str.size(); ++k)
    {
      str[k] = strWork[i++];
    }
  }
  else
  {
    str.clear();
  }
}

static void tolower(string &str)
{
  for(size_t i=0; i<str.size(); ++i)
  {
    str[i] = ::tolower(str[i]);
  }
}

static string normalize(string str)
{
  //fprintf(stderr, "DBG: normalize: \"%s\" --> ", str.c_str());

  trim(str);
  //fprintf(stderr, "\"%s\" --> ", str.c_str());

  tolower(str);
  //fprintf(stderr, "\"%s\"\n", str.c_str());

  return str;
}

static bool parseCpuInfo(string &strCpuHw,
                         string &strCpuRev,
                         int    &numCores)
{
  const size_t  maxlen = 256; 

  FILE   *fp;
  char    line[maxlen];
  char   *tokKey, *tokVal;
  string  strKey;

  strCpuHw.clear();
  strCpuRev.clear();
  numCores = 0;

  if( (fp = fopen("/proc/cpuinfo", "r")) == NULL )
  {
    return false;
  }

  while( fgets(line, maxlen, fp) != NULL )
  {
    line[maxlen-1] = 0;   // paranoia

    if( (tokKey = strtok(line, ":")) == NULL )
    {
      continue;
    }
    else if( (tokVal = strtok(NULL, "\n")) == NULL )
    {
      continue;
    }

    strKey = normalize(tokKey);

    if( strKey == "processor" )
    {
      ++numCores;
    }
    else if( strKey == "hardware" )
    {
      strCpuHw = tokVal;
      trim(strCpuHw);
    }
    else if( strKey == "revision" )
    {
      strCpuRev = tokVal;
      trim(strCpuRev);
    }
  }

  fclose(fp);

  return true;
}

static bool parseLsbRelease(string &strDistribId,
                            string &strDistribRel,
                            string &strDistribCode,
                            string &strDistribDesc)
{
  const size_t  maxlen = 256; 

  FILE   *fp;
  char    line[maxlen];
  char   *tokKey, *tokVal;
  string  strKey;

  strDistribId.clear();
  strDistribRel.clear();
  strDistribCode.clear();
  strDistribDesc.clear();

  if( (fp = fopen("/etc/lsb-release", "r")) == NULL )
  {
    return false;
  }

  while( fgets(line, maxlen, fp) != NULL )
  {
    line[maxlen-1] = 0;   // paranoia

    if( (tokKey = strtok(line, "=")) == NULL )
    {
      continue;
    }
    else if( (tokVal = strtok(NULL, "\n")) == NULL )
    {
      continue;
    }
    
    strKey = normalize(tokKey);

    if( strKey == "distrib_id" )
    {
      strDistribId = tokVal;
      trim(strDistribId);
    }
    else if( strKey == "distrib_release" )
    {
      strDistribRel = tokVal;
      trim(strDistribRel);
    }
    else if( strKey == "distrib_codename" )
    {
      strDistribCode = tokVal;
      trim(strDistribCode);
    }
    else if( strKey == "distrib_description" )
    {
      strDistribDesc = tokVal;
      trim(strDistribDesc);
    }
  }

  fclose(fp);

  return true;
}

static bool parseEthMacAddr(string &strEthMacAddr)
{
  const size_t  maxlen = 256; 

  FILE  *fp;
  char  line[maxlen];
  int   n;

  strEthMacAddr.clear();

  if( (fp = fopen("/sys/class/net/eth0/address", "r")) == NULL )
  {
    return false;
  }

  if (fgets(line, maxlen, fp) != NULL )
  {
    line[maxlen-1] = 0;   // paranoia
    n = strlen(line);
    if( n > 1 )           // zap newline
    {
      line[n] = 0;
    }
    strEthMacAddr = line;
    trim(strEthMacAddr);
  }

  fclose(fp);

  return true;
}

static bool parseMmcCid(string &strMmcCid)
{
  const size_t  maxlen = 256; 

  FILE  *fp;
  char  line[maxlen];
  int   n;

  strMmcCid.clear();

  if( (fp = fopen("/sys/block/mmcblk0/device/cid", "r")) == NULL )
  {
    return false;
  }

  if (fgets(line, maxlen, fp) != NULL )
  {
    line[maxlen-1] = 0;   // paranoia
    n = strlen(line);
    if( n > 1 )           // zap newline
    {
      line[n] = 0;
    }
    strMmcCid = line;
    trim(strMmcCid);
  }

  fclose(fp);

  return true;
}

static DiagStats readInfo()
{
  string          strCpuHw;
  string          strCpuRev;
  int             numCores;
  string          strDistribId;
  string          strDistribRel;
  string          strDistribCode;
  string          strDistribDesc;
  struct utsname  utsInfo;
  string          strEthMacAddr;
  string          strMmcCid;

  const char     *sTag;
  DiagStats       stats;

  printSubHdr("Main Processor Info");

  ++stats.testCnt;
  if( parseCpuInfo(strCpuHw, strCpuRev, numCores) )
  {
    ++stats.passCnt;
    sTag = PassTag;
  }
  else
  {
    sTag = FailTag;
  }
  printTestResult(sTag, "Get CPU info.");

  ++stats.testCnt;
  if( parseLsbRelease(strDistribId, strDistribRel,
                      strDistribCode, strDistribDesc) )
  {
    ++stats.passCnt;
    sTag = PassTag;
  }
  else
  {
    sTag = FailTag;
  }
  printTestResult(sTag, "Get system release info.");

  ++stats.testCnt;
  if( uname(&utsInfo) == 0 )
  {
    ++stats.passCnt;
    sTag = PassTag;
  }
  else
  {
    memset(&utsInfo, 0, sizeof(struct utsname));
    sTag = FailTag;
  }
  printTestResult(sTag, "Get kernel info.");

  ++stats.testCnt;
  if( parseEthMacAddr(strEthMacAddr) )
  {
    ++stats.passCnt;
    sTag = PassTag;
  }
  else
  {
    sTag = FailTag;
  }
  printTestResult(sTag, "Get Ethernet hardware info."); 

  ++stats.testCnt;
  if( parseMmcCid(strMmcCid) )
  {
    ++stats.passCnt;
    sTag = PassTag;
  }
  else
  {
    sTag = FailTag;
  }
  printTestResult(sTag, "Get eMMC CID info."); 

  printf("\n");
  printf("Main Processor Summary:\n");
  printf("  Hardware:\n");
  printf("    Product:           %s\n", ProdName);
  printf("    SOC:               %s\n", "Samsung Exynos5 Octa ARM Cortex");
  printf("    Architecture:      %s\n", utsInfo.machine);
  printf("    Hardware Rev:      %s\n", strCpuRev.c_str());
  printf("    Number of Cores:   %d\n", numCores);
  printf("    eMMC CID:          %s\n", strMmcCid.c_str());
  printf("    Ethernet MAC:      %s\n", strEthMacAddr.c_str());
  printf("  Operating System:\n");
  printf("    Kernel:            %s\n", utsInfo.sysname);
  printf("    Release:           %s\n", utsInfo.release);
  printf("    Version:           %s\n", utsInfo.version);
  printf("    Node:              %s\n", utsInfo.nodename);
  printf("  Distribution:\n");
  printf("    Name:              %s\n", strDistribId.c_str());
  printf("    Release:           %s\n", strDistribRel.c_str());
  printf("    Code Name:         %s\n", strDistribCode.c_str());
  printf("    Description:       %s\n", strDistribDesc.c_str());
  printf("  System Devices:\n");
  printf("    Motor Controllers: %s\n", LaeDevMotorCtlrs);
  printf("    I2C Bus:           %s\n", LaeDevI2C_0);
  printf("                       %s\n", LaeDevI2C_1);
  printf("                       %s\n", LaeDevI2C_2);
  printf("    IMU:               %s\n", LaeDevIMU);
  printf("    Dynamixel Bus:     %s\n", LaeDevDynabus);
  printf("    Front Camera:      %s\n", LaeDevFCam);
  printf("\n");

  return stats;
}

DiagStats runCpuDiagnostics()
{
  DiagStats   statsTest;
  DiagStats   statsTotal;

  printHdr("Main CPU Diagnostics");

  //
  // Read Main Processor Info
  //
  statsTest = readInfo();

  printSubTotals(statsTest);

  statsTotal += statsTest;

#if 0
  //
  // X Tests
  //
  if( !statsTotal.fatal )
  {
    statsTest = readInfo();

    printSubTotals(statsTest);

    statsTotal += statsTest;
  }
#endif
 
  //
  // Summary
  //
  printTotals(statsTotal);

  return statsTotal;
}
