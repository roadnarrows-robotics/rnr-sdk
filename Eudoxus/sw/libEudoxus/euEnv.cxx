////////////////////////////////////////////////////////////////////////////////
//
// Package:   Eudoxus
//
// Library:   libEudoxus
//
// File:      euEnv.cxx
//
/*! \file
 *
 * $LastChangedDate: 2016-01-18 14:13:40 -0700 (Mon, 18 Jan 2016) $
 * $Rev: 4263 $
 *
 * \brief Standard Eudoxus application environnment values.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2012-2016.  RoadNarrows LLC
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
// Unless otherwise noted, all materials contained are copyrighted and may not
// be used except as provided in these terms and conditions or in the copyright
// notice (documents and software ) or other proprietary notice provided with
// the relevant materials.
//
//
// IN NO EVENT SHALL THE AUTHOR, ROADNARROWS, OR ANY MEMBERS/EMPLOYEES/
// CONTRACTORS OF ROADNARROWS OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
// PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
// EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
// THE POSSIBILITY OF SUCH DAMAGE.
//
// THE AUTHORS AND  ROADNARROWS SPECIFICALLY DISCLAIM ANY WARRANTIES,
// INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
// "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
// PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
//
 * @EulaEnd@
 */
////////////////////////////////////////////////////////////////////////////////

#include "Eudoxus/euConf.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <string>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Eudoxus/Eudoxus.h"
#include "Eudoxus/euUtils.h"

using namespace std;


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Calculate the difference between two time-of-day times.
 *
 * \param t1  Later time instance.
 * \param t0  Earlier time instance.
 *
 * \return Microsecond difference.
 */
static bool fileexist(const char *sFileName)
{
  if( (sFileName == NULL) || (*sFileName == 0) )
  {
    LOGERROR("Bad argument: No file name.");
    return false;
  }

  else if( access(sFileName, F_OK|R_OK) != 0 )
  {
    LOGDIAG3("%s: No (readable) file.", sFileName);
    return false;
  }

  else
  {
    return true;
  }
}


// ---------------------------------------------------------------------------
// Public Interface
// ---------------------------------------------------------------------------

namespace eu
{

  /*!
   * \brief Determine the NI XML configurtion file name.
   *  
   *  To disable an background task, set the corresponding period to 
   *  \ref NO_TASK.
   *
   * \param lPosCtlPeriod       Position control \h_usec period per servo.
   * \param lDynamicsMonPeriod  Dynamics monitoring \h_usec period per servo.
   * \param lHealthMonPeriod    Health monitoring \h_usec period per servo.
   */
  string getConfigFileName(const char *sCmdOption)
  {
    string  strFileName;
    string  strUserCfgFileName;
    char   *sEnvVal;
  
    if( ((sEnvVal = getenv(EU_ENV_HOME)) != NULL) && (*sEnvVal != 0) )
    {
      strUserCfgFileName  = sEnvVal;
      strUserCfgFileName += "/" ;
      strUserCfgFileName += EU_NI_CONFIG_FILE_BASENAME;
    }
  
    // command-line option override
    if( (sCmdOption != NULL) && (*sCmdOption != 0) )
    {
      if( fileexist(sCmdOption) )
      {
        strFileName = sCmdOption;
      }
      else
      {
        LOGSYSERROR("%s", sCmdOption);
      }
    }
  
    // environment variable override
    else if( ((sEnvVal = getenv(EU_ENV_NI_CONFIG_FILE)) != NULL) &&
              (*sEnvVal != 0) )
    {
      if( fileexist(sEnvVal) )
      {
        strFileName = sEnvVal;
      }
      else
      {
        LOGSYSERROR("%s", sEnvVal);
      }
    }
  
    // home default
    else if( !strUserCfgFileName.empty() &&
              fileexist(strUserCfgFileName.c_str()) )
    {
      strFileName = strUserCfgFileName;
    }
  
    // system default
    else if( fileexist(EU_NI_SYS_CONFIG_FILE) )
    {
      strFileName = EU_NI_SYS_CONFIG_FILE;
    }
  
    // no file found
    else
    {
      LOGSYSERROR("%s", sEnvVal);
    }
  
    return strFileName;
  }

} // namespace eu
