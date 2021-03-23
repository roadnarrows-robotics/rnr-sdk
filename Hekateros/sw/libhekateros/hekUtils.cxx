////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Library:   libhekateros
//
// File:      hekUtils.cxx
//
//
/*! \file
 *
 * $LastChangedDate: 2015-04-23 15:45:00 -0600 (Thu, 23 Apr 2015) $
 * $Rev: 3956 $
 *
 * \brief Hekateros utilities.
 *
 * \author Daniel Packard (daniel@roadnarrows.com)
 * \author Robin Knight   (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2012-2017. RoadNarrows LLC.\n
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


#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
#include <libgen.h>
#include <stdio.h>
#include <errno.h>

#include <string>
#include <sstream>
#include <vector>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "Hekateros/hekateros.h"
#include "Hekateros/hekUtils.h"

using namespace std;


//------------------------------------------------------------------------------
// Private Interface
//------------------------------------------------------------------------------

/*!
 * \ingroup libhek
 * \brief \h_hek Error Code String Table.
 *
 * Table is indexed by \h_hek error codes (see \ref hek_ecodes). Keep
 * in sync.
 */
static const char *EcodeStrTbl[] =
{
  "Ok",                                     ///< [HEK_OK]

  "Error",                                  ///< [HEK_ECODE_GEN]
  "System error",                           ///< [HEK_ECODE_SYS]
  "Internal error",                         ///< [HEK_ECODE_INTERNAL]
  "Bad value",                              ///< [HEK_ECODE_BAD_VAL]
  "Too big",                                ///< [HEK_ECODE_TOO_BIG]
  "Too small",                              ///< [HEK_ECODE_TOO_SMALL]
  "Value out-of-range",                     ///< [HEK_ECODE_RANGE]
  "Invalid operation",                      ///< [HEK_ECODE_BAD_OP]
  "Operation timed out",                    ///< [HEK_ECODE_TIMEDOUT]
  "Device not found",                       ///< [HEK_ECODE_NO_DEV]
  "No resource available",                  ///< [HEK_ECODE_NO_RSRC]
  "Resource busy",                          ///< [HEK_ECODE_BUSY]
  "Cannot execute",                         ///< [HEK_ECODE_NO_EXEC]
  "Permissions denied",                     ///< [HEK_ECODE_PERM]
  "Dynamixel chain or servo error",         ///< [HEK_ECODE_DYNA]
  "Video error",                            ///< [HEK_ECODE_VIDEO]
  "Bad format",                             ///< [HEK_ECODE_FORMAT]
  "BotSense error",                         ///< [HEK_ECODE_BOTSENSE]
  "File not found",                         ///< [HEK_ECODE_NO_FILE]
  "XML error",                              ///< [HEK_ECODE_XML]
  "Robot is in an alarmed state",           ///< [HEK_ECODE_ALARMED]
  "Operation interrupted",                  ///< [HEK_ECODE_INTR]
  "Robotic link(s) movement obstructed",    ///< [HEK_ECODE_COLLISION]
  "Robot emergency stopped",                ///< [HEK_ECODE_ESTOP]

  "Invalid error code"                      ///< [HEK_ECODE_BADEC]
};


//------------------------------------------------------------------------------
// Public Interface
//------------------------------------------------------------------------------

const char *hekateros::getStrError(const int ecode)
{
  int ec = ecode >= 0 ? ecode : -ecode;

  if( ec >= arraysize(EcodeStrTbl) )
  {
    ec = HEK_ECODE_BADEC;
  }

  return EcodeStrTbl[ec];
}

uint_t hekateros::strToVersion(const string &str)
{
  int   nMajor    = 0;
  int   nMinor    = 0;
  int   nRevision = 0;

  sscanf(str.c_str(), "%d.%d.%d", &nMajor, &nMinor, &nRevision);

  return HEK_VERSION(nMajor, nMinor, nRevision);
}

bool hekateros::operator<(const struct timeval& lhs, const struct timeval& rhs)
{
  if( lhs.tv_sec < rhs.tv_sec )
  {
    return true;
  }
  else if( (lhs.tv_sec == rhs.tv_sec) && (lhs.tv_usec < rhs.tv_usec) )
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool hekateros::operator==(const struct timeval& lhs,
                           const struct timeval& rhs)
{
  return (lhs.tv_sec == rhs.tv_sec) && (lhs.tv_usec == rhs.tv_usec)?
            true: false;
}

bool hekateros::operator>(const struct timeval& lhs, const struct timeval& rhs)
{
  return !((lhs < rhs) || (lhs == rhs));
}

struct timeval hekateros::operator+(const struct timeval& op1,
                                    const struct timeval& op2)
{
  struct timeval sum = op1;

  sum.tv_sec  += op2.tv_sec;
  sum.tv_usec += op2.tv_usec;

  if( sum.tv_usec > MILLION )
  {
    ++sum.tv_sec;
    sum.tv_usec -= MILLION;
  }

  return sum;
}

struct timeval hekateros::operator-(const struct timeval& op1,
                                const struct timeval& op2)
{
  struct timeval diff;

  diff.tv_sec = op1.tv_sec - op2.tv_sec;

  if( op1.tv_usec >= op2.tv_usec)
  {
    diff.tv_usec = op1.tv_usec - op2.tv_usec;
  }
  else
  {
    --diff.tv_sec;
    diff.tv_usec = MILLION + op1.tv_usec - op2.tv_usec;
  }

  return diff;
}

long hekateros::dt_usec(struct timeval& t1, struct timeval& t0)
{
  struct timeval  dt = t1 - t0;

  return (long)dt.tv_sec * MILLION + (long)dt.tv_usec;
}

double hekateros::dt(struct timeval& t1, struct timeval& t0)
{
  if( t0 < t1 )
  {
    return (double)dt_usec(t1, t0) / (double)MILLION;
  }
  else
  {
    return -(double)dt_usec(t0, t1) / (double)MILLION;
  }
}

bool hekateros::operator<(const struct timespec& lhs,
                          const struct timespec& rhs)
{
  if( lhs.tv_sec < rhs.tv_sec )
  {
    return true;
  }
  else if( (lhs.tv_sec == rhs.tv_sec) && (lhs.tv_nsec < rhs.tv_nsec) )
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool hekateros::operator==(const struct timespec& lhs,
                           const struct timespec& rhs)
{
  return (lhs.tv_sec == rhs.tv_sec) && (lhs.tv_nsec == rhs.tv_nsec)?
            true: false;
}

bool hekateros::operator>(const struct timespec& lhs,
                          const struct timespec& rhs)
{
  return !((lhs < rhs) || (lhs == rhs));
}

struct timespec hekateros::operator+(const struct timespec& op1,
                                     const struct timespec& op2)
{
  struct timespec sum = op1;

  sum.tv_sec  += op2.tv_sec;
  sum.tv_nsec += op2.tv_nsec;

  if( sum.tv_nsec > BILLION )
  {
    ++sum.tv_sec;
    sum.tv_nsec -= BILLION;
  }

  return sum;
}

struct timespec hekateros::operator-(const struct timespec& op1,
                                     const struct timespec& op2)
{
  struct timespec diff;

  diff.tv_sec = op1.tv_sec - op2.tv_sec;

  if( op1.tv_nsec >= op2.tv_nsec)
  {
    diff.tv_nsec = op1.tv_nsec - op2.tv_nsec;
  }
  else
  {
    --diff.tv_sec;
    diff.tv_nsec = BILLION + op1.tv_nsec - op2.tv_nsec;
  }

  return diff;
}

long long hekateros::dt_nsec(struct timespec& t1, struct timespec& t0)
{
  struct timespec dt = t1 - t0;

  return (long long)dt.tv_sec * BILLION + (long long)dt.tv_nsec;
}

double hekateros::dt(struct timespec& t1, struct timespec& t0)
{
  if( t0 < t1 )
  {
    return (double)dt_nsec(t1, t0) / (double)BILLION;
  }
  else
  {
    return -(double)dt_nsec(t0, t1) / (double)BILLION;
  }
}
 
string hekateros::getRealDeviceName(const string &strDevName)
{
  char    buf[MAX_PATH+1];
  ssize_t len;

  //
  // Symbolic link.
  //
  if( (len = readlink(strDevName.c_str(), buf, MAX_PATH)) > 0 )
  {
    buf[len] = 0;

    // absollute path
    if( buf[0] == '/' )
    {
      string strRealDevName(buf);
      return strRealDevName;
    }

    // relative path
    else
    {
      char          s[strDevName.size()+1];
      stringstream  ss;

      strcpy(s, strDevName.c_str());

      char *sDirName = dirname(s);

      ss << sDirName << "/" << buf;

      return ss.str();
    }
  }

  //
  // Real device.
  //
  else
  {
    return strDevName;
  }
}

vector<string> &hekateros::split(const string   &s,
                                 char           delim,
                                 vector<string> &elems)
{
  stringstream ss(s);
  string item;

  while( getline(ss, item, delim) )
  {
    elems.push_back(item);
  }
  return elems;
}

vector<string> hekateros::split(const string &s, char delim)
{
  vector<string> elems;

  split(s, delim, elems);

  return elems;
}
