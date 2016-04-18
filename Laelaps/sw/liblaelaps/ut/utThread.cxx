////////////////////////////////////////////////////////////////////////////////
//
// Package:   Laelaps
//
// Program:   utThread   
//
// File:      utThread.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-01-15 12:45:22 -0700 (Thu, 15 Jan 2015) $
 * $Rev: 3857 $
 *
 * \brief Unit test liblaelaps thread base class.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2015  RoadNarrows
 * (http://www.RoadNarrows.com)
 * \n All Rights Reserved
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

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/opts.h"
#include "rnr/pkg.h"

#include "rnr/Random.h"

#include "Laelaps/laelaps.h"
#include "Laelaps/laeThread.h"

#include "version.h"

using namespace std;
using namespace rnr;
using namespace laelaps;

/*!
 * \ingroup apps
 * \defgroup unittest utThread
 * \{
 */

#define APP_EC_OK       0   ///< success exit code
#define APP_EC_ARGS     2   ///< command-line options/arguments error exit code
#define APP_EC_EXEC     4   ///< execution exit code

static char    *Argv0;              ///< the command
static double   OptsHz    = 2;      ///< thread hertz rate
static bool_t   OptsRand  = false;  ///< thread random jitter

/*!
 * \brief Program information.
 */
static OptsPgmInfo_T PgmInfo =
{
  // usage_args
  NULL,

  // synopsis
  "Unit test liblaelaps LaeThread base class.",

  // long_desc = 
  "The %P command unit tests the liblaelap LaeThread base class operation.",

  // diagnostics
  NULL
};

/*!
 * \brief Command line options information.
 */
static OptsInfo_T OptsInfo[] =
{
  // -h, --hertz`
  {
    "hertz`",             // long_opt
    'u',                  // short_opt
    required_argument,    // has_arg
    true,                 // has_default
    &OptsHz,              // opt_addr
    OptsCvtArgFloat,      // fn_cvt
    OptsFmtFloat,         // fn_fmt
    NULL,                 // arg_name
                          // opt desc
    "Thread execution cycle hertz run rate."
  },

  // -r, --random`
  {
    "random`",            // long_opt
    'r',                  // short_opt
    no_argument,          // has_arg
    true,                 // has_default
    &OptsRand,            // opt_addr
    OptsCvtArgBool,       // fn_cvt
    OptsFmtBool,          // fn_fmt
    NULL,                 // arg_name
                          // opt desc
    "Do [not] randomly jitter the scheduled execution."
  },

  {NULL, }
};

class UTThread : public LaeThread   
{
public:
  UTThread() : LaeThread("UnitTest")
  {
  }

  virtual ~UTThread()
  {
  }

protected:
  Random  m_rand;

  virtual void transToReady()
  {
    printf("UTThread::transToReady, state=%d\n", m_eState);
  }

  virtual void transToRunning()
  {
    printf("UTThread::transToRunning, state=%d\n", m_eState);
  }

  virtual void transToExit()
  {
    printf("UTThread::transToExit, state=%d\n", m_eState);
  }

  virtual void exec()
  {
    ulong_t usec;
    double  f;

    LaeThread::exec();

    if( OptsRand )
    {
      f = (double)m_rand.uniform(0.0, 2.0);
      usec = (ulong_t)(f * m_fTExec * 1000000);
      usleep(usec);
    }
  }
};

int getch()
{
  struct termios old = {0};
  char           c = 0;

  if( tcgetattr(0, &old) < 0 )
  {
    return EOF;
  }

  old.c_lflag &= ~ICANON;
  old.c_lflag &= ~ECHO;
  old.c_cc[VMIN] = 1;
  old.c_cc[VTIME] = 0;

  if( tcsetattr(0, TCSANOW, &old) < 0 )
  {
    return EOF;
  }

  if( read(0, &c, 1) < 0 )
  {
    return EOF;
  }

  old.c_lflag |= ICANON;
  old.c_lflag |= ECHO;

  if( tcsetattr(0, TCSADRAIN, &old) < 0 )
  {
  }

  return c;
}

int kbhit()
{
  int   c;

  while( (c = getch()) == 0 );

  return c;
}

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
  UTThread   th;

  mainInit(argc, argv);

  printf("Press any key to terminate.\n\n");

  th.createThread(15);
  th.runThread(OptsHz);

  while( true )
  {
    //usleep(1000);
    //break;
    if( kbhit() )
    {
      break;
    }
  }

  th.terminateThread();

  return APP_EC_OK;
}

/*!
 * \}
 */
