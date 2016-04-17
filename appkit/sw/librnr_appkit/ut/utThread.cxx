////////////////////////////////////////////////////////////////////////////////
//
// Package:   appkit
//
// Program:   utThread   
//
// File:      utThread.cxx
//
/*! \file
 *
 * $LastChangedDate: 2015-11-09 17:38:34 -0700 (Mon, 09 Nov 2015) $
 * $Rev: 4195 $
 *
 * \brief Unit test Thread base class.
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

#include "rnr/Thread.h"
#include "rnr/Random.h"

#include "version.h"

using namespace std;
using namespace rnr;

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
  "Unit test librnr_appkit Thread base class.",

  // long_desc = 
  "The %P command unit tests the librnr_appkit Thread base class operation.",

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

class UTThread : public Thread   
{
public:
  UTThread() : Thread("UnitTest")
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

    Thread::exec();

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
    //usleep(500);
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
