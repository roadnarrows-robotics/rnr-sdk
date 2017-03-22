////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// File:      example_shm.c
//
/*! \file
 *
 * $LastChangedDate: 2013-02-08 12:20:58 -0700 (Fri, 08 Feb 2013) $
 * $Rev: 2676 $
 *
 * \brief Example of librnr support for shared memory.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2013-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
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

#include <time.h>
#include <stdio.h>
#include <string.h>
#include <libgen.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/opts.h"
#include "rnr/shm.h"
#include "rnr/install.h"

#include "version.h"

// 
// The command with option and argument values.
//
static char    *Argv0;                  ///< the command
static bool_t   OptsNoMutex = false;    ///< do [not] disable mutex
static bool_t   OptsNoColor = false;    ///< do [not] disable color esc sequence

/*!
 * \brief Program information.
 */
static OptsPgmInfo_T PgmInfo =
{
  .synopsis = "Example of librnr shared memory mutex.",

  .long_desc = 
    "The %P command forks to separate process, each with two threads that all "
    "write to stdout. With mutexing enabled, the output should not be "
    "intereleaved between the processes and threads."
};

/*!
 * \brief Command line options information.
 */
static OptsInfo_T OptsInfo[] =
{
  // --no-mutex
  {
    .long_opt     = "no-mutex",
    .short_opt    = OPTS_NO_SHORT,
    .has_arg      = no_argument,
    .has_default  = true,
    .opt_addr     = &OptsNoMutex,
    .fn_cvt       = OptsCvtArgBool,
    .fn_fmt       = OptsFmtBool,
    .arg_name     = NULL,
    .opt_desc     = "Disable shared memory mutex."
  },

  // --no-color
  {
    .long_opt     = "no-color",
    .short_opt    = OPTS_NO_SHORT,
    .has_arg      = no_argument,
    .has_default  = true,
    .opt_addr     = &OptsNoColor,
    .fn_cvt       = OptsCvtArgBool,
    .fn_fmt       = OptsFmtBool,
    .arg_name     = NULL,
    .opt_desc     = "Disable color coding output."
  },

  {NULL, }
};

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
  // name of this process
  Argv0 = basename(argv[0]);

  // parse input options
  argv = OptsGet(Argv0, &PkgInfo, &PgmInfo, OptsInfo, true, &argc, argv);

  return 1;
}

//
// State
//
#define N_PROCS             2   ///< number of processes
#define N_THREADS           2   ///< number of threads/process
#define WHO_CHILD           0   ///< child process
#define WHO_PARENT          1   ///< parent process
#define RED_QUEEN           0   ///< red queen identifier
#define CHESHIRE_CAT        1   ///< cheshire cat identifier
#define WHITE_QUEEN         2   ///< white queen identifier
#define MARCH_HARE          3   ///< march hare identifier
#define N_AINW_CHARS        4   ///< number of Alice in Wonderland characters
#define COLOR_RED_QUEEN     LOG_COLOR_PRE LOG_COLOR_LIGHT_RED
                                ///< red queen text color
#define COLOR_CHESHIRE_CAT  LOG_COLOR_PRE LOG_COLOR_LIGHT_YELLOW
                                ///< cheshire cat text color
#define COLOR_WHITE_QUEEN   LOG_COLOR_PRE "1;37m"
                                ///< white queen text color
#define COLOR_MARCH_HARE    LOG_COLOR_PRE LOG_COLOR_LIGHT_BLUE
                                ///< march hare text color
#define COLOR_POST          LOG_COLOR_POST
                                ///< post color string

key_t         ShmKey = 0xdead4;         ///< share memory key
shm_mutex_t   ShmMutex;                 ///< share memory mutex
uint_t        Seed;                     ///< random generator seed
int           Who;                      ///< who: child or parent process
pthread_t     Thread[N_THREADS];        ///< thread
int           ThreadState[N_THREADS];   ///< thread run/die state

char *WhoName[N_PROCS]   = {" child", "parent"};  ///< process name

/*!
 * \brief Alice in Wonderland character ids.
 */
int AinWCharId[N_PROCS][N_THREADS] = 
{
  {RED_QUEEN,   CHESHIRE_CAT},      // child process
  {WHITE_QUEEN, MARCH_HARE}         // parent process
};

/*!
 * \brief Alice in Wonderland character names.
 */
char *AinWCharName[N_AINW_CHARS] =
{
  "Red Queen",    "Cheshire Cat",   // child process
  "White Queen",  "March Hare"      // parent process
};

/*!
 * \brief Alice in Wonderland character text colors.
 */
char *Color[N_AINW_CHARS] =
{
  COLOR_RED_QUEEN,
  COLOR_CHESHIRE_CAT,
  COLOR_WHITE_QUEEN,
  COLOR_MARCH_HARE
};

/*!
 * \brief Alice in Wonderland character text.
 */
char *Say[N_AINW_CHARS] =
{
  "Off with their heads, off with their heads!",
  "Only the insane equate pain with success.",
  "But I don't forget and I don't forgive.",
  "Perhaps as this is May I won't be raving mad at least not as March."
};

/*!
 * \brief Seed random number generator.
 */
static void randseed()
{
  struct timespec t;

  clock_gettime(CLOCK_REALTIME, &t);
  Seed = (uint_t)t.tv_nsec;
}

/*!
 * \brief Sleep thread for a random time.
 *
 * Actual sleep time will be between [0, max].
 *
 * \param max   Maximum time to sleep (usec). 
 */
static void randusleep(uint_t max)
{
  uint_t    n = (uint_t)rand_r(&Seed);
  float     r;
  uint_t    t;

  r = (float)n / (float)RAND_MAX;
  t = (uint_t)((float)max * r);

  usleep(t);
}

/*!
 * \brief Lock mutex.
 */
static void lock()
{
  if( !OptsNoMutex )
  {
    shm_mutex_lock(&ShmMutex);
  }
}

/*!
 * \brief Unlock mutex.
 */
static void unlock()
{
  if( !OptsNoMutex )
  {
    shm_mutex_unlock(&ShmMutex);
  }
}

/*!
 * \brief Thread main.
 *
 * \param pArg    Pointer to character's id.
 *
 * \return Returns NULL.
 */
static void *threadmain(void *pArg)
{
  int     me = *(int *)pArg;          // character id
  int     n = me - N_THREADS * Who;   // thread index
  char   *sColorPre;                  // precolor string
  char   *sText;                      // characters text
  char   *sColorPost;                 // postcolor string

  LOGDIAG1("%s: %s thread created.", WhoName[Who], AinWCharName[me]);

  if( OptsNoColor )
  {
    sColorPre = "";
    sText = Say[me];
    sColorPost = "";
  }
  else
  {
    sColorPre = Color[me];
    sText = Say[me];
    sColorPost = COLOR_POST;
  }

  while( ThreadState[n] )
  {
    randusleep(100000);

    lock();

    //printf("%s%s%s", sColorPre, sText, sColorPost);
    printf("%s", sColorPre); usleep(10000);
    printf("%s: ", AinWCharName[me]); usleep(10000);
    printf("%s", sText); usleep(10000);
    printf("%s", sColorPost); usleep(10000);
    printf("\n");

    unlock();
  }

  LOGDIAG1("%s: %s thread killed.", WhoName[Who], AinWCharName[me]);

  return NULL;
}

/*!
 * \brief Create all threads for process.
 *
 * \return Ruturns 0 on success, -1 on failure.
 */
static int createThreads()
{
  int   i;

  for(i=0; i<N_THREADS; ++i)
  {
    ThreadState[i] = 1;

    if( pthread_create(&Thread[i], NULL, threadmain, &AinWCharId[Who][i]) != 0 )
    {
      LOGSYSERROR("%s: thread %d", WhoName[Who], i);
      return -1;
    }
  }

  return 0;
}

/*!
 * \brief Kill all threads in process.
 */
static void killThreads()
{
  int   i;

  for(i=0; i<N_THREADS; ++i)
  {
    ThreadState[i] = 0;
    pthread_join(Thread[i], NULL);
  }
}

/*!
 * \brief Run example for a process.
 */
static void runExample(int who)
{
  Who = who;

  randseed();
  randusleep(1000000);
  
  LOGDIAG1("%s: creating shared memory mutex.", WhoName[Who]);

  if( !OptsNoMutex )
  {
    if( shm_mutex_init(ShmKey, &ShmMutex) < 0 )
    {
      LOGERROR("%s: failed to create mutex.", WhoName[Who]);
    }
  }

  if( createThreads() < 0 )
  {
    return;
  }

  switch( Who )
  {
    case WHO_CHILD:
      sleep(3);
      break;
    case WHO_PARENT:
      sleep(4);
      break;
    default:
      break;
  }

  killThreads();

  if( !OptsNoMutex )
  {
    shm_mutex_destroy(&ShmMutex);
  }
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
  pid_t pid;

  if( !init(argc, argv) )
  {
    return 2;
  }

  switch( (pid = fork()) )
  {
    case -1:
      LOGERROR("Failed to fork child process.");
      return 8;
    case 0:     // child
      runExample(WHO_CHILD);
      break;
    default:    // parent
      runExample(WHO_PARENT);
      break;
  }

  return 0;
}
