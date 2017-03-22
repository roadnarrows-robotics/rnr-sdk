////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// Library:   librnr
//
// File:      log.c
//
/*! \file
 *
 * $LastChangedDate: 2014-12-06 13:48:30 -0700 (Sat, 06 Dec 2014) $
 * $Rev: 3823 $
 *
 * \brief Logger definitions.
 *
 * Logging supports error and diagnostics (debugging) reports to stderr or a
 * specified file. Logging reporting is filtered by a simple threshold level.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2005-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 */
//
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

#include <stdio.h>
#include <stdarg.h>
#include <time.h>
#include <string.h>
#include <pthread.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/new.h"

//
// Local variables in namespace.
//
#define LOG_VAR_FILENAME  LOGNS_PUT(LogFileName)  ///< define in namespace
#define LOG_VAR_FP        LOGNS_PUT(LogFp)        ///< define in namespace

#ifdef LOG
int LOG_VAR_THRESHOLD = LOG_LEVEL_DFT;
                        ///< Log threshold level (log iff level <= threshold)

bool_t LOG_VAR_COLOR_EN = LOG_COLOR_EN_DFT;
                        ///< Log in color (false = disable, true = enable)

static bool_t LOG_VAR_TIMESTAMP_EN = LOG_TIMESTAMP_EN_DFT;
                        ///< Timestamp logging (false = disable, true = enable)

static const char   *LOG_VAR_FILENAME = NULL;   ///< Log File Name
static FILE         *LOG_VAR_FP = NULL;         ///< Opened Log file

static pthread_mutex_t MutexLog = PTHREAD_MUTEX_INITIALIZER;
#endif // LOG

/*! 
 *  \brief Set new logging threshold level.
 *
 *  All logging at the level <= threshold level will be enabled.
 *
 *  This function is always defined allow blind sets.
 *
 *  Actual Identifier: \p \<LOGNS\>LogSetThresholdLevel()
 *
 *  \param nLevel New threshold level.
 *
 *  \return New adjusted logging level. If logging is not availble
 *  (i.e. LOG was not defined) then LOGGING_NA is returned.
 */
int LOG_SET_THRESHOLD(int nLevel)
{
#ifdef LOG
  int   nOldLevel;

  nOldLevel = LOG_VAR_THRESHOLD;

  if( nLevel < LOG_LEVEL_OFF )
  {
    LOG_VAR_THRESHOLD = LOG_LEVEL_OFF;
  }
  else
  {
    LOG_VAR_THRESHOLD = nLevel;
  }

  if( (nOldLevel >= LOG_LEVEL_DIAG1) || (LOG_VAR_THRESHOLD >= LOG_LEVEL_DIAG1) )
  {
    if( LOG_VAR_COLOR_EN )
    {
      LOGGER(LOGARGS_DIAG(2, "Logging level set to %d", LOG_VAR_THRESHOLD));
    }
    else
    {
      LOGGER(LOGARGS_DIAG_PLAIN(2, "Logging level set to %d",
            LOG_VAR_THRESHOLD));
    }
  }

  return LOG_VAR_THRESHOLD;
#else
  return LOGGING_NA;
#endif // LOG
}

/*! 
 *  \brief Get current logging threshold level.
 *
 *  This function is always defined to safeley determine if logging is 
 *  available.
 *
 *  Actual Identifier: \p \<LOGNS\>LogGetThresholdLevel()
 *
 *  \return Current logging threshold. If logging is not available,
 *  (i.e. LOG was not defined) then LOGGING_NA is returned.
 */
int LOG_GET_THRESHOLD()
{
#ifdef LOG
  return LOG_VAR_THRESHOLD;
#else
  return LOGGING_NA;
#endif // LOG
}

/*! 
 *  \brief Enable/disable logging in color.
 *
 *  This function is always defined to allow blind sets.
 *
 *  Actual Identifier: \p \<LOGNS\>LogSetColorEnable()
 *
 *  \param enable   Enable(true) or disable(false).
 */
void LOG_SET_COLOR_ENABLE(bool_t enable)
{
#ifdef LOG
  LOG_VAR_COLOR_EN = enable;
#endif // LOG
}

/*! 
 *  \brief Enable/disable log timestamps.
 *
 *  This function is always defined to allow blind sets.
 *
 *  Actual Identifier: \p \<LOGNS\>LogSetTimestampEnable()
 *
 *  \param enable   Enable(true) or disable(false).
 */
void LOG_SET_TIMESTAMP_ENABLE(bool_t enable)
{
#ifdef LOG
  LOG_VAR_TIMESTAMP_EN = enable;
#endif // LOG
}

/*! 
 * \brief Set new logging output file.
 *
 * The file is opened and truncated.
 *
 * This function is always defined allow blind sets.
 *
 * Actual Identifier: \p \<LOGNS\>LogSetLogFile()
 *
 * \todo On file open add time-stamp first entry.
 *
 * \param sLogFileName Name of new log file.
 *
 * \return On success, returns 0. On failure, return -1 with errno set
 * appropriately.
 */
int LOG_SET_LOGFILE(const char *sLogFileName)
{
#ifdef LOG
  FILE  *fp;

  // already opened
  if( (LOG_VAR_FILENAME != NULL) && !strcmp(sLogFileName, LOG_VAR_FILENAME) )
  {
    return 0;
  }

  //
  // "Open" new log file.
  //
  if( !strcmp(sLogFileName, LOG_FILENAME_STDERR) )
  {
    fp = stderr;
  }
  else if( !strcmp(sLogFileName, LOG_FILENAME_STDOUT) )
  {
    fp = stdout;
  }
  else if( (fp = fopen(sLogFileName, "a+")) == NULL )
  {
    LOGSYSERROR("%s", sLogFileName);
    return -1;
  }

  LOGDIAG1("--- End Of Log %s ---", LOG_VAR_FILENAME);

  // 
  // Close old log file.
  //
  if( (LOG_VAR_FILENAME != NULL) 
      && strcmp(LOG_VAR_FILENAME, LOG_FILENAME_STDERR) 
      && strcmp(LOG_VAR_FILENAME, LOG_FILENAME_STDOUT) )
  {
    fclose(LOG_VAR_FP);
  }

  //
  // Set new log file data
  //
  LOG_VAR_FP        = fp;
  delete((char *)LOG_VAR_FILENAME);
  LOG_VAR_FILENAME  = new_strdup(sLogFileName);
  LOGDIAG1("--- Start Of Log %s ---", LOG_VAR_FILENAME);
#endif // LOG

  return 0;
}

/*! 
 *  \brief Get logging output stream file name.
 *
 *  Actual Identifier: \p \<LOGNS\>LogGetFileName()
 *
 *  \return On success, returns 0. On failure, return -1 with errno set
 *  appropriately.
 */
#ifdef LOG
const char *LOG_GET_LOGFILE()
{
  if( LOG_VAR_FILENAME != NULL )
  {
    return LOG_VAR_FILENAME;
  }
  else
  {
    return LOG_FILENAME_DFT;
  }
}
#endif // LOG

/*! 
 * \brief Attach opened file pointer as the new logging output stream.
 *
 * Actual Identifier: \p \<LOGNS\>LogAttachLogFp()
 *
 * \param fp          Opened FILE*.
 * \param sFpFileName File name associated with fp.
 */
#ifdef LOG
void LOG_ATTACH_LOGFP(FILE *fp, const char *sFpFileName)
{
  LOG_VAR_FP        = fp;
  delete((char *)LOG_VAR_FILENAME);
  LOG_VAR_FILENAME  = new_strdup(sFpFileName);
}
#endif // LOG

/*! 
 * \brief Get current logging output stream file pointer.
 *
 * This function is handing if an application has some complicated output
 * for the logging stream.
 *
 * Actual Identifier: \p \<LOGNS\>LogGetLogFp()
 *
 * \return FILE*
 */
#ifdef LOG
FILE *LOG_GET_LOGFP()
{
  return LOG_VAR_FP;
}
#endif // LOG

/*! 
 *  \brief Print loggging diagnostics, debug, error, and system error messages
 *  to log output stream..
 *
 *  Actual Identifier: \p \<LOGNS\>LogPrintf()
 *
 *  \param sFmt   Format string.
 *  \param ...    Variable format arguments.
 */
#ifdef LOG
void LOGGER(const char *sFmt, ...)
{
  va_list   ap;

  pthread_mutex_lock(&MutexLog);

  // lazy init
  if( LOG_VAR_FP == NULL )
  {
    LOG_VAR_FP = LOG_FP_DFT;
    LOG_VAR_FILENAME = new_strdup(LOG_FILENAME_STDERR);
  }

  if( LOG_WITH_TIMESTAMP() )
  {
    struct timespec tsNow;

    clock_gettime(CLOCK_REALTIME, &tsNow);
    fprintf(LOG_VAR_FP, "[%ld.%09ld] ", tsNow.tv_sec, tsNow.tv_nsec);
  }

  va_start(ap, sFmt);
  vfprintf(LOG_VAR_FP, sFmt, ap);
  fprintf(LOG_VAR_FP, "\n");
  va_end(ap);
  fflush(LOG_VAR_FP);

  pthread_mutex_unlock(&MutexLog);
}
#endif // LOG

/*! 
 *  \brief Parse function call argument format string to determine type.
 *
 *  Type range is determined by what va_arg() takes.
 *
 *  \param sFmt Argument format string.
 *
 *  \return Quasi-specifier as in fprintf(3).
 *  Returns '?' for unreckonized formats.
 */
#ifdef LOG
static int LogVaFmtType(char *sFmt)
{
  bool_t  bPercent = false;
  char   *s;
  char    last;
  char    length = 'i';

  for(last=0, s=sFmt; s && *s; last = *s, s++)
  {
    if( *s == '%')
    {
      if( last == '%' )
      {
        bPercent = false;
        length = 'i';
      }
      else
      {
        bPercent = true;
      }
    }

    else if( bPercent )
    {
      switch( *s )
      {
        case 'h':
          length = 'h'; // short
          break;
        case 'l':
          length = 'l'; // long
          break;
        case 'c':
          return 'c';   // char
        case 'e':
        case 'E':
        case 'f':
        case 'F':
        case 'g':
        case 'G':
          return 'f';   // double
        case 'd':
        case 'i':
          switch( length )
          {
            case 'l':
              return 'l';   // long integer
            default:
              return 'd';   // integer
          }
          break;
        case 'o':
        case 'u':
        case 'x':
        case 'X':
          switch( length )
          {
            case 'l':
              return 'U';   // long unsigned
            default:
              return 'u';   // unsigned integer
          }
          break;
        case 's':
          return 's';   // string
        case 'p':
          return 'p';   // pointer
      }
    }
  }
  return '?';
}

/*! 
 *  \brief Print function call diagnostics tracing to log output stream..
 *
 *  Actual Identifier: \p \<LOGNS\>LogCallPrintf()
 *
 *  \param sPreface   Logging preface string.
 *  \param nLevel     Logging level.
 *  \param sFile      File holding function definition.
 *  \param nLine      File line number.
 *  \param sFuncName  Function name string.
 *  \param ...        Pairs of function format_string,argument pairs terminated
 *                    by NULL,0
 */
void LOGGER_CALL(const char *sPreface, int nLevel, const char *sFile, int nLine,
                  const char *sFuncName, ...)
{
  bool_t  bIsOk = true;
  char   *sArgFmt;
  char   *sSep = ""; 
  va_list ap;

  pthread_mutex_lock(&MutexLog);

  // lazy init
  if( LOG_VAR_FP == NULL )
  {
    LOG_VAR_FP = LOG_FP_DFT;
    LOG_VAR_FILENAME = new_strdup(LOG_FILENAME_STDERR);
  }

  if( LOG_WITH_TIMESTAMP() )
  {
    struct timespec tsNow;

    clock_gettime(CLOCK_REALTIME, &tsNow);
    fprintf(LOG_VAR_FP, "[%ld.%09ld] ", tsNow.tv_sec, tsNow.tv_nsec);
  }

  va_start(ap, sFuncName);

  if( LogColorEnable )
  {
    fprintf(LOG_VAR_FP,
          "%s" LOG_COLOR_DIAG "Diag%d: %s[%d] " LOG_COLOR_POST "%s(",
          sPreface, nLevel-1, sFile, nLine, sFuncName);
  }
  else
  {
    fprintf(LOG_VAR_FP,
          "%s" "Diag%d: %s[%d] " "%s(",
          sPreface, nLevel-1, sFile, nLine, sFuncName);
  }

  while( bIsOk )
  {
    // function call argument format
    sArgFmt = va_arg(ap, char *);
    if( sArgFmt == NULL )
    {
      break;
    }

    // separator
    fprintf(LOG_VAR_FP, "%s", sSep);
    sSep = ",";

    // function call argument type and print
    switch( LogVaFmtType(sArgFmt) )
    {
      case 'd':
        fprintf(LOG_VAR_FP, sArgFmt, va_arg(ap, int));
        break;
      case 'l':
        fprintf(LOG_VAR_FP, sArgFmt, va_arg(ap, long));
        break;
      case 'u':
        fprintf(LOG_VAR_FP, sArgFmt, va_arg(ap, unsigned int));
        break;
      case 'U':
        fprintf(LOG_VAR_FP, sArgFmt, va_arg(ap, unsigned long));
        break;
      case 'f':
        fprintf(LOG_VAR_FP, sArgFmt, va_arg(ap, double));
        break;
      case 's':
        fprintf(LOG_VAR_FP, sArgFmt, va_arg(ap, char *));
        break;
      case 'p':
        fprintf(LOG_VAR_FP, sArgFmt, va_arg(ap, void *));
        break;
      case 'c':
        fprintf(LOG_VAR_FP, sArgFmt, va_arg(ap, int));
        break;
      default:
        fprintf(LOG_VAR_FP, "UNKNOWN_FORMAT");
        bIsOk = false;
        break;
    }
  }

  fprintf(LOG_VAR_FP, ")\n");
  va_end(ap);

  fflush(LOG_VAR_FP);

  pthread_mutex_unlock(&MutexLog);
}
#endif // LOG
