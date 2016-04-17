////////////////////////////////////////////////////////////////////////////////
//
// Package:   BotSense
//
// Library:   libbsclient
//
// File:      bsLibUtils.c
//
/*! \file
 *
 * $LastChangedDate: 2011-10-19 15:05:47 -0600 (Wed, 19 Oct 2011) $
 * $Rev: 1398 $
 *
 * \brief Library utilities.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2009-2010.  RoadNarrows LLC.
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
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

#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <string.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "cbLibInternal.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------


// ---------------------------------------------------------------------------
// Internal Interface 
// ---------------------------------------------------------------------------

/*! 
 * \brief Mark the current time. Resolution is microseconds.
 *
 * \param pTvMark   Pointer to timeval structure to be populated with
 *                  the current system time in seconds and useconds.
 */
void timer_mark(struct timeval *pTvMark)
{
  if( gettimeofday(pTvMark, NULL) != OK )
  {
    LOGSYSERROR("gettimeofday()");
    timerclear(pTvMark);
  }
}

/*! 
 * \brief Calculate the elapsed time between the given time mark and this call.
 *
 * \param pTvMark   Pointer to timeval holding time mark.
 *
 * \return 
 * Number of microseconds elasped. If the marked time is invalid or the current
 * time cannot be ascertained, UINT_MAX is returned.
 */
uint_t timer_elapsed(struct timeval *pTvMark)
{
  struct timeval  tvEnd, tvDelta;

  timer_mark(&tvEnd);

  if( !timerisset(pTvMark) || !timerisset(&tvEnd) )
  {
    return UINT_MAX;
  }

  tvDelta.tv_sec = tvEnd.tv_sec - pTvMark->tv_sec;
  if( tvEnd.tv_usec < pTvMark->tv_usec )
  {
    tvDelta.tv_sec--;
    tvEnd.tv_usec += 1000000;
  }
  tvDelta.tv_usec = tvEnd.tv_usec - pTvMark->tv_usec;

  return (uint_t)(tvDelta.tv_sec * 1000000 + tvDelta.tv_usec);
}

#ifdef LOG
/*! 
 * \brief Log data bytes.
 *
 * Print diagnostic logging of the contents of a buffer of bytes.
 *
 * \param sBufName  Name of buffer.
 * \param buf       Buffer.
 * \param uCount    Number of bytes to log.
 */
void bsLogBuf(const char *sBufName, byte_t buf[], size_t uCount)
{
  FILE   *fp;   // log file pointer
  size_t  i;    // working index

  fp = LOG_GET_LOGFP();

  fprintf(fp, "%sDiag%d: %s=", LOG_PREFACE, LOG_GET_THRESHOLD()-1, sBufName);
  for(i=0; i<uCount; ++i)
  {
    if( (i % 16) == 0 )
    {
      fprintf(fp, "\n");
    }
    fprintf(fp, " 0x%02x", buf[i]);
  }
  fprintf(fp, "\n");
}
#endif // LOG

#ifdef LOG
/*! 
 * \brief Log ascii data bytes.
 *
 * Print diagnostic logging of the contents of a buffer of ASCII bytes.
 *
 * \param sBufName  Name of buffer.
 * \param buf       Buffer.
 * \param uCount    Number of bytes to log.
 */
void bsLogAsciiBuf(const char *sBufName, byte_t buf[], size_t uCount)
{
  FILE   *fp;   // log file pointer
  size_t  i;    // working index

  fp = LOG_GET_LOGFP();

  fprintf(fp, "%sDiag%d: %s=", LOG_PREFACE, LOG_GET_THRESHOLD()-1, sBufName);
  for(i=0; i<uCount; ++i)
  {
    if( isprint(buf[i]) || isspace(buf[i]) )
    {
      switch(buf[i])
      {
        case '\f':
          fprintf(fp, "\\f");
          break;
        case '\n':
          fprintf(fp, "\\n");
          break;
        case '\r':
          fprintf(fp, "\\r");
          break;
        case '\t':
          fprintf(fp, "\\t");
          break;
        case '\v':
          fprintf(fp, "\\v");
          break;
        default:
          fprintf(fp, "%c", buf[i]);
          break;
      }
    }
    else
    {
      fprintf(fp, "\\x%02x", buf[i]);
    }
  }
  fprintf(fp, "\n");
}
#endif // LOG
