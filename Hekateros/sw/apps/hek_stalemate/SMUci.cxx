////////////////////////////////////////////////////////////////////////////////
//
// Package:   Hekateros
//
// Program:   StaleMate
//
// File:      SMUci.cxx
//
/*! \file
 *
 * $LastChangedDate: 2012-06-13 10:47:00 -0600 (Wed, 13 Jun 2012) $
 * $Rev: 2043 $
 *
 * \brief StaleMate Universal Chess Interface.
 *
 * \author Rob Shiely (rob@roadnarrows.com)
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * \author Daniel Packard (daniel@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2011.  RoadNarrows
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 */
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
////////////////////////////////////////////////////////////////////////////////

#include <sys/types.h>
#include <sys/wait.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <signal.h>
#include <libgen.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <cstring>
#include <iostream>
#include <fstream>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"

#include "rnr/rnrWin.h"

#include "StaleMate.h"
#include "StaleMateTune.h"

using namespace std;
using namespace rnrWin;


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

// switch
#define UCI_TRACE_ENABLE

#ifdef UCI_TRACE_ENABLE

#define UCI_TRACE_FILE "ucitrace.log"

static FILE *FpUciTrace = NULL;

#define UCI_TRACE(fmt, ...) \
  do \
  { \
    if( FpUciTrace != NULL )\
    { \
      fprintf(FpUciTrace, fmt, ##__VA_ARGS__); \
      fflush(FpUciTrace); \
    } \
  } while(0)

#define UCI_TRACE_OPEN() \
  do \
  { \
    if( (FpUciTrace = fopen(UCI_TRACE_FILE, "w")) != NULL ) \
    { \
      UCI_TRACE("### Start UCI Tracing.\n\n"); \
    } \
  } while(0)

#else

#define UCI_TRACE(fmt, ...)
#define UCI_TRACE_OPEN()

#endif // UCI_TRACE_ENABLE


#define UCI_MAX_BUF_SIZE      80

// (partial) response strings
const char* const UciRspCheckMate   = "mates";
const char* const UciRspDraw        = "1/2";
const char* const UciRspResign      = "resign";
const char* const UciRspInvalidMove = "Invalid move";

static int    UciPipeToChess[2];
static int    UciPipeFromChess[2];
static pid_t  UciPidChild;
static bool   UciWaitingForEngineMoveRsp;
static char   UciEngineMoveRspBuf[UCI_MAX_BUF_SIZE];
static int    UciMovePairCnt;

PRAGMA_IGNORED(sign-conversion)
/*!
 * \brief FD_SET() wrapper with no annoying warnings.
 * \param fd    File descriptor to add to set.
 * \param pset  Pointer to fd set.
 */
static inline void fdset_nowarn(int fd, fd_set *pset)
{
  FD_SET(fd, pset);
}
PRAGMA_WARNING(sign-conversion)

/*! 
 * \brief Mark the current time. Resolution is microseconds.
 *
 * \param pTvMark   Pointer to timeval structure to be populated with
 *                  the current system seconds and useconds.
 */
static inline void timer_mark(struct timeval *pTvMark)
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

static int UciReadLine(char buf[], size_t sizeBuf)
{
  uint_t          usec = 100000;    // 1/10th second
  struct timeval  timeout;
  fd_set          rset;
  int             fd = UciPipeFromChess[0];
  int             nFd;
  ssize_t         n;
  ssize_t         nBytes = 0;
  bool            bDone = false;

  buf[0] = 0;

  //
  // Read the data until either 1) count bytes are read, 2) a time out occurs,
  // or 3) an error occurs.
  //
  while( (nBytes < sizeBuf-1) && !bDone )
  {
    FD_ZERO(&rset);
    fdset_nowarn(fd, &rset);

    // (re)load timeout (gets munged after each select())
    timeout.tv_sec  = (time_t)(usec / 1000000);
    timeout.tv_usec = (time_t)(usec % 1000000);

    // wait to bytes to be available to be read
    nFd = select(fd+1, &rset, NULL, NULL, &timeout);

    // system error occurred on select - interpret
    if( nFd < 0 )
    {
      switch(errno)
      {
        case EAGAIN:        // non-blocking timeout, retry
        case EINTR:         // read was interrupted, retry
          break;
        default:            // non-recoverable error
          buf[nBytes] = 0;
          LOGSYSERROR("select(%d,...)", fd);
          return -HEK_ECODE_SYS;
      }
    }

    // select() timeout occurred
    else if( nFd == 0 )
    {
      LOGDIAG4("select() on read timed out.");
      bDone = true;
    }

    // read the available data from the pipe
    else if( (n = read(fd, buf+nBytes, (size_t)1)) == 1 )
    {
      switch( buf[nBytes] )
      {
        case '\n':    // received a response line
          buf[nBytes] = 0;
          //LOGDIAG4("%s(): %zd bytes response line.", LOGFUNCNAME, nBytes);
          return HEK_OK;
        case '\r':    // received a character, but ignore
          break;
        default:      // received a character
          // got some data
          nBytes += 1;
          break;
      }
    }

    // system error occurred on read - interpret
    else if( n < 0 )
    {
      switch(errno)
      {
        case EAGAIN:        // non-blocking timeout, try again
        case EINTR:         // read was interrupted, try again
          break;
        default:            // non-recoverable error
          buf[nBytes] = 0;
          LOGSYSERROR("select(%d,...)", fd);
          return -HEK_ECODE_SYS;
      }
    }

    // got a select but no bytes 
    else if( n == 0 )
    {
      LOGDIAG4("select() ok, but read()=0.");
      bDone = true;
    }
  }

  LOGDIAG4("%s(): %zd bytes partial response line.", LOGFUNCNAME, nBytes);
  
  buf[nBytes] = 0;

  return -HEK_ECODE_TIMEDOUT;
}

static void UciFlushIn()
{
  char    buf[256];

  UCI_TRACE("UCI: Flush:\n", buf);
  while( UciReadLine(buf, sizeof(buf)) == HEK_OK )
  {
    UCI_TRACE("%s", buf);
  }
  UCI_TRACE("\nUCI: End Flush\n");
}


static int UciParseMoveRsp(const char  bufRsp[],
                           const char *sExpectedMove,
                           char        bufAlgSqFrom[],
                           char        bufAlgSqTo[])
{
  int   n;
  char  bufEcho[UCI_MAX_BUF_SIZE];
  
  bufAlgSqFrom[0] = 0;
  bufAlgSqTo[0] = 0;

  n = 0;
  bufEcho[0] = 0;

  if( strcasestr(bufRsp, UciRspCheckMate) != NULL )
  {
    bufAlgSqFrom[0] = UciCodeCheckMate;
    bufAlgSqFrom[1] = 0;
    bufAlgSqTo[0]   = UciCodeCheckMate;
    bufAlgSqTo[1]   = 0;

    return HEK_OK;
  }

  else if( !strncasecmp(bufRsp, UciRspResign, strlen(UciRspResign)) )
  {
    bufAlgSqFrom[0] = UciCodeResign;
    bufAlgSqFrom[1] = 0;
    bufAlgSqTo[0]   = UciCodeResign;
    bufAlgSqTo[1]   = 0;

    return HEK_OK;
  }

  else if( !strncasecmp(bufRsp, UciRspDraw, strlen(UciRspDraw)) )
  {
    bufAlgSqFrom[0] = UciCodeDraw;
    bufAlgSqFrom[1] = 0;
    bufAlgSqTo[0]   = UciCodeDraw;
    bufAlgSqTo[1]   = 0;

    return HEK_OK;
  }

  else if( !strncasecmp(UciEngineMoveRspBuf, UciRspInvalidMove,
                                              strlen(UciRspInvalidMove)) )
  {
    bufAlgSqFrom[0] = UciCodeInvalid;
    bufAlgSqFrom[1] = 0;
    bufAlgSqTo[0]   = UciCodeInvalid;
    bufAlgSqTo[1]   = 0;
  }

  sscanf(bufRsp, "%d. %s", &n, bufEcho);
  
  if( (n == 0) || (bufEcho[0] == 0) )
  {
    LOGERROR("UCI: Bad response '%s'", bufRsp);
    return -HEK_ECODE_GEN;
  }

  else if( n != UciMovePairCnt ) 
  {
    LOGERROR("UCI: Response move out-of-sequence: Expected %d, got %d.",
        UciMovePairCnt, n);
    return -HEK_ECODE_GEN;
  }

  else if( (sExpectedMove != NULL) && strcasecmp(sExpectedMove, bufEcho) )
  {
    LOGERROR("UCI: Response move unexpected: Expected '%s', got '%s'.",
        sExpectedMove, bufEcho);
    return -HEK_ECODE_GEN;
  }

  else if( !strncasecmp(UciEngineMoveRspBuf, UciRspResign,
                                              strlen(UciRspResign)) )
  {
      bufAlgSqFrom[0] = UciCodeResign;
      bufAlgSqFrom[1] = 0;
      bufAlgSqTo[0]   = UciCodeResign;
      bufAlgSqTo[1]   = 0;
  }

  else if( !strncasecmp(UciEngineMoveRspBuf, UciRspDraw, strlen(UciRspDraw)) )
  {
    bufAlgSqFrom[0] = UciCodeDraw;
    bufAlgSqFrom[1] = 0;
    bufAlgSqTo[0]   = UciCodeDraw;
    bufAlgSqTo[1]   = 0;
  }

  else if( !strncasecmp(UciEngineMoveRspBuf, UciRspInvalidMove,
                                              strlen(UciRspInvalidMove)) )
  {
    bufAlgSqFrom[0] = UciCodeInvalid;
    bufAlgSqFrom[1] = 0;
    bufAlgSqTo[0]   = UciCodeInvalid;
    bufAlgSqTo[1]   = 0;
  }

  else
  {
    bufAlgSqFrom[0] = bufEcho[0];
    bufAlgSqFrom[1] = bufEcho[1];
    bufAlgSqFrom[2] = 0;

    bufAlgSqTo[0]   = bufEcho[2];
    bufAlgSqTo[1]   = bufEcho[3];
    bufAlgSqTo[2]   = 0;
  }

  return HEK_OK;
}

// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

// RDK TODO make variable arguments
int StaleMateUciOneTimeInit(StaleMateSession &session, const char *sChessApp)
{
  char   *argv[8];

  // make stalemate to chess app pipe
  if( pipe(UciPipeToChess) == -1 )
  {
    LOGSYSERROR("Failed to create pipe to chess application.");
    return -HEK_ECODE_SYS;
   }

  // make chess app to stalemate pipe
  if ( pipe(UciPipeFromChess) == -1 )
  {
    LOGSYSERROR("Failed to create pipe from chess application.");
    return -HEK_ECODE_SYS;
  }

  // for stalemate
  UciPidChild = fork();

  // fork fail error
  if( UciPidChild == -1 )
  {
    LOGSYSERROR("Could not fork %s.", sChessApp);
    return -HEK_ECODE_SYS;
  }

  //
  // Child Process
  //
  if( UciPidChild == 0 )
  {
    // close unused pipe ends 
    close(UciPipeToChess[1]);
    close(UciPipeFromChess[0]);

    // duplciate pipe ends to standard input and output
    dup2(UciPipeToChess[0], 0);
    dup2(UciPipeFromChess[1], 1);

    // build exec arguments
    argv[0] = (char *)sChessApp;
    //argv[1] = NULL;
    //argv[1] = (char *)"-x";
    //argv[1] = (char *)"-u";
    argv[2] = NULL;

    // exec chess application
    execvp(argv[0], argv);

    // if here then exec failed
    exit(1);
  }

  //
  // Parent Process
  //
  else
  {
    LOGDIAG1("Forked-exec'ed %s, child pid=%u.", sChessApp, UciPidChild);

    // close unused pipe ends 
    close(UciPipeToChess[0]);
    close(UciPipeFromChess[1]);

    if( fcntl(UciPipeFromChess[0], F_SETFL, O_NONBLOCK) < 0 )
    {
      LOGSYSERROR("fcntl(%d,F_SETFL,O_NONBLOCK)", UciPipeFromChess[0]);
      return -HEK_ECODE_SYS;
    }

    UCI_TRACE_OPEN();

    sleep(1);

    StaleMateUciCfg(session);

    return HEK_OK;
  }
}

void StaleMateUciKill(StaleMateSession &session)
{
  if( !session.m_game.bUseChessEngine )
  {
    return;
  }

  kill(UciPidChild, SIGTERM);
  waitpid(UciPidChild, NULL, WEXITED);
}

int StaleMateUciTrans(StaleMateSession &session,
                      const char       *sReq,
                      char              bufRsp[],
                      size_t            sizeRspBuf)
{
  ssize_t   n;
  size_t    i;
  int       rc;

  if( !session.m_game.bUseChessEngine )
  {
    return HEK_OK;
  }

  //UciFlushIn();

  if( sizeRspBuf > 0 )
  {
    UCI_TRACE("UCI: Req: \"%s\"\n", sReq);
  }
  else
  {
    UCI_TRACE("UCI: Cmd: \"%s\"\n", sReq);
  }

  n = write(UciPipeToChess[1], sReq, strlen(sReq));
  n = write(UciPipeToChess[1], "\n", 1);

  if( n < 0 )
  {
    LOGSYSERROR("write(%d, \"%s\", %uz)",
                        UciPipeToChess[1], sReq, strlen(sReq));
    return -HEK_ECODE_SYS;
  }

  if( sizeRspBuf > 0 )
  {
    rc = UciReadLine(bufRsp, sizeRspBuf);
    UCI_TRACE("UCI: Rsp: \"%s\"\n", bufRsp);
    return rc;
  }
  else
  {
    return HEK_OK;
  }
}

void StaleMateUciCfg(StaleMateSession &session)
{
  char    bufRsp[UCI_MAX_BUF_SIZE];
  int     n;

  if( !session.m_game.bUseChessEngine )
  {
    return;
  }

  UciFlushIn();

  // place in xboard mode
  StaleMateUciTrans(session, "xboard", NULL, 0);

  StaleMateUciTrans(session, "depth 4", bufRsp, sizeof(bufRsp));
  LOGDIAG3("depth rsp: %s", bufRsp);

  // TODO other engine config here

  return;
}

void StaleMateUciNewGame(StaleMateSession &session)
{
  char    bufRsp[UCI_MAX_BUF_SIZE];
  int     rc;

  if( !session.m_game.bUseChessEngine )
  {
    return;
  }

  StaleMateUciTrans(session, "new", NULL, 0);

  UciMovePairCnt = 1;

  // hekateros is white, make the first move.
  if( session.m_game.bHekHasWhite )
  {
    UciWaitingForEngineMoveRsp = true;
    UciEngineMoveRspBuf[0] = 0;
    StaleMateUciTrans(session, "go", NULL, 0);
  }
  else
  {
    UciWaitingForEngineMoveRsp = false;
  }

  UciFlushIn();
}

// get move response from engine
bool StaleMateUciGetEnginesMove(StaleMateSession &session,
                                char             bufAlgSqFrom[],
                                char             bufAlgSqTo[])
{
  char  bufRsp[UCI_MAX_BUF_SIZE];
  int   rc;

  rc = UciReadLine(bufRsp, sizeof(bufRsp));

  UCI_TRACE("UCI: Eng: \"%s\"\n", bufRsp);

  switch( rc )
  {
    case HEK_OK:
      strcat(UciEngineMoveRspBuf, bufRsp);
      LOGDIAG3("Engine's Response Move: '%s'.", UciEngineMoveRspBuf);

      rc = UciParseMoveRsp(UciEngineMoveRspBuf, NULL, bufAlgSqFrom, bufAlgSqTo);

      if( rc == HEK_OK )
      {
        // hekateros is black - advace the move pair count
        if( !session.m_game.bHekHasWhite )
        {
          UciMovePairCnt++;
        }

        UciWaitingForEngineMoveRsp = false;
      }
      return true;

    case HEK_ECODE_TIMEDOUT:
      strcat(UciEngineMoveRspBuf, bufRsp);
      LOGDIAG3("Engine's Response Move (partial): '%s'.", UciEngineMoveRspBuf);
      return false;

    default:
      UciEngineMoveRspBuf[0] = 0;
      return false;
  }
}

int StaleMateUciPlayersMove(StaleMateSession &session,
                            const char       *sAlgSqFrom,
                            const char       *sAlgSqTo,
                            char             *pUciCode)
{
  char  bufMove[UCI_MAX_BUF_SIZE];
  char  bufRsp[UCI_MAX_BUF_SIZE];
  char  bufAlgSqFrom[UCI_MOVE_BUF_SIZE];
  char  bufAlgSqTo[UCI_MOVE_BUF_SIZE];
  int   rc;

  sprintf(bufMove, "%s%s", sAlgSqFrom, sAlgSqTo);

  rc = StaleMateUciTrans(session, bufMove, bufRsp, sizeof(bufRsp));

  if( rc != DYNA_OK )
  {
    LOGERROR("UCI: No response to opponent's move %s.", bufMove);
    return rc;
  }

  rc = UciParseMoveRsp(bufRsp, bufMove, bufAlgSqFrom, bufAlgSqTo);

  if( rc == HEK_OK )
  {
    // move was not a valid move
    switch( bufAlgSqFrom[0] )
    {
      case UciCodeInvalid:
      case UciCodeError:
        *pUciCode = UciCodeInvalid;
        return -HEK_ECODE_NO_EXEC;
      case UciCodeDraw:
      case UciCodeResign:
      case UciCodeCheckMate:
        *pUciCode = bufAlgSqFrom[0];
      default:
        *pUciCode = 0;
        break;
    }

    // opponent is black - advace the move pair count
    if( session.m_game.bHekHasWhite )
    {
      UciMovePairCnt++;
    }

    // heks move - wait for engine's response
    UciWaitingForEngineMoveRsp = true;
    UciEngineMoveRspBuf[0] = 0;
  }

  return rc;
}

void StaleMateUciSelfPlay(StaleMateSession &session, bool bIsNewGame)
{
  static int nMoves;

  char    bufRsp[UCI_MAX_BUF_SIZE];

  if( bIsNewGame )
  {
    session.m_game.bHekHasWhite = true;
    StaleMateUciTrans(session, "new", NULL, 0);
    StaleMateUciTrans(session, "depth 4", bufRsp, sizeof(bufRsp));
    UciMovePairCnt = 0;
    nMoves = 0;
    UciFlushIn();
  }

  UciWaitingForEngineMoveRsp = true;
  UciEngineMoveRspBuf[0] = 0;

  StaleMateUciTrans(session, "go", NULL, 0);

  ++nMoves;
  if( nMoves & 0x01 )
  {
    ++UciMovePairCnt;
  }
}
