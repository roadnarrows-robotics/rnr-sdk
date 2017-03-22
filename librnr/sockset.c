////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// Library:   librnr
//
// File:      sockset.c
//
/*! \file
 *
 * $LastChangedDate: 2010-05-03 13:38:47 -0600 (Mon, 03 May 2010) $
 * $Rev: 362 $
 *
 * \brief Socket Sets services definitions.
 *
 *  A socket set is composed of three open socket read/write subsets:
 *  <table class="rnr-def">
 *  <tr>
 *    <td>Active</td>
 *    <td>SD's (socked descriptors) active for read/write I/O.</td>
 *  </tr>
 *  <tr>
 *    <td>OnHold</td>
 *    <td>SD's managed but are on hold (not select()'ed).</td>
 *  </tr>
 *  <tr>
 *    <td>Select</td>
 *    <td>The select() working subset. The Active subset is copied into
 *        the Select set and passed into select(). On select()'s return,
 *        the Select set is the subset SD's with I/O events.</td>
 *  </tr>
 *  </table>
 *
 * \note
 *  These socket functions were inspired by the <em>camserv</em> socket.c
 *  functions written by Jon Travis (jtravis@p00p.org)
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

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#include "rnr/rnrconfig.h"
#include "rnr/sock.h"
#include "rnr/sockset.h"
#include "rnr/log.h"
#include "rnr/new.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------


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

/*!
 * \brief FD_CLR() wrapper with no annoying warnings.
 * \param fd    File descriptor to add to set.
 * \param pset  Pointer to fd set.
 */
static inline void fdclr_nowarn(int fd, fd_set *pset)
{
  FD_CLR(fd, pset);
}

/*!
 * \brief FD_ISSET() wrapper with no annoying warnings.
 * \param fd    File descriptor to add to set.
 * \param pset  Pointer to fd set.
 * \return Returusn non-zero if set, else zero.
 */
static inline int fdisset_nowarn(int fd, fd_set *pset)
{
  return FD_ISSET(fd, pset);
}
PRAGMA_WARNING(sign-conversion)

PRAGMA_IGNORED(conversion)
/*!
 * \brief ntohs() wrapper with no annoying warnings.
 * \param huShort Unsigned short integer.
 * \return Host byte order.
 */
static inline ushort_t ntohs_nowarn(ushort_t huShort)
{
  return ntohs(huShort);
}

/*!
 * \brief htons() wrapper with no annoying warnings.
 * \param huShort Unsigned short integer.
 * \return Network byte order.
 */
static inline ushort_t htons_nowarn(ushort_t huShort)
{
  return htons(huShort);
}
PRAGMA_WARNING(conversion)

/*!
 * \brief Socket Set Structure
 */
struct sockset_t
{
  int       m_sdHighest;                  ///< highest socket descriptor value
  fd_set    m_sdSetSelect[SOCK_IO_NUMOF]; ///< select socket descriptor set
  fd_set    m_sdSetActive[SOCK_IO_NUMOF]; ///< active socket descriptor set
  fd_set    m_sdSetOnHold[SOCK_IO_NUMOF]; ///< on-hold socket descriptor set
  Socket_T *m_pSocket[FD_SETSIZE];        ///< associated sockets
};

/*! 
 * \brief Find the highest socket descriptor number in the Active subset.
 *
 * \param pSockSet  Pointer to socket set.
 *
 * \return Highest socket descriptor. -1 if none found.
 */
static int SockSetFindHighestSd(SockSet_T *pSockSet)
{
  int   sd;
  int   sdHighest;

  for(sd=0, sdHighest=-1; sd <= pSockSet->m_sdHighest; ++sd)
  {
    if(   fdisset_nowarn(sd, &(pSockSet->m_sdSetActive[SOCK_IO_READ]))
       || fdisset_nowarn(sd, &(pSockSet->m_sdSetActive[SOCK_IO_WRITE])) )
    {
      if( sd > sdHighest )
      {
        sdHighest = sd;
      }
    }
  }

  return sdHighest;
}

/*! 
 * \brief Check if socket is open and within valid range.
 *
 * \param pSocket   Pointer to socket.
 *
 * \return
 * Returns socket descriptor if valid.\n
 * Otherwise the appropriate socket return code \< 0 is returned.
 */
static int SockSetChkSd(Socket_T *pSocket)
{
  int   sd;   // socket descriptor

  CHKPTR(pSocket, SOCK_RC_EBADSOCK);

  // this socket isn't open
  if( !SocketStateIsOpen(pSocket) )
  {
    LOGERROR("Socket not open");
    return SOCK_RC_EBADSD;
  }

  sd = SocketAttrGetSd(pSocket);

  if( sd >= FD_SETSIZE )
  {
    LOGERROR("Socket '%s' descriptor %d > %d: too big", 
        SocketAttrGetLocalName(pSocket), sd, FD_SETSIZE);
    return SOCK_RC_EBADSD;
  }

  return sd;
}


// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Allocate a new SockSet.
 *
 * \return Pointer to allocated SockSet.
 */
SockSet_T *SockSetNew()
{
  SockSet_T *pSockSet = NEW(SockSet_T);

  pSockSet->m_sdHighest = -1;

  FD_ZERO(&(pSockSet->m_sdSetSelect[SOCK_IO_READ]));
  FD_ZERO(&(pSockSet->m_sdSetSelect[SOCK_IO_WRITE]));
  FD_ZERO(&(pSockSet->m_sdSetActive[SOCK_IO_READ]));
  FD_ZERO(&(pSockSet->m_sdSetActive[SOCK_IO_WRITE]));
  FD_ZERO(&(pSockSet->m_sdSetOnHold[SOCK_IO_READ]));
  FD_ZERO(&(pSockSet->m_sdSetOnHold[SOCK_IO_WRITE]));

  return pSockSet;
}

/*!
 * \brief Delete an allocated SockSet.
 *
 * \param pSockSet  Pointer to SockSet.
 */
void SockSetDelete(SockSet_T *pSockSet)
{
  if( pSockSet == NULL )
  {
    return;
  }
  delete(pSockSet);
}

/*!
 * \brief Add a Socket to a SockSet.
 *
 * Socket is added to either the on-hold or active socket set, depending on the
 * given boolean value.
 *
 * \param pSockSet        Pointer to SockSet.
 * \param pSocket         Pointer to Socket.
 * \param bActivateRead   Do [not] add to active read set.
 * \param bActivateWrite  Do [not] add to active write set.
 *
 * \return Returns OK(0) on success, \h_le 0 on error.
 */
int SockSetAdd(SockSet_T *pSockSet,
               Socket_T  *pSocket, 
               bool_t     bActivateRead,
               bool_t     bActivateWrite)
{
  int   sd;   // socket descriptor

  CHKPTR(pSockSet, SOCK_RC_EBADSOCK);

  // Check data and return socket descriptor
  if( (sd = SockSetChkSd(pSocket)) < 0 )
  {
    return sd;
  }

  LOGDIAG4CALL(_TPTR(pSockSet), _TSTR(SocketAttrGetLocalName(pSocket)),
      _TBOOL(bActivateRead), _TBOOL(bActivateWrite));

  //
  // Add sd to appropriate read socket descriptor set
  //
  if( bActivateRead ) // place read in the active set
  {
    fdset_nowarn(sd, &(pSockSet->m_sdSetActive[SOCK_IO_READ]));
    if( pSockSet->m_sdHighest < sd )
    {
      pSockSet->m_sdHighest = sd;
    }
  }
  else  // place read in the on-hold set
  {
    fdset_nowarn(sd, &(pSockSet->m_sdSetOnHold[SOCK_IO_READ]));
  }

  //
  // Add sd to appropriate write socket descriptor set
  //
  if( bActivateWrite ) // place write in the active set
  {
    fdset_nowarn(sd, &(pSockSet->m_sdSetActive[SOCK_IO_WRITE]));
    if( pSockSet->m_sdHighest < sd )
    {
      pSockSet->m_sdHighest = sd;
    }
  }
  else  // place write in the on-hold set
  {
    fdset_nowarn(sd, &(pSockSet->m_sdSetOnHold[SOCK_IO_WRITE]));
  }

  // add socket 
  pSockSet->m_pSocket[sd] = pSocket;

  return OK;
}

/*!
 * \brief Remove Socket from SockSet.
 *
 * \param pSockSet  Pointer to SockSet.
 * \param pSocket   Pointer to Socket.
 *
 * \return Returns OK(0) on success, \h_le 0 on error.
 */
int SockSetRemove(SockSet_T *pSockSet, Socket_T *pSocket)
{
  int   sd;   // socket descriptor

  CHKPTR(pSockSet, SOCK_RC_EBADSOCK);

  // Check data and return socket descriptor
  if( (sd = SockSetChkSd(pSocket)) < 0 )
  {
    return sd;
  }

  LOGDIAG4CALL(_TPTR(pSockSet), _TSTR(SocketAttrGetLocalName(pSocket)));

  // remove socket descriptor from all sets
  fdclr_nowarn(sd, &(pSockSet->m_sdSetActive[SOCK_IO_READ]));
  fdclr_nowarn(sd, &(pSockSet->m_sdSetOnHold[SOCK_IO_READ]));
  fdclr_nowarn(sd, &(pSockSet->m_sdSetActive[SOCK_IO_WRITE]));
  fdclr_nowarn(sd, &(pSockSet->m_sdSetOnHold[SOCK_IO_WRITE]));

  // remove socket
  pSockSet->m_pSocket[sd] = NULL;

  // find new highest socket descriptor
  pSockSet->m_sdHighest = SockSetFindHighestSd(pSockSet);

  return OK;
}

/*!
 * \brief Activate Socket read/write operations in SockSet.
 *
 * Socket is already part of the SockSet.
 *
 * \param pSockSet        Pointer to SockSet.
 * \param pSocket         Pointer to Socket.
 * \param bActivateRead   Do [not] add to active read set.
 * \param bActivateWrite  Do [not] add to active write set.
 *
 * \return Returns OK(0) on success, \h_le 0 on error.
 */
int SockSetActivate(SockSet_T *pSockSet,
                    Socket_T  *pSocket, 
                    bool_t     bActivateRead,
                    bool_t     bActivateWrite)
{
  int   sd;   // socket descriptor

  CHKPTR(pSockSet, SOCK_RC_EBADSOCK);

  // Check data and return socket descriptor
  if( (sd = SockSetChkSd(pSocket)) < 0 )
  {
    return sd;
  }

  if( bActivateRead )
  {
    fdclr_nowarn(sd, &(pSockSet->m_sdSetOnHold[SOCK_IO_READ]));
    fdset_nowarn(sd, &(pSockSet->m_sdSetActive[SOCK_IO_READ]));
    if( sd > pSockSet->m_sdHighest )
    {
      pSockSet->m_sdHighest = sd;
    }
  }

  if( bActivateWrite )
  {
    fdclr_nowarn(sd, &(pSockSet->m_sdSetOnHold[SOCK_IO_WRITE]));
    fdset_nowarn(sd, &(pSockSet->m_sdSetActive[SOCK_IO_WRITE]));
    if( sd > pSockSet->m_sdHighest )
    {
      pSockSet->m_sdHighest = sd;
    }
  }

  pSockSet->m_sdHighest = SockSetFindHighestSd(pSockSet);

  return OK;
}

/*!
 * \brief Put Socket read/write operations in SockSet on hold.
 *
 * Socket is already part of the SockSet.
 *
 * \param pSockSet    Pointer to SockSet.
 * \param pSocket     Pointer to Socket.
 * \param bHoldRead   Do [not] add to hold read set.
 * \param bHoldWrite  Do [not] add to hold write set.
 *
 * \return Returns OK(0) on success, \h_le 0 on error.
 */
int SockSetPutOnHold(SockSet_T *pSockSet,
                     Socket_T  *pSocket,
                     bool_t     bHoldRead,
                     bool_t     bHoldWrite)
{
  int   sd;   // socket descriptor

  CHKPTR(pSockSet, SOCK_RC_EBADSOCK);

  // Check data and return socket descriptor
  if( (sd = SockSetChkSd(pSocket)) < 0 )
  {
    return sd;
  }

  if( bHoldRead )
  {
    fdclr_nowarn(sd, &(pSockSet->m_sdSetActive[SOCK_IO_READ]));
    fdset_nowarn(sd, &(pSockSet->m_sdSetOnHold[SOCK_IO_READ]));
  }

  if( bHoldWrite )
  {
    fdclr_nowarn(sd, &(pSockSet->m_sdSetActive[SOCK_IO_WRITE]));
    fdset_nowarn(sd, &(pSockSet->m_sdSetOnHold[SOCK_IO_WRITE]));
  }

  pSockSet->m_sdHighest = SockSetFindHighestSd(pSockSet);

  return OK;
}

/*!
 * \brief Perform select() on SockSet's Active sets.
 *
 * \param pSockSet  Pointer to SockSet.
 * \param pTimeOut  Select time out.
 *
 * \return 
 * On success, returns number of socket descriptors found in the active state.\n
 * On time out, returns 0.\n
 * On error, returns \h_le 0.
 */
int SockSetSelect(SockSet_T *pSockSet, struct timeval *pTimeOut)
{
  struct timeval  tvSave = {0, 0};  // saved timeout value
  int             nSd;              // number of socket descriptors selected
  int             nTries = 0;       // number of tries when interrupted

  CHKPTR(pSockSet, SOCK_RC_EBADSOCK);

  // save timeout value - can get munged by select()
  if( pTimeOut != NULL )
  {
    tvSave = *pTimeOut;
  }

  //
  // Try at most three times to select.
  //
  while(nTries++ < 3)
  {
    // build select read/write sets
    pSockSet->m_sdSetSelect[SOCK_IO_READ] = 
                                      pSockSet->m_sdSetActive[SOCK_IO_READ];
    pSockSet->m_sdSetSelect[SOCK_IO_WRITE] = 
                                      pSockSet->m_sdSetActive[SOCK_IO_WRITE];

    //
    // Wait with timeout (optional) for read/write availability
    //
    nSd = select(pSockSet->m_sdHighest+1,
                 &(pSockSet->m_sdSetSelect[SOCK_IO_READ]),
                 &(pSockSet->m_sdSetSelect[SOCK_IO_WRITE]),
                 NULL,
                 pTimeOut);

    // restore mangled time out
    if( pTimeOut != NULL )
    {
      *pTimeOut = tvSave;
    }

    // system error occurred
    if( nSd < 0 )
    {
      switch(errno)
      {
        case EINTR: // interrupted - try again
          break;
        default:
          LOGSYSERROR("select()");
          return SOCK_RC_ESYSERR;
      }
    }

    // read/write sets available or timeout
    else
    {
      return nSd;
    }
  }

  LOGERROR("Failed %d times to select()", nTries);

  return SOCK_RC_EFAIL;
}

/*!
 * \brief Start iteration of Sockets in the given set in SockSet.
 *
 * \param pSockSet  Pointer to SockSet.
 * \param eSet      One of: \ref SOCK_SET_ONHOLD \ref SOCK_SET_ACTIVE
 *                          \ref SOCK_SET_SELECTED.
 * \param eIO       One of: \ref SOCK_IO_READ \ref SOCK_IO_WRITE.
 * \param pIter     SockSet interator.
 *
 * \return Pointer to first Socket if found, else NULL.
 */
Socket_T *SockSetIterFirst(SockSet_T     *pSockSet,
                           int            eSet,
                           int            eIO,
                           SockSetIter_T *pIter)
{
  CHKPTR(pSockSet, NULL);
  SOCK_SET_CHK_SET(eSet, NULL);
  SOCK_CHK_IO(eIO, NULL);
  CHKPTR(pIter, NULL);

  pIter->m_pSockSet = pSockSet;
  pIter->m_eSet     = eSet;
  pIter->m_eIO      = eIO;
  pIter->m_sdCur    = -1;

  switch(eSet)
  {
    case SOCK_SET_ONHOLD:
      pIter->m_sdSet = pSockSet->m_sdSetOnHold[eIO];
      break;
    case SOCK_SET_ACTIVE:
      pIter->m_sdSet = pSockSet->m_sdSetActive[eIO];
      break;
    case SOCK_SET_SELECTED:
    default:
      pIter->m_sdSet = pSockSet->m_sdSetSelect[eIO];
      break;
  }

  return SockSetIterNext(pIter);
}

/*!
 * \brief Next Socket in iteration over the initialized socket set of SockSet.
 *
 * \param pIter     SockSet interator.
 *
 * \return Pointer to next Socket if found, else NULL.
 */
Socket_T *SockSetIterNext(SockSetIter_T *pIter)
{
  int   sd;

  CHKPTR(pIter, NULL);
  CHKPTR(pIter->m_pSockSet, NULL);

  for(sd = pIter->m_sdCur + 1; sd <= pIter->m_pSockSet->m_sdHighest; ++sd)
  {
    if( fdisset_nowarn(sd, &(pIter->m_sdSet)) )
    {
      pIter->m_sdCur = sd;
      return pIter->m_pSockSet->m_pSocket[sd];
    }
  }

  // no more selected sockets found
  pIter->m_sdCur = sd;

  return NULL;
}
