////////////////////////////////////////////////////////////////////////////////
/*! \file
 *
 * \brief Socket Sets services definitions.
 *
 * \note
 *  These socket functions were inspired by the <em>camserv</em> socket.c
 *  functions written by Jon Travis (jtravis@p00p.org)
 *
 * \pkgsynopsis
 * RoadNarrows Robotics Common Library 1
 *
 * \pkgcomponent{Library}
 * librnr
 *
 * \pkgfile{rnr/sockset.h}
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \pkgcopyright{2005-2018,RoadNarrows LLC.,http://www.roadnarrows.com}
 *
 * \license{MIT}
 *
 * \EulaBegin
 * See the README and EULA files for any copyright and licensing information.
 * \EulaEnd
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _RNR_SOCKSET_H
#define _RNR_SOCKSET_H

#include <sys/time.h>
#include <sys/select.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/sock.h"

C_DECLS_BEGIN


/*!
 * The Socket Set Structure
 */
typedef struct sockset_t  SockSet_T;

/*!
 * Socket Set Iterator Structor
 */
typedef struct
{
  SockSet_T *m_pSockSet;  ///< socket set 
  int        m_eSet;      ///< which socket set to iterate over
  int        m_eIO;       ///< which read/write subset to iterate over
  int        m_sdCur;     ///< current iterator position
  fd_set     m_sdSet;     ///< set to iterate over
} SockSetIter_T;

//
// Socket Set Enumeration
//
#define SOCK_SET_ONHOLD     0   ///< on-hold socket descriptor set
#define SOCK_SET_ACTIVE     1   ///< active socket descriptor set
#define SOCK_SET_SELECTED   2   ///< post select() selected socket desc. set
#define SOCK_SET_NUMOF      3   ///< number of managed socket descriptor sets

/*!
 * \brief Check socket set enum.
 * \param set   Socket descriptor set enum.
 */
#define SOCK_SET_CHK_SET(set, ...) \
  CHKEXPR_INT(set, ((set) >= 0) && ((set) < SOCK_SET_NUMOF), __VA_ARGS__)


//------------------------------------------------------------------------------
// Prototypes
//------------------------------------------------------------------------------

extern SockSet_T *SockSetNew();

extern void SockSetDelete(SockSet_T *pSockSet);

extern int SockSetAdd(SockSet_T  *pSockSet,
                      Socket_T   *pSocket, 
                      bool_t      bActivateRead,
                      bool_t      bActivateWrite);

extern int SockSetRemove(SockSet_T *pSockSet, Socket_T *pSocket);

extern int SockSetActivate(SockSet_T *pSockSet,
                           Socket_T  *pSocket, 
                           bool_t     bActivateRead,
                           bool_t     bActivateWrite);

extern int SockSetPutOnHold(SockSet_T  *pSockSet,
                            Socket_T   *pSocket,
                            bool_t      bHoldRead,
                            bool_t      bHoldWrite);

extern int SockSetSelect(SockSet_T *pSockSet, struct timeval *pTimeOut);

extern Socket_T *SockSetIterFirst(SockSet_T      *pSockSet,
                                  int             eSet,
                                  int             eIO,
                                  SockSetIter_T  *pIter);

extern Socket_T *SockSetIterNext(SockSetIter_T *pIter);

C_DECLS_END


#endif // _RNR_SOCKSET_H
