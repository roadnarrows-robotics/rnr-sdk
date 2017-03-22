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
 * \copyright
 *   \h_copy 2005-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
 *
 * \license{MIT}
 *
 * \EulaBegin
 * Permission is hereby granted, without written agreement and without
 * license or royalty fees, to use, copy, modify, and distribute this
 * software and its documentation for any purpose, provided that
 * (1) The above copyright notice and the following two paragraphs
 * appear in all copies of the source code and (2) redistributions
 * including binaries reproduces these notices in the supporting
 * documentation.   Substantial modifications to this software may be
 * copyrighted by their authors and need not follow the licensing terms
 * described here, provided that the new terms are clearly indicated in
 * all files where they apply.
 * \n\n
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
 * OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 * \n\n
 * THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
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
