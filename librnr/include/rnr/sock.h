////////////////////////////////////////////////////////////////////////////////
/*! \file
 *
 * \brief Socket services declarations.
 *
 * These socket functions were inspired by the camserv socket.c functions 
 * written by Jon Travis
 * (\ref sock_h_original_src "see below").
 *
 * \pkgsynopsis
 * RoadNarrows Robotics Common Library 1
 *
 * \pkgcomponent{Library}
 * librnr
 *
 * \pkgfile{rnr/sock.h}
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
 *
 * <hr>
 * \anchor sock_h_original_src
 * \par Original Source Comment Block
 *
 * \par Original Author
 *  Jon Travis (jtravis@p00p.org)
 *
 * \par Original Copyright
 * N/A
 *
 * \par Original Header
 * N/A
 *
 * <hr>
 */
////////////////////////////////////////////////////////////////////////////////

#ifndef _RNR_SOCK_H
#define _RNR_SOCK_H

/*! Allow IPV6 */
#define USE_IPV6

#include <sys/types.h>
#include <sys/socket.h>


#ifdef ARCH_cygwin
#endif // ARCH_cygwin

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/simplebuf.h"

//
// Cygwin work-arounds
//
#ifdef ARCH_cygwin

#endif // ARCH_cygwin

//
// Debug valves
//
#undef  debugSock         ///< general debugging
#undef  debugSockUTest    ///< unit tests

//
// Unit tests
//
#if defined(debugSockUTest) && !defined(debugSock)
#define  debugSock          ///< debug sockets
#endif

C_DECLS_BEGIN

/*!
 * Socket Flavors (a distillation of the various POSIX domains, families, 
 * protocals, and types).
 */
typedef enum
{
  SockFlavorUndef           = 0x0000, ///< undefined

  // Domain (Application Family)
  SockFlavorDomainIPv4      = 0x0001, ///< IPv4
  SockFlavorDomainIPv6      = 0x0002, ///< IPv6
  SockFlavorDomainUnix      = 0x0003, ///< UNIX (local)
  SockFlavorDomainMask      = 0x0003, ///< mask

  // Protocol
  SockFlavorProtocolTcp     = 0x0004, ///< TCP
  SockFlavorProtocolUdp     = 0x0008, ///< UDP (DGRAM)
  SockFlavorProtocolMask    = 0x000C, ///< mask

  // Function
  SockFlavorFuncListener    = 0x0010, ///< listener socket
  SockFlavorFuncConnection  = 0x0020, ///< connection/connectionless 
  SockFlavorFuncMask        = 0x0030  ///< mask

} SockFlavor;

//
// The Socket Structure
//
typedef struct socket_t   Socket_T;   ///< the Socket

//
// Socket Return Codes
//
#define SOCK_RC_OK            0     ///< no error
#define SOCK_RC_EFAIL       (-1)    ///< general, unspecified error
#define SOCK_RC_IODONE      (-2)    ///< finished current I/O
#define SOCK_RC_EBUFFULL    (-3)    ///< buffer is full
#define SOCK_RC_EBADSOCK    (-4)    ///< socket is bad
#define SOCK_RC_ENOBUF      (-5)    ///< no buffer 
#define SOCK_RC_EBADSD      (-6)    ///< bad/closed socket descriptor
#define SOCK_RC_ESYSERR     (-7)    ///< system error occurred

//
// I/O Buffer Indices
//
#define SOCK_IO_READ     0   ///< read index
#define SOCK_IO_WRITE    1   ///< write index
#define SOCK_IO_NUMOF    2   ///< number of indices

/*!
 * \brief Check I/O index
 * \param io  I/O buffer index
 */
#define SOCK_CHK_IO(io, ...) \
  CHKEXPR_INT(io, ((io) >= 0) && ((io) < SOCK_IO_NUMOF), __VA_ARGS__)

//
// Special Data Values
//
#define SOCK_SD_CLOSED        (-1)  ///< closed socket descriptor value
#define SOCK_PORT_NONE        (-1)  ///< no port number specified


//
// Prototypes
//

// ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~ 
// Socket Utilities
// ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~ 

extern char *SocketThisHostName();


// ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~ 
// Socket New/Delete Functions
// ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~ 

extern Socket_T *SocketNew();

extern void SocketDelete(Socket_T *pSocket);


// ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~ 
// Socket Read/Write Buffer Functions
// ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~ 

extern SimpleBuf_T *SocketBufNewBuf(Socket_T *pSocket,
                                    int       eIOBuf,
                                    size_t    nBufSize);

extern SimpleBuf_T *SocketBufSetBuf(Socket_T *pSocket,
                                    int       eIOBuf, 
                                    byte_t   *pBuf,
                                    size_t    nBufSize);

extern SimpleBuf_T *SocketBufGetBuf(Socket_T *pSocket, int eIOBuf);

extern byte_t *SocketBufGetRawBuf(Socket_T *pSocket, int eIOBuf);

extern size_t SocketBufGetSize(Socket_T *pSocket, int eIOBuf);

extern size_t SocketBufGetLen(Socket_T *pSocket, int eIOBuf);

extern void SocketBufClear(Socket_T *pSocket, int eIOBuf);


// ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~ 
// Socket Attribute Access Functions
// ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~ 

// Client Data Attributes
extern void *SocketAttrGetClientData(Socket_T *pSocket);

extern void SocketAttrSetClientData(Socket_T *pSocket, void *pClientData);

// Local Name Attributes
extern const char *SocketAttrGetLocalName(Socket_T *pSocket);

extern void SocketAttrSetLocalName(Socket_T *pSocket, const char *sHostName);

// Remote Name Attributes
extern const char *SocketAttrGetRemoteName(Socket_T *pSocket);

// Remote Name
extern void SocketAttrSetRemoteName(Socket_T        *pSocket,
                                    struct sockaddr *pAddr,
                                    const char      *sHostName);

// Socket Descriptor Attributes
extern int SocketAttrGetSd(Socket_T *pSocket);

extern void SocketAttrSetSd(Socket_T *pSocket, int sd);

// IP Port Attributes
extern int SocketAttrGetPort(Socket_T *pSocket);

extern void SocketAttrSetPort(Socket_T *pSocket, int ipPort);

// Socket Flavor and State Attributes
extern SockFlavor SocketAttrGetFlavor(Socket_T *pSocket);

extern void SocketAttrSetFlavor(Socket_T *pSocket, SockFlavor eFlavor);

extern bool_t SocketAttrHasFlavor(Socket_T *pSocket, int nMask, int nFlavor);

/*!
 * \brief Test if socket is a listener flavor.
 *
 * \param pSocket Pointer to socket.
 *
 * \return Returns true or false.
 */
INLINE_IN_H bool_t SocketAttrIsListener(Socket_T *pSocket)
{
  return SocketAttrHasFlavor(pSocket, 
                             SockFlavorFuncMask,
                             SockFlavorFuncListener);
}

/*!
 * \brief Test if socket is connection flavor.
 *
 * \param pSocket Pointer to socket.
 *
 * \return Returns true or false.
 */
INLINE_IN_H bool_t SocketAttrIsConnection(Socket_T *pSocket)
{
  return SocketAttrHasFlavor(pSocket, 
                             SockFlavorFuncMask,
                             SockFlavorFuncConnection);
}

/*!
 * \brief Test if socket is connectionless flavor.
 *
 * \param pSocket Pointer to socket.
 *
 * \return Returns true or false.
 */
INLINE_IN_H bool_t SocketAttrIsConnectionless(Socket_T *pSocket)
{
  return SocketAttrIsConnection(pSocket)? false: true;
}

extern int SocketAttrSetNonBlocking(Socket_T *pSocket);


// ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~ 
// Socket State Functions
// ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~ 

extern bool_t SocketStateIsOpen(Socket_T *pSocket);

extern bool_t SocketStateIsErrored(Socket_T *pSocket);


// ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~ 
// Socket Open/Close Functions
// ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~ 

extern Socket_T *SocketOpenTcpListener(int         ipPortListener, 
                                       const char *sLocalHostName);

extern Socket_T *SocketOpenTcpConnection(const char *sRemoteHostName, 
                                         int         ipPortRemote);

extern Socket_T *SocketOpenUdp(int ipPortUdp, const char *sLocalHostName);

extern Socket_T *SocketOpenUnix(int ipPortUdp, int nType);

extern Socket_T *SocketOpenUnixPair(int nType);

extern Socket_T *SocketAccept(Socket_T *pSocketListener, bool_t bNonBlock);

extern int SocketClose(Socket_T *pSocket);


// ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~ 
// Socket Read/Write Functions
// ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~ 

extern ssize_t SocketRead(Socket_T *pSocket);

extern ssize_t SocketWrite(Socket_T *pSocket);

extern ssize_t SocketWriteAll(Socket_T *pSocket);

#ifdef debugSockUTest
/*! Unit Test Function. */
extern void SocketUTest();
#endif

C_DECLS_END


#endif // _RNR_SOCK_H
