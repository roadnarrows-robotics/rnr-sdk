////////////////////////////////////////////////////////////////////////////////
//
// Package:   RoadNarrows Robotics Common Library 1
//
// Library:   librnr
//
// File:      sock.c
//
/*! \file
 *
 * $LastChangedDate: 2012-07-10 12:37:42 -0600 (Tue, 10 Jul 2012) $
 * $Rev: 2078 $
 *
 * \brief Socket services definitions.
 *
 * These socket functions were inspired by the camserv socket.c functions 
 * written by Jon Travis (see below).
 *
 * \todo Socket_T *SocketOpenUnix(int ipPortUdp, int nType)
 * \todo Socket_T *SocketOpenUnixPair(int nType)
 * \todo fix cygwin shortcommings.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2005-2010.  RoadNarrows LLC.
 * (http://www.roadnarrows.com) \n
 * All Rights Reserved
 *
 * <hr>
 * \par Original Source and Copyright:
 *
 * \par Original Author:
 *  Jon Travis (jtravis@p00p.org)
 *
 * <hr>
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

#define USE_IPV6    ///< enable IPV6 (Cygwin)

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>

#include "rnr/rnrconfig.h"
#include "rnr/sock.h"
#include "rnr/simplebuf.h"
#include "rnr/log.h"
#include "rnr/new.h"


// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

#ifndef HOST_NAME_MAX
#define HOST_NAME_MAX 64      ///< maximum host name length
#endif

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
 * Special Host Names
 */
const char *SockStrClosed         = "** Closed **";       ///< closed socket
const char *SockStrDisconnected   = "** Disconnected **"; ///< disconnected
const char *SockStrAnyHost        = "ANYHOST";            ///< any host
const char *SockStrLocalHost      = "LOCALHOST";          ///< local host
const char *SockStrUnknownHost    = "UNKNOWNHOST";        ///< unknown host

/*!
 * Socket Structure
 */
struct socket_t
{
  int            m_sd;                    ///< (open) socket descriptor
  int            m_ipPort;                ///< socket port
  SockFlavor     m_eFlavor;               ///< socket type
  char          *m_sLocalName;            ///< local name
  char          *m_sRemoteName;           ///< remote name
  SimpleBuf_T   *m_pIOBuf[SOCK_IO_NUMOF]; ///< read/write buffers
  void          *m_pClientData;           ///< client data
};

/*!
 * \brief Resolve host name into IP address (IPV4 only).
 *
 * \param sHostName Host name in dotted or DNS network format. 
 * \param pAddrInet IP address
 *
 * \return Returns 0 on success, -1 on failure.
 */
static int SockResolveHostName(const char *sHostName, struct in_addr *pAddrInet)
{
  struct hostent  *pHostEntry;
  struct in_addr   addrInet;

  // hostname: "www.xxx.yyy.zzz" (IPv4)
  if( inet_aton(sHostName, &addrInet) != 0 )
  {
    memcpy(pAddrInet, &addrInet, sizeof(addrInet));
  }
  // hostname: "www.xxx.yyy.zzz" (IPv4) or "networkname" (may use DNS)
  else if( (pHostEntry = gethostbyname(sHostName)) != NULL )
  {
    memcpy(pAddrInet, pHostEntry->h_addr, (size_t)pHostEntry->h_length);
  }
  else
  {
    LOGERROR("gethostbyname(%s): %s", sHostName, hstrerror(h_errno));
    return -1; 
  }

  return OK;
}

/*!
 * \brief Allocate formated internet name (IPV4, IPV6).
 *
 * \par Generated Name Format:
 * \verbatim
 * addrname_inet:
 *    hostname '('dottedaddr')'
 *    hostname '('dottedaddr port')'
 * hostname:
 *    given
 *    lookup
 *    dottedaddr
 * dottedaddr:
 *    ipv4_format
 *    ipv6_format
 * port: 
 *    ':' number
 * \endverbatim
 *
 * \param af        Internet socket protocol family.
 * \param pAddr     Socket address.
 * \param sHostName Host name. NULLL to force lookup.
 *
 * \return Returns pointer to allocated name string.
 */
static char *SocketAddrNameInet(int              af,
                                struct sockaddr *pAddr, 
                                const char      *sHostName)
{
  void   *pAddrIn;
  size_t  nAddrSize;
  char    bufAddr[INET6_ADDRSTRLEN];
  uint_t  uPort;
  char    bufPort[16];
  char    bufHost[NI_MAXHOST];
  char   *sHost;
  char   *sAddrName;

  bufAddr[0] = 0;
  bufPort[0] = 0;
  bufHost[0] = 0;

  // ipv4
  if( af == AF_INET )
  {
    nAddrSize = sizeof(struct sockaddr_in);
    pAddrIn = &(((struct sockaddr_in *)pAddr)->sin_addr);
    uPort = (uint_t)(((struct sockaddr_in *)pAddr)->sin_port);
  }
  // ipv6
  else if( af == AF_INET6 )
  {
    nAddrSize = sizeof(struct sockaddr_in6);
    pAddrIn = &(((struct sockaddr_in6 *)pAddr)->sin6_addr);
    uPort = (uint_t)(((struct sockaddr_in6 *)pAddr)->sin6_port);
  }
  // bug
  else
  {
    return NULL;
  }

  // dotted address
  if( inet_ntop(af, pAddrIn, bufAddr, (socklen_t)sizeof(bufAddr)) != NULL )
  {
    if( uPort != 0 )
    {
      snprintf(bufPort, sizeof(bufPort), ":%hu", ntohs_nowarn((ushort_t)uPort));
      bufPort[sizeof(bufPort)-1] = 0;
    }
  }
  else
  {
    LOGSYSERROR("inet_ntop(%d, ...)", af);
    strcpy(bufAddr, "ADDR???");
  }

  // use given hostname
  if( sHostName != NULL )
  {
    sHost = (char *)sHostName;
  }
 
  // any host address
  else if(   (af == AF_INET) 
          && (((struct in_addr *)pAddrIn)->s_addr == INADDR_ANY) )
  {
    sHost = (char *)SockStrAnyHost;
  }

  // lookup hostname (makes dotted if cannot find)
  else
  {
    sHost = bufHost;
    getnameinfo(pAddr, (socklen_t)nAddrSize,
                bufHost, (socklen_t)sizeof(bufHost),
                NULL, (socklen_t)0, NI_NOFQDN);
  }

  sAddrName = NEWSTR(strlen(sHost)+strlen(bufAddr)+strlen(bufPort)+3);

  sprintf(sAddrName, "%s(%s%s)", sHost, bufAddr, bufPort);

  return sAddrName;
}

/*!
 * \brief Allocate formated UNIX name.
 *
 * \par Generated Name Format:
 * \verbatim
 * addrname_unix:
 *    hostname '('unixpath')'
 * hostname:
 *    given
 *    localhost
 *    "LOCALHOST"
 * \endverbatim
 *
 * \param pAddrUnix UNIX socket address.
 * \param sHostName Host name. NULLL to force lookup.
 *
 * \return Returns pointer to allocated name string.
 */
static char *SocketAddrNameUnix(struct sockaddr_un *pAddrUnix, 
                                const char         *sHostName)
{
  char    bufHost[NI_MAXHOST];
  char   *sHost;
  char   *sAddrName;

  // use given hostname
  if( sHostName != NULL )
  {
    sHost = (char *)sHostName;
  }
  // lookup local hostname
  else if( gethostname(bufHost, sizeof(bufHost)) < 0 )
  {
    sHost = bufHost;
  }
  else
  {
    sHost = (char *)SockStrLocalHost;
  }

  sAddrName = NEWSTR(strlen(sHost)+strlen(pAddrUnix->sun_path)+3);

  sprintf(sAddrName, "%s(%s)", sHost, pAddrUnix->sun_path);

  return sAddrName;
}

/*!
 * \brief Allocate formated address name.
 *
 * \par Generated Name Format:
 * \verbatim
 * addrname:
 *    addrname_inet
 *    addrname_unix
 * \endverbatim
 *
 * \param pAddr     Socket address.
 * \param sHostName Host name. NULLL to force lookup.
 *
 * \return Returns pointer to allocated name string.
 */
static char *SocketAddrName(struct sockaddr *pAddr, const char *sHostName)
{
  int   af;

  af = pAddr->sa_family;

  switch(af)
  {
    case AF_INET:
    case AF_INET6:
      return SocketAddrNameInet(af, pAddr, sHostName);
    case AF_UNIX:
      return SocketAddrNameUnix((struct sockaddr_un *)pAddr, sHostName);
    default:
      return new_strdup("AF???");
  }
}


// ---------------------------------------------------------------------------
// Public Interface 
// ---------------------------------------------------------------------------

// ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~ 
// Socket Utilities
// ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~ 

/*!
 * \brief Get the host string name of this network node (computer).
 *
 * \return Returns pointer to allocated name string.
 */
char *SocketThisHostName()
{
  char             sHostName[HOST_NAME_MAX];
  char             sAddr[INET6_ADDRSTRLEN];
  struct hostent  *pHostEntry;
  char            *p;

  // get the host name of this processor
  if( gethostname(sHostName, sizeof(sHostName)) == -1 )
  {
    LOGSYSERROR("gethostname(...)");
    return new_strdup(SockStrUnknownHost);
  }
  sHostName[sizeof(sHostName)-1] = 0;

  // get this host entry info
  if( (pHostEntry = gethostbyname(sHostName)) == NULL )
  {
    LOGERROR("gethostbyname(%s): %s", sHostName, hstrerror(h_errno));
    return new_strdup(sHostName);
  }

  // dotted address to string
  if( pHostEntry->h_addr_list  && pHostEntry->h_addr_list[0] )
  {
    if( inet_ntop(pHostEntry->h_addrtype, pHostEntry->h_addr_list[0],
                  sAddr, (socklen_t)sizeof(sAddr)) == NULL )
    {
      LOGSYSERROR("inet_ntop(%d, ...)", pHostEntry->h_addrtype);
      strcpy(sAddr, "ADDR???");
    }
  }
  else
  {
    strcpy(sAddr, "ADDR???");
  }
  sAddr[sizeof(sAddr)-1] = 0;

  p = NEWSTR(strlen(sHostName)+strlen(sAddr)+2);
  sprintf(p, "%s(%s)", sHostName, sAddr);

  return p;
}


// ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~ 
// Socket New/Delete Functions
// ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~ 

/*!
 * \brief Allocate a new, unconnected socket.
 *
 * \return Returns pointer to allocated Socket_T *.
 */
Socket_T *SocketNew()
{
  Socket_T  *pSocket = NEW(Socket_T);

  pSocket->m_sd               = SOCK_SD_CLOSED;
  pSocket->m_ipPort           = SOCK_PORT_NONE;
  pSocket->m_eFlavor          = SockFlavorUndef;

  SocketAttrSetLocalName(pSocket, SockStrClosed);
  SocketAttrSetRemoteName(pSocket, NULL, SockStrDisconnected);

  pSocket->m_pIOBuf[SOCK_IO_READ]  = SimpleBufNew();
  pSocket->m_pIOBuf[SOCK_IO_WRITE] = SimpleBufNew();

  pSocket->m_pClientData      = NULL;

  return pSocket;
}

/*!
 * \brief Delate an allocated socket.
 *
 * \warning Socket should be closed.
 */
void SocketDelete(Socket_T *pSocket)
{
  if( pSocket == NULL )
  {
    return;
  }

  delete(pSocket->m_sLocalName);
  delete(pSocket->m_sRemoteName);
  SimpleBufDelete(pSocket->m_pIOBuf[SOCK_IO_READ]);
  SimpleBufDelete(pSocket->m_pIOBuf[SOCK_IO_WRITE]);
  delete(pSocket);
}


// ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~ 
// Socket Buffer Functions
// ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~ 

/*!
 * \brief Allocate a socket buffer and attach to Socket.
 *
 * \param pSocket   Pointer to Socket.
 * \param eIOBuf    One of: \ref SOCK_IO_READ \ref SOCK_IO_WRITE.
 * \param nBufSize  Buffer size (bytes).
 *
 * \return Returns pointer to newly allocated buffer. 
 */
SimpleBuf_T *SocketBufNewBuf(Socket_T *pSocket, int eIOBuf, size_t nBufSize)
{
  CHKPTR(pSocket, NULL);
  SOCK_CHK_IO(eIOBuf, NULL);

  SimpleBufDelete(pSocket->m_pIOBuf[eIOBuf]);
  pSocket->m_pIOBuf[eIOBuf]  = SimpleBufNewWithBuf(nBufSize);
  return pSocket->m_pIOBuf[eIOBuf];
}

/*!
 * \brief Attach a buffer to Socket.
 *
 * \param pSocket   Pointer to Socket.
 * \param eIOBuf    One of: \ref SOCK_IO_READ \ref SOCK_IO_WRITE.
 * \param pBuf      Pointer to buffer.
 * \param nBufSize  Buffer size (bytes).
 *
 * \return Returns pointer to buffer. 
 */
SimpleBuf_T *SocketBufSetBuf(Socket_T *pSocket,
                             int       eIOBuf,
                             byte_t   *pBuf,
                             size_t    nBufSize)
{
  CHKPTR(pSocket, NULL);
  SOCK_CHK_IO(eIOBuf, NULL);

  SimpleBufSetBuf(pSocket->m_pIOBuf[eIOBuf], pBuf, nBufSize, (size_t)0);
  return pSocket->m_pIOBuf[eIOBuf];
}

/*!
 * \brief Get Socket buffer.
 *
 * \param pSocket   Pointer to Socket.
 * \param eIOBuf    One of: \ref SOCK_IO_READ \ref SOCK_IO_WRITE.
 *
 * \return Returns pointer to simplebuffer. 
 */
SimpleBuf_T *SocketBufGetBuf(Socket_T *pSocket, int eIOBuf)
{
  CHKPTR(pSocket, NULL);
  SOCK_CHK_IO(eIOBuf, NULL);

  return pSocket->m_pIOBuf[eIOBuf];
}

/*!
 * \brief Get Socket raw buffer.
 *
 * \param pSocket   Pointer to Socket.
 * \param eIOBuf    One of: \ref SOCK_IO_READ \ref SOCK_IO_WRITE.
 *
 * \return Returns pointer to byte buffer. 
 */
byte_t *SocketBufGetRawBuf(Socket_T *pSocket, int eIOBuf)
{
  CHKPTR(pSocket, NULL);
  SOCK_CHK_IO(eIOBuf, NULL);

  return pSocket->m_pIOBuf[eIOBuf]->m_pRWBuf;
}

/*!
 * \brief Get Socket buffer size.
 *
 * \param pSocket   Pointer to Socket.
 * \param eIOBuf    One of: \ref SOCK_IO_READ \ref SOCK_IO_WRITE.
 *
 * \return Returns size of buffer in bytes.
 */
size_t SocketBufGetSize(Socket_T *pSocket, int eIOBuf)
{
  CHKPTR(pSocket, (size_t)0);
  SOCK_CHK_IO(eIOBuf, (size_t)0);

  return SimpleBufHasSize(pSocket->m_pIOBuf[eIOBuf]);
}

/*!
 * \brief Get number of bytes in Socket buffer.
 *
 * \param pSocket   Pointer to Socket.
 * \param eIOBuf    One of: \ref SOCK_IO_READ \ref SOCK_IO_WRITE.
 *
 * \return Returns number of bytes currently in buffer.
 */
size_t SocketBufGetLen(Socket_T *pSocket, int eIOBuf)
{
  CHKPTR(pSocket, (size_t)0);
  SOCK_CHK_IO(eIOBuf, (size_t)0);

  return SimpleBufHasLen(pSocket->m_pIOBuf[eIOBuf]);
}

/*!
 * \brief Clear a Socket buffer.
 *
 * \param pSocket   Pointer to Socket.
 * \param eIOBuf    One of: \ref SOCK_IO_READ \ref SOCK_IO_WRITE.
 */
void SocketBufClear(Socket_T *pSocket, int eIOBuf)
{
  CHKPTR(pSocket);
  SOCK_CHK_IO(eIOBuf);

  SimpleBufClear(pSocket->m_pIOBuf[eIOBuf]);
}


// ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~ 
// Socket Attribute Access Functions
// ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~ 

/*!
 * \brief Get Socket client user data.
 *
 * \param pSocket   Pointer to Socket.
 *
 * \return Returns pointer to user defined data.
 */
void *SocketAttrGetClientData(Socket_T *pSocket)
{
  CHKPTR(pSocket, NULL);
  return pSocket->m_pClientData;
}

/*!
 * \brief Get Socket client user data.
 *
 * \param pSocket     Pointer to Socket.
 * \param pClientData User defined data.
 */
void SocketAttrSetClientData(Socket_T *pSocket, void *pClientData)
{
  CHKPTR(pSocket);
  pSocket->m_pClientData = pClientData;
}

/*!
 * \brief Get Socket local name.
 *
 * \param pSocket     Pointer to Socket.
 *
 * \return Returns string.
 */
const char *SocketAttrGetLocalName(Socket_T *pSocket)
{
  CHKPTR(pSocket, NULL);
  return pSocket->m_sLocalName;
}

/*!
 * \brief Set Socket local name.
 *
 * \param pSocket     Pointer to Socket.
 * \param sHostName   Local host name string.
 */
void SocketAttrSetLocalName(Socket_T *pSocket, const char *sHostName)
{
  struct sockaddr_un addr;    // largest address size (is there a better way?)
  socklen_t          sizAddr;

  CHKPTR(pSocket);

  delete(pSocket->m_sLocalName);

  sizAddr = (socklen_t)sizeof(addr);

  // closed
  if( !SocketStateIsOpen(pSocket) )
  {
    pSocket->m_sLocalName = new_strdup(SockStrClosed);
  }

  // error leg - simply use host name
  else if( getsockname(pSocket->m_sd, (struct sockaddr *)&addr, &sizAddr) < 0 )
  {
    LOGSYSERROR("getsockname(%d, ...)", pSocket->m_sd);
    pSocket->m_sLocalName = new_strdup(sHostName);
  }

  // get local address
  else 
  {
    pSocket->m_sLocalName = SocketAddrName((struct sockaddr *)&addr, sHostName);
  }
}

/*!
 * \brief Get Socket remote name.
 *
 * \param pSocket     Pointer to Socket.
 *
 * \return Returns string.
 */
const char *SocketAttrGetRemoteName(Socket_T *pSocket)
{
  CHKPTR(pSocket, NULL);
  return pSocket->m_sRemoteName;
}

/*!
 * \brief Set Socket remote name.
 *
 * \param pSocket     Pointer to Socket.
 * \param pAddr       Socket address.
 * \param sHostName   Remote host name string.
 */
void SocketAttrSetRemoteName(Socket_T        *pSocket,
                             struct sockaddr *pAddr,
                             const char      *sHostName)
{
  CHKPTR(pSocket);

  delete(pSocket->m_sRemoteName);

  // closed
  if( !SocketStateIsOpen(pSocket) )
  {
    pSocket->m_sRemoteName = new_strdup(SockStrDisconnected);
  }

  // set preferred address named
  else if( pAddr != NULL )
  {
    pSocket->m_sRemoteName = SocketAddrName(pAddr, sHostName);
  }

  // simply use host name
  else if( sHostName != NULL )
  {
    pSocket->m_sRemoteName = new_strdup(sHostName);
  }

  // error
  else
  {
    pSocket->m_sRemoteName = new_strdup(SockStrDisconnected);
  }
}

/*!
 * \brief Get Socket socket (file) descriptor.
 *
 * \param pSocket   Pointer to Socket.
 *
 * \return Returns socket descriptor.
 */
int SocketAttrGetSd(Socket_T *pSocket)
{
  CHKPTR(pSocket, SOCK_SD_CLOSED);
  return pSocket->m_sd;
}

/*!
 * \brief Set Socket socket (file) descriptor.
 *
 * \param pSocket   Pointer to Socket.
 * \param sd        Socket descriptor.
 */
void SocketAttrSetSd(Socket_T *pSocket, int sd)
{
  CHKPTR(pSocket);
  if( (pSocket->m_sd = sd) < 0 )
  {
    SocketClose(pSocket);
  }
}

/*!
 * \brief Get Socket port number.
 *
 * \param pSocket   Pointer to Socket.
 *
 * \return Returns port number.
 */
int SocketAttrGetPort(Socket_T *pSocket)
{
  CHKPTR(pSocket, SOCK_PORT_NONE);
  return pSocket->m_ipPort;
}

/*!
 * \brief Set Socket port number.
 *
 * \param pSocket   Pointer to Socket.
 * \param ipPort    Port number.
 */
void SocketAttrSetPort(Socket_T *pSocket, int ipPort)
{
  CHKPTR(pSocket);
  pSocket->m_ipPort = ipPort;
}

/*!
 * \brief Get Socket flavor.
 *
 * \param pSocket   Pointer to Socket.
 *
 * \return Returns flavor enumeration.
 */
SockFlavor SocketAttrGetFlavor(Socket_T *pSocket)
{
  CHKPTR(pSocket, SockFlavorUndef);
  return pSocket->m_eFlavor;
}

/*!
 * \brief Set Socket flavor.
 *
 * \param pSocket   Pointer to Socket.
 * \param eFlavor   Flavor enumeration.
 */
void SocketAttrSetFlavor(Socket_T *pSocket, SockFlavor eFlavor)
{
  CHKPTR(pSocket);
  pSocket->m_eFlavor = eFlavor;
}

/*!
 * \brief Check if Socket has an assigned flavor.
 *
 * \param pSocket   Pointer to Socket.
 * \param nMask     Flavor mask of relevant bits.
 * \param nFlavor   Flavor bits.
 *
 * \return Returns true or false.
 */
bool_t SocketAttrHasFlavor(Socket_T *pSocket, int nMask, int nFlavor)
{
  CHKPTR(pSocket, false);

  // flavors only apply to open sockets
  if( pSocket->m_sd == SOCK_SD_CLOSED )
  {
    return false;
  }
  else
  {
    return ((int)(pSocket->m_eFlavor) & nMask) == nFlavor? true: false;
  }
}

/*!
 * \brief Set Socket for non-blockiong I/O.
 *
 * \param pSocket   Pointer to Socket.
 *
 * \return Returns 0 on success, \h_lt 0 on failure.
 */
int SocketAttrSetNonBlocking(Socket_T *pSocket)
{
  CHKPTR(pSocket, -1);

  if( fcntl(pSocket->m_sd, F_SETFL, O_NONBLOCK) < 0 )
  {
    LOGSYSERROR("fcntl(%d,F_SETFL,O_NONBLOCK)", pSocket->m_sd);
    return -1;
  }
  return OK;
}

#if 0 // RDK TODO

/*!
 * \brief Set Socket for non-blockiong I/O.
 *
 * \param pSocket   Pointer to Socket.
 *
 * \return Returns 0 on success, \h_lt 0 on failure.
 */
int SocketAttrSetRcvTimeOut(Socket_T *pSocket, uint_t uSec)
{
  int   rc;

  CHKPTR(pSocket, -1);

  if( (rc = getsockopt(pSocket->m_sd, SO_RCVTIMEO, &tv)) < 0 )
  {
    LOGSYSERROR("getsockopt(%d,SO_RCVTIMEO,...)", pSocket->m_sd);
    return -1;
  }

  LOGDIAG3(x, y);

  return OK;
}
#endif // RDK


// ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~ 
// Socket State Functions
// ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~ 

/*!
 * \brief Check if Socket is open.
 *
 * \param pSocket   Pointer to Socket.
 *
 * \return Returns true or false.
 */
bool_t SocketStateIsOpen(Socket_T *pSocket)
{
  if( pSocket == NULL )
  {
    return false;
  }
  else
  {
    return pSocket->m_sd == SOCK_SD_CLOSED? false: true;
  }
}

/*!
 * \brief Check if Socket is in some kind of errored state.
 *
 * The fcntl() non-blocking function is used to test the socket.
 *
 * \param pSocket   Pointer to Socket.
 *
 * \return Returns true or false.
 */
bool_t SocketStateIsErrored(Socket_T *pSocket)
{
  int fopts = 0;

  if( (pSocket == NULL) || (pSocket->m_sd == SOCK_SD_CLOSED) )
  {
    return false;
  }
  else if( fcntl(pSocket->m_sd, F_GETFL, &fopts) >= 0 )
  {
    return false;
  }
  else
  {
    return true;
  }
}


// ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~ 
// Socket Open/Close Functions
// ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~ 

/*!
 * \brief Create and open a new TCP/IP passive listener socket on this host.
 *
 * \param ipPortListener  Port number to listen on.
 * \param sLocalHostName  String name of local host to bind to. If NULL, then
 *                        this host will automatically choose the IP address.
 *
 * \return:
 *  Returns a new listener Socket_T pointer on success, NULL on failure.
 */
Socket_T *SocketOpenTcpListener(int ipPortListener, const char *sLocalHostName)
{
	int									sd;
	struct sockaddr_in	addrInet;
	int									backlog = 8;
	int									nVal;
  Socket_T           *pSocket;

	LOGDIAG4CALL(_TINT(ipPortListener), _TSTR(sLocalHostName));

	//
	// Create socket in protocal familiy INET (IPv4), of type STREAM (TCP).
	//
	if( (sd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0 )
	{
		LOGSYSERROR("socket(%d,%d,%d)", PF_INET, SOCK_STREAM, IPPROTO_TCP);
		return NULL;
	}

	//
	// Disallow binding if another local task already listening on this address.
  // After socking binding and listen, then this sock option will disallow 
  // binding from any task to this port on any local address. 
	// Note: Non-critacal, so continue on errors.
	//
	nVal = 1;
	if( setsockopt(sd, SOL_SOCKET, SO_REUSEADDR,
				             &nVal, (socklen_t)sizeof(nVal)) < 0 )
	{
		LOGSYSERROR("setsockopt(%d,SOL_SOCKET,SO_REUSEADDR)",  sd);
	}

	//
	// Enable sending of keep-alive messages on connection-oriented sockets.
	// Note: Non-critacal, so continue on errors.
	//
	nVal = 1;
	if( setsockopt(sd, SOL_SOCKET, SO_KEEPALIVE,
				             &nVal, (socklen_t)sizeof(nVal)) < 0 )
	{
		LOGSYSERROR("setsockopt(%d,SOL_SOCKET,SO_KEEPALIVE)",  sd);
	}

	//
	// Bind this socket to application family INET (IP) at the given port.
	//
  addrInet.sin_family = AF_INET;				        // ip application family
	addrInet.sin_port 	= htons_nowarn((uint16_t)ipPortListener);
                                                // listen port number

  //
  // Let bind() automatically fill in the IP address of the machine where
  // the calling process is running on.
  // 
  if( sLocalHostName == NULL )
  {
	  addrInet.sin_addr.s_addr = htonl(INADDR_ANY);
  }

  //
  // Bind to a specific local address
  //
  else if( SockResolveHostName(sLocalHostName, &(addrInet.sin_addr)) < 0 )
  {
    LOGERROR("SockResolveHostName() failed");
    close(sd);
    return NULL;
  }

	//
	// Bind socket to the address.
	//
	if( bind(sd, (struct sockaddr *)&addrInet, (socklen_t)sizeof(addrInet)) < 0 )
	{
		LOGSYSERROR("bind(%d, ...)", sd);
		close(sd);
		return NULL;
	}

	//
	// Mark this bound socket as a listener for new incoming connections that 
	// can then be accepted. The backlog is the maximum queue length of pending
	// connection requests.
	//
	if( listen(sd, backlog) < 0 )
	{
		LOGSYSERROR("listen(%d,%d)", sd, backlog);
		close(sd);
		return NULL;
	}

  //
  // Now create a new socket and set the appropriate attributes
  //
  pSocket = SocketNew();

  SocketAttrSetSd(pSocket, sd);
  SocketAttrSetFlavor(pSocket, 
      SockFlavorDomainIPv4|SockFlavorProtocolTcp|SockFlavorFuncListener);
  SocketAttrSetLocalName(pSocket, NULL);
  SocketAttrSetRemoteName(pSocket, NULL, SockStrAnyHost);

  LOGDIAG4("Opened Socket Listener on %s", SocketAttrGetLocalName(pSocket));

	return pSocket;
}

/*!
 * \brief Create and open a new TCP/IP socket to the given remote host and
 * port.
 *
 * \param sRemoteHostName   Dotted address or network name.
 * \param ipPortRemote      Remote port number.
 *
 * \return
 *  Returns a new TCP Socket_T pointer on success, NULL on failure.
 */
Socket_T *SocketOpenTcpConnection(const char *sRemoteHostName, int ipPortRemote)
{
	int									sd;
	struct sockaddr_in	addrInet;
  Socket_T           *pSocket;

	//
	// Create socket in protocal familiy INET (IPv4), of type STREAM (TCP).
	//
	if( (sd = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0 )
	{
		LOGSYSERROR("socket(%d,%d,%d)", PF_INET, SOCK_STREAM, IPPROTO_TCP);
		return NULL;
	}

  //
  // Set IPv4 address of remote 
  //
  addrInet.sin_family = AF_INET;				            // ip application family
	addrInet.sin_port 	= htons_nowarn((ushort_t)ipPortRemote);
                                                    // remote port number

  if( SockResolveHostName(sRemoteHostName, &(addrInet.sin_addr)) < 0 )
  {
    LOGERROR("SockResolveHostName() failed");
    close(sd);
    return NULL;
  }

  //
  // Connect to remote end
  //
	if(connect(sd, (struct sockaddr *)&addrInet, (socklen_t)sizeof(addrInet)) < 0)
	{
		LOGSYSERROR("connect(%d, ...)", sd);
		close(sd);
		return NULL;
	}

  //
  // Now create a new socket and set the appropriate attributes
  //
  pSocket = SocketNew();

  SocketAttrSetSd(pSocket, sd);
  SocketAttrSetFlavor(pSocket, 
      SockFlavorDomainIPv4|SockFlavorProtocolTcp|SockFlavorFuncConnection);
  SocketAttrSetLocalName(pSocket, NULL);
  SocketAttrSetRemoteName(pSocket, (struct sockaddr *)&addrInet, 
                          sRemoteHostName);

  LOGDIAG4("Opened Connection %s", SocketAttrGetRemoteName(pSocket));

	return pSocket;
}

/*!
 * \brief Create and open a new UDP/IP connectionless socket.
 * port.
 *
 * \param ipPortUdp       Binding UDP port number.
 * \param sLocalHostName  String name of local host to bind to. If NULL, then
 *                        this host will automatically choose the IP address.
 *
 * \return
 *  Returns a new UDP Socket_T pointer on success, NULL on failure.
 */
Socket_T *SocketOpenUdp(int ipPortUdp, const char *sLocalHostName)
{
	int									sd;
	struct sockaddr_in	addrInet;
  Socket_T           *pSocket;

	LOGDIAG4CALL(_TINT(ipPortUdp), _TSTR(sLocalHostName));

	//
	// Create socket in protocal familiy INET (IPv4), of type DateGram (UDP).
	//
	if( (sd = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0 )
	{
		LOGSYSERROR("socket(%d,%d,%d)", PF_INET, SOCK_DGRAM, IPPROTO_UDP);
		return NULL;
	}

	//
	// Bind this socket to application family INET (IP) at the given port.
	//
	memset((void*)&addrInet, 0, sizeof(addrInet));
  addrInet.sin_family = AF_INET;				          // ip application family
	addrInet.sin_port 	= htons_nowarn((ushort_t)ipPortUdp);
                                                  // listen port number

  
  //
  // Let bind() automatically fill in the IP address of the machine where
  // the calling process is running on.
  // 
  if( sLocalHostName == NULL )
  {
	  addrInet.sin_addr.s_addr = htonl(INADDR_ANY);
  }

  //
  // Bind to a specific local address
  //
  else if( SockResolveHostName(sLocalHostName, &(addrInet.sin_addr)) < 0 )
  {
    LOGERROR("SockResolveHostName() failed");
    close(sd);
    return NULL;
  }

	//
	// Bind socket to the address.
	//
	if( bind(sd, (struct sockaddr *)&addrInet, (socklen_t)sizeof(addrInet)) < 0 )
	{
		LOGSYSERROR("bind(%d, ...)", sd);
		close(sd);
		return NULL;
	}

  //
  // Now create a new socket and set the appropriate attributes
  //
  pSocket = SocketNew();

  SocketAttrSetSd(pSocket, sd);
  SocketAttrSetFlavor(pSocket, 
      SockFlavorDomainIPv4|SockFlavorProtocolUdp);
  SocketAttrSetLocalName(pSocket, NULL);
  SocketAttrSetRemoteName(pSocket, NULL, SockStrAnyHost);

  LOGDIAG4("Opened UDP Socket %s", SocketAttrGetLocalName(pSocket));

	return pSocket;
}

/*!
 * \brief Create and open a new UNIX connectionless socket.
 *
 * \todo Create and open a new UNIX connectionless socket.
 *
 * \param ipPortUdp   Binding UDP port number.
 * \param nType       Type of...
 *
 * \return
 *  Returns a new UNIX Socket_T pointer on success, NULL on failure.
 */
Socket_T *SocketOpenUnix(int ipPortUdp, int nType)
{
  return NULL;
}

/*!
 * \brief Create and open a new UNIX pair socket.
 *
 * \todo Create and open a new UNIX pair socket.
 *
 * \param nType       Type of...
 *
 * \return
 *  Returns a new UNIX Socket_T pointer on success, NULL on failure.
 */
Socket_T *SocketOpenUnixPair(int nType)
{
  return NULL;
}

/*!
 * \brief Accept a client connection requst from the listener socket.
 *
 * The new socket will be set to non-blocking I/O mode if requested.
 *
 * \param pSocketListener Pointer to open listener socket.
 * \param  bNonBlock      Do [not] set socket to non-blocking.
 *
 * Return Value:
 *  Newly allocated and accepted open Socket_T pointer on success,
 *  NULL on failure.
 */
Socket_T *SocketAccept(Socket_T *pSocketListener, bool_t bNonBlock)
{
	int									sd;
  struct sockaddr     saddr;
  unsigned int        addrlen = sizeof(saddr);
  Socket_T           *pSocket;

	LOGDIAG4CALL("pSocketListener->%s", pSocketListener->m_sLocalName);

  // accept incoming connection
  if((sd = accept(pSocketListener->m_sd, &saddr, (socklen_t *)&addrlen)) == -1)
  {
		LOGSYSERROR("accept(%d,...)", pSocketListener->m_sd);
    return NULL;
  }

  //
  // Now create a new socket and set the appropriate attributes
  //
  pSocket = SocketNew();

  SocketAttrSetSd(pSocket, sd);
  SocketAttrSetPort(pSocket, ((struct sockaddr_in*)&saddr)->sin_port);
  SocketAttrSetFlavor(pSocket, 
      SockFlavorDomainIPv4|SockFlavorProtocolTcp|SockFlavorFuncConnection);
  SocketAttrSetLocalName(pSocket, NULL);
  SocketAttrSetRemoteName(pSocket, &saddr, NULL); 

  // set non-blocking
  if( bNonBlock )
  {
    if( SocketAttrSetNonBlocking(pSocket) == -1 )
    {
		  LOGERROR("SockAttrSetNonBlocking() failed");
      SocketClose(pSocket);
      SocketDelete(pSocket);
      return NULL;
    }
  }

  LOGDIAG4("Accepted connection from %s", SocketAttrGetRemoteName(pSocket));

	return pSocket;
}

/*!
 * \brief Close a Socket.
 *
 * \param pSocket   Pointer to Socket.
 *
 * \return Returns 0 on success, \h_lt 0 on failure.
 */
int SocketClose(Socket_T *pSocket)
{
  CHKPTR(pSocket, -1);

  if( pSocket->m_sd != SOCK_SD_CLOSED )
  {
    shutdown(pSocket->m_sd, SHUT_RDWR);

    if( close(pSocket->m_sd) < 0 )
    {
      LOGSYSERROR("close(%d)", pSocket->m_sd);
    }

    if( SocketAttrIsConnection(pSocket) )
    {
      LOGDIAG4("Closed Connection %s", SocketAttrGetRemoteName(pSocket));
    }
    else
    {
      LOGDIAG4("Closed Socket %s", SocketAttrGetLocalName(pSocket));
    }

    SocketAttrSetSd(pSocket, SOCK_SD_CLOSED);
    SocketAttrSetFlavor(pSocket, SockFlavorUndef);
    SocketAttrSetLocalName(pSocket, SockStrClosed);
    SocketAttrSetRemoteName(pSocket, NULL, SockStrDisconnected);
  }

  return 0;
}


// ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~ 
// Socket Read/Write Functions
// ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~  ~ 

/*!
 * \brief Reads the next available bytes from the Socket into in the Socket's
 *  read buffer.
 *
 * \note Read are from the connection to the read buffer.
 *
 * \param pSocket Pointer to Socket.
 *
 * \returns
 *  Number of bytes read on success (may be zero if a timeout or an interupt
 *  occurred). Returns \h_lt 0 (RC_SOCK_E*) return code on error.
 */
ssize_t SocketRead(Socket_T *pSocket)
{
  SimpleBuf_T *pBuf;
  size_t       nBytes;
  ssize_t      nRead;

  CHKPTR(pSocket, SOCK_RC_EBADSOCK);
  
  // socket isn't open
  if( !SocketStateIsOpen(pSocket) )
  {
    LOGERROR("Socket %s is not open for reading", pSocket->m_sLocalName);
    return SOCK_RC_EBADSD;
  }

  // socket's simple read buffer
  pBuf = pSocket->m_pIOBuf[SOCK_IO_READ];

  // no read buffer present
  CHKPTR(pBuf, SOCK_RC_ENOBUF);
  CHKPTR(pBuf->m_pRWBuf, SOCK_RC_ENOBUF);

  // bytes remaining available in buffer
  nBytes = SimpleBufHasAvail(pBuf);

  // read buffer is full
  if( nBytes == 0 )
  {
    return SOCK_RC_EBUFFULL;
  }

  // read
  nRead = read(pSocket->m_sd, SimpleBufGetWritePtr(pBuf), nBytes);

  LOGDIAG4("SocketRead: read()=%d", nRead);

  // system error occurred - interpret
  if( nRead < 0 )
  {
    switch(errno)
    {
      case EAGAIN:  // non-blocking, but read would have caused blocking
      case EINTR:   // read was interrupted
        return 0;
      default:
        LOGSYSERROR("read()");
        return SOCK_RC_ESYSERR;
    }
  }

  SimpleBufWriteSeek(pBuf, (size_t)nRead, SIMPLEBUF_REL);

  return nRead;
}

/*!
 * \brief Writes the bytes from the Socket's write buffer to the Socket
 * connection.
 *
 * Only one write attempt is made.
 *
 * \note Writes are from the Socket write buffer.
 *
 * \param pSocket  Pointer to Socket.
 *
 * \returns
//  Number of bytes written on success (may be zero if a timeout or an interupt
//  occurred). Returns \h_lt 0 (RC_SOCK_E*) return code on error.
 */
ssize_t SocketWrite(Socket_T *pSocket)
{
  SimpleBuf_T *pBuf;
  size_t       nBytes;
  ssize_t      nWritten;

  CHKPTR(pSocket, SOCK_RC_EBADSOCK);
  
  // socket isn't open
  if( !SocketStateIsOpen(pSocket) )
  {
    LOGERROR("Socket is not open for writing");
    return SOCK_RC_EBADSD;
  }

  // socket's simple read buffer
  pBuf = pSocket->m_pIOBuf[SOCK_IO_WRITE];

  // no write buffer present
  CHKPTR(pBuf, SOCK_RC_ENOBUF);
  CHKPTR(pBuf->m_pRWBuf, SOCK_RC_ENOBUF);

  // bytes in buffer
  nBytes = SimpleBufHasLen(pBuf);

  // read buffer is empty - finished writing this buffer
  if( nBytes == 0 )
  {
    return SOCK_RC_IODONE;
  }
  
  // write
  nWritten = write(pSocket->m_sd, SimpleBufGetReadPtr(pBuf), nBytes);

  // system error occurred - interpret
  if( nWritten < 0 )
  {
    switch(errno)
    {
      case EAGAIN:  // non-blocking, but write would have caused blocking
      case EINTR:   // write was interrupted
        return 0;
      default:
        LOGSYSERROR("write()");
        return SOCK_RC_ESYSERR;
    }
  }

  SimpleBufReadSeek(pBuf, (size_t)nWritten, SIMPLEBUF_REL);

  return nWritten;
}

/*!
 * \brief Writes all of the bytes from the socket's write buffer until 
 * no more bytes or error.
 *
 * \param pSocket Pointer to Socket.
 *
 * \return
 *  Number of bytes written on success.
 *  Returns \h_lt 0 (RC_SOCK_E*) return code on error.
 */
ssize_t SocketWriteAll(Socket_T *pSocket)
{
  ssize_t n, nWritten=0;

  // write all of the bytes in the buffer
  while( (n = SocketWrite(pSocket)) >= 0 )
  {
    nWritten += n;
  }

  // all bytes have been 
  if( n == SOCK_RC_IODONE )
  {
    return nWritten;
  }

  // error occurred
  else
  {
    return n;
  }
}

#ifdef debugSockUTest

/*!
 * \brief Socket Unit Tester.
 */
void SocketUTest()
{
  Socket_T  *pSocket;

  LOGDIAG4("SocketUTest: SocketOpenTcpListener(9192, ANYHOST)");
  if( (pSocket = SocketOpenTcpListener(9192, NULL)) 
         == NULL ) 
  {
    LOGERROR("SocketOpenTcpListener() failed");
  }
  else
  {
    LOGDIAG4("  LocalName='%s'", SocketAttrGetLocalName(pSocket)); 
    LOGDIAG4("  RemoteName='%s'", SocketAttrGetRemoteName(pSocket)); 
    SocketClose(pSocket);
    SocketDelete(pSocket);
  }

  LOGDIAG4("SocketUTest: SocketOpenTcpListener(9192, nouturn)");
  if( (pSocket = SocketOpenTcpListener(9192, "nouturn")) 
         == NULL ) 
  {
    LOGERROR("SocketOpenTcpListener() failed");
  }
  else
  {
    LOGDIAG4("  LocalName='%s'", SocketAttrGetLocalName(pSocket)); 
    LOGDIAG4("  RemoteName='%s'", SocketAttrGetRemoteName(pSocket)); 
    SocketClose(pSocket);
    SocketDelete(pSocket);
  }

  LOGDIAG4("SocketUTest: SocketOpenTcpConnection(pedxing, 21)");
  if( (pSocket = SocketOpenTcpConnection("pedxing", 21)) == NULL ) 
  {
    LOGERROR("SocketOpenTcpConnection() failed");
  }
  else
  {
    LOGDIAG4("  LocalName='%s'", SocketAttrGetLocalName(pSocket)); 
    LOGDIAG4("  RemoteName='%s'", SocketAttrGetRemoteName(pSocket)); 
    SocketClose(pSocket);
    SocketDelete(pSocket);
  }

  LOGDIAG4("SocketUTest: SocketOpenUdp(9193, ANYHOST)");
  if( (pSocket = SocketOpenUdp(9193, NULL)) == NULL ) 
  {
    LOGERROR("SocketOpenUdp() failed");
  }
  else
  {
    LOGDIAG4("  LocalName='%s'", SocketAttrGetLocalName(pSocket)); 
    LOGDIAG4("  RemoteName='%s'", SocketAttrGetRemoteName(pSocket)); 
    SocketClose(pSocket);
    SocketDelete(pSocket);
  }

  LOGDIAG4("SocketUTest: SocketOpenUdp(9193, 192.168.1.2)");
  if( (pSocket = SocketOpenUdp(9193, "192.168.2.2")) == NULL ) 
  {
    LOGERROR("SocketOpenUdp() failed");
  }
  else
  {
    LOGDIAG4("  LocalName='%s'", SocketAttrGetLocalName(pSocket)); 
    LOGDIAG4("  RemoteName='%s'", SocketAttrGetRemoteName(pSocket)); 
    SocketClose(pSocket);
    SocketDelete(pSocket);
  }
}

#endif // debugSockUTest
