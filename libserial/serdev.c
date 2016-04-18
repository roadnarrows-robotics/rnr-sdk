////////////////////////////////////////////////////////////////////////////////
//
// Package:   RCB3
//
// Library:   libserial
//
// File:      serdev.c
//
/*! \file
 *
 * $LastChangedDate: 2015-02-24 12:58:53 -0700 (Tue, 24 Feb 2015) $
 * $Rev: 3870 $
 * 
 * \brief RS-232 serial device communication definitions.
 *
 * Ported from ipserddev.c (RoadNarrows)
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2006-2015.  RoadNarrows LLC.
 * (http://www.roadnarrows.com)
 * \n All Rights Reserved
 */
/*
 * @EulaBegin@
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
 * @EulaEnd@
 */
//
////////////////////////////////////////////////////////////////////////////////

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <limits.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <time.h>
#include <errno.h>
#include <string.h>

#include "rnr/rnrconfig.h"
#include "rnr/log.h"
#include "rnr/new.h"
#include "rnr/serdev.h"



// ---------------------------------------------------------------------------
// Private Interface 
// ---------------------------------------------------------------------------

/*!
 * \brief Bitwise complement.
 *
 * \param bits  Bits.
 */
#define NOTBITS(bits) ~((uint_t)(bits))

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

// ...........................................................................
// SerDev Specific Logging
// ...........................................................................

#undef SERDEV_DEBUG   // define for more debugging

#ifdef LOG
/*! 
 * \brief Log read/written bytes to log stream.
 * \param buffer    Buffer of bytes.
 * \param size      Number of bytes in buffer to log.
 * \param bNewLine  Do [not] output trailing newline.
 */
#define SERDEVLOG(buffer, size, bNewLine) \
  {if(LOGABLE(LOG_LEVEL_DIAG4)) {SerDevLogBytes(buffer, size, bNewLine);}}

/*! 
 * \brief Write newline character to log stream.
 */
#define SERDEVLOGNL() \
  {if(LOGABLE(LOG_LEVEL_DIAG4)) {fprintf(LOG_GET_LOGFP(), "\n");}}

#else
#define SERDEVLOG(buffer, size, bNewLine) ///< no logging facilities - noop

#define SERDEVLOGNL()                     ///< no logging facilities - noop

#endif // LOG


#ifdef LOG
/*! 
 * \brief Log read/written bytes to log stream.
 *
 * \param buffer    Buffer of bytes.
 * \param size      Number of bytes in buffer to log.
 * \param bNewLine  Do [not] output trailing newline.
 */
static void SerDevLogBytes(byte_t buffer[], size_t size, bool_t bNewLine)
{
  size_t   n;
  FILE    *fp;
  char    *sep = " ";

  if( size == 0 )
  {
    return;
  }

  fp = LOG_GET_LOGFP();

  for(n=0; n<size; ++n)
  {
    fprintf(fp, "%s0x%02x", sep, buffer[n]);
  }

  if( bNewLine )
  {
    fprintf(fp, "\n");
  }
}
#endif // LOG


// ...........................................................................
// Timer Utilities
// ...........................................................................

/*! 
 * \brief Mark the current time. Resolution is microseconds.
 *
 * \param pTvMark   Pointer to timeval structure to be populated with
 *                  the current system seconds and useconds.
 */
static inline void timer_mark(struct timeval  *pTvMark)
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
static uint_t timer_elapsed(struct timeval *pTvMark)
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

// ...........................................................................
// RS-232 Serial Device I/O Controll and Configuration
// ...........................................................................

//
// Ioctl commands and values. If not defined in termios.h, then use
// linux defaults
//
#ifdef FIONREAD
#define SERDEV_FIONREAD   FIONREAD  ///< number in input queue
#else
#define SERDEV_FIONREAD   0x541b    ///< number in input queue
#endif

#define SERDEV_NO_ATTR  0xffffffff  ///< Special "No Attribute" value.

/*!
 * \brief Serial Device Attribute Key - Value Pair structure.
 */
typedef struct
{
  void        *m_pKey; ///< attribute key
  unsigned int m_uVal; ///< attribute value 
} SerDevAttrKvp_T;

/*!
 * \brief Serial device attribute macros.
 * \param attr Attribute define.
 */
#define SERDEV_ATTR_ENTRY(attr) { (void *)#attr , attr }

/*!
 * \brief Get I/O defined attribute value.
 * \param attr Attribute define.
 */
#define SERDEV_GET_ATTR(attr) \
  SerDevAttrLookup(SerDevOptAttrTbl, \
    arraysize(SerDevOptAttrTbl), (void *)#attr, SerDevOptAttrCmp)

/*!
 * \brief Serial option attribute comparator
 * \param pKey1   Attribute key 1
 * \param pKey2   Attribute key 2
 */
static inline int SerDevOptAttrCmp(const void *pKey1, const void *pKey2)
{
  return strcmp((const char *)pKey1, (const char *)pKey2);
}

/*!
 * \brief SerDevOptAttr table.
 *  Dynamic method to determine serial device optional defined attributes
 *  for each plaform architecture.
 */
static SerDevAttrKvp_T SerDevOptAttrTbl[] =
{
#ifdef ECHOCTL
  SERDEV_ATTR_ENTRY(ECHOCTL),
#endif
#ifdef ECHOKE
  SERDEV_ATTR_ENTRY(ECHOKE),
#endif
#ifdef IOCLC
  SERDEV_ATTR_ENTRY(IOCLC),
#endif
#ifdef PARMRK
  SERDEV_ATTR_ENTRY(PARMRK),
#endif
#ifdef IXANY
  SERDEV_ATTR_ENTRY(IXANY),
#endif
#ifdef CRTSCTS
  SERDEV_ATTR_ENTRY(CRTSCTS),
#endif
#ifdef CNEW_RTSCTS   // try it with alternate constant name
  SERDEV_ATTR_ENTRY(CNEW_RTSCTS),
#endif
  {(void *)"*noattr*",  SERDEV_NO_ATTR} // make sure something is in the table
};
                                                                                
/*!
 * \brief Serial device supported baudrate macros.
 * \param baudrate Integer baudrate.
 */
#define SERDEV_BAUDRATE_ENTRY(baudrate) \
  { (void *)((unsigned long)baudrate), B##baudrate }

/*!
 * \brief Get baudrate defined value.
 * \param baudrate Integer baudrate.
 */
#define SERDEV_GET_BAUDRATE(baudrate) \
  SerDevAttrLookup(SerDevBaudRateTbl, arraysize(SerDevBaudRateTbl), \
    (void *)((unsigned long)baudrate), SerDevBaudRateCmp)

/*!
 * \brief BaudRate attribute comparator
 * \param pKey1   Attribute key 1
 * \param pKey2   Attribute key 2
 */
static inline int SerDevBaudRateCmp(const void *pKey1, const void *pKey2)
{
  return (unsigned long)pKey1 == (unsigned long)pKey2? 0: 1;
}

/*!
 * \brief SerDevBaudRate table.
 *  Dynamic method to determine Serial device supported baud rates
 *  for each plaform architecture.
 */
static SerDevAttrKvp_T SerDevBaudRateTbl[] =
{
#ifdef B50
  SERDEV_BAUDRATE_ENTRY(50),
#endif
#ifdef B75
  SERDEV_BAUDRATE_ENTRY(75),
#endif
#ifdef B110
  SERDEV_BAUDRATE_ENTRY(110),
#endif
#ifdef B134
  SERDEV_BAUDRATE_ENTRY(134),
#endif
#ifdef B150
  SERDEV_BAUDRATE_ENTRY(150),
#endif
#ifdef B200
  SERDEV_BAUDRATE_ENTRY(200),
#endif
#ifdef B300
  SERDEV_BAUDRATE_ENTRY(300),
#endif
#ifdef B600
  SERDEV_BAUDRATE_ENTRY(600),
#endif
#ifdef B1200
  SERDEV_BAUDRATE_ENTRY(1200),
#endif
#ifdef B1800
  SERDEV_BAUDRATE_ENTRY(1800),
#endif
#ifdef B2400
  SERDEV_BAUDRATE_ENTRY(2400),
#endif
#ifdef B4800
  SERDEV_BAUDRATE_ENTRY(4800),
#endif
#ifdef B9600
  SERDEV_BAUDRATE_ENTRY(9600),
#endif
#ifdef B19200
  SERDEV_BAUDRATE_ENTRY(19200),
#endif
#ifdef B38400
  SERDEV_BAUDRATE_ENTRY(38400),
#endif
#ifdef B57600
  SERDEV_BAUDRATE_ENTRY(57600),
#endif
#ifdef B115200
  SERDEV_BAUDRATE_ENTRY(115200),
#endif
#ifdef B230400
  SERDEV_BAUDRATE_ENTRY(230400),
#endif
#ifdef B460800
  SERDEV_BAUDRATE_ENTRY(460800),
#endif
#ifdef B500000
  SERDEV_BAUDRATE_ENTRY(500000),
#endif
#ifdef B576000
  SERDEV_BAUDRATE_ENTRY(576000),
#endif
#ifdef B921600
  SERDEV_BAUDRATE_ENTRY(921600),
#endif
#ifdef B1000000
  SERDEV_BAUDRATE_ENTRY(1000000),
#endif
#ifdef B1152000
  SERDEV_BAUDRATE_ENTRY(1152000),
#endif
#ifdef B1500000
  SERDEV_BAUDRATE_ENTRY(1500000),
#endif
#ifdef B2000000
  SERDEV_BAUDRATE_ENTRY(2000000),
#endif
#ifdef B2250000
  SERDEV_BAUDRATE_ENTRY(2250000),
#endif
#ifdef B2500000
  SERDEV_BAUDRATE_ENTRY(2500000),
#endif
#ifdef B3000000
  SERDEV_BAUDRATE_ENTRY(3000000),
#endif
#ifdef B3500000
  SERDEV_BAUDRATE_ENTRY(3500000),
#endif
#ifdef B4000000
  SERDEV_BAUDRATE_ENTRY(4000000),
#endif
};

/*! 
 * \brief Lookup the attribute value associated with the key.
 *
 * \param tbl[]         Attribute Key-Value Pair table.
 * \param nTblEntries   Number of Key-Value Pair table entries.
 * \param pKey          Key to lookup.
 * \param cmp           Attribute key comparator function.
 *
 * \return 
 * The associated attribute value on success, or SERDEV_NO_ATTR if the search
 * failed.
 */
static unsigned int SerDevAttrLookup(SerDevAttrKvp_T tbl[], size_t nTblEntries,
    const void *pKey, int (*cmp)(const void *, const void *))
{
  int i;

  for(i=0; i<nTblEntries; ++i)
  {
    if( !cmp(tbl[i].m_pKey, pKey) )
    {
      return tbl[i].m_uVal;
    }
  }
  return SERDEV_NO_ATTR;
}

/*! 
 * \brief (Re)configure a serial device attibutes.
 *
 * \param fd          File descriptor to the opened serial device.
 * \param nBaudRate   Baud rate.
 * \param nByteSize   Bytes size in bits 5...8.
 * \param cParity     Parity. One of: 'N', 'E', 'O'
 * \param nStopBits   Number of stop bits 1, 2
 * \param bRtsCts     Do [not] use hardware flow control.
 * \param bXonXoff    Do [not] use software flow control.
 *
 * \return 
 *  Returns OK(0) on success.
 * \n On failure, errno is set and RC_ERROR(-1) is returned.
 */
static int SerDevConfigure(int fd,
                          int nBaudRate,
                          int nByteSize,
                          int cParity,
                          int nStopBits,
                          bool_t bRtsCts,
                          bool_t bXonXoff)
{
  struct termios  tios;
  speed_t         ospeed, ispeed;
  uint_t          attr;

  if( fd < 0 )
  {
    LOGERROR("Invalid serial device file descriptor: %d", fd);
    errno = EBADF;
    return RC_ERROR;
  }

  // get the current device attributes
  if( tcgetattr(fd, &tios) == -1 )
  {
    LOGSYSERROR("tcgetattr(%d,...) failed", fd);
    return RC_ERROR;
  }

  // set up raw mode / no echo / binary
  tios.c_cflag |=  (CLOCAL|CREAD);
  tios.c_lflag &= NOTBITS(ICANON|ECHO|ECHOE|ECHOK|ECHONL|ISIG|IEXTEN);
                                    // |ECHOPRT
  if( (attr = SERDEV_GET_ATTR(ECHOCTL)) != SERDEV_NO_ATTR )
  {
    tios.c_lflag &= NOTBITS(attr);
  }
  if( (attr = SERDEV_GET_ATTR(ECHOKE)) != SERDEV_NO_ATTR )
  {
    tios.c_lflag &= NOTBITS(attr);
  }
        
  tios.c_oflag &= NOTBITS(OPOST);

  tios.c_iflag &= NOTBITS(INLCR|IGNCR|ICRNL|IGNBRK);
  if( (attr = SERDEV_GET_ATTR(IOCLC)) != SERDEV_NO_ATTR )
  {
    tios.c_iflag &= NOTBITS(attr);
  }
  if( (attr = SERDEV_GET_ATTR(PARMRK)) != SERDEV_NO_ATTR )
  {
    tios.c_iflag &= NOTBITS(attr);
  }
        
  // set up baudrate
  if((ospeed = SERDEV_GET_BAUDRATE(nBaudRate)) == SERDEV_NO_ATTR)
  {
    LOGERROR("Invalid/unsupported baud rate: %u", nBaudRate);
    errno = EINVAL;
    return RC_ERROR;
  }
  ispeed = ospeed;
  if( cfsetospeed(&tios, ospeed) == -1 )   // set output baudrate in structure
  {
    LOGSYSERROR("tcsetospeed(tios,%u)", ospeed);
    return RC_ERROR;
  }
  if( cfsetispeed(&tios, ispeed) == -1 )   // set input baudrate in structure
  {
    LOGSYSERROR("tcsetispeed(tios,%u)", ispeed);
    return RC_ERROR;
  }

  // set up data character size (bits)
  tios.c_cflag &= NOTBITS(CSIZE);
  switch( nByteSize )
  {
    case 8:
      tios.c_cflag |= CS8;
      break;
    case 7:
      tios.c_cflag |= CS7;
      break;
    case 6:
      tios.c_cflag |= CS6;
      break;
    case 5:
      tios.c_cflag |= CS5;
      break;
    default:
      LOGERROR("Invalid data byte size: %u", nByteSize);
      errno = EINVAL;
      return RC_ERROR;
  }

  // set up stopbits
  switch( nStopBits )
  {
    case 1:
      tios.c_cflag &= NOTBITS(CSTOPB);
      break;
    case 2:
      tios.c_cflag |=  (CSTOPB);
      break;
    default:
      LOGERROR("Invalid number of stop bits: %u", nStopBits);
      errno = EINVAL;
      return RC_ERROR;
  }

  // set up parity
  tios.c_iflag &= NOTBITS(INPCK|ISTRIP);
  switch( cParity )
  {
    case 'N':
      tios.c_cflag &= NOTBITS(PARENB|PARODD);
      break;
    case 'E':
      tios.c_cflag &= NOTBITS(PARODD);
      tios.c_cflag |=  (PARENB);
      break;
    case 'O':
      tios.c_cflag |=  (PARENB|PARODD);
      break;
    default:
      LOGERROR("Invalid parity: %c", cParity);
      errno = EINVAL;
      return RC_ERROR;
  }

  // set up xon/xoff software flow control
  if( (attr = SERDEV_GET_ATTR(IXANY)) != SERDEV_NO_ATTR )
  {
    if( bXonXoff )
    {
      tios.c_iflag |=  (IXON|IXOFF); // |IXANY)
    }
    else
    {
      tios.c_iflag &= NOTBITS(IXON|IXOFF|IXANY);
    }
  }
  else
  {
    if( bXonXoff )
    {
      tios.c_iflag |=  (IXON|IXOFF);
    }
    else
    {
      tios.c_iflag &= NOTBITS(IXON|IXOFF);
    }
  }

  // set up rts/cts hardware flow control
  if( (attr = SERDEV_GET_ATTR(CRTSCTS)) != SERDEV_NO_ATTR )
  {
    if( bRtsCts )
    {
      tios.c_cflag |=  (CRTSCTS);
    }
    else
    {
      tios.c_cflag &= NOTBITS(CRTSCTS);
    }
  }
  // try it with alternate constant name
  else if( (attr = SERDEV_GET_ATTR(CNEW_RTSCTS)) != SERDEV_NO_ATTR )
  {
    if( bRtsCts )
    {
      tios.c_cflag |=  (attr);
    }
    else
    {
      tios.c_cflag &= NOTBITS(attr);
    }
  }
  else
  {
    LOGDIAG4("Do not know how to set/clear RTS/CTS hardware flow control.");
  }
        
  // set up read buffering for non-blocking (vmin, vtime are byte values)
  // select() will handle timeouts.
  tios.c_cc[VMIN]  = 0;   // vmin is minimum number of characters to be read.
  tios.c_cc[VTIME] = 0;   // vtime is read timout in deciseconds 

  // activate the serial device settings
  if( tcsetattr(fd, TCSANOW, &tios) == -1 )
  {
    LOGSYSERROR("tcseattr(%d,TCSANOW,...)", fd);
    return RC_ERROR;
  }

  return OK;
}


// ---------------------------------------------------------------------------
// Public Interface
// ---------------------------------------------------------------------------

int SerDevOpen(const char *sSerDevName,
               int        nBaudRate,
               int        nByteSize,
               int        cParity,
               int        nStopBits,
               bool_t     bRtsCts,
               bool_t     bXonXoff)
{
  int fd;

  LOGDIAG4CALL( _TSTR(sSerDevName),
                _TINT(nBaudRate),
                _TINT(nByteSize),
                _TCHAR(cParity),
                _TINT(nStopBits),
                _TBOOL(bRtsCts),
                _TBOOL(bXonXoff) );

  fd = open(sSerDevName, O_RDWR|O_NONBLOCK|O_NOCTTY);

  if( fd == -1 )
  {
    LOGSYSERROR("open(%s,...)", sSerDevName);
    return RC_ERROR;
  }

  else if( SerDevConfigure(fd, nBaudRate, nByteSize, cParity,
                            nStopBits, bRtsCts, bXonXoff) != OK )
  {
    LOGERROR("SerDevConfigure(): failed");
    close(fd);
    return RC_ERROR;
  }

  else
  {
    LOGDIAG4("Serial device %s opened", sSerDevName);
    return fd;
  }
}

int SerDevClose(int fd)
{
  LOGDIAG4CALL(_TINT(fd));

  if( fd < 0 )
  {
    LOGERROR("Invalid serial device file descriptor: %d", fd);
    errno = EBADF;
    return RC_ERROR;
  }
  else
  {
    return close(fd);
  }
}

ssize_t SerDevReadLine(int     fd,
                       char    buffer[],
                       size_t  count,
                       char   *eol,
                       uint_t  usec)
{
  struct timeval  tstart;         // start of read line
  uint_t          tdelta;         // read delta time
  int             c;              // received character
  ssize_t         len = 0;        // length of line sans eol sequence
  bool_t          bInEol = false; // [not] in end-of-line state
  char           *sSave = eol;    // save eol pointer

  buffer[0] = 0;

  // must have an end-of-line termination sequence
  if( (eol == NULL) || (*eol == 0) )
  {
    LOGERROR("No end-of-line sequence specified.");
    return RC_ERROR;
  }

  // start 
  timer_mark(&tstart);

  //
  // Read until the end of the end-of-line sequence or an error has occurred.
  //
  while( len < count-1 )
  {
    // read character
    c = SerDevGetc(fd, usec);

    // read error occurred
    if( c < 0 )
    {
      buffer[len] = 0;
      return RC_ERROR;
    }

    // end-of-line sequence
    else if( c == *eol )
    {
      // start of eol
      if( !bInEol )
      {
        buffer[len] = 0;
        bInEol = true;
      }

      ++eol;

      // got a full line
      if( *eol == 0 )
      {
        return len;
      }
    }

    // unexpected character in end-of-line sequence
    else if( bInEol )
    {
      LOGWARN("Unexpected character '%c' found in end-of-line sequence \"%s\".",
          c, sSave);
      return len;
    }

    // normal character
    else
    {
      buffer[len++] = (char)c;
    }

    // delta time
    tdelta = timer_elapsed(&tstart);

    // timed out
    if( (usec > 0) && (tdelta >= usec) )
    {
      LOGDIAG4("%s() timed out", LOGFUNCNAME);
      buffer[len] = 0;
      return RC_ERROR;
    }
  }

  buffer[len] = 0;

  LOGERROR("Line > %zu characters long.", count);

  return RC_ERROR;
}

ssize_t SerDevWriteLine(int     fd,
                        char   *buffer,
                        char   *eol,
                        uint_t  usec)
{
  size_t  lenBuf, lenEol;
  ssize_t m, n;

  // must have an end-of-line termination sequence
  if( (eol == NULL) || (*eol == 0) )
  {
    LOGERROR("No end-of-line sequence specified.");
    return RC_ERROR;
  }

  lenBuf = strlen(buffer);
  lenEol = strlen(eol);

  m = SerDevWrite(fd, (byte_t *)buffer, lenBuf, usec);
  n = SerDevWrite(fd, (byte_t *)eol, lenEol, usec);

  if( (m == lenBuf) && (n == lenEol) )
  {
    return m;
  }
  else
  {
    return RC_ERROR;
  }
}

int SerDevGetc(int fd, uint_t usec)
{
  byte_t          byte;
  fd_set          rset;
  struct timeval  timeout;
  int             nFd;
  ssize_t         n;

  LOGDIAG4CALL(_TINT(fd), _TUINT(usec));

  FD_ZERO(&rset);
  fdset_nowarn(fd, &rset);

  // wait for character with timeout
  if( usec > 0 )
  {
    // load timeout
    timeout.tv_sec  = (time_t)(usec / 1000000);
    timeout.tv_usec = (time_t)(usec % 1000000);

    nFd = select(fd+1, &rset, NULL, NULL, &timeout);
  }

  // block indefinitely for character
  else 
  {
    nFd = select(fd+1, &rset, NULL, NULL, NULL);
  }

  // system error occurred
  if( nFd < 0 )
  {
    LOGSYSERROR("select(%d,...)", fd);
    return RC_ERROR;
  }

  // select() timeout occurred
  else if( nFd == 0 )
  {
    LOGDIAG4("select() on read timed out");
    errno = ETIMEDOUT;
    return RC_ERROR;
  }

  // data available from serial device, but error on read
  else if( (n = read(fd, &byte, (size_t)1)) < 0 )
  {
    LOGSYSERROR("read(%d,...)", fd);
    return RC_ERROR;
  }

  // nothing read
  else if( n == 0 )
  {
    LOGERROR("0=read(%d,...)", fd);
    errno = EIO;
    return RC_ERROR;
  }

  // read character
  else
  {
    LOGDIAG4("%s() byte=0x%02x read", LOGFUNCNAME, byte);
    return (int)(((uint_t)(byte)) & 0x00ff);
  }
}

int SerDevPutc(int fd, byte_t byte, uint_t usec)
{
  fd_set          wset;
  struct timeval  timeout;
  int             nFd;
  ssize_t         n;

  LOGDIAG4CALL(_TINT(fd), _THEX(byte), _TUINT(usec));

  FD_ZERO(&wset);
  fdset_nowarn(fd, &wset);

  // wait for output with timeout
  if( usec > 0 )
  {
    // load timeout (gets munged after each select())
    timeout.tv_sec  = (time_t)(usec / 1000000);
    timeout.tv_usec = (time_t)(usec % 1000000);

    nFd = select(fd+1, NULL, &wset, NULL, &timeout);
  }

  // block indefinitely for output
  else 
  {
    nFd = select(fd+1, NULL, &wset, NULL, NULL);
  }

  // system error occurred
  if( nFd < 0 )
  {
    LOGSYSERROR("select(%d,...)", fd);
    return RC_ERROR;
  }

  // select() timeout occurred
  else if( nFd == 0 )
  {
    LOGDIAG4("select() on write timed out");
    errno = ETIMEDOUT;
    return RC_ERROR;
  }

  // data available from serial device (but error on write)
  else if( (n = write(fd, &byte, (size_t)1)) < 0 )
  {
    LOGSYSERROR("write(%d,...)", fd);
    return RC_ERROR;
  }

  // nothing written
  else if( n == 0 )
  {
    LOGERROR("0=write(%d,...)", fd);
    errno = EIO;
    return RC_ERROR;
  }

  // character written
  else
  {
    LOGDIAG4("%s() byte=0x%02x written", LOGFUNCNAME, byte);
    return (int)(((uint_t)(byte)) & 0x00ff);
  }
}

ssize_t SerDevRead(int fd, byte_t *buffer, size_t count, uint_t usec)
{
  bool_t          bNonBlocking;
  ssize_t         nBytes = 0;
  struct timeval  tstart;
  uint_t          tdelta;
  fd_set          rset;
  struct timeval  timeout;
  int             nFd;
  ssize_t         n;

  LOGDIAG4CALL(_TINT(fd), _TPTR(buffer), _TUINT(count), _TUINT(usec));

  bNonBlocking = usec > 0? true: false;

  while( nBytes < count )
  {
    FD_ZERO(&rset);
    fdset_nowarn(fd, &rset);

    // wait for input with timeout
    if( bNonBlocking )
    {
      timer_mark(&tstart);

      // (re)load timeout (gets munged after each select())
      timeout.tv_sec  = (time_t)(usec / 1000000);
      timeout.tv_usec = (time_t)(usec % 1000000);

      nFd = select(fd+1, &rset, NULL, NULL, &timeout);
    }
    // block indefinitely for input
    else 
    {
      nFd = select(fd+1, &rset, NULL, NULL, NULL);
    }

    // system error occurred
    if( nFd < 0 )
    {
      LOGSYSERROR("select(%d,...)", fd);
      return (ssize_t)(RC_ERROR);
    }

    // select() timeout occurred
    else if( nFd == 0 )
    {
      LOGDIAG4("select() on read timed out");
    }

    // data available from serial device (but error on read)
    else if( (n = read(fd, buffer+nBytes, count-(size_t)nBytes)) < 0 )
    {
      LOGSYSERROR("read(%d,...)", fd);
      return (ssize_t)(RC_ERROR);
    }

    // read some data
    else
    {
      nBytes += n;
    }

    // determine time left for non-blocking read timeout
    if( (nBytes < count) && bNonBlocking )
    {
      tdelta = timer_elapsed(&tstart);
      if( tdelta >= usec )
      {
        LOGDIAG4("%s() timed out", LOGFUNCNAME);
        break;
      }
      else
      {
        usec -= tdelta;
      }
    }
  }

  LOGDIAG4("%s() %lu bytes read", LOGFUNCNAME, nBytes);
  SERDEVLOG(buffer, (size_t)nBytes, true);

  return nBytes;
}

ssize_t SerDevWrite(int fd, byte_t *buffer, size_t count, uint_t usec)
{
  bool_t          bNonBlocking;
  ssize_t         nBytes = 0;
  struct timeval  tstart;
  uint_t          tdelta;
  fd_set          wset;
  struct timeval  timeout;
  int             nFd;
  ssize_t         n;

  LOGDIAG4CALL(_TINT(fd), _TPTR(buffer), _TUINT(count), _TUINT(usec));

  bNonBlocking = usec > 0? true: false;

  while( nBytes < count )
  {
    FD_ZERO(&wset);
    fdset_nowarn(fd, &wset);

    // wait for output with timeout
    if( bNonBlocking )
    {
      timer_mark(&tstart);

      // (re)load timeout (gets munged after each select())
      timeout.tv_sec  = (time_t)(usec / 1000000);
      timeout.tv_usec = (time_t)(usec % 1000000);

      nFd = select(fd+1, NULL, &wset, NULL, &timeout);
    }
    // block indefinitely for output
    else 
    {
      nFd = select(fd+1, NULL, &wset, NULL, NULL);
    }

    // system error occurred
    if( nFd < 0 )
    {
      LOGSYSERROR("select(%d,...)", fd);
      return (ssize_t)(RC_ERROR);
    }

    // select() timeout occurred
    else if( nFd == 0 )
    {
      LOGDIAG4("select() on write timed out");
    }

    // data available from serial device (but error on read)
    else if( (n = write(fd, buffer+nBytes, count-(size_t)nBytes)) < 0 )
    {
      LOGSYSERROR("write(%d,...)", fd);
      return (ssize_t)(RC_ERROR);
    }

    // written some data
    else
    {
      nBytes += n;
    }

    // determine time left for non-blocking read timeout
    if( (nBytes < count) && bNonBlocking )
    {
      tdelta = timer_elapsed(&tstart);
      if( tdelta >= usec )
      {
        LOGDIAG4("%s() timed out", LOGFUNCNAME);
        break;
      }
      else
      {
        usec -= tdelta;
      }
    }
  }

  LOGDIAG4("%s() %lu bytes written", LOGFUNCNAME, nBytes);
  SERDEVLOG(buffer, (size_t)nBytes, true);

  return nBytes;
}

ssize_t SerDevFIFOInputCount(int fd)
{
  unsigned int  nBytes = 0;
  int           rc;

  rc = ioctl(fd, (unsigned long)SERDEV_FIONREAD, &nBytes);
  if( rc == -1 )
  {
    // not a tty, something is hosed 
    if( errno != ENOTTY )
    {
      LOGSYSERROR("ioctl(%d,0x%x,...)", fd, SERDEV_FIONREAD);
      return (ssize_t)(RC_ERROR);
    }
    else
    {
      return (ssize_t)0;
    }
  }
  else
  {
    return (ssize_t)nBytes;
  }
}

#ifdef SERDEV_DEBUG
ssize_t SerDevFIFOInputFlush(int fd)
{
  byte_t  b[256];
  ssize_t n;
  ssize_t nBytes = 0;

  LOGDIAG4CALL(_TINT(fd));

  //
  // Flush existing input characters by reading and logging the data in the
  // FIFO.
  //
  while( (n = SerDevFIFOInputCount(fd)) > 0 )
  {
    n = n < arraysize(b)? n: arraysize(b);
    n = SerDevRead(fd, b, (size_t)n, 500000); 
    if( n > 0 )
    {
      SERDEVLOG(b, (size_t)n, false);
      nBytes += n;
    }
  }
  if( nBytes > 0 )
  {
    SERDEVLOGNL();
  }

  return nBytes;
}
#else
ssize_t SerDevFIFOInputFlush(int fd)
{
  ssize_t n;

  LOGDIAG4CALL(_TINT(fd));

  n = SerDevFIFOInputCount(fd);

  tcflush(fd, TCIFLUSH);

  return n;
}
#endif // SERDEV_DEBUG

void SerDevFIFOOutputFlush(int fd)
{
  LOGDIAG4CALL(_TINT(fd));

  tcflush(fd, TCOFLUSH);
}

void SerDevFIFOOutputDrain(int fd)
{
  LOGDIAG4CALL(_TINT(fd));

  tcdrain(fd);
}

int SerDevSetBaudRate(int fd, int nBaudRate)
{
  struct termios  tios;
  speed_t         ospeed, ispeed;

  LOGDIAG4CALL(_TINT(fd), _TINT(nBaudRate));

  // get the current device attributes
  if( tcgetattr(fd, &tios) == -1 )
  {
    LOGSYSERROR("tcgetattr(%d,...) failed", fd);
    return RC_ERROR;
  }

  // set up baudrate
  if((ospeed = SERDEV_GET_BAUDRATE(nBaudRate)) == SERDEV_NO_ATTR)
  {
    LOGERROR("Invalid/unsupported baud rate: %u", nBaudRate);
    errno = EINVAL;
    return RC_ERROR;
  }
  ispeed = ospeed;
  if( cfsetospeed(&tios, ospeed) == -1 )   // set output baudrate in structure
  {
    LOGSYSERROR("tcsetospeed(tios,%u)", ospeed);
    return RC_ERROR;
  }
  if( cfsetispeed(&tios, ispeed) == -1 )   // set input baudrate in structure
  {
    LOGSYSERROR("tcsetispeed(tios,%u)", ispeed);
    return RC_ERROR;
  }

  return OK;
}

int SerDevSetByteSize(int fd, int nByteSize)
{
  struct termios  tios;

  LOGDIAG4CALL(_TINT(fd), _TINT(nByteSize));

  // set up data character size (bits)
  tios.c_cflag &= NOTBITS(CSIZE);
  switch( nByteSize )
  {
    case 8:
      tios.c_cflag |= CS8;
      break;
    case 7:
      tios.c_cflag |= CS7;
      break;
    case 6:
      tios.c_cflag |= CS6;
      break;
    case 5:
      tios.c_cflag |= CS5;
      break;
    default:
      LOGERROR("Invalid data byte size: %u", nByteSize);
      errno = EINVAL;
      return RC_ERROR;
  }

  return OK;
}

int SerDevSetParity(int fd, int cParity)
{
  struct termios  tios;

  LOGDIAG4CALL(_TINT(fd), _TCHAR(cParity));

  // set up parity
  tios.c_iflag &= NOTBITS(INPCK|ISTRIP);
  switch( cParity )
  {
    case 'N':
      tios.c_cflag &= NOTBITS(PARENB|PARODD);
      break;
    case 'E':
      tios.c_cflag &= NOTBITS(PARODD);
      tios.c_cflag |=  (PARENB);
      break;
    case 'O':
      tios.c_cflag |=  (PARENB|PARODD);
      break;
    default:
      LOGERROR("Invalid parity: %c", cParity);
      errno = EINVAL;
      return RC_ERROR;
  }

  return OK;
}

int SerDevSetStopBits(int fd, int nStopBits)
{
  struct termios  tios;

  LOGDIAG4CALL(_TINT(fd), _TINT(nStopBits));

  // set up stopbits
  switch( nStopBits )
  {
    case 1:
      tios.c_cflag &= NOTBITS(CSTOPB);
      break;
    case 2:
      tios.c_cflag |=  (CSTOPB);
      break;
    default:
      LOGERROR("Invalid number of stop bits: %u", nStopBits);
      errno = EINVAL;
      return RC_ERROR;
  }

  return OK;
}

int SerDevSetHwFlowControl(int fd, bool_t bRtsCts)
{
  struct termios  tios;
  unsigned int    attr;

  LOGDIAG4CALL(_TINT(fd), _TBOOL(bRtsCts));

  // get the current device attributes
  if( tcgetattr(fd, &tios) == -1 )
  {
    LOGSYSERROR("tcgetattr(%d,...) failed", fd);
    return RC_ERROR;
  }

  // set up rts/cts hardware flow control
  if( (attr = SERDEV_GET_ATTR(CRTSCTS)) != SERDEV_NO_ATTR )
  {
    if( bRtsCts )
    {
      tios.c_cflag |=  (CRTSCTS);
    }
    else
    {
      tios.c_cflag &= NOTBITS(CRTSCTS);
    }
  }
  // try it with alternate constant name
  else if( (attr = SERDEV_GET_ATTR(CNEW_RTSCTS)) != SERDEV_NO_ATTR )
  {
    if( bRtsCts )
    {
      tios.c_cflag |=  (attr);
    }
    else
    {
      tios.c_cflag &= NOTBITS(attr);
    }
  }
  else
  {
    LOGDIAG4("Do not know how to set/clear RTS/CTS hardware flow control.");
    return RC_ERROR;
  }

  // flush data
  if( tcflush(fd, TCIOFLUSH) == -1 )
  {
    LOGSYSERROR("tcflush(%d,TCIOFLUSH)", fd);
  }

  // set hw flow control
  if( tcsetattr(fd, TCSANOW, &tios) == -1 )
  {
    LOGSYSERROR("tcseattr(%d,TCSANOW,...)", fd);
    return RC_ERROR;
  }

  return OK;
}

int SerDevSetSwFlowControl(int fd, bool_t bXonXoff)
{
  struct termios  tios;
  unsigned int    attr;

  LOGDIAG4CALL(_TINT(fd), _TBOOL(bXonXoff));

  // get the current device attributes
  if( tcgetattr(fd, &tios) == -1 )
  {
    LOGSYSERROR("tcgetattr(%d,...) failed", fd);
    return RC_ERROR;
  }

  // set up xon/xoff software flow control
  if( (attr = SERDEV_GET_ATTR(IXANY)) != SERDEV_NO_ATTR )
  {
    if( bXonXoff )
    {
      tios.c_iflag |=  (IXON|IXOFF); // |IXANY)
    }
    else
    {
      tios.c_iflag &= NOTBITS(IXON|IXOFF|IXANY);
    }
  }
  else
  {
    if( bXonXoff )
    {
      tios.c_iflag |=  (IXON|IXOFF);
    }
    else
    {
      tios.c_iflag &= NOTBITS(IXON|IXOFF);
    }
  }

  // set sw flow control
  if( tcsetattr(fd, TCSANOW, &tios) == -1 )
  {
    LOGSYSERROR("tcseattr(%d,TCSANOW,...)", fd);
    return RC_ERROR;
  }

  return OK;
}

int SerDevAssertRTS(int fd)
{
  int status;

  if( ioctl(fd, TIOCMGET, &status) == -1 )
  {
    LOGSYSERROR("ioctl(%d,TIOCMGET,...)", fd);
    return RC_ERROR;
  }

  status |= TIOCM_RTS;

  if( ioctl(fd, TIOCMSET, &status) == -1 )
  {
    LOGSYSERROR("ioctl(%d,TIOCMSET,0x%x)", fd, status);
    return RC_ERROR;
  }

  return OK;
}

int SerDevDeassertRTS(int fd)
{
  int status;

  if( ioctl(fd, TIOCMGET, &status) == -1 )
  {
    LOGSYSERROR("ioctl(%d,TIOCMGET,...)", fd);
    return RC_ERROR;
  }

  status &= ~TIOCM_RTS;

  if( ioctl(fd, TIOCMSET, &status) == -1 )
  {
    LOGSYSERROR("ioctl(%d,TIOCMSET,0x%x)", fd, status);
    return RC_ERROR;
  }

  return OK;
}

int SerDevAssertCTS(int fd)
{
  int status;

  if( ioctl(fd, TIOCMGET, &status) == -1 )
  {
    LOGSYSERROR("ioctl(%d,TIOCMGET,...)", fd);
    return RC_ERROR;
  }

  status |= TIOCM_CTS;

  if( ioctl(fd, TIOCMSET, &status) == -1 )
  {
    LOGSYSERROR("ioctl(%d,TIOCMSET,0x%x)", fd, status);
    return RC_ERROR;
  }

  return OK;
}

int SerDevDeassertCTS(int fd)
{
  int status;

  if( ioctl(fd, TIOCMGET, &status) == -1 )
  {
    LOGSYSERROR("ioctl(%d,TIOCMGET,...)", fd);
    return RC_ERROR;
  }

  status &= ~TIOCM_CTS;

  if( ioctl(fd, TIOCMSET, &status) == -1 )
  {
    LOGSYSERROR("ioctl(%d,TIOCMSET,0x%x)", fd, status);
    return RC_ERROR;
  }

  return OK;
}


#if 0
int SerDevAssertRTS(int fd)
{
  int   modem = TIOCM_RTS;    // modem bits to set

  // set the indicated modem bits
  if( ioctl(fd, TIOCMBIS, &modem) == -1 )
  {
    LOGSYSERROR("ioctl(%d,TIOCMBIS,0x%x)", modem);
    return RC_ERROR;
  }

  return OK;
}

int SerDevDeassertRTS(int fd)
{
  int   modem = TIOCM_RTS;    // modem bits to clear

  // clear the indicated modem bits
  if( ioctl(fd, TIOCMBIC, &modem) == -1 )
  {
    LOGSYSERROR("ioctl(%d,TIOCMBIC,0x%x)", modem);
    return RC_ERROR;
  }

  return OK;
}

int SerDevAssertCTS(int fd)
{
  int   modem = TIOCM_CTS;    // modem bits to set

  // set the indicated modem bits
  if( ioctl(fd, TIOCMBIS, &modem) == -1 )
  {
    LOGSYSERROR("ioctl(%d,TIOCMBIS,0x%x)", modem);
    return RC_ERROR;
  }

  return OK;
}

int SerDevDeassertCTS(int fd)
{
  int   modem = TIOCM_CTS;    // modem bits to clear

  // clear the indicated modem bits
  if( ioctl(fd, TIOCMBIC, &modem) == -1 )
  {
    LOGSYSERROR("ioctl(%d,TIOCMBIC,0x%x)", modem);
    return RC_ERROR;
  }

  return OK;
}
#endif
