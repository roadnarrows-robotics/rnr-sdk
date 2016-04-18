////////////////////////////////////////////////////////////////////////////////
//
// Package:   RCB3
//
// Library:   libserial
//
// File:      serdev.h
//
/*! \file
 *
 * $LastChangedDate: 2015-01-07 20:22:07 -0700 (Wed, 07 Jan 2015) $
 * $Rev: 3839 $
 *
 * \brief RS-232 serial device communication declarations and defines.
 *
 * Ported from ipser.h (RoadNarrows)
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \par Copyright:
 * (C) 2006-2015  RoadNarrows LLC.
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
////////////////////////////////////////////////////////////////////////////////

#ifndef _SERDEV_H
#define _SERDEV_H

#include "rnr/rnrconfig.h"

C_DECLS_BEGIN

//
// Common character end-of-line terminator sequences.
//
#ifndef EOL_CRLF
#define EOL_CRLF  "\r\n"    // carriage-return,line-feed
#endif
#ifndef EOL_LFCR
#define EOL_LFCR  "\n\r"    // line-feed,carriage-return
#endif
#ifndef EOL_CR
#define EOL_CR    "\r"    // carriage-return
#endif
#ifndef EOL_NL
#define EOL_NL    "\n"    // line-feed
#endif


//-----------------------------------------------------------------------------
// Prototypes
//-----------------------------------------------------------------------------

/*! 
 * \brief Open and configure serial device for communication.
 *
 * \param sSerDevName Serial device port name (e.g. /dev/ttyS0).
 * \param nBaudRate   Baud rate.
 * \param nByteSize   Bytes size in bits 5...8.
 * \param cParity     Parity. One of: 'N', 'E', 'O'
 * \param nStopBits   Number of stop bits 1, 2
 * \param bRtsCts     Do [not] use hardware flow control.
 * \param bXonXoff    Do [not] use software flow control.
 *
 * \return 
 *  On success, a file descriptor(>=0) to the opened serial device.
 * \n On failure, errno is set and RC_ERROR(-1) is returned.
 */
extern int SerDevOpen(const char *sSerDevName,
                      int nBaudRate,
                      int nByteSize,
                      int cParity,
                      int nStopBits,
                      bool_t bRtsCts,
                      bool_t bXonXoff);

/*! 
 * \brief Close serial device port.
 *
 * \param fd          File descriptor to the opened serial device.
 *
 * \return 
 * On success, OK(0) is returned.
 * \n On failure, errno is set and RC_ERROR(-1) is returned.
 */
extern int SerDevClose(int fd);

/*! 
 * \brief Read a ASCII character line from the serial device.
 *
 * Read up to \em count bytes into \em buffer from the serial device input.
 * This call is non-blocking if the timeout value \em usec is greater than
 * zero. Otherwise the read can block indefinitely.
 *
 * The end-of-line sequence defines the end of the line. It is stripped from the
 * return buffer.
 *
 * \param fd            File descriptor to the opened serial device.
 * \param [out] buffer  Output read buffer.
 * \param count         Number of bytes to read.
 * \param eol           End-of-line character sequence \> 0.
 * \param usec          Timeout in microseconds.
 * \n                   If \em usec \> 0, an upper timeout limit is placed on
 *                      the read.
 * \n                   If \em usec == 0, then the read will block indefinitely
 *                      until \em count bytes are read or an I/O error occurred.
 *
 * \return 
 * On success, the number of bytes read into buffer.
 * \n On failure, errno is set and RC_ERROR(-1) is returned.
 * \n In either case, the buffer is guaranteed to be NULL terminated.
 */
extern ssize_t SerDevReadLine(int     fd,
                              char    buffer[],
                              size_t  count,
                              char   *eol,
                              uint_t  usec);

/*! 
 * \brief Write null-terminated ASCII character line to serial device.
 *
 *  Write line plus end-of-line sequence.
 *  This call is non-blocking if the timeout value \em usec is greater
 *  than zero. Otherwise the write can block indefinitely.
 *
 * \param fd            File descriptor to the opened serial device.
 * \param [in] buffer   Buffer to write.
 * \param eol           End-of-line character sequence to append to buffer
 *                      output.
 * \param usec          Timeout in microseconds.
 * \n                   If \em usec \> 0, an upper timeout limit is placed on
 *                      the write.
 * \n                   If \em usec == 0, then the write will block indefinitely
 *                      until \em count bytes are written or an I/O error
 *                      occurred.
 *
 * \return 
 *  On success, the number of bytes written is returned.
 * \n On failure, errno is set and RC_ERROR(-1) is returned.
 */
extern ssize_t SerDevWriteLine(int     fd,
                               char   *buffer,
                               char   *eol,
                               uint_t  usec);

/*! 
 * \brief Get 1 character from the serial device.
 *
 *  Read 1 character from the serial device input.
 *  This call is non-blocking if the timeout value \em usec is greater than
 *  zero. Otherwise the read can block indefinitely.
 *
 * \param fd        File descriptor to the opened serial device.
 * \param usec      Timeout in microseconds.
 * \n               If \em usec \> 0, an upper timeout limit is placed on the
 *                  read.
 * \n               If \em usec == 0, then the read will block indefinitely
 *                  until \em count bytes are read or an I/O error occurred.
 *
 * \return 
 * On success, returns the character read as unsigned char cast to an int.
 * \n On failure, errno is set and RC_ERROR(-1) is returned.
 */
extern int SerDevGetc(int fd, uint_t usec);

/*! 
 * \brief Put one character to serial device.
 *
 *  Write 1 character to the serial device output.
 *  This call is non-blocking if the timeout value \em usec is greater than
 *  zero. Otherwise the write can block indefinitely.
 *
 * \param fd        File descriptor to the opened serial device.
 * \param byte      Character to write.
 * \param usec      Timeout in microseconds.
 * \n               If \em usec \> 0, an upper timeout limit is placed on the
 *                  write.
 * \n               If \em usec == 0, then the write will block indefinitely
 *                  until \em count bytes are written or an I/O error occurred.
 *
 * \return 
 * On success, returns the character written as unsigned char cast to an int.
 * \n On failure, errno is set and RC_ERROR(-1) is returned.
 */
extern int SerDevPutc(int fd, byte_t byte, uint_t usec);

/*! 
 * \brief Read from the serial device.
 *
 *  Read up to \em count bytes into \em buffer from the serial device input.
 *  This call is non-blocking if the timeout value \em usec is greater than
 *  zero. Otherwise the read can block indefinitely.
 *
 *  Note the the bytes read can be less than the \em count.
 *
 * \param fd        File descriptor to the opened serial device.
 * \param buffer    Output read buffer.
 * \param count     Number of bytes to read.
 * \param usec      Timeout in microseconds.
 * \n               If \em usec \> 0, an upper timeout limit is placed on the
 *                  read.
 * \n               If \em usec == 0, then the read will block indefinitely
 *                  until \em count bytes are read or an I/O error occurred.
 *
 * \return 
 * On success, the number of bytes read into buffer.
 * \n On failure, errno is set and RC_ERROR(-1) is returned.
 */
extern ssize_t SerDevRead(int fd,
                          byte_t *buffer,
                          size_t count,
                          uint_t usec);

/*! 
 * \brief Write to serial device.
 *
 *  Write up to \em count bytes from the \em buffer to the serial device
 *  output. This call is non-blocking if the timeout value \em usec is greater
 *  than zero. Otherwise the write can block indefinitely.
 *
 *  Note that the number of bytes written can be less than the \em count.
 *
 * \param fd        File descriptor to the opened serial device.
 * \param buffer    Input write buffer.
 * \param count     Number of of bytes to write.
 * \param usec      Timeout in microseconds.
 * \n               If \em usec \> 0, an upper timeout limit is placed on the
 *                  write.
 * \n               If \em usec == 0, then the write will block indefinitely
 *                  until \em count bytes are written or an I/O error occurred.
 *
 * \return 
 *  On success, the number of bytes written is returned.
 * \n On failure, errno is set and RC_ERROR(-1) is returned.
 */
extern ssize_t SerDevWrite(int fd,
                          byte_t *buffer,
                          size_t count,
                          uint_t usec);

/*! 
 * \brief Determine the number of bytes in the input FIFO of the serial device.
 *
 * The input FIFO holds data received but not read.
 *
 * \param fd        File descriptor to the opened serial device.
 *
 * \return 
 * On success, the number of bytes in the input FIFO.
 * \n On failure, errno is set and RC_ERROR(-1) is returned.
 */
extern ssize_t SerDevFIFOInputCount(int fd);

/*! 
 * \brief Flush the input FIFO buffer, discarding all data in buffer. 
 *
 * The input FIFO holds data received but not read.
 *
 * This function does not block.
 *
 * \param fd      File descriptor to the opened serial device.
 *
 * \return The number of bytes flushed.
 */
extern ssize_t SerDevFIFOInputFlush(int fd);

/*! 
 * \brief Flush output FIFO buffer, discarding all data in buffer. 
 *
 * The output FIFO holds data written to the device but not transmitted.
 *
 * This function does not block.
 *
 * \param fd      File descriptor to the opened serial device.
 */
extern void SerDevFIFOOutputFlush(int fd);

/*! 
 * \brief Transmit (drain) all data written to the output FIFO buffer.
 *
 * The output FIFO holds data written to the device but not transmitted.
 *
 * This function blocks until all of the FIFO data has been transmitted.
 *
 * \param fd      File descriptor to the opened serial device.
 */
extern void SerDevFIFOOutputDrain(int fd);

/*! 
 * \brief Checks to see if there are bytes in the input queue availabe 
 * to be read.
 *
 * \param fd  File descriptor to the opened serial device.
 *
 * \return Returns true if bytes are available, else false.
 */
INLINE_IN_H bool_t SerDevIsInputDataPresent(int fd)
{
  return SerDevFIFOInputCount(fd)>0? true: false;
}

/*! 
 * \brief Set the baudrate.
 *
 * \param fd          File descriptor to the opened serial device.
 * \param nBaudRate   Baud rate.
 *
 * \return 
 *  Returns OK(0) on success.
 * \n On failure, errno is set and RC_ERROR(-1) is returned.
 */
extern int SerDevSetBaudRate(int fd, int nBaudRate);

/*! 
 * \brief Set the byte size.
 *
 * \param fd          File descriptor to the opened serial device.
 * \param nByteSize   Bytes size in bits 5...8.
 *
 * \return 
 *  Returns OK(0) on success.
 * \n On failure, errno is set and RC_ERROR(-1) is returned.
 */
extern int SerDevSetByteSize(int fd, int nByteSize);

/*! 
 * \brief Set the parity.
 *
 * \param fd          File descriptor to the opened serial device.
 * \param cParity     Parity. One of: 'N', 'E', 'O'
 *
 * \return 
 *  Returns OK(0) on success.
 * \n On failure, errno is set and RC_ERROR(-1) is returned.
 */
extern int SerDevSetParity(int fd, int cParity);

/*! 
 * \brief Set the number of stop bits.
 *
 * \param fd          File descriptor to the opened serial device.
 * \param nStopBits   Number of stop bits 1, 2
 *
 * \return 
 *  Returns OK(0) on success.
 * \n On failure, errno is set and RC_ERROR(-1) is returned.
 */
extern int SerDevSetStopBits(int fd, int nStopBitgs);

/*! 
 * \brief Set hardware flow control state.
 *
 * \param fd          File descriptor to the opened serial device.
 * \param bRtsCts     Do [not] use hardware flow control.
 *
 * \return 
 *  Returns OK(0) on success.
 * \n On failure, errno is set and RC_ERROR(-1) is returned.
 */
extern int SerDevSetHwFlowControl(int fd, bool_t bRtsCts);

/*! 
 * \brief Set software flow control state.
 *
 * \param fd          File descriptor to the opened serial device.
 * \param bXonXoff    Do [not] use software flow control.
 *
 * \return 
 *  Returns OK(0) on success.
 * \n On failure, errno is set and RC_ERROR(-1) is returned.
 */
extern int SerDevSetSwFlowControl(int fd, bool_t bXonXoff);

/*! 
 * \brief Assert RTS (request to send).
 *
 * \param fd          File descriptor to the opened serial device.
 *
 * \return 
 *  Returns OK(0) on success.
 * \n On failure, errno is set and RC_ERROR(-1) is returned.
 */
extern int SerDevAssertRTS(int fd);

/*! 
 * \brief De-assert RTS (request to send).
 *
 * \param fd          File descriptor to the opened serial device.
 *
 * \return 
 *  Returns OK(0) on success.
 * \n On failure, errno is set and RC_ERROR(-1) is returned.
 */
extern int SerDevDeassertRTS(int fd);

/*! 
 * \brief Assert RTS (clear to send).
 *
 * \param fd          File descriptor to the opened serial device.
 *
 * \return 
 *  Returns OK(0) on success.
 * \n On failure, errno is set and RC_ERROR(-1) is returned.
 */
extern int SerDevAssertCTS(int fd);

/*! 
 * \brief De-assert RTS (clear to send).
 *
 * \param fd          File descriptor to the opened serial device.
 *
 * \return 
 *  Returns OK(0) on success.
 * \n On failure, errno is set and RC_ERROR(-1) is returned.
 */
extern int SerDevDeassertCTS(int fd);


C_DECLS_END


#endif // _SERDEV_H
