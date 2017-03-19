////////////////////////////////////////////////////////////////////////////////
//
// Package:   BotSense
//
// File:      bsSerial.h
//
/*! \file
 *
 * $LastChangedDate: 2010-08-20 11:36:38 -0600 (Fri, 20 Aug 2010) $
 * $Rev: 568 $
 *
 * \brief \h_botsense bsProxy client library RS-232 serial interface.
 *
 * \todo Add bsSerialReqFlushInput() function.
 * \todo Add bsSerialReqIoctl() function.
 * \todo Add bsSerialReqReadLine() function.
 * \todo Add bsSerialReqWriteLine() function.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 *
 * \copyright
 *   \h_copy 2010-2017. RoadNarrows LLC.\n
 *   http://www.roadnarrows.com\n
 *   All Rights Reserved
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

#ifndef _BSSERIAL_H
#define _BSSERIAL_H

#include "rnr/rnrconfig.h"

#include "botsense/BotSense.h"
#include "botsense/libBotSense.h"

#if defined(__windows__) || defined(__cygwin__)
  #define BS_SER_DEV_NAME_DFT "/dev/com0"         ///< default serial device
#else // linux or osx
  #define BS_SER_DEV_NAME_DFT "/dev/ttyS0"        ///< default serial device
#endif 

#define BS_SER_SERVER_MOD   "libbsserver_serial"  ///< server plugin dll module
#define BS_SER_CLIENT_LIB   "libbsclient_serial"  ///< client app library


extern const char *bsSerialGetMsgName(BsClient_P   pClient,
                                      BsVConnHnd_T hndVConn,
                                      uint_t       uMsgId);

extern int bsSerialReqOpen(BsClient_P  pClient,
                           const char *sDevName,
                           int         nBaudRate,
                           int         nByteSize,
                           int         cParity,
                           int         nStopBits,
                           bool_t      bRtsCts,
                           bool_t      bXonXoff,
                           bool_t      bInitTrace);

extern int bsSerialReqClose(BsClient_P pClient, BsVConnHnd_T hndVConn);

extern int bsSerialReqRead(BsClient_P   pClient,
                           BsVConnHnd_T hndVConn,
                           size_t       uReadLen,
                           byte_t       rbuf[]);

extern int bsSerialReqWrite(BsClient_P    pClient,
                            BsVConnHnd_T  hndVConn,
                            byte_t        wbuf[],
                            size_t        uWriteLen);

extern int bsSerialReqTrans(BsClient_P    pClient,
                            BsVConnHnd_T  hndVConn,
                            byte_t        wbuf[],
                            size_t        uWriteLen,
                            size_t        uReadLen,
                            byte_t        rbuf[]);


#endif // _BSSERIAL_H
