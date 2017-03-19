////////////////////////////////////////////////////////////////////////////////
//
// Package:   BotSense
//
// File:      bsI2C.h
//
/*! \file
 *
 * $LastChangedDate: 2010-08-20 11:36:38 -0600 (Fri, 20 Aug 2010) $
 * $Rev: 568 $
 *
 * \brief \h_botsense bsProxy client library \h_i2c bus interface.
 *
 * \todo Add bsI2CReqCheck() function.
 * \todo Add bsI2CReqReadLine() function.
 * \todo Add bsI2CReqWriteLine() function.
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

#ifndef _BSI2C_H
#define _BSI2C_H

#include "rnr/rnrconfig.h"
#include "rnr/i2c.h"

#include "botsense/BotSense.h"
#include "botsense/libBotSense.h"

#define BS_I2C_DEV_NAME_DFT "/dev/i2c-0"        ///< default \h_i2c bus device

#define BS_I2C_SERVER_MOD   "libbsserver_i2c"   ///< server plugin dll module
#define BS_I2C_CLIENT_LIB   "libbsclient_i2c"   ///< client app library


extern const char *bsI2CGetMsgName(BsClient_P   pClient,
                                   BsVConnHnd_T hndVConn,
                                   uint_t       uMsgId);

extern int bsI2CReqOpen(BsClient_P  pClient,
                        const char *sDevName,
                        bool_t      bInitTrace);

extern int bsI2CReqClose(BsClient_P pClient, BsVConnHnd_T hndVConn);

extern int bsI2CReqRead(BsClient_P   pClient,
                        BsVConnHnd_T hndVConn,
                        i2c_addr_t   i2cAddr,
                        size_t       uReadLen,
                        byte_t       rbuf[]);

extern int bsI2CReqWrite(BsClient_P   pClient,
                         BsVConnHnd_T hndVConn,
                         i2c_addr_t   i2cAddr,
                         byte_t       wbuf[],
                         size_t       uWriteLen);

extern int bsI2CReqTrans(BsClient_P   pClient,
                         BsVConnHnd_T hndVConn,
                         i2c_addr_t   i2cAddr,
                         byte_t       wbuf[],
                         size_t       uWriteLen,
                         size_t       uReadLen,
                         byte_t       rbuf[]);

extern int bsI2CReqScan(BsClient_P   pClient,
                        BsVConnHnd_T hndVConn,
                        i2c_addr_t   bufScan[],
                        size_t       sizeScanBuf);


#endif // _BSI2C_H
