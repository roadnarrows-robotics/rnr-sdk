/******************************************************************************
 *
 * Package:   BotSense
 *
 * File:      bsSerial.i
 *
 * $LastChangedDate$
 * $Rev$
 */

/*!
 * \file
 *
 * \brief BotSense serial library python swig interface definitions file.
 *
 * \author Robin Knight (robin.knight@roadnarrows.com)
 * 
 * \par Copyright:
 *   (C) 2010.  RoadNarrows LLC.
 *   (http://www.roadnarrows.com)
 *   All Rights Reserved
 */

/*
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
 *
 * IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
 * OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
 * PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
 * DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
 * EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 * THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
 * "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
 *
 ******************************************************************************/

%module bsSerial
%{
#include "botsense/BotSense.h"
#include "botsense/bsSerial.h"
%}

%begin
%{
/*! \file
 *  \brief Swig generated serial wrapper c file.
 */
%}

/* 
 * Required RNR C Types
 */
typedef unsigned char byte_t;
typedef unsigned short ushort_t;
typedef unsigned int uint_t;
typedef unsigned long ulong_t;
typedef int bool_t;

/*
 * Required BotSense C Types
 */
typedef struct _bsClientStruct *BsClient_P;
typedef int BsVConnHnd_T;

/* the swigged interface */
%include "botsense/bsSerial.h"

/*
 * Higher-level python interface to the BotSense serial C library.
 */
%pythoncode
%{

"""
BotSense Serial Python Inline Extensions and Wrappers.
"""

## \file 
## \package BotSense.bsSerial
##
## \brief BotSense Swigged Serial Python Interface Module.
##
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \par Copyright:
##   (C) 2010.  RoadNarrows LLC.\n
##   (http://www.roadnarrows.com)\n
##   All Rights Reserved
##

import BotSenseCore  as bsCore
import BotSenseError as bsError
import BotSenseTypes as bsTypes

def SerialGetMsgName(client, hndVConn, msgId):
  """ Get the serial message name.

      For each (virtual connection, message id) 2-tuple, there can be a known
      name string (provided the id is valid and an application provides the
      information).

      Parameters:
        client    - BotSenseTypes.BsClient instance.
        hndVConn  - Virtual connection handle (ignored).
        msgId     - Serial message id.

      Return:
        Returns message name string if it can be determined.
        Otherwise returns 'unknown'.
  """
  bsTypes.BsClient.ChkClient(client)
  return bsSerialGetMsgName(client.pClient, hndVConn, msgId)

def SerialReqOpen(client, devName, baudRate=9600, byteSize=8, parity='N',
                      stopBits=1, rtscts=False, xonxoff=False, trace=False):
  """ Proxy server request to establish a virtual connection to an RS-232
      serial device.

      Raises a BotSenseError exception on failure.

      Parameters:
        client    - BotSenseTypes.BsClient instance.
        devName   - Proxied serial device name (e.g. /dev/ttyS0).
        baudRate  -  Baud rate.
        byteSize  - Bytes size in bits 5...8.
        parity    - Parity. One of: 'N', 'E', 'O'
        stopBits  - Number of stop bits 1, 2
        rtscts    - Do [not] use hardware flow control.
        xonxoff   - Do [not] use software flow control.
        trace     - Initial message tracing enable(true)/disable(false) state.

      Return
        New virtual connection handle.
  """
  bsTypes.BsClient.ChkClient(client)
  hndVConn = bsSerialReqOpen(client.pClient, devName, baudRate, byteSize,
                        ord(parity), stopBits, rtscts, xonxoff, trace)
  bsError.ChkReturnIsNonNeg(hndVConn,
      "Client %s: Serial Open request failed." % (client.AttrGetName()))
  return hndVConn

def SerialReqClose(client, hndVConn):
  """ Proxy server request to close client's proxied serial device virtual
      connection.

      Raises a BotSenseError exception on failure.

      Parameters:
        client    - BotSenseTypes.BsClient instance.
        hndVConn  - Virtual connection handle.
  """
  bsTypes.BsClient.ChkClient(client)
  rc = bsSerialReqClose(client.pClient, hndVConn)
  bsError.ChkReturnIsOk(rc,
      "Client %s: Serial Close request failed." % (client.AttrGetName()))

def SerialReqRead(client, hndVConn, readLen):
  """ Serial request to read from a proxied RS-232 serial device.

      Raises a BotSenseError exception on failure.

      Parameters:
        client    - BotSenseTypes.BsClient instance.
        hndVConn  - Virtual connection handle.
        readLen   - Number of bytes to read.

      Return:
        Buffer list of read bytes.
  """
  bsTypes.BsClient.ChkClient(client)
  rbytebuf = bsTypes.ByteBuf(readLen)
  n = bsSerialReqRead(client.pClient, hndVConn, readLen, rbytebuf.getSwigObj())
  bsError.ChkReturnIsNonNeg(n,
      "Client %s: Serial Read request failed." % (client.AttrGetName()))
  rbytebuf.copyFromSwigObj(n)
  return rbytebuf.buf

def SerialReqWrite(client, hndVConn, wbuf):
  """ Serial request to write to a proxied RS-232 serial device.

      Raises a BotSenseError exception on failure.

      Parameters:
        client    - BotSenseTypes.BsClient instance.
        hndVConn  - Virtual connection handle.
        wbuf      - Write buffer.

      Return:
        Number of bytes written.
  """
  bsTypes.BsClient.ChkClient(client)
  wbytebuf = bsTypes.ByteBuf.Clone(wbuf)
  wbytebuf.copyToSwigObj(len(wbytebuf))
  n = bsSerialReqWrite(client.pClient, hndVConn, wbytebuf.getSwigObj(),
                        len(wbytebuf))
  bsError.ChkReturnIsNonNeg(n,
      "Client %s: Serial Write request failed." % (client.AttrGetName()))
  return n

def SerialReqTrans(client, hndVConn, wbuf, readLen):
  """ Serial request to execute a write-read transaction to a proxied
      RS-232 serial device.

      Raises a BotSenseError exception on failure.

      Parameters:
        client    - BotSenseTypes.BsClient instance.
        hndVConn  - Virtual connection handle.
        wbuf      - Write buffer.
        readLen   - Number of bytes to read.

      Return:
        Buffer list of read bytes.
  """
  bsTypes.BsClient.ChkClient(client)
  wbytebuf = bsTypes.ByteBuf.Clone(wbuf)
  wbytebuf.copyToSwigObj(len(wbytebuf))
  rbytebuf = bsTypes.ByteBuf(readLen)
  n = bsSerialReqTrans(client.pClient, hndVConn,
                                wbytebuf.getSwigObj(), len(wbytebuf),
                                readLen, rbytebuf.getSwigObj())
  bsError.ChkReturnIsNonNeg(n,
      "Client %s: Serial Transaction request failed." % (client.AttrGetName()))
  rbytebuf.copyFromSwigObj(n)
  return rbytebuf.buf

%}
