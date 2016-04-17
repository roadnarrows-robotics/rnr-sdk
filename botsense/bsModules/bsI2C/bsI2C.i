/******************************************************************************
 *
 * Package:   BotSense
 *
 * File:      bsI2C.i
 *
 * $LastChangedDate$
 * $Rev$
 */

/*!
 * \file
 *
 * \brief BotSense \h_i2c library python swig interface definitions file.
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

%module bsI2C
%{
#include "botsense/BotSense.h"
#include "botsense/bsI2C.h"
%}

%begin
%{
/*! \file
 *  \brief Swig generated I2C wrapper c file.
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
typedef ushort_t i2c_addr_t;

/*
 * Required BotSense C Types
 */
typedef struct _bsClientStruct *BsClient_P;
typedef int BsVConnHnd_T;

%include "carrays.i"

/* the swigged interface */
%include "botsense/bsI2C.h"

%array_functions(i2c_addr_t, i2cAddrArray);

/*
 * Higher-level python interface to the BotSense I2C C library.
 */
%pythoncode
%{

#
# BotSense I2C Python Wrappers
#

"""
BotSense I2C Python Inline Extensions and Wrappers.
"""

## \file 
## \package BotSense.bsI2C
##
## \brief BotSense Swigged I2C Python Interface Module.
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

# maximum number of scanned I2C attached device addresses
I2CMaxNumOfScannedAddrs = 256

def I2CGetMsgName(client, hndVConn, msgId):
  """ Get the I2C message name.

      For each (virtual connection, message id) 2-tuple, there can be a known
      name string (provided the id is valid and an application provides the
      information).

      Parameters:
        client    - BotSenseTypes.BsClient instance.
        hndVConn  - Virtual connection handle (ignored).
        msgId     - I2C message id.

      Return:
        Returns message name string if it can be determined.
        Otherwise returns 'unknown'.
  """
  bsTypes.BsClient.ChkClient(client)
  return bsI2CGetMsgName(client.pClient, hndVConn, msgId)

def I2CReqOpen(client, devName, trace=False):
  """ Proxy server request to establish a virtual connection to an 
      I2C bus device.

      Raises a BotSenseError exception on failure.

      Parameters:
        client    - BotSenseTypes.BsClient instance.
        devName   - Proxied I2C device name (e.g. /dev/i2c-0).
        trace     - Initial message tracing enable(true)/disable(false) state.

      Return
        New virtual connection handle.
  """
  bsTypes.BsClient.ChkClient(client)
  hndVConn = bsI2CReqOpen(client.pClient, devName, trace)
  bsError.ChkReturnIsNonNeg(hndVConn,
      "Client %s: I2C Open request failed." % (client.AttrGetName()))
  return hndVConn

def I2CReqClose(client, hndVConn):
  """ Proxy server request to close client's proxied I2C bus device virtual
      connection.

      Raises a BotSenseError exception on failure.

      Parameters:
        client    - BotSenseTypes.BsClient instance.
        hndVConn  - Virtual connection handle.
  """
  bsTypes.BsClient.ChkClient(client)
  rc = bsI2CReqClose(client.pClient, hndVConn)
  bsError.ChkReturnIsOk(rc,
      "Client %s: I2C Close request failed." % (client.AttrGetName()))

def I2CReqRead(client, hndVConn, i2cAddr, readLen):
  """ I2C request to read from a device attached to a proxied I2C bus.

      Raises a BotSenseError exception on failure.

      Parameters:
        client    - BotSenseTypes.BsClient instance.
        hndVConn  - Virtual connection handle.
        i2cAddr   - Address of attached device on the I2C bus.
        readLen   - Number of bytes to read.

      Return:
        Buffer list of read bytes.
  """
  bsTypes.BsClient.ChkClient(client)
  rbytebuf = bsTypes.ByteBuf(readLen)
  n = bsI2CReqRead(client.pClient, hndVConn, i2cAddr, readLen,
                  rbytebuf.getSwigObj())
  bsError.ChkReturnIsNonNeg(n,
      "Client %s: I2C Read request failed." % (client.AttrGetName()))
  rbytebuf.copyFromSwigObj(n)
  return rbytebuf.buf

def I2CReqWrite(client, hndVConn, i2cAddr, wbuf):
  """ I2C request to write to a device attached to a proxied I2C bus.

      Raises a BotSenseError exception on failure.

      Parameters:
        client    - BotSenseTypes.BsClient instance.
        hndVConn  - Virtual connection handle.
        i2cAddr   - Address of attached device on the I2C bus.
        wbuf      - Write buffer.

      Return:
        Number of bytes written.
  """
  bsTypes.BsClient.ChkClient(client)
  wbytebuf = bsTypes.ByteBuf.Clone(wbuf)
  wbytebuf.copyToSwigObj(len(wbytebuf))
  n = bsI2CReqWrite(client.pClient, hndVConn, i2cAddr, wbytebuf.getSwigObj(),
                        len(wbytebuf))
  bsError.ChkReturnIsNonNeg(n,
      "Client %s: I2C Write request failed." % (client.AttrGetName()))
  return n

def I2CReqTrans(client, hndVConn, i2cAddr, wbuf, readLen):
  """ I2C request to execute a write-read transaction to a device
      attached to a proxied I2C bus.

      Raises a BotSenseError exception on failure.

      Parameters:
        client    - BotSenseTypes.BsClient instance.
        hndVConn  - Virtual connection handle.
        i2cAddr   - Address of attached device on the I2C bus.
        wbuf      - Write buffer.
        readLen   - Number of bytes to read.

      Return:
        Buffer list of read bytes.
  """
  bsTypes.BsClient.ChkClient(client)
  wbytebuf = bsTypes.ByteBuf.Clone(wbuf)
  wbytebuf.copyToSwigObj(len(wbytebuf))
  rbytebuf = bsTypes.ByteBuf(readLen)
  n = bsI2CReqTrans(client.pClient, hndVConn, i2cAddr,
                                wbytebuf.getSwigObj(), len(wbytebuf),
                                readLen, rbytebuf.getSwigObj())
  bsError.ChkReturnIsNonNeg(n,
      "Client %s: I2C Transaction request failed." % (client.AttrGetName()))
  rbytebuf.copyFromSwigObj(n)
  return rbytebuf.buf

def I2CReqScan(client, hndVConn):
  """ I2C request to scan a proxied I2C bus for attached devices.

      Raises a BotSenseError exception on failure.

      Parameters:
        client    - BotSenseTypes.BsClient instance.
        hndVConn  - Virtual connection handle.

      Return:
        List of I2C address of discovered attached devices.
  """
  bsTypes.BsClient.ChkClient(client)
  swigObj = new_i2cAddrArray(I2CMaxNumOfScannedAddrs)
  n = bsI2CReqScan(client.pClient, hndVConn, swigObj, I2CMaxNumOfScannedAddrs)
  bsError.ChkReturnIsNonNeg(n,
      "Client %s: I2C Read request failed." % (client.AttrGetName()))
  addrs = []
  for i in xrange(n):
    addrs += [ i2cAddrArray_getitem(swigObj, i) ]
  return addrs

%}
