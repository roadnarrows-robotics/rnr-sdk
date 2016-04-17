/******************************************************************************
 *
 * Package:   BotSense
 *
 * File:      bsNull.i
 *
 * $LastChangedDate$
 * $Rev$
 */

/*!
 * \file
 *
 * \brief BotSense /dev/null library python swig interface definitions file.
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

%module bsNull
%{
#include "botsense/BotSense.h"
#include "botsense/bsNull.h"
%}

%begin
%{
/*! \file
 *  \brief Swig generated null wrapper c file.
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
%include "botsense/bsNull.h"


/*
 * Higher-level python interface to the BotSense Null C library.
 */
%pythoncode
%{

"""
BotSense DevNull Python Inline Extensions and Wrappers.
"""

## \file 
## \package BotSense.bsNull
##
## \brief BotSense Swigged Null Python Interface Module.
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


def NullGetMsgName(client, hndVConn, msgId):
  """ Get the DevNull message name.

      For each (virtual connection, message id) 2-tuple, there can be a known
      name string (provided the id is valid and an application provides the
      information).

      Parameters:
        client    - BotSenseTypes.BsClient instance.
        hndVConn  - Virtual connection handle (ignored).
        msgId     - DevNull message id.

      Return:
        Returns message name string if it can be determined.
        Otherwise returns 'unknown'.
  """
  bsTypes.BsClient.ChkClient(client)
  return bsNullGetMsgName(client.pClient, hndVConn, msgId)

def NullReqOpen(client, trace=False):
  """ Proxy server request to establish a virtual connection to the /dev/null
      device.

      Raises a BotSenseError exception on failure.

      Parameters:
        client    - BotSenseTypes.BsClient instance.
        trace     - Initial message tracing enable(true)/disable(false) state.

      Return
        New virtual connection handle.
  """
  bsTypes.BsClient.ChkClient(client)
  hndVConn = bsNullReqOpen(client.pClient, trace)
  bsError.ChkReturnIsNonNeg(hndVConn,
      "Client %s: DevNull Open request failed." % (client.AttrGetName()))
  return hndVConn

def NullReqClose(client, hndVConn):
  """ Proxy server request to close client's proxied /dev/null device virtual
      connection.

      Raises a BotSenseError exception on failure.

      Parameters:
        client    - BotSenseTypes.BsClient instance.
        hndVConn  - Virtual connection handle.
  """
  bsTypes.BsClient.ChkClient(client)
  rc = bsNullReqClose(client.pClient, hndVConn)
  bsError.ChkReturnIsOk(rc,
      "Client %s: DevNull Close request failed." % (client.AttrGetName()))

def NullReqWrite(client, hndVConn, wbuf):
  """ DevNull request to write /dev/null.

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
  n = bsNullReqWrite(client.pClient, hndVConn, wbytebuf.getSwigObj(),
                        len(wbytebuf))
  bsError.ChkReturnIsNonNeg(n,
      "Client %s: DevNull Write request failed." % (client.AttrGetName()))
  return n

%}
