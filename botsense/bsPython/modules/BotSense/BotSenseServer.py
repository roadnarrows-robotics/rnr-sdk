## \package BotSense.BotSenseServer
#
# \file 
#
# \brief \h_botsense client - server connection and request functions.
# 
# \pkgsynopsis
#   \h_bs_plain Python Interface
# 
# \pkgcomponent{Module}
#   \h_bs_plain.BotSenseServer
#
# \pkgfile{BotSense/BotSenseServer.py}
#
# \sa
# \htmlonly
#  <a href="../pydoc/BotSense.BotSenseServer.html">PyDoc Generated Documentation</a>
# \endhtmlonly
#
# \author Robin Knight (robin.knight@roadnarrows.com)
#  
# \license{MIT}
#  
# \copyright
#   \h_copy 2010-2017. RoadNarrows LLC.\n
#   http://www.roadnarrows.com\n
#   All Rights Reserved
#
# \EulaBegin
# Permission is hereby granted, without written agreement and without
# license or royalty fees, to use, copy, modify, and distribute this
# software and its documentation for any purpose, provided that
# (1) The above copyright notice and the following two paragraphs
# appear in all copies of the source code and (2) redistributions
# including binaries reproduces these notices in the supporting
# documentation.   Substantial modifications to this software may be
# copyrighted by their authors and need not follow the licensing terms
# described here, provided that the new terms are clearly indicated in
# all files where they apply.
# \n\n
# IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
# OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
# PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
# DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
# EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGE.
# \n\n
# THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
# "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
# PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
# \EulaEnd

# //////////////////////////////////////////////////////////////////////////////

import BotSenseCore as bsCore
import BotSenseError as bsError
import BotSenseTypes as bsTypes

## \brief Connect to the bsProxy server.
#
# Raises a BotSenseError exception on failure.
#
# \param client         BotSenseTypes.BsClient instance.
# \param serverHostName Server's hostname in either a network or a 
#                       dotted IP address.
# \param serverPort     Server's listen port number.
#
def ServerConnect(client, serverHostName='localhost',
                  serverPort=bsCore.BSPROXY_LISTEN_PORT_DFT):
  bsTypes.BsClient.ChkClient(client)
  rc = bsCore.bsServerConnect(client.pClient, serverHostName, serverPort)
  bsError.ChkReturnIsOk(rc,
      "Client %s: Failed to connect to bsProxy server at %s:%d." % \
          (client.AttrGetName(), serverHostName, serverPort))

## \brief Disconnect from the bsProxy server.
#
# Raises a BotSenseError exception on failure.
#
# \param client   BotSenseTypes.BsClient instance.
#
def ServerDisconnect(client):
  bsTypes.BsClient.ChkClient(client)
  rc = bsCore.bsServerDisconnect(client.pClient)
  bsError.ChkReturnIsOk(rc,
    "Client %s: Failed to disconnect from server." % (client.AttrGetName()))

## \brief Proxy server request to get the bsProxy server's version string.
#
# Raises a BotSenseError exception on failure.
#
# \param client   BotSenseTypes.BsClient instance.
#
# \return Version string response.
#
def ServerReqGetVersion(client):
  verstr = '\x00' * 256
  rc = bsCore.bsServerReqGetVersion(client.pClient, verstr, len(verstr))
  bsError.ChkReturnIsOk(rc,
      "Client %s: Server GetVersion request failed." % (client.AttrGetName()))
  return verstr.rstrip('\x00')

## \brief Proxy server request to loopback the given message.
#
# Raises a BotSenseError exception on failure.
#
# \param client       BotSenseTypes.BsClient instance.
# \param loopbackMsg  Message string to loopback.
#
# \return Loopbacked message response.
#
def ServerReqLoopback(client, loopbackMsg):
  bsTypes.BsClient.ChkClient(client)
  txrxMsg = loopbackMsg
  rc = bsCore.bsServerReqLoopback(client.pClient, txrxMsg)
  bsError.ChkReturnIsOk(rc,
      "Client %s: Server Loopback request failed." % (client.AttrGetName()))
  return txrxMsg

## \brief Proxy server request to set the server's diagnostics logging level.
#
# Raises a BotSenseError exception on failure.
#
# \param client   BotSenseTypes.BsClient instance.
# \param level    Log level.
#
def ServerReqSetLogging(client, level):
  bsTypes.BsClient.ChkClient(client)
  rc = bsCore.bsServerReqSetLogging(client.pClient, level)
  bsError.ChkReturnIsOk(rc,
      "Client %s: Server SetLogging request failed." % (client.AttrGetName()))

## \brief Proxy server request to enable/disable message tracing on a virtual
#         connection.
#
# Raises a BotSenseError exception on failure.
#
# \param client   BotSenseTypes.BsClient instance.
# \param hndVConn Virtual connection handle.
# \param trace    Enable(true) or disable(false) message tracing.
#
def ServerReqMsgTrace(client, hndVConn, trace):
  bsTypes.BsClient.ChkClient(client)
  if trace:
    trace = True
  else:
    trace = False
  rc = bsCore.bsServerReqMsgTrace(client.pClient, hndVConn, trace)
  bsError.ChkReturnIsOk(rc,
      "Client %s: Server MsgTrace request failed." % (client.AttrGetName()))

## \brief Proxy server request to establish a virtual connection to the device
#         end point.
#
# The device is open if not already opened by another virtual connection.
# Otherwise it is attached to this vconn.
#
# The interface module is dynamically loaded into the server and provides
# the set of services for the client application communicating with the
# device.
#
# Raises a BotSenseError exception on failure.
#
# \param client   BotSenseTypes.BsClient instance.
# \param devName  Device path name string.
# \param modName  Interface module path name string.
# \param argbuf   Packed buffer of module-specific open parameters.
# \param appInfo  Application-specific information and callbacks (optional).
#                 Set to None if no info. Set any member to None to ignore
#                 that value.
# \param trace    Initial message tracing enable(true)/disable(false) state.
#
# \return New virtual connection handle.
#
def ServerReqOpenDev(client, devName, modName, argbuf, appInfo, trace=False):
  bsTypes.BsClient.ChkClient(client)
  if appInfo is not None:
    bsError.ChkType(appInfo, bsTypes.BsClientAppInfo)
  argByteBuf = bsTypes.ByteBuf.Clone(argbuf)
  argByteBuf.copyToSwigObj(len(argByteBuf))
  hndVConn = bsCore.bsServerReqOpenDev(client.pClient, devName, modName,
                    argByteBuf.getSwigObj(), len(argByteBuf), appInfo, trace)
  bsError.ChkReturnIsNonNeg(hndVConn,
      "Client %s: Server OpenDev request failed." % (client.AttrGetName()))
  return hndVConn

## \brief Proxy server request to close a client's virtual connection.
#
# Raises a BotSenseError exception on failure.
#
# \param client   BotSenseTypes.BsClient instance.
# \param hndVConn Virtual connection handle to close.
#
def ServerReqCloseDev(client, hndVConn):
  bsTypes.BsClient.ChkClient(client)
  rc = bsCore.bsServerReqCloseDev(client.pClient, hndVConn)
  bsError.ChkReturnIsOk(rc,
      "Client %s: Server CloseDev request failed." % (client.AttrGetName()))

## \brief Proxy server request to retrieve the server's list of virtual
#         connection handles for this client.
#
# Raises a BotSenseError exception on failure.
#
# \param client   BotSenseTypes.BsClient instance.
#
# \return List of handles.
#
def ServerReqGetVConnList(client):
  bsTypes.BsClient.ChkClient(client)
  vec = bsCore.BsVecHandles_T()
  rc = bsCore.bsServerReqGetVConnList(client.pClient, vec)
  bsError.ChkReturnIsOk(rc,
      "Client %s: Server GetVConnList request failed." % (client.AttrGetName()))
  listHnds = []
  for i in xrange(vec.m_uCount):
    listHnds += [bsCore.hndVConnArray_getitem(vec.m_vecHnd, i)]
  return listHnds

## \brief Proxy server request to retrieve the servers's information for a
#         given virtual connection.
#
# Raises a BotSenseError exception on failure.
#
# \param client   BotSenseTypes.BsClient instance.
# \param hndVConn Virtual connection handle.
#
# \return Proxied device info.
#
def ServerReqGetVConnInfo(client, hndVConn):
  bsTypes.BsClient.ChkClient(client)
  info = bsTypes.BsVConnInfo()
  rc = bsCore.bsServerReqGetVConnInfo(client.pClient, hndVConn, info)
  bsError.ChkReturnIsOk(rc,
      "Client %s: Server GetVConnInfo request failed." % (client.AttrGetName()))
  return info
