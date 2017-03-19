#!/usr/bin/python
###############################################################################
#
# File: bsUnitTest.py
#

""" 
BotSense Unit Test Script
"""

## \file 
##
## $LastChangedDate: 2010-09-13 10:25:05 -0600 (Mon, 13 Sep 2010) $
## $Rev: 581 $
##
## \brief BotSense Unit Tester
##
## \todo Re-write to fit into BotSense 3.0
##
## \sa
## \htmlonly
##  <a href="../pydoc/bsUnitTest.html">PyDoc Generated Documentation</a>
## \endhtmlonly
##
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \copyright
##   \h_copy 2007-2017. RoadNarrows LLC.\n
##   http://www.roadnarrows.com\n
##   All Rights Reserved
##

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
#
# IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS/EMPLOYEES
# OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE LIABLE TO ANY
# PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
# DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION,
# EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGE.
#
# THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
# FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER IS ON AN
# "AS IS" BASIS, AND THE AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO
# PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.
#
###############################################################################


import sys
import time

try:
  import readline   # optional module
except ImportError:
  pass

import BotSense.BotSenseTypes as bsTypes

#--
def CmdBPFoot(args):
  cmdName = 'bpfoot'
  msgId   = BsProxyMsgIdDevCmd
  if len(args) < 2:
    print >>sys.stderr, "error: %s missing argument(s)" % cmdName
    return
  whichfoot = args[0]
  footcmd   = args[1]
  if whichfoot not in ['bpfoot_left', 'bpfoot_right']:
    print >>sys.stderr, "error: %s %s: whichfoot foot?" % (cmdName, args[0])
    return
  handle = BsClientHandles.get(whichfoot)
  if handle is None:
    print >>sys.stderr, "error: %s: proxied device not found" % whichfoot
    return
  if footcmd == 'getids':
    devCmdId = BsProxyBPFootCmdIdGetIds
    cmd = BsPacker.PackMsg(msgId,
                [ {'fval':handle, 'ftype':'u8'},
                  {'fval':devCmdId, 'ftype':'u8'},
                ])
    sendCmd(BsClientSock, cmd)
    rsp = recvRsp(BsClientSock)
    if rsp:
      vals = BsPacker.UnpackMsg(rsp, msgId,
                [ {'fname':'devId', 'ftype':'u16'},
                  {'fname':'version', 'ftype':'u8'}
                ])
      if vals['_rc'] == 'ok':
        print "%s %s device_id=0x%04x version=0x%02x" % \
          (whichfoot, cmdName, vals['devId'], vals['version'])
      else:
        print "error:", vals['_error_msg']
  elif footcmd == 'getraw':
    devCmdId = BsProxyBPFootCmdIdGetRaw
    cmd = BsPacker.PackMsg(msgId,
                [ {'fval':handle, 'ftype':'u8'},
                  {'fval':devCmdId, 'ftype':'u8'},
                ])
    sendCmd(BsClientSock, cmd)
    rsp = recvRsp(BsClientSock)
    if rsp:
      vals = BsPacker.UnpackMsg(rsp, msgId,
              [{'fname':'press_raw', 'ftype':'u16', 'fcount':8}])
      if vals['_rc'] == 'ok':
        print "%s pressure_raw" % (whichfoot),
        for press in vals['press_raw']:
          print press,
        print
      else:
        print "error:", vals['_error_msg']
  elif footcmd == 'getcooked':
    devCmdId = BsProxyBPFootCmdIdGetCooked
    cmd = BsPacker.PackMsg(msgId,
                [ {'fval':handle, 'ftype':'u8'},
                  {'fval':devCmdId, 'ftype':'u8'},
                ])
    sendCmd(BsClientSock, cmd)
    rsp = recvRsp(BsClientSock)
    if rsp:
      vals = BsPacker.UnpackMsg(rsp, msgId,
              [{'fname':'press_cal', 'ftype':'u16', 'fcount':8}])
      if vals['_rc'] == 'ok':
        print "%s pressure_cal" % (whichfoot),
        for press in vals['press_cal']:
          print press,
        print
      else:
        print "error:", vals['_error_msg']
  else:
    print >>sys.stderr, "error: %s: unknown device command" % args[1]

#--
def CmdDevOpen(args):
  cmdName = 'devadd'
  msgId   = BsProxyMsgIdDevOpen
  if len(args) < 2:
    print >>sys.stderr, "error: %s: missing argument(s)" % cmdName
    return
  devTypeName = args[0]
  if devTypeName == 'i2c':
    devType = BsProxyDevTypeI2C
  elif devTypeName == 'bpfoot_left' or devType == 'bpfoot_right':
    devType = BsProxyDevTypeBPFoot
  elif devTypeName == 'bpimu':
    devType = BsProxyDevTypeBPImu
  else:
    print >>sys.stderr, "error: 0x%02x: unknown type" % devTypeName
    return
  i2cAddr = str2int(args[1])
  if i2cAddr is None:
    return
  if len(args) > 2:
    devName = args[2]
  else:
    devName = '/dev/null'
  cmd = BsPacker.PackMsg(msgId,
          [ {'fval':devType, 'ftype':'u8'},
            {'fval':i2cAddr, 'ftype':'u8'},
            {'fval':devName, 'ftype':'zstr'}
          ])
  sendCmd(BsClientSock, cmd)
  rsp = recvRsp(BsClientSock)
  if rsp:
    vals = BsPacker.UnpackMsg(rsp, msgId,  
                          [{'fname':'handle', 'ftype':'u8'}],
                          BsPacker.GetLastTid())
    if vals['_rc'] == 'ok':
      print "proxied device %s's handle is %d" % (devTypeName, vals['handle'])
      BsClientHandles[devTypeName] = vals['handle']
    else:
      print "error:", vals['_error_msg']

#--
def CmdDevClose(args):
  cmdName = 'devdel'
  msgId   = BsProxyMsgIdDevClose
  if len(args) < 1:
    print >>sys.stderr, "error: %s missing argument(s)" % cmdName
    return None
  handle = str2int(args[0])
  if handle is None:
    return
  cmd = BsPacker.PackMsg(msgId, [{'fval':handle, 'ftype':'u8'}])
  sendCmd(BsClientSock, cmd)
  rsp = recvRsp(BsClientSock)
  if rsp:
    vals = BsPacker.UnpackMsg(rsp, msgId, [{'fname':'handle', 'ftype':'u8'}])
    if vals['_rc'] == 'ok':
      for k,v in BsClientHandles.iteritems():
        if v == handle:
          del BsClientHandles[k]
          break
    else:
      print "error:", vals['_error_msg']

#--
def CmdOpen(args):
  cmdName = 'open'
  if len(args) > 0:
    addr = args[0]
  else:
    addr = '127.0.0.1'
  if len(args) > 1:
    port = str2int(args[1])
  else:
    port = 9195
  if port is None:
    return
  rc = BsClientSock.connect_ex((addr, port))
  if rc != 0:
    print "Failed to connect, rc=%d (%s)" % \
          (rc, errno.errorcode.get(rc, 'UNKNOWN'))
    return
  BsClientHandles.clear()
  print "Connected to bsproxy @ %s:%d" % (addr, port)

#--
def CmdClose():
  global BsClientSock
  BsClientSock.close()
  BsClientHandles.clear()

#--
def CmdLoopback(args):
  cmdName = 'loopback'
  msgId   = BsProxyMsgIdLoopback
  if len(args) > 0:
    max = str2int(args[0])
  else:
    max = None
  if len(args) > 1:
    s = args[1]
  else:
    s = 'from ut.py'
  cnt = 0
  while True:
    loopmsg = "loopback "+str(cnt)+" "+s+"\n"
    cmd = BsPacker.PackMsg(msgId, [{'fval':loopmsg, 'ftype':'zstr'}])
    sendCmd(BsClientSock, cmd)
    rsp = recvRsp(BsClientSock)
    if rsp:
      vals = BsPacker.UnpackMsg(rsp, msgId, 
                          [{'fname':'loopmsg', 'ftype':'zstr'}],
                          BsPacker.GetLastTid())
      if vals['_rc'] == 'ok':
        print vals['loopmsg'],
      else:
        print "error:", vals['_error_msg']
    cnt = cnt + 1
    if max is not None and cnt > max:
      return
    time.sleep(1)

#--
def CmdVersion():
  cmdName = 'version'
  msgId   = BsProxyMsgIdVersion
  cmd = BsPacker.PackMsg(msgId, [])
  sendCmd(BsClientSock, cmd)
  rsp = recvRsp(BsClientSock)
  if rsp:
    vals = BsPacker.UnpackMsg(rsp, msgId, [{'fname':'version', 'ftype':'zstr'}])
    if vals['_rc'] == 'ok':
      print 'BotSense Server Version: %s' % vals['version']
    else:
      print "error:", vals['_error_msg']

#--
def CmdInfo():
  cmdName = 'info'
  msgId   = BsProxyMsgIdProxyInfo
  cmd = BsPacker.PackMsg(msgId, [])
  sendCmd(BsClientSock, cmd)
  rsp = recvRsp(BsClientSock)
  if rsp:
    vals = BsPacker.UnpackMsg(rsp, msgId, [])
    if vals['_rc'] == 'ok':
      print 'Server proxied devices:',
      n = vals['blen']
      k = vals['_pos']
      while n > 0:
        print '0x%02x' % ord(rsp[k]),
        n -= 1
        k += 1
      print
    else:
      print "error:", vals['_error_msg']

#--
def CmdLog(args):
  cmdName = 'log'
  msgId   = BsProxyMsgIdLog
  if len(args) < 1:
    print >>sys.stderr, "error: %s: missing argument(s)" % cmdName
    return
  level = str2int(args[0])
  if level is None:
    return
  cmd = BsPacker.PackMsg(msgId, [{'fval':level, 'ftype':'u8'}])
  sendCmd(BsClientSock, cmd)
  rsp = recvRsp(BsClientSock)
  if rsp:
    vals = BsPacker.UnpackMsg(rsp, msgId, [{'fname':'level', 'ftype':'u8'}])
    if vals['_rc'] == 'ok':
      print 'Server logging level: %d' % vals['level']
    else:
      print "error:", vals['_error_msg']

#--
def CmdI2CScan(args):
  cmdName = 'i2cscan'
  msgId   = BsProxyMsgIdDevScan
  if len(args) < 1:
    print >>sys.stderr, "error: %s: missing argument(s)" % cmdName
    return
  handle = str2int(args[0])
  if handle is None:
    return
  cmd = BsPacker.PackMsg(msgId, [{'fval':handle, 'ftype':'u8'}])
  sendCmd(BsClientSock, cmd)
  rsp = recvRsp(BsClientSock)
  if rsp:
    vals = BsPacker.UnpackMsg(rsp, msgId, [{'fname':'nScanned', 'ftype':'u8'}])
    if vals['_rc'] == 'ok':
      nScanned = vals['nScanned']
      print "Scanned %d I2C devices" % nScanned
      for byte in rsp[5:]:
        if nScanned <= 0:
          break
        print "0x%02x" % ord(byte),
        nScanned -= 1
    else:
      print "error:", vals['_error_msg']


BsClientHandles = {}

while True:
  print """
The bsUnitTest python application is being rewritten to fit into BotSense 3.0.
Goodbye.
"""
  sys.exit(8)
  cmd = raw_input("bsclient$ ")
  args = cmd.split()
  if not args:
    pass
  elif args[0] == 'quit':
    break
  elif args[0] == 'open':
    CmdOpen(args[1:])
  elif args[0] == 'close':
    CmdClose()
  elif args[0] == 'loopback':
    CmdLoopback(args[1:])
  elif args[0] == 'version':
    CmdVersion()
  elif args[0] == 'info':
    CmdInfo()
  elif args[0] == 'log':
    CmdLog(args[1:])
  elif args[0] == 'i2cscan':
    CmdI2CScan(args[1:])
  elif args[0] == 'devopen':
    handle = CmdDevOpen(args[1:])
  elif args[0] == 'devclose':
    CmdDevClose(args[1:])
  elif args[0] == 'bpfoot_left' or args[0] == 'bpfoot_right':
    CmdBPFoot(args)
  elif args[0] == 'help':
    print >>sys.stderr, """\
BotSense Server UnitTest Help
-----------------------------

*BrainPack Foot Commands
  Two BP Feet (left and right) are supported. Use command 'devadd' to add each
  foot to the bsproxy proxied devices.

  General Foot Command Format:
    bpfoot_left footcmd [byte...]
    bpfoot_right footcmd [byte...]
      footcmd   - Foot-specific command.
      byte...   - Foot-specific command bytes, if any.

  Specific Foot Commands:
  ... getids          - Get BrainPack foot identifiers.
  ... getraw          - Get foot raw sensor data.
  ... getcooked       - Get foot cooked sensor data.
 

*BotSense Proxy Connection Commands
  Opens, closes, and tests bsproxy connection.

  Commands:
  open [addr [port]]  - Open connection on IP address and port.
                          addr  - IP address. Default: 127.0.0.1
                          port  - IP port. Default: 9195
  close               - Close connection.
  loopback [cnt]      - Loopback cnt times.
                          cnt   - Number of loops. Default is forever.
  version             - Server version.
  info                - List of server supported proxied device types.
  log                 - Set server log level.
  i2cscan handle      - Scan the I2C Bus for available I2C attached devices.
                          handle  - allocated proxied device handle


*Proxied Device General Commands
  Opens, closes, and lists client's proxied devices.

  Commands:
  devopen type i2caddr [devname]
                      - Open proxied device.
                          type    - One of: i2c bpfoot_left bpfoot_right bpimu.
                          i2caddr - I2C address [0x01 - 0x7f].
                          devname - device name. Default: /dev/null
  devclose handle     - Close proxied device.
                          handle  - allocated proxied device handle


*BotSense UnitTest Commands
  help                - Print this help.
  quit                - Quit UnitTest shell.
"""
  else:
    print >>sys.stderr, "Error:", args[0], "; unknown command"

