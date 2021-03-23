################################################################################
#
# BsProxyClient.py
#

""" BotSense IP Proxy Client module

The BsProxyClient class provide a messaging interface between controlling
client and the BotSense IP server.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2007.11.12

Copyright (C) 2007.  RoadNarrows LLC.
"""

#
# All Rights Reserved
#
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
################################################################################

import sys
import socket
import errno

import Fusion.KHR2.Cmd.BsProxyPacker as BsProxyPacker
import Fusion.KHR2.Cmd.BsProxyMsgDef as MsgDef

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

# all supported proxied devices
BsProxiedDevAll = [
        'i2c', 'bpfoot_left', 'bpfoot_right', 'bphand_left', 'bphand_right',
        'bpimu', 'bpcomass', 'rs232', 'rcb3']

# proxied device subsets
BsProxiedDevFeet      = ['bpfoot_left', 'bpfoot_right']
BsProxiedDevHands     = ['bphand_left', 'bphand_right']
BsProxiedDevBrainPack = ['bpimu', 'bpcompass'] + BsProxiedDevFeet + \
                          BsProxiedDevHands
BsProxiedDevI2CClass  = ['i2c'] + BsProxiedDevBrainPack
BsProxiedDevRS23Class = ['rs232', 'rcb3']

BsProxiedDevId        = {
    'i2c':          MsgDef.BsProxyDevTypeI2C,
    'bpfoot_left':  MsgDef.BsProxyDevTypeBPFoot,
    'bpfoot_right': MsgDef.BsProxyDevTypeBPFoot,
    'bpimu':        MsgDef.BsProxyDevTypeBPIMU,
    'bphand_left':  MsgDef.BsProxyDevTypeBPHand,
    'bphand_right': MsgDef.BsProxyDevTypeBPHand,
    'bpcompass':    MsgDef.BsProxyDevTypeBPCompass,
    'rc232':        MsgDef.BsProxyDevTypeRS232,
    'rcb3':         MsgDef.BsProxyDevTypeRCB3
}

#-------------------------------------------------------------------------------
# CLASS: BsProxyClient
#-------------------------------------------------------------------------------
class BsProxyClient:
  """ BotSense Proxy IP Client for BrainPack I2C devices

      Supported proxied devices:
        See BsProxiedDevAll
  """

  #--
  def __init__(self, bsproxy_addr='127.0.0.1', bsproxy_port=9195):
    """ Initialized client instance.

        Parameters:
          bsproxy_addr   - IP address of server (dotted or dns name)
          bsproxy_port   - TCP port number of server.
    """
    self.mBsProxyAddr   = bsproxy_addr        # proxy server IP address
    self.mBsProxyPort   = bsproxy_port        # proxy server TCP port
    self.mBsSocket      = socket.socket()     # client socket
    self.mBsIsConn      = False               # client is [not] connected
    self.mBsProxiedDev  = {}                  # BrainPack proxied devices
    for proxdev in BsProxiedDevAll:
      self.mBsProxiedDev[proxdev] = {'i2c':0x00, 'handle':None}
    self.mPacker        = BsProxyPacker.BsProxyPacker() # message [un]packer

  #--
  def SetProxyAddr(self, bsproxy_addr):
    """ Set the IP address of the location of proxy server.

        Parameters:
          bsproxy_addr   - IP address of server (dotted or dns name)
    """
    self.mBsProxyAddr = bsproxy_addr

  #--
  def SetProxyPort(self, bsproxy_port):
    """ Set the TCP port that the proxy server's listener.

        Parameters:
          bsproxy_port   - TCP port number of server.
    """
    self.mBsProxyPort = bsproxy_port


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # BotSense Client-Server Comands/Responses
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def CmdBpFoot(self, whichfoot, footcmd):
    """ BrainPack foot commands.

        Parameters:
          whichfoot   - One of: 'bpfoot_left', 'bpfoot_right'
          footcmd     - One of: 'getraw', 'getcooked', 'getids', 'setcal'

        Return value:
          Dictionary of unpacked response values. For the given footcmd:
            'getraw'      = {'raw_data': [press_raw, ...]}
            'getcooked'   = {'cooked_data': [press_cal, ...]}
            'getids'      = {'device_id':devid, 'version':version}
            'setcal'      = None 
    """
    msgId   = MsgDef.BsProxyMsgIdDevCmd

    if not self.mBsIsConn:
      print("error: connection to server not open", file=sys.stderr)
      return None

    if whichfoot not in BsProxiedDevFeet:
      print("error: %s %s: which foot?" % (cmdName, args[0]), file=sys.stderr)
      return None

    handle = self.mBsProxiedDev[whichfoot]['handle']
    if handle is None or handle < 0:
      print("error: %s: proxied device not found" % whichfoot, file=sys.stderr)
      return None

    # get raw pressure data command
    if footcmd == 'getraw':
      devCmdId = MsgDef.BsProxyBPFootCmdIdGetRaw
      cmd = self.mPacker.PackMsg(msgId,
                [ {'fval':handle, 'ftype':'u8'},
                  {'fval':devCmdId, 'ftype':'u8'},
                ])
      self.sendCmd(cmd)
      rsp = self.recvRsp()
      if not rsp:
        print("error: no response", file=sys.stderr)
        return None
      vals = self.mPacker.UnpackMsg(rsp, msgId,
              [{'fname':'press_raw', 'ftype':'u16',
                'fcount':MsgDef.BsProxyBPFootNumOfSensors}])
      if vals['_rc'] == 'ok':
        return {'raw_data': vals['press_raw']}
      else:
        print("error:", vals['_error_msg'], file=sys.stderr)
        return None

    # get calibrated data command
    elif footcmd == 'getcooked':
      devCmdId = MsgDef.BsProxyBPFootCmdIdGetCooked
      cmd = self.mPacker.PackMsg(msgId,
                [ {'fval':handle, 'ftype':'u8'},
                  {'fval':devCmdId, 'ftype':'u8'},
                ])
      self.sendCmd(cmd)
      rsp = self.recvRsp()
      if not rsp:
        print("error: no response", file=sys.stderr)
        return None
      vals = self.mPacker.UnpackMsg(rsp, msgId,
              [{'fname':'press_cal', 'ftype':'u16',
                'fcount':MsgDef.BsProxyBPFootNumOfSensors}])
      if vals['_rc'] == 'ok':
        return {'cooked_data': vals['press_cal']}
      else:
        print("error:", vals['_error_msg'], file=sys.stderr)
        return None

    # get device id and firmware version command
    elif footcmd == 'getids':
      devCmdId = MsgDef.BsProxyBPFootCmdIdGetIds
      cmd = self.mPacker.PackMsg(msgId,
                [ {'fval':handle, 'ftype':'u8'},
                  {'fval':devCmdId, 'ftype':'u8'},
                ])
      self.sendCmd(cmd)
      rsp = self.recvRsp()
      if not rsp:
        print("error: no response", file=sys.stderr)
        return None
      vals = self.mPacker.UnpackMsg(rsp, msgId,
                [ {'fname':'devId', 'ftype':'u16'},
                  {'fname':'version', 'ftype':'u8'}
                ])
      if vals['_rc'] == 'ok':
        return {'device_id':vals['devId'], 'version':vals['version']}
      else:
        print("error:", vals['_error_msg'], file=sys.stderr)
        return None

    # set foot calibration command
    elif footcmd == 'setcal':
      devCmdId = MsgDef.BsProxyBPFootCmdIdCal
      cmd = self.mPacker.PackMsg(msgId,
                [ {'fval':handle, 'ftype':'u8'},
                  {'fval':devCmdId, 'ftype':'u8'},
                ])
      self.sendCmd(cmd)
      rsp = self.recvRsp()
      if not rsp:
        print("error: no response", file=sys.stderr)
        return None
      vals = self.mPacker.UnpackMsg(rsp, msgId, [])
      if vals['_rc'] == 'error':
        print("error:", vals['_error_msg'], file=sys.stderr)
      return None

    # bad command
    else:
      print("error: %s: unknown device command" % args[1], file=sys.stderr)
      return None
  
  #--
  def CmdBpIMU(self, imucmd, orientation=0):
    """ BrainPack IMU commands.

        Parameters:
          imucmd     - One of: 'getraw', 'getcooked', 'getids', 'setcal'

        Return value:
          Dictionary of unpacked response values. For the given imucmd:
            'getraw'      = {'raw_data': [x_accel, y_accel, z_accel]}
            'getcooked'   = {'cooked_data': [x_accel, y_accel, z_accel]}
            'getids'      = {'device_id':devid, 'version':version}
            'setcal'      = None 
    """
    msgId   = MsgDef.BsProxyMsgIdDevCmd

    if not self.mBsIsConn:
      print("error: connection to server not open", file=sys.stderr)
      return None

    handle = self.mBsProxiedDev['bpimu']['handle']
    if handle is None or handle < 0:
      print("error: bpimu: proxied device not found", file=sys.stderr)
      return None

    # get raw acceleration data command
    if imucmd == 'getraw':
      devCmdId = MsgDef.BsProxyBPIMUCmdIdGetRaw
      cmd = self.mPacker.PackMsg(msgId,
                [ {'fval':handle, 'ftype':'u8'},
                  {'fval':devCmdId, 'ftype':'u8'},
                ])
      self.sendCmd(cmd)
      rsp = self.recvRsp()
      if not rsp:
        print("error: no response", file=sys.stderr)
        return None
      vals = self.mPacker.UnpackMsg(rsp, msgId,
              [{'fname':'accel_raw', 'ftype':'u8',
                'fcount':MsgDef.BsProxyBPIMUNumOfSensors}])
      if vals['_rc'] == 'ok':
        return {'raw_data': vals['accel_raw']}
      else:
        print("error:", vals['_error_msg'], file=sys.stderr)
        return None

    # get calibrated acceleration data command
    elif imucmd == 'getcooked':
      devCmdId = MsgDef.BsProxyBPIMUCmdIdGetCooked
      cmd = self.mPacker.PackMsg(msgId,
                [ {'fval':handle, 'ftype':'u8'},
                  {'fval':devCmdId, 'ftype':'u8'},
                ])
      self.sendCmd(cmd)
      rsp = self.recvRsp()
      if not rsp:
        print("error: no response", file=sys.stderr)
        return None
      vals = self.mPacker.UnpackMsg(rsp, msgId,
              [{'fname':'accel_cal', 'ftype':'u8',
                'fcount':MsgDef.BsProxyBPIMUNumOfSensors}])
      if vals['_rc'] == 'ok':
        return {'cooked_data': vals['accel_cal']}
      else:
        print("error:", vals['_error_msg'], file=sys.stderr)
        return None

    # get device id and firmware version command
    elif imucmd == 'getids':
      devCmdId = MsgDef.BsProxyBPIMUCmdIdGetIds
      cmd = self.mPacker.PackMsg(msgId,
                [ {'fval':handle, 'ftype':'u8'},
                  {'fval':devCmdId, 'ftype':'u8'},
                ])
      self.sendCmd(cmd)
      rsp = self.recvRsp()
      if not rsp:
        print("error: no response", file=sys.stderr)
        return None
      vals = self.mPacker.UnpackMsg(rsp, msgId,
                [ {'fname':'devId', 'ftype':'u16'},
                  {'fname':'version', 'ftype':'u8'}
                ])
      if vals['_rc'] == 'ok':
        return {'device_id':vals['devId'], 'version':vals['version']}
      else:
        print("error:", vals['_error_msg'], file=sys.stderr)
        return None

    # set foot calibration command
    elif imucmd == 'setcal':
      devCmdId = MsgDef.BsProxyBPIMUCmdIdCal
      cmd = self.mPacker.PackMsg(msgId,
                [ {'fval':handle, 'ftype':'u8'},
                  {'fval':devCmdId, 'ftype':'u8'},
                  {'fval':orientation, 'ftype':'u8'}
                ])
      self.sendCmd(cmd)
      rsp = self.recvRsp()
      if not rsp:
        print("error: no response", file=sys.stderr)
        return None
      vals = self.mPacker.UnpackMsg(rsp, msgId, [])
      if vals['_rc'] == 'error':
        print("error:", vals['_error_msg'], file=sys.stderr)
      return None

    # bad command
    else:
      print("error: %s: unknown device command" % args[1], file=sys.stderr)
      return None
  
  #--
  def CmdRCB3Version(self):
    """ RCB-3 command to get RCB-3 version string.

        Return value:
          Version string.
    """
    rsp = self.CmdDevCmd('rcb3', MsgDef.RCB3CmdIdGetVersion,
              [],
              [{'fname':'version', 'ftype':'zstr'}])
    if rsp:
      return rsp['version']
    else:
      return None

  #--
  def CmdRCB3GetPos(self):
    """ RCB-3 command to get servo positions.

        Return value:
          Vector of servo positions in degrees.
    """
    rsp = self.CmdDevCmd('rcb3', MsgDef.RCB3CmdIdGetCurPos,
              [],
              [{'fname':'pos', 'ftype':'u16', 'fcount':24}])
    if rsp:
      poslist = []
      for pos in rsp['pos']:
        poslist += [MsgDef.RCB3PosRawToDeg(pos)]
      return poslist
    else:
      return None

  #--
  def CmdRCB3Move(self, servo, speed, pos):
    """ RCB-3 command to move one servo.

        Parameters:
          servo   - Servo number [0,23]
          speed   - Servo speed [1,127]
          pos     - Servo end position in degrees.

        Return value:
          (speed, pos)
    """
    pos = MsgDef.RCB3PosDegToRaw(pos)
    rsp = self.CmdDevCmd('rcb3', MsgDef.RCB3CmdIdMoveServo,
        [{'fval':servo, 'ftype':'u8'},
         {'fval':pos, 'ftype':'u16'},
         {'fval':speed, 'ftype':'u8'}],
        [])
    if rsp:
      return (speed, pos)
    else:
      return None
  
  #--
  def CmdRCB3Play(self, playindex):
    """ RCB-3 command to playback a motion or scenario..

        Parameters:
          playindex   - Play index [0-84]

        Return value:
          playindex
    """
    rsp = self.CmdDevCmd('rcb3', MsgDef.RCB3CmdIdPlay,
        [{'fval':playindex, 'ftype':'u8'}],
        [])
    if rsp:
      return playindex
    else:
      return None
  
  #--
  def CmdRCB3Stop(self):
    """ RCB-3 command to stop all robot motion.

        Return value:
          True
    """
    rsp = self.CmdDevCmd('rcb3', MsgDef.RCB3CmdIdStop, [], [])
    if rsp:
      return True
    else:
      return None

  #--
  def CmdI2CScan(self, proxdev):
    """ BrainPack I2C scan command.

        Parameters:
          proxdev   - One of the supported proxied devices.

        Return value:
          List of addresses of discovered I2C devices.
    """
    msgId   = MsgDef.BsProxyMsgIdDevScan

    if not self.mBsIsConn:
      print("error: connection to server not open", file=sys.stderr)
      return None

    handle = self.mBsProxiedDev[proxdev]['handle']
    if handle is None or handle < 0:
      print("error: %s: proxied device not found" % proxdev, file=sys.stderr)
      return None

    cmd = self.mPacker.PackMsg(msgId, [{'fval':handle, 'ftype':'u8'}])
    self.sendCmd(cmd)
    rsp = self.recvRsp()
    if not rsp:
      print("error: no response", file=sys.stderr)
      return None
    vals = self.mPacker.UnpackMsg(rsp, msgId,
                                  [{'fname':'num_scanned', 'ftype':'u8'}])
    if vals['_rc'] == 'ok':
      num_scanned = vals['num_scanned']
      scanned = []
      for byte in rsp[vals['_pos']:]:
        if num_scanned <= 0:
          break
        if type(byte) == str:
          scanned += [ord(byte)]
        else:
          scanned += [byte]
        num_scanned -= 1
      return scanned
    else:
      print("error:", vals['_error_msg'], file=sys.stderr)
      return None
  
  #--
  def CmdDevCmd(self, proxdev, devCmdId, packFmt, unpackFmt):
    """ General Device command.

        Parameters:
          proxdev   - One of the supported proxied devices.
          devCmdId  - Proxied device command id.
          packFmt   - Data to send in Packer format sans handle and cmd id.
          unpackFmt - Data to receive in Packer format.

        Return value:
          Packer.UnpackMsg() return value dictionary.
    """
    msgId   = MsgDef.BsProxyMsgIdDevCmd

    if not self.mBsIsConn:
      print("error: connection to server not open", file=sys.stderr)
      return None

    handle = self.mBsProxiedDev[proxdev]['handle']
    if handle is None or handle < 0:
      print("error: %s: proxied device not found" % proxdev, file=sys.stderr)
      return None

    packAll = \
        [{'fval':handle, 'ftype':'u8'}, {'fval':devCmdId, 'ftype':'u8'}] + \
        packFmt
    cmd = self.mPacker.PackMsg(msgId, packAll)
    self.sendCmd(cmd)
    rsp = self.recvRsp()
    if not rsp:
      print("error: no response", file=sys.stderr)
      return None
    vals = self.mPacker.UnpackMsg(rsp, msgId, unpackFmt)
    if vals['_rc'] == 'ok':
      return vals
    else:
      print("error:", vals['_error_msg'], file=sys.stderr)
      return None
   
  #--
  def CmdDevOpen(self, proxdev, i2cAddr, devName):
    """ Open a proxied device.

        Parameters:
          proxdev   - One of the supported proxied devices.
          i2cAddr   - I2C address of attached device.
          devName   - Name of device file.

        Return value:
          Handle to proxied device.
    """
    msgId   = MsgDef.BsProxyMsgIdDevOpen

    if not self.mBsIsConn:
      print("error: connection to server not open", file=sys.stderr)
      return None

    devType = BsProxiedDevId.get(proxdev)
    if not devType:
      print("error: 0x%02x: unknown type" % proxdev, file=sys.stderr)
      return None

    if proxdev in BsProxiedDevBrainPack:
      cmd = self.mPacker.PackMsg(msgId,
            [ {'fval':devType, 'ftype':'u8'},
              {'fval':i2cAddr, 'ftype':'u8'},
              {'fval':devName, 'ftype':'zstr'}
            ])
    else:
      cmd = self.mPacker.PackMsg(msgId,
            [ {'fval':devType, 'ftype':'u8'},
              {'fval':devName, 'ftype':'zstr'}
            ])
    self.sendCmd(cmd)
    rsp = self.recvRsp()
    if not rsp:
      print("error: no response", file=sys.stderr)
      return None
    vals = self.mPacker.UnpackMsg(rsp, msgId,  
                          [{'fname':'handle', 'ftype':'u8'}],
                          self.mPacker.GetLastTid())
    if vals['_rc'] == 'ok':
      handle = vals['handle']
      print("proxied device %s's handle is %d" % (proxdev, handle))
      self.mBsProxiedDev[proxdev]['i2c']     = i2cAddr
      self.mBsProxiedDev[proxdev]['handle']  = handle
      return handle
    else:
      print("error:", vals['_error_msg'], file=sys.stderr)
      return None
  
  #--
  def CmdDevClose(self, handle):
    """ Closed proxied device.

        Parameters:
          handle    - Handle to opened proxied deivce.

        Return Value:
          True on success, False on failure.
    """
    msgId   = MsgDef.BsProxyMsgIdDevDel

    if not self.mBsIsConn:
      print("error: connection to server not open", file=sys.stderr)
      return None

    cmd = self.mPacker.PackMsg(msgId, [{'fval':handle, 'ftype':'u8'}])
    self.sendCmd(cmd)
    rsp = self.recvRsp()
    if not rsp:
      print("error: no response", file=sys.stderr)
      return None
    vals = self.mPacker.UnpackMsg(rsp, msgId,
                                [{'fname':'handle', 'ftype':'u8'}])
    if vals['_rc'] == 'ok':
      for proxdev in self.mBsProxiedDev.iterkeys():
        if self.BsClientDev[proxdev]['handle'] == handle:
          self.mBsProxiedDev[proxdev]['handle'] = None
          return True
      return False
    else:
      print("error:", vals['_error_msg'], file=sys.stderr)
      return False
  

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # BotSense Server Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def CmdGetVersion(self):
    """ Get BotSense Proxy server's version.

        Return value:
          Version string.
    """
    msgId   = MsgDef.BsProxyMsgIdVersion

    if not self.mBsIsConn:
      print("error: connection to server not open", file=sys.stderr)
      return None

    cmd = self.mPacker.PackMsg(msgId, [])
    self.sendCmd(cmd)
    rsp = self.recvRsp()
    if not rsp:
      print("error: no response", file=sys.stderr)
      return None
    vals = self.mPacker.UnpackMsg(rsp, msgId,
                                  [{'fname':'version', 'ftype':'zstr'}])
    if vals['_rc'] == 'ok':
      print(vals['version'])
      return vals['version']
    else:
      print("error:", vals['_error_msg'], file=sys.stderr)
      return None
  
  #--
  def CmdSetLogLevel(self, level):
    """ Get BotSense Proxy server's version.

        Parameters:
          level   - Symbolic or integer log level.

        Return value:
          Version string.
    """
    msgId   = MsgDef.BsProxyMsgIdLog

    if not self.mBsIsConn:
      print("error: connection to server not open", file=sys.stderr)
      return None

    logLevels = ['off', 'error', 'diag1', 'diag2', 'diag3', 'diag4', 'diag5'] 

    try:
      nLevel = logLevels.index(level)
    except ValueError:
      nLevel = level

    if type(nLevel) != int:
      print("error: %s: invalid log level" % repr(level), file=sys.stderr)
      return None

    cmd = self.mPacker.PackMsg(msgId, [{'fval':nLevel, 'ftype':'u8'}])
    self.sendCmd(cmd)

    rsp = self.recvRsp()
    if not rsp:
      print("error: no response", file=sys.stderr)
      return None
    vals = self.mPacker.UnpackMsg(rsp, msgId,
                                  [{'fname':'level', 'ftype':'u8'}])
    if vals['_rc'] == 'ok':
      print("Server log level set to", vals['level'], ".")
      return vals['level']
    else:
      print("error:", vals['_error_msg'], file=sys.stderr)
      return None
  
  #--
  def CmdLoopback(self, msg):
    """ Get BotSense Proxy server's version.

        Parameters:
          level   - Symbolic or integer log level.

        Return value:
          Version string.
    """
    msgId   = MsgDef.BsProxyMsgIdLoopback

    if not self.mBsIsConn:
      print("error: connection to server not open", file=sys.stderr)
      return None

    cmd = self.mPacker.PackMsg(msgId, [{'fval':msg, 'ftype':'zstr'}])
    self.sendCmd(cmd)

    rsp = self.recvRsp()
    if not rsp:
      print("error: no response", file=sys.stderr)
      return None
    vals = self.mPacker.UnpackMsg(rsp, msgId,
                                  [{'fname':'msg', 'ftype':'zstr'}])
    if vals['_rc'] == 'ok':
      print("Loopbacked: '%s'" % vals['msg'])
      return vals['msg']
    else:
      print("error:", vals['_error_msg'], file=sys.stderr)
      return None
  
  #--
  def CmdGetProxyInfo(self):
    """ Get BotSense Proxy server's supported proxied device types.

        Return value:
          Version string.
    """
    msgId   = MsgDef.BsProxyMsgIdProxyInfo

    if not self.mBsIsConn:
      print("error: connection to server not open", file=sys.stderr)
      return None

    cmd = self.mPacker.PackMsg(msgId, [])
    self.sendCmd(cmd)

    rsp = self.recvRsp()
    if not rsp:
      print("error: no response", file=sys.stderr)
      return None
    vals = self.mPacker.UnpackMsg(rsp, msgId, [])
    if vals['_rc'] == 'ok':
      num_dev = vals['blen']
      devidlist = []
      for byte in rsp[vals['_pos']:]:
        if num_dev <= 0:
          break
        if type(byte) == str:
          devidlist += [ord(byte)]
        else:
          devidlist += [byte]
        num_dev -= 1
      print("Server supported proxied device ids:", devidlist)
      proxlist = []
      for proxdev,devid in BsProxiedDevIds.iteritems():
        try:
          devidlist.index(devid)
          proxlist += [proxdev]
        except:
          pass
      return proxlist
    else:
      print("error:", vals['_error_msg'], file=sys.stderr)
      return None
  
  #--
  def Connect(self, bsproxy_addr=None, bsproxy_port=None):
    """ Open TCP connection to BotSense Proxy server.

        Parameters:
          bsproxy_addr   - IP address of server (dotted or dns name)
          bsproxy_port   - TCP port number of server.

        Return value:
          True on success, False on failure.
    """
    if self.mBsIsConn:
      self.Disconnect()
    if bsproxy_addr is not None:
      self.mBsProxyAddr = bsproxy_addr
    if bsproxy_port is not None:
      self.mBsProxyPort = bsproxy_port
    rc = self.mBsSocket.connect_ex((self.mBsProxyAddr, self.mBsProxyPort))
    if rc != 0:
      print("Failed to connect, rc=%d (%s)" % \
          (rc, errno.errorcode.get(rc, 'UNKNOWN')))
      return False
    for proxdev in self.mBsProxiedDev.iterkeys():
      self.mBsProxiedDev[proxdev]['handle'] = None
    self.mBsIsConn = True
    print("Connected to bsproxy@%s:%d" % (self.mBsProxyAddr, self.mBsProxyPort))
    return True

  #--
  def Disconnect(self):
    """ Close connection to BotSense IP server.
    """
    if self.mBsIsConn:
      self.mBsIsConn = False
      self.mBsSocket.close()
      del self.mBsSocket                # cannot be used after close(), delete
      self.mBsSocket = socket.socket()  # and recreate
      for proxdev in self.mBsProxiedDev.iterkeys():
        self.mBsProxiedDev[proxdev]['handle'] = None
      print("Disconnected from bsproxy@%s:%d" % \
        (self.mBsProxyAddr, self.mBsProxyPort))


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Basic Message Transmisioin Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def sendCmd(self, cmd, trace=False):
    """ Send IP message to server.

        Parameters:
          cmd   - Command to send. Can be of type list or string.
          trace - Do [not] dump of command bytes to stdout.
    """
    if type(cmd) is list:
      sCmd = self.makeMsg(cmd)
    else:
      sCmd = cmd
    if trace:
      self.mPacker.DumpBytes(sCmd)
    self.mBsSocket.sendall(sCmd)
    
  #--
  def recvRsp(self, trace=False):
    """ Receive IP message from server.

        Parameters:
          trace - Do [not] dump of response bytes to stdout.

        Return value:
          String of recieved bytes.
    """
    rsp = self.mBsSocket.recv(MsgDef.BsProxyMsgMaxLen)
    if trace:
      self.mPacker.DumpBytes(rsp)
    return rsp
  
  #--
  def makeMsg(self, bytelist):
    """ Convert list of bytes to string.

        Parameters:
          bytelist  - byte list

        Return value:
          Converted string.
    """
    sMsg = ''
    for byte in bytelist:
      sMsg += "%c" % (byte)
    return sMsg
