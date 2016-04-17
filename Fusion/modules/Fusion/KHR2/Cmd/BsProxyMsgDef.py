################################################################################
#
# BsProxyMsgDef.py
#

""" BotSense IP Proxy Server Message Definitions module.

The messages define are exchanged between the BotSense Proxy Server and any
set of registerd BotSense clients.

Notes: 
  1. See C header "botsense/bsproxy_if.h" for C equivalents for the proxy
     and BrainPack sensors.
  2. See C header "rcb3/rcb3prot.h" for C equivalents for the RCB-3.
  3. In the future, I need to swig the headers to keep files in sync.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2007.11.11

Copyright (C) 2007, RoadNarrows LLC.
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

#
# Messaging limits
#
BsProxyMsgMaxLen        = 256   # max. total length in bytes

#
# BotSense command/response layout 
#
BsProxyCmdHdrLen        = 3     # msgid + tid + blen
BsProxyRspHdrLen        = 4     # msgid + tid + pf + blen
BsProxyCmdHdrMsgIdIdx   = 0     # command message id index
BsProxyRspHdrMsgIdIdx   = 0     # response message id index
BsProxyCmdHdrTidIdx     = 1     # command transaction id index
BsProxyRspHdrTidIdx     = 1     # response transaction id index
BsProxyCmdHdrBLenIdx    = 2     # command body length index
BsProxyRspHdrPFIdx      = 2     # response pass/fail index
BsProxyRspHdrBLenIdx    = 3     # response body length index
BsProxyCmdBodyIdx       = 3     # command body start index
BsProxyRspBodyIdx       = 4     # response body start index
BsProxyCmdBodyLenMax    = BsProxyMsgMaxLen-BsProxyCmdHdrLen
                                # max. command body length
BsProxyRspBodyLenMax    = BsProxyMsgMaxLen-BsProxyRspHdrLen
                                # max. response body length

#
# BotSense IP Proxy Server Message Ids
#
BsProxyMsgIdLog         = 'l'   # set diagnostics logging level
BsProxyMsgIdVersion     = 'v'   # proxy server version
BsProxyMsgIdError       = 'e'   # proxy server error message
BsProxyMsgIdLoopback    = 'b'   # loopback command body
BsProxyMsgIdProxyInfo   = '?'   # get info on all proxied devices
BsProxyMsgIdDevOpen     = 'o'   # open a [new] proxied device
BsProxyMsgIdDevClose    = 'c'   # close a proxied device
BsProxyMsgIdDevRead     = 'r'   # raw read of proxied device data
BsProxyMsgIdDevWrite    = 'w'   # raw write to proxied device
BsProxyMsgIdDevTrans    = 't'   # raw write/read transaction
BsProxyMsgIdDevIoctl    = 'i'   # ioctl configuration
BsProxyMsgIdDevScan     = 's'   # scan for all connected devices
BsProxyMsgIdDevCmd      = '!'   # proxied device specific command

#
# Proxied device types
#
BsProxyDevTypeNone      = 0x00  # no device
BsProxyDevTypeI2C       = 0x01  # generic I2C device
BsProxyDevTypeBPFoot    = 0x02  # BrainPack I2C foot
BsProxyDevTypeBPIMU     = 0x03  # BrainPack I2C Inertia Measurement Unit
BsProxyDevTypeBPHand    = 0x04  # BrainPack I2C hand 
BsProxyDevTypeBPCompass = 0x05  # BrainPack I2C compass 
BsProxyDevTypeRS232     = 0x06  # generic RS-232 serial device
BsProxyDevTypeRCB3      = 0x07  # RCB-3 robot controller board over serial

#
# Response pass/fail field values
#
BsProxyRspPass          = 'P'
BsProxyRspFail          = 'F'

#
# BrainPack Foot Proxied Device Specific Commands
#
BsProxyBPFootCmdIdGetIds    = 'i' # get identifiers
BsProxyBPFootCmdIdCal       = 'c' # calibrate foot sensors
BsProxyBPFootCmdIdGetRaw    = 'r' # get raw sensor data 
BsProxyBPFootCmdIdGetCooked = 'd' # get cooked sensor data 

BsProxyBPFootNumOfSensors   = 8   # number of foot sensors

#
# BrainPack Inertia Measurement Unit Proxied Device Specific Commands
#
BsProxyBPIMUCmdIdGetIds     = 'i' # get identifiers
BsProxyBPIMUCmdIdCal        = 'c' # calibrate sensors
BsProxyBPIMUCmdIdGetRaw     = 'r' # get raw sensor data 
BsProxyBPIMUCmdIdGetCooked  = 'd' # get cooked sensor data 
BsProxyBPIMUCmdIdSetOrient  = 'o' # set IMU orientation

BsProxyBPIMUNumOfSensors    = 3   # number of accelerometer sensors

#
# BrainPack RCB-3 Controller Proxied Device Specific Commands
#
RCB3CmdIdGetCurPos    = 0xf0      # get servos current position
RCB3CmdIdStop         = 0xf3      # stop all movement
RCB3CmdIdPlay         = 0xf4      # playback a motion or scenario
RCB3CmdIdMoveServo    = 0xfe      # move one servo
RCB3CmdIdGetVersion   = 0xff      # get RCB-3 version 

RCB3_PORT_NUM_MAX     = 24

RCB3_POS_CENTER       = 16384
RCB3_POS_MIN          =     1
RCB3_POS_MAX          = 32767
RCB3_POS_RAW_90DEG    =   300 

def RCB3PosDegToRaw(degrees):
  """ Convert RCB-3 servo position in degrees to raw position. """
  return int(RCB3_POS_CENTER + ((RCB3_POS_RAW_90DEG * (degrees)) / 90.0))

def RCB3PosRawToDeg(raw):
  """ Convert RCB-3 servo raw position to degrees. """
  if raw>=RCB3_POS_MIN and raw<=RCB3_POS_MAX:
    return float(raw-RCB3_POS_CENTER) * 90.0 / float(RCB3_POS_RAW_90DEG)
  else:
    return 0.0

