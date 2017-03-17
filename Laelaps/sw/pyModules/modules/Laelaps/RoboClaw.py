#
# Module:   Laelaps.RoboClaw
#
# Package:  RoadNarrows Laelaps Robotic Mobile Platform Package
#
# Link:     https://github.com/roadnarrows-robotics/laelaps
#
# File:     RoboClaw.py
#
## \file
##
## \brief Python RoboClaw motor controller class and data.
##
## This code is based on the freely available python source from Ion Motion
## Control.
##
## \author: Robin Knight (robin.knight@roadnarrows.com)
## 
## \par Copyright:
##   \h_copy 2015-2016. RoadNarrows LLC.\n
##   http://www.roadnarrows.com\n
##   All Rights Reserved
#
# @EulaBegin@
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
# @EulaEnd@
#

import os
import sys
import serial
import struct
import time

# Defines messages from swigged RoboClaw.h file.
from Laelaps.RoboClawMsgs import *

"""
Duplicate message interface prior to swig of header file.

# . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
# RoboClaw Addresses
# . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
AddrMin = 0x80  ## minimum controller address
AddrMax = 0x87  ## maximum controller address
AddrDft = 0x80  ## default controller address

# . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
# RoboClaw Commands
# . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

# compatibility mode commands
CmdDriveForwardMot1     =  0  ## drive motor 1 forward
CmdDriveBackwardMot1    =  1  ## drive motor 1 backward
CmdSetMinMainVolt       =  2  ## set main battery minimum voltage
CmdSetMaxMainVolt       =  3  ## set main battery maximum voltage
CmdDriveForwardMot2     =  4  ## drive motor 1 forward
CmdDriveBackwardMot2    =  5  ## drive motor 2 backward
CmdDriveMot1            =  6  ## drive motor 1 forward/back (7-bit)
CmdDriveMot2            =  7  ## drive motor 2 foward/back (7-bit)

# mix mode commands
CmdMixDriveForward      =  8  ## drive motors forward
CmdMixDriveBackward     =  9  ## drive motors backward
CmdMixTurnRight         = 10  ## drive motors to turn right
CmdMixTurnLeft          = 11  ## drive motors to turn left
CmdMixDrive             = 12  ## drive motors foward/back (7-bit)
CmdMixTurn              = 13  ## drive motors to turn R/L (7-bit)
  
# advance commands
CmdReadEncoderMot1      = 16  ## read motor 1 encoder
CmdReadEncoderMot2      = 17  ## read motor 2 encoder
CmdReadSpeedMot1        = 18  ## read motor 1 speed (qpss)
CmdReadSpeedMot2        = 19  ## read motor 2 speed (qpss)
CmdResetEncoderRegs     = 20  ## reset encoder registers (quadra)
CmdReadFwVersion        = 21  ## read firmware version
CmdSetEncoderReg1       = 22  ## set encoder register 1 (quadrature)
CmdSetEncoderReg2       = 23  ## set encoder register 2 (quadrature)
CmdReadMainBattVolt     = 24  ## read main battery voltage
CmdReadLogicVolt        = 25  ## read logic battery voltage
CmdSetMinLogicVolt      = 26  ## set logic battery minimum voltage
CmdSetMaxLogicVolt      = 27  ## set logic battery maximum voltage
CmdSetVelPidMot1        = 28  ## set motor 1 velocity PID constants
CmdSetVelPidMot2        = 29  ## set motor 2 velocity PID constants
CmdRead125SpeedMot1     = 30  ## read motor 1 speed (pulses/125th sec)
CmdRead125SpeedMot2     = 31  ## read motor 2 speed (pulses/125th sec)
CmdDriveDutyMot1        = 32  ## drive motor 1 at duty cycle (no quad.)
CmdDriveDutyMot2        = 33  ## drive motor 2 at duty cycle (no quad.)
CmdDriveDuty            = 34  ## drive motors at duty cycle (no quad.)
CmdDriveQMot1           = 35  ## drive motor 1 at quad. pulses/second
CmdDriveQMot2           = 36  ## drive motor 2 at quad. pulses/second
CmdDriveQ               = 37  ## drive motors at quad. pulses/second
CmdDriveQAccelMot1      = 38  ## drive motor 1 at quad. pps with accel.
CmdDriveQAccelMot2      = 39  ## drive motor 2 at quad. pps with accel.
CmdDriveQAccel          = 40  ## drive motors at quad. pps with accel.
CmdBufDriveQDistMot1    = 41  ## buffered drive motor 1 to dist at qpps 
CmdBufDriveQDistMot2    = 42  ## buffered drive motor 2 to dist at qpps
CmdBufDriveQDist        = 43  ## buffered drive motors to dist at qpps
CmdBufDriveQAccelDistMot1 = 44 
                   ## buffered drive motor 1 to dist at qpps with accel.
CmdBufDriveQAccelDistMot2 = 45 
                   ## buffered drive motor 2 to dist at qpps with accel.
CmdBufDriveQAccelDist   = 46 
                   ## buffered drive motors to dist at qpps with accel.
CmdReadBufLen           = 47  ## read motors bufferd command length

# more here...

CmdReadMotorDraw        = 49  ## read motors amp draw
CmdDriveQAccel2         = 50  ## drive motros at qpps with 2 accel vals
CmdBufDriveQAccel2Dist  = 51 
                            ## buffered drive motors to dist at qpps w/ 2 accel
# more here...

CmdReadVelPidMot1       = 55  ## read motor 1 velocity PID constants
CmdReadVelPidMot2       = 56  ## read motor 2 velocity PID constants
CmdSetMainBattCutoffs   = 57  ## set main battery voltage cutoffs
CmdSetLogicCutoffs      = 58  ## set logic voltage cutoffs
CmdReadMainBattCutoffs  = 59  ## read main battery cutoff settings
CmdReadLogicCutoffs     = 60  ## read logic voltage cutoff settings
CmdSetPosPidMot1        = 61  ## set motor 1 position PID constants
CmdSetPosPidMot2        = 62  ## set motor 2 position PID constants
CmdReadPosPidMot1       = 63  ## read motor 1 position PID constants
CmdReadPosPidMot2       = 64  ## read motor 2 position PID constants
CmdBufDriveQProfPosMot1 = 65 
                  ## drive motor 1 with signed qpps, accel, deccel and position
CmdBufDriveQProfPosMot2 = 66 
                  ## drive motor 2 with signed qpps, accel, deccel and position
CmdBufDriveQProfPos     = 67 
                  ## drive motors with signed qpps, accel, deccel and position

# more here...

CmdReadTemp             = 82  ## read board temperature
CmdReadTemp2            = 83  ## read board second temperature
CmdReadStatus           = 90  ## read current board status
CmdReadEncoderMode      = 91  ## read encoder mode for both motors
CmdSetEncoderModeMot1   = 92  ## set motor 1 encoder mode
CmdSetEncoderModeMot2   = 93  ## set motor 2 encoder mode
CmdWriteEEPROM          = 94  ## write settings to EEPROM

# more here...

# Note: RoboClass User Manual v5 error. The Cmd*MaxCurrentMotn command numbers
#       are shifted down by 1.
CmdSetMaxCurrentMot1    = 133 ## set motor 1 maximum current limit
CmdSetMaxCurrentMot2    = 134 ## set motor 2 maximum current limit
CmdReadMaxCurrentMot1   = 135 ## read motor 1 maximum current limit
CmdReadMaxCurrentMot2   = 136 ## read motor 2 maximum current limit


# . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
# RoboClaw Packet Checksum - Deprecated
# . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
CheckSumMask = 0x7f  ## checksum 7-bit mask
AckReqBit    = 0x80  ## request ack to write commands
 
# . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
# RoboClaw Packet CRC
# . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

# . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
# RoboClaw Responses
# . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
RspAck = 0xff   ## ack response to write commands

# . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
# RoboClaw Message Parameters
# . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

ParamVelPidQppsDft  = 44000       ## default qpps max speed
ParamVelPidPDft     = 0x00010000  ## default vel proportional const
ParamVelPidIDft     = 0x00008000  ## default vel integrative const
ParamVelPidDDft     = 0x00004000  ## default vel derivative const
ParamVelPidCvt      = 0x00010000  ## vel PID conversion

ParamAmpScale       =  0.01       ## value * s = amps
ParamAmpMin         =  0.0        ## minimum amps
ParamAmpMax         = 15.0        ## maximum amps

Duplicate interface end
"""


# ------------------------------------------------------------------------------
# RoboClawException Class
# ------------------------------------------------------------------------------

##
## \brief RoboClaw exception class.
##
class RoboClawException(Exception):
  ##
  ## \brief Constructor.
  ##
  ## \param msg   Error message string.
  ##
  def __init__(self, msg):
    ## error message attribute
    self.message = 'Roboclaw: ' + msg

  def __repr__(self):
    return "RoboClawException(%s)" % (repr(self.message))

  def __str__(self):
    return self.message


#-------------------------------------------------------------------------------
# RoboClaw Class
#-------------------------------------------------------------------------------

##
## \brief RobotClaw Motor Controller class.
##
class RoboClaw:

  #
  ## \brief Constructor.
  #
  def __init__(self):
    self.m_checksum     = 0           ## checksum (deprecated)
    self.m_port         = None        ## serial port
    self.m_addrLast     = 0           ## last address
    self.m_fnChipSelect = self.noop   ## board chip select
    self.m_isOpen       = False       ## port is [not] open

  #
  ## \brief Open connection to motor controller(s).
  ##
  ## \param device        Serial device name.
  ## \param baudrate      Serial baud rate.
  ## \param fnChipSelect  Motor controller selection function.
  #
  def open(self, device, baudrate, fnChipSelect=None):
    try:
      self.m_port = serial.Serial(device, baudrate=baudrate, timeout=0.5)
    except serial.SerialException as inst:
      raise RoboClawException(inst.message)
    self.m_port.flushInput()
    self.m_port.flushOutput()
    self.m_checksum = 0
    self.m_addrLast = 0
    if fnChipSelect is None:
      self.m_fnChipSelect = self.noop
    else:
      self.m_fnChipSelect = fnChipSelect
    self.m_isOpen = True

  def close(self):
    self.m_port.close()
    self.m_isOpen = False

  def isOpen(self):
    return self.m_isOpen

  def noop(self, port, addrSel, addrLast):
    pass


  # . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  # Firmware Versions >= 4.1.11
  # . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  #
  ## \brief Execute command with acknowledgement response.
  ##
  ## Command:   addr cmdId [data] [crc16]
  ## Response:  ack
  ##
  #
  def execCmdWithAckRsp(self, addr, cmdId, sCmdData, appendCrc = False):
    sCmd = chr(addr) + chr(cmdId) + sCmdData
    crcCmd = self.crc16(0, sCmd)
    if appendCrc:
      sCmd += self.packU16(crcCmd)
    try:
      self.m_port.write(sCmd)
    except (serial.SerialException, serial.SerialTimeoutException) as inst:
      raise RoboClawException(inst.message)
    try:
      sRsp = self.m_port.read(1)
    except (serial.SerialException, serial.SerialTimeoutException) as inst:
      raise RoboClawException(inst.message)
    if len(sRsp) < 1:
        raise RoboClawException("Command %u: Received no ack" % (cmdId))
    else:
      ack = self.unpackU8(sRsp)
      if ack != RspAck:
        raise RoboClawException("Command %u: Received bad ack 0x%02x" % \
            (cmdId, ack))

  #
  ## \brief Execute command with data response.
  ##
  ## Command:   addr cmdId [data...] [crc16]
  ## Response:  data... crc16
  ##
  #
  def execCmdWithDataRsp(self, addr, cmdId, sCmdData, lenRsp, appendCrc=False):
    sCmd = chr(addr) + chr(cmdId) + sCmdData
    crcCmd = self.crc16(0, sCmd)
    if appendCrc:
      sCmd += packU16(crcCmd)
    try:
      self.m_port.write(sCmd)
    except (serial.SerialException, serial.SerialTimeoutException) as inst:
      raise RoboClawException(inst.message)
    try:
      sRsp = self.m_port.read(lenRsp)
    except (serial.SerialException, serial.SerialTimeoutException) as inst:
      raise RoboClawException(inst.message)
    if len(sRsp) != lenRsp:
      raise RoboClawException("Command %u: Partial response: " \
        "Expected %d bytes, received %d bytes" % (cmdId, lenRsp, len(sRsp)))
    if len(sRsp) >= 2:
      crcRsp = self.unpackU16(sRsp[-2:])
      sRsp = sRsp[:-2]
    else:
      crcRsp = 0
    crcCalc = self.crc16(crcCmd, sRsp)
    if crcRsp != crcCalc:
      raise RoboClawException("Command %u: CRC mismatch: " \
        "Expected 0x%04x, received 0x%04x" % (cmdId, crcCalc, crcRsp))
    return sRsp

  def packU8(self, val):
    return struct.pack('>B', val)

  def packS8(self, val):
    return struct.pack('>b',val)

  def packU16(self, val):
    return struct.pack('>H',val)

  def packS16(self, val):
    return struct.pack('>h',val)

  def packU32(self, val):
    return struct.pack('>L',val)

  def packS32(self, val):
    return struct.pack('>l',val)

  def unpackU8(self, s):
    return struct.unpack('>B', s[:1])[0]

  def unpackS8(self, s):
    return struct.unpack('>b', s[:1])[0]

  def unpackU16(self, s):
    return struct.unpack('>H', s[:2])[0]

  def unpackS16(self, s):
    return struct.unpack('>h', s[:2])[0]

  def unpackU32(self, s):
    return struct.unpack('>L', s[:4])[0]

  def unpackS32(self, s):
    return struct.unpack('>l', s[:4])[0]

  def crc16(self, crc, buf):
    for b in buf:
      crc = crc ^ (ord(b) << 8);
      for bit in range(0,8):
        if crc & 0x8000:
          crc = (crc << 1) ^ 0x1021;
        else:
          crc = crc << 1;
    return crc & 0xffff;


  # . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  # Firmware Versions < 4.1.11 Deprecated
  # . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  def sendCmdHdr(self, addr, command):
    raise RoboClaw("sendCmdHdr: Deprecated");
    self.m_fnChipSelect(self.m_port, addr, self.m_addrLast)
    self.m_checksum  = addr
    self.m_checksum += command
    try:
      self.m_port.write(chr(addr))
      self.m_port.write(chr(command))
    except (serial.SerialException, serial.SerialTimeoutException) as inst:
      raise RoboClawException(inst.message)
    except struct.error as inst:
      raise RoboClawException(inst.message)
    self.m_addrLast = addr

  def readU8(self):
    try:
      val = struct.unpack('>B',self.m_port.read(1))
    except (serial.SerialException, serial.SerialTimeoutException) as inst:
      raise RoboClawException(inst.message)
    except struct.error as inst:
      raise RoboClawException(inst.message)
    self.m_checksum += val[0]
    return val[0]

  def readS8(self):
    try:
      val = struct.unpack('>b',self.m_port.read(1))
    except (serial.SerialException, serial.SerialTimeoutException) as inst:
      raise RoboClawException(inst.message)
    except struct.error as inst:
      raise RoboClawException(inst.message)
    self.m_checksum += val[0]
    return val[0]

  def readU16(self):
    try:
      val = struct.unpack('>H',self.m_port.read(2))
    except (serial.SerialException, serial.SerialTimeoutException) as inst:
      raise RoboClawException(inst.message)
    except struct.error as inst:
      raise RoboClawException(inst.message)
    self.m_checksum += (val[0]&0xFF)
    self.m_checksum += (val[0]>>8)&0xFF
    return val[0]

  def readS16(self):
    try:
      val = struct.unpack('>h',self.m_port.read(2))
    except (serial.SerialException, serial.SerialTimeoutException) as inst:
      raise RoboClawException(inst.message)
    except struct.error as inst:
      raise RoboClawException(inst.message)
    self.m_checksum += val[0]
    self.m_checksum += (val[0]>>8)&0xFF
    return val[0]

  def readU32(self):
    try:
      val = struct.unpack('>L',self.m_port.read(4))
    except (serial.SerialException, serial.SerialTimeoutException) as inst:
      raise RoboClawException(inst.message)
    except struct.error as inst:
      raise RoboClawException(inst.message)
    self.m_checksum += val[0]
    self.m_checksum += (val[0]>>8)&0xFF
    self.m_checksum += (val[0]>>16)&0xFF
    self.m_checksum += (val[0]>>24)&0xFF
    return val[0]

  def readS32(self):
    try:
      val = struct.unpack('>l',self.m_port.read(4))
    except (serial.SerialException, serial.SerialTimeoutException) as inst:
      raise RoboClawException(inst.message)
    except struct.error as inst:
      raise RoboClawException(inst.message)
    self.m_checksum += val[0]
    self.m_checksum += (val[0]>>8)&0xFF
    self.m_checksum += (val[0]>>16)&0xFF
    self.m_checksum += (val[0]>>24)&0xFF
    return val[0]

  def writeU8(self, val):
    self.m_checksum += val
    try:
      return self.m_port.write(struct.pack('>B',val))
    except (serial.SerialException, serial.SerialTimeoutException) as inst:
      raise RoboClawException(inst.message)
    except struct.error as inst:
      raise RoboClawException(inst.message)

  def writeS8(self, val):
    self.m_checksum += val
    try:
      return self.m_port.write(struct.pack('>b',val))
    except (serial.SerialException, serial.SerialTimeoutException) as inst:
      raise RoboClawException(inst.message)
    except struct.error as inst:
      raise RoboClawException(inst.message)

  def writeU16(self, val):
    self.m_checksum += val
    self.m_checksum += (val>>8)&0xFF
    try:
      return self.m_port.write(struct.pack('>H',val))
    except (serial.SerialException, serial.SerialTimeoutException) as inst:
      raise RoboClawException(inst.message)
    except struct.error as inst:
      raise RoboClawException(inst.message)

  def writeS16(self, val):
    self.m_checksum += val
    self.m_checksum += (val>>8)&0xFF
    try:
      return self.m_port.write(struct.pack('>h',val))
    except (serial.SerialException, serial.SerialTimeoutException) as inst:
      raise RoboClawException(inst.message)
    except struct.error as inst:
      raise RoboClawException(inst.message)

  def writeU32(self, val):
    self.m_checksum += val
    self.m_checksum += (val>>8)&0xFF
    self.m_checksum += (val>>16)&0xFF
    self.m_checksum += (val>>24)&0xFF
    try:
      return self.m_port.write(struct.pack('>L',val))
    except (serial.SerialException, serial.SerialTimeoutException) as inst:
      raise RoboClawException(inst.message)
    except struct.error as inst:
      raise RoboClawException(inst.message)

  def writeS32(self, val):
    self.m_checksum += val
    self.m_checksum += (val>>8)&0xFF
    self.m_checksum += (val>>16)&0xFF
    self.m_checksum += (val>>24)&0xFF
    try:
      return self.m_port.write(struct.pack('>l',val))
    except (serial.SerialException, serial.SerialTimeoutException) as inst:
      raise RoboClawException(inst.message)
    except struct.error as inst:
      raise RoboClawException(inst.message)


  # . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
  # Firmware Versions >= 4.1.11
  # . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

  ## Command: 0
  def M1Forward(self, addr, val):
    self.execCmdWithAckRsp(addr, CmdDriveForwardMot1, self.packU8(val), True)
  
  ## Command: 1
  def M1Backward(self, addr, val):
    self.execCmdWithAckRsp(addr, CmdDriveBackwardMot1, self.packU8(val), True)
  
  ## Command: 2
  def setMinMainBattery(self, addr, val):
    self.execCmdWithAckRsp(addr, CmdSetMinMainVolt, self.packU8(val), True)
  
  ## Command: 3
  def setMaxMainBattery(self, addr, val):
    self.execCmdWithAckRsp(addr, CmdSetMaxMainVolt, self.packU8(val), True)
  
  ## Command: 4
  def M2Forward(self, addr, val):
    self.execCmdWithAckRsp(addr, CmdDriveForwardMot2, self.packU8(val), True)
  
  ## Command: 5
  def M2Backward(self, addr, val):
    self.execCmdWithAckRsp(addr, CmdDriveBackwardMot2, self.packU8(val), True)
  
  ## Command: 6
  def DriveM1(self, addr, val):
    self.execCmdWithAckRsp(addr, CmdDriveMot1, self.packU8(val), True)
  
  ## Command: 7
  def DriveM2(self, addr, val):
    self.execCmdWithAckRsp(addr, CmdDriveMot2, self.packU8(val), True)
  
  ## Command: 8
  def ForwardMixed(self, addr, val):
    self.execCmdWithAckRsp(addr, CmdMixDriveForward, self.packU8(val), True)
  
  ## Command: 9
  def BackwardMixed(self, addr, val):
    self.execCmdWithAckRsp(addr, CmdMixDriveBackward, self.packU8(val), True)
  
  ## Command: 10
  def RightMixed(self, addr, val):
    self.execCmdWithAckRsp(addr, CmdMixTurnRight, self.packU8(val), True)
  
  ## Command: 11
  def LeftMixed(self, addr, val):
    self.execCmdWithAckRsp(addr, CmdMixTurnLeft, self.packU8(val), True)
  
  ## Command: 12
  def DriveMixed(self, addr, val):
    self.execCmdWithAckRsp(addr, CmdMixDrive, self.packU8(val), True)
  
  ## Command: 13
  def TurnMixed(self, addr, val):
    self.execCmdWithAckRsp(addr, CmdMixTurn, self.packU8(val), True)
  
  ## Command: 16
  def readM1Encoder(self, addr):
    sRsp = self.execCmdWithDataRsp(addr, CmdReadEncoderMot1, '', 7)
    enc    = self.unpackU32(sRsp)
    status = self.unpackU8(sRsp[4:])
    return (enc,status)
  
  ## Command: 17
  def readM2Encoder(self, addr):
    sRsp = self.execCmdWithDataRsp(addr, CmdReadEncoderMot2, '', 7)
    enc    = self.unpackU32(sRsp)
    status = self.unpackU8(sRsp[4:])
    return (enc,status)
  
  ## Command: 18
  def readM1Speed(self, addr):
    sRsp = self.execCmdWithDataRsp(addr, CmdReadSpeedMot1, '', 7)
    speed  = self.unpackU32(sRsp)
    status = self.unpackU8(sRsp[4:])
    return (speed, status)
  
  ## Command: 19
  def readM2Speed(self, addr):
    sRsp = self.execCmdWithDataRsp(addr, CmdReadSpeedMot2, '', 7)
    speed  = self.unpackU32(sRsp)
    status = self.unpackU8(sRsp[4:])
    return (speed, status)
  
  ## Command: 20
  def resetEncoderCnts(self, addr):
    self.execCmdWithAckRsp(addr, CmdResetEncoderCntrs, '', True)
  
  ## Command: 21
  def readVersion(self, addr):
    sCmd = chr(addr) + chr(CmdReadFwVersion)
    crcCmd = self.crc16(0, sCmd)
    try:
      self.m_port.write(sCmd)
    except (serial.SerialException, serial.SerialTimeoutException) as inst:
      raise RoboClawException(inst.message)
    try:
      sRsp = self.m_port.read(48)   # max bytes, can be less
    except serial.SerialException as inst:
      raise RoboClawException(inst.message)
    except serial.SerialTimeoutException:
      pass
    if len(sRsp) >= 2:
      crcRsp = self.unpackU16(sRsp[-2:])
      sRsp = sRsp[:-2]
    else:
      crcRsp = 0
    crcCalc = self.crc16(crcCmd, sRsp)
    if crcRsp != crcCalc:
      raise RoboClawException("Command %u: CRC mismatch: " \
        "Expected 0x%04x, received 0x%04x" % (sCmd[1], crcCalc, crcRsp))
    else:
      sVer = sRsp[:-2]    # strip ending NL+0
    return sVer
  
  ## Command: 24
  def readMainBattery(self, addr):
    sRsp = self.execCmdWithDataRsp(addr, CmdReadMainBattVolt, '', 4)
    return self.unpackU16(sRsp);
  
  ## Command: 25
  def readLogicBattery(self, addr):
    sRsp = self.execCmdWithDataRsp(addr, CmdReadLogicVolt, '', 4)
    return self.unpackU16(sRsp);
  
  ## Command: 28
  def setM1Pidq(self, addr, p, i, d, qpps):
    sData = ''
    sData += self.packU32(d)
    sData += self.packU32(p)
    sData += self.packU32(i)
    sData += self.packU32(qpps)
    self.execCmdWithAckRsp(addr, CmdSetVelPidMot1, sData, True)
  
  ## Command: 29
  def setM2Pidq(self, addr, p, i, d, qpps):
    sData = ''
    sData += self.packU32(d)
    sData += self.packU32(p)
    sData += self.packU32(i)
    sData += self.packU32(qpps)
    self.execCmdWithAckRsp(addr, CmdSetVelPidMot2, sData, True)
  
  ## Command: 30
  def readM1instspeed(self, addr):
    sRsp = self.execCmdWithDataRsp(addr, CmdRead125SpeedMot1, '', 7)
    speed   = self.unpackU32(sRsp)
    status  = self.unpackU8(sRsp[4:])
    return (speed,status)
  
  ## Command: 31
  def readM2instspeed(self, addr):
    sRsp = self.execCmdWithDataRsp(addr, CmdRead125SpeedMot2, '', 7)
    speed   = self.unpackU32(sRsp)
    status  = self.unpackU8(sRsp[4:])
    return (speed,status)
  
  ## Command: 32
  def setM1Duty(self, addr, val):
    self.execCmdWithAckRsp(addr, CmdDriveDutyMot1, self.packS16(val), True)
  
  ## Command: 33
  def setM2Duty(self, addr, val):
    self.execCmdWithAckRsp(addr, CmdDriveDutyMot2, self.packS16(val), True)
  
  ## Command: 34
  def setMixedDuty(self, addr, m1, m2):
    sData = ''
    sData += self.packS16(m1)
    sData += self.packS16(m2)
    self.execCmdWithAckRsp(addr, CmdDriveDuty, sData, True)
  
  ## Command: 35
  def setM1Speed(self, addr, val):
    self.execCmdWithAckRsp(addr, CmdDriveQMot1, self.packS32(val), True)
  
  ## Command: 36
  def setM2Speed(self, addr, val):
    self.execCmdWithAckRsp(addr, CmdDriveQMot2, self.packS32(val), True)
  
  ## Command: 37
  def setMixedSpeed(self, addr, m1, m2):
    sData = ''
    sData += self.packS32(m1)
    sData += self.packS32(m2)
    self.execCmdWithAckRsp(addr, CmdDriveQ, sData, True)
  
  ## Command: 38
  def setM1SpeedAccel(self, addr, accel,speed):
    sData = ''
    sData += self.packU32(accel)
    sData += self.packS32(speed)
    self.execCmdWithAckRsp(addr, CmdDriveQAccelMot1, sData, True)
  
  ## Command: 39
  def setM2SpeedAccel(self, addr, accel, speed):
    sData = ''
    sData += self.packU32(accel)
    sData += self.packS32(speed)
    self.execCmdWithAckRsp(addr, CmdDriveQAccelMot2, sData, True)
  
  ## Command: 40
  def setMixedSpeedAccel(self, addr, accel, speed1, speed2):
    sData = ''
    sData += self.packU32(accel)
    sData += self.packS32(speed1)
    sData += self.packS32(speed2)
    self.execCmdWithAckRsp(addr, CmdDriveQAccel, sData, True)
  
  ## Command: 41
  def setM1SpeedDistance(self, addr, speed, distance, buffered):
    sData = ''
    sData += self.packS32(speed)
    sData += self.packU32(distance)
    sData += self.packU8(buffered)
    self.execCmdWithAckRsp(addr, CmdBufDriveQDistMot1, sData, True)
  
  ## Command: 42
  def setM2SpeedDistance(self, addr, speed, distance, buffered):
    sData = ''
    sData += self.packS32(speed)
    sData += self.packU32(distance)
    sData += self.packU8(buffered)
    self.execCmdWithAckRsp(addr, CmdBufDriveQDistMot2, sData, True)
  
  ## Command: 43
  def setMixedSpeedDistance(self, addr, speed1, distance1, speed2, distance2,
                                  buffered):
    sData = ''
    sData += self.packS32(speed1)
    sData += self.packU32(distance1)
    sData += self.packS32(speed2)
    sData += self.packU32(distance2)
    sData += self.packU8(buffered)
    self.execCmdWithAckRsp(addr, CmdBufDriveQDist, sData, True)
  
  ## Command: 44
  def setM1SpeedAccelDistance(self, addr, accel, speed, distance, buffered):
    sData = ''
    sData += self.packU32(accel)
    sData += self.packS32(speed)
    sData += self.packU32(distance)
    sData += self.packU8(buffered)
    self.execCmdWithAckRsp(addr, CmdBufDriveQAccelDistMot1, sData, True)
  
  ## Command: 45
  def setM2SpeedAccelDistance(self, addr, accel, speed, distance, buffered):
    sData = ''
    sData += self.packU32(accel)
    sData += self.packS32(speed)
    sData += self.packU32(distance)
    sData += self.packU8(buffered)
    self.execCmdWithAckRsp(addr, CmdBufDriveQAccelDistMot2, sData, True)
  
  ## Command: 46
  def setMixedSpeedAccelDistance(self, addr, accel, speed1, distance1,
                                                    speed2, distance2,
                                                    buffered):
    sData = ''
    sData += self.packU32(accel)
    sData += self.packS32(speed1)
    sData += self.packU32(distance1)
    sData += self.packS32(speed2)
    sData += self.packU32(distance2)
    sData += self.packU8(buffered)
    self.execCmdWithAckRsp(addr, CmdBufDriveQAccelDist, sData, True)
  
  ## Command: 47
  def readBufferCnts(self, addr):
    sRsp = self.execCmdWithDataRsp(addr, CmdReadBufLen, '', 4)
    buffer1 = self.unpackU8(sRsp)
    buffer2 = self.unpackU8(sRsp[1:])
    return (buffer1,buffer2)
  
  ## Command: 49
  ## Note:  RoboClaw User Manual v5 error. Values are signed 16 bits.
  def readCurrents(self, addr):
    sRsp = self.execCmdWithDataRsp(addr, CmdReadMotorDraw, '', 6)
    amp1 = self.unpackU16(sRsp)
    amp2 = self.unpackU16(sRsp[2:])
    if amp1 & 0x8000:
      amp1 -= 0x10000
    if amp2 & 0x8000:
      amp2 -= 0x10000
    return (amp1,amp2)
  
  ## Command: 50
  def setMixedSpeedIAccel(self, addr, accel1, speed1, accel2, speed2):
    sData = ''
    sData += self.packU32(accel1)
    sData += self.packS32(speed1)
    sData += self.packU32(accel2)
    sData += self.packS32(speed2)
    self.execCmdWithAckRsp(addr, CmdDriveQAccel2, sData, True)
  
  ## Command: 51
  def setMixedSpeedIAccelDistance(self, addr, accel1, speed1, distance1,
                                              accel2, speed2, distance2,
                                              buffered):
    sData = ''
    sData += self.packU32(accel1)
    sData += self.packS32(speed1)
    sData += self.packU32(distance1)
    sData += self.packU32(accel2)
    sData += self.packS32(speed2)
    sData += self.packU32(distance2)
    sData += self.packU8(buffered)
    self.execCmdWithAckRsp(addr, CmdBufDriveQAccel2Dist, sData, True)
  
  ## Command: 52
  def setM1DutyAccel(self, addr, accel, duty):
    sData = ''
    sData += self.packS16(duty)
    sData += self.packU16(accel)
    self.execCmdWithAckRsp(addr, 52, sData, True)
  
  ## Command: 53
  def setM2DutyAccel(self, addr, accel, duty):
    sData = ''
    sData += self.packS16(duty)
    sData += self.packU16(accel)
    self.execCmdWithAckRsp(addr, 53, sData, True)
  
  ## Command: 54
  def setMixedDutyAccel(self, addr, accel1, duty1, accel2, duty2):
    sData = ''
    sData += self.packS16(duty1)
    sData += self.packU16(accel1)
    sData += self.packS16(duty2)
    sData += self.packU16(accel2)
    self.execCmdWithAckRsp(addr, 54, sData, True)
  
  ## Command: 55
  def readM1Pidq(self, addr):
    sRsp = self.execCmdWithDataRsp(addr, CmdReadVelPidMot1, '', 18)
    p     = self.unpackU32(sRsp)
    i     = self.unpackU32(sRsp[4:])
    d     = self.unpackU32(sRsp[8:])
    qpps  = self.unpackU32(sRsp[12:])
    return (p,i,d,qpps)
  
  ## Command: 56
  def readM2Pidq(self, addr):
    sRsp = self.execCmdWithDataRsp(addr, CmdReadVelPidMot2, '', 18)
    p     = self.unpackU32(sRsp)
    i     = self.unpackU32(sRsp[4:])
    d     = self.unpackU32(sRsp[8:])
    qpps  = self.unpackU32(sRsp[12:])
    return (p,i,d,qpps)
  
  ## Command: 57
  def setMainBatterySettings(self, addr, minV, maxV):
    sData = ''
    sData += self.packU16(minV)
    sData += self.packU16(maxV)
    self.execCmdWithAckRsp(addr, CmdSetMainBattCutoffs, sData, True)
  
  ## Command: 58
  def setLogicBatterySettings(self, addr, minV, maxV):
    sData = ''
    sData += self.packU16(minV)
    sData += self.packU16(maxV)
    self.execCmdWithAckRsp(addr, CmdSetLogicCutoffs, sData, True)
  
  ## Command: 59
  def readMainBatterySettings(self, addr):
    sRsp = self.execCmdWithDataRsp(addr, CmdReadMainBattCutoffs, '', 6)
    minV = self.unpackU16(sRsp)
    maxV = self.unpackU16(sRsp[2:])
    return (minV,maxV)
  
  ## Command: 60
  def readLogicBatterySettings(self, addr):
    sRsp = self.execCmdWithDataRsp(addr, CmdReadLocicCutoffs, '', 6)
    minV = self.unpackU16(sRsp)
    maxV = self.unpackU16(sRsp[2:])
    return (minV,maxV)
  
  ## Command: 61
  def setM1PositionConstants(self, addr, kp, ki, kd, kimax,
                                          deadzone, minP, maxP):
    sData = ''
    self.writeU32(kd)
    self.writeU32(kp)
    self.writeU32(ki)
    self.writeU32(kimax)
    self.writeU32(deadzone)
    self.writeU32(minP)
    self.writeU32(maxP)
    self.execCmdWithAckRsp(addr, CmdSetPosPidMot1, sData, True)
  
  ## Command: 62
  def setM2PositionConstants(self, addr, kp, ki, kd, kimax,
                                          deadzone, minP, maxP):
    sData = ''
    self.writeU32(kd)
    self.writeU32(kp)
    self.writeU32(ki)
    self.writeU32(kimax)
    self.writeU32(deadzone)
    self.writeU32(minP)
    self.writeU32(maxP)
    self.execCmdWithAckRsp(addr, CmdSetPosPidMot2, sData, True)
  
  ## Command: 63
  def readM1PositionConstants(self, addr):
    sRsp = self.execCmdWithDataRsp(addr, CmdReadPosPidMot1, '', 30)
    p         = self.unpackU32(sRsp)
    i         = self.unpackU32(sRsp[4:])
    d         = self.unpackU32(sRsp[8:])
    imax      = self.unpackU32(sRsp[12:])
    deadzone  = self.unpackU32(sRsp[16:])
    minP      = self.unpackU32(sRsp[20:])
    maxP      = self.unpackU32(sRsp[24:])
    return (p,i,d,imax,deadzone,minP,maxP)
  
  ## Command: 64
  def readM2PositionConstants(self, addr):
    sRsp = self.execCmdWithDataRsp(addr, CmdReadPosPidMot2, '', 30)
    p         = self.unpackU32(sRsp)
    i         = self.unpackU32(sRsp[4:])
    d         = self.unpackU32(sRsp[8:])
    imax      = self.unpackU32(sRsp[12:])
    deadzone  = self.unpackU32(sRsp[16:])
    minP      = self.unpackU32(sRsp[20:])
    maxP      = self.unpackU32(sRsp[24:])
    return (p,i,d,imax,deadzone,minP,maxP)
  
  ## Command: 65
  def setM1SpeedAccelDeccelPosition(self, addr, accel, speed, deccel, position,
                                          buffered):
    sData = ''
    sData += self.packU32(accel)
    sData += self.packU32(speed)
    sData += self.packU32(deccel)
    sData += self.packU32(position)
    sData += self.packU8(buffered)
    self.execCmdWithAckRsp(addr, CmdBufDriveQProfPosMot1, sData, True)
  
  ## Command: 66
  def setM2SpeedAccelDeccelPosition(self, addr, accel, speed, deccel, position,
                                          buffered):
    sData = ''
    sData += self.packU32(accel)
    sData += self.packU32(speed)
    sData += self.packU32(deccel)
    sData += self.packU32(position)
    sData += self.packU8(buffered)
    self.execCmdWithAckRsp(addr, CmdBufDriveQProfPosMot2, sData, True)
  
  ## Command: 67
  def setMixedSpeedAccelDeccelPosition(self, addr,
                                        accel1, speed1, deccel1, position1,
                                        accel2, speed2, deccel2, position2,
                                        buffered):
    sData = ''
    sData += self.packU32(accel1)
    sData += self.packU32(speed1)
    sData += self.packU32(deccel1)
    sData += self.packU32(position1)
    sData += self.packU32(accel2)
    sData += self.packU32(speed2)
    sData += self.packU32(deccel2)
    sData += self.packU32(position2)
    sData += self.packU8(buffered)
    self.execCmdWithAckRsp(addr, CmdBufDriveQProfPos, sData, True)
  
  ## Command: 82
  def readTemperature(self, addr):
    sRsp = self.execCmdWithDataRsp(addr, CmdReadTemp, '', 4)
    return self.unpackU16(sRsp)
  
  ## Command: 90
  def readStatus(self, addr):
    sRsp = self.execCmdWithDataRsp(addr, CmdReadStatus, '', 4)
    return self.unpackU16(sRsp)

  ## Command: 91
  def readEncoderMode(self, addr):
    sRsp = self.execCmdWithDataRsp(addr, CmdReadEncoderMode, '', 4)
    mode1 = self.unpackU8(sRsp)
    mode2 = self.unpackU8(sRsp[1:])
    return (mode1, mode2)

  ## Command: 92
  def setM1EncoderMode(self, addr, mode):
    self.execCmdWithAckRsp(addr, CmdSetEncoderModeMot1, self.packU8(mode), True)

  ## Command: 93
  def setM2EncoderMode(self, addr, mode):
    self.execCmdWithAckRsp(addr, CmdSetEncoderModeMot2, self.packU8(mode), True)

  ## Command: 94
  def writeSettings(self, addr):
    self.execCmdWithAckRsp(addr, CmdWriteEEPROM, '')

  ## Command: 133
  ## Note:  RoboClass User Manual v5 error. Values are 32 bits.
  def setM1MaxCurrentLimit(self, addr, maxAmps):
    maxAmps = int(maxAmps)
    sData = ''
    sData += self.packU32(maxAmps)
    sData += self.packU32(0)
    self.execCmdWithAckRsp(addr, CmdSetMaxCurrentMot1, sData, True)

  ## Command: 134
  ## Note:  RoboClass User Manual v5 error. Values are 32 bits.
  def setM2MaxCurrentLimit(self, addr, maxAmps):
    maxAmps = int(maxAmps)
    sData = ''
    sData += self.packU32(maxAmps)
    sData += self.packU32(0)
    self.execCmdWithAckRsp(addr, CmdSetMaxCurrentMot2, sData, True)

  ## Command: 135
  ## Note:  RoboClass User Manual v5 error. Values are 32 bits.
  def readM1MaxCurrentLimit(self, addr):
    sRsp = self.execCmdWithDataRsp(addr, CmdReadMaxCurrentMot1, '', 10)
    return self.unpackU32(sRsp)

  ## Command: 136
  ## Note:  RoboClass User Manual v5 error. Values are 32 bits.
  def readM2MaxCurrentLimit(self, addr):
    sRsp = self.execCmdWithDataRsp(addr, CmdReadMaxCurrentMot2, '', 10)
    return self.unpackU32(sRsp)


# ------------------------------------------------------------------------------
# UT main
# ------------------------------------------------------------------------------
if __name__ == '__main__':
  print "Roboclaw Unit Test Example"
  print

  addr = AddrDft

  # Create motor controller object.
  motorctlr = RoboClaw()

  # Open communiction
  try:
    motorctlr.open("/dev/ttyACM0", 1000000)
  except RoboClawException as inst:
    print >>sys.stderr, "Error:", inst.message
    sys.exit(4)

  # Get version string
  rcv = motorctlr.readVersion(addr)
  print rcv[:-3]

  #
  # Print settings
  #
  minV, maxV = motorctlr.readLogicBatterySettings(addr)
  print "Logic Battery:    [%.1f, %.1f]" % (minV/10.0, maxV/10.0)

  minV, maxV = motorctlr.readMainBatterySettings(addr)
  print "Main Battery:     [%.1f, %.1f]" % (minV/10.0, maxV/10.0)

  mode1, mode2 = motorctlr.readEncoderMode(addr)
  print "Encoder Modes:    0x%02x, 0x%02x" % (mode1, mode2)

  motorctlr.setM1Pidq(addr, 0x00010000, 0x00008000, 0x00004000, 44000)
  motorctlr.setM2Pidq(addr, 0x00010000, 0x00008000, 0x00004000, 44000)

  p,i,d,qpps = motorctlr.readM1Pidq(addr)
  print "M1 Velocity PID:"
  print "  P    = 0x%08x" % (p / 65536.0)
  print "  I    = 0x%08x" % (i / 65536.0)
  print "  D    = 0x%08x" % (d / 65536.0)
  print "  QPPS = %u" % (qpps)

  p,i,d,qpps = motorctlr.readM2Pidq(addr)
  print "M2 Velocity PID:"
  print "  P    = 0x%08x" % (p / 65536.0)
  print "  I    = 0x%08x" % (i / 65536.0)
  print "  D    = 0x%08x" % (d / 65536.0)
  print "  QPPS = %u" % (qpps)

  print

  print "All stop"
  motorctlr.setM1Speed(addr, 0)
  motorctlr.setM2Speed(addr, 0)

  print "Reset encoders"
  motorctlr.resetEncoderCnts(addr)

  print

  cnt = 0
  maxcnt = 15
  tgtspeed1 = 0
  tgtspeed2 = 0

  while True:
    cnt=cnt+1
    print "Count = ",cnt
  
    print "  Status:        0x%04x" % (motorctlr.readStatus(addr))
    print "  Temperature:   %.1f" % (motorctlr.readTemperature(addr)/10.0)
    print "  Main Battery:  %.1f" % (motorctlr.readMainBattery(addr)/10.0)
    print "  Logic Battery: %.1f" % (motorctlr.readLogicBattery(addr)/10.0)

    m1cur, m2cur = motorctlr.readCurrents(addr)
    print "  Current M1:    %.1f" % (m1cur/100.0)
    print "  Current M2:    %.1f" % (m2cur/100.0)
  
    pos1, status1 = motorctlr.readM1Encoder(addr)
    pos2, status2 = motorctlr.readM2Encoder(addr)
    print "  Position M1:   %u, 0x%02x" % (pos1, status1)
    print "  Position M2:   %u, 0x%02x" % (pos2, status2)

    tgtspeed1 += 1000
    tgtspeed2 += 1000

    if tgtspeed1 > 20000 or tgtspeed2 > 20000:
      tgtspeed1 = 0
      tgtspeed2 = 0

    print "  Set Speeds:    %u, %u" % (tgtspeed1, tgtspeed2)
    motorctlr.setM1Speed(addr, tgtspeed1)
    motorctlr.setM2Speed(addr, tgtspeed2)

    speed1, status1 = motorctlr.readM1Speed(addr)
    speed2, status2 = motorctlr.readM2Speed(addr)
    print "  Speed M1:      %u, 0x%02x" % (speed1, status1)
    print "  Speed M2:      %u, 0x%02x" % (speed2, status2)

    time.sleep(2)

    if cnt > maxcnt:
      break

  print "Stop"
  motorctlr.setM1Speed(addr, 0)
  motorctlr.setM2Speed(addr, 0)
