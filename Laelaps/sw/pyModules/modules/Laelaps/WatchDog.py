###############################################################################
#
# Module:   Laelaps.WatchDog
#
# Package:  RoadNarrows Laelaps Robotic Mobile Platform Package
#
# Link:     https://github.com/roadnarrows-robotics/laelaps
#
# File:     WatchDog.py
#
## \file 
##
## $LastChangedDate: 2015-07-23 16:47:32 -0600 (Thu, 23 Jul 2015) $
## $Rev: 4038 $
##
## \brief Laelaps WatchDog subprocessor Interface.
##
## The main processors communicates with the subprocessor via a Two-Wire
## (I2C, SMBus) Interface.
##
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \par Copyright
##   \h_copy 2015-2017. RoadNarrows LLC.\n
##   http://www.roadnarrows.com\n
##   All Rights Reserved
##
# @EulaBegin@
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
# @EulaEnd@
#
###############################################################################

import sys
import os
import time

import rnr.i2c as i2c

import Laelaps.SysConf as SysConf
from Laelaps.WatchDogMsgs import *


# ------------------------------------------------------------------------------
# Global Data
# ------------------------------------------------------------------------------

# return codes
WDRcOk  = 'ok'      ## success
WDRcErr = 'error'   ## operation failed

# error messages
_WDErrMsgOpen = 'No open I2C bus device.'


# ------------------------------------------------------------------------------
# Class WatchDog
# ------------------------------------------------------------------------------

#
## \brief Laelaps WatchDog Interface Class.
#
class WatchDog:

  #
  ## \brief Constructor
  #
  def __init__(self):
    self.m_i2c    = None
    self.m_addr   = LaeI2CAddrArduino
    self.m_owner  = False

    # shadow data
    self.m_fw = {
        'version':            0,
        'battery_is_charging':False,
        'battery_soc':        LaeWdArgBattChargeMax,
        'alarms':             LaeWdArgAlarmNone,
        'rgb':                {'override': False, 'red':0, 'green':0, 'blue':0},
        'digital':            {},
        'analog':             {},
        'enables':            {'motor_ctlr_en': False,
                               'aux_port_batt_en': False,
                               'aux_port_5v_en': False},
        'voltages':           {'jack': 0.0, 'batt': 0.0} }

    for pin in range(LaeWdArgDPinNumMin, LaeWdArgDPinNumMax+1):
      self.m_fw['digital'][pin] = {'dir': LaeWdArgDPinDirIn, 'val': 0}

    for pin in range(LaeWdArgAInPinNumMin, LaeWdArgAInPinNumMax+1):
      self.m_fw['analog'][pin] = {'dir': LaeWdArgDPinDirIn, 'val': 0}
  
  #
  ## \brief Attach the open I2C object.
  ##
  ## \param i2cbus    An rnr.i2c.i2c() constructed and open object.
  #
  def attach(self, i2cbus):
    self.close()
    self.m_i2c    = i2cbus
    self.m_owner  = False
 
  #
  ## \brief Open the I2C bus device.
  ##
  ## \param devName   Device name.
  ##
  ## \return True on success, False on failure.
  #
  def open(self, devName):
    self.close()
    try:
      self.m_i2c = i2c.i2c()
      self.m_i2c.open(devName)
    except i2c.I2CException as inst:
      self.printError("open", inst.message)
      rc = False
    else:
      self.m_owner = True
      rc = True
    return rc
    
  #
  ## \brief Close the I2C bus device.
  ##
  ## The device must be owned by this class instance.
  #
  def close(self):
    if self.m_i2c is None:
      return
    elif not self.m_i2c.is_open():
      return
    elif not self.m_owner:
      return
    self.m_i2c.close()
    
  #
  ## \brief Test if the open/attached I2C bus device is open.
  ##
  ## \return True or False.
  #
  def isOpen(self):
    if self.m_i2c is None:
      return False
    else:
      return self.m_i2c.is_open()

  #
  ## \brief Synchronize watchdog state with the subprocessor state.
  #
  def sync(self):
    self.cmdGetFwVersion()            # must be first
    self.cmdEnableMotorCtlrs(False)
    self.cmdEnableAuxPort5V(True)
    self.cmdEnableAuxPortBatt(True)
    self.cmdSetBatterySoC(LaeWdArgBattChargeMax)
    self.cmdSetAlarms(LaeWdArgAlarmNone)
    self.cmdResetRgbLed()
    self.cmdPetTheDog()

  #
  ## \brief Get the shadowed subprocessor value.
  ##
  ## No I/O is performed.
  ##
  ## \return Dictionary of shadowed values.
  #
  def getShadow(self):
    return self.m_fw

  #
  ## \brief Execute command to pet the watchdog.
  ##
  ## \return {'rc': rc, key: value}\n
  #
  def cmdPetTheDog(self):
    cmdId = LaeWdCmdIdPetDog

    key = 'battery_is_charging'
    ret = {'rc': WDRcOk, key: self.m_fw[key]}

    # check connection
    if not self.isOpen():
      self.printError(_WDErrMsgOpen)
      ret['rc'] = WDRcErr
      return ret

    cmd   = [chr(cmdId)]

    # send command (and receive response)
    if self.m_fw['version'] <= 1:
      try:
        self.m_i2c.write(self.m_addr, cmd)
      except i2c.I2CException as inst:
        self.printError("CmdId %d" % (cmdId), inst.message)
        ret['rc'] = WDRcErr
    else: # versions 2+
      rspLen = LaeWdRspLenPetDog_2
      try:
        rsp = self.m_i2c.write_read(self.m_addr, cmd, rspLen)
      except i2c.I2CException as inst:
        self.printError("CmdId %d" % (cmdId), inst.message)
        ret['rc'] = WDRcErr
      else:
        if len(rsp) == rspLen:
          if rsp[0]:
            self.m_fw[key] = True
          else:
            self.m_fw[key] = False
          ret[key] = self.m_fw[key]

    return ret

  #
  ## \brief Execute command to get the firmware version.
  ##
  ## \return {'rc': rc, key: value}
  #
  def cmdGetFwVersion(self):
    cmdId   = LaeWdCmdIdGetVersion
    rspLen  = LaeWdRspLenGetVersion

    key = 'version'
    ret = {'rc': WDRcOk, key: self.m_fw[key]}

    # check connection
    if not self.isOpen():
      self.printError(_WDErrMsgOpen)
      ret['rc'] = WDRcErr
      return ret

    cmd = [chr(cmdId)]

    # send command and receive response
    try:
      rsp = self.m_i2c.write_read(self.m_addr, cmd, rspLen)
    except i2c.I2CException as inst:
      self.printError("CmdId %d" % (cmdId), inst.message)
      ret['rc'] = WDRcErr
    else:
      if len(rsp) == rspLen:
        self.m_fw[key] = ord(rsp[0])
        ret[key] = self.m_fw[key]

    return ret

  #
  ## \brief Execute command to set the battery state of charge.
  ##
  ## \param soc   State of charge [0-100].
  ##
  ## \return {'rc': rc, key: value}
  #
  def cmdSetBatterySoC(self, soc):
    cmdId = LaeWdCmdIdSetAlarms

    key = 'battery_soc'
    ret = {'rc': WDRcOk, key: self.m_fw[key]}

    # check connection
    if not self.isOpen():
      self.printError(_WDErrMsgOpen)
      ret['rc'] = WDRcErr
      return ret

    if soc < LaeWdArgBattChargeMin:
      soc = LaeWdArgBattChargeMin
    elif soc > LaeWdArgBattChargeMax:
      soc = LaeWdArgBattChargeMax
    else:
      soc = int(soc)

    cmd = [chr(cmdId), chr(soc)]

    # send command
    try:
      self.m_i2c.write(self.m_addr, cmd)
    except i2c.I2CException as inst:
      self.printError("CmdId %d" % (cmdId), inst.message)
      ret['rc'] = WDRcErr
    else:
      self.m_fw[key] = soc
      ret[key] = self.m_fw[key]

    return ret

  #
  ## \brief Execute command to set/clear alarms.
  ##
  ## \param alarms  Or'ed alarm bits.
  ##
  ## \return {'rc': rc, key: value}
  #
  def cmdSetAlarms(self, alarms):
    cmdId = LaeWdCmdIdSetAlarms

    key = 'alarms'
    ret = {'rc': WDRcOk, key: self.m_fw[key]}

    # check connection
    if not self.isOpen():
      self.printError(_WDErrMsgOpen)
      ret['rc'] = WDRcErr
      return ret

    alarms &= LaeWdArgAlarmMask

    b1  = chr(((alarms >> 8) & 0xff))
    b2  = chr(((alarms) & 0xff))
    cmd = [chr(cmdId), b1, b2]

    # send command
    try:
      self.m_i2c.write(self.m_addr, cmd)
    except i2c.I2CException as inst:
      self.printError("CmdId %d" % (cmdId), inst.message)
      ret['rc'] = WDRcErr
    else:
      self.m_fw[key] = alarms
      ret[key] = self.m_fw[key]

    return ret

  #
  ## \brief Execute command to set the RGB LED.
  ##
  ## \param red     Red component [0-255]
  ## \param green   Green component [0-255]
  ## \param blue    Blue component [0-255]
  ##
  ## \return {'rc': rc, key: value}
  #
  def cmdSetRgbLed(self, red, green, blue):
    cmdId = LaeWdCmdIdSetRgbLed

    key = 'rgb'
    ret = {'rc': WDRcOk, key: self.m_fw[key]}

    # check connection
    if not self.isOpen():
      self.printError(_WDErrMsgOpen)
      ret['rc'] = WDRcErr
      return ret

    red   &= 0xff
    green &= 0xff
    blue  &= 0xff

    b1  = chr(red)
    b2  = chr(green)
    b3  = chr(blue)
    cmd = [chr(cmdId), b1, b2, b3]

    # send command
    try:
      self.m_i2c.write(self.m_addr, cmd)
    except i2c.I2CException as inst:
      self.printError("CmdId %d" % (cmdId), inst.message)
      ret['rc'] = WDRcErr
    else:
      self.m_fw[key]['override']  = True
      self.m_fw[key]['red']       = red
      self.m_fw[key]['green']     = green
      self.m_fw[key]['blue']      = blue
      ret[key] = self.m_fw[key]

    return ret

  #
  ## \brief Reset the RGB LED to state defaults.
  ##
  ## \return {'rc': rc, key: value}
  #
  def cmdResetRgbLed(self):
    cmdId = LaeWdCmdIdResetRgbLed

    key = 'rgb'
    ret = {'rc': WDRcOk, key: self.m_fw[key]}

    # check connection
    if not self.isOpen():
      self.printError(_WDErrMsgOpen)
      ret['rc'] = WDRcErr
      return ret

    cmd   = [chr(cmdId)]

    # send command
    try:
      self.m_i2c.write(self.m_addr, cmd)
    except i2c.I2CException as inst:
      self.printError("CmdId %d" % (cmdId), inst.message)
      ret['rc'] = WDRcErr
    else:
      self.m_fw[key]['override'] = False
      ret[key] = self.m_fw[key]

    return ret

  #
  ## \brief Execute command to configure a digital I/O pin.
  ##
  ## \param pin         Digital pin number.
  ## \param direction   Pin direction. 0 == input, 1 == output.
  ##
  ## \return {'rc': rc, key: value}
  #
  def cmdConfigDPin(self, pin, direction):
    cmdId = LaeWdCmdIdConfigDPin

    key = 'digital'
    ret = {'rc': WDRcOk, key: {}}

    if (pin < LaeWdArgDPinNumWMin) or (pin > LaeWdArgDPinNumWMax):
      self.printError("CmdId %d" % (cmdId), "Pin %d invalid" % (pin))
      ret['rc'] = WDRcErr
      return ret
    
    ret[key] = self.m_fw[key][pin]
    
    # check connection
    if not self.isOpen():
      self.printError(_WDErrMsgOpen)
      ret['rc'] = WDRcErr
      return ret

    if direction > 0:
      direction = LaeWdArgDPinDirOut
    else:
      direction = LaeWdArgDPinDirIn

    b1  = chr(pin)
    b2  = chr(direction)
    cmd = [chr(cmdId), b1, b2]

    # send command
    try:
      self.m_i2c.write(self.m_addr, cmd)
    except i2c.I2CException as inst:
      self.printError("CmdId %d" % (cmdId), inst.message)
      ret['rc'] = WDRcErr
    else:
      self.m_fw[key][pin]['dir'] = direction
      ret[key] = self.m_fw[key][pin]

    return ret

  #
  ## \brief Execute command to read a digital I/O pin's value.
  ##
  ## \param pin         Digital pin number.
  ##
  ## \return {'rc': rc, key: value}
  #
  def cmdReadDPin(self, pin):
    cmdId  = LaeWdCmdIdReadDPin
    rspLen = LaeWdRspLenReadDPin

    key = 'digital'
    ret = {'rc': WDRcOk, key: {}}

    if (pin < LaeWdArgDPinNumMin) or (pin > LaeWdArgDPinNumMax):
      self.printError("CmdId %d" % (cmdId), "Pin %d invalid" % (pin))
      ret['rc'] = WDRcErr
      return ret
    
    ret[key] = self.m_fw[key][pin]
    
    # check connection
    if not self.isOpen():
      self.printError(_WDErrMsgOpen)
      ret['rc'] = WDRcErr
      return ret

    b1  = chr(pin)
    cmd = [chr(cmdId), b1]

    # send command
    try:
      rsp = self.m_i2c.write_read(self.m_addr, cmd, rspLen)
    except i2c.I2CException as inst:
      self.printError("CmdId %d" % (cmdId), inst.message)
      ret['rc'] = WDRcErr
    else:
      if len(rsp) == rspLen:
        self.m_fw[key][pin]['val'] = ord(rsp[1])
        ret[key] = self.m_fw[key][pin]

    return ret

  #
  ## \brief Execute command to wite a value to a digital I/O pin.
  ##
  ## \param pin     Digital pin number.
  ## \param value   Pin value. 0 == low, 1 == high.
  ##
  ## \return {'rc': rc, key: value}
  #
  def cmdWriteDPin(self, pin, val):
    cmdId = LaeWdCmdIdWriteDPin

    key = 'digital'
    ret = {'rc': WDRcOk, key: {}}

    if (pin < LaeWdArgDPinNumWMin) or (pin > LaeWdArgDPinNumWMax):
      self.printError("CmdId %d" % (cmdId), "Pin %d invalid" % (pin))
      ret['rc'] = WDRcErr
      return ret
    
    ret[key] = self.m_fw[key][pin]
    
    # check connection
    if not self.isOpen():
      self.printError(_WDErrMsgOpen)
      ret['rc'] = WDRcErr
      return ret

    if val > 0:
      val = LaeWdArgDPinValHigh
    else:
      val = LaeWdArgDPinValLow

    b1  = chr(pin)
    b2  = chr(val)
    cmd = [chr(cmdId), b1, b2]

    # send command
    try:
      self.m_i2c.write(self.m_addr, cmd)
    except i2c.I2CException as inst:
      self.printError("CmdId %d" % (cmdId), inst.message)
      ret['rc'] = WDRcErr
    else:
      self.m_fw[key][pin]['val'] = val
      ret[key] = self.m_fw[key][pin]

    return ret


  #
  ## \brief Execute command to read an analog I/O pin.
  ##
  ## \param pin   Analog pin number.
  ##
  ## \return {'rc': rc, key: value}
  #
  def cmdReadAPin(self, pin):
    cmdId  = LaeWdCmdIdReadAPin
    rspLen = LaeWdRspLenReadAPin

    key = 'analog'
    ret = {'rc': WDRcOk, key: {}}

    if (pin < LaeWdArgAInPinNumMin) or (pin > LaeWdArgAInPinNumMax):
      self.printError("CmdId %d" % (cmdId), "Pin %d invalid" % (pin))
      ret['rc'] = WDRcErr
      return ret
    
    ret[key] = self.m_fw[key][pin]
    
    # check connection
    if not self.isOpen():
      self.printError(_WDErrMsgOpen)
      ret['rc'] = WDRcErr
      return ret

    b1  = chr(pin)
    cmd = [chr(cmdId), b1]

    # send command
    try:
      rsp = self.m_i2c.write_read(self.m_addr, cmd, rspLen)
    except i2c.I2CException as inst:
      self.printError("CmdId %d" % (cmdId), inst.message)
      ret['rc'] = WDRcErr
    else:
      if len(rsp) == rspLen:
        b1 = ord(rsp[1])
        b2 = ord(rsp[2])
        val = b1 << 8 | b2
        self.m_fw[key][pin]['fal'] = val
        ret[key] = self.m_fw[key][pin]

    return ret

  #
  ## \brief Execute command to enable/disable power to motor controllers.
  ##
  ## \param enable      Enable (True) or disable (False).
  ##
  ## \return {'rc': rc, key: value}
  #
  def cmdEnableMotorCtlrs(self, enable):
    cmdId  = LaeWdCmdIdEnableMotorCtlrs
    rspLen = LaeWdRspLenEnableMotorCtlrs

    key     = 'enables'
    subkey  = 'motor_ctlr_en'
    ret     = {'rc': WDRcOk, subkey: self.m_fw[key][subkey]}

    # check connection
    if not self.isOpen():
      self.printError(_WDErrMsgOpen)
      ret['rc'] = WDRcErr
      return ret

    # unknown firmware
    if self.m_fw['version'] == 0:
      return ret
    # always enabled
    elif self.m_fw['version'] == 1:
      self.m_fw[key][subkey] = True
      ret[subkey] = self.m_fw[key][subkey]
      return ret

    if enable:
      val = LaeWdArgDPinValHigh
    else:
      val = LaeWdArgDPinValLow

    b1  = chr(val)
    cmd = [chr(cmdId), b1]

    # send command
    try:
      rsp = self.m_i2c.write_read(self.m_addr, cmd, rspLen)
    except i2c.I2CException as inst:
      self.printError("CmdId %d" % (cmdId), inst.message)
      ret['rc'] = WDRcErr
    else:
      if (len(rsp) == rspLen) and (ord(rsp[0]) == LaeWdArgPass):
        self.m_fw[key][subkey] = enable
        ret[subkey] = self.m_fw[key][subkey]

    return ret

  #
  ## \brief Execute command to enable/disable regulated 5V output to aux. port.
  ##
  ## \param enable      Enable (True) or disable (False).
  ##
  ## \return {'rc': rc, key: value}
  #
  def cmdEnableAuxPort5V(self, enable):
    cmdId  = LaeWdCmdIdEnableAuxPort

    key     = 'enables'
    subkey  = 'aux_port_5v_en'
    ret     = {'rc': WDRcOk, subkey: self.m_fw[key][subkey]}

    # check connection
    if not self.isOpen():
      self.printError(_WDErrMsgOpen)
      ret['rc'] = WDRcErr
      return ret

    # unknown firmware
    if self.m_fw['version'] == 0:
      return ret
    # always enabled
    elif self.m_fw['version'] == 1:
      self.m_fw[key][subkey] = True
      ret[subkey] = self.m_fw[key][subkey]
      return ret

    if enable:
      val = LaeWdArgDPinValHigh
    else:
      val = LaeWdArgDPinValLow

    b1  = chr(LaeWdArgAuxPort5V)
    b2  = chr(val)
    cmd = [chr(cmdId), b1, b2]

    # send command
    try:
      self.m_i2c.write(self.m_addr, cmd)
    except i2c.I2CException as inst:
      self.printError("CmdId %d" % (cmdId), inst.message)
      ret['rc'] = WDRcErr
    else:
      self.m_fw[key][subkey] = enable
      ret[subkey] = self.m_fw[key][subkey]

    return ret

  #
  ## \brief Execute command to enable/disable battery output to aux. port.
  ##
  ## \param enable      Enable (True) or disable (False).
  ##
  ## \return {'rc': rc, key: value}
  #
  def cmdEnableAuxPortBatt(self, enable):
    cmdId  = LaeWdCmdIdEnableAuxPort

    key     = 'enables'
    subkey  = 'aux_port_batt_en'
    ret     = {'rc': WDRcOk, subkey: self.m_fw[key][subkey]}

    # check connection
    if not self.isOpen():
      self.printError(_WDErrMsgOpen)
      ret['rc'] = WDRcErr
      return ret

    # unknown firmware
    if self.m_fw['version'] == 0:
      return ret
    # always enabled
    elif self.m_fw['version'] == 1:
      self.m_fw[key][subkey] = True
      ret[subkey] = self.m_fw[key][subkey]
      return ret

    if enable:
      val = LaeWdArgDPinValHigh
    else:
      val = LaeWdArgDPinValLow

    b1  = chr(LaeWdArgAuxPortBatt)
    b2  = chr(val)
    cmd = [chr(cmdId), b1, b2]

    # send command
    try:
      self.m_i2c.write(self.m_addr, cmd)
    except i2c.I2CException as inst:
      self.printError("CmdId %d" % (cmdId), inst.message)
      ret['rc'] = WDRcErr
    else:
      self.m_fw[key][subkey] = enable
      ret[subkey] = self.m_fw[key][subkey]

    return ret

  #
  ## \brief Execute command to read enable lines.
  ##
  ## \return {'rc': rc, key: value}
  #
  def cmdReadEnables(self):
    cmdId  = LaeWdCmdIdReadEnables
    rspLen = LaeWdRspLenReadEnables

    key = 'enables'
    ret = {'rc': WDRcOk, key: self.m_fw[key]}

    # check connection
    if not self.isOpen():
      self.printError(_WDErrMsgOpen)
      ret['rc'] = WDRcErr
      return ret

    # unknown firmware or unsupported feature
    if self.m_fw['version'] < 2:
      return ret

    cmd = [chr(cmdId)]

    # send command
    try:
      rsp = self.m_i2c.write_read(self.m_addr, cmd, rspLen)
    except i2c.I2CException as inst:
      self.printError("CmdId %d" % (cmdId), inst.message)
      ret['rc'] = WDRcErr
    else:
      if len(rsp) == rspLen:
        val0 = ord(rsp[0])
        val1 = ord(rsp[1])
        val2 = ord(rsp[2])
      if val0:
        self.m_fw[key]['motor_ctlr_en'] = True
      else:
        self.m_fw[key]['motor_ctlr_en'] = False
      if val1:
        self.m_fw[key]['aux_port_batt_en'] = True
      else:
        self.m_fw[key]['aux_port_batt_en'] = False
      if val2:
        self.m_fw[key]['aux_port_5v_en'] = True
      else:
        self.m_fw[key]['aux_port_5v_en'] = False
      ret[key] = self.m_fw[key]

    return ret

  #
  ## \brief Execute command to read sensed voltages.
  ##
  ## \return {'rc': rc, key: value}
  #
  def cmdReadVoltages(self):
    cmdId  = LaeWdCmdIdReadVolts
    rspLen = LaeWdRspLenReadVolts

    key = 'voltages'
    ret = {'rc': WDRcOk, key: self.m_fw[key]}

    # check connection
    if not self.isOpen():
      self.printError(_WDErrMsgOpen)
      ret['rc'] = WDRcErr
      return ret

    # unknown firmware or unsupported feature
    if self.m_fw['version'] < 2:
      return ret

    cmd = [chr(cmdId)]

    # send command
    try:
      rsp = self.m_i2c.write_read(self.m_addr, cmd, rspLen)
    except i2c.I2CException as inst:
      self.printError("CmdId %d" % (cmdId), inst.message)
      ret['rc'] = WDRcErr
    else:
      if len(rsp) == rspLen:
        val0 = ord(rsp[0])
        val1 = ord(rsp[1])
      self.m_fw[key]['jack'] = val0 * LaeWdargVScale
      self.m_fw[key]['batt'] = val1 * LaeWdargVScale
      ret[key] = self.m_fw[key]

    return ret

  #
  ## \brief Print error to stderr.
  ##
  ## \param emsg1   Error message one (required).
  ## \param emsg2   Error message two (optional).
  #
  def printError(self, emsg1, emsg2=None):
    if emsg2 is None:
      print("Error: WatchDog: {0}".format(emsg1), file=sys.stderr)
    else:
      print("Error: WatchDog: {0}: {1}".format(emsg1, emsg2), file=sys.stderr)
