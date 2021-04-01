#! /usr/bin/env python3

###############################################################################
#
# Package:  RoadNarrows Robotics Laelaps Robotic Mobile Platform Package
#
# File: laelaps_speed
#
## \file 
##
## $LastChangedDate: 2015-09-09 14:00:46 -0600 (Wed, 09 Sep 2015) $
## $Rev: 4080 $
##
## \brief Run one motor at the given speed.
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
# Unless otherwise stated explicitly, all materials contained are copyrighted
# and may not be used without RoadNarrows LLC's written consent,
# except as provided in these terms and conditions or in the copyright
# notice (documents and software) or other proprietary notice provided with
# the relevant materials.
# 
# IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY 
# MEMBERS/EMPLOYEES/CONTRACTORS OF ROADNARROWS OR DISTRIBUTORS OF THIS SOFTWARE
# BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
# CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS
# DOCUMENTATION, EVEN IF THE AUTHORS OR ANY OF THE ABOVE PARTIES HAVE BEEN
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
# THE AUTHORS AND  ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY WARRANTIES,
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
import termios
import fcntl
import time
import math
import random
import getopt

from serial.serialutil import SerialException, SerialTimeoutException

import Laelaps.SysConf as SysConf
import Laelaps.WatchDog as WatchDog
import Laelaps.RoboClaw as RoboClaw

MotorNames = ['left_front', 'right_front', 'left_rear', 'right_rear']
TestSep = "\
-------------------------------------------------------------------------------"

# ------------------------------------------------------------------------------
# Keyboard Class
# ------------------------------------------------------------------------------

class keyboard():
  def __init__(self):
    fd = sys.stdin.fileno()
    self.oldterm = termios.tcgetattr(fd)
    self.oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)

  def __del__(self):
    self.sane()

  def sane(self):
    fd = sys.stdin.fileno()
    termios.tcsetattr(fd, termios.TCSAFLUSH, self.oldterm)
    fcntl.fcntl(fd, fcntl.F_SETFL, self.oldflags)

  def setNonBlocking(self):
    fd = sys.stdin.fileno()
    newattr = termios.tcgetattr(fd)
    newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
    termios.tcsetattr(fd, termios.TCSANOW, newattr)
    fcntl.fcntl(fd, fcntl.F_SETFL, self.oldflags | os.O_NONBLOCK)

  def getchar(self):
    c = None
    try:
      c = sys.stdin.read(1)
    except IOError:
      pass
    return c

# ------------------------------------------------------------------------------
# Fake Motor Controller Class
# ------------------------------------------------------------------------------

class fakemc():
  def __init__(self):
    pass

  def open(self, dev, baud, fnChipSelect=None):
    self.dev = dev
    self.baud = baud
    self.m1_speed = {}
    self.m2_speed = {}
    self.m1_encoder = {}
    self.m2_encoder = {}

  def close(self):
    pass

  def readVersion(self, addr):
    return "1.5.6 firmwareXYZ"

  def readMainBatterySettings(self, addr):
    return (60.1, 324.0)

  def readLogicBatterySettings(self, addr):
    raise SerialTimeoutException('bogus')

  def readEncoderMode(self, addr):
    return (0, 0)

  def readM1Pidq(self, addr):
    return (0x00010000, 0x00008000, 0x00004000,  10000)

  def readM2Pidq(self, addr):
    return (0x00010000, 0x00008000, 0x00004000,  10000)

  def readErrorState(self, addr):
    return 0

  def readTemperature(self, addr):
    return 288

  def readMainBattery(self, addr):
    return 111

  def readLogicBattery(self, addr):
    return 49

  def readCurrents(self, addr):
    return (75.0, 49.0)

  def readM1Speed(self, addr):
    if addr not in self.m1_speed:
      self.m1_speed[addr] = 0
    return (self.m1_speed[addr], 0x00)

  def readM2Speed(self, addr):
    if addr not in self.m2_speed:
      self.m2_speed[addr] = 0
    return (self.m2_speed[addr], 0x00)

  def setM1Speed(self, addr, speed):
    self.m1_speed[addr] = speed

  def setM2Speed(self, addr, speed):
    self.m2_speed[addr] = speed

  def readM1Encoder(self, addr):
    if addr not in self.m1_encoder:
      self.m1_encoder[addr] = random.randint(0, 1000000)
    return (self.m1_encoder[addr], 0)

  def readM2Encoder(self, addr):
    if addr not in self.m2_encoder:
      self.m2_encoder[addr] = random.randint(0, 1000000)
    return (self.m2_encoder[addr], 0x02)

  def resetEncoderCnts(self, addr):
    self.m1_encoder[addr] = 0
    self.m2_encoder[addr] = 0


# ------------------------------------------------------------------------------
# Speed Class
# ------------------------------------------------------------------------------

class Speed():
  def __init__(self, kwargs):
    self.kwargs = kwargs

    self.ctlrKeys   = ['front', 'rear']
    self.ctlrInfo   = {
        'front': {'addr': SysConf.MotorCtlrAddrFront, 'name': 'front'},
        'rear':  {'addr': SysConf.MotorCtlrAddrRear,  'name': 'rear'}
    }

    self.motorKeys  = ['left_front', 'right_front', 'left_rear', 'right_rear']
    self.motorInfo  = {
        'left_front':
          {'ctlr_key': 'front', 'motor_index': 0, 'name': 'left_front'},
        'right_front':
          {'ctlr_key': 'front', 'motor_index': 1, 'name': 'right_front'},
        'left_rear':
          {'ctlr_key': 'rear', 'motor_index': 0, 'name': 'left_rear'},
        'right_rear':
          {'ctlr_key': 'rear', 'motor_index': 1, 'name': 'right_rear'},
    }

  def run(self, keepalive):
    motorkey  = self.kwargs['motor']
    goalspeed = self.kwargs['speed']

    print()
    print(TestSep)
    print("*** Laelaps %s(%d) motor speed=%d test ***" % \
        (motorkey, self.motorKeys.index(motorkey), goalspeed))
    print()

    # Create motor controller object
    if self.kwargs['fake']:
      motorctlr = fakemc()
    else:
      motorctlr = RoboClaw.RoboClaw()

    # Open communiction on serial bus to Laelaps multi-dropped motor controllers
    device  = SysConf.MotorCtlrDevName
    baud    = SysConf.MotorCtlrBaudRate
    
    print("Open motor controller serial interface %s@%d" % (device, baud))
    try:
      motorctlr.open(device, baud, None)
    except RoboClaw.RoboClawException as inst:
      print('Error:', inst.message)
      print()

    if motorctlr.isOpen():
      addr  = self.ctlrInfo[self.motorInfo[motorkey]['ctlr_key']]['addr']
      mi    = self.motorInfo[motorkey]['motor_index']

      if mi == 0:
        setSpeed    = motorctlr.setM1Speed
        readSpeed   = motorctlr.readM1Speed
        readEncoder = motorctlr.readM1Encoder
      else:
        setSpeed    = motorctlr.setM2Speed
        readSpeed   = motorctlr.readM2Speed
        readEncoder = motorctlr.readM2Encoder

      motorctlr.resetEncoderCnts(addr)

      try:
        setSpeed(addr, goalspeed)
      except RoboClaw.RoboClawException as inst:
        print("Failed to set speed")
        return

      print()
      print("Press any key to abort")
      print()

      kb = keyboard()
      kb.setNonBlocking()

      n     = 0
      nErrs = 0
      while kb.getchar() is None:
        try:
          curspeed, status = readSpeed(addr)
          curenc, status = readEncoder(addr)
          print("%5d. %10d  %14d\r" % (n, curspeed, curenc), end='')
          sys.stdout.flush()
          n += 1
          if nErrs > 0:
            nErrs -= 1
        except RoboClaw.RoboClawException as inst:
          nErrs += 1
          if nErrs >= 5:
            break
        time.sleep(0.1)
        keepalive()

      print("                                                               \r")

      try:
        setSpeed(addr, 0)
        print("Stopped")
      except RoboClaw.RoboClawException as inst:
        print("Failed to stop")

      motorctlr.close()


# ------------------------------------------------------------------------------
# MotorCtlrEnable Class
# ------------------------------------------------------------------------------

class MotorCtlrEnable():
  def __init__(self):
    # The WatchDog subprocessor controls the enable lines to the motor
    # controllers.
    self.m_wd = WatchDog.WatchDog()
    self.m_doDisable = False

  def enable(self):
    if not self.m_wd.open(SysConf.SensorDevName):
      print('Error: Failed to open connection to watchdog subprocessor.')
      print('       Continuing...')
      return

    # get firmware version 
    self.m_wd.cmdGetFwVersion()

    # read motor controller enable lines state
    ret = self.m_wd.cmdReadEnables()
    if ret['rc'] != 'ok':
      print('Error: Failed to read enable lines.')
      print('       Continuing...')
      return

    if ret['enables']['motor_ctlr_en']:
      print("Warning: Motor controllers already enabled - may be in use.")
      return

    self.m_doDisable = True

    ret = self.m_wd.cmdEnableMotorCtlrs(True)
    if ret['rc'] != 'ok':
      print('Error: Failed to enable motor controllers.')
      print('       Continuing...')
      return

    time.sleep(0.5)

    print("Motor controllers enabled.")

  def keepalive(self):
    if self.m_wd.isOpen():
      self.m_wd.cmdPetTheDog()

  def cleanup(self):
    if self.m_doDisable:
      ret = self.m_wd.cmdEnableMotorCtlrs(False)
      if ret['rc'] == 'ok':
        print("Motor controllers disabled.")
    self.m_wd.close()


# ------------------------------------------------------------------------------
# Exception Class usage
# ------------------------------------------------------------------------------

##
## \brief Unit test command-line exception class.
##
## Raise usage excpetion.
##
class usage(Exception):

  ##
  ## \brief Constructor.
  ##
  ## \param msg   Error message string.
  ##
  def __init__(self, msg):
    ## error message attribute
    self.msg = msg


# ------------------------------------------------------------------------------
# Class application
# ------------------------------------------------------------------------------

##
## \brief Laelaps diagnositcs application class.
##
class application():

  #
  ## \brief Constructor.
  #
  def __init__(self):
    self._Argv0 = os.path.basename(__file__)
    self.m_win = None

  #
  ## \brief Print usage error.
  ##
  ## \param emsg  Error message string.
  #
  def printUsageErr(self, emsg):
    if emsg:
      print("%s: Error: %s" % (self._Argv0, emsg))
    else:
      print("%s: Error" % (self._Argv0))
    print("Try '%s --help' for more information." % (self._Argv0))

  ## \brief Print Command-Line Usage Message.
  def printUsage(self):
    print(\
"""
usage: %s [OPTIONS] MOTOR SPEED
       %s --help

Run Laelaps speed test.

Options and arguments:
    --fake        : Run diagnostics on fake hardware."
-h, --help        : Display this help and exit.

MOTOR             : Motor name/id to test. One of:
  left_front right_front left_rear right_rear
    or
  0 1 2 3
SPEED             : Signed qpps speed of motor, with positive being forwards.

Exit Status:
  On success, 0. An exit status of 128 indicates usage 
  error.
"""  % (self._Argv0, self._Argv0))
 
  #
  ## \brief Get command-line options
  ##  
  ## \param argv          Argument list. If not None, then overrides
  ##                      command-line arguments.
  ## \param [out] kwargs  Keyword argument list.  
  ##
  ## \return Parsed keyword arguments.
  #
  def getOptions(self, argv=None, **kwargs):
    if argv is None:
      argv = sys.argv

    self._Argv0 = os.path.basename(kwargs.get('argv0', __file__))

    # defaults
    kwargs['debug'] = False
    kwargs['fake']  = False

    # parse command-line options
    try:
      opts, args = getopt.getopt(argv[1:], "f?h",
          ['help', 'fake', ''])
    except getopt.error as msg:
      raise usage(msg)
    for opt, optarg in opts:
      if opt in ('-f', '--fake'):
        kwargs['fake'] = True
      elif opt in ('-h', '--help', '-?'):
        self.printUsage()
        sys.exit(0)

    if len(args) < 1:
      self.printUsageErr("No motor specified.")
      sys.exit(128)
    elif len(args) < 2:
      self.printUsageErr("No speed specified.")
      sys.exit(128)

    kwargs['motor'] = None
    try:
      i = int(args[0])
      if (i >= 0) and (i < len(MotorNames)):
        kwargs['motor'] = MotorNames[i]
    except ValueError:
      try:
        i = MotorNames.index(args[0])
        kwargs['motor'] = args[0]
      except ValueError:
        pass

    if kwargs['motor'] is None:
      self.printUsageErr("{0}: Not a motor.".format(args[0]))
      sys.exit(128)

    try:
      kwargs['speed'] = int(args[1])
    except ValueError:
      self.printUsageErr("%s: Not an integer." % (args[1]))
      sys.exit(128)

    return kwargs

  #
  ## \brief Run application.
  ##    
  ## \param argv    Optional argument list to override command-line arguments.
  ## \param kwargs  Optional keyword argument list.
  ##
  ## \return Exit code.
  #
  def run(self, argv=None, **kwargs):
  
    # parse command-line options and arguments
    try:
      self.kwargs = self.getOptions(argv, **kwargs)
    except usage as e:
      print(e.msg)
      return 128
    
    en = MotorCtlrEnable()
    en.enable()

    go = Speed(self.kwargs)
    go.run(en.keepalive)

    en.cleanup()
      

# ------------------------------------------------------------------------------
# main
# ------------------------------------------------------------------------------
if __name__ == '__main__':
  app = application();
  sys.exit( app.run() );
