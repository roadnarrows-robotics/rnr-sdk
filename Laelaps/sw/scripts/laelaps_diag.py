#! /usr/bin/env python

###############################################################################
#
# Package:  RoadNarrows Robotics Laelaps Robotic Mobile Platform Package
#
# File: laelaps_diag.py
#
## \file 
##
## $LastChangedDate: 2016-02-22 18:11:12 -0700 (Mon, 22 Feb 2016) $
## $Rev: 4328 $
##
## \brief Perform Laelaps diagnostics (python version).
##
## \note The sensors and watchdog interface is via I2C. A simple I2C python
## module needs to be found or written.
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
import time
import math
import random
import struct
import getopt

from serial.serialutil import SerialException, SerialTimeoutException

import Laelaps.SysConf as SysConf
import Laelaps.RoboClaw as RoboClaw

Diagnostics = ['motors', 'tofs', 'imu', 'watchdog']
DiagSep = "\
-------------------------------------------------------------------------------"

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
    if not self.m1_speed.has_key(addr):
      self.m1_speed[addr] = 0
    return (self.m1_speed[addr], 0x00)

  def readM2Speed(self, addr):
    if not self.m2_speed.has_key(addr):
      self.m2_speed[addr] = 0
    return (self.m2_speed[addr], 0x00)

  def setM1Speed(self, addr, speed):
    self.m1_speed[addr] = speed

  def setM2Speed(self, addr, speed):
    self.m2_speed[addr] = speed

  def readM1Encoder(self, addr):
    if not self.m1_encoder.has_key(addr):
      self.m1_encoder[addr] = random.randint(0, 1000000)
    return (self.m1_encoder[addr], 0)

  def readM2Encoder(self, addr):
    if not self.m2_encoder.has_key(addr):
      self.m2_encoder[addr] = random.randint(0, 1000000)
    return (self.m2_encoder[addr], 0x02)

  def resetEncoderCnts(self, addr):
    self.m1_encoder[addr] = 0
    self.m2_encoder[addr] = 0


# ------------------------------------------------------------------------------
# Motors Diagnostics Class
# ------------------------------------------------------------------------------

class DiagMotors():
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

  def run(self):
    print
    print DiagSep
    print "*** Laelaps Motors Diagnostics ***"
    print

    testCnt = 0
    passCnt = 0

    # Create motor controller object
    if self.kwargs['fake']:
      motorctlr = fakemc()
    else:
      motorctlr = RoboClaw.RoboClaw()

    # Open communiction on serial bus to Laelaps multi-dropped motor controllers
    device  = SysConf.MotorCtlrDevName
    baud    = SysConf.MotorCtlrBaudRate
    cs      = SysConf.MotorCtlrChipSelectGpio
    testCnt += 1
    print "Test: Open motor controller serial interface %s@%d" % (device, baud)
    try:
      motorctlr.open(device, baud, cs)
      passCnt += 1
    except RoboClaw.RoboClawException as inst:
      print 'Error:', inst.message
      print
      return 0
    print "    ... %d/%d tests passed." % (passCnt, testCnt)
    print

    for k in self.ctlrKeys:
      m, n = self.testMotorCtlrState(motorctlr, self.ctlrInfo[k]['addr'],
                                      self.ctlrInfo[k]['name'])
      passCnt += m
      testCnt += n

    for k in self.motorKeys:
      m, n = self.testMotorState(motorctlr,
                          self.ctlrInfo[self.motorInfo[k]['ctlr_key']]['addr'],
                          self.motorInfo[k]['motor_index'],
                          self.motorInfo[k]['name'])
      passCnt += m
      testCnt += n


    m, n = self.testMotors(motorctlr)
    passCnt += m
    testCnt += n

    motorctlr.close()

    print "Summary:"
    print "  ... %d/%d motors diagnostic tests passed." % (passCnt, testCnt)

    if passCnt == testCnt:
      return 1
    else:
      return 0

  def testMotorCtlrState(self, motorctlr, addr, name):
    print "Test: Read %s motor controller state (address=0x%02x)." % \
        (name, addr)

    testCnt = 0
    passCnt = 0

    tests = [
      ["  Version:             ",
              self.serReadHelper, (motorctlr.readVersion, addr),
              self.good,
              lambda v: v[:-3] ],
      ["  Main Battery Range:  ",
              self.serReadHelper, (motorctlr.readMainBatterySettings, addr),
              self.good,
              lambda v: "%.1fV, %.1fV" % (v[0]/10.0, v[1]/10.0) ],
      ["  Logic Battery Range: ",
              self.serReadHelper, (motorctlr.readLogicBatterySettings, addr),
              self.good,
              lambda v: "%.1fV, %.1fV" % (v[0]/10.0, v[1]/10.0) ],
      ["  Error State:         ",
              self.serReadHelper, (motorctlr.readErrorState, addr),
              self.good,
              lambda v: "0x%02x" % (v) ],
      ["  Temperature:         ",
              self.serReadHelper, (motorctlr.readTemperature, addr),
              self.good,
              lambda v: "%.1fC" % (v/10.0) ],
      ["  Main Battery:        ",
              self.serReadHelper, (motorctlr.readMainBattery, addr),
              self.good,
              lambda v: "%.1fV" % (v/10.0) ],
      ["  Logic Battery:       ",
              self.serReadHelper, (motorctlr.readLogicBattery, addr),
              self.good,
              lambda v: "%.1fV" % (v/10.0) ],
    ]

    # run tests
    for t in tests:
      testCnt += 1
      if self.runTest(t[0], t[1], t[2], t[3], t[4]):
        passCnt +=1

    print "    ... %d/%d tests passed." % (passCnt, testCnt)
    print

    return (passCnt, testCnt)

  def testMotorState(self, motorctlr, addr, motorIndex, name):
    print "Test: Read %s motor state." % (name)

    testCnt = 0
    passCnt = 0

    if motorIndex == 0:
      readPidq    = motorctlr.readM1Pidq
      readEncoder = motorctlr.readM1Encoder
      readSpeed   = motorctlr.readM1Speed
    else:
      readPidq    = motorctlr.readM2Pidq
      readEncoder = motorctlr.readM2Encoder
      readSpeed   = motorctlr.readM2Speed

    tests = [
      ["  Encoder Mode: ",
              self.serReadHelper, (motorctlr.readEncoderMode, addr),
              lambda v: self.chkEncoderMode(motorIndex, v),
              lambda v: self.fmtEncoderMode(motorIndex, v) ],
      ["  Vel. PID:     ",
              self.serReadHelper, (readPidq, addr),
              self.good,
              lambda v: self.fmtPidq(motorIndex, v) ],
      ["  Encoder:      ",
              self.serReadHelper, (readEncoder, addr),
              self.good,
              lambda v: "%u, 0x%02x" % (v[0], v[1]) ],
      ["  Speed:        ",
              self.serReadHelper, (readSpeed, addr),
              self.good,
              lambda v: "%u, 0x%02x" % (v[0], v[1]) ],
      ["  Current:      ",
              self.serReadHelper, (motorctlr.readCurrents, addr),
              self.good,
              lambda v: "%.3fA" % (v[motorIndex]/100.0) ],
    ]

    # run tests
    for t in tests:
      testCnt += 1
      if self.runTest(t[0], t[1], t[2], t[3], t[4]):
        passCnt +=1

    print "    ... %d/%d tests passed." % (passCnt, testCnt)
    print

    return (passCnt, testCnt)

  def testMotors(self, motorctlr):
    print "Test: Drive motors."

    testCnt = 0
    passCnt = 0

    addrFront = self.ctlrInfo['front']['addr']
    addrRear  = self.ctlrInfo['rear']['addr']

    speedProf = [500, 1000, 2500, 5000, 2500, 1000, 500,
                -500, -1000, -2500, -5000, -2500, -1000, -500,
                0]

    testCnt += 1
    if self.runTest("  All stop:             ",
                        self.stopHelper, (motorctlr,),
                        self.good, 
                        lambda v: "[PASS]"):
      passCnt += 1

    testCnt += 1
    if self.runTest("  Reset front encoders: ",
            self.serWriteHelper, (motorctlr.resetEncoderCnts, addrFront),
            self.good, 
            lambda v: "[PASS]"):
      passCnt += 1

    testCnt += 1
    if self.runTest("  Reset rear encoders:  ",
            self.serWriteHelper, (motorctlr.resetEncoderCnts, addrRear),
            self.good, 
            lambda v: "[PASS]"):
      passCnt += 1

    print "  Loop through speed profile:"

    for speed in speedProf:
      for k in self.motorKeys:
        testCnt += 1
        if self.runTest("    Set %*s motor speed = %-6d: " % (11, k, speed),
            self.setSpeedHelper, (motorctlr, k, speed),
            self.good, 
            lambda v: "[PASS] (speed = %d)" % (v)):
          passCnt += 1
      print
      time.sleep(1.0)

    print "    ... %d/%d tests passed." % (passCnt, testCnt)
    print

    return (passCnt, testCnt)

  def runTest(self, what, fnExec, argsExec, fnChk, fnFmt):
    v, pf, e  = fnExec(*argsExec)
    if pf:
      pf, e = fnChk(v)
    if pf:
      s  = fnFmt(v)
    elif e is not None:
      s = "[FAIL] (%s)" % (e)
    else:
      s = '[FAIL]'
    print "%s%s" % (what, s)
    return pf

  def serReadHelper(self, fnRead, *argsRead):
    try:
      v  = fnRead(*argsRead)
      pf = True
      e  = None
    except RoboClaw.RoboClawException as inst:
      v  = None
      pf = False
      e  = inst.message
    return (v, pf, e)

  def serWriteHelper(self, fnWrite, *argsWrite):
    try:
      fnWrite(*argsWrite)
      pf = True
      e  = None
    except RoboClaw.RoboClawException as inst:
      pf = False
      e  = inst.message
    return (None, pf, e)

  def stopHelper(self, motorctlr):
    try:
      motorctlr.setM1Speed(self.ctlrInfo['front']['addr'], 0)
      motorctlr.setM2Speed(self.ctlrInfo['front']['addr'], 0)
      motorctlr.setM1Speed(self.ctlrInfo['rear']['addr'],  0)
      motorctlr.setM2Speed(self.ctlrInfo['rear']['addr'],  0)
      v  = (0, 0, 0, 0)
      pf = True
      e  = None
    except RoboClaw.RoboClawException as inst:
      v  = None
      pf = False
      e  = inst.message
    return (v, pf, e)

  def setSpeedHelper(self, motorctlr, motorkey, speed):
    addr  = self.ctlrInfo[self.motorInfo[motorkey]['ctlr_key']]['addr']
    mi    = self.motorInfo[motorkey]['motor_index']
    if mi == 0:
      setSpeed  = motorctlr.setM1Speed
      readSpeed = motorctlr.readM1Speed
    else:
      setSpeed  = motorctlr.setM2Speed
      readSpeed = motorctlr.readM2Speed
    try:
      setSpeed(addr, speed)
    except RoboClaw.RoboClawException as inst:
      return (None, False, inst.message)
    curspeed = 0
    for i in range(0, 10):
      try:
        curspeed, status = readSpeed(addr)
        if speed == 0:
          if abs(curspeed) < 10:
            return (curspeed, True, None)
        else:
          if math.fabs(speed-curspeed)/math.fabs(speed) <= 0.05:
            return (curspeed, True, None)
      except RoboClaw.RoboClawException as inst:
        return (None, False, inst.message)
      time.sleep(0.1)
    return (curspeed, False, "timeout reaching speed, curspeed=%d" % (curspeed))

  def good(self, args):
    return (True, None)

  def chkEncoderMode(self, mi, v):
    if v[mi] == 0x00:
      return (True, None)
    else:
      e = "0x%02x != required quadrature encoding" % (v[mi])
      return (False, e)

  def fmtEncoderMode(self, mi, v):
    s = "0x%02x " % (v[mi])
    if v[mi] == 0:
      s += "(quadrature encoding)"
    return s

  def fmtPidq(self, motorIndex, v):
    f = 65536.0
    s = "Kp=%f, Ki=%f, Kd=%f, max_qpps=%u" % (v[0]/f, v[1]/f, v[2]/f, v[3])
    return s


# ------------------------------------------------------------------------------
# Time-of-Flight Diagnostics Class
# ------------------------------------------------------------------------------

class DiagToFs():
  def __init__(self, kwargs):
    self.kwargs = kwargs

  def run(self):
    print
    print DiagSep
    print "*** Laelaps Time-of-Flight Diagnostics ***"
    print

    print "Diagnostics not supported yet: [FAIL]"
    print
    return 0


# ------------------------------------------------------------------------------
# Inertia Measurement Unit Diagnostics Class
# ------------------------------------------------------------------------------

class DiagImu():
  def __init__(self, kwargs):
    self.kwargs = kwargs

  def run(self):
    print
    print DiagSep
    print "*** Laelaps Inertia Measurement Unit Diagnostics ***"
    print

    print "Diagnostics not supported yet: [FAIL]"
    print
    return 0


# ------------------------------------------------------------------------------
# Arduino Watchdog Diagnostics Class
# ------------------------------------------------------------------------------

class DiagWatchdog():
  def __init__(self, kwargs):
    self.kwargs = kwargs

  def run(self):
    print
    print DiagSep
    print "*** Laelaps Arduino Watchdog Diagnostics ***"
    print

    print "Diagnostics not supported yet: [FAIL]"
    print
    return 0


# ------------------------------------------------------------------------------
# Exception Class usage
# ------------------------------------------------------------------------------

##
## \brief Usage exception class.
##
## Raise usage exception.
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
      print "%s: Error: %s" % (self._Argv0, emsg)
    else:
      print "%s: Error" % (self._Argv0)
    print "Try '%s --help' for more information." % (self._Argv0)

  ## \brief Print Command-Line Usage Message.
  def printUsage(self):
    print \
"""
usage: %s [OPTIONS] DIAG [DIAG ...]
       %s --help

Run Laelaps diagnostic(s).

Options and arguments:
    --fake        : Run diagnostics on fake hardware."
-h, --help        : Display this help and exit.

DIAG              : Diagnostic to run. One of:
  all       - Run all dignostics.
  motors    - Run motors diagnotics.
  tofs      - Run time-of-flight sensors diagnostics.
  imu       - Run Inertia Measurement Unit diagnostics.
  watchdog  - Run Arduino watchdog sub-processor diagnostics.

Exit Status:
  The number of diagnostics passed. An exit status of 128 indicates usage 
  error.
"""  % (self._Argv0, self._Argv0)
 
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
    except getopt.error, msg:
      raise usage(msg)
    for opt, optarg in opts:
      if opt in ('-f', '--fake'):
        kwargs['fake'] = True
      elif opt in ('-h', '--help', '-?'):
        self.printUsage()
        sys.exit(0)

    if len(args) < 1:
      self.printUsageErr("No diagnostic(s) specified.")
      sys.exit(128)
    try:
      args.index('all')
      kwargs['diags'] = Diagnostics
    except ValueError:
      kwargs['diags'] = args

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
    except usage, e:
      print e.msg
      return 128

    passCnt = 0
    diagCnt = 0
    for diag in self.kwargs['diags']:
      try:
        Diagnostics.index(diag)
      except ValueError:
        print "%s: Unknown diagnostic - ignoring" % (diag)
        continue
      diagCnt += 1
      if diag == 'motors':
        diag = DiagMotors(self.kwargs)
      elif diag == 'tofs':
        diag = DiagToFs(self.kwargs)
      elif diag == 'imu':
        diag = DiagImu(self.kwargs)
      elif diag == 'watchdog':
        diag = DiagWatchdog(self.kwargs)
      passCnt += diag.run()
      
    print
    print "      ~~~"
    print "%d/%d diagnostics passed." % (passCnt, diagCnt)
    print

    return passCnt


# ------------------------------------------------------------------------------
# main
# ------------------------------------------------------------------------------
if __name__ == '__main__':
  app = application();
  sys.exit( app.run() );
