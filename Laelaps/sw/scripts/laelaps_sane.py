#! /usr/bin/env python

###############################################################################
#
# Package:  RoadNarrows Robotics Laelaps Robotic Mobile Platform Package
#
# File: laelaps_sane
#
## \file 
##
## $LastChangedDate: 2015-09-09 14:00:46 -0600 (Wed, 09 Sep 2015) $
## $Rev: 4080 $
##
## \brief Place a Laelaps subsystem in a sane state.
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
import getopt
import time

from serial.serialutil import SerialException, SerialTimeoutException

import Laelaps.SysConf as SysConf
import Laelaps.WatchDog as WatchDog
import Laelaps.RoboClaw as RoboClaw

SaneSubSys = ['motors']
BigSep = "\
-------------------------------------------------------------------------------"

# ------------------------------------------------------------------------------
# Sanity Class
# ------------------------------------------------------------------------------

class Sanity():
  def __init__(self, kwargs):
    self.kwargs = kwargs

  def run(self):
    tf = True
    for subsys in self.kwargs['subsys']:
      if subsys == 'motors':
        tf &= self.runMotorControllersSanity()
    return tf

  def runMotorControllersSanity(self):
    # Controller info
    ctlrInfo   = {
        'front': {'addr': SysConf.MotorCtlrAddrFront},
        'rear':  {'addr': SysConf.MotorCtlrAddrRear}
    }

    # TODO enable here
    #powered = Gpio.areMotorCtlrsPowered()
    #if powered == True:
    #  print "Warning: Motor controllers may be in use by another application."
    #else:
    #  Gpio.enableMotorCtlrsPower(True)
    #  time.sleep(0.5)

    # Create motor controller object
    motorctlr = RoboClaw.RoboClaw()

    # Open communiction on serial bus to Laelaps multi-dropped motor controllers
    device  = SysConf.MotorCtlrDevName
    baud    = SysConf.MotorCtlrBaudRate
    #cs      = SysConf.MotorCtlrChipSelectGpio  # deprecated
    print "Opening motor controller serial interface %s@%d" % (device, baud)
    try:
      motorctlr.open(device, baud)
    except RoboClaw.RoboClawException as inst:
      print 'Error:', inst.message
      print

    tf = True

    if motorctlr.isOpen():
      for k, d in ctlrInfo.iteritems():
        tf &= self.saneMotorCtlr(motorctlr, k, d['addr'])
      motorctlr.close()

    if not powered:
      Gpio.enableMotorCtlrsPower(False)

    return tf

  def saneMotorCtlr(self, motorctlr, key, addr):
    print
    print BigSep
    print "*** Restore Laelaps %s motor controller sanity ***" % (key)
    print

    # defaults (very weak PID)
    #Kp    = RoboClaw.ParamVelPidPDft
    #Ki    = RoboClaw.ParamVelPidIDft
    #Kd    = RoboClaw.ParamVelPidDDft
    #Qpps  = RoboClaw.ParamVelPidQppsDft

    # typical laelaps PID
    Kp    = 500 * RoboClaw.ParamVelPidCvt
    Ki    = 100 * RoboClaw.ParamVelPidCvt
    Kd    =   5 * RoboClaw.ParamVelPidCvt
    Qpps  = 10000

    # prevent motor burn out and/or robot shut off from system max limits
    maxAmps= 4.5 / RoboClaw.ParamAmpScale

    try:
      motorctlr.setMainBatterySettings(addr, 60, 340)   # 6.0V - 34.0V
    except RoboClaw.RoboClawException as inst:
      print "Failed to set main battery cutoffs.", inst.message
      return False
    try:
      motorctlr.setLogicBatterySettings(addr, 55, 340)    # 5.5V - 34.0V
    except RoboClaw.RoboClawException as inst:
      print "Failed to set logic battery cutoffs.", inst.message
      return False
    try:
      motorctlr.setM1EncoderMode(addr, 0)
    except RoboClaw.RoboClawException as inst:
      print "Failed to set motor 1 encoder mode to quadrature.", inst.message
      return False
    try:
      motorctlr.setM2EncoderMode(addr, 0)
    except RoboClaw.RoboClawException as inst:
      print "Failed to set motor 2 encoder mode to quadrature.", inst.message
      return False
    try:
      motorctlr.setM1Pidq(addr, Kp, Ki, Kd, Qpps)
    except RoboClaw.RoboClawException as inst:
      print "Failed to set motor 1 velocity PID.", inst.message
      return False
    try:
      motorctlr.setM2Pidq(addr, Kp, Ki, Kd, Qpps)
    except RoboClaw.RoboClawException as inst:
      print "Failed to set motor 2 velocity PID:", inst.message
      return False
    try:
      motorctlr.setM1MaxCurrentLimit(addr, maxAmps)
    except RoboClaw.RoboClawException as inst:
      print "Failed to set motor 1 maximum current.", inst.message
      return False
    try:
      motorctlr.setM2MaxCurrentLimit(addr, maxAmps)
    except RoboClaw.RoboClawException as inst:
      print "Failed to set motor 2 maximum current.", inst.message
      return False
    # Note: write does not seem to work on controller
    #try:
    #  motorctlr.writeSettings(addr)
    #except RoboClaw.RoboClawException as inst:
    #  print "Failed to save to EEPROM:", inst.message
    #  return False
    return True


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
      print "%s: Error: %s" % (self._Argv0, emsg)
    else:
      print "%s: Error" % (self._Argv0)
    print "Try '%s --help' for more information." % (self._Argv0)

  ## \brief Print Command-Line Usage Message.
  def printUsage(self):
    print \
"""
usage: %s [OPTIONS] SUBSYS [SUBSYS ...]
       %s --help

Laelaps sanity.

Options and arguments:
-h, --help        : Display this help and exit.

SUBSYS            : Laelaps subsystem. One of: motors
                      motors  - Motors and motor controllers

Exit Status:
  On success, 0. An exit status of 128 indicates usage 
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
    kwargs['debug']  = False
    kwargs['subsys'] = []

    # parse command-line options
    try:
      opts, args = getopt.getopt(argv[1:], "?h",
          ['help', ''])
    except getopt.error, msg:
      raise usage(msg)
    for opt, optarg in opts:
      if opt in ('-h', '--help', '-?'):
        self.printUsage()
        sys.exit(0)

    if len(args) < 1:
      self.printUsageErr("No subsystems specified.")
      sys.exit(128)

    for subsys in args:
      if subsys in SaneSubSys:
        kwargs['subsys'] += [subsys]
      else:
        self.printUsageErr("%s: Unknown/unsupported subsystem." % (subsys))
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
    except usage, e:
      print e.msg
      return 128
    
    sane = Sanity(self.kwargs)

    tf = sane.run()

    if tf:
      return 0
    else:
      return 256
      

# ------------------------------------------------------------------------------
# main
# ------------------------------------------------------------------------------
if __name__ == '__main__':
  app = application();
  sys.exit( app.run() );
