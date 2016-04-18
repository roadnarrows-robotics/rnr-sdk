#! /usr/bin/env python

###############################################################################
#
# Package:  RoadNarrows Robotics Laelaps Robotic Mobile Platform Package
#
# File: laelaps_stop
#
## \file 
##
## $LastChangedDate: 2015-09-25 18:32:17 -0600 (Fri, 25 Sep 2015) $
## $Rev: 4109 $
##
## \brief Stop the Laelaps.
##
## \author Robin Knight (robin.knight@roadnarrows.com)
##  
## \par Copyright:
##   (C) 2015-2016.  RoadNarrows LLC.\n
##   (http://www.roadnarrows.com)\n
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

import os
import sys
import time
import getopt

import Laelaps.SysConf as SysConf
import Laelaps.WatchDog as WatchDog
import Laelaps.RoboClaw as RoboClaw

Argv0   = os.path.basename(__file__)
CliArgs = {}

##
## \brief Unit test command-line exception class.
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

#
## \brief Print 'try' message'
#
def printTry():
  print "Try '%s --help' for more information." % (Argv0)

#
## \brief Print usage error.
##
## \param emsg  Error message string.
#
def printUsageErr(emsg):
  if emsg:
    print "%s: Error: %s" % (Argv0, emsg)
  else:
    print "%s: Error" % (Argv0)
  printTry()

## \brief Print Command-Line Usage Message.
def printUsage():
    print \
"""
usage: %s [OPTIONS]
       %s --help

Stop all Laelaps motors.

Options and arguments:
-h, --help        : Display this help and exit.

Exit Status:
  Returns exits status of 0 on success. An exit status of 128 indicates usage 
  error.
"""  % (Argv0, Argv0)
 
#
## \brief Get command-line options
##  
## \param [in,out] kwargs  Command-line keyword dictionary.  
##
## \return Parsed keyword arguments.
#
def getOptions(kwargs):
  argv = sys.argv

  # defaults
  kwargs['debug'] = False

  # parse command-line options
  try:
    opts, args = getopt.getopt(argv[1:], "?h", ['help', ''])
  except getopt.error, msg:
    raise usage(msg)
  for opt, optarg in opts:
    if opt in ('-h', '--help', '-?'):
      printUsage()
      sys.exit(0)

  return kwargs

#
# Parse command-line options and arguments
#
try:
  CliArgs = getOptions(CliArgs)
except usage, e:
  printUsageErr(e.msg)
  sys.exit(128)

DoStop = True

#
# WatchDog subprocessor control the enable lines to the motor controllers.
#
wd = WatchDog.WatchDog()

if wd.open(SysConf.SensorDevName):
  wd.cmdGetFwVersion()
  ret = wd.cmdReadEnables()
  if ret['rc'] == 'ok':
    if not ret['enables']['motor_ctlr_en']:
      print "Motor controllers already disabled, nothing to stop."
      DoStop = False
else:
  print 'Error: Failed to open connection to watchdog subprocessor.'
  print '       Continuing...'


if DoStop:
  motorctlr = RoboClaw.RoboClaw()

  #
  # Stop all motors.
  #
  try:
    motorctlr.open(SysConf.MotorCtlrDevName, SysConf.MotorCtlrBaudRate)
    motorctlr.setMixedSpeed(SysConf.MotorCtlrAddrFront, 0, 0)
    motorctlr.setMixedSpeed(SysConf.MotorCtlrAddrRear,  0, 0)
    motorctlr.close()
  except RoboClaw.RoboClawException as inst:
    print 'Error:', inst.message

if wd.isOpen():
  wd.close()
