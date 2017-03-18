#
# Module:   Laelaps.Gpio
#
# Package:  RoadNarrows Laelaps Robotic Mobile Platform Package
#
# Link:     https://github.com/roadnarrows-robotics/laelaps
#
# File:     Gpio.py
#
## \file
##
## $LastChangedDate: 2016-02-02 13:47:13 -0700 (Tue, 02 Feb 2016) $  
## $Rev: 4293 $ 
## 
## \brief Laelaps Odroid GPIO control.
##
## \author: Robin Knight (robin.knight@roadnarrows.com)
## 
## \par Copyright
##   \h_copy 2015-2017. RoadNarrows LLC.\n
##   http://www.roadnarrows.com\n
##   All Rights Reserved
#
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

"""
Laelaps Odroid GPIO control.
"""

import sys

from Laelaps.SysConf import *

#
## \brief Read value of a GPIO pin.
##
## \param gpio    Exported GPIO number.
##
## \return On success returns '0' or '1'. On error, '' is returned.
#
def gpioReadValue(gpio):
  valFileName = "/sys/class/gpio/gpio%d/value" % (gpio)
  try:
    fp = open(valFileName, "r")
  except IOError as inst:
    print >>sys.stderr, "Error:", inst
    return ''
  try:
    val = fp.read(1)
  except IOError as inst:
    print >>sys.stderr, "Error:", inst
    val = ''
  fp.close()
  return val

#
## \brief Write a value to a GPIO pin.
##
## \param gpio    Exported GPIO number.
## \param value   Value to write.
##                If '0', 0, or False, value written is '0'.
##                If '1', non-zero, or True, value written is '1'.
##
## \return On success returns '0' or '1'. On error, '' is returned.
#
def gpioWriteValue(gpio, value):
  valFileName = "/sys/class/gpio/gpio%d/value" % (gpio)
  try:
    fp = open(valFileName, "w")
  except IOError as inst:
    print >>sys.stderr, "Error:", inst
    return ''
  if (value == '0') or (value == '1'):
    val = value
  elif value:
    val = '1'
  else:
    val = '0'
  try:
    val = fp.write(val+'\n')
  except IOError as inst:
    print >>sys.stderr, "Error:", inst
    val = ''
  fp.close()
  return val

#
## \brief Laelaps specific chip select function using digital GPIO.
##
## \note Deprecated
##
## The Laelaps uses GPIO 173 to select the front or rear motor controllers.
## For the Odroid, this pin is also the RTS line. Hardware flow control is not
## used, so this pin is available for signalling.
##
## \param port      Opened serial port (not used).
## \param addrSel   Address of motor controller to be selected.
## \param addrLast  Address of last motor controller selected.
#
def MotorCtlrChipSelectGpio(port, addrSel, addrLast):
  if addrSel != addrLast:
    if addrSel == MotorCtlrAddrFront:
      val = 1
    else:
      val = 0
    gpioWriteValue(GpioMotorCtlrCs, val);

#
## \brief Laelaps specific chip select function using RTS.
##
## \note Deprecated
##
## \note The RTS signal in current Odroid distro does not work, so this function
## is useless.
##
## \param port      Opened serial port.
## \param addrSel   Address of motor controller to be selected.
## \param addrLast  Address of last motor controller selected.
#
def MotorCtlrChipselectRts(port, addrSel, addrLast):
  if addrSel != addrLast:
    if addrSel == MotorCtlrAddrFront:
      level = True
    else:
      level = False
    port.setRTS(level)

#
## \brief Enable/disable power to Laelaps motor controllers.
##
## \param state   True to enable, False to disable.
##
def enableMotorCtlrsPower(state):
  gpioWriteValue(GpioMotorCtlrEn, state);

#
## \brief Test if the motor controllers are powered
##
## \return Returns True if powered, False if unpowered.
##
def areMotorCtlrsPowered():
  if gpioReadValue(GpioMotorCtlrEn) == '1':
    return True
  else:
    return False
