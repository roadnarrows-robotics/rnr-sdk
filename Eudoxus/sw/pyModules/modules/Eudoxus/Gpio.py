#
# Module:   Eudoxus.Gpio
#
# Package:  RoadNarrows Eudoxus 3D Sensor Package
#
# Link:     https://github.com/roadnarrows-robotics/eudoxus
#
# File:     Gpio.py
#
## \file
##
## $LastChangedDate: 2016-02-02 13:47:13 -0700 (Tue, 02 Feb 2016) $  
## $Rev: 4293 $ 
## 
## \brief Eudoxus Odroid GPIO control.
##
## \author: Robin Knight (robin.knight@roadnarrows.com)
## 
## \copyright
##   \h_copy 2016-2017. RoadNarrows LLC.\n
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
Eudoxus Odroid GPIO control.
"""

import sys

from Eudoxus.SysConf import *

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
