#
# Module:   Eudoxus.SysConf
#
# Package:  RoadNarrows Eudoxus 3D Sensor Package
#
# Link:     https://github.com/roadnarrows-robotics/eudoxus
#
# File:     SysConf.py
#
## \file
##
## $LastChangedDate: 2016-02-02 13:47:13 -0700 (Tue, 02 Feb 2016) $  
## $Rev: 4293 $ 
## 
## \brief Eudoxus system configuration.
##
## \author: Robin Knight (robin.knight@roadnarrows.com)
## 
## \par Copyright:
##   (C) 2016.  RoadNarrows LLC.
##   (http://www.roadnarrows.com)
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
Eudoxus system configuration.
"""

import os

# . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
# Eudoxus GPIO
# . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

"""
The internal odroid GPIO pins are used to enable/disable/reset various hardware
subsystems. The exported numbers controll are found under /sys/class/gpio.
"""

## User status LED
##
## States:  0 = off
##          1 = on
GpioUserLed = 18


## User push button
##
## States:  0 = pushed
##          1 = released
GpioMotorCtlrEn = 19
