#
# Module:   Laelaps.SysConf
#
# Package:  RoadNarrows Laelaps Robotic Mobile Platform Package
#
# Link:     https://github.com/roadnarrows-robotics/laelaps
#
# File:     SysConf.py
#
## \file
##
## $LastChangedDate: 2016-02-02 13:47:13 -0700 (Tue, 02 Feb 2016) $  
## $Rev: 4293 $ 
## 
## \brief Laelaps system configuration.
##
## \author: Robin Knight (robin.knight@roadnarrows.com)
## 
## \par Copyright:
##   (C) 2015-2016.  RoadNarrows LLC.
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
Laelaps system configuration.
"""

import os

# . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
# Laelaps GPIO
# . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

"""
The internal odroid GPIO pins are used to enable/disable/reset various hardware
subsystems. The exported numbers controll are found under /sys/class/gpio.
"""

## Motor controller chip select (deprecated)
##
## States:  0 = select rear motor controller
##          1 = select front motor controller
GpioMotorCtlrCs   = 173


## Motor controller enable.
##
## States:  0 = disable power to both motor controllers
##          1 = enable power to both motor controllers
GpioMotorCtlrEn   = 174

## Watchdog sub-processor reset.
##
## States:  1 to 0 = edge trigger to reset Arduino sub-processor. 
GpioWdReset       = 189

## I2C multiplexer reset.
##
## States:  1 to 0 = edge trigger to reset I2C multiplexer. 
GpioI2CMuxReset   = 190

## Top deck battery out enable.
##
## States:  0 = disable battery power out to top deck port.
##          1 = enable battery power out to top deck port.
GpioDeckBattEn    = 191

## Top deck regulated 5V out enable.
##
## States:  0 = disable 5V power out to top deck port.
##          1 = enable 5V power out to top deck port.
GpioDeck5VEn      = 192


# . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
# Laelaps Motor Controllers
# . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

"""
There are two motor controllers connected to a shared serial device (i.e. The
motor controllers are multi-dropped). Each motor controller has an unique 8-bit
address. 

For Laelaps, the front and rear motor controllers control the front and rear
two motors, respectively.
"""

MotorCtlrBoard      = "RoboClaw"        ## motor controller board
MotorCtlrFirmware   = "Ion"             ## motor controller firmware
MotorCtlrDevName    = "/dev/ttySAC0"    ## serial device name
MotorCtlrSymName    = "/dev/motorctlrs" ## serial device symbolic linked name
MotorCtlrBaudRate   = 115200            ## serial device baud rate
MotorCtlrAddrFront  = 0x80              ## front motor controller address
MotorCtlrAddrRear   = 0x81              ## rear motor controller address


# . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
# Laelaps IMU
# . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

ImuBoard      = "Naze32"        ## IMU board
ImuFirmware   = "CleanFlight"   ## IMU firmware
ImuDevName    = "/dev/ttyUSB0"  ## IMU serial USB device name
ImuSymName    = "/dev/imu"      ## IMU serial USB symbolic linked name
ImuBaudRate   = 115200          ## IMU serial baud rate


# . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
# Sensor I2C Bus 
# . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

SensorDevName = "/dev/i2c-3"         ## Sensors device
SensorSymName = "/dev/i2c-sensors"   ## Sensors symbolic linked name


# . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
# Time-of-Flight Sensors
# . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .


# . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
# WatchDog Arduino Sub-Processor
# . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .

WdBoard      = "Uno"          ## Watchdog board
WdFirmware   = "RoadNarrows"  ## Watchdog firmware
