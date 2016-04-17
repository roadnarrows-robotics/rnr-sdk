################################################################################
#
# KheValues.py
#

""" Virtual Khepera Robot Global Values Module

Virtual Khepera II Robot global data values module.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.02.01

Copyright (C) 2006.  RoadNarrows LLC.
"""

#
# All Rights Reserved
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
################################################################################


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

#
# MIME Types
# 

# vKhepera Robot MIME Types
KheMimeType                   = 'robot/KheperaII'

# Sensor MIME SubTypes
KheSensorMimeTypeProximity    = 'sensor/reflective-irled'
KheSensorMimeTypeAmbient      = 'sensor/ambient-irled'
KheSensorMimeTypeDistMeas     = 'sensor/distmeas-irled'
KheSensorMimeTypeGP2D120      = KheSensorMimeTypeDistMeas + '/GP2D120'
KheSensorMimeTypeOdometer     = 'sensor/odometer-pwm'
KheSensorMimeTypeSpeedometer  = 'sensor/speedometer-pwm'

# Derived Sensor MIME SubTypes
KheSensorMimeTypeTime         = 'sensor/time'

# Effector MIME SubTypes
KheEffectorMimeTypeWheelMotor = 'effector/wheel-motor-pwm' 
KheEffectorMimeTypeLed        = 'effector/led-user' 


#-------------------------------------------------------------------------------
# Functions
#-------------------------------------------------------------------------------

