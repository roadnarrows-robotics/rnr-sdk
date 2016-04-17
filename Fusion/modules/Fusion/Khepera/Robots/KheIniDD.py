################################################################################
#
# KheIniDD.py
#

""" Virtual Khepera Robot 'Ini' Definition Dictionary.

Virtual Khepera II Robot 'Ini' specific definition dictionary.
Must conform to the Fusion Format. (See Fusion/Core/FusionIniDD.py)

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.12.16

Copyright (C) 2005, 2006.  RoadNarrows LLC.
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


import Fusion.Khepera.Robots.KheValues as kvals
import Fusion.Khepera.Cmd.KheCmdBase as KheCmdBase

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------


# vKhepera ini sections
IniDDSectOpts       = kvals.KheMimeType + '/' + 'options'
IniDDSectConn       = kvals.KheMimeType + '/' + 'connection'
IniDDSectProximity  = kvals.KheMimeType + '/' + kvals.KheSensorMimeTypeProximity
IniDDSectAmbient    = kvals.KheMimeType + '/' + kvals.KheSensorMimeTypeAmbient
IniDDSectGP2D120    = kvals.KheMimeType + '/' + kvals.KheSensorMimeTypeGP2D120


#-------------------------------------------------------------------------------
# Functions
#-------------------------------------------------------------------------------

def GetIniDD():
  """ Get virtual Khepera Ini Definition Data.

      Return Value:
        Ini definition dictionary.
  """
  iniDD = {
    # section
    IniDDSectOpts: ['Khepera II robot options',
    {
      'UseProximitySensors':    [True, 'Do [not] use proximity sensors.'],
      'UseAmbientSensors':      [False,'Do [not] use ambient sensors.'],
      'UseDistMeasSensors':     [True, 'Do [not] use dist. measuring sensors.'],
      'UseOdometrySensors':     [True, 'Do [not] use odometry sensors.'],
      'UseSpeedometerSensors':  [True, 'Do [not] use speedometer sensors.'],
      'AutoConnect':            [False,'Do [not] autoconnect after loading.'],
      'ExecCycle':              [0.10, 
        'Execution sense/react cycle time (seconds).'],
      'ExecStepSize':           [1.0, "Execution 'Step' size (seconds)."]
    }],

    IniDDSectConn: ['Khepera II robot connection settings',
    {
      'port':         [None,  'Connection port (device).'],
      'baudrate':     [9600,  'Connection baudrate.'],
      'bytesize':     [8,     'Connection bytesize.'],
      'parity':       ['N',   'Connection parity.'],
      'stopbits':     [2,     'Connection stopbits.']
    }],

    IniDDSectProximity: ['Khepera II robot Proximity IR LED sensors',
      _GetIniDDProxCal()],

    IniDDSectAmbient: ['Khepera II robot Ambient IR LED sensors',
      _GetIniDDAmbCal()],

    IniDDSectGP2D120 + '/' + 'front': ['Khepera II robot front GP2D120 sensor',
    {
      'unitsG':       ['adc', "Calibration units of GP2D120 output G. "
                              "One of: 'adc', 'volts'"],
      'unitsR':       ['mm',  'Calibration units of distance R. '
                              "One of: 'mm', 'cm', 'in'"],
      'calData':      [None,  'Calibration data list [(G1,R1), (G2,R2) ...]']
    }]
  }

  return iniDD

def _GetIniDDProxCal():
  ddsect = {}
  for id in KheCmdBase.KheIrSensorOrder:
    ddsect['prox_'+id] = [
      {
        'enabled': True,
        'k_brightness': KheCmdBase.KheIrProxDftKBrightness,
        'noise_floor': KheCmdBase.KheIrProxDftNoiseFloor
      },
      'Calibration parameters'
    ]
  return ddsect

def _GetIniDDAmbCal():
  ddsect = {}
  for id in KheCmdBase.KheIrSensorOrder:
    ddsect['amb_'+id] = [
      {
        'enabled': True,
        'k_brightness': KheCmdBase.KheIrAmbDftKBrightness,
        'noise_floor': KheCmdBase.KheIrAmbDftNoiseFloor
      },
      'Calibration parameters.'
    ]
  return ddsect
