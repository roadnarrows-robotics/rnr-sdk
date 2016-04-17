################################################################################
#
# HemiIniDD.py
#

""" Virtual Hemisson Robot 'Ini' Definition Dictionary.

Virtual Hemisson Robot 'Ini' specific definition dictionary.
Must conform to the Fusion Format. (See Fusion/Core/FusionIniDD.py)

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.03.07

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


import Fusion.Hemisson.Robots.HemiValues as hvals
import Fusion.Hemisson.Cmd.HemiCmdBase as HemiBase
import Fusion.Hemisson.Cmd.HemiCmdLinCam as HemiLinCam
import Fusion.Hemisson.Cmd.HemiCmdTts as HemiTts
import Fusion.Hemisson.Cmd.HemiCmdUss as HemiUss

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------


# vHemisson ini sections
IniDDSectOpts       = hvals.HemiMimeType + '/' + 'options'
IniDDSectConn       = hvals.HemiMimeType + '/' + 'connection'
IniDDSectProximity  = hvals.HemiMimeType + '/' + \
                                              hvals.HemiSensorMimeTypeProximity
IniDDSectAmbient    = hvals.HemiMimeType + '/' + hvals.HemiSensorMimeTypeAmbient
IniDDSectLinCam     = hvals.HemiMimeType + '/' + hvals.HemiSensorMimeTypeLinCam
IniDDSectUss        = hvals.HemiMimeType + '/' + hvals.HemiSensorMimeTypeUss
IniDDSectTts        = hvals.HemiMimeType + '/' + hvals.HemiEffectorMimeTypeTts


#-------------------------------------------------------------------------------
# Functions
#-------------------------------------------------------------------------------

def GetIniDD():
  """ Get virtual Hemisson Ini Definition Data.

      Return Value:
        Ini definition dictionary.
  """
  iniDD = {
    # section
    IniDDSectOpts: ['Hemisson robot options',
    {
      'UseProximitySensors':    [True, 'Do [not] use proximity sensors.'],
      'UseAmbientSensors':      [False,'Do [not] use ambient sensors.'],
      'UseSpeedometerSensors':  [True, 'Do [not] use speedometer sensors.'],
      'UseLinCamSensor':        [False, 'Do [not] use linear camara.'],
      'UseUssSensor':           [False, 'Do [not] use ultrasonic sensor.'],
      'AutoConnect':            [False,'Do [not] autoconnect after loading.'],
      'ExecCycle':              [0.20, 
        'Execution sense/react cycle time (seconds).'],
      'ExecStepSize':           [1.0, "Execution 'Step' size (seconds)."]
    }],

    IniDDSectConn: ['Hemisson robot connection settings',
    {
      'port':         [None,    'Connection port (device).'],
      'baudrate':     [115200,  'Connection baudrate.'],
      'bytesize':     [8,       'Connection bytesize.'],
      'parity':       ['N',     'Connection parity.'],
      'stopbits':     [1,       'Connection stopbits.']
    }],

    IniDDSectProximity: ['Hemisson robot Proximity IR LED sensors',
      _GetIniDDProxCal()],

    IniDDSectAmbient: ['Hemisson robot Ambient IR LED sensors',
      _GetIniDDAmbCal()],

    IniDDSectLinCam: ['Hemisson robot linear camera module settings',
    {
      'exposure':   [HemiLinCam.LinCamExposureTimeDft, 'Exposure time (msec).'],
      'threshold':  [HemiLinCam.LinCamThresholdDft,
                        'Q intensity threshold (gray-level).'],
    }],

    IniDDSectTts: ['Hemisson robot text-to-speech module settings',
    {
      'gain':   [HemiTts.TtsSpeakerGainDft, 'Speaker gain.'],
      'pitch':  [HemiTts.TtsVoicePitchDft, 'Text conversion voice pitch.'],
      'rate':   [HemiTts.TtsVoiceRateDft,
                  'Text conversion voice speaking rate.'],
    }],

    IniDDSectUss: ['Hemisson robot ultrasonic sensor module settings',
    {
      'range':    [6000, 'Maximum range (mm).'],
      'units':    ['mm', 'Measurement units'],
    }],

  }

  return iniDD

def _GetIniDDProxCal():
  ddsect = {}
  for id in HemiBase.HemiIrSensorOrder:
    ddsect['prox_'+id] = [
      {
        'enabled': True,
        'k_brightness': HemiBase.HemiIrProxDftKBrightness,
        'noise_floor': HemiBase.HemiIrProxDftNoiseFloor
      },
      'Calibration parameters'
    ]
  return ddsect

def _GetIniDDAmbCal():
  ddsect = {}
  for id in HemiBase.HemiIrSensorOrder:
    ddsect['amb_'+id] = [
      {
        'enabled': True,
        'k_brightness': HemiBase.HemiIrAmbDftKBrightness,
        'noise_floor': HemiBase.HemiIrAmbDftNoiseFloor
      },
      'Calibration parameters.'
    ]
  return ddsect
