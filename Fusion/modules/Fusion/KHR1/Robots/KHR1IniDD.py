################################################################################
#
# KHR1IniDD.py
#

""" Virtual KHR-1 Robot 'Ini' Definition Dictionary.

Virtual KHR-1 Robot 'Ini' specific definition dictionary.
Must conform to the Fusion Format. (See Fusion/Core/FusionIniDD.py)

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2007.01.07

Copyright (C) 2007.  RoadNarrows LLC.
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


import Fusion.KHR1.Robots.KHR1Values as kvals
import Fusion.KHR1.Cmd.KHR1CmdBase as KHR1CmdBase

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------


# vHemisson ini sections
IniDDSectOpts       = kvals.KHR1MimeType + '/' + 'options'
IniDDSectConn       = kvals.KHR1MimeType + '/' + 'connection'
IniDDSectChan       = kvals.KHR1MimeType + '/' + 'channels'
IniDDSectTrim       = kvals.KHR1MimeType + '/' + 'trim'


#-------------------------------------------------------------------------------
# Functions
#-------------------------------------------------------------------------------

#--
def GetIniDD():
  """ Get virtual KHR-1 Ini Definition Data.

      Return Value:
        Ini definition dictionary.
  """
  iniDD = {
    # section
    IniDDSectOpts: ['KHR-1 robot options',
    {
      'board_ids':        [KHR1CmdBase.KHR1BoardIdListDft, 'KHR-1 Board IDs'],
      'servo_version':    [KHR1CmdBase.RCB1ServoVersionBlue, 'Servo version'],
      'AutoConnect':      [False,'Do [not] autoconnect after loading.'],
      'ExecCycle':        [0.20, 'Execution sense/react cycle time (seconds).'],
      'ExecStepSize':     [1.0, "Execution 'Step' size (seconds)."]
    }],

    IniDDSectConn: ['KHR-1 robot connection settings',
    {
      'port':         [None,    'Connection port (device).'],
      'baudrate':     [115200,  'Connection baudrate.'],
      'bytesize':     [8,       'Connection bytesize.'],
      'parity':       ['N',     'Connection parity.'],
      'stopbits':     [1,       'Connection stopbits.']
    }],

    IniDDSectChan: ['KHR-1 robot active servos',
    {
      'active_channels': [_GetIniDDActiveServos(), 'Active channels']
    }],

    IniDDSectTrim: ['KHR-1 robot servo trim',
    {
      'trim': [KHR1CmdBase.KHR1FacDftTrim, 'Servo trim list']
    }]
  }

  return iniDD

#--
def _GetIniDDActiveServos():
  ddsect = {}
  for mnem,chan in KHR1CmdBase.KHR1FacDftActiveServos.iteritems():
    ddsect[mnem] = chan
  return ddsect
