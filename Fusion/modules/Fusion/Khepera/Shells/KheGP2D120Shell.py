#! /usr/bin/env python
################################################################################
#
# KheGP2D120Shell.py
#

""" Khepera II Base and GP2D120 Sensor Command-Line Shell Module

Khepera II GP2D120 serial command-line shell provides a command-line
interface to most of the commands available on the Khepera II base
robot plus command extensions for the Sharp GP2D120 distance measurement
IR LED sensor.

This shell is GuiTerm auto-capable.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.12.28

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

try:
  import readline   # optional module
except ImportError:
  pass

import Fusion.Gui.GuiTypes as gt
import Fusion.Gui.GuiWinShell as gterm

import Fusion.Khepera.Cmd.KheCmdGP2D120 as KheCmdGP2D120

import Fusion.Utils.Shell as Shell
import Fusion.Khepera.Shells.KheRawShell as KheRawShell
import Fusion.Khepera.Shells.KheBaseShell as KheBaseShell


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
# CLASS: KheGP2D120Shell
#-------------------------------------------------------------------------------
class KheGP2D120Shell(KheBaseShell.KheBaseShell):
  """ Khepera GP2D120 Shell Class. """

  #--
  def __init__(self, args=(), **kwargs):
    """ Initialze Raw Shell.

        Parameters:
          args    - arguments (not used)
                    kwargs  - dictionary of shell options arguments
            robotCmds   - Robot commands object. Default: KheCmdGP2D120()
            **shopts    - Shell base, raw and core options.
          
    """
    # initialize shell base object
    KheBaseShell.KheBaseShell.__init__(self, args, **kwargs)

  #--
  def Config(self, **kwargs):
    """ Configure Options Override. 

        Parameters:
          kwargs  - dictionary of options arguments
    """
    # this shell arguments defaults
    dfts = {
      'argv0': __file__,
      'shName': 'Khepera GP2D120 GenIO',
      'shVersion': '1.1',
      'shPS1': 'khegp2d120$ ',
      'shPS2': 'khegp2d120> ',
      'robotCmds': None
    }

    # set shell argument defaults, but caller has precedence
    for key, value in dfts.items():
      if key not in kwargs or not kwargs[key]:
        if key == 'robotCmds':
          kwargs[key] = KheCmdGP2D120.KheCmdGP2D120()
        else:
          kwargs[key] = value

    # set all options from arguments
    KheBaseShell.KheBaseShell.Config(self, **kwargs)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # The Command Dictionary (See Shell.py)
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def InitCmdDict(self):
    """ Initialize Command Dictionary Override.

        Return Value:
          None
    """
    # set shell base object's commands
    KheBaseShell.KheBaseShell.InitCmdDict(self)

    # this shell's additional or override commands
    cmds = {
      # measdist command
      'measdist': {
        'help': {
          'desc': "Measure the distance calculated by the calibrated "
                  "GP2D120 sensor. Measured distance is in mm.",
          'usage': "measdist [incraw]",
          'args': {
            'incraw': "Include raw sensor data."
          },
          'rvals': "Return measured distance in mm. If 'incraw' is specified, "
                   "then return 2-tuple (<mm>, <raw>)."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + KheBaseShell.ReArgIncRaw + Shell.ReEnd,
            'cvt':    [self.CvtTrue]
          },
          { 'refmt':  Shell.ReStart + Shell.ReEnd,
            'cvt':    []
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdGP2D120MeasureDist
      },

      # calgp2d120 command
      'calgp2d120': {
        'help': {
          'desc': "Get the GP2D120 calibration parameters.",
          'usage': "calgp2d120",
          'rvals': "GP2D120 calibration data."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReEnd,
            'cvt':    []
          }
        ],
        'prereq': [],
        'exec':   self.CmdGetCalGP2D120
      }
    }
  
    # now add the additional commands to the dictionary
    self.AddToCmdDict(cmds)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Shell Commands
  #   All commands must return the 2-tuple (rc, rval) where:
  #     ('err', <errstr>) or ('ok', <data)>)
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  def CmdGP2D120MeasureDist(self, incraw=False):
    """  GP2D120 Measure Distance Shell Command. """
    dist = self.mRobotCmds.CmdGP2D120MeasureDist(incraw=incraw)
    if dist is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', dist
  
  def CmdGetCalGP2D120(self):
    """ Get the GP2D120 Calibration Data Shell Command. """
    return 'ok', self.mRobotCmds.GP2D120GetCalibration()
  

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Parameter Converters 
  #   All converters must return the 2-tuple (rc, rval) where:
  #     ('err', <errstr>) or ('ok', <data)>)
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Shell Prerequisites 
  #   All prerequisite checkers must return either True (ok) or False.
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Response Post-Processors
  #   All response post-processors must print any formatted data to fout.
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Shell Callback Overrides
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


#-------------------------------------------------------------------------------
# Main
#-------------------------------------------------------------------------------

#--
if __name__ == "__main__":
  import sys

  sys.exit( KheRawShell.main( shclass=KheGP2D120Shell,
                              argv0=__file__, 
                              shName='Khepera GP2D120 GenIO'))
