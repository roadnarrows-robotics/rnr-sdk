#! /usr/bin/env python
################################################################################
#
# HemiBaseShell.py
#

""" Hemisson Base Command-Line Shell Module

Hemisson base serial command-line shell provides a command-line
interface to most of the commands available on the Hemisson
base robot.

This shell is GuiTerm auto-capable.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.08.24

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

import Fusion.Hemisson.Cmd.HemiCmdBase as HemiCmdBase

import Fusion.Utils.Shell as Shell
import Fusion.Hemisson.Shells.HemiRawShell as HemiRawShell


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

# command arguments
ReArgIncRaw = Shell.ReX('incraw')


#-------------------------------------------------------------------------------
# CLASS: HemiBaseShell
#-------------------------------------------------------------------------------
class HemiBaseShell(HemiRawShell.HemiRawShell):
  """ Hemisson Base Shell Class. """

  #--
  def __init__(self, args=(), **kwargs):
    """ Initialze Shell.

        Parameters:
          args    - arguments (not used)
          kwargs  - dictionary of shell options arguments
            robotCmds   - Robot commands object. Default: HemiCmdBase()
            **shopts    - Shell raw and core options.
          
    """
    # initialize shell base object
    HemiRawShell.HemiRawShell.__init__(self, args, **kwargs)

  #--
  def Config(self, **kwargs):
    """ Configure Options Override. 

        Parameters:
          kwargs  - dictionary of options arguments
    """
    # this shell arguments defaults
    dfts = {
      'argv0': __file__,
      'shName': 'Hemisson Base',
      'shVersion': '1.1',
      'shPS1': 'hemibase$ ',
      'shPS2': 'hemibase> ',
      'robotCmds': None
    }

    # set shell argument defaults, but caller has precedence
    for key, value in dfts.iteritems():
      if not kwargs.has_key(key) or not kwargs[key]:
        if key == 'robotCmds':
          kwargs[key] = HemiCmdBase.HemiCmdBase()
        else:
          kwargs[key] = value

    # set all options from arguments
    HemiRawShell.HemiRawShell.Config(self, **kwargs)


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
    HemiRawShell.HemiRawShell.InitCmdDict(self)

    # this shell's additional or override commands
    cmds = {
      # version command
      'version': {
        'help': {
          'desc': "Get Hemisson OS version.",
          'usage': "version",
          'rvals': "OS version string."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReEnd,
            'cvt':    []
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdGetVersion
      },

      # go command
      'go': {
        'help': {
          'desc': "Move Hemisson at the specified speed.",
          'usage': "go <motorLeft> <motorRight>",
          'args': {
            '<motorLeft>':  "Left motor speed [-9, 9]",
            '<motorRight>': "Right motor speed [-9, 9]"
          },
          'rvals': "New speed 2-tuple (<motorLeft>, <motorRight>)."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReArgInt + Shell.ReArgInt +
                      Shell.ReEnd,
            'cvt':    [self.CvtInt]
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdSetSpeed
      },

      # stop command
      'stop': {
        'help': {
          'desc': "Stop Hemisson's motion.",
          'usage': "stop",
          'rvals': "Stopped speed 2-tuple (0, 0)."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReEnd,
            'cvt':    []
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdStop
      },

      # forward command
      'forward': {
        'help': {
          'desc': "Move Hemisson forward at the specified speed.",
          'usage': "forward <motor>",
          'args': {
            '<motor>': "Left and right motor speeds [0, 9] (0 = stop)",
          },
          'rvals': "New speed 2-tuple (<motor>, <motor>)."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReArgPosInt + Shell.ReEnd,
            'cvt':    [self.CvtInt]
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdForward
      },

      # backward command
      'backward': {
        'help': {
          'desc': "Move Hemisson backward at the specified speed.",
          'usage': "backward <motor>",
          'args': {
            '<motor>': "Left and right motor speeds [0, 9] (0 = stop)",
          },
          'rvals': "New speed 2-tuple (-<motor>, -<motor>)."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReArgPosInt + Shell.ReEnd,
            'cvt':    [self.CvtInt]
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdBackward
      },

      # twirl command
      'twirl': {
        'help': {
          'desc': "Twirl Hemisson about its z-axis at the specified speed.",
          'usage': "twirl <motor>",
          'args': {
            '<motor>':  "Twirl speed [-9, 9]",
          },
          'rvals': "New speed 2-tuple (<motor>, -<motor>)."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReArgInt + Shell.ReEnd,
            'cvt':    [self.CvtInt]
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdTwirl
      },

      # speed command
      'speed': {
        'help': {
          'desc': "Get Hemisson's current speed.",
          'usage': "speed",
          'rvals': "Current speed 2-tuple (<motorLeft>, <motorRight>)."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReEnd,
            'cvt':    []
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdGetSpeed
      },

      # proximity command
      'proximity': {
        'help': {
          'desc': "Read Hemisson proximity IR LED sensors.",
          'usage': "proximity [incraw]",
          'args': {
            'incraw':  "Include raw proximity data."
          },
          'rvals': "Dictionary of read proximity sensors calibrated distances "
                   "in mm. If 'incraw' is specified, each sensor value is a "
                   "2-tuple (<mm>, <raw>)."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + ReArgIncRaw + Shell.ReEnd,
            'cvt':    [self.CvtTrue]
          },
          { 'refmt':  Shell.ReStart + Shell.ReEnd,
            'cvt':    []
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdReadProximitySensors
      },

      # ambient command
      'ambient': {
        'help': {
          'desc': "Read Hemisson ambient IR LED sensors.",
          'usage': "ambient [incraw]",
          'args': {
            'incraw':  "Include raw ambient data."
          },
          'rvals': "Dictionary of read ambient sensors calibrated distances "
                   "in mm. If 'incraw' is specified, each sensor value is a "
                   "2-tuple (<mm>, <raw>)."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + ReArgIncRaw + Shell.ReEnd,
            'cvt':    [self.CvtTrue]
          },
          { 'refmt':  Shell.ReStart + Shell.ReEnd,
            'cvt':    []
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdReadAmbientSensors
      },

      # calproximity command
      'calproximity': {
        'help': {
          'desc': "Get proximity sensors' calibration parameters.",
          'usage': "calproximity",
          'rvals': "Proximity calibration data."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReEnd,
            'cvt':    []
          }
        ],
        'prereq': [],
        'exec':   self.CmdGetCalProximity
      },

      # calambient command
      'calambient': {
        'help': {
          'desc': "Get ambient sensors' calibration parameters.",
          'usage': "calambient",
          'rvals': "Ambient calibration data."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReEnd,
            'cvt':    []
          }
        ],
        'prereq': [],
        'exec':   self.CmdGetCalAmbient
      },

      # beep command
      'beep': {
        'help': {
          'desc': "Turn Hemisson's buzzer on/off.",
          'usage': "beep <state>",
          'args': {
            '<state>': "Buzzer state. One of: on off.",
          },
          'rvals': "New buzzer state.."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReArgOnOff + Shell.ReEnd,
            'cvt':    [self.CvtOnOff]
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdSetBeep
      },

      # led command
      'led': {
        'help': {
          'desc': "Set Hemisson's left and right LED states.",
          'usage': "led <ledLeft> <ledRight>",
          'args': {
            '<ledLeft>': "Left LED state. One of: on off.",
            '<ledRight>': "Right LED state. One of: on off.",
          },
          'rvals': "New LED states 2-tuple (<ledLeft>, <ledRigth>)."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReArgOnOff + Shell.ReArgOnOff + 
                      Shell.ReEnd,
            'cvt':    [self.CvtOnOff]
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdSetBaseLedState
      },

      # scan command
      'scan': {
        'help': {
          'desc': "Scan for Hemisson I2C modules.",
          'usage': "scan",
          'rvals': "List of detected I2C modules."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReEnd,
            'cvt':    []
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdScanForModules
      },
    }
  
    # now add the additional commands to the dictionary
    self.AddToCmdDict(cmds)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Shell Commands
  #   All commands must return the 2-tuple (rc, rval) where:
  #     ('err', <errstr>) or ('ok', <data)>)
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def CmdGetVersion(self):
    """ Get Hemisson Version Shell Command. """
    ver = self.mRobotCmds.CmdGetVersion()
    if ver is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', ver
  
  #--
  def CmdGetSpeed(self):
    """ Get Hemisson Speed Shell Command. """
    speed = self.mRobotCmds.CmdGetSpeed()
    if speed is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', speed
  
  #--
  def CmdSetSpeed(self, motorLeft, motorRight):
    """ Set Hemisson Speed Shell Command. """
    speed = self.mRobotCmds.CmdSetSpeed(motorLeft, motorRight)
    if speed is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', speed
  
  #--
  def CmdStop(self):
    """ Stop the Hemisson Motion Shell Command. """
    return self.CmdSetSpeed(0, 0)
  
  #--
  def CmdForward(self, motor):
    """ Move the Hemisson Forward Shell Command. """
    if motor < 0:
      motor = -motor
    return self.CmdSetSpeed(motor, motor)
  
  #--
  def CmdBackward(self, motor):
    """ Move the Hemisson Backward Shell Command. """
    if motor < 0:
      motor = -motor
    return self.CmdSetSpeed(-motor, -motor)
  
  #--
  def CmdTwirl(self, motor):
    """ Move the Hemisson Backward Shell Command. """
    return self.CmdSetSpeed(motor, -motor)
  
  #--
  def CmdSetBeep(self, state):
    """ Turn Hemisson buzzer on/off. """
    state = self.mRobotCmds.CmdSetBeep(state)
    if state is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', state

  #--
  def CmdSetBaseLedState(self, ledLeft, ledRight):
    """ Set Hemisson Base User LED State Shell Command. """
    state = self.mRobotCmds.CmdSetBaseLedStates(ledLeft, ledRight)
    if state is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', (ledLeft, ledRight)
  
  #--
  def CmdReadProximitySensors(self, incraw=False):
    """ Get Hemisson Proximity IR LED Sensors Shell Command. """
    sensors = self.mRobotCmds.CmdReadProximitySensors(incraw=incraw)
    if sensors is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', sensors
  
  #--
  def CmdReadAmbientSensors(self, incraw=False):
    """ Get Hemisson Ambient IR LED Sensors Shell Command. """
    sensors = self.mRobotCmds.CmdReadAmbientSensors(incraw=incraw)
    if sensors is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', sensors
  
  #--
  def CmdGetCalProximity(self):
    """ Get proximity sensors' calibration parameters Shell Command. """
    cal = self.mRobotCmds.ProximitySensorsGetCalParams()
    return 'ok', cal

  #--
  def CmdGetCalAmbient(self):
    """ Set ambient sensors' calibration parameters Shell Command. """
    cal = self.mRobotCmds.AmbientSensorsGetCalParams()
    return 'ok', cal

  #--
  def CmdScanForModules(self):
    """ Scan for attached and supported Hemisson modules. """
    rsp = self.mRobotCmds.CmdScanForModules()
    if rsp is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', rsp
  

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

  sys.exit( HemiRawShell.main(shclass=HemiBaseShell,
                              argv0=__file__,
                              shName="Hemisson Base") )
