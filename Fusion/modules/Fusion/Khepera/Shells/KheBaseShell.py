#! /usr/bin/env python
################################################################################
#
# KheBaseShell.py
#

""" Khepera II Base Command-Line Shell Module

Khepera II base serial command-line shell provides a command-line
interface to most of the commands available on the Khepera II
base robot.

This shell is GuiTerm auto-capable.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.12.24

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

import Fusion.Khepera.Cmd.KheCmdBase as KheCmdBase

import Fusion.Utils.Shell as Shell
import Fusion.Khepera.Shells.KheRawShell as KheRawShell


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

# command arguments
ReArgIncRaw = Shell.ReX('incraw')


#-------------------------------------------------------------------------------
# CLASS: KheBaseShell
#-------------------------------------------------------------------------------
class KheBaseShell(KheRawShell.KheRawShell):
  """ Khepera Base Shell Class. """

  #--
  def __init__(self, args=(), **kwargs):
    """ Initialze Raw Shell.

        Parameters:
          args    - arguments (not used)
          kwargs  - dictionary of shell options arguments
            robotCmds   - Robot commands object. Default: KheCmdBase()
            **shopts    - Shell raw and core options.
          
    """
    # initialize shell base object
    KheRawShell.KheRawShell.__init__(self, args, **kwargs)

  #--
  def Config(self, **kwargs):
    """ Configure Options Override. 

        Parameters:
          kwargs  - dictionary of options arguments
    """
    # this shell arguments defaults
    dfts = {
      'argv0': __file__,
      'shName': 'Khepera Base',
      'shVersion': '1.1',
      'shPS1': 'khebase$ ',
      'shPS2': 'khebase> ',
      'robotCmds': None
    }

    # set shell argument defaults, but caller has precedence
    for key, value in dfts.items():
      if key not in kwargs or not kwargs[key]:
        if key == 'robotCmds':
          kwargs[key] = KheCmdBase.KheCmdBase()
        else:
          kwargs[key] = value

    # set all options from arguments
    KheRawShell.KheRawShell.Config(self, **kwargs)


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
    KheRawShell.KheRawShell.InitCmdDict(self)

    # this shell's additional or override commands
    cmds = {
      # version command
      'version': {
        'help': {
          'desc': "Get Khepera OS version.",
          'usage': "version",
          'rvals': "OS version 2-tuple (<verBios>, <verProtocol>)."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReEnd,
            'cvt':    []
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdGetVersion
      },

      # setspeed command
      'setspeed': {
        'help': {
          'desc': "Set Khepera's speed.",
          'usage': "setspeed <motorLeft> <motorRight>",
          'args': {
            '<motorLeft>':  "Left motor speed in mm/s [-1000.0, 1000.0]",
            '<motorRight>': "Right motor speed in mm/s [-1000.0, 1000.0]"
          },
          'rvals': "New speed 2-tuple (<motorLeft>, <motorRight>)."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReArgFloat + Shell.ReArgFloat +
                      Shell.ReEnd,
            'cvt':    [self.CvtFloat]
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdSetSpeed
      },

      # stop command
      'stop': {
        'help': {
          'desc': "Stop Khepera's motion.",
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
          'desc': "Move Khepera forward at the specified speed.",
          'usage': "forward <motor>",
          'args': {
            '<motor>': "Left and right motor speeds in mm/s [0.0, 1000.0].",
          },
          'rvals': "New speed 2-tuple (<motor>, <motor>)."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReArgFloat + Shell.ReEnd,
            'cvt':    [self.CvtFloat]
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdForward
      },

      # backward command
      'backward': {
        'help': {
          'desc': "Move Khepera backward at the specified speed.",
          'usage': "backward <motor>",
          'args': {
            '<motor>': "Left and right motor speeds [0.0, 1000.0].",
          },
          'rvals': "New speed 2-tuple (-<motor>, -<motor>)."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReArgFloat + Shell.ReEnd,
            'cvt':    [self.CvtFloat]
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdBackward
      },

      # speed command
      'speed': {
        'help': {
          'desc': "Get Khepera's current speed.",
          'usage': "speed [incraw]",
          'args': {
            'incraw': "Include raw speed data."
          },
          'rvals': ["Current speed 2-tuple in mm/s "
                    "(<motorLeft>, <motorRight>).",
                    "If 'incraw' is specified, then include raw speed data:",
                    "  (<motorLeft>, <motorRight>, <rawLeft>, <rawRight>)."]
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
        'exec':   self.CmdGetSpeed
      },

      # moveto command
      'moveto': {
        'help': {
          'desc': "Move Khepera to the given odometer positions.",
          'usage': "moveto <posLeft> <posRight>",
          'args': {
            '<posLeft>':  "Target left motor position in mm [0.0, 671088.0]",
            '<posRight>': "Target right motor position in mm [0.0, 671088.0]"
          },
          'rvals': "Target odometer positions (<posLeft>, <posRight>)."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReArgFloat + Shell.ReArgFloat +
                      Shell.ReEnd,
            'cvt':    [self.CvtFloat]
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdMoveToPos
      },

      # setodometry command
      'setodometry': {
        'help': {
          'desc': "Set Khepera odometry.",
          'usage': "setodometry <posLeft> <posRight>",
          'args': {
            '<posLeft>':  "Left motor position in mm [0.0, 343597384.0]",
            '<posRight>': "Right motor position in mm [0.0, 343597384.0]"
          },
          'rvals': "New odometer positions (<posLeft>, <posRight>)."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReArgFloat + Shell.ReArgFloat +
                      Shell.ReEnd,
            'cvt':    [self.CvtFloat]
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdSetOdometry
      },

      # odometry command
      'odometry': {
        'help': {
          'desc': "Get Khepera's odometry.",
          'usage': "odometry [incraw]",
          'args': {
            'incraw': "Include raw odometry data."
          },
          'rvals': ["Current odometry 2-tuple in mm (<posLeft>, <posRight>).",
                   "If 'incraw' is specified, then include raw odometry data:",
                   "  (<posLeft>, <posRight>, <rawLeft>, <rawRight>)."]
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
        'exec':   self.CmdGetOdometry
      },

      # proximity command
      'proximity': {
        'help': {
          'desc': "Read Khepera proximity IR LED sensors.",
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
          'desc': "Read Khepera ambient IR LED sensors.",
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

      # adc command
      'adc': {
        'help': {
          'desc': "Read the Analog to Digital Converter channel.",
          'usage': "adc <channel>",
          'args': {
            '<channel>':  "Channel number [0, 5]."
          },
          'rvals': "ADC value."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReArgPosInt + Shell.ReEnd,
            'cvt':    [self.CvtInt]
          }
        ],
        'prereq': [],
        'exec':   self.CmdReadAdc
      },

      # led command
      'led': {
        'help': {
          'desc': "Set Khepera user LED state.",
          'usage': "led <led_num> <state>",
          'args': {
            '<led_num>': "User LED number [0, 1].",
            '<state>': "LED state. One of: on off.",
          },
          'rvals': "New LED state."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReArgPosInt + Shell.ReArgOnOff + 
                      Shell.ReEnd,
            'cvt':    [self.CvtInt, self.CvtOnOff]
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdSetBaseLedState
      },

      # extbusread command
      'extbusread': {
        'help': {
          'desc': "Read a byte from the Khepera extension bus at the given "
                   "bus address.",
          'usage': "extbusread <addr>",
          'args': {
            '<addr>':  "Bus address [0, 63]."
          },
          'rvals': "Read byte value."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReArgPosInt + Shell.ReEnd,
            'cvt':    [self.CvtInt]
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdExtBusRead
      },

      # extbuswrite command
      'extbuswrite': {
        'help': {
          'desc': "Write a byte to the Khepera extension bus at the given "
                   "bus address.",
          'usage': "extbuswrite <addr> <byte>",
          'args': {
            '<addr>': "Bus address [0, 63].",
            '<byte>': "Byte value."
          },
          'rvals': "Written byte value."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReArgPosInt + Shell.ReArgPosInt +
                      Shell.ReEnd,
            'cvt':    [self.CvtInt]
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdExtBusWrite
      },

      # turret command
      'turret': {
        'help': {
          'desc': "Write a a command to a Khepera turret.",
          'usage': "turret <tid> <tcmd>",
          'args': {
            '<tid>': "Turret id [0, 31].",
            '<tcmd>': "Turret specific command string."
          },
          'rvals': "Turret command specific response string."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReArgPosInt + Shell.ReArgStr +
                      Shell.ReEnd,
            'cvt':    [self.CvtInt, self.CvtStr]
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdTurret
      }
    }
  
    # now add the additional commands to the dictionary
    self.AddToCmdDict(cmds)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Shell Commands
  #   All commands must return the 2-tuple (rc, rval) where:
  #     ('err', <errstr>) or ('ok', <data)>)
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  def CmdGetVersion(self):
    """ Get Khepera Version Shell Command. """
    ver = self.mRobotCmds.CmdGetVersion()
    if ver is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', ver
  
  def CmdGetSpeed(self, incraw=False):
    """ Get Khepera Speed Shell Command. """
    speed = self.mRobotCmds.CmdGetSpeed(incraw=incraw)
    if speed is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', speed
  
  def CmdSetSpeed(self, motorLeft, motorRight):
    """ Set Khepera Speed Shell Command. """
    speed = self.mRobotCmds.CmdSetSpeed(motorLeft, motorRight)
    if speed is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', speed
  
  def CmdStop(self):
    """ Stop the Khepera Motion Shell Command. """
    return self.CmdSetSpeed(0, 0)
  
  def CmdForward(self, motor):
    """ Move the Khepera Forward Shell Command. """
    if motor < 0.0:
      motor = -motor
    return self.CmdSetSpeed(motor, motor)
  
  def CmdBackward(self, motor):
    """ Move the Khepera Backward Shell Command. """
    if motor < 0.0:
      motor = -motor
    return self.CmdSetSpeed(-motor, -motor)
  
  def CmdMoveToPos(self, posLeft, posRight):
    """ Move Khepera to Position Shell Command. """
    pos = self.mRobotCmds.CmdMoveToPos(posLeft, posRight)
    if pos is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', pos
  
  def CmdSetOdometry(self, odometerLeft, odometerRight, incraw=False):
    """ Set Khepera Odometry Shell Command. """
    pos = self.mRobotCmds.CmdSetOdometry(odometerLeft, odometerRight, 
                                          incraw=incraw)
    if pos is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', pos
  
  def CmdGetOdometry(self, incraw=False):
    """ Set Khepera Odometry Shell Command. """
    pos = self.mRobotCmds.CmdGetOdometry(incraw=incraw)
    if pos is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', pos
  
  def CmdReadAdc(self, channel):
    """ Read Khepera ADC Channel Shell Command. """
    val = self.mRobotCmds.CmdReadAdc(channel)
    if val is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', val
  
  def CmdSetBaseLedState(self, led, state):
    """ Set Khepera Base User LED State Shell Command. """
    state = self.mRobotCmds.CmdSetBaseLedState(led, state)
    if state is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', state
  
  def CmdReadProximitySensors(self, incraw=False):
    """ Get Khepera Proximity IR LED Sensors Shell Command. """
    sensors = self.mRobotCmds.CmdReadProximitySensors(incraw=incraw)
    if sensors is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', sensors
  
  def CmdReadAmbientSensors(self, incraw=False):
    """ Get Khepera Ambient IR LED Sensors Shell Command. """
    sensors = self.mRobotCmds.CmdReadAmbientSensors(incraw=incraw)
    if sensors is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', sensors
  
  def CmdGetCalProximity(self):
    """ Get proximity sensors' calibration parameters Shell Command. """
    cal = self.mRobotCmds.ProximitySensorsGetCalParams()
    return 'ok', cal

  def CmdGetCalAmbient(self):
    """ Set ambient sensors' calibration parameters Shell Command. """
    cal = self.mRobotCmds.AmbientSensorsGetCalParams()
    return 'ok', cal

  def CmdExtBusRead(self, addr):
    """ Read from the Khepera Extension Bus Shell Command. """
    byte = self.mRobotCmds.CmdExtBusRead(addr)
    if byte is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', byte
  
  def CmdExtBusWrite(self, addr, byte):
    """ Write to the Khepera Extension Bus Shell Command. """
    byte = self.mRobotCmds.CmdExtBusWrite(addr, byte)
    if byte is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', byte
  
  def CmdTurret(self, tid, tcmd):
    """ Write a Shell Command to a Khepera Turret Shell Command. """
    rsp = self.mRobotCmds.CmdTurret(tid, tcmd)
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
  # Common Shell Prerequisites 
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

  sys.exit( KheRawShell.main(shclass=KheBaseShell,
                            argv0=__file__,
                            shName='Khepera Base') )
