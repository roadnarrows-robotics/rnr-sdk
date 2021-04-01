#! /usr/bin/env python
################################################################################
#
# KHR1BaseShell.py
#

""" KHR-1 Base Command-Line Shell Module

KHR-1 base serial command-line shell provides a command-line
interface to most of the commands available on the KHR-1
base robot.

This shell is GuiTerm auto-capable.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.12.14

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

try:
  import readline   # optional module
except ImportError:
  pass

import Fusion.Gui.GuiTypes as gt
import Fusion.Gui.GuiWinShell as gterm

import Fusion.KHR1.Cmd.KHR1CmdBase as KHR1CmdBase

import Fusion.Utils.Shell as Shell
import Fusion.KHR1.Shells.KHR1RawShell as KHR1RawShell


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

# command modifiers
ReAttrName    = Shell.ReX('act|active|bid|bidlist|ver|version')

# command verbs
ReGet         = Shell.ReX('get')
ReSet         = Shell.ReX('set')
ReReset       = Shell.ReX('reset')
ReInc         = Shell.ReX('inc')
RePlay        = Shell.ReX('play')
ReDump        = Shell.ReX('dump')

# command modifier arguments
ReSpeed       = Shell.ReX('?:sp|speed') + Shell.ReArgPosInt
ReChanList    = Shell.ReX('?:ch|channels') + Shell.ReArgAll
RePos         = Shell.ReX('pos')
RePosNum      = Shell.ReX('?:pos') + Shell.ReArgPosInt
ReIndex       = Shell.ReX('index')
ReIndexNum    = Shell.ReX('?:index') + Shell.ReArgPosInt
ReCount       = Shell.ReX('count')
ReCountNum    = Shell.ReX('?:count') + Shell.ReArgPosInt
ReMotion      = Shell.ReX('mo|motion')
ReMotionNum   = Shell.ReX('?:mo|motion') + Shell.ReArgPosInt
ReSleepMotion = Shell.ReX('sleep|motion')


#-------------------------------------------------------------------------------
# CLASS: KHR1BaseShell
#-------------------------------------------------------------------------------
class KHR1BaseShell(KHR1RawShell.KHR1RawShell):
  """ KHR-1 Base Shell Class. """

  #--
  def __init__(self, args=(), **kwargs):
    """ Initialze Shell.

        Parameters:
          args    - arguments (not used)
          kwargs  - dictionary of shell options arguments
            robotCmds   - Robot commands object. Default: KHR1CmdBase()
            **shopts    - Shell raw and core options.
          
    """
    # initialize shell base object
    KHR1RawShell.KHR1RawShell.__init__(self, args, **kwargs)

  #--
  def Config(self, **kwargs):
    """ Configure Options Override. 

        Parameters:
          kwargs  - dictionary of options arguments
    """
    # this shell arguments defaults
    dfts = {
      'argv0': __file__,
      'shName': 'KHR-1 Base',
      'shVersion': '1.0',
      'shPS1': 'khr-1$ ',
      'shPS2': 'khr-1> ',
      'robotCmds': None
    }

    # set shell argument defaults, but caller has precedence
    for key, value in dfts.items():
      if key not in kwargs or not kwargs[key]:
        if key == 'robotCmds':
          kwargs[key] = KHR1CmdBase.KHR1CmdBase()
        else:
          kwargs[key] = value

    # set all options from arguments
    KHR1RawShell.KHR1RawShell.Config(self, **kwargs)


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
    KHR1RawShell.KHR1RawShell.InitCmdDict(self)

    # this shell's additional or override commands
    cmds = {
      # attribute command
      'attribute': {
        'help': {
          'desc': [
            "Get or set KHR-1 attributes.",
            "  'get' will get the current attribute value(s).",
            "  'set active' defines all active servos by mnemonic "
            "assignment.",
            "  'set bidlist' sets the list of KHR-1 Board IDs.",
            "  'set version' sets the servo version of the KHR-1."
          ],
          'usage': [
            "attribute <name> get",
            "attribute act[ive] set <mnem_0> <ch_0> ...",
            "attribute bid[list] set <bid_0> [<bid_1> ...]",
            "attribute ver[sion] set <ver>"
          ],
          'args': {
            '<name>': "One of: act[ive] bid[list] ver[sion].",
            '<mnem_k>': "Mnemonic k assigned to channel <ch_k>.",
            '<ch_k>': "Channel number k [0-<numofchannels>).",
            '<bid_k>': "Board ID k for this KHR-1 [0,31].",
            '<ver>':  "KHR-1 servo version. One of: red blue"
          },
          'rvals': "Return (new) current attribute value(s)."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReX('act|active') + ReSet +
                      Shell.ReArgAll + Shell.ReEnd,
            'cvt':    [self.CvtStr, self.CvtStr, self.CvtActiveServos]
          },
          { 'refmt':  Shell.ReStart + Shell.ReX('bid|bidlist') + ReSet +
                      Shell.ReArgAll + Shell.ReEnd,
            'cvt':    [self.CvtStr, self.CvtStr, self.CvtBidList]
          },
          { 'refmt':  Shell.ReStart + Shell.ReX('ver|version') + ReSet +
                      Shell.ReX('red|blue') + Shell.ReEnd,
            'cvt':    [self.CvtStr]
          },
          { 'refmt':  Shell.ReStart + ReAttrName + ReGet + Shell.ReEnd,
            'cvt':    [self.CvtStr]
          }
        ],
        'prereq': [],
        'exec':   self.CmdAttr
      },

      # position command
      'position': {
        'help': {
          'desc': [
            "Get or set KHR-1's current position.",
            "  'get' will get the current robot's position.",
            "  'set' will set (move) the robot to a new position.",
            "  'inc' will incrementally set (move) the robot to a new "
            "position.",
          ],
          'usage': [
            "position get",
            "position set sp[eed] <speed> ch[annels] <deg_0> ...",
            "position inc sp[eed] <speed> mnem_0 <deg_0> ..."
          ],
          'args': {
            '<speed>': "Servo speed [0,7], 0 == fastest, 7 == slowest.",
            '<mnem_k>': "Assigned channel mnemonic k (unique partial "
            "matching ok).",
            '<deg_k>':  [
              "Channel k's servo position value",
              "  0-180  = end position in degrees",
              "  <spec> = 'red' version special values (see doc.)"
            ],
          },
          'rvals': "Return (new) current ordered channel list."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + ReSet + ReSpeed + ReChanList + 
                      Shell.ReEnd,
            'cvt':    [self.CvtStr, self.CvtSpeed, self.CvtChanList]
          },
          { 'refmt':  Shell.ReStart + ReInc + ReSpeed + Shell.ReArgAll + 
                      Shell.ReEnd,
            'cvt':    [self.CvtStr, self.CvtSpeed, self.CvtChanMnemList]
          },
          { 'refmt':  Shell.ReStart + ReGet + Shell.ReEnd,
            'cvt':    [self.CvtStr]
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdPos
      },

      # home command
      'home': {
        'help': {
          'desc': [
            "Operations on KHR-1's home position.",
            "  'get' will get the current saved home position.",
            "  'goto' will go to the current saved home position.",
            "  'set' will set the current position of the robot as its "
            "new home.",
            "  'reset' will reset the home position to the factory default."
          ],
          'usage': [
            "home get",
            "home goto",
            "home set",
            "home reset"
          ],
          'rvals': "Return (new) home position ordered channel list."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + ReSet + Shell.ReEnd,
            'cvt':    [self.CvtStr]
          },
          { 'refmt':  Shell.ReStart + ReReset + Shell.ReEnd,
            'cvt':    [self.CvtStr]
          },
          { 'refmt':  Shell.ReStart + ReGet + Shell.ReEnd,
            'cvt':    [self.CvtStr]
          },
          { 'refmt':  Shell.ReStart + Shell.ReX('goto') + Shell.ReEnd,
            'cvt':    [self.CvtStr]
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdHome
      },

      # motion command
      'motion': {
        'help': {
          'desc': [
            "Operations for KHR-1's programmed motions.",
            "  'get pos' will get the given position data from a motion.",
            "  'set pos' will set the position data for a motion.",
            "  'inc pos' will incrementally set the position data for "
            "a motion.",
            "  'get count' will get the position count of a motion.",
            "  'set count' will set the position cont for a motion.",
            "  'dump' will dump all data associated with a motion.",
            "  'play' will play a motion."
          ],
          'usage': [
            "motion <mo_num> get pos <pos_num>",
            "motion <mo_num> set pos <pos_num> sp[eed] <speed> ch[annels]"
            " <deg_0> ...",
            "motion <mo_num> inc pos <pos_num> sp[eed] <speed>"
            " <mnem_0> <deg_0> ...",
            "motion <mo_num> get count",
            "motion <mo_num> set count <pos_cnt>",
            "motion <mo_num> dump",
            "motion <mo_num> play"
          ],
          'args': {
            '<mo_num>': "Motion number [0,39].",
            '<pos_num>': "Position number [0,99].",
            '<pos_cnt>': "Position count [0,100].",
            '<speed>': "Servo speed [0,7], 0 == fastest, 7 == slowest.",
            '<mnem_k>': "Assigned channel mnemonic k (unique partial "
            "matching ok).",
            '<deg_k>':  [
              "Channel k's servo position value",
              "  0-180  = end position in degrees",
              "  <spec> = 'red' version special values (see doc.)"
            ],
          },
          'rvals': "Return labeled dictionary of the effected motion."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReArgPosInt + ReSet + RePos +
                      Shell.ReArgPosInt + ReSpeed + ReChanList + Shell.ReEnd,
            'cvt':    [self.CvtMotionNum, self.CvtStr, self.CvtStr, 
                       self.CvtPosNum, self.CvtSpeed, self.CvtChanList]
          },
          { 'refmt':  Shell.ReStart + Shell.ReArgPosInt + ReInc + RePos +
                      Shell.ReArgPosInt + ReSpeed + Shell.ReArgAll + 
                      Shell.ReEnd,
            'cvt':    [self.CvtMotionNum, self.CvtStr, self.CvtStr, 
                       self.CvtPosNum, self.CvtSpeed, self.CvtChanMnemList]
          },
          { 'refmt':  Shell.ReStart + Shell.ReArgPosInt + ReGet + RePos +
                      Shell.ReArgPosInt + Shell.ReEnd,
            'cvt':    [self.CvtMotionNum, self.CvtStr, self.CvtStr, 
                       self.CvtPosNum]
          },
          { 'refmt':  Shell.ReStart + Shell.ReArgPosInt + ReSet + ReCount +
                      Shell.ReArgPosInt + Shell.ReEnd,
            'cvt':    [self.CvtMotionNum, self.CvtStr, self.CvtStr,
                       self.CvtPosCnt]
          },
          { 'refmt':  Shell.ReStart + Shell.ReArgPosInt + ReGet + ReCount +
                      Shell.ReEnd,
            'cvt':    [self.CvtMotionNum, self.CvtStr, self.CvtStr]
          },
          { 'refmt':  Shell.ReStart + Shell.ReArgPosInt + ReDump + Shell.ReEnd,
            'cvt':    [self.CvtMotionNum, self.CvtStr]
          },
          { 'refmt':  Shell.ReStart + Shell.ReArgPosInt + RePlay + Shell.ReEnd,
            'cvt':    [self.CvtMotionNum, self.CvtStr]
          },
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdMotion
      },

      # scenario command
      'scenario': {
        'help': {
          'desc': [
            "Operations for KHR-1's programmed scenarios.",
            #"  'get all' will get full scenario data .",
            "  'get index' will get the motion assigned at a scenario index.",
            "  'set index' will set a motion number at a scenario index.",
            "  'get count' will get the motion count for a scenario.",
            "  'set count' will set the motion count for a scenario.",
            "  'dump' will dump all data associated with a scenario.",
            "  'play' will play a scenario."
          ],
          'usage': [
            #"scenario <sc_num> get all",
            "scenario <sc_num> get index <idx>",
            "scenario <sc_num> set index <idx> mo[tion] <mo_num>",
            "scenario <sc_num> get count",
            "scenario <sc_num> set count <motion_cnt>",
            "scenario <sc_num> dump",
            "scenario <sc_num> play"
          ],
          'args': {
            '<sc_num>': "Motion number [0,3].",
            '<mo_num>': "Motion number [0,39].",
            '<idx>': "Scenario index [0,199].",
            '<motion_cnt>': "Scenario motion count [0,200].",
          },
          'rvals': "Return labeled dictionary of the effected scenario."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReArgPosInt + ReSet + ReIndex +
                      Shell.ReArgPosInt + ReMotionNum + Shell.ReEnd,
            'cvt':    [self.CvtScenarioNum, self.CvtStr, self.CvtStr,
                       self.CvtScenarioIndex, self.CvtMotionNum]
          },
          { 'refmt':  Shell.ReStart + Shell.ReArgPosInt + ReGet + ReIndex +
                      Shell.ReArgPosInt + Shell.ReEnd,
            'cvt':    [self.CvtScenarioNum, self.CvtStr, self.CvtStr,
                       self.CvtScenarioIndex]
          },
          { 'refmt':  Shell.ReStart + Shell.ReArgPosInt + ReSet + ReCount +
                      Shell.ReArgPosInt + Shell.ReEnd,
            'cvt':    [self.CvtScenarioNum, self.CvtStr, self.CvtStr,
                       self.CvtScenarioMotionCnt]
          },
          { 'refmt':  Shell.ReStart + Shell.ReArgPosInt + ReGet + ReCount +
                      Shell.ReEnd,
            'cvt':    [self.CvtScenarioNum, self.CvtStr, self.CvtStr]
          },
          { 'refmt':  Shell.ReStart + Shell.ReArgPosInt + ReDump + Shell.ReEnd,
            'cvt':    [self.CvtScenarioNum, self.CvtStr]
          },
          { 'refmt':  Shell.ReStart + Shell.ReArgPosInt + RePlay + Shell.ReEnd,
            'cvt':    [self.CvtScenarioNum, self.CvtStr]
          },
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdScenario
      },

      # trim command
      'trim': {
        'help': {
          'desc': "Get or set KHR-1's trim position.",
          'usage': [
            "trim get",
            "trim set ch[annels] <deg_0> ..."
          ],
          'args': {
            '<deg_k>': "Channel k's servo trim position value [-20,19]."
          },
          'rvals': "Return (new) ordered channel trim list."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + ReSet + ReChanList + Shell.ReEnd,
            'cvt':    [self.CvtStr, self.CvtChanTrimList]
          },
          { 'refmt':  Shell.ReStart + ReGet + Shell.ReEnd,
            'cvt':    [self.CvtStr]
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdTrim
      },

      # swbits command
      'swbits': {
        'help': {
          'desc': "Get or set KHR-1's software switch bits.",
          'usage': [
            "swbits get",
            "swbits set <bit> <switch> [<bit> <switch>]"
          ],
          'args': {
            '<bit>': "Software bit. One of: sleep motion.",
            '<switch>': "Switch postion. One of: off on."
          },
          'rvals': "Return (new) software switch bit positions."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + ReSet + ReSleepMotion + Shell.ReArgOnOff +
                      ReSleepMotion + Shell.ReArgOnOff + Shell.ReEnd,
            'cvt':    [self.CvtStr, self.CvtStr, self.CvtOnOff, self.CvtStr,
                       self.CvtOnOff]
          },
          { 'refmt':  Shell.ReStart + ReSet + ReSleepMotion + Shell.ReArgOnOff +
                      Shell.ReEnd,
            'cvt':    [self.CvtStr, self.CvtStr, self.CvtOnOff]
          },
          { 'refmt':  Shell.ReStart + ReGet + Shell.ReEnd,
            'cvt':    [self.CvtStr]
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdSwBits
      },
  
      # rcb-1 command
      'rcb-1': {
        'help': {
          'desc': [
            "Operations for KHR-1's RCB-1 Board IDs.",
            "  Note 1: These operations only apply to the connected RCB-1.",
            "  Note 2: The 'get' operation does not always return the expected "
            "value."
          ],
          'usage': [
            "rcb-1 get",
            "rcb-1 set <bid>" 
          ],
          'args': {
            '<bid>': "Connected Board ID [0,31]."
          },
          'rvals': "Return (new) current Board ID."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + ReSet + Shell.ReArgPosInt + Shell.ReEnd,
            'cvt':    [self.CvtStr, self.CvtBidList]
          },
          { 'refmt':  Shell.ReStart +  ReGet + Shell.ReEnd,
            'cvt':    [self.CvtStr]
          }
        ],
        'prereq': [],
        'exec':   self.CmdRcb1BoardId
      }
    }

    # now add the additional commands to the dictionary
    self.AddToCmdDict(cmds)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Shell Commands
  #   All commands must return the 2-tuple (rc, rval) where:
  #     ('err', <errstr>) or ('ok', <data)>)
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def CmdAttr(self, mod, op, *args):
    """ Get/Set KHR-1 attributes. """
    if mod in ['act', 'active']:
      if op == 'get':
        return 'ok', self.mRobotCmds.AttrGetActiveServos()
      elif op == 'set':
        return 'ok', self.mRobotCmds.AttrSetActiveServos(**args[0])
      else:
        return 'err', 'Bug: unknown verb: %s' % repr(op)
    elif mod in ['bid', 'bidlist']:
      if op == 'get':
        return 'ok', self.mRobotCmds.AttrGetBoardIdList()
      elif op == 'set':
        return 'ok', self.mRobotCmds.AttrSetBoardIdList(args[0])
      else:
        return 'err', 'Bug: unknown verb: %s' % repr(op)
    elif mod in ['ver', 'version']:
      if op == 'get':
        return 'ok', self.mRobotCmds.AttrGetServoVersion()
      elif op == 'set':
        return 'ok', self.mRobotCmds.AttrSetServoVersion(args[0])
      else:
        return 'err', 'Bug: unknown verb: %s' % repr(op)
    else:
      return 'err', 'Bug: unknown modifier: %s' % repr(mod)
  
  #--
  def CmdPos(self, op, *args):
    """ Get/Set KHR-1 current position. """
    if op == 'get':
      chanList = self.mRobotCmds.CmdGetCurPos()
    elif op == 'set':
      chanList = self.mRobotCmds.CmdSetCurPos(*args)
    elif op == 'inc':
      chanList = self.mRobotCmds.CmdSetIncCurPos(args[0], **args[1])
    else:
      return 'err', 'Bug: unknown verb: %s' % repr(op)
    if not chanList:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', chanList
  
  #--
  def CmdHome(self, op, *args):
    """ Get/Set/GoTo/Reset KHR-1 home position. """
    if op == 'get':
      chanList = self.mRobotCmds.CmdGetHomePos()
    elif op == 'set':
      chanList = self.mRobotCmds.CmdSetCurPosAsHomePos()
    elif op == 'goto':
      chanList = self.mRobotCmds.CmdGoToHomePos()
    elif op == 'reset':
      chanList = self.mRobotCmds.CmdResetHomePos()
    else:
      return 'err', 'Bug: unknown verb: %s' % repr(op)
    if not chanList:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', chanList

  #--
  def CmdMotion(self, motionNum, op, *args):
    """ Operations for KHR-1 motion data. """
    if op == 'get':
      if args[0] == 'pos':
        rvals = self.mRobotCmds.CmdGetMotionPos(motionNum, args[1])
      elif args[0] == 'count':
        rvals = self.mRobotCmds.CmdGetMotionPosCnt(motionNum)
      else:
        return 'err', 'Bug: unknown modifier arg: %s' % repr(args[0])
    elif op == 'set':
      if args[0] == 'pos':
        rvals = self.mRobotCmds.CmdSetMotionPos(motionNum, args[1], args[2],
                                                  args[3])
      elif args[0] == 'count':
        rvals = self.mRobotCmds.CmdSetMotionPosCnt(motionNum, args[1])
      else:
        return 'err', 'Bug: unknown modifier arg: %s' % repr(args[0])
    elif op == 'inc':
      rvals = self.mRobotCmds.CmdSetIncMotionPos(motionNum, args[1], args[2],
                                                  **args[3])
    elif op == 'dump':
      rvals = self.mRobotCmds.CmdDumpMotion(motionNum)
    elif op == 'play':
      rvals = self.mRobotCmds.CmdPlayMotion(motionNum)
    else:
      return 'err', 'Bug: unknown verb: %s' % repr(op)
    if rvals is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', rvals

  #--
  def CmdScenario(self, scenarioNum, op, *args):
    """ Operations for KHR-1 scenario data. """
    if op == 'get':
      if args[0] == 'index':
        rvals = self.mRobotCmds.CmdGetScenarioPos(scenarioNum, args[1])
      elif args[0] == 'count':
        rvals = self.mRobotCmds.CmdGetScenarioMotionCnt(scenarioNum)
      else:
        return 'err', 'Bug: unknown modifier arg: %s' % repr(args[0])
    elif op == 'set':
      if args[0] == 'index':
        rvals = self.mRobotCmds.CmdSetScenarioMotion(scenarioNum, args[1],
                                                      args[2])
      elif args[0] == 'count':
        rvals = self.mRobotCmds.CmdSetScenarioMotionCnt(scenarioNum, args[1])
      else:
        return 'err', 'Bug: unknown modifier arg: %s' % repr(args[0])
    elif op == 'dump':
      rvals = self.mRobotCmds.CmdDumpScenario(scenarioNum)
    elif op == 'play':
      rvals = self.mRobotCmds.CmdPlayScenario(scenarioNum)
    else:
      return 'err', 'Bug: unknown verb: %s' % repr(op)
    if rvals is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', rvals

  #--
  def CmdTrim(self, op, *args):
    """ Operations for KHR-1 trim. """
    if op == 'get':
      chanList = self.mRobotCmds.CmdGetTrim()
    elif op == 'set':
      chanList = self.mRobotCmds.CmdSetTrim(args[0])
    else:
      return 'err', 'Bug: unknown verb: %s' % repr(op)
    if chanList is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', chanList

  #--
  def CmdSwBits(self, op, *args):
    """ Operations for KHR-1 Software Switch Bits. """
    if op == 'get':
      rvals = self.mRobotCmds.CmdGetSwBits()
    elif op == 'set':
      newSwBits = {}
      n = 0
      while n < len(args):
        newSwBits[args[n]] = args[n+1]
        n += 2
      if len(newSwBits) < 2:  # new switches under specified, get current
        curSwBits = self.mRobotCmds.CmdGetSwBits()
        if curSwBits is None:
          return 'err', self.mRobotCmds.GetErrStr()
        for k,v in curSwBits.items():
          if k not in newSwBits:
            newSwBits[k] = v
      rvals = self.mRobotCmds.CmdSetSwBits(**newSwBits)
    else:
      return 'err', 'Bug: unknown verb: %s' % repr(op)
    if rvals is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', rvals

  #--
  def CmdRcb1BoardId(self, op, *args):
    """ Operations for KHR-1 connected RCB-1 Board ID. """
    if op == 'get':
      bid = self.mRobotCmds.RCB1CmdGetBoardId()
    elif op == 'set':
      bid = self.mRobotCmds.RCB1CmdSetBoardId(args[0][0])
    else:
      return 'err', 'Bug: unknown verb: %s' % repr(op)
    if bid is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', bid


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Parameter Converters 
  #   All converters must return the 2-tuple (rc, rval) where:
  #     ('err', <errstr>) or ('ok', <data)>)
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def CvtBidList(self, s):
    """ Convert Argument into Board Id List. """
    rc, vals = self.CvtIntList(s, KHR1CmdBase.RCB1BoardIdMin, 
                                  KHR1CmdBase.RCB1BoardIdMax)
    if rc == 'err':
      return rc, vals
    if len(vals) == 0:
      return 'err', 'no Board IDs specified'
    elif len(vals) > KHR1CmdBase.RCB1NumOfBoardIds:
      return 'err', 'too many Board IDs specified'
    return 'ok', vals
  
  #--
  def CvtSpeed(self, s):
    """ Convert Argument into Servo Speed. """
    rc, vals = self.CvtIntList(s, KHR1CmdBase.RCB1ServoSpeedMax,
                                  KHR1CmdBase.RCB1ServoSpeedMin)
    if rc == 'err':
      return rc, vals
    return 'ok', vals[0]

  #--
  def CvtChanList(self, s):
    """ Convert Argument into Channel List. """
    rc, vals = self.CvtIntList(s)
    if rc == 'err':
      return rc, vals
    ver = self.mRobotCmds.AttrGetServoVersion()
    numch = self.mRobotCmds.AttrHasNumOfChannels()
    if len(vals) == 0:
      return 'err', 'no channels values specified'
    elif len(vals) != numch:
      return 'err', '%d channel values required: %d specified' % \
        (numch, len(vals))
    n = 0
    for pos in vals:
      rc, val = self._ChkChanPos(ver, n, pos)
      if rc == 'err':
        return rc, val
      n += 1
    return 'ok', vals

  #--
  def CvtChanTrimList(self, s):
    """ Convert Argument into Channel Trim List. """
    return self.CvtIntList(s, KHR1CmdBase.RCB1ServoTrimMin,
                              KHR1CmdBase.RCB1ServoTrimMax)

  #--
  def CvtChanMnemList(self, s):
    """ Convert Argument into Active Channel Mnemonic Position List. """
    ver = self.mRobotCmds.AttrGetServoVersion()
    actList = self.mRobotCmds.AttrGetActiveServos()
    args = s.split()
    kwargs = {}
    n = 0
    while n < len(args):
      inMnem = args[n]
      if n+1 >= len(args):
        return 'err', 'missing position value for channel %s' % repr(inMnem)
      else:
        inPos = args[n+1]
      l = len(inMnem)
      matches = []
      for mnem in actList.keys():
        if len(mnem) >= l and mnem[:l] == inMnem:
          matches += [mnem]
      if len(matches) == 0:
        return 'err', 'channel mnemonic not found: %s' % repr(inMnem)
      elif len(matches) > 1:
        return 'err', 'ambiguous channel mnemonic: %s matches %s' % \
                        (repr(inMnem), matches)
      rc, val = self.CvtIntList(inPos)
      if rc == 'err':
        return rc, val
      pos = val[0]
      rc, val = self._ChkChanPos(ver, matches[0], pos)
      if rc == 'err':
        return rc, val
      kwargs[matches[0]] = pos
      n += 2
    return 'ok', kwargs

  #--
  def CvtActiveServos(self, s):
    """ Convert Argument into Active Channel Mnemonic Assignment List. """
    args = s.split()
    maxch = self.mRobotCmds.AttrHasNumOfChannels() - 1
    chanList = []
    actList = {}
    n = 0
    while n < len(args):
      mnem = args[n]
      if n+1 >= len(args):
        return 'err', 'missing channel number for mnemonic %s' % repr(mnem)
      else:
        inCh = args[n+1]
      if mnem in actList:
        return 'err', 'mnemonic %s is not unique' % repr(mnem)
      rc, val = self.CvtIntList(inCh, 0, maxch)
      if rc == 'err':
        return rc, val
      ch = val[0]
      if chanList.count(ch) > 0:
        return 'err', 'channal %d multiply assigned' % ch
      chanList += [ch]
      actList[mnem] = ch
      n += 2
    return 'ok', actList

  #--
  def CvtScenarioNum(self, s):
    """ Convert Argument into Scenario Number. """
    rc, vals = self.CvtIntList(s, KHR1CmdBase.RCB1ScenarioMin,
                                  KHR1CmdBase.RCB1ScenarioMax)
    if rc == 'err':
      return rc, vals
    return 'ok', vals[0]


  #--
  def CvtMotionNum(self, s):
    """ Convert Argument into Motion Number. """
    rc, vals = self.CvtIntList(s, KHR1CmdBase.RCB1MotionNumMin,
                                  KHR1CmdBase.RCB1MotionNumMax)
    if rc == 'err':
      return rc, vals
    return 'ok', vals[0]

  #--
  def CvtPosNum(self, s):
    """ Convert Argument into Motion Position Number. """
    rc, vals = self.CvtIntList(s, KHR1CmdBase.RCB1PosNumMin,
                                  KHR1CmdBase.RCB1PosNumMax)
    if rc == 'err':
      return rc, vals
    return 'ok', vals[0]

  #--
  def CvtPosCnt(self, s):
    """ Convert Argument into Motion Position Count. """
    rc, vals = self.CvtIntList(s, KHR1CmdBase.RCB1PosNumMin,
                                  KHR1CmdBase.RCB1NumOfMotionPos)
    if rc == 'err':
      return rc, vals
    return 'ok', vals[0]

  #--
  def CvtScenarioIndex(self, s):
    """ Convert Argument into Scenario Motion Index. """
    rc, vals = self.CvtIntList(s, KHR1CmdBase.RCB1MotionIdxMin,
                                  KHR1CmdBase.RCB1MotionIdxMax)
    if rc == 'err':
      return rc, vals
    return 'ok', vals[0]

  #--
  def CvtScenarioMotionCnt(self, s):
    """ Convert Argument into Scenario Motion Count. """
    rc, vals = self.CvtIntList(s, KHR1CmdBase.RCB1MotionIdxMin,
                                  KHR1CmdBase.RCB1NumOfScenarioMotions)
    if rc == 'err':
      return rc, vals
    return 'ok', vals[0]

  #--
  def _ChkChanPos(self, ver, chan, pos):
    """ Check Channel Position Number. """
    if (pos < KHR1CmdBase.RCB1ServoPosMin or \
        pos > KHR1CmdBase.RCB1ServoPosMax) \
          and \
       (pos < KHR1CmdBase.RCB1ServoPosSpecMin or \
        pos > KHR1CmdBase.RCB1ServoPosSpecMax):
      return 'err', 'channel %s out-of-range: %d' % (repr(chan), pos)
    elif (ver != KHR1CmdBase.RCB1ServoVersionRed) \
          and (pos >= KHR1CmdBase.RCB1ServoPosSpecMin and \
               pos <= KHR1CmdBase.RCB1ServoPosSpecMin):
      return 'err', 'servo version %s: ' \
          'does not support channel %d special value: 0x%02x' % \
            (repr(ver), repr(chan), pos)
    else:
      return 'ok', pos


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

  sys.exit( KHR1RawShell.main(shclass=KHR1BaseShell,
                              argv0=__file__,
                              shName="KHR-1 Base") )
