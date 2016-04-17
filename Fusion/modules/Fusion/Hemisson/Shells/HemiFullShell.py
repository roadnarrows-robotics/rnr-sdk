#! /usr/bin/env python
################################################################################
#
# HemiFullShell.py
#

""" Hemisson Full-UP Command-Line Shell Module

Hemisson full-up serial command-line shell provides a command-line
interface to the Hemisson base robot plus all supported modules.

The command/response interface for all Hemisson commands require the
HemiOS v1.50RNe+ version of the robot operating system.

This shell is GuiTerm auto-capable.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.03.06

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

import re

try:
  import readline   # optional module
except ImportError:
  pass

import Fusion.Gui.GuiTypes as gt
import Fusion.Gui.GuiWinShell as gterm

import Fusion.Hemisson.Cmd.HemiCmdLinCam as HemiCmdLinCam
import Fusion.Hemisson.Cmd.HemiCmdTts as HemiCmdTts
import Fusion.Hemisson.Cmd.HemiCmdUss as HemiCmdUss
import Fusion.Hemisson.Cmd.HemiCmdAll as HemiCmdAll

import Fusion.Utils.Shell as Shell
import Fusion.Hemisson.Shells.HemiRawShell as HemiRawShell
import Fusion.Hemisson.Shells.HemiBaseShell as HemiBaseShell


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

#
# Argument Regular Expressions.
#
ReArgZone  = '\s+(all|a|left|l|middle|m|right|r)' # lincam zone argument
ReArgFree  = '\s+([^#\n]*)'                       # free form line
ReArgUnits = '\s+(inch|i|cm|c|usec|u)'            # USS units


#-------------------------------------------------------------------------------
# CLASS: HemiFullShell
#-------------------------------------------------------------------------------
class HemiFullShell(HemiBaseShell.HemiBaseShell):
  """ Hemisson Full Shell Class. """

  #--
  def __init__(self, args=(), **kwargs):
    """ Initialze Raw Shell.

        Parameters:
          args    - arguments (not used)
          kwargs  - dictionary of shell options arguments
            robotCmds   - Robot commands object. Default: HemiCmdAll()
            **shopts    - Shell base, raw and core options.
          
    """
    # initialize shell base object
    HemiBaseShell.HemiBaseShell.__init__(self, args, **kwargs)

  #--
  def Config(self, **kwargs):
    """ Configure Options Override. 

        Parameters:
          kwargs  - dictionary of options arguments
    """
    # this shell arguments defaults
    dfts = {
      'argv0': __file__,
      'shName': 'Hemisson Full',
      'shVersion': '1.1',
      'shPS1': 'hemi$ ',
      'shPS2': 'hemi> ',
      'robotCmds': None
    }

    # set shell argument defaults, but caller has precedence
    for key, value in dfts.iteritems():
      if not kwargs.has_key(key) or not kwargs[key]:
        if key == 'robotCmds':
          kwargs[key] = HemiCmdAll.HemiCmdAll()
        else:
          kwargs[key] = value

    # set all options from arguments
    HemiBaseShell.HemiBaseShell.Config(self, **kwargs)


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
    HemiBaseShell.HemiBaseShell.InitCmdDict(self)

    # this shell's additional or override commands
    cmds = {
      #--
      # Linear Camera
      #--

      # linear camera version command
      'lcver': {
        'help': {
          'desc': "Get the Linear Camera firmware version.",
          'usage': "lcver",
          'rvals': "Linear Camera version string."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReEnd,
            'cvt':    []
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdLinCamGetVersion
      },

      # linear camera p-grab command
      'lcpgrab': {
        'help': {
          'desc': "Grab unthresholded pixels from the Linear Camera.",
          'usage': "lcpgrab [<zone>]",
          'args': {
            '<zone>': "Camera zone. One of: a[ll] l[eft] m[iddle] r[ight]. "
                      "default: all",
          },
          'rvals': "List of gray-scale pixel values, from left to right."
        },
        'parse': [
          { 
            'refmt':  Shell.ReStart + ReArgZone + Shell.ReEnd,
            'cvt':    [self.CvtZone]
          },
          { 'refmt':  Shell.ReStart + Shell.ReEnd,
            'cvt':    []
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdLinCamPGrab
      },

      # linear camera q-grab command
      'lcqgrab': {
        'help': {
          'desc': "Grab thresholded pixels from the Linear Camera.",
          'usage': "lcqgrab [<zone>]",
          'args': {
            '<zone>': "Camera zone. One of: a[ll] l[eft] m[iddle] r[ight]. "
                      "default: all",
          },
          'rvals': "List of black or white pixel values, from left to right."
        },
        'parse': [
          { 
            'refmt':  Shell.ReStart + ReArgZone + Shell.ReEnd,
            'cvt':    [self.CvtZone]
          },
          { 'refmt':  Shell.ReStart + Shell.ReEnd,
            'cvt':    []
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdLinCamQGrab
      },

      # linear camera settings command
      'lcsettings': {
        'help': {
          'desc': "Get the current Linear Camera settings.",
          'usage': "lcsettings",
          'rvals': "Dictionary of current settings."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReEnd,
            'cvt':    []
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdLinCamGetSettings
      },

      # linear camera exposure command
      'lcexposure': {
        'help': {
          'desc': "Set Linear Camera exposure time.",
          'usage': "lcexposure <msec>",
          'args': {
            '<msec>': "Exposure time in milliseconds [0, 255]."
          },
          'rvals': "New exposure time."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReArgPosInt + Shell.ReEnd,
            'cvt':    [self.CvtInt]
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdLinCamSetExposureTime
      },

      # linear camera threshold command
      'lcthreshold': {
        'help': {
          'desc': "Set Linear Camera intensity Q threshold.",
          'usage': "lcthreshold <intensity>",
          'args': {
            '<intensity>': "Pixel gray-scale intensity [1, 255]"
          },
          'rvals': "New threshold intensity value.",
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReArgPosInt + Shell.ReEnd,
            'cvt':    [self.CvtInt]
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdLinCamSetQThreshold
      },

      # linear camera led command
      'lcled': {
        'help': {
          'desc': "Turn Linear Camera's LED on/off.",
          'usage': "lcled <state>",
          'args': {
            '<state>': "LED state. One of: on off."
          },
          'rvals': "New LED state."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReArgOnOff + Shell.ReEnd,
            'cvt':    [self.CvtOnOff]
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdLinCamSetLedState
      },

      #--
      # Text-To-Speech
      #--

      # TTS version command
      'ttsver': {
        'help': {
          'desc': "Get the Text-To-Speech firmware version.",
          'usage': "ttsver",
          'rvals': "Text-To-Speech version string.",
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReEnd,
            'cvt':    []
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdTtsGetVersion
      },

      # TTS pitch command
      'ttspitch': {
        'help': {
          'desc': "Set TTS pitch of voice in text-to-speech conversion.",
          'usage': "ttspitch <pitch>",
          'args': {
            '<pitch>': ["Voice pitch [0, 7].",
                        "  0 = highest pitch (micky mouse)",
                        "  7 = lowest pitch (barry white)"]
          },
          'rvals': "New pitch value.",
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReArgPosInt + Shell.ReEnd,
            'cvt':    [self.CvtInt]
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdTtsSetVoicePitch
      },

      # TTS rate command
      'ttsrate': {
        'help': {
          'desc': "Set TTS rate of voice in text-to-speech conversion.",
          'usage': "ttsrate <rate>",
          'args': {
            '<rate>': ["Voice rate [0, 3].",
                        "  0 = slowest",
                        "  3 = fastest"]
          },
          'rvals': "New rate value.",
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReArgPosInt + Shell.ReEnd,
            'cvt':    [self.CvtInt]
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdTtsSetVoiceRate
      },

      # TTS gain command
      'ttsgain': {
        'help': {
          'desc': "Set TTS speaker gain (volume).",
          'usage': "ttsgain <gain>",
          'args': {
            '<gain>': ["Speaker gain [0, 7].",
                        "  0 = loadest",
                        "  7 = quietest"]
          },
          'rvals': "New gain value.",
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReArgPosInt + Shell.ReEnd,
            'cvt':    [self.CvtInt]
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdTtsSetSpeakerGain
      },

      # TTS query command
      'ttsquery': {
        'help': {
          'desc': "Query the Text-To-Speech speaking state.",
          'usage': "ttsquery",
          'rvals': "Returns 'speaking' or 'notspeaking'"
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReEnd,
            'cvt':    []
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdTtsQuery
      },

      # TTS settings command
      'ttssettings': {
        'help': {
          'desc': "Get the current Text-To-Speech settings.",
          'usage': "ttssettings",
          'rvals': "Dictionary of current settings."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReEnd,
            'cvt':    []
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdTtsGetSettings
      },

      # TTS say command
      'ttssay': {
        'help': {
          'desc': "Speak a TTS message.",
          'usage': "ttssay <msg>",
          'args': {
            '<msg>': "Text message to say"
          },
          'rvals': "<msg>"
        },
        'parse': [
          { 'refmt':  Shell.ReStart + ReArgFree + Shell.ReEnd,
            'cvt':    [self.CvtStr]
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdTtsSay
      },

      # TTS ysay command
      'ttsysay': {
        'help': {
          'desc': "Speak a TTS message and wait until the speaking is "
                   "finished.",
          'usage': "ttsysay <msg>",
          'args': {
            '<msg>': "Text message to say"
          },
          'rvals': "<msg>"
        },
        'parse': [
          { 'refmt':  Shell.ReStart + ReArgFree + Shell.ReEnd,
            'cvt':    [self.CvtStr]
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdTtsSaySync
      },

      # TTS canned command
      'ttscanned': {
        'help': {
          'desc': "Speak a canned (pre-stored) TTS message.",
          'usage': "ttscanned <msg_num>",
          'args': {
            '<msg_num>': "Number of pre-stored message [1, 30]"
          },
          'rvals': "<msg_num>"
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReArgPosInt + Shell.ReEnd,
            'cvt':    [self.CvtInt]
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdTtsSpeakCannedMsg
      },

      # TTS orate command
      'ttsorate': {
        'help': {
          'desc': "Give a TTS (synchronized) speech.",
          'usage': "ttsorate <multi_line_msg>",
          'args': {
            '<multi_line_msg>': "Multi-line message terminated by string 'EOS'"
          },
          'rvals': "Returns 'applause' on completion."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + ReArgFree + Shell.ReEnd,
            'cvt':    [self.CvtStr]
          },
          { 'refmt':  Shell.ReStart + Shell.ReEnd,
            'cvt':    []
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdTtsOrate
      },

      # TTS abc command
      'ttsabc': {
        'help': {
          'desc': "TTS says his ABC's.",
          'usage': "ttsabc",
          'rvals': "Returns 'nowwhatdoyouthinkofme?'"
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReEnd,
            'cvt':    []
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdTtsAbc
      },

      #--
      # UltraSonic Sensor
      #--

      # USS version command
      'ussver': {
        'help': {
          'desc': "Get the UltraSonic Sensor firmware version.",
          'usage': "ussver",
          'rvals': "UltraSonic Sensor version string.",
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReEnd,
            'cvt':    []
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdUssGetVersion
      },

      # USS ping command
      'ussping': {
        'help': {
          'desc': "USS ping by taking a measurement and returning the "
                  "number of echo values.",
          'usage': "ussping [<num_echoes>]",
          'args': {
            '<num_echoes>': "Number of echoes to return [1, 17]. "
                            "default: 17.",
          },
          'rvals': "List of ping distances in current units from "
                   "the closest to farthest echo."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReArgPosInt + Shell.ReEnd,
            'cvt':    [self.CvtInt]
          },
          { 'refmt':  Shell.ReStart + Shell.ReEnd,
            'cvt':    []
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdUssPing
      },

      # USS measure command
      'ussmeasure': {
        'help': {
          'desc': "Take a USS measurement in the current units and at "
                  "the current maximum range. The ambient light intensity "
                  "will also be measured.",
          'usage': "ussmeasure",
          'rvals': "Returns 'completed'"
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReEnd,
            'cvt':    []
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdUssTakeMeasurement
      },

      # USS echoes command
      'ussechoes': {
        'help': {
          'desc': "Get a number of USS echoes saved from the last "
                  "measurement.",
          'usage': "ussechoes [<num_echoes>]",
          'args': {
            '<num_echoes>': "Number of echoes to return [1, 17]. "
                            "default: 17.",
          },
          'rvals': "List of ping distances in current units from "
                   "the closest to farthest echo."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReArgPosInt + Shell.ReEnd,
            'cvt':    [self.CvtInt]
          },
          { 'refmt':  Shell.ReStart + Shell.ReEnd,
            'cvt':    []
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdUssGetEchoSet
      },

      # USS 1echo command
      'uss1echo': {
        'help': {
          'desc': "Get one echo <echo_num> from the USS echoes saved from the "
                  "last measurement.",
          'usage': "uss1echo <echo_num>",
          'args': {
            '<echo_num>': ["Echo number [0, 16]",
                          " 0 = closest",
                          "16 = farthest"]
          },
          'rvals': "Echo distance in current units."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReArgPosInt + Shell.ReEnd,
            'cvt':    [self.CvtInt]
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdUssGetOneEcho
      },

      # USS light command
      'usslight': {
        'help': {
          'desc': "Get the USS ambient light intensity from the saved "
                  "last measurement.",
          'usage': "usslight",
          'rvals': "Gray-scale light intensity [0-255]."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReEnd,
            'cvt':    []
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdUssGetLightIntensity
      },

      # USS range command
      'ussrange': {
        'help': {
          'desc': "Set maximum range for the USS. The range value is in "
                  "the current units. The maximum range the USS supports "
                  "is 11 meters.",
          'usage': "ussrange <range>",
          'args': {
            '<range>': "Maximum range [1, 65535]"
          },
          'rvals': "Actual range set in hardware."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReArgPosInt + Shell.ReEnd,
            'cvt':    [self.CvtInt]
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdUssSetMaxRange
      },

      # USS units command
      'ussunits': {
        'help': {
          'desc': "Set the units for all subsequent USS measurements.",
          'usage': "ussunits <units>",
          'args': {
            '<units>': "Measurement units. One of: i[nch] c[m] u[sec]"
          },
          'rvals': "New measurement <units>."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + ReArgUnits + Shell.ReEnd,
            'cvt':    [self.CvtUnits]
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdUssSetUnits
      },

      # USS settings command
      'usssettings': {
        'help': {
          'desc': "Get the current USS settings.",
          'usage': "usssettings",
          'rvals': "Dictionary of current settings."
        },
        'parse': [
          { 'refmt':  Shell.ReStart + Shell.ReEnd,
            'cvt':    []
          }
        ],
        'prereq': [self.PreReqOpen],
        'exec':   self.CmdUssGetSettings
      },
    }

    # now add the additional commands to the dictionary
    self.AddToCmdDict(cmds)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Shell Commands
  #   All commands must return the 2-tuple (rc, rval) where:
  #     ('err', <errstr>) or ('ok', <data)>)
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  def CmdLinCamGetVersion(self):
    """ Get Hemisson Linear Camera Firmware Version Shell Command. """
    ver = self.mRobotCmds.LinCamCmdGetVersion();
    if ver is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', 'LinCam_v' + repr(ver)

  def CmdLinCamPGrab(self, zone=HemiCmdLinCam.LinCamZoneAll):
    """ Grab Linear Camera Unthresholded Pixels Shell Command. """
    rsp = self.mRobotCmds.LinCamCmdGrabPPixels(zone)
    if rsp is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', rsp

  def CmdLinCamQGrab(self, zone=HemiCmdLinCam.LinCamZoneAll):
    """ Grab Linear Camera Thresholded Pixels Shell Command. """
    rsp = self.mRobotCmds.LinCamCmdGrabQPixels(zone)
    if rsp is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', rsp

  def CmdLinCamGetSettings(self):
    """ Get Linear Camera Settings Shell Command. """
    et = self.mRobotCmds.LinCamCmdGetExposureTime()
    if et is None:
      return 'err', self.mRobotCmds.GetErrStr()
    th = self.mRobotCmds.LinCamCmdGetQThreshold()
    if th is None:
      return 'err', self.mRobotCmds.GetErrStr()
    return 'ok', {'exposure':et, 'threshold':th}

  def CmdLinCamSetExposureTime(self, et):
    """ Set Linear Camera Exposure Time Shell Command. """
    rsp = self.mRobotCmds.LinCamCmdSetExposureTime(et)
    if rsp is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', rsp

  def CmdLinCamSetQThreshold(self, th):
    """ Set Linear Camera Exposure Time Shell Command. """
    rsp = self.mRobotCmds.LinCamCmdSetQThreshold(th)
    if rsp is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', rsp

  def CmdLinCamSetLedState(self, state):
    """ Set Linear Camera Exposure Time Shell Command. """
    rsp = self.mRobotCmds.LinCamCmdSetLedState(state)
    if rsp is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', rsp

  def CmdTtsGetVersion(self):
    """ Get Hemisson Text-To-Speech Firmware Version Shell Command. """
    ver = self.mRobotCmds.TtsCmdGetVersion();
    if ver is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', 'TTS_v' + repr(ver)

  def CmdTtsSetVoicePitch(self, pitch):
    """ Set Hemisson Text-To-Speech Voice Pitch Shell Command. """
    rsp = self.mRobotCmds.TtsCmdSetVoicePitch(pitch);
    if rsp is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', rsp

  def CmdTtsSetVoiceRate(self, rate):
    """ Set Hemisson Text-To-Speech Voice Rate Shell Command. """
    rsp = self.mRobotCmds.TtsCmdSetVoiceRate(rate);
    if rsp is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', rsp

  def CmdTtsSetSpeakerGain(self, gain):
    """ Set Hemisson Text-To-Speech Speaker Gain Shell Command. """
    rsp = self.mRobotCmds.TtsCmdSetSpeakerGain(gain);
    if rsp is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', rsp

  def CmdTtsQuery(self):
    """ Query Hemisson Text-To-Speech Speaking State Shell Command. """
    rsp = self.mRobotCmds.TtsCmdQueryState();
    if rsp is None:
      return 'err', self.mRobotCmds.GetErrStr()
    elif rsp == HemiCmdTts.TtsStateSpeaking:
      return 'ok', 'speaking'
    elif rsp == HemiCmdTts.TtsStateNotSpeaking:
      return 'ok', 'notspeaking'
    else:
      return 'err', 'unknown query response: %s' % resp(rsp)

  def CmdTtsGetSettings(self):
    """ Get Text-To-Speech Settings Shell Command. """
    rsp = self.mRobotCmds.TtsCmdGetSettings()
    if rsp is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', rsp

  def CmdTtsSay(self, msg):
    """ Speak a Hemisson Text-To-Speech Message Command. """
    rsp = self.mRobotCmds.TtsCmdSay(msg);
    if rsp is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', rsp

  def CmdTtsSaySync(self, msg):
    """ Synchronized Speak a Hemisson Text-To-Speech Message Command. """
    rsp = self.mRobotCmds.TtsCmdSaySync(msg);
    if rsp is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', rsp

  def CmdTtsSpeakCannedMsg(self, msgnum):
    """ Speak a Hemisson Text-To-Speech Canned Message Shell Command. """
    rsp = self.mRobotCmds.TtsCmdSpeakCannedMsg(msgnum);
    if rsp is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', rsp

  def CmdTtsOrate(self, msg=''):
    """ Give a Hemisson Text-To-Speech Long Speech Shell Command. """
    speech = ''
    goteof = False
    remind = True
    while not goteof and not self.mDoQuit:
      match = re.match('(.*)EOS', msg)
      if match:
        msg = match.group(1)
        goteof = True
      if self.mIsInteractive and not goteof and remind:
        self.Print("(enter 'EOS' at end of speech)")
        remind = False
      if msg != '':
        msg = msg.strip()
        if speech:
          speech += ' '
        speech += msg
      if not goteof:
        if self.mIsInteractive:
          msg = self.GetUserInput(self.mOpts['shPS2'])
        else:
          msg = self.mScriptFile.readline()
          if not msg:
            goteof = True
            self.mDoQuit = True
            self.mScriptFile.close()
    if speech == '':
      return 'ok', ''
    rsp = self.mRobotCmds.TtsCmdGiveASpeech(speech)
    if rsp is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', rsp

  def CmdTtsAbc(self):
    """ Text-To-Speech Say's Her ABC's Shell Command. """
    abcs =    "'A' 'B' 'C' 'D' 'E' 'F' 'G' 'H' 'I' 'J' 'K' 'L' 'M'" \
           +  "'N' 'O' 'P' cue 'R' ess 'T' 'U' 'V' 'doubleu' 'X' 'Y' 'Z'"
    rsp = self.mRobotCmds.TtsCmdGiveASpeech(abcs)
    if rsp is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', "nowwhatdoyouthinkofme?"

  def CmdUssGetVersion(self):
    """ Get Hemisson UltraSonic Sensor Firmware Version Shell Command. """
    ver = self.mRobotCmds.UssCmdGetVersion();
    if ver is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', 'USS_v' + repr(ver)

  def CmdUssPing(self, numechoes=HemiCmdUss.UssEchoMaxNum):
    """ Ping the Hemisson UltraSonic Sensor Shell Command. """
    rsp = self.mRobotCmds.UssCmdPing(numechoes);
    if rsp is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', rsp

  def CmdUssTakeMeasurement(self):
    """ Take a Hemisson UltraSonic Sensor Measurement Shell Command. """
    ver = self.mRobotCmds.UssCmdTakeMeasurement();
    if ver is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', repr(ver)

  def CmdUssGetEchoSet(self, numechoes=HemiCmdUss.UssEchoMaxNum):
    """ Return the Hemisson UltraSonic Sensor Echo Set Shell Command. """
    rsp = self.mRobotCmds.UssCmdGetEchoSet(numechoes);
    if rsp is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', rsp

  def CmdUssGetOneEcho(self, echonum):
    """ Return the Hemisson UltraSonic Sensor One Echo Shell Command. """
    rsp = self.mRobotCmds.UssCmdGetOneEcho(echonum);
    if rsp is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', rsp

  def CmdUssGetLightIntensity(self):
    """ Get Hemisson UltraSonic Sensor Light Intensity Shell Command. """
    rsp = self.mRobotCmds.UssCmdGetLightIntensity();
    if rsp is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', rsp

  def CmdUssSetMaxRange(self, range):
    """ Set the Hemisson UltraSonic Sensor Maximum Echo Range
        Shell Command.
    """
    rsp = self.mRobotCmds.UssCmdSetMaxRange(range);
    if rsp is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', rsp

  def CmdUssSetUnits(self, units):
    """ Set the Hemisson UltraSonic Sensor Measurement Units 
        Shell Command.
    """
    rsp = self.mRobotCmds.UssCmdSetUnits(units);
    if rsp is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', rsp

  def CmdUssGetSettings(self):
    """ Get USS Settings Shell Command. """
    rsp = self.mRobotCmds.UssCmdGetSettings()
    if rsp is None:
      return 'err', self.mRobotCmds.GetErrStr()
    else:
      return 'ok', rsp


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Parameter Converters 
  #   All converters must return the 2-tuple (rc, rval) where:
  #     ('err', <errstr>) or ('ok', <data)>)
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  def CvtZone(self, s):
    """ Convert Linear Camera Zone into Integer Code. """
    if s == 'all' or s == 'a':
      return 'ok', HemiCmdLinCam.LinCamZoneAll
    elif s == 'left' or s == 'l':
      return 'ok', HemiCmdLinCam.LinCamZoneLeft
    elif s == 'middle' or s == 'm':
      return 'ok', HemiCmdLinCam.LinCamZoneMiddle
    elif s == 'right' or s == 'r':
      return 'ok', HemiCmdLinCam.LinCamZoneRight
    else:
      return 'ok', HemiCmdLinCam.LinCamZoneAll

  def CvtUnits(self, s):
    """ Convert USS Units into Character Code. """
    if s == 'inch' or s == 'i':
      return 'ok', 'I'
    elif s == 'cm' or s == 'c':
      return 'ok', 'C'
    elif s == 'usec' or s == 'u':
      return 'ok', 'U'
    else:
      return 'ok', 'C'


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

  sys.exit( HemiRawShell.main( shclass=HemiFullShell,
                               argv0=__file__,
                               shName='Hemisson Full') )
