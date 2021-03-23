################################################################################
#
# vBrainDemoPhobia.py
#

""" Hemisson Demophobic Brain 

Ths Hemisson demophopic brain uses the UltraSonic Sensor Module to scan
its environment to detect people (and objects). Anything object within its
comfort zone, causes avoidance and panic. That is, this brain exhibits
demophobia - the fear of people, crowds. Must be an engineer's brain...

The Text-To-Speech module is used to convey the Hemisson's emotional
state.

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

import time
import random
import threading as thread
import math
import Tkinter as tk

import Fusion.Utils.Tools as utils
import Fusion.Utils.SM as SM

import Fusion.Core.Gluon as Gluon
import Fusion.Core.vBrain as vBrain
import Fusion.Core.vBrainThreaded as vBrainThreaded

import Fusion.Gui.GuiDlgAbout as GuiDlgAbout
import Fusion.Gui.GuiDlgMsgBox as msgbox
import Fusion.Gui.GuiDlgExecOpt as GuiDlgExecOpt

import Fusion.Hemisson.Robots.HemiValues as HemiValues
import Fusion.Hemisson.Cmd.HemiCmdUss as HemiUss
import Fusion.Hemisson.Cmd.HemiCmdTts as HemiTts


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

BrainMimeType       = 'brain/DemoPhobia'
BrainIniDDSectOpts  = BrainMimeType + '/' + 'options'

# Ini Definition Dictionary.
BrainIniDD = {
  # section
  BrainIniDDSectOpts: ['DemoPhobia brain options',
  {
    # Execution
    'ExecCycle':          [0.1, 'Execution think/act cycle time (seconds).'],
    'ExecStepSize':       [1.0, "Execution 'Step' size (seconds)."]
  }]
}


# UltraSonic Sensor Noise
ComfortDist             = 200  # comfort distance (zone) mm

# Sound Device
SoundDevUnknown         = 0   # unknown sound device
SoundDevBuzzer          = 1   # internal annoying Base buzzer
SoundDevTts             = 2   # Text-To-Speech module


#-------------------------------------------------------------------------------
# CLASS: vBrainDemoPhobia
#-------------------------------------------------------------------------------
class vBrainDemoPhobia(vBrainThreaded.vBrainThreaded):
  """ Demophobic vBrain Class. """

  #--
  def __init__(self, client=None, debuglevel=0, debugfout=None):
    """ Initialize vBrain instance.

        Parameters:
          client      - Gluon client
          debuglevel  - none to all [0, 5] (default: none)
          debugfout   - opened debug output file (default: stdout)
    """
    # base class
    vBrainThreaded.vBrainThreaded.__init__(self,
        serverId=self.HasName(), client=client,
        debuglevel=debuglevel, debugfout=debugfout)

  #--
  def vBrainInit(self):
    """ One-time vBrain initialization during object instantiation. """
    # Brain State
    self.mSm = SM.SM('SMPhobia', s_init='init',
        smTbl={
          'init': {'opGen':self.SMThinkInit},
          'scan': {'opGen':self.SMThinkScanning},
          'panicback': {'opGen':self.SMThinkPanicBack},
          'panicspin': {'opGen':self.SMThinkPanicSpin},
          'panicforward': {'opGen':self.SMThinkPanicForward},
        })

    # Emotional Fear Whinings
    self.mFearTtsList = [
      'Run fool, run, run, run',
      'Oh my, oh my, oh my',
      'This is not happening',
      'Giants are everywhere',
      'Why me, Why me, Why me',
      "boo hoo boo hoo boo hoo",
      "waaa waaa waa",
      "no, no, no, no",
      "There are people, run"
    ]

    # Emotional Security Dribble
    self.mSafeTtsList = [
      "I am safe",
      "No people is good people",
      "Demophobia is rational"
    ]

    # standard threaded brain initialization
    vBrainThreaded.vBrainThreaded.vBrainInit(self);

    # brain gui initializations (after vBrainThreaded)
    self.GuiInit()

    # add menu bar
    self.GSSetServerMenuBarList(self.mMenuBarList)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vBrainDemoPhobia Attribute Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def IsType(self):
    """ Returns the brain type.

        Return Value:
          The brain MIME type string.
    """
    return BrainMimeType

  #--
  def HasName(self):
    """ Returns the short brain name(s) string.

        Return Value:
          The brain name string.
    """
    return 'DemoPhobiaBrain'

  #--
  def IsVersion(self):
    """ Returns the vBrain version(s) string.

        Return Value:
          The brain version(s) string.
    """
    return '1.0'

  #--
  def HasDesc(self):
    """ Returns a short description of this vBrain.

        Return Value:
          Multi-line description string.
    """
    sDesc = """Demophobic Brain."""
    return sDesc


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vBrain Ini (Re)Initalization Members Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def IniInit(self):
    """ Initialize from parsed 'ini' configuration. """
    self.mIniDD = BrainIniDD
    ini         = self.GSGetIni()

    # load all non-existing ini entries with defaults
    for section,sdata in self.mIniDD.iteritems():
      optdict = sdata[1]
      for option,odata in optdict.iteritems():
        if ini.IniGet(section, option) == ini.NullObj:
          ini.IniSet(section, option, odata[0])

    # load vBrain run-time options
    self.IniInitOpt()

    # load think-act operational settings
    self.IniInitThinkAct()

  #--
  def IniInitOpt(self):
    """ Initialized vBrain options from parsed configuration. """
    iniDD   = self.mIniDD
    ini     = self.GSGetIni()
    section = BrainIniDDSectOpts
    optDict = iniDD[section][1]

    self.mOpt = {}

    for option in optDict:
      self.mOpt[option] = ini.IniGet(section, option)

    self.BrumOpts()

    self.SetExecSizes(self.mOpt['ExecCycle'], self.mOpt['ExecStepSize'])

  #--
  def IniInitThinkAct(self):
    """ Initialized think-act operational settings from parsed
        configuration.
    """
    pass


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vBrainThreaded to vRobot Peer Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def PeerRobotNull(self):
    """ Set the vBrain peer data to the null set.

        Return Value:
          None
    """
    self.mUssGroup = HemiValues.HemiSensorMimeTypeUss

    self.mPeerRobotCallbacks['bellumsetgoals'] = self.PeerRobotCbNoOp
    self.mPeerRobotCallbacks[self.mUssGroup] = self.PeerRobotCbNoOp
    self.mPeerRobotCallbacks['say'] = self.PeerRobotCbNoOp

  #--
  def PeerRobotEstablish(self):
    """ Establish the vBrain to vRobot peer interface when the vRobot
        peer registers with this vBrain.

        Implementation specific.

        Return Value:
          True if vRobot peer is compatible with this vBrain and the
          vRobot has sufficient resources. Else return False.
    """
    # validate robot
    if self.mPeer.IsType() != HemiValues.HemiMimeType:
      self.GSReportErrorStatus(
          'vBrain does not support vRobot type: %s' % self.mPeer.IsType())
      return False

    # null sensor data
    self.PeerRobotNull()

    # search available sensors for the relevant sensors and mark
    sensorDict = self.mPeer.HasSensorTypes()
    hasSufficient = False

    for id,params in sensorDict.iteritems():
      mimetype = params['mimetype']

      # USS sensor
      if mimetype == self.mUssGroup:
          self.mPeerRobotCallbacks[mimetype] = self.mPeer.ShadowGet
          hasSufficient = True

    # not sufficient sensors for this brain
    if not hasSufficient:
      self.GSReportErrorStatus(
          'vBrain does not have sufficient vRobot %s sensors available' % \
              self.mPeer.IsType())
      return False

    # bellum goal setting callback
    self.mPeerRobotCallbacks['bellumsetgoals'] = self.mPeer.BellumSetGoals

    # TTS callbacks
    self.mPeerRobotCallbacks['say'] = self.mPeer.EffectSay

    self.mSoundDev = SoundDevTts

    return True

  #--
  def PeerRobotCbNoOpLoc(self, groupId):
    """ A locaton NoOp vRobot peer callback. All agruments are ignored and
        no true peer callback is performed.
        
        Return Value:
          An empty dictionary {}.
        
        """
    return {'pathspeed':0.0}


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vBrainDemoPhobia (Cere)Brum Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def BrumNull(self):
    """ Set the vBrains's high-level states to the null set.

        Return Value:
          Returns new current brum state.
    """
    # user options Brum dependencies
    self.BrumOpts()

    # intentions
    self.mBrum['speed_left']  = 0
    self.mBrum['speed_right'] = 0
    self.mBrum['pings']       = []
    self.mBrum['motionCnt']   = 0
    self.mBrum['motionLimit'] = 0
    self.mBrum['emo']         = False 

    return self.mBrum

  #--
  def BrumOpts(self):
    """ 'Fixed' Brum values based on user options. """
    if self.mOpt['ExecCycle'] <= 0.5:
      self.mBodySpinRate  = 7
      self.mBodyGoRate    = 9
    else:
      self.mBodySpinRate  = 3
      self.mBodyGoRate    = 5



  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vBrainThreaded Do's
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def DoEStop(self):
    """ Do an emergency stop of the brain.

        Callback hook for vBrainThreaded emergency stopping event.
    """
    pass


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vBrain Think-Act Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def ThinkActInit(self):
    """ Initialize Think-Act Operations.

        Event: Starting event

        Execution Context: Calling thread
    """
    vBrainThreaded.vBrainThreaded.ThinkActInit(self)
    self.GSReportNormalStatus("DemoPhobia")
 
  #--
  def ThinkAct(self):
    """ Apply state transition (think) then act on new state.

        Execution Context: Cog thread
    """
    if self.mOnHold:
      return

    if __debug__: self.mDbg.d4print('ThinkAct')

    # per execution cycle up fronts
    self.mBrum['dt'] = self.mOpt['ExecCycle']

    # retrieve sensory data
    if self.mPeer:
      self.mBrum['pings'] = \
        self.mPeerRobotCallbacks[self.mUssGroup](self.mUssGroup)['uss']
    else:
      self.mBrum['pings'] = []

    # closest ping
    if len(self.mBrum['pings']) > 0:
      cdist = self.mBrum['pings'][0]    # current closest echo
    else:
      cdist = 6000.0

    # brum state transition (think)
    self.mSm.Transition(cdist)

    # act
    self.mPeerRobotCallbacks['bellumsetgoals'](
        speed_left  = self.mBrum['speed_left'],
        speed_right = self.mBrum['speed_right']
    )

    # and act
    if  self.mPeer and \
        self.mPeer.mCmd.TtsCmdQueryState() == HemiTts.TtsStateNotSpeaking:
      if self.mBrum['emo']:
        tts = self.mFearTtsList[random.randrange(0, len(self.mFearTtsList))]
        self.mPeerRobotCallbacks['say'](tts, writtenColor='red')
      else:
        tts = self.mSafeTtsList[random.randrange(0, len(self.mSafeTtsList))]
        self.mPeerRobotCallbacks['say'](tts, writtenColor='green')

  #--
  def SMThinkInit(self, state, cdist):
    motion = random.randrange(0, 2)
    if motion == 0:
      self.mBrum['speed_left'] = random.randrange(-self.mBodySpinRate,
                                                   self.mBodySpinRate+1)
      self.mBrum['speed_right'] = -self.mBrum['speed_left']
      self.mBrum['motionLimit'] = random.randrange(3, 5)
    else:
      self.mBrum['speed_left'] = self.mBrum['speed_right'] = self.mBodyGoRate
      self.mBrum['motionLimit'] = random.randrange(3, 8)
    self.mBrum['motionCnt'] = 0
    return 'scan'

  #--
  def SMThinkScanning(self, state, cdist):
    if cdist != 0 and cdist <= ComfortDist:
      self.mBrum['speed_left'] = self.mBrum['speed_right'] = -self.mBodyGoRate
      self.mBrum['motionCnt'] = 0
      self.mBrum['motionLimit'] = random.randrange(1, 5)
      state = 'panicback'
    elif self.mBrum['motionCnt'] < self.mBrum['motionLimit']:
      self.mBrum['motionCnt'] += 1
    else:
      self.mBrum['motionCnt'] = 0
      state = 'init'
    return state

  #--
  def SMThinkPanicBack(self, state, cdist):
    if self.mBrum['motionCnt'] < self.mBrum['motionLimit']:
      self.mBrum['motionCnt'] += 1
    else:
      motion = random.randrange(0, 2)
      if motion == 0:
        self.mBrum['speed_left'] = -self.mBodySpinRate
      else:
        self.mBrum['speed_left'] = self.mBodySpinRate
      self.mBrum['speed_right'] = -self.mBrum['speed_left']
      self.mBrum['motionCnt'] = 0
      self.mBrum['motionLimit'] = random.randrange(1, 3)
      state = 'panicspin'
    self.mBrum['emo'] = True
    return state

  #--
  def SMThinkPanicSpin(self, state, cdist):
    if self.mBrum['motionCnt'] < self.mBrum['motionLimit']:
      self.mBrum['motionCnt'] += 1
    else:
      self.mBrum['speed_left'] = self.mBrum['speed_right'] = self.mBodyGoRate
      self.mBrum['motionCnt'] = 0
      self.mBrum['motionLimit'] = random.randrange(1, 5)
      state = 'panicforward'
    self.mBrum['emo'] = True
    return state

  #--
  def SMThinkPanicForward(self, state, cdist):
    if cdist != 0 and cdist <= ComfortDist:
      self.mBrum['speed_left'] = self.mBrum['speed_right'] = -self.mBodyGoRate
      self.mBrum['motionCnt'] = 0
      self.mBrum['motionLimit'] = random.randrange(1, 5)
      self.mBrum['emo'] = True
      state = 'panicback'
    else:
      self.mBrum['emo'] = False
      self.mBrum['motionCnt'] += 1
      self.mBrum['motionLimit'] = random.randrange(3, 8)
      state = 'scan'
    return state


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vBrain Gui 
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def GuiInit(self):
    """ Initialize GUI. """
    # initialize menubar list
    self.mMenuBarList = Gluon.GluonMenuBarList()
    self.mMenuBarList.add('Brain|Brain Options...', 'command', 
        callback=self.GuiCbBrainOptions)
    self.mMenuBarList.add('Brain', 'separator')
    self.mMenuBarList.add('Brain|Brain Visualizer', 'command', 
        callback=self.GuiCbBrainViz)
    self.mMenuBarList.add('Help|About Brain...', 'command', 
        callback=self.GuiCbHelpAbout)

    self.mGuiVizWin     = None

  #--
  def GuiDeinit(self):
    """ Deinitialize GUI objects. """
    pass

  #--
  def GuiCbBrainOptions(self):
    """ 'Options' menu callback. """
    if __debug__: self.mDbg.d1print('Brain|Brain Options')
    
    section = BrainIniDDSectOpts

    # option defaults
    iniDD   = self.mIniDD
    optDict = iniDD[section][1]
    optDfts = {}
    for option,odata in optDict.iteritems():
      optDfts[option] = odata[0]

    # get parsed ini configuation (guaranteed to exist)
    ini           = self.GSGetIni()
    section       = BrainIniDDSectOpts
    settingNames  = GuiDlgExecOpt.GetSettingNames()
    iniSettings   = ini.IniGetSubItems(section, settingNames)
    lastSettings  = utils.tuples2dict(iniSettings)

    # get parent gui object for this dialog
    parent = self.GSGuiGetParent()

    # the dialog
    dlg = GuiDlgExecOpt.GuiDlgExecOpt(parent,
                  lastSettings=lastSettings, title=self.HasName()+' Options')

    # Serial connection has been successfully opened
    if dlg.result: 

      opts = dlg.result

      # update ini with current connection settings
      iniSettings = utils.dict2tuples(opts)
      ini.IniSetModifiedItems(section, iniSettings)

      # Re-init settings
      self.IniInitOpt()

    # go back to parent gui
    self.GSGuiRaiseParent()

  #--
  def GuiCbBrainViz(self):
    """ 'Visualize' menu callback. """
    if __debug__: self.mDbg.d1print('Brain|Brain Visualizer')
    msgbox.WarningBox('Not implemented yet.')

  #--
  def GuiCbHelpAbout(self):
    """ 'About' menu callback. """
    if __debug__: self.mDbg.d1print('Help|About Brain')

    # get parent gui object for this dialog
    parent = self.GSGuiGetParent()

    verstr = 'v' + self.IsVersion()

    GuiDlgAbout.GuiDlgAbout(parent,
                    name=self.HasName(),
                    version=verstr,
                    descTitle='Hemisson DemoPhobic Virtual Brain',
                    desc="""\
Ths Hemisson demophopic brain uses the UltraSonic Sensor Module to scan
its environment to detect people (and objects). Anything object within its
comfort zone, causes avoidance and panic. That is, this brain exhibits
demophobia - the fear of people, crowds. Must be an engineer's brain...

The Text-To-Speech module is used to convey the Hemisson's emotional
state.""",
                    copyright='RoadNarrows LLC\n(C) 2006')

    # go back to parent gui
    self.GSGuiRaiseParent()


#-------------------------------------------------------------------------------
# Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  def tstCreate(level=2):
    """ Create vHemisson test environment. """
    k = vBrainDemoPhobia(debuglevel=level)
    k.ExecLoad()
    return k

  def tstShortRun(k, sec=10):
    """ Short run test """
    iv = 0.1
    i = iv
    state = k.GetBrainState()
    if state == Gluon.EServerState.Ready:
      k.ExecStart()
    while i < sec:
      print('thinking', i)
      time.sleep(iv)
      i += iv
    k.ExecUnload()

  def main():
    """ vBrainDemoPhobia Unit Test Main """
    k = tstCreate()
    tstShortRun(k, 2)

  # run test
  main()
