################################################################################
#
# vBrainOpticFlow.py
#

""" Optical Flow Hunter Virtual Brain Module.

The objective of the OpticFlowBrain is to search for 'prey' and hunt it
down.

When a significant Blob of Interest (BOI) is identified in the hunter's
(robot's) visual processing organ (camera and software), it uses optic
flow of the BOI to drive its neural-muscle system (software plus the left
and right wheel motors) to track and run down its prey (BOI).

The robot under control is a K-Team Hemisson robot with an attached Linear
Camera module. The linear camera has 102 pixels with a horizontal viewing
angle approximately 100 degrees wide. Each camera pixel records a 8-bit 
gray-level value. 

Two DC motors drive the left and right wheels. Steering is accomplished
by varying the power to each wheel motor. The Hemisson if holonomic.
There is no odometry nor other positioning sensors available on the robot.

A Hemisson Text-To-Speech module is also attached to the robot. This
module is used to simply convey the hunter's emotional state in a somewhat
humorous way.

Although the Hemisson also has IR LED proximity sensors, they are not used
by this brain.

The cognitive algorithms of this brain use a dynamic approach (attractor
dynamics) to fuse the IR LED sensor information into a repulsive field,
while the location sensors, along with the circling goal, provide an
attractive field.

This brain is based on optical flow literature. The simple motor control
ideas came from examples in:
  Valentino Braitenberg, 'Vehicles: Experiments in Synthetic Psychology'

Robots inspired by Brainenberg's hypothetical robots are commonly known as
Braitenberg Vehicals or Robots.

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
import threading as thread
import math
import Tkinter as tk

import Fusion.Utils.Tools as utils
import Fusion.Utils.Pt as pt

import Fusion.Core.Gluon as Gluon
import Fusion.Core.vBrain as vBrain
import Fusion.Core.vBrainThreaded as vBrainThreaded

import Fusion.Gui.GuiDlgAbout as GuiDlgAbout
import Fusion.Gui.GuiDlgMsgBox as msgbox

import Fusion.Hemisson.Robots.HemiValues as HemiValues
import Fusion.Hemisson.Cmd.HemiCmdLinCam as HemiLinCam
import Fusion.Hemisson.Cmd.HemiCmdTts as HemiTts

import Fusion.Hemisson.Gui.GuiDlgOpticFlowOpt as GuiDlgOpticFlowOpt
import Fusion.Hemisson.Gui.GuiWinOpticFlowViz as GuiWinOpticFlowViz

import Fusion.Hemisson.Brains.SciPrj as SciPrj


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

BrainMimeType       = 'brain/OpticFlow'
BrainIniDDSectOpts  = BrainMimeType + '/' + 'options'

# Ini Definition Dictionary.
BrainIniDD = {
  # section
  BrainIniDDSectOpts: ['OpticFlow brain options',
  {
    # Design Parameters
    'boi_threshold':      [20,  'Blob of Interest gray-level threshold.'],
    'hunter_speed':       [5,   "Hunter's speed during the hunt"],

    # Execution
    'ExecCycle':          [0.1, 'Execution think/act cycle time (seconds).'],
    'ExecStepSize':       [1.0, "Execution 'Step' size (seconds)."]
  }]
}

# some math constants
pi          = math.pi
halfpi      = math.pi / 2.0
threehalfpi = math.pi * 3.0 / 2.0
twopi       = math.pi * 2.0


#-------------------------------------------------------------------------------
# CLASS: vBrainOpticFlow
#-------------------------------------------------------------------------------
class vBrainOpticFlow(vBrainThreaded.vBrainThreaded):
  """ Optic Flow Hunter vBrain Class. """

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
    # standard threaded brain initialization
    vBrainThreaded.vBrainThreaded.vBrainInit(self);

    # brain gui initializations (after vBrainThreaded)
    self.GuiInit()

    # add menu bar
    self.GSSetServerMenuBarList(self.mMenuBarList)

    # science project initializaton
    SciPrj.SciPrjInit(self)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vBrainOpticFlow Attribute Member Functions
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
    return 'OpticFlowBrain'

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
    sDesc = """Optic Flow Hunter Brain."""
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
    self.mLinCamGroup = HemiValues.HemiSensorMimeTypeLinCam

    self.mPeerRobotCallbacks['bellumsetgoals'] = self.PeerRobotCbNoOp
    self.mPeerRobotCallbacks[self.mLinCamGroup] = self.PeerRobotCbNoOp
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

      # linear camera sensor
      if mimetype == self.mLinCamGroup:
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

    # other callbacks
    self.mPeerRobotCallbacks['say'] = self.mPeer.EffectSay

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
  # vBrainOpticFlow (Cere)Brum Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def BrumNull(self):
    """ Set the vBrains's high-level states to the null set.

        Return Value:
          Returns new current brum state.
    """
    # user options Brum dependencies
    self.BrumOpts()

    # linear camera pixels
    self.mBrum['raw_pixels']     = [0] * HemiLinCam.LinCamNumPixels
    self.mBrum['cooked_pixels']  = [0] * HemiLinCam.LinCamNumPixels

    # blobs
    self.mBrum['blobs']   = {}
    self.mBrum['boi_id']  = None

    # intentions
    self.mBrum['speed_left']  = 0
    self.mBrum['speed_right'] = 0

    return self.mBrum

  #--
  def BrumOpts(self):
    """ 'Fixed' Brum values based on user options. """
    self.mBrum['boi_threshold'] = self.mOpt['boi_threshold']
    self.mBrum['hunter_speed']  = self.mOpt['hunter_speed']


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
    self.GSReportNormalStatus("Searchin' for love.")
 
  #--
  def ThinkAct(self):
    """ Apply optic flow dynamics to hunt and 'capture' prey.

        Execution Context: Cog thread
    """
    if self.mOnHold:
      return

    if __debug__: self.mDbg.d4print('ThinkAct')

    # per execution cycle up fronts
    self.mBrum['dt'] = self.mOpt['ExecCycle']

    # retrieve sensory data
    self.mBrum['raw_pixels'] = \
        self.mPeerRobotCallbacks[self.mLinCamGroup](self.mLinCamGroup)['linear_camera']

    # cooked pixels
    self.mBrum['cooked_pixels'] = self.mBrum['raw_pixels']

    # science project hook 
    SciPrj.SciPrjThinkAct()
 
    self.mPeerRobotCallbacks['bellumsetgoals'](
        speed_left  = self.mBrum['speed_left'],
        speed_right = self.mBrum['speed_right']
    )


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Brain Specialities  
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def Say(self, behavior, spokenText, writtenText=None):
    """ Say the spoken text, showing the written.

        Parameters:
          behavior    - current hemisson behavior
          spokenText  - text to say with the TTS
          writtenText - text to display. Default is spokenText.

        Return Value:
          None
    """
    if spokenText and self.mPeer and \
        self.mPeer.mCmd.TtsCmdQueryState() == HemiTts.TtsStateNotSpeaking:

      # map behavior to displayed text color
      if behavior == 'hunting':
        writtenColor = 'blue'
      elif behavior == 'chasing':
        writtenColor = 'green'
      elif behavior == 'eating':
        writtenColor = 'red'
      else:
        writtenColor = 'black'

      # speak and show
      if writtenText:
        writtenText = behavior + ': ' + writtenText
      else:
        writtenText = behavior + ': ' + spokenText
      self.mPeerRobotCallbacks['say'](spokenText, writtenText, writtenColor)
      if __debug__: self.mDbg.d3print(writtenText)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vBrain Gui 
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def GuiInit(self):
    """ Initialize GUI. """
    # initialize menubar list
    self.mMenuBarList = Gluon.GluonMenuBarList()
    self.mMenuBarList.add('Brain|Options...', 'command', 
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
    if __debug__: self.mDbg.d1print('Brain|Options')
    
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
    settingNames  = GuiDlgOpticFlowOpt.GetSettingNames()
    iniSettings   = ini.IniGetSubItems(section, settingNames)
    lastSettings  = utils.tuples2dict(iniSettings)

    # get parent gui object for this dialog
    parent = self.GSGuiGetParent()

    # the dialog
    dlg = GuiDlgOpticFlowOpt.GuiDlgOpticFlowOpt(parent, optDfts,
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
    """ 'Visualizer' menu callback. """
    if __debug__: self.mDbg.d1print('Brain|Brain Visualizer')
    self.GSGuiWinStart('VizOpticFlow',          # window ID
        GuiWinOpticFlowViz.GuiWinOpticFlowViz,  # start object
        self)                                   # arguments to start

  #--
  def GuiVizUpdate(self, what):
    """ Update Visualization Window if present.
      
        Parameters:
          what  - what to update. 'all' is to update all graphics.
    """
    self.GSGuiWinUpdate('VizOpticFlow', what)

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
                    descTitle='Optic Flow Hunter Virtual Brain',
                    desc="""\
The objective of the OpticFlowBrain is to search for 'prey' and hunt it down.

When a significant Blob of Interest (BOI) is identified in the hunter's
(robot's) visual processing organ (camera and software), it uses optic flow of
the BOI to drive its neural-muscle system (software plus the left and right
wheel motors) to track and capture its prey (BOI).

The robot under control is a K-Team Hemisson robot with an attached Linear
Camera module. The linear camera has 102 pixels with a horizontal viewing angle
approximately 100 degrees wide. Each camera pixel records a 8-bit gray-level
value. 

Two DC motors drive the left and right wheels. Steering is accomplished by
varying the power to each wheel motor. The Hemisson if holonomic.
There is no odometry nor other positioning sensors available on the robot.

A Hemisson Text-To-Speech module is also attached to the robot. This module is
used to simply convey the hunter's emotional state in a somewhat humorous way.

Although the Hemisson also has IR LED proximity sensors, they are not used by
this brain.

This brain is based on optical flow literature. The simple motor control ideas
came from examples in:
  Valentino Braitenberg, 'Vehicles: Experiments in Synthetic Psychology'

Robots inspired by Brainenberg's hypothetical robots are commonly known as
Braitenberg Vehicals or Robots

Sarah Knight and Amanda Whitney designed the Optic Flow algorithms behind this brain
for a science project.""",
                    copyright='RoadNarrows LLC\n(C) 2006')

    # go back to parent gui
    self.GSGuiRaiseParent()


#-------------------------------------------------------------------------------
# Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  def tstCreate(level=2):
    """ Create vHemisson test environment. """
    k = vBrainOpticFlow(debuglevel=level)
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
    """ vBrainOpticFlow Test Main """
    k = tstCreate()
    tstShortRun(k, 2)

  # run test
  main()
