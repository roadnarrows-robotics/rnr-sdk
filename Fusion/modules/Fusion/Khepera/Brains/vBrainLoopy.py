################################################################################
#
# vBrainLoopy.py
#

""" A No-Brainer Virtual Brain Module.

This brain simply loops cw taking advantage of the built-in vKhepera
PID control of its Bellum goals. Environment sensory input is ignored,
so things can go bump.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2006.03.01

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
import math

import Fusion.Utils.Tools as utils

import Fusion.Core.Gluon as Gluon
import Fusion.Core.vBrain as vBrain
import Fusion.Core.vBrainThreaded as vBrainThreaded

import Fusion.Gui.GuiDlgAbout as GuiDlgAbout
import Fusion.Gui.GuiDlgMsgBox as msgbox

import Fusion.Khepera.Robots.KheValues as KheValues
import Fusion.Khepera.Gui.GuiDlgCircleOpt as GuiDlgCircleOpt


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

# Brain MIME Type
BrainMimeType       = 'brain/vBrainLoopy'
BrainIniDDSectOpts  = BrainMimeType + '/' + 'options'

# Ini Definition Dictionary.
BrainIniDD = {
  # section
  BrainIniDDSectOpts: ['vBrainLoopy brain options',
  {
    'speed':          [50.0,  'Khepera speed (mm/s).'],
    'radius':         [100.0, 'Circle radius (mm).'],

    # Execution
    'ExecCycle':      [0.10, 'Execution think/act cycle time (seconds).'],
    'ExecStepSize':   [1.0,  "Execution 'Step' size (seconds)."]
  }],
}
 

#-------------------------------------------------------------------------------
# CLASS: vBrainLoopy
#-------------------------------------------------------------------------------
class vBrainLoopy(vBrainThreaded.vBrainThreaded):
  """ Circling Khepera Brain Class. """

  #--
  def __init__(self, client=None, debuglevel=0, debugfout=None):
    """ Initialize instance.

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


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Attribute Member Functions
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
    return 'vBrainLoopy'

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
    sDesc = """Circling Khepera brain."""
    return sDesc


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Ini (Re)Initalization Members Functions
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

    self.SetExecSizes(self.mOpt['ExecCycle'], self.mOpt['ExecStepSize'])

  #--
  def IniInitThinkAct(self):
    """ Initialized think-act operational settings from parsed
        configuration.
    """
    pass


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vBrain to vRobot Peer Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def PeerRobotNull(self):
    """ Set the vBrain peer data to the null set.

        Return Value:
          None
    """
    self.mPeerRobotCallbacks['bellumsetgoals'] = self.PeerRobotCbNoOp
    self.mPeerRobotCallbacks[KheValues.KheSensorMimeTypeOdometer] = \
        self.PeerRobotCbNoOpLoc

  #--
  def PeerRobotEstablish(self):
    """ Establish the vBrain to vRobot peer interface when the vRobot
        peer registers with this vBrain.

        Implementation specific.

        Return Value:
          True if vRobot peer is compatible with this vBrain and the
          vRobot has sufficient resources. Else return False.
    """
    if self.mPeer.IsType() != KheValues.KheMimeType:
      self.GSReportErrorStatus(
          'vBrain does not support vRobot type: %s' % self.mPeer.IsType())
      return False

    # null sensor data
    self.PeerRobotNull()

    # robot location callback
    self.mPeerRobotCallbacks[KheValues.KheSensorMimeTypeOdometer] = \
        self.mPeer.ShadowGet

    # bellum goal setting callback
    self.mPeerRobotCallbacks['bellumsetgoals'] = self.mPeer.BellumSetGoals

    # this robot is okay
    return True

  #--
  def PeerRobotCbNoOpLoc(self, groupId):
    """ A locaton NoOp vRobot peer callback. All agruments are ignored and
        no true peer callback is performed.
        
        Return Value:
          An empty dictionary {}.
        
        """
    return {'pathdist':0.0, 'theta':0.0, 'x':0.0, 'y':0.0}


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vBrain Do's
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def DoEStop(self):
    """ Do an emergency stop of the brain.

        Callback hook for vBrainThreaded emergency stopping event.
    """
    pass


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Think-Act Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def ThinkActInit(self):
    """ Initialize Think-Act Operations.

        Event: Starting event

        Execution Context: Calling thread
    """
    vBrainThreaded.vBrainThreaded.ThinkActInit(self)
    self.mPeerRobotCallbacks['bellumsetgoals'](speed=self.mOpt['speed'])
    self.GSReportNormalStatus("Khepera's goal is to travel cw in a circle.")

  #--
  def ThinkAct(self):
    """ Don't think, just act.

        Execution Context: Think thread
    """
    if self.mOnHold:
      return

    if __debug__: self.mDbg.d4print('ThinkAct')

    # retrieve robot location
    loc = self.mPeerRobotCallbacks[KheValues.KheSensorMimeTypeOdometer]\
                        (KheValues.KheSensorMimeTypeOdometer)
    dist = loc['pathdist']
    theta = dist / (2.0 * self.mOpt['radius'])
    self.mPeerRobotCallbacks['bellumsetgoals'](theta = -theta,
                                               speed=self.mOpt['speed'])


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Gui 
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def GuiInit(self):
    """ Initialize GUI. """
    # Menubar list
    self.mMenuBarList = Gluon.GluonMenuBarList()
    self.mMenuBarList.add('Brain|Brain Options...', 'command', 
        callback=self.GuiCbBrainOptions)
    self.mMenuBarList.add('Brain', 'separator')
    self.mMenuBarList.add('Help|About Brain...', 'command', 
        callback=self.GuiCbHelpAbout)
 
  #--
  def GuiDeinit(self):
    """ Deinitialize GUI objects. """
    pass

  #--
  def GuiCbBrainOptions(self):
    """ Standard 'Options' menu callback. """
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
    settingNames  = GuiDlgCircleOpt.GetSettingNames()
    iniSettings   = ini.IniGetSubItems(section, settingNames)
    lastSettings  = utils.tuples2dict(iniSettings)

    # get parent gui object for this dialog
    parent = self.GSGuiGetParent()

    # the dialog
    # USERACTION: if specific options dialog, change
    dlg = GuiDlgCircleOpt.GuiDlgCircleOpt(parent,
                  lastSettings=lastSettings,
                  title=self.HasName()+' Options')

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
  def GuiCbHelpAbout(self):
    """ 'About' menu callback. """
    if __debug__: self.mDbg.d1print('Help|About Brain')

    # get parent gui object for this dialog
    parent = self.GSGuiGetParent()

    verstr = self.HasName() + ' v' + self.IsVersion()

    GuiDlgAbout.GuiDlgAbout(parent,
                    name=self.HasName(),
                    version=verstr,
                    descTitle='Loopy Brain',
                    desc="""\
This brain simply loops cw taking advantage of the built-in vKhepera
PID control of its Bellum goals. Environment sensory input is ignored,
so things can go bump.""",
                    copyright='RoadNarrows LLC\n(C) 2006')

                    
    # go back to parent gui
    self.GSGuiRaiseParent()


#-------------------------------------------------------------------------------
# Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  def tstCreate(level=2):
    """ Create vKhepera test environment. """
    k = vBrainLoopy(debuglevel=level)
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
    """ vBrainLoopy Test Main """
    k = tstCreate()
    tstShortRun(k, 3)

  # run test
  main()
