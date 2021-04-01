################################################################################
#
# TBrain.py
#

""" User Template for a Threaded Virtual Brain.

Instructions:
  Copy this file to your development area, renaming it appropriately.
  Search for the string 'USERACTION' for things you should do and
  USER<x> for things to change.
  Don't forget to add:
    your directory to the 'BrainPluginPath'
    create or update __plugins__.py file entry

Remember, this is just one example of a template for quick development.
See the Core/vBrainThreaded.py for full threaded brain capabilities or
Core/vBrain.py for even more unrestricted designs.

The vBrainThreaded execution framework is defined as: ThinkAct()

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

import Fusion.Gui.GuiDlgExecOpt as GuiDlgExecOpt
import Fusion.Gui.GuiDlgAbout as GuiDlgAbout
import Fusion.Gui.GuiDlgMsgBox as msgbox


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------
# USERACTION: add any specific global data here.


# USERACTION: typical ini definition dictionary. add entries as needed
# USERACTION: if the dd is shared across brains, then move to separate file


# USERACTION: set your brain's mime type here
# Brain MIME Type
BrainMimeType       = 'brain/USERBRAINSUBTYPE'

# USERACTION: add ini sections names here
BrainIniDDSectOpts  = BrainMimeType + '/' + 'options'

# Ini Definition Dictionary.
BrainIniDD = {
  # section
  BrainIniDDSectOpts: ['USERBRAINNAME brain options',
  {
    # USERACTION: Add other ini options section data here

    # Execution
    'ExecCycle':          [0.10, 'Execution think/act cycle time (seconds).'],
    'ExecStepSize':       [1.0, "Execution 'Step' size (seconds)."]
  }],

  # USERACTION: add other ini sections here
}
 

# USERACTION: change USERBRAIN to your class name.
#-------------------------------------------------------------------------------
# CLASS: USERBRAIN
#-------------------------------------------------------------------------------
class USERBRAIN(vBrainThreaded.vBrainThreaded):
  """ User Template for Virtual Brain Threaded Virtual Class. """

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
    # USERACTION: initialize any specific data and resources here

    # standard threaded brain initialization
    vBrainThreaded.vBrainThreaded.vBrainInit(self);

    # brain gui initializations (after vBrainThreaded)
    self.GuiInit()

    # add menu bar
    self.GSSetServerMenuBarList(self.mMenuBarList)

    # add tool bar
    # USERACTION: delete if you do not have a toolbar
    self.GSSetServerToolBarList(self.mToolBarList)
 

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
    # USERACTION: set your brain's name
    return 'USERBRAINNAME'

  #--
  def IsVersion(self):
    """ Returns the vBrain version(s) string.

        Return Value:
          The brain version(s) string.
    """
    # USERACTION: set your brain's version
    return '1.0'

  #--
  def HasDesc(self):
    """ Returns a short description of this vBrain.

        Return Value:
          Multi-line description string.
    """
    # USERACTION: set your brain's short description
    sDesc = """USERBRAINDESC."""
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
    for section,sdata in self.mIniDD.items():
      optdict = sdata[1]
      for option,odata in optdict.items():
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
    # USERACTION: add any 'think and act' specific ini configuration here
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
    # USERACTION: null out any peer to peer data here. this brain should
    # USERACTION: operator without an attached robot.

  #--
  def PeerRobotEstablish(self):
    """ Establish the vBrain to vRobot peer interface when the vRobot
        peer registers with this vBrain.

        Implementation specific.

        Return Value:
          True if vRobot peer is compatible with this vBrain and the
          vRobot has sufficient resources. Else return False.
    """
    # USERACTION: validate this robot
    if self.mPeer.IsType() != 'robot/USERROBOTSUBTYPE':
      self.GSReportErrorStatus(
          'vBrain does not support vRobot type: %s' % self.mPeer.IsType())
      return False

    # null sensor data
    self.PeerRobotNull()

    # USERACTION: add specific brain-robot initialization here
    # search available sensors for the relevant sensors and mark

    # this robot is okay
    return True


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # (Cere)Brum Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def BrumNull(self):
    """ Set the vBrains's high-level states to the null set.

        Return Value:
          Returns new current brum state.
    """
    # user options Brum dependencies
    self.BrumOpts()

    # USERACTION: initialize any brum fixed inate data here
    self.mBrum['USERBRUMKEY1'] = 'USERBRUMDATA1'
    self.mBrum['USERBRUMKEY2'] = 'USERBRUMDATA2'

    return self.mBrum

  #--
  def BrumOpts(self):
    """ 'Fixed' Brum values based on user options. """
    # USERACTION: add any brum option dependent data here
    pass


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vBrain Do's
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def DoEStop(self):
    """ Do an emergency stop of the brain.

        Callback hook for vBrainThreaded emergency stopping event.
    """
    # USERACTION: typically a brain doesn't need a emergency stop.
    # USERACTION: but if it does, add the code here
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
    # USERACTION: add any think-act initialization here
 
  #--
  def ThinkAct(self):
    """ Think-Act hierarchy.

        Execution Context: Cog thread
    """
    if self.mOnHold:
      return

    if __debug__: self.mDbg.d4print('ThinkAct')

    # USERACTION: add think and act operations here


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Gui 
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def GuiInit(self):
    """ Initialize GUI. """
    # USERACTION: add other menu items as needed
    # Menubar list
    self.mMenuBarList = Gluon.GluonMenuBarList()
    self.mMenuBarList.add('Brain|Options...', 'command', 
        callback=self.GuiCbBrainOptions)
    self.mMenuBarList.add('Brain', 'separator')
    self.mMenuBarList.add('Help|About Brain...', 'command', 
        callback=self.GuiCbHelpAbout)

    # Plugin Toolbar list
    # USERACTION: add toolbar items as needed, or delete
    self.mToolBarList = Gluon.GluonToolBarList()
    # USERACTION: self.mToolBarList.add(...)
 
  #--
  def GuiDeinit(self):
    """ Deinitialize GUI objects. """
    # USERACTION: do any gui de-initialization here
    pass

  #--
  def GuiCbBrainOptions(self):
    """ Standard 'Options' menu callback. """
    if __debug__: self.mDbg.d1print('Brain|Options')
    
    section = BrainIniDDSectOpts

    # option defaults
    iniDD   = self.mIniDD
    optDict = iniDD[section][1]
    optDfts = {}
    for option,odata in optDict.items():
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
    # USERACTION: if specific options dialog, change
    dlg = GuiDlgExecOpt.GuiDlgExecOpt(parent,
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
                    descTitle='USERABOUTTITLE',
                    desc="""\
USERABOUTDESCLINE1
...
USERABOUTDESCLINEN""",
                    # USERACTION: logoImage=USERIMAGEFILENAME,
                    copyright='USERCOPYRIGHT')
                    
    # go back to parent gui
    self.GSGuiRaiseParent()
