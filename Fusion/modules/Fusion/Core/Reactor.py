################################################################################
#
# Reactor.py
#

""" Robotic Fusion Reactor Module.

Version:  $LastChangedDate: 2010-01-21 10:32:54 -0700 (Thu, 21 Jan 2010) $
Revision: $Rev: 231 $

The Robotic Fusion Reactor fuses the physical/simulated robot, its brain,
the GUI, and all types of reporting into a coherent whole.

There are two threads executing in the Reactor space:
  Tkinter.mainloop()
    - The Tkinter thread that processes all GUI events. mainloop() cannot
      block, otherwise other Fusion threads issuing Tkinter calls will
      block until the mainloop() can process the calls, but it is blocked...
  Mirror
    - The Fusion Reactor event processing thread. This thread is used
      to prevent potential deadlocks in the mainloop().

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.11.15

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

import  os
import  sys
import  time
import  threading as thread
import  math

import  Tkinter as tk
import  tkMessageBox
import  tkFont

import  Fusion.Core.Values as Values
import  Fusion.Core.Mirror as Mirror

import  Fusion.Utils.Enum as enum
import  Fusion.Utils.Tools as utils

import  Fusion.Gui.GuiTypes as gt
import  Fusion.Gui.GuiUtils as gut
import  Fusion.Gui.GuiDlgMsgBox as msgbox
import  Fusion.Gui.GuiMenuBar as GuiMenuBar
import  Fusion.Gui.GuiToolBar as GuiToolBar
import  Fusion.Gui.GuiTextBar as GuiTextBar
import  Fusion.Gui.GuiDlgOpen as GuiDlgOpen
import  Fusion.Gui.GuiDlgSaveAs as GuiDlgSaveAs
import  Fusion.Gui.GuiDlgFusionPref as GuiDlgFusionPref
import  Fusion.Gui.GuiDlgPlugin as GuiDlgPlugin
import  Fusion.Gui.GuiDlgDebug as GuiDlgDebug
import  Fusion.Gui.GuiWinIniListing as GuiWinIniListing
import  Fusion.Gui.GuiDlgAbout as GuiDlgAbout
import  Fusion.Gui.GuiDlgAskYesOrNo as GuiDlgAskYesOrNo

import  Fusion.Core.FusionIniDD as FusionIniDD
import  Fusion.Core.Gluon as Gluon
import  Fusion.Core.vRobot as vRobot
import  Fusion.Core.vBrain as vBrain

#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

# Fusion Application Name
FusionAppName  = 'Reactor'

#
# Correlated vRobot and vBrain Server State Matrix.
# This matrix is symetrical about the diagonal (m_ij = m_ji). Hence, 
# the highest state is the row index, while the lower state is the column
# index.
# Server states are:
#   None NotLoaded NotReady Ready Paused Stepping Running Errored
#
BadCorrState  = -1
_CorrStateLowerMatrix = [
  # None
  [Gluon.EServerState.None],
  # NotLoaded
  [Gluon.EServerState.NotLoaded,    Gluon.EServerState.NotLoaded],
  # NotReady
  [Gluon.EServerState.NotReady,     Gluon.EServerState.NotLoaded,
   Gluon.EServerState.NotReady],
  # Ready
  [Gluon.EServerState.Ready,        Gluon.EServerState.NotLoaded,
   Gluon.EServerState.NotReady,     Gluon.EServerState.Ready],
  # Paused
  [Gluon.EServerState.Paused,       BadCorrState,
   BadCorrState,                    BadCorrState,
   Gluon.EServerState.Paused],
  # Stepping
  [Gluon.EServerState.Stepping,     BadCorrState,
   BadCorrState,                    Gluon.EServerState.Ready,
   Gluon.EServerState.Paused,       Gluon.EServerState.Stepping],
  # Running
  [Gluon.EServerState.Running,      BadCorrState,
   BadCorrState,                    Gluon.EServerState.Ready,
   Gluon.EServerState.Paused,       Gluon.EServerState.Stepping,
   Gluon.EServerState.Running],
  # Errored
  [Gluon.EServerState.Errored,      Gluon.EServerState.Errored,
   Gluon.EServerState.Errored,      Gluon.EServerState.Errored,
   Gluon.EServerState.Errored,      Gluon.EServerState.Errored,
   Gluon.EServerState.Errored,      Gluon.EServerState.Errored]
]

# Empty Server Plugin Dictionary
_EmptyPlugin = {
  'pluginPath':       None,
  'pluginEPointName': None,
  'pluginMod':        None,
  'pluginEPoint':     None
}

#
# Fusion Core Actions
#   None      no current action
#   Plugin    select a server plug-in 
#   EStop     emergency stop all servers
#   Load      load all plugged in servers
#   Start     start all servers running
#   Resume    resume all servers running after being suspended
#   Pause     pause (suspend) all servers from running
#   Step      step all servers one execution cycle
#   Stop      stop all servers and 'rewind' to ready
#   Unload    unload all servers
#   Unplug    deselect a server plug-in
#   Quit      quit fusion
#
EAction = enum.Enum(
  'None Plugin Load EStop Start Resume Pause Step Stop Unload Unplug Quit')


#-------------------------------------------------------------------------------
# CLASS: Reactor
#-------------------------------------------------------------------------------
class Reactor(Gluon.GluonClient):
  """ Robotic Fusion Reactor Class.

      Supported Fusion Reactor client-server models:
        1. Reactor(client) <--> vRobot(server)

        2. Reactor(client) <--> vBrain(server)

        3.           Reactor(client) 
                     /            \\
           vBrain(server)  <-->  vRobot(server)
                          (peer)

      Note: These models will change in Fusion 1.0 to a more generalized
            Gluon Peer model.
  """

  def __init__(self, vRobot=None, vBrain=None, iniFileName=None,
                     debuglevel=0, debugfout=None):
    """ Initialize Fusion instance.

        Parameters:
          vRobot      - instance of [derived] virtual robot
          vBrain      - instance of [derived] virtual brain
          iniFileName - additional ini configuration filename
          debuglevel  - none to all [0, 5] (default: none)
          debugfout   - opened debug output file (default: stdout)
    """
    Values.InitOnce()

    Gluon.GluonClient.__init__(self, clientId=FusionAppName,
                                     debuglevel=debuglevel,
                                     debugfout=debugfout)

    # Most basic initialization
    self.Init()

    # Load configuration data
    self.IniInit(iniFileName)

    # Draw GUI
    self.GuiInitBody()

    # Turn on Gui Messaging Services
    self.StartGuiMessagingServices()

    # Set all menubar states
    self.GuiSetAllStates()

    # Post-Initialize and show user any relevant messages (after gui is up)
    self.PostInit()

    # Auto-plugin the servers (command-line argument(s) take precedence)
    if vBrain or vRobot:
      self.PluginRobot(vRobot=vRobot)  # robot server object
      self.PluginBrain(vBrain=vBrain)  # brain server object
    else:
      self.IniAutoPlugin()          # robot and brain from ini

  #--
  def HasName(self):
    """ Returns the Fusion 'official name. """
    return FusionAppName

  #--
  def IsVersion(self):
    """ Returns this Fusion's version string.  """
    return Values.FusionVersion

  #--
  def Init(self):
    """ The most basic, one-time Fusion initialzation. """
    self.mTitle       = FusionAppName # title of this fusion
    self.mRobotPlugin = _EmptyPlugin  # robot plugin dictionary 
    self.mRobot       = None          # the instantiated robot
    self.mBrainPlugin = _EmptyPlugin  # brain plugin dictionary 
    self.mBrain       = None          # the instantiated brain
    self.mGmsEnabled  = False         # turn off Gui Messaging Services
                                      #  during initialization
    self.mCurAction   = EAction.None  # no current fusion action
    self.mCorrState   = Gluon.EServerState.None # correlated servers' state

  #--
  def PostInit(self):
    """ Initialize any data and show the user any pertinate settings
        after the gui is initialized.
    """
    self.GuiSetTitle()
    self.ShowMsg('Configuration read from: %s' % repr(self.mIniSessIniList))
    if self.mIniSessSaveFileName:
      self.ShowMsg('Current save configuration file is %s' % \
          repr(self.mIniSessSaveFileName))
    self.GuiShowRobotInfo()
    self.GuiShowBrainInfo()

    # start Reactor Mirror event dispatcher
    self.MirrorInit()


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Fusion Play Execcutive Controls
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  def PushEStop(self):
    if __debug__: self.mDbg.d1print('PushEStop')
    self.ShowMsg(' * Pushed Emergency Stop *')
    self.mCurAction = EAction.EStop
    if self.HasRobot():
      self.mRobot.ExecEStop()
    if self.HasBrain():
      self.mBrain.ExecEStop()
    self.mCurAction = EAction.None

  def PushPlay(self):
    if __debug__: self.mDbg.d1print('PushPlay')
    if self.mCorrState == Gluon.EServerState.Ready:
      self.ShowMsg(' * Pushed Start Play *')
      self.mCurAction = EAction.Start
      if self.HasRobot():
        self.mRobot.ExecStart()
      if self.HasBrain():
        self.mBrain.ExecStart()
    elif self.mCorrState == Gluon.EServerState.Paused:
      self.ShowMsg(' * Pushed Resume Play *')
      self.mCurAction = EAction.Resume
      if self.HasRobot():
        self.mRobot.ExecResume()
      if self.HasBrain():
        self.mBrain.ExecResume()
    self.mCurAction = EAction.None

  def PushPause(self):
    if __debug__: self.mDbg.d1print('PushPause')
    self.ShowMsg(' * Pushed Pause *')
    self.mCurAction = EAction.Pause
    if self.HasRobot():
      self.mRobot.ExecSuspend()
    if self.HasBrain():
      self.mBrain.ExecSuspend()
    self.mCurAction = EAction.None

  def PushStep(self):
    if __debug__: self.mDbg.d1print('PushStep')
    self.ShowMsg(' * Pushed Step *')
    self.mCurAction = EAction.Step
    if self.HasRobot():
      self.mRobot.ExecStep()
    if self.HasBrain():
      self.mBrain.ExecStep()
    self.mCurAction = EAction.None

  def PushStop(self):
    if __debug__: self.mDbg.d1print('PushStop')
    self.ShowMsg(' * Pushed Stop *')
    self.mCurAction = EAction.Stop
    if self.HasRobot():
      self.mRobot.ExecStop()
    if self.HasBrain():
      self.mBrain.ExecStop()
    self.mCurAction = EAction.None

  def PushLoadUnload(self):
    if __debug__: self.mDbg.d1print('PushLoadUnload')
    if not self.HasRobotOrBrain():
      return
    if self.mCorrState == Gluon.EServerState.NotLoaded:
      self.ShowMsg(' * Pushed Load *')
      self.mCurAction = EAction.Load
      self.LoadServers()
    else:
      self.ShowMsg(' * Pushed Unload *')
      self.mCurAction = EAction.Unload
      self.UnloadServers()
    self.mCurAction = EAction.None


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Fusion Client Override Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def GCGuiGetParent(self):
    """ Get Fusion's parent GUI object.

        Return Value:
          Returns the GUI object.
    """
    return self.mGuiRoot

  def GCGuiRaiseParent(self):
    """ Raise GUI parent object to front of display.

        Return Value:
          None
    """
    if self.mGuiRoot:
      self.mGuiRoot.tkraise()

  #--
  def GCGuiWinGetOptions(self, serverId, winId):
    """ Retrieves any saved options for the server Gui window,
        plus the core GuiWin options.

        Parameters:
          serverId  - server identification
          winId     - Gui window unique id

        Return Value:
          **options - list of previous configuration options (if any),
                      plus GuiWin core options.
    """
    return self.GuiChildWinGetOptions(serverId, winId)

  #--
  def GCGuiWinRegister(self, serverId, winId, win):
    """ Register server GUI window with the client.

        Parameters:
          serverId  - server identification
          winId     - Gui window unique id. None if no id.
          win       - Gui window object

        Return Value:
          None
    """
    self.GuiChildWinRegister(serverId, winId, win)

  #--
  def GCGuiWinUnregister(self, serverId, winId, win, **saveOpts):
    """ Unregister server GUI window with the client.

        Parameters:
          serverId  - registered server id string
          winId     - Gui window unique id. None if no id.
          win       - Gui window object
          saveOpts  - window keyword configuration options to save 

        Return Value:
          None
    """
    self.GuiChildWinUnregister(serverId, winId, win, **saveOpts)

  #--
  def GCGuiWinStart(self, serverId, winId, callobj, *args, **kwargs):
    """ Start a server child window. Any ini saved configuration is 
        retrieved and added to the keyword arguments. The started 
        window is automatically registered with the client.

        Parameters:
          serverId  - server identification
          winId     - Gui window unique id. None if no id.
          callobj   - callable object to start the window. Must return
                      Gui window object.
          args      - arguments passed to callobj()
          kwargs    - keyword arguments passed to callobj().

        Return Value:
          Gui window object
    """
    return self.GuiChildWinStart(serverId, winId, callobj, *args, **kwargs)

  #--
  def GCGuiWinUpdate(self, serverId, winId, *args, **kwargs):
    """ Make callback to server child window update function.

        Parameters:
          serverId  - server identification
          winId     - gui window unique id.
          *args     - arguments to update callback
          *kwargs   - keyword arguments to update callback

        Return Value:
          None
    """
    self.GuiChildWinUpdate(serverId, winId, *args, **kwargs)

  #--
  def GCReportServerStateChanged(self, serverId, newState):
    """ React to server state change. """
    msg = '%s: %s(%d)' % (serverId, Gluon.EServerState.name(newState), newState)
    if __debug__: self.mDbg.d3print(msg)
    self.ShowMsg(msg)
    self.mCorrState = self.CorrServerStates()
    self.GuiSetAllStates()
    self.Prompt()

  #--
  def GCReportNormalStatus(self, serverId, msg):
    """ Received a normal status message from registered Gluon server.
        
        Parameters:
          serverId  - registered server id string
          msg       - server message string

        Return Value:
          None
    """
    self.ShowMsg('%s: %s' % (serverId, msg))

  #--
  def GCReportErrorStatus(self, serverId, emsg):
    """ Received an error status message from registered Gluon server.
        
        Parameters:
          serverId  - registered server id string
          emsg      - server error message string

        Return Value:
          None
    """
    self.ShowError('%s: %s' % (serverId, emsg))


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Fusion Ini Configuration Member Functions 
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def IniInit(self, iniOther):
    """ Open and parse ini file(s) setting defaults to any missing data.
        Ini associated session data (not saved) is also initialized.
        From the ini settings, basic configuration actions are performed.

        Parameters:
          iniOther  - addition ini file name to parse

        Return Value:
          None.
    """
    # standard ini locations
    iniFusion   = Values.FusionIniFileName
    iniStartup  = utils.canonicalpath(Values.FusionEnvVarStartup)
    iniHome     = Values.FusionUserIniFileName
    iniOther    = utils.canonicalpath(iniOther)

    # build ini list
    self.mIniSessSaveFileName = None
    iniList = []
    if iniFusion:
      iniList.append(iniFusion)
    if iniStartup and iniStartup not in iniList:
      iniList.append(iniStartup)
      self.mIniSessSaveFileName = iniStartup
    if iniHome and iniHome not in iniList:
      iniList.append(iniHome)
      self.mIniSessSaveFileName = iniHome
    if iniOther and iniOther not in iniList:
      iniList.append(iniOther)
      self.mIniSessSaveFileName = iniOther

    ini = self.GCGetIni()   # get ini parsed object
    ini.IniRemoveAll()      # remove all old sections and options
    ini.IniOpen(iniList)    # parse ini files in order
    ini.ClearModifiedFlag() # clear ini modifications flag

    # ini session list
    self.mIniSessIniList = iniList

    # fusion ini definition dictionary
    self.mIniDD = FusionIniDD.GetIniDD()

    # Load all non-existing ini entries with defaults
    for section,sdata in self.mIniDD.iteritems():
      optdict = sdata[1]
      for option,odata in optdict.iteritems():
        if ini.IniGet(section, option) == ini.NullObj:
          ini.IniSet(section, option, odata[0])

    # session debug file name and # pointer
    self.mIniSessDebugFile = ('<stdout>', sys.stdout)

    # session initial starting browse directories for 'Plugin' dialog
    self.mIniSessRobotPluginInitialDir  = '.'
    self.mIniSessBrainPluginInitialDir  = '.'

    # set Fusion preferences
    self.IniInitPref()

    # set debugging configuration
    self.IniInitDebug()

    # add ini on-modification callback
    ini.SetOnModifyCallback(self.IniCbOnModify)

  #--
  def IniInitPref(self):
    """ Initialize fusion preferences from ini configuration. """
    ini = self.GCGetIni()
    section = FusionIniDD.IniDDSectMain
    self.mPref = {}
    settingNames = GuiDlgFusionPref.GetSettingNames()
    # set pref: Note all ini option,value's should be set by this time
    for name in settingNames:
      self.mPref[name] = ini.IniGet(section, name)

  #--
  def IniInitDebug(self):
    """ Initialize debugging from ini configuration. """
    ini = self.GCGetIni()
    section = FusionIniDD.IniDDSectMain
    dbg = {}
    settingNames = GuiDlgDebug.GetSettingNames()
    for name in settingNames:
      if name == 'DebugFileName':
        dbg[name] = ini.IniGetOrDft(section, name, '<stdout>')
      else:
        dbg[name] = ini.IniGetOrDft(section, name, 0)
    self.DebugSet(dbg['DebugLevelFusion'], dbg['DebugLevelRobot'], 
                  dbg['DebugLevelBrain'], dbg['DebugFileName'])

  #--
  def IniAutoPlugin(self):
    """ Auto-plugin any servers specified in the parsed ini configuration. """
    if not self.mPref['AutoPlugin']:  # option disabled
      return

    ini = self.GCGetIni()
    section = FusionIniDD.IniDDSectMain

    # vRobot
    option      = 'RobotPlugin'
    pluginList  = ini.IniGet(section, option)
    if pluginList != None:
      if pluginList and type(pluginList) == list and len(pluginList) == 2:
        modFile    = utils.canonicalpath(pluginList[0])
        ePointName = utils.canonicalexpr(pluginList[1])
        dictPlugin = self._IniPlugin(option, modFile, ePointName, 
                                    'vRobot', vRobot.vRobot)
        if dictPlugin is not None:
          self.PluginRobot(vRobotPlugin=dictPlugin)
      else:
        self.ShowError('%s: badly formed plugin value: %s' % \
            (option, repr(pluginList)))

    # vBrain
    option      = 'BrainPlugin'
    pluginList  = ini.IniGet(section, option)
    if pluginList != None:
      if pluginList and type(pluginList) == list and len(pluginList) == 2:
        modFile    = utils.canonicalpath(pluginList[0])
        ePointName = utils.canonicalexpr(pluginList[1])
        dictPlugin = self._IniPlugin(option, modFile, ePointName, 
                                    'vBrain', vBrain.vBrain)
        if dictPlugin is not None:
          self.PluginBrain(vBrainPlugin=dictPlugin)
      else:
        self.ShowError('%s: badly formed plugin value: %s' % \
            (option, repr(pluginList)))

  #--
  def _IniPlugin(self, option, modFile, ePointName, ifName, ifClass):
    """ Auto-plugion helper plugs in module and verifies the
        interface.

        Parameters:
          option      - ini plugin option name
          modFile     - python module fully qualified file name
          ePointName  - entry point name in modFile
          ifName      - name of expected entry point interface
          ifClass     - expected entry point base class interface

        Return Value:
          On success, return plugin dictionary.
          On failure, return None.
    """
    if not modFile:
      self.ShowError('%s: no python filename specified.' % option)
      return None
    elif not ePointName:
      self.ShowError('%s: %s: no entry point name specified.' % \
          (option, modFile))
      return None
    if type(modFile) != str or type(ePointName) != str:
      self.ShowError('%s: badly formed plugin value: [%s, %s]' % \
          (option, repr(modFile), repr(ePointName)))
      return None

    mod = os.path.basename(modFile)
    modbase, modext = os.path.splitext(mod)

    # split off any '.py' extension
    if not modbase:
      self.ShowError("%s: badly formed module filename: %s" % (option, modFile))
      return None

    # Try importing plugin module
    try:
      plugin = utils.importmodule(modFile, modbase)
    except ImportError, err:
      self.ShowError('%s: %s: %s' % (option, modFile, err))
      return None
    except:
      self.ShowError('%s: %s: %s' % (option, modFile, sys.exc_info()[0]))
      return None

    # Try attaching to any entry point
    try:
      ePoint = eval('plugin.'+ePointName)
    except AttributeError:
      utils.delmodule(plugin)
      del plugin
      self.ShowError('%s: %s: %s attribute not found' % \
          (option, modFile, repr(ePointName)))
      return None

    # verify the interface
    try:
      isif = issubclass(ePoint, ifClass)
    except TypeError, err:
      self.ShowError('%s: %s plugin entry point %s: not a class.' % \
          (option, modFile, repr(ePointName)))
      del ePoint
      utils.delmodule(plugin)
      return None
    if not isif:
      self.ShowError("%s: %s plugin entry point %s: "
                     'not a derived %s class.' % \
              (option, modFile, repr(ePointName), repr(ifName)))
      del ePoint
      utils.delmodule(plugin)
      return None

    # if here, then the ini plugin is good
    dictPlugin = {
      'pluginPath':       modFile,
      'pluginEPointName': ePointName,
      'pluginMod':        plugin,
      'pluginEPoint':     ePoint
    }
    return dictPlugin

  #--
  def IniLastMinutes(self):
    """ Ini last minute configuration updates given current settings.
        Typically this is called just prior to saving.
    """
    ini = self.GCGetIni()
    section = FusionIniDD.IniDDSectMain
    if self.mPref['AutoSavePlugins']:
      modPath     = self.mRobotPlugin['pluginPath']
      ePointName  = self.mRobotPlugin['pluginEPointName']
      if modPath and ePointName:
        ini.IniSet(section, 'RobotPlugin', [modPath, ePointName])
      else:
        ini.IniSet(section, 'RobotPlugin', None)
      modPath     = self.mBrainPlugin['pluginPath']
      ePointName  = self.mBrainPlugin['pluginEPointName']
      if modPath and ePointName:
        ini.IniSet(section, 'BrainPlugin', [modPath, ePointName])
      else:
        ini.IniSet(section, 'BrainPlugin', None)

  #--
  def IniSave(self, filename):
    """ Save ini configuration to file. """
    ini = self.GCGetIni()
    if not filename:
      return
    self.IniLastMinutes()
    try:
      ini.IniWrite(filename)
    except IOError, err:
      msgbox.ErrorBox(err)
      self.ShowError(err)
      return
    ini.ClearModifiedFlag()
    self.ShowMsg('Configuration saved to %s' % filename)

  #--
  def IniCbOnModify(self, modifiedFlag):
    """ Callback to mark that the parsed ini data has [not] been modified.

        Parameters:
          modifiedFlag  - ini has [not] been modified

        Return Value:
          None
    """
    self.GuiSetFileStates()
    self.GuiSetTitle()


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Fusion vBrain and vRobot Member Functions 
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def _DelPlugin(self, server, dictPlugin):
    """ Delete vRobot or vBrain plugin. """
    if server:
      del server
    if dictPlugin['pluginEPoint']:
      del dictPlugin['pluginEPoint']
    if dictPlugin['pluginMod']:
      utils.delmodule(dictPlugin['pluginMod'])
    return _EmptyPlugin

  #--
  def PluginRobot(self, vRobot=None, vRobotPlugin=_EmptyPlugin):
    """ Plug-in the vRobot attached module and entry point.
        
        Parameters:
          vRobot        - instantiated vRobot object
          vRobotPlugin  - vRobot plugin dictionary

        Return Value:
          None
    """
    self.mCurAction = EAction.Plugin

    # delete old robot (better be in a stopped state)
    self.mRobotPlugin = self._DelPlugin(self.mRobot, self.mRobotPlugin)

    # get debug level
    debuglevel = self.GCGetIni().IniGetOrDft(FusionIniDD.IniDDSectMain,
                                             'DebugLevelRobot', 0)

    self.mRobotPlugin   = vRobotPlugin

    if vRobot:
      self.mRobot = vRobot
      self.mRobot.GSSetDebugLevel(debuglevel,
                                  debugfout=self.mIniSessDebugFile[1])
    elif self.mRobotPlugin['pluginEPoint']:
      self.mRobot = self.mRobotPlugin['pluginEPoint'](
                                      debuglevel=debuglevel,
                                      debugfout=self.mIniSessDebugFile[1])
    else:
      self.mRobot = None

    if self.mRobot:
      serverId = self.mRobot.GSGetServerId()
      self.GCRegisterServer(serverId, self.mRobot)
      self.mRobot.GSRegisterClient(self)
      if self.mPref['AutoSavePlugins']:
        self.GCGetIni().SetModifiedFlag()

    self.mCurAction = EAction.None

    self.GuiSetPluginStates()
    self.GuiShowRobotInfo()
    self.Prompt()

  #--
  def UnplugRobot(self):
    """ Unplug any attached vRobot.

        Return Value:
          None
    """
    self.mCurAction = EAction.Unplug
    serverId = self.mRobot.GSGetServerId()
    self.mRobot.GSUnregisterClient()
    self.GCUnregisterServer(serverId)
    self.mRobotPlugin = self._DelPlugin(self.mRobot, self.mRobotPlugin)
    self.mRobot = None
    self.mCurAction = EAction.None
    self.GuiSetPluginStates()
    self.GuiShowRobotInfo()
    if self.mPref['AutoSavePlugins']:
      self.GCGetIni().SetModifiedFlag()
    self.Prompt()

  #--
  def PluginBrain(self, vBrain=None, vBrainPlugin=_EmptyPlugin):
    """ Plug-in vBrain the attached module and entry point.
        
        Parameters:
          vBrain        - instantiated vBrain object
          vBrainPlugin  - vBrain plugin dictionary

        Return Value:
          None
    """
    self.mCurAction = EAction.Plugin

    # delete old robot (better be in a stopped state)
    self.mBrainPlugin = self._DelPlugin(self.mBrain, self.mBrainPlugin)

    # get debug level
    debuglevel = self.GCGetIni().IniGetOrDft(FusionIniDD.IniDDSectMain,
                                            'DebugLevelBrain', 0)

    self.mBrainPlugin   = vBrainPlugin

    if vBrain:
      self.mBrain = vBrain
      self.mBrain.GSSetDebugLevel(debuglevel,
                                  debugfout=self.mIniSessDebugFile[1])
    elif self.mBrainPlugin['pluginEPoint']:
      self.mBrain = self.mBrainPlugin['pluginEPoint'](
                                      debuglevel=debuglevel,
                                      debugfout=self.mIniSessDebugFile[1])
    else:
      self.mBrain = None

    if self.mBrain:
      serverId = self.mBrain.GSGetServerId()
      self.GCRegisterServer(serverId, self.mBrain)
      self.mBrain.GSRegisterClient(self)
      if self.mPref['AutoSavePlugins']:
        self.GCGetIni().SetModifiedFlag()

    self.mCurAction = EAction.None

    self.GuiSetPluginStates()
    self.GuiShowBrainInfo()
    self.Prompt()

  #--
  def UnplugBrain(self):
    """ Unplug any attached vBrain.

        Return Value:
          None
    """
    self.mCurAction = EAction.Unplug
    serverId = self.mBrain.GSGetServerId()
    self.mBrain.GSUnregisterClient()
    self.GCUnregisterServer(serverId)
    self.mBrainPlugin = self._DelPlugin(self.mBrain, self.mBrainPlugin)
    self.mBrain = None
    self.mCurAction = EAction.None
    self.GuiSetPluginStates()
    self.GuiShowBrainInfo()
    if self.mPref['AutoSavePlugins']:
      self.GCGetIni().SetModifiedFlag()
    self.Prompt()

  #--
  def HasRobotOrBrain(self):
    """ Returns True if there is a robot or brain, False otherwise. """
    if self.mRobot or self.mBrain:
      return True
    else:
      return False

  #--
  def HasRobot(self):
    """ Returns True if there is a robot, False otherwise. """
    if self.mRobot:
      return True
    else:
      return False

  #--
  def HasBrain(self):
    """ Returns True if there is a brain, False otherwise. """
    if self.mBrain:
      return True
    else:
      return False

  #--
  def GetRobotState(self):
    """ Returns current robot server state. """
    if not self.HasRobot():
      return Gluon.EServerState.None
    else:
      return self.mRobot.GSGetServerState()

  #--
  def GetBrainState(self):
    """ Returns current brain server state. """
    if not self.HasBrain():
      return Gluon.EServerState.None
    else:
      return self.mBrain.GSGetServerState()
  
  #--
  def CorrServerStates(self):
    """ Correlate the vBrain and vRobot states.

        Return Value:
          Correlated (joint) server state.
    """
    robotState    = self.GetRobotState()
    brainState    = self.GetBrainState()
    oldCorrState  = self.mCorrState

    #
    # A drastic 'going down' action, so choose the state with the highest value.
    #
    if self.mCurAction in [EAction.Stop, EAction.Unload, EAction.Quit]:
      if robotState >= brainState:
        self.mCorrState = robotState
      else:
        self.mCorrState = brainState

    #
    # All other actions correlate well. 
    #
    elif robotState >= brainState:
      self.mCorrState = _CorrStateLowerMatrix[robotState][brainState]
    else:
      self.mCorrState = _CorrStateLowerMatrix[brainState][robotState]

    # correlated states error
    if self.mCorrState == BadCorrState:
      msg = 'Bad correlated states: vRobot=%s(%d), vBrain=%s(%d)' % \
              (repr(Gluon.EServerState.name(robotState)), robotState,
               repr(Gluon.EServerState.name(brainState)), brainState)
      self.ShowError(msg)
      if __debug__: self.mDbg.d2print(msg)
      self.mCorrState = Gluon.EServerState.Errored

    # report only changes
    elif self.mCorrState != oldCorrState:
      msg = 'Correlated state: %s(%d)' % \
          (Gluon.EServerState.name(self.mCorrState), self.mCorrState)
      self.ShowMsg(msg)
      if __debug__: self.mDbg.d2print(msg)

    return self.mCorrState


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Fusion Gluon General Server Member Functions 
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def LoadServers(self):
    """ Load the vBrain and/or vRobot Gluon Servers.

        Return Value:
          None
    """
    if self.HasRobot():
      self.mMirror.AsyncRequest('server_load', server=self.mRobot,
                                     peer=self.mBrain)

    if self.HasBrain():
      self.mMirror.AsyncRequest('server_load', server=self.mBrain,
                                     peer=self.mRobot)

  #--
  def UnloadServers(self):
    """ Unload the vBrain and the vRobot Gluon Servers.

        Execution Context: mainloop thread

        Return Value:
          None.
    """
    # queue request to Mirror to unload brain server
    self.mMirror.AsyncRequest('server_unload', server=self.mBrain)

    # queue request to Mirror to unload robot server
    self.mMirror.AsyncRequest('server_unload', server=self.mRobot)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Fusion Mirror Dispatch Member Functions 
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def MirrorInit(self):
    """ Initialize the Fusion Reactor Mirror Thread.

        Return Value:
          None.
    """
    # create Mirror thread
    self.mMirror = Mirror.Mirror(self.mGuiRoot, dbgobj=self.mDbg)

    # register mirror requests
    self.mMirror.AsyncRequestRegister('server_load', self.MirrorCbLoadServer)
    self.mMirror.AsyncRequestRegister('server_unload',
        self.MirrorCbUnloadServer)
    self.mMirror.AsyncRequestRegister('fusion_quit', self.MirrorCbQuit)

  #--
  def MirrorCbLoadServer(self, request, **kwargs):
    """ Load the Gluon server.

        Return Value:
          None
    """
    # parse event request
    server = kwargs['server']
    peer = kwargs['peer']

    # no server specified
    if not server:
      return

    # load the server
    server.ExecLoad()

    # register the server's peer
    if peer:
      if not server.GSRegisterPeer(peer):   # incompatible peer
        self.ShowError('%s - %s incompatibility.' % \
            (server.HasName(), peer.HasName()))
        return

    # initialize server GUI interface add-ons to Fusion
    self.ServerGuiInit(server)

  #--
  def MirrorCbUnloadServer(self, request, **kwargs):
    """ Unload the Gluon server.

        Execution Context: Mirror thread

        Parameters:
          **kwargs  - keyword=value arguments. Arguments used:
              server=server   - [derived] GluonServer object

        Return Value:
          None.
    """
    # parse event request
    server = kwargs['server']

    # no server specified
    if not server:
      return

    # no server loaded
    elif server.GSGetServerState() in [Gluon.EServerState.None,
                                       Gluon.EServerState.NotLoaded]:
      return

    # unload the server
    server.ExecUnload()

    # wait for the server to unload
    maxtries = 5
    timewait = 0.1
    tries = 0
    while tries < maxtries:
      if server.GSGetServerState() == Gluon.EServerState.NotLoaded:
        break
      time.sleep(timewait)
      tries += 1
    if tries >= maxtries:
      self.ShowError("%s did not unload after %3.1f seconds" % \
                      (server.HasName(), maxtries * timewait))

    # unregister peer
    server.GSUnregisterPeer()

    # go back to mainloop() context
    self.mGuiRoot.after_idle(self._UnloadServerGui, server)

  #--
  def _UnloadServerGui(self, server):
    """ Server GUI deinitialization. """
    self.ServerGuiUnloadMenuBarItems(server)
    self.ServerGuiUnloadToolBarItems(server)
    self.GuiChildWinDestroyOwners(server.GSGetServerId())

  #--
  def MirrorCbQuit(self, request):
    """ Mirror call to quit Fusion's GUI.

        Execution Context: Mirror thread

        Parameters:
          **kwargs  - keyword arguments. Arguments used: None
                
        Return Value:
          None.
    """
    # go back to mainloop() context
    self.mGuiRoot.after_idle(self._GuiDestroy)

  #--
  def ServerGuiInit(self, server):
    """ Initialize the server's GUI.

        Parameters:
          server  - [derived] GluonServer object

        Return Value:
          None.
    """
    self.ServerGuiLoadMenuBarItems(server)
    self.ServerGuiLoadToolBarItems(server)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Fusion vBrain and vRobot GUI Member Functions 
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def ServerGuiLoadMenuBarItems(self, server):
    """ Load the server's menubar items.

        Parameters:
          server  - [derived] GluonServer object

        Return Value:
          None.
    """
    serverId = server.GSGetServerId()
    mbList   = server.GSGetServerMenuBarList()
    for mbItem in mbList:
      self.mMenuBar.AddMenuItem(mbItem['path'], mbItem['type'], owner=serverId,
                                disabledStates=mbItem['disabledStates'],
                                command=mbItem['command'])

  #--
  def ServerGuiUnloadMenuBarItems(self, server):
    """ Unload the server's menubar items.

        Parameters:
          server  - [derived] GluonServer object

        Return Value:
          None.
    """
    serverId = server.GSGetServerId()
    self.mMenuBar.DelMenuItemsByOwner(serverId)

  #--
  def ServerGuiLoadToolBarItems(self, server):
    """ Load the server's toolbar items.

        Parameters:
          server  - [derived] GluonServer object

        Return Value:
          None.
    """
    serverId = server.GSGetServerId()
    tbList   = server.GSGetServerToolBarList()
    if tbList:
      self.GuiToolBarNew(serverId, serverId, tbList)

  #--
  def ServerGuiUnloadToolBarItems(self, server):
    """ Unload the server's toolbar items.

        Parameters:
          server  - [derived] GluonServer object

        Return Value:
          None.
    """
    serverId = server.GSGetServerId()
    self.GuiToolBarDel(serverId, serverId)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Fusion Other Member Functions 
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def MakeIniDD(self):
    """ Make 'ini' Definition Dictionary by merging the client with
        all server dictionaries.
    """
    # Fusion client 'ini' definition dictionary
    fusionDD = FusionIniDD.GetIniDD()

    # Merge robot's DD
    if self.HasRobot():
      robotDD = self.mRobot.GSGetServerIniDD()
      fusionDD = FusionIniDD.MergeIniDD(fusionDD, robotDD)

    # Merge brain's DD
    if self.HasBrain():
      brainDD = self.mBrain.GSGetServerIniDD()
      fusionDD = FusionIniDD.MergeIniDD(fusionDD, brainDD)

    return fusionDD

  #--
  def DebugSet(self, dlFusion, dlRobot, dlBrain, newFileName):
    """ Set debug levels and debug output of Fusion and any connected
        vBrain and vRobot servers.

        Parameters:
          dlFusion    - Fusion debug level
          dlRobot     - vRobot debug level
          dlBrain     - vBrain debug level
          newFileName - new debug output filename

        Return Value:
          None
    """
    oldFileName, oldFout = self.mIniSessDebugFile
    if oldFileName != '<stdout>' and oldFileName != '<stderr>' and \
        oldFileName != newFileName:
      self.DebugSetAllLevels(0, 0, 0) # shut it down, before closing
      oldFout.close()
      self.mIniSessDebugFile = ('<stdout>', sys.stdout)
    if newFileName == '<stdout>':
      self.mIniSessDebugFile = ('<stdout>', sys.stdout)
    elif newFileName == '<stderr>':
      self.mIniSessDebugFile = ('<stderr>', sys.stderr)
    else:
      try:
        fp = open(newFileName, 'w')
        self.mIniSessDebugFile = (newFileName, fp)
      except IOError, err:
        self.ShowError('Debug file %s: using <stdout>' % err)
        self.mIniSessDebugFile = ('<stdout>', sys.stdout)
    self.DebugSetAllLevels(dlFusion, dlRobot, dlBrain, 
        self.mIniSessDebugFile[1])

  #--
  def DebugSetAllLevels(self, dlFusion, dlRobot, dlBrain, debugfout=None):
    """ Set debug levels and debug output stream of Fusion and any connected
        vBrain and vRobot servers.

        Parameters:
          dlFusion    - Fusion debug level
          dlRobot     - vRobot debug level
          dlBrain     - vBrain debug level
          debugfout   - opened debug output stream 

        Return Value:
          None
    """
    self.GCSetDebugLevel(dlFusion, debugfout)
    if self.HasBrain():
      self.mBrain.GSSetDebugLevel(dlBrain, debugfout)
    if self.HasRobot():
      self.mRobot.GSSetDebugLevel(dlRobot, debugfout)

  #--
  def StartGuiMessagingServices(self):
    """ Initialize and start Gui Message Services. """
    self.mGmsEnabled  = True 
    self.mCurPrompt   = None
    self.Prompt()

  #--
  def Prompt(self):
    """ Prompt user for an action. Prompt message is determined by the
        current states of Fusion.

        RDK!!! need current action

        Return Value:
          None
    """
    if not self.mGmsEnabled:
      return

    if self.mCorrState == Gluon.EServerState.None:
      prompt = "Select a vRobot and/or a vBrain plug-in"
    elif self.mCorrState == Gluon.EServerState.NotLoaded:
      if not self.HasRobotOrBrain():
        prompt = "Select a vRobot and/or a vBrain plug-in"
      elif not self.HasBrain():
        prompt = "Select a vBrain plug-in or push 'Load' to initialize"
      elif not self.HasRobot():
        prompt = "Select a vRobot plug-in or push 'Load' to initialize"
      else:
        prompt = "Push 'Load' to initialize"
    elif self.mCorrState == Gluon.EServerState.NotReady:
      if self.HasRobot() and self.HasBrain():
        prompt = "Enable the vRobot and the vBrain"
      elif self.HasBrain():
        prompt = "Enable the vBrain"
      elif self.HasRobot():
        prompt = "Enable the vRobot"
    elif self.mCorrState == Gluon.EServerState.Ready:
      prompt = "Push 'Play' or 'Step' to run"
    elif self.mCorrState == Gluon.EServerState.Running:
      prompt = "Push 'Suspend' or 'Step' to suspend, 'Stop' to start over"
    elif self.mCorrState == Gluon.EServerState.Paused:
      prompt = "Push 'Play' or 'Step' to resume, 'Stop' to start over"
    elif self.mCorrState == Gluon.EServerState.Errored:
      prompt = "Push 'Unload' to clear the fatal error(s)"
    else:
      prompt = ""

    # prompt, but avoid redundancies
    if prompt != self.mCurPrompt:
      self.mCurPrompt = prompt
      self.ShowPrompt(prompt)

  #--
  def ShowPrompt(self, pmsg):
    """ Show prompt message on the history bar and the prompt bar.
        
        Parameters:
          pmsg  - prompt message string

        Return Value:
          None
     """
    if not self.mGmsEnabled:
      return

    if pmsg:
      self.mHistoryBar.ShowPrompt(pmsg)
      self.mPromptBar.ShowPrompt(pmsg)
    else:
      self.mPromptBar.ShowNormalStatus('')  # clear prompt bar

  def ShowMsg(self, msg):
    """ Show normal message on the history bar.
        
        Parameters:
          msg  - message string

        Return Value:
          None
     """
    if self.mGmsEnabled:
      self.mHistoryBar.ShowNormalStatus(msg)
    else:
      print msg

  def ShowError(self, emsg):
    """ Show error message on the history bar.
        
        Parameters:
          emsg  - error message string

        Return Value:
          None
     """
    if self.mGmsEnabled:
      self.mHistoryBar.ShowErrorStatus(emsg)
    else:
      print >>sys.stderr, emsg
    if __debug__: self.mDbg.d2print(emsg)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Fusion Gui 
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def GuiInitBody(self):
    """ Draw the GUI interface. """
    self.mGuiRoot = tk.Tk()
    self.mGuiRoot.wm_title(self.mTitle)
    self.mGuiRoot.protocol('WM_DELETE_WINDOW', self.GuiCbQuit)
    
    # lists of disabled states
    self._GuiInitDisabledList()

    # the gui
    self.GuiMenuBarInit()
    self.GuiToolBarInit(1)
    self.GuiVizFeedbackInit(1)
    self.GuiPluginFrameInit(2)
    self.GuiHistoryBarInit(3)
    self.GuiPromptBarInit(4)

    # child window control init
    self.GuiChildWinInit()

    # calculate important window and widget dimensions used for resizing
    self._GuiCalcDim()

    # bind resizer
    self.mGuiRoot.bind('<Configure>', self.GuiCbResize )

    self.GuiSetFusionGeo()

  #--
  def _GuiCalcDim(self):
    """ Caculate widget dimensions needed for resizing effort. """
    # force idletask to determine size
    self.mGuiRoot.update_idletasks()

    # current window dimensions
    self.mWinGeo = gut.geometry(self.mGuiRoot)

    # history bar widget dimensions
    textWidget = self.mHistoryBar.GetTextWidget()
    textgeo = gut.geometry(textWidget)
    self.mSbWOffset = self.mWinGeo[0] - textgeo[0]
    self.mSbHOffset = self.mWinGeo[1] - textgeo[1]

    # prompt bar widget dimensions (info entries will key off of this dimension)
    textWidget = self.mPromptBar.GetTextWidget()
    textgeo = gut.geometry(textWidget)
    self.mPbWOffset = self.mWinGeo[0] - textgeo[0]

  #--
  def GuiSetFusionGeo(self):
    """ Set fusion window geometry based on parsed ini. """
    geostr    = None
    ini       = self.GCGetIni()
    section   = FusionIniDD.IniDDSectWin
    option    = 'Win_Fusion'
    settings  = ini.IniGetOrDft(section, option, None)
    if settings:
      if settings.has_key('geometry'):
        geostr = settings['geometry']
    if geostr:
      try:
        self.mGuiRoot.wm_geometry(geostr)
      except tk.TclError, msg:
        self.ShowError('%s: %s' % (option, msg))

  #--
  def _GuiInitDisabledList(self):
    """ Initialize GUI menu/button disable list """
    plugin_corr_disable = [
      Gluon.EServerState.NotReady,
      Gluon.EServerState.Ready,
      Gluon.EServerState.Paused,
      Gluon.EServerState.Stepping,
      Gluon.EServerState.Running,
      Gluon.EServerState.Errored]

    # tag: {statespace: statelist, ...}
    self._GuiDisableList = {
      'file_open': {
        'plugin': ['norobot', 'nobrain', 'allplugins']
      },
      'file_save': {
        'file': ['nofile']
      },
      'exec_play': {
        'corr': [Gluon.EServerState.None,
                 Gluon.EServerState.NotLoaded, 
                 Gluon.EServerState.NotReady,
                 Gluon.EServerState.Stepping,
                 Gluon.EServerState.Running,
                 Gluon.EServerState.Errored]
      },
      'exec_pause': {
        'corr': Gluon.ServerInvalidExecStateTbl[Gluon.EServerExec.Suspend]
      },
      'exec_step': {
        'corr': Gluon.ServerInvalidExecStateTbl[Gluon.EServerExec.Step]
      },
      'exec_stop': {
        'corr': Gluon.ServerInvalidExecStateTbl[Gluon.EServerExec.Stop]
      },
      'exec_estop': {
        'corr': Gluon.ServerInvalidExecStateTbl[Gluon.EServerExec.EStop]
      },
      'exec_load_unload': {
        'plugin': ['noplugins'],
      },
      'server_plugin': {
        'corr': plugin_corr_disable
      },
      'robot_unplug': {
        'plugin': ['norobot', 'noplugins'],
        'corr':   plugin_corr_disable
      },
      'brain_unplug': {
        'plugin':['nobrain', 'noplugins'],
        'corr':   plugin_corr_disable
      },
    }

  #--
  def GuiMenuBarInit(self):
    """ Initialize menubar. """

    # new menubar
    self.mMenuBar = GuiMenuBar.GuiMenuBar(self.mGuiRoot)

    # File menubar items
    self.mMenuBar.AddMenuItem('File', 'cascade', owner='root')
    self.mMenuBar.AddMenuItem('File|Open Ini...', 'command', owner='root',
        command=self.GuiCbFileOpenIni,
        disabledStates=self._GuiDisableList['file_open'])
    self.mMenuBar.AddMenuItem('File|Save Ini', 'command', owner='root',
        command=self.GuiCbFileSaveIni,
        disabledStates=self._GuiDisableList['file_save'])
    self.mMenuBar.AddMenuItem('File|Save Ini As...', 'command', owner='root',
        command=self.GuiCbFileSaveIniAs)
    self.mMenuBar.AddMenuItem('File', 'separator', owner='root')
    self.mMenuBar.AddMenuItem('File|Exit', 'command', owner='root',
        command=self.GuiCbQuit)

    # Edit menubar items
    self.mMenuBar.AddMenuItem('Edit|Copy', 'command', owner='root',
        command=self.GuiCbEditCopy)
    self.mMenuBar.AddMenuItem('Edit', 'separator', owner='root')
    self.mMenuBar.AddMenuItem('Edit|Select All', 'command', owner='root',
        command=self.GuiCbEditSelectAll)
    self.mMenuBar.AddMenuItem('Edit', 'separator', owner='root')
    self.mMenuBar.AddMenuItem('Edit|Preferences', 'command', owner='root',
        command=self.GuiCbEditPreferences)

    # Exec menubar items
    self.mMenuBar.AddMenuItem('Exec', 'cascade', owner='root')
    self.mMenuBar.AddMenuItem('Exec|Play', 'command', owner='root',
        command=self.PushPlay,
        disabledStates=self._GuiDisableList['exec_play'])
    self.mMenuBar.AddMenuItem('Exec|Pause', 'command', owner='root',
        command=self.PushPause,
        disabledStates=self._GuiDisableList['exec_pause'])
    self.mMenuBar.AddMenuItem('Exec|Step', 'command', owner='root',
        command=self.PushStep,
        disabledStates=self._GuiDisableList['exec_step'])
    self.mMenuBar.AddMenuItem('Exec|Stop', 'command', owner='root',
        command=self.PushStop,
        disabledStates=self._GuiDisableList['exec_stop'])
    self.mMenuBar.AddMenuItem('Exec', 'separator', owner='root')
    self.mMenuBar.AddMenuItem('Exec|Emergency Stop', 'command', owner='root',
        command=self.PushEStop,
        disabledStates=self._GuiDisableList['exec_estop'])
    self.mMenuBar.AddMenuItem('Exec', 'separator', owner='root')
    self.mMenuBar.AddMenuItem('Exec|Load/Unload', 'command', owner='root',
        command=self.PushLoadUnload,
        disabledStates=self._GuiDisableList['exec_load_unload'])

    # Robot menubar items
    self.mMenuBar.AddMenuItem('Robot', 'cascade', owner='root')
    self.mMenuBar.AddMenuItem('Robot|Select...', 'command', owner='root',
        command=self.GuiCbRobotSelect,
        disabledStates=self._GuiDisableList['server_plugin'])
    self.mMenuBar.AddMenuItem('Robot|Deselect', 'command', owner='root',
        command=self.GuiCbRobotDeselect,
        disabledStates=self._GuiDisableList['robot_unplug'])
    self.mMenuBar.AddMenuItem('Robot', 'separator', owner='root')

    # Brain menubar items
    self.mMenuBar.AddMenuItem('Brain', 'cascade', owner='root')
    self.mMenuBar.AddMenuItem('Brain|Select...', 'command', owner='root',
        command=self.GuiCbBrainSelect,
        disabledStates=self._GuiDisableList['server_plugin'])
    self.mMenuBar.AddMenuItem('Brain|Deselect', 'command', owner='root',
        command=self.GuiCbBrainDeselect,
        disabledStates=self._GuiDisableList['brain_unplug'])
    self.mMenuBar.AddMenuItem('Brain', 'separator', owner='root')

    # Tools menubar items
    self.mMenuBar.AddMenuItem('Tools|Debug...', 'command', owner='root',
        command=self.GuiCbToolsDebug)
    self.mMenuBar.AddMenuItem("Tools|Env...", 'command', owner='root',
        command=self.GuiCbToolsEnv)
    self.mMenuBar.AddMenuItem('Tools', 'separator', owner='root')
    self.mMenuBar.AddMenuItem("Tools|'Ini' Listing", 'command', owner='root',
        command=self.GuiCbToolsIniListing)
    self.mMenuBar.AddMenuItem('Tools|Gluon Tree', 'command', owner='root',
        command=self.GuiCbToolsGluonTree)
    self.mMenuBar.AddMenuItem('Tools|PerfMeter', 'command', owner='root',
        command=self.GuiCbToolsPerfMeter)
    # comment out when not using
    #self.mMenuBar.AddMenuItem("Tools|RDK", 'command', owner='root',
    #    command=self.GuiCbToolsRDK)

    # Help menubar items
    self.mMenuBar.AddMenuItem('Help|About...', 'command', owner='root',
        command=self.GuiCbHelpAbout)
    self.mMenuBar.AddMenuItem('Help|About RoadNarrows...', 'command', 
        owner='root',
        command=self.GuiCbHelpAboutRoadNarrows)
    self.mMenuBar.AddMenuItem('Help|EULA...', 'command', owner='root',
        command=self.GuiCbHelpEULA)
    self.mMenuBar.AddMenuItem('Help', 'separator', owner='root')

  #--
  def GuiToolBarInit(self, row):
    """ Initialize toolbar. """
    tbframe = tk.Frame(self.mGuiRoot, relief=tk.FLAT, borderwidth=1)
    tbframe.grid(row=row, column=0, padx=3, ipadx=3, ipady=3, 
               sticky=tk.W)

    self.mToolBarFrame  = tbframe
    self.mToolBarColumn = 0
    self.mToolBars      = {}
    self.mToolBarSpace  = {}

    row = 0
    column = 0

    # Plugin Toolbar
    tbList = Gluon.GluonToolBarList()
    tbList.add('plugin_robot', self.GuiCbRobotSelect, 
        tooltip='Select a vRobot plug-in.',
        disabledStates=self._GuiDisableList['server_plugin'],
        imagefile=gut.GetFusionImageFileName('Robot.gif'))
    tbList.add('plugin_brain', self.GuiCbBrainSelect, 
        tooltip='Select a vBrain plug-in.',
        disabledStates=self._GuiDisableList['server_plugin'],
        imagefile=gut.GetFusionImageFileName('Brain.gif'))
    self.GuiToolBarNew('root', 'plugin', tbList)

    # Play Executives Toolbar
    tbList = Gluon.GluonToolBarList()
    tbList.add('play', self.PushPlay, 
        tooltip='Start/Resume Play execution',
        disabledStates=self._GuiDisableList['exec_play'],
        imagefile=gut.GetFusionImageFileName('Play.gif'))

    tbList.add('pause', self.PushPause, 
        tooltip='Pause execution',
        disabledStates=self._GuiDisableList['exec_pause'],
        imagefile=gut.GetFusionImageFileName('Pause.gif'))

    tbList.add('step', self.PushStep, 
        tooltip="Step execution for 'Step Size' seconds",
        disabledStates=self._GuiDisableList['exec_step'],
        imagefile=gut.GetFusionImageFileName('Step.gif'))

    tbList.add('stop', self.PushStop, 
        tooltip='Stop exectuion, rewinding to start',
        disabledStates=self._GuiDisableList['exec_stop'],
        imagefile=gut.GetFusionImageFileName('Stop.gif'))

    tbList.add('eject', self.PushLoadUnload, 
        tooltip='Load any selected vRobot and/or vBrain',
        disabledStates=self._GuiDisableList['exec_load_unload'],
        imagefile=gut.GetFusionImageFileName('Load.gif'),
        altStates={'corr': [Gluon.EServerState.NotReady,
                            Gluon.EServerState.Ready,
                            Gluon.EServerState.Paused,
                            Gluon.EServerState.Stepping,
                            Gluon.EServerState.Running,
                            Gluon.EServerState.Errored]},
        alttooltip='Unload any selected vRobot and/or vBrain',
        altimagefile=gut.GetFusionImageFileName('Eject.gif'))

    tbList.addsep(2)

    tbList.add('estop', self.PushEStop, 
        tooltip='Emergency stop',
        disabledStates=self._GuiDisableList['exec_estop'],
        imagefile=gut.GetFusionImageFileName('EStop.gif'))

    self.GuiToolBarNew('root', 'exec', tbList)

  #--
  def GuiToolBarNew(self, ownerId, tbid, tbList):
    """ Create new toolbar and add to list of toolbars.

        Parameters:
          ownerId - owner Id string
          tbid    - toolbar unique Id string
          tbList  - GluonToolBarList object
    """
    toolBar = GuiToolBar.GuiToolBar(self.mToolBarFrame,
                                    column=self.mToolBarColumn)
    for item in tbList:
      if item['type'] == 'button':
        toolBar.AddButton(item['label'], item['command'],
                          owner=ownerId, **item['options'])
      elif item['type'] == 'sep':
        toolBar.AddSpace(item['space'])
    self.mToolBars[tbid] = toolBar

    self.mToolBarColumn += 1

    # Spacer
    e = tk.Label(self.mToolBarFrame, text='  ')
    e.grid(row=0, column=self.mToolBarColumn)
    self.mToolBarSpace[tbid] = e

    self.mToolBarColumn += 1

  #--
  def GuiToolBarDel(self, ownerId, tbid):
    """ Create new toolbar and add to list of toolbars.

        Parameters:
          ownerId - owner Id string
          tbid    - toolbar unique Id string
    """
    if self.mToolBars.has_key(tbid):
      del self.mToolBars[tbid]
      del self.mToolBarSpace[tbid]
      self.mToolBarColumn -= 2

  #--
  def GuiPluginFrameInit(self, row):
    """ Initialize plugin frame. """
    pframe = tk.Frame(self.mGuiRoot, relief=tk.RAISED, borderwidth=1)
    pframe.grid(row=row, column=0, padx=3, ipadx=3, ipady=3, 
               sticky=tk.N+tk.S+tk.W+tk.E)

    font = tkFont.Font(pframe, font=gt.FontHelv10Bold)

    row = 0
    column = 0

    self.mEntryRobotInfo = tk.Entry(pframe, relief=tk.SUNKEN, width=84,
        font=font, state='readonly')
    self.mEntryRobotInfo.grid(row=row, column=column, padx=0, pady=0,
        sticky=tk.W+tk.E)

    row += 1

    self.mEntryBrainInfo = tk.Entry(pframe, relief=tk.SUNKEN, width=84,
        font=font, state='readonly')
    self.mEntryBrainInfo.grid(row=row, column=column, padx=0, pady=0,
        sticky=tk.W+tk.E)

  #--
  def GuiHistoryBarInit(self, row):
    """ Initialize history bar. """
    sbframe = tk.Frame(self.mGuiRoot, relief=tk.RAISED, borderwidth=1)
    sbframe.grid(row=row, column=0, padx=3, ipadx=3, ipady=3, 
               sticky=tk.N+tk.S+tk.W+tk.E)

    self.mHistoryBar = GuiTextBar.GuiTextBar(sbframe, height=5)

  #--
  def GuiPromptBarInit(self, row):
    """ Initialize prompt bar. """
    pbframe = tk.Frame(self.mGuiRoot, relief=tk.RAISED, borderwidth=1)
    pbframe.grid(row=row, column=0, padx=3, ipadx=3, ipady=3, 
               sticky=tk.N+tk.S+tk.W+tk.E)

    self.mPromptBar = GuiTextBar.GuiTextBar(pbframe, height=1, width=84,
                        maxHistory=1, lineNums=False)

  #--
  def GuiVizFeedbackInit(self, row):
    """ Initialize visual feedbacks of status """
    playImages = []
    for n in range(0,4):
      filename = gut.GetFusionImageFileName('BlockyWalk%d.gif' % n)
      playImages += [filename]
    notloadedImage = gut.GetFusionImageFileName('BlockyNotLoaded.gif')
    disabledImage = gut.GetFusionImageFileName('BlockyDisabled.gif')
    stoppedImage  = gut.GetFusionImageFileName('BlockyStopped.gif')
    erroredImage = gut.GetFusionImageFileName('BlockyErrored.gif')
    w = gut.ActiveImageWidget(self.mGuiRoot,
        period=0.25,
        activesets = {
          'notloaded':[notloadedImage],
          'disabled':[disabledImage],
          'errored':[erroredImage],
          'stopped':[stoppedImage],
          'playing': playImages
        },
        activetag='notloaded')
    w.grid(row=row, column=0, sticky=tk.E)
    self.mVizExecState = w

  #--
  def GuiSetAllStates(self):
    """ Set all menubar and toolbar items to NORMAL/DISABLE states. """
    # File states
    self.GuiSetFileStates()

    # Plugin states
    self.GuiSetPluginStates()

    # Correlated server states
    self.GuiSetCorrStates()

    # Robot states
    if self.HasRobot():
      robotId    = self.mRobot.GSGetServerId()
      robotState = self.GetRobotState()
      self.GuiSetServerStates(robotId, robotState)

    # Brain states
    if self.HasBrain():
      brainId    = self.mBrain.GSGetServerId()
      brainState = self.GetBrainState()
      self.GuiSetServerStates(brainId, brainState)

    # Visual status states
    self.GuiSetVizFeedbackStates()

  #--
  def GuiSetFileStates(self):
    """ Set 'file' menubar and toolbar items to NORMAL/DISABLED states. """
    ini = self.GCGetIni()
    if not ini.IsModified():
      state = 'nochanges'
    elif not self.mIniSessSaveFileName:
      state = 'nofile'
    else:
      state = 'cansave'
    self.mMenuBar.SetMenuItemStates('file', state, 'File')
    for tb in self.mToolBars.itervalues():
      tb.SetButtonStates('file', state)

  #--
  def GuiSetPluginStates(self):
    """ Set 'plugin' menubar and toolbar items to NORMAL/DISABLED states. """
    if not self.HasRobotOrBrain():
      state = 'noplugins'
    elif not self.HasRobot():
      state = 'norobot'
    elif not self.HasBrain():
      state = 'nobrain'
    else:
      state = 'allplugins'
    self.mMenuBar.SetMenuItemStates('plugin', state)
    for tb in self.mToolBars.itervalues():
      tb.SetButtonStates('plugin', state)

  #--
  def GuiSetCorrStates(self):
    """ Set 'corr' menubar and toolbar items to NORMAL/DISABLED states. """
    self.mMenuBar.SetMenuItemStates('corr', self.mCorrState)
    for tb in self.mToolBars.itervalues():
      tb.SetButtonStates('corr', self.mCorrState)

  #--
  def GuiSetServerStates(self, serverId, state):
    """ Set serverId menubar and toolbar items to NORMAL/DISABLED states. """
    self.mMenuBar.SetMenuItemStates(serverId, state)
    for tb in self.mToolBars.itervalues():
      tb.SetButtonStates(serverId, state)

  #--
  def GuiSetVizFeedbackStates(self):
    """ Set visual user-feedback states """
    if self.mCorrState in [Gluon.EServerState.Running, 
                           Gluon.EServerState.Stepping]:
      self.mVizExecState.SetActive('playing')
    elif self.mCorrState in [Gluon.EServerState.Ready,
                             Gluon.EServerState.Paused]:
      self.mVizExecState.SetActive('stopped')
    elif self.mCorrState in [Gluon.EServerState.None,
                             Gluon.EServerState.NotLoaded]:
      self.mVizExecState.SetActive('notloaded')
    elif self.mCorrState in [Gluon.EServerState.NotReady]:
      self.mVizExecState.SetActive('disabled')
    else:
      self.mVizExecState.SetActive('errored')

  #--
  def GuiSetTitle(self):
    """ Set Fusion title. """
    modifiedFlag = self.GCGetIni().IsModified()
    title = self.mTitle
    if self.mIniSessSaveFileName:
      title += ' - %s' % self.mIniSessSaveFileName 
    if modifiedFlag:
      title += ' (+)'
    self.mGuiRoot.wm_title(title)

  #--
  def GuiShowRobotInfo(self):
    """ Show current robot information. """
    if self.mRobot:
      info = 'vRobot:  %s v%s (%s)' % \
        (self.mRobot.HasName(), self.mRobot.IsVersion(), self.mRobot.IsType())
    else:
      info = 'vRobot:  <none>'
    self.mEntryRobotInfo['state'] = tk.NORMAL
    self.mEntryRobotInfo['fg'] = gt.ColorBlack
    self.mEntryRobotInfo.delete(0, tk.END)
    self.mEntryRobotInfo.insert(0, info)
    self.mEntryRobotInfo['state'] = 'readonly'

  #--
  def GuiShowBrainInfo(self):
    """ Show current brain information. """
    if self.mBrain:
      info = 'vBrain:  %s v%s (%s)' % \
        (self.mBrain.HasName(), self.mBrain.IsVersion(), self.mBrain.IsType())
    else:
      info = 'vBrain:  <none>'
    self.mEntryBrainInfo['state'] = tk.NORMAL
    self.mEntryBrainInfo['fg'] = gt.ColorBlack
    self.mEntryBrainInfo.delete(0, tk.END)
    self.mEntryBrainInfo.insert(0, info)
    self.mEntryBrainInfo['state'] = 'readonly'

  #--
  def GuiQueryUserAndSave(self):
    """ If configuration changes exist, query user for action, and
        save is desired.

        Return Value:
          'ok'      - Continue normal actions after this dialog because
                      the user selecting 'yes' or 'no'.
          'cancel'  - Abort normal actions after this dialog because
                      the user selected 'cancel'.
    """
    if not self.GCGetIni().IsModified():
      return 'ok'
    if self.mIniSessSaveFileName:
      if self.mPref['AutoSave']:
        ans = 'yes'
      else:
        dlg = GuiDlgAskYesOrNo.GuiDlgAskYesOrNo(self.mGuiRoot,
                'Save Current Configuration',
                'Current configuration has changed. Save?',
                addcancel=True)
        ans = dlg.result
      if ans == 'yes':
        self.IniSave(self.mIniSessSaveFileName)
    else:
      dlg = GuiDlgAskYesOrNo.GuiDlgAskYesOrNo(self.mGuiRoot,
          'Save Current Configuration As...',
          'Current configuration has changed. Save AS?',
          addcancel=True)
      ans = dlg.result
      if ans == 'yes':
        self.GuiCbFileSaveIniAs()
    if ans in ['yes', 'no']:
      return 'ok'
    else:
      return 'cancel'


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Fusion Gui Child Window Management
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def GuiChildGetOptions(self, ownerId, winId):
    """ Retrieves any saved options for the child Gui window,
        plus the core GuiWin options.

        Parameters:
          ownerId   - owner Id string
          winId     - Gui window unique id string. None if no id.

        Return Value:
          **options - list of previous configuration options (if any),
                      plus GuiWin core options. No options indicates
                      an error.
    """
    if ownerId == FusionAppName:
      ondestroy = self.GuiChildWinFusionUnregister
    else:
      server = self.GCGetServer(ownerId)
      if not server:
        self.ShowError('Bad server id: %s' % repr(ownerId))
        return {}
      ondestroy = server.GSGuiWinUnregister

    # get any ini previously saved options
    options = self.GuiChildGetIniOptions(ownerId, winId)

    # non-ini options
    options[Values.FusionCWinKeyOnDestroy] = ondestroy
    if winId:
      options[Values.FusionCWinKeyWinId] = winId

    return options

  #--
  def GuiChildGetIniOptions(self, ownerId, winId):
    """ Retrieves any previously saved ini options for the child Gui window.

        Ini Section Format:
          [Fusion/windows]
          CWin_<ownerId> = {<winId>:{geometry:geo, ...} ...}

        Parameters:
          ownerId   - owner Id string
          winId     - Gui window unique id string. None if no id.

        Return Value:
          **options - list of previous configuration options (if any)
    """
    if not winId:
      return {}

    ini       = self.GCGetIni()
    section   = FusionIniDD.IniDDSectWin
    option    = 'CWin_' + ownerId
    winOwner  = ini.IniGetOrDft(section, option, None)

    if type(winOwner) != dict:
      return {}
    elif not winOwner:
      return {}
    elif not winOwner.has_key(winId):
      return {}
    else:
      return winOwner[winId]

  #--
  def GuiChildWinSetIniOptions(self, ownerId, winId, **saveOpts):
    """ Sets any new options for the child Gui window.

        Ini Section Format:
          [Fusion/windows]
          CWin_<ownerId> = {<winId>:{geometry:geo, ...} ...}

        Parameters:
          ownerId   - owner Id string
          winId     - Gui window unique id string. None if no id.
          saveOpts  - window keyword configuration options to save 

        Return Value:
          None.
    """
    if not winId:
      return
    elif len(saveOpts) == 0:
      return

    ini       = self.GCGetIni()
    section   = FusionIniDD.IniDDSectWin
    option    = 'CWin_' + ownerId
    winOwner  = ini.IniGetOrDft(section, option, {})

    if type(winOwner) != dict:
      winOwner = {}

    winOwner[winId] = saveOpts
    ini.IniSetModifiedItems(section, [(option, winOwner)])

  #--
  def GuiChildWinInit(self):
    """ Initialize attached child window session database. Session data
        are organized first by owner, then by window id. Since multiple
        windows of the same type (i.e. window id) may be running, unique
        window instances are further identified by a session id.

        Session Data BNF:
          mGuiChildWinDB:
            {}
            ownerIds

          ownerIds:
            ownId
            ownIds, ownId
          
          ownId:
            <ownerId>: {winIds}

          winIds:
            winId
            windId, winId

          winId:
            <winId>: {sessIds}

          sessIds:
            sessId
            sessIds, sessId

          sessId:
            <sessId>: {'win':<win>, 'update':<func>}
            
    """
    self.mGuiChildWinDB = {}

  #--
  def GuiChildWinRegister(self, ownerId, winId, win):
    """ Register child GUI window.

        Parameters:
          ownerId   - owner Id string
          winId     - Gui window unique id string. None if no id.
          win       - Gui window object

        Return Value:
          None.
    """
    if not win:
      return

    # session unique
    sessId = win.winfo_id()

    self.GCLock()
    if not self.mGuiChildWinDB.has_key(ownerId):
      self.mGuiChildWinDB[ownerId] = {}
    if not self.mGuiChildWinDB[ownerId].has_key(winId):
      self.mGuiChildWinDB[ownerId][winId] = {}
    self.mGuiChildWinDB[ownerId][winId][sessId] = {}
    self.mGuiChildWinDB[ownerId][winId][sessId]['win'] = win
    try:
      if callable(win.WinQueueRequest):
        self.mGuiChildWinDB[ownerId][winId][sessId]['update'] = \
                                                          win.WinQueueRequest
    except AttributeError:
      pass
    self.GCUnlock()
    if __debug__:
      self.mDbg.d3print('Child window %s.%s.%s registered' % \
          (ownerId, winId, sessId))

  #--
  def GuiChildWinUnregister(self, ownerId, winId, win, **saveOpts):
    """ Unregister child GUI window.

        Parameters:
          ownerId   - owner Id string
          winId     - Gui window unique id string. None if no id.
          win       - Gui window object
          saveOpts  - window keyword configuration options to save 

        Return Value:
          None
    """
    if not win:
      return

    # session unique
    sessId = win.winfo_id()

    self.GuiChildWinSetIniOptions(ownerId, winId, **saveOpts)

    self.GCLock()
    try:
      del self.mGuiChildWinDB[ownerId][winId][sessId]
    except KeyError:  # not present - ignore
      pass
    self.GCUnlock()
    if __debug__:
      self.mDbg.d3print('Child window %s.%s.%s unregistered' % \
          (ownerId, winId, sessId))

  #--
  def GuiChildWinStart(self, ownerId, winId, callobj, *args, **kwargs):
    """ Start a child window. Any ini saved configuration is 
        retrieved and added to the keyword arguments. The started 
        window is automatically registered with the client.

        Parameters:
          ownerId   - owner Id string
          winId     - Gui window unique id string. None if no id.
          callobj   - callable object to start the window. Must return
                      Gui window object.
          args      - arguments passed to callobj()
          kwargs    - keyword arguments passed to callobj().

        Return Value:
          Gui window object
    """
    options = self.GuiChildGetOptions(ownerId, winId)
    if not options:   # then error
      return None
    for k,v in options.iteritems():
      kwargs[k] = v
    win = callobj(self.mGuiRoot, *args, **kwargs)
    self.GuiChildWinRegister(ownerId, winId, win)
    return win

  #--
  def GuiChildWinUpdate(self, ownerId, winId, *args, **kwargs):
    """ Call child window update function for all active child window
        sessions of the owner with the given window id.

        Parameters:
          ownerId   - owner Id string
          winId     - gui window unique id.
          *args     - arguments to update callback
          *kwargs   - keyword arguments to update callback

        Return Value:
          None
    """
    sessIds = self.mGuiChildWinDB.get(ownerId, {}).get(winId)
    if not sessIds:
      return
    for session in sessIds.itervalues():
      updatefunc = session.get('update')
      if updatefunc and session['win'].isAlive:
        try:
          updatefunc(*args, **kwargs)
        except tk.TclError, msg:
          if __debug__: self.mDbg.d3print('%s: %s' % (winId, msg))
          pass

  #--
  def GuiChildWinDestroyOwners(self, ownerId):
    """ Destroy all of the child windows of the owner.

        Parameters:
          ownerId   - owner Id string

        Return Value:
          None.
    """
    # get a copy of the owner's windows
    self.GCLock()
    winList = []
    winIds = self.mGuiChildWinDB.get(ownerId,{})
    for sessIds in winIds.itervalues():
      for session in sessIds.itervalues():
        winList.append(session['win'])
    self.GCUnlock()
    
    # destroy the owner's windows which should typically make a callback
    # to GuiChildWinUnegister()
    for win in winList:
      try:
        win.destroy()
      except TclError:
        pass

    # now delete all of the owners session data
    self.GCLock()
    try:
      del self.mGuiChildWinDB[ownerId]
    except KeyError:  # not present - ignore
      pass
    self.GCUnlock()

  #--
  def GuiChildWinDestroyAll(self):
    """ Destroy all child windows. """
    self.GCLock()
    ownerList = self.mGuiChildWinDB.keys()
    self.GCUnlock()
    for ownerId in ownerList:
      self.GuiChildWinDestroyOwners(ownerId)

  #--
  def GuiChildWinFusionUnregister(self, winId, win, **saveOpts):
    """ Wrapper to unregister Reactor owned registered child window.

        Parameters:
          winId     - Gui window unique id string. None if no id.
          win       - Gui window object
          saveOpts  - window keyword configuration options to save 

        Return Value:
          None
    """
    self.GuiChildWinUnregister(FusionAppName, winId, win, **saveOpts)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Fusion Gui Callbacks
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def GuiCbResize(self, event):
    """ Resize callback event. """
    geo = gut.geometry(self.mGuiRoot)

    # no change
    if geo == self.mWinGeo:
      return

    # different size
    elif geo[gut.W] != self.mWinGeo[gut.W] or geo[gut.H] != self.mWinGeo[gut.H]:
      # resize history bar width and height
      self.mHistoryBar.configure(units='pixels', 
          width=geo[gut.W]-self.mSbWOffset, height=geo[gut.H]-self.mSbHOffset)

      # resize prompt bar width
      self.mPromptBar.configure(units='pixels', 
          width=geo[0]-self.mPbWOffset, height=None)
      textWidget = self.mPromptBar.GetTextWidget()
      width = int(textWidget['width'])  # in characters

      # resize info entries
      self.mEntryBrainInfo['width'] = width
      self.mEntryRobotInfo['width'] = width

    self.mWinGeo = geo

    ini     = self.GCGetIni()
    section = FusionIniDD.IniDDSectWin
    option  = 'Win_Fusion'
    win  = ini.IniGetOrDft(section, option, {})
    if type(win) != dict:
      win = {}
    win['geometry'] = self.mGuiRoot.wm_geometry()
    ini.IniSetModifiedItems(section, [(option, win)])

  #--
  def GuiCbQuit(self):
    """ Quit callback. """
    if __debug__: self.mDbg.d1print('GuiCbQuit')
    self.mCurAction = EAction.Quit
    self.UnloadServers()
    self.mMirror.AsyncRequest('fusion_quit')

  #--
  def _GuiDestroy(self):
    """ Really destroy the Reactor GUI. """
    self.GuiChildWinDestroyAll()
    if self.GuiQueryUserAndSave() == 'ok':
      self.mMirror.AsyncRequestMirrorDie()
      self.mGuiRoot.destroy()
    else:
      self.mCurAction = EAction.None

  #--
  def GuiCbFileOpenIni(self):
    """ File|Open Ini callback. """
    if __debug__: self.mDbg.d1print('File|Open Ini')
    if self.GuiQueryUserAndSave() == 'cancel':
      return
    dlg = GuiDlgOpen.GuiDlgOpen(self.mGuiRoot, None,
                  title='Open Configuration File',
                  filetypes=[('Configuration files', '*.ini', 'TEXT'),
                             ('All files', '*')],
                  defaultextension='.ini')
    if dlg.result:
      self.mIniSessSaveFileName = dlg.result
      ini = self.GCGetIni()
      ini.IniRemoveAll()
      ini.IniOpen(self.mIniSessSaveFileName)
      ini.ClearModifiedFlag()
      self.ShowMsg("Configuration read from %s" % self.mIniSessSaveFileName) 

    # go back to parent gui
    self.GCGuiRaiseParent()

  #--
  def GuiCbFileSaveIni(self):
    """ File|Save Ini callback. """
    if __debug__: self.mDbg.d1print('File|Save Ini')
    self.IniSave(self.mIniSessSaveFileName)

  #--
  def GuiCbFileSaveIniAs(self):
    """ File|Save Ini As callback. """
    if __debug__: self.mDbg.d1print('File|Save Ini As')
    dlg = GuiDlgSaveAs.GuiDlgSaveAs(self.mGuiRoot, None,
                  title='Save Configuration As',
                  filetypes=[('Configuration files', '*.ini', 'TEXT'),
                             ('All files', '*')],
                  defaultextension='.ini')
    if dlg.result:
      if dlg.result['status'] == 'notsaved':
        self.mIniSessSaveFileName = dlg.result['filename']
        self.IniSave(self.mIniSessSaveFileName)
      elif dlg.result['status'] == 'error':
        self.ShowError(dlg.result['errmsg'])

    # go back to parent gui
    self.GCGuiRaiseParent()

  #--
  def GuiCbEditCopy(self):
    """ Edit|Copy callback. """
    if __debug__: self.mDbg.d1print('Edit|Copy')
    self.mHistoryBar.EditCopy()

  #--
  def GuiCbEditSelectAll(self):
    """ Edit|Select All callback. """
    if __debug__: self.mDbg.d1print('Edit|Select All')
    self.mHistoryBar.EditSelectAll()

  #--
  def GuiCbEditPreferences(self):
    """ Edit|Preferences callback. """
    if __debug__: self.mDbg.d1print('Edit|Preferences')
    settingNames = GuiDlgFusionPref.GetSettingNames()
    ini = self.GCGetIni()
    section = FusionIniDD.IniDDSectMain
    iniSettings = ini.IniGetSubItems(section, settingNames)
    lastSettings = utils.tuples2dict(iniSettings)
    dlg = GuiDlgFusionPref.GuiDlgFusionPref(self.mGuiRoot,
                                            lastSettings=lastSettings)
    if dlg.result:
      iniSettings = utils.dict2tuples(dlg.result)
      if ini.IniSetModifiedItems(section, iniSettings) > 0:
        self.IniInitPref()

    # go back to parent gui
    self.GCGuiRaiseParent()

  #--
  def GuiCbRobotSelect(self):
    """ Robot|Select callback. """
    if __debug__: self.mDbg.d1print('Robot|Select')
    ini = self.GCGetIni()
    section = FusionIniDD.IniDDSectMain
    paths = ini.IniGet(section, 'RobotPluginPath')
    pathList = utils.canonicalsplitpaths(paths)
    dlg = GuiDlgPlugin.GuiDlgPlugin(self.mGuiRoot,
        'vRobots', dirList=pathList,
        initialdir=self.mIniSessRobotPluginInitialDir,
        title='vRobot Plugins')
    if dlg.result:
      self.mIniSessRobotPluginInitialDir = dlg.result['initialdir']
      del dlg.result['initialdir']

      # make sure we got a vRobot
      if not dlg.result['pluginEPoint']:
        self.ShowError('vRobot plugin %s: '
                       'no derived vRobot class entry point found.' % \
                repr(dlg.result['pluginPath']))
        utils.delmodule(dlg.result['pluginMod'])
        return

      # verify the vRobot interface
      try:
        isvrobot = issubclass(dlg.result['pluginEPoint'], vRobot.vRobot)
      except TypeError, err:
        self.ShowError('vRobot plugin entry point %s: not a class.' % \
                repr(dlg.result['pluginEPointName']))
        del dlg.result['pluginEPoint']
        utils.delmodule(dlg.result['pluginMod'])
        return
      if isvrobot:
        self.PluginRobot(vRobotPlugin=dlg.result)
        self.ShowMsg('vRobot selected')
      else:
        self.ShowError("vRobot plugin entry point %s: "
                       'not a derived vRobot class.' % \
                repr(dlg.result['pluginEPointName']))
        del dlg.result['pluginEPoint']
        utils.delmodule(dlg.result['pluginMod'])

  #--
  def GuiCbRobotDeselect(self):
    """ Robot|Deselect callback. """
    if __debug__: self.mDbg.d1print('Robot|Deselect')
    self.UnplugRobot()
    self.ShowMsg('vRobot deselected')

  #--
  def GuiCbBrainSelect(self):
    """ Brain|Select callback. """
    if __debug__: self.mDbg.d1print('Brain|Select')
    ini = self.GCGetIni()
    section = FusionIniDD.IniDDSectMain
    paths = ini.IniGet(section, 'BrainPluginPath')
    pathList = utils.canonicalsplitpaths(paths)
    dlg = GuiDlgPlugin.GuiDlgPlugin(self.mGuiRoot,
        'vBrains', dirList=pathList,
        initialdir=self.mIniSessBrainPluginInitialDir,
        title='vBrain Plugins')
    if dlg.result:
      self.mIniSessBrainPluginInitialDir = dlg.result['initialdir']
      del dlg.result['initialdir']

      # make sure we got a vBrain
      if not dlg.result['pluginEPoint']:
        self.ShowError('vBrain plugin %s: '
                       'no derived vBrain class entry point found.' % \
                repr(dlg.result['pluginPath']))
        utils.delmodule(dlg.result['pluginMod'])
        return

      # verify the vBrain interface
      try:
        isvbrain = issubclass(dlg.result['pluginEPoint'], vBrain.vBrain)
      except TypeError, err:
        self.ShowError('vBrain plugin entry point %s: not a class.' % \
                repr(dlg.result['pluginEPointName']))
        utils.delmodule(dlg.result['pluginMod'])
        del dlg.result['pluginEPoint']
        return
      if isvbrain:
        self.PluginBrain(vBrainPlugin=dlg.result)
        self.ShowMsg('vBrain selected')
      else:
        self.ShowError("vBrain plugin entry point %s: "
                       'not a derived vBrain class.' % \
                repr(dlg.result['pluginEPointName']))
        utils.delmodule(dlg.result['pluginMod'])
        del dlg.result['pluginEPoint']

  #--
  def GuiCbBrainDeselect(self):
    """ Brain|Deselect callback. """
    if __debug__: self.mDbg.d1print('Brain|Deselect')
    self.UnplugBrain()
    self.ShowMsg('vBrain deselected')

  #--
  def GuiCbToolsDebug(self):
    """ Tools|Debug callback. """
    if __debug__: self.mDbg.d1print('Tools|Debug')
    settingNames = GuiDlgDebug.GetSettingNames()
    ini = self.GCGetIni()
    section = FusionIniDD.IniDDSectMain
    iniSettings = ini.IniGetSubItems(section, settingNames)
    lastSettings = utils.tuples2dict(iniSettings)
    dlg = GuiDlgDebug.GuiDlgDebug(self.mGuiRoot, lastSettings=lastSettings)
    if dlg.result:
      self.DebugSet(dlg.result['DebugLevelFusion'],
                    dlg.result['DebugLevelRobot'], 
                    dlg.result['DebugLevelBrain'],
                    dlg.result['DebugFileName'])
      iniSettings = utils.dict2tuples(dlg.result)
      ini.IniSetModifiedItems(section, iniSettings)

    # go back to parent gui
    self.GCGuiRaiseParent()

  #--
  def GuiCbToolsEnv(self):
    """ Tools|Env callback. """
    if __debug__: self.mDbg.d1print("Tools|Env")
    env = '    %s Environment\n\n' % (Values.FusionPkgName)
    nvList, evList = Values._listValues()
    nvList.sort()
    env += '    %s Global Variables\n' % (Values.FusionPkgName)
    for v in nvList:
      env += v + '\n'
    env += '\n    Environment Variables\n'
    for v in evList:
      env += v + '\n'
    msgbox.InfoBox(env, justify=tk.LEFT)

  #--
  def GuiCbToolsIniListing(self):
    """ Tools|Ini Listing callback. """
    if __debug__: self.mDbg.d1print("Tools|'Ini' Listing")
    iniDD = self.MakeIniDD()
    ini = self.GCGetIni()

    self.GuiChildWinStart(FusionAppName, 'iniListing',
                GuiWinIniListing.GuiWinIniListing,
                ini, self.IsVersion(), iniDD=iniDD)

  #--
  def GuiCbToolsGluonTree(self):
    """ 'Tree' menu callback. """
    if __debug__: self.mDbg.d1print('Tools|Gluon Tree')
    msgbox.WarningBox('Not implemented yet.')
    
  #--
  def GuiCbToolsPerfMeter(self):
    """ Tools|PerfMeter. """
    if __debug__: self.mDbg.d1print("Tools|PerfMeter")
    msgbox.WarningBox('Not implemented yet.')

  #--
  #def GuiCbToolsRDK(self):
  #  """ Tools|RDK callback. Comment out when not using to text x"""
  #  ini = self.GCGetIni()
  #  plugin = ini.IniGet(FusionAppName, 'RobotPlugin')
  #  if __debug__: self.mDbg.d1print('plugin = %s' % repr(plugin))
  #  robot = plugin[1](debuglevel=2)

  #--
  def GuiCbHelpAbout(self):
    """ Help|About callback. """
    if __debug__: self.mDbg.d1print('Help|About')
    imageFile = gut.GetFusionImageFileName(gt.ImageRNRLogo)
    GuiDlgAbout.GuiDlgAbout(self.mGuiRoot,
                    name=self.HasName(),
                    version='Version ' + self.IsVersion(),
                    descTitle='Fusion Robotics Demo Application',
                    desc=\
"""Fusion is a RoadNarrows developed demonstration application. It is free
to use, modify, and distribute. See About|EULA for more details.

Fusion is a Gluon Client that fuses a vRobot Gluon Server and/or a vBrain 
Gluon Server into one controlling structure. Gluon is the simple control
and information exchange interface between the client and server.

Both the vBrains and vRobots are plugable python modules. Any derived
application conforming to the vBrain and vRobot interface will work within
Fusion construct. 

Supported client-server models are:
 Fusion(client) <--Gluon--> vRobot(server)
 Fusion(client) <--Gluon--> vBrain(server)
 Fusion(client) <--Gluon(2)--> [vBrain(server) <--peer--> vRobot(server)]\
""",
                    copyright='RoadNarrows LLC\n(C) 2006',
                    logoImage=imageFile)

  #--
  def GuiCbHelpAboutRoadNarrows(self):
    """ Help|About RoadNarrows callback. """
    if __debug__: self.mDbg.d1print('Help|About RoadNarrows')
    imageFile = gut.GetFusionImageFileName(gt.ImageRNRLogo)
    GuiDlgAbout.GuiDlgAbout(self.mGuiRoot,
                    name='RoadNarrows LLC',
                    desc="""\
RoadNarrows LLC is a small robotics company based in Colorado, U.S.A.

RoadNarrows specializes in the sells and support of robots for the
Research and Education markets. RoadNarrows is actively in the process
of designing new hardware and software for the next generation of
Research and Education robots.

RoadNarrows LLC
1151 Eagle Dr. #140
Loveland, Colorado
U.S.A. 80537

url: www.roadnarrowsrobotics.com
email: oneway@roadnarrowsrobotics.com
ph: +1 970.593.0370
fax: +1 970.461.9969\
""",
                    copyright='RoadNarrows LLC\n(TM) 2006',
                    logoImage=imageFile)

  #--
  def GuiCbHelpEULA(self):
    """ Help|EULA callback. """
    if __debug__: self.mDbg.d1print('Help|EULA')
    imageFile = gut.GetFusionImageFileName(gt.ImageInfo)
    GuiDlgAbout.GuiDlgAbout(self.mGuiRoot,
                    name='Fusion End-User License Agreement',
                    descTitle='The EULA',
                    desc="""\
Permission is hereby granted, without written agreement and without license or
royalty fees, to use, copy, modify, and distribute this software and its
documentation for any purpose, provided that (1) The above copyright notice and
the following two paragraphs appear in all copies of the source code and (2)
redistributions including binaries reproduces these notices in the supporting
documentation.   Substantial modifications to this software may be copyrighted
by their authors and need not follow the licensing terms described here,
provided that the new terms are clearly indicated in all files where they apply.

IN NO EVENT SHALL THE AUTHOR, ROADNARROWS LLC, OR ANY MEMBERS OR
EMPLOYEES OF ROADNARROW LLC OR DISTRIBUTORS OF THIS SOFTWARE BE
LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL,
OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE OF THIS SOFTWARE
AND ITS DOCUMENTATION, EVEN IF THE AUTHORS OR ANY OF THE ABOVE 
PARTIES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

THE AUTHOR AND ROADNARROWS LLC SPECIFICALLY DISCLAIM ANY
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. THE
SOFTWARE PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND THE
AUTHORS AND DISTRIBUTORS HAVE NO OBLIGATION TO PROVIDE
MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.\
""",
                    copyright='All Rights Reserved by RoadNarrows LLC\n2006.',
                    logoImage=imageFile)

  #--
  def GuiCbNull(self):
    """ Null callback. """
    if __debug__: self.mDbg.d1print('GuiCbNull')
    pass


#-------------------------------------------------------------------------------
# CLASS: Unit Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  #import Fusion.Khepera.Robots.vKhepera as vk

  def main():
    """ Fusion Unit Test Main """
    #robot = vk.vKhepera(debuglevel=1)
    #reactor = Reactor(vRobot=robot, debuglevel=3)
    reactor = Reactor(debuglevel=3)
    reactor.mGuiRoot.mainloop()

  # run unit test
  main()
