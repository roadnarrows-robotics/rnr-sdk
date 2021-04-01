################################################################################
#
# vKHR1.py
#

""" Virtual KHR-1 Robot.

Virtual KHR-1 Robot shadows the physical KHR-1 robot including all
supported I2C modules.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2007.01.06

Copyright (C) 2007.  RoadNarrows LLC.
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
import tkinter as tk

import Fusion.Utils.Tools as utils

import Fusion.Core.Gluon as Gluon
import Fusion.Core.vRobot as vRobot
import Fusion.Core.vRobotThreaded as vRobotThreaded

import Fusion.Gui.GuiTypes as gt
import Fusion.Gui.GuiUtils as gut
import Fusion.Gui.GuiDlgSerConn as GuiDlgSerConn
import Fusion.Gui.GuiWinShell as GuiWinShell
import Fusion.Gui.GuiDlgAbout as GuiDlgAbout
import Fusion.Gui.GuiDlgMsgBox as msgbox

import Fusion.KHR1.Cmd.KHR1CmdBase as KHR1CmdBase

import Fusion.KHR1.Shells.KHR1BaseShell as KHR1BaseShell

import Fusion.KHR1.Robots.KHR1Values as kvals
import Fusion.KHR1.Robots.KHR1IniDD as KHR1IniDD

import Fusion.KHR1.Gui.GuiDlgKHR1Opt as GuiDlgKHR1Opt
import Fusion.KHR1.Gui.GuiDlgKHR1ServoChan as GuiDlgKHR1ServoChan


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

# sensor converted value, raw value indices
CVTVAL  = 0
RAWVAL  = 1

#-------------------------------------------------------------------------------
# CLASS: vKHR1
#-------------------------------------------------------------------------------
class vKHR1(vRobotThreaded.vRobotThreaded):
  """ Virtual KHR-1 Robot Class. """

  #--
  def __init__(self, client=None, debuglevel=0, debugfout=None):
    """ Initialize vKHR1 instance.

        Parameters:
          client      - Gluon client
          debuglevel  - none to all [0, 5] (default: none)
          debugfout   - opened debug output file (default: stdout)
    """
    # base class
    vRobotThreaded.vRobotThreaded.__init__(self, 
        serverId=self.HasName(), client=client,
        debuglevel=debuglevel, debugfout=debugfout)

  #--
  def vRobotInit(self):
    """ One-time vRobot initialization during object instantiation. """
    # KHR-1 serial commands
    self.mCmd = KHR1CmdBase.KHR1CmdBase(dbgobj=self.mDbg)
    self.mCommIsUp = self.mCmd.IsOpen()

    # KHR-1 gui initializations
    self.GuiInit()

    # common threaded robot initialization (includes ini initialization)
    vRobotThreaded.vRobotThreaded.vRobotInit(self)

    # add menu bar
    self.GSSetServerMenuBarList(self.mMenuBarList)

    # add tool bar
    self.GSSetServerToolBarList(self.mToolBarList)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vKHR1 Attribute Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def IsType(self):
    """ Returns the KHR-1 robot type.

        Return Value:
          The KHR-1 robot MIME type string.
    """
    return kvals.KHR1MimeType

  #--
  def HasName(self):
    """ Returns the short vKHR1 name(s) string.

        Return Value:
          The robot name(s) string which may include either or both
          the vRobot and physical robot names.
    """
    return 'vKHR1'

  #--
  def IsVersion(self):
    """ Returns the vKHR1 version(s) string.

        Return Value:
          The vRobot version(s) string.
    """
    return '0.10'

  #--
  def IsRobotVersion(self):
    """ Returns the KHR-1 robot version(s) string.

        Return Value:
          Returns version string.
    """
    ver = None
    return self.mCmd.AttrGetServoVersion()

  #--
  def HasDesc(self):
    """ Returns a short description of this vRobot.

        Return Value:
          Multi-line description string.
    """
    sDesc = """\
KHR-1 humanoid robot with 24 servo channels.
Manufactured by Kondo of Japan."""
    return sDesc

  #--
  def HasSensorTypes(self):
    """ Returns the dictionary of the sensors and their corresponding
        properties that are available and supported by this vRobot.
        The dictionary is keyed by 'sensorId', which is vRobot unique.
    
        Return Value:
          Dictionary of sensor id's and properties in the format:
            {sensorId:{'mimetype':<type>...}, ... }
    """
    sensorDict = {}

    # derived time 'sensor'
    sensorDict['time_stamp'] = {
      'mimetype': kvals.KHR1SensorMimeTypeTime,
      'units': 's'
    }
    sensorDict['dt'] = {
      'mimetype': kvals.KHR1SensorMimeTypeTime,
      'units': 's'
    }

    servos = self.mCmd.AttrGetActiveServos()
    for mnem,channel in servos.items():
      id = 'servo_' + mnem
      sensorDict[id] = {
        'mimetype': kvals.KHR1SensorMimeTypeServo,
        'mnem': mnem,
        'channel': channel,
        'angrange': [0, 180],
        'units': 'degrees'
      }
  
    return sensorDict

  #--
  def HasEffectorTypes(self):
    """ Returns the dictionary of the effectors and their corresponding
        properties that are available and supported by this vRobot.
        The dictionary is keyed by 'effectorId', which is vRobot unique.
    
        Return Value:
          Dictionary of effector id's and properties in the format:
            {effectorId:{'mimetype':<type>...}, ... }
    """
    effectorDict = {}

    servos = self.mCmd.AttrGetActiveServos()
    for mnem,channel in servos.items():
      id = 'servo_' + mnem
      effectorDict[id] = {
        'mimetype': kvals.KHR1EffectorMimeTypeServo,
        'mnem': mnem,
        'channel': channel,
        'angrange': [0, 180],
        'units': 'degrees'
      }
  
    return effectorDict
  
  #--
  def HasPhysicalProperties(self):
    """ Returns the dictionary of KHR-1 physical properties.
    
        Return Value:
          Dictionary of physical properties in the format:
            {property:<val>, ... }
    """
    return {
      'height': {'val': 340, 'units': 'mm'},
      'baseweight': {'val': 1200, 'units': 'g'},
      'dog': 17
    }

  #--
  def HasBellumGoalControls(self):
    """ Returns the maximal list of goal controls (kwGoals) supported
        by vKHR1. 
    
        Return Value:
          List of goal controls.
    """
    return []


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vKHR1 Ini (Re)Initialization Members Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def IniInit(self):
    """ Initialize from parsed 'ini' configuration. """

    self.mIniDD = KHR1IniDD.GetIniDD()
    ini         = self.GSGetIni()

    # load all non-existing ini entries with defaults
    for section,sdata in self.mIniDD.items():
      optdict = sdata[1]
      for option,odata in optdict.items():
        if ini.IniGet(section, option) == ini.NullObj:
          ini.IniSet(section, option, odata[0])

    # load vKHR1 run-time options
    self.IniInitOpt()

    # load vKHR1 connection settings
    self.IniInitConn()

    # load VKHR1 active channel list
    self.IniInitChan()

    # load VKHR1 servo trim list
    self.IniInitTrim()

    # load sense-react operational settings
    self.IniInitSenseReact()

  #--
  def IniInitOpt(self):
    """ Initialized vKHR1 options from parsed configuration. """
    iniDD   = self.mIniDD
    ini     = self.GSGetIni()
    section = KHR1IniDD.IniDDSectOpts
    optDict = iniDD[section][1]

    self.mOpt = {}

    for option in optDict:
      self.mOpt[option] = ini.IniGet(section, option)

    self.mCmd.AttrSetBoardIdList(ini.IniGet(section, 'board_ids'))
    self.mCmd.AttrSetServoVersion(ini.IniGet(section, 'servo_version'))
    self.SetExecSizes(self.mOpt['ExecCycle'], self.mOpt['ExecStepSize'])

  #--
  def IniInitConn(self):
    """ Initialized vKHR1 connection settings from parsed configuation. """
    iniDD   = self.mIniDD
    ini     = self.GSGetIni()
    section = KHR1IniDD.IniDDSectConn
    optDict = iniDD[section][1]

    self.mConn = {}

    for option in optDict:
      self.mConn[option] = ini.IniGet(section, option)

  #--
  def IniInitChan(self):
    """ Initialized vKHR1 active servo channel configuration.
    """
    iniDD   = self.mIniDD
    ini     = self.GSGetIni()
    section = KHR1IniDD.IniDDSectChan

    self.mCmd.AttrSetActiveServos(**ini.IniGet(section, 'active_channels'))

  #--
  def IniInitTrim(self):
    """ Initialized vKHR1 servo trim configuration.
    """
    iniDD   = self.mIniDD
    ini     = self.GSGetIni()
    section = KHR1IniDD.IniDDSectTrim

    self.mCalTrim = ini.IniGet(section, 'trim')

  #--
  def IniInitSenseReact(self):
    """ Initialized sense-react operational settings from parsed
        configuration.
    """
    pass

 
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vKHR1 Sensor and Shadow Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def ShadowInit(self):
    """ Initialize the shadowed data.

        The Shadow captures the sensory states of the (simulated) robot
        plus any derived states into a dictionary of data. This data can
        be readily accessed by the vRobot's reactive subsystem and by the
        upper layer vBrain functions.

        Return Value:
          None.
    """
    self.GSLock()

    # the shadow
    self.mShadow = {}

    # time 'sensor' group
    groupId = kvals.KHR1SensorMimeTypeTime
    self.mShadow[groupId]          = {}
    self.mShadow[groupId]['dt_i']  = 0.0
    self.mShadow[groupId]['t_0']   = 0.0
    self.mShadow[groupId]['t_i']   = 0.0

    groupId = kvals.KHR1SensorMimeTypeServo
    self.mShadow[groupId]          = {}
    servos = self.mCmd.AttrGetActiveServos()
    for mnem in servos.keys():
      self.mShadow[groupId]['servo_'+mnem] = 0

    self.GSUnlock()

  #--
  def ShadowUpdate(self, groupId='all'):
    """ Automatic update of the shadow data group(s) from the current
        states of the robot. The vRobot automatically updates the shadow
        in it's execution cycle.  Ancillary threads (e.g. windows) and
        vBrain functions may also request updates. 

        Note: all shadow groups are updated to keep synchronicity.

        Parameters:
          groupId  - shadow group Id. (ignored)
        
        Return Value:
          None.
    """
    self.GSLock()

    # time 'sensor' group (always on)
    group = self.mShadow[kvals.KHR1SensorMimeTypeTime]
    t_now = time.time()
    group['dt_i'] = t_now - group['t_i']
    group['t_i']  = t_now
    if __debug__: self.mDbg.d4print(' %s: %s:' % \
                        (kvals.KHR1SensorMimeTypeTime, repr(group)))

    # servo group
    self.mShadow[kvals.KHR1SensorMimeTypeServo] = self.SenseServoPos()

    self.GSUnlock()

  #--
  def SenseServoPos(self):
    """ Read the active servo positions. The read is only attempted if
        communication to the robot is up.

        The data returned is a dictionary of read servo positions.

        Ancillary threads (e.g. windows) and vBrain functions may also 
        request sensor updates. 

        Return Value:
          Dictionary of read values on success. Dictionary of default
          values on failure.
    """
    sensorData = {}
    readData = None
    if self.IsCommUp(): # communication must be up and the module attached
      readData = self.mCmd.CmdGetCurPos()
    if readData is None:  # data dropout/no comm - fill with last position
      sensorData = self.mShadow[kvals.KHR1SensorMimeTypeServo]
    else:
      servos = self.mCmd.AttrGetActiveServos()
      for mnem,channel in servos.items():
        sensorData['servo_'+mnem] = readData[channel]
    if __debug__: self.mDbg.d4print(' %s: %s:' % \
                      (kvals.KHR1SensorMimeTypeServo, repr(sensorData)))
    return sensorData


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vRobot Effector Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def EffectPos(self, speed, **servos):
    """ Set KHR-1 servo positions.

        Parameters:
          speed       - Servos speed [0,7], 7 == slowest, 0 == fastest.
          **servos    - Keyword arguments <mnem>=<pos> specifying
                        new servo positions.

        Return Value:
          None
    """
    if self.IsCommUp():
      chanList = self.mCmd.CmdSetIncCurPos(speed, **servos)
      if chanList:
        sensorData = {}
        servos = self.mCmd.AttrGetActiveServos()
        for mnem,channel in servos.items():
          sensorData['servo_'+mnem] = readData[channel]
        self.mShadow[kvals.KHR1SensorMimeTypeServo] = sensorData


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vRobot (Cere)Bellum Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def BellumSetGoals(self, **kwGoals):
    """ Set new robot low-level goals. The old goal set is replaced by
        this new goal set.

        Parameters:
          kwGoals   - argument list of keyword=value goals.
           Keywords:
        
        Return Value:
          Returns new current goal set. 
    """
    return self.mBellumGoalSet

  #--
  def BellumNullGoals(self):
    """ Set the vKHR1's low-level goals to the null set
        (i.e. resting state).

        Return Value:
          Returns new current goal set.
    """
    return self.mBellumGoalSet


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vKHR1 Do's
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def DoEStop(self):
    """ Do an emergency stop of the physical robot.

        Callback on vRobotThreaded emergency stopping event.

        Return Value:
          None
    """
    maxTries  = 5
    tries     = 0
    while tries < maxTries:
      if self.mCmd.CmdSetSwBits(sleep=1):
        if __debug__: self.mDbg.d2print(
            'Emergency Stop of KHR-1 robot succeeded')
        return
      tries += 1
    if __debug__:
      self.mDbg.d2print('Emergency Stop of KHR-1 robot Failed: %s' % \
          self.mCmd.GetErrStr())

  #--
  def DoLoadInit(self):
    """ Load vRobot Initializer.

        Callback on vRobotThreaded loading event.

        Return Value:
          None
    """
    vRobotThreaded.vRobotThreaded.DoLoadInit(self)

    # possibly auto connect
    if not self.IsCommUp() and self.mOpt['AutoConnect']:
      if self.mConn['port'] and self.mConn['baudrate']:
        self.pRobotConnect(self.mConn['port'], self.mConn['baudrate'])


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vKHR1 Sense-React Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def SenseReactInit(self):
    """ Initialize Sense-React Operations.

        Event: Starting event

        Execution Context: Calling thread
    """
    vRobotThreaded.vRobotThreaded.SenseReactInit(self)

    # start 'time' sensor
    group = self.mShadow['sensor/time']
    group['t_0']  = group['t_i'] = time.time()  # time 0
    group['dt_i'] = 0.0                         # delta time

  #--
  def SenseReact(self):
    """ Sense-React Behavior. Track to current set of goals.
        Goals are speed.

        Execution Context: Bot thread
    """
    # on hold
    if self.mOnHold:
      return

    if __debug__: self.mDbg.d4print('SenseReact')

    # shadow robot sensors
    self.ShadowUpdate()


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vKHR1 Physical Robot Members Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def pRobotInit(self):
    """ Place the physical KHR-1 into a known, initialized state.

        Return Value:
          None.
    """
    if not self.IsCommUp():
      self.GSReportErrorStatus("Cannot initialize the KHR-1: no connection")
      return
    self.mCmd.CmdSetSwBits(sleep=0, motion=0)
    self.mCmd.CmdGoToHomePos()

  #--
  def pRobotConnect(self, port, baudrate=115200, bytesize=8, parity='N',
                                stopbits=1):
    """ Connect to the KHR-1 robot.

        Parameters:
          port      - serial connection port
          baudrate  - serial connection baudrate (always 115200)
          bytesize  - byte size (always 8)
          parity    - parity (always none)
          stopbits  - stop bits (always 1) 

        Return Value:
          None
    """
    self.GSReportNormalStatus("Opening serial connection %s %d-%d-%s-%d..." % \
                     (port, 115200, 8, 'N', 1))
    try:
      self.mCmd.Open(port, baudrate=baudrate)
      self.SetCommStatus(True)
    except IOError as err:
      s = "%s" % err
      self.GSReportErrorStatus(s)
      self.SetCommStatus(False)
    if self.IsCommUp():
      self.mCmd.FlushInput()

  #--
  def pRobotDisconnect(self):
    """ Disconnnect communicaton channel with the KHR-1. The KHR-1
        is stopped prior to closing the channel.

        Return Value:
          None.
    """
    if self.IsCommUp():
      self.mCmd.CmdGoToHomePos()
      self.mCmd.Close()
      self.SetCommStatus(False)

  #--
  def pRobotHold(self):
    """ Hold position of physical robot.

        Return Value:
          None.
    """
    pass
    #if self.IsCommUp():
    #  self.mCmd.CmdGoToHomePos()


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vKHR1 Gui and Support
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def GuiInit(self):
    """ Initialize vKHR1 GUI. """
    # Menubar list
    self.mMenuBarList = Gluon.GluonMenuBarList()

    self.mMenuBarList.add('Robot|Robot Options...', 'command', 
        callback=self.GuiCbRobotOptions)
    self.mMenuBarList.add('Robot|Servo Channels...', 'command', 
        callback=self.GuiCbRobotOptServoChannels)

    self.mMenuBarList.add('Robot', 'separator')
    self.mMenuBarList.add('Robot|Connect...', 'command', 
        callback=self.GuiCbRobotConnect,
        disabledStates={self.mServerId:[
          Gluon.EServerState.Ready,
          Gluon.EServerState.Running,
          Gluon.EServerState.Paused,
          Gluon.EServerState.Stepping
        ]})
    self.mMenuBarList.add('Robot|Disconnect', 'command', 
        callback=self.GuiCbRobotDisconnect,
        disabledStates={self.mServerId:[
          Gluon.EServerState.NotLoaded,
          Gluon.EServerState.NotReady
        ]})

    self.mMenuBarList.add('Robot', 'separator')
    self.mMenuBarList.add('Robot|Trim...', 'command', 
        callback=self.GuiCbRobotTrim)

    self.mMenuBarList.add('Robot', 'separator')
    self.mMenuBarList.add('Robot|Robot Visualizer', 'command', 
        callback=self.GuiCbRobotViz)
    self.mMenuBarList.add('Robot|Shell', 'command', 
        callback=self.GuiCbRobotShell)

    self.mMenuBarList.add('Help|About Robot...', 'command', 
        callback=self.GuiCbHelpAbout)

    # Plugin Toolbar list
    self.mToolBarList = Gluon.GluonToolBarList()
    self.mToolBarList.add('connect',
        self.GuiCbRobotConnect, 
        tooltip='Establish a serial connection with the KHR-1',
        imagefile=gut.GetFusionImageFileName('SerConn.gif'),
        altStates={self.mServerId:[
          Gluon.EServerState.Ready,
          Gluon.EServerState.Running,
          Gluon.EServerState.Paused,
          Gluon.EServerState.Stepping
        ]},
        alttooltip='Disconnect serial connection to the KHR-1',
        altimagefile=gut.GetFusionImageFileName('SerDisc.gif'))

  #--
  def GuiDeinit(self):
    """ Deinitialize GUI objects. """
    pass

  #--
  def GuiCbRobotOptions(self):
    """ 'Options' menu callback. """
    if __debug__: self.mDbg.d1print('Robot|Options')
    
    lastSettings  = {}

    # Get parsed ini configuation (ini guraunteed to exist)
    ini           = self.GSGetIni()
    section       = KHR1IniDD.IniDDSectOpts
    settingNames  = GuiDlgKHR1Opt.GetSettingNames()
    iniSettings   = ini.IniGetSubItems(section, settingNames)
    lastSettings  = utils.tuples2dict(iniSettings)

    # get parent gui object for this dialog
    parent = self.GSGuiGetParent()

    # the dialog
    dlg = GuiDlgKHR1Opt.GuiDlgKHR1Opt(parent, lastSettings=lastSettings)

    # options have been okay'ed
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
  def GuiCbRobotOptServoChannels(self):
    """ 'Servo Channel Options' menu callback """
    if __debug__: self.mDbg.d1print('Robot|Servo Channels')

    # Get parsed ini configuation (ini guraunteed to exist)
    ini           = self.GSGetIni()
    section       = KHR1IniDD.IniDDSectChan
    iniSettings   = ini.IniGetItems(section)
    lastSettings  = utils.tuples2dict(iniSettings)

    # get parent gui object for this dialog
    parent = self.GSGuiGetParent()

    # the dialog
    dlg = GuiDlgKHR1ServoChan.GuiDlgKHR1ServoChan(parent,
            lastSettings=lastSettings,
            defaultSettings={
              'active_channels':KHR1CmdBase.KHR1FacDftActiveServos
            })

    # options have been okay'ed
    if dlg.result: 

      opts = dlg.result

      # update ini with current connection settings
      iniSettings = utils.dict2tuples(opts)
      ini.IniSetModifiedItems(section, iniSettings)

      # Re-init settings
      self.IniInitChan()

    # go back to parent gui
    self.GSGuiRaiseParent()

  #--
  def GuiCbRobotConnect(self):
    """ 'Robot|Connect' menu callback. """
    if __debug__: self.mDbg.d1print('Robot|Connect')
    
    # RDK!!! hack until ToolBar support SM
    if self.IsCommUp():
      self.GuiCbRobotDisconnect()
      return

    portHistory   = []
    lastSettings  = {}

    # Get parsed ini configuation (ini guraunteed to exist)
    ini     = self.GSGetIni()
    section = KHR1IniDD.IniDDSectConn
    optList = ini.IniGetReItems(section, 'port[0-9]+')
    for opt,val in optList:
      portHistory += [val]
    portHistory.sort()
    settingNames = GuiDlgSerConn.GetSettingNames()
    iniSettings = ini.IniGetSubItems(section, settingNames)
    lastSettings = utils.tuples2dict(iniSettings)

    # get KHR-1 serial port supported values 
    listBaudRates = self.mCmd.GetSupportedBaudRates()
    listByteSizes = self.mCmd.GetSupportedByteSizes()
    listParities  = self.mCmd.GetSupportedParities()
    listStopBits  = self.mCmd.GetSupportedStopBits()

    # get parent gui object for this dialog
    parent = self.GSGuiGetParent()

    # the dialog
    dlg = GuiDlgSerConn.GuiDlgSerConn(parent, self.mCmd.Open,
                                      portHistory=portHistory,
                                      lastSettings=lastSettings,
                                      validBaudRates=listBaudRates,
                                      validByteSizes=listByteSizes,
                                      validParities=listParities,
                                      validStopBits=listStopBits)

    # Serial connection has been successfully opened
    if dlg.result: 

      curOpened = dlg.result

      # update ini with current connection settings
      iniSettings = utils.dict2tuples(curOpened)
      ini.IniSetModifiedItems(section, iniSettings)

      # update port history
      isNewPort = True
      for val in portHistory:
        if val == curOpened['port']:
          isNewPort = False
          break
      if isNewPort:
        ini.IniRemoveReOptions(section, 'port[0-9]+')
        i = 1
        for val in portHistory:
          ini.IniSet(section, "port%d" % i, val)
          i += 1
        ini.IniSet(section, "port%d" % i, curOpened['port'])

      # Re-init settings
      self.IniInitConn()

      # Set new communication status
      self.SetCommStatus(True)

      # flush any residuals
      self.mCmd.FlushInput()

    # go back to parent gui
    self.GSGuiRaiseParent()

  #--
  def GuiCbRobotDisconnect(self):
    """ 'Disonnect' menu callback. """
    if __debug__: self.mDbg.d1print('Robot|Disconnect')
    self.SetCommStatus(False)
    self.mCmd.Close()

  #--
  def GuiCbRobotTrim(self):
    """ 'Trim Servos Calibration' menu callback """
    if __debug__: self.mDbg.d1print('Robot|Trim')
    msgbox.WarningBox('Not implemented yet.')
    if self.IsCommUp():
      self.GSReportNormalStatus("Trimming KHR-1")
      self.mCmd.CmdSetTrim(self.mCalTrim)

  #--
  def GuiCbRobotViz(self):
    """ 'Visualize Robot' menu callback. """
    if __debug__: self.mDbg.d1print('Robot|Robot Visualizer')
    msgbox.WarningBox('Not implemented yet.')

  #--
  def GuiCbRobotShell(self):
    """ 'Shell' menu callback. """
    if __debug__: self.mDbg.d1print('Robot|Shell')
    self.GSGuiWinStart('Shell',                     # window ID
                       GuiWinShell.GuiWinShell,     # start object
                       KHR1BaseShell.KHR1BaseShell, # arguments to start
                       title='vKHR1 Command Shell',
                       robotCmds=self.mCmd)

  #--
  def GuiCbHelpAbout(self):
    """ 'Shell' menu callback. """
    if __debug__: self.mDbg.d1print('Help|About Robot')

    # get parent gui object for this dialog
    parent = self.GSGuiGetParent()

    verstr = self.HasName() + ' v' + self.IsVersion() + '\n' + 'KHR-1 v' + \
              self.IsRobotVersion()
    imageFile = gut.GetFusionImageFileName(gt.ImageKHR1)

    GuiDlgAbout.GuiDlgAbout(parent,
                    name=self.HasName(),
                    version=verstr,
                    descTitle='RoadNarrows vKHR1 vRobot Demo',
                    desc="""\
vKHR1 is a virtual Robot (vRobot) that interfaces with the KHR-1
physical robot. vKHR1 provides access to the KHR-1 built in servos.
The KHR-1 has two RCB-1 servo controller boards, each with 12
channels, to controll up to 24 servos. By default, the KHR-1 has
17 servos, providing 17 Degress of Freedom.

vKHR1 provides the full set of KHR-1 commands, plus movement goal
tracking (future).""",
                    copyright='RoadNarrows LLC\n(C) 2006',
                    logoImage=imageFile)

    # go back to parent gui
    self.GSGuiRaiseParent()


#-------------------------------------------------------------------------------
# Unit Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  def tstCreate(level=2):
    """ Create vKHR1 unit test environment. """
    k = vKHR1(debuglevel=level)
    port = '/dev/ttyS0'
    print("Opening port %s..." % port)
    k.pRobotConnect(port)
    k.SetCommStatus(True)
    print("Port %s opened" % port)
    k.ExecLoad()
    return k
  
  def tstShortRun(k, sec=10):
    """ Short run test """
    iv = 0.1
    i = iv
    state = k.GetRobotState()
    if state == Gluon.EServerState.Ready:
      k.ExecStart()
    while i < sec:
      print('Tick %4.1f' % i)
      print(k.ShadowGet(kvals.KHR1SensorMimeTypeServo))
      time.sleep(iv)
      i += iv
    print(k.ShadowGet(kvals.KHR1SensorMimeTypeServo))
    k.ExecUnload()
  
  #--
  def main():
    """ vKHR1 Unit Test Main """
    k = tstCreate()
    tstShortRun(k, 10)

  # run unit test
  main()
