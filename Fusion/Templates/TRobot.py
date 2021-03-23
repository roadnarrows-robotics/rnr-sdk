################################################################################
#
# TRobot.py
#

""" User Template for a Threaded Virtual Robot.

Instructions:
  Copy this file to your development area, renaming it appropriately.
  Search for the string 'USERACTION' for things you should do and
  USER<x> for things to change.
  Don't forget to add:
    your directory to the 'RobotPluginPath'
    create or update __plugins__.py file entry

Remember, this is just one example of a template for quick development.
See the Core/vRobotThreaded.py for full threaded robot capabilities or
Core/vRobot.py for even more unrestricted designs.

The execution cycle is defined as: SenseReact()

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
import Fusion.Core.vRobot as vRobot
import Fusion.Core.vRobotThreaded as vRobotThreaded

import Fusion.Gui.GuiUtils as gut
import Fusion.Gui.GuiDlgExecOpt as GuiDlgExecOpt
import Fusion.Gui.GuiDlgSerConn as GuiDlgSerConn
import Fusion.Gui.GuiDlgAbout as GuiDlgAbout
import Fusion.Gui.GuiDlgMsgBox as msgbox


#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------
# USERACTION: add any specific global data here.


# USERACTION: typical ini definition dictionary. add entries as needed
# USERACTION: if the dd is shared across bots, then move to separate file


# USERACTION: set your robot's mime type here
# Robot MIME Type
RobotMimeType       = 'robot/USERROBOTSUBTYPE'

# Sensor MIME SubTypes
# USERACTION: set your robot's sensor mime subtypes here
SensorMimeTypeUSER  = 'sensor/USER'   # a real sensor type
SensorMimeTypeTime  = 'sensor/time'   # a derived sensor type

# Effector MIME SubTypes
# USERACTION: set your robot's effector mime subtypes here
EffectorMimeTypeUSER  = 'effector/USER'   # a real effector type


# USERACTION: add ini section names here
RobotIniDDSectOpts  = RobotMimeType + '/' + 'options'
RobotIniDDSectConn  = RobotMimeType + '/' + 'connection'

# Ini Definition Dictionary.
RobotIniDD = {
  # section
  RobotIniDDSectOpts: ['USERROBOTNAME robot options',
  {
    # USERACTION: Add other ini options section data here

    # Execution
    'ExecCycle':          [0.10, 'Execution think/act cycle time (seconds).'],
    'ExecStepSize':       [1.0, "Execution 'Step' size (seconds)."]
  }],

  # section
  # USERACTION: standard serial connection dialog. delete or modify as necess.
  RobotIniDDSectConn: ['USERROBOTNAME robot connection settings',
    {
      'port':         [None,  'Connection port (device).'],
      'baudrate':     [9600,  'Connection baudrate.'],
      'bytesize':     [8,     'Connection bytesize.'],
      'parity':       ['N',   'Connection parity.'],
      'stopbits':     [1,     'Connection stopbits.']
    }],

  # USERACTION: add other ini sections here
}
 

# USERACTION: change USERROBOT to your class name.
#-------------------------------------------------------------------------------
# CLASS: USERROBOT
#-------------------------------------------------------------------------------
class USERROBOT(vRobotThreaded.vRobotThreaded):
  """ User Template for Virtual Robot Threaded Virtual Class. """

  #--
  def __init__(self, client=None, debuglevel=0, debugfout=None):
    """ Initialize instance.

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
    # USERACTION: initialize any specific data and resources here

    # standard threaded robot initialization
    vRobotThreaded.vRobotThreaded.vRobotInit(self);

    # robot gui initializations (after vRobotThreaded)
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
    """ Returns the robot type.

        Return Value:
          The robot MIME type string.
    """
    return RobotMimeType

  #--
  def HasName(self):
    """ Returns the short robot name(s) string.

        Return Value:
          The robot name string.
    """
    # USERACTION: set your robot's name
    return 'USERROBOTNAME'

  #--
  def IsVersion(self):
    """ Returns the vRobot version(s) string.

        Return Value:
          The robot version(s) string.
    """
    # USERACTION: set your robot's version
    return '1.0'

  #--
  def IsRobotVersion(self):
    """ Returns the physical robot version(s) string.

        Return Value:
          Returns versions string list [ver1, ver2, ...] if connected.
          Else returns 'unknown'.
    """
    vers = None
    if self.mCommIsUp:
      # USERACTION: get the robot's version string(s)
      pass
    return 'unknown'

  #--
  def HasDesc(self):
    """ Returns a short description of this vRobot.

        Return Value:
          Multi-line description string.
    """
    # USERACTION: set your robot's short description
    sDesc = """USERROBOTDESC."""
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
    # USERACTION: add all robot real and derived sensors here
    sensorDict = {}

    # USERACTION: modify or delete
    # derived time 'sensor'
    sensorDict['time_stamp'] = {
      'mimetype': SensorMimeTypeTime,
      'units': 's'
    }
    sensorDict['dt'] = {
      'mimetype': SensorMimeTypeTime,
      'units': 's'
    }

    # USERACTION: example of adding real sensors
    # USER sensors
    for angle in ['90', '0', '270']:
      id = 'USER_%d' + angle 
      zeta = math.radians(angle)
      sensorDict[id] = {
        'mimetype': SensorMimeTypeUSER,
        'zeta': zeta,
        'angrange':math.radians(30.0),
        'range': [3.0, 2000.0],
        'rawunits': 'adc-10bit',
        'units': 'mm'
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
    # USERACTION: add all robot real effectors here
    effectorDict = {}

    # USERACTION: example of adding real effectors
    # motors
    effectorDict['motor_left'] = {
      'mimetype': EffectorMimeTypeUSER,
      'zeta': math.radians(90),
      'resolution': 0.5,
      'range': [-300.0, 300.0],
      'units': 'mm/s'
    }
    effectorDict['motor_right'] = {
      'mimetype': EffectorMimeTypeUSER,
      'zeta': math.radians(270),
      'resolution': 0.5,
      'range': [-300.0, 300.0],
      'units': 'mm/s'
    }

    return effectorDict
 
  #--
  def HasPhysicalProperties(self):
    """ Returns the dictionary of vRobot physical properties.
    
        Return Value:
          Dictionary of physical properties in the format:
            {property:<val>, ... }
    """
    # USERACTION: example of robot's physical properties
    # USERACTION: modify as needed
    return {
      'diameter': {'val': 100.0, 'units': 'mm'},
      'baseheight': {'val': 50.0, 'units': 'mm'},
      'baseweight': {'val': 1000.0, 'units': 'g'},
      'wheelbase': {'val': 90.0, 'units': 'mm'},
    }

  #--
  def HasBellumGoalControls(self):
    """ Returns the maximal list of goal controls (kwGoals) supported
        by this vRobot. 
    
        Return Value:
          List of goal controls.
    """
    # USERACTION: example of robot's bellum capabilities
    # USERACTION: modify as needed
    return ['speed', 'theta']


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Ini (Re)Initialization Members Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def IniInit(self):
    """ Initialize from parsed 'ini' configuration. """
    self.mIniDD = RobotIniDD
    ini         = self.GSGetIni()

    # load all non-existing ini entries with defaults
    for section,sdata in self.mIniDD.iteritems():
      optdict = sdata[1]
      for option,odata in optdict.iteritems():
        if ini.IniGet(section, option) == ini.NullObj:
          ini.IniSet(section, option, odata[0])

    # load vRobot run-time options
    self.IniInitOpt()

    # load vRobot connection settings
    self.IniInitConn()

    # load sense and react operational settings
    self.IniInitSenseReact()

  #--
  def IniInitOpt(self):
    """ Initialized vRobot options from parsed configuration. """
    iniDD   = self.mIniDD
    ini     = self.GSGetIni()
    section = RobotIniDDSectOpts
    optDict = iniDD[section][1]

    self.mOpt = {}

    for option in optDict:
      self.mOpt[option] = ini.IniGet(section, option)

    self.SetExecSizes(self.mOpt['ExecCycle'], self.mOpt['ExecStepSize'])

  #--
  def IniInitConn(self):
    """ Initialized serial connection settings from parsed configuation. """
    iniDD   = self.mIniDD
    ini     = self.GSGetIni()
    section = RobotIniDDSectConn
    optDict = iniDD[section][1]

    self.mConn = {}

    for option in optDict:
      self.mConn[option] = ini.IniGet(section, option)

  #--
  def IniInitSenseReact(self):
    """ Initialized sense and react operational settings from parsed
        configuration.
    """
    # USERACTION: add any 'sense-react' specific ini configuration here
    pass


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Sensor Shadow Member Functions
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
    # USERACTION: all shadowed sensor data should be initialized here
    self.GSLock()

    # the shadow
    self.mShadow = {}

    # time 'sensor' group
    groupId = SensorMimeTypeTime
    self.mShadow[groupId]          = {}
    self.mShadow[groupId]['dt_i']  = 0.0
    self.mShadow[groupId]['t_0']   = 0

    # USERACTION: example of initializing real sensors
    # USER sensors
    groupId = SensorMimeTypeUSER
    self.mShadow[groupId] = {}
    for angle in ['90', '0', '270']:
      id = 'USER_%d' + angle 
      self.mShadow[groupId][id] = 0.0

    self.GSUnlock()

  #--
  def ShadowUpdate(self, groupId='all'):
    """ Update of the shadow data group(s) from the current states of
        the robot. The vRobot automatically updates the shadow in it's
        execution cycle.  Ancillary threads (e.g. windows) and vBrain
        functions may also request updates. 

        Note: all shadow groups are updated to keep synchronicity.

        Parameters:
          groupId  - shadow group Id. (ignored)
        
        Return Value:
          Returns dictionary of current read sensor values.
    """
    # USERACTION: all shadowed sensor data should be updated from the
    # USERACTION: physical/simulate robot sensors.
    self.GSLock()

    # USERACTION: modify or delete
    # time 'sensor' group
    group = self.mShadow[SensorMimeTypeTime]
    t_now = time.time()
    group['dt_i'] = t_now - group['t_i']
    group['t_i']  = t_now
    if __debug__:
      self.mDbg.d4print(' %s: %s:' % (SensorMimeTypeTime, repr(group)))

    # USERACTION: example of reading real sensors
    # USER sensors
    group = self.mShadow[SensorMimeTypeUSER]
    for angle in ['90', '0', '270']:
      id = 'USER_%d' + angle 
      self.mShadow[id] = 0.0 # USERACTION: read real data here
    if __debug__:
      self.mDbg.d4print(' %s: %s:' % (SensorMimeTypeUSER, repr(group)))

    self.GSUnlock()


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # (Cere)Bellum Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def BellumSetGoals(self, **kwGoals):
    """ Set new vRobot low-level goals. The old goal set is replaced by
        this new goal set.

        Parameters:
          kwGoals  - argument list of keyword=value goals(s) 
                     (implementation specific)
        
        Return Value:
          Returns the new current goal set. 
    """
    for k,v in kwGoals.iteritems():
      # USERACTION: do goal checks here
      self.mBellumGoalSet[k] = v
    # USERACTION: do goal final checks and action here
    return self.mBellumGoalSet

  #--
  def BellumNullGoals(self):
    """ Set the vRobot's low-level goals to the null set
        (i.e. resting state).

        Return Value:
          Returns the new current goal set (implementation specific). 
    """
    # USERACTION: null out all bellum goals here
    self.mBellumGoalSet['speed'] = 0.0
    self.mBellumGoalSet['theta'] = 0.0
    return self.mBellumGoalSet


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vRobot Do's
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
      if True:    # USERACTION: stop the robot here
        if __debug__:
          self.mDbg.d2print('Emergency Stop of robot succeeded')
        return
      tries += 1
    if __debug__:
      self.mDbg.d2print('Emergency Stop of robot Failed')



  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Sense-React Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def SenseReactInit(self):
    """ Initialize Sense-React Operations.

        Event: Starting event

        Execution Context: Calling thread
    """
    vRobotThreaded.vRobotThreaded.SenseReactInit(self)

    # USERACTION: add any sense initialiazation here

    # USERACTION: modify or delete
    # start 'time' sensor
    group = self.mShadow['sensor/time']
    group['t_0']  = group['t_i'] = time.time()  # time 0
    group['dt_i'] = 0.0                         # delta time

  #--
  def SenseReact(self):
    """ Sense-React Behavior.

        Execution Context: Bot thread
    """
    if self.mOnHold:
      return

    vRobotThreaded.vRobotThreaded.SenseReact()

    # USERACTION: sense and react to sensory input to track to golas here


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Physical Robot Members Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def pRobotInit(self):
    """ Place the physical robot into a known, initialized state.

        Return Value:
          None.
    """

    if not self.IsCommUp():
      self.GSReportErrorStatus("Cannot initialize the robot: no connection")
      return
    # USERACTION: initialize the physical robot. typically this includes
    # USERACTION: stopping the robot, zeroing odometry, etc

  #--
  def pRobotConnect(self, port, baudrate, bytesize, parity, stopbits):
    """ Connect to the robot.

        Parameters:
          port      - serial connection port
          baudrate  - serial connection baudrate
          bytesize  - data byte size (bits)
          parity    - data parity: one of: 'E', 'O', 'N'
          stopbits  - number of stop bits 

        Return Value:
          None
    """
    self.GSReportNormalStatus("Opening serial connection %s %d-%d-%s-%d..." % \
                     (port, baudrate, bytesize, parity, stopbits))
    try:
      # USERACTION: do real connect here
      self.SetCommStatus(True)
    except IOError as err:
      s = "%s" % err
      self.GSReportErrorStatus(s)
      self.SetCommStatus(False)

  #--
  def pRobotDisconnect(self):
    """ Disconnnect communicaton channel with the robot. The robot
        is stopped prior to closing the channel.

        Return Value:
          None.
    """
    if self.IsCommUp():
      # USERACTION: do real disconnect here
      self.SetCommStatus(False)

  #--
  def pRobotHold(self):
    """ Hold position of physical robot.

        Return Value:
          None.
    """
    # USERACTION: stop the robot, but don't reset any robot sensors,
    # USERACTION: state, etc
    pass


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # Gui 
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def GuiInit(self):
    """ Initialize GUI. """
    # USERACTION: add other menu items as needed
    # Menubar list
    self.mMenuBarList = Gluon.GluonMenuBarList()
    self.mMenuBarList.add('Robot|Options...', 'command', 
        callback=self.GuiCbRobotOptions)

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
    self.mMenuBarList.add('Help|About Robot...', 'command', 
        callback=self.GuiCbHelpAbout)

    # Plugin Toolbar list
    # USERACTION: add toolbar items as needed, or delete
    self.mToolBarList = Gluon.GluonToolBarList()
    self.mToolBarList.add('connect',
        self.GuiCbRobotConnect, 
        tooltip='Establish a serial connection with the USERROBOTNAME',
        imagefile=gut.GetFusionImageFileName('SerConn.gif'),
        altStates={self.mServerId:[
          Gluon.EServerState.Ready,
          Gluon.EServerState.Running,
          Gluon.EServerState.Paused,
          Gluon.EServerState.Stepping
        ]},
        alttooltip='Disconnect serial connection to the USERROBOT',
        altimagefile=gut.GetFusionImageFileName('SerDisc.gif'))

  #--
  def GuiDeinit(self):
    """ Deinitialize GUI objects. """
    # USERACTION: do any gui de-initialization here
    pass

  #--
  def GuiCbRobotOptions(self):
    """ Standard 'Options' menu callback. """
    if __debug__: self.mDbg.d1print('Robot|Options')
    
    section = RobotIniDDSectOpts

    # option defaults
    iniDD   = self.mIniDD
    optDict = iniDD[section][1]
    optDfts = {}
    for option,odata in optDict.iteritems():
      optDfts[option] = odata[0]

    # get parsed ini configuation (guaranteed to exist)
    ini           = self.GSGetIni()
    section       = RobotIniDDSectOpts
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
  def GuiCbRobotConnect(self):
    """ 'Robot|Connect' menu callback. """
    if __debug__: self.mDbg.d1print('Robot|Connect')
    
    portHistory   = []
    lastSettings  = {}

    # Get parsed ini configuation (ini guraunteed to exist)
    ini     = self.GSGetIni()
    section = RobotIniDDSectConn
    optList = ini.IniGetReItems(section, 'port[0-9]+')
    for opt,val in optList:
      portHistory += [val]
    portHistory.sort()
    settingNames = GuiDlgSerConn.GetSettingNames()
    iniSettings = ini.IniGetSubItems(section, settingNames)
    lastSettings = utils.tuples2dict(iniSettings)

    # get parent gui object for this dialog
    parent = self.GSGuiGetParent()

    # the dialog
    dlg = GuiDlgSerConn.GuiDlgSerConn(parent, self.OpenCmd,
                                      portHistory=portHistory,
                                      lastSettings=lastSettings)

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

    # go back to parent gui
    self.GSGuiRaiseParent()

  #--
  def OpenCmd(self, port, baudrate=9600, bytesize=8, parity='N', stopbits=1):
    """ Open serial connection to the physical robot. """
    # USERACTION: open serial connection to robot. modify or delete and
    # USERACTION: replace with real function.
    pass

  #--
  def GuiCbRobotDisconnect(self):
    """ 'Disonnect' menu callback. """
    if __debug__: self.mDbg.d1print('Robot|Disconnect')
    self.SetCommStatus(False)
    # USERACTION: do real disconnect here

  #--
  def GuiCbHelpAbout(self):
    """ 'About' menu callback. """
    if __debug__: self.mDbg.d1print('Help|About Robot')

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
