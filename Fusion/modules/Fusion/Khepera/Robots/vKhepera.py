################################################################################
#
# vKhepera.py
#

""" Virtual Khepera Robot.

Virtual Khepera II Robot shadows the physical Khepera II robot.

Author: Robin D. Knight
Email:  robin.knight@roadnarrowsrobotics.com
URL:    http://www.roadnarrowsrobotics.com
Date:   2005.11.08

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

import time
import threading as thread
import math

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

import Fusion.Khepera.Cmd.KheCmdBase as KheBase
import Fusion.Khepera.Cmd.KheCmdGP2D120 as KheCmd
import Fusion.Khepera.Shells.KheGP2D120Shell as KheCmdShell
import Fusion.Khepera.Robots.KheValues as kvals
import Fusion.Khepera.Robots.KheIniDD as KheIniDD
import Fusion.Khepera.Gui.GuiDlgKheOpt as GuiDlgKheOpt
import Fusion.Khepera.Gui.GuiDlgKheCalIrLed as GuiDlgKheCalIrLed
import Fusion.Khepera.Gui.GuiWinKheTrip as GuiWinKheTrip
import Fusion.Khepera.Gui.GuiWinKheNav as GuiWinKheNav



#-------------------------------------------------------------------------------
# Global Data
#-------------------------------------------------------------------------------

twopi   = 2.0 * math.pi

# sensor converted value, raw value indices
CVTVAL  = 0
RAWVAL  = 1

#-------------------------------------------------------------------------------
# CLASS: vKhepera
#-------------------------------------------------------------------------------
class vKhepera(vRobotThreaded.vRobotThreaded):
  """ Virtual Khepera Robot Class. """

  #--
  def __init__(self, client=None, debuglevel=0, debugfout=None):
    """ Initialize vKhepera instance.

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
    # khepera serial commands
    self.mCmd = KheCmd.KheCmdGP2D120(dbgobj=self.mDbg)
    self.mCommIsUp = self.mCmd.IsOpen()

    # khepera gui initializations
    self.GuiInit()

    # common threaded robot initialization (includes ini initialization)
    vRobotThreaded.vRobotThreaded.vRobotInit(self)

    # add menu bar
    self.GSSetServerMenuBarList(self.mMenuBarList)

    # add tool bar
    self.GSSetServerToolBarList(self.mToolBarList)


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vKhepera Attribute Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def IsType(self):
    """ Returns the Khepera robot type.

        Return Value:
          The Khepera robot MIME type string.
    """
    return kvals.KheMimeType

  #--
  def HasName(self):
    """ Returns the short vKhepera name(s) string.

        Return Value:
          The robot name(s) string which may include either or both
          the vRobot and physical robot names.
    """
    return 'vKhepera'

  #--
  def IsVersion(self):
    """ Returns the vKhepera version(s) string.

        Return Value:
          The vRobot version(s) string.
    """
    return '1.1'

  #--
  def IsRobotVersion(self):
    """ Returns the Khepera robot version(s) string.

        Return Value:
          Returns versions string list [bios, protocol] if connected.
          Else returns 'unknown'.
    """
    vers = None
    if self.mCommIsUp:
      vers = self.mCmd.CmdGetVersion()
      if vers:
        return [vers[0], vers[1]]
    return 'unknown'

  #--
  def HasDesc(self):
    """ Returns a short description of this vRobot.

        Return Value:
          Multi-line description string.
    """
    sDesc = """\
Khepera II robot with a mounted Sharp GP2D120 IR LED sensor.
Manufactured by K-Team of Switzerland."""
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
      'mimetype': kvals.KheSensorMimeTypeTime,
      'units': 's'
    }
    sensorDict['dt'] = {
      'mimetype': kvals.KheSensorMimeTypeTime,
      'units': 's'
    }

    # proximity sensors
    angrange = math.radians(KheBase.KheIrSensorAngRange)
    n = 0
    for irsensor in KheBase.KheIrSensorOrder:
      id = 'prox_' + irsensor
      zeta = math.radians(KheBase.KheIrSensorAngle[n])
      sensorDict[id] = {
        'mimetype': kvals.KheSensorMimeTypeProximity,
        'zeta': zeta,
        'angrange':angrange,
        'range': [KheBase.KheIrSensorValMin, KheBase.KheIrSensorValMax],
        'rawunits': 'adc-10bit',
        'units': 'mm'
      }
      n += 1

    # ambient sensors
    angrange = math.radians(KheBase.KheIrSensorAngRange)
    n = 0
    for irsensor in KheBase.KheIrSensorOrder:
      id = 'amb_' + irsensor
      zeta = math.radians(KheBase.KheIrSensorAngle[n])
      sensorDict[id] = {
        'mimetype': kvals.KheSensorMimeTypeAmbient,
        'zeta': zeta,
        'angrange':angrange,
        'range': [KheBase.KheIrSensorValMin, KheBase.KheIrSensorValMax],
        'rawunits': 'adc-10bit',
        'units': 'mm'
      }
      n += 1

    # odometry
    sensorDict['odometer_left'] = {
      'mimetype': kvals.KheSensorMimeTypeOdometer,
      'zeta': math.radians(90),
      'range': [0, KheBase.KheOdometerMmMax],
      'resolution': KheBase.KheOdometerMmpt,
      'rawunits': 'ticks',
      'units': 'mm'
    }
    sensorDict['odometer_right'] = {
      'mimetype': kvals.KheSensorMimeTypeOdometer,
      'zeta': math.radians(270),
      'range': [0, KheBase.KheOdometerMmMax],
      'resolution': KheBase.KheOdometerMmpt,
      'rawunits': 'ticks',
      'units': 'mm'
    }
    sensorDict['pathdist'] = {
      'mimetype': kvals.KheSensorMimeTypeOdometer,
      'units': 'mm'
    }
    sensorDict['theta'] = {
      'mimetype': kvals.KheSensorMimeTypeOdometer,
      'units': 'rad'
    }
    sensorDict['x'] = {
      'mimetype': kvals.KheSensorMimeTypeOdometer,
      'units': 'mm'
    }
    sensorDict['y'] = {
      'mimetype': kvals.KheSensorMimeTypeOdometer,
      'units': 'mm'
    }
  
    # speedometers
    sensorDict['speedometer_left'] = {
      'mimetype': kvals.KheSensorMimeTypeSpeedometer,
      'zeta': math.radians(90),
      'range': [-KheBase.KheSpeedMmpsPosMax, KheBase.KheSpeedMmpsPosMax],
      'resolution': KheBase.KheSpeedMmpsPosMin,
      'rawunits': 'pulses/10ms',
      'units': 'mm/s'
    }
    sensorDict['speedometer_right'] = {
      'mimetype': kvals.KheSensorMimeTypeSpeedometer,
      'zeta': math.radians(270),
      'range': [-KheBase.KheSpeedMmpsPosMax, KheBase.KheSpeedMmpsPosMax],
      'resolution': KheBase.KheSpeedMmpsPosMin,
      'rawunits': 'pulses/10ms',
      'units': 'mm/s'
    }
    sensorDict['pathspeed'] = {
      'mimetype': kvals.KheSensorMimeTypeSpeedometer,
      'units': 'mm/s'
    }
  
    # Sharp GP2D120 IR LED range finder add-on
    sensorDict['distmeas_front0'] = {
      'mimetype': kvals.KheSensorMimeTypeGP2D120,
      'manufacturer': 'Sharp',
      'product': 'GP2D120',
      'zeta': 0.0,
      'angrange': math.radians(20),
      'range': [KheCmd.GP2D120DistMin, KheCmd.GP2D120DistMax],
      'resolution': 2.0,
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
    effectorDict = {}

    # motors
    effectorDict['motor_left'] = {
      'mimetype': kvals.KheEffectorMimeTypeWheelMotor,
      'zeta': math.radians(90),
      'resolution': KheBase.KheSpeedMmpsPosMin,
      'range': [-KheBase.KheSpeedMmpsPosMax, KheBase.KheSpeedMmpsPosMax],
      'units': 'mm/s'
    }
    effectorDict['motor_right'] = {
      'mimetype': kvals.KheEffectorMimeTypeWheelMotor,
      'zeta': math.radians(270),
      'resolution': KheBase.KheSpeedMmpsPosMin,
      'range': [-KheBase.KheSpeedMmpsPosMax, KheBase.KheSpeedMmpsPosMax],
      'units': 'mm/s'
    }

    # LEDs
    effectorDict['led0'] = {
      'mimetype': kvals.KheEffectorMimeTypeLed,
      'color': 'red',
      'range': [0, 1],
      'units': 'off/on'
    }
    effectorDict['led1'] = {
      'mimetype': kvals.KheEffectorMimeTypeLed,
      'color': 'red',
      'range': [0, 1],
      'units': 'off/on'
    }

    return effectorDict
  
  #--
  def HasPhysicalProperties(self):
    """ Returns the dictionary of KheperaII physical properties.
    
        Return Value:
          Dictionary of physical properties in the format:
            {property:<val>, ... }
    """
    return {
      'diameter': {'val': KheBase.KheBaseDiameter, 'units': 'mm'},
      'baseheight': {'val': KheBase.KheBaseHeight, 'units': 'mm'},
      'baseweight': {'val': KheBase.KheBaseWeight, 'units': 'g'},
      'wheelbase': {'val': KheBase.KheWheelBase, 'units': 'mm'},
      'payloadweight': {'val': KheBase.KhePayloadWeight, 'units': 'g'}
    }

  #--
  def HasBellumGoalControls(self):
    """ Returns the maximal list of goal controls (kwGoals) supported
        by vKhepera. 
    
        Return Value:
          List of goal controls.
    """
    return ['speed', 'theta', 'dspeed', 'dtheta']


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vKhepara Ini (Re)Initialization Members Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def IniInit(self):
    """ Initialize from parsed 'ini' configuration. """

    self.mIniDD = KheIniDD.GetIniDD()
    ini         = self.GSGetIni()

    # load all non-existing ini entries with defaults
    for section,sdata in self.mIniDD.iteritems():
      optdict = sdata[1]
      for option,odata in optdict.iteritems():
        if ini.IniGet(section, option) == ini.NullObj:
          ini.IniSet(section, option, odata[0])

    # load vKhepera run-time options
    self.IniInitOpt()

    # load vKhepera connection settings
    self.IniInitConn()

    # load vKhepera proximity ir led sensor settings
    self.IniInitProximity()

    # load vKhepera ambient ir led sensor settings
    self.IniInitAmbient()

    # load vKhepera GP2D120 sensor settings
    self.IniInitGP2D120()

    # load sense-react operational settings
    self.IniInitSenseReact()

  #--
  def IniInitOpt(self):
    """ Initialized vKhepera options from parsed configuration. """
    iniDD   = self.mIniDD
    ini     = self.GSGetIni()
    section = KheIniDD.IniDDSectOpts
    optDict = iniDD[section][1]

    self.mOpt = {}

    for option in optDict:
      self.mOpt[option] = ini.IniGet(section, option)
      if option == 'UseOdometrySensors':
        if self.mOpt[option]:
          run_time='enabled'
        else:
          run_time='disabled'
        self.GSGuiWinUpdate('VizTrip', 'cfg', run_time=run_time)
        self.GSGuiWinUpdate('Navigator', 'cfg', run_time=run_time)

    self.SetExecSizes(self.mOpt['ExecCycle'], self.mOpt['ExecStepSize'])

  #--
  def IniInitConn(self):
    """ Initialized vKhepera connection settings from parsed configuation. """
    iniDD   = self.mIniDD
    ini     = self.GSGetIni()
    section = KheIniDD.IniDDSectConn
    optDict = iniDD[section][1]

    self.mConn = {}

    for option in optDict:
      self.mConn[option] = ini.IniGet(section, option)

  #--
  def IniInitProximity(self):
    """ Initialized vKhepera proximity sensor data from parsed 
        configuration.
    """
    ini         = self.GSGetIni()
    section     = KheIniDD.IniDDSectProximity

    options = ini.IniGetItems(section)
    calParams = {}

    for option,value in options:
      calParams[option] = value
    self.mCmd.ProximitySensorsSetCalParams(calParams)

  #--
  def IniInitAmbient(self):
    """ Initialized vKhepera ambient sensor data from parsed 
        configuration.
    """
    ini         = self.GSGetIni()
    section     = KheIniDD.IniDDSectAmbient

    options = ini.IniGetItems(section)
    calParams = {}

    for option,value in options:
      calParams[option] = value
    self.mCmd.AmbientSensorsSetCalParams(calParams)

  #--
  def IniInitGP2D120(self):
    """ Initialized vKhepera GP2D120 sensor data from parsed configuration. """
    iniDD   = self.mIniDD
    ini     = self.GSGetIni()
    section = KheIniDD.IniDDSectGP2D120 + '/' + 'front'
    optDict = iniDD[section][1]

    calData   = ini.IniGet(section, 'calData')
    usingDft  = False

    if not calData:
      dftCalib = self.mCmd.GP2D120GetDefaultCalibration()
      ini.IniSet(section, 'unitsG', dftCalib['unitsG'])
      ini.IniSet(section, 'unitsR', dftCalib['unitsR'])
      ini.IniSet(section, 'calData', dftCalib['calData'])
      usingDft  = True

    msg = 'Calibrating GP2D120 sensor'
    if usingDft:
      msg += ' to factory defaults'
    self.GSReportNormalStatus(msg)
    if __debug__: self.mDbg.d2print(msg)
    try:
      self.mCmd.GP2D120Calibrate(ini.IniGet(section, 'calData'), 
                          unitsG=ini.IniGet(section, 'unitsG'),
                          unitsR=ini.IniGet(section, 'unitsR'))
      return
    except ValueError as msg:
      self.GSReportErrorStatus('GP2D120 sensor: ' + msg)
      if __debug__: self.mDbg.d2print('Error: '+ msg)

    # calibration didn't work, try factory defaults
    if not usingDft:
      dftCalib = self.mCmd.GP2D120GetDefaultCalibration()
      ini.IniSet(section, 'unitsG', dftCalib['unitsG'])
      ini.IniSet(section, 'unitsR', dftCalib['unitsR'])
      ini.IniSet(section, 'calData', dftCalib['calData'])

      msg = 'Recalibrating GP2D120 sensor with factory defaults'
      self.GSReportNormalStatus(msg)
      if __debug__: self.mDbg.d2print(msg)
      try:
        self.mCmd.GP2D120Calibrate(ini.IniGet(section, 'calData'), 
                            unitsG=ini.IniGet(section, 'unitsG'),
                            unitsR=ini.IniGet(section, 'unitsR'))
      except ValueError as msg:
        pass  # what can we do?

  def IniInitSenseReact(self):
    """ Initialized sense-react operational settings from parsed
        configuration.
    """
    # smallest positive distance increment possible per execution cycle 
    mindist = (KheBase.KheSpeedMmpsPosMin * self.mOpt['ExecCycle'])

    # delta theta threshold
    self.mEpsilonTheta = math.fmod(mindist/KheBase.KheWheelBase, twopi)

    # fudge
    self.mEpsilonTheta   = self.mEpsilonTheta + 0.1 * self.mEpsilonTheta

        
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vKhepera Sensor and Shadow Member Functions
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
    groupId = kvals.KheSensorMimeTypeTime
    self.mShadow[groupId]          = {}
    self.mShadow[groupId]['dt_i']  = 0.0
    self.mShadow[groupId]['t_0']   = 0.0
    self.mShadow[groupId]['t_i']   = 0.0

    # proximity ir led sensors group
    groupId = kvals.KheSensorMimeTypeProximity
    self.mShadow[groupId] = {}
    for ir in KheBase.KheIrSensorOrder:
      self.mShadow[groupId]['prox_'+ir] = (0, 0)

    # ambient ir led sensors group
    groupId = kvals.KheSensorMimeTypeAmbient
    self.mShadow[groupId]    = {}
    for ir in KheBase.KheIrSensorOrder:
      self.mShadow[groupId]['amb_'+ir] = (0, 0)

    # odometer sensors group
    groupId = kvals.KheSensorMimeTypeOdometer
    self.mShadow[groupId]                   = {}
    self.mShadow[groupId]['odometer_left']  = (0, 0)
    self.mShadow[groupId]['odometer_right'] = (0, 0)
    self.mShadow[groupId]['pathdist']       = 0.0
    self.mShadow[groupId]['theta']          = 0.0
    self.mShadow[groupId]['x']              = 0.0
    self.mShadow[groupId]['y']              = 0.0

    # speedometer sensors group
    groupId = kvals.KheSensorMimeTypeSpeedometer
    self.mShadow[groupId]                      = {}
    self.mShadow[groupId]['speedometer_left']  = (0, 0)
    self.mShadow[groupId]['speedometer_right'] = (0, 0)
    self.mShadow[groupId]['pathspeed']         = 0.0

    # distance measuring ir led sensors group
    groupId = kvals.KheSensorMimeTypeGP2D120
    self.mShadow[groupId]                    = {}
    self.mShadow[groupId]['distmeas_front0'] = (-1, 0)

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
    self.GSLock()

    # time 'sensor' group (always on)
    group = self.mShadow[kvals.KheSensorMimeTypeTime]
    t_now = time.time()
    group['dt_i'] = t_now - group['t_i']
    group['t_i']  = t_now
    if __debug__: self.mDbg.d4print(' %s: %s:' % \
                        (kvals.KheSensorMimeTypeTime, repr(group)))

    # proximity ir led sensors group
    if self.mOpt['UseProximitySensors']:
      self.mShadow[kvals.KheSensorMimeTypeProximity] = self.SenseProximity()
      
    # ambient ir led sensors group
    if self.mOpt['UseAmbientSensors']:
      self.mShadow[kvals.KheSensorMimeTypeAmbient] = self.SenseAmbient()

    # distance measuring ir led sensors group
    if self.mOpt['UseDistMeasSensors']:
      self.mShadow[kvals.KheSensorMimeTypeGP2D120] = self.SenseGP2D120()

    # speedometer sensors group and location group (partial)
    if self.mOpt['UseSpeedometerSensors']:
      self.mShadow[kvals.KheSensorMimeTypeSpeedometer] = self.SenseSpeedometer()

    # odometry sensors group and location group (partial)
    if self.mOpt['UseOdometrySensors']:
      self.mShadow[kvals.KheSensorMimeTypeOdometer] = self.SenseOdometer()

    self.GSUnlock()

  #--
  def SenseProximity(self):
    """ Read the Proximity IR Sensor data. The read is only attempted if
        communication to the robot is up.

        The data returned is a dictionary of sensed proximity sensor values
        where each datum is a 2-tuple (<calib>, <raw>) of calibrated and
        raw data. 

        Ancillary threads (e.g. windows) and vBrain functions may also 
        request sensor updates. 

        Return Value:
          Dictionary of read values on success. Dictionary of default
          values on failure.
    """
    sensorData = {}
    if self.IsCommUp(): # communication must be up and the module attached
      sensorData = self.mCmd.CmdReadProximitySensors(incraw=True)
    if not sensorData:  # data dropout - fill with default values
      sensorData = {}
      for ir in KheBase.KheIrSensorOrder:
        sensorData['prox_'+ir] = (0, 0)
    if __debug__: self.mDbg.d4print(' %s: %s:' % \
                      (kvals.KheSensorMimeTypeProximity, repr(sensorData)))
    return sensorData
 
  #--
  def SenseAmbient(self):
    """ Read the Ambient IR Sensor data. The read is only attempted if
        communication to the robot is up.

        The data returned is a dictionary of sensed proximity sensor values
        where each datum is a 2-tuple (<calib>, <raw>) of calibrated and
        raw data. 

        Ancillary threads (e.g. windows) and vBrain functions may also 
        request sensor updates. 

        Return Value:
          Dictionary of read values on success. Dictionary of default
          values on failure.
    """
    sensorData = {}
    if self.IsCommUp(): # communication must be up
      sensorData = self.mCmd.CmdReadAmbientSensors(incraw=True)
    if not sensorData:  # data dropout - fill with default values
      sensorData = {}
      for ir in KheBase.KheIrSensorOrder:
        sensorData['amb_'+ir] = (0, 0)
    if __debug__: self.mDbg.d4print(' %s: %s:' % \
                      (kvals.KheSensorMimeTypeAmbient, repr(sensorData)))
    return sensorData
 
  #--
  def SenseGP2D120(self):
    """ Read the GP2D120 IR distance measuring sensor data. The read is
        only attempted if communication to the robot is up.

        The data returned is a dictionary of sensed proximity sensor values
        where each datum is a 2-tuple (<calib>, <raw>) of calibrated and
        raw data. 

        Ancillary threads (e.g. windows) and vBrain functions may also 
        request sensor updates. 

        Return Value:
          Dictionary of read values on success. Dictionary of default
          values on failure.
    """
    sensorData = {}
    readData = None
    if self.IsCommUp(): # communication must be up
      readData = self.mCmd.CmdGP2D120MeasureDist(incraw=True)
    if readData is not None:  # got the data
      sensorData['distmeas_front0'] = readData
    else:   # data dropout - fill with default values
      sensorData['distmeas_front0'] = KheCmd.GP2D120DistInf
    if __debug__: self.mDbg.d4print(' %s: %s:' % \
                      (kvals.KheSensorMimeTypeGP2D120, repr(sensorData)))
    return sensorData

   #--
  def SenseSpeedometer(self):
    """ Read the speedometer data. The read is only attempted if
        communication to the robot is up.

        The data returned is a dictionary of read speedometer values
        for each motor.

        Ancillary threads (e.g. windows) and vBrain functions may also 
        request sensor updates. 

        Return Value:
          Dictionary of read values on success. Dictionary of last
          known values on failure.
    """
    # copies are suppose to be atomic, hence thread safe
    sensorData  = self.mShadow[kvals.KheSensorMimeTypeSpeedometer].copy()

    readData = None
    if self.IsCommUp(): # communication must be up
      readData = self.mCmd.CmdGetSpeed(incraw=True)
    if readData is not None:  # got the data
      sensorData['speedometer_left']  = (readData[0], readData[2])
      sensorData['speedometer_right'] = (readData[1], readData[3])
      speedLeft   = sensorData['speedometer_left'][CVTVAL]
      speedRight  = sensorData['speedometer_right'][CVTVAL]
      sensorData['pathspeed']  = (speedLeft + speedRight) / 2.0

      # update any registered window
      self.GSGuiWinUpdate('VizTrip', 'speed', **sensorData)
      self.GSGuiWinUpdate('Navigator', 'speed', **sensorData)

    if __debug__: self.mDbg.d4print(' %s: %s:' % \
                        (kvals.KheSensorMimeTypeSpeedometer, repr(sensorData)))
    return sensorData

  #--
  def SenseOdometer(self):
    """ Read the odometry data. The read is only attempted if
        communication to the robot is up.

        The data returned is a dictionary of read odometer values
        for each motor.

        Ancillary threads (e.g. windows) and vBrain functions may also 
        request sensor updates. 

        Return Value:
          Dictionary of read values on success. Dictionary of last
          known values on failure.
    """
    # copies are suppose to be atomic, hence thread safe
    sensorData  = self.mShadow[kvals.KheSensorMimeTypeOdometer]

    readData = None
    if self.IsCommUp(): # communication must be up
      readData = self.mCmd.CmdGetOdometry(incraw=True)
    if readData is not None:  # got the data
      sensorData['odometer_left']  = (readData[0], readData[2])
      sensorData['odometer_right'] = (readData[1], readData[3])

      lastDist    = sensorData['pathdist']
      distLeft    = sensorData['odometer_left'][CVTVAL]
      distRight   = sensorData['odometer_right'][CVTVAL]

      sensorData['pathdist'] = (distLeft + distRight) / 2.0
      sensorData['theta'] = self.ThetaCalc(distLeft, distRight)

      ddist = sensorData['pathdist'] - lastDist
      sensorData['x'] += math.cos(sensorData['theta']) * ddist
      sensorData['y'] += math.sin(sensorData['theta']) * ddist

      # update any registered window
      self.GSGuiWinUpdate('VizTrip', 'loc', **sensorData)
      self.GSGuiWinUpdate('Navigator', 'loc', **sensorData)

    if __debug__: self.mDbg.d4print(' %s: %s:' % \
                        (kvals.KheSensorMimeTypeOdometer, repr(sensorData)))
    return sensorData

  #--
  def SenseCfgOdometer(self, odometerLeft=0.0, odometerRight=0.0):
    """ Configure Khepera Odometry Sensors. The location 'sensor' will
        be automatically adjusted.

        Updates odometery shadow data.

        Parameters:
          odometerLeft  -  left motor odometer position in mm
                            [0.0, 343597384.0]
          odometerRight - right motor odometer position in mm
                            [0.0, 343597384.0]

        Return Value:
          None.
    """
    # copies are suppose to be atomic, hence thread safe
    sensorData  = self.mShadow[kvals.KheSensorMimeTypeOdometer]

    readData = None
    if self.IsCommUp():
      readData = self.mCmd.CmdSetOdometry(odometerLeft, odometerRight,
                                          incraw=True)
    if readData is not None:  # got the data
      sensorData['odometer_left']  = (readData[0], readData[2])
      sensorData['odometer_right'] = (readData[1], readData[3])
      distLeft    = sensorData['odometer_left'][CVTVAL]
      distRight   = sensorData['odometer_right'][CVTVAL]
      sensorData['theta'] = self.ThetaCalc(distLeft, distRight)
      sensorData['pathdist']   = 0.0
      sensorData['x'] = 0.0    # RDK: is there a way?
      sensorData['y'] = 0.0    # RDK: is there a way?

    self.mShadow[kvals.KheSensorMimeTypeOdometer] = sensorData.copy()

  #--
  def ThetaCalc(self, odometerLeft, odometerRight):
    """ Theta calculator.
    
        Parameters:
          odometerLeft  -  left motor odometer position in mm 
          odometerRight - right motor odometer position in mm 

        Return Value:
          Robot angle (radians) since initial starting point of (0.0, 0.0)
    """      
    arc = odometerRight - odometerLeft    # arc length
    theta = arc / KheBase.KheWheelBase    # raw theta
    theta = math.fmod(theta, twopi)
    if theta < 0.0:
      theta += twopi
    return theta


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vRobot Effector Member Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


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
             'speed'  - track to new speed (mm/s)
             'dspeed' - track to new delta speed (mm/s)
             'theta'  - track to new direction (radians)
             'dtheta' - track to new delta direction (radians)
          If both speed and ds (theta and dtheta) are specified
          one of the two conflicting goals are arbitrarily chosen.
        
        Return Value:
          Returns new current goal set. 
    """
    groupId = kvals.KheSensorMimeTypeOdometer
    speed = theta = None
    for k, v in kwGoals.iteritems():
      if k == 'speed':
        speed = v
      elif k == 'dspeed':
        speed = self.mShadow[kvals.KheSensorMimeTypeSpeedometer]['pathspeed']+v
      elif k == 'theta':
        theta = v
      elif k == 'dtheta':
        theta = self.mShadow[kvals.KheSensorMimeTypeOdometer]['theta']+v
    if speed is not None: 
      speed = self.mCmd.GetNearestSpeed(speed);
    else:
      speed = self.mBellumGoalSet['speed']
    if theta is not None:
      theta = math.fmod(theta, twopi)
      if theta < 0.0:
        theta += twopi
    else:
      theta = self.mBellumGoalSet['theta']
    self.mBellumGoalSet = {'speed': speed, 'theta': theta}

    self.GSGuiWinUpdate('Navigator', 'goals', **self.mBellumGoalSet)

    return self.mBellumGoalSet

  #--
  def BellumNullGoals(self):
    """ Set the vKhepera's low-level goals to the null set
        (i.e. resting state).

        Return Value:
          Returns new current goal set.
    """
    self.mBellumGoalSet['speed'] = 0.0
    self.mBellumGoalSet['theta'] = 0.0
    self.GSGuiWinUpdate('Navigator', 'goals', **self.mBellumGoalSet)
    return self.mBellumGoalSet


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vKhepera Do's
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
      if self.mCmd.CmdStop():
        if __debug__: self.mDbg.d2print(
            'Emergency Stop of Khepera robot succeeded')
        return
      tries += 1
    if __debug__:
      self.mDbg.d2print('Emergency Stop of Khepera robot Failed: %s' % \
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
  # vKhepara Sense-React Member Functions
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

    # Control Parameters
    self.mSpeedPosMax  = KheBase.KheSpeedMmpsPosMax # positive maximum speed
    self.mSpeedPosMin  = KheBase.KheSpeedMmpsPosMin # positive minimum speed
    self.mEpsilonSpeed = self.mSpeedPosMin / 2.0    # delta speed threshold

    # pid parameters
    self.mKp        = 0.5     # proportional constant
    self.mKi        = 0.01    # integral constant
    self.mKd        = 0.005   # derivative constant
    self.mWp        = 1.0     # proportional setpoint weight
    self.mWd        = 1.0     # derivative setpoint weight
    self.mAccumErr  = 0.0     # accumulated error
    self.mErrLast   = 0.0     # last error 

    # speed shadow group
    self.mSpeedGroup = self.mShadow['sensor/speedometer-pwm']

  #--
  def _dtheta(self, theta_1, theta_2):
    """ calculate delta theta in range (-pi, pi] """
    dtheta = theta_1 - theta_2
    if dtheta > math.pi:
      dtheta = dtheta - twopi
    elif dtheta < -math.pi:
      dtheta = dtheta + twopi
    return dtheta

  #--
  def SenseReact(self):
    """ Sense-React Behaviour. Track to current set of goals.
        Goals are speed and world direction.

        A Proportional-Integral-Derivative (PID) controller is used.

        Execution Context: Bot thread
    """
    # react on hold
    if self.mOnHold:
      return

    if __debug__: self.mDbg.d4print('SenseReact')

    # shadow robot sensors
    self.ShadowUpdate()

    # Nomenclature:
    #   subscripts: c = current state, g = goal target, l = left, r = right
    #   prefixes:   d = delta
    #   velocity = (speed, theta) polar coordinates
    #   epsilon = threshold

    # current direction and speeds
    theta_c   = self.mShadow[kvals.KheSensorMimeTypeOdometer]['theta']
    speed_l_c = self.mSpeedGroup['speedometer_left'][CVTVAL]
    speed_r_c = self.mSpeedGroup['speedometer_right'][CVTVAL]
    #print("theta_c=%f, speed_l/r_c=(%f,%f)" % (theta_c, speed_l_c, speed_r_c))

    # goal set
    speed_g = self.mBellumGoalSet['speed']
    theta_g = self.mBellumGoalSet['theta']

    # delta theta (-pi, pi]
    dtheta = self._dtheta(theta_c, theta_g)

    # delta t
    dt = self.mShadow['sensor/time']['dt_i']
    if dt == 0.0:
      dt = 0.01

    # dt errors
    err_p = self._dtheta(theta_c, self.mWp * theta_g)
    err_i = dtheta
    err_d = self._dtheta(theta_c, self.mWd * theta_g)

    # accumulated error
    self.mAccumErr += (err_i * dt)

    # pid - proportional-integral-derivative
    dtheta_u = self.mKp * err_p + \
               self.mKi * self.mAccumErr + \
               self.mKd * (err_d - self.mErrLast) / dt
    dtheta_u = math.fmod(dtheta_u, math.pi) # (-pi, pi)

    # new last error
    self.mErrLast = err_d

    # theta scaler k: 
    #   the greater the direction difference, the greater the turning speed
    k_theta = dtheta_u / math.pi  # (-1, 1)

    # determine turn component in speed goal
    if math.fabs(dtheta_u) >= self.mEpsilonTheta: # over delta theta threshold
      speed_theta = k_theta * speed_g           # take fraction of current speed
      # make sure there is some speed
      if k_theta > 0.0:
        speed_theta += self.mSpeedPosMin
      else:
        speed_theta -= self.mSpeedPosMin
    else:
      speed_theta = 0.0

    # back off linear speed component if limits are exceeded
    if speed_g + speed_theta > self.mSpeedPosMax:
      speed_g = self.mSpeedPosMax - speed_theta
    elif speed_g + speed_theta < -self.mSpeedPosMax:
      speed_g = -self.mSpeedPosMax - speed_theta
    #print("speed_g=%f" % (speed_g))

    # new left and right motor speed goals
    speed_l_g = speed_g + speed_theta
    speed_r_g = speed_g - speed_theta
    #print("speed_l/r_g=(%f,%f)" % (speed_l_g, speed_r_g))

    # send new speed setting only if there is a change in speed
    speed_l_g = self.mCmd.GetNearestSpeed(speed_l_g);
    speed_r_g = self.mCmd.GetNearestSpeed(speed_r_g);
    if speed_l_g != speed_l_c or speed_r_g != speed_r_c:
      self.mCmd.CmdSetSpeed(speed_l_g, speed_r_g)

    #if speed_g != 0.0 or theta_g != 0.0:
    #  print('theta_g=%.2f, theta_c=%.2f, dtheta_u=%.2f, speed=(%.1f, %.1f)' % \
    #    (math.degrees(theta_g), math.degrees(theta_c), math.degrees(dtheta_u),
    #     speed_l_g, speed_r_g))

    if __debug__:
      self.mDbg.d4print(
          ' cur(%.2f,%.2f) -> goal(%.2f,%.2f) -> motor(%.1f,%.1f)' % \
          (self.mShadow[kvals.KheSensorMimeTypeSpeedometer]['pathspeed'],
            theta_c, speed_g, self.mBellumGoalSet['theta'],
           speed_l_g, speed_r_g))


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vKhepara Physical Robot Members Functions
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def pRobotInit(self):
    """ Place the physical Khepera into a known, initialized state.

        Return Value:
          None.
    """

    if not self.IsCommUp():
      self.GSReportErrorStatus("Cannot initialize the Khepera: no connection")
      return
    self.mCmd.CmdStop()
    self.mCmd.CmdSetOdometry(0, 0)

  #--
  def pRobotConnect(self, port, baudrate):
    """ Connect to the Khepera robot.

        Parameters:
          port      - serial connection port
          baudrate  - serial connection baudrate

        Return Value:
          None
    """
    self.GSReportNormalStatus("Opening serial connection %s %d-%d-%s-%d..." % \
                     (port, baudrate, 8, 'N', 2))
    try:
      self.mCmd.Open(port, baudrate=baudrate)
      self.SetCommStatus(True)
    except IOError as err:
      s = "%s" % err
      self.GSReportErrorStatus(s)
      self.SetCommStatus(False)

  #--
  def pRobotDisconnect(self):
    """ Disconnnect communicaton channel with the Khepera. The Khepera
        is stopped prior to closing the channel.

        Return Value:
          None.
    """
    if self.IsCommUp():
      self.mCmd.CmdStop()
      self.mCmd.Close()
      self.SetCommStatus(False)

  #--
  def pRobotHold(self):
    """ Hold position of physical robot.

        Return Value:
          None.
    """
    if self.IsCommUp():
      self.mCmd.CmdStop()


  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  # vKhepara Gui and Support
  # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  #--
  def GuiInit(self):
    """ Initialize vKhepera GUI. """
    # Menubar list
    self.mMenuBarList = Gluon.GluonMenuBarList()

    self.mMenuBarList.add('Robot|Robot Options...', 'command', 
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
    self.mMenuBarList.add('Robot|Calibration|Proximity...', 'command', 
        callback=self.GuiCbRobotCalProximity)
    self.mMenuBarList.add('Robot|Calibration|Ambient...', 'command', 
        callback=self.GuiCbRobotCalAmbient)
    self.mMenuBarList.add('Robot|Calibration|GP2D120...', 'command', 
        callback=self.GuiCbRobotCalGP2D120)

    self.mMenuBarList.add('Robot', 'separator')
    self.mMenuBarList.add('Robot|Robot Visualizer', 'command', 
        callback=self.GuiCbRobotViz)
    self.mMenuBarList.add('Robot|Trip Vizualizer', 'command', 
        callback=self.GuiCbRobotVizTrip)
    self.mMenuBarList.add('Robot|Navigator', 'command', 
        callback=self.GuiCbRobotNav)
    self.mMenuBarList.add('Robot|Shell', 'command', 
        callback=self.GuiCbRobotShell)

    self.mMenuBarList.add('Help|About Robot...', 'command', 
        callback=self.GuiCbHelpAbout)

    # Plugin Toolbar list
    self.mToolBarList = Gluon.GluonToolBarList()
    self.mToolBarList.add('connect',
        self.GuiCbRobotConnect, 
        tooltip='Establish a serial connection with the Khepera',
        imagefile=gut.GetFusionImageFileName('SerConn.gif'),
        altStates={self.mServerId:[
          Gluon.EServerState.Ready,
          Gluon.EServerState.Running,
          Gluon.EServerState.Paused,
          Gluon.EServerState.Stepping
        ]},
        alttooltip='Disconnect serial connection to the Khepera',
        altimagefile=gut.GetFusionImageFileName('SerDisc.gif'))

  #--
  def GuiDeinit(self):
    """ Deinitialize GUI objects. """
    pass

  #--
  def GuiCbRobotOptions(self):
    """ 'Robot Options' menu callback. """
    if __debug__: self.mDbg.d1print('Robot|Options')
    
    lastSettings  = {}

    # Get parsed ini configuation (ini guraunteed to exist)
    ini           = self.GSGetIni()
    section       = KheIniDD.IniDDSectOpts
    settingNames  = GuiDlgKheOpt.GetSettingNames()
    iniSettings   = ini.IniGetSubItems(section, settingNames)
    lastSettings  = utils.tuples2dict(iniSettings)

    # get parent gui object for this dialog
    parent = self.GSGuiGetParent()

    # the dialog
    dlg = GuiDlgKheOpt.GuiDlgKheOpt(parent, lastSettings=lastSettings)

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
    section = KheIniDD.IniDDSectConn
    optList = ini.IniGetReItems(section, 'port[0-9]+')
    for opt,val in optList:
      portHistory += [val]
    portHistory.sort()
    settingNames = GuiDlgSerConn.GetSettingNames()
    iniSettings = ini.IniGetSubItems(section, settingNames)
    lastSettings = utils.tuples2dict(iniSettings)

    # get Khepera serial port supported values 
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
  def GuiCbRobotCalProximity(self):
    """ 'Proximity Sensor Calibration' menu callback """
    if __debug__: self.mDbg.d1print('Robot|Calibration|Proximity')

    sensorMimeType  = kvals.KheSensorMimeTypeProximity
    sensorName      = 'Proximity'

    # get parsed ini configuation (ini guraunteed to exist)
    ini           = self.GSGetIni()
    section       = KheIniDD.IniDDSectProximity
    iniSettings   = ini.IniGetItems(section)
    lastSettings  = utils.tuples2dict(iniSettings)
    factSettings  = {
      'KBrightnessMin': KheBase.KheIrProxMinKBrightness,
      'KBrightnessMax': KheBase.KheIrProxMaxKBrightness,
      'KBrightnessDft': KheBase.KheIrProxDftKBrightness,
      'NoiseFloorMin': KheBase.KheIrProxMinNoiseFloor,
      'NoiseFloorMax': KheBase.KheIrProxMaxNoiseFloor,
      'NoiseFloorDft': KheBase.KheIrProxDftNoiseFloor,
      'MaxDist': KheBase.KheIrProxMaxDist * 2   # for graphing
    }

    # get parent gui object for this dialog
    parent = self.GSGuiGetParent()

    # the dialog
    dlg = GuiDlgKheCalIrLed.GuiDlgKheCalIrLed(parent, 
                                  sensorMimeType,
                                  sensorName,
                                  self.mCmd.ProximitySensorGetDftCalibration,
                                  factSettings,
                                  lastSettings)

    # options have been okay'ed
    if dlg.result: 

      # update ini with current connection settings
      iniSettings = utils.dict2tuples(dlg.result)
      ini.IniSetModifiedItems(section, iniSettings)

      # re-init settings
      self.IniInitProximity()

    # go back to parent gui
    self.GSGuiRaiseParent()

  #--
  def GuiCbRobotCalAmbient(self):
    """ 'Ambient Sensor Calibration' menu callback """
    if __debug__: self.mDbg.d1print('Robot|Calibration|Ambient')

    sensorMimeType  = kvals.KheSensorMimeTypeAmbient
    sensorName      = 'Ambient'

    # get parsed ini configuation (ini guraunteed to exist)
    ini           = self.GSGetIni()
    section       = KheIniDD.IniDDSectAmbient
    iniSettings   = ini.IniGetItems(section)
    lastSettings  = utils.tuples2dict(iniSettings)
    factSettings  = {
      'KBrightnessMin': KheBase.KheIrAmbMinKBrightness,
      'KBrightnessMax': KheBase.KheIrAmbMaxKBrightness,
      'KBrightnessDft': KheBase.KheIrAmbDftKBrightness,
      'NoiseFloorMin': KheBase.KheIrAmbMinNoiseFloor,
      'NoiseFloorMax': KheBase.KheIrAmbMaxNoiseFloor,
      'NoiseFloorDft': KheBase.KheIrAmbDftNoiseFloor,
      'MaxDist': KheBase.KheIrAmbMaxDist   # for graphing
    }

    # get parent gui object for this dialog
    parent = self.GSGuiGetParent()

    # the dialog
    dlg = GuiDlgKheCalIrLed.GuiDlgKheCalIrLed(parent, 
                                  sensorMimeType,
                                  sensorName,
                                  self.mCmd.AmbientSensorGetDftCalibration,
                                  factSettings,
                                  lastSettings)

    # options have been okay'ed
    if dlg.result: 

      # update ini with current connection settings
      iniSettings = utils.dict2tuples(dlg.result)
      ini.IniSetModifiedItems(section, iniSettings)

      # re-init settings
      self.IniInitAmbient()

    # go back to parent gui
    self.GSGuiRaiseParent()

  #--
  def GuiCbRobotCalGP2D120(self):
    """ 'GP2D120 Sensor Calibration' menu callback """
    if __debug__: self.mDbg.d1print('Robot|Calibration|GP2D120')
    msgbox.WarningBox('Not implemented yet.')

  #--
  def GuiCbRobotViz(self):
    """ 'Robot Visualizer' menu callback. """
    if __debug__: self.mDbg.d1print('Robot|Robot Visualizer')
    msgbox.WarningBox('Not implemented yet.')

  #--
  def GuiCbRobotVizTrip(self):
    """ 'Trip Visualizer' menu callback. """
    if __debug__: self.mDbg.d1print('Robot|Trip Visualizer')
    self.GSGuiWinStart('VizTrip',           # window ID
        GuiWinKheTrip.GuiWinKheTrip,        # start object
        sense_loc=self.SenseOdometer,       # arguments to start
        sense_speed=self.SenseSpeedometer,
        reset_loc=self.SenseCfgOdometer)
    if self.mOpt['UseOdometrySensors']:
      run_time='enabled'
    else:
      run_time='disabled'
    self.GSGuiWinUpdate('VizTrip', 'cfg', run_time=run_time)

  #--
  def GuiCbRobotNav(self):
    """ 'Navigator' menu callback. """
    if __debug__: self.mDbg.d1print('Robot|Navigator')
    self.GSGuiWinStart('Navigator',         # window ID
        GuiWinKheNav.GuiWinKheNav,          # start object
        sense_loc=self.SenseOdometer,       # arguments to start
        sense_speed=self.SenseSpeedometer,
        bellum_set=self.BellumSetGoals)
    if self.mOpt['UseOdometrySensors']:
      run_time='enabled'
    else:
      run_time='disabled'
    self.GSGuiWinUpdate('Navigator', 'cfg', run_time=run_time)

  #--
  def GuiCbRobotShell(self):
    """ 'Shell' menu callback. """
    if __debug__: self.mDbg.d1print('Robot|Shell')
    self.GSGuiWinStart('Shell',             # window ID
        GuiWinShell.GuiWinShell,            # start object
        KheCmdShell.KheGP2D120Shell,        # arguments to start
        title='vKhepera Command Shell',
        robotCmds=self.mCmd)

  #--
  def GuiCbHelpAbout(self):
    """ 'Shell' menu callback. """
    if __debug__: self.mDbg.d1print('Help|About Robot')

    # get parent gui object for this dialog
    parent = self.GSGuiGetParent()

    verstr = self.HasName() + ' v' + self.IsVersion() + '\n' + 'KheperaII '
    botver = self.IsRobotVersion()
    if type(botver) is list:
      verstr += 'Bios v' + botver[0] + ', Protocol v' + botver[1]
    else:
      verstr += botver
    imageFile = gut.GetFusionImageFileName(gt.ImageKhepera)

    GuiDlgAbout.GuiDlgAbout(parent,
                    name=self.HasName(),
                    version=verstr,
                    descTitle='RoadNarrows vKhepera vRobot Demo',
                    desc="""\
vKhepera is a virtual Robot (vRobot) that interfaces with the Khepera II
physical robot. vKhepera provides full access to its built in sensors and
effectors. The Sharp GP2D120 distance measuring sensor is also supported
via the General I/O turret. vKhepera provides the full set of Khepera
commands, plus movement goal tracking.

The goals are specified as any combination of speed and direction:
 * speed - track to an absolute speed (mm/s)
 * dspeed - track to delta speed (mm/s) from the current speed
 * theta -  track to absolute direction (radians)
 * dtheta - track to delta direction (radians) from the current direction""",
                    copyright='RoadNarrows LLC\n(C) 2006',
                    logoImage=imageFile)

    # go back to parent gui
    self.GSGuiRaiseParent()


#-------------------------------------------------------------------------------
# Unit Test Code
#-------------------------------------------------------------------------------

if __name__ == '__main__':
  def tstCreate(level=2):
    """ Create vKhepera unit test environment. """
    k = vKhepera(debuglevel=level)
    k.mCmd.Open('/dev/ttyS0')
    k.SetCommStatus(True)
    k.ExecLoad()
    return k
  
  def tstShortRun(k, sec=10):
    """ Short run test """
    iv = 0.1
    i = iv
    print(k.BellumGetGoals())
    state = k.GetRobotState()
    if state == Gluon.EServerState.Ready:
      k.ExecStart()
      k.BellumSetGoals(theta=math.pi, speed=20.0) # turn 180 going 20mm/s
    while i < sec:
      print(k.ShadowGet(kvals.KheSensorMimeTypeOdometer))
      time.sleep(iv)
      i += iv
    print(k.ShadowGet(kvals.KheSensorMimeTypeOdometer))
    k.ExecUnload()
  
  def main():
    """ vKhepera Unit Test Main """
    k = tstCreate()
    tstShortRun(k, 10)

  # run unit test
  main()
